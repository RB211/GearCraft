//! Triangle meshes for 3D printing and other mesh tools: STL, OBJ and 3MF.
//!
//! Every part is a closed, manifold mesh with its triangles wound
//! anticlockwise seen from outside (each edge used once in each
//! direction), which is what slicers and the 3MF format require. Faces
//! share their boundary vertices rather than meeting at cracks.

use crate::bevel::BevelGeom;
use crate::export::{alone, fit_arcs, tolerance};
use crate::gear::{self, GearParams};
use crate::hypoid::Member;
use crate::step::{Patch, TIP_POINTS, tooth_data};
use crate::train::{self, Frame3, Gear, Placement, Project};
use crate::view3d::{ear_clip, sample_path};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::Path;
use std::sync::Arc;

type V3 = [f64; 3];

/// Chord tolerance for curved edges, mm.
const CHORD: f64 = 0.005;

/// Longest triangle edge across a curved tooth end, mm.
const END_EDGE: f64 = 0.8;

#[derive(Default)]
pub struct Mesh {
    pub verts: Vec<V3>,
    pub tris: Vec<[u32; 3]>,
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn rot_z(a: f64, p: V3) -> V3 {
    let (s, c) = a.sin_cos();
    [p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2]]
}

/// Segments for a full circle of radius r within the chord tolerance.
fn circle_segments(r: f64) -> usize {
    let step = 2.0 * (1.0 - CHORD / r.max(CHORD * 2.0)).acos();
    ((TAU / step).ceil() as usize).max(48)
}

impl Mesh {
    fn v(&mut self, p: V3) -> u32 {
        self.verts.push(p);
        self.verts.len() as u32 - 1
    }

    /// One face's triangles, all turned over if they face away from
    /// `out` (taken over the whole face, so a curved face is fine).
    fn face(&mut self, tris: &[[u32; 3]], out: V3) {
        self.face_by(tris, |_| out);
    }

    /// As `face`, with the outward direction given at each triangle's
    /// centre, for a face that wraps far round the axis.
    fn face_by(&mut self, tris: &[[u32; 3]], out: impl Fn(V3) -> V3) {
        let facing: f64 = tris
            .iter()
            .map(|t| {
                let [a, b, c] = t.map(|i| self.verts[i as usize]);
                let mid = [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0, (a[2] + b[2] + c[2]) / 3.0];
                dot(cross(sub(b, a), sub(c, a)), out(mid))
            })
            .sum();
        let flip = facing < 0.0;
        self.tris.extend(tris.iter().map(|&[a, b, c]| if flip { [a, c, b] } else { [a, b, c] }));
    }

    /// Quads between rows of vertex indices [i][j], facing `out` (given
    /// at each point).
    fn grid(&mut self, g: &[Vec<u32>], out: impl Fn(V3) -> V3) {
        let mut tris = Vec::new();
        for i in 0..g.len() - 1 {
            for j in 0..g[i].len() - 1 {
                tris.push([g[i][j], g[i + 1][j], g[i + 1][j + 1]]);
                tris.push([g[i][j], g[i + 1][j + 1], g[i][j + 1]]);
            }
        }
        self.face_by(&tris, out);
    }

    /// A loop of vertices (azimuth rising) joined to a smaller loop, or
    /// to a centre point, in the plane between them.
    fn annulus(&mut self, outer: &[u32], inner: Option<&[u32]>, out: V3) {
        let ang = |m: &Mesh, i: u32| {
            let p = m.verts[i as usize];
            p[1].atan2(p[0])
        };
        let mut tris = Vec::new();
        match inner {
            None => {
                let c = {
                    let z = self.verts[outer[0] as usize][2];
                    self.v([0.0, 0.0, z])
                };
                for i in 0..outer.len() {
                    tris.push([c, outer[i], outer[(i + 1) % outer.len()]]);
                }
            }
            Some(inner) => {
                // Both loops' azimuths unwrapped from the outer start.
                let a0 = ang(self, outer[0]);
                let unwrap = |a: f64| a0 + (a - a0).rem_euclid(TAU);
                let start = (0..inner.len())
                    .min_by(|&x, &y| {
                        let d = |i: usize| {
                            let r = (ang(self, inner[i]) - a0).rem_euclid(TAU);
                            r.min(TAU - r)
                        };
                        d(x).total_cmp(&d(y))
                    })
                    .unwrap();
                let inner: Vec<u32> = (0..inner.len()).map(|k| inner[(start + k) % inner.len()]).collect();
                let lift = |list: &[u32], m: &Mesh| -> Vec<f64> {
                    let mut out: Vec<f64> = list.iter().map(|&i| unwrap(ang(m, i))).collect();
                    // The inner loop can start just before the outer one.
                    if out[0] > a0 + PI {
                        out[0] -= TAU;
                    }
                    for k in 1..out.len() {
                        while out[k] < out[k - 1] {
                            out[k] += TAU;
                        }
                    }
                    out
                };
                let (ao, ai) = (lift(outer, self), lift(&inner, self));
                let (no, ni) = (outer.len(), inner.len());
                let (mut i, mut j) = (0, 0);
                while i < no || j < ni {
                    let next_o = if i < no { if i + 1 < no { ao[i + 1] } else { ao[0] + TAU } } else { f64::MAX };
                    let next_i = if j < ni { if j + 1 < ni { ai[j + 1] } else { ai[0] + TAU } } else { f64::MAX };
                    if next_o <= next_i {
                        tris.push([outer[i % no], outer[(i + 1) % no], inner[j % ni]]);
                        i += 1;
                    } else {
                        tris.push([outer[i % no], inner[(j + 1) % ni], inner[j % ni]]);
                        j += 1;
                    }
                }
            }
        }
        self.face(&tris, out);
    }

    /// The wall between two rings of vertices one above the other.
    fn bore(&mut self, lo: &[u32], hi: &[u32]) {
        let n = lo.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let p = self.verts[lo[i] as usize];
            // Out of the material is into the hole, towards the axis.
            let out = [-p[0], -p[1], 0.0];
            self.face(&[[lo[i], lo[j], hi[j]], [lo[i], hi[j], hi[i]]], out);
        }
    }

    /// A ring of vertices round the axis.
    fn ring(&mut self, r: f64, z: f64, n: usize) -> Vec<u32> {
        (0..n).map(|i| i as f64 * TAU / n as f64).map(|a| self.v([r * a.cos(), r * a.sin(), z])).collect()
    }

    /// Triangles filling a polygon of vertex indices, clipped in the
    /// plane `flat` maps them to.
    fn polygon(&mut self, idx: &[u32], flat: impl Fn(V3) -> [f64; 2], out: V3) {
        let pts: Vec<[f64; 2]> = idx.iter().map(|&i| flat(self.verts[i as usize])).collect();
        let tris: Vec<[u32; 3]> = ear_clip(&pts).into_iter().map(|t| t.map(|k| idx[k])).collect();
        self.face(&tris, out);
    }

    /// A polygon on a curved surface: clipped in the surface's own (flat,
    /// mm-scaled) coordinates, its long inner edges split until none is
    /// over `max` and the new points lifted back onto the surface. The
    /// boundary is left as it is, so it still meets the faces around it.
    /// A triangle on a long boundary edge always keeps a long inner edge,
    /// so triangles under max²/8 are not split further: every split
    /// halves two triangles, which bounds the splitting.
    fn curved_polygon(&mut self, idx: &[u32], flat: impl Fn(V3) -> [f64; 2], lift: impl Fn([f64; 2]) -> V3, max: f64, out: V3) {
        let pts: Vec<[f64; 2]> = idx.iter().map(|&i| flat(self.verts[i as usize])).collect();
        self.curved_polygon_at(idx, pts, lift, max, out);
    }

    /// As `curved_polygon`, with the flat point of each corner given, so a
    /// vertex can stand at two corners (a seam the polygon is cut along).
    fn curved_polygon_at(&mut self, idx: &[u32], mut pts: Vec<[f64; 2]>, lift: impl Fn([f64; 2]) -> V3, max: f64, out: V3) {
        let mut tris = ear_clip(&pts);
        let key = |a: usize, b: usize| (a.min(b), a.max(b));
        let mut by_edge: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
        let len = |pts: &[[f64; 2]], e: (usize, usize)| (pts[e.0][0] - pts[e.1][0]).hypot(pts[e.0][1] - pts[e.1][1]);
        let area = |pts: &[[f64; 2]], t: [usize; 3]| {
            let [a, b, c] = t.map(|k| pts[k]);
            ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])).abs() / 2.0
        };
        let min_area = max * max / 8.0;
        for (t, tri) in tris.iter().enumerate() {
            for k in 0..3 {
                by_edge.entry(key(tri[k], tri[(k + 1) % 3])).or_default().push(t);
            }
        }
        let mut todo: Vec<(usize, usize)> = by_edge.keys().copied().filter(|&e| len(&pts, e) > max).collect();
        // In a fixed order, so the same part always meshes the same way.
        todo.sort_unstable();
        while let Some(e) = todo.pop() {
            // Only inner edges (two triangles) are split.
            let Some(ts) = by_edge.get(&e).cloned().filter(|ts| ts.len() == 2) else { continue };
            if ts.iter().any(|&t| area(&pts, tris[t]) < min_area) {
                continue;
            }
            let m = pts.len();
            pts.push([(pts[e.0][0] + pts[e.1][0]) / 2.0, (pts[e.0][1] + pts[e.1][1]) / 2.0]);
            for t in ts {
                let tri = tris[t];
                let r = (0..3).find(|&k| !(tri[k] == e.0 || tri[k] == e.1)).unwrap();
                let [p, q, c] = [tri[(r + 1) % 3], tri[(r + 2) % 3], tri[r]];
                for k in 0..3 {
                    if let Some(v) = by_edge.get_mut(&key(tri[k], tri[(k + 1) % 3])) {
                        v.retain(|&x| x != t);
                    }
                }
                tris[t] = [p, m, c];
                tris.push([m, q, c]);
                for (ti, nt) in [(t, tris[t]), (tris.len() - 1, [m, q, c])] {
                    for k in 0..3 {
                        let ek = key(nt[k], nt[(k + 1) % 3]);
                        by_edge.entry(ek).or_default().push(ti);
                        if len(&pts, ek) > max {
                            todo.push(ek);
                        }
                    }
                }
            }
            by_edge.remove(&e);
        }
        let mut global: Vec<u32> = idx.to_vec();
        for &p in &pts[idx.len()..] {
            global.push(self.v(lift(p)));
        }
        let tris: Vec<[u32; 3]> = tris.into_iter().map(|t| t.map(|k| global[k])).collect();
        self.face(&tris, out);
    }

    fn placed(mut self, frame: Frame3, rot: f64) -> Mesh {
        for p in &mut self.verts {
            *p = frame.apply(rot_z(rot, *p));
        }
        self
    }

    #[cfg(test)]
    pub fn volume(&self) -> f64 {
        self.tris
            .iter()
            .map(|t| {
                let [a, b, c] = t.map(|i| self.verts[i as usize]);
                dot(a, cross(b, c)) / 6.0
            })
            .sum()
    }
}

// ---- spur gears and pulleys -------------------------------------------------

/// A spur gear or pulley: the arc-fitted outline extruded through the
/// face width, the bore through it. Each end face is filled a tooth at a
/// time, between rays from the bore through the middle of each space.
fn spur(p: &GearParams) -> Option<Mesh> {
    let prof = gear::tooth_profile(p)?;
    let path = fit_arcs(&gear::outline(p, &prof), tolerance(p.module));
    let mut lp = sample_path(&path, 0.05);
    if signed_area(&lp) < 0.0 {
        lp.reverse();
    }
    let z = p.teeth as usize;
    let pitch = TAU / z as f64;
    // Split the outline where each ray first meets it.
    let mut cuts: Vec<(usize, [f64; 2])> = Vec::new();
    for t in 0..z {
        let a = (t as f64 + 0.5) * pitch;
        let (s, c) = a.sin_cos();
        let mut best: Option<(f64, usize, [f64; 2])> = None;
        for i in 0..lp.len() {
            let (p0, p1) = (lp[i], lp[(i + 1) % lp.len()]);
            // Side of the ray's line, and distance along it.
            let side = |q: [f64; 2]| q[1] * c - q[0] * s;
            let (d0, d1) = (side(p0), side(p1));
            if (d0 < 0.0) == (d1 < 0.0) || (d0 - d1).abs() < 1e-300 {
                continue;
            }
            let f = d0 / (d0 - d1);
            let q = [p0[0] + f * (p1[0] - p0[0]), p0[1] + f * (p1[1] - p0[1])];
            let along = q[0] * c + q[1] * s;
            if along > 0.0 && best.is_none_or(|b| along < b.0) {
                best = Some((along, i, q));
            }
        }
        let (_, i, q) = best?;
        cuts.push((i, q));
    }
    // The outline with the cut points in, and where each landed.
    let mut outline: Vec<[f64; 2]> = Vec::with_capacity(lp.len() + z);
    let mut at = vec![0usize; z];
    let mut order: Vec<usize> = (0..z).collect();
    order.sort_by(|&x, &y| {
        let (a, b) = (cuts[x], cuts[y]);
        let key = |i: usize, q: [f64; 2]| {
            let p0 = lp[i];
            (i, (q[0] - p0[0]).hypot(q[1] - p0[1]))
        };
        let (ka, kb) = (key(a.0, a.1), key(b.0, b.1));
        ka.0.cmp(&kb.0).then(ka.1.total_cmp(&kb.1))
    });
    let mut next = 0;
    for (i, &q) in lp.iter().enumerate() {
        outline.push(q);
        while next < z && cuts[order[next]].0 == i {
            at[order[next]] = outline.len();
            outline.push(cuts[order[next]].1);
            next += 1;
        }
    }
    let h = p.face_width.max(0.01);
    let mut m = Mesh::default();
    let n = outline.len();
    let bottom: Vec<u32> = outline.iter().map(|q| m.v([q[0], q[1], 0.0])).collect();
    let top: Vec<u32> = outline.iter().map(|q| m.v([q[0], q[1], h])).collect();
    for i in 0..n {
        let j = (i + 1) % n;
        let (a, b) = (outline[i], outline[j]);
        let out = [b[1] - a[1], a[0] - b[0], 0.0];
        m.face(&[[bottom[i], bottom[j], top[j]], [bottom[i], top[j], top[i]]], out);
    }
    // The bore, with a vertex on every ray.
    let rb = p.hole() / 2.0;
    let per = (circle_segments(rb.max(1.0)) / z).max(2);
    let ring = |m: &mut Mesh, zz: f64| -> Vec<u32> {
        if rb <= 0.0 {
            return vec![m.v([0.0, 0.0, zz]); 1];
        }
        (0..z * per)
            .map(|k| {
                let a = 0.5 * pitch + pitch * k as f64 / per as f64;
                m.v([rb * a.cos(), rb * a.sin(), zz])
            })
            .collect()
    };
    let (hole0, hole1) = (ring(&mut m, 0.0), ring(&mut m, h));
    if rb > 0.0 {
        m.bore(&hole0, &hole1);
    }
    for (rim, hole, up) in [(&bottom, &hole0, false), (&top, &hole1, true)] {
        let out = [0.0, 0.0, if up { 1.0 } else { -1.0 }];
        for t in 0..z {
            let (o0, o1) = (at[t], at[(t + 1) % z]);
            let mut idx = Vec::new();
            let foot = |k: usize| if rb > 0.0 { hole[(k * per) % hole.len()] } else { hole[0] };
            idx.push(foot(t));
            let mut i = o0;
            loop {
                idx.push(rim[i]);
                if i == o1 {
                    break;
                }
                i = (i + 1) % n;
            }
            if rb > 0.0 {
                // Back along the bore to the start.
                for k in (0..=per).rev() {
                    idx.push(hole[(t * per + k) % hole.len()]);
                }
                idx.pop();
            }
            m.polygon(&idx, |q| [q[0], q[1]], out);
        }
    }
    Some(m)
}

fn signed_area(lp: &[[f64; 2]]) -> f64 {
    (0..lp.len()).map(|i| {
        let (a, b) = (lp[i], lp[(i + 1) % lp.len()]);
        a[0] * b[1] - b[0] * a[1]
    }).sum::<f64>() / 2.0
}

// ---- straight bevel gears ---------------------------------------------------

/// A straight bevel gear about its apex: flanks along rays between the
/// front and back cones, tooth ends on those cones (each closed along the
/// root circle), flat faces from the root circle in to the bore.
fn bevel(g: &BevelGeom) -> Mesh {
    let mut m = Mesh::default();
    let k = g.front_ratio();
    let lp = g.outline();
    let n = lp.len();
    let z = g.teeth as usize;
    let per = n / z;
    let nr = g.profile.radii.len();
    let back_pt = |i: usize| {
        let [gamma, phi] = lp[i];
        BevelGeom::point(gamma, phi, g.back(gamma))
    };
    let back: Vec<u32> = (0..n).map(|i| m.v(back_pt(i))).collect();
    let front: Vec<u32> = (0..n).map(|i| {
        let p = back_pt(i);
        m.v([k * p[0], k * p[1], k * p[2]])
    }).collect();
    for i in 0..n {
        let j = (i + 1) % n;
        let (pa, pb) = (back_pt(i), back_pt(j));
        let r = [pa[0] + pb[0], pa[1] + pb[1], pa[2] + pb[2]];
        let out = cross(sub(pb, pa), r);
        m.face(&[[front[i], front[j], back[j]], [front[i], back[j], back[i]]], out);
    }
    // Tooth ends, and the rim (root corners and lands) for the flat faces.
    let mut rims = [Vec::new(), Vec::new()];
    for t in 0..z {
        let s0 = t * per;
        let s1 = s0 + 2 * nr + 4;
        let c = t as f64 * TAU / z as f64;
        for (e, (ring, sign)) in [(&back, 1.0), (&front, -1.0)].into_iter().enumerate() {
            let idx: Vec<u32> = (s0..=s1).map(|i| ring[i]).collect();
            let ray = BevelGeom::point(g.delta, c, sign);
            let scale = if e == 0 { 1.0 } else { k };
            let sd = g.delta.sin() * g.cone_distance;
            m.curved_polygon(
                &idx,
                |q| {
                    let a = c + (q[1].atan2(q[0]) - c + PI).rem_euclid(TAU) - PI;
                    [a * sd, q[0].hypot(q[1]).atan2(q[2]) * g.cone_distance]
                },
                |f| {
                    let gamma = f[1] / g.cone_distance;
                    BevelGeom::point(gamma, f[0] / sd, scale * g.back(gamma))
                },
                END_EDGE,
                ray,
            );
            rims[e].push(ring[s0]);
            rims[e].extend((s1..s0 + per).map(|i| ring[i]));
        }
    }
    let rb = (g.bore / 2.0).min(g.front_face().1 * 0.92);
    let (zb, _) = g.back_face();
    let (zf, _) = g.front_face();
    let segs = circle_segments(rb.max(1.0));
    let holes = (rb > 0.0).then(|| (m.ring(rb, zb, segs), m.ring(rb, zf, segs)));
    m.annulus(&rims[0], holes.as_ref().map(|h| &h.0[..]), [0.0, 0.0, 1.0]);
    m.annulus(&rims[1], holes.as_ref().map(|h| &h.1[..]), [0.0, 0.0, -1.0]);
    if let Some((hb, hf)) = &holes {
        m.bore(hf, hb);
    }
    m
}

// ---- hypoid gears -----------------------------------------------------------

/// Sections across the face, per generated section.
const HYPOID_REFINE: usize = 3;

/// A hypoid gear or pinion about its pitch apex, from the same smooth
/// surfaces as its STEP solid, sampled finely across the face.
fn hypoid(mb: &Member) -> Mesh {
    let mut m = Mesh::default();
    let z = mb.teeth as usize;
    let pitch = mb.pitch();
    let ns = mb.stations.len();
    let u_params: Vec<f64> = (0..ns).map(|k| k as f64 / (ns - 1) as f64).collect();
    let nu = (ns - 1) * HYPOID_REFINE + 1;
    let us: Vec<f64> = (0..nu).map(|a| a as f64 / (nu - 1) as f64).collect();
    let (sg, cg) = mb.blank.pitch.sin_cos();
    let patches: Vec<[Patch; 3]> = (0..z).map(|t| tooth_data(mb, t).map(|d| Patch::through(&d, &u_params))).collect();
    // Seams first: each is shared by two teeth.
    let seam: Vec<Vec<u32>> = (0..z).map(|t| us.iter().map(|&u| m.v(patches[t][0].eval(u, 0.0))).collect()).collect();
    let mut right = Vec::with_capacity(z);
    let mut left = Vec::with_capacity(z);
    let mut tips = Vec::with_capacity(z);
    for t in 0..z {
        let next = (t + 1) % z;
        let [r, tp, l] = &patches[t];
        let rg: Vec<Vec<u32>> = (0..nu)
            .map(|a| {
                let mut row = vec![seam[t][a]];
                row.extend(r.vp[1..].iter().map(|&v| m.v(r.eval(us[a], v))));
                row
            })
            .collect();
        let lg: Vec<Vec<u32>> = (0..nu)
            .map(|a| {
                let mut row = vec![seam[next][a]];
                row.extend(l.vp[1..].iter().map(|&v| m.v(l.eval(us[a], v))));
                row
            })
            .collect();
        let tg: Vec<Vec<u32>> = (0..nu)
            .map(|a| {
                let mut row = vec![*rg[a].last().unwrap()];
                row.extend(tp.vp[1..TIP_POINTS - 1].iter().map(|&v| m.v(tp.eval(us[a], v))));
                row.push(*lg[a].last().unwrap());
                row
            })
            .collect();
        // Out of the flanks round the axis, and off the cone for the tip: a
        // worm's teeth wrap far round, so each is taken where it is.
        let phi = |p: V3| p[1].atan2(p[0]);
        m.grid(&rg, |p| [phi(p).sin(), -phi(p).cos(), 0.0]);
        m.grid(&lg, |p| [-phi(p).sin(), phi(p).cos(), 0.0]);
        m.grid(&tg, |p| [cg * phi(p).cos(), cg * phi(p).sin(), -sg]);
        right.push(rg);
        left.push(lg);
        tips.push(tg);
    }
    // Ends: per tooth, from the hub circle up round the tooth; the hub
    // circle's points also bound the flat faces.
    let faces = mb.hub_faces();
    let mut hub_loops = [Vec::new(), Vec::new()];
    for (e, (a, k)) in [(0, 0), (nu - 1, ns - 1)].into_iter().enumerate() {
        let (zh, rh) = faces[e];
        let mut arc = (circle_segments(rh) / z).max(4);
        // A single start is cut down its middle (see below): an even count
        // puts a foot there.
        if z == 1 {
            arc += arc % 2;
        }
        let seam_phi = |t: usize| mb.right[k][0][1] + t as f64 * pitch;
        let feet: Vec<Vec<u32>> = (0..z)
            .map(|t| {
                (0..arc)
                    .map(|i| {
                        let phi = seam_phi(t) + pitch * i as f64 / arc as f64;
                        m.v([rh * phi.cos(), rh * phi.sin(), zh])
                    })
                    .collect()
            })
            .collect();
        for t in 0..z {
            let next = (t + 1) % z;
            let mut idx = vec![feet[t][0]];
            idx.extend(&right[t][a]);
            idx.extend(&tips[t][a][1..tips[t][a].len() - 1]);
            idx.extend(left[t][a].iter().rev());
            idx.push(feet[next][0]);
            idx.extend(feet[t][1..].iter().rev());
            let centre = seam_phi(t) + pitch / 2.0;
            let sign = if e == 0 { -1.0 } else { 1.0 };
            let out = [sign * sg * centre.cos(), sign * sg * centre.sin(), sign * cg];
            let st = mb.stations[k];
            let scale = mb.arc_scale(k);
            // On a cylinder (a worm drive) the end is a plane, clipped as it
            // lies, so the hub circle keeps its true bend. A single-start
            // worm's one tooth goes the whole way round, its seam at both
            // ends, so it is cut in two from the middle of the tip down to
            // the hub. A cone's end is laid out flat by azimuth and depth.
            let plane = sg.abs() < 1e-12;
            let mut last = centre - pitch / 2.0;
            let pts: Vec<[f64; 2]> = idx
                .iter()
                .map(|&i| {
                    let q = m.verts[i as usize];
                    if plane {
                        return [q[0], q[1]];
                    }
                    let a = q[1].atan2(q[0]);
                    last += (a - last + PI).rem_euclid(TAU) - PI;
                    [last * scale, q[0].hypot(q[1]) * cg - q[2] * sg]
                })
                .collect();
            let lift = |f: [f64; 2]| if plane { [f[0], f[1], st] } else { mb.point(st, f[1], f[0] / scale) };
            if z == 1 {
                let tip_mid = 1 + right[t][a].len() + (tips[t][a].len() - 1) / 2 - 1;
                let foot_mid = idx.len() - arc / 2;
                let mut first: Vec<usize> = (0..=tip_mid).collect();
                first.extend(foot_mid..idx.len());
                for part in [first, (tip_mid..=foot_mid).collect()] {
                    let (ix, pt): (Vec<u32>, Vec<[f64; 2]>) = part.into_iter().map(|j| (idx[j], pts[j])).unzip();
                    m.curved_polygon_at(&ix, pt, lift, END_EDGE, out);
                }
            } else {
                m.curved_polygon_at(&idx, pts, lift, END_EDGE, out);
            }
            hub_loops[e].extend(&feet[t]);
        }
    }
    let rb = (mb.bore / 2.0).max(0.0);
    let (lo, hi) = if faces[0].0 < faces[1].0 { (0, 1) } else { (1, 0) };
    let segs = circle_segments(rb.max(1.0));
    let holes = (rb > 0.0).then(|| [m.ring(rb, faces[0].0, segs), m.ring(rb, faces[1].0, segs)]);
    for e in 0..2 {
        let up = e == hi;
        m.annulus(&hub_loops[e].clone(), holes.as_ref().map(|h| &h[e][..]), [0.0, 0.0, if up { 1.0 } else { -1.0 }]);
    }
    if let Some(h) = &holes {
        m.bore(&h[lo], &h[hi]);
    }
    m
}

// ---- parts and files --------------------------------------------------------

/// One part, placed in world millimetres.
pub struct Part {
    pub name: String,
    pub mesh: Mesh,
}

/// The parts `keep` picks, placed as they sit in the train at
/// `driver_angle`, or just part `only`, at the origin. `cached` hands over
/// hypoid teeth already generated (by ring gear id).
pub fn parts(
    project: &Project,
    driver_angle: f64,
    only: Option<u64>,
    keep: impl Fn(&Gear) -> bool,
    cached: impl Fn(u64) -> Option<Arc<(Member, Member)>>,
) -> Vec<Part> {
    let solved = project.solve();
    let place = |i: usize, member: Option<&Member>| {
        let s = &solved[i];
        if only.is_none() { (s.frame, s.phase + s.ratio * driver_angle) } else { (alone(&project.gears[i], member), 0.0) }
    };
    let mut out = Vec::new();
    for (i, g) in project.gears.iter().enumerate() {
        if !(only.is_none_or(|id| id == g.id) && keep(g)) || g.params.is_generated() || !g.params.is_buildable() {
            continue;
        }
        let mesh = match BevelGeom::new(&g.params) {
            Some(b) => Some(bevel(&b)),
            None => spur(&g.params),
        };
        if let Some(mesh) = mesh {
            let (frame, rot) = place(i, None);
            out.push((i, Part { name: g.name.clone(), mesh: mesh.placed(frame, rot) }));
        }
    }
    for ring in &project.gears {
        let (Placement::Hypoid { parent, .. } | Placement::Worm { parent, .. }) = ring.placement else { continue };
        let Some(pi) = project.index_of(parent) else { continue };
        let pinion = &project.gears[pi];
        let wanted: Vec<(&Gear, usize)> =
            [(pinion, 0), (ring, 1)].into_iter().filter(|(g, _)| only.is_none_or(|id| id == g.id) && keep(g)).collect();
        if wanted.is_empty() {
            continue;
        }
        let pair = cached(ring.id)
            .or_else(|| Some(Arc::new(train::pair_spec(&pinion.params, &ring.params, &ring.placement, project.ball())?.generate().ok()?)));
        let Some(pair) = pair else { continue };
        for (g, which) in wanted {
            let Some(i) = project.index_of(g.id) else { continue };
            let member = if which == 0 { &pair.0 } else { &pair.1 };
            let (frame, rot) = place(i, Some(member));
            out.push((i, Part { name: g.name.clone(), mesh: hypoid(member).placed(frame, rot) }));
        }
    }
    out.sort_by_key(|(i, _)| *i);
    out.into_iter().map(|(_, p)| p).collect()
}

/// Write STL, OBJ or 3MF, chosen by the file extension. Always in mm.
pub fn write(path: &Path, parts: &[Part]) -> std::io::Result<()> {
    let ext = path.extension().map(|e| e.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
    let name = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    let bytes = match ext.as_str() {
        "obj" => obj(parts).into_bytes(),
        "3mf" => three_mf(parts, &name)?,
        _ => stl(parts, &name),
    };
    std::fs::write(path, bytes)
}

/// Binary STL: every part's triangles in one solid.
pub fn stl(parts: &[Part], name: &str) -> Vec<u8> {
    let count: usize = parts.iter().map(|p| p.mesh.tris.len()).sum();
    let mut out = Vec::with_capacity(84 + 50 * count);
    let mut header = format!("GearCraft {name}").into_bytes();
    header.resize(80, b' ');
    out.extend(header);
    out.extend((count as u32).to_le_bytes());
    for p in parts {
        for t in &p.mesh.tris {
            let [a, b, c] = t.map(|i| p.mesh.verts[i as usize]);
            let n = cross(sub(b, a), sub(c, a));
            let l = dot(n, n).sqrt().max(1e-300);
            for v in [[n[0] / l, n[1] / l, n[2] / l], a, b, c] {
                for x in v {
                    out.extend((x as f32).to_le_bytes());
                }
            }
            out.extend([0u8, 0]);
        }
    }
    out
}

fn num(x: f64) -> String {
    let s = format!("{x:.5}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.to_string() }
}

/// Wavefront OBJ: one object per part.
pub fn obj(parts: &[Part]) -> String {
    let mut out = String::from("# GearCraft, millimetres\n");
    let mut base = 1;
    for p in parts {
        let _ = writeln!(out, "o {}", p.name.replace(char::is_whitespace, "_"));
        for v in &p.mesh.verts {
            let _ = writeln!(out, "v {} {} {}", num(v[0]), num(v[1]), num(v[2]));
        }
        for t in &p.mesh.tris {
            let _ = writeln!(out, "f {} {} {}", t[0] as usize + base, t[1] as usize + base, t[2] as usize + base);
        }
        base += p.mesh.verts.len();
    }
    out
}

fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

/// 3MF: a zip holding the model, one object per part.
pub fn three_mf(parts: &[Part], name: &str) -> std::io::Result<Vec<u8>> {
    let mut model = String::from(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<model unit=\"millimeter\" xml:lang=\"en-US\" \
         xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n",
    );
    let _ = writeln!(model, " <metadata name=\"Title\">{}</metadata>", xml_escape(name));
    model.push_str(" <metadata name=\"Application\">GearCraft</metadata>\n <resources>\n");
    for (i, p) in parts.iter().enumerate() {
        let _ = writeln!(model, "  <object id=\"{}\" type=\"model\" name=\"{}\">\n   <mesh>\n    <vertices>", i + 1, xml_escape(&p.name));
        for v in &p.mesh.verts {
            let _ = writeln!(model, "     <vertex x=\"{}\" y=\"{}\" z=\"{}\"/>", num(v[0]), num(v[1]), num(v[2]));
        }
        model.push_str("    </vertices>\n    <triangles>\n");
        for t in &p.mesh.tris {
            let _ = writeln!(model, "     <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>", t[0], t[1], t[2]);
        }
        model.push_str("    </triangles>\n   </mesh>\n  </object>\n");
    }
    model.push_str(" </resources>\n <build>\n");
    for i in 0..parts.len() {
        let _ = writeln!(model, "  <item objectid=\"{}\"/>", i + 1);
    }
    model.push_str(" </build>\n</model>\n");
    let types = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\
                 <Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>\
                 <Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/></Types>\n";
    let rels = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\
                <Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" \
                Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/></Relationships>\n";
    zip(&[("[Content_Types].xml", types.as_bytes()), ("_rels/.rels", rels.as_bytes()), ("3D/3dmodel.model", model.as_bytes())])
}

/// A zip archive of deflated files.
fn zip(files: &[(&str, &[u8])]) -> std::io::Result<Vec<u8>> {
    let mut out = Vec::new();
    let mut central = Vec::new();
    for &(name, data) in files {
        let mut enc = flate2::write::DeflateEncoder::new(Vec::new(), flate2::Compression::default());
        enc.write_all(data)?;
        let packed = enc.finish()?;
        let crc = crc32fast::hash(data);
        let offset = out.len() as u32;
        let common = |v: &mut Vec<u8>| {
            v.extend(20u16.to_le_bytes()); // version needed
            v.extend(0u16.to_le_bytes()); // flags
            v.extend(8u16.to_le_bytes()); // deflate
            v.extend(0u16.to_le_bytes()); // time
            v.extend(0x21u16.to_le_bytes()); // date: 1980-01-01
            v.extend(crc.to_le_bytes());
            v.extend((packed.len() as u32).to_le_bytes());
            v.extend((data.len() as u32).to_le_bytes());
            v.extend((name.len() as u16).to_le_bytes());
            v.extend(0u16.to_le_bytes()); // extra
        };
        out.extend(0x04034b50u32.to_le_bytes());
        common(&mut out);
        out.extend(name.as_bytes());
        out.extend(&packed);
        central.extend(0x02014b50u32.to_le_bytes());
        central.extend(20u16.to_le_bytes()); // made by
        common(&mut central);
        central.extend([0u8; 6]); // comment length, disk, internal attributes
        central.extend(0u32.to_le_bytes()); // external attributes
        central.extend(offset.to_le_bytes());
        central.extend(name.as_bytes());
    }
    let start = out.len() as u32;
    out.extend(&central);
    out.extend(0x06054b50u32.to_le_bytes());
    out.extend([0u8; 4]);
    out.extend((files.len() as u16).to_le_bytes());
    out.extend((files.len() as u16).to_le_bytes());
    out.extend((central.len() as u32).to_le_bytes());
    out.extend(start.to_le_bytes());
    out.extend(0u16.to_le_bytes());
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hypoid::Role;
    use crate::train::{Link, PitchUnit};
    use crate::worm;
    use std::collections::HashMap;

    /// Each edge used once in each direction: closed, manifold and wound
    /// consistently.
    fn assert_watertight(m: &Mesh, what: &str) {
        let mut edges: HashMap<(u32, u32), i32> = HashMap::new();
        for t in &m.tris {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                assert_ne!(a, b, "{what}: degenerate triangle");
                *edges.entry((a, b)).or_default() += 1;
            }
        }
        for (&(a, b), &n) in &edges {
            assert_eq!(n, 1, "{what}: edge {a}-{b} used {n} times one way");
            assert_eq!(edges.get(&(b, a)), Some(&1), "{what}: edge {a}-{b} has no partner");
        }
        assert!(m.volume() > 0.0, "{what}: inside out");
    }

    fn project() -> Project {
        let u = PitchUnit::Module;
        let mut p = Project::empty();
        let a = p.add_root(u.starter(24), 0.0, 0.0);
        p.add_child(a, u.starter(12), Link::Mesh);
        let solid = p.add_root(GearParams { bore: 0.0, ..u.starter(17) }, 0.0, 120.0);
        let _ = solid;
        let q = p.add_root(u.starter_pulley(20), 300.0, 0.0);
        p.add_child(q, u.starter_pulley(30), Link::Belt);
        let b = p.add_root(u.starter_bevel(20), 0.0, -200.0);
        p.add_child(b, u.starter_bevel(40), Link::Bevel);
        let h = p.add_root(u.starter_hypoid(Role::Pinion), 300.0, -300.0);
        p.add_child(h, u.starter_hypoid(Role::Ring), Link::Hypoid);
        // A 3-start bevel worm, its threads wrapping far round.
        let w = p.add_root(GearParams { teeth: 3, ..u.starter_worm(Role::Pinion) }, -300.0, 0.0);
        p.add_child(w, u.starter_worm(Role::Ring), Link::Hypoid);
        // Worm drives: a single-start worm (its one thread goes the whole
        // way round each end) with a throated wheel, and a bored 2-start
        // left-hand worm with a straight wheel.
        let w = p.add_root(u.starter_worm_drive(worm::Role::Worm), -300.0, -300.0);
        p.add_child(w, u.starter_worm_drive(worm::Role::Wheel), Link::Worm);
        let w = p.add_root(GearParams { teeth: 2, bore: 6.0, ..u.starter_worm_drive(worm::Role::Worm) }, -600.0, -300.0);
        let wheel = p.add_child(w, u.starter_worm_drive(worm::Role::Wheel), Link::Worm);
        if let Some(Placement::Worm { throated, left_hand, .. }) = p.gears.iter_mut().find(|g| g.id == wheel).map(|g| &mut g.placement) {
            (*throated, *left_hand) = (false, true);
        }
        // A globoid drive, its hourglass worm wrapped round the wheel.
        let w = p.add_root(u.starter_worm_drive(worm::Role::Worm), -900.0, -300.0);
        let wheel = p.add_child(w, GearParams { teeth: 40, ..u.starter_worm_drive(worm::Role::Wheel) }, Link::Worm);
        p.make_globoid(p.index_of(wheel).unwrap(), true);
        p.normalize();
        p
    }

    /// Made for the printer, every kind of part is still watertight and
    /// has lost material: its teeth drawn in by a quarter of the nozzle
    /// width and its bore widened by half of it.
    #[test]
    fn printed_parts_are_smaller_and_still_watertight() {
        let mut p = project();
        let designed = parts(&p, 0.4, None, |_| true, |_| None);
        p.for_machining = false;
        p.for_printing = true;
        p.nozzle = 0.4;
        p.normalize();
        let printed = parts(&p, 0.4, None, |_| true, |_| None);
        assert_eq!(printed.len(), designed.len());
        for (a, b) in designed.iter().zip(&printed) {
            assert_watertight(&b.mesh, &format!("printed {}", b.name));
            let (va, vb) = (a.mesh.volume(), b.mesh.volume());
            assert!(vb < va, "{}: {vb} printed is no smaller than {va} designed", a.name);
            assert!(vb > 0.5 * va, "{}: {vb} printed is far smaller than {va} designed", a.name);
        }
    }

    #[test]
    fn every_part_is_watertight() {
        let p = project();
        let parts = parts(&p, 0.4, None, |_| true, |_| None);
        assert_eq!(parts.len(), p.gears.len());
        for part in &parts {
            assert_watertight(&part.mesh, &part.name);
        }
    }

    #[test]
    fn a_part_alone_sits_on_its_export_origin() {
        use crate::train::{Datum, ExportOrigin};
        let u = PitchUnit::Module;
        let mut p = Project::empty();
        let spur = p.add_root(u.starter(24), 50.0, 20.0);
        let bevel = p.add_root(u.starter_bevel(20), 0.0, -200.0);
        let worm = p.add_root(u.starter_worm_drive(worm::Role::Worm), -300.0, -300.0);
        p.add_child(worm, u.starter_worm_drive(worm::Role::Wheel), Link::Worm);
        let span = |p: &Project, id: u64, datum: Datum, offset: f64| {
            let mut p = p.clone();
            p.gears.iter_mut().find(|g| g.id == id).unwrap().export_origin = ExportOrigin { datum, offset };
            let part = parts(&p, 0.3, Some(id), |_| true, |_| None).remove(0);
            let z = part.mesh.verts.iter().map(|v| v[2]);
            (z.clone().fold(f64::MAX, f64::min), z.fold(f64::MIN, f64::max))
        };
        let close = |a: f64, b: f64| (a - b).abs() < 1e-6;
        let fw = p.gears[0].params.face_width;
        let (lo, hi) = span(&p, spur, Datum::Own, 0.0);
        assert!(close(lo, 0.0) && close(hi, fw), "{lo} {hi}");
        let (lo, hi) = span(&p, spur, Datum::Middle, 0.0);
        assert!(close(lo, -fw / 2.0) && close(hi, fw / 2.0), "{lo} {hi}");
        let (lo, hi) = span(&p, spur, Datum::Top, 2.5);
        assert!(close(lo, -fw - 2.5) && close(hi, -2.5), "{lo} {hi}");
        // A bevel's own origin is its pitch apex, below the teeth; its
        // flat back face is the top.
        let (lo, _) = span(&p, bevel, Datum::Own, 0.0);
        assert!(lo > 0.0, "{lo}");
        let (_, hi) = span(&p, bevel, Datum::Top, 0.0);
        assert!(close(hi, 0.0), "{hi}");
        let (lo, hi) = span(&p, worm, Datum::Middle, 0.0);
        assert!(close(lo, -hi), "{lo} {hi}");
    }

    #[test]
    fn spur_volume_matches_the_outline() {
        let g = PitchUnit::Module.starter(24);
        let m = spur(&g).unwrap();
        let prof = gear::tooth_profile(&g).unwrap();
        let area = signed_area(&gear::outline(&g, &prof)).abs() - PI * (g.bore / 2.0).powi(2);
        let want = area * g.face_width;
        assert!((m.volume() - want).abs() / want < 1e-3, "{} vs {want}", m.volume());
    }

    #[test]
    fn files_are_well_formed() {
        let p = project();
        let parts = parts(&p, 0.0, None, |_| true, |_| None);
        let tris: usize = parts.iter().map(|p| p.mesh.tris.len()).sum();
        let b = stl(&parts, "t");
        assert_eq!(b.len(), 84 + 50 * tris);
        assert_eq!(u32::from_le_bytes(b[80..84].try_into().unwrap()) as usize, tris);
        let o = obj(&parts);
        assert_eq!(o.lines().filter(|l| l.starts_with("f ")).count(), tris);
        assert_eq!(o.lines().filter(|l| l.starts_with("o ")).count(), parts.len());
        let z = three_mf(&parts, "t").unwrap();
        // The central directory: each entry where it says, and the model
        // inflates back to what was written.
        let end = z.len() - 22;
        assert_eq!(&z[end..end + 4], &0x06054b50u32.to_le_bytes());
        let count = u16::from_le_bytes([z[end + 10], z[end + 11]]) as usize;
        let mut at = u32::from_le_bytes(z[end + 16..end + 20].try_into().unwrap()) as usize;
        for _ in 0..count {
            assert_eq!(&z[at..at + 4], &0x02014b50u32.to_le_bytes(), "central entry");
            let name_len = u16::from_le_bytes([z[at + 28], z[at + 29]]) as usize;
            let local = u32::from_le_bytes(z[at + 42..at + 46].try_into().unwrap()) as usize;
            assert_eq!(&z[local..local + 4], &0x04034b50u32.to_le_bytes(), "local header");
            let name = std::str::from_utf8(&z[at + 46..at + 46 + name_len]).unwrap();
            if name == "3D/3dmodel.model" {
                let packed = u32::from_le_bytes(z[local + 18..local + 22].try_into().unwrap()) as usize;
                let start = local + 30 + name_len;
                let mut text = String::new();
                std::io::Read::read_to_string(&mut flate2::read::DeflateDecoder::new(&z[start..start + packed]), &mut text).unwrap();
                assert_eq!(text.matches("<triangle ").count(), tris);
            }
            at += 46 + name_len;
        }
    }

    /// `GEARCRAFT_MESH_OUT=dir cargo test write_mesh_samples` writes the
    /// sample train as STL, OBJ and 3MF.
    #[test]
    fn write_mesh_samples() {
        let Ok(dir) = std::env::var("GEARCRAFT_MESH_OUT") else { return };
        let dir = std::path::Path::new(&dir);
        let p = project();
        let parts = parts(&p, 0.0, None, |_| true, |_| None);
        for ext in ["stl", "obj", "3mf"] {
            write(&dir.join(format!("train.{ext}")), &parts).unwrap();
        }
    }
}


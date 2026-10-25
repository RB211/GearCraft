//! The 3D view: parts as solids of their face width, drawn by a small
//! software renderer (orthographic camera, flat shading, back faces
//! skipped, a depth buffer) into an image shown as a texture. Gear trains
//! are a few tens of thousands of triangles, comfortably drawn per frame.

use crate::bevel::BevelGeom;
use crate::export;
use crate::gear::{self, FillMesh, GearParams, Profile};
use crate::hypoid::Member;
use crate::pulley::BeltLoop;
use crate::train::Frame3;
use eframe::egui::{self, Color32, Pos2, Rect};
use std::f64::consts::{FRAC_PI_2, TAU};

/// Triangles in a part's own frame: axis along z, back face at z = 0.
#[derive(Default)]
pub struct Solid {
    pub verts: Vec<[f64; 3]>,
    /// Indices and the outward normal.
    pub tris: Vec<([u32; 3], [f64; 3])>,
}

impl Solid {
    fn vert(&mut self, p: [f64; 3]) -> u32 {
        self.verts.push(p);
        self.verts.len() as u32 - 1
    }

    fn quad(&mut self, a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3], n: [f64; 3]) {
        let (ia, ib, ic, id) = (self.vert(a), self.vert(b), self.vert(c), self.vert(d));
        self.tris.push(([ia, ib, ic], n));
        self.tris.push(([ia, ic, id], n));
    }

    /// Walls standing on a closed loop from z0 to z1. `outward` says which
    /// side the normals face: true for a CCW outer boundary.
    fn walls(&mut self, lp: &[[f64; 2]], z0: f64, z1: f64, outward: bool) {
        let n = lp.len();
        for i in 0..n {
            let (p, q) = (lp[i], lp[(i + 1) % n]);
            let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
            let l = dx.hypot(dy);
            if l < 1e-12 {
                continue;
            }
            let s = if outward { 1.0 } else { -1.0 };
            let nrm = [s * dy / l, -s * dx / l, 0.0];
            self.quad([p[0], p[1], z0], [q[0], q[1], z0], [q[0], q[1], z1], [p[0], p[1], z1], nrm);
        }
    }

    /// A planar cap from a 2D triangulation, at height z facing +z or -z.
    fn cap(&mut self, verts: &[[f64; 2]], tris: &[[u32; 3]], z: f64, up: bool) {
        let base = self.verts.len() as u32;
        self.verts.extend(verts.iter().map(|v| [v[0], v[1], z]));
        let n = [0.0, 0.0, if up { 1.0 } else { -1.0 }];
        self.tris.extend(tris.iter().map(|t| ([base + t[0], base + t[1], base + t[2]], n)));
    }
}

/// A closed bulge path (as fitted for export) back to points, with arcs
/// split into steps of at most `max_step` radians.
pub fn sample_path(path: &[([f64; 2], f64)], max_step: f64) -> Vec<[f64; 2]> {
    let n = path.len();
    let mut out = Vec::with_capacity(n * 2);
    for i in 0..n {
        let (p, bulge) = path[i];
        let q = path[(i + 1) % n].0;
        out.push(p);
        if bulge.abs() < 1e-12 {
            continue;
        }
        let sweep = 4.0 * bulge.atan();
        let steps = (sweep.abs() / max_step).ceil() as usize;
        if steps < 2 {
            continue;
        }
        // Centre from the chord and the sweep.
        let (mx, my) = ((p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0);
        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
        let chord = dx.hypot(dy);
        let h = chord / 2.0 / (sweep / 2.0).tan();
        let c = [mx - dy / chord * h, my + dx / chord * h];
        let r = (p[0] - c[0]).hypot(p[1] - c[1]);
        let a0 = (p[1] - c[1]).atan2(p[0] - c[0]);
        for k in 1..steps {
            let a = a0 + sweep * k as f64 / steps as f64;
            out.push([c[0] + r * a.cos(), c[1] + r * a.sin()]);
        }
    }
    out
}

/// A spur gear or pulley: its outline (arc-fitted, then sampled coarsely
/// enough to stay light) as walls, the bore, and the fill mesh as caps.
pub fn gear_solid(p: &GearParams, prof: &Profile, fill: &FillMesh) -> Solid {
    let outline = gear::outline(p, prof);
    let path = export::fit_arcs(&outline, export::tolerance(p.module) * 20.0);
    let lp = sample_path(&path, 0.35);
    let h = p.face_width.max(0.01);
    let mut s = Solid::default();
    s.walls(&lp, 0.0, h, true);
    if p.bore > 0.0 {
        let r = p.bore / 2.0;
        let n = 64;
        let hole: Vec<[f64; 2]> =
            (0..n).map(|i| i as f64 * TAU / n as f64).map(|a| [r * a.cos(), r * a.sin()]).collect();
        s.walls(&hole, 0.0, h, false);
    }
    s.cap(&fill.verts, &fill.tris, h, true);
    s.cap(&fill.verts, &fill.tris, 0.0, false);
    s
}

/// A bevel gear about its cone apex (axis +z, back face furthest out):
/// each tooth outline point is a ray from the apex, the flanks run along
/// those rays between the front and back cones, and the tooth ends, flat
/// faces and bore close it off.
pub fn bevel_solid(g: &BevelGeom) -> Solid {
    let k = g.front_ratio();
    let ray = |gamma: f64, phi: f64| BevelGeom::point(gamma, phi, 1.0);
    let mut s = Solid::default();
    let lp = g.outline();
    let n = lp.len();
    for i in 0..n {
        let ([ga, fa], [gb, fb]) = (lp[i], lp[(i + 1) % n]);
        let (ra, rb) = (ray(ga, fa), ray(gb, fb));
        let (ba, bb) = (g.back(ga), g.back(gb));
        let outer = |r: [f64; 3], d: f64| [r[0] * d, r[1] * d, r[2] * d];
        let (pa, pb) = (outer(ra, ba), outer(rb, bb));
        let t = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
        // Outward lies to the right of travel, seen from outside the
        // sphere: t × r.
        let nrm = normalize(cross(t, normalize([ra[0] + rb[0], ra[1] + rb[1], ra[2] + rb[2]])));
        s.quad(outer(ra, k * ba), outer(rb, k * bb), pb, pa, nrm);
    }
    // Tooth ends on the back and front cones, a strip per tooth from the
    // root up; the bottom row lies on the faces' rim circle.
    let z = g.teeth as usize;
    let (radii, half) = (&g.profile.radii, &g.profile.half);
    let across = 6;
    let pitch = TAU / z as f64;
    for (scale, sign) in [(1.0, 1.0), (k, -1.0)] {
        for t in 0..z {
            let c = t as f64 * pitch;
            let row = |i: usize, j: usize| {
                let phi = c - half[i] + 2.0 * half[i] * j as f64 / across as f64;
                let r = ray(radii[i], phi);
                let d = scale * g.back(radii[i]);
                [r[0] * d, r[1] * d, r[2] * d]
            };
            let nrm = |j: usize| {
                let phi = c + half[0] * (2.0 * j as f64 / across as f64 - 1.0);
                let r = ray(g.delta, phi);
                [sign * r[0], sign * r[1], sign * r[2]]
            };
            for i in 0..radii.len() - 1 {
                for j in 0..across {
                    let (a, b, cc, d) = (row(i, j), row(i, j + 1), row(i + 1, j + 1), row(i + 1, j));
                    if sign > 0.0 { s.quad(a, b, cc, d, nrm(j)) } else { s.quad(b, a, d, cc, nrm(j)) }
                }
            }
        }
    }
    // Flat back and front faces: annuli from the bore out to the root
    // circle, through the same azimuths as the tooth ends' bottom rows.
    let mut rim: Vec<f64> = Vec::new();
    for t in 0..z {
        let c = t as f64 * pitch;
        for j in 0..across {
            rim.push(c - half[0] + 2.0 * half[0] * j as f64 / across as f64);
        }
        let span = pitch - 2.0 * half[0];
        let steps = ((span / 0.05).ceil() as usize).max(2);
        for j in 0..steps {
            rim.push(c + half[0] + span * j as f64 / steps as f64);
        }
    }
    let rb = (g.bore / 2.0).max(0.0);
    for ((zf, rf), up) in [(g.back_face(), true), (g.front_face(), false)] {
        let m = rim.len();
        for i in 0..m {
            let (a, b) = (rim[i], rim[(i + 1) % m]);
            let o = |phi: f64, r: f64| [r * phi.cos(), r * phi.sin(), zf];
            let nrm = [0.0, 0.0, if up { 1.0 } else { -1.0 }];
            if rb > 0.0 {
                if up { s.quad(o(a, rb), o(b, rb), o(b, rf), o(a, rf), nrm) } else { s.quad(o(b, rb), o(a, rb), o(a, rf), o(b, rf), nrm) }
            } else {
                let (o0, oa, ob) = (s.vert(o(0.0, 0.0)), s.vert(o(a, rf)), s.vert(o(b, rf)));
                s.tris.push((if up { [o0, oa, ob] } else { [o0, ob, oa] }, nrm));
            }
        }
    }
    if rb > 0.0 {
        let m = 64;
        let hole: Vec<[f64; 2]> = (0..m).map(|i| i as f64 * TAU / m as f64).map(|a| [rb * a.cos(), rb * a.sin()]).collect();
        s.walls(&hole, g.front_face().0, g.back_face().0, false);
    }
    s
}

/// A hypoid gear or pinion from its generated sections (axis +z, pitch
/// apex at the origin): flank grids, tip lands, the toe and heel end cones
/// down to the hub circles, flat faces and the bore.
pub fn hypoid_solid(m: &Member) -> Solid {
    let mut s = Solid::default();
    let z = m.teeth as usize;
    let pitch = m.pitch();
    let last = m.stations.len() - 1;
    let (sg, cg) = m.blank.pitch.sin_cos();
    // Outward directions: along the cone, and off its surface.
    let along = |phi: f64| [sg * phi.cos(), sg * phi.sin(), cg];
    let off = |phi: f64| [cg * phi.cos(), cg * phi.sin(), -sg];
    let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    // A grid of points as quads, its normals turned to agree with `hint`
    // (one outward direction somewhere in the middle).
    let grid = |s: &mut Solid, g: &[Vec<[f64; 3]>], hint: [f64; 3]| {
        let (ni, nj) = (g.len(), g[0].len());
        let q = |i: usize, j: usize| cross(sub(g[i + 1][j], g[i][j]), sub(g[i][j + 1], g[i][j]));
        let flip = if dot(q(ni / 2 - (ni > 2) as usize, nj / 2 - (nj > 2) as usize), hint) < 0.0 { -1.0 } else { 1.0 };
        for i in 0..ni - 1 {
            for j in 0..nj - 1 {
                let c = cross(sub(g[i + 1][j + 1], g[i][j]), sub(g[i][j + 1], g[i + 1][j]));
                let l = dot(c, c).sqrt();
                if l < 1e-15 {
                    continue;
                }
                let n = [flip * c[0] / l, flip * c[1] / l, flip * c[2] / l];
                s.quad(g[i][j], g[i + 1][j], g[i + 1][j + 1], g[i][j + 1], n);
            }
        }
    };
    let tip_steps = 4;
    for tooth in 0..z {
        let rot = tooth as f64 * pitch;
        let (right, left) = (m.flank(false, tooth), m.flank(true, tooth));
        let mid = m.stations.len() / 2;
        let at = |side: &Vec<Vec<[f64; 2]>>, j: usize| side[mid][j][1] + rot;
        let pm = |side: &Vec<Vec<[f64; 2]>>| at(side, side[mid].len() / 2);
        let e_phi = |phi: f64, sign: f64| [-sign * phi.sin(), sign * phi.cos(), 0.0];
        grid(&mut s, &right, e_phi(pm(&m.right), -1.0));
        grid(&mut s, &left, e_phi(pm(&m.left), 1.0));
        // Tip land between the flanks' top points.
        let tip: Vec<Vec<[f64; 3]>> = (0..=last)
            .map(|k| {
                let (a, b) = (*m.right[k].last().unwrap(), *m.left[k].last().unwrap());
                (0..=tip_steps)
                    .map(|i| {
                        let f = i as f64 / tip_steps as f64;
                        m.point(m.stations[k], a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f + rot)
                    })
                    .collect()
            })
            .collect();
        let top = m.right[mid].len() - 1;
        grid(&mut s, &tip, off((at(&m.right, top) + at(&m.left, top)) / 2.0));
    }
    // Toe and heel ends: per tooth, the section from the hub circle up
    // round the tooth, clipped into triangles on the (φ, t) plane.
    let hub_steps = 12;
    for (e, k) in [(0usize, 0usize), (1, last)] {
        let st = m.stations[k];
        let ht = m.hub[e];
        let scale = st * sg;
        let base = m.right[k][0][1];
        let mut poly: Vec<[f64; 2]> = vec![[base, ht]];
        poly.extend(m.right[k].iter().map(|&[t, phi]| [phi, t]));
        poly.extend(m.left[k].iter().rev().map(|&[t, phi]| [phi, t]));
        let end = m.left[k][0][1];
        poly.push([end, ht]);
        for i in 1..hub_steps {
            poly.push([end + (base - end) * i as f64 / hub_steps as f64, ht]);
        }
        let flat: Vec<[f64; 2]> = poly.iter().map(|&[phi, t]| [phi * scale, t]).collect();
        let tris = ear_clip(&flat);
        let sign = if e == 0 { -1.0 } else { 1.0 };
        for tooth in 0..z {
            let rot = tooth as f64 * pitch;
            let pts: Vec<[f64; 3]> = poly.iter().map(|&[phi, t]| m.point(st, t, phi + rot)).collect();
            let hint = along(base + rot + pitch / 2.0);
            for t in &tris {
                let [a, b, c] = t.map(|i| pts[i]);
                let n = cross(sub(b, a), sub(c, a));
                let l = dot(n, n).sqrt();
                if l < 1e-15 {
                    continue;
                }
                let f = if dot(n, hint) * sign < 0.0 { -1.0 } else { 1.0 };
                let n = [f * n[0] / l, f * n[1] / l, f * n[2] / l];
                let (ia, ib, ic) = (s.vert(a), s.vert(b), s.vert(c));
                s.tris.push(([ia, ib, ic], n));
            }
        }
    }
    // Flat faces from the hub circles in to the bore, facing away from
    // each other, and the bore between them.
    let faces = m.hub_faces();
    let rb = (m.bore / 2.0).max(0.0);
    let segs = 128;
    for e in 0..2 {
        let (zf, rf) = faces[e];
        let up = zf > faces[1 - e].0;
        let nrm = [0.0, 0.0, if up { 1.0 } else { -1.0 }];
        for i in 0..segs {
            let (a, b) = (i as f64 * TAU / segs as f64, (i + 1) as f64 * TAU / segs as f64);
            let o = |phi: f64, r: f64| [r * phi.cos(), r * phi.sin(), zf];
            if rb > 0.0 {
                s.quad(o(a, rb), o(b, rb), o(b, rf), o(a, rf), nrm);
            } else {
                let (o0, oa, ob) = (s.vert(o(0.0, 0.0)), s.vert(o(a, rf)), s.vert(o(b, rf)));
                s.tris.push(([o0, oa, ob], nrm));
            }
        }
    }
    if rb > 0.0 {
        let hole: Vec<[f64; 2]> = (0..64).map(|i| i as f64 * TAU / 64.0).map(|a| [rb * a.cos(), rb * a.sin()]).collect();
        let (z0, z1) = (faces[0].0.min(faces[1].0), faces[0].0.max(faces[1].0));
        s.walls(&hole, z0, z1, false);
    }
    s
}

/// Triangles filling a simple polygon, by ear clipping.
pub fn ear_clip(poly: &[[f64; 2]]) -> Vec<[usize; 3]> {
    let n = poly.len();
    let area: f64 = (0..n).map(|i| {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        a[0] * b[1] - b[0] * a[1]
    }).sum();
    let mut idx: Vec<usize> = if area >= 0.0 { (0..n).collect() } else { (0..n).rev().collect() };
    let cr = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    let mut out = Vec::with_capacity(n.saturating_sub(2));
    while idx.len() > 3 {
        let m = idx.len();
        let ear = (0..m).find(|&i| {
            let (a, b, c) = (poly[idx[(i + m - 1) % m]], poly[idx[i]], poly[idx[(i + 1) % m]]);
            if cr(a, b, c) <= 0.0 {
                return false;
            }
            !idx.iter().enumerate().any(|(j, &p)| {
                if j == i || j == (i + m - 1) % m || j == (i + 1) % m {
                    return false;
                }
                let q = poly[p];
                cr(a, b, q) >= 0.0 && cr(b, c, q) >= 0.0 && cr(c, a, q) >= 0.0
            })
        });
        // A degenerate remainder: cut the flattest corner.
        let i = ear.unwrap_or_else(|| {
            (0..m)
                .max_by(|&x, &y| {
                    let f = |i: usize| cr(poly[idx[(i + m - 1) % m]], poly[idx[i]], poly[idx[(i + 1) % m]]);
                    f(x).total_cmp(&f(y))
                })
                .unwrap()
        });
        out.push([idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]]);
        idx.remove(i);
    }
    if idx.len() == 3 {
        out.push([idx[0], idx[1], idx[2]]);
    }
    out
}

/// The belt as a solid in world coordinates: backing and teeth, `width`
/// wide, centred on `z_mid`. `teeth` gives each tooth's arc-length
/// station; `land`/`back`/`tooth` are offsets from the pitch line.
#[allow(clippy::too_many_arguments)]
pub fn belt_solid(
    lp: &BeltLoop,
    stations: &[f64],
    land: f64,
    back: f64,
    depth: f64,
    half_base: f64,
    half_tip: f64,
    z_mid: f64,
    width: f64,
) -> Solid {
    // Walk the loop once, emitting (s, inner offset) pairs: the land
    // between teeth and the four corners of each tooth.
    let len = lp.len();
    let mut marks: Vec<(f64, f64)> = Vec::new();
    let mut sorted: Vec<f64> = stations.iter().map(|s| s.rem_euclid(len)).collect();
    sorted.sort_by(f64::total_cmp);
    let step = (len / 600.0).max(0.2);
    for (k, &c) in sorted.iter().enumerate() {
        let next = if k + 1 < sorted.len() { sorted[k + 1] } else { sorted[0] + len };
        marks.push((c - half_base, land));
        marks.push((c - half_tip, land - depth));
        marks.push((c + half_tip, land - depth));
        marks.push((c + half_base, land));
        let (a, b) = (c + half_base, next - half_base);
        let n = ((b - a) / step).ceil().max(1.0) as usize;
        for i in 1..n {
            marks.push((a + (b - a) * i as f64 / n as f64, land));
        }
    }
    let point = |s: f64, off: f64| {
        let (p, n) = lp.at(s);
        [p[0] + n[0] * off, p[1] + n[1] * off]
    };
    let inner: Vec<[f64; 2]> = marks.iter().map(|&(s, off)| point(s, off)).collect();
    let outer: Vec<[f64; 2]> = marks.iter().map(|&(s, _)| point(s, back)).collect();
    let (z0, z1) = (z_mid - width / 2.0, z_mid + width / 2.0);
    let mut sol = Solid::default();
    // The loop runs CCW, so the outer boundary faces out, the inner in.
    sol.walls(&outer, z0, z1, true);
    sol.walls(&inner, z0, z1, false);
    let n = marks.len();
    for (z, up) in [(z1, true), (z0, false)] {
        let nrm = [0.0, 0.0, if up { 1.0 } else { -1.0 }];
        for i in 0..n {
            let j = (i + 1) % n;
            let q = |p: [f64; 2]| [p[0], p[1], z];
            sol.quad(q(inner[i]), q(inner[j]), q(outer[j]), q(outer[i]), nrm);
        }
    }
    sol
}

/// Orbit camera looking at `target`; z is up in the world.
#[derive(Clone, Copy)]
pub struct Camera {
    pub target: [f64; 3],
    /// Round the vertical axis and up from the horizontal, radians.
    pub yaw: f64,
    pub pitch: f64,
    /// Pixels per mm.
    pub scale: f64,
}

impl Default for Camera {
    fn default() -> Self {
        Camera { target: [0.0; 3], yaw: -FRAC_PI_2 - 0.6, pitch: 0.6, scale: 4.0 }
    }
}

impl Camera {
    /// Unit vectors: right, up, and forward (into the screen).
    fn basis(&self) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        let f = [-cp * cy, -cp * sy, -sp];
        let r = [-sy, cy, 0.0];
        let u = cross(r, f);
        (r, u, f)
    }

    pub fn orbit(&mut self, dx: f64, dy: f64) {
        self.yaw -= dx * 0.008;
        self.pitch = (self.pitch + dy * 0.008).clamp(-1.55, 1.55);
    }

    pub fn pan(&mut self, dx: f64, dy: f64) {
        let (r, u, _) = self.basis();
        for k in 0..3 {
            self.target[k] -= (r[k] * dx - u[k] * dy) / self.scale;
        }
    }

    /// Zoom by `factor` keeping the point under `at` (screen) fixed.
    pub fn zoom(&mut self, rect: Rect, at: Pos2, factor: f64) {
        let (r, u, _) = self.basis();
        let (dx, dy) = ((at.x - rect.center().x) as f64, (at.y - rect.center().y) as f64);
        let before = self.scale;
        self.scale = (self.scale * factor).clamp(0.02, 5000.0);
        let k = 1.0 / before - 1.0 / self.scale;
        for i in 0..3 {
            self.target[i] += (r[i] * dx - u[i] * dy) * k;
        }
    }

    pub fn project(&self, rect: Rect, p: [f64; 3]) -> (Pos2, f64) {
        let (r, u, f) = self.basis();
        let d = [p[0] - self.target[0], p[1] - self.target[1], p[2] - self.target[2]];
        let c = rect.center();
        (Pos2::new(c.x + (dot(d, r) * self.scale) as f32, c.y - (dot(d, u) * self.scale) as f32), dot(d, f))
    }

    /// Frame a box of world points in `rect`.
    pub fn fit(&mut self, rect: Rect, lo: [f64; 3], hi: [f64; 3]) {
        self.target = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0];
        let mut w: f64 = 1.0;
        let mut h: f64 = 1.0;
        let (r, u, _) = self.basis();
        for i in 0..8 {
            let p = [
                if i & 1 == 0 { lo[0] } else { hi[0] },
                if i & 2 == 0 { lo[1] } else { hi[1] },
                if i & 4 == 0 { lo[2] } else { hi[2] },
            ];
            let d = [p[0] - self.target[0], p[1] - self.target[1], p[2] - self.target[2]];
            w = w.max(2.0 * dot(d, r).abs());
            h = h.max(2.0 * dot(d, u).abs());
        }
        self.scale = (rect.width() as f64 / w).min(rect.height() as f64 / h) * 0.85;
    }
}

/// A solid placed in the world: turned by `rot` about its own z axis,
/// then set in `frame`.
pub struct Placed<'a> {
    pub solid: &'a Solid,
    pub frame: Frame3,
    pub rot: f64,
    pub color: Color32,
    pub id: u64,
}

/// One rendered frame: the picture, and which part each pixel shows.
pub struct Frame {
    pub image: egui::ColorImage,
    ids: Vec<u32>,
    parts: Vec<u64>,
    origin: Pos2,
    ppp: f32,
}

impl Default for Frame {
    fn default() -> Self {
        Frame { image: egui::ColorImage::new([1, 1], vec![Color32::TRANSPARENT]), ids: vec![u32::MAX], parts: Vec::new(), origin: Pos2::ZERO, ppp: 1.0 }
    }
}

impl Frame {
    /// The part showing at screen point `p`.
    pub fn pick(&self, p: Pos2) -> Option<u64> {
        let [w, h] = self.image.size;
        let x = ((p.x - self.origin.x) * self.ppp).floor();
        let y = ((p.y - self.origin.y) * self.ppp).floor();
        if x < 0.0 || y < 0.0 || x as usize >= w || y as usize >= h {
            return None;
        }
        let id = self.ids[y as usize * w + x as usize];
        self.parts.get(id as usize).copied()
    }
}

/// A screen-space triangle ready to fill: corners in pixels, depth as a
/// plane z = a·x + b·y + c, flat colour and the part index.
struct Tri {
    p: [[f32; 2]; 3],
    plane: [f32; 3],
    y0: f32,
    y1: f32,
    color: Color32,
    part: u32,
}

/// Rasterise the parts into `rect` (at `ppp` pixels per point) with a
/// depth buffer, so faces hide each other properly whatever their size.
/// Horizontal bands of the image are filled on separate threads.
pub fn render(rect: Rect, ppp: f32, cam: &Camera, parts: &[Placed], background: Color32) -> Frame {
    let w = ((rect.width() * ppp).round() as usize).max(1);
    let h = ((rect.height() * ppp).round() as usize).max(1);
    let (r, u, f) = cam.basis();
    // Light from over the viewer's left shoulder.
    let light = normalize([-f[0] + 0.5 * u[0] - 0.4 * r[0], -f[1] + 0.5 * u[1] - 0.4 * r[1], -f[2] + 0.5 * u[2] - 0.4 * r[2]]);
    let mut tris: Vec<Tri> = Vec::new();
    for (pi, part) in parts.iter().enumerate() {
        let (s, c) = part.rot.sin_cos();
        let frame = part.frame;
        let world = |p: [f64; 3]| frame.apply([p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2]]);
        let screen: Vec<[f32; 3]> = part
            .solid
            .verts
            .iter()
            .map(|&v| {
                let (q, d) = cam.project(rect, world(v));
                [(q.x - rect.min.x) * ppp, (q.y - rect.min.y) * ppp, d as f32]
            })
            .collect();
        let [cr, cg, cb, _] = part.color.to_array();
        for (t, n) in &part.solid.tris {
            let n = frame.turn([n[0] * c - n[1] * s, n[0] * s + n[1] * c, n[2]]);
            if dot(n, f) >= 0.0 {
                continue;
            }
            let [a, b, d] = [screen[t[0] as usize], screen[t[1] as usize], screen[t[2] as usize]];
            let area = (b[0] - a[0]) * (d[1] - a[1]) - (b[1] - a[1]) * (d[0] - a[0]);
            let (y0, y1) = (a[1].min(b[1]).min(d[1]), a[1].max(b[1]).max(d[1]));
            let (x0, x1) = (a[0].min(b[0]).min(d[0]), a[0].max(b[0]).max(d[0]));
            if area.abs() < 1e-6 || y1 < 0.0 || y0 > h as f32 || x1 < 0.0 || x0 > w as f32 {
                continue;
            }
            // Depth plane through the three corners.
            let (e1, e2) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [d[0] - a[0], d[1] - a[1], d[2] - a[2]]);
            let nz = e1[0] * e2[1] - e1[1] * e2[0];
            let pa = -(e1[1] * e2[2] - e1[2] * e2[1]) / nz;
            let pb = -(e1[2] * e2[0] - e1[0] * e2[2]) / nz;
            let plane = [pa, pb, a[2] - pa * a[0] - pb * a[1]];
            let k = (0.38 + 0.62 * dot(n, light).max(0.0)) as f32;
            let shade = |v: u8| (v as f32 * k).min(255.0) as u8;
            tris.push(Tri {
                p: [[a[0], a[1]], [b[0], b[1]], [d[0], d[1]]],
                plane,
                y0,
                y1,
                color: Color32::from_rgb(shade(cr), shade(cg), shade(cb)),
                part: pi as u32,
            });
        }
    }

    let mut color = vec![background; w * h];
    let mut ids = vec![u32::MAX; w * h];
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get()).clamp(1, 16);
    let band = h.div_ceil(threads);
    std::thread::scope(|scope| {
        for (k, (cs, is)) in color.chunks_mut(band * w).zip(ids.chunks_mut(band * w)).enumerate() {
            let tris = &tris;
            scope.spawn(move || {
                let top = k * band;
                let rows = cs.len() / w;
                let mut depth = vec![f32::INFINITY; cs.len()];
                for t in tris {
                    if t.y1 < top as f32 || t.y0 > (top + rows) as f32 {
                        continue;
                    }
                    fill(t, w, top, rows, |i, z| {
                        if z < depth[i] {
                            depth[i] = z;
                            cs[i] = t.color;
                            is[i] = t.part;
                        }
                    });
                }
            });
        }
    });
    soften_edges(&mut color, &ids, w, h);
    let image = egui::ColorImage::new([w, h], color);
    Frame { image, ids, parts: parts.iter().map(|p| p.id).collect(), origin: rect.min, ppp }
}

/// Fill one triangle within rows `top..top + rows`, a row at a time over
/// just the span it covers, calling `put(index in the band, depth)` for
/// each pixel centre inside.
fn fill(t: &Tri, w: usize, top: usize, rows: usize, mut put: impl FnMut(usize, f32)) {
    let r0 = (t.y0 - 0.5).ceil().max(top as f32) as usize;
    let r1 = ((t.y1 - 0.5).floor() as isize).min((top + rows) as isize - 1);
    if r1 < r0 as isize {
        return;
    }
    for y in r0..=r1 as usize {
        let py = y as f32 + 0.5;
        let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
        for e in 0..3 {
            let (a, b) = (t.p[e], t.p[(e + 1) % 3]);
            if (a[1] <= py && b[1] > py) || (b[1] <= py && a[1] > py) {
                let x = a[0] + (py - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
                lo = lo.min(x);
                hi = hi.max(x);
            }
        }
        if lo > hi {
            continue;
        }
        let x0 = (lo - 0.5).ceil().max(0.0) as usize;
        let x1 = ((hi - 0.5).floor() as isize).min(w as isize - 1);
        if x1 < x0 as isize {
            continue;
        }
        let row = (y - top) * w;
        let mut z = t.plane[0] * (x0 as f32 + 0.5) + t.plane[1] * py + t.plane[2];
        for x in x0..=x1 as usize {
            put(row + x, z);
            z += t.plane[0];
        }
    }
}

/// Cheap anti-aliasing: where two parts (or a part and the background)
/// meet, blend the pixel a little with its neighbours.
fn soften_edges(color: &mut [Color32], ids: &[u32], w: usize, h: usize) {
    let src = color.to_vec();
    for y in 1..h.saturating_sub(1) {
        for x in 1..w.saturating_sub(1) {
            let i = y * w + x;
            let id = ids[i];
            if ids[i - 1] == id && ids[i + 1] == id && ids[i - w] == id && ids[i + w] == id {
                continue;
            }
            let mut acc = [0u32; 3];
            for j in [i, i, i - 1, i + 1, i - w, i + w] {
                let c = src[j];
                acc[0] += c.r() as u32;
                acc[1] += c.g() as u32;
                acc[2] += c.b() as u32;
            }
            color[i] = Color32::from_rgb((acc[0] / 6) as u8, (acc[1] / 6) as u8, (acc[2] / 6) as u8);
        }
    }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn normalize(a: [f64; 3]) -> [f64; 3] {
    let l = dot(a, a).sqrt();
    [a[0] / l, a[1] / l, a[2] / l]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sampled_path_stays_on_the_outline() {
        let p = GearParams::default();
        let prof = gear::tooth_profile(&p).unwrap();
        let outline = gear::outline(&p, &prof);
        let path = export::fit_arcs(&outline, export::tolerance(p.module) * 20.0);
        let pts = sample_path(&path, 0.35);
        assert!(pts.len() < outline.len() / 3, "{} vs {}", pts.len(), outline.len());
        let rmax = pts.iter().map(|q| q[0].hypot(q[1])).fold(0.0, f64::max);
        let rmin = pts.iter().map(|q| q[0].hypot(q[1])).fold(f64::MAX, f64::min);
        assert!((rmax - p.tip_radius()).abs() < 0.02 && (rmin - p.root_radius()).abs() < 0.02);
    }

    #[test]
    fn camera_fit_and_projection_agree() {
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(800.0, 600.0));
        let mut cam = Camera::default();
        cam.fit(rect, [-50.0, -50.0, 0.0], [50.0, 50.0, 10.0]);
        let (c, _) = cam.project(rect, cam.target);
        assert!((c - rect.center()).length() < 1e-3);
        for p in [[-50.0, -50.0, 0.0], [50.0, 50.0, 10.0], [50.0, -50.0, 10.0]] {
            assert!(rect.contains(cam.project(rect, p).0));
        }
    }
}

#[cfg(test)]
mod bench {
    use super::*;

    /// Not a check: prints how long a frame of a busy train takes.
    #[test]
    fn frame_time() {
        if std::env::var("GEARCRAFT_BENCH").is_err() {
            return;
        }
        let solids: Vec<Solid> = [16u32, 48, 14, 40, 72, 120]
            .iter()
            .map(|&z| {
                let p = GearParams { teeth: z, face_width: 10.0, bore: 8.0, ..Default::default() };
                let prof = gear::tooth_profile(&p).unwrap();
                gear_solid(&p, &prof, &gear::fill_mesh(&p, &prof))
            })
            .collect();
        let parts: Vec<Placed> = solids
            .iter()
            .enumerate()
            .map(|(i, s)| Placed { solid: s, frame: Frame3::at([i as f64 * 60.0, 0.0, 0.0]), rot: 0.0, color: Color32::GRAY, id: i as u64 })
            .collect();
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(840.0, 700.0));
        let mut cam = Camera::default();
        cam.fit(rect, [-60.0, -130.0, 0.0], [430.0, 130.0, 10.0]);
        let tris: usize = solids.iter().map(|s| s.tris.len()).sum();
        let t = std::time::Instant::now();
        for _ in 0..10 {
            render(rect, 1.0, &cam, &parts, Color32::BLACK);
        }
        println!("{tris} triangles: {:.1} ms/frame at 840x700", t.elapsed().as_secs_f64() * 100.0);
        let t = std::time::Instant::now();
        for _ in 0..10 {
            render(rect, 2.0, &cam, &parts, Color32::BLACK);
        }
        println!("at 2x: {:.1} ms/frame", t.elapsed().as_secs_f64() * 100.0);
    }
}

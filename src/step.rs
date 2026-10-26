//! STEP (ISO 10303-21, AP214) output: each part as an exact B-rep solid.
//!
//! A part is its arc-fitted outline (lines and circular arcs, the same as
//! the DXF) extruded through its face width, with the bore as a hole. So
//! every side face is a plane or a cylinder and the ends are planes; no
//! faceting. The bore is two half cylinders, which every importer accepts
//! without needing seam edges.
//!
//! A bevel gear is exact the same way: each flank is a cone through the
//! apex on a cubic B-spline (fitted to the spherical involute by its roll
//! angle), the tip and root lands are cones about the axis, the tooth ends
//! lie on the back and front cones, and the hub faces are planes.

use crate::bevel::BevelGeom;
use crate::export::{BevelPart, HypoidPart, Shape};
use crate::hypoid::Member;
use crate::train::Frame3;
use std::f64::consts::{FRAC_PI_2, TAU};
use std::fmt::Write;

/// One loop segment: from `a` to `b`, straight or an arc about `c`
/// (radius `r`) running anticlockwise if `ccw`.
#[derive(Clone, Copy)]
struct Seg {
    a: [f64; 2],
    b: [f64; 2],
    arc: Option<([f64; 2], f64, bool)>,
}

fn segments(path: &[([f64; 2], f64)]) -> Vec<Seg> {
    let n = path.len();
    (0..n)
        .filter_map(|i| {
            let (a, bulge) = path[i];
            let b = path[(i + 1) % n].0;
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let chord = dx.hypot(dy);
            if chord < 1e-9 {
                return None;
            }
            if bulge.abs() < 1e-9 {
                return Some(Seg { a, b, arc: None });
            }
            let sweep = 4.0 * bulge.atan();
            let h = chord / 2.0 / (sweep / 2.0).tan();
            let c = [(a[0] + b[0]) / 2.0 - dy / chord * h, (a[1] + b[1]) / 2.0 + dx / chord * h];
            let r = (a[0] - c[0]).hypot(a[1] - c[1]);
            Some(Seg { a, b, arc: Some((c, r, sweep > 0.0)) })
        })
        .collect()
}

/// The bore as two clockwise half circles (a hole runs clockwise).
fn hole(center: [f64; 2], r: f64) -> Vec<Seg> {
    let p = [center[0] + r, center[1]];
    let q = [center[0] - r, center[1]];
    vec![Seg { a: p, b: q, arc: Some((center, r, false)) }, Seg { a: q, b: p, arc: Some((center, r, false)) }]
}

struct Writer {
    out: String,
    next: usize,
    /// Set while writing a part built in its own coordinates.
    place: Option<(Frame3, f64)>,
}

impl Writer {
    fn to_world(&self, p: [f64; 3], is_point: bool) -> [f64; 3] {
        let Some((frame, rot)) = self.place else { return p };
        let (s, c) = rot.sin_cos();
        let q = [p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2]];
        if is_point { frame.apply(q) } else { frame.turn(q) }
    }

    fn add(&mut self, entity: impl AsRef<str>) -> usize {
        let id = self.next;
        self.next += 1;
        let _ = writeln!(self.out, "#{id}={};", entity.as_ref());
        id
    }

    fn point(&mut self, p: [f64; 3]) -> usize {
        let p = self.to_world(p, true);
        self.add(format!("CARTESIAN_POINT('',({},{},{}))", num(p[0]), num(p[1]), num(p[2])))
    }

    fn dir(&mut self, d: [f64; 3]) -> usize {
        let d = self.to_world(d, false);
        self.add(format!("DIRECTION('',({},{},{}))", num(d[0]), num(d[1]), num(d[2])))
    }

    fn axis(&mut self, at: [f64; 3], z: [f64; 3], x: [f64; 3]) -> usize {
        let (p, z, x) = (self.point(at), self.dir(z), self.dir(x));
        self.add(format!("AXIS2_PLACEMENT_3D('',#{p},#{z},#{x})"))
    }

    fn line(&mut self, a: [f64; 3], b: [f64; 3]) -> usize {
        let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let p = self.point(a);
        let dir = self.dir([d[0] / l, d[1] / l, d[2] / l]);
        let v = self.add(format!("VECTOR('',#{dir},{})", num(l)));
        self.add(format!("LINE('',#{p},#{v})"))
    }

    fn vertex(&mut self, p: [f64; 3]) -> usize {
        let p = self.point(p);
        self.add(format!("VERTEX_POINT('',#{p})"))
    }

    /// A circle about the axis at height z.
    fn circle(&mut self, z: f64, r: f64) -> usize {
        let ax = self.axis([0.0, 0.0, z], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
        self.add(format!("CIRCLE('',#{ax},{})", num(r)))
    }

    /// A cone about the axis, radius r at height z, widening by `semi`
    /// towards +z (or towards -z if `down`).
    fn cone(&mut self, z: f64, r: f64, semi: f64, down: bool) -> usize {
        let ax = self.axis([0.0, 0.0, z], [0.0, 0.0, if down { -1.0 } else { 1.0 }], [1.0, 0.0, 0.0]);
        self.add(format!("CONICAL_SURFACE('',#{ax},{},{})", num(r), num(semi)))
    }

    fn plane(&mut self, at: [f64; 3], normal: [f64; 3], x: [f64; 3]) -> usize {
        let ax = self.axis(at, normal, x);
        self.add(format!("PLANE('',#{ax})"))
    }

    fn edge(&mut self, va: usize, vb: usize, curve: usize, same: bool) -> usize {
        self.add(format!("EDGE_CURVE('',#{va},#{vb},#{curve},{})", flag(same)))
    }

    fn face(&mut self, surface: usize, same: bool, bounds: &[(Vec<(usize, bool)>, bool)]) -> usize {
        let mut ids = Vec::new();
        for (i, (edges, _)) in bounds.iter().enumerate() {
            let oriented: Vec<String> = edges
                .iter()
                .map(|&(e, fwd)| format!("#{}", self.add(format!("ORIENTED_EDGE('',*,*,#{e},{})", flag(fwd)))))
                .collect();
            let lp = self.add(format!("EDGE_LOOP('',({}))", oriented.join(",")));
            let kind = if i == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
            ids.push(format!("#{}", self.add(format!("{kind}('',#{lp},.T.)"))));
        }
        self.add(format!("ADVANCED_FACE('',({}),#{surface},{})", ids.join(","), flag(same)))
    }
}

fn num(v: f64) -> String {
    let v = if v.abs() < 5e-13 { 0.0 } else { v };
    let s = format!("{v:.10}");
    let s = s.trim_end_matches('0');
    if s.ends_with('.') { format!("{s}0") } else { s.to_string() }
}

fn flag(b: bool) -> &'static str {
    if b { ".T." } else { ".F." }
}

/// Edges, vertices and side faces for one closed loop, plus the loop's
/// bottom and top edges (in loop order) for the end faces.
struct LoopEdges {
    bottom: Vec<usize>,
    top: Vec<usize>,
}

fn walls(w: &mut Writer, segs: &[Seg], z0: f64, z1: f64, faces: &mut Vec<usize>) -> LoopEdges {
    let n = segs.len();
    let at = |p: [f64; 2], z: f64| [p[0], p[1], z];
    let mut v0 = Vec::with_capacity(n);
    let mut v1 = Vec::with_capacity(n);
    let mut uprights = Vec::with_capacity(n);
    for s in segs {
        let (p0, p1) = (w.point(at(s.a, z0)), w.point(at(s.a, z1)));
        let (a, b) = (w.add(format!("VERTEX_POINT('',#{p0})")), w.add(format!("VERTEX_POINT('',#{p1})")));
        let up = w.line(at(s.a, z0), at(s.a, z1));
        uprights.push(w.edge(a, b, up, true));
        v0.push(a);
        v1.push(b);
    }
    let mut bottom = Vec::with_capacity(n);
    let mut top = Vec::with_capacity(n);
    for (i, s) in segs.iter().enumerate() {
        let j = (i + 1) % n;
        let curve = |w: &mut Writer, z: f64| match s.arc {
            None => (w.line(at(s.a, z), at(s.b, z)), true),
            Some((c, r, ccw)) => {
                let ax = w.axis(at(c, z), [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
                (w.add(format!("CIRCLE('',#{ax},{})", num(r))), ccw)
            }
        };
        let (cb, sb) = curve(w, z0);
        let (ct, st) = curve(w, z1);
        let eb = w.edge(v0[i], v0[j], cb, sb);
        let et = w.edge(v1[i], v1[j], ct, st);
        // Outward is to the right of the direction of travel: out of the
        // material for an anticlockwise outline and a clockwise hole.
        let (surface, same) = match s.arc {
            None => {
                let (dx, dy) = (s.b[0] - s.a[0], s.b[1] - s.a[1]);
                let l = dx.hypot(dy);
                let ax = w.axis(at(s.a, z0), [dy / l, -dx / l, 0.0], [dx / l, dy / l, 0.0]);
                (w.add(format!("PLANE('',#{ax})")), true)
            }
            Some((c, r, ccw)) => {
                let ax = w.axis(at(c, z0), [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
                // A cylinder's normal points away from its axis: outward
                // for a convex (anticlockwise) arc, inward for a concave one.
                (w.add(format!("CYLINDRICAL_SURFACE('',#{ax},{})", num(r))), ccw)
            }
        };
        // Viewed from outside: along the bottom, up, back along the top, down.
        faces.push(w.face(surface, same, &[(vec![(eb, true), (uprights[j], true), (et, false), (uprights[i], false)], true)]));
        bottom.push(eb);
        top.push(et);
    }
    LoopEdges { bottom, top }
}

/// One shape as a MANIFOLD_SOLID_BREP; returns its id.
fn solid(w: &mut Writer, shape: &Shape) -> usize {
    let (z0, z1) = (shape.z, shape.z + shape.height.max(1e-3));
    let mut faces = Vec::new();
    let outer = walls(w, &segments(&shape.path), z0, z1, &mut faces);
    let inner = (shape.bore > 0.0).then(|| walls(w, &hole(shape.center, shape.bore / 2.0), z0, z1, &mut faces));
    let c = shape.center;
    for (z, up) in [(z1, true), (z0, false)] {
        let normal = [0.0, 0.0, if up { 1.0 } else { -1.0 }];
        let ax = w.axis([c[0], c[1], z], normal, [1.0, 0.0, 0.0]);
        let plane = w.add(format!("PLANE('',#{ax})"));
        // From above the loops run anticlockwise (outline) and clockwise
        // (hole) as stored; from below, both reverse.
        let bound = |edges: &[usize]| -> Vec<(usize, bool)> {
            if up { edges.iter().map(|&e| (e, true)).collect() } else { edges.iter().rev().map(|&e| (e, false)).collect() }
        };
        let pick = |l: &LoopEdges| if up { l.top.clone() } else { l.bottom.clone() };
        let mut bounds = vec![(bound(&pick(&outer)), true)];
        if let Some(h) = &inner {
            bounds.push((bound(&pick(h)), true));
        }
        faces.push(w.face(plane, true, &bounds));
    }
    let list: Vec<String> = faces.iter().map(|f| format!("#{f}")).collect();
    let shell = w.add(format!("CLOSED_SHELL('',({}))", list.join(",")));
    w.add(format!("MANIFOLD_SOLID_BREP('{}',#{shell})", text(&shape.name)))
}

fn scale(k: f64, p: [f64; 3]) -> [f64; 3] {
    [k * p[0], k * p[1], k * p[2]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn unit(a: [f64; 3]) -> [f64; 3] {
    let l = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    scale(1.0 / l, a)
}

/// How one stretch of a bevel tooth outline runs between two corner rays.
#[derive(Clone, Copy)]
enum Run {
    /// Flank below the base cone: in a plane through the axis.
    Radial,
    /// Involute flank, roll angle from `t0` to `t1`; `side` is -1 on the
    /// right flank (φ = c - half), +1 on the left.
    Involute { c: f64, side: f64, t0: f64, t1: f64 },
    Tip,
    Root,
}

/// A cubic B-spline through the involute flank on the back cone, as
/// Bézier pieces from exact points and tangents. Returns the control
/// points and knots (in travel order).
fn flank_curve(g: &BevelGeom, c: f64, side: f64, t0: f64, t1: f64) -> (Vec<[f64; 3]>, Vec<f64>) {
    let at = |theta: f64| {
        let (gamma, sweep) = g.roll(theta);
        BevelGeom::point(gamma, c + side * (g.base_half - sweep), g.back(gamma))
    };
    let n = 12;
    let ts: Vec<f64> = (0..=n).map(|i| t0 + (t1 - t0) * i as f64 / n as f64).collect();
    let eps = 1e-6 * (t1 - t0).abs().max(1e-3);
    let mut ctrl = vec![at(ts[0])];
    for i in 0..n {
        let (a, b) = (ts[i], ts[i + 1]);
        let h = b - a;
        let d = |t: f64| scale(1.0 / (2.0 * eps), sub(at(t + eps), at(t - eps)));
        let (pa, pb) = (at(a), at(b));
        let (da, db) = (d(a), d(b));
        ctrl.push([pa[0] + da[0] * h / 3.0, pa[1] + da[1] * h / 3.0, pa[2] + da[2] * h / 3.0]);
        ctrl.push([pb[0] - db[0] * h / 3.0, pb[1] - db[1] * h / 3.0, pb[2] - db[2] * h / 3.0]);
        ctrl.push(pb);
    }
    // Knots must increase along travel; the left flank rolls backwards.
    let knots = ts.iter().map(|t| (t - t0).abs()).collect();
    (ctrl, knots)
}

fn knot_lists(knots: &[f64]) -> (String, String) {
    let n = knots.len();
    let mults: Vec<&str> = (0..n).map(|i| if i == 0 || i == n - 1 { "4" } else { "3" }).collect();
    let values: Vec<String> = knots.iter().map(|&k| num(k)).collect();
    (mults.join(","), values.join(","))
}

/// One bevel gear as a MANIFOLD_SOLID_BREP, written in its own frame
/// (apex at the origin, axis +z) and placed by `w.place`. None if the
/// teeth are too degenerate to bound faces (pointed, or no root land).
fn bevel_solid(w: &mut Writer, g: &BevelGeom, name: &str) -> Option<usize> {
    let z = g.teeth as usize;
    let pitch = TAU / z as f64;
    let (ht, hr) = (g.half_at(g.tip), g.half_at(g.root));
    if ht <= 1e-6 || pitch - 2.0 * hr <= 1e-6 || g.tip >= FRAC_PI_2 - 1e-3 {
        return None;
    }
    let k = g.front_ratio();
    let back = |gamma: f64, phi: f64| BevelGeom::point(gamma, phi, g.back(gamma));
    let (zb, rbk) = g.back_face();
    let (zf, rfk) = g.front_face();
    let tb = back(g.tip, 0.0);
    let (tz, tr) = (tb[2], tb[0]);
    let (r_out, d) = (g.cone_distance, g.delta);

    let root_b = w.circle(zb, rbk);
    let root_f = w.circle(zf, rfk);
    let tip_b = w.circle(tz, tr);
    let tip_f = w.circle(k * tz, k * tr);
    let root_cone = w.cone(zb, rbk, g.root, false);
    let tip_cone = w.cone(tz, tr, g.tip, false);
    let back_cone = w.cone(r_out * d.cos(), r_out * d.sin(), FRAC_PI_2 - d, true);
    let front_cone = w.cone(k * r_out * d.cos(), k * r_out * d.sin(), FRAC_PI_2 - d, true);

    // Corner rays round the whole gear in outline order, and the runs
    // between them. Below the base cone a flank is radial.
    let radial = g.base > g.root + 1e-9;
    let g_inv = g.base.max(g.root);
    let (th0, th1) = (g.roll_at(g_inv), g.roll_at(g.tip));
    let mut corners: Vec<(f64, f64)> = Vec::new();
    let mut runs: Vec<Run> = Vec::new();
    // Which corner starts each tooth, and the run index of each root land.
    let mut tooth_start = Vec::with_capacity(z);
    for t in 0..z {
        let c = t as f64 * pitch;
        tooth_start.push(corners.len());
        corners.push((g.root, c - hr));
        if radial {
            runs.push(Run::Radial);
            corners.push((g_inv, c - hr));
        }
        runs.push(Run::Involute { c, side: -1.0, t0: th0, t1: th1 });
        corners.push((g.tip, c - ht));
        runs.push(Run::Tip);
        corners.push((g.tip, c + ht));
        runs.push(Run::Involute { c, side: 1.0, t0: th1, t1: th0 });
        if radial {
            corners.push((g_inv, c + hr));
            runs.push(Run::Radial);
        }
        corners.push((g.root, c + hr));
        runs.push(Run::Root);
    }
    let n = corners.len();
    let mut vb = Vec::with_capacity(n);
    let mut vf = Vec::with_capacity(n);
    let mut rays = Vec::with_capacity(n);
    for &(gamma, phi) in &corners {
        let p = back(gamma, phi);
        let (b, f) = (w.vertex(p), w.vertex(scale(k, p)));
        let line = w.line(scale(k, p), p);
        rays.push(w.edge(f, b, line, true));
        vb.push(b);
        vf.push(f);
    }

    let mut faces = Vec::new();
    let mut eb = Vec::with_capacity(n);
    let mut ef = Vec::with_capacity(n);
    for (i, run) in runs.iter().enumerate() {
        let j = (i + 1) % n;
        let (pi, pj) = (back(corners[i].0, corners[i].1), back(corners[j].0, corners[j].1));
        let (cb, cf, surface) = match *run {
            Run::Radial => {
                let mid = unit([pi[0] + pj[0], pi[1] + pj[1], pi[2] + pj[2]]);
                let normal = unit(cross(sub(pj, pi), mid));
                let phi = corners[i].1;
                let plane = w.plane([0.0; 3], normal, [phi.cos(), phi.sin(), 0.0]);
                (w.line(pi, pj), w.line(scale(k, pi), scale(k, pj)), plane)
            }
            Run::Involute { c, side, t0, t1 } => {
                let (ctrl, knots) = flank_curve(g, c, side, t0, t1);
                let (mults, values) = knot_lists(&knots);
                let back_pts: Vec<usize> = ctrl.iter().map(|&p| w.point(p)).collect();
                let front_pts: Vec<usize> = ctrl.iter().map(|&p| w.point(scale(k, p))).collect();
                let list = |ids: &[usize]| ids.iter().map(|i| format!("#{i}")).collect::<Vec<_>>().join(",");
                let curve = |w: &mut Writer, ids: &[usize]| {
                    w.add(format!(
                        "B_SPLINE_CURVE_WITH_KNOTS('',3,({}),.UNSPECIFIED.,.F.,.F.,({mults}),({values}),.UNSPECIFIED.)",
                        list(ids)
                    ))
                };
                let (cb, cf) = (curve(w, &back_pts), curve(w, &front_pts));
                // u along the outline, v from the front cone to the back:
                // the natural normal, u × v, points out of the tooth.
                let rows: Vec<String> = front_pts.iter().zip(&back_pts).map(|(f, b)| format!("(#{f},#{b})")).collect();
                let surface = w.add(format!(
                    "B_SPLINE_SURFACE_WITH_KNOTS('',3,1,({}),.UNSPECIFIED.,.F.,.F.,.F.,({mults}),(2,2),({values}),(0.0,1.0),.UNSPECIFIED.)",
                    rows.join(",")
                ));
                (cb, cf, surface)
            }
            Run::Tip => (tip_b, tip_f, tip_cone),
            Run::Root => (root_b, root_f, root_cone),
        };
        let (b, f) = (w.edge(vb[i], vb[j], cb, true), w.edge(vf[i], vf[j], cf, true));
        // Seen from outside: along the front, out the next ray, back along
        // the back end, in along this ray.
        faces.push(w.face(surface, true, &[(vec![(f, true), (rays[j], true), (b, false), (rays[i], false)], true)]));
        eb.push(b);
        ef.push(f);
    }

    // Tooth ends: each tooth's back edges closed by its foot on the rim
    // circle; the front end the mirror image.
    let per_tooth = n / z;
    let mut rim_b = Vec::new();
    let mut rim_f = Vec::new();
    for &s0 in &tooth_start {
        let l0 = s0 + per_tooth - 1;
        let foot_b = w.edge(vb[s0], vb[l0], root_b, true);
        let foot_f = w.edge(vf[s0], vf[l0], root_f, true);
        let mut lb: Vec<(usize, bool)> = (s0..l0).map(|i| (eb[i], true)).collect();
        lb.push((foot_b, false));
        faces.push(w.face(back_cone, true, &[(lb, true)]));
        let mut lf = vec![(foot_f, true)];
        lf.extend((s0..l0).rev().map(|i| (ef[i], false)));
        faces.push(w.face(front_cone, false, &[(lf, true)]));
        rim_b.extend([foot_b, eb[l0]]);
        rim_f.extend([foot_f, ef[l0]]);
    }

    // Hub faces and bore.
    let rb = (g.bore / 2.0).min(rfk * 0.92);
    let bore = (rb > 0.0).then(|| walls(w, &hole([0.0, 0.0], rb), zf, zb, &mut faces));
    let plane_b = w.plane([0.0, 0.0, zb], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let mut bounds = vec![(rim_b.iter().map(|&e| (e, true)).collect::<Vec<_>>(), true)];
    if let Some(h) = &bore {
        bounds.push((h.top.iter().map(|&e| (e, true)).collect(), true));
    }
    faces.push(w.face(plane_b, true, &bounds));
    let plane_f = w.plane([0.0, 0.0, zf], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]);
    let mut bounds = vec![(rim_f.iter().rev().map(|&e| (e, false)).collect::<Vec<_>>(), true)];
    if let Some(h) = &bore {
        bounds.push((h.bottom.iter().rev().map(|&e| (e, false)).collect(), true));
    }
    faces.push(w.face(plane_f, true, &bounds));

    let list: Vec<String> = faces.iter().map(|f| format!("#{f}")).collect();
    let shell = w.add(format!("CLOSED_SHELL('',({}))", list.join(",")));
    Some(w.add(format!("MANIFOLD_SOLID_BREP('{}',#{shell})", text(name))))
}

// ---- hypoid gears -----------------------------------------------------------

/// Averaged knots for cubic interpolation at `params` (clamped, Piegl &
/// Tiller 9.8).
fn interp_knots(params: &[f64]) -> Vec<f64> {
    let n = params.len();
    let mut k = vec![0.0; 4];
    for j in 1..n - 3 {
        k.push((params[j] + params[j + 1] + params[j + 2]) / 3.0);
    }
    k.extend([1.0; 4]);
    k
}

/// The cubic B-spline basis functions on `knots`, all of them, at u.
fn basis(knots: &[f64], u: f64) -> Vec<f64> {
    let m = knots.len() - 1;
    let n = m - 3;
    let mut span = 3;
    while span < n - 1 && u >= knots[span + 1] {
        span += 1;
    }
    let mut b = vec![0.0; m];
    b[span] = 1.0;
    for d in 1..=3 {
        for i in 0..m - d {
            let left = if knots[i + d] > knots[i] { (u - knots[i]) / (knots[i + d] - knots[i]) * b[i] } else { 0.0 };
            let right = if knots[i + d + 1] > knots[i + 1] {
                (knots[i + d + 1] - u) / (knots[i + d + 1] - knots[i + 1]) * b[i + 1]
            } else {
                0.0
            };
            b[i] = left + right;
        }
    }
    b.truncate(n);
    b
}

/// Control points of the cubic B-spline through `pts` at `params`.
fn interpolate(pts: &[[f64; 3]], params: &[f64], knots: &[f64]) -> Vec<[f64; 3]> {
    let n = pts.len();
    let mut a: Vec<Vec<f64>> = params.iter().map(|&u| basis(knots, u)).collect();
    let mut rhs = pts.to_vec();
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs())).unwrap();
        a.swap(c, p);
        rhs.swap(c, p);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            if f != 0.0 {
                for k in c..n {
                    a[r][k] -= f * a[c][k];
                }
                for d in 0..3 {
                    rhs[r][d] -= f * rhs[c][d];
                }
            }
        }
    }
    let mut x = vec![[0.0; 3]; n];
    for r in (0..n).rev() {
        let mut v = rhs[r];
        for k in r + 1..n {
            for d in 0..3 {
                v[d] -= a[r][k] * x[k][d];
            }
        }
        x[r] = [v[0] / a[r][r], v[1] / a[r][r], v[2] / a[r][r]];
    }
    x
}

/// Parameters by chord length along each row, averaged over the rows.
fn chord_params(rows: &[Vec<[f64; 3]>]) -> Vec<f64> {
    let n = rows[0].len();
    let mut acc = vec![0.0; n];
    for row in rows {
        let d: Vec<f64> = (1..n).map(|j| {
            let v = sub(row[j], row[j - 1]);
            (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
        }).collect();
        let total: f64 = d.iter().sum();
        let mut s = 0.0;
        for j in 1..n {
            s += d[j - 1];
            acc[j] += if total > 0.0 { s / total } else { j as f64 / (n - 1) as f64 };
        }
    }
    let k = rows.len() as f64;
    let mut p: Vec<f64> = acc.iter().map(|a| a / k).collect();
    p[n - 1] = 1.0;
    p
}

/// A bicubic B-spline surface through a grid of points [u][v]: its
/// control net and knots. Rows along u share `u_params`, so two patches
/// built from the same row of points share that boundary exactly.
pub(crate) struct Patch {
    net: Vec<Vec<[f64; 3]>>,
    uk: Vec<f64>,
    vk: Vec<f64>,
    /// Where the data rows sit along v.
    pub(crate) vp: Vec<f64>,
}

impl Patch {
    pub(crate) fn through(data: &[Vec<[f64; 3]>], u_params: &[f64]) -> Patch {
        let v_params = chord_params(data);
        let (uk, vk) = (interp_knots(u_params), interp_knots(&v_params));
        let rows: Vec<Vec<[f64; 3]>> = data.iter().map(|r| interpolate(r, &v_params, &vk)).collect();
        let nv = rows[0].len();
        let cols: Vec<Vec<[f64; 3]>> =
            (0..nv).map(|j| interpolate(&rows.iter().map(|r| r[j]).collect::<Vec<_>>(), u_params, &uk)).collect();
        let net = (0..rows.len()).map(|i| cols.iter().map(|c| c[i]).collect()).collect();
        Patch { net, uk, vk, vp: v_params }
    }

    pub(crate) fn eval(&self, u: f64, v: f64) -> [f64; 3] {
        let (bu, bv) = (basis(&self.uk, u), basis(&self.vk, v));
        let mut out = [0.0; 3];
        for (i, row) in self.net.iter().enumerate() {
            for (j, q) in row.iter().enumerate() {
                for d in 0..3 {
                    out[d] += bu[i] * bv[j] * q[d];
                }
            }
        }
        out
    }

    /// du × dv near the middle of the patch.
    fn normal(&self) -> [f64; 3] {
        let (i, j) = (self.net.len() / 2, self.net[0].len() / 2);
        let du = sub(self.net[i + 1][j], self.net[i - 1][j]);
        let dv = sub(self.net[i][j + 1], self.net[i][j - 1]);
        cross(du, dv)
    }
}

fn distinct_knots(k: &[f64]) -> (String, String) {
    let mut vals: Vec<f64> = Vec::new();
    let mut mults: Vec<usize> = Vec::new();
    for &x in k {
        if vals.last().is_some_and(|&l| (x - l).abs() < 1e-12) {
            *mults.last_mut().unwrap() += 1;
        } else {
            vals.push(x);
            mults.push(1);
        }
    }
    let m: Vec<String> = mults.iter().map(|m| m.to_string()).collect();
    let v: Vec<String> = vals.iter().map(|&x| num(x)).collect();
    (m.join(","), v.join(","))
}

/// Clamped cubic B-splines (each on knots 0..1) joined end to end into
/// one, with a C0 knot at each joint so each piece is kept exactly.
fn join_splines(pieces: &[(Vec<[f64; 3]>, Vec<f64>)]) -> (Vec<[f64; 3]>, Vec<f64>) {
    let mut ctrl: Vec<[f64; 3]> = Vec::new();
    let mut knots = vec![0.0; 4];
    let n = pieces.len();
    for (i, (c, k)) in pieces.iter().enumerate() {
        ctrl.extend(if i == 0 { &c[..] } else { &c[1..] });
        let off = i as f64;
        knots.extend(k[4..k.len() - 4].iter().map(|x| x + off));
        knots.extend(std::iter::repeat_n(off + 1.0, if i + 1 == n { 4 } else { 3 }));
    }
    let span = n as f64;
    (ctrl, knots.into_iter().map(|x| x / span).collect())
}

fn spline_curve(w: &mut Writer, ctrl: &[[f64; 3]], knots: &[f64]) -> usize {
    let ids: Vec<String> = ctrl.iter().map(|&p| format!("#{}", w.point(p))).collect();
    let (m, v) = distinct_knots(knots);
    w.add(format!(
        "B_SPLINE_CURVE_WITH_KNOTS('',3,({}),.UNSPECIFIED.,.F.,.F.,({m}),({v}),.UNSPECIFIED.)",
        ids.join(",")
    ))
}

fn spline_surface(w: &mut Writer, p: &Patch) -> usize {
    let rows: Vec<String> = p
        .net
        .iter()
        .map(|r| format!("({})", r.iter().map(|&q| format!("#{}", w.point(q))).collect::<Vec<_>>().join(",")))
        .collect();
    let (um, uv) = distinct_knots(&p.uk);
    let (vm, vv) = distinct_knots(&p.vk);
    w.add(format!(
        "B_SPLINE_SURFACE_WITH_KNOTS('',3,3,({}),.UNSPECIFIED.,.F.,.F.,.F.,({um}),({vm}),({uv}),({vv}),.UNSPECIFIED.)",
        rows.join(",")
    ))
}

/// Points across the tip land on each section.
pub(crate) const TIP_POINTS: usize = 5;

/// A hypoid tooth's surfaces as grids of points [section][across]: its
/// right flank and left flank (seam to tip) and the tip land between.
pub(crate) fn tooth_data(m: &Member, tooth: usize) -> [Vec<Vec<[f64; 3]>>; 3] {
    let rot = tooth as f64 * m.pitch();
    let tip = (0..m.stations.len())
        .map(|k| {
            let (a, b) = (*m.right[k].last().unwrap(), *m.left[k].last().unwrap());
            (0..TIP_POINTS)
                .map(|i| {
                    let f = i as f64 / (TIP_POINTS - 1) as f64;
                    m.point(m.stations[k], a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f + rot)
                })
                .collect()
        })
        .collect();
    [m.flank(false, tooth), tip, m.flank(true, tooth)]
}

/// A hypoid gear or pinion as a MANIFOLD_SOLID_BREP, in its own frame
/// (pitch apex at the origin, axis +z) and placed by `w.place`. Each
/// flank and tip land is a bicubic B-spline through the generated
/// sections; neighbouring patches are built from the same rows of points
/// so their shared edges are exact iso-curves of both. The ends lie on the
/// section cones at the toe and heel, then flat hub faces and the bore.
fn hypoid_brep(w: &mut Writer, m: &Member, name: &str) -> Option<usize> {
    let hub = m.hub_faces();
    if m.bore / 2.0 >= hub[0].1.min(hub[1].1) {
        return None;
    }
    let z = m.teeth as usize;
    let pitch = m.pitch();
    let ns = m.stations.len();
    let last = ns - 1;
    let u_params: Vec<f64> = (0..ns).map(|k| k as f64 / last as f64).collect();
    let gamma = m.blank.pitch;
    let (sg, cg) = gamma.sin_cos();
    let e_phi = |phi: f64, sign: f64| [-sign * phi.sin(), sign * phi.cos(), 0.0];

    // Patches per tooth: right flank, tip land, left flank.
    let mut patches: Vec<[(Patch, bool); 3]> = Vec::with_capacity(z);
    for tooth in 0..z {
        let rot = tooth as f64 * pitch;
        let [right, tip, left] = tooth_data(m, tooth);
        let mid = ns / 2;
        let phi_of = |side: &Vec<Vec<[f64; 2]>>| side[mid][side[mid].len() / 2][1] + rot;
        let tip_phi = (m.right[mid].last().unwrap()[1] + m.left[mid].last().unwrap()[1]) / 2.0 + rot;
        let off = [cg * tip_phi.cos(), cg * tip_phi.sin(), -sg];
        let make = |data: &[Vec<[f64; 3]>], out: [f64; 3]| {
            let p = Patch::through(data, &u_params);
            let agrees = {
                let n = p.normal();
                n[0] * out[0] + n[1] * out[1] + n[2] * out[2] > 0.0
            };
            (p, agrees)
        };
        patches.push([make(&right, e_phi(phi_of(&m.right), -1.0)), make(&tip, off), make(&left, e_phi(phi_of(&m.left), 1.0))]);
    }

    // Corner vertices per tooth and end: seam, right tip, left tip.
    let corner = |p: &Patch, u: usize, v: usize| p.net[if u == 0 { 0 } else { p.net.len() - 1 }][if v == 0 { 0 } else { p.net[0].len() - 1 }];
    let mut seam_v = vec![[0usize; 2]; z];
    let mut rtip_v = vec![[0usize; 2]; z];
    let mut ltip_v = vec![[0usize; 2]; z];
    for t in 0..z {
        for e in 0..2 {
            seam_v[t][e] = w.vertex(corner(&patches[t][0].0, e, 0));
            rtip_v[t][e] = w.vertex(corner(&patches[t][0].0, e, 1));
            ltip_v[t][e] = w.vertex(corner(&patches[t][2].0, e, 1));
        }
    }
    let row = |p: &Patch, v: usize| -> Vec<[f64; 3]> { p.net.iter().map(|r| if v == 0 { r[0] } else { *r.last().unwrap() }).collect() };
    let col = |p: &Patch, u: usize| -> Vec<[f64; 3]> { if u == 0 { p.net[0].clone() } else { p.net.last().unwrap().clone() } };
    // Edges along the face (u): seam, right tip, left tip.
    let mut seam_e = Vec::with_capacity(z);
    let mut rtip_e = Vec::with_capacity(z);
    let mut ltip_e = Vec::with_capacity(z);
    for t in 0..z {
        let [(r, _), _, (l, _)] = &patches[t];
        let c = spline_curve(w, &row(r, 0), &r.uk);
        seam_e.push(w.edge(seam_v[t][0], seam_v[t][1], c, true));
        let c = spline_curve(w, &row(r, 1), &r.uk);
        rtip_e.push(w.edge(rtip_v[t][0], rtip_v[t][1], c, true));
        let c = spline_curve(w, &row(l, 1), &l.uk);
        ltip_e.push(w.edge(ltip_v[t][0], ltip_v[t][1], c, true));
    }
    // Edges across the ends (v): right flank, tip, left flank, per end.
    let mut rend = vec![[0usize; 2]; z];
    let mut tend = vec![[0usize; 2]; z];
    let mut lend = vec![[0usize; 2]; z];
    for t in 0..z {
        let next = (t + 1) % z;
        let [(r, _), (tp, _), (l, _)] = &patches[t];
        for e in 0..2 {
            let c = spline_curve(w, &col(r, e), &r.vk);
            rend[t][e] = w.edge(seam_v[t][e], rtip_v[t][e], c, true);
            let c = spline_curve(w, &col(tp, e), &tp.vk);
            tend[t][e] = w.edge(rtip_v[t][e], ltip_v[t][e], c, true);
            let c = spline_curve(w, &col(l, e), &l.vk);
            lend[t][e] = w.edge(seam_v[next][e], ltip_v[t][e], c, true);
        }
    }

    let mut faces = Vec::new();
    // A patch's boundary anticlockwise in (u, v): v = 0 forwards, u = 1
    // up, v = 1 back, u = 0 down; reversed if the outside is the other way.
    let mut patch_face = |w: &mut Writer, p: &Patch, agrees: bool, b: usize, r: usize, t: usize, l: usize| {
        let surf = spline_surface(w, p);
        let lp = vec![(b, true), (r, true), (t, false), (l, false)];
        let lp = if agrees { lp } else { lp.into_iter().rev().map(|(e, f)| (e, !f)).collect() };
        faces.push(w.face(surf, agrees, &[(lp, true)]));
    };
    for t in 0..z {
        let next = (t + 1) % z;
        let [(r, ra), (tp, ta), (l, la)] = &patches[t];
        patch_face(w, r, *ra, seam_e[t], rend[t][1], rtip_e[t], rend[t][0]);
        patch_face(w, tp, *ta, rtip_e[t], tend[t][1], ltip_e[t], tend[t][0]);
        patch_face(w, l, *la, seam_e[next], lend[t][1], ltip_e[t], lend[t][0]);
    }

    // Tooth ends. Each is a ruled patch from the hub circle up to the
    // tooth's end outline (its right flank, tip and left flank end edges,
    // joined into one curve), lying close to the section cone. Its sides
    // are straight lines on the cone below the seams, and its foot, a
    // spline on the hub circle, is shared with the flat face.
    let mut hub_v = vec![[0usize; 2]; z];
    let mut side_e = vec![[0usize; 2]; z];
    for t in 0..z {
        let rot = t as f64 * pitch;
        for (e, k) in [(0usize, 0usize), (1, last)] {
            let (zh, rh) = hub[e];
            let phi = m.right[k][0][1] + rot;
            let foot = [rh * phi.cos(), rh * phi.sin(), zh];
            hub_v[t][e] = w.vertex(foot);
            let line = w.line(foot, corner(&patches[t][0].0, e, 0));
            side_e[t][e] = w.edge(hub_v[t][e], seam_v[t][e], line, true);
        }
    }
    let mut foot_e = vec![[0usize; 2]; z];
    for t in 0..z {
        let next = (t + 1) % z;
        let rot = t as f64 * pitch;
        let [(r, _), (tp, _), (l, _)] = &patches[t];
        for (e, k) in [(0usize, 0usize), (1, last)] {
            let (zh, rh) = hub[e];
            let back: Vec<[f64; 3]> = col(l, e).into_iter().rev().collect();
            let back_k: Vec<f64> = l.vk.iter().rev().map(|x| 1.0 - x).collect();
            let pieces = [(col(r, e), r.vk.clone()), (col(tp, e), tp.vk.clone()), (back, back_k)];
            let (top, knots) = join_splines(&pieces);
            // The foot in matching pieces, between the same azimuths.
            let phis = [
                m.right[k][0][1],
                m.right[k].last().unwrap()[1],
                m.left[k].last().unwrap()[1],
                m.left[k][0][1],
            ]
            .map(|p| p + rot);
            let feet: Vec<(Vec<[f64; 3]>, Vec<f64>)> = pieces
                .iter()
                .enumerate()
                .map(|(i, (c, kn))| {
                    let greville: Vec<f64> = (0..c.len()).map(|j| (kn[j + 1] + kn[j + 2] + kn[j + 3]) / 3.0).collect();
                    let pts: Vec<[f64; 3]> = greville
                        .iter()
                        .map(|&g| {
                            let a = phis[i] + (phis[i + 1] - phis[i]) * g;
                            [rh * a.cos(), rh * a.sin(), zh]
                        })
                        .collect();
                    (interpolate(&pts, &greville, kn), kn.clone())
                })
                .collect();
            let (bottom, _) = join_splines(&feet);
            let curve = spline_curve(w, &bottom, &knots);
            foot_e[t][e] = w.edge(hub_v[t][e], hub_v[next][e], curve, true);
            // Outward: towards the apex at the toe, away at the heel.
            let i = top.len() / 2;
            let n = cross(sub(bottom[i + 1], bottom[i - 1]), sub(top[i], bottom[i]));
            let mid = (phis[1] + phis[2]) / 2.0;
            let out = [sg * mid.cos(), sg * mid.sin(), cg];
            let agrees = (n[0] * out[0] + n[1] * out[1] + n[2] * out[2] > 0.0) == (e == 1);
            let rows: Vec<String> =
                bottom.iter().zip(&top).map(|(&b, &q)| format!("(#{},#{})", w.point(b), w.point(q))).collect();
            let (um, uv) = distinct_knots(&knots);
            let surf = w.add(format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',3,1,({}),.UNSPECIFIED.,.F.,.F.,.F.,({um}),(2,2),({uv}),(0.0,1.0),.UNSPECIFIED.)",
                rows.join(",")
            ));
            let lp = vec![
                (foot_e[t][e], true),
                (side_e[next][e], true),
                (lend[t][e], true),
                (tend[t][e], false),
                (rend[t][e], false),
                (side_e[t][e], false),
            ];
            let lp = if agrees { lp } else { lp.into_iter().rev().map(|(x, f)| (x, !f)).collect() };
            faces.push(w.face(surf, agrees, &[(lp, true)]));
        }
    }
    // Flat faces and the bore.
    let rb = (m.bore / 2.0).max(0.0);
    let (lo, hi) = if hub[0].0 < hub[1].0 { (0, 1) } else { (1, 0) };
    let bore = (rb > 0.0).then(|| walls(w, &hole([0.0, 0.0], rb), hub[lo].0, hub[hi].0, &mut faces));
    for (e, up) in [(hi, true), (lo, false)] {
        let zf = hub[e].0;
        let plane = w.plane([0.0, 0.0, zf], [0.0, 0.0, if up { 1.0 } else { -1.0 }], [1.0, 0.0, 0.0]);
        let ring: Vec<(usize, bool)> =
            if up { (0..z).map(|t| (foot_e[t][e], true)).collect() } else { (0..z).rev().map(|t| (foot_e[t][e], false)).collect() };
        let mut bounds = vec![(ring, true)];
        if let Some(h) = &bore {
            let inner = if up { h.top.iter().map(|&x| (x, true)).collect() } else { h.bottom.iter().rev().map(|&x| (x, false)).collect() };
            bounds.push((inner, true));
        }
        faces.push(w.face(plane, true, &bounds));
    }

    let list: Vec<String> = faces.iter().map(|f| format!("#{f}")).collect();
    let shell = w.add(format!("CLOSED_SHELL('',({}))", list.join(",")));
    Some(w.add(format!("MANIFOLD_SOLID_BREP('{}',#{shell})", text(name))))
}

/// STEP strings are ISO 8859-1 between single quotes, which double.
fn text(s: &str) -> String {
    s.chars().filter(|c| c.is_ascii() && !c.is_ascii_control()).collect::<String>().replace('\'', "''")
}

pub fn step(shapes: &[Shape], bevels: &[BevelPart], hypoids: &[HypoidPart], name: &str) -> String {
    let name = if name.is_empty() { "gearcraft".to_string() } else { text(name) };
    let mut w = Writer { out: String::new(), next: 1, place: None };
    let ctx = w.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
    w.add(format!("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{ctx})"));
    let pctx = w.add(format!("PRODUCT_CONTEXT('',#{ctx},'mechanical')"));
    let product = w.add(format!("PRODUCT('{name}','{name}','',(#{pctx}))"));
    let formation = w.add(format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
    let dctx = w.add(format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{ctx},'design')"));
    let def = w.add(format!("PRODUCT_DEFINITION('design','',#{formation},#{dctx})"));
    let pds = w.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{def})"));
    let mm = w.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
    let rad = w.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
    let sr = w.add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
    let tol = w.add(format!(
        "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#{mm},'distance_accuracy_value','confusion accuracy')"
    ));
    let geo = w.add(format!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{tol}))\
         GLOBAL_UNIT_ASSIGNED_CONTEXT((#{mm},#{rad},#{sr}))REPRESENTATION_CONTEXT('',''))"
    ));
    let origin = w.axis([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let mut items = vec![format!("#{origin}")];
    for s in shapes {
        items.push(format!("#{}", solid(&mut w, s)));
    }
    for b in bevels {
        w.place = Some((b.frame, b.rot));
        if let Some(id) = bevel_solid(&mut w, &b.geom, &b.name) {
            items.push(format!("#{id}"));
        }
        w.place = None;
    }
    for h in hypoids {
        w.place = Some((h.frame, h.rot));
        if let Some(id) = hypoid_brep(&mut w, &h.member, &h.name) {
            items.push(format!("#{id}"));
        }
        w.place = None;
    }
    let rep = w.add(format!("ADVANCED_BREP_SHAPE_REPRESENTATION('{name}',({}),#{geo})", items.join(",")));
    w.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})"));

    let mut out = String::new();
    out.push_str("ISO-10303-21;\nHEADER;\n");
    out.push_str("FILE_DESCRIPTION(('GearCraft gear train'),'2;1');\n");
    let _ = writeln!(out, "FILE_NAME('{name}.step','',(''),(''),'GearCraft {}','GearCraft','');", env!("CARGO_PKG_VERSION"));
    out.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 }'));\nENDSEC;\nDATA;\n");
    out.push_str(&w.out);
    out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export;
    use crate::train::Project;

    #[test]
    fn every_edge_is_used_twice_with_opposite_senses() {
        // A closed, manifold shell: each edge bounds exactly two faces,
        // once in each direction.
        let shapes = export::shapes(&Project::default(), 0.0, None, |_| true);
        let text = step(&shapes, &[], &[], "t");
        assert_closed(&text);
        assert_eq!(text.matches("MANIFOLD_SOLID_BREP").count(), shapes.len());
    }

    #[test]
    fn bevel_gears_are_closed_too() {
        use crate::train::{Link, PitchUnit};
        let mut p = Project::empty();
        let a = p.add_root(PitchUnit::Module.starter_bevel(20), 0.0, 0.0);
        p.add_child(a, PitchUnit::Module.starter_bevel(40), Link::Bevel);
        // Small enough that the root lies inside the base cone, so the
        // flanks have a radial part as well.
        let c = p.add_child(a, PitchUnit::Module.starter_bevel(9), Link::Bevel);
        p.normalize();
        let i = p.index_of(c).unwrap();
        let g = BevelGeom::new(&p.gears[i].params).unwrap();
        assert!(g.base > g.root);
        let bevels = export::bevels(&p, 0.3, None, |_| true);
        assert_eq!(bevels.len(), 3);
        let text = step(&[], &bevels, &[], "t");
        assert_closed(&text);
        assert_eq!(text.matches("MANIFOLD_SOLID_BREP").count(), 3);
    }

    /// `GEARCRAFT_BEVEL_STEP=dir cargo test write_bevel_samples` writes a
    /// bevel pair (and its project) for checking in a CAD program.
    #[test]
    fn write_bevel_samples() {
        use crate::train::{Link, PitchUnit};
        let Ok(dir) = std::env::var("GEARCRAFT_BEVEL_STEP") else { return };
        let dir = std::path::Path::new(&dir);
        let mut p = Project::empty();
        let a = p.add_root(PitchUnit::Module.starter_bevel(20), 0.0, 0.0);
        p.add_child(a, PitchUnit::Module.starter_bevel(40), Link::Bevel);
        p.normalize();
        std::fs::write(dir.join("pair.gearcraft"), serde_json::to_string_pretty(&p).unwrap()).unwrap();
        for (i, g) in p.gears.iter().enumerate() {
            let one = export::bevels(&p, 0.0, Some(g.id), |_| true);
            std::fs::write(dir.join(format!("bevel{i}.step")), step(&[], &one, &[], "b")).unwrap();
        }
        std::fs::write(dir.join("pair.step"), step(&[], &export::bevels(&p, 0.0, None, |_| true), &[], "pair")).unwrap();
    }

    fn hypoid_project() -> Project {
        use crate::hypoid::Role;
        use crate::train::{Link, PitchUnit};
        let mut p = Project::empty();
        let a = p.add_root(PitchUnit::Module.starter_hypoid(Role::Pinion), 0.0, 0.0);
        p.add_child(a, PitchUnit::Module.starter_hypoid(Role::Ring), Link::Hypoid);
        p.normalize();
        p
    }

    #[test]
    fn worm_drives_are_closed() {
        use crate::train::{Link, PitchUnit, Placement};
        use crate::worm::Role;
        let u = PitchUnit::Module;
        for (starts, throated) in [(1, true), (3, false)] {
            let mut p = Project::empty();
            let w = p.add_root(crate::gear::GearParams { teeth: starts, ..u.starter_worm_drive(Role::Worm) }, 0.0, 0.0);
            p.add_child(w, u.starter_worm_drive(Role::Wheel), Link::Worm);
            if let Placement::Worm { throated: t, .. } = &mut p.gears[1].placement {
                *t = throated;
            }
            p.normalize();
            let parts = export::hypoids(&p, 0.2, None, |_| true, |_| None);
            assert_eq!(parts.len(), 2);
            let text = step(&[], &[], &parts, "t");
            assert_closed(&text);
            assert_eq!(text.matches("MANIFOLD_SOLID_BREP").count(), 2, "{starts} starts");
        }
    }

    /// `GEARCRAFT_WORM_STEP=dir cargo test write_worm_samples` writes worm
    /// drives (and their projects) for checking in a CAD program: a
    /// single-start worm with a throated wheel, and a 3-start left-hand one
    /// with a straight wheel.
    #[test]
    fn write_worm_samples() {
        use crate::train::{Link, PitchUnit, Placement};
        use crate::worm::Role;
        let Ok(dir) = std::env::var("GEARCRAFT_WORM_STEP") else { return };
        let dir = std::path::Path::new(&dir);
        let u = PitchUnit::Module;
        for (name, starts, throated) in [("worm1", 1, true), ("worm3", 3, false)] {
            let mut p = Project::empty();
            let w = p.add_root(crate::gear::GearParams { teeth: starts, ..u.starter_worm_drive(Role::Worm) }, 0.0, 0.0);
            p.add_child(w, u.starter_worm_drive(Role::Wheel), Link::Worm);
            if let Placement::Worm { throated: t, left_hand, .. } = &mut p.gears[1].placement {
                (*t, *left_hand) = (throated, !throated);
            }
            p.gears[1].params.backlash = 0.1;
            p.normalize();
            std::fs::write(dir.join(format!("{name}.gearcraft")), serde_json::to_string_pretty(&p).unwrap()).unwrap();
            for (i, g) in p.gears.iter().enumerate() {
                let one = export::hypoids(&p, 0.0, Some(g.id), |_| true, |_| None);
                std::fs::write(dir.join(format!("{name}_{i}.step")), step(&[], &[], &one, "w")).unwrap();
            }
            let all = export::hypoids(&p, 0.3, None, |_| true, |_| None);
            std::fs::write(dir.join(format!("{name}_pair.step")), step(&[], &[], &all, "pair")).unwrap();
        }
    }

    #[test]
    fn hypoid_gears_are_closed() {
        let parts = export::hypoids(&hypoid_project(), 0.2, None, |_| true, |_| None);
        assert_eq!(parts.len(), 2);
        let text = step(&[], &[], &parts, "t");
        assert_closed(&text);
        assert_eq!(text.matches("MANIFOLD_SOLID_BREP").count(), 2);
    }

    /// `GEARCRAFT_HYPOID_STEP=dir cargo test write_hypoid_samples` writes
    /// the car hypoid set (and its project) for checking in a CAD program.
    #[test]
    fn write_hypoid_samples() {
        let Ok(dir) = std::env::var("GEARCRAFT_HYPOID_STEP") else { return };
        let dir = std::path::Path::new(&dir);
        let mut p = hypoid_project();
        // Some backlash, so the flanks are clear rather than touching.
        p.gears[1].params.backlash = 0.1;
        std::fs::write(dir.join("hypoid.gearcraft"), serde_json::to_string_pretty(&p).unwrap()).unwrap();
        for (i, g) in p.gears.iter().enumerate() {
            let one = export::hypoids(&p, 0.0, Some(g.id), |_| true, |_| None);
            std::fs::write(dir.join(format!("hypoid{i}.step")), step(&[], &[], &one, "h")).unwrap();
        }
        // The pair at a few positions through one pinion tooth's mesh.
        for k in 0..4 {
            let a = std::f64::consts::TAU / 11.0 * k as f64 / 4.0;
            let all = export::hypoids(&p, a, None, |_| true, |_| None);
            std::fs::write(dir.join(format!("hypoid_pair{k}.step")), step(&[], &[], &all, "pair")).unwrap();
        }
    }

    fn assert_closed(text: &str) {
        let mut uses: std::collections::HashMap<String, (u32, u32)> = Default::default();
        for line in text.lines().filter(|l| l.contains("ORIENTED_EDGE")) {
            let inner = &line[line.find("*,*,").unwrap() + 4..];
            let (edge, sense) = inner.trim_end_matches(");").split_once(',').unwrap();
            let e = uses.entry(edge.to_string()).or_default();
            if sense == ".T." { e.0 += 1 } else { e.1 += 1 }
        }
        assert!(!uses.is_empty());
        for (edge, (f, b)) in uses {
            assert_eq!((f, b), (1, 1), "edge {edge}");
        }
    }

    #[test]
    fn numbers_always_have_a_decimal_point() {
        assert_eq!(num(1.0), "1.0");
        assert_eq!(num(-2.5), "-2.5");
        assert_eq!(num(1e-14), "0.0");
        assert_eq!(num(0.1234567891234), "0.1234567891");
    }
}

#[cfg(test)]
mod cloud {
    //! A point cloud off the hypoid STEP surfaces, run through the mesh.
    use super::*;
    use crate::hypoid::{Spec, tests::car};

    fn rz(a: f64, x: [f64; 3]) -> [f64; 3] {
        let (s, c) = a.sin_cos();
        [x[0] * c - x[1] * s, x[0] * s + x[1] * c, x[2]]
    }

    /// Tooth 0's flank surfaces, as written to STEP, sampled densely.
    fn cloud(m: &Member) -> Vec<[f64; 3]> {
        let ns = m.stations.len();
        let up: Vec<f64> = (0..ns).map(|k| k as f64 / (ns - 1) as f64).collect();
        let mut pts = Vec::new();
        for left in [false, true] {
            let p = Patch::through(&m.flank(left, 0), &up);
            for i in 0..=50 {
                for j in 0..=80 {
                    pts.push(p.eval(i as f64 / 50.0, j as f64 / 80.0));
                }
            }
        }
        pts
    }

    #[test]
    fn step_surfaces_mesh_without_colliding() {
        for spec in [car(), Spec { offset: 0.0, ..car() }] {
            let d = spec.design().unwrap();
            let (pin, ring) = (d.pinion().unwrap(), d.ring().unwrap());
            // The ring's surfaces hug the slots its cutter makes.
            let bulge = cloud(&ring).iter().map(|&y| d.any_slot(y, 0.0)).fold(f64::MIN, f64::max);
            assert!(bulge < 0.01, "ring surface {bulge} mm into its slot");
            // The pinion's surfaces, every tooth, through a ring pitch of
            // motion, stay out of the true ring gear.
            let rb = d.ring_blank();
            let k = d.sense * spec.ring_teeth as f64 / spec.pinion_teeth as f64;
            let pitch = TAU / spec.pinion_teeth as f64;
            let pc = cloud(&pin);
            let mut deepest = f64::MIN;
            for step in 0..48 {
                let theta = TAU / spec.ring_teeth as f64 * step as f64 / 48.0;
                for tooth in 0..spec.pinion_teeth {
                    for &q in &pc {
                        let y = rz(-theta, d.pinion_to_design(rz(k * theta + tooth as f64 * pitch, q)));
                        let v = rb.inside_pub([y[0].hypot(y[1]), y[2]]).min(-d.any_slot(y, 0.0));
                        deepest = deepest.max(v);
                    }
                }
            }
            assert!(deepest < 1e-3, "pinion surface {deepest} mm into the ring (offset {})", spec.offset);
        }
    }
}

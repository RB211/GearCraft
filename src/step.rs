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
use crate::export::{BevelPart, Shape};
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

/// STEP strings are ISO 8859-1 between single quotes, which double.
fn text(s: &str) -> String {
    s.chars().filter(|c| c.is_ascii() && !c.is_ascii_control()).collect::<String>().replace('\'', "''")
}

pub fn step(shapes: &[Shape], bevels: &[BevelPart], name: &str) -> String {
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
        let text = step(&shapes, &[], "t");
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
        let text = step(&[], &bevels, "t");
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
            std::fs::write(dir.join(format!("bevel{i}.step")), step(&[], &one, "b")).unwrap();
        }
        std::fs::write(dir.join("pair.step"), step(&[], &export::bevels(&p, 0.0, None, |_| true), "pair")).unwrap();
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

//! DXF (R12, which every CAM and laser package reads) and SVG output.
//!
//! Outlines are written as circular arcs fitted to the generated tooth
//! profile, not as the thousands of tiny segments the profile is computed
//! with: CAD sketches and CAM toolpaths both choke on those, and points a
//! fraction of a micron apart get merged by CAD import and tangle the loop.

use crate::gear;
use crate::train::{Gear, Project};
use std::fmt::Write;
use std::path::Path;

/// One gear ready to write, in world millimetres: a closed outline as
/// (vertex, bulge of the segment leaving it) pairs, DXF style, and the bore.
pub struct Shape {
    pub name: String,
    pub path: Vec<([f64; 2], f64)>,
    pub center: [f64; 2],
    pub bore: f64,
}

/// Fit a closed outline with lines and circular arcs. Every input point
/// lies within `tol` of the result. Returns (vertex, bulge) pairs where the
/// bulge is tan(included angle / 4), positive for counter-clockwise arcs.
pub fn fit_arcs(outline: &[[f64; 2]], tol: f64) -> Vec<([f64; 2], f64)> {
    let mut p: Vec<[f64; 2]> = Vec::with_capacity(outline.len() + 1);
    for &q in outline {
        if p.last().is_none_or(|l: &[f64; 2]| dist(*l, q) > tol * 0.01) {
            p.push(q);
        }
    }
    while p.len() > 1 && dist(p[0], *p.last().unwrap()) <= tol * 0.01 {
        p.pop();
    }
    if p.len() < 3 {
        return p.into_iter().map(|q| (q, 0.0)).collect();
    }
    p.push(p[0]);
    let n = p.len() - 1;
    let mut out = Vec::new();
    let mut i = 0;
    while i < n {
        // Gallop out to the first span that does not fit, then bisect back.
        let mut good = (i + 1, 0.0);
        let mut step = 2;
        let mut bad = None;
        while i + step <= n {
            match fit_span(&p, i, i + step, tol) {
                Some(b) => good = (i + step, b),
                None => {
                    bad = Some(i + step);
                    break;
                }
            }
            step *= 2;
        }
        let (mut lo, mut hi) = (good.0, bad.unwrap_or(n + 1));
        if bad.is_none() && good.0 < n {
            if let Some(b) = fit_span(&p, i, n, tol) {
                good = (n, b);
            }
            lo = good.0;
        }
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            match fit_span(&p, i, mid, tol) {
                Some(b) => {
                    good = (mid, b);
                    lo = mid;
                }
                None => hi = mid,
            }
        }
        out.push((p[i], good.1));
        i = good.0;
    }
    out
}

/// The bulge of a line or arc from p[i] to p[j] passing within `tol` of
/// every point between, if there is one.
fn fit_span(p: &[[f64; 2]], i: usize, j: usize, tol: f64) -> Option<f64> {
    if j == i + 1 {
        return Some(0.0);
    }
    let (a, b) = (p[i], p[j]);
    let chord = [b[0] - a[0], b[1] - a[1]];
    let len = chord[0].hypot(chord[1]);
    if len < tol {
        return None;
    }
    let off = |q: [f64; 2]| ((q[0] - a[0]) * chord[1] - (q[1] - a[1]) * chord[0]) / len;
    if p[i + 1..j].iter().all(|&q| off(q).abs() <= tol) {
        return Some(0.0);
    }
    // Circle through both ends and the middle point.
    let m = p[(i + j) / 2];
    let c = circumcenter(a, m, b)?;
    let r = dist(c, a);
    // Points must sit on the circle and progress monotonically from a to b.
    // `off` is positive to the right of a→b, where a CCW arc's middle lies.
    let ccw = off(m) > 0.0;
    let ang = |q: [f64; 2]| (q[1] - c[1]).atan2(q[0] - c[0]);
    let a0 = ang(a);
    let sweep = |q: [f64; 2]| {
        let d = (ang(q) - a0).rem_euclid(std::f64::consts::TAU);
        if ccw { d } else { std::f64::consts::TAU - d }
    };
    let total = sweep(b);
    if total >= std::f64::consts::PI {
        return None;
    }
    let mut last = 0.0;
    for &q in &p[i + 1..j] {
        let s = sweep(q);
        if (dist(c, q) - r).abs() > tol || s < last - 1e-12 || s > total {
            return None;
        }
        last = s;
    }
    let bulge = (total / 4.0).tan();
    Some(if ccw { bulge } else { -bulge })
}

fn circumcenter(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Option<[f64; 2]> {
    let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    if d.abs() < 1e-18 {
        return None;
    }
    let (a2, b2, c2) = (a[0] * a[0] + a[1] * a[1], b[0] * b[0] + b[1] * b[1], c[0] * c[0] + c[1] * c[1]);
    Some([
        (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d,
        (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d,
    ])
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// Fitting tolerance: half a micron at 2 module, scaling with tooth size.
pub fn tolerance(module: f64) -> f64 {
    (module * 2.5e-4).max(2e-5)
}

pub fn place(q: [f64; 2], center: [f64; 2], rot: f64) -> [f64; 2] {
    let (s, c) = rot.sin_cos();
    [center[0] + q[0] * c - q[1] * s, center[1] + q[0] * s + q[1] * c]
}

/// The whole train as it stands at `driver_angle`, or just gear `only`,
/// which goes out at the origin with tooth 0 on +X, ready to cut.
/// The parts `keep` picks: placed as they sit in the train (turned to
/// `driver_angle`), or, for a single part (`only`), at the origin.
pub fn shapes(project: &Project, driver_angle: f64, only: Option<u64>, keep: impl Fn(&Gear) -> bool) -> Vec<Shape> {
    let solved = project.solve();
    project
        .gears
        .iter()
        .zip(&solved)
        .filter(|(g, _)| only.is_none_or(|id| id == g.id) && keep(g))
        .filter_map(|(g, s)| {
            let prof = gear::tooth_profile(&g.params)?;
            let (center, rot) = if only.is_none() { (s.center, s.phase + s.ratio * driver_angle) } else { ([0.0, 0.0], 0.0) };
            let outline: Vec<[f64; 2]> =
                gear::outline(&g.params, &prof).into_iter().map(|q| place(q, center, rot)).collect();
            Some(Shape {
                name: g.name.clone(),
                path: fit_arcs(&outline, tolerance(g.params.module)),
                center,
                bore: g.params.bore,
            })
        })
        .collect()
}

/// Write DXF or SVG, chosen by the file extension, in inches or mm.
pub fn write(path: &Path, shapes: &[Shape], inch: bool) -> std::io::Result<()> {
    let is_svg = path.extension().is_some_and(|e| e.eq_ignore_ascii_case("svg"));
    std::fs::write(path, if is_svg { svg(shapes, inch) } else { dxf(shapes, inch) })
}

/// Shapes rescaled from mm into the output unit.
fn in_units(shapes: &[Shape], inch: bool) -> Vec<Shape> {
    let k = if inch { 1.0 / 25.4 } else { 1.0 };
    shapes
        .iter()
        .map(|s| Shape {
            name: s.name.clone(),
            // Bulges are ratios, so they survive scaling unchanged.
            path: s.path.iter().map(|&(q, b)| ([q[0] * k, q[1] * k], b)).collect(),
            center: [s.center[0] * k, s.center[1] * k],
            bore: s.bore * k,
        })
        .collect()
}

pub fn dxf(shapes: &[Shape], inch: bool) -> String {
    let shapes = &in_units(shapes, inch);
    let mut s = String::new();
    let pair = |s: &mut String, code: i32, value: &str| {
        let _ = writeln!(s, "{code}\n{value}");
    };
    let num = |v: f64| format!("{v:.8}");
    pair(&mut s, 0, "SECTION");
    pair(&mut s, 2, "HEADER");
    pair(&mut s, 9, "$ACADVER");
    pair(&mut s, 1, "AC1009");
    pair(&mut s, 9, "$INSUNITS");
    // $INSUNITS: 1 = inches, 4 = millimetres.
    pair(&mut s, 70, if inch { "1" } else { "4" });
    pair(&mut s, 0, "ENDSEC");
    pair(&mut s, 0, "SECTION");
    pair(&mut s, 2, "TABLES");
    pair(&mut s, 0, "TABLE");
    pair(&mut s, 2, "LAYER");
    pair(&mut s, 70, "2");
    for (name, color) in [("GEAR", "7"), ("BORE", "1")] {
        pair(&mut s, 0, "LAYER");
        pair(&mut s, 2, name);
        pair(&mut s, 70, "0");
        pair(&mut s, 62, color);
        pair(&mut s, 6, "CONTINUOUS");
    }
    pair(&mut s, 0, "ENDTAB");
    pair(&mut s, 0, "ENDSEC");
    pair(&mut s, 0, "SECTION");
    pair(&mut s, 2, "ENTITIES");
    for sh in shapes {
        pair(&mut s, 0, "POLYLINE");
        pair(&mut s, 8, "GEAR");
        pair(&mut s, 66, "1");
        pair(&mut s, 70, "1");
        for &([x, y], bulge) in &sh.path {
            pair(&mut s, 0, "VERTEX");
            pair(&mut s, 8, "GEAR");
            pair(&mut s, 10, &num(x));
            pair(&mut s, 20, &num(y));
            if bulge != 0.0 {
                pair(&mut s, 42, &format!("{bulge:.10}"));
            }
        }
        pair(&mut s, 0, "SEQEND");
        pair(&mut s, 8, "GEAR");
        if sh.bore > 0.0 {
            pair(&mut s, 0, "CIRCLE");
            pair(&mut s, 8, "BORE");
            pair(&mut s, 10, &num(sh.center[0]));
            pair(&mut s, 20, &num(sh.center[1]));
            pair(&mut s, 40, &num(sh.bore / 2.0));
        }
    }
    pair(&mut s, 0, "ENDSEC");
    pair(&mut s, 0, "EOF");
    s
}

pub fn svg(shapes: &[Shape], inch: bool) -> String {
    let shapes = &in_units(shapes, inch);
    let unit = if inch { "in" } else { "mm" };
    let mut lo = [f64::MAX; 2];
    let mut hi = [f64::MIN; 2];
    for (q, _) in shapes.iter().flat_map(|s| &s.path) {
        for k in 0..2 {
            lo[k] = lo[k].min(q[k]);
            hi[k] = hi[k].max(q[k]);
        }
    }
    if shapes.is_empty() {
        lo = [0.0; 2];
        hi = [1.0; 2];
    }
    let margin = if inch { 0.08 } else { 2.0 };
    let (w, h) = (hi[0] - lo[0] + 2.0 * margin, hi[1] - lo[1] + 2.0 * margin);
    // SVG's Y axis points down; flip so the drawing matches the canvas.
    let mut s = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w:.4}{unit}\" height=\"{h:.4}{unit}\" \
         viewBox=\"{:.5} {:.5} {w:.5} {h:.5}\">\n<g transform=\"scale(1,-1)\" fill=\"none\" \
         stroke=\"black\" stroke-width=\"{}\">\n",
        lo[0] - margin,
        -hi[1] - margin,
        if inch { 0.004 } else { 0.1 },
    );
    for sh in shapes {
        let _ = write!(s, "<g id=\"{}\">\n<path d=\"", xml_escape(&sh.name));
        let n = sh.path.len();
        if let Some(([x, y], _)) = sh.path.first() {
            let _ = write!(s, "M{x:.6},{y:.6} ");
        }
        for k in 0..n {
            let (a, bulge) = sh.path[k];
            let [x, y] = sh.path[(k + 1) % n].0;
            if bulge == 0.0 {
                let _ = write!(s, "L{x:.6},{y:.6} ");
            } else {
                let theta = 4.0 * bulge.abs().atan();
                let r = dist(a, [x, y]) / (2.0 * (theta / 2.0).sin());
                let large = u8::from(theta > std::f64::consts::PI);
                // The path is in Y-up coordinates, where sweep 1 is counter-clockwise.
                let sweep = u8::from(bulge > 0.0);
                let _ = write!(s, "A{r:.6},{r:.6} 0 {large} {sweep} {x:.6},{y:.6} ");
            }
        }
        s.push_str("Z\"/>\n");
        if sh.bore > 0.0 {
            let _ = writeln!(
                s,
                "<circle cx=\"{:.4}\" cy=\"{:.4}\" r=\"{:.4}\"/>",
                sh.center[0],
                sh.center[1],
                sh.bore / 2.0
            );
        }
        s.push_str("</g>\n");
    }
    s.push_str("</g>\n</svg>\n");
    s
}

fn xml_escape(t: &str) -> String {
    t.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gear::GearParams;

    /// Distance from q to the arc or line leaving vertex k.
    fn seg_dist(path: &[([f64; 2], f64)], k: usize, q: [f64; 2]) -> f64 {
        let (a, bulge) = path[k];
        let b = path[(k + 1) % path.len()].0;
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len2 = dx * dx + dy * dy;
        if bulge == 0.0 {
            let t = (((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / len2).clamp(0.0, 1.0);
            return dist(q, [a[0] + dx * t, a[1] + dy * t]);
        }
        let theta = 4.0 * bulge.atan();
        let r = len2.sqrt() / (2.0 * (theta / 2.0).sin().abs());
        let h = r * (theta / 2.0).cos().abs() * if theta.abs() > std::f64::consts::PI { -1.0 } else { 1.0 };
        let l = len2.sqrt();
        let left = [-dy / l, dx / l];
        let m = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
        let s = if bulge > 0.0 { 1.0 } else { -1.0 };
        let c = [m[0] + left[0] * h * s, m[1] + left[1] * h * s];
        (dist(q, c) - r).abs()
    }

    #[test]
    fn arcs_follow_the_profile_with_few_segments() {
        for (z, x, m) in [(20, 0.0, 2.0), (8, 0.0, 1.0), (60, 0.2, 0.5), (12, 0.5, 25.4 / 12.0)] {
            let g = GearParams { teeth: z, profile_shift: x, module: m, ..Default::default() };
            let prof = gear::tooth_profile(&g).unwrap();
            let outline = gear::outline(&g, &prof);
            let tol = tolerance(m);
            let path = fit_arcs(&outline, tol);
            let per_tooth = path.len() as f64 / z as f64;
            assert!(per_tooth <= 24.0, "z={z}: {per_tooth} segments per tooth");
            // Every source point is near some fitted segment, and no
            // segment is short enough for CAD import to merge its ends.
            for &q in &outline {
                let d = (0..path.len()).map(|k| seg_dist(&path, k, q)).fold(f64::MAX, f64::min);
                assert!(d <= tol * 1.01, "z={z}: point off by {d}");
            }
            let shortest = (0..path.len()).map(|k| dist(path[k].0, path[(k + 1) % path.len()].0)).fold(f64::MAX, f64::min);
            assert!(shortest > 1e-3, "z={z}: shortest segment {shortest}");
        }
    }

    #[test]
    fn dxf_has_no_pitch_circle_and_uses_bulges() {
        let text = dxf(&shapes(&Project::default(), 0.0, None, |_| true), false);
        assert!(!text.contains("PITCH"));
        assert!(text.contains("\n42\n"));
        assert_eq!(text.matches("\nCIRCLE\n").count(), 2); // the two bores
    }
}

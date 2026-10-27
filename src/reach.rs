//! Can a ball end mill reach the teeth on the 4-axis mill?
//!
//! Every part is cut spinning on the A axis about its own axis, the tool
//! square to that axis. So at each point of a tooth surface the ball can
//! sit on it (its centre a radius out along the surface's normal) with
//! the tool pointing any way square to the part's axis that keeps the
//! point on the ball's lower half; the point is reachable if, along one
//! of those ways, the ball and a shank as wide as it clear the part.
//!
//! The part is a cloud of points on its tooth surfaces with their outward
//! normals, in the part's own frame (axis +z). A tool body that cut into
//! the part would have some of the cloud inside it, the cloud being far
//! denser than the ball is wide.

use crate::bevel::BevelGeom;
use crate::gear::{self, GearParams};
use crate::hypoid::Member;
use crate::step::{Patch, tooth_data};
use std::collections::HashMap;
use std::f64::consts::TAU;

type V3 = [f64; 3];

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn mul(k: f64, a: V3) -> V3 {
    [k * a[0], k * a[1], k * a[2]]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn unit(a: V3) -> V3 {
    mul(1.0 / dot(a, a).sqrt().max(1e-300), a)
}

/// Tooth surfaces as points with outward unit normals; `test` marks the
/// ones to check (a tooth's worth: the rest are there to be hit).
#[derive(Default)]
pub struct Surface {
    pub pts: Vec<V3>,
    pub normals: Vec<V3>,
    pub test: Vec<bool>,
}

impl Surface {
    fn push(&mut self, p: V3, n: V3, test: bool) {
        self.pts.push(p);
        self.normals.push(unit(n));
        self.test.push(test);
    }
}

/// What the check found.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Reach {
    /// Points checked, and those no tool direction reaches.
    pub tested: usize,
    pub missed: Vec<V3>,
    /// The radii from the part's axis the missed points span.
    pub radii: Option<[f64; 2]>,
}

impl Reach {
    /// The share of the tooth surface out of reach.
    pub fn share(&self) -> f64 {
        self.missed.len() as f64 / self.tested.max(1) as f64
    }
}

/// The share of a part's teeth out of reach that is only stray points at
/// sharp corners, not worth reshaping or warning about.
pub const STRAY: f64 = 0.005;

/// Tool directions tried either side of the one nearest the normal.
const FAN: [f64; 9] = [0.0, 20.0, -20.0, 40.0, -40.0, 60.0, -60.0, 80.0, -80.0];

/// Check every test point of `s` for a ball end mill of Ø `ball`.
pub fn check(s: &Surface, ball: f64) -> Reach {
    let rb = ball / 2.0;
    let mut out = Reach::default();
    if rb <= 0.0 || s.pts.is_empty() {
        return out;
    }
    // Cloud points in cubes a ball radius across.
    let cell = rb.max(1e-3);
    let key = |p: V3| [(p[0] / cell).floor() as i32, (p[1] / cell).floor() as i32, (p[2] / cell).floor() as i32];
    let mut grid: HashMap<[i32; 3], Vec<u32>> = HashMap::new();
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    for (i, &p) in s.pts.iter().enumerate() {
        grid.entry(key(p)).or_default().push(i as u32);
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    // The contact surface itself sits a ball radius off the centre; a
    // root fillet the ball's own size (the roots are made so) only just,
    // and the splines through its samples a few percent closer. That the
    // ball fits the roots is made sure of where they are designed: here
    // it is whether the tool can get there.
    let clear = rb * 0.92 - 1e-4;
    // Whether the capsule from c along d, out of the cloud's box, holds
    // a cloud point.
    let hits = |c: V3, d: V3| {
        let mut len = f64::MAX;
        for k in 0..3 {
            if d[k].abs() > 1e-12 {
                let far = if d[k] > 0.0 { hi[k] + rb - c[k] } else { lo[k] - rb - c[k] };
                len = len.min((far / d[k]).max(0.0));
            }
        }
        let len = len.min(1e4);
        let steps = (len / cell).ceil() as i64 + 1;
        // A step is a cell long, so the cubes round a step only overlap
        // those round the two before.
        let mut recent: [Vec<[i32; 3]>; 3] = Default::default();
        for i in 0..=steps {
            let q = add(c, mul((i as f64 * cell).min(len), d));
            let k0 = key(q);
            let [older, old, now] = &mut recent;
            std::mem::swap(older, old);
            std::mem::swap(old, now);
            now.clear();
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let k = [k0[0] + dx, k0[1] + dy, k0[2] + dz];
                        if now.contains(&k) || old.contains(&k) || older.contains(&k) {
                            continue;
                        }
                        now.push(k);
                        let Some(ids) = grid.get(&k) else { continue };
                        for &j in ids {
                            let w = sub(s.pts[j as usize], c);
                            let along = dot(w, d).clamp(0.0, len);
                            let off = sub(w, mul(along, d));
                            if dot(off, off) < clear * clear {
                                return true;
                            }
                        }
                    }
                }
            }
        }
        false
    };
    // At most a few thousand points checked, spread evenly, on threads
    // of their own.
    let picked: Vec<usize> = (0..s.pts.len()).filter(|&i| s.test[i]).collect();
    let stride = (picked.len() / 4000).max(1);
    let picked: Vec<usize> = picked.into_iter().skip(stride - 1).step_by(stride).collect();
    let reached = |i: usize| {
        let n = s.normals[i];
        let c = add(s.pts[i], mul(rb, n));
        // Square to the axis, nearest the normal; a normal along the axis
        // leaves every way round open.
        let flat = [n[0], n[1], 0.0];
        let fan: Vec<f64> = if dot(flat, flat).sqrt() > 1e-3 {
            let base = flat[1].atan2(flat[0]);
            FAN.iter().map(|a| base + a.to_radians()).collect()
        } else {
            (0..12).map(|k| TAU * k as f64 / 12.0).collect()
        };
        fan.iter().any(|&a| {
            let d = [a.cos(), a.sin(), 0.0];
            dot(d, n) >= -1e-9 && !hits(c, d)
        })
    };
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get()).clamp(1, 16);
    let missed: Vec<usize> = std::thread::scope(|scope| {
        let jobs: Vec<_> = picked
            .chunks(picked.len().div_ceil(threads).max(1))
            .map(|part| scope.spawn(|| part.iter().copied().filter(|&i| !reached(i)).collect::<Vec<_>>()))
            .collect();
        jobs.into_iter().flat_map(|j| j.join().unwrap()).collect()
    });
    out.tested = picked.len();
    for i in missed {
        let p = s.pts[i];
        out.missed.push(p);
        let r = p[0].hypot(p[1]);
        out.radii = Some(match out.radii {
            Some([a, b]) => [a.min(r), b.max(r)],
            None => [r, r],
        });
    }
    out
}

/// Missed points of tooth 0 (`missed`) repeated on all `teeth`, and for
/// a spur gear's single section (`face` its width) along the face, to
/// show where the tool can't reach; thinned to a few thousand.
pub fn on_every_tooth(missed: &[V3], teeth: u32, face: Option<f64>) -> Vec<V3> {
    let layers: Vec<f64> = match face {
        Some(f) => (0..=6).map(|k| f * k as f64 / 6.0).collect(),
        None => vec![0.0],
    };
    let per = (12000 / (teeth.max(1) as usize * layers.len())).max(1);
    let stride = missed.len().div_ceil(per).max(1);
    let mut out = Vec::new();
    for t in 0..teeth {
        let (s, c) = (TAU * t as f64 / teeth as f64).sin_cos();
        for q in missed.iter().step_by(stride) {
            for &z in &layers {
                out.push([q[0] * c - q[1] * s, q[0] * s + q[1] * c, q[2] + z]);
            }
        }
    }
    out
}

/// A closed loop of (x, y, tag) with no step longer than `step` says;
/// `wrap` is added to the first point's y to close it (2π for a loop of
/// azimuths). Points between keep the tag of the one before.
fn densify(pts: &[[f64; 3]], step: impl Fn([f64; 3], [f64; 3]) -> f64, wrap: f64) -> Vec<[f64; 3]> {
    let mut out = Vec::new();
    let n = pts.len();
    for i in 0..n {
        let (a, mut b) = (pts[i], pts[(i + 1) % n]);
        if i + 1 == n {
            b[1] += wrap;
        }
        let k = step(a, b).ceil().max(1.0) as usize;
        for j in 0..k {
            let t = j as f64 / k as f64;
            out.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2]]);
        }
    }
    out
}

/// A spur gear or pulley: its teeth are the same all along the axis, and
/// a tool square to the axis lies in the plane of the outline, so one
/// section is the whole story. Tooth 0 and the spaces either side are
/// checked.
pub fn spur(p: &GearParams, ball: f64) -> Option<Surface> {
    let prof = gear::tooth_profile(p)?;
    let raw = gear::outline(p, &prof);
    let z = p.teeth as usize;
    let per = raw.len() / z;
    let gap = (ball / 6.0).max(1e-3);
    // Tooth 0 runs from index 0 to `per`; mark it before densifying.
    let tagged: Vec<[f64; 3]> = raw.iter().enumerate().map(|(i, q)| [q[0], q[1], if i < per { 1.0 } else { 0.0 }]).collect();
    let pts = densify(&tagged, |a, b| (b[0] - a[0]).hypot(b[1] - a[1]) / gap, 0.0);
    let mut s = Surface::default();
    let m = pts.len();
    for i in 0..m {
        let (a, b) = (pts[(i + m - 1) % m], pts[(i + 1) % m]);
        // Anticlockwise outline: outward is the tangent turned clockwise.
        let n = [b[1] - a[1], -(b[0] - a[0]), 0.0];
        s.push([pts[i][0], pts[i][1], 0.0], n, pts[i][2] > 0.5);
    }
    Some(s)
}

/// A straight bevel gear: its tooth outline on the sphere, swept along
/// the rays from the apex between the front and back cones.
pub fn bevel(g: &BevelGeom, ball: f64) -> Surface {
    let z = g.teeth as usize;
    let raw = g.outline();
    let per = raw.len() / z;
    let k = g.front_ratio();
    let heel = g.back(g.tip);
    let gap = (ball / 6.0).max(1e-3);
    // Arc on the unit sphere, at the heel in mm.
    let tagged: Vec<[f64; 3]> = raw
        .iter()
        .enumerate()
        .map(|(i, q)| {
            let t = (i / per) as i64;
            [q[0], q[1], (t.min(z as i64 - t)) as f64]
        })
        .collect();
    let pts = densify(&tagged, |a, b| (b[0] - a[0]).hypot((b[1] - a[1]) * a[0].sin()) * heel / gap, TAU);
    let near = if z <= 8 { z as f64 } else { 3.0 };
    let mut s = Surface::default();
    let m = pts.len();
    let at = |q: [f64; 3], rho: f64| BevelGeom::point(q[0], q[1], rho);
    for i in 0..m {
        let q = pts[i];
        if q[2] > near {
            continue;
        }
        let (a, b) = (pts[(i + m - 1) % m], pts[(i + 1) % m]);
        let (back, front) = (g.back(q[0]), k * g.back(q[0]));
        let steps = ((back - front) / gap).ceil().max(1.0) as usize;
        for j in 0..=steps {
            let rho = front + (back - front) * j as f64 / steps as f64;
            let p = at(q, rho);
            // Along the outline (anticlockwise seen from outside the
            // sphere) and out along the ray: outward is their cross.
            let along = sub(at(b, rho), at(a, rho));
            let ray = unit(p);
            s.push(p, cross(along, ray), q[2] == 0.0);
        }
    }
    s
}

/// A generated member (hypoid, spiral bevel, bevel worm, worm or wheel):
/// the flank and tip patches its STEP faces are made from.
pub fn member(m: &Member, ball: f64) -> Surface {
    let z = m.teeth as usize;
    let ns = m.stations.len();
    let u_params: Vec<f64> = (0..ns).map(|k| k as f64 / (ns - 1) as f64).collect();
    let gap = (ball / 3.0).max(1e-3);
    let near: Vec<usize> = if z <= 8 { (0..z).collect() } else { (0..=3).chain(z - 3..z).collect() };
    let mut s = Surface::default();
    for t in near {
        let data = tooth_data(m, t);
        let mid = ns / 2;
        for (which, d) in data.iter().enumerate() {
            let patch = Patch::through(d, &u_params);
            // Lengths across and along the patch, for the sample counts.
            let span = |a: &[V3]| a.windows(2).map(|w| dot(sub(w[1], w[0]), sub(w[1], w[0])).sqrt()).sum::<f64>();
            let along = span(&d.iter().map(|row| row[row.len() / 2]).collect::<Vec<_>>());
            let across = span(&d[mid]);
            let (nu, nv) = ((along / gap).ceil().max(2.0) as usize, (across / gap).ceil().max(2.0) as usize);
            // Outward: off the flanks round the axis, off the tip away
            // from it.
            let q = d[mid][d[mid].len() / 2];
            let phi = q[1].atan2(q[0]);
            let out = match which {
                0 => [phi.sin(), -phi.cos(), 0.0],
                2 => [-phi.sin(), phi.cos(), 0.0],
                _ => [q[0], q[1], 0.0],
            };
            // One-sided differences, stepping inwards at the far edges.
            let h = 1e-4;
            let at = |u: f64, v: f64| {
                let p = patch.eval(u, v);
                let (hu, hv) = (if u + h > 1.0 { -h } else { h }, if v + h > 1.0 { -h } else { h });
                let du = mul(1.0 / hu, sub(patch.eval(u + hu, v), p));
                let dv = mul(1.0 / hv, sub(patch.eval(u, v + hv), p));
                (p, cross(du, dv))
            };
            let flip = if dot(at(0.5, 0.5).1, out) < 0.0 { -1.0 } else { 1.0 };
            for a in 0..=nu {
                for b in 0..=nv {
                    let (u, v) = (a as f64 / nu as f64, b as f64 / nv as f64);
                    let (p, n) = at(u, v);
                    s.push(p, mul(flip, n), t == 0);
                }
            }
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bevel::Bevel;

    #[test]
    fn spur_teeth_are_reached() {
        // Undercut too: on so few teeth the spaces are wide enough for a
        // tool leaned across them to see under the flanks.
        for teeth in [30, 8] {
            let p = GearParams { teeth, ..Default::default() };
            let r = check(&spur(&p, 1.0).unwrap(), 1.0);
            assert!(r.tested > 100);
            assert!(r.missed.is_empty(), "{teeth} teeth: {} of {} missed at {:?}", r.missed.len(), r.tested, r.radii);
        }
    }

    #[test]
    fn bevel_teeth_are_reached() {
        for delta in [26.565, 63.435] {
            let p = GearParams { teeth: 30, module: 2.0, face_width: 10.0, bevel: Some(Bevel { pitch_angle: delta }), ball: 1.0, ..Default::default() };
            let g = BevelGeom::new(&p).unwrap();
            let r = check(&bevel(&g, 1.0), 1.0);
            assert!(r.tested > 100);
            assert!(r.missed.is_empty(), "bevel {delta}°: {} of {} missed at {:?}", r.missed.len(), r.tested, r.radii);
        }
    }

    /// A dovetail slot: its undercut corners face away from every tool
    /// square to the axis.
    #[test]
    fn a_dovetail_is_out_of_reach() {
        // Anticlockwise: a block 20 wide and 10 high, a slot 4 deep in
        // its top, 6 wide at the mouth and 10 at the bottom.
        let loop_ = [[-10.0, 0.0], [10.0, 0.0], [10.0, 10.0], [3.0, 10.0], [5.0, 6.0], [-5.0, 6.0], [-3.0, 10.0], [-10.0, 10.0]];
        let tagged: Vec<[f64; 3]> = loop_.iter().map(|q| [q[0], q[1] + 20.0, 1.0]).collect();
        let pts = densify(&tagged, |a, b| (b[0] - a[0]).hypot(b[1] - a[1]) / 0.1, 0.0);
        let mut s = Surface::default();
        let m = pts.len();
        for i in 0..m {
            let (a, b) = (pts[(i + m - 1) % m], pts[(i + 1) % m]);
            s.push([pts[i][0], pts[i][1], 0.0], [b[1] - a[1], -(b[0] - a[0]), 0.0], true);
        }
        let r = check(&s, 1.0);
        let bottom = r.missed.iter().filter(|q| q[1] < 26.5).count();
        assert!(bottom > 0, "the slot's undercut corners are reached");
        // Everything outside the slot is in reach.
        assert!(r.missed.iter().all(|q| q[1] < 30.0 - 1e-6 && q[0].abs() < 5.0 + 1e-6), "{:?}", r.missed);
    }

    /// Hypoid and worm teeth are open to a tool square to the axis; the
    /// spiral bevel ring's spaces curve too much for a ball this size to
    /// get down them, and are flagged.
    #[test]
    fn generated_members() {
        use crate::hypoid::Spec;
        use crate::hypoid::tests::car;
        let worms = [crate::worm::tests::spec(1, 30, true), crate::worm::tests::spec(2, 30, false)];
        let mut parts: Vec<(String, Member, f64, f64)> = Vec::new();
        for sp in worms {
            let d = sp.design().unwrap();
            let ball = 2.0 * d.ball;
            parts.push((format!("worm {}", sp.throated), d.worm().unwrap(), ball, 0.005));
            parts.push((format!("wheel {}", sp.throated), d.wheel().unwrap(), ball, 0.005));
        }
        let hypoid = Spec { ball: 2.5, ..car() };
        let d = hypoid.design().unwrap();
        parts.push(("hypoid pinion".into(), d.pinion().unwrap(), hypoid.ball, 0.0));
        parts.push(("hypoid ring".into(), d.ring().unwrap(), hypoid.ball, 0.08));
        for (what, m, ball, most) in parts {
            let r = check(&member(&m, ball), ball);
            assert!(r.tested > 500, "{what}: {} tested", r.tested);
            assert!(r.share() <= most, "{what}: {:.3} out of reach at {:?}", r.share(), r.radii);
        }
        let spiral = Spec { offset: 0.0, ball: 2.4, ..car() };
        let ring = spiral.design().unwrap().ring().unwrap();
        let r = check(&member(&ring, spiral.ball), spiral.ball);
        assert!(r.share() > 0.05, "spiral bevel ring: {:.3}", r.share());
    }
}


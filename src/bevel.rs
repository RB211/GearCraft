//! Straight bevel gears with spherical involute teeth.
//!
//! A bevel gear's teeth live on a cone. Every tooth flank is a cone through
//! the apex whose cross-section on a sphere about the apex is a spherical
//! involute: the curve traced by a point on a great circle rolling on the
//! base cone, the exact counterpart of the plane involute. So the tooth
//! shrinks in proportion towards the apex and a mating pair has conjugate
//! action along the whole face.
//!
//! Coordinates: apex at the origin, axis +z, the back (large end) of the
//! gear away from the apex. Directions from the apex are given by their
//! polar angle γ from the axis and azimuth φ, tooth 0 centred on φ = 0.
//!
//! Proportions follow the equal-addendum system: addendum 1 m and
//! dedendum 1.25 m at the large end (m is the outer transverse module), the
//! tooth ends on the back and front cones, a flat back and front face
//! across the hub, and the bore through both.

use crate::gear::{GearParams, Issue, Profile};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

/// What a bevel gear takes from its mate: the pitch cone angle, set when
/// the pair is solved.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Bevel {
    /// Pitch cone half-angle δ, degrees.
    pub pitch_angle: f64,
}

impl Default for Bevel {
    fn default() -> Self {
        Bevel { pitch_angle: 45.0 }
    }
}

/// Pitch cone angles of a pair with `z1` and `z2` teeth at shaft angle
/// `sigma` (radians): δ₁ = atan(sin Σ / (z₂/z₁ + cos Σ)), δ₂ = Σ − δ₁.
pub fn pitch_angles(z1: u32, z2: u32, sigma: f64) -> (f64, f64) {
    let d1 = sigma.sin().atan2(z2 as f64 / z1 as f64 + sigma.cos());
    (d1, sigma - d1)
}

/// A root fillet, the same in every section through the apex: a circle
/// in (γ, φ·sin γ), the polar angle and arc round the axis on the unit
/// sphere, so its radius grows with the distance from the apex.
#[derive(Clone, Copy, Debug)]
pub struct Fillet {
    /// Angular radius, the centre (γ, arc from the tooth's middle), and
    /// the polar angle where it runs onto the flank.
    pub radius: f64,
    pub centre: [f64; 2],
    pub foot: f64,
}

impl Fillet {
    /// Arc from the tooth's middle at polar angle γ (root ≤ γ ≤ foot).
    fn arc(&self, gamma: f64) -> f64 {
        let [gc, yc] = self.centre;
        yc - (self.radius * self.radius - (gamma - gc) * (gamma - gc)).max(0.0).sqrt()
    }

    /// The fillet by the angle ψ round its centre from the root (0) to the
    /// foot: polar angle and arc from the tooth's middle.
    pub fn at(&self, psi: f64) -> (f64, f64) {
        let [gc, yc] = self.centre;
        (gc - self.radius * psi.cos(), yc - self.radius * psi.sin())
    }

    /// ψ at the foot.
    pub fn end(&self) -> f64 {
        ((self.centre[0] - self.foot) / self.radius).clamp(-1.0, 1.0).acos()
    }
}

/// Everything needed to draw or export one bevel gear.
#[derive(Clone, Debug)]
pub struct BevelGeom {
    pub teeth: u32,
    /// Pitch, base, tip and root cone half-angles, radians.
    pub delta: f64,
    pub base: f64,
    pub tip: f64,
    pub root: f64,
    /// Outer cone distance (apex to the pitch circle at the large end) and
    /// the face width along the cone.
    pub cone_distance: f64,
    pub face: f64,
    /// Polar angles γ (in `radii`) and tooth half-angles in azimuth (in
    /// `half`), from the root cone to the tip cone.
    pub profile: Profile,
    pub bore: f64,
    /// Tooth half-angle in azimuth on the base cone; at polar angle γ it
    /// is this less the involute's sweep.
    pub base_half: f64,
    /// The root fillet, None if there is none. If the ball end mill is too
    /// big for the spaces, the biggest that fits.
    pub fillet: Option<Fillet>,
}

impl BevelGeom {
    pub fn new(p: &GearParams) -> Option<BevelGeom> {
        let (geom, lowest, toe) = Self::unfilleted(p)?;
        // The fillet is tightest at the toe: there it is the ball's radius.
        let want = if p.ball > 0.0 { p.ball / 2.0 / toe } else { p.root_fillet * p.module / geom.cone_distance };
        let geom = match geom.with_fillet(want, lowest) {
            Some(g) => g,
            None => {
                // The biggest that fits.
                let (mut lo, mut hi) = (0.0, want);
                for _ in 0..30 {
                    let mid = (lo + hi) / 2.0;
                    if geom.with_fillet(mid, lowest).is_some() { lo = mid } else { hi = mid }
                }
                geom.with_fillet(lo, lowest)?
            }
        };
        if geom.root <= 0.02 || geom.root >= PI / 2.0 - 0.01 {
            return None;
        }
        Some(BevelGeom { profile: geom.tooth_profile(), ..geom })
    }

    /// The biggest ball end mill Ø that rounds the roots at the toe.
    pub fn largest_ball(p: &GearParams) -> Option<f64> {
        let (geom, lowest, toe) = Self::unfilleted(p)?;
        let (mut lo, mut hi) = (0.0, p.module / toe);
        for _ in 0..30 {
            let mid = (lo + hi) / 2.0;
            if geom.with_fillet(mid, lowest).is_some() { lo = mid } else { hi = mid }
        }
        Some(2.0 * lo * toe)
    }

    /// The gear with no root fillet, the polar angle the mate's tips reach
    /// down to, and the toe's distance from the apex.
    fn unfilleted(p: &GearParams) -> Option<(BevelGeom, f64, f64)> {
        let bevel = p.bevel?;
        let delta = bevel.pitch_angle.to_radians();
        let alpha = p.alpha();
        if p.module <= 0.0 || p.teeth < 4 || !(0.01..PI / 2.0 + 0.6).contains(&delta) || alpha <= 0.0 {
            return None;
        }
        let r_pitch = p.module * p.teeth as f64 / 2.0;
        let cone_distance = r_pitch / delta.sin();
        let face = p.face_width.clamp(0.05 * cone_distance, 0.6 * cone_distance);
        let tip = delta + p.module * p.addendum / cone_distance;
        let root = delta - p.module * p.dedendum / cone_distance;
        let base = (delta.sin() * alpha.cos()).asin();
        let geom = BevelGeom {
            teeth: p.teeth,
            delta,
            base,
            tip,
            root,
            cone_distance,
            face,
            profile: Profile { radii: vec![], half: vec![] },
            bore: p.bore,
            base_half: 0.0,
            fillet: None,
        };
        let r_pitch = cone_distance * delta.sin();
        let base_half = PI / (2.0 * p.teeth as f64) - p.backlash / (4.0 * r_pitch) + geom.involute(delta);
        let geom = BevelGeom { base_half, ..geom };
        let toe = cone_distance * geom.front_ratio();
        // Roots deep enough that the flanks run on below the mate's tips
        // (less a thousandth of a module, as for spur gears).
        let lowest = delta - p.module * (p.addendum - 1e-3) / cone_distance;
        Some((geom, lowest, toe))
    }

    /// The gear with a root fillet of angular radius `rho`, its roots
    /// deepened if need be so the fillet runs onto the flank no higher
    /// than polar angle `lowest`. None if the space is too narrow for it.
    fn with_fillet(&self, rho: f64, lowest: f64) -> Option<BevelGeom> {
        if rho <= 1e-9 {
            return Some(BevelGeom { fillet: None, ..self.clone() });
        }
        let mut root = self.root;
        for _ in 0..12 {
            let f = self.fillet_at(root, rho)?;
            let over = f.foot - lowest;
            if over <= 1e-8 {
                // Keep a land between the fillets.
                let mid = PI / self.teeth as f64 * root.sin();
                return (f.arc(root) <= 0.99 * mid).then(|| BevelGeom { root, fillet: Some(f), ..self.clone() });
            }
            root -= over;
        }
        None
    }

    /// The circle of angular radius ρ touching the root cone at `root` and
    /// the flank: its centre is ρ off the flank, square to it, at γ =
    /// root + ρ.
    fn fillet_at(&self, root: f64, rho: f64) -> Option<Fillet> {
        let gc = root + rho;
        // The flank's slope and the space-side normal's γ part at γ.
        let slope = |g: f64| {
            let h = 1e-7;
            (self.flank_arc(g + h) - self.flank_arc(g - h)) / (2.0 * h)
        };
        let off = |g: f64| {
            let d = slope(g);
            g - rho * d / (1.0 + d * d).sqrt() - gc
        };
        let (mut lo, mut hi) = (root, root + 2.0 * rho);
        if off(lo) > 0.0 || off(hi) < 0.0 {
            return None;
        }
        for _ in 0..40 {
            let mid = (lo + hi) / 2.0;
            if off(mid) < 0.0 { lo = mid } else { hi = mid }
        }
        let foot = (lo + hi) / 2.0;
        let d = slope(foot);
        let yc = self.flank_arc(foot) + rho / (1.0 + d * d).sqrt();
        (foot > root && self.half_flank(foot) > 0.0).then_some(Fillet { radius: rho, centre: [gc, yc], foot })
    }

    /// The flank's arc from the tooth's middle on the unit sphere.
    fn flank_arc(&self, gamma: f64) -> f64 {
        self.half_flank(gamma) * gamma.sin()
    }

    fn half_flank(&self, gamma: f64) -> f64 {
        self.base_half - self.involute(gamma.max(self.base))
    }

    /// Azimuth swept by the spherical involute of the base cone from its
    /// start on the base cone out to polar angle γ.
    fn involute(&self, gamma: f64) -> f64 {
        let gb = self.base;
        if gamma <= gb {
            return 0.0;
        }
        // Roll angle θ about the axis; the point lies ψ = θ·sin γb along
        // the rolling great circle from where it touches the base cone.
        let theta = (gamma.cos() / gb.cos()).clamp(-1.0, 1.0).acos() / gb.sin();
        let psi = theta * gb.sin();
        let x = psi.cos() * gb.sin() * theta.cos() + psi.sin() * theta.sin();
        let y = psi.cos() * gb.sin() * theta.sin() - psi.sin() * theta.cos();
        y.atan2(x).abs()
    }

    /// The involute by its roll angle θ, which is smooth where γ is not
    /// (at the base cone): polar angle and the azimuth swept.
    pub fn roll(&self, theta: f64) -> (f64, f64) {
        let gb = self.base;
        let psi = theta * gb.sin();
        let gamma = (gb.cos() * psi.cos()).clamp(-1.0, 1.0).acos();
        let x = psi.cos() * gb.sin() * theta.cos() + psi.sin() * theta.sin();
        let y = psi.cos() * gb.sin() * theta.sin() - psi.sin() * theta.cos();
        (gamma, y.atan2(x))
    }

    /// Roll angle at which the involute reaches polar angle γ.
    pub fn roll_at(&self, gamma: f64) -> f64 {
        let gb = self.base;
        (gamma.cos() / gb.cos()).clamp(-1.0, 1.0).acos() / gb.sin()
    }

    /// Tooth half-angle in azimuth at polar angle γ.
    pub fn half_at(&self, gamma: f64) -> f64 {
        match self.fillet {
            Some(f) if gamma < f.foot => f.arc(gamma.max(self.root)) / gamma.sin(),
            _ => self.half_flank(gamma).max(0.0),
        }
    }

    /// Where the flank proper starts: the fillet's foot, or the root.
    pub fn flank_foot(&self) -> f64 {
        self.fillet.map_or(self.root, |f| f.foot)
    }

    fn tooth_profile(&self) -> Profile {
        let n = 48;
        let foot = self.flank_foot();
        let mut radii: Vec<f64> = (0..=n).map(|i| foot + (self.tip - foot) * i as f64 / n as f64).collect();
        // The fillet by equal turns round its centre.
        if let Some(f) = self.fillet {
            let m = 12;
            radii.extend((0..m).map(|i| f.at(f.end() * i as f64 / m as f64).0));
        }
        if self.base > foot && self.base < self.tip {
            radii.push(self.base);
        }
        radii.sort_by(f64::total_cmp);
        // Half a tooth at the pitch cone, less half the backlash (as an
        // angle on the large-end pitch circle), set in `base_half`. Below
        // the base cone the flank runs straight down to the root.
        let half = radii.iter().map(|&g| self.half_at(g)).collect();
        Profile { radii, half }
    }

    /// Outer and inner ends of the teeth: distance from the apex along the
    /// ray at polar angle γ to the back cone, and the ratio of the front
    /// cone's distance to it.
    pub fn back(&self, gamma: f64) -> f64 {
        self.cone_distance / (gamma - self.delta).cos()
    }

    pub fn front_ratio(&self) -> f64 {
        (self.cone_distance - self.face) / self.cone_distance
    }

    /// Heights of the flat back and front faces along the axis, and their
    /// radii (out to the root cone).
    pub fn back_face(&self) -> (f64, f64) {
        let z = self.back(self.root) * self.root.cos();
        (z, z * self.root.tan())
    }

    pub fn front_face(&self) -> (f64, f64) {
        let (z, r) = self.back_face();
        (z * self.front_ratio(), r * self.front_ratio())
    }

    /// A point at polar angle γ, azimuth φ, distance ρ from the apex.
    pub fn point(gamma: f64, phi: f64, rho: f64) -> [f64; 3] {
        [rho * gamma.sin() * phi.cos(), rho * gamma.sin() * phi.sin(), rho * gamma.cos()]
    }

    /// The tooth outline in (γ, φ) as a closed anticlockwise loop: up the
    /// right flank, across the tip, down the left flank, along the root.
    pub fn outline(&self) -> Vec<[f64; 2]> {
        let z = self.teeth as usize;
        let pitch = TAU / z as f64;
        let (radii, half) = (&self.profile.radii, &self.profile.half);
        let n = radii.len();
        let mut out = Vec::new();
        for k in 0..z {
            let c = k as f64 * pitch;
            for i in 0..n {
                out.push([radii[i], c - half[i]]);
            }
            let (ht, hr) = (half[n - 1], half[0]);
            for j in 1..6 {
                out.push([radii[n - 1], c - ht + 2.0 * ht * j as f64 / 6.0]);
            }
            for i in (0..n).rev() {
                out.push([radii[i], c + half[i]]);
            }
            let span = pitch - 2.0 * hr;
            let steps = ((span / 0.05).ceil() as usize).max(2);
            for j in 1..steps {
                out.push([radii[0], c + hr + span * j as f64 / steps as f64]);
            }
        }
        out
    }

    /// Outside diameter at the large end (across the tooth tips).
    pub fn outside_diameter(&self) -> f64 {
        2.0 * self.back(self.tip) * self.tip.sin()
    }

    /// Virtual (Tredgold) spur tooth count, for undercut and contact checks.
    pub fn virtual_teeth(&self) -> f64 {
        self.teeth as f64 / self.delta.cos().abs().max(1e-6)
    }

    pub fn issues(&self, len: impl Fn(f64) -> String) -> Vec<Issue> {
        let mut out = Vec::new();
        let zv = self.virtual_teeth();
        let half = &self.profile.half;
        if half.last().is_some_and(|&h| h <= 0.0) {
            out.push(Issue::error("Teeth are pointed: the flanks meet below the tip cone."));
        }
        if self.root < self.base - 1e-9 && zv < 14.0 {
            out.push(Issue::warn(format!(
                "Only {zv:.1} virtual teeth: a generated pinion this small is undercut. \
                 Use more teeth or a larger shaft angle on the mate."
            )));
        }
        if self.face > self.cone_distance / 3.0 + 1e-9 {
            out.push(Issue::warn(format!(
                "Face width over a third of the cone distance ({}): the small end gets weak.",
                len(self.cone_distance / 3.0)
            )));
        }
        let (_, r_front) = self.front_face();
        if self.bore > 0.0 && self.bore / 2.0 >= r_front * 0.92 {
            out.push(Issue::error(format!("Bore too big: the front face is only {} across.", len(2.0 * r_front))));
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gear(teeth: u32, delta: f64) -> BevelGeom {
        let p = GearParams { teeth, module: 2.0, face_width: 12.0, bevel: Some(Bevel { pitch_angle: delta }), ..Default::default() };
        BevelGeom::new(&p).unwrap()
    }

    #[test]
    fn pitch_angles_of_a_right_angle_pair() {
        let (d1, d2) = pitch_angles(20, 40, PI / 2.0);
        assert!((d1 - 0.5f64.atan()).abs() < 1e-12);
        assert!((d1 + d2 - PI / 2.0).abs() < 1e-12);
        // Equal teeth at 90°: mitre gears, 45° each.
        let (m1, m2) = pitch_angles(30, 30, PI / 2.0);
        assert!((m1 - PI / 4.0).abs() < 1e-12 && (m2 - PI / 4.0).abs() < 1e-12);
    }

    #[test]
    fn spherical_involute_leaves_the_base_cone_square() {
        let g = gear(30, 45.0);
        // On the unit sphere the involute's polar angle grows like the
        // square of the roll and its azimuth like the cube: tiny near the
        // base, then growing.
        let a = g.involute(g.base + 1e-4);
        let b = g.involute(g.base + 4e-4);
        assert!(a > 0.0 && a < 1e-5, "{a}");
        assert!((b / a - 8.0).abs() < 0.2, "{}", b / a);
    }

    #[test]
    fn tooth_is_half_a_pitch_thick_at_the_pitch_cone() {
        let g = gear(24, 30.0);
        let i = g.profile.radii.iter().position(|&r| r >= g.delta).unwrap();
        let (r0, r1) = (g.profile.radii[i - 1], g.profile.radii[i]);
        let t = (g.delta - r0) / (r1 - r0);
        let h = g.profile.half[i - 1] + t * (g.profile.half[i] - g.profile.half[i - 1]);
        assert!((h - PI / 48.0).abs() < 2e-4, "{h}");
    }

    #[test]
    fn matches_the_virtual_spur_gear_near_the_pitch_cone() {
        // Tredgold: the large-end tooth is close to a spur tooth of z/cos δ
        // teeth. Compare tip thickness, arc on the back sphere vs spur.
        let g = gear(20, 26.565);
        let zv = g.virtual_teeth();
        let spur = GearParams { teeth: zv.round() as u32, module: 2.0, ..Default::default() };
        let rs = |p: &GearParams| p.thickness_at(p.tip_radius());
        let ours = 2.0 * g.profile.half.last().unwrap() * g.cone_distance * g.tip.sin();
        let theirs = rs(&spur);
        assert!((ours - theirs).abs() / theirs < 0.12, "{ours} vs {theirs}");
    }

    #[test]
    fn roll_angle_traces_the_same_involute() {
        let g = gear(24, 30.0);
        for gamma in [g.base + 1e-3, g.delta, g.tip] {
            let (gm, sweep) = g.roll(g.roll_at(gamma));
            assert!((gm - gamma).abs() < 1e-12);
            assert!((sweep - g.involute(gamma)).abs() < 1e-12, "{sweep} {}", g.involute(gamma));
        }
    }

    #[test]
    fn faces_meet_the_cones() {
        let g = gear(30, 45.0);
        let (zb, rb) = g.back_face();
        // Back face edge is where the root cone reaches the back cone.
        let p = BevelGeom::point(g.root, 0.0, g.back(g.root));
        assert!((p[2] - zb).abs() < 1e-9 && (p[0] - rb).abs() < 1e-9);
        // The back cone is square to the pitch cone at the pitch circle.
        assert!((g.back(g.delta) - g.cone_distance).abs() < 1e-9);
    }

    #[test]
    fn roots_are_milled_by_the_ball() {
        for (teeth, delta, ball) in [(20, 26.565, 1.0), (40, 63.435, 1.1), (30, 45.0, 1.2), (9, 20.0, 0.8)] {
            let p = GearParams {
                teeth,
                module: 2.0,
                face_width: 12.0,
                bevel: Some(Bevel { pitch_angle: delta }),
                ball,
                ..Default::default()
            };
            let g = BevelGeom::new(&p).unwrap();
            let largest = BevelGeom::largest_ball(&p).unwrap();
            assert!(ball <= largest, "{teeth} teeth: Ø{ball} doesn't fit (Ø{largest:.3} does)");
            // At the toe, where the teeth are smallest, on the sphere
            // through it.
            let d = g.cone_distance * g.front_ratio();
            let mid = PI / teeth as f64;
            let (radii, half) = (&g.profile.radii, &g.profile.half);
            let pts: Vec<[f64; 2]> = std::iter::once([0.0, 0.0])
                .chain(radii.iter().zip(half).map(|(&gm, &h)| [(gm - g.root) * d, (mid - h) * gm.sin() * d]))
                .collect();
            let fits = crate::gear::ball_fits(&pts, ball / 2.0);
            assert!(fits > 0.97, "{teeth} teeth, Ø{ball}: only {fits:.3} of the ball fits");
            // The flanks run on below the mate's tips (bar a thousandth).
            let tips = g.delta - p.module * (p.addendum - 1e-3) / g.cone_distance;
            assert!(g.flank_foot() <= tips + 1e-7, "{teeth} teeth: fillet rises {:.2e} past the mate's tips", g.flank_foot() - tips);
        }
        // Too big for the spaces: the biggest fillet that fits, flagged.
        let p = GearParams { teeth: 30, bevel: Some(Bevel { pitch_angle: 45.0 }), ball: 4.0, ..Default::default() };
        let g = BevelGeom::new(&p).unwrap();
        let largest = BevelGeom::largest_ball(&p).unwrap();
        assert!(largest < 4.0 && g.fillet.is_some_and(|f| (2.0 * f.radius * g.cone_distance * g.front_ratio() - largest).abs() < 1e-3));
    }
}

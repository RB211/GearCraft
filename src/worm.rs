//! Cylindrical worm drives: a worm and the wheel it turns, shafts square
//! and apart by the centre distance.
//!
//! The worm is an involute helicoid (ZI, DIN 3975): seen across its axis
//! each thread is an involute of its base cylinder, and the flanks are
//! straight along lines tangent to that cylinder, so a lathe tool set on
//! the tangent plane, or a flat grinding wheel, cuts them exactly.
//!
//! The wheel is either throated, cut as a hob cuts it: rolling the worm
//! (thread tips raised by the clearance, thickened by the backlash) through
//! a blank whose tip is a torus round the worm's axis, so every point the
//! threads pass through is cleared away and the teeth wrap round the worm
//! in line contact. Or it is straight: an involute helical gear at the
//! worm's lead angle, which meets an involute worm at a point.
//!
//! Both come out as `hypoid::Member`s with a flat pitch cone (a cylinder:
//! sections are planes across the axis, depth is radius), so the 3D view
//! and every export build them as they build hypoid gears.
//!
//! Frames. The worm's axis is +z, its threads running from z = 0 to its
//! length L, the middle of a thread facing +x at z = L/2. The wheel's axis
//! is +z, its faces at z = 0 and its face width b; the worm lies at
//! azimuth π on the wheel, its axis along +y through (−a, 0, b/2).

use crate::gear::{self, GearParams, ToothStandard, inv};
use crate::hypoid::{self, Blank, Member, PROFILE_POINTS};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

/// Which half of a worm drive a part is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Role {
    Worm,
    Wheel,
}

/// Starts a worm may have.
pub const STARTS: std::ops::RangeInclusive<u32> = 1..=6;

/// What a worm drive is made from.
#[derive(Clone, Debug, PartialEq)]
pub struct Spec {
    pub starts: u32,
    pub teeth: u32,
    /// The worm's axial module (the wheel's transverse module), mm.
    pub module: f64,
    /// Normal pressure angle, degrees.
    pub pressure_angle: f64,
    /// Addendum, dedendum and the wheel's root fillet, × module.
    pub addendum: f64,
    pub dedendum: f64,
    pub root_fillet: f64,
    /// The worm's pitch diameter, mm.
    pub diameter: f64,
    /// Threaded length of the worm, mm.
    pub length: f64,
    /// Face width of the wheel, mm.
    pub face: f64,
    pub throated: bool,
    pub left_hand: bool,
    /// Thinning of the wheel's teeth, circular mm on its pitch circle.
    pub backlash: f64,
    pub worm_bore: f64,
    pub wheel_bore: f64,
}

/// The solved drive.
#[derive(Clone, Debug, PartialEq)]
pub struct Design {
    pub spec: Spec,
    /// Axial pitch and lead, mm; lead angle at the pitch diameter.
    pub axial_pitch: f64,
    pub lead: f64,
    pub lead_angle: f64,
    /// Worm pitch, tip, root and base radii, and its transverse pressure
    /// angle (radians).
    pub worm_radius: f64,
    pub worm_tip: f64,
    pub worm_root: f64,
    pub worm_base: f64,
    pub worm_alpha: f64,
    /// Wheel pitch radius, tip radius at the middle of the face (the
    /// throat), outside (rim) radius, and root radius at the middle.
    pub wheel_radius: f64,
    pub wheel_throat: f64,
    pub wheel_rim: f64,
    pub wheel_root: f64,
    /// Transverse pressure angle of the straight wheel, radians.
    pub wheel_alpha: f64,
    pub center: f64,
    pub clearance: f64,
    /// θ_wheel = sense · (starts / teeth) · θ_worm.
    pub sense: f64,
}

/// A pitch diameter that suits `module` with the usual diameter quotient
/// (pitch diameter over module) of 10.
pub fn default_diameter(module: f64) -> f64 {
    10.0 * module
}

impl Spec {
    pub fn design(&self) -> Result<Design, String> {
        let m = self.module;
        let (z1, z2) = (self.starts, self.teeth);
        if !(m > 0.0) || !STARTS.contains(&z1) || z2 < 10 {
            return Err("A worm drive needs a module, 1 to 6 starts and at least 10 wheel teeth.".into());
        }
        let (ha, hf) = (self.addendum * m, self.dedendum * m);
        let r1 = self.diameter / 2.0;
        if r1 - hf < 0.5 * m {
            return Err("The worm is too thin for its threads: make its pitch diameter bigger.".into());
        }
        let axial_pitch = PI * m;
        let lead = z1 as f64 * axial_pitch;
        let lead_angle = (lead / (TAU * r1)).atan();
        if lead_angle > 45f64.to_radians() {
            return Err("The lead angle is over 45°: give the worm fewer starts or a bigger diameter.".into());
        }
        let an = self.pressure_angle.to_radians();
        if !(an > 5f64.to_radians() && an < 35f64.to_radians()) {
            return Err("The pressure angle must be between 5° and 35°.".into());
        }
        let worm_alpha = (an.tan() / lead_angle.sin()).atan();
        let r2 = m * z2 as f64 / 2.0;
        let worm_tip = r1 + ha;
        let wheel_throat = r2 + ha;
        let d = Design {
            spec: self.clone(),
            axial_pitch,
            lead,
            lead_angle,
            worm_radius: r1,
            worm_tip,
            worm_root: r1 - hf,
            worm_base: r1 * worm_alpha.cos(),
            worm_alpha,
            wheel_radius: r2,
            wheel_throat,
            wheel_rim: if self.throated { wheel_throat + 0.5 * m } else { wheel_throat },
            wheel_root: r2 - hf,
            wheel_alpha: (an.tan() / lead_angle.cos()).atan(),
            center: r1 + r2,
            clearance: hf - ha,
            sense: if self.left_hand { -1.0 } else { 1.0 },
        };
        if d.worm_half(worm_tip) < 0.02 * m / worm_tip {
            return Err("The worm's threads come to a point: lower the pressure angle or the addendum.".into());
        }
        if self.length <= 0.0 || self.face <= 0.0 {
            return Err("The worm's length and the wheel's face width must be more than 0.".into());
        }
        Ok(d)
    }
}

/// Resample a flank given seam to tip as (radius, azimuth) points, evenly
/// along its length, to the profile points every member uses.
fn resample(pts: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut acc = vec![0.0];
    for w in pts.windows(2) {
        let r = (w[0][0] + w[1][0]) / 2.0;
        acc.push(acc.last().unwrap() + (w[1][0] - w[0][0]).hypot(r * (w[1][1] - w[0][1])));
    }
    let total = *acc.last().unwrap();
    (0..=PROFILE_POINTS)
        .map(|i| {
            let target = total * i as f64 / PROFILE_POINTS as f64;
            let k = acc.partition_point(|&a| a < target).clamp(1, acc.len() - 1);
            let w = ((target - acc[k - 1]) / (acc[k] - acc[k - 1]).max(1e-300)).clamp(0.0, 1.0);
            [pts[k - 1][0] + w * (pts[k][0] - pts[k - 1][0]), pts[k - 1][1] + w * (pts[k][1] - pts[k - 1][1])]
        })
        .collect()
}

/// A tooth's right flank, seam to tip, from its half-angle at each radius:
/// along the floor from the middle of the space, then up the flank.
fn right_flank(pitch: f64, radii: &[f64], half: impl Fn(usize) -> f64) -> Vec<[f64; 2]> {
    let r0 = radii[0];
    let floor = 8;
    let mut pts: Vec<[f64; 2]> =
        (0..floor).map(|i| [r0, -pitch / 2.0 + (pitch / 2.0 - half(0)) * i as f64 / floor as f64]).collect();
    pts.extend(radii.iter().enumerate().map(|(i, &r)| [r, -half(i)]));
    resample(&pts)
}

/// A cylindrical member's blank: its tip (or rim) radius over a length.
fn cylinder(tip: f64, length: f64, throat: Option<[f64; 3]>) -> Blank {
    Blank { pitch: 0.0, face_point: [tip, 0.0], face_angle: 0.0, inner: 0.0, outer: length, throat }
}

/// The hub circles' radius: clear of the bore, under the roots.
fn hub_radius(bore: f64, root: f64, depth: f64, what: &str) -> Result<f64, String> {
    let r = if bore > 0.0 { bore / 2.0 + 0.25 * depth } else { 0.0 }.max(0.5 * root);
    if r > root - 0.05 * depth {
        return Err(format!("The bore is too big for the {what}: make it smaller."));
    }
    Ok(r)
}

/// Sections per turn of a worm's thread, so the flanks between them
/// stay within a few microns of the helix.
const SECTIONS_PER_TURN: f64 = 24.0;

impl Design {
    fn whole_depth(&self) -> f64 {
        (self.spec.addendum + self.spec.dedendum) * self.spec.module
    }

    /// Half the angle a worm thread spans at radius r, across the axis:
    /// an involute above the base circle, radial below it.
    pub fn worm_half(&self, r: f64) -> f64 {
        let a = (self.worm_base / r.max(self.worm_base)).acos();
        PI / (2.0 * self.spec.starts as f64) + inv(self.worm_alpha) - inv(a)
    }

    /// How far inside a worm thread a point is (worm frame, z measured
    /// from the middle of the worm), positive in the worm. `tip` raises
    /// the thread tips and `thick` widens the threads (radians a side);
    /// the threads run on past the worm's ends, as on a hob.
    fn worm_inside(&self, p: [f64; 3], tip: f64, thick: f64) -> f64 {
        let r = p[0].hypot(p[1]);
        let pitch = TAU / self.spec.starts as f64;
        let turn = self.sense * TAU * p[2] / self.lead;
        let d = (p[1].atan2(p[0]) - turn + pitch / 2.0).rem_euclid(pitch) - pitch / 2.0;
        // Azimuth to millimetres, roughly normal to the thread.
        let scale = r * self.lead / (TAU * r).hypot(self.lead);
        let thread = (self.worm_half(r) + thick - d.abs()) * scale;
        thread.min(self.worm_tip + tip - r).max(self.worm_root - r)
    }

    /// A wheel point (wheel frame, wheel turned by ψ) in the worm's frame
    /// (z from the worm's middle), the worm turned by θ.
    fn wheel_to_worm(&self, x: [f64; 3], psi: f64, theta: f64) -> [f64; 3] {
        let (s, c) = psi.sin_cos();
        let y = [x[0] * c - x[1] * s, x[0] * s + x[1] * c, x[2]];
        let q = [self.center + y[0], self.spec.face / 2.0 - y[2], y[1]];
        let (s, c) = theta.sin_cos();
        [q[0] * c + q[1] * s, -q[0] * s + q[1] * c, q[2]]
    }

    /// Wheel turns per worm turn, signed.
    pub fn ratio(&self) -> f64 {
        self.sense * self.spec.starts as f64 / self.spec.teeth as f64
    }

    /// The worm: one cross-section of its threads, turned along the helix.
    pub fn worm(&self) -> Result<Member, String> {
        let z1 = self.spec.starts;
        let pitch = TAU / z1 as f64;
        let n = 60;
        let radii: Vec<f64> =
            (0..=n).map(|i| self.worm_root + (self.worm_tip - self.worm_root) * i as f64 / n as f64).collect();
        let right = right_flank(pitch, &radii, |i| self.worm_half(radii[i]));
        let length = self.spec.length;
        let turns = length / self.lead;
        let sections = ((turns * SECTIONS_PER_TURN).ceil() as usize).max(10) + 1;
        let stations: Vec<f64> = (0..sections).map(|k| length * k as f64 / (sections - 1) as f64).collect();
        let turn = |z: f64| self.sense * TAU * (z - length / 2.0) / self.lead;
        let hub = hub_radius(self.spec.worm_bore, self.worm_root, self.whole_depth(), "worm")?;
        Ok(Member {
            teeth: z1,
            blank: cylinder(self.worm_tip, length, None),
            t_root: vec![self.worm_root; sections],
            t_tip: vec![self.worm_tip; sections],
            right: stations.iter().map(|&z| right.iter().map(|&[r, a]| [r, a + turn(z)]).collect()).collect(),
            left: stations.iter().map(|&z| right.iter().map(|&[r, a]| [r, -a + turn(z)]).collect()).collect(),
            stations,
            hub: [hub; 2],
            bore: self.spec.worm_bore,
        })
    }

    /// The wheel, throated or straight.
    pub fn wheel(&self) -> Result<Member, String> {
        let mut m = if self.spec.throated { self.throated_wheel()? } else { self.straight_wheel()? };
        let root = m.t_root.iter().copied().fold(f64::MAX, f64::min);
        let hub = hub_radius(self.spec.wheel_bore, root, self.whole_depth(), "wheel")?;
        m.hub = [hub; 2];
        Ok(m)
    }

    /// How far a wheel point (section s, radius t, azimuth φ) lies in the
    /// space the worm cuts: positive where some position of the worm,
    /// rolling with the wheel as a hob, has material (thread tips raised
    /// by the clearance, threads thickened by the backlash). The max of
    /// its inside measure over the wheel turns that bring the point past
    /// the worm.
    fn hobbed(&self, s: f64, t: f64, phi: f64) -> f64 {
        let k = self.ratio();
        let reach = self.worm_tip + self.clearance;
        let thick = PI * self.spec.backlash / self.lead;
        let x = [t * phi.cos(), t * phi.sin(), s];
        let w = s - self.spec.face / 2.0;
        let near = reach * reach - w * w;
        // How far the point can turn from facing the worm and still be
        // within the threads' reach.
        let open = ((self.center - near.max(0.0).sqrt()) / t).max(-1.0);
        if near <= 0.0 || open >= 1.0 {
            // Out of reach even facing the worm: by how much.
            return reach - (self.center - t).hypot(w);
        }
        let spread = open.acos() * 1.05 + 1e-3;
        let at = |theta: f64| self.worm_inside(self.wheel_to_worm(x, k * theta, theta), self.clearance, thick);
        let centre = ((PI - phi + PI).rem_euclid(TAU) - PI) / k;
        let half = spread / k.abs();
        // A dozen samples each time a thread passes.
        let n = ((24.0 * half / (TAU / self.spec.starts as f64)).ceil() as usize).clamp(24, 2000);
        let step = 2.0 * half / n as f64;
        let mut samples: Vec<(f64, f64)> = (0..=n).map(|i| centre - half + step * i as f64).map(|th| (th, at(th))).collect();
        samples.sort_by(|a, b| b.1.total_cmp(&a.1));
        samples.iter().take(3).map(|&(th, _)| hypoid::golden(&at, th - step, th + step, 30).1).fold(f64::MIN, f64::max)
    }

    /// A throated wheel, hobbed by the worm.
    fn throated_wheel(&self) -> Result<Member, String> {
        let b = self.spec.face;
        let blank = cylinder(self.wheel_rim, b, Some([self.center, self.center - self.wheel_throat, b / 2.0]));
        let gap = |s: f64, t: f64, phi: f64| self.hobbed(s, t, phi);
        hypoid::build_member(self.spec.teeth, blank, self.spec.wheel_bore, [0.0; 2], 15, &gap)
            .map_err(|e| format!("The wheel could not be cut ({e}): check the face width and worm diameter."))
    }

    /// A straight wheel: an involute helical gear at the lead angle, its
    /// cross-section cut by the transverse rack.
    fn straight_wheel(&self) -> Result<Member, String> {
        let sp = &self.spec;
        let p = GearParams {
            module: sp.module,
            teeth: sp.teeth,
            pressure_angle: self.wheel_alpha.to_degrees(),
            addendum: sp.addendum,
            dedendum: sp.dedendum,
            root_fillet: sp.root_fillet,
            standard: ToothStandard::Custom,
            backlash: sp.backlash,
            profile_shift: 0.0,
            ..GearParams::default()
        };
        let prof = gear::tooth_profile(&p).ok_or("These settings do not make a wheel.")?;
        let pitch = TAU / sp.teeth as f64;
        let right = right_flank(pitch, &prof.radii, |i| prof.half[i]);
        let b = sp.face;
        let sections = 11;
        let stations: Vec<f64> = (0..sections).map(|k| b * k as f64 / (sections - 1) as f64).collect();
        // A space faces the worm's thread at the middle of the face, and
        // the teeth slope with the threads there.
        let turn = |s: f64| PI + pitch / 2.0 + self.sense * self.lead_angle.tan() * (s - b / 2.0) / self.wheel_radius;
        let (rf, ra) = (prof.radii[0], *prof.radii.last().unwrap());
        Ok(Member {
            teeth: sp.teeth,
            blank: cylinder(ra, b, None),
            t_root: vec![rf; sections],
            t_tip: vec![ra; sections],
            right: stations.iter().map(|&s| right.iter().map(|&[r, a]| [r, a + turn(s)]).collect()).collect(),
            left: stations.iter().map(|&s| right.iter().map(|&[r, a]| [r, -a + turn(s)]).collect()).collect(),
            stations,
            hub: [0.0; 2],
            bore: sp.wheel_bore,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    pub fn spec(starts: u32, teeth: u32, throated: bool) -> Spec {
        Spec {
            starts,
            teeth,
            module: 2.0,
            pressure_angle: 20.0,
            addendum: 1.0,
            dedendum: 1.25,
            root_fillet: 0.38,
            diameter: 20.0,
            length: 30.0,
            face: 16.0,
            throated,
            left_hand: false,
            backlash: 0.0,
            worm_bore: 0.0,
            wheel_bore: 10.0,
        }
    }

    /// Over a turn of the worm, the deepest any wheel flank point goes
    /// into the worm (its real threads: no clearance or backlash).
    fn deepest(d: &Design, wheel: &Member) -> f64 {
        let k = d.ratio();
        let mut deepest = f64::MIN;
        for step in 0..24 {
            let theta = TAU * step as f64 / 24.0;
            for tooth in 0..wheel.teeth as usize {
                for left in [false, true] {
                    for sec in wheel.flank(left, tooth) {
                        for p in &sec[PROFILE_POINTS / 3..] {
                            let v = d.worm_inside(d.wheel_to_worm(*p, k * theta, theta), 0.0, 0.0);
                            deepest = deepest.max(v);
                        }
                    }
                }
            }
        }
        deepest
    }

    #[test]
    fn straight_wheels_run_with_their_worms() {
        for (starts, teeth, left) in [(1, 30, false), (2, 40, true), (4, 24, false)] {
            let d = Spec { left_hand: left, ..spec(starts, teeth, false) }.design().unwrap();
            let wheel = d.wheel().unwrap();
            let deepest = deepest(&d, &wheel);
            assert!(deepest < 0.01, "{starts}/{teeth}: the wheel cuts {deepest:.4} mm into the worm");
            assert!(deepest > -0.05, "{starts}/{teeth}: the wheel never comes near the worm ({deepest:.4} mm)");
            d.worm().unwrap();
        }
    }

    #[test]
    fn throated_wheels_are_hobbed_by_their_worms() {
        for (starts, teeth, left) in [(1, 30, false), (3, 31, true), (6, 24, false)] {
            let d = Spec { left_hand: left, ..spec(starts, teeth, true) }.design().unwrap();
            let wheel = d.wheel().unwrap();
            let deepest = deepest(&d, &wheel);
            assert!(deepest < 0.01, "{starts}/{teeth}: the wheel cuts {deepest:.4} mm into the worm");
            assert!(deepest > -0.02, "{starts}/{teeth}: the wheel never touches the worm ({deepest:.4} mm)");
        }
    }

    #[test]
    fn solved_drives_sit_in_mesh() {
        // The wheel's flanks, placed as the train solves and exports them,
        // run against the true worm without cutting into it.
        use crate::train::{Link, PitchUnit, Placement, Project};
        let u = PitchUnit::Module;
        for (starts, throated, left) in [(1, true, false), (2, false, true)] {
            let mut p = Project::empty();
            let w = p.add_root(GearParams { teeth: starts, ..u.starter_worm_drive(Role::Worm) }, 10.0, -5.0);
            p.add_child(w, u.starter_worm_drive(Role::Wheel), Link::Worm);
            p.gears[0].axial = 3.0;
            if let Placement::Worm { angle, throated: t, left_hand, .. } = &mut p.gears[1].placement {
                (*angle, *t, *left_hand) = (35.0, throated, left);
            }
            p.normalize();
            let d = crate::train::worm_spec(&p.gears[0].params, &p.gears[1].params, &p.gears[1].placement)
                .unwrap()
                .design()
                .unwrap();
            let mut deepest = f64::MIN;
            for step in 0..8 {
                let drive = 0.37 + step as f64 * 0.7;
                let parts = crate::export::hypoids(&p, drive, None, |_| true, |_| None);
                let (wm, wh) = (&parts[0], &parts[1]);
                for tooth in 0..wh.member.teeth as usize {
                    for left in [false, true] {
                        for row in wh.member.flank(left, tooth) {
                            for q in &row[PROFILE_POINTS / 3..] {
                                let (s, c) = wh.rot.sin_cos();
                                let x = wh.frame.apply([q[0] * c - q[1] * s, q[0] * s + q[1] * c, q[2]]);
                                let f = wm.frame;
                                let rel: Vec<f64> = (0..3).map(|k| x[k] - f.origin[k]).collect();
                                let local = [f.x, f.y, f.z].map(|a| a[0] * rel[0] + a[1] * rel[1] + a[2] * rel[2]);
                                let (s, c) = (-wm.rot).sin_cos();
                                let y = [local[0] * c - local[1] * s, local[0] * s + local[1] * c, local[2] - d.spec.length / 2.0];
                                deepest = deepest.max(d.worm_inside(y, 0.0, 0.0));
                            }
                        }
                    }
                }
            }
            assert!(deepest < 0.01 && deepest > -0.05, "{starts} starts: deepest {deepest}");
        }
    }

    #[test]
    fn worm_threads_follow_the_lead() {
        let d = spec(2, 30, true).design().unwrap();
        let w = d.worm().unwrap();
        let (k0, k1) = (0, w.stations.len() - 1);
        let turn = (w.right[k1][10][1] - w.right[k0][10][1]) / TAU;
        assert!((turn - d.spec.length / d.lead).abs() < 1e-9);
        // The thread's middle at mid-length faces the wheel (+x).
        let mid = w.stations.len() / 2;
        let a = (w.right[mid].last().unwrap()[1] + w.left[mid].last().unwrap()[1]) / 2.0;
        assert!(a.abs() < 1e-9, "{a}");
        // A thread point measures as on the surface of the worm.
        let [r, phi] = w.right[mid][PROFILE_POINTS / 2];
        let z = w.stations[mid] - d.spec.length / 2.0;
        assert!(d.worm_inside([r * phi.cos(), r * phi.sin(), z], 0.0, 0.0).abs() < 1e-5);
    }
}



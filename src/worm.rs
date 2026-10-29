//! Cylindrical worm drives: a worm and the wheel it turns, shafts square
//! and apart by the centre distance.
//!
//! The worm is an involute helicoid (ZI, DIN 3975): seen across its axis
//! each thread is an involute of its base cylinder, and the flanks are
//! straight along lines tangent to that cylinder, so a lathe tool set on
//! the tangent plane, or a flat grinding wheel, cuts them exactly.
//!
//! The wheel is either throated: its tooth spaces are every point the
//! running worm's threads (thickened by the backlash) pass through, their
//! flanks run on down to a root that is a torus round the worm's axis, in
//! a blank whose tip is another, so the teeth wrap round the worm in line
//! contact. Or it is straight: an involute helical gear at the worm's lead
//! angle, which meets an involute worm at a point.
//!
//! Both parts are milled from metal on a 4-axis machine, each turning on
//! its own axis under a ball end mill, not hobbed. So every root is a
//! surface designed to be cut, its corners rounded to the ball's radius,
//! and a drive whose spaces the ball can't get to the bottom of, or whose
//! fillets would stand in the way of the other part's tips, is refused.
//!
//! A globoid (double-enveloping) drive's worm is an hourglass that wraps
//! round the wheel, so several teeth bear at once. It is the Hindley form
//! (AGMA 6135): cut by a straight-sided tool that swings round the wheel's
//! axis as the worm turns, so in the worm's axial plane every thread flank
//! is a straight side of a wheel tooth, each touching the base circle
//! round the wheel's axis, and the threads' tips and roots are arcs round
//! that axis. Its wheel is throated, as above. The
//! threads split the pitch 45 : 55 with the wheel's teeth, the bronze
//! getting the larger share.
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
    /// Addendum and dedendum, × module, and the radius of the ball end
    /// mill that cuts the roots, × module: every root fillet's.
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
    /// An hourglass worm and throated wheel (double-enveloping).
    pub globoid: bool,
    pub left_hand: bool,
    /// Thinning of the wheel's teeth, circular mm on its pitch circle.
    pub backlash: f64,
    pub worm_bore: f64,
    pub wheel_bore: f64,
    /// Nozzle Ø, mm, of the 3D printer the drive is made on: every thread
    /// and tooth surface is eroded by a quarter of it (the bores above are
    /// already widened by half of it); 0 when it is not printed.
    pub nozzle: f64,
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
    /// Radius round the worm's axis of a throated wheel's root, a torus.
    pub root_form: f64,
    /// The ball end mill's radius.
    pub ball: f64,
    /// θ_wheel = sense · (starts / teeth) · θ_worm.
    pub sense: f64,
    /// A globoid worm's form.
    pub hourglass: Option<Hourglass>,
}

/// A globoid worm's form, in its axial plane: every thread flank is a
/// straight side of a wheel tooth, touching the base circle round the
/// wheel's axis, and the thread tips and roots are arcs round that axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hourglass {
    /// Base circle radius (AGMA 6135's d_b / 2).
    pub base: f64,
    /// Angle round the wheel's axis from the middle of a wheel tooth to
    /// where its flank touches the base circle.
    touch: f64,
    /// Radii round the wheel's axis of the thread tips and roots.
    tip_arc: f64,
    root_arc: f64,
}

/// A globoid wheel tooth's share of the pitch (AGMA 6135 4.20).
const WHEEL_SHARE: f64 = 0.55;

/// The ball end mill's centre where it rounds the corners at the bottom
/// of a wheel tooth space, on one surface a fixed angle round the worm's
/// axis (see `Design::corners`).
#[derive(Clone, Copy, Debug)]
struct Corner {
    /// The space's middle, turned on the wheel.
    mid: f64,
    /// The centre's distance from the middle (mm, square to the thread),
    /// and the angle it turns through from the root to the flank, at the
    /// right and left corners.
    centre: [f64; 2],
    turn: [f64; 2],
}

/// Corners across the face, every `step` round the worm's axis.
struct Corners {
    first: f64,
    step: f64,
    rows: Vec<Corner>,
}

impl Corners {
    fn at(&self, psi: f64) -> Corner {
        let x = ((psi - self.first) / self.step).clamp(0.0, (self.rows.len() - 1) as f64);
        let i = (x.floor() as usize).min(self.rows.len() - 2);
        let (a, b, f) = (self.rows[i], self.rows[i + 1], x - i as f64);
        let mix = |p: f64, q: f64| if p == f64::MAX || q == f64::MAX { f64::MAX } else { p + (q - p) * f };
        Corner {
            mid: mix(a.mid, b.mid),
            centre: [mix(a.centre[0], b.centre[0]), mix(a.centre[1], b.centre[1])],
            turn: [mix(a.turn[0], b.turn[0]), mix(a.turn[1], b.turn[1])],
        }
    }
}

/// A pitch diameter that suits `module` with the usual diameter quotient
/// (pitch diameter over module) of 10.
pub fn default_diameter(module: f64) -> f64 {
    10.0 * module
}

/// AGMA 6135's first guess at a globoid worm's pitch diameter for a
/// wheel of `teeth` at `module`: d ≈ a^0.875 / 1.47 (mm), a being the
/// centre distance, rounded to 0.1 mm.
pub fn globoid_diameter(module: f64, teeth: u32) -> f64 {
    let mut d = default_diameter(module);
    for _ in 0..30 {
        d = ((d + module * teeth as f64) / 2.0).powf(0.875) / 1.47;
    }
    (d * 10.0).round() / 10.0
}

/// AGMA 6135's fewest wheel teeth for a globoid drive at centre distance
/// `center` (table 2), or None below its smallest, 38 mm.
pub fn globoid_least_teeth(center: f64) -> Option<u32> {
    const TABLE: [(f64, u32); 7] = [(38.0, 24), (65.0, 24), (90.0, 30), (150.0, 40), (330.0, 50), (480.0, 50), (585.0, 60)];
    TABLE.iter().rev().find(|&&(from, _)| center >= from).map(|&(_, z)| z)
}

/// DIN 3975's least threaded length for a worm, mm.
pub fn least_length(starts: u32, teeth: u32, module: f64) -> f64 {
    let z2 = teeth as f64;
    (if starts <= 2 { 11.0 + 0.06 * z2 } else { 12.5 + 0.09 * z2 }) * module
}

/// Running friction between the flanks for some usual pairs of materials.
/// Rough guides: friction falls as the sliding speeds up.
pub const MATERIALS: [(&str, f64); 5] = [
    ("Hardened steel on bronze, oiled", 0.05),
    ("Steel on cast iron, oiled", 0.08),
    ("Steel on acetal or nylon", 0.15),
    ("Steel on bronze, dry", 0.2),
    ("Printed plastic (PLA, PETG), dry", 0.25),
];

pub const DEFAULT_FRICTION: f64 = 0.05;

/// Friction at rest over friction running: breaking away takes more.
const BREAKAWAY: f64 = 2.0;

/// The thinnest worm, pitch diameter over module, `back_drivable` makes.
const LEAST_QUOTIENT: f64 = 8.0;

/// The friction angle across the threads for friction `mu` at normal
/// pressure angle `pressure_angle` (degrees).
pub fn friction_angle(mu: f64, pressure_angle: f64) -> f64 {
    (mu / pressure_angle.to_radians().cos()).atan()
}

/// The least lead angle `back_drivable` aims for: past the breakaway
/// friction angle by 5°, and not under 15°.
pub fn back_drive_lead(mu: f64, pressure_angle: f64) -> f64 {
    (friction_angle(BREAKAWAY * mu, pressure_angle) + 5f64.to_radians()).max(15f64.to_radians())
}

/// Whether the wheel can turn the worm.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BackDrive {
    /// Friction holds the worm even with the drive running.
    Locked,
    /// The wheel keeps a turning worm going, but friction holds one at
    /// rest (though vibration may let it creep).
    Running,
    /// The wheel can start the worm from rest.
    Free,
}

impl Design {
    /// Efficiency with the worm driving, at running friction `mu`.
    pub fn efficiency(&self, mu: f64) -> f64 {
        let rho = friction_angle(mu, self.spec.pressure_angle);
        self.lead_angle.tan() / (self.lead_angle + rho).tan()
    }

    /// Efficiency with the wheel driving the worm; 0 when it can't.
    pub fn back_efficiency(&self, mu: f64) -> f64 {
        let rho = friction_angle(mu, self.spec.pressure_angle);
        ((self.lead_angle - rho).tan() / self.lead_angle.tan()).max(0.0)
    }

    pub fn back_drive(&self, mu: f64) -> BackDrive {
        if self.lead_angle <= friction_angle(mu, self.spec.pressure_angle) {
            BackDrive::Locked
        } else if self.lead_angle <= friction_angle(BREAKAWAY * mu, self.spec.pressure_angle) {
            BackDrive::Running
        } else {
            BackDrive::Free
        }
    }
}

impl Spec {
    pub fn design(&self) -> Result<Design, String> {
        let d = self.solve()?;
        if let Some(trouble) = d.ball_trouble(d.ball) {
            let largest = self.largest_ball().unwrap_or(0.0);
            return Err(format!("{trouble}: use a ball of Ø{:.2} mm or smaller, or a bigger module.", (largest * 100.0).floor() / 100.0));
        }
        Ok(d)
    }

    /// The drive's dimensions, whether or not the ball can cut it.
    pub fn solve(&self) -> Result<Design, String> {
        let m = self.module;
        let (z1, z2) = (self.starts, self.teeth);
        if !(m > 0.0) || !STARTS.contains(&z1) || z2 < 10 {
            return Err("A worm drive needs a module, 1 to 6 starts and at least 10 wheel teeth.".into());
        }
        // The roots sit at least the ball's radius under the other part's
        // tips, so the ball's fillets stay clear of them.
        let ball = self.root_fillet * m;
        let ha = self.addendum * m;
        let hf = ha + (self.dedendum * m - ha).max(ball);
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
        let throated = self.throated || self.globoid;
        let hourglass = self.globoid.then(|| {
            // The wheel tooth's flanks lean at the axial pressure angle to
            // its middle line, so they touch a circle that far round.
            let axial = (an.tan() / lead_angle.cos()).atan();
            let half = WHEEL_SHARE * PI / z2 as f64;
            let base = r2 * (axial + half).sin();
            Hourglass { base, touch: PI / 2.0 - axial, tip_arc: r2 - ha, root_arc: r2 + hf }
        });
        let mut d = Design {
            spec: Spec { throated, ..self.clone() },
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
            wheel_rim: if throated { wheel_throat + 0.5 * m } else { wheel_throat },
            wheel_root: r2 - hf,
            wheel_alpha: (an.tan() / lead_angle.cos()).atan(),
            center: r1 + r2,
            clearance: hf - ha,
            sense: if self.left_hand { -1.0 } else { 1.0 },
            hourglass,
            root_form: 0.0,
            ball,
        };
        if d.worm_half(worm_tip) < 0.02 * m / worm_tip {
            return Err("The worm's threads come to a point: lower the pressure angle or the addendum.".into());
        }
        if self.length <= 0.0 || self.face <= 0.0 {
            return Err("The worm's length and the wheel's face width must be more than 0.".into());
        }
        d.root_form = d.wheel_root_form();
        if let Some(h) = &d.hourglass {
            if d.globoid_half(h, h.root_arc) >= PI / z2 as f64 {
                return Err("The wheel's teeth come to a point: lower the pressure angle or the dedendum.".into());
            }
            // Past the base circle a thread flank would lean out over its
            // root: no tool could cut it, and the worm couldn't be fitted.
            if self.length > 2.0 * h.base {
                return Err(format!(
                    "The worm is {:.2} mm long, past its base circle Ø of {:.2} mm, so its end threads would \
                     overhang their roots: shorten it to {:.2} mm, AGMA 6135's length.",
                    self.length,
                    2.0 * h.base,
                    d.effective_length().unwrap_or(0.0)
                ));
            }
        }
        Ok(d)
    }

    /// The biggest ball end mill (Ø, mm) that can cut this drive, or None
    /// if it fails otherwise.
    pub fn largest_ball(&self) -> Option<f64> {
        let [worm, wheel] = self.solve().ok()?.narrowest_spaces();
        Some(2.0 * worm.min(wheel).max(0.0))
    }

    /// A globoid drive's longest worm (its base circle Ø) and AGMA 6135's
    /// length, whatever its worm's length now; None if it fails otherwise.
    pub fn globoid_lengths(&self) -> Option<(f64, f64)> {
        let d = Spec { length: 1.0, ..self.clone() }.solve().ok()?;
        Some((2.0 * d.hourglass?.base, d.effective_length()?))
    }

    /// This drive made so the wheel can start the worm from rest at
    /// friction `mu`: the fewest starts (no fewer than now) that reach
    /// `back_drive_lead` on a worm no thicker than now and not too thin,
    /// the wheel's teeth multiplied to keep the ratio unless
    /// `hold_teeth`, and the worm lengthened if DIN 3975 asks. A drive
    /// that already back-drives comes back as it is.
    pub fn back_drivable(&self, mu: f64, hold_teeth: bool) -> Result<Spec, String> {
        let m = self.module;
        if self.solve().is_ok_and(|d| d.back_drive(mu) == BackDrive::Free) {
            return Ok(self.clone());
        }
        let target = back_drive_lead(mu, self.pressure_angle);
        let ratio = self.teeth as f64 / self.starts as f64;
        let thinnest = LEAST_QUOTIENT.min(self.diameter / m) * m;
        for z1 in self.starts.max(*STARTS.start())..=*STARTS.end() {
            // Rounded down to 0.01 mm, which only steepens the lead.
            let diameter = (self.diameter.min(z1 as f64 * m / target.tan()) * 100.0).floor() / 100.0;
            if diameter < thinnest - 0.01 {
                continue;
            }
            let teeth = if hold_teeth { self.teeth } else { (ratio * z1 as f64).round().clamp(20.0, 400.0) as u32 };
            // A globoid worm's length is bound by its base circle instead.
            let least = if self.globoid { 0.0 } else { (least_length(z1, teeth, m) * 100.0).ceil() / 100.0 };
            let length = self.length.max(least);
            let sp = Spec { starts: z1, teeth, diameter, length, ..self.clone() };
            if sp.solve().is_ok_and(|d| d.back_drive(mu) == BackDrive::Free) {
                return Ok(sp);
            }
        }
        Err(format!(
            "Even {} starts can't reach the {:.1}° lead angle this friction needs without making the worm too thin: \
             choose slicker materials or oil them.",
            STARTS.end(),
            target.to_degrees()
        ))
    }
}

/// Profile points along a flank's foot, the floor and the ball's round
/// up to the flank; the rest are up the flank.
const FOOT_POINTS: usize = PROFILE_POINTS / 4;

/// Resample a flank given seam to tip as (radius, azimuth) points, its
/// foot (up to `pts[foot]`) and the flank above each evenly along its
/// length, to the profile points every member uses. Split so, a profile
/// point stays on the same part of the flank from section to section,
/// though the thread's height changes along the worm (its chamfered
/// ends), and the surfaces through the sections stay on the tooth.
fn resample(pts: &[[f64; 2]], foot: usize) -> Vec<[f64; 2]> {
    let even = |pts: &[[f64; 2]], n: usize| -> Vec<[f64; 2]> {
        let mut acc = vec![0.0];
        for w in pts.windows(2) {
            let r = (w[0][0] + w[1][0]) / 2.0;
            acc.push(acc.last().unwrap() + (w[1][0] - w[0][0]).hypot(r * (w[1][1] - w[0][1])));
        }
        let total = *acc.last().unwrap();
        (0..=n)
            .map(|i| {
                let target = total * i as f64 / n as f64;
                let k = acc.partition_point(|&a| a < target).clamp(1, acc.len() - 1);
                let w = ((target - acc[k - 1]) / (acc[k] - acc[k - 1]).max(1e-300)).clamp(0.0, 1.0);
                [pts[k - 1][0] + w * (pts[k][0] - pts[k - 1][0]), pts[k - 1][1] + w * (pts[k][1] - pts[k - 1][1])]
            })
            .collect()
    };
    let mut out = even(&pts[..=foot], FOOT_POINTS);
    out.extend(&even(&pts[foot..], PROFILE_POINTS - FOOT_POINTS)[1..]);
    out
}

/// A tooth's right flank, seam to tip, from its half-angle at each radius:
/// along the floor from the middle of the space, then up the flank.
fn right_flank(pitch: f64, radii: &[f64], half: impl Fn(usize) -> f64) -> Vec<[f64; 2]> {
    let r0 = radii[0];
    let floor = 8;
    let mut pts: Vec<[f64; 2]> =
        (0..floor).map(|i| [r0, -pitch / 2.0 + (pitch / 2.0 - half(0)) * i as f64 / floor as f64]).collect();
    pts.extend(radii.iter().enumerate().map(|(i, &r)| [r, -half(i)]));
    resample(&pts, floor)
}

/// A tooth's right flank, seam to tip, as `right_flank` makes it, from
/// its half-angle `half` at a radius, as a ball end mill of radius `ball`
/// cuts it: under `mate`, where the other part's tips don't reach, the
/// space gets no narrower than the ball, and the corner between flank and
/// floor is rounded to it. The ball rolls in the plane square to the
/// thread, `across` mm per radian at a radius, which is near enough flat
/// across the ball.
fn milled_flank(
    pitch: f64,
    [root, mate, tip]: [f64; 3],
    half: impl Fn(f64) -> f64,
    across: impl Fn(f64) -> f64,
    ball: f64,
) -> Vec<[f64; 2]> {
    let n = 60;
    let radii = |from: f64| (0..=n).map(move |i| from + (tip - from) * i as f64 / n as f64);
    if ball <= 0.0 {
        let radii: Vec<f64> = radii(root).collect();
        return right_flank(pitch, &radii, |i| half(radii[i]));
    }
    // Half the space's width at a radius, and the ball's centre: its
    // radius off the floor, as far out as it can go without cutting the
    // flank.
    let width = |r: f64| {
        let w = (pitch / 2.0 - half(r)) * across(r);
        if r < mate { w.max(ball) } else { w }
    };
    let half = |r: f64| pitch / 2.0 - width(r) / across(r);
    let centre = root + ball;
    let (mut wc, mut touch) = (f64::MAX, centre);
    let m = 200;
    for i in 0..=m {
        let r = root + 2.0 * ball * i as f64 / m as f64;
        let v = width(r) - (ball * ball - (r - centre).powi(2)).max(0.0).sqrt();
        if v < wc {
            (wc, touch) = (v, r);
        }
    }
    let wc = wc.max(0.0);
    let point = |r: f64, w: f64| [r, -(pitch / 2.0 - w / across(r))];
    let floor = if wc > 1e-9 { 8 } else { 1 };
    let mut pts: Vec<[f64; 2]> = (0..floor).map(|i| point(root, wc * i as f64 / floor as f64)).collect();
    // Round the ball from the floor to where it meets the flank.
    let end = (width(touch) - wc).atan2(centre - touch);
    let arc = 16;
    pts.extend((0..arc).map(|i| {
        let a = end * i as f64 / arc as f64;
        point(centre - ball * a.cos(), wc + ball * a.sin())
    }));
    let foot = pts.len();
    pts.extend(radii(touch).map(|r| [r, -half(r)]));
    resample(&pts, foot)
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

/// The worm's ends are chamfered at this angle to the end face, from its
/// outside down to where the wheel's tips reach, so no part-thread is
/// left as a feather edge for the wheel to bear on.
pub const END_CHAMFER: f64 = 30.0;

/// The thread's tip radius `e` from the nearer end, outside radius `tip`
/// and the wheel's tips reaching to `mate`: the chamfer cone, run into
/// the outside by a smooth corner (a quadratic blend half the chamfer's
/// depth across, so the sections between take it without a kink).
pub fn chamfered_tip(tip: f64, mate: f64, e: f64) -> f64 {
    let cone = mate + e.max(0.0) / END_CHAMFER.to_radians().tan();
    let w = 0.5 * (tip - mate).max(0.0);
    let h = (w - (tip - cone).abs()).max(0.0);
    tip.min(cone) - if w > 0.0 { h * h / (4.0 * w) } else { 0.0 }
}

/// Distance from the nearer chamfer's foot to where the threads reach
/// their full outside: the length each end loses to its chamfer.
pub fn chamfer_length(tip: f64, mate: f64) -> f64 {
    1.5 * (tip - mate).max(0.0) * END_CHAMFER.to_radians().tan()
}

impl Design {
    fn whole_depth(&self) -> f64 {
        (self.spec.addendum + self.spec.dedendum) * self.spec.module
    }

    /// The worm's tip and root radii at z from its middle: arcs round the
    /// wheel's axis on a globoid worm.
    fn worm_tip_at(&self, z: f64) -> f64 {
        match &self.hourglass {
            Some(h) => self.center - (h.tip_arc * h.tip_arc - z * z).max(0.0).sqrt(),
            None => self.worm_tip,
        }
    }

    fn worm_root_at(&self, z: f64) -> f64 {
        match &self.hourglass {
            Some(h) => self.center - (h.root_arc * h.root_arc - z * z).max(0.0).sqrt(),
            None => self.worm_root,
        }
    }

    /// The worm's outside radius at its ends, its biggest.
    pub fn worm_rim(&self) -> f64 {
        self.worm_tip_at(self.spec.length / 2.0)
    }

    /// How far into the worm the wheel's tips reach, a distance `z` from
    /// its middle along its axis.
    fn worm_mate_at(&self, z: f64) -> f64 {
        let ha = self.spec.addendum * self.spec.module;
        match &self.hourglass {
            Some(_) => self.center - ((self.wheel_radius + ha).powi(2) - z * z).max(0.0).sqrt(),
            None => self.worm_radius - ha,
        }
    }

    /// The worm's end chamfers: the radius they run down to at the ends,
    /// and the length each end's thread takes to reach its full outside.
    pub fn end_chamfer(&self) -> (f64, f64) {
        let end = self.spec.length / 2.0;
        let mate = self.worm_mate_at(end);
        (mate, chamfer_length(self.worm_tip_at(end), mate))
    }

    /// AGMA 6135's thread length for a globoid worm: its base circle Ø
    /// less 2 % of the centre distance.
    pub fn effective_length(&self) -> Option<f64> {
        self.hourglass.map(|h| 2.0 * h.base - 0.02 * self.center)
    }

    /// Wheel teeth the worm spans on the wheel's pitch circle: those a
    /// globoid worm bears on at once.
    pub fn teeth_spanned(&self) -> f64 {
        2.0 * (self.spec.length / (2.0 * self.wheel_radius)).min(1.0).asin() * self.spec.teeth as f64 / TAU
    }

    /// Half the angle round the wheel's axis a globoid worm's thread
    /// spans, ρ from that axis in the axial plane: the gap between two
    /// wheel teeth, whose straight flanks touch the base circle.
    fn globoid_half(&self, h: &Hourglass, rho: f64) -> f64 {
        PI / self.spec.teeth as f64 - (h.touch - (h.base / rho.max(h.base)).acos())
    }

    /// The wheel's root: a torus round the worm's axis, the clearance
    /// beyond the farthest any thread tip comes within the face (a
    /// globoid worm's end threads stand taller), and on a globoid drive
    /// no nearer than AGMA 6135's root form radius, twice the clearance
    /// past the throat's tips.
    fn wheel_root_form(&self) -> f64 {
        let (a, half_face) = (self.center, self.spec.face / 2.0);
        let mut far = self.worm_tip;
        let n = 40;
        for i in 0..=n {
            let y = self.spec.length / 2.0 * i as f64 / n as f64;
            let r = self.worm_tip_at(y);
            for j in 0..=n {
                let psi = PI / 2.0 * j as f64 / n as f64;
                let w = r * psi.sin();
                if w > half_face {
                    break;
                }
                // Distance from the circle the torus is swept round.
                let q = (a - r * psi.cos()).hypot(y);
                far = far.max((a - q).hypot(w));
            }
        }
        let agma = if self.hourglass.is_some() { self.worm_tip + 2.0 * self.clearance } else { 0.0 };
        (far + self.clearance).max(agma)
    }

    /// Half the narrowest width, square to the threads, of a worm thread
    /// space and of a wheel tooth space (mm): where the other part's tips
    /// reach, below which the spaces are cut no narrower than the ball.
    fn narrowest_spaces(&self) -> [f64; 2] {
        let thick = PI * self.spec.backlash / self.lead;
        let mate = self.spec.addendum * self.spec.module;
        match &self.hourglass {
            Some(h) => {
                let (z2, ratio) = (self.spec.teeth as f64, self.spec.starts as f64 / self.spec.teeth as f64);
                let (rho, tip) = (self.wheel_radius + mate, h.tip_arc);
                let cos = self.lead_angle.cos();
                [
                    (PI / z2 - self.globoid_half(h, rho)) * rho * cos,
                    (self.globoid_half(h, tip) + thick * ratio) * tip * cos,
                ]
            }
            None => {
                let r = self.worm_radius - mate;
                [
                    (PI / self.spec.starts as f64 - self.worm_half(r)) * self.across(r),
                    (self.worm_half(self.worm_tip) + thick) * self.across(self.worm_tip),
                ]
            }
        }
    }

    /// Millimetres square to a cylindrical worm's thread per radian round
    /// its axis, at radius r.
    fn across(&self, r: f64) -> f64 {
        let lead = self.lead / TAU;
        lead * r / r.hypot(lead)
    }

    /// Why a ball end mill of radius `ball` can't cut this drive, if it
    /// can't: it must get down between the other part's tips.
    fn ball_trouble(&self, ball: f64) -> Option<String> {
        let [worm, wheel] = self.narrowest_spaces();
        let (what, space) = if worm < wheel { ("worm's thread", worm) } else { ("wheel's tooth", wheel) };
        (space < ball).then(|| {
            format!(
                "The {what} spaces narrow to {:.2} mm where the other part's tips reach, too narrow for a \
                 Ø{:.2} mm ball end mill to cut below",
                2.0 * space,
                2.0 * ball
            )
        })
    }

    /// Half the angle a worm thread spans at radius r, across the axis:
    /// an involute above the base circle, radial below it.
    pub fn worm_half(&self, r: f64) -> f64 {
        let a = (self.worm_base / r.max(self.worm_base)).acos();
        PI / (2.0 * self.spec.starts as f64) + inv(self.worm_alpha) - inv(a)
    }

    /// How far inside a worm thread a point is (worm frame, z measured
    /// from the middle of the worm), positive in the worm. `tip` raises
    /// the thread tips, or with None the flanks run on past them, though
    /// the thread gets no thinner than the ball; `thick` widens the
    /// threads (radians a side). A cylindrical worm's threads run on past
    /// its ends.
    fn worm_inside(&self, p: [f64; 3], tip: Option<f64>, thick: f64) -> f64 {
        let r = p[0].hypot(p[1]);
        let past = |tip_r: f64, half: f64, scale: f64| {
            if tip.is_none() && r > tip_r { half.max(self.ball / scale) } else { half }
        };
        let raise = tip.unwrap_or(f64::INFINITY);
        if let Some(h) = &self.hourglass {
            // Turned into the axial plane facing the wheel, the point is
            // in a thread if it is between two of the tool's teeth there.
            let (z, ratio) = (p[2], self.spec.starts as f64 / self.spec.teeth as f64);
            let rho = (self.center - r).hypot(z);
            let pitch = TAU / self.spec.teeth as f64;
            let at = z.atan2(self.center - r) - self.sense * ratio * p[1].atan2(p[0]);
            let d = (at + pitch / 2.0).rem_euclid(pitch) - pitch / 2.0;
            let scale = rho * self.lead_angle.cos();
            let half = past(self.worm_tip_at(z), self.globoid_half(h, rho) + thick * ratio, scale);
            let thread = (half - d.abs()) * scale;
            // Unlike a cylindrical worm's, the threads stop at the ends.
            return thread
                .min(self.worm_tip_at(z) + raise - r)
                .max(self.worm_root_at(z) - r)
                .min(self.spec.length / 2.0 - z.abs());
        }
        let pitch = TAU / self.spec.starts as f64;
        let turn = self.sense * TAU * p[2] / self.lead;
        let d = (p[1].atan2(p[0]) - turn + pitch / 2.0).rem_euclid(pitch) - pitch / 2.0;
        // Azimuth to millimetres, roughly normal to the thread.
        let scale = self.across(r);
        let thread = (past(self.worm_tip, self.worm_half(r) + thick, scale) - d.abs()) * scale;
        thread.min(self.worm_tip + raise - r).max(self.worm_root - r)
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
        if let Some(h) = &self.hourglass {
            return self.globoid_worm(h);
        }
        let z1 = self.spec.starts;
        let pitch = TAU / z1 as f64;
        let mate = self.worm_mate_at(0.0);
        // For the 3D printer the threads are eroded by a quarter of the
        // nozzle width: thinner, with a lower tip and deeper root, and the root
        // corners rounder by as much.
        let d = self.spec.nozzle / 4.0;
        let (root, outside) = (self.worm_root - d, self.worm_tip - d);
        let flank = |tip: f64| {
            milled_flank(pitch, [root, mate, tip], |r| self.worm_half(r) - d / self.across(r), |r| self.across(r), self.ball + d)
        };
        let full = flank(outside);
        let length = self.spec.length;
        let turns = length / self.lead;
        let sections = ((turns * SECTIONS_PER_TURN).ceil() as usize).max(10) + 1;
        let stations: Vec<f64> = (0..sections).map(|k| length * k as f64 / (sections - 1) as f64).collect();
        let turn = |z: f64| self.sense * TAU * (z - length / 2.0) / self.lead;
        let hub = hub_radius(self.spec.worm_bore, self.worm_root, self.whole_depth(), "worm")?;
        // The chamfered ends' sections have their own, shorter, threads.
        let tips: Vec<f64> = stations.iter().map(|&z| chamfered_tip(outside, mate, z.min(length - z))).collect();
        let profiles: Vec<Vec<[f64; 2]>> =
            tips.iter().map(|&t| if t < outside - 1e-12 { flank(t) } else { full.clone() }).collect();
        Ok(Member {
            teeth: z1,
            blank: cylinder(outside, length, None),
            t_root: vec![root; sections],
            t_tip: tips,
            right: stations.iter().zip(&profiles).map(|(&z, p)| p.iter().map(|&[r, a]| [r, a + turn(z)]).collect()).collect(),
            left: stations.iter().zip(&profiles).map(|(&z, p)| p.iter().map(|&[r, a]| [r, -a + turn(z)]).collect()).collect(),
            stations,
            hub: [hub; 2],
            bore: self.spec.worm_bore,
            raised: 0.0,
        })
    }

    /// A globoid worm: on each section across its axis, the threads as
    /// the tool's teeth leave them, between the tip and root arcs.
    fn globoid_worm(&self, h: &Hourglass) -> Result<Member, String> {
        let z1 = self.spec.starts;
        let pitch = TAU / z1 as f64;
        let ratio = self.spec.teeth as f64 / z1 as f64;
        let length = self.spec.length;
        let sections = ((length / self.lead * SECTIONS_PER_TURN).ceil() as usize).max(10) + 1;
        let stations: Vec<f64> = (0..sections).map(|k| length * k as f64 / (sections - 1) as f64).collect();
        let (mut right, mut left, mut t_root, mut t_tip) = (vec![], vec![], vec![], vec![]);
        // For the 3D printer the threads are eroded by a quarter of the nozzle width.
        let d = self.spec.nozzle / 4.0;
        for &s in &stations {
            let z = s - length / 2.0;
            let root = self.worm_root_at(z) - d;
            let rho = |r: f64| (self.center - r).hypot(z);
            // A turn of the worm moves a thread round the wheel's axis.
            let across = |r: f64| rho(r) / ratio * self.lead_angle.cos();
            let half = |r: f64| self.globoid_half(h, rho(r)) * ratio - d / across(r);
            // The wheel's tips, round its axis.
            let mate = self.worm_mate_at(z);
            let tip = chamfered_tip(self.worm_tip_at(z) - d, mate, s.min(length - s));
            let flank = milled_flank(pitch, [root, mate, tip], half, across, self.ball + d);
            // The thread's middle sits where the tool's gap was when the
            // point passed the axial plane.
            let turn = |r: f64| self.sense * z.atan2(self.center - r) * ratio;
            right.push(flank.iter().map(|&[r, a]| [r, a + turn(r)]).collect());
            left.push(flank.iter().map(|&[r, a]| [r, -a + turn(r)]).collect());
            t_root.push(root);
            t_tip.push(tip);
        }
        let hub = hub_radius(self.spec.worm_bore, self.worm_root, self.whole_depth(), "worm")?;
        Ok(Member {
            teeth: z1,
            blank: cylinder(self.worm_rim(), length, Some([self.center, h.tip_arc, length / 2.0])).eroded(d),
            t_root,
            t_tip,
            right,
            left,
            stations,
            hub: [hub; 2],
            bore: self.spec.worm_bore,
            raised: 0.0,
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
    /// space the running worm needs: positive where some position of the
    /// worm, turning the wheel, has a thread there (thickened by the
    /// backlash, its flanks run on past its tip, so the space's flanks
    /// run down to the root). The max of its inside measure over the
    /// wheel turns that bring the point past the worm.
    fn swept(&self, s: f64, t: f64, phi: f64) -> f64 {
        let k = self.ratio();
        let reach = self.root_form + 3.0 * self.ball + self.clearance;
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
        let at = |theta: f64| self.worm_inside(self.wheel_to_worm(x, k * theta, theta), None, thick);
        let centre = ((PI - phi + PI).rem_euclid(TAU) - PI) / k;
        let half = spread / k.abs();
        // A dozen samples each time a thread passes.
        let n = ((24.0 * half / (TAU / self.spec.starts as f64)).ceil() as usize).clamp(24, 2000);
        let step = 2.0 * half / n as f64;
        let mut samples: Vec<(f64, f64)> = (0..=n).map(|i| centre - half + step * i as f64).map(|th| (th, at(th))).collect();
        samples.sort_by(|a, b| b.1.total_cmp(&a.1));
        samples.iter().take(3).map(|&(th, _)| hypoid::golden(&at, th - step, th + step, 30).1).fold(f64::MIN, f64::max)
    }

    /// Across a wheel tooth space, the share of its width that is square
    /// to the worm's thread there, ρ from the worm's axis.
    fn square(&self, rho: f64) -> f64 {
        match self.hourglass {
            Some(_) => self.lead_angle.cos(),
            None => rho / rho.hypot(self.lead / TAU),
        }
    }

    /// Where the ball end mill rounds the corners at the bottom of the
    /// wheel's tooth spaces, across the face. At the root the corners run
    /// round the worm's axis, so each surface of points a fixed angle ψ
    /// round it (at ρ from it, turned φ on the wheel) cuts them square.
    /// On each, the ball's centre sits its radius off the root, as close
    /// to the middle of the space as the flank lets it.
    fn corners(&self) -> Corners {
        let (ball, rho0) = (self.ball, self.root_form);
        let reach = ((self.spec.face / 2.0) / (rho0 - 2.0 * ball)).min(1.0).asin();
        let n = 41;
        let pitch = TAU / self.spec.teeth as f64;
        let at = |psi: f64, rho: f64| (self.spec.face / 2.0 + rho * psi.sin(), self.center - rho * psi.cos());
        let row = |psi: f64| -> Corner {
            // The space's middle, 2 balls up, from the one facing the worm.
            let (s, t) = at(psi, rho0 - 2.0 * ball);
            let open = |phi: f64| self.swept(s, t, phi);
            let k = 48;
            let guess = (0..k).map(|i| PI - pitch / 2.0 + pitch * i as f64 / k as f64).max_by(|a, b| open(*a).total_cmp(&open(*b)));
            let guess = guess.unwrap_or(PI);
            let mid = hypoid::golden(&open, guess - pitch / k as f64, guess + pitch / k as f64, 40).0;
            let centre_at = rho0 - ball;
            let mut out = Corner { mid, centre: [f64::MAX; 2], turn: [0.0; 2] };
            let m = 12;
            for (side, dir) in [(0, -1.0), (1, 1.0)] {
                // Half the space's width at ρ: out from the middle to the
                // flank, by bisection.
                let width = |rho: f64| {
                    let (s, t) = at(psi, rho);
                    let gap = |phi: f64| self.swept(s, t, phi);
                    if gap(mid) <= 0.0 {
                        return None;
                    }
                    let (mut lo, mut hi) = (0.0, pitch / 2.0);
                    for _ in 0..40 {
                        let d = (lo + hi) / 2.0;
                        if gap(mid + dir * d) > 0.0 { lo = d } else { hi = d }
                    }
                    Some(t * lo * self.square(rho))
                };
                // How far out the ball's centre can go for the flank at ρ.
                let limit = |rho: f64| width(rho).map(|w| w - (ball * ball - (rho - centre_at).powi(2)).max(0.0).sqrt());
                let step = 2.0 * ball / m as f64;
                let best = (0..=m)
                    .map(|j| rho0 - 2.0 * ball + step * j as f64)
                    .filter_map(|rho| Some((rho, limit(rho)?)))
                    .min_by(|a, b| a.1.total_cmp(&b.1));
                if let Some((rho, _)) = best {
                    let (lo, hi) = ((rho - step).max(rho0 - 2.0 * ball), (rho + step).min(rho0));
                    let rho = hypoid::golden(&|r| -limit(r).unwrap_or(f64::MAX), lo, hi, 30).0;
                    if let (Some(centre), Some(w)) = (limit(rho), width(rho)) {
                        out.centre[side] = centre;
                        out.turn[side] = (w - centre).atan2(rho - centre_at);
                    }
                }
            }
            out
        };
        let step = 2.0 * reach / (n - 1) as f64;
        let mut rows: Vec<Corner> = std::thread::scope(|scope| {
            let hs: Vec<_> = (0..n).map(|i| scope.spawn(move || row(-reach + step * i as f64))).collect();
            hs.into_iter().map(|h| h.join().unwrap()).collect()
        });
        // Keep the middles running on from row to row.
        for i in 1..rows.len() {
            let prev = rows[i - 1].mid;
            rows[i].mid = prev + ((rows[i].mid - prev + pitch / 2.0).rem_euclid(pitch) - pitch / 2.0);
        }
        Corners { first: -reach, step, rows }
    }

    /// How far a wheel point lies in what the ball end mill cuts: the
    /// space the worm needs, down to the root torus, with the corners
    /// between them rounded to the ball.
    fn milled(&self, corners: &Corners, s: f64, t: f64, phi: f64) -> f64 {
        let ball = self.ball;
        let w = s - self.spec.face / 2.0;
        let rho = (self.center - t).hypot(w);
        let floor = self.root_form - rho;
        let cut = self.swept(s, t, phi).min(floor);
        if ball <= 0.0 || floor > 2.0 * ball || floor < -ball {
            return cut;
        }
        let c = corners.at(w.atan2(self.center - t));
        let pitch = TAU / self.spec.teeth as f64;
        let off = (phi - c.mid + pitch / 2.0).rem_euclid(pitch) - pitch / 2.0;
        let side = usize::from(off >= 0.0);
        // From the ball's centre: down towards the root, and out across.
        let d = [rho - (self.root_form - ball), t * off.abs() * self.square(rho) - c.centre[side]];
        let angle = d[1].atan2(d[0]);
        if c.centre[side] == f64::MAX || angle < 0.0 || angle > c.turn[side] {
            return cut;
        }
        cut.min(ball - d[0].hypot(d[1]))
    }

    /// A throated wheel, as the ball end mill cuts it.
    fn throated_wheel(&self) -> Result<Member, String> {
        let b = self.spec.face;
        let blank = cylinder(self.wheel_rim, b, Some([self.center, self.center - self.wheel_throat, b / 2.0]));
        let corners = self.corners();
        // For the 3D printer the teeth are eroded by a quarter of the nozzle width.
        let d = self.spec.nozzle / 4.0;
        let gap = |s: f64, t: f64, phi: f64| self.milled(&corners, s, t, phi) + d;
        hypoid::build_member(self.spec.teeth, blank.eroded(d), self.spec.wheel_bore, [0.0; 2], 15, &gap)
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
            dedendum: (self.wheel_radius - self.wheel_root) / sp.module,
            root_fillet: sp.root_fillet,
            standard: ToothStandard::Custom,
            backlash: sp.backlash,
            profile_shift: 0.0,
            nozzle: sp.nozzle,
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
            raised: 0.0,
        })
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    pub fn spec(starts: u32, teeth: u32, throated: bool) -> Spec {
        Spec {
            starts,
            teeth,
            module: 2.0,
            pressure_angle: 20.0,
            addendum: 1.0,
            dedendum: 1.25,
            // A Ø0.96 mm ball.
            root_fillet: 0.24,
            diameter: 20.0,
            length: 30.0,
            face: 16.0,
            throated,
            globoid: false,
            left_hand: false,
            backlash: 0.0,
            worm_bore: 0.0,
            wheel_bore: 10.0,
            nozzle: 0.0,
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
                            let v = d.worm_inside(d.wheel_to_worm(*p, k * theta, theta), Some(0.0), 0.0);
                            deepest = deepest.max(v);
                        }
                    }
                }
            }
        }
        deepest
    }

    #[test]
    fn back_driving_follows_the_lead_angle() {
        // 1 start at 10 modules: a 5.7° lead.
        let d = spec(1, 30, true).design().unwrap();
        assert_eq!(d.back_drive(0.05), BackDrive::Running);
        assert_eq!(d.back_drive(0.12), BackDrive::Locked);
        assert_eq!(d.back_efficiency(0.12), 0.0);
        assert!(d.efficiency(0.05) > 0.6 && d.efficiency(0.05) < 0.7);
        assert!(d.back_efficiency(0.05) > 0.0 && d.back_efficiency(0.05) < d.efficiency(0.05));
        let steep = spec(4, 30, true).design().unwrap();
        assert_eq!(steep.back_drive(0.05), BackDrive::Free);
        assert!(steep.efficiency(0.05) > 0.85);
    }

    #[test]
    fn back_drivable_drives_add_starts_and_keep_the_ratio() {
        for (mu, hold) in [(0.05, false), (0.05, true), (0.15, false), (0.25, false)] {
            let sp = spec(1, 30, true);
            let got = sp.back_drivable(mu, hold).unwrap();
            let d = got.design().unwrap();
            assert_eq!(d.back_drive(mu), BackDrive::Free, "μ {mu}");
            assert!(d.lead_angle >= back_drive_lead(mu, 20.0) - 1e-9 || got.diameter == sp.diameter);
            assert!(got.diameter <= sp.diameter && got.diameter >= 16.0 - 0.01);
            assert!(got.length >= least_length(got.starts, got.teeth, 2.0) - 1e-9);
            if hold {
                assert_eq!(got.teeth, 30);
            } else {
                assert_eq!(got.teeth, 30 * got.starts);
            }
            assert_eq!(got.back_drivable(mu, hold).unwrap(), got);
        }
        assert_eq!(spec(1, 30, true).back_drivable(0.05, false).unwrap().starts, 3);
        assert!(spec(1, 30, true).back_drivable(0.6, false).is_err());
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
    fn throated_wheels_run_with_their_worms() {
        for (starts, teeth, left) in [(1, 30, false), (3, 31, true), (6, 24, false)] {
            let d = Spec { left_hand: left, ..spec(starts, teeth, true) }.design().unwrap();
            let wheel = d.wheel().unwrap();
            let deepest = deepest(&d, &wheel);
            assert!(deepest < 0.01, "{starts}/{teeth}: the wheel cuts {deepest:.4} mm into the worm");
            assert!(deepest > -0.02, "{starts}/{teeth}: the wheel never touches the worm ({deepest:.4} mm)");
        }
    }

    /// A globoid drive with AGMA 6135's proportions.
    fn globoid(starts: u32, teeth: u32, left: bool) -> Design {
        let sp = Spec { globoid: true, left_hand: left, diameter: globoid_diameter(2.0, teeth), ..spec(starts, teeth, true) };
        // The biggest ball that fits, if the usual one doesn't.
        let sp = Spec { root_fillet: sp.root_fillet.min(0.99 * Spec { length: 1.0, ..sp.clone() }.largest_ball().unwrap() / 4.0), ..sp };
        let short = Spec { length: 1.0, ..sp.clone() }.design().unwrap();
        Spec { length: short.effective_length().unwrap(), face: 2.0 * short.worm_root, ..sp }.design().unwrap()
    }

    #[test]
    fn globoid_wheels_run_with_their_worms() {
        for (starts, teeth, left) in [(1, 40, false), (2, 41, true), (3, 40, false), (4, 30, false)] {
            let d = globoid(starts, teeth, left);
            let wheel = d.wheel().unwrap();
            let deepest = deepest(&d, &wheel);
            assert!(deepest < 0.01, "{starts}/{teeth}: the wheel cuts {deepest:.4} mm into the worm");
            assert!(deepest > -0.02, "{starts}/{teeth}: the wheel never touches the worm ({deepest:.4} mm)");
            // The worm spans about an eighth of the wheel's teeth.
            let spanned = d.teeth_spanned() / teeth as f64;
            assert!(spanned > 0.1 && spanned < 0.16, "{starts}/{teeth}: spans {spanned:.3} of the teeth");
        }
    }

    #[test]
    fn globoid_wheel_teeth_are_straight_in_the_middle() {
        // In the wheel's middle plane each flank is the tool's straight
        // side, touching the base circle.
        let d = globoid(1, 40, false);
        let base = d.hourglass.unwrap().base;
        let wheel = d.wheel().unwrap();
        let mid = wheel.stations.len() / 2;
        assert!((wheel.stations[mid] - d.spec.face / 2.0).abs() < 1e-9);
        let r2 = d.wheel_radius;
        for side in [&wheel.right[mid], &wheel.left[mid]] {
            let pts: Vec<[f64; 2]> = side
                .iter()
                .filter(|q| (q[0] - r2).abs() < 1.6)
                .map(|&[t, phi]| [t * phi.cos(), t * phi.sin()])
                .collect();
            assert!(pts.len() > 5);
            let (a, b) = (pts[0], *pts.last().unwrap());
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let l = dx.hypot(dy);
            let off = |p: [f64; 2]| ((p[0] - a[0]) * dy - (p[1] - a[1]) * dx) / l;
            for p in &pts {
                assert!(off(*p).abs() < 0.005, "flank is {:.4} mm off straight", off(*p));
            }
            let from_centre = off([0.0, 0.0]).abs();
            assert!((from_centre - base).abs() < 0.01, "flank line {from_centre:.4} from the axis, base {base:.4}");
        }
    }

    #[test]
    fn worm_ends_are_chamfered() {
        for (name, d) in [("cylindrical", spec(1, 30, true).design().unwrap()), ("globoid", globoid(1, 40, false))] {
            let m = d.worm().unwrap();
            let k = m.stations.len() - 1;
            let mate = |z: f64| match &d.hourglass {
                Some(_) => d.center - ((d.wheel_radius + d.spec.addendum * d.spec.module).powi(2) - z * z).max(0.0).sqrt(),
                None => d.worm_radius - d.spec.addendum * d.spec.module,
            };
            let half = d.spec.length / 2.0;
            // Down to the wheel's tips at the ends, full in the middle.
            for (e, z) in [(0, -half), (k, half)] {
                assert!((m.t_tip[e] - mate(z)).abs() < 1e-9, "{name}: end {e} tip {} not {}", m.t_tip[e], mate(z));
                let top = m.right[e].last().unwrap()[0];
                assert!((top - mate(z)).abs() < 1e-6, "{name}: end {e} thread reaches {top}");
            }
            assert!((m.t_tip[k / 2] - d.worm_tip_at(m.stations[k / 2] - half)).abs() < 1e-9, "{name}: middle chamfered");
            // The tip rises smoothly: no step from section to section
            // turns by more than the chamfer's angle allows.
            let slope = 1.0 / END_CHAMFER.to_radians().tan();
            for w in m.t_tip.windows(2).zip(m.stations.windows(2)) {
                let ((a, b), (sa, sb)) = ((w.0[0], w.0[1]), (w.1[0], w.1[1]));
                assert!((b - a).abs() <= slope * (sb - sa) * 1.0001 + 1e-9, "{name}: tip jumps {a} to {b}");
            }
        }
    }

    #[test]
    fn roots_are_milled_by_the_ball() {
        let drives = [
            ("throated", spec(1, 30, true).design().unwrap()),
            ("big ball", Spec { root_fillet: 0.38, length: 26.0, face: 18.0, ..spec(1, 30, true) }.design().unwrap()),
            ("globoid", globoid(1, 40, false)),
        ];
        for (name, d) in drives {
            let (worm, wheel) = (d.worm().unwrap(), d.wheel().unwrap());
            // The wheel's root is the torus round the worm's axis: no
            // ridges, whatever the worm's threads do near the faces.
            for (k, &s) in wheel.stations.iter().enumerate() {
                let w = s - d.spec.face / 2.0;
                let torus = d.center - (d.root_form * d.root_form - w * w).sqrt();
                assert!((wheel.t_root[k] - torus).abs() < 1e-3, "{name}: section {k} root {} off the torus {torus}", wheel.t_root[k]);
            }
            // Square to the threads in the middle, the ball reaches every
            // point of the roots.
            let mid = wheel.stations.len() / 2;
            for prof in [&wheel.right[mid], &wheel.left[mid]] {
                let seam = prof[0][1];
                let pts: Vec<[f64; 2]> = prof
                    .iter()
                    .map(|&[t, phi]| {
                        let rho = d.center - t;
                        [d.root_form - rho, t * (phi - seam).abs() * d.square(rho)]
                    })
                    .collect();
                let fits = crate::gear::ball_fits(&pts, d.ball);
                assert!(fits > 0.97, "{name}: only {fits:.3} of the ball fits the wheel's root");
            }
            let mid = worm.stations.len() / 2;
            let z = worm.stations[mid] - d.spec.length / 2.0;
            for prof in [&worm.right[mid], &worm.left[mid]] {
                let seam = prof[0][1];
                let across = |r: f64| match &d.hourglass {
                    Some(_) => (d.center - r).hypot(z) * d.spec.teeth as f64 / d.spec.starts as f64 * d.lead_angle.cos(),
                    None => d.across(r),
                };
                let pts: Vec<[f64; 2]> =
                    prof.iter().map(|&[r, a]| [r - prof[0][0], (a - seam).abs() * across(r)]).collect();
                let fits = crate::gear::ball_fits(&pts, d.ball);
                assert!(fits > 0.97, "{name}: only {fits:.3} of the ball fits the worm's root");
            }
        }
    }

    #[test]
    fn a_ball_too_big_for_the_spaces_is_refused() {
        let sp = Spec { root_fillet: 1.0, ..spec(1, 30, true) };
        let largest = sp.largest_ball().unwrap();
        assert!(largest > 1.0 && largest < 2.0, "{largest}");
        let err = sp.design().unwrap_err();
        assert!(err.contains("too narrow") && err.contains(&format!("Ø{:.2} mm or smaller", (largest * 100.0).floor() / 100.0)), "{err}");
        // Just under the biggest that fits, it cuts.
        assert!(Spec { root_fillet: 0.99 * largest / 4.0, ..sp }.design().is_ok());
    }

    #[test]
    fn globoid_worms_are_hourglasses_on_the_tools_gaps() {
        let d = globoid(2, 41, false);
        let w = d.worm().unwrap();
        let (k0, mid) = (0, w.stations.len() / 2);
        // (The ends' outside is chamfered off; the hourglass is under it.)
        let end = d.worm_tip_at(w.stations[k0] - d.spec.length / 2.0);
        assert!(end > w.t_tip[mid] + 0.5 && w.t_root[k0] > w.t_root[mid] + 0.5);
        assert!((w.t_tip[mid] - d.worm_tip).abs() < 0.01);
        // At the throat the thread is 45 % of the axial pitch thick on the
        // pitch diameter.
        let thick = |side: &Vec<[f64; 2]>| {
            let i = side.iter().position(|q| q[0] >= d.worm_radius).unwrap();
            let (a, b) = (side[i - 1], side[i]);
            a[1] + (b[1] - a[1]) * (d.worm_radius - a[0]) / (b[0] - a[0])
        };
        let span = thick(&w.left[mid]) - thick(&w.right[mid]);
        let axial = span * d.lead / TAU;
        assert!((axial - 0.45 * d.axial_pitch).abs() < 0.02, "thread {axial:.4}, pitch {:.4}", d.axial_pitch);
        // Its flank points measure as on the worm, where the wheel's tips
        // work (below, the space is kept as wide as the ball).
        for k in [0, mid, w.stations.len() - 1] {
            for i in [PROFILE_POINTS / 2, PROFILE_POINTS - 2] {
                let [r, phi] = w.right[k][i];
                let z = w.stations[k] - d.spec.length / 2.0;
                let mate = d.center - ((d.wheel_radius + d.spec.addendum * d.spec.module).powi(2) - z * z).max(0.0).sqrt();
                if r < mate {
                    continue;
                }
                let v = d.worm_inside([r * phi.cos(), r * phi.sin(), z], Some(0.0), 0.0);
                assert!(v.abs() < 1e-4 || k != mid, "{v}");
                assert!(v.abs() < 1e-3, "station {k}: {v}");
            }
        }
        // Past the base circle it can't be cut.
        assert!(Spec { length: 2.0 * d.hourglass.unwrap().base + 0.1, ..d.spec.clone() }.design().is_err());
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
            let d = crate::train::worm_spec(&p.gears[0].params, &p.gears[1].params, &p.gears[1].placement, p.ball())
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
                                deepest = deepest.max(d.worm_inside(y, Some(0.0), 0.0));
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
        assert!(d.worm_inside([r * phi.cos(), r * phi.sin(), z], Some(0.0), 0.0).abs() < 1e-5);
    }
}

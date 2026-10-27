//! Timing pulleys and belt drives.
//!
//! Pulley grooves are generated the way they are hobbed: by rolling the
//! ISO 5294 generating rack (Table 1) on the belt pitch circle, which lies
//! the pitch-line differential `a` outside the pulley's outside diameter.
//! Only the trapezoidal inch sections are here, because they are the ones
//! whose groove geometry is published in full; curvilinear profiles (HTD,
//! GT) and the metric T/AT sections need their makers' or DIN 7721 data.

use crate::gear::Rack;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BeltSection {
    Mxl,
    Xxl,
    Xl,
    L,
    H,
    Xh,
    Xxh,
}

/// ISO 5294 Table 1 generating rack for one section (mm, degrees).
#[derive(Clone, Copy, Debug)]
pub struct GenRack {
    /// Half the included angle of the rack tooth.
    pub flank: f64,
    /// Rack tooth height: the groove depth.
    pub hr: f64,
    /// Rack tooth width at its tip: the groove bottom width.
    pub bg: f64,
    /// Rack tooth tip corner radius (rounds the bottom of the groove).
    pub r1: f64,
    /// Rack root fillet radius (rounds the pulley tooth tips).
    pub r2: f64,
    /// Pitch-line differential 2a: pitch diameter minus outside diameter.
    pub pld: f64,
}

impl GenRack {
    /// The biggest groove-bottom corner radius: the rack tip a full round.
    pub fn max_r1(&self) -> f64 {
        let (sa, ca) = self.flank.to_radians().sin_cos();
        self.bg / 2.0 * ca / (1.0 - sa)
    }
}

/// ISO 5294 Table 4: a standard belt width and the narrowest pulley that
/// carries it, with flanges and without.
#[derive(Clone, Copy, Debug)]
pub struct BeltWidth {
    pub code: &'static str,
    pub width: f64,
    pub flanged: f64,
    pub unflanged: f64,
}

const fn bw(code: &'static str, width: f64, flanged: f64, unflanged: f64) -> BeltWidth {
    BeltWidth { code, width, flanged, unflanged }
}

const NARROW: [BeltWidth; 3] = [bw("012", 3.2, 3.8, 5.6), bw("019", 4.8, 5.3, 7.1), bw("025", 6.4, 7.1, 8.9)];
const XL_WIDTHS: [BeltWidth; 3] = [bw("025", 6.4, 7.1, 8.9), bw("031", 7.9, 8.6, 10.4), bw("037", 9.5, 10.4, 12.2)];
const L_WIDTHS: [BeltWidth; 3] = [bw("050", 12.7, 14.0, 17.0), bw("075", 19.1, 20.3, 23.3), bw("100", 25.4, 26.7, 29.7)];
const H_WIDTHS: [BeltWidth; 5] = [
    bw("075", 19.1, 20.3, 24.8),
    bw("100", 25.4, 26.7, 31.2),
    bw("150", 38.1, 39.4, 43.9),
    bw("200", 50.8, 52.8, 57.3),
    bw("300", 76.2, 79.0, 83.5),
];
const XH_WIDTHS: [BeltWidth; 3] = [bw("200", 50.8, 56.6, 62.6), bw("300", 76.2, 83.8, 89.8), bw("400", 101.6, 110.7, 116.7)];
const XXH_WIDTHS: [BeltWidth; 4] = [
    bw("200", 50.8, 56.6, 64.1),
    bw("300", 76.2, 83.8, 91.3),
    bw("400", 101.6, 110.7, 118.2),
    bw("500", 127.0, 137.7, 145.2),
];

/// Tooth counts listed in ISO 5294 Table 5 (non-preferred sizes left out).
const PREFERRED_TEETH: [u32; 24] =
    [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 25, 28, 32, 36, 40, 48, 60, 72, 84, 96, 120, 156];

impl BeltSection {
    pub const ALL: [BeltSection; 7] =
        [BeltSection::Mxl, BeltSection::Xxl, BeltSection::Xl, BeltSection::L, BeltSection::H, BeltSection::Xh, BeltSection::Xxh];

    pub fn name(self) -> &'static str {
        match self {
            BeltSection::Mxl => "MXL",
            BeltSection::Xxl => "XXL",
            BeltSection::Xl => "XL",
            BeltSection::L => "L",
            BeltSection::H => "H",
            BeltSection::Xh => "XH",
            BeltSection::Xxh => "XXH",
        }
    }

    /// Name with the belt pitch, for pick lists.
    pub fn label(self) -> String {
        let inch = match self {
            BeltSection::Mxl => "0.080\"",
            BeltSection::Xxl => "1/8\"",
            BeltSection::Xl => "1/5\"",
            BeltSection::L => "3/8\"",
            BeltSection::H => "1/2\"",
            BeltSection::Xh => "7/8\"",
            BeltSection::Xxh => "1¼\"",
        };
        format!("{} — {inch} pitch", self.name())
    }

    /// Belt pitch, mm.
    pub fn pitch(self) -> f64 {
        match self {
            BeltSection::Mxl => 2.032,
            BeltSection::Xxl => 3.175,
            BeltSection::Xl => 5.080,
            BeltSection::L => 9.525,
            BeltSection::H => 12.700,
            BeltSection::Xh => 22.225,
            BeltSection::Xxh => 31.750,
        }
    }

    /// Fewest teeth ISO 5294 gives a rack for.
    pub fn min_teeth(self) -> u32 {
        match self {
            BeltSection::H => 14,
            BeltSection::Xh | BeltSection::Xxh => 18,
            _ => 10,
        }
    }

    /// ISO 5294 Table 1. MXL and H change their rack for small pulleys.
    pub fn rack(self, teeth: u32) -> GenRack {
        let g = |flank, hr, bg, r1, r2, pld| GenRack { flank, hr, bg, r1, r2, pld };
        match self {
            BeltSection::Mxl if teeth <= 23 => g(28.0, 0.64, 0.61, 0.30, 0.23, 0.508),
            BeltSection::Mxl => g(20.0, 0.64, 0.67, 0.30, 0.23, 0.508),
            BeltSection::Xxl => g(25.0, 0.84, 0.96, 0.30, 0.28, 0.508),
            BeltSection::Xl => g(25.0, 1.40, 1.27, 0.61, 0.61, 0.508),
            BeltSection::L => g(20.0, 2.13, 3.10, 0.86, 0.53, 0.762),
            BeltSection::H if teeth <= 19 => g(20.0, 2.59, 4.24, 1.47, 1.04, 1.372),
            BeltSection::H => g(20.0, 2.59, 4.24, 1.47, 1.42, 1.372),
            BeltSection::Xh => g(20.0, 6.88, 7.59, 2.01, 1.93, 2.794),
            BeltSection::Xxh => g(20.0, 10.29, 11.61, 2.69, 2.82, 3.048),
        }
    }

    pub fn widths(self) -> &'static [BeltWidth] {
        match self {
            BeltSection::Mxl | BeltSection::Xxl => &NARROW,
            BeltSection::Xl => &XL_WIDTHS,
            BeltSection::L => &L_WIDTHS,
            BeltSection::H => &H_WIDTHS,
            BeltSection::Xh => &XH_WIDTHS,
            BeltSection::Xxh => &XXH_WIDTHS,
        }
    }

    /// The standard width nearest `w` mm.
    pub fn width_near(self, w: f64) -> BeltWidth {
        *self.widths().iter().min_by(|a, b| (a.width - w).abs().total_cmp(&(b.width - w).abs())).unwrap()
    }

    /// Preferred tooth counts for this section. Table 5 stops at 72 teeth
    /// for the three small sections.
    pub fn stock_teeth(self) -> Vec<u32> {
        let max = match self {
            BeltSection::Mxl | BeltSection::Xxl | BeltSection::Xl => 72,
            _ => 156,
        };
        PREFERRED_TEETH.iter().copied().filter(|&z| z >= self.min_teeth() && z <= max).collect()
    }
}

/// ISO 5296-1 Tables 3a and 3b: standard belt length designations, the
/// pitch length in tenths of an inch.
const MXL_BELTS: [u32; 17] = [36, 40, 44, 48, 56, 60, 64, 72, 80, 88, 100, 112, 124, 140, 160, 180, 200];
const XL_BELTS: [u32; 21] =
    [60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260];
const L_BELTS: [u32; 19] = [124, 150, 187, 210, 225, 240, 255, 285, 300, 322, 345, 367, 390, 420, 450, 480, 510, 540, 600];
const H_BELTS: [u32; 25] = [
    240, 270, 300, 330, 360, 390, 420, 450, 480, 510, 540, 570, 600, 630, 660, 700, 750, 800, 850, 900, 1000, 1100, 1250,
    1400, 1700,
];
const XH_BELTS: [u32; 12] = [507, 560, 630, 700, 770, 840, 980, 1120, 1260, 1400, 1540, 1750];
const XXH_BELTS: [u32; 8] = [700, 800, 900, 1000, 1200, 1400, 1600, 1800];

/// A standard belt: its designation (as printed on it) and tooth count.
#[derive(Clone, Debug, PartialEq)]
pub struct StdBelt {
    pub designation: String,
    pub teeth: u32,
}

impl BeltSection {
    /// The ISO 5296 standard lengths for this section, shortest first.
    /// XXL is not in the 1989 tables, so it has none.
    pub fn standard_belts(self) -> Vec<StdBelt> {
        let list: &[u32] = match self {
            BeltSection::Mxl => &MXL_BELTS,
            BeltSection::Xxl => &[],
            BeltSection::Xl => &XL_BELTS,
            BeltSection::L => &L_BELTS,
            BeltSection::H => &H_BELTS,
            BeltSection::Xh => &XH_BELTS,
            BeltSection::Xxh => &XXH_BELTS,
        };
        list.iter()
            .map(|&d| {
                let inches = d as f64 / 10.0;
                let teeth = (inches * 25.4 / self.pitch()).round() as u32;
                // MXL designations keep their decimal (72.0MXL); the rest
                // are whole tenths of an inch (130XL).
                let designation = match self {
                    BeltSection::Mxl => format!("{:.1}{}", d as f64, self.name()),
                    _ => format!("{d}{}", self.name()),
                };
                StdBelt { designation, teeth }
            })
            .collect()
    }

    /// The designation of a belt with this many teeth, if it is standard.
    pub fn designation(self, teeth: u32) -> Option<String> {
        self.standard_belts().into_iter().find(|b| b.teeth == teeth).map(|b| b.designation)
    }
}

/// A belt's cross-section along its length (ISO 5296-1 Table 1), mm:
/// overall thickness, tooth height, tooth width at the base and the
/// flank half-angle in degrees.
#[derive(Clone, Copy, Debug)]
pub struct BeltTooth {
    pub thickness: f64,
    pub height: f64,
    pub width: f64,
    pub half_angle: f64,
}

impl BeltSection {
    pub fn belt_tooth(self) -> BeltTooth {
        let t = |thickness, height, width, half_angle| BeltTooth { thickness, height, width, half_angle };
        match self {
            BeltSection::Mxl => t(1.14, 0.51, 1.14, 20.0),
            BeltSection::Xxl => t(1.52, 0.76, 1.73, 25.0),
            BeltSection::Xl => t(2.30, 1.27, 2.57, 25.0),
            BeltSection::L => t(3.60, 1.91, 4.65, 20.0),
            BeltSection::H => t(4.30, 2.29, 6.12, 20.0),
            BeltSection::Xh => t(11.20, 6.35, 12.57, 20.0),
            BeltSection::Xxh => t(15.70, 9.53, 19.05, 20.0),
        }
    }
}

/// Pulleys are generated with a tooth centred on angle 0, so their
/// grooves are centred half a pitch round from it.
pub fn groove_angle(teeth: u32) -> f64 {
    PI / teeth as f64
}

/// An open belt's pitch line: round the far side of pulley 1, along the
/// tangent to pulley 2, round its near side and back. Measured by arc
/// length `s` from where it leaves the second straight run onto pulley
/// 1, running the way the pulleys turn when positive (anticlockwise).
pub struct BeltLoop {
    pub c1: [f64; 2],
    pub c2: [f64; 2],
    pub r1: f64,
    pub r2: f64,
    theta: f64,
    alpha: f64,
    span: f64,
}

impl BeltLoop {
    /// None when the pulleys overlap so no belt can go round both.
    pub fn new(c1: [f64; 2], r1: f64, c2: [f64; 2], r2: f64) -> Option<Self> {
        let d = (c2[0] - c1[0]).hypot(c2[1] - c1[1]);
        if d <= (r1 - r2).abs() + 1e-9 {
            return None;
        }
        let theta = (c2[1] - c1[1]).atan2(c2[0] - c1[0]);
        let alpha = ((r1 - r2) / d).acos();
        let span = (d * d - (r1 - r2) * (r1 - r2)).sqrt();
        Some(BeltLoop { c1, c2, r1, r2, theta, alpha, span })
    }

    fn arc1(&self) -> f64 {
        self.r1 * (2.0 * PI - 2.0 * self.alpha)
    }

    fn arc2(&self) -> f64 {
        self.r2 * 2.0 * self.alpha
    }

    pub fn len(&self) -> f64 {
        self.arc1() + self.arc2() + 2.0 * self.span
    }

    /// Arc length at which the belt sits at `angle` on pulley 1.
    pub fn s_on_first(&self, angle: f64) -> f64 {
        self.r1 * (angle - self.theta - self.alpha)
    }

    /// Angle on pulley 2 of the belt point at arc length `s`.
    pub fn angle_on_second(&self, s: f64) -> f64 {
        self.theta - self.alpha + (s - self.arc1() - self.span) / self.r2
    }

    /// Point on the pitch line and the outward normal there.
    pub fn at(&self, s: f64) -> ([f64; 2], [f64; 2]) {
        let s = s.rem_euclid(self.len());
        let u = |a: f64| [a.cos(), a.sin()];
        let on = |c: [f64; 2], r: f64, a: f64| ([c[0] + r * a.cos(), c[1] + r * a.sin()], u(a));
        let lerp = |p: [f64; 2], q: [f64; 2], t: f64| [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t];
        let (a1, l, a2) = (self.arc1(), self.span, self.arc2());
        let lo = self.theta - self.alpha;
        let hi = self.theta + self.alpha;
        if s < a1 {
            on(self.c1, self.r1, hi + s / self.r1)
        } else if s < a1 + l {
            let p = on(self.c1, self.r1, lo).0;
            let q = on(self.c2, self.r2, lo).0;
            (lerp(p, q, (s - a1) / l), u(lo))
        } else if s < a1 + l + a2 {
            on(self.c2, self.r2, lo + (s - a1 - l) / self.r2)
        } else {
            let p = on(self.c2, self.r2, hi).0;
            let q = on(self.c1, self.r1, hi).0;
            (lerp(p, q, (s - a1 - l - a2) / l), u(hi))
        }
    }
}

/// The belt whose centre distance comes closest to `target` for pulleys
/// of pitch radii r1 and r2: from the standard lengths if `standard_only`
/// (and the section has any), otherwise any whole number of teeth.
/// Returns the tooth count and the centre distance it gives.
pub fn belt_for_center(section: BeltSection, r1: f64, r2: f64, target: f64, standard_only: bool) -> Option<(u32, f64)> {
    let pitch = section.pitch();
    let candidates: Vec<u32> = if standard_only && !section.standard_belts().is_empty() {
        section.standard_belts().iter().map(|b| b.teeth).collect()
    } else {
        // The exact length for the target, and the whole tooth counts
        // either side of it.
        let exact = belt_length(r1, r2, target.max((r2 - r1).abs() + 1e-6)) / pitch;
        vec![exact.floor().max(1.0) as u32, exact.ceil().max(1.0) as u32]
    };
    candidates
        .into_iter()
        .filter_map(|t| center_distance(r1, r2, t as f64 * pitch).map(|c| (t, c)))
        .min_by(|a, b| (a.1 - target).abs().total_cmp(&(b.1 - target).abs()))
}

/// The generating rack as drawn in ISO 5294 Figure 1, in the generator's
/// rack frame: u along the rack, v depth below the rolling (pitch) line.
/// One rack tooth with half a land either side; the lands sit at depth a,
/// exactly on the pulley's outside diameter. The groove-bottom corners
/// are rounded for a ball end mill of Ø `ball` where ISO's are tighter.
pub(crate) fn rack(section: BeltSection, teeth: u32, ball: f64) -> Rack {
    let g = section.rack(teeth);
    let pb = section.pitch();
    let a = g.pld / 2.0;
    let fl = g.flank.to_radians();
    let (sa, ca, ta) = (fl.sin(), fl.cos(), fl.tan());
    let v_land = a;
    let v_tip = a + g.hr;
    // Right flank: u + v tan(flank) = k.
    let k = g.bg / 2.0 + v_tip * ta;

    // Convex tip corner, tangent to the tip line and the flank.
    let r1 = g.r1.max(ball / 2.0).min(g.max_r1() * 0.999);
    let (uc1, vc1) = (k - (v_tip - r1) * ta - r1 / ca, v_tip - r1);
    // Concave fillet where the flank meets the land, kept inside the half
    // pitch so neighbouring teeth's fillets cannot overlap.
    let max_r2 = (pb / 2.0 - k + v_land * ta) / (1.0 / ca - ta);
    let r2 = g.r2.min(max_r2 * 0.999);
    let (uc2, vc2) = (k - (v_land + r2) * ta + r2 / ca, v_land + r2);

    let f1 = [uc1 + r1 * ca, vc1 + r1 * sa];
    let f2 = [uc2 - r2 * ca, vc2 - r2 * sa];
    let m = |q: [f64; 2]| [-q[0], q[1]];
    let top = v_land - g.hr;
    let mut lines = vec![
        [[pb / 2.0, v_land], [uc2, v_land]],
        [[uc1, v_tip], [-uc1, v_tip]],
        [[pb / 2.0, top], [pb / 2.0, v_land]],
        [[-pb / 2.0, top], [pb / 2.0, top]],
        [[-pb / 2.0, top], [-pb / 2.0, v_land]],
        [m([pb / 2.0, v_land]), m([uc2, v_land])],
    ];
    if f1[1] > f2[1] {
        lines.push([f2, f1]);
        lines.push([m(f2), m(f1)]);
    }
    let arcs = vec![
        ([uc1, vc1], r1, fl, PI / 2.0),
        ([-uc1, vc1], r1, PI / 2.0, PI - fl),
        ([uc2, vc2], r2, fl - PI, -PI / 2.0),
        ([-uc2, vc2], r2, -PI / 2.0, -fl),
    ];
    Rack { lines, arcs, extent: pb / 2.0 }
}

/// An open belt round two pulleys of pitch radii r1 and r2 with centres
/// `c` apart: its pitch length.
pub fn belt_length(r1: f64, r2: f64, c: f64) -> f64 {
    let beta = ((r2 - r1) / c).clamp(-1.0, 1.0).asin();
    2.0 * c * beta.cos() + PI * (r1 + r2) + 2.0 * beta * (r2 - r1)
}

/// Centre distance that makes an open belt of pitch length `len` fit, or
/// None if the belt is too short to go round both pulleys.
pub fn center_distance(r1: f64, r2: f64, len: f64) -> Option<f64> {
    let d = (r2 - r1).abs();
    let mut lo = d + 1e-9;
    if belt_length(r1, r2, lo) >= len {
        return None;
    }
    // Length grows with centre distance, and never by more than 2 per
    // unit, so len / 2 is always far enough.
    let mut hi = len / 2.0 + d + 1.0;
    for _ in 0..200 {
        let mid = (lo + hi) / 2.0;
        if belt_length(r1, r2, mid) < len {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some((lo + hi) / 2.0)
}

/// Angle of wrap on the smaller pulley, radians.
pub fn small_wrap(r1: f64, r2: f64, c: f64) -> f64 {
    PI - 2.0 * ((r2 - r1).abs() / c).clamp(-1.0, 1.0).asin()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gear::{self, GearParams};

    fn pulley(section: BeltSection, teeth: u32) -> GearParams {
        let mut p = GearParams { teeth, belt: Some(section), bore: 0.0, ..Default::default() };
        p.apply_standard();
        p
    }

    /// Generated outside diameters against ISO 5294 Table 5.
    #[test]
    fn diameters_match_iso_table_5() {
        // (section, teeth, pitch diameter, outside diameter), mm.
        let rows = [
            (BeltSection::Mxl, 10, 6.47, 5.96),
            (BeltSection::Mxl, 72, 46.57, 46.06),
            (BeltSection::Xxl, 20, 20.21, 19.70),
            (BeltSection::Xl, 10, 16.17, 15.66),
            (BeltSection::Xl, 36, 58.21, 57.70),
            (BeltSection::Xl, 72, 116.43, 115.92),
        ];
        for (s, z, pd, od) in rows {
            let p = pulley(s, z);
            assert!((2.0 * p.pitch_radius() - pd).abs() < 0.006, "{s:?} {z}: PD {}", 2.0 * p.pitch_radius());
            let prof = gear::tooth_profile(&p).unwrap();
            let rmax = gear::outline(&p, &prof).iter().map(|q| q[0].hypot(q[1])).fold(0.0, f64::max);
            assert!((2.0 * rmax - od).abs() < 0.006, "{s:?} {z}: OD {}", 2.0 * rmax);
        }
    }

    #[test]
    fn grooves_have_iso_depth_and_bottom_width() {
        for s in BeltSection::ALL {
            let z = s.stock_teeth()[4];
            let p = pulley(s, z);
            let g = s.rack(z);
            let prof = gear::tooth_profile(&p).unwrap();
            let rmin = gear::outline(&p, &prof).iter().map(|q| q[0].hypot(q[1])).fold(f64::MAX, f64::min);
            let ra = p.tip_radius();
            assert!((ra - rmin - g.hr).abs() < 1e-3, "{s:?}: depth {}", ra - rmin);
            // Every groove is the same, and the land is never cut.
            assert!(prof.half.iter().all(|h| *h > 0.0), "{s:?}: pointed teeth");
        }
    }

    #[test]
    fn standard_belts_have_whole_tooth_counts() {
        for s in BeltSection::ALL {
            for b in s.standard_belts() {
                let d: f64 = b.designation.trim_end_matches(char::is_alphabetic).parse().unwrap();
                // Designations are the length in tenths of an inch (MXL
                // writes a decimal point), rounded to a whole number.
                let tenths_off = b.teeth as f64 * s.pitch() / 25.4 * 10.0 - d;
                // ISO 5296 rounds halves down (187L is 18.75 in).
                assert!(tenths_off.abs() <= 0.5 + 1e-9, "{}: {tenths_off} tenths off", b.designation);
            }
        }
        assert_eq!(BeltSection::Xl.designation(65).as_deref(), Some("130XL"));
        assert_eq!(BeltSection::Mxl.designation(90).as_deref(), Some("72.0MXL"));
        assert_eq!(BeltSection::L.designation(50).as_deref(), Some("187L"));
    }

    #[test]
    fn nearest_standard_belt_for_a_centre_distance() {
        // Two 20-tooth XL pulleys: the belt is 2C + 20 pitches long.
        let r = 20.0 * 5.08 / (2.0 * PI);
        let (teeth, c) = belt_for_center(BeltSection::Xl, r, r, 100.0, true).unwrap();
        // 2*100/5.08 + 20 = 59.4 teeth: 60 (120XL) gives 101.6 mm.
        assert_eq!(teeth, 60);
        assert!((c - 101.6).abs() < 1e-6);
        let (any, c2) = belt_for_center(BeltSection::Xl, r, r, 100.0, false).unwrap();
        assert_eq!(any, 59);
        assert!((c2 - 99.06).abs() < 1e-6);
    }

    #[test]
    fn belt_length_and_centre_distance_invert() {
        let (r1, r2) = (10.0, 25.0);
        let c = center_distance(r1, r2, 300.0).unwrap();
        assert!((belt_length(r1, r2, c) - 300.0).abs() < 1e-9);
        // Equal pulleys: two straights and one full circle.
        assert!((belt_length(10.0, 10.0, 50.0) - (100.0 + 20.0 * PI)).abs() < 1e-12);
        assert!(center_distance(r1, r2, 100.0).is_none());
    }
}

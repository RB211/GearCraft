//! Timing pulleys and belt drives.
//!
//! Pulley grooves are generated the way they are hobbed: by rolling a
//! generating rack on the belt pitch circle, which lies the pitch-line
//! differential `a` outside the pulley's outside diameter. That is also
//! how a belt's straight run enters the pulley, so the groove is the room
//! the entering belt tooth needs.
//!
//! The inch trapezoidal sections use the ISO 5294 racks (Table 1). Those
//! racks are the ISO 5296 belt tooth with its flanks kept and its tip let
//! down by about a tenth of the tooth height. The other sections' groove
//! data are not published in full (ISO 13050, DIN 7721 and the makers'
//! own), so their racks are built from the published belt tooth by that
//! same rule; pitch-line differentials are the makers' catalogue values.

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
    Htd3m,
    Htd5m,
    Htd8m,
    Htd14m,
    Htd20m,
    Gt2,
    Gt3,
    Gt5,
    T2_5,
    T5,
    T10,
    T20,
    At5,
    At10,
    At20,
}

/// Belt sections that share a tooth form and a source.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    /// Inch trapezoidal, ISO 5294 / ISO 5296.
    Iso,
    /// HTD round tooth, ISO 13050 profile H.
    Htd,
    /// Gates PowerGrip GT.
    Gt,
    /// Metric trapezoidal, DIN 7721.
    T,
    /// Metric trapezoidal, wide-tooth AT.
    At,
}

impl Family {
    pub const ALL: [Family; 5] = [Family::Iso, Family::Htd, Family::Gt, Family::T, Family::At];

    pub fn name(self) -> &'static str {
        match self {
            Family::Iso => "Trapezoidal, inch (ISO 5294)",
            Family::Htd => "HTD",
            Family::Gt => "GT",
            Family::T => "T (DIN 7721)",
            Family::At => "AT",
        }
    }

    /// Where the groove comes from, for the Teeth panel.
    pub fn groove_note(self) -> &'static str {
        match self {
            Family::Iso => "ISO 5294 groove.",
            Family::Htd => "Groove from the ISO 13050 HTD belt tooth, a tenth deeper, as ISO 5294 does for its belts. \
                 No maker's groove data.",
            Family::Gt => "Groove from Gates' published GT belt tooth, a tenth deeper, as ISO 5294 does for its belts. \
                 Gates' own groove is not published.",
            Family::T | Family::At => "Groove from the belt tooth, a tenth deeper, as ISO 5294 does for its belts. \
                 Not checked against DIN 7721's groove table.",
        }
    }
}

/// How much deeper than the belt tooth the generating rack reaches, as a
/// fraction of the tooth height: ISO 5294's racks give 8 to 13 % (MXL 25 %).
const TIP_CLEARANCE: f64 = 0.1;

/// The generating rack for one section (mm, degrees).
#[derive(Clone, Copy, Debug)]
pub struct GenRack {
    /// Rack tooth height: the groove depth.
    pub hr: f64,
    /// Pitch-line differential 2a: pitch diameter minus outside diameter.
    pub pld: f64,
    pub form: RackForm,
}

#[derive(Clone, Copy, Debug)]
pub enum RackForm {
    /// Straight flanks and rounded corners, as ISO 5294 Table 1 gives
    /// them. HTD racks take this form with the tip a full round.
    Straight {
        /// Half the included angle of the rack tooth.
        flank: f64,
        /// Rack tooth width at its tip: the groove bottom width.
        bg: f64,
        /// Rack tooth tip corner radius (rounds the bottom of the groove).
        r1: f64,
        /// Rack root fillet radius (rounds the pulley tooth tips).
        r2: f64,
    },
    /// The GT tooth: a tip arc between two flank arcs whose centres sit
    /// `offset` across the tooth's centreline, `lift` below the lands.
    /// Above the flank centres the flanks run straight down to the lands.
    Gt { tip: f64, flank: f64, offset: f64, fillet: f64, lift: f64 },
}

impl GenRack {
    /// The groove-bottom radius the standard gives.
    pub fn bottom_radius(&self) -> f64 {
        match self.form {
            RackForm::Straight { r1, .. } => r1,
            RackForm::Gt { tip, .. } => tip,
        }
    }

    /// The biggest groove-bottom radius: for straight flanks the rack tip a
    /// full round; a GT groove bottom can't grow without leaving its flanks.
    pub fn max_r1(&self) -> f64 {
        match self.form {
            RackForm::Straight { flank, bg, .. } => {
                let (sa, ca) = flank.to_radians().sin_cos();
                bg / 2.0 * ca / (1.0 - sa)
            }
            RackForm::Gt { tip, .. } => tip,
        }
    }
}

/// A standard belt width and the narrowest pulley that carries it, with
/// flanges and without.
#[derive(Clone, Copy, Debug)]
pub struct BeltWidth {
    /// ISO 5294's width code; empty for the metric sections.
    pub code: &'static str,
    pub width: f64,
    pub flanged: f64,
    pub unflanged: f64,
}

const fn bw(code: &'static str, width: f64, flanged: f64, unflanged: f64) -> BeltWidth {
    BeltWidth { code, width, flanged, unflanged }
}

/// A metric belt width. No standard gives their pulley faces, so they
/// follow ISO 5294 Table 4's margins: about 0.15 of a pitch over the belt
/// between flanges, half a pitch over it without.
const fn mw(width: f64, pitch: f64) -> BeltWidth {
    BeltWidth { code: "", width, flanged: width + 0.15 * pitch, unflanged: width + 0.5 * pitch }
}

// ISO 5294 Table 4.
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

// Stock widths of the metric sections, mm, from makers' catalogues.
const HTD3M_WIDTHS: [BeltWidth; 3] = [mw(6.0, 3.0), mw(9.0, 3.0), mw(15.0, 3.0)];
const HTD5M_WIDTHS: [BeltWidth; 3] = [mw(9.0, 5.0), mw(15.0, 5.0), mw(25.0, 5.0)];
const HTD8M_WIDTHS: [BeltWidth; 4] = [mw(20.0, 8.0), mw(30.0, 8.0), mw(50.0, 8.0), mw(85.0, 8.0)];
const HTD14M_WIDTHS: [BeltWidth; 5] = [mw(40.0, 14.0), mw(55.0, 14.0), mw(85.0, 14.0), mw(115.0, 14.0), mw(170.0, 14.0)];
const HTD20M_WIDTHS: [BeltWidth; 5] =
    [mw(115.0, 20.0), mw(170.0, 20.0), mw(230.0, 20.0), mw(290.0, 20.0), mw(340.0, 20.0)];
const GT2_WIDTHS: [BeltWidth; 3] = [mw(3.0, 2.0), mw(6.0, 2.0), mw(9.0, 2.0)];
const GT3_WIDTHS: [BeltWidth; 3] = [mw(6.0, 3.0), mw(9.0, 3.0), mw(15.0, 3.0)];
const GT5_WIDTHS: [BeltWidth; 3] = [mw(9.0, 5.0), mw(15.0, 5.0), mw(25.0, 5.0)];
const T2_5_WIDTHS: [BeltWidth; 3] = [mw(4.0, 2.5), mw(6.0, 2.5), mw(10.0, 2.5)];
const T5_WIDTHS: [BeltWidth; 3] = [mw(10.0, 5.0), mw(16.0, 5.0), mw(25.0, 5.0)];
const T10_WIDTHS: [BeltWidth; 4] = [mw(16.0, 10.0), mw(25.0, 10.0), mw(32.0, 10.0), mw(50.0, 10.0)];
const T20_WIDTHS: [BeltWidth; 4] = [mw(32.0, 20.0), mw(50.0, 20.0), mw(75.0, 20.0), mw(100.0, 20.0)];
const AT5_WIDTHS: [BeltWidth; 3] = [mw(10.0, 5.0), mw(16.0, 5.0), mw(25.0, 5.0)];
const AT10_WIDTHS: [BeltWidth; 4] = [mw(16.0, 10.0), mw(25.0, 10.0), mw(32.0, 10.0), mw(50.0, 10.0)];
const AT20_WIDTHS: [BeltWidth; 4] = [mw(32.0, 20.0), mw(50.0, 20.0), mw(75.0, 20.0), mw(100.0, 20.0)];

/// Tooth counts listed in ISO 5294 Table 5 (non-preferred sizes left out).
const PREFERRED_TEETH: [u32; 24] =
    [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 25, 28, 32, 36, 40, 48, 60, 72, 84, 96, 120, 156];

/// Tooth counts the makers stock in the metric sections.
const METRIC_TEETH: [u32; 36] = [
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 24, 25, 26, 28, 30, 32, 34, 36, 38, 40, 44, 48, 56, 60, 64,
    72, 80, 90, 112, 144, 168, 192, 216,
];

impl BeltSection {
    pub const ALL: [BeltSection; 22] = [
        BeltSection::Mxl,
        BeltSection::Xxl,
        BeltSection::Xl,
        BeltSection::L,
        BeltSection::H,
        BeltSection::Xh,
        BeltSection::Xxh,
        BeltSection::Htd3m,
        BeltSection::Htd5m,
        BeltSection::Htd8m,
        BeltSection::Htd14m,
        BeltSection::Htd20m,
        BeltSection::Gt2,
        BeltSection::Gt3,
        BeltSection::Gt5,
        BeltSection::T2_5,
        BeltSection::T5,
        BeltSection::T10,
        BeltSection::T20,
        BeltSection::At5,
        BeltSection::At10,
        BeltSection::At20,
    ];

    pub fn family(self) -> Family {
        use BeltSection::*;
        match self {
            Mxl | Xxl | Xl | L | H | Xh | Xxh => Family::Iso,
            Htd3m | Htd5m | Htd8m | Htd14m | Htd20m => Family::Htd,
            Gt2 | Gt3 | Gt5 => Family::Gt,
            T2_5 | T5 | T10 | T20 => Family::T,
            At5 | At10 | At20 => Family::At,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            BeltSection::Mxl => "MXL",
            BeltSection::Xxl => "XXL",
            BeltSection::Xl => "XL",
            BeltSection::L => "L",
            BeltSection::H => "H",
            BeltSection::Xh => "XH",
            BeltSection::Xxh => "XXH",
            BeltSection::Htd3m => "3M",
            BeltSection::Htd5m => "5M",
            BeltSection::Htd8m => "8M",
            BeltSection::Htd14m => "14M",
            BeltSection::Htd20m => "20M",
            BeltSection::Gt2 => "2GT",
            BeltSection::Gt3 => "3GT",
            BeltSection::Gt5 => "5GT",
            BeltSection::T2_5 => "T2.5",
            BeltSection::T5 => "T5",
            BeltSection::T10 => "T10",
            BeltSection::T20 => "T20",
            BeltSection::At5 => "AT5",
            BeltSection::At10 => "AT10",
            BeltSection::At20 => "AT20",
        }
    }

    /// A pulley's short name: 14XL for the inch sections, as ISO 5294
    /// writes them; the metric ones' names start with a digit, so 14-5M.
    pub fn pulley_name(self, teeth: u32) -> String {
        match self.family() {
            Family::Iso => format!("{teeth}{}", self.name()),
            _ => format!("{teeth}-{}", self.name()),
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
            _ => return match self.family() {
                Family::Htd => format!("HTD {} — {} mm pitch", self.name(), self.pitch()),
                _ => format!("{} — {} mm pitch", self.name(), self.pitch()),
            },
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
            BeltSection::Htd3m | BeltSection::Gt3 => 3.0,
            BeltSection::Htd5m | BeltSection::Gt5 | BeltSection::T5 | BeltSection::At5 => 5.0,
            BeltSection::Htd8m => 8.0,
            BeltSection::Htd14m => 14.0,
            BeltSection::Htd20m | BeltSection::T20 | BeltSection::At20 => 20.0,
            BeltSection::Gt2 => 2.0,
            BeltSection::T2_5 => 2.5,
            BeltSection::T10 | BeltSection::At10 => 10.0,
        }
    }

    /// Fewest teeth the standard gives a rack for, or the makers stock.
    pub fn min_teeth(self) -> u32 {
        match self {
            BeltSection::H => 14,
            BeltSection::Xh | BeltSection::Xxh => 18,
            BeltSection::Htd5m | BeltSection::Gt2 | BeltSection::T10 | BeltSection::At5 => 12,
            BeltSection::Htd8m => 22,
            BeltSection::Htd14m => 28,
            BeltSection::Htd20m => 34,
            BeltSection::Gt3 | BeltSection::T20 | BeltSection::At10 => 16,
            BeltSection::Gt5 | BeltSection::At20 => 18,
            _ => 10,
        }
    }

    /// Who sets `min_teeth`, for warnings.
    pub fn min_teeth_source(self) -> &'static str {
        if self.family() == Family::Iso { "ISO 5294 gives" } else { "Makers list" }
    }

    /// Pitch-line differential 2a, mm. ISO 5294's are in its racks; the
    /// rest are PD − OD from Gates' and Optibelt's pulley tables (T20 and
    /// AT20, which neither lists, from their belts' backing like T10's).
    pub fn pld(self) -> f64 {
        match self {
            BeltSection::Htd3m | BeltSection::Gt3 => 0.762,
            BeltSection::Htd5m | BeltSection::Gt5 => 1.143,
            BeltSection::Htd8m => 1.372,
            BeltSection::Htd14m => 2.794,
            BeltSection::Htd20m => 4.32,
            BeltSection::Gt2 => 0.508,
            BeltSection::T2_5 => 0.55,
            BeltSection::T5 => 0.85,
            BeltSection::T10 | BeltSection::At10 => 1.85,
            BeltSection::At5 => 1.25,
            BeltSection::T20 | BeltSection::At20 => 2.85,
            iso => iso.rack(iso.min_teeth()).pld,
        }
    }

    /// ISO 5294 Table 1 for the inch sections (MXL and H change their rack
    /// for small pulleys); the others' racks are their belt tooth let down.
    pub fn rack(self, teeth: u32) -> GenRack {
        let g = |flank, hr, bg, r1, r2, pld| GenRack { hr, pld, form: RackForm::Straight { flank, bg, r1, r2 } };
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
            _ => self.belt_tooth().rack(self.pld()),
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
            BeltSection::Htd3m => &HTD3M_WIDTHS,
            BeltSection::Htd5m => &HTD5M_WIDTHS,
            BeltSection::Htd8m => &HTD8M_WIDTHS,
            BeltSection::Htd14m => &HTD14M_WIDTHS,
            BeltSection::Htd20m => &HTD20M_WIDTHS,
            BeltSection::Gt2 => &GT2_WIDTHS,
            BeltSection::Gt3 => &GT3_WIDTHS,
            BeltSection::Gt5 => &GT5_WIDTHS,
            BeltSection::T2_5 => &T2_5_WIDTHS,
            BeltSection::T5 => &T5_WIDTHS,
            BeltSection::T10 => &T10_WIDTHS,
            BeltSection::T20 => &T20_WIDTHS,
            BeltSection::At5 => &AT5_WIDTHS,
            BeltSection::At10 => &AT10_WIDTHS,
            BeltSection::At20 => &AT20_WIDTHS,
        }
    }

    /// The standard width nearest `w` mm.
    pub fn width_near(self, w: f64) -> BeltWidth {
        *self.widths().iter().min_by(|a, b| (a.width - w).abs().total_cmp(&(b.width - w).abs())).unwrap()
    }

    /// Preferred tooth counts for this section. Table 5 stops at 72 teeth
    /// for the three small sections; makers' metric lists run longer the
    /// bigger the pitch.
    pub fn stock_teeth(self) -> Vec<u32> {
        let (list, max): (&[u32], u32) = match self {
            BeltSection::Mxl | BeltSection::Xxl | BeltSection::Xl => (&PREFERRED_TEETH, 72),
            _ if self.family() == Family::Iso => (&PREFERRED_TEETH, 156),
            _ if self.pitch() <= 3.0 => (&METRIC_TEETH, 72),
            _ if self.pitch() <= 5.0 => (&METRIC_TEETH, 112),
            _ => (&METRIC_TEETH, 216),
        };
        list.iter().copied().filter(|&z| z >= self.min_teeth() && z <= max).collect()
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
    /// XXL is not in the 1989 tables, and the metric sections are made in
    /// whatever lengths their makers choose, so they have none.
    pub fn standard_belts(self) -> Vec<StdBelt> {
        let list: &[u32] = match self {
            BeltSection::Mxl => &MXL_BELTS,
            BeltSection::Xl => &XL_BELTS,
            BeltSection::L => &L_BELTS,
            BeltSection::H => &H_BELTS,
            BeltSection::Xh => &XH_BELTS,
            BeltSection::Xxh => &XXH_BELTS,
            _ => &[],
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

    /// The designation of a belt with this many teeth: an inch belt's if
    /// it is standard; a metric belt is named for its pitch length in mm
    /// (400-5M, T5/400), whether or not a maker stocks it.
    pub fn designation(self, teeth: u32) -> Option<String> {
        let mm = teeth as f64 * self.pitch();
        match self.family() {
            Family::Iso => self.standard_belts().into_iter().find(|b| b.teeth == teeth).map(|b| b.designation),
            Family::Htd | Family::Gt => Some(format!("{mm}-{}", self.name())),
            Family::T | Family::At => Some(format!("{}/{mm}", self.name())),
        }
    }
}

/// A belt's cross-section along its length, mm: overall thickness, tooth
/// height, the tooth's width where it meets the land, the fillet there
/// and the shape of the rest. ISO 5296-1 Table 1 for the inch sections;
/// ISO 13050 (HTD), Gates (GT) and DIN 7721 (T) for the others.
#[derive(Clone, Copy, Debug)]
pub struct BeltTooth {
    pub thickness: f64,
    pub height: f64,
    pub width: f64,
    pub fillet: f64,
    pub form: ToothForm,
}

#[derive(Clone, Copy, Debug)]
pub enum ToothForm {
    /// Straight flanks at `half_angle` degrees, tip corners rounded.
    Trapezoid { half_angle: f64, tip: f64 },
    /// HTD: a round tip of this radius, the flanks straight lines from it
    /// down to the land.
    Round { radius: f64 },
    /// GT: flank arcs of radius `flank` centred on the land, `offset`
    /// across the centreline, and a tip arc tangent to both.
    Gt { flank: f64, offset: f64 },
}

impl BeltSection {
    pub fn belt_tooth(self) -> BeltTooth {
        let trap = |thickness, height, width, half_angle, tip, fillet| BeltTooth {
            thickness,
            height,
            width,
            fillet,
            form: ToothForm::Trapezoid { half_angle, tip },
        };
        let t = |thickness, height, width, half_angle| trap(thickness, height, width, half_angle, 0.0, 0.0);
        let htd = |thickness, height, width, radius, fillet| BeltTooth {
            thickness,
            height,
            width,
            fillet,
            form: ToothForm::Round { radius },
        };
        let gt = |thickness, height, flank: f64, offset: f64, fillet| BeltTooth {
            thickness,
            height,
            width: 2.0 * (flank - offset),
            fillet,
            form: ToothForm::Gt { flank, offset },
        };
        match self {
            BeltSection::Mxl => t(1.14, 0.51, 1.14, 20.0),
            BeltSection::Xxl => t(1.52, 0.76, 1.73, 25.0),
            BeltSection::Xl => t(2.30, 1.27, 2.57, 25.0),
            BeltSection::L => t(3.60, 1.91, 4.65, 20.0),
            BeltSection::H => t(4.30, 2.29, 6.12, 20.0),
            BeltSection::Xh => t(11.20, 6.35, 12.57, 20.0),
            BeltSection::Xxh => t(15.70, 9.53, 19.05, 20.0),
            // ISO 13050 profile H: tooth height, foot (fillet) and head
            // radii. The 5M and 8M widths are ISO's; 3M and 14M are
            // makers' figures; 20M's is 8M's in proportion.
            BeltSection::Htd3m => htd(2.40, 1.17, 1.78, 0.87, 0.26),
            BeltSection::Htd5m => htd(3.80, 2.06, 3.05, 1.49, 0.42),
            BeltSection::Htd8m => htd(6.00, 3.38, 5.15, 2.46, 0.70),
            BeltSection::Htd14m => htd(10.00, 6.02, 9.40, 4.50, 1.25),
            BeltSection::Htd20m => htd(13.20, 8.40, 13.60, 6.50, 1.90),
            // Gates' GT2 profile (R1, R2, b, h as Misumi lists them).
            BeltSection::Gt2 => gt(1.38, 0.75, 1.00, 0.40, 0.15),
            BeltSection::Gt3 => gt(2.40, 1.14, 1.52, 0.61, 0.25),
            BeltSection::Gt5 => gt(3.80, 1.93, 2.54, 1.03, 0.51),
            // DIN 7721-1: 40° teeth. AT teeth are 50°.
            BeltSection::T2_5 => trap(1.30, 0.70, 1.50, 20.0, 0.2, 0.3),
            BeltSection::T5 => trap(2.20, 1.20, 2.65, 20.0, 0.4, 0.6),
            BeltSection::T10 => trap(4.50, 2.50, 5.30, 20.0, 0.6, 0.8),
            BeltSection::T20 => trap(8.00, 5.00, 10.15, 20.0, 0.8, 1.2),
            BeltSection::At5 => trap(2.70, 1.20, 2.50, 25.0, 0.4, 0.6),
            BeltSection::At10 => trap(4.50, 2.50, 5.00, 25.0, 0.6, 0.8),
            BeltSection::At20 => trap(8.00, 5.00, 10.00, 25.0, 0.8, 1.2),
        }
    }
}

impl BeltTooth {
    /// The angle an HTD tooth's straight flanks lean in from square to the
    /// land, radians: the line from the tooth's foot tangent to its head.
    fn round_flank(&self, radius: f64) -> f64 {
        // (w/2) cos φ − (h − R) sin φ = R.
        let (a, b) = (self.width / 2.0, self.height - radius);
        (radius / a.hypot(b)).clamp(-1.0, 1.0).acos() - b.atan2(a)
    }

    /// The GT tip radius that is tangent to both flank arcs and reaches
    /// the tooth height. Gates' rounded figures agree with it to 0.03 mm.
    fn gt_tip(&self, flank: f64, offset: f64) -> f64 {
        let h = self.height;
        (flank * flank - offset * offset - h * h) / (2.0 * (flank - h))
    }

    /// The generating rack: the tooth with its flanks kept and its tip let
    /// down by a tenth of its height, as ISO 5294 does.
    fn rack(&self, pld: f64) -> GenRack {
        let c = TIP_CLEARANCE * self.height;
        let hr = self.height + c;
        let straight = |flank: f64, r1: Option<f64>| {
            let bg = self.width - 2.0 * hr * flank.tan();
            let form = |r1| RackForm::Straight { flank: flank.to_degrees(), bg, r1, r2: self.fillet };
            let full = GenRack { hr, pld, form: form(0.0) }.max_r1();
            GenRack { hr, pld, form: form(r1.unwrap_or(full)) }
        };
        match self.form {
            ToothForm::Trapezoid { half_angle, tip } => straight(half_angle.to_radians(), Some(tip)),
            // A round head between straight flanks: let down, it is still
            // the full round between them, a shade smaller.
            ToothForm::Round { radius } => straight(self.round_flank(radius), None),
            ToothForm::Gt { flank, offset } => GenRack {
                hr,
                pld,
                form: RackForm::Gt { tip: self.gt_tip(flank, offset), flank, offset, fillet: self.fillet, lift: c },
            },
        }
    }

    /// The tooth as drawn: (along the belt, depth below the land) from the
    /// left foot round the tip to the right foot, fillets left out.
    pub fn outline(&self) -> Vec<[f64; 2]> {
        let (w, h) = (self.width / 2.0, self.height);
        let arc = |c: [f64; 2], r: f64, t0: f64, t1: f64, out: &mut Vec<[f64; 2]>| {
            for k in 0..=12 {
                let t = t0 + (t1 - t0) * k as f64 / 12.0;
                out.push([c[0] + r * t.cos(), c[1] + r * t.sin()]);
            }
        };
        let mut out = vec![[-w, 0.0]];
        match self.form {
            ToothForm::Trapezoid { half_angle, tip: r } => {
                let al = half_angle.to_radians();
                let tip = (w - h * al.tan()).max(0.05);
                let r = r.min(0.999 * tip * al.cos() / (1.0 - al.sin()));
                if r > 0.0 {
                    let xc = w - (h - r) * al.tan() - r / al.cos();
                    arc([-xc, h - r], r, PI - al, PI / 2.0, &mut out);
                    arc([xc, h - r], r, PI / 2.0, al, &mut out);
                } else {
                    out.extend([[-tip, h], [tip, h]]);
                }
            }
            ToothForm::Round { radius } => {
                let phi = self.round_flank(radius);
                arc([0.0, h - radius], radius, PI - phi, phi, &mut out);
            }
            ToothForm::Gt { flank, offset } => {
                let tip = self.gt_tip(flank, offset);
                let theta = (h - tip).atan2(offset);
                arc([offset, 0.0], flank, PI, PI - theta, &mut out);
                arc([0.0, h - tip], tip, PI - theta, theta, &mut out);
                arc([-offset, 0.0], flank, theta, 0.0, &mut out);
            }
        }
        out.push([w, 0.0]);
        out.dedup_by(|a, b| (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9);
        out
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
/// exactly on the pulley's outside diameter. Straight-flanked grooves have
/// their bottom corners rounded for a ball end mill of Ø `ball` where the
/// standard's are tighter; a GT groove bottom is what it is.
pub(crate) fn rack(section: BeltSection, teeth: u32, ball: f64) -> Rack {
    let g = section.rack(teeth);
    let pb = section.pitch();
    let a = g.pld / 2.0;
    let top = a - g.hr;
    let m = |q: [f64; 2]| [-q[0], q[1]];
    // The closing edges above the lands, well clear of the blank.
    let mut lines = vec![[[pb / 2.0, top], [pb / 2.0, a]], [[-pb / 2.0, top], [pb / 2.0, top]], [[-pb / 2.0, top], [-pb / 2.0, a]]];
    let mut arcs = Vec::new();
    match g.form {
        RackForm::Straight { flank, bg, r1, r2 } => {
            let fl = flank.to_radians();
            let (sa, ca, ta) = (fl.sin(), fl.cos(), fl.tan());
            let v_land = a;
            let v_tip = a + g.hr;
            // Right flank: u + v tan(flank) = k.
            let k = bg / 2.0 + v_tip * ta;

            // Convex tip corner, tangent to the tip line and the flank.
            let r1 = r1.max(ball / 2.0).min(g.max_r1() * 0.999);
            let (uc1, vc1) = (k - (v_tip - r1) * ta - r1 / ca, v_tip - r1);
            // Concave fillet where the flank meets the land, kept inside
            // the half pitch so neighbouring teeth's fillets cannot overlap.
            let max_r2 = (pb / 2.0 - k + v_land * ta) / (1.0 / ca - ta);
            let r2 = r2.min(max_r2 * 0.999);
            let (uc2, vc2) = (k - (v_land + r2) * ta + r2 / ca, v_land + r2);

            let f1 = [uc1 + r1 * ca, vc1 + r1 * sa];
            let f2 = [uc2 - r2 * ca, vc2 - r2 * sa];
            lines.extend([[[pb / 2.0, v_land], [uc2, v_land]], [[uc1, v_tip], [-uc1, v_tip]], [m([pb / 2.0, v_land]), m([uc2, v_land])]]);
            if f1[1] > f2[1] {
                lines.push([f2, f1]);
                lines.push([m(f2), m(f1)]);
            }
            arcs.extend([
                ([uc1, vc1], r1, fl, PI / 2.0),
                ([-uc1, vc1], r1, PI / 2.0, PI - fl),
                ([uc2, vc2], r2, fl - PI, -PI / 2.0),
                ([-uc2, vc2], r2, -PI / 2.0, -fl),
            ]);
        }
        RackForm::Gt { tip, flank, offset, fillet, lift } => {
            // The right flank's arc is centred left of the centreline; it
            // is square to the lands level with its centre, and runs
            // straight up from there to them.
            let fc = [-offset, a + lift];
            let tc = [0.0, a + g.hr - tip];
            let theta = (tc[1] - fc[1]).atan2(tc[0] - fc[0]);
            let side = flank - offset;
            // The land fillet: on the straight run if it is short enough
            // to fit, otherwise tangent to the arc.
            let (qc, t_top, t_fillet) = if fillet <= lift {
                lines.push([[side, a + fillet], [side, a + lift]]);
                lines.push([m([side, a + fillet]), m([side, a + lift])]);
                ([side + fillet, a + fillet], 0.0, -PI)
            } else {
                let (dv, du) = (fillet - lift, ((flank + fillet).powi(2) - (fillet - lift).powi(2)).sqrt());
                ([fc[0] + du, a + fillet], dv.atan2(du), (-dv).atan2(-du))
            };
            lines.push([[pb / 2.0, a], [qc[0], a]]);
            lines.push([m([pb / 2.0, a]), m([qc[0], a])]);
            arcs.extend([
                (fc, flank, t_top, theta),
                (m(fc), flank, PI - theta, PI - t_top),
                (tc, tip, theta, PI - theta),
                (qc, fillet, t_fillet, -PI / 2.0),
                (m(qc), fillet, -PI / 2.0, -PI - t_fillet),
            ]);
        }
    }
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

    /// Metric outside diameters against Optibelt's and Gates' pulley
    /// tables: (section, teeth, pitch diameter, outside diameter), mm.
    #[test]
    fn metric_diameters_match_catalogues() {
        let rows = [
            (BeltSection::Htd3m, 10, 9.55, 8.79),
            (BeltSection::Htd5m, 12, 19.10, 17.96),
            (BeltSection::Htd5m, 28, 44.56, 43.42),
            (BeltSection::Htd5m, 150, 238.73, 237.59),
            (BeltSection::Htd8m, 22, 56.02, 54.65),
            (BeltSection::Htd14m, 48, 213.90, 211.11),
            (BeltSection::Gt2, 20, 12.73, 12.22),
            (BeltSection::T5, 10, 15.92, 15.07),
            (BeltSection::T10, 12, 38.20, 36.35),
            (BeltSection::At5, 16, 25.47, 24.22),
            (BeltSection::At10, 20, 63.66, 61.81),
        ];
        for (s, z, pd, od) in rows {
            let p = pulley(s, z);
            assert!((2.0 * p.pitch_radius() - pd).abs() < 0.006, "{s:?} {z}: PD {}", 2.0 * p.pitch_radius());
            let prof = gear::tooth_profile(&p).unwrap();
            let rmax = gear::outline(&p, &prof).iter().map(|q| q[0].hypot(q[1])).fold(0.0, f64::max);
            assert!((2.0 * rmax - od).abs() < 0.03, "{s:?} {z}: OD {}", 2.0 * rmax);
        }
    }

    /// The rule the metric racks are built by, read off ISO's own: each
    /// ISO 5294 rack is its ISO 5296 belt tooth with the flanks kept and
    /// the tip let down 8 to 13 % of the tooth height (MXL's 25 %).
    #[test]
    fn iso_racks_are_their_belt_tooth_let_down() {
        for s in BeltSection::ALL.into_iter().filter(|s| s.family() == Family::Iso) {
            let z = s.stock_teeth().last().copied().unwrap();
            let (g, t) = (s.rack(z), s.belt_tooth());
            let RackForm::Straight { flank, bg, .. } = g.form else { panic!() };
            let ToothForm::Trapezoid { half_angle, .. } = t.form else { panic!() };
            assert_eq!(flank, half_angle, "{s:?}");
            let let_down = g.hr - t.height;
            let belt_tip = t.width - 2.0 * t.height * flank.to_radians().tan();
            let rack_there = bg + 2.0 * let_down * flank.to_radians().tan();
            assert!((belt_tip - rack_there).abs() < 0.06, "{s:?}: belt tip {belt_tip}, rack {rack_there}");
            let frac = let_down / t.height;
            assert!((0.07..0.26).contains(&frac), "{s:?}: let down {frac}");
        }
    }

    /// Every belt tooth, wrapped round the pitch circle as it runs, sits in
    /// its groove without touching the pulley, from the fewest teeth up.
    /// (ISO's sections are left out: their belts' tip radii aren't here,
    /// and the grooves' corners are rounder than a sharp tip.)
    #[test]
    fn seated_belt_teeth_clear_the_grooves() {
        for s in BeltSection::ALL.into_iter().filter(|s| s.family() != Family::Iso) {
            for z in [s.min_teeth(), 60] {
                let p = pulley(s, z);
                let prof = gear::tooth_profile(&p).unwrap();
                let (rp, a) = (p.pitch_radius(), s.rack(z).pld / 2.0);
                let half_at = |rho: f64| {
                    let i = prof.radii.partition_point(|&r| r < rho).clamp(1, prof.radii.len() - 1);
                    let (r0, r1) = (prof.radii[i - 1], prof.radii[i]);
                    let t = ((rho - r0) / (r1 - r0)).clamp(0.0, 1.0);
                    prof.half[i - 1] + (prof.half[i] - prof.half[i - 1]) * t
                };
                for q in s.belt_tooth().outline() {
                    let rho = rp - a - q[1];
                    if rho >= p.tip_radius() - 1e-9 {
                        continue;
                    }
                    let gap = PI / z as f64 - (q[0] / rp).abs() - half_at(rho);
                    assert!(gap * rho > -1e-3, "{s:?} {z}: tooth point {q:?} {:.4} mm into the pulley", -gap * rho);
                }
            }
        }
    }

    #[test]
    fn gt_tip_matches_gates_radius() {
        for (s, r3) in [(BeltSection::Gt2, 0.555), (BeltSection::Gt3, 0.85), (BeltSection::Gt5, 1.37)] {
            let RackForm::Gt { tip, .. } = s.rack(30).form else { panic!() };
            assert!((tip - r3).abs() < 0.03, "{s:?}: tip {tip}");
        }
    }

    #[test]
    fn htd_groove_bottom_is_near_the_head_radius() {
        for s in BeltSection::ALL.into_iter().filter(|s| s.family() == Family::Htd) {
            let ToothForm::Round { radius } = s.belt_tooth().form else { panic!() };
            let g = s.rack(40);
            assert!((g.bottom_radius() - radius).abs() < 0.04 * radius, "{s:?}: {}", g.bottom_radius());
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

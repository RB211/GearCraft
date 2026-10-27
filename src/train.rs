//! The project: a gear train as a list of gears and timing pulleys, each
//! either fixed in space (a root), meshing with an earlier gear, belted to
//! an earlier pulley, or sharing an earlier gear's axle (a compound).
//! Parents always come before their children in `gears`, so the train can
//! be solved in one pass and can never loop.

use crate::gear::{self, GearParams, Issue, MeshInfo, ToothStandard};
use crate::bevel::{self, Bevel, BevelGeom};
use crate::hypoid::{self, Design, Role};
use crate::pulley::{self, BeltSection};
use crate::worm;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::ops::RangeInclusive;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Placement {
    /// Fixed centre. Roots turn at the driver speed.
    Root { x: f64, y: f64, phase: f64 },
    /// Meshes with `parent`; `angle` (degrees) is the direction from the
    /// parent's centre to this one.
    Mesh { parent: u64, angle: f64 },
    /// On the same arbor as `parent`, turning with it; `phase` in degrees.
    Axle { parent: u64, phase: f64 },
    /// A pulley driven from pulley `parent` by an open belt of
    /// `belt_teeth` teeth; the belt sets the centre distance along
    /// `angle`. `target` is the centre distance last asked for (0 if
    /// none), kept to show how close the chosen belt came.
    Belt {
        parent: u64,
        angle: f64,
        belt_teeth: u32,
        #[serde(default)]
        target: f64,
    },
    /// A bevel gear meshing with bevel `parent`: their apexes meet and
    /// this gear's axis is `shaft_angle` degrees from the parent's, leaning
    /// towards azimuth `angle` round the parent's axis.
    Bevel { parent: u64, angle: f64, shaft_angle: f64 },
    /// A hypoid (or, with no offset, spiral bevel) ring gear driven by
    /// hypoid pinion `parent`. The pinion's shaft passes `offset` mm from
    /// this gear's axis at `shaft_angle` degrees to it; `spiral` is the
    /// pinion's mean spiral angle (degrees). `angle` turns this gear round
    /// the pinion's axis. A cutter radius of 0 picks one to suit.
    Hypoid {
        parent: u64,
        angle: f64,
        offset: f64,
        shaft_angle: f64,
        spiral: f64,
        #[serde(default)]
        reverse_hand: bool,
        #[serde(default)]
        cutter_radius: f64,
    },
    /// The wheel of a worm drive turned by worm `parent`: its axis square
    /// to the worm's, `diameter` the worm's pitch diameter (at the throat
    /// of a globoid worm, an hourglass wrapping round the wheel).
    /// `angle` turns the wheel round the worm's axis. `friction` is the
    /// flanks' running friction, which decides whether the wheel can
    /// drive the worm back.
    Worm {
        parent: u64,
        angle: f64,
        diameter: f64,
        throated: bool,
        #[serde(default)]
        globoid: bool,
        left_hand: bool,
        #[serde(default = "default_friction")]
        friction: f64,
    },
}

impl Placement {
    /// Whether this part turns against its parent (meshing or belted) rather
    /// than sitting fixed or sharing its axle.
    pub fn meshes(&self) -> bool {
        !matches!(self, Placement::Root { .. } | Placement::Axle { .. })
    }

    pub fn parent(&self) -> Option<u64> {
        match *self {
            Placement::Root { .. } => None,
            Placement::Mesh { parent, .. }
            | Placement::Axle { parent, .. }
            | Placement::Belt { parent, .. }
            | Placement::Bevel { parent, .. }
            | Placement::Hypoid { parent, .. }
            | Placement::Worm { parent, .. } => Some(parent),
        }
    }
}

/// How a new gear is attached to its parent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Link {
    Mesh,
    Axle,
    Belt,
    Bevel,
    Hypoid,
    Worm,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Gear {
    pub id: u64,
    pub name: String,
    pub params: GearParams,
    pub placement: Placement,
    /// Keep module and pressure angle in step with the meshing parent.
    #[serde(default = "yes")]
    pub match_parent: bool,
    /// Along the axis, mm: from the parent's face (a gear on the same
    /// shaft starts where its parent ends; a meshing or belted one is
    /// centred on its partner's face), or from z = 0 for a fixed part.
    #[serde(default)]
    pub axial: f64,
    /// Where the part's origin goes when it is exported on its own.
    #[serde(default)]
    pub export_origin: ExportOrigin,
}

/// A point on a part's axis, for the origin of a part exported on its
/// own: a datum, moved `offset` mm along the axis (+Z as exported).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ExportOrigin {
    pub datum: Datum,
    pub offset: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Datum {
    /// The part's own origin: a cone gear's pitch apex, or else the
    /// bottom face.
    #[default]
    Own,
    Bottom,
    Middle,
    Top,
}

impl ExportOrigin {
    /// Height of the origin in the part's own frame, given the heights of
    /// its two end faces there (`faces`, low then high).
    pub fn height(&self, faces: [f64; 2], cone: bool) -> f64 {
        let base = match self.datum {
            Datum::Own if cone => 0.0,
            Datum::Own | Datum::Bottom => faces[0],
            Datum::Middle => (faces[0] + faces[1]) / 2.0,
            Datum::Top => faces[1],
        };
        base + self.offset
    }
}

fn yes() -> bool {
    true
}

/// Preferred modules (ISO 54, first and second choice).
pub const STD_MODULES: [f64; 20] =
    [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0];
/// Diametral pitches stocked by the usual inch gear suppliers.
pub const STD_DPS: [f64; 25] = [
    1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 24.0, 32.0, 40.0, 48.0,
    64.0, 72.0, 80.0, 96.0, 120.0,
];

/// The entry of `list` closest to `v` on a log scale (pitch steps are
/// roughly geometric).
fn nearest(list: &[f64], v: f64) -> f64 {
    list.iter().copied().min_by(|a, b| (a / v).ln().abs().total_cmp(&(b / v).ln().abs())).unwrap_or(v)
}

/// The project's unit system. Geometry is always held in millimetres;
/// this only changes what is shown, typed and exported.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum PitchUnit {
    /// Module, lengths in mm.
    #[default]
    Module,
    /// Diametral pitch, lengths in inches.
    DiametralPitch,
}

impl PitchUnit {
    pub fn is_inch(self) -> bool {
        self == PitchUnit::DiametralPitch
    }
    /// Millimetres per display unit.
    pub fn factor(self) -> f64 {
        if self.is_inch() { 25.4 } else { 1.0 }
    }
    pub fn suffix(self) -> &'static str {
        if self.is_inch() { " in" } else { " mm" }
    }
    pub fn decimals(self) -> usize {
        if self.is_inch() { 4 } else { 3 }
    }
    /// A sensible fresh pulley: XL, the section most often stocked, with
    /// a 3/8 in belt.
    pub fn starter_pulley(self, teeth: u32) -> GearParams {
        let width = BeltSection::Xl.width_near(9.5);
        let mut p = GearParams {
            teeth,
            belt: Some(BeltSection::Xl),
            belt_width: width.width,
            face_width: width.unflanged,
            bore: if self.is_inch() { 0.25 * 25.4 } else { 6.0 },
            ..Default::default()
        };
        p.apply_standard();
        p
    }

    /// A sensible fresh bevel gear in this system; its cone is set once it
    /// has a mate.
    pub fn starter_bevel(self, teeth: u32) -> GearParams {
        let p = self.starter(teeth);
        GearParams { face_width: 4.0 * p.module, bevel: Some(Bevel::default()), ..p }
    }

    /// Half of a car-style hypoid set: 11:41, module 5 at the heel.
    pub fn starter_hypoid(self, role: Role) -> GearParams {
        let m = if self.is_inch() { 25.4 / 5.0 } else { 5.0 };
        let p = self.starter(if role == Role::Pinion { 11 } else { 41 });
        match role {
            Role::Pinion => GearParams { module: m, bore: 3.0 * m, face_width: 6.4 * m, hypoid: Some(role), ..p },
            Role::Ring => GearParams { module: m, bore: 22.0 * m, face_width: 6.4 * m, hypoid: Some(role), ..p },
        }
    }

    /// Half of a bevel worm set: a 3-start worm and a 30-tooth wheel,
    /// module 4 at the heel. The worm is solid, as it is usually cut on
    /// its shaft.
    pub fn starter_worm(self, role: Role) -> GearParams {
        let m: f64 = if self.is_inch() { 25.4 / 6.0 } else { 4.0 };
        let p = self.starter(if role == Role::Pinion { 3 } else { 30 });
        let face = (0.3 * m * 30.0 / 2.0).round();
        let bore = if role == Role::Pinion { 0.0 } else { 7.5 * m };
        GearParams { module: m, bore, face_width: face, hypoid: Some(role), worm: true, ..p }
    }

    /// Half of a worm drive: a single-start worm and a 30-tooth wheel.
    /// The worm is solid, as it is usually cut on its shaft; it is about
    /// as long as DIN 3975 asks for, the wheel's face three quarters of the
    /// worm's outside diameter.
    pub fn starter_worm_drive(self, role: worm::Role) -> GearParams {
        let m: f64 = if self.is_inch() { 25.4 / 12.0 } else { 2.0 };
        let wheel_teeth = 30.0;
        let p = self.starter(if role == worm::Role::Worm { 1 } else { 30 });
        let (face, bore) = match role {
            worm::Role::Worm => (((11.0 + 0.06 * wheel_teeth) * m).round(), 0.0),
            worm::Role::Wheel => ((0.75 * (worm::default_diameter(m) + 2.0 * m)).round(), p.bore),
        };
        GearParams { module: m, face_width: face, bore, worm_drive: Some(role), ..p }
    }

    /// A sensible fresh gear in this system.
    pub fn starter(self, teeth: u32) -> GearParams {
        match self {
            PitchUnit::Module => GearParams { teeth, ..Default::default() },
            PitchUnit::DiametralPitch => {
                let mut p = GearParams {
                    teeth,
                    module: 25.4 / 12.0,
                    standard: ToothStandard::AgmaFullDepth,
                    bore: 0.25 * 25.4,
                    face_width: 0.5 * 25.4,
                    ..Default::default()
                };
                p.apply_standard();
                p
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Project {
    pub version: u32,
    pub gears: Vec<Gear>,
    pub next_id: u64,
    pub driver_rpm: f64,
    pub pitch_unit: PitchUnit,
    /// Ø of the ball end mill the parts are cut with, mm; 0 until it is
    /// chosen, the roots then filleted to each part's own radius.
    pub ball_end_mill: f64,
    /// Shape the teeth for the ball end mill on the 4-axis mill: roots
    /// rounded to it, and the ring gears' slot floors raised where it
    /// can't reach. Off for parts made another way (3D printed, say),
    /// which keep the teeth as designed; the ball is remembered.
    pub for_machining: bool,
}

impl Default for Project {
    fn default() -> Self {
        Self::new(PitchUnit::Module)
    }
}

/// Where a gear ends up and how it turns relative to the driver.
#[derive(Clone, Debug)]
pub struct Solved {
    pub center: [f64; 2],
    /// Angular velocity relative to the driver (negative = opposite way).
    pub ratio: f64,
    /// Rotation of the local outline when the driver is at angle 0.
    pub phase: f64,
    pub mesh: Option<MeshInfo>,
    pub belt: Option<BeltInfo>,
    pub depth: usize,
    /// Back face along the axis; the part runs to z + face width.
    pub z: f64,
    /// Where the part sits in 3D: its own axes (z along its shaft) and
    /// origin (the back face centre, or a bevel gear's cone apex).
    pub frame: Frame3,
    /// A hypoid ring gear's solved pair design.
    pub hypoid: Option<Box<Design>>,
    /// A worm wheel's solved drive.
    pub worm: Option<Box<worm::Design>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame3 {
    pub origin: [f64; 3],
    pub x: [f64; 3],
    pub y: [f64; 3],
    pub z: [f64; 3],
}

impl Frame3 {
    pub fn at(origin: [f64; 3]) -> Self {
        Frame3 { origin, x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] }
    }

    /// A local point in world coordinates.
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let o = self.origin;
        std::array::from_fn(|k| o[k] + p[0] * self.x[k] + p[1] * self.y[k] + p[2] * self.z[k])
    }

    /// A local direction in world coordinates.
    pub fn turn(&self, d: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|k| d[0] * self.x[k] + d[1] * self.y[k] + d[2] * self.z[k])
    }
}

fn v_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn v_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn v_unit(a: [f64; 3]) -> [f64; 3] {
    let l = v_dot(a, a).sqrt().max(1e-300);
    [a[0] / l, a[1] / l, a[2] / l]
}

/// The hypoid set made by pinion `pinion` and ring gear `ring` placed by
/// `placement` (a Placement::Hypoid), cut by a ball end mill of Ø `ball`
/// (0: none chosen).
pub fn hypoid_spec(pinion: &GearParams, ring: &GearParams, placement: &Placement, ball: f64) -> Option<hypoid::Spec> {
    let Placement::Hypoid { offset, shaft_angle, spiral, reverse_hand, cutter_radius, .. } = *placement else {
        return None;
    };
    Some(hypoid::Spec {
        pinion_teeth: pinion.teeth,
        ring_teeth: ring.teeth,
        module: ring.module,
        pressure_angle: ring.pressure_angle,
        face: ring.face_width,
        offset,
        shaft_angle,
        spiral,
        reverse_hand,
        cutter_radius,
        backlash: ring.backlash,
        pinion_bore: pinion.bore,
        ring_bore: ring.bore,
        ball,
    })
}

/// The worm drive made by worm `worm` and wheel `wheel` placed by
/// `placement` (a Placement::Worm), cut by a ball end mill of Ø `ball`
/// (0: none chosen, so the wheel's root fillet, or the biggest ball that
/// fits if that's smaller).
pub fn worm_spec(worm: &GearParams, wheel: &GearParams, placement: &Placement, ball: f64) -> Option<worm::Spec> {
    let Placement::Worm { diameter, throated, globoid, left_hand, .. } = *placement else {
        return None;
    };
    let sp = worm::Spec {
        starts: worm.teeth,
        teeth: wheel.teeth,
        module: wheel.module,
        pressure_angle: wheel.pressure_angle,
        addendum: wheel.addendum,
        dedendum: wheel.dedendum,
        root_fillet: if ball > 0.0 { ball / 2.0 / wheel.module } else { wheel.root_fillet },
        diameter,
        length: worm.face_width,
        face: wheel.face_width,
        throated,
        globoid,
        left_hand,
        backlash: wheel.backlash,
        worm_bore: worm.bore,
        wheel_bore: wheel.bore,
    };
    // With no ball chosen, the biggest that fits, up to the wheel's fillet.
    let largest = if ball > 0.0 { None } else { sp.largest_ball() };
    Some(match largest {
        Some(d) => worm::Spec { root_fillet: sp.root_fillet.min(0.99 * d / 2.0 / sp.module), ..sp },
        None => sp,
    })
}

fn default_friction() -> f64 {
    worm::DEFAULT_FRICTION
}

/// A pair whose teeth are generated together: a hypoid or spiral bevel
/// set (bevel worms included), or a worm drive.
#[derive(Clone, Debug, PartialEq)]
pub enum PairSpec {
    Hypoid(hypoid::Spec),
    Worm(worm::Spec),
}

impl PairSpec {
    /// The driver's teeth (pinion or worm), then the driven's.
    pub fn generate(&self) -> Result<(hypoid::Member, hypoid::Member), String> {
        match self {
            PairSpec::Hypoid(sp) => {
                let d = sp.design().ok_or_else(|| "These settings have no hypoid solution.".to_string())?;
                d.members()
            }
            PairSpec::Worm(sp) => {
                let d = sp.design()?;
                Ok((d.worm()?, d.wheel()?))
            }
        }
    }
}

/// The generated pair a ring gear or wheel `driven`, placed by
/// `placement`, makes with its driver, cut by a ball end mill of Ø `ball`.
pub fn pair_spec(driver: &GearParams, driven: &GearParams, placement: &Placement, ball: f64) -> Option<PairSpec> {
    match placement {
        Placement::Hypoid { .. } => hypoid_spec(driver, driven, placement, ball).map(PairSpec::Hypoid),
        Placement::Worm { .. } => worm_spec(driver, driven, placement, ball).map(PairSpec::Worm),
        _ => None,
    }
}

/// Along its own axis, where a part starts and ends relative to its frame
/// origin: a spur gear or pulley from its back face, a bevel gear from its
/// apex (front face to back face).
fn axial_extent(p: &GearParams) -> (f64, f64) {
    match BevelGeom::new(p) {
        Some(g) => (g.front_face().0, g.back_face().0),
        None => (0.0, p.face_width),
    }
}

/// How a belted pulley pair sits.
#[derive(Clone, Debug)]
pub struct BeltInfo {
    pub center: f64,
    /// Belt pitch length, mm.
    pub length: f64,
    pub teeth: u32,
    /// ISO 5296 designation, if the belt is a standard length.
    pub designation: Option<String>,
    /// Wrap on the smaller pulley, radians, and the teeth engaged there.
    pub wrap: f64,
    pub teeth_in_mesh: f64,
    /// The belt cannot reach round both pulleys.
    pub too_short: bool,
    /// Not two pulleys of the same section.
    pub mismatch: bool,
}

impl BeltInfo {
    pub fn issues(&self) -> Vec<Issue> {
        let mut out = Vec::new();
        if self.mismatch {
            out.push(Issue::error("A belt needs a pulley at each end, both for the same belt section."));
            return out;
        }
        if self.too_short {
            out.push(Issue::error("The belt is too short to go round both pulleys."));
            return out;
        }
        if self.teeth_in_mesh < 6.0 {
            out.push(Issue::warn(format!(
                "Only {:.1} teeth engaged on the smaller pulley; belts want 6 or more.",
                self.teeth_in_mesh
            )));
        }
        out
    }
}

impl Project {
    /// A starter pair in the given unit system.
    pub fn new(unit: PitchUnit) -> Self {
        let mut p = Self { version: 1, gears: Vec::new(), next_id: 1, driver_rpm: 10.0, pitch_unit: unit, ball_end_mill: 0.0, for_machining: true };
        let a = p.add_root(unit.starter(20), 0.0, 0.0);
        let wheel = GearParams { bore: unit.starter(40).bore * 1.25, ..unit.starter(40) };
        p.add_child(a, wheel, Link::Mesh);
        p
    }

    /// Switch unit system and resize every gear to the nearest standard
    /// pitch of the new system, with that system's usual tooth standard.
    /// Gears with custom rack proportions keep them.
    pub fn convert_to(&mut self, unit: PitchUnit) {
        // Pulleys keep their belt section: its pitch is fixed either way.
        for g in self.gears.iter_mut().filter(|g| !g.params.is_pulley()) {
            let p = &mut g.params;
            let custom = p.standard == ToothStandard::Custom;
            match unit {
                PitchUnit::DiametralPitch => {
                    let dp = nearest(&STD_DPS, 25.4 / p.module);
                    p.module = 25.4 / dp;
                    if !custom {
                        p.standard = if (p.pressure_angle - 14.5).abs() < 1e-9 {
                            ToothStandard::Agma14_5
                        } else if dp >= 20.0 {
                            ToothStandard::AgmaFinePitch
                        } else {
                            ToothStandard::AgmaFullDepth
                        };
                    }
                }
                PitchUnit::Module => {
                    p.module = nearest(&STD_MODULES, p.module);
                    if !custom {
                        p.standard = ToothStandard::Iso53;
                    }
                }
            }
        }
        self.pitch_unit = unit;
        self.normalize();
    }

    #[cfg(test)]
    pub fn empty() -> Self {
        Self { gears: Vec::new(), ..Default::default() }
    }

    pub fn index_of(&self, id: u64) -> Option<usize> {
        self.gears.iter().position(|g| g.id == id)
    }

    pub fn get(&self, id: u64) -> Option<&Gear> {
        self.gears.iter().find(|g| g.id == id)
    }

    fn fresh_id(&mut self) -> u64 {
        let id = self.next_id.max(self.gears.iter().map(|g| g.id + 1).max().unwrap_or(1));
        self.next_id = id + 1;
        id
    }

    /// "Gear 3", "Pulley 2": the lowest number its kind has free.
    pub fn fresh_name(&self, params: &GearParams) -> String {
        let kind = kind_name(params);
        let mut n = 1;
        loop {
            let name = format!("{kind} {n}");
            if !self.gears.iter().any(|g| g.name == name) {
                return name;
            }
            n += 1;
        }
    }

    pub fn add_root(&mut self, params: GearParams, x: f64, y: f64) -> u64 {
        let id = self.fresh_id();
        let name = self.fresh_name(&params);
        self.gears.push(Gear { id, name, params, placement: Placement::Root { x, y, phase: 0.0 }, match_parent: true, axial: 0.0, export_origin: ExportOrigin::default() });
        id
    }

    /// Add a gear or pulley attached to `parent`, inserted after the
    /// parent's subtree so the list keeps reading like the train.
    pub fn add_child(&mut self, parent: u64, mut params: GearParams, link: Link) -> u64 {
        let Some(pi) = self.index_of(parent) else { return 0 };
        let pp = self.gears[pi].params.clone();
        // Point away from the parent's own parent so a chain grows outward
        // instead of folding back on itself, and clear of its siblings.
        let direction = || {
            let angle = match (&self.gears[pi].placement, link) {
                (_, Link::Bevel) => 0.0,
                (Placement::Mesh { angle, .. } | Placement::Belt { angle, .. }, _) => *angle,
                _ => 0.0,
            };
            let taken: Vec<f64> = self
                .gears
                .iter()
                .filter_map(|g| match g.placement {
                    Placement::Mesh { parent: p, angle }
                    | Placement::Belt { parent: p, angle, .. }
                    | Placement::Bevel { parent: p, angle, .. }
                        if p == parent =>
                    {
                        Some(angle)
                    }
                    _ => None,
                })
                .collect();
            let mut a = angle;
            while taken.iter().any(|t| ((t - a + 540.0) % 360.0 - 180.0).abs() < 30.0) {
                a += 60.0;
            }
            a % 360.0
        };
        let placement = match link {
            Link::Axle => Placement::Axle { parent, phase: 0.0 },
            Link::Mesh => {
                params.module = pp.module;
                params.pressure_angle = pp.pressure_angle;
                Placement::Mesh { parent, angle: direction() }
            }
            Link::Bevel => {
                params.module = pp.module;
                params.pressure_angle = pp.pressure_angle;
                params.bevel = params.bevel.or(Some(Bevel::default()));
                Placement::Bevel { parent, angle: direction(), shaft_angle: 90.0 }
            }
            Link::Hypoid => {
                params.hypoid = Some(Role::Ring);
                params.bevel = None;
                Placement::Hypoid {
                    parent,
                    angle: 0.0,
                    // A new bevel worm's shafts meet; its threads wrap further.
                    offset: if params.worm { 0.0 } else { (0.03 * params.module * params.teeth as f64).round() * 5.0 },
                    shaft_angle: 90.0,
                    spiral: if params.worm { 50.0 } else { 45.0 },
                    reverse_hand: false,
                    cutter_radius: 0.0,
                }
            }
            Link::Worm => {
                params.worm_drive = Some(worm::Role::Wheel);
                params.bevel = None;
                params.hypoid = None;
                Placement::Worm {
                    parent,
                    angle: 0.0,
                    diameter: worm::default_diameter(params.module),
                    throated: true,
                    globoid: false,
                    left_hand: false,
                    friction: worm::DEFAULT_FRICTION,
                }
            }
            Link::Belt => {
                params.belt = pp.belt.or(params.belt);
                params.belt_width = pp.belt_width;
                params.apply_standard();
                let section = params.belt.unwrap_or(BeltSection::Xl);
                let (rp, rc) = (pp.pitch_radius(), params.pitch_radius());
                let belt_teeth = pulley::belt_for_center(section, rp, rc, starting_center(&pp, &params), true)
                    .map(|(t, _)| t)
                    .unwrap_or(100);
                Placement::Belt { parent, angle: direction(), belt_teeth, target: 0.0 }
            }
        };
        let id = self.fresh_id();
        let name = self.fresh_name(&params);
        let at = self.subtree_end(pi);
        self.gears.insert(at, Gear { id, name, params, placement, match_parent: link != Link::Axle, axial: 0.0, export_origin: ExportOrigin::default() });
        id
    }

    /// Index one past the last descendant of the gear at `i`.
    fn subtree_end(&self, i: usize) -> usize {
        let mut inside = vec![self.gears[i].id];
        let mut j = i + 1;
        while j < self.gears.len() {
            match self.gears[j].placement.parent() {
                Some(p) if inside.contains(&p) => {
                    inside.push(self.gears[j].id);
                    j += 1;
                }
                _ => break,
            }
        }
        j
    }

    pub fn descendants(&self, id: u64) -> Vec<u64> {
        let mut out = vec![id];
        for g in &self.gears {
            if let Some(p) = g.placement.parent() {
                if out.contains(&p) {
                    out.push(g.id);
                }
            }
        }
        out.remove(0);
        out
    }

    /// Give parts still carrying the other kind's default name ("Gear 2"
    /// on a pulley, from older files or a type switch) a name of their own.
    pub fn fix_default_names(&mut self) {
        for i in 0..self.gears.len() {
            let g = &self.gears[i];
            let own = kind_name(&g.params);
            let is_default = ["Gear", "Pulley", "Bevel", "Pinion", "Ring gear", "Worm", "Worm wheel"].iter().filter(|k| **k != own).any(|k| {
                g.name.strip_prefix(k).and_then(|n| n.trim().parse::<u32>().ok()).is_some()
            });
            if is_default {
                let params = g.params.clone();
                self.gears[i].name = String::new();
                self.gears[i].name = self.fresh_name(&params);
            }
        }
    }

    /// Remove a gear and everything attached to it.
    pub fn remove(&mut self, id: u64) {
        let mut doomed = self.descendants(id);
        doomed.push(id);
        self.gears.retain(|g| !doomed.contains(&g.id));
    }

    /// The ball end mill Ø the teeth are shaped for: none when they are
    /// not made for machining.
    pub fn ball(&self) -> f64 {
        if self.for_machining { self.ball_end_mill } else { 0.0 }
    }

    /// Give every part the ball end mill its roots are rounded for.
    pub fn apply_ball(&mut self) {
        let ball = self.ball();
        for g in &mut self.gears {
            g.params.ball = ball;
        }
    }

    /// Repair anything a hand-edited or older file could get wrong: unknown
    /// or later parents become roots, linked tooth systems are synced, and
    /// standard rack coefficients are re-derived (fine pitch depends on DP).
    pub fn normalize(&mut self) {
        self.apply_ball();
        // Bevel cones: the first mate of a gear fixes its cone; later
        // mates follow from their tooth counts (the cones share an apex).
        let mut coned = vec![false; self.gears.len()];
        for i in 0..self.gears.len() {
            if let Some(p) = self.gears[i].placement.parent() {
                match self.gears[..i].iter().position(|g| g.id == p) {
                    Some(pi) => {
                        let meshes = matches!(self.gears[i].placement, Placement::Mesh { .. } | Placement::Bevel { .. });
                        if let Placement::Bevel { shaft_angle, .. } = self.gears[i].placement {
                            let (zp, zc) = (self.gears[pi].params.teeth, self.gears[i].params.teeth);
                            if self.gears[pi].params.is_bevel() && self.gears[i].params.is_bevel() && zp > 0 && zc > 0 {
                                let cd = if coned[pi] {
                                    let pd = self.gears[pi].params.bevel.unwrap_or_default().pitch_angle.to_radians();
                                    (zc as f64 * pd.sin() / zp as f64).clamp(-1.0, 1.0).asin()
                                } else {
                                    let (pd, cd) = bevel::pitch_angles(zp, zc, shaft_angle.to_radians());
                                    self.gears[pi].params.bevel = Some(Bevel { pitch_angle: pd.to_degrees() });
                                    coned[pi] = true;
                                    cd
                                };
                                self.gears[i].params.bevel = Some(Bevel { pitch_angle: cd.to_degrees() });
                                coned[i] = true;
                            }
                        }
                        if matches!(self.gears[i].placement, Placement::Hypoid { .. }) {
                            // The ring gear sets the pair's teeth.
                            let src = self.gears[i].params.clone();
                            let dst = &mut self.gears[pi].params;
                            dst.module = src.module;
                            dst.pressure_angle = src.pressure_angle;
                            dst.hypoid = Some(Role::Pinion);
                            dst.worm = src.worm;
                            self.gears[i].params.hypoid = Some(Role::Ring);
                        }
                        if matches!(self.gears[i].placement, Placement::Worm { .. }) {
                            // The wheel sets the drive's teeth.
                            let src = self.gears[i].params.clone();
                            let dst = &mut self.gears[pi].params;
                            dst.module = src.module;
                            dst.pressure_angle = src.pressure_angle;
                            dst.standard = src.standard;
                            dst.addendum = src.addendum;
                            dst.dedendum = src.dedendum;
                            dst.root_fillet = src.root_fillet;
                            dst.worm_drive = Some(worm::Role::Worm);
                            self.gears[i].params.worm_drive = Some(worm::Role::Wheel);
                        }
                        let belted = matches!(self.gears[i].placement, Placement::Belt { .. });
                        if belted && self.gears[i].match_parent && self.gears[pi].params.is_pulley() {
                            let (belt, width) = (self.gears[pi].params.belt, self.gears[pi].params.belt_width);
                            let c = &mut self.gears[i].params;
                            if c.belt != belt || c.belt_width != width {
                                c.face_width = belt.map(|b| b.width_near(width).unflanged).unwrap_or(c.face_width);
                            }
                            c.belt = belt;
                            c.belt_width = width;
                        }
                        if meshes && self.gears[i].match_parent {
                            let src = self.gears[pi].params.clone();
                            let dst = &mut self.gears[i].params;
                            dst.module = src.module;
                            dst.pressure_angle = src.pressure_angle;
                            dst.standard = src.standard;
                            dst.addendum = src.addendum;
                            dst.dedendum = src.dedendum;
                            dst.root_fillet = src.root_fillet;
                        }
                    }
                    None => self.gears[i].placement = Placement::Root { x: 0.0, y: 0.0, phase: 0.0 },
                }
            }
            self.gears[i].params.apply_standard();
        }
        self.next_id = self.next_id.max(self.gears.iter().map(|g| g.id + 1).max().unwrap_or(1));
    }

    pub fn solve(&self) -> Vec<Solved> {
        let mut out: Vec<Solved> = Vec::with_capacity(self.gears.len());
        for g in &self.gears {
            let parent = g.placement.parent().and_then(|p| self.gears.iter().position(|q| q.id == p));
            let s = match (&g.placement, parent) {
                (Placement::Mesh { angle, .. }, Some(pi)) => {
                    let pg = &self.gears[pi];
                    let ps = &out[pi];
                    let mi = gear::mesh(&pg.params, &g.params);
                    let beta = angle.to_radians();
                    let (zp, zc) = (pg.params.teeth as f64, g.params.teeth as f64);
                    // With the parent turned so a tooth points along the
                    // line of centres, this gear needs a space facing back.
                    let phase = beta + PI + PI / zc - (ps.phase - beta) * zp / zc;
                    Solved {
                        center: [ps.center[0] + mi.center * beta.cos(), ps.center[1] + mi.center * beta.sin()],
                        ratio: -ps.ratio * zp / zc,
                        phase,
                        mesh: Some(mi),
                        belt: None,
                        depth: ps.depth + 1,
                        z: 0.0,
                        frame: Frame3::at([0.0; 3]),
                        hypoid: None,
                        worm: None,
                    }
                }
                (Placement::Belt { angle, belt_teeth, .. }, Some(pi)) => {
                    let pg = &self.gears[pi];
                    let ps = &out[pi];
                    let bi = belt_info(&pg.params, &g.params, *belt_teeth);
                    let beta = angle.to_radians();
                    let (zp, zc) = (pg.params.teeth as f64, g.params.teeth as f64);
                    let center = [ps.center[0] + bi.center * beta.cos(), ps.center[1] + bi.center * beta.sin()];
                    // Turn this pulley so its grooves take the belt teeth
                    // that the parent's grooves put on the belt.
                    let (rp, rc) = (pg.params.pitch_radius(), g.params.pitch_radius());
                    let phase = match pulley::BeltLoop::new(ps.center, rp, center, rc) {
                        Some(lp) if !bi.too_short && !bi.mismatch => {
                            let s0 = lp.s_on_first(ps.phase + pulley::groove_angle(pg.params.teeth));
                            lp.angle_on_second(s0) - pulley::groove_angle(g.params.teeth)
                        }
                        _ => 0.0,
                    };
                    Solved {
                        center,
                        // An open belt turns both pulleys the same way.
                        ratio: ps.ratio * zp / zc,
                        phase,
                        mesh: None,
                        belt: Some(bi),
                        depth: ps.depth + 1,
                        z: 0.0,
                        frame: Frame3::at([0.0; 3]),
                        hypoid: None,
                        worm: None,
                    }
                }
                (Placement::Bevel { angle, .. }, Some(pi)) => {
                    let pg = &self.gears[pi];
                    let ps = &out[pi];
                    let (zp, zc) = (pg.params.teeth as f64, g.params.teeth as f64);
                    let beta = angle.to_radians();
                    let pd = pg.params.bevel.unwrap_or_default().pitch_angle.to_radians();
                    let cd = g.params.bevel.unwrap_or_default().pitch_angle.to_radians();
                    let f = ps.frame;
                    // Lean the axis away from the parent's by the two pitch
                    // cones; they touch along the generator at azimuth β.
                    let towards = f.turn([beta.cos(), beta.sin(), 0.0]);
                    let sigma = pd + cd;
                    let axis: [f64; 3] = std::array::from_fn(|k| sigma.cos() * f.z[k] + sigma.sin() * towards[k]);
                    let contact: [f64; 3] = std::array::from_fn(|k| pd.cos() * f.z[k] + pd.sin() * towards[k]);
                    // Local x points away from the contact line, so the
                    // contact sits at azimuth π as it does for a spur mesh.
                    let along = v_dot(contact, axis);
                    let x = v_unit(std::array::from_fn(|k| -(contact[k] - along * axis[k])));
                    let frame = Frame3 { origin: f.origin, x, y: v_cross(axis, x), z: axis };
                    Solved {
                        center: [f.origin[0], f.origin[1]],
                        ratio: -ps.ratio * zp / zc,
                        phase: PI + PI / zc - (ps.phase - beta) * zp / zc,
                        mesh: None,
                        belt: None,
                        depth: ps.depth + 1,
                        z: f.origin[2],
                        frame,
                        hypoid: None,
                        worm: None,
                    }
                }
                (Placement::Hypoid { angle, .. }, Some(pi)) => {
                    let pg = &self.gears[pi];
                    let ps = &out[pi];
                    let design = hypoid_spec(&pg.params, &g.params, &g.placement, self.ball()).and_then(|sp| sp.design());
                    let f = ps.frame;
                    let beta = angle.to_radians();
                    let turn = |v: [f64; 3]| {
                        let (sn, cs) = beta.sin_cos();
                        f.turn([v[0] * cs - v[1] * sn, v[0] * sn + v[1] * cs, v[2]])
                    };
                    match design {
                        Some(d) => {
                            // The ring's frame from where generation put it
                            // relative to the pinion, turned round the
                            // pinion's axis by β.
                            let (o, x, y, z) = d.ring_in_pinion();
                            let (so, co) = beta.sin_cos();
                            let origin = f.apply([o[0] * co - o[1] * so, o[0] * so + o[1] * co, o[2]]);
                            let frame = Frame3 { origin, x: turn(x), y: turn(y), z: turn(z) };
                            let k = d.sense * pg.params.teeth as f64 / g.params.teeth as f64;
                            Solved {
                                center: [origin[0], origin[1]],
                                ratio: k * ps.ratio,
                                phase: k * (ps.phase - beta),
                                mesh: None,
                                belt: None,
                                depth: ps.depth + 1,
                                z: origin[2],
                                frame,
                                hypoid: Some(Box::new(d)),
                                worm: None,
                            }
                        }
                        None => Solved {
                            center: ps.center,
                            ratio: ps.ratio,
                            phase: 0.0,
                            mesh: None,
                            belt: None,
                            depth: ps.depth + 1,
                            z: f.origin[2],
                            frame: f,
                            hypoid: None,
                            worm: None,
                        },
                    }
                }
                (Placement::Worm { angle, .. }, Some(pi)) => {
                    let pg = &self.gears[pi];
                    let ps = &out[pi];
                    let f = ps.frame;
                    let design = worm_spec(&pg.params, &g.params, &g.placement, self.ball()).and_then(|sp| sp.solve().ok());
                    match design {
                        Some(d) => {
                            // Across the worm's axis towards azimuth β, at the
                            // middle of its threads; the wheel's axis square
                            // to both, the worm at azimuth π on the wheel.
                            let beta = angle.to_radians();
                            let (sb, cb) = beta.sin_cos();
                            let x = f.turn([cb, sb, 0.0]);
                            let z = f.turn([sb, -cb, 0.0]);
                            let centre = f.apply([d.center * cb, d.center * sb, pg.params.face_width / 2.0]);
                            let origin: [f64; 3] = std::array::from_fn(|k| centre[k] - z[k] * g.params.face_width / 2.0);
                            let k = d.ratio();
                            Solved {
                                center: [origin[0], origin[1]],
                                ratio: k * ps.ratio,
                                phase: k * (ps.phase - beta),
                                mesh: None,
                                belt: None,
                                depth: ps.depth + 1,
                                z: origin[2],
                                frame: Frame3 { origin, x, y: f.z, z },
                                hypoid: None,
                                worm: Some(Box::new(d)),
                            }
                        }
                        None => Solved {
                            center: ps.center,
                            ratio: ps.ratio,
                            phase: 0.0,
                            mesh: None,
                            belt: None,
                            depth: ps.depth + 1,
                            z: f.origin[2],
                            frame: f,
                            hypoid: None,
                            worm: None,
                        },
                    }
                }
                (Placement::Axle { phase, .. }, Some(pi)) => {
                    let ps = &out[pi];
                    Solved {
                        center: ps.center,
                        ratio: ps.ratio,
                        phase: ps.phase + phase.to_radians(),
                        mesh: None,
                        belt: None,
                        depth: ps.depth + 1,
                        z: 0.0,
                        frame: Frame3::at([0.0; 3]),
                        hypoid: None,
                        worm: None,
                    }
                }
                (Placement::Root { x, y, phase }, _) => Solved {
                    center: [*x, *y],
                    ratio: 1.0,
                    phase: phase.to_radians(),
                    mesh: None,
                    belt: None,
                    depth: 0,
                    z: 0.0,
                    frame: Frame3::at([0.0; 3]),
                    hypoid: None,
                    worm: None,
                },
                // Unreachable after normalize(); draw it at the origin.
                (_, None) => {
                    Solved {
                        center: [0.0, 0.0],
                        ratio: 1.0,
                        phase: 0.0,
                        mesh: None,
                        belt: None,
                        depth: 0,
                        z: 0.0,
                        frame: Frame3::at([0.0; 3]),
                        hypoid: None,
                        worm: None,
                    }
                }
            };
            let mut s = s;
            match (&g.placement, parent) {
                (Placement::Bevel { .. } | Placement::Hypoid { .. } | Placement::Worm { .. }, _) => {}
                (Placement::Axle { .. }, Some(pi)) => {
                    // Stacked along the parent's shaft: this part starts
                    // where the parent ends.
                    let f = out[pi].frame;
                    let shift = axial_extent(&self.gears[pi].params).1 + g.axial - axial_extent(&g.params).0;
                    s.frame = Frame3 { origin: f.apply([0.0, 0.0, shift]), ..f };
                    s.z = s.frame.origin[2];
                }
                (Placement::Mesh { .. } | Placement::Belt { .. }, Some(pi)) => {
                    s.z = g.axial + out[pi].z + (self.gears[pi].params.face_width - g.params.face_width) / 2.0;
                    s.frame = Frame3::at([s.center[0], s.center[1], s.z]);
                }
                _ => {
                    s.z = g.axial;
                    s.frame = Frame3::at([s.center[0], s.center[1], s.z]);
                }
            }
            out.push(s);
        }
        out
    }

    /// The pair a part's ratio is quoted for, as (driving, driven) indices:
    /// the part it meshes with or is belted to and itself, or, for a part
    /// placed on its own, the first part placed against it.
    pub fn ratio_pair(&self, i: usize) -> Option<(usize, usize)> {
        let g = &self.gears[i];
        if g.placement.meshes() {
            return Some((self.index_of(g.placement.parent()?)?, i));
        }
        let c = self.gears.iter().position(|c| c.placement.meshes() && c.placement.parent() == Some(g.id))?;
        Some((i, c))
    }

    /// How many parts a part meshes with or is belted to.
    fn mates(&self, i: usize) -> usize {
        let g = &self.gears[i];
        let children = self.gears.iter().filter(|c| c.placement.meshes() && c.placement.parent() == Some(g.id)).count();
        children + g.placement.meshes() as usize
    }

    /// Give the pair `ratio_pair(i)` the tooth counts that come nearest
    /// `target` (driven : driving), at most `max_teeth` each. A part that
    /// also meshes with another keeps its teeth, so the train's other
    /// ratios stand.
    pub fn size_pair_to_ratio(&mut self, i: usize, target: f64, max_teeth: u32) -> Result<Resized, String> {
        let (a, b) = self.ratio_pair(i).ok_or("This part has no partner to set a ratio with.")?;
        if !(target > 0.0 && target.is_finite()) {
            return Err("The ratio must be more than 0.".into());
        }
        let range = |k: usize| {
            let p = &self.gears[k].params;
            let (lo, hi) = tooth_range(p);
            if self.mates(k) > 1 {
                return p.teeth..=p.teeth;
            }
            // The worm's starts and a hypoid pinion's teeth have their own cap.
            let hi = if p.is_worm_thread() { hi } else { hi.min(max_teeth) };
            lo..=hi.max(lo)
        };
        let (ra, rb) = (range(a), range(b));
        if self.mates(a) > 1 && self.mates(b) > 1 {
            return Err("Both gears mesh with other parts too: changing either would change those ratios.".into());
        }
        let now = (self.gears[a].params.teeth, self.gears[b].params.teeth);
        let (za, zb) = teeth_for_ratio(target, ra, rb, now).ok_or("No tooth counts fit the limits.")?;
        self.gears[a].params.teeth = za;
        self.gears[b].params.teeth = zb;
        self.normalize();
        let held = [a, b].into_iter().find(|&k| self.mates(k) > 1).map(|k| self.gears[k].name.clone());
        Ok(Resized { driving: za, driven: zb, held })
    }

    /// Make worm drive wheel `w` a globoid drive with AGMA 6135's
    /// proportions: the worm's threads as long as the base circle allows
    /// (less 2 % of the centre distance) and the wheel's face as wide as
    /// the worm's root diameter. `size_worm` also picks the worm's pitch
    /// diameter for the wheel.
    pub fn make_globoid(&mut self, w: usize, size_worm: bool) {
        let Placement::Worm { parent, .. } = self.gears[w].placement else { return };
        let Some(k) = self.index_of(parent) else { return };
        let (m, z2) = (self.gears[w].params.module, self.gears[w].params.teeth);
        if let Placement::Worm { globoid, throated, diameter, .. } = &mut self.gears[w].placement {
            (*globoid, *throated) = (true, true);
            if size_worm {
                *diameter = worm::globoid_diameter(m, z2);
            }
        }
        // Sized with a short worm, which any design allows.
        let spec = worm_spec(&self.gears[k].params, &self.gears[w].params, &self.gears[w].placement, self.ball());
        let Some(d) = spec.and_then(|s| worm::Spec { length: 1.0, ..s }.solve().ok()) else { return };
        let floor = |v: f64| (v * 10.0).floor() / 10.0;
        self.gears[k].params.face_width = floor(d.effective_length().unwrap_or(d.spec.length));
        self.gears[w].params.face_width = floor(2.0 * d.worm_root);
        self.normalize();
    }

    /// Make the worm drive part `i` belongs to back-drivable at its
    /// friction (see `worm::Spec::back_drivable`), returning the drive as
    /// it was and as it is now. The wheel gains teeth to keep the ratio if
    /// `keep_ratio`, unless it also meshes with another part.
    pub fn make_back_drivable(&mut self, i: usize, keep_ratio: bool) -> Result<(worm::Spec, worm::Spec), String> {
        let (k, w) = self.ratio_pair(i).ok_or("This part has no partner.")?;
        let wheel = &self.gears[w];
        let Placement::Worm { friction, .. } = wheel.placement else {
            return Err("Only a worm drive can be made back-drivable.".into());
        };
        let old = worm_spec(&self.gears[k].params, &wheel.params, &wheel.placement, self.ball()).ok_or("This is not a worm drive.")?;
        if self.mates(k) > 1 {
            return Err("The worm turns another wheel too: changing its starts would change that drive.".into());
        }
        let new = old.back_drivable(friction, !keep_ratio || self.mates(w) > 1)?;
        let worm = &mut self.gears[k].params;
        worm.teeth = new.starts;
        worm.face_width = new.length;
        self.gears[w].params.teeth = new.teeth;
        if let Placement::Worm { diameter, .. } = &mut self.gears[w].placement {
            *diameter = new.diameter;
        }
        self.normalize();
        Ok((old, new))
    }

    /// Change a belted pulley's belt to the one whose centre distance comes
    /// closest to `target` (standard lengths only if `standard_only`); the
    /// pulley then moves to where that belt puts it.
    pub fn fit_belt(&mut self, id: u64, target: f64, standard_only: bool) -> Option<(u32, f64)> {
        let i = self.index_of(id)?;
        let pi = self.gears[i].placement.parent().and_then(|p| self.index_of(p))?;
        let (pp, cp) = (&self.gears[pi].params, &self.gears[i].params);
        let section = cp.belt.filter(|s| pp.belt == Some(*s))?;
        let found = pulley::belt_for_center(section, pp.pitch_radius(), cp.pitch_radius(), target, standard_only)?;
        if let Placement::Belt { belt_teeth, target: t, .. } = &mut self.gears[i].placement {
            *belt_teeth = found.0;
            *t = target;
        }
        Some(found)
    }

    /// Pairs of gears whose tip circles overlap without meshing or sharing
    /// an axle.
    pub fn collisions(&self, solved: &[Solved]) -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        for i in 0..self.gears.len() {
            for j in i + 1..self.gears.len() {
                let (a, b) = (&self.gears[i], &self.gears[j]);
                if a.params.is_bevel() || b.params.is_bevel() || a.params.is_generated() || b.params.is_generated() {
                    continue;
                }
                let related = |x: &Gear, y: &Gear| matches!(x.placement, Placement::Mesh { parent, .. } if parent == y.id);
                if related(a, b) || related(b, a) {
                    continue;
                }
                let (ca, cb) = (solved[i].center, solved[j].center);
                let d = (ca[0] - cb[0]).hypot(ca[1] - cb[1]);
                if d < 1e-6 {
                    continue; // same arbor
                }
                if d < a.params.tip_radius() + b.params.tip_radius() - 1e-6 {
                    out.push((i, j));
                }
            }
        }
        out
    }
}

/// What `size_pair_to_ratio` did: the pair's new teeth, and the part that
/// kept its teeth because it meshes with another.
pub struct Resized {
    pub driving: u32,
    pub driven: u32,
    pub held: Option<String>,
}

/// The fewest and most teeth a part of this kind can take. A spur gear
/// stops at undercut (or where it is now, if it is already below), a
/// worm at its starts.
fn tooth_range(p: &GearParams) -> (u32, u32) {
    match p.worm_drive {
        Some(worm::Role::Worm) => return (*worm::STARTS.start(), *worm::STARTS.end()),
        Some(worm::Role::Wheel) => return (20, 400),
        None => {}
    }
    match (p.hypoid, p.worm) {
        (Some(Role::Pinion), true) => return (3, 7),
        (Some(Role::Pinion), false) => return (5, 60),
        (Some(Role::Ring), _) => return (10, 400),
        (None, _) => {}
    }
    if let Some(b) = p.belt {
        return (b.min_teeth(), 400);
    }
    if p.is_bevel() {
        return (10, 400);
    }
    let sin = p.pressure_angle.to_radians().sin();
    let clean = (2.0 * (p.addendum - p.profile_shift) / (sin * sin)).ceil().max(6.0) as u32;
    (clean.min(p.teeth.max(6)), 400)
}

/// Tooth counts (driving, driven) in the given ranges whose ratio
/// driven / driving comes nearest `target`; of equally near ones, the
/// pair closest in size to `now`.
pub fn teeth_for_ratio(target: f64, a: RangeInclusive<u32>, b: RangeInclusive<u32>, now: (u32, u32)) -> Option<(u32, u32)> {
    let size = |za: u32, zb: u32| (za as f64 + zb as f64 - (now.0 + now.1) as f64).abs() + 1e-3 * (za as f64 - now.0 as f64).abs();
    let mut best: Option<((u32, u32), f64, f64)> = None;
    for za in a {
        let ideal = target * za as f64;
        for zb in [ideal.floor(), ideal.ceil()] {
            let zb = (zb.max(0.0) as u32).clamp(*b.start(), *b.end());
            let err = ((zb as f64 / za as f64) / target - 1.0).abs();
            let sz = size(za, zb);
            let better = match best {
                None => true,
                Some((_, e, s)) => err < e - 1e-12 || (err <= e + 1e-12 && sz < s),
            };
            if better {
                best = Some(((za, zb), err, sz));
            }
        }
    }
    best.map(|b| b.0)
}

/// Where a new belt drive starts: about a pulley diameter of daylight
/// between the two.
pub fn starting_center(a: &GearParams, b: &GearParams) -> f64 {
    a.tip_radius() + b.tip_radius() + 2.0 * a.pitch_radius().max(b.pitch_radius())
}

fn kind_name(params: &GearParams) -> &'static str {
    match params.worm_drive {
        Some(worm::Role::Worm) => return "Worm",
        Some(worm::Role::Wheel) => return "Worm wheel",
        None => {}
    }
    match (params.hypoid, params.worm) {
        (Some(Role::Pinion), false) => return "Pinion",
        (Some(Role::Ring), false) => return "Ring gear",
        (Some(Role::Pinion), true) => return "Worm",
        (Some(Role::Ring), true) => return "Worm wheel",
        (None, _) => {}
    }
    if params.is_pulley() {
        "Pulley"
    } else if params.is_bevel() {
        "Bevel"
    } else {
        "Gear"
    }
}

fn belt_info(a: &GearParams, b: &GearParams, belt_teeth: u32) -> BeltInfo {
    let section = a.belt.filter(|s| b.belt == Some(*s));
    let mismatch = section.is_none();
    let pitch = section.or(a.belt).or(b.belt).map(|s| s.pitch()).unwrap_or(PI * a.module);
    let (ra, rb) = (a.pitch_radius(), b.pitch_radius());
    let length = belt_teeth as f64 * pitch;
    let solved = pulley::center_distance(ra, rb, length);
    // Too short: park the pulley just clear so it is still visible.
    let center = solved.unwrap_or(a.tip_radius() + b.tip_radius() + pitch);
    let wrap = pulley::small_wrap(ra, rb, center);
    let small = if ra <= rb { a.teeth } else { b.teeth };
    BeltInfo {
        center,
        length,
        teeth: belt_teeth,
        designation: section.and_then(|s| s.designation(belt_teeth)),
        wrap,
        teeth_in_mesh: small as f64 * wrap / (2.0 * PI),
        too_short: solved.is_none(),
        mismatch,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Parts made other than on the mill (3D printed, say) keep their
    /// teeth as designed: no rounding to the ball, no raised floors. The
    /// ball stays chosen for when machining is back on, as it is for
    /// files from before the setting.
    #[test]
    fn parts_not_machined_keep_their_teeth_as_designed() {
        let mut p = Project::empty();
        let a = p.add_root(PitchUnit::Module.starter_hypoid(Role::Pinion), 0.0, 0.0);
        p.add_child(a, PitchUnit::Module.starter_hypoid(Role::Ring), Link::Hypoid);
        if let Placement::Hypoid { offset, .. } = &mut p.gears[1].placement {
            *offset = 0.0;
        }
        p.ball_end_mill = 2.0;
        p.normalize();
        let ring = |p: &Project| {
            let spec = pair_spec(&p.gears[0].params, &p.gears[1].params, &p.gears[1].placement, p.ball()).unwrap();
            spec.generate().unwrap().1
        };
        assert!(ring(&p).raised > 0.0);
        assert!(p.gears.iter().all(|g| g.params.ball == 2.0));
        p.for_machining = false;
        p.normalize();
        assert_eq!(ring(&p).raised, 0.0);
        assert!(p.gears.iter().all(|g| g.params.ball == 0.0));
        assert_eq!(p.ball_end_mill, 2.0);
        let old: Project = serde_json::from_str(r#"{"ball_end_mill": 2.0}"#).unwrap();
        assert!(old.for_machining);
    }

    #[test]
    fn simple_pair_turns_opposite_at_tooth_ratio() {
        let p = Project::default();
        let s = p.solve();
        assert_eq!(s.len(), 2);
        assert!((s[1].ratio + 0.5).abs() < 1e-12);
        assert!((s[1].center[0] - 60.0).abs() < 1e-9);
    }

    #[test]
    fn compound_train_ratio_multiplies() {
        let mut p = Project::empty();
        let a = p.add_root(GearParams { teeth: 10, ..Default::default() }, 0.0, 0.0);
        let b = p.add_child(a, GearParams { teeth: 60, ..Default::default() }, Link::Mesh);
        let c = p.add_child(b, GearParams { teeth: 8, ..Default::default() }, Link::Axle);
        let _d = p.add_child(c, GearParams { teeth: 64, ..Default::default() }, Link::Mesh);
        let s = p.solve();
        assert!((s[3].ratio - 10.0 / 60.0 * 8.0 / 64.0).abs() < 1e-12);
    }

    fn inside(poly: &[[f64; 2]], q: [f64; 2]) -> bool {
        let mut c = false;
        for i in 0..poly.len() {
            let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
            if (a[1] > q[1]) != (b[1] > q[1]) && q[0] < a[0] + (q[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) {
                c = !c;
            }
        }
        c
    }

    #[test]
    fn meshing_teeth_never_overlap() {
        // (pinion teeth, shift, wheel teeth, shift, direction)
        for (za, xa, zb, xb, dir) in [(13, 0.3, 31, 0.0, 37.0), (8, 0.0, 30, 0.0, 0.0), (20, 0.0, 20, 0.5, 200.0)] {
            check_mesh(za, xa, zb, xb, dir);
        }
    }

    fn check_mesh(za: u32, xa: f64, zb: u32, xb: f64, dir: f64) {
        let mut p = Project::empty();
        let pa = GearParams { teeth: za, profile_shift: xa, backlash: 0.05, ..Default::default() };
        let a = p.add_root(pa, 0.0, 0.0);
        let b = p.add_child(a, GearParams { teeth: zb, profile_shift: xb, backlash: 0.05, ..Default::default() }, Link::Mesh);
        if let Placement::Mesh { angle, .. } = &mut p.gears[1].placement {
            *angle = dir;
        }
        let s = p.solve();
        let place = |i: usize, t: f64| {
            let g = &p.gears[i].params;
            let prof = gear::tooth_profile(g).unwrap();
            let a = s[i].phase + s[i].ratio * t;
            let (sn, cs) = a.sin_cos();
            gear::outline(g, &prof)
                .into_iter()
                .map(|[x, y]| [s[i].center[0] + x * cs - y * sn, s[i].center[1] + x * sn + y * cs])
                .collect::<Vec<_>>()
        };
        for k in 0..12 {
            let t = k as f64 * 0.05;
            let (oa, ob) = (place(0, t), place(1, t));
            let bad = ob.iter().filter(|&&q| inside(&oa, q)).count();
            assert_eq!(bad, 0, "driver angle {t}: {bad} points of gear {b} inside gear {a}");
        }
    }

    #[test]
    fn converting_picks_nearest_standard_pitch() {
        let mut p = Project::default();
        p.convert_to(PitchUnit::DiametralPitch);
        // 2 mm module is 12.7 DP; the nearest stock pitch is 12.
        assert!((25.4 / p.gears[0].params.module - 12.0).abs() < 1e-9);
        assert_eq!(p.gears[1].params.standard, ToothStandard::AgmaFullDepth);
        p.convert_to(PitchUnit::Module);
        // 12 DP is 2.117 mm; back to 2.
        assert!((p.gears[0].params.module - 2.0).abs() < 1e-12);
        assert_eq!(p.gears[0].params.standard, ToothStandard::Iso53);
    }

    #[test]
    fn belted_pulleys_turn_together_at_tooth_ratio() {
        let mut p = Project::empty();
        let a = p.add_root(PitchUnit::DiametralPitch.starter_pulley(20), 0.0, 0.0);
        let b = p.add_child(a, PitchUnit::DiametralPitch.starter_pulley(40), Link::Belt);
        p.normalize();
        let s = p.solve();
        let bi = s[1].belt.as_ref().unwrap();
        assert!(bi.designation.is_some(), "starts on a standard belt");
        assert!((s[1].ratio - 0.5).abs() < 1e-12, "same direction, 20:40");
        let c = s[1].center[0].hypot(s[1].center[1]);
        let (ra, rb) = (p.gears[0].params.pitch_radius(), p.gears[1].params.pitch_radius());
        assert!((pulley::belt_length(ra, rb, c) - bi.length).abs() < 1e-6);
        assert!(bi.issues().is_empty(), "{:?}", bi.issues());

        // Ask for 150 mm: the nearest standard XL belt, and the pulley moves.
        let (teeth, got) = p.fit_belt(b, 150.0, true).unwrap();
        assert!(BeltSection::Xl.designation(teeth).is_some());
        let s = p.solve();
        assert!((s[1].center[0].hypot(s[1].center[1]) - got).abs() < 1e-9);
        assert!((got - 150.0).abs() < 5.08, "within half a belt pitch step");

        // The driven pulley's grooves sit under belt teeth: a groove centre
        // on pulley 2 maps to a whole number of pitches from one on pulley 1.
        let s = p.solve();
        let (pa, pb) = (&p.gears[0].params, &p.gears[1].params);
        let lp = pulley::BeltLoop::new(s[0].center, pa.pitch_radius(), s[1].center, pb.pitch_radius()).unwrap();
        let s1 = lp.s_on_first(s[0].phase + pulley::groove_angle(pa.teeth));
        let groove2 = s[1].phase + pulley::groove_angle(pb.teeth);
        let s2 = lp.angle_on_second(s1);
        let k = (groove2 - s2) / (2.0 * PI / pb.teeth as f64);
        assert!((k - k.round()).abs() < 1e-9, "{k}");
        let len = lp.len();
        assert!((len - s[1].belt.as_ref().unwrap().length).abs() < 1e-6, "{len}");
    }

    #[test]
    fn parts_stack_along_their_shafts() {
        let mut p = Project::empty();
        let a = p.add_root(GearParams { face_width: 10.0, ..Default::default() }, 0.0, 0.0);
        let b = p.add_child(a, GearParams { teeth: 60, face_width: 6.0, ..Default::default() }, Link::Mesh);
        let c = p.add_child(b, GearParams { teeth: 12, face_width: 8.0, ..Default::default() }, Link::Axle);
        p.add_child(c, GearParams { teeth: 48, face_width: 4.0, ..Default::default() }, Link::Mesh);
        p.normalize();
        let s = p.solve();
        assert_eq!(s.iter().map(|s| s.z).collect::<Vec<_>>(), [0.0, 2.0, 8.0, 10.0]);
        // An offset moves the part and everything after it along the shaft.
        p.gears[2].axial = 1.5;
        let s = p.solve();
        assert_eq!(s[3].z, 11.5);
    }

    fn deg(p: &Project, i: usize) -> f64 {
        p.gears[i].params.bevel.unwrap().pitch_angle
    }

    #[test]
    fn bevel_pair_shares_an_apex_and_meets_at_the_shaft_angle() {
        let mut p = Project::empty();
        let a = p.add_root(PitchUnit::Module.starter_bevel(20), 30.0, 0.0);
        let b = p.add_child(a, PitchUnit::Module.starter_bevel(40), Link::Bevel);
        p.normalize();
        let s = p.solve();
        assert!((deg(&p, 0) - 0.5f64.atan().to_degrees()).abs() < 1e-9);
        assert!((deg(&p, 0) + deg(&p, 1) - 90.0).abs() < 1e-9);
        assert_eq!(s[0].frame.origin, s[1].frame.origin);
        assert!(v_dot(s[0].frame.z, s[1].frame.z).abs() < 1e-12);
        assert!((s[1].ratio + 0.5).abs() < 1e-12);
        // The contact line lies on both pitch cones.
        let d0 = deg(&p, 0).to_radians();
        let contact = s[0].frame.turn([d0.sin(), 0.0, d0.cos()]);
        assert!((v_dot(contact, s[1].frame.z) - deg(&p, 1).to_radians().cos()).abs() < 1e-12);
        // At azimuth π in the wheel's own frame, as for a spur mesh.
        let local = [v_dot(contact, s[1].frame.x), v_dot(contact, s[1].frame.y)];
        assert!(local[0] < 0.0 && local[1].abs() < 1e-12);

        // A third gear on the wheel follows the wheel's cone, so its shaft
        // is square to the wheel's too.
        let c = p.add_child(b, PitchUnit::Module.starter_bevel(20), Link::Bevel);
        p.normalize();
        let s = p.solve();
        let i = p.index_of(c).unwrap();
        assert!((deg(&p, i) - deg(&p, 0)).abs() < 1e-9);
        assert!(v_dot(s[1].frame.z, s[i].frame.z).abs() < 1e-9);

        // Another shaft angle.
        if let Placement::Bevel { shaft_angle, .. } = &mut p.gears[1].placement {
            *shaft_angle = 60.0;
        }
        p.normalize();
        let s = p.solve();
        assert!((deg(&p, 0) + deg(&p, 1) - 60.0).abs() < 1e-9);
        assert!((v_dot(s[0].frame.z, s[1].frame.z) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn bevel_on_a_shaft_starts_at_the_back_face_before_it() {
        let mut p = Project::empty();
        let a = p.add_root(PitchUnit::Module.starter_bevel(20), 0.0, 0.0);
        let b = p.add_child(a, PitchUnit::Module.starter_bevel(40), Link::Bevel);
        let c = p.add_child(b, PitchUnit::Module.starter_bevel(16), Link::Axle);
        p.normalize();
        let s = p.solve();
        let i = p.index_of(c).unwrap();
        let back = axial_extent(&p.gears[1].params).1;
        let front = axial_extent(&p.gears[i].params).0;
        let (u, v) = (s[1].frame.apply([0.0, 0.0, back]), s[i].frame.apply([0.0, 0.0, front]));
        assert!((0..3).all(|k| (u[k] - v[k]).abs() < 1e-9), "{u:?} {v:?}");
        assert_eq!(s[i].ratio, s[1].ratio);
    }

    #[test]
    fn parts_are_named_by_kind() {
        let mut p = Project::empty();
        let a = p.add_root(PitchUnit::Module.starter_pulley(20), 0.0, 0.0);
        let b = p.add_child(a, PitchUnit::Module.starter_pulley(30), Link::Belt);
        p.add_child(b, GearParams::default(), Link::Axle);
        let names: Vec<&str> = p.gears.iter().map(|g| g.name.as_str()).collect();
        assert_eq!(names, ["Pulley 1", "Pulley 2", "Gear 1"]);

        // An older file's pulley called "Gear 2" is renamed; a custom name is kept.
        p.gears[0].name = "Gear 2".into();
        p.gears[1].name = "Motor".into();
        p.fix_default_names();
        assert_eq!(p.gears[0].name, "Pulley 1");
        assert_eq!(p.gears[1].name, "Motor");
    }

    #[test]
    fn bevel_worm_sets_are_named_and_start_with_their_shafts_meeting() {
        let mut p = Project::empty();
        let w = p.add_root(PitchUnit::Module.starter_worm(Role::Pinion), 0.0, 0.0);
        p.add_child(w, PitchUnit::Module.starter_worm(Role::Ring), Link::Hypoid);
        assert!(matches!(p.gears[1].placement, Placement::Hypoid { offset: 0.0, .. }));
        // A worm that lost its flag (a hand-edited file).
        p.gears[0].params.worm = false;
        p.normalize();
        let names: Vec<&str> = p.gears.iter().map(|g| g.name.as_str()).collect();
        assert_eq!(names, ["Worm 1", "Worm wheel 1"]);
        assert!(p.gears[0].params.worm, "the wheel sets the pair's kind");
    }

    #[test]
    fn teeth_for_ratio_finds_exact_pairs_near_the_current_size() {
        assert_eq!(teeth_for_ratio(2.5, 12..=100, 12..=100, (20, 40)), Some((18, 45)));
        assert_eq!(teeth_for_ratio(2.5, 12..=100, 12..=100, (40, 60)), Some((28, 70)));
        // A worm: 3 to 7 starts, one wheel tooth per unit of ratio.
        assert_eq!(teeth_for_ratio(12.0, 3..=7, 10..=200, (3, 30)), Some((3, 36)));
        // Out of reach: the nearest the limits allow.
        assert_eq!(teeth_for_ratio(20.0, 12..=100, 12..=100, (20, 40)), Some((12, 100)));
        // A held gear keeps its teeth.
        assert_eq!(teeth_for_ratio(3.0, 17..=17, 12..=100, (17, 30)), Some((17, 51)));
    }

    #[test]
    fn sizing_to_a_ratio_keeps_gears_that_mesh_elsewhere() {
        let mut p = Project::empty();
        let a = p.add_root(PitchUnit::Module.starter(20), 0.0, 0.0);
        let b = p.add_child(a, PitchUnit::Module.starter(30), Link::Mesh);
        let r = p.size_pair_to_ratio(1, 3.0, 120).unwrap();
        assert_eq!((p.gears[0].params.teeth, p.gears[1].params.teeth), (r.driving, r.driven));
        assert_eq!(r.driven, 3 * r.driving);
        assert!(r.held.is_none());
        // The root's own pair is the same one.
        assert_eq!(p.ratio_pair(0), Some((0, 1)));
        // Once b drives a third gear, b holds its teeth for the a-b pair.
        p.add_child(b, PitchUnit::Module.starter(25), Link::Mesh);
        let held = p.gears[1].params.teeth;
        let r = p.size_pair_to_ratio(1, 2.0, 120).unwrap();
        assert_eq!(r.driven, held);
        assert_eq!(r.held.as_deref(), Some(p.gears[1].name.as_str()));
        // A middle gear of three meshes on both sides: nothing to change.
        p.add_child(a, PitchUnit::Module.starter(25), Link::Mesh);
        assert!(p.size_pair_to_ratio(1, 2.0, 120).is_err());
    }

    #[test]
    fn worm_sets_size_to_a_ratio_with_their_starts() {
        let mut p = Project::empty();
        let w = p.add_root(PitchUnit::Module.starter_worm(Role::Pinion), 0.0, 0.0);
        p.add_child(w, PitchUnit::Module.starter_worm(Role::Ring), Link::Hypoid);
        let r = p.size_pair_to_ratio(1, 9.5, 120).unwrap();
        assert_eq!((r.driving, r.driven), (4, 38));
        assert!(p.solve()[1].hypoid.is_some());
    }

    #[test]
    fn worm_drives_are_named_and_sized_by_their_starts() {
        let mut p = Project::empty();
        let w = p.add_root(PitchUnit::Module.starter_worm_drive(worm::Role::Worm), 0.0, 0.0);
        p.add_child(w, PitchUnit::Module.starter_worm_drive(worm::Role::Wheel), Link::Worm);
        p.normalize();
        let names: Vec<&str> = p.gears.iter().map(|g| g.name.as_str()).collect();
        assert_eq!(names, ["Worm 1", "Worm wheel 1"]);
        let s = p.solve();
        assert!((s[1].ratio.abs() - 1.0 / 30.0).abs() < 1e-12);
        // The wheel's axis is square to the worm's, a centre distance off.
        let d = s[1].worm.as_ref().unwrap();
        assert!(v_dot(s[1].frame.z, s[0].frame.z).abs() < 1e-12);
        assert!((d.center - 40.0).abs() < 1e-12);
        let r = p.size_pair_to_ratio(1, 40.0, 120).unwrap();
        assert_eq!((r.driving, r.driven), (1, 40));
        // Exact with 2 starts or 4: 4 keeps the wheel nearer its size.
        let r = p.size_pair_to_ratio(1, 12.5, 120).unwrap();
        assert_eq!((r.driving, r.driven), (4, 50));
    }

    #[test]
    fn worm_drives_are_made_back_drivable_from_either_part() {
        for part in [0, 1] {
            let mut p = Project::empty();
            let w = p.add_root(PitchUnit::Module.starter_worm_drive(worm::Role::Worm), 0.0, 0.0);
            p.add_child(w, PitchUnit::Module.starter_worm_drive(worm::Role::Wheel), Link::Worm);
            p.normalize();
            assert_eq!(p.solve()[1].worm.as_ref().unwrap().back_drive(worm::DEFAULT_FRICTION), worm::BackDrive::Running);
            let (old, new) = p.make_back_drivable(part, true).unwrap();
            assert_eq!((old.starts, old.teeth), (1, 30));
            assert_eq!((new.starts, new.teeth), (3, 90));
            assert_eq!((p.gears[0].params.teeth, p.gears[1].params.teeth), (3, 90));
            let d = p.solve()[1].worm.clone().unwrap();
            assert_eq!(d.back_drive(worm::DEFAULT_FRICTION), worm::BackDrive::Free);
            // The fillet follows the biggest ball that fits, none being chosen.
            assert_eq!(worm::Spec { root_fillet: new.root_fillet, ..d.spec }, new);
        }
        let mut p = Project::empty();
        let w = p.add_root(PitchUnit::Module.starter_worm_drive(worm::Role::Worm), 0.0, 0.0);
        p.add_child(w, PitchUnit::Module.starter_worm_drive(worm::Role::Wheel), Link::Worm);
        p.normalize();
        let (_, new) = p.make_back_drivable(1, false).unwrap();
        assert_eq!((new.starts, new.teeth), (3, 30));
    }

    #[test]
    fn belt_to_a_gear_is_flagged() {
        let mut p = Project::empty();
        let a = p.add_root(GearParams::default(), 0.0, 0.0);
        p.add_child(a, PitchUnit::Module.starter_pulley(20), Link::Belt);
        p.normalize();
        let s = p.solve();
        assert!(s[1].belt.as_ref().unwrap().mismatch);
    }

    #[test]
    fn remove_takes_the_subtree() {
        let mut p = Project::default();
        let first = p.gears[1].id;
        p.add_child(first, GearParams::default(), Link::Axle);
        p.remove(first);
        assert_eq!(p.gears.len(), 1);
    }
}

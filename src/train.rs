//! The project: a gear train as a list of gears and timing pulleys, each
//! either fixed in space (a root), meshing with an earlier gear, belted to
//! an earlier pulley, or sharing an earlier gear's axle (a compound).
//! Parents always come before their children in `gears`, so the train can
//! be solved in one pass and can never loop.

use crate::gear::{self, GearParams, Issue, MeshInfo, ToothStandard};
use crate::bevel::{self, Bevel, BevelGeom};
use crate::pulley::{self, BeltSection};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

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
}

impl Placement {
    pub fn parent(&self) -> Option<u64> {
        match *self {
            Placement::Root { .. } => None,
            Placement::Mesh { parent, .. }
            | Placement::Axle { parent, .. }
            | Placement::Belt { parent, .. }
            | Placement::Bevel { parent, .. } => Some(parent),
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
        let mut p = Self { version: 1, gears: Vec::new(), next_id: 1, driver_rpm: 10.0, pitch_unit: unit };
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
        self.gears.push(Gear { id, name, params, placement: Placement::Root { x, y, phase: 0.0 }, match_parent: true, axial: 0.0 });
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
        self.gears.insert(at, Gear { id, name, params, placement, match_parent: link != Link::Axle, axial: 0.0 });
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
            let is_default = ["Gear", "Pulley", "Bevel"].iter().filter(|k| **k != own).any(|k| {
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

    /// Repair anything a hand-edited or older file could get wrong: unknown
    /// or later parents become roots, linked tooth systems are synced, and
    /// standard rack coefficients are re-derived (fine pitch depends on DP).
    pub fn normalize(&mut self) {
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
                    }
                }
            };
            let mut s = s;
            match (&g.placement, parent) {
                (Placement::Bevel { .. }, _) => {}
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
                if a.params.is_bevel() || b.params.is_bevel() {
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

/// Where a new belt drive starts: about a pulley diameter of daylight
/// between the two.
pub fn starting_center(a: &GearParams, b: &GearParams) -> f64 {
    a.tip_radius() + b.tip_radius() + 2.0 * a.pitch_radius().max(b.pitch_radius())
}

fn kind_name(params: &GearParams) -> &'static str {
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

use crate::bevel::BevelGeom;
use crate::chrome::{self, Icon};
use crate::hypoid::{self, Member, Role};
use crate::view3d;
use crate::worm;
use crate::export::{self, place};
use crate::mesh;
use crate::gear::{self, FillMesh, GearParams, Issue, Level, ToothStandard};
use crate::pulley::{self, BeltSection};
use crate::reach;
use crate::train::{self, Datum, Gear, Link, Placement, PitchUnit, Project, STD_DPS, STD_MODULES, Solved};
use eframe::egui::{
    self, Align2, Color32, FontId, Key, KeyboardShortcut, Modifiers, Pos2, Rect, RichText, Sense, Shape, Stroke, Ui,
    Vec2,
};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};
use std::ops::RangeInclusive;
use std::path::{Path, PathBuf};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};

const FILE_EXT: &str = "gearcraft";


/// A generated pair's teeth (a hypoid set, bevel worm or worm drive),
/// made off the UI thread. The last good solids stay on show while a new
/// spec is being generated.
struct PairJob {
    spec: train::PairSpec,
    rx: Option<mpsc::Receiver<Result<(Member, Member), String>>>,
    members: Option<Arc<(Member, Member)>>,
    error: Option<String>,
}

/// What a reach check is of: a part made from its own settings, or a
/// generated member (its pair's spec, and whether it is the driven one).
#[derive(Clone, PartialEq)]
enum ReachOf {
    Own(GearParams),
    Member(train::PairSpec, bool),
}

/// Whether the ball end mill, square to a part's axis on the 4-axis mill,
/// reaches its teeth: checked off the UI thread for the selected part.
struct ReachJob {
    of: ReachOf,
    ball: f64,
    rx: Option<mpsc::Receiver<Arc<Checked>>>,
    done: Option<Arc<Checked>>,
}

/// A reach check's findings, and the points out of reach on every tooth
/// in the part's frame, for drawing.
struct Checked {
    reach: reach::Reach,
    dots: Vec<[f64; 3]>,
}

struct Cached {
    params: GearParams,
    outline: Vec<[f64; 2]>,
    fill: FillMesh,
    solid: view3d::Solid,
}

/// A change offered beside the problem it clears.
enum Fix {
    WormLength(f64),
    WheelFace(f64),
    /// The project's ball end mill Ø.
    Ball(f64),
}

/// A belt's layout for drawing: its pitch-line loop, offsets of the land,
/// back and tooth tip from the pitch line, tooth half-widths at base and
/// tip, and where each tooth sits along the loop.
struct BeltShape {
    lp: pulley::BeltLoop,
    land: f64,
    back: f64,
    /// The tooth's outline: along the belt, depth below the land.
    tooth: Vec<[f64; 2]>,
    stations: Vec<f64>,
}

/// Canvas camera: world point at the middle of the canvas, pixels per mm.
struct View {
    center: [f64; 2],
    scale: f64,
}

#[derive(Clone, Copy)]
enum Drag {
    Pan,
    /// Moving a root gear; offset from its centre to the grab point.
    Move(u64, [f64; 2]),
    /// Swinging a meshing gear or belted pulley around its parent.
    Swing(u64),
}

#[derive(Clone, Copy, PartialEq)]
enum Pending {
    New(PitchUnit),
    Open,
    Quit,
}

/// What a file dialog's path is for.
enum PathFor {
    Open,
    /// Save the project there, then carry on with what was waiting on it.
    Save(Option<Pending>),
    /// Write an export there, gathered when it was asked for, as `ext`.
    Export(String, Box<dyn FnOnce(&Path) -> std::io::Result<()>>),
}

/// A file dialog, open on a thread of its own: waiting for it on the UI
/// thread stalls the window until the desktop calls it unresponsive.
struct FileDialogJob {
    rx: mpsc::Receiver<Option<PathBuf>>,
    what: PathFor,
}

/// What the ribbon is set up to make. Each kind of gear gets its own
/// workspace and tools; a project can still mix them.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum Workspace {
    Spur,
    Bevel,
    Worm,
    BevelWorm,
    Pulleys,
}

impl Workspace {
    const ALL: [Workspace; 5] = [Workspace::Spur, Workspace::Bevel, Workspace::Worm, Workspace::BevelWorm, Workspace::Pulleys];
    /// On the roadmap, listed so the menu shows where they will go.
    const PLANNED: [&str; 4] = ["Internal & Planetary", "Helical", "Clock Wheels", "Escapements"];

    fn name(self) -> &'static str {
        match self {
            Workspace::Spur => "Spur Gears",
            Workspace::Bevel => "Bevel Gears",
            Workspace::Worm => "Worm Drives",
            Workspace::BevelWorm => "Bevel Worms",
            Workspace::Pulleys => "Timing Pulleys",
        }
    }

    fn glyph(self) -> &'static str {
        match self {
            Workspace::Spur => "⚙",
            Workspace::Bevel => "🔻",
            Workspace::Worm => "🐛",
            Workspace::BevelWorm => "🌀",
            Workspace::Pulleys => "🔗",
        }
    }

    fn icon(self) -> Icon<'static> {
        match self {
            Workspace::Spur => Icon::Gear,
            Workspace::Bevel => Icon::Bevel,
            Workspace::Worm => Icon::WormDrive,
            Workspace::BevelWorm => Icon::Worm,
            Workspace::Pulleys => Icon::Pulley,
        }
    }

    fn hint(self) -> &'static str {
        match self {
            Workspace::Spur => "Involute spur gears: meshing pairs, compound trains",
            Workspace::Bevel => "Straight bevel gears with spherical involute teeth, at any shaft angle",
            Workspace::Worm => "A worm of 1 to 6 starts, straight or hourglass (globoid), turning a wheel at right angles, for big reductions",
            Workspace::BevelWorm => "A 3 to 7-start conical worm driving a wheel, its shaft meeting or passing the wheel's axis",
            Workspace::Pulleys => "MXL to XXH timing pulleys and the belts between them",
        }
    }

    /// Whether the parts have a flat plan view and outline (bevel gears
    /// and worms leave the plane).
    fn flat(self) -> bool {
        !matches!(self, Workspace::Bevel | Workspace::Worm | Workspace::BevelWorm)
    }

    fn of(params: &GearParams) -> Self {
        if params.is_pulley() {
            Workspace::Pulleys
        } else if params.worm_drive.is_some() {
            Workspace::Worm
        } else if params.worm {
            Workspace::BevelWorm
        } else if params.is_bevel() || params.is_hypoid() {
            Workspace::Bevel
        } else {
            Workspace::Spur
        }
    }
}

struct Show {
    pitch: bool,
    base: bool,
    grid: bool,
    labels: bool,
}

pub struct GearCraftApp {
    project: Project,
    /// Last state pushed to undo; compared each frame to spot edits.
    committed: Project,
    undo: Vec<Project>,
    redo: Vec<Project>,
    path: Option<PathBuf>,
    dirty: bool,
    selected: Option<u64>,
    cache: HashMap<u64, Cached>,
    /// Generated pairs by ring gear or wheel id.
    pairs: HashMap<u64, PairJob>,
    /// Reach checks by part id.
    reach: HashMap<u64, ReachJob>,
    view: View,
    fit_pending: bool,
    /// Keep re-fitting when the canvas resizes until the user pans or zooms.
    auto_fit: bool,
    canvas_size: Vec2,
    playing: bool,
    driver_angle: f64,
    show: Show,
    drag: Option<Drag>,
    cursor: Option<[f64; 2]>,
    status: Option<(String, Instant)>,
    pending: Option<Pending>,
    file_dialog: Option<FileDialogJob>,
    /// Asking whether to convert the gears for a Metric/Inch switch.
    unit_switch: Option<PitchUnit>,
    /// Belt fitting picks from the ISO 5296 lengths only.
    belt_standard_only: bool,
    /// The Ratio section's target for each pair, keyed by the driven part.
    ratio_target: HashMap<u64, f64>,
    /// The most teeth "Size to ratio" may give a gear.
    ratio_max_teeth: u32,
    /// What the last sizing did: the driven part, the pair's teeth then
    /// (driving, driven), and the message, shown while those still hold.
    ratio_result: Option<(u64, u32, u32, String)>,
    /// The last Make back-drivable: the wheel, the drive it made, and what
    /// changed.
    back_result: Option<(u64, worm::Spec, String)>,
    /// Make back-drivable gives the wheel more teeth to keep the ratio.
    back_keep_ratio: bool,
    /// The biggest ball end mill a hypoid set takes, by its settings with
    /// no ball: a search over designs, so kept.
    hypoid_ball: Option<(hypoid::Spec, Option<f64>)>,
    workspace: Workspace,
    /// What was selected in each workspace, restored on switching back.
    workspace_selection: HashMap<Workspace, u64>,
    /// Showing the 3D view instead of the plan.
    three_d: bool,
    camera: view3d::Camera,
    /// Last 3D frame's triangles, for picking.
    frame3d: view3d::Frame,
    texture3d: Option<egui::TextureHandle>,
    title: String,
}

impl GearCraftApp {
    pub fn new(cc: &eframe::CreationContext<'_>, file: Option<PathBuf>) -> Self {
        chrome::install_fonts(&cc.egui_ctx);
        let project = Project::default();
        let mut app = Self {
            selected: project.gears.first().map(|g| g.id),
            committed: project.clone(),
            project,
            undo: Vec::new(),
            redo: Vec::new(),
            path: None,
            dirty: false,
            cache: HashMap::new(),
            pairs: HashMap::new(),
            reach: HashMap::new(),
            view: View { center: [0.0, 0.0], scale: 4.0 },
            fit_pending: true,
            auto_fit: true,
            canvas_size: Vec2::ZERO,
            playing: false,
            driver_angle: 0.0,
            show: Show { pitch: true, base: false, grid: true, labels: true },
            drag: None,
            cursor: None,
            status: None,
            pending: None,
            file_dialog: None,
            unit_switch: None,
            belt_standard_only: true,
            ratio_target: HashMap::new(),
            ratio_max_teeth: 120,
            ratio_result: None,
            back_result: None,
            back_keep_ratio: true,
            hypoid_ball: None,
            workspace: Workspace::Spur,
            workspace_selection: HashMap::new(),
            three_d: false,
            camera: view3d::Camera::default(),
            frame3d: view3d::Frame::default(),
            texture3d: None,
            title: String::new(),
        };
        if let Some(f) = file {
            app.load(f);
        }
        app
    }

    /// Make the worm drive part `idx` belongs to back-drivable, noting
    /// what changed for the Back-driving section.
    fn make_back_drivable(&mut self, idx: usize) {
        match self.project.make_back_drivable(idx, self.back_keep_ratio) {
            Ok((old, new)) => {
                let n = |z: u32, what: &str| format!("{z} {what}{}", if z == 1 { "" } else { "s" });
                let mut note = format!(
                    "Worm now {} at {} pitch Ø; wheel {} teeth",
                    n(new.starts, "start"),
                    len(new.diameter, self.project.pitch_unit),
                    new.teeth
                );
                let (was, now) = (old.teeth as f64 / old.starts as f64, new.teeth as f64 / new.starts as f64);
                if (was - now).abs() > 1e-9 {
                    note += &format!("; the ratio is {now:.4} : 1, was {was:.4} : 1");
                }
                if new.length > old.length {
                    note += &format!("; worm lengthened to {}", len(new.length, self.project.pitch_unit));
                }
                let id = self.project.gears[self.pair_driven(idx).unwrap_or(idx)].id;
                self.back_result = Some((id, new, note + "."));
            }
            Err(e) => self.flash(e),
        }
    }

    fn flash(&mut self, msg: impl Into<String>) {
        self.status = Some((msg.into(), Instant::now()));
    }

    // ---- workspaces ------------------------------------------------------

    /// Whether a part belongs to the open workspace. Only those are drawn,
    /// listed and picked; the others wait, untouched, in their own.
    fn shown(&self, g: &Gear) -> bool {
        Workspace::of(&g.params) == self.workspace
    }

    /// Bevel gears only make sense in 3D: their axes leave the plane.
    fn in_3d(&self) -> bool {
        self.three_d || !self.workspace.flat()
    }

    fn first_shown(&self) -> Option<u64> {
        self.project.gears.iter().find(|g| self.shown(g)).map(|g| g.id)
    }

    fn switch_workspace(&mut self, to: Workspace) {
        if to == self.workspace {
            return;
        }
        if let Some(id) = self.selected {
            self.workspace_selection.insert(self.workspace, id);
        }
        self.workspace = to;
        self.selected = self
            .workspace_selection
            .get(&to)
            .copied()
            .filter(|id| self.project.get(*id).is_some_and(|g| self.shown(g)))
            .or_else(|| self.first_shown());
        self.drag = None;
        self.fit_pending = true;
    }

    // ---- files -----------------------------------------------------------

    fn replace_project(&mut self, project: Project, path: Option<PathBuf>) {
        self.selected = project.gears.first().map(|g| g.id);
        self.workspace_selection.clear();
        if let Some(g) = project.gears.first() {
            self.workspace = Workspace::of(&g.params);
        }
        let mut project = project;
        project.fix_default_names();
        self.committed = project.clone();
        self.project = project;
        self.undo.clear();
        self.redo.clear();
        self.path = path;
        self.dirty = false;
        self.driver_angle = 0.0;
        self.fit_pending = true;
    }

    fn load(&mut self, path: PathBuf) {
        let result = std::fs::read_to_string(&path)
            .map_err(|e| e.to_string())
            .and_then(|t| serde_json::from_str::<Project>(&t).map_err(|e| e.to_string()));
        match result {
            Ok(mut p) => {
                p.normalize();
                let name = path.display().to_string();
                self.replace_project(p, Some(path));
                self.flash(format!("Opened {name}"));
            }
            Err(e) => self.flash(format!("Could not open {}: {e}", path.display())),
        }
    }

    /// Open a file dialog filtered to `label` (`ext`): to save, offering
    /// `name`, or to pick a file. The path comes back to
    /// `poll_file_dialog`, which uses it for `what`.
    fn ask_path(&mut self, label: &str, ext: &str, name: Option<String>, what: PathFor) {
        if self.file_dialog.is_some() {
            self.flash("A file dialog is already open");
            return;
        }
        let (tx, rx) = mpsc::channel();
        let (label, ext) = (label.to_owned(), ext.to_owned());
        std::thread::spawn(move || {
            let dialog = rfd::FileDialog::new().add_filter(&label, &[&ext]);
            let path = match name {
                Some(name) => dialog.set_file_name(name).save_file(),
                None => dialog.pick_file(),
            };
            let _ = tx.send(path);
        });
        self.file_dialog = Some(FileDialogJob { rx, what });
    }

    /// Carry out a file dialog's job once it has an answer.
    fn poll_file_dialog(&mut self, ctx: &egui::Context) {
        let Some(job) = &self.file_dialog else { return };
        let path = match job.rx.try_recv() {
            Ok(path) => path,
            Err(mpsc::TryRecvError::Empty) => {
                ctx.request_repaint_after(Duration::from_millis(100));
                return;
            }
            Err(mpsc::TryRecvError::Disconnected) => None,
        };
        let Some(job) = self.file_dialog.take() else { return };
        // None: the dialog was cancelled.
        let Some(mut path) = path else { return };
        match job.what {
            PathFor::Open => self.load(path),
            PathFor::Save(then) => {
                if path.extension().is_none() {
                    path.set_extension(FILE_EXT);
                }
                if self.write_project(path) {
                    if let Some(what) = then {
                        self.perform(ctx, what);
                    }
                }
            }
            PathFor::Export(ext, write) => {
                if path.extension().is_none() {
                    path.set_extension(ext);
                }
                match write(&path) {
                    Ok(()) => self.flash(format!("Exported {}", path.display())),
                    Err(e) => self.flash(format!("Could not write {}: {e}", path.display())),
                }
            }
        }
    }

    fn open_dialog(&mut self) {
        self.ask_path("GearCraft project", FILE_EXT, None, PathFor::Open);
    }

    /// Save to the project's file, asking where first if it has none (or
    /// for Save As); `then` follows once it is saved. True only if it was
    /// saved here and now, without asking.
    fn save(&mut self, save_as: bool, then: Option<Pending>) -> bool {
        match (&self.path, save_as) {
            (Some(p), false) => self.write_project(p.clone()),
            _ => {
                self.ask_path("GearCraft project", FILE_EXT, Some(format!("train.{FILE_EXT}")), PathFor::Save(then));
                false
            }
        }
    }

    fn write_project(&mut self, path: PathBuf) -> bool {
        let text = serde_json::to_string_pretty(&self.project).expect("project serialises");
        match std::fs::write(&path, text) {
            Ok(()) => {
                self.flash(format!("Saved {}", path.display()));
                self.path = Some(path);
                self.dirty = false;
                true
            }
            Err(e) => {
                self.flash(format!("Could not save {}: {e}", path.display()));
                false
            }
        }
    }

    /// Export as `ext`: dxf or svg outlines, a step solid, or an stl, 3mf
    /// or obj mesh.
    fn export(&mut self, whole_train: bool, ext: &str) {
        let only = if whole_train { None } else { self.selected };
        // Teeth already generated for the current settings are reused.
        let cached = |id: u64| self.pairs.get(&id).filter(|j| j.rx.is_none()).and_then(|j| j.members.clone());
        if matches!(ext, "stl" | "3mf" | "obj") {
            let parts = mesh::parts(&self.project, self.driver_angle, only, |g| self.shown(g), cached);
            let first = parts.first().map(|p| p.name.as_str());
            let first = first.map(str::to_owned);
            self.save_export(whole_train, first, ext, Box::new(move |path| mesh::write(path, &parts)));
            return;
        }
        let shapes = export::shapes(&self.project, self.driver_angle, only, |g| self.shown(g));
        let bevels = export::bevels(&self.project, self.driver_angle, only, |g| self.shown(g));
        let hypoids = if ext == "step" {
            export::hypoids(&self.project, self.driver_angle, only, |g| self.shown(g), cached)
        } else {
            Vec::new()
        };
        let solid_only = only.and_then(|id| self.project.get(id)).is_some_and(|g| g.params.is_generated());
        if (!bevels.is_empty() || solid_only) && ext != "step" {
            self.flash("DXF and SVG are flat outlines: export bevel gears as STEP or a mesh.");
            return;
        }
        let first = shapes.first().map(|s| &s.name).or(bevels.first().map(|b| &b.name)).or(hypoids.first().map(|h| &h.name));
        let first = first.cloned();
        let inch = self.project.pitch_unit.is_inch();
        self.save_export(whole_train, first, ext, Box::new(move |path| export::write(path, &shapes, &bevels, &hypoids, inch)));
    }

    /// Ask where to put an export, named after the project for the whole
    /// train or after the part, `first`, for one, and write it there.
    fn save_export(
        &mut self,
        whole_train: bool,
        first: Option<String>,
        ext: &str,
        write: Box<dyn FnOnce(&Path) -> std::io::Result<()>>,
    ) {
        let Some(first) = first else {
            self.flash("Nothing to export");
            return;
        };
        let label = ext.to_ascii_uppercase();
        let stem = if whole_train {
            self.path.as_ref().and_then(|p| p.file_stem()).map(|s| s.to_string_lossy().into_owned()).unwrap_or("train".into())
        } else {
            first.replace(' ', "_")
        };
        self.ask_path(&label, ext, Some(format!("{stem}.{ext}")), PathFor::Export(ext.to_owned(), write));
    }

    /// Run a New/Open/Quit, or ask first if there are unsaved changes.
    fn request(&mut self, ctx: &egui::Context, what: Pending) {
        if self.dirty {
            self.pending = Some(what);
        } else {
            self.perform(ctx, what);
        }
    }

    fn perform(&mut self, ctx: &egui::Context, what: Pending) {
        match what {
            Pending::New(unit) => self.replace_project(Project::new(unit), None),
            Pending::Open => self.open_dialog(),
            Pending::Quit => {
                self.dirty = false;
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
        }
    }

    // ---- editing ---------------------------------------------------------

    /// Add a gear (or pulley, if `pulley`) linked to the selected one.
    fn add_gear(&mut self, link: Link, pulley: bool) {
        let Some(parent) = self.selected.and_then(|id| self.project.get(id)) else {
            self.add_train(pulley);
            return;
        };
        let pp = &parent.params;
        let unit = self.project.pitch_unit;
        match link {
            Link::Mesh if pp.is_pulley() => {
                self.flash("Pulleys don't mesh: add a belted pulley (B), or a gear on its axle (A).");
                return;
            }
            Link::Belt if !pp.is_pulley() => {
                self.flash("Belts run between pulleys: put a pulley on this gear's axle first (P).");
                return;
            }
            _ => {}
        }
        // A meshing gear or belted pulley starts as a copy so the pair is
        // 1:1 until edited; a compound is usually a fraction of its wheel.
        let params = match link {
            Link::Mesh | Link::Belt | Link::Bevel | Link::Hypoid | Link::Worm => pp.clone(),
            Link::Axle => match (pulley, pp.belt) {
                (true, Some(b)) => GearParams { teeth: (pp.teeth / 2).max(b.min_teeth()), ..pp.clone() },
                (true, None) => GearParams { bore: pp.bore, ..unit.starter_pulley(20) },
                (false, Some(_)) => GearParams { bore: pp.bore, ..unit.starter(24) },
                (false, None) => GearParams { teeth: (pp.teeth / 4).max(8), bore: pp.bore, ..pp.clone() },
            },
        };
        let id = self.project.add_child(parent.id, params, link);
        self.selected = Some(id);
    }

    /// Add a bevel gear meshing with (or on the shaft of) the selected
    /// one, or a fresh pair if none is selected.
    fn add_bevel(&mut self, link: Link) {
        let Some(parent) = self.selected.and_then(|id| self.project.get(id)).filter(|g| g.params.is_bevel()) else {
            self.add_bevel_pair();
            return;
        };
        let pp = &parent.params;
        let params = match link {
            Link::Axle => GearParams { teeth: (pp.teeth / 2).max(12), ..pp.clone() },
            _ => pp.clone(),
        };
        let id = self.project.add_child(parent.id, params, link);
        self.selected = Some(id);
    }

    /// A 20-tooth pinion driving a 40-tooth wheel at right angles.
    fn add_bevel_pair(&mut self) {
        let x = self.clear_x();
        let unit = self.project.pitch_unit;
        let pinion = self.project.add_root(unit.starter_bevel(20), x, 0.0);
        let wheel = self.project.add_child(pinion, unit.starter_bevel(40), Link::Bevel);
        self.selected = Some(wheel);
    }

    /// A car-style hypoid set: an 11-tooth pinion driving a 41-tooth ring
    /// gear, its shaft 30 mm below the ring's centre.
    fn add_hypoid_pair(&mut self) {
        let x = self.clear_x();
        let unit = self.project.pitch_unit;
        let pinion = self.project.add_root(unit.starter_hypoid(Role::Pinion), x, 0.0);
        let ring = self.project.add_child(pinion, unit.starter_hypoid(Role::Ring), Link::Hypoid);
        self.project.normalize();
        self.selected = Some(ring);
    }

    /// A bevel worm set: a 3-start worm driving a 30-tooth wheel, the
    /// shafts meeting at right angles.
    fn add_worm_pair(&mut self) {
        let x = self.clear_x();
        let unit = self.project.pitch_unit;
        let worm = self.project.add_root(unit.starter_worm(Role::Pinion), x, 0.0);
        let wheel = self.project.add_child(worm, unit.starter_worm(Role::Ring), Link::Hypoid);
        self.project.normalize();
        self.selected = Some(wheel);
    }

    /// A worm drive: a single-start worm turning a 30-tooth wheel.
    fn add_worm_drive(&mut self) {
        let x = self.clear_x();
        let unit = self.project.pitch_unit;
        let w = self.project.add_root(unit.starter_worm_drive(worm::Role::Worm), x, 0.0);
        let wheel = self.project.add_child(w, unit.starter_worm_drive(worm::Role::Wheel), Link::Worm);
        self.project.normalize();
        self.selected = Some(wheel);
    }

    /// A globoid drive: a single-start hourglass worm wrapped round a
    /// 40-tooth wheel, sized by AGMA 6135.
    fn add_globoid_drive(&mut self) {
        let x = self.clear_x();
        let unit = self.project.pitch_unit;
        let w = self.project.add_root(unit.starter_worm_drive(worm::Role::Worm), x, 0.0);
        let wheel = GearParams { teeth: 40, ..unit.starter_worm_drive(worm::Role::Wheel) };
        let wheel = self.project.add_child(w, wheel, Link::Worm);
        if let Some(i) = self.project.index_of(wheel) {
            self.project.make_globoid(i, true);
        }
        self.selected = Some(wheel);
    }

    /// Where a new train can go, clear of the parts on show.
    fn clear_x(&self) -> f64 {
        let solved = self.project.solve();
        let right = self
            .project
            .gears
            .iter()
            .zip(&solved)
            .filter(|(g, _)| self.shown(g))
            .map(|(g, s)| s.center[0] + g.params.tip_radius())
            .fold(f64::MIN, f64::max);
        if right == f64::MIN { 0.0 } else { right + 40.0 }
    }

    fn add_train(&mut self, pulley: bool) {
        let solved = self.project.solve();
        let right = self
            .project
            .gears
            .iter()
            .zip(&solved)
            .filter(|(g, _)| self.shown(g))
            .map(|(g, s)| s.center[0] + g.params.tip_radius())
            .fold(f64::MIN, f64::max);
        let x = if right == f64::MIN { 0.0 } else { right + 40.0 };
        let unit = self.project.pitch_unit;
        let params = if pulley { unit.starter_pulley(20) } else { unit.starter(24) };
        let id = self.project.add_root(params, x, 0.0);
        self.selected = Some(id);
    }

    fn delete_selected(&mut self) {
        if let Some(id) = self.selected {
            let i = self.project.index_of(id).unwrap_or(0);
            self.project.remove(id);
            self.selected = self.project.gears[..i.min(self.project.gears.len())]
                .iter()
                .rev()
                .find(|g| self.shown(g))
                .map(|g| g.id)
                .or_else(|| self.first_shown());
        }
    }

    /// Fold edits into the undo history once the user lets go: no button
    /// held and no field being typed in, so a drag is one undo step.
    fn track_history(&mut self, ctx: &egui::Context) {
        if self.project == self.committed {
            return;
        }
        let busy = ctx.input(|i| i.pointer.any_down()) || ctx.memory(|m| m.focused().is_some());
        if !busy {
            let prev = std::mem::replace(&mut self.committed, self.project.clone());
            self.undo.push(prev);
            if self.undo.len() > 200 {
                self.undo.remove(0);
            }
            self.redo.clear();
        }
        self.dirty = true;
    }

    fn undo(&mut self) {
        if let Some(p) = self.undo.pop() {
            self.redo.push(std::mem::replace(&mut self.project, p));
            self.after_history();
        }
    }

    fn redo(&mut self) {
        if let Some(p) = self.redo.pop() {
            self.undo.push(std::mem::replace(&mut self.project, p));
            self.after_history();
        }
    }

    fn after_history(&mut self) {
        self.committed = self.project.clone();
        self.dirty = true;
        if !self.selected.and_then(|id| self.project.get(id)).is_some_and(|g| self.shown(g)) {
            self.selected = self.first_shown();
        }
    }

    fn refresh_cache(&mut self, ctx: &egui::Context) {
        let ids: Vec<u64> = self.project.gears.iter().map(|g| g.id).collect();
        self.cache.retain(|id, _| ids.contains(id));
        self.refresh_pairs(ctx);
        self.reach.retain(|id, _| ids.contains(id));
        self.refresh_reach(ctx);
        for g in &self.project.gears {
            if g.params.is_generated() || self.cache.get(&g.id).is_some_and(|c| c.params == g.params) {
                continue;
            }
            if let Some(geom) = BevelGeom::new(&g.params).filter(|_| g.params.is_buildable()) {
                let solid = view3d::bevel_solid(&geom);
                let fill = FillMesh { verts: Vec::new(), tris: Vec::new() };
                self.cache.insert(g.id, Cached { params: g.params.clone(), outline: Vec::new(), fill, solid });
                continue;
            }
            match gear::tooth_profile(&g.params).filter(|_| !g.params.is_bevel()) {
                Some(prof) => {
                    let outline = gear::outline(&g.params, &prof);
                    let fill = gear::fill_mesh(&g.params, &prof);
                    let solid = view3d::gear_solid(&g.params, &prof, &fill);
                    self.cache.insert(g.id, Cached { params: g.params.clone(), outline, fill, solid });
                }
                None => {
                    self.cache.remove(&g.id);
                }
            }
        }
    }

    /// Start generating any pair whose spec changed, and pick up finished
    /// ones.
    fn refresh_pairs(&mut self, ctx: &egui::Context) {
        let mut live = Vec::new();
        for ring in &self.project.gears {
            let Some(pinion) = ring.placement.parent().and_then(|p| self.project.get(p)) else { continue };
            let Some(spec) = train::pair_spec(&pinion.params, &ring.params, &ring.placement, self.project.ball()) else { continue };
            live.push(ring.id);
            let job = self.pairs.entry(ring.id).or_insert_with(|| PairJob {
                spec: spec.clone(),
                rx: None,
                members: None,
                error: None,
            });
            if job.spec != spec || (job.rx.is_none() && job.members.is_none() && job.error.is_none()) {
                let (tx, rx) = mpsc::channel();
                let (sp, ctx) = (spec.clone(), ctx.clone());
                std::thread::spawn(move || {
                    let _ = tx.send(sp.generate());
                    ctx.request_repaint();
                });
                job.spec = spec;
                job.rx = Some(rx);
                // The old settings' error no longer applies.
                job.error = None;
            }
            let done = job.rx.as_ref().and_then(|rx| rx.try_recv().ok());
            if let Some(r) = done {
                job.rx = None;
                match r {
                    Ok(pair) => {
                        job.members = Some(Arc::new(pair));
                        job.error = None;
                    }
                    Err(e) => {
                        job.members = None;
                        job.error = Some(e);
                    }
                }
                let empty = || FillMesh { verts: Vec::new(), tris: Vec::new() };
                match &job.members {
                    Some(pair) => {
                        for (id, params, m) in [(pinion.id, &pinion.params, &pair.0), (ring.id, &ring.params, &pair.1)] {
                            let solid = view3d::hypoid_solid(m);
                            self.cache.insert(id, Cached { params: params.clone(), outline: Vec::new(), fill: empty(), solid });
                        }
                    }
                    None => {
                        self.cache.remove(&pinion.id);
                        self.cache.remove(&ring.id);
                    }
                }
            }
        }
        self.pairs.retain(|id, _| live.contains(id));
    }

    /// What part `idx`'s reach check is of, and its generated pair; None
    /// with no ball chosen, or while a generated part's teeth are not
    /// ready.
    fn reach_of(&self, idx: usize) -> Option<(ReachOf, Option<Arc<(Member, Member)>>)> {
        let g = &self.project.gears[idx];
        if self.project.ball() <= 0.0 || !g.params.is_buildable() {
            return None;
        }
        if !g.params.is_generated() {
            return Some((ReachOf::Own(g.params.clone()), None));
        }
        let r = self.pair_driven(idx)?;
        let job = self.pairs.get(&self.project.gears[r].id).filter(|j| j.rx.is_none())?;
        Some((ReachOf::Member(job.spec.clone(), r == idx), Some(job.members.clone()?)))
    }

    /// Part `idx`'s reach check: None when there is none to make, else
    /// its findings, or None inside while it runs.
    fn reach_state(&self, idx: usize) -> Option<Option<Arc<Checked>>> {
        let (of, _) = self.reach_of(idx)?;
        let ball = self.project.ball();
        let job = self.reach.get(&self.project.gears[idx].id).filter(|j| j.of == of && j.ball == ball);
        Some(job.and_then(|j| j.done.clone()))
    }

    /// Start checking the selected part's reach when what it is made from
    /// changes, and pick up finished checks.
    fn refresh_reach(&mut self, ctx: &egui::Context) {
        for job in self.reach.values_mut() {
            if let Some(c) = job.rx.as_ref().and_then(|rx| rx.try_recv().ok()) {
                job.rx = None;
                job.done = Some(c);
            }
        }
        let Some(idx) = self.selected.and_then(|id| self.project.index_of(id)) else { return };
        let Some((of, pair)) = self.reach_of(idx) else { return };
        let ball = self.project.ball();
        let id = self.project.gears[idx].id;
        if self.reach.get(&id).is_some_and(|j| j.of == of && j.ball == ball) {
            return;
        }
        let (tx, rx) = mpsc::channel();
        let (what, ctx) = (of.clone(), ctx.clone());
        std::thread::spawn(move || {
            let (surface, teeth, face) = match (&what, &pair) {
                (ReachOf::Member(_, driven), Some(pair)) => {
                    let m = if *driven { &pair.1 } else { &pair.0 };
                    (Some(reach::member(m, ball)), m.teeth, None)
                }
                (ReachOf::Own(p), _) if p.is_bevel() => (BevelGeom::new(p).map(|g| reach::bevel(&g, ball)), p.teeth, None),
                (ReachOf::Own(p), _) => (reach::spur(p, ball), p.teeth, Some(p.face_width)),
                _ => (None, 1, None),
            };
            let reach = surface.map(|s| reach::check(&s, ball)).unwrap_or_default();
            let dots = reach::on_every_tooth(&reach.missed, teeth, face);
            let _ = tx.send(Arc::new(Checked { reach, dots }));
            ctx.request_repaint();
        });
        self.reach.insert(id, ReachJob { of, ball, rx: Some(rx), done: None });
    }

    /// Whether any pair is still being generated.
    fn pair_pending(&self) -> bool {
        self.pairs.values().any(|j| j.rx.is_some())
    }

    /// The generated pair a part belongs to: its ring gear's or wheel's
    /// index.
    fn pair_driven(&self, idx: usize) -> Option<usize> {
        let g = &self.project.gears[idx];
        if !g.params.is_generated() {
            return None;
        }
        if matches!(g.placement, Placement::Hypoid { .. } | Placement::Worm { .. }) {
            return Some(idx);
        }
        self.project
            .gears
            .iter()
            .position(|r| matches!(r.placement, Placement::Hypoid { parent, .. } | Placement::Worm { parent, .. } if parent == g.id))
    }

    /// Heights of part `idx`'s end faces along its own axis, or None while
    /// a generated part's teeth are not ready.
    fn own_faces(&self, idx: usize) -> Option<[f64; 2]> {
        let g = &self.project.gears[idx];
        let Some(r) = self.pair_driven(idx) else { return Some(export::faces(&g.params, None)) };
        let pair = self.pairs.get(&self.project.gears[r].id).filter(|j| j.rx.is_none())?.members.clone()?;
        Some(export::faces(&g.params, Some(if r == idx { &pair.1 } else { &pair.0 })))
    }

    // ---- view ------------------------------------------------------------

    fn to_screen(&self, rect: Rect, p: [f64; 2]) -> Pos2 {
        let c = rect.center();
        Pos2::new(
            c.x + ((p[0] - self.view.center[0]) * self.view.scale) as f32,
            c.y - ((p[1] - self.view.center[1]) * self.view.scale) as f32,
        )
    }

    fn to_world(&self, rect: Rect, p: Pos2) -> [f64; 2] {
        let c = rect.center();
        [
            self.view.center[0] + (p.x - c.x) as f64 / self.view.scale,
            self.view.center[1] - (p.y - c.y) as f64 / self.view.scale,
        ]
    }

    fn fit(&mut self, rect: Rect, solved: &[Solved]) {
        let mut lo = [f64::MAX; 2];
        let mut hi = [f64::MIN; 2];
        for (g, s) in self.project.gears.iter().zip(solved).filter(|(g, _)| self.shown(g)) {
            let r = g.params.tip_radius().max(1.0);
            for k in 0..2 {
                lo[k] = lo[k].min(s.center[k] - r);
                hi[k] = hi[k].max(s.center[k] + r);
            }
        }
        if lo[0] > hi[0] {
            self.view = View { center: [0.0, 0.0], scale: 4.0 };
            return;
        }
        self.view.center = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
        let w = (hi[0] - lo[0]).max(1.0);
        let h = (hi[1] - lo[1]).max(1.0);
        self.view.scale = (rect.width() as f64 / w).min(rect.height() as f64 / h) * 0.85;
    }

    /// The smallest gear under the pointer, so a pinion on top of its wheel
    /// can still be picked.
    /// Overlaps among the parts on screen.
    fn visible_collisions(&self, solved: &[Solved]) -> Vec<(usize, usize)> {
        let g = &self.project.gears;
        self.project.collisions(solved).into_iter().filter(|&(i, j)| self.shown(&g[i]) && self.shown(&g[j])).collect()
    }

    fn hit(&self, solved: &[Solved], w: [f64; 2]) -> Option<u64> {
        self.project
            .gears
            .iter()
            .zip(solved)
            .filter(|(g, s)| self.shown(g) && (w[0] - s.center[0]).hypot(w[1] - s.center[1]) <= g.params.tip_radius())
            .min_by(|a, b| a.0.params.tip_radius().total_cmp(&b.0.params.tip_radius()))
            .map(|(g, _)| g.id)
    }

    fn canvas(&mut self, ui: &mut Ui) {
        if self.in_3d() {
            self.canvas_3d(ui);
            return;
        }
        let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = resp.rect;
        let mut solved = self.project.solve();
        if rect.size() != self.canvas_size {
            self.canvas_size = rect.size();
            self.fit_pending |= self.auto_fit;
        }
        if self.fit_pending {
            self.fit(rect, &solved);
            self.fit_pending = false;
            self.auto_fit = true;
        }

        if let Some(hover) = resp.hover_pos() {
            let (scroll, zoom) = ui.input(|i| (i.smooth_scroll_delta().y, i.zoom_delta()));
            let factor = zoom as f64 * (scroll as f64 * 0.0025).exp();
            if factor != 1.0 {
                self.auto_fit = false;
                let before = self.to_world(rect, hover);
                self.view.scale = (self.view.scale * factor).clamp(0.02, 5000.0);
                let after = self.to_world(rect, hover);
                self.view.center[0] += before[0] - after[0];
                self.view.center[1] += before[1] - after[1];
            }
        }
        self.cursor = resp.hover_pos().map(|p| self.to_world(rect, p));

        if resp.drag_started() {
            let grab = resp.interact_pointer_pos().map(|p| self.to_world(rect, p));
            let primary = ui.input(|i| i.pointer.primary_down());
            self.drag = Some(Drag::Pan);
            if let (true, Some(w)) = (primary, grab) {
                if let Some(id) = self.hit(&solved, w) {
                    self.selected = Some(id);
                    let i = self.project.index_of(id).unwrap();
                    match self.project.gears[i].placement {
                        Placement::Root { .. } => {
                            let c = solved[i].center;
                            self.drag = Some(Drag::Move(id, [w[0] - c[0], w[1] - c[1]]));
                        }
                        Placement::Mesh { .. } | Placement::Belt { .. } => self.drag = Some(Drag::Swing(id)),
                        Placement::Axle { .. } | Placement::Bevel { .. } | Placement::Hypoid { .. } | Placement::Worm { .. } => {}
                    }
                }
            }
        }
        if resp.dragged() {
            let pointer = resp.interact_pointer_pos().map(|p| self.to_world(rect, p));
            match (self.drag, pointer) {
                (Some(Drag::Pan), _) => {
                    self.auto_fit = false;
                    let d = resp.drag_delta();
                    self.view.center[0] -= d.x as f64 / self.view.scale;
                    self.view.center[1] += d.y as f64 / self.view.scale;
                }
                (Some(Drag::Move(id, off)), Some(w)) => {
                    if let Some(g) = self.project.gears.iter_mut().find(|g| g.id == id) {
                        if let Placement::Root { x, y, .. } = &mut g.placement {
                            *x = w[0] - off[0];
                            *y = w[1] - off[1];
                        }
                    }
                }
                (Some(Drag::Swing(id)), Some(w)) => {
                    let i = self.project.index_of(id).unwrap();
                    let pi = self.project.gears[i].placement.parent().and_then(|p| self.project.index_of(p));
                    if let (Placement::Mesh { angle, .. } | Placement::Belt { angle, .. }, Some(pi)) =
                        (&mut self.project.gears[i].placement, pi)
                    {
                        let pc = solved[pi].center;
                        let mut a = (w[1] - pc[1]).atan2(w[0] - pc[0]).to_degrees();
                        if ui.input(|i| i.modifiers.shift) {
                            a = (a / 15.0).round() * 15.0;
                        }
                        *angle = (a + 360.0) % 360.0;
                    }
                }
                _ => {}
            }
            solved = self.project.solve();
        }
        if resp.drag_stopped() {
            self.drag = None;
        }
        if resp.clicked() {
            if let Some(p) = resp.interact_pointer_pos() {
                self.selected = self.hit(&solved, self.to_world(rect, p));
            }
        }
        if resp.double_clicked() {
            self.fit_pending = true;
        }

        let painter = painter.with_clip_rect(rect);
        let visuals = ui.visuals().clone();
        painter.rect_filled(rect, 0.0, visuals.extreme_bg_color);
        if self.show.grid {
            self.draw_grid(&painter, rect, &visuals);
        }
        self.refresh_cache(ui.ctx());
        let collisions = self.visible_collisions(&solved);
        self.draw_gears(&painter, rect, &visuals, &solved, &collisions);

        if !self.project.gears.iter().any(|g| self.shown(g)) {
            painter.text(
                rect.center(),
                Align2::CENTER_CENTER,
                match self.workspace {
                    Workspace::Spur => "No gears yet. Use \"New Train\" to place one.",
                    Workspace::Bevel => "No bevel gears yet. Use \"New Pair\" to place one.",
                    Workspace::Worm => "No worm drives yet. Use \"Worm Drive\" to place one.",
                    Workspace::BevelWorm => "No bevel worms yet. Use \"Worm Set\" to place one.",
                    Workspace::Pulleys => "No pulleys yet. Use \"New Pulley\" to place one.",
                },
                FontId::proportional(16.0),
                visuals.weak_text_color(),
            );
        }
        let hint = if rect.width() > 820.0 {
            "Scroll: zoom · Drag empty space: pan · Drag a part: move it, or swing it round its \
             partner (Shift snaps 15°) · Double-click: fit"
        } else {
            "Scroll zoom · drag pan · drag part to move/swing"
        };
        painter.text(
            rect.left_bottom() + Vec2::new(8.0, -6.0),
            Align2::LEFT_BOTTOM,
            hint,
            FontId::proportional(11.0),
            visuals.weak_text_color(),
        );
    }

    fn draw_grid(&self, painter: &egui::Painter, rect: Rect, visuals: &egui::Visuals) {
        // Minor spacing is the smallest power of ten (in the project's
        // units) at least 12 px apart.
        let unit = self.project.pitch_unit;
        let exp = (12.0 / (self.view.scale * unit.factor())).log10().ceil() as i32;
        let minor = 10f64.powi(exp) * unit.factor();
        let lo = self.to_world(rect, rect.left_bottom());
        let hi = self.to_world(rect, rect.right_top());
        let base = visuals.weak_text_color();
        let faint = base.gamma_multiply(0.12);
        let strong = base.gamma_multiply(0.25);
        let axis = base.gamma_multiply(0.5);
        let mut i = (lo[0] / minor).floor() as i64;
        while (i as f64) * minor <= hi[0] {
            let x = i as f64 * minor;
            let color = if i == 0 { axis } else if i % 10 == 0 { strong } else { faint };
            let sx = self.to_screen(rect, [x, 0.0]).x;
            painter.line_segment([Pos2::new(sx, rect.top()), Pos2::new(sx, rect.bottom())], Stroke::new(1.0, color));
            i += 1;
        }
        let mut j = (lo[1] / minor).floor() as i64;
        while (j as f64) * minor <= hi[1] {
            let y = j as f64 * minor;
            let color = if j == 0 { axis } else if j % 10 == 0 { strong } else { faint };
            let sy = self.to_screen(rect, [0.0, y]).y;
            painter.line_segment([Pos2::new(rect.left(), sy), Pos2::new(rect.right(), sy)], Stroke::new(1.0, color));
            j += 1;
        }
        let label = format!("grid {:.*}{}", (-exp).max(0) as usize, minor / unit.factor(), unit.suffix());
        painter.text(rect.right_top() + Vec2::new(-8.0, 6.0), Align2::RIGHT_TOP, label, FontId::proportional(11.0), base);
    }

    /// Where the belt driving part `i` runs, at the current drive angle;
    /// None if there is no workable belt (drawn as a dashed error line).
    fn belt_shape(&self, i: usize, solved: &[Solved]) -> Option<BeltShape> {
        let g = &self.project.gears[i];
        let s = &solved[i];
        let bi = s.belt.as_ref().filter(|b| !b.too_short && !b.mismatch)?;
        let pi = g.placement.parent().and_then(|p| self.project.index_of(p))?;
        let pp = &self.project.gears[pi].params;
        let section = g.params.belt?;
        let lp = pulley::BeltLoop::new(solved[pi].center, pp.pitch_radius(), s.center, g.params.pitch_radius())?;
        let tooth = section.belt_tooth();
        // Offsets from the pitch line, outward positive: the belt's land
        // rides on the pulley tips, half the PLD inside it.
        let land = -section.rack(g.params.teeth).pld / 2.0;
        let back = land + tooth.thickness - tooth.height;
        let pitch = lp.len() / bi.teeth as f64;
        let rot = solved[pi].phase + solved[pi].ratio * self.driver_angle;
        let s0 = lp.s_on_first(rot + pulley::groove_angle(pp.teeth));
        let stations = (0..bi.teeth).map(|k| s0 + k as f64 * pitch).collect();
        Some(BeltShape { lp, land, back, tooth: tooth.outline(), stations })
    }

    /// Belts drawn as they sit: the backing on the pulley tips, each tooth
    /// in a groove. The loop is laid out from the parent pulley's grooves,
    /// and the solve phases the driven pulley to match, so they stay
    /// engaged as the train runs.
    fn draw_belts(&self, painter: &egui::Painter, rect: Rect, visuals: &egui::Visuals, solved: &[Solved]) {
        let color = if visuals.dark_mode { Color32::from_gray(88) } else { Color32::from_gray(52) };
        for (i, (g, s)) in self.project.gears.iter().zip(solved).enumerate() {
            if s.belt.is_none() || !self.shown(g) {
                continue;
            }
            let Some(b) = self.belt_shape(i, solved) else {
                if let Some(pi) = g.placement.parent().and_then(|p| self.project.index_of(p)) {
                    let ends = [self.to_screen(rect, solved[pi].center), self.to_screen(rect, s.center)];
                    painter.extend(Shape::dashed_line(&ends, Stroke::new(1.5, visuals.error_fg_color), 6.0, 4.0));
                }
                continue;
            };
            let place = |s: f64, off: f64, along: f64| {
                let (p, n) = b.lp.at(s);
                let t = [-n[1], n[0]];
                self.to_screen(rect, [p[0] + n[0] * off + t[0] * along, p[1] + n[1] * off + t[1] * along])
            };

            let len = b.lp.len();
            let n = ((len * self.view.scale / 3.0) as usize).clamp(120, 4000);
            let mid = (b.land + b.back) / 2.0;
            let mut band: Vec<Pos2> = (0..=n).map(|k| place(len * k as f64 / n as f64, mid, 0.0)).collect();
            band.push(band[1]);
            let width = ((b.back - b.land) * self.view.scale) as f32;
            painter.add(Shape::line(band, Stroke::new(width.max(1.5), color)));

            if len / b.stations.len() as f64 * self.view.scale < 3.0 {
                continue;
            }
            // Start inside the backing so tooth and band join without a seam.
            let root = b.land + 0.25 * (b.back - b.land);
            let (first, last) = (b.tooth[0][0], b.tooth[b.tooth.len() - 1][0]);
            for &at in &b.stations {
                let mut shape = vec![place(at, root, first)];
                shape.extend(b.tooth.iter().map(|q| place(at, b.land - q[1], q[0])));
                shape.push(place(at, root, last));
                painter.add(Shape::convex_polygon(shape, color, Stroke::NONE));
            }
        }
    }

    // ---- 3D view ---------------------------------------------------------

    fn fit_3d(&mut self, rect: Rect, solved: &[Solved]) {
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        for (g, s) in self.project.gears.iter().zip(solved).filter(|(g, _)| self.shown(g)) {
            let mut r = g.params.tip_radius().max(1.0);
            let (mut z0, mut z1) = match BevelGeom::new(&g.params) {
                Some(b) => (b.front_face().0, b.back(b.tip) * b.tip.cos()),
                None => (0.0, g.params.face_width),
            };
            // Generated teeth are only known once made.
            if let Some(c) = self.cache.get(&g.id).filter(|_| g.params.is_generated()) {
                r = c.solid.verts.iter().map(|v| v[0].hypot(v[1])).fold(1.0, f64::max);
                z0 = c.solid.verts.iter().map(|v| v[2]).fold(f64::MAX, f64::min);
                z1 = c.solid.verts.iter().map(|v| v[2]).fold(f64::MIN, f64::max);
            }
            for i in 0..8 {
                let local = [if i & 1 == 0 { -r } else { r }, if i & 2 == 0 { -r } else { r }, if i & 4 == 0 { z0 } else { z1 }];
                let p = s.frame.apply(local);
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
        }
        if lo[0] > hi[0] {
            self.camera = view3d::Camera::default();
            return;
        }
        self.camera.fit(rect, lo, hi);
    }

    fn canvas_3d(&mut self, ui: &mut Ui) {
        let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = resp.rect;
        let solved = self.project.solve();
        self.refresh_cache(ui.ctx());
        if rect.size() != self.canvas_size {
            self.canvas_size = rect.size();
            self.fit_pending |= self.auto_fit;
        }
        if self.fit_pending {
            self.fit_3d(rect, &solved);
            self.fit_pending = false;
            self.auto_fit = true;
        }
        if let Some(hover) = resp.hover_pos() {
            let (scroll, zoom) = ui.input(|i| (i.smooth_scroll_delta().y, i.zoom_delta()));
            let factor = zoom as f64 * (scroll as f64 * 0.0025).exp();
            if factor != 1.0 {
                self.auto_fit = false;
                self.camera.zoom(rect, hover, factor);
            }
        }
        if resp.dragged() {
            self.auto_fit = false;
            let d = resp.drag_delta();
            let (orbit, shift) = ui.input(|i| (i.pointer.primary_down(), i.modifiers.shift));
            if orbit && !shift {
                self.camera.orbit(d.x as f64, d.y as f64);
            } else {
                self.camera.pan(d.x as f64, d.y as f64);
            }
        }
        if resp.clicked() {
            if let Some(p) = resp.interact_pointer_pos() {
                self.selected = self.frame3d.pick(p);
            }
        }
        if resp.double_clicked() {
            self.fit_pending = true;
        }
        self.cursor = None;

        let visuals = ui.visuals().clone();
        let painter = painter.with_clip_rect(rect);
        painter.rect_filled(rect, 0.0, visuals.extreme_bg_color);
        let belts: Vec<(view3d::Solid, u64)> = (0..self.project.gears.len())
            .filter(|&i| self.shown(&self.project.gears[i]) && solved[i].belt.is_some())
            .filter_map(|i| {
                let b = self.belt_shape(i, &solved)?;
                let g = &self.project.gears[i];
                let pi = g.placement.parent().and_then(|p| self.project.index_of(p))?;
                let z_mid = solved[pi].z + self.project.gears[pi].params.face_width / 2.0;
                let solid = view3d::belt_solid(
                    &b.lp, &b.stations, b.land, b.back, &b.tooth, z_mid, g.params.belt_width,
                );
                Some((solid, g.id))
            })
            .collect();
        let belt_color = if visuals.dark_mode { Color32::from_gray(70) } else { Color32::from_gray(60) };
        let mut parts: Vec<view3d::Placed> = Vec::new();
        for (i, (g, s)) in self.project.gears.iter().zip(&solved).enumerate() {
            let Some(c) = self.cache.get(&g.id).filter(|_| self.shown(g)) else { continue };
            let base = gear_color_3d(i, s.depth);
            let color = if Some(g.id) == self.selected { base.lerp_to_gamma(chrome::ACCENT, 0.45) } else { base };
            let rot = s.phase + s.ratio * self.driver_angle;
            parts.push(view3d::Placed { solid: &c.solid, frame: s.frame, rot, color, id: g.id });
        }
        for (solid, id) in &belts {
            parts.push(view3d::Placed { solid, frame: train::Frame3::at([0.0; 3]), rot: 0.0, color: belt_color, id: *id });
        }
        let ppp = ui.ctx().pixels_per_point();
        self.frame3d = view3d::render(rect, ppp, &self.camera, &parts, visuals.extreme_bg_color);
        let image = self.frame3d.image.clone();
        match &mut self.texture3d {
            Some(t) => t.set(image, egui::TextureOptions::NEAREST),
            None => self.texture3d = Some(ui.ctx().load_texture("view3d", image, egui::TextureOptions::NEAREST)),
        }
        if let Some(t) = &self.texture3d {
            let uv = Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0));
            painter.image(t.id(), rect, uv, Color32::WHITE);
        }

        // The selected part's export origin, on its axis.
        let sel = self.selected.filter(|&id| parts.iter().any(|p| p.id == id)).and_then(|id| self.project.index_of(id));
        if let Some((i, f)) = sel.and_then(|i| Some((i, self.own_faces(i)?))) {
            let (g, s) = (&self.project.gears[i], &solved[i]);
            let h = g.export_origin.height(f, g.params.is_bevel() || g.params.is_hypoid());
            let at = |z: f64| self.camera.project(rect, s.frame.apply([0.0, 0.0, z])).0;
            let o = at(h);
            let axis = Stroke::new(1.0, chrome::ACCENT.gamma_multiply(0.7));
            painter.line_segment([at(f[0].min(h)), at(f[1].max(h))], axis);
            painter.circle(o, 4.0, chrome::ACCENT, Stroke::new(1.5, visuals.extreme_bg_color));
            painter.text(o + Vec2::new(8.0, -8.0), Align2::LEFT_BOTTOM, "Export origin", FontId::proportional(11.0), chrome::ACCENT);
        }

        // Where the ball end mill can't reach the selected part's teeth.
        if let Some((i, c)) = sel.and_then(|i| Some((i, self.reach_state(i)??))) {
            let s = &solved[i];
            let (sn, cs) = (s.phase + s.ratio * self.driver_angle).sin_cos();
            let slack = 0.3 * self.project.ball() + 0.05;
            for q in &c.dots {
                let (p, d) = self.camera.project(rect, s.frame.apply([q[0] * cs - q[1] * sn, q[0] * sn + q[1] * cs, q[2]]));
                if rect.contains(p) && self.frame3d.sees(p, d, slack) {
                    painter.circle_filled(p, 1.5, visuals.error_fg_color);
                }
            }
        }

        if self.pair_pending() {
            painter.text(
                rect.left_top() + Vec2::new(10.0, 8.0),
                Align2::LEFT_TOP,
                "⏳ Generating teeth…",
                FontId::proportional(13.0),
                visuals.weak_text_color(),
            );
        }
        if parts.is_empty() {
            painter.text(
                rect.center(),
                Align2::CENTER_CENTER,
                "Nothing to show in 3D yet.",
                FontId::proportional(16.0),
                visuals.weak_text_color(),
            );
        }
        painter.text(
            rect.left_bottom() + Vec2::new(8.0, -6.0),
            Align2::LEFT_BOTTOM,
            "Drag: orbit · Shift+drag or right-drag: pan · Scroll: zoom · Click: select · Double-click: fit",
            FontId::proportional(11.0),
            visuals.weak_text_color(),
        );
    }

    fn draw_gears(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        visuals: &egui::Visuals,
        solved: &[Solved],
        collisions: &[(usize, usize)],
    ) {
        let dark = visuals.dark_mode;
        let edge = visuals.text_color().gamma_multiply(0.8);
        let accent = visuals.selection.stroke.color;
        let error = visuals.error_fg_color;
        let mut selected_outline = None;

        for (i, (g, s)) in self.project.gears.iter().zip(solved).enumerate() {
            if !self.shown(g) {
                continue;
            }
            let center = self.to_screen(rect, s.center);
            let ra_px = (g.params.tip_radius() * self.view.scale) as f32;
            if !rect.expand(ra_px).contains(center) {
                continue;
            }
            let fill = gear_color(i, s.depth, dark);
            let rot = s.phase + s.ratio * self.driver_angle;
            let Some(c) = self.cache.get(&g.id) else {
                // Unbuildable parameters: show where the gear would be.
                painter.circle_stroke(center, ra_px.max(4.0), Stroke::new(1.5, error));
                continue;
            };
            if ra_px < 3.0 {
                painter.circle_filled(center, ra_px.max(1.5), fill);
                continue;
            }
            let mut mesh = egui::Mesh::default();
            mesh.reserve_vertices(c.fill.verts.len());
            mesh.reserve_triangles(c.fill.tris.len());
            for &q in &c.fill.verts {
                mesh.colored_vertex(self.to_screen(rect, place(q, s.center, rot)), fill);
            }
            for t in &c.fill.tris {
                mesh.add_triangle(t[0], t[1], t[2]);
            }
            painter.add(Shape::mesh(mesh));
            let pts: Vec<Pos2> = c.outline.iter().map(|&q| self.to_screen(rect, place(q, s.center, rot))).collect();
            if Some(g.id) == self.selected {
                selected_outline = Some(pts.clone());
            }
            painter.add(Shape::closed_line(pts, Stroke::new(1.0, edge)));
            if g.params.bore > 0.0 {
                painter.circle_stroke(center, (g.params.hole() / 2.0 * self.view.scale) as f32, Stroke::new(1.0, edge));
            }
            // Arbor mark, plus a tick so rotation is visible on plain gears.
            let tick = (ra_px * 0.15).clamp(3.0, 10.0);
            painter.circle_filled(center, 2.0, edge);
            let dir = Vec2::new(rot.cos() as f32, -rot.sin() as f32);
            let r0 = (g.params.hole() / 2.0 * self.view.scale) as f32 + 2.0;
            painter.line_segment([center + dir * r0, center + dir * (r0 + tick)], Stroke::new(1.5, edge));
        }

        self.draw_belts(painter, rect, visuals, solved);
        for &(i, j) in collisions {
            for k in [i, j] {
                let r = (self.project.gears[k].params.tip_radius() * self.view.scale) as f32;
                let pts = circle_points(self.to_screen(rect, solved[k].center), r);
                painter.extend(Shape::dashed_line(&pts, Stroke::new(1.5, error), 6.0, 4.0));
            }
        }

        // Selection overlay on top of everything.
        if let Some(idx) = self.selected.and_then(|id| self.project.index_of(id)) {
            let g = &self.project.gears[idx];
            let s = &solved[idx];
            let center = self.to_screen(rect, s.center);
            if let Some(pts) = selected_outline {
                painter.add(Shape::closed_line(pts, Stroke::new(2.0, accent)));
            }
            let px = |r: f64| (r * self.view.scale) as f32;
            if self.show.pitch {
                let pts = circle_points(center, px(g.params.pitch_radius()));
                painter.extend(Shape::dashed_line(&pts, Stroke::new(1.0, accent), 8.0, 5.0));
            }
            if self.show.base {
                let pts = circle_points(center, px(g.params.base_radius()));
                painter.extend(Shape::dashed_line(&pts, Stroke::new(1.0, accent.gamma_multiply(0.6)), 2.0, 4.0));
            }
            if let Some(pi) = g.placement.parent().and_then(|p| self.project.index_of(p)) {
                let pc = self.to_screen(rect, solved[pi].center);
                if pc.distance(center) > 1.0 {
                    painter.extend(Shape::dashed_line(&[pc, center], Stroke::new(1.0, accent), 4.0, 4.0));
                }
            }
        }

        if self.show.labels {
            for (g, s) in self.project.gears.iter().zip(solved).filter(|(g, _)| self.shown(g)) {
                let ra_px = (g.params.tip_radius() * self.view.scale) as f32;
                if ra_px < 40.0 {
                    continue;
                }
                let at = self.to_screen(rect, s.center) + Vec2::new(0.0, 14.0);
                let text = match g.params.belt {
                    Some(b) => format!("{}  {}", g.name, b.pulley_name(g.params.teeth)),
                    None => format!("{}  {}T", g.name, g.params.teeth),
                };
                let font = FontId::proportional(12.0);
                let galley = painter.layout_no_wrap(text, font, visuals.text_color());
                let r = Align2::CENTER_TOP.anchor_size(at, galley.size()).expand(3.0);
                painter.rect_filled(r, 3.0, visuals.extreme_bg_color.gamma_multiply(0.8));
                painter.galley(r.min + Vec2::splat(3.0), galley, visuals.text_color());
            }
        }
    }

    // ---- panels ----------------------------------------------------------

    fn doc_name(&self) -> String {
        self.path
            .as_ref()
            .and_then(|p| p.file_name())
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "Untitled".into())
    }

    /// The application bar: brand, menus, file actions and the document.
    fn menu_bar(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();
        chrome::app_bar_style(ui);
        egui::MenuBar::new().ui(ui, |ui| {
            chrome::brand(ui);
            chrome::bar_separator(ui);
            ui.menu_button(RichText::new("File").size(14.0), |ui| {
                let current = self.project.pitch_unit;
                let shortcut = |u: PitchUnit| if u == current { "Ctrl+N" } else { "" };
                if ui.add(egui::Button::new("New metric project").shortcut_text(shortcut(PitchUnit::Module))).clicked() {
                    self.request(&ctx, Pending::New(PitchUnit::Module));
                }
                if ui
                    .add(egui::Button::new("New inch project").shortcut_text(shortcut(PitchUnit::DiametralPitch)))
                    .clicked()
                {
                    self.request(&ctx, Pending::New(PitchUnit::DiametralPitch));
                }
                if ui.add(egui::Button::new("Open…").shortcut_text("Ctrl+O")).clicked() {
                    self.request(&ctx, Pending::Open);
                }
                if ui.add(egui::Button::new("Save").shortcut_text("Ctrl+S")).clicked() {
                    self.save(false, None);
                }
                if ui.add(egui::Button::new("Save As…").shortcut_text("Ctrl+Shift+S")).clicked() {
                    self.save(true, None);
                }
                ui.separator();
                ui.menu_button("Export", |ui| {
                    let has_sel = self.selected.is_some();
                    // Bevel gears and worms have no flat outline to draw.
                    let flat = self.workspace.flat();
                    let why = "Bevel gears and worms export as STEP or a mesh only";
                    let solid = [("stl", "STL (mesh)"), ("3mf", "3MF (mesh)"), ("obj", "OBJ (mesh)")];
                    for (ext, label) in [("step", "STEP (3D solid)")].into_iter().chain(solid).chain([("dxf", "DXF"), ("svg", "SVG")]) {
                        let ok = !matches!(ext, "dxf" | "svg") || flat;
                        let r = ui.add_enabled(has_sel && ok, egui::Button::new(format!("Selected part as {label}…")));
                        if r.on_disabled_hover_text(if ok { "Select a part first" } else { why }).clicked() {
                            self.export(false, ext);
                        }
                    }
                    ui.separator();
                    for (ext, label) in [("step", "STEP (3D solids)")].into_iter().chain(solid).chain([("dxf", "DXF"), ("svg", "SVG")]) {
                        let ok = !matches!(ext, "dxf" | "svg") || flat;
                        let r = ui.add_enabled(ok, egui::Button::new(format!("Whole train as {label}…")));
                        if r.on_disabled_hover_text(why).clicked() {
                            self.export(true, ext);
                        }
                    }
                });
                ui.separator();
                if ui.add(egui::Button::new("Quit").shortcut_text("Ctrl+Q")).clicked() {
                    self.request(&ctx, Pending::Quit);
                }
            });
            ui.menu_button(RichText::new("Edit").size(14.0), |ui| {
                if ui.add_enabled(!self.undo.is_empty(), egui::Button::new("Undo").shortcut_text("Ctrl+Z")).clicked() {
                    self.undo();
                }
                if ui
                    .add_enabled(!self.redo.is_empty(), egui::Button::new("Redo").shortcut_text("Ctrl+Shift+Z"))
                    .clicked()
                {
                    self.redo();
                }
                ui.separator();
                let has_sel = self.selected.is_some();
                match self.workspace {
                    Workspace::Spur => {
                        if ui.add_enabled(has_sel, egui::Button::new("Add meshing gear").shortcut_text("M")).clicked() {
                            self.add_gear(Link::Mesh, false);
                        }
                        if ui.add_enabled(has_sel, egui::Button::new("Add gear on same shaft").shortcut_text("A")).clicked() {
                            self.add_gear(Link::Axle, false);
                        }
                        if ui.button("New gear train").clicked() {
                            self.add_train(false);
                        }
                    }
                    Workspace::Bevel => {
                        if ui.add_enabled(has_sel, egui::Button::new("Add meshing bevel").shortcut_text("M")).clicked() {
                            self.add_bevel(Link::Bevel);
                        }
                        if ui.add_enabled(has_sel, egui::Button::new("Add bevel on same shaft").shortcut_text("A")).clicked() {
                            self.add_bevel(Link::Axle);
                        }
                        if ui.button("New bevel pair").clicked() {
                            self.add_bevel_pair();
                        }
                        if ui.button("New hypoid set").clicked() {
                            self.add_hypoid_pair();
                        }
                    }
                    Workspace::Worm => {
                        if ui.button("New worm drive").clicked() {
                            self.add_worm_drive();
                        }
                        if ui.button("New globoid drive").clicked() {
                            self.add_globoid_drive();
                        }
                    }
                    Workspace::BevelWorm => {
                        if ui.button("New bevel worm set").clicked() {
                            self.add_worm_pair();
                        }
                    }
                    Workspace::Pulleys => {
                        if ui.add_enabled(has_sel, egui::Button::new("Add belt drive").shortcut_text("B")).clicked() {
                            self.add_gear(Link::Belt, true);
                        }
                        if ui.add_enabled(has_sel, egui::Button::new("Add pulley on same shaft").shortcut_text("P")).clicked() {
                            self.add_gear(Link::Axle, true);
                        }
                        if ui.button("New pulley").clicked() {
                            self.add_train(true);
                        }
                    }
                }
                ui.separator();
                if ui.add_enabled(has_sel, egui::Button::new("Delete").shortcut_text("Del")).clicked() {
                    self.delete_selected();
                }
            });
            ui.menu_button(RichText::new("View").size(14.0), |ui| {
                if ui.add(egui::Button::new("Fit to window").shortcut_text("F")).clicked() {
                    self.fit_pending = true;
                }
                ui.separator();
                ui.checkbox(&mut self.show.grid, "Grid");
                ui.checkbox(&mut self.show.pitch, "Pitch circle (selected)");
                ui.checkbox(&mut self.show.base, "Base circle (selected)");
                ui.checkbox(&mut self.show.labels, "Labels");
            });
            chrome::bar_separator(ui);
            if chrome::bar_icon(ui, "💾", "Save (Ctrl+S)", true) {
                self.save(false, None);
            }
            if chrome::bar_icon(ui, "⟲", "Undo (Ctrl+Z)", !self.undo.is_empty()) {
                self.undo();
            }
            if chrome::bar_icon(ui, "⟳", "Redo (Ctrl+Shift+Z)", !self.redo.is_empty()) {
                self.redo();
            }
            chrome::bar_separator(ui);
            chrome::doc_tab(ui, &self.doc_name(), self.dirty);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(RichText::new(concat!("v", env!("CARGO_PKG_VERSION"))).size(13.0));
            });
        });
    }

    /// The workspace card, then the ribbon: the workspace's own tools
    /// followed by the groups every workspace shares.
    fn ribbon(&mut self, ui: &mut Ui) {
        ui.with_layout(egui::Layout::left_to_right(egui::Align::Min), |ui| {
            let card = chrome::workspace_card(ui, self.workspace.icon(), self.workspace.name())
                .on_hover_text("Choose what to design");
            egui::Popup::menu(&card).show(|ui| {
                ui.set_min_width(220.0);
                for w in Workspace::ALL {
                    let text = RichText::new(format!("{}   {}", w.glyph(), w.name())).size(14.0);
                    if ui.selectable_label(self.workspace == w, text).on_hover_text(w.hint()).clicked() {
                        self.switch_workspace(w);
                    }
                }
                ui.separator();
                ui.label(RichText::new("Coming later").small().weak());
                for name in Workspace::PLANNED {
                    ui.add_enabled(false, egui::Button::new(RichText::new(name).size(14.0)).frame(false));
                }
            });
            ui.add_space(4.0);
            chrome::ribbon_frame(ui).show(ui, |ui| {
                ui.horizontal(|ui| {
                    self.workspace_tools(ui);
                    self.shared_tools(ui);
                    ui.add_space(ui.available_width());
                });
            });
        });
    }

    fn workspace_tools(&mut self, ui: &mut Ui) {
        let big = chrome::big_button;
        match self.workspace {
            Workspace::Spur => chrome::group(ui, "Create", |ui| {
                if big(ui, Icon::Gear, "Mesh Gear", false, true).on_hover_text("A gear meshing with the selected one (M)").clicked() {
                    self.add_gear(Link::Mesh, false);
                }
                if big(ui, Icon::Glyph("◎"), "Axle Gear", false, true).on_hover_text("A gear on the selected part's axle (A)").clicked() {
                    self.add_gear(Link::Axle, false);
                }
                if big(ui, Icon::Glyph("➕"), "New Train", false, true).on_hover_text("A free-standing driver gear").clicked() {
                    self.add_train(false);
                }
            }),
            Workspace::Bevel => chrome::group(ui, "Create", |ui| {
                if big(ui, Icon::Bevel, "Mesh Bevel", false, true).on_hover_text("A bevel gear meshing with the selected one (M)").clicked() {
                    self.add_bevel(Link::Bevel);
                }
                if big(ui, Icon::Glyph("◎"), "Same Shaft", false, true).on_hover_text("A bevel gear on the selected gear's shaft (A)").clicked() {
                    self.add_bevel(Link::Axle);
                }
                if big(ui, Icon::Glyph("➕"), "New Pair", false, true).on_hover_text("A 20-tooth pinion and 40-tooth wheel at right angles").clicked() {
                    self.add_bevel_pair();
                }
                if big(ui, Icon::Bevel, "Hypoid Set", false, true)
                    .on_hover_text("A spiral-toothed pinion and ring gear with offset shafts, like a car's final drive")
                    .clicked()
                {
                    self.add_hypoid_pair();
                }
            }),
            Workspace::Worm => chrome::group(ui, "Create", |ui| {
                if big(ui, Icon::WormDrive, "Worm Drive", false, true)
                    .on_hover_text("A single-start worm turning a 30-tooth wheel at right angles: 30 : 1")
                    .clicked()
                {
                    self.add_worm_drive();
                }
                if big(ui, Icon::WormDrive, "Globoid Drive", false, true)
                    .on_hover_text(
                        "An hourglass worm wrapped round a 40-tooth wheel (double-enveloping): several teeth \
                         bear at once, for far more load in the same space",
                    )
                    .clicked()
                {
                    self.add_globoid_drive();
                }
            }),
            Workspace::BevelWorm => chrome::group(ui, "Create", |ui| {
                if big(ui, Icon::Worm, "Worm Set", false, true)
                    .on_hover_text("A 3-start conical worm driving a 30-tooth wheel, the shafts meeting at right angles")
                    .clicked()
                {
                    self.add_worm_pair();
                }
            }),
            Workspace::Pulleys => chrome::group(ui, "Create", |ui| {
                if big(ui, Icon::Pulley, "Belt Drive", false, true).on_hover_text("Add a pulley driven by a belt from the selected pulley (B)").clicked() {
                    self.add_gear(Link::Belt, true);
                }
                if big(ui, Icon::Glyph("◎"), "Same Shaft", false, true).on_hover_text("Add a pulley on the same shaft as the selected part (P)").clicked() {
                    self.add_gear(Link::Axle, true);
                }
                if big(ui, Icon::Glyph("➕"), "New Pulley", false, true).on_hover_text("Add a free-standing driver pulley").clicked() {
                    self.add_train(true);
                }
            }),
        }
    }

    fn shared_tools(&mut self, ui: &mut Ui) {
        let big = chrome::big_button;
        chrome::group(ui, "Units", |ui| {
            let current = self.project.pitch_unit;
            for (unit, icon, label, hint) in [
                (PitchUnit::Module, "mm", "Metric", "Module pitch, dimensions in millimetres"),
                (PitchUnit::DiametralPitch, "in", "Inch", "Diametral pitch, dimensions in inches"),
            ] {
                if big(ui, Icon::Text(icon), label, current == unit, true).on_hover_text(hint).clicked() && current != unit {
                    if self.project.gears.is_empty() {
                        self.project.pitch_unit = unit;
                    } else {
                        self.unit_switch = Some(unit);
                    }
                }
            }
        });
        chrome::group(ui, "Making", |ui| {
            let unit = self.project.pitch_unit;
            let machined = self.project.for_machining;
            let tip = if machined {
                "The teeth are shaped for the ball end mill on the 4-axis mill: roots rounded to it, and ring gear \
                 slot floors raised where it can't reach. Click to keep the teeth as designed instead."
            } else {
                "The teeth are as designed. Click to shape them for the ball end mill on the 4-axis mill."
            };
            if big(ui, Icon::Text("CNC"), "Machined", machined, true).on_hover_text(tip).clicked() {
                self.project.for_machining = !machined;
                if self.project.for_machining {
                    self.project.for_printing = false;
                }
            }
            ui.add_enabled_ui(machined, |ui| {
                ui.vertical(|ui| {
                    ui.add_space(8.0);
                    ui.label("Ball end mill Ø");
                    len_drag(ui, &mut self.project.ball_end_mill, unit, 0.0..=25.0, 0.01).on_hover_text(
                        "The ball end mill the teeth are cut with, on a 4-axis mill. Every root, on every kind of part, \
                         is rounded to it, and cut deep enough that the rounding stays under the other part's tips. \
                         0: not chosen yet.",
                    );
                });
            });
            let printed = self.project.for_printing;
            let tip = if printed {
                "The parts allow for the 3D printer's nozzle: every tooth surface and bore wall is drawn in by a \
                 quarter of the nozzle width, so the teeth come out thinner and the bores wider by half of it, and the \
                 printed surfaces land where they are designed. Click to keep the parts as designed instead."
            } else {
                "The parts are as designed. Click to allow for the 3D printer's nozzle: teeth thinner and bores \
                 wider by half its width."
            };
            if big(ui, Icon::Text("FDM"), "Printed", printed, true).on_hover_text(tip).clicked() {
                self.project.for_printing = !printed;
                if self.project.for_printing {
                    self.project.for_machining = false;
                }
            }
            ui.add_enabled_ui(printed, |ui| {
                ui.vertical(|ui| {
                    ui.add_space(8.0);
                    ui.label("Nozzle width");
                    len_drag(ui, &mut self.project.nozzle, unit, 0.0..=2.0, 0.01).on_hover_text(
                        "The printer's nozzle Ø. Every tooth surface and bore wall is drawn in by a quarter of it: \
                         the teeth thinner and the bores wider by half the width.",
                    );
                });
            });
        });
        chrome::group(ui, "Motion", |ui| {
            let (icon, label) = if self.playing { ("⏸", "Pause") } else { ("▶", "Run") };
            if big(ui, Icon::Glyph(icon), label, self.playing, true).on_hover_text("Space").clicked() {
                self.playing = !self.playing;
            }
            if big(ui, Icon::Glyph("⏮"), "Reset", false, true).on_hover_text("Back to the start position").clicked() {
                self.driver_angle = 0.0;
            }
            ui.vertical(|ui| {
                ui.add_space(8.0);
                ui.label("Driver");
                ui.add(egui::DragValue::new(&mut self.project.driver_rpm).speed(0.1).range(-600.0..=600.0).suffix(" rpm"));
            });
        });
        chrome::group(ui, "View", |ui| {
            let plan_ok = self.workspace.flat();
            for (three_d, label, hint) in [(false, "2D", "Plan view"), (true, "3D", "Solids of their face width; drag to orbit")] {
                let hint = if plan_ok || three_d { hint } else { "Bevel gears and worms are shown in 3D only" };
                if big(ui, Icon::Text(label), if three_d { "Solid" } else { "Plan" }, self.in_3d() == three_d, plan_ok || three_d)
                    .on_hover_text(hint)
                    .clicked()
                    && self.three_d != three_d
                {
                    self.three_d = three_d;
                    self.fit_pending = true;
                }
            }
            if big(ui, Icon::Glyph("⛶"), "Fit", false, true).on_hover_text("F, or double-click the canvas").clicked() {
                self.fit_pending = true;
            }
            if big(ui, Icon::Glyph("○"), "Pitch", self.show.pitch, !self.in_3d()).on_hover_text("Pitch circle of the selected part").clicked() {
                self.show.pitch = !self.show.pitch;
            }
            if big(ui, Icon::Glyph("◎"), "Base", self.show.base, !self.in_3d()).on_hover_text("Base circle of the selected gear").clicked() {
                self.show.base = !self.show.base;
            }
        });
    }

    fn tree(&mut self, ui: &mut Ui) {
        ui.heading("Train");
        ui.add_space(4.0);
        let solved = self.project.solve();
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            let ws = self.workspace;
            for (g, s) in self.project.gears.iter().zip(&solved).filter(|(g, _)| Workspace::of(&g.params) == ws) {
                let (icon, how) = match g.placement {
                    Placement::Root { .. } => ("⏺", "driver"),
                    Placement::Mesh { .. } => ("⚙", "meshes"),
                    Placement::Axle { .. } => ("◎", "same axle"),
                    Placement::Belt { .. } => ("🔗", "belt"),
                    Placement::Bevel { .. } | Placement::Hypoid { .. } | Placement::Worm { .. } => ("⚙", "meshes"),
                };
                ui.horizontal(|ui| {
                    ui.add_space(s.depth.min(12) as f32 * 14.0);
                    let text = RichText::new(format!("{icon}  {}", g.name));
                    let r = ui.selectable_label(self.selected == Some(g.id), text);
                    let size = match g.params.belt {
                        Some(b) => b.pulley_name(g.params.teeth),
                        None => format!("{}T", g.params.teeth),
                    };
                    ui.label(RichText::new(size).weak());
                    let r = r.on_hover_text(format!("{how} · ratio {}", fmt_ratio(s.ratio)));
                    if r.clicked() {
                        self.selected = Some(g.id);
                    }
                });
            }
        });
    }

    fn properties(&mut self, ui: &mut Ui) {
        let idx = self.selected.and_then(|id| self.project.index_of(id)).filter(|&i| self.shown(&self.project.gears[i]));
        let Some(idx) = idx else {
            ui.heading("Properties");
            let what = if self.workspace == Workspace::Pulleys { "pulley" } else { "gear" };
            ui.label(RichText::new(format!("Select a {what} on the canvas or in the train list.")).weak());
            return;
        };
        let solved = self.project.solve();
        let collisions = self.visible_collisions(&solved);
        let unit = self.project.pitch_unit;
        let rpm = self.project.driver_rpm;
        // Partners come from this workspace (plus a current partner from
        // another, so an existing link still shows).
        let own_parent = self.project.gears[idx].placement.parent();
        let earlier: Vec<(u64, String)> = self.project.gears[..idx]
            .iter()
            .filter(|g| self.shown(g) || Some(g.id) == own_parent)
            .map(|g| (g.id, g.name.clone()))
            .collect();
        let parent_idx = self.project.gears[idx].placement.parent().and_then(|p| self.project.index_of(p));
        let parent_name = parent_idx.map(|p| self.project.gears[p].name.clone());
        let others: Vec<String> = collisions
            .iter()
            .filter_map(|&(i, j)| match (i == idx, j == idx) {
                (true, _) => Some(self.project.gears[j].name.clone()),
                (_, true) => Some(self.project.gears[i].name.clone()),
                _ => None,
            })
            .collect();
        let s = solved[idx].clone();
        let worm = self.project.gears[idx].params.worm;
        let earlier_pinions: Vec<(u64, String)> = self.project.gears[..idx]
            .iter()
            .filter(|g| g.params.hypoid == Some(Role::Pinion) && g.params.worm == worm)
            .map(|g| (g.id, g.name.clone()))
            .collect();
        let earlier_worms: Vec<(u64, String)> = self.project.gears[..idx]
            .iter()
            .filter(|g| g.params.worm_drive == Some(worm::Role::Worm))
            .map(|g| (g.id, g.name.clone()))
            .collect();
        let pair_driven = self.pair_driven(idx);
        let hypoid_design = pair_driven.and_then(|r| solved[r].hypoid.clone());
        let worm_design = pair_driven.and_then(|r| solved[r].worm.clone());
        let hypoid_cutter = hypoid_design.as_ref().map(|d| d.cutter.radius);
        // Whether the pair's teeth are being generated, and why they failed.
        let pair_state = pair_driven.map(|r| {
            let job = self.pairs.get(&self.project.gears[r].id);
            (job.is_some_and(|j| j.rx.is_some()), job.and_then(|j| j.error.clone()))
        });
        let earlier_pulleys: Vec<(u64, String)> = self.project.gears[..idx]
            .iter()
            .filter(|g| g.params.is_pulley())
            .map(|g| (g.id, g.name.clone()))
            .collect();
        // Belt fits need the whole project, so they run after the panel.
        let mut fit_req: Option<f64> = None;
        let mut fit_start = false;
        let pair = self.project.ratio_pair(idx).map(|(a, b)| {
            let (ga, gb) = (&self.project.gears[a], &self.project.gears[b]);
            RatioPair {
                driven: gb.id,
                partner: self.project.gears[if b == idx { a } else { b }].name.clone(),
                drives: a == idx,
                teeth: (ga.params.teeth, gb.params.teeth),
                worm: ga.params.is_worm_thread(),
                belt_center: solved[b].belt.as_ref().map(|bi| bi.center),
            }
        });
        let mut ratio_target = pair.as_ref().map(|pr| {
            let now = pr.teeth.1 as f64 / pr.teeth.0 as f64;
            self.ratio_target.get(&pr.driven).copied().unwrap_or(now)
        });
        let ratio_note = pair.as_ref().and_then(|pr| {
            self.ratio_result.as_ref().filter(|r| (r.0, r.1, r.2) == (pr.driven, pr.teeth.0, pr.teeth.1)).map(|r| r.3.clone())
        });
        let mut ratio_req: Option<f64> = None;
        let mut globoid_req = false;
        // Problems with a button that clears them, and the one pressed.
        let mut fixable: Vec<(Issue, String, Fix)> = Vec::new();
        let mut fix_req: Option<Fix> = None;
        // A globoid worm too long for its base circle, or a drive the ball
        // end mill can't cut, has no teeth, so they are checked here, from
        // the drive's settings.
        let ball = self.project.ball();
        let machined = self.project.for_machining;
        let drive_spec = pair_driven.and_then(|r| {
            let wheel = &self.project.gears[r];
            let k = self.project.index_of(wheel.placement.parent()?)?;
            train::worm_spec(&self.project.gears[k].params, &wheel.params, &wheel.placement, ball)
        });
        let globoid_spec = drive_spec.as_ref().filter(|s| s.globoid);
        let too_long = globoid_spec.and_then(|sp| sp.globoid_lengths().filter(|l| sp.length > l.0).map(|l| (sp.length, l)));
        // What the ball end mill can cut: the Ø the roots are rounded for
        // as things stand, the biggest that fits, what limits it, and what
        // sets the roots with no ball chosen.
        let hypoid_spec = pair_driven.filter(|_| hypoid_design.is_some()).and_then(|r| {
            let ring = &self.project.gears[r];
            let k = self.project.index_of(ring.placement.parent()?)?;
            train::hypoid_spec(&self.project.gears[k].params, &ring.params, &ring.placement, 0.0)
        });
        let hypoid_largest = hypoid_spec.map(|sp| match &self.hypoid_ball {
            Some((key, d)) if *key == sp => *d,
            _ => {
                let d = sp.largest_ball();
                self.hypoid_ball = Some((sp, d));
                d
            }
        });
        let p = &self.project.gears[idx].params;
        let room: Option<(f64, Option<f64>, String, &str)> = if let Some(sp) = &drive_spec {
            worm_design.as_ref().map(|d| {
                let why = "where the other part's tips reach, the spaces are narrower than it, so it can't cut below them";
                (2.0 * d.ball, sp.largest_ball(), why.to_string(), "the biggest that fits up to the wheel's root fillet")
            })
        } else if let (Some(d), Some(largest)) = (&hypoid_design, hypoid_largest) {
            let why = d.ball_trouble().unwrap_or("the spaces are narrower than it at the bottom");
            Some((2.0 * d.cutter.edge, largest, why.to_string(), "the ring cutter's usual edge radius"))
        } else {
            p.ball_room().map(|(now, most)| {
                let (why, unset) = if p.is_pulley() {
                    let unset = match p.belt.map(|b| b.family()) {
                        Some(pulley::Family::Iso) => "from ISO 5294's groove corners",
                        _ => "from the belt tooth's tip",
                    };
                    ("the grooves are narrower than it at the bottom", unset)
                } else if p.is_bevel() {
                    ("at the toe the spaces are narrower than it at the bottom", "at the toe, from the root fillet under Rack")
                } else {
                    ("the spaces are narrower than it at the bottom", "from the root fillet under Rack")
                };
                (now, Some(most), why.to_string(), unset)
            })
        };
        // The biggest ball that fits, rounded down to what is shown.
        let largest_ball = room.as_ref().and_then(|r| r.1).map(|d| shown_floor(d, unit));
        let ball_too_big = largest_ball.filter(|&d| ball > 0.0 && ball > d + 1e-9 && too_long.is_none());
        let bigger = if self.project.gears[idx].params.is_pulley() { "a bigger belt section" } else { "a bigger module" };
        // The worm drive's friction lives on its wheel's placement.
        let mut friction = pair_driven.and_then(|r| match self.project.gears[r].placement {
            Placement::Worm { friction, .. } => Some(friction),
            _ => None,
        });
        let back_note = pair_driven
            .and_then(|r| self.back_result.as_ref().filter(|b| b.0 == self.project.gears[r].id))
            .filter(|b| worm_design.as_ref().is_some_and(|d| d.spec == b.1))
            .map(|b| b.2.clone());
        let mut back_req = false;
        let reach = self.reach_state(idx);
        // How far the ring's slot floors were raised for the ball's reach.
        let raised = pair_driven
            .and_then(|r| self.pairs.get(&self.project.gears[r].id))
            .filter(|j| j.rx.is_none())
            .and_then(|j| j.members.as_ref())
            .map(|m| m.1.raised)
            .filter(|&h| h > 0.0);

        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            let faces = self.own_faces(idx);
            let g = &mut self.project.gears[idx];
            ui.horizontal(|ui| {
                ui.label("Name");
                ui.text_edit_singleline(&mut g.name);
            });
            ui.add_space(6.0);

            if let (Some(pr), Some(target)) = (&pair, &mut ratio_target) {
                section(ui, "Ratio", |ui| {
                    egui::Grid::new("ratio").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                        ui.label(if pr.drives { "Drives" } else { "Driven by" });
                        ui.label(&pr.partner);
                        ui.end_row();
                        let (za, zb) = pr.teeth;
                        let teeth = match (pr.worm, za) {
                            (true, 1) => format!("{zb} teeth : 1 start"),
                            (true, _) => format!("{zb} teeth : {za} starts"),
                            _ => format!("{zb} : {za} teeth"),
                        };
                        ui.label("Pair ratio");
                        ui.label(RichText::new(format!("{:.4} : 1  ·  {teeth}", zb as f64 / za as f64)).monospace())
                            .on_hover_text("Turns of the driving part for one turn of the driven one: driven teeth over driving teeth.");
                        ui.end_row();
                        ui.label("Target");
                        ui.add(egui::DragValue::new(target).range(0.01..=1000.0).speed(0.01).max_decimals(4).suffix(" : 1"))
                            .on_hover_text("The pair ratio you want, driven : driving.");
                        ui.end_row();
                        ui.label("Max teeth");
                        ui.add(egui::DragValue::new(&mut self.ratio_max_teeth).range(10..=400))
                            .on_hover_text("The most teeth either part may get. A worm keeps to its own range of starts.");
                        ui.end_row();
                        ui.label("");
                        if ui
                            .button("Size to ratio")
                            .on_hover_text(
                                "Pick tooth counts for both parts that come nearest the target, with no undercut, \
                                 keeping the pair close to its present size. A part that also turns another \
                                 keeps its teeth.",
                            )
                            .clicked()
                        {
                            ratio_req = Some(*target);
                        }
                        ui.end_row();
                    });
                    if let Some(note) = &ratio_note {
                        ui.label(RichText::new(note).weak());
                    }
                });
            }

            if let (Some(d), Some(mu)) = (&worm_design, &mut friction) {
                section(ui, "Back-driving", |ui| {
                    egui::Grid::new("backdrive").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                        ui.label("Friction μ");
                        ui.horizontal(|ui| {
                            ui.add(egui::DragValue::new(mu).range(0.01..=0.6).speed(0.002).max_decimals(3)).on_hover_text(
                                "Running friction between the threads and the teeth. It sets the efficiency and \
                                 whether the wheel can drive the worm.",
                            );
                            ui.menu_button("Materials ⏷", |ui| {
                                for (name, m) in worm::MATERIALS {
                                    if ui.button(format!("{name}  ·  {m}")).clicked() {
                                        *mu = m;
                                        ui.close();
                                    }
                                }
                            });
                        });
                        ui.end_row();
                        let deg = |a: f64| format!("{:.2}°", a.to_degrees());
                        let rho = worm::friction_angle(*mu, d.spec.pressure_angle);
                        value_row(ui, "Lead angle", deg(d.lead_angle));
                        ui.label("Friction angle");
                        ui.label(RichText::new(deg(rho)).monospace()).on_hover_text(
                            "The lead angle must be above this for the wheel to turn the worm at all, and \
                             well above it to start the worm from rest.",
                        );
                        ui.end_row();
                        value_row(ui, "Efficiency, worm driving", format!("{:.0} %", 100.0 * d.efficiency(*mu)));
                        value_row(ui, "Efficiency, wheel driving", format!("{:.0} %", 100.0 * d.back_efficiency(*mu)));
                        let (verdict, hint) = match d.back_drive(*mu) {
                            worm::BackDrive::Locked => (
                                "No: self-locking",
                                "Friction holds the worm, even running. Don't rely on it as a brake: shock and \
                                 vibration can still slip it.",
                            ),
                            worm::BackDrive::Running => (
                                "Only once turning",
                                "The wheel keeps a turning worm going, but won't start it from rest, and \
                                 vibration may let it creep.",
                            ),
                            worm::BackDrive::Free => ("Yes", "The wheel can turn the worm from rest."),
                        };
                        ui.label("Wheel drives worm");
                        ui.label(RichText::new(verdict).monospace()).on_hover_text(hint);
                        ui.end_row();
                        ui.label("");
                        ui.add_enabled_ui(d.back_drive(*mu) != worm::BackDrive::Free, |ui| {
                            ui.horizontal(|ui| {
                                if ui
                                    .button("Make back-drivable")
                                    .on_hover_text(
                                        "Steepen the lead angle until the wheel can start the worm: add starts, \
                                         and thin the worm if it needs it, to no less than 8 modules.",
                                    )
                                    .clicked()
                                {
                                    back_req = true;
                                }
                                ui.checkbox(&mut self.back_keep_ratio, "Keep ratio").on_hover_text(
                                    "Give the wheel as many times the teeth as the worm gains starts, so the ratio \
                                     stands but the wheel grows. Off: the wheel keeps its teeth and size, and the \
                                     ratio drops.",
                                );
                            });
                        });
                        ui.end_row();
                    });
                    if let Some(note) = &back_note {
                        ui.label(RichText::new(note).weak());
                    }
                });
            }

            if let Some(mut belt) = g.params.belt {
                let locked = g.match_parent && matches!(g.placement, Placement::Belt { .. });
                section(ui, "Teeth", |ui| {
                    egui::Grid::new("pulley").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                        ui.label("Belt section");
                        ui.add_enabled_ui(!locked, |ui| {
                            egui::ComboBox::from_id_salt("section").selected_text(belt.label()).height(520.0).show_ui(ui, |ui| {
                                for f in pulley::Family::ALL {
                                    if f != pulley::Family::Iso {
                                        ui.separator();
                                    }
                                    ui.label(RichText::new(f.name()).weak().small());
                                    for b in BeltSection::ALL.into_iter().filter(|b| b.family() == f) {
                                        ui.selectable_value(&mut belt, b, b.label());
                                    }
                                }
                            });
                        });
                        ui.end_row();
                        if Some(belt) != g.params.belt {
                            let w = belt.width_near(g.params.belt_width);
                            g.params.belt = Some(belt);
                            g.params.belt_width = w.width;
                            g.params.face_width = w.unflanged;
                            g.params.teeth = g.params.teeth.max(belt.min_teeth());
                        }
                        if matches!(g.placement, Placement::Belt { .. }) {
                            ui.label("");
                            ui.checkbox(&mut g.match_parent, "Same belt as partner")
                                .on_hover_text("Copy the belt section and width from the pulley at the other end of the belt.");
                            ui.end_row();
                        }

                        ui.label("Teeth");
                        ui.horizontal(|ui| {
                            ui.add(egui::DragValue::new(&mut g.params.teeth).range(belt.min_teeth()..=400).speed(0.2));
                            egui::ComboBox::from_id_salt("stock").selected_text("Stock sizes").show_ui(ui, |ui| {
                                for z in belt.stock_teeth() {
                                    ui.selectable_value(&mut g.params.teeth, z, format!("{z} teeth"));
                                }
                            });
                        });
                        ui.end_row();

                        ui.label("Belt width");
                        ui.add_enabled_ui(!locked, |ui| {
                            let label = |w: &crate::pulley::BeltWidth| match w.code {
                                "" => len(w.width, unit),
                                code => format!("{code} ({})", len(w.width, unit)),
                            };
                            let current = belt.width_near(g.params.belt_width);
                            egui::ComboBox::from_id_salt("width").selected_text(label(&current)).show_ui(ui, |ui| {
                                for w in belt.widths() {
                                    if ui.selectable_label(w.width == g.params.belt_width, label(w)).clicked() {
                                        g.params.belt_width = w.width;
                                        g.params.face_width = w.unflanged;
                                    }
                                }
                            });
                        });
                        ui.end_row();
                    });
                    ui.label(
                        RichText::new(format!(
                            "{} Picking a width sets the face width for an unflanged pulley.",
                            belt.family().groove_note()
                        ))
                        .weak()
                        .small(),
                    );
                });
            } else if let Some(role) = g.params.worm_drive {
                section(ui, "Teeth", |ui| {
                    egui::Grid::new("teeth").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                        if role == worm::Role::Worm {
                            ui.label("Starts");
                            ui.horizontal(|ui| {
                                for z in worm::STARTS {
                                    ui.selectable_value(&mut g.params.teeth, z, z.to_string());
                                }
                            })
                            .response
                            .on_hover_text("The worm's threads. Each start turns the wheel a tooth further per turn of the worm.");
                            ui.end_row();
                            return;
                        }
                        ui.label("Teeth");
                        ui.add(egui::DragValue::new(&mut g.params.teeth).range(10..=400).speed(0.2));
                        ui.end_row();
                        ui.label(if unit.is_inch() { "Diametral pitch (axial)" } else { "Module (axial)" });
                        ui.horizontal(|ui| pitch_input(ui, &mut g.params.module, unit))
                            .response
                            .on_hover_text("The worm's axial pitch over π, which is also the wheel's module across its face.");
                        ui.end_row();
                        ui.label("Pressure angle");
                        ui.horizontal(|ui| {
                            for pa in [14.5, 20.0, 25.0] {
                                ui.selectable_value(&mut g.params.pressure_angle, pa, format!("{pa}°"));
                            }
                            ui.add(egui::DragValue::new(&mut g.params.pressure_angle).range(10.0..=30.0).speed(0.1).suffix("°"));
                        })
                        .response
                        .on_hover_text("Normal to the threads. 20° is usual.");
                        ui.end_row();
                        ui.label("Backlash");
                        len_drag(ui, &mut g.params.backlash, unit, 0.0..=5.0, 0.002)
                            .on_hover_text("Thinning of the wheel's teeth on its pitch circle.");
                        ui.end_row();
                    });
                    if role == worm::Role::Worm {
                        ui.label(RichText::new("Module, pressure angle and backlash are set on the wheel.").weak().small());
                    }
                });
            } else if let Some(role) = g.params.hypoid {
                section(ui, "Teeth", |ui| {
                    egui::Grid::new("teeth").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                        if worm && role == Role::Pinion {
                            ui.label("Starts");
                            ui.horizontal(|ui| {
                                for z in 3..=7 {
                                    ui.selectable_value(&mut g.params.teeth, z, z.to_string());
                                }
                            })
                            .response
                            .on_hover_text("The worm's threads: its teeth. More starts turn the wheel faster.");
                        } else {
                            ui.label("Teeth");
                            let range = if role == Role::Pinion { 5..=60 } else { 10..=200 };
                            ui.add(egui::DragValue::new(&mut g.params.teeth).range(range).speed(0.2));
                        }
                        ui.end_row();
                        if role == Role::Ring {
                            ui.label(if unit.is_inch() { "Diametral pitch (heel)" } else { "Module (heel)" });
                            ui.horizontal(|ui| pitch_input(ui, &mut g.params.module, unit));
                            ui.end_row();
                            ui.label("Pressure angle");
                            ui.horizontal(|ui| {
                                for pa in [20.0, 22.5] {
                                    ui.selectable_value(&mut g.params.pressure_angle, pa, format!("{pa}°"));
                                }
                                ui.add(egui::DragValue::new(&mut g.params.pressure_angle).range(12.0..=30.0).speed(0.1).suffix("°"));
                            });
                            ui.end_row();
                            ui.label("Backlash");
                            len_drag(ui, &mut g.params.backlash, unit, 0.0..=5.0, 0.002)
                                .on_hover_text("Thinning of the ring gear's teeth, normal to them on the pitch cone.");
                            ui.end_row();
                        }
                    });
                    if role == Role::Pinion {
                        let on = if worm { "wheel" } else { "ring gear" };
                        ui.label(RichText::new(format!("Tooth size, pressure angle and face width are set on the {on}.")).weak().small());
                    }
                });
            } else {
                let locked = g.match_parent && matches!(g.placement, Placement::Mesh { .. } | Placement::Bevel { .. });
                let is_bevel = g.params.is_bevel();
                section(ui, "Teeth", |ui| {
                    egui::Grid::new("teeth").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                        ui.label("Teeth");
                        ui.add(egui::DragValue::new(&mut g.params.teeth).range(3..=2000).speed(0.2));
                        ui.end_row();

                        ui.label(if unit.is_inch() { "Diametral pitch" } else { "Module" });
                        ui.add_enabled_ui(!locked, |ui| ui.horizontal(|ui| pitch_input(ui, &mut g.params.module, unit)));
                        ui.end_row();

                        ui.label("Pressure angle");
                        ui.add_enabled_ui(!locked, |ui| {
                            ui.horizontal(|ui| {
                                for pa in [14.5, 20.0, 25.0] {
                                    ui.selectable_value(&mut g.params.pressure_angle, pa, format!("{pa}°"));
                                }
                                ui.add(
                                    egui::DragValue::new(&mut g.params.pressure_angle)
                                        .range(5.0..=40.0)
                                        .speed(0.1)
                                        .suffix("°"),
                                );
                            });
                        });
                        ui.end_row();

                        ui.label("Tooth standard");
                        ui.add_enabled_ui(!locked, |ui| {
                            egui::ComboBox::from_id_salt("standard")
                                .selected_text(g.params.standard.label())
                                .show_ui(ui, |ui| {
                                    for st in ToothStandard::ALL {
                                        ui.selectable_value(&mut g.params.standard, st, st.label()).on_hover_text(st.hint());
                                    }
                                })
                                .response
                                .on_hover_text(g.params.standard.hint());
                        });
                        ui.end_row();

                        if matches!(g.placement, Placement::Mesh { .. } | Placement::Bevel { .. }) {
                            ui.label("");
                            ui.checkbox(&mut g.match_parent, "Same tooth system as partner").on_hover_text(
                                "Copy pitch, pressure angle and rack proportions from the meshing partner. \
                                 Pitch and pressure angle must match for gears to mesh.",
                            );
                            ui.end_row();
                        }

                        if is_bevel {
                            g.params.profile_shift = 0.0;
                        } else {
                        ui.label("Profile shift x");
                        ui.add(egui::DragValue::new(&mut g.params.profile_shift).range(-1.0..=1.5).speed(0.005).max_decimals(3))
                            .on_hover_text("Moves the cutter out (+) or in (−) by x × module. Positive shift cures undercut on small pinions.");
                        ui.end_row();
                        }

                        ui.label("Backlash");
                        len_drag(ui, &mut g.params.backlash, unit, 0.0..=5.0, 0.002)
                            .on_hover_text("Tooth thinning on the pitch circle. A pair's backlash is the sum of both.");
                        ui.end_row();
                    });
                    egui::CollapsingHeader::new("Rack (advanced)").default_open(false).show(ui, |ui| {
                        let per = if unit.is_inch() { "/P" } else { "× m" };
                        let mut edited = false;
                        ui.add_enabled_ui(!locked, |ui| {
                            egui::Grid::new("rack").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                                let mut coef = |ui: &mut Ui, label: &str, v: &mut f64, range: RangeInclusive<f64>| {
                                    ui.label(label);
                                    let r = egui::DragValue::new(v).range(range).speed(0.002).max_decimals(4).suffix(per);
                                    edited |= ui.add(r).changed();
                                    ui.end_row();
                                };
                                coef(ui, "Addendum", &mut g.params.addendum, 0.2..=2.0);
                                coef(ui, "Dedendum", &mut g.params.dedendum, 0.3..=2.5);
                                coef(ui, "Root fillet radius", &mut g.params.root_fillet, 0.0..=0.6);
                            });
                        });
                        if edited {
                            g.params.standard = ToothStandard::Custom;
                        } else if g.params.standard != ToothStandard::Custom {
                            ui.label(RichText::new("Set by the tooth standard; editing switches to Custom.").weak().small());
                        }
                        if g.params.ball > 0.0 {
                            let deeper = g.params.cut_dedendum() > g.params.dedendum + 1e-9 && !g.params.is_bevel();
                            ui.label(
                                RichText::new(format!(
                                    "The roots are rounded for the Ø{} ball end mill under Machining, not the fillet here{}.",
                                    len(g.params.ball, unit),
                                    if deeper { ", and cut deeper to suit" } else { "" }
                                ))
                                .weak()
                                .small(),
                            );
                        }
                    });
                });
            }

            section(ui, "Blank", |ui| {
                egui::Grid::new("blank").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                    ui.label("Bore Ø");
                    let r = len_drag(ui, &mut g.params.bore, unit, 0.0..=10000.0, 0.1);
                    if g.params.nozzle > 0.0 && g.params.bore > 0.0 {
                        r.on_hover_text(format!(
                            "Made Ø{}: widened by half the nozzle width under Making, so the printed hole comes out as designed.",
                            len(g.params.hole(), unit)
                        ));
                    }
                    ui.end_row();
                    if g.params.hypoid != Some(Role::Pinion) {
                        let thread = g.params.worm_drive == Some(worm::Role::Worm);
                        ui.label(if thread { "Thread length" } else { "Face width" });
                        let r = len_drag(ui, &mut g.params.face_width, unit, 0.0..=10000.0, 0.1);
                        if g.params.is_bevel() || g.params.is_hypoid() {
                            r.on_hover_text("Along the cone, from the back (large) end towards the apex.");
                        } else if thread {
                            r.on_hover_text("The threaded length of the worm; its middle meets the wheel.");
                        } else if g.params.worm_drive.is_some() {
                            r.on_hover_text("A throated wheel's face is usually about ¾ of the worm's outside diameter.");
                        }
                        ui.end_row();
                    }
                });
            });

            section(ui, "Placement", |ui| {
                let kind = match g.placement {
                    Placement::Root { .. } => 0,
                    Placement::Mesh { .. } | Placement::Bevel { .. } | Placement::Hypoid { .. } | Placement::Worm { .. } => 1,
                    Placement::Axle { .. } => 2,
                    Placement::Belt { .. } => 3,
                };
                let is_pulley = g.params.is_pulley();
                let is_bevel = g.params.is_bevel();
                // A ring gear stays with its pinion; a pinion is fixed or on a shaft.
                let is_ring = g.params.hypoid == Some(Role::Ring) || g.params.worm_drive == Some(worm::Role::Wheel);
                let is_pinion = g.params.hypoid == Some(Role::Pinion) || g.params.worm_drive == Some(worm::Role::Worm);
                let mut new_kind = kind;
                if !is_ring { ui.horizontal_wrapped(|ui| {
                    ui.selectable_value(&mut new_kind, 0, "Fixed");
                    ui.add_enabled_ui(!earlier.is_empty(), |ui| {
                        if (!is_pulley || kind == 1) && !is_pinion {
                            ui.selectable_value(&mut new_kind, 1, "Meshes with");
                        }
                        ui.selectable_value(&mut new_kind, 2, "On axle of");
                    });
                    if is_pulley || kind == 3 {
                        ui.add_enabled_ui(!earlier_pulleys.is_empty(), |ui| {
                            ui.selectable_value(&mut new_kind, 3, "Belt to");
                        });
                    }
                }); }
                if new_kind != kind {
                    let parent = g.placement.parent().or(earlier.last().map(|e| e.0)).unwrap_or(0);
                    let angle = match g.placement {
                        Placement::Mesh { angle, .. }
                        | Placement::Bevel { angle, .. }
                        | Placement::Hypoid { angle, .. }
                        | Placement::Worm { angle, .. } => angle,
                        _ => 0.0,
                    };
                    g.placement = match new_kind {
                        0 => Placement::Root { x: s.center[0], y: s.center[1], phase: 0.0 },
                        1 if is_bevel => Placement::Bevel { parent, angle, shaft_angle: 90.0 },
                        1 => Placement::Mesh { parent, angle },
                        2 => Placement::Axle { parent, phase: 0.0 },
                        _ => {
                            let parent = Some(parent)
                                .filter(|p| earlier_pulleys.iter().any(|e| e.0 == *p))
                                .or(earlier_pulleys.last().map(|e| e.0))
                                .unwrap_or(0);
                            fit_start = true;
                            Placement::Belt { parent, angle, belt_teeth: 0, target: 0.0 }
                        }
                    };
                }
                let belt_section = g.params.belt;
                egui::Grid::new("place").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| match &mut g.placement {
                    Placement::Root { x, y, phase } => {
                        ui.label("Centre X");
                        len_drag(ui, x, unit, f64::NEG_INFINITY..=f64::INFINITY, 0.5);
                        ui.end_row();
                        ui.label("Centre Y");
                        len_drag(ui, y, unit, f64::NEG_INFINITY..=f64::INFINITY, 0.5);
                        ui.end_row();
                        ui.label("Phase");
                        ui.add(egui::DragValue::new(phase).speed(0.5).range(-360.0..=360.0).suffix("°"));
                        ui.end_row();
                    }
                    Placement::Mesh { parent, angle } => {
                        ui.label("Partner");
                        parent_combo(ui, parent, &earlier);
                        ui.end_row();
                        ui.label("Direction");
                        ui.add(egui::DragValue::new(angle).speed(0.5).range(-360.0..=360.0).suffix("°"))
                            .on_hover_text("Angle of the line from the partner's centre to this gear's. You can also drag the gear on the canvas.");
                        ui.end_row();
                    }
                    Placement::Belt { parent, angle, belt_teeth, target } => {
                        ui.label("Partner");
                        parent_combo(ui, parent, &earlier_pulleys);
                        ui.end_row();
                        ui.label("Direction");
                        ui.add(egui::DragValue::new(angle).speed(0.5).range(-360.0..=360.0).suffix("°"))
                            .on_hover_text("Angle of the line from the partner's centre to this pulley's. You can also drag the pulley on the canvas.");
                        ui.end_row();

                        ui.label("Centre distance");
                        let actual = s.belt.as_ref().map(|b| b.center).unwrap_or(0.0);
                        let mut want = if *target > 0.0 { *target } else { actual };
                        let r = len_drag(ui, &mut want, unit, 1.0..=100_000.0, 0.5).on_hover_text(
                            "The spacing you want. The belt that comes closest is picked and the pulley moves to \
                             where that belt puts it.",
                        );
                        if r.changed() {
                            fit_req = Some(want);
                        }
                        ui.end_row();
                        ui.label("");
                        let r = ui.checkbox(&mut self.belt_standard_only, "Standard belts only").on_hover_text(
                            "Pick from the ISO 5296 stock lengths. Off: any whole number of teeth, for \
                             made-to-order belts. Metric belts have no standard lengths, so any whole number of \
                             teeth is used either way: check your maker's list.",
                        );
                        if r.changed() {
                            fit_req = Some(want);
                        }
                        ui.end_row();

                        ui.label("Belt");
                        let name = belt_section.and_then(|b| b.designation(*belt_teeth));
                        egui::ComboBox::from_id_salt("belt")
                            .selected_text(name.unwrap_or_else(|| "Non-standard".into()))
                            .show_ui(ui, |ui| {
                                for sb in belt_section.map(|b| b.standard_belts()).unwrap_or_default() {
                                    let text = format!("{}  ·  {} teeth", sb.designation, sb.teeth);
                                    if ui.selectable_label(*belt_teeth == sb.teeth, text).clicked() {
                                        *belt_teeth = sb.teeth;
                                        *target = 0.0;
                                    }
                                }
                            });
                        ui.end_row();
                        ui.label("Belt teeth");
                        if ui.add(egui::DragValue::new(belt_teeth).range(10..=5000).speed(0.2)).changed() {
                            *target = 0.0;
                        }
                        ui.end_row();
                    }
                    Placement::Bevel { parent, angle, shaft_angle } => {
                        ui.label("Partner");
                        parent_combo(ui, parent, &earlier);
                        ui.end_row();
                        ui.label("Shaft angle");
                        ui.horizontal(|ui| {
                            for a in [45.0, 60.0, 90.0, 120.0] {
                                ui.selectable_value(shaft_angle, a, format!("{a}°"));
                            }
                            ui.add(egui::DragValue::new(shaft_angle).speed(0.5).range(10.0..=170.0).suffix("°"));
                        })
                        .response
                        .on_hover_text("Angle between the two shafts. 90° is the usual right-angle drive.");
                        ui.end_row();
                        ui.label("Direction");
                        ui.add(egui::DragValue::new(angle).speed(0.5).range(-360.0..=360.0).suffix("°"))
                            .on_hover_text("Which way round the partner's axis this gear's shaft leans.");
                        ui.end_row();
                    }
                    Placement::Hypoid { parent, angle, offset, shaft_angle, spiral, reverse_hand, cutter_radius } => {
                        ui.label(if worm { "Worm" } else { "Pinion" });
                        parent_combo(ui, parent, &earlier_pinions);
                        ui.end_row();
                        ui.label("Shaft offset");
                        let hint = if worm {
                            "How far the worm's shaft passes from the wheel's axis. 0: the shafts meet. \
                             More offset makes a fatter worm, and turns the wheel's teeth straighter, \
                             so the overlap along the face falls."
                        } else {
                            "How far the pinion's shaft passes from the ring gear's axis. Positive puts a \
                             car's pinion below the ring's centre; 0 makes a spiral bevel pair."
                        };
                        len_drag(ui, offset, unit, -10000.0..=10000.0, 0.1).on_hover_text(hint);
                        ui.end_row();
                        ui.label("Shaft angle");
                        ui.horizontal(|ui| {
                            ui.selectable_value(shaft_angle, 90.0, "90°");
                            ui.add(egui::DragValue::new(shaft_angle).speed(0.5).range(30.0..=150.0).suffix("°"));
                        })
                        .response
                        .on_hover_text("Angle between the shafts, seen along the offset.");
                        ui.end_row();
                        ui.label("Spiral angle");
                        if worm {
                            ui.add(egui::DragValue::new(spiral).speed(0.2).range(20.0..=65.0).suffix("°")).on_hover_text(
                                "The worm's mean spiral angle: 90° less its lead angle. Steeper wraps the \
                                 threads further round and overlaps more teeth along the face.",
                            );
                        } else {
                            ui.add(egui::DragValue::new(spiral).speed(0.2).range(0.0..=60.0).suffix("°"))
                                .on_hover_text("The pinion's mean spiral angle. Cars use about 45–50°.");
                        }
                        ui.end_row();
                        ui.label("Hand");
                        let hint = if worm {
                            "Make the worm right-handed; it is left-handed unless this is ticked."
                        } else {
                            "Swap the spiral hands from the usual ones for this offset (a left-hand \
                             pinion with the pinion below centre)."
                        };
                        ui.checkbox(reverse_hand, "Opposite hand").on_hover_text(hint);
                        ui.end_row();
                        ui.label("Cutter radius");
                        let mut auto = *cutter_radius <= 0.0;
                        ui.horizontal(|ui| {
                            if ui.checkbox(&mut auto, "Auto").changed() {
                                *cutter_radius = if auto { 0.0 } else { hypoid_cutter.unwrap_or(100.0) };
                            }
                            if !auto {
                                len_drag(ui, cutter_radius, unit, 1.0..=2000.0, 0.5);
                            }
                        })
                        .response
                        .on_hover_text("Radius of the face-mill cutter that cuts the ring gear's slots; it sets the tooth curve.");
                        ui.end_row();
                        ui.label("Direction");
                        ui.add(egui::DragValue::new(angle).speed(0.5).range(-360.0..=360.0).suffix("°"))
                            .on_hover_text(if worm { "Turns the wheel round the worm's shaft." } else { "Turns the ring gear round the pinion's shaft." });
                        ui.end_row();
                    }
                    Placement::Worm { parent, angle, diameter, throated, globoid, left_hand, .. } => {
                        ui.label("Worm");
                        parent_combo(ui, parent, &earlier_worms);
                        ui.end_row();
                        ui.label("Worm pitch Ø");
                        len_drag(ui, diameter, unit, 1.0..=10000.0, 0.1).on_hover_text(
                            "Bigger makes a stiffer worm with a flatter lead angle: more likely to self-lock, \
                             but less efficient. About 10 modules is usual.",
                        );
                        ui.end_row();
                        ui.label("Type");
                        let kind = if *globoid { 2 } else if *throated { 1 } else { 0 };
                        let mut new_kind = kind;
                        ui.horizontal(|ui| {
                            ui.selectable_value(&mut new_kind, 0, "Straight").on_hover_text(
                                "A straight helical wheel, simple to make, touching the worm at a point: for light duty.",
                            );
                            ui.selectable_value(&mut new_kind, 1, "Throated").on_hover_text(
                                "Teeth hollowed to wrap round the worm, shaped to every place its threads pass, for \
                                 line contact and real load (single-enveloping).",
                            );
                            ui.selectable_value(&mut new_kind, 2, "Globoid").on_hover_text(
                                "An hourglass worm wrapped round a throated wheel (double-enveloping), so about \
                                 an eighth of the wheel's teeth bear at once. It has to be lined up closely, and \
                                 goes in from the side, not along its axis. Choosing it sizes the worm's length \
                                 and the wheel's face by AGMA 6135.",
                            );
                        });
                        if new_kind != kind {
                            (*throated, *globoid) = (new_kind >= 1, new_kind == 2);
                            globoid_req = *globoid;
                        }
                        ui.end_row();
                        ui.label("Hand");
                        ui.checkbox(left_hand, "Left hand").on_hover_text("Wind the threads and teeth left-handed; right-handed is usual.");
                        ui.end_row();
                        ui.label("Direction");
                        ui.add(egui::DragValue::new(angle).speed(0.5).range(-360.0..=360.0).suffix("°"))
                            .on_hover_text("Turns the wheel round the worm's shaft.");
                        ui.end_row();
                    }
                    Placement::Axle { parent, phase } => {
                        ui.label("Shares axle with");
                        parent_combo(ui, parent, &earlier);
                        ui.end_row();
                        ui.label("Phase");
                        ui.add(egui::DragValue::new(phase).speed(0.5).range(-360.0..=360.0).suffix("°"));
                        ui.end_row();
                    }
                });
                if !is_ring { egui::Grid::new("axial").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                    ui.label("Axial offset");
                    let hint = match g.placement {
                        Placement::Root { .. } => "Position of the back face along the axis.",
                        Placement::Axle { .. } => "Gap along the shaft after the part it shares the shaft with.",
                        _ => "Shift along the axis from centred on the partner's face.",
                    };
                    len_drag(ui, &mut g.axial, unit, -10000.0..=10000.0, 0.1).on_hover_text(hint);
                    ui.end_row();
                    if !g.params.is_bevel() && !g.params.is_hypoid() {
                        value_row(ui, "Faces at", format!("{} … {}", len(s.z, unit), len(s.z + g.params.face_width, unit)));
                    }
                }); }
            });

            section(ui, "Export origin", |ui| {
                let cone = g.params.is_bevel() || g.params.is_hypoid();
                let o = &mut g.export_origin;
                egui::Grid::new("export_origin").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                    ui.label("Origin at");
                    // A flat part's own origin is its bottom face.
                    let shown = if !cone && o.datum == Datum::Own { Datum::Bottom } else { o.datum };
                    let mut datum = shown;
                    let name = |d: Datum| match d {
                        Datum::Own => "Pitch apex",
                        Datum::Bottom => "Bottom face",
                        Datum::Middle => "Mid-face",
                        Datum::Top => "Top face",
                    };
                    egui::ComboBox::from_id_salt("datum").selected_text(name(datum)).show_ui(ui, |ui| {
                        for d in [Datum::Own, Datum::Bottom, Datum::Middle, Datum::Top] {
                            if d != Datum::Own || cone {
                                ui.selectable_value(&mut datum, d, name(d));
                            }
                        }
                    })
                    .response
                    .on_hover_text("A point on the gear's axis. Bottom and top are along +Z as exported.");
                    if datum != shown {
                        o.datum = datum;
                    }
                    ui.end_row();
                    ui.label("Offset");
                    len_drag(ui, &mut o.offset, unit, -10000.0..=10000.0, 0.1)
                        .on_hover_text("Moves the origin along the axis from the point picked above.");
                    ui.end_row();
                    if let Some(f) = faces {
                        let h = o.height(f, cone);
                        value_row(ui, "Faces at Z", format!("{} … {}", len(f[0] - h, unit), len(f[1] - h, unit)));
                    }
                });
                ui.label(RichText::new("Used when this part is exported on its own; a whole train keeps its layout.").weak().small());
            });

            let p = g.params.clone();
            let asked = match g.placement {
                Placement::Belt { target, .. } => target,
                _ => 0.0,
            };
            let fmt = |v: f64| len(v, unit);
            let mut issues = p.issues(fmt);
            match &pair_state {
                Some((_, Some(_))) if too_long.is_some() || ball_too_big.is_some() => {}
                Some((_, Some(e))) => issues.push(Issue { level: Level::Error, text: e.clone() }),
                Some(_) if p.is_hypoid() && hypoid_design.is_none() => {
                    let what = if p.worm { "bevel worm" } else { "hypoid" };
                    issues.push(Issue { level: Level::Error, text: format!("These settings have no {what} solution.") })
                }
                None if p.hypoid == Some(Role::Pinion) => issues.push(Issue {
                    level: Level::Warning,
                    text: if p.worm {
                        "No wheel: add a set with Worm Set, or point a wheel at this worm.".into()
                    } else {
                        "No ring gear: add one with Hypoid Set, or point a ring gear at this pinion.".into()
                    },
                }),
                None if p.worm_drive == Some(worm::Role::Worm) => issues.push(Issue {
                    level: Level::Warning,
                    text: "No wheel: add a drive with Worm Drive, or point a wheel at this worm.".into(),
                }),
                _ => {}
            }
            if let Some(d) = &worm_design {
                let least = worm::least_length(d.spec.starts, d.spec.teeth, d.spec.module);
                if let Some(eff) = d.effective_length() {
                    let eff = tenth(eff);
                    if d.spec.length < 0.9 * eff {
                        fixable.push((
                            Issue::warn(format!(
                                "The worm is {} long; AGMA 6135 makes it {}, wrapping more teeth.",
                                fmt(d.spec.length),
                                fmt(eff)
                            )),
                            format!("Lengthen the worm to {}", fmt(eff)),
                            Fix::WormLength(eff),
                        ));
                    }
                    let face = tenth(2.0 * d.worm_root);
                    if d.spec.face > 1.1 * face {
                        fixable.push((
                            Issue::warn(format!(
                                "The wheel's face is {}, wider than the worm's root Ø: its ends barely mesh. \
                                 AGMA 6135 makes it {}.",
                                fmt(d.spec.face),
                                fmt(face)
                            )),
                            format!("Narrow the face to {}", fmt(face)),
                            Fix::WheelFace(face),
                        ));
                    }
                    if let Some(z) = worm::globoid_least_teeth(d.center).filter(|&z| d.spec.teeth < z) {
                        issues.push(Issue::warn(format!(
                            "AGMA 6135 asks for at least {z} wheel teeth at this centre distance: give the wheel more \
                             teeth, and the worm more starts to keep the ratio."
                        )));
                    }
                } else if d.spec.length < least - 1e-6 {
                    issues.push(Issue::warn(format!(
                        "The worm is shorter than DIN 3975 asks for ({}): the end teeth lose contact early.",
                        fmt(least)
                    )));
                }
                if d.spec.throated && d.hourglass.is_none() && d.spec.face > 2.0 * d.worm_tip {
                    issues.push(Issue::warn("The wheel's face is wider than the worm: its ends barely mesh."));
                }
            }
            if let Some(d) = hypoid_design.as_ref().filter(|_| p.worm) {
                if d.face_contact_ratio() < 1.0 {
                    issues.push(Issue {
                        level: Level::Warning,
                        text: "The teeth overlap less than one pitch along the face, so the drive will be rough: \
                               raise the spiral angle or the face width."
                            .into(),
                    });
                }
            }
            section(ui, "Dimensions", |ui| {
                if !p.is_buildable() {
                    return;
                }
                if let Some(role) = p.worm_drive {
                    if pair_state.as_ref().is_some_and(|s| s.0) {
                        ui.label(RichText::new("⏳ Generating teeth…").weak());
                    }
                    let Some(d) = &worm_design else { return };
                    let deg = |a: f64| format!("{:.3}°", a.to_degrees());
                    let hand = if d.spec.left_hand { "left hand" } else { "right hand" };
                    egui::Grid::new("dims").num_columns(2).spacing([12.0, 4.0]).striped(true).show(ui, |ui| {
                        if let (worm::Role::Worm, Some(h)) = (role, &d.hourglass) {
                            value_row(ui, "Lead angle (throat)", format!("{}, {hand}", deg(d.lead_angle)));
                            value_row(ui, "Axial pitch", fmt(d.axial_pitch));
                            value_row(ui, "Lead", fmt(d.lead));
                            value_row(ui, "Pitch Ø (throat)", fmt(2.0 * d.worm_radius));
                            value_row(ui, "Throat Ø", fmt(2.0 * d.worm_tip));
                            value_row(ui, "Outside Ø (ends)", fmt(2.0 * d.worm_rim()));
                            value_row(ui, "Root Ø (throat)", fmt(2.0 * d.worm_root));
                            value_row(ui, "Base circle Ø", fmt(2.0 * h.base));
                            value_row(ui, "Thread length (AGMA)", fmt(d.effective_length().unwrap_or(0.0)));
                        } else if role == worm::Role::Worm {
                            value_row(ui, "Lead angle", format!("{}, {hand}", deg(d.lead_angle)));
                            value_row(ui, "Axial pitch", fmt(d.axial_pitch));
                            value_row(ui, "Lead", fmt(d.lead));
                            value_row(ui, "Pitch Ø", fmt(2.0 * d.worm_radius));
                            value_row(ui, "Outside Ø", fmt(2.0 * d.worm_tip));
                            value_row(ui, "Root Ø", fmt(2.0 * d.worm_root));
                            value_row(ui, "Diameter quotient", format!("{:.3}", 2.0 * d.worm_radius / d.spec.module));
                        } else {
                            value_row(ui, "Helix angle", format!("{}, {hand}", deg(d.lead_angle)));
                            value_row(ui, "Pitch Ø", fmt(2.0 * d.wheel_radius));
                            if d.spec.throated {
                                value_row(ui, "Throat Ø", fmt(2.0 * d.wheel_throat));
                                value_row(ui, "Outside Ø", fmt(2.0 * d.wheel_rim));
                            } else {
                                value_row(ui, "Outside Ø", fmt(2.0 * d.wheel_throat));
                            }
                            if d.spec.throated {
                                value_row(ui, "Root Ø (throat)", fmt(2.0 * (d.center - d.root_form)));
                            } else {
                                value_row(ui, "Root Ø", fmt(2.0 * d.wheel_root));
                            }
                        }
                        value_row(ui, "Root fillet", fmt(d.ball));
                        value_row(ui, "Root clearance", fmt(d.clearance));
                        if role == worm::Role::Worm {
                            let (mate, long) = d.end_chamfer();
                            value_row_tip(
                                ui,
                                "End chamfers",
                                format!("{:.0}° to the ends, to Ø{}", worm::END_CHAMFER, fmt(2.0 * mate)),
                                "Each end of the thread is chamfered down to where the wheel's tips reach, \
                                 so no part-thread is left as a feather edge.",
                            );
                            value_row_tip(
                                ui,
                                "Full threads",
                                fmt((d.spec.length - 2.0 * long).max(0.0)),
                                "The length of worm with the thread at its full outside, between the chamfers.",
                            );
                        }
                        if d.hourglass.is_some() {
                            value_row(ui, "Teeth wrapped", format!("{:.1}", d.teeth_spanned()));
                        }
                        value_row(ui, "Centre distance", fmt(d.center));
                        value_row(ui, "Ratio", format!("{:.4} : 1", d.spec.teeth as f64 / d.spec.starts as f64));
                    });
                    return;
                }
                if let Some(role) = p.hypoid {
                    if pair_state.as_ref().is_some_and(|s| s.0) {
                        ui.label(RichText::new("⏳ Generating teeth…").weak());
                    }
                    let Some(d) = &hypoid_design else { return };
                    let deg = |a: f64| format!("{:.3}°", a.to_degrees());
                    let hand = |left: bool| if left { "left hand" } else { "right hand" };
                    egui::Grid::new("dims").num_columns(2).spacing([12.0, 4.0]).striped(true).show(ui, |ui| {
                        if role == Role::Ring {
                            value_row(ui, "Pitch cone angle", deg(d.ring_pitch));
                            value_row(ui, "Face cone angle", deg(d.ring_face_angle()));
                            value_row(ui, "Root cone angle", deg(d.ring_root_angle()));
                            value_row(ui, "Mean spiral angle", format!("{}, {}", deg(d.ring_spiral.abs()), hand(hypoid::ring_left_hand(d))));
                            value_row(ui, "Pitch Ø (heel)", fmt(d.spec.module * d.spec.ring_teeth as f64));
                            value_row(ui, "Mean pitch Ø", fmt(2.0 * d.ring_radius));
                            value_row(ui, "Mean cone distance", fmt(d.ring_cone));
                            value_row(ui, "Addendum / dedendum", format!("{} / {}", fmt(d.ring_addendum), fmt(d.ring_dedendum)));
                        } else {
                            if p.worm {
                                value_row(ui, "Starts", d.spec.pinion_teeth.to_string());
                                value_row(ui, "Mean lead angle", deg(PI / 2.0 - d.pinion_spiral.abs()));
                                value_row(ui, "Pitch Ø (heel)", fmt(d.spec.module * d.spec.pinion_teeth as f64));
                            }
                            value_row(ui, "Pitch cone angle", deg(d.pinion_pitch));
                            value_row(ui, "Face cone angle", deg(d.pinion_face_angle()));
                            value_row(ui, "Root cone angle", deg(d.pinion_root_angle()));
                            value_row(ui, "Mean spiral angle", format!("{}, {}", deg(d.pinion_spiral.abs()), hand(hypoid::pinion_left_hand(d))));
                            value_row(ui, "Mean pitch Ø", fmt(2.0 * d.pinion_radius));
                            value_row(ui, "Mean cone distance", fmt(d.pinion_cone));
                            value_row(ui, "Addendum / dedendum", format!("{} / {}", fmt(d.pinion_addendum), fmt(d.pinion_dedendum)));
                        }
                        value_row(ui, "Ratio", format!("{:.4} : 1", d.spec.ring_teeth as f64 / d.spec.pinion_teeth as f64));
                        if p.worm {
                            value_row(ui, "Face contact ratio", format!("{:.2}", d.face_contact_ratio()));
                        }
                        value_row(ui, "Shaft offset", fmt(d.spec.offset));
                        value_row(ui, "Mean normal module", format!("{:.4} mm", d.mn));
                        value_row(ui, "Whole depth", fmt(d.depth + d.clearance));
                        value_row(ui, "Cutter radius", fmt(d.cutter.radius));
                        if let Some(h) = raised {
                            let (label, tip) = if role == Role::Ring {
                                ("Slot floors raised", "So a ball end mill square to the ring's axis can get down its curved slots, their floors are raised by up to this much.")
                            } else {
                                ("Tips cut down", "The ring's slot floors are raised for the ball end mill's reach, so the tips are cut down by up to this much to keep their clearance.")
                            };
                            value_row_tip(ui, label, format!("up to {}", fmt(h)), tip);
                        }
                    });
                    return;
                }
                if let Some(b) = p.belt {
                    let rack = b.rack(p.teeth);
                    let w = b.width_near(p.belt_width);
                    egui::Grid::new("dims").num_columns(2).spacing([12.0, 4.0]).striped(true).show(ui, |ui| {
                        value_row(ui, "Pitch Ø", fmt(2.0 * p.pitch_radius()));
                        value_row(ui, "Outside Ø", fmt(2.0 * p.tip_radius()));
                        value_row(ui, "Root Ø", fmt(2.0 * p.root_radius()));
                        value_row(ui, "Belt pitch", fmt(b.pitch()));
                        value_row(ui, "Groove depth", fmt(rack.hr));
                        value_row(ui, "Pitch line differential", fmt(rack.pld));
                        value_row(ui, "Min face, flanged", fmt(w.flanged));
                        value_row(ui, "Min face, unflanged", fmt(w.unflanged));
                    });
                    return;
                }
                if let Some(b) = BevelGeom::new(&p) {
                    let deg = |a: f64| format!("{:.3}°", a.to_degrees());
                    egui::Grid::new("dims").num_columns(2).spacing([12.0, 4.0]).striped(true).show(ui, |ui| {
                        value_row(ui, "Pitch cone angle", deg(b.delta));
                        value_row(ui, "Tip cone angle", deg(b.tip));
                        value_row(ui, "Root cone angle", deg(b.root));
                        value_row(ui, "Base cone angle", deg(b.base));
                        value_row(ui, "Cone distance", fmt(b.cone_distance));
                        value_row(ui, "Pitch Ø (large end)", fmt(2.0 * p.pitch_radius()));
                        value_row(ui, "Outside Ø", fmt(b.outside_diameter()));
                        value_row(ui, "Face width (max R/3)", format!("{} ({})", fmt(b.face), fmt(b.cone_distance / 3.0)));
                        value_row(ui, "Virtual teeth", format!("{:.1}", b.virtual_teeth()));
                        let (zb, _) = b.back_face();
                        value_row(ui, "Apex to back face", fmt(zb));
                        value_row(ui, "Circular pitch", fmt(p.circular_pitch()));
                    });
                    return;
                }
                egui::Grid::new("dims").num_columns(2).spacing([12.0, 4.0]).striped(true).show(ui, |ui| {
                    value_row(ui, "Pitch Ø", fmt(2.0 * p.pitch_radius()));
                    value_row(ui, "Tip Ø", fmt(2.0 * p.tip_radius()));
                    value_row(ui, "Root Ø", fmt(2.0 * p.root_radius()));
                    value_row(ui, "Base Ø", fmt(2.0 * p.base_radius()));
                    value_row(ui, "Circular pitch", fmt(p.circular_pitch()));
                    value_row(ui, "Tooth thickness", fmt(p.pitch_thickness()));
                    value_row(ui, "Tip land", fmt(p.thickness_at(p.tip_radius()).max(0.0)));
                    value_row(ui, "Whole depth", fmt(p.tip_radius() - p.root_radius()));
                    value_row(ui, "Min shift (no undercut)", format!("{:.3}", p.min_shift_no_undercut()));
                    if unit.is_inch() {
                        value_row(ui, "Module (equivalent)", format!("{:.4} mm", p.module));
                    } else {
                        value_row(ui, "Diametral pitch (equivalent)", format!("{:.4} /in", 25.4 / p.module));
                    }
                });
            });

            section(ui, "Motion", |ui| {
                egui::Grid::new("motion").num_columns(2).spacing([12.0, 4.0]).striped(true).show(ui, |ui| {
                    value_row(ui, "Ratio to driver", fmt_ratio(s.ratio));
                    let speed = rpm * s.ratio;
                    value_row(ui, "Speed", format!("{speed:.4} rpm {}", if speed < 0.0 { "(CW)" } else if speed > 0.0 { "(CCW)" } else { "" }));
                    value_row(ui, "One turn every", fmt_period(speed));
                });
            });

            if let (Some(mi), Some(pn)) = (&s.mesh, &parent_name) {
                issues.extend(mi.issues(p.module, fmt));
                section(ui, &format!("Mesh with {pn}"), |ui| {
                    egui::Grid::new("mesh").num_columns(2).spacing([12.0, 4.0]).striped(true).show(ui, |ui| {
                        value_row(ui, "Centre distance", fmt(mi.center));
                        if (mi.center - mi.standard_center).abs() > 1e-6 {
                            value_row(ui, "Standard centre distance", fmt(mi.standard_center));
                        }
                        value_row(ui, "Working pressure angle", format!("{:.3}°", mi.working_pa.to_degrees()));
                        value_row(ui, "Contact ratio", format!("{:.3}", mi.contact_ratio));
                        value_row(ui, "Tip clearance", fmt(mi.clearance));
                        value_row(ui, "Backlash (pair)", fmt(mi.backlash));
                    });
                });
            }
            if let (Some(bi), Some(pn)) = (&s.belt, &parent_name) {
                issues.extend(bi.issues());
                section(ui, &format!("Belt to {pn}"), |ui| {
                    egui::Grid::new("beltinfo").num_columns(2).spacing([12.0, 4.0]).striped(true).show(ui, |ui| {
                        value_row(ui, "Belt", bi.designation.clone().unwrap_or_else(|| "Non-standard length".into()));
                        value_row(ui, "Belt teeth", bi.teeth.to_string());
                        value_row(ui, "Pitch length", fmt(bi.length));
                        value_row(ui, "Centre distance", fmt(bi.center));
                        if asked > 0.0 && (bi.center - asked).abs() > 1e-6 {
                            let d = bi.center - asked;
                            value_row(ui, "Asked for", fmt(asked));
                            value_row(ui, "Difference", format!("{}{}", if d > 0.0 { "+" } else { "" }, fmt(d)));
                        }
                        value_row(ui, "Wrap, smaller pulley", format!("{:.1}°", bi.wrap.to_degrees()));
                        value_row(ui, "Teeth in mesh", format!("{:.1}", bi.teeth_in_mesh));
                    });
                });
            }
            for o in others {
                issues.push(Issue { level: Level::Error, text: format!("Overlaps {o} without meshing.") });
            }

            if let (Some(largest), Some((_, _, why, _))) = (ball_too_big, &room) {
                fixable.insert(
                    0,
                    (
                        Issue::error(format!(
                            "The Ø{} ball end mill is too big for these teeth: {why}. The biggest that fits is Ø{}; \
                             {bigger} takes a bigger one.",
                            fmt(ball),
                            fmt(largest)
                        )),
                        format!("Cut with a Ø{} ball end mill", fmt(largest)),
                        Fix::Ball(largest),
                    ),
                );
            } else if let (0.0, true, Some((now, _, _, unset))) = (ball, machined, &room) {
                // Shown as the setting will hold it, so pressing the fix
                // leaves the teeth as they are.
                let now = shown_floor(*now, unit);
                fixable.push((
                    Issue::warn(format!(
                        "No ball end mill is chosen, so the roots are rounded for a Ø{} ball, {unset}. Choose the one you'll cut with under Machining, or keep this.",
                        fmt(now)
                    )),
                    format!("Cut with a Ø{} ball end mill", fmt(now)),
                    Fix::Ball(now),
                ));
            }
            if let Some((now, (most, agma))) = too_long {
                let agma = tenth(agma);
                fixable.insert(
                    0,
                    (
                        Issue::error(format!(
                            "The worm is {} long, past its base circle Ø of {}: its end threads would overhang \
                             their roots, so nothing could cut them.",
                            fmt(now),
                            fmt(most)
                        )),
                        format!("Shorten the worm to {} (AGMA 6135)", fmt(agma)),
                        Fix::WormLength(agma),
                    ),
                );
            }
            if let (Some(h), Some(d)) = (raised, &hypoid_design) {
                if h > 0.25 * d.depth {
                    issues.push(Issue::warn(format!(
                        "For a Ø{} ball end mill square to the ring's axis to get down its curved slots, their floors \
                         are raised by up to {} and the pinion's tips cut down to match: the teeth touch over {:.0}% \
                         less of their working depth there. Less spiral angle needs less.",
                        fmt(ball),
                        fmt(h),
                        100.0 * h / d.depth
                    )));
                }
            }
            match &reach {
                Some(Some(c)) if ball_too_big.is_none() && c.reach.share() > reach::STRAY => {
                    let [a, b] = c.reach.radii.unwrap_or_default();
                    let advice = if p.is_hypoid() && !p.worm {
                        "the slots curve along the face more than a tool square to the axis can follow, even with \
                         their floors raised by half the working depth; less spiral angle opens them up"
                    } else if !p.is_generated() && !p.is_bevel() && !p.is_pulley() {
                        "the flanks are undercut, facing away from every way the tool can come in; \
                         a positive profile shift, or a smaller ball, clears them"
                    } else {
                        "a smaller ball gets further in"
                    };
                    issues.push(Issue::warn(format!(
                        "On the 4-axis mill, a Ø{} ball end mill square to the part's axis can't reach {:.1}% of the \
                         teeth, from {} to {} off the axis (marked in 3D): {advice}.",
                        fmt(ball),
                        100.0 * c.reach.share(),
                        fmt(a),
                        fmt(b)
                    )));
                }
                _ => {}
            }
            ui.add_space(6.0);
            if matches!(reach, Some(None)) {
                ui.label(RichText::new("⏳ Checking the ball end mill's reach…").weak());
            }
            if issues.is_empty() && fixable.is_empty() {
                ui.label(RichText::new("✔ No problems found").color(ui.visuals().weak_text_color()));
            }
            for (is, label, fix) in fixable {
                let color =
                    if is.level == Level::Error { ui.visuals().error_fg_color } else { ui.visuals().warn_fg_color };
                let icon = if is.level == Level::Error { "⛔" } else { "⚠" };
                ui.label(RichText::new(format!("{icon} {}", is.text)).color(color));
                if ui.button(label).clicked() {
                    fix_req = Some(fix);
                }
            }
            for is in issues {
                let (icon, color) = match is.level {
                    Level::Warning => ("⚠", ui.visuals().warn_fg_color),
                    Level::Error => ("⛔", ui.visuals().error_fg_color),
                };
                ui.label(RichText::new(format!("{icon} {}", is.text)).color(color));
            }
        });

        if let (Some(pr), Some(t)) = (&pair, ratio_target) {
            self.ratio_target.insert(pr.driven, t);
        }
        if let (Some(r), Some(mu)) = (pair_driven, friction) {
            if let Placement::Worm { friction, .. } = &mut self.project.gears[r].placement {
                *friction = mu;
            }
        }
        if back_req {
            self.make_back_drivable(idx);
        }
        if globoid_req {
            self.project.make_globoid(idx, false);
        }
        if let Some(fix) = fix_req {
            match (fix, pair_driven) {
                (Fix::WormLength(l), Some(r)) => {
                    let worm = self.project.gears[r].placement.parent().and_then(|p| self.project.index_of(p));
                    if let Some(k) = worm {
                        self.project.gears[k].params.face_width = l;
                    }
                }
                (Fix::WheelFace(b), Some(r)) => self.project.gears[r].params.face_width = b,
                (Fix::Ball(d), _) => self.project.ball_end_mill = d,
                _ => {}
            }
            self.project.normalize();
        }
        if let (Some(pr), Some(t)) = (&pair, ratio_req) {
            match self.project.size_pair_to_ratio(idx, t, self.ratio_max_teeth) {
                Ok(r) => {
                    // A belt keeps the pulleys about as far apart as they were.
                    if let Some(c) = pr.belt_center {
                        self.project.fit_belt(pr.driven, c, self.belt_standard_only);
                    }
                    let got = r.driven as f64 / r.driving as f64;
                    let off = (got / t - 1.0) * 100.0;
                    let mut note = if off.abs() < 1e-9 {
                        format!("Exact: {} : {}", r.driven, r.driving)
                    } else {
                        format!("Nearest within the limits: {got:.4} : 1, {off:+.2} % off")
                    };
                    if let Some(h) = r.held {
                        note += &format!(". {h} kept its teeth, as it also turns another part");
                    }
                    self.ratio_result = Some((pr.driven, r.driving, r.driven, note + "."));
                }
                Err(e) => self.flash(e),
            }
        }

        let g = &self.project.gears[idx];
        let id = g.id;
        if fit_start {
            let pp = g.placement.parent().and_then(|p| self.project.get(p));
            fit_req = pp.map(|pp| train::starting_center(&pp.params, &g.params));
        }
        if let Some(t) = fit_req {
            if self.project.fit_belt(id, t, self.belt_standard_only).is_none() {
                self.flash("No belt fits: both ends need pulleys of the same belt section.");
            } else if fit_start {
                if let Placement::Belt { target, .. } = &mut self.project.gears[idx].placement {
                    *target = 0.0;
                }
            }
        }
    }

    fn status_bar(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            match self.cursor {
                Some([x, y]) => {
                    let u = self.project.pitch_unit;
                    let (f, d) = (u.factor(), u.decimals());
                    ui.monospace(format!("X {:9.d$}  Y {:9.d$}{}", x / f, y / f, u.suffix()))
                }
                None => ui.monospace(" "),
            };
            ui.separator();
            ui.label(format!("{:.2} px/mm", self.view.scale));
            ui.separator();
            let n = self.project.gears.iter().filter(|g| self.shown(g)).count();
            ui.label(match self.workspace {
                Workspace::Spur => format!("{n} gears"),
                Workspace::Bevel => format!("{n} bevel gears"),
                Workspace::Worm => format!("{n} worm drive parts"),
                Workspace::BevelWorm => format!("{n} bevel worm parts"),
                Workspace::Pulleys => format!("{n} pulleys"),
            });
            if let Some((msg, at)) = &self.status {
                if at.elapsed() < Duration::from_secs(6) {
                    ui.separator();
                    ui.label(msg);
                }
            }
        });
    }

    fn shortcuts(&mut self, ctx: &egui::Context) {
        let cmd = |k| KeyboardShortcut::new(Modifiers::COMMAND, k);
        let cmd_shift = |k| KeyboardShortcut::new(Modifiers::COMMAND | Modifiers::SHIFT, k);
        let pressed = |s: KeyboardShortcut| ctx.input_mut(|i| i.consume_shortcut(&s));
        if pressed(cmd_shift(Key::S)) {
            self.save(true, None);
        } else if pressed(cmd(Key::S)) {
            self.save(false, None);
        }
        if pressed(cmd(Key::O)) {
            self.request(ctx, Pending::Open);
        }
        if pressed(cmd(Key::N)) {
            self.request(ctx, Pending::New(self.project.pitch_unit));
        }
        if pressed(cmd(Key::Q)) {
            self.request(ctx, Pending::Quit);
        }
        if pressed(cmd_shift(Key::Z)) || pressed(cmd(Key::Y)) {
            self.redo();
        } else if pressed(cmd(Key::Z)) {
            self.undo();
        }
        // Single-key shortcuts only when no text field has the keyboard.
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let key = |k| ctx.input(|i| i.key_pressed(k) && i.modifiers.is_none());
        if key(Key::Space) {
            self.playing = !self.playing;
        }
        if key(Key::F) {
            self.fit_pending = true;
        }
        // Each workspace's own tools: M/A for gears, B/P for pulleys.
        match self.workspace {
            Workspace::Spur => {
                if key(Key::M) {
                    self.add_gear(Link::Mesh, false);
                }
                if key(Key::A) {
                    self.add_gear(Link::Axle, false);
                }
            }
            Workspace::Bevel => {
                if key(Key::M) {
                    self.add_bevel(Link::Bevel);
                }
                if key(Key::A) {
                    self.add_bevel(Link::Axle);
                }
            }
            Workspace::Worm | Workspace::BevelWorm => {}
            Workspace::Pulleys => {
                if key(Key::B) {
                    self.add_gear(Link::Belt, true);
                }
                if key(Key::P) {
                    self.add_gear(Link::Axle, true);
                }
            }
        }
        if key(Key::Delete) {
            self.delete_selected();
        }
        if key(Key::Escape) {
            self.selected = None;
        }
    }

    fn unsaved_dialog(&mut self, ctx: &egui::Context) {
        let Some(what) = self.pending else { return };
        let mut choice = None;
        egui::Modal::new(egui::Id::new("unsaved")).show(ctx, |ui| {
            ui.set_width(320.0);
            ui.heading("Save changes?");
            ui.add_space(4.0);
            ui.label("This train has changes that have not been saved.");
            ui.add_space(10.0);
            ui.horizontal(|ui| {
                if ui.button("Save").clicked() {
                    choice = Some(0);
                }
                if ui.button("Don't save").clicked() {
                    choice = Some(1);
                }
                if ui.button("Cancel").clicked() || ui.input(|i| i.key_pressed(Key::Escape)) {
                    choice = Some(2);
                }
            });
        });
        match choice {
            Some(0) => {
                self.pending = None;
                if self.save(false, Some(what)) {
                    self.perform(ctx, what);
                }
            }
            Some(1) => {
                self.pending = None;
                self.dirty = false;
                self.perform(ctx, what);
            }
            Some(_) => self.pending = None,
            None => {}
        }
    }

    fn unit_dialog(&mut self, ctx: &egui::Context) {
        let Some(unit) = self.unit_switch else { return };
        let (to, from_desc, to_desc) = match unit {
            PitchUnit::DiametralPitch => ("inch", "metric (module)", "standard diametral pitch with AGMA tooth proportions"),
            PitchUnit::Module => ("metric", "inch (diametral pitch)", "standard module with ISO 53 tooth proportions"),
        };
        let n = self.project.gears.len();
        let mut choice = None;
        egui::Modal::new(egui::Id::new("unit_switch")).show(ctx, |ui| {
            ui.set_width(380.0);
            ui.heading(format!("Switch to {to} gears?"));
            ui.add_space(4.0);
            ui.label(format!(
                "The {n} gear{} in this project {} {from_desc}.",
                if n == 1 { "" } else { "s" },
                if n == 1 { "is" } else { "are" }
            ));
            ui.add_space(4.0);
            ui.label(format!("Convert: resize each gear to the nearest {to_desc}. Tooth counts stay the same."));
            ui.label("Keep sizes: leave the gears exactly as they are and just show the other units.");
            ui.add_space(10.0);
            ui.horizontal(|ui| {
                if ui.button("Convert gears").clicked() {
                    choice = Some(0);
                }
                if ui.button("Keep sizes").clicked() {
                    choice = Some(1);
                }
                if ui.button("Cancel").clicked() || ui.input(|i| i.key_pressed(Key::Escape)) {
                    choice = Some(2);
                }
            });
        });
        match choice {
            Some(0) => {
                self.project.convert_to(unit);
                self.unit_switch = None;
                self.flash(format!("Converted {n} gears to {to}"));
            }
            Some(1) => {
                self.project.pitch_unit = unit;
                self.unit_switch = None;
            }
            Some(_) => self.unit_switch = None,
            None => {}
        }
    }

    fn update_title(&mut self, ctx: &egui::Context) {
        let name = self.doc_name();
        let title = format!("{}{name} — GearCraft", if self.dirty { "• " } else { "" });
        if title != self.title {
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(title.clone()));
            self.title = title;
        }
    }
}

impl eframe::App for GearCraftApp {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        // New parts, and a changed setting, take the ball end mill and
        // the nozzle.
        self.project.apply_making();
        self.poll_file_dialog(&ctx);
        if ctx.input(|i| i.viewport().close_requested()) && self.dirty {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.pending = Some(Pending::Quit);
        }
        if self.pending.is_none() && self.unit_switch.is_none() {
            self.shortcuts(&ctx);
        }
        if self.playing {
            let dt = ctx.input(|i| i.stable_dt).min(0.1) as f64;
            self.driver_angle = (self.driver_angle + self.project.driver_rpm * TAU / 60.0 * dt) % (TAU * 1e6);
            ctx.request_repaint();
        }

        egui::Panel::top("menu")
            .exact_size(chrome::APP_BAR_HEIGHT)
            .frame(chrome::app_bar_frame())
            .show_separator_line(false)
            .show(ui, |ui| self.menu_bar(ui));
        egui::Panel::top("ribbon")
            .frame(egui::Frame::new().fill(ui.visuals().panel_fill).inner_margin(egui::Margin::same(6)))
            .show(ui, |ui| self.ribbon(ui));
        egui::Panel::bottom("status").show(ui, |ui| self.status_bar(ui));
        egui::Panel::left("tree").resizable(true).default_size(220.0).min_size(160.0).show(ui, |ui| self.tree(ui));
        egui::Panel::right("props")
            .resizable(true)
            .default_size(340.0)
            .min_size(280.0)
            .show(ui, |ui| self.properties(ui));
        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| self.canvas(ui));

        self.project.normalize();
        self.track_history(&ctx);
        self.unsaved_dialog(&ctx);
        self.unit_dialog(&ctx);
        self.update_title(&ctx);
        if self.status.as_ref().is_some_and(|(_, at)| at.elapsed() < Duration::from_secs(7)) {
            ctx.request_repaint_after(Duration::from_millis(500));
        }
    }
}

fn circle_points(c: Pos2, r: f32) -> Vec<Pos2> {
    let n = ((r * 0.5) as usize).clamp(24, 720);
    (0..=n)
        .map(|i| {
            let a = i as f32 / n as f32 * std::f32::consts::TAU;
            c + Vec2::new(a.cos(), a.sin()) * r
        })
        .collect()
}

/// Lighter than the plan colours: the 3D shading darkens them.
fn gear_color_3d(index: usize, depth: usize) -> Color32 {
    let hue = ((index as f32 * 0.618_034 + depth as f32 * 0.07) % 1.0 + 0.55) % 1.0;
    egui::ecolor::Hsva::new(hue, 0.32, 0.86, 1.0).into()
}

fn gear_color(index: usize, depth: usize, dark: bool) -> Color32 {
    let hue = ((index as f32 * 0.618_034 + depth as f32 * 0.07) % 1.0 + 0.55) % 1.0;
    let (s, v) = if dark { (0.35, 0.42) } else { (0.30, 0.88) };
    egui::ecolor::Hsva::new(hue, s, v, 1.0).into()
}

fn section(ui: &mut Ui, title: &str, body: impl FnOnce(&mut Ui)) {
    egui::CollapsingHeader::new(RichText::new(title).strong()).default_open(true).show(ui, body);
    ui.add_space(2.0);
}

fn value_row(ui: &mut Ui, label: &str, value: String) {
    ui.label(label);
    ui.label(RichText::new(value).monospace());
    ui.end_row();
}

/// A `value_row` explained on hover.
fn value_row_tip(ui: &mut Ui, label: &str, value: String, tip: &str) {
    ui.label(label).on_hover_text(tip);
    ui.label(RichText::new(value).monospace()).on_hover_text(tip);
    ui.end_row();
}

fn parent_combo(ui: &mut Ui, parent: &mut u64, choices: &[(u64, String)]) {
    let current = choices.iter().find(|c| c.0 == *parent).map(|c| c.1.as_str()).unwrap_or("?");
    egui::ComboBox::from_id_salt("parent").selected_text(current).show_ui(ui, |ui| {
        for (id, name) in choices {
            ui.selectable_value(parent, *id, name);
        }
    });
}

/// Rounded down to 0.1 mm, so a length set from a limit stays inside it.
fn tenth(v: f64) -> f64 {
    (v * 10.0).floor() / 10.0
}

/// A length (mm) rounded down to the places it is shown to, so a value set
/// from a limit stays inside it.
fn shown_floor(v: f64, unit: PitchUnit) -> f64 {
    let k = 10f64.powi(unit.decimals() as i32 - 1) / unit.factor();
    (v * k + 1e-9).floor() / k
}

/// A length (mm) in the project's units.
fn len(v: f64, unit: PitchUnit) -> String {
    format!("{:.*}{}", unit.decimals(), v / unit.factor(), unit.suffix())
}

/// A length field shown in the project's units but stored in mm.
fn len_drag(ui: &mut Ui, v: &mut f64, unit: PitchUnit, range_mm: RangeInclusive<f64>, speed_mm: f64) -> egui::Response {
    let f = unit.factor();
    let mut d = *v / f;
    let r = ui.add(
        egui::DragValue::new(&mut d)
            .range(*range_mm.start() / f..=*range_mm.end() / f)
            .speed(speed_mm / f)
            .max_decimals(unit.decimals() + 1)
            .suffix(unit.suffix()),
    );
    // Only write back on an edit, so redisplaying never drifts the value.
    if r.changed() {
        *v = d * f;
    }
    r
}

/// Module or diametral pitch entry, plus a pick list of standard sizes.
fn pitch_input(ui: &mut Ui, module: &mut f64, unit: PitchUnit) {
    if unit.is_inch() {
        let mut dp = 25.4 / *module;
        let r = ui.add(egui::DragValue::new(&mut dp).range(0.25..=500.0).speed(0.05).max_decimals(4).suffix(" DP"));
        if r.changed() {
            *module = 25.4 / dp;
        }
        egui::ComboBox::from_id_salt("std_pitch").selected_text("Standard").width(80.0).show_ui(ui, |ui| {
            for d in STD_DPS {
                if ui.selectable_label((dp - d).abs() < 1e-9, format!("{d} DP")).clicked() {
                    *module = 25.4 / d;
                }
            }
        });
    } else {
        ui.add(egui::DragValue::new(module).range(0.05..=100.0).speed(0.01).max_decimals(4).suffix(" mm"));
        egui::ComboBox::from_id_salt("std_pitch").selected_text("Standard").width(80.0).show_ui(ui, |ui| {
            for m in STD_MODULES {
                if ui.selectable_label((*module - m).abs() < 1e-9, format!("{m} mm")).clicked() {
                    *module = m;
                }
            }
        });
    }
}

/// The pair the Ratio section is about, read before the panel borrows the
/// project.
struct RatioPair {
    driven: u64,
    partner: String,
    drives: bool,
    /// (driving, driven)
    teeth: (u32, u32),
    worm: bool,
    belt_center: Option<f64>,
}

fn fmt_ratio(r: f64) -> String {
    if r == 0.0 {
        return "0".into();
    }
    let dir = if r < 0.0 { " (reversed)" } else { "" };
    let a = r.abs();
    if a >= 1.0 { format!("{a:.4} : 1{dir}") } else { format!("1 : {:.4}{dir}", 1.0 / a) }
}

fn fmt_period(rpm: f64) -> String {
    if rpm.abs() < 1e-12 {
        return "never".into();
    }
    let secs = 60.0 / rpm.abs();
    if secs < 120.0 {
        format!("{secs:.3} s")
    } else if secs < 7200.0 {
        format!("{:.3} min", secs / 60.0)
    } else if secs < 172_800.0 {
        format!("{:.3} h", secs / 3600.0)
    } else {
        format!("{:.3} days", secs / 86400.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::train::Project;
    use egui_kittest::{Harness, kittest::Queryable};

    /// A project with a gear pair and a belted pulley pair.
    fn mixed() -> Project {
        let mut p = Project::empty();
        let g = p.add_root(PitchUnit::Module.starter(24), 0.0, 0.0);
        p.add_child(g, PitchUnit::Module.starter(36), Link::Mesh);
        let q = p.add_root(PitchUnit::Module.starter_pulley(20), 300.0, 0.0);
        p.add_child(q, PitchUnit::Module.starter_pulley(30), Link::Belt);
        p.normalize();
        p
    }

    #[test]
    fn switching_workspace_shows_only_its_parts() {
        let mut harness = egui_kittest::Harness::builder().with_size(Vec2::new(1400.0, 860.0)).build_eframe(|cc| {
            let mut app = GearCraftApp::new(cc, None);
            app.replace_project(mixed(), None);
            app
        });
        harness.run_steps(3);
        let listed = |h: &Harness<'_, GearCraftApp>, name: &str| h.query_by_label_contains(name).is_some();
        assert!(listed(&harness, "Gear 1") && !listed(&harness, "Pulley 1"));
        assert_eq!(harness.state().selected, harness.state().project.gears.first().map(|g| g.id));

        harness.state_mut().switch_workspace(Workspace::Pulleys);
        harness.run_steps(3);
        assert!(!listed(&harness, "Gear 1") && listed(&harness, "Pulley 1"));
        let app = harness.state();
        let sel = app.selected.and_then(|id| app.project.get(id)).unwrap();
        assert!(sel.params.is_pulley(), "selection moves to a pulley");

        // Back again: the gears return, with the gear selection restored.
        let pulley = app.selected;
        harness.state_mut().switch_workspace(Workspace::Spur);
        harness.run_steps(3);
        assert!(listed(&harness, "Gear 1") && !listed(&harness, "Pulley 1"));
        assert_eq!(harness.state().selected, harness.state().project.gears.first().map(|g| g.id));
        harness.state_mut().switch_workspace(Workspace::Pulleys);
        assert_eq!(harness.state().selected, pulley);
        assert_eq!(harness.state().project.gears.len(), 4, "nothing was removed");
    }

    #[test]
    fn a_globoid_worm_too_long_for_its_base_circle_is_shortened_by_its_fix() {
        let u = PitchUnit::Module;
        let mut p = Project::empty();
        let w = p.add_root(u.starter_worm_drive(worm::Role::Worm), 0.0, 0.0);
        p.add_child(w, u.starter_worm_drive(worm::Role::Wheel), Link::Worm);
        p.make_globoid(1, true);
        p.gears[0].params.face_width = 30.0;
        let mut harness = egui_kittest::Harness::builder().with_size(Vec2::new(1400.0, 1500.0)).build_eframe(|cc| {
            let mut app = GearCraftApp::new(cc, None);
            app.replace_project(p, None);
            app.switch_workspace(Workspace::Worm);
            app.selected = Some(w);
            app
        });
        harness.run_steps(3);
        harness.get_by_label_contains("past its base circle");
        harness.get_by_label_contains("Shorten the worm to").click();
        harness.run_steps(3);
        let app = harness.state();
        let (worm, wheel) = (&app.project.gears[0], &app.project.gears[1]);
        let spec = train::worm_spec(&worm.params, &wheel.params, &wheel.placement, app.project.ball_end_mill).unwrap();
        let (_, agma) = spec.globoid_lengths().unwrap();
        assert!(worm.params.face_width < 30.0 && agma - worm.params.face_width < 0.1);
        assert!(spec.design().is_ok());
        assert!(harness.query_by_label_contains("past its base circle").is_none());
    }

    #[test]
    fn a_ball_end_mill_too_big_for_the_teeth_is_swapped_by_its_fix() {
        let u = PitchUnit::Module;
        let mut p = Project::empty();
        let w = p.add_root(u.starter_worm_drive(worm::Role::Worm), 0.0, 0.0);
        p.add_child(w, u.starter_worm_drive(worm::Role::Wheel), Link::Worm);
        p.ball_end_mill = 3.0;
        let mut harness = egui_kittest::Harness::builder().with_size(Vec2::new(1400.0, 1500.0)).build_eframe(|cc| {
            let mut app = GearCraftApp::new(cc, None);
            app.replace_project(p, None);
            app.switch_workspace(Workspace::Worm);
            app.selected = Some(w);
            app
        });
        harness.run_steps(3);
        harness.get_by_label_contains("ball end mill is too big");
        harness.get_by_label_contains("Cut with a").click();
        harness.run_steps(3);
        let app = harness.state();
        let (worm, wheel) = (&app.project.gears[0], &app.project.gears[1]);
        let spec = train::worm_spec(&worm.params, &wheel.params, &wheel.placement, app.project.ball_end_mill).unwrap();
        let largest = spec.largest_ball().unwrap();
        assert!(app.project.ball_end_mill <= largest && largest - app.project.ball_end_mill < 0.01);
        assert!(spec.design().is_ok());
        assert!(harness.query_by_label_contains("too big").is_none());
        assert!(harness.query_by_label_contains("No ball end mill").is_none());
    }

    /// Machined and Printed are two ways of making the parts: choosing one
    /// drops the other, and the nozzle reaches the parts.
    #[test]
    fn printed_and_machined_are_either_or() {
        let mut p = Project::empty();
        p.add_root(PitchUnit::Module.starter(24), 0.0, 0.0);
        let mut harness = Harness::builder().with_size(Vec2::new(1400.0, 1500.0)).build_eframe(|cc| {
            let mut app = GearCraftApp::new(cc, None);
            app.replace_project(p, None);
            app
        });
        harness.run_steps(3);
        assert!(harness.state().project.for_machining);
        harness.get_by_label_contains("Printed").click();
        harness.run_steps(3);
        let project = &harness.state().project;
        assert!(project.for_printing && !project.for_machining);
        assert_eq!(project.gears[0].params.nozzle, 0.4);
        assert_eq!(project.gears[0].params.hole(), project.gears[0].params.bore + 0.2);
        harness.get_by_label_contains("Machined").click();
        harness.run_steps(3);
        let project = &harness.state().project;
        assert!(project.for_machining && !project.for_printing);
        assert_eq!(project.gears[0].params.nozzle, 0.0);
    }

    /// With machining off there is no ball to choose, so no asking.
    #[test]
    fn parts_not_machined_ask_for_no_ball() {
        for machined in [true, false] {
            let mut p = Project::empty();
            let first = p.add_root(PitchUnit::Module.starter(24), 0.0, 0.0);
            p.for_machining = machined;
            let mut harness = Harness::builder().with_size(Vec2::new(1400.0, 1500.0)).build_eframe(|cc| {
                let mut app = GearCraftApp::new(cc, None);
                app.replace_project(p, None);
                app.selected = Some(first);
                app
            });
            harness.run_steps(3);
            assert_eq!(harness.query_by_label_contains("No ball end mill").is_some(), machined);
        }
    }

    #[test]
    fn every_kind_of_part_takes_the_ball_end_mill_fix() {
        let u = PitchUnit::Module;
        let kinds = [
            ("spur", u.starter(24), u.starter(40), Link::Mesh),
            ("bevel", u.starter_bevel(20), u.starter_bevel(40), Link::Bevel),
            ("pulley", u.starter_pulley(20), u.starter_pulley(30), Link::Belt),
            ("hypoid", u.starter_hypoid(Role::Pinion), u.starter_hypoid(Role::Ring), Link::Hypoid),
        ];
        for (name, a, b, link) in kinds {
            let mut p = Project::empty();
            let first = p.add_root(a, 0.0, 0.0);
            p.add_child(first, b, link);
            p.ball_end_mill = 30.0;
            let mut harness = egui_kittest::Harness::builder().with_size(Vec2::new(1400.0, 1500.0)).build_eframe(|cc| {
                let mut app = GearCraftApp::new(cc, None);
                app.replace_project(p, None);
                app.switch_workspace(Workspace::of(&app.project.gears[0].params));
                app.selected = Some(first);
                app
            });
            harness.run_steps(3);
            harness.get_by_label_contains("ball end mill is too big");
            harness.get_by_label_contains("Cut with a").click();
            harness.run_steps(3);
            let ball = harness.state().project.ball_end_mill;
            assert!(ball > 0.1 && ball < 30.0, "{name}: Ø{ball}");
            assert!(harness.query_by_label_contains("too big").is_none(), "{name}: still too big at Ø{ball}");
            assert!(harness.query_by_label_contains("No ball end mill").is_none(), "{name}");
        }
    }

    /// Waits for the selected part's teeth and reach check, and gives
    /// the share out of reach.
    fn checked_reach(harness: &mut Harness<'_, GearCraftApp>) -> f64 {
        let start = Instant::now();
        loop {
            harness.step();
            let app = harness.state();
            let idx = app.selected.and_then(|id| app.project.index_of(id)).unwrap();
            if let Some(Some(c)) = app.reach_state(idx) {
                let share = c.reach.share();
                harness.run_steps(2);
                return share;
            }
            assert!(start.elapsed() < Duration::from_secs(120), "no reach check");
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    /// A spiral bevel ring's slots curve along its face, so a tool kept
    /// square to its axis can't follow them all the way down: their
    /// floors are raised until it can, and that is said. A spur gear's
    /// teeth are all reached as they are.
    #[test]
    fn teeth_out_of_reach_are_flagged() {
        let mut harness = Harness::builder().with_size(Vec2::new(1400.0, 1500.0)).build_eframe(|cc| {
            let mut app = GearCraftApp::new(cc, None);
            app.switch_workspace(Workspace::Bevel);
            app.add_hypoid_pair();
            let r = app.project.index_of(app.selected.unwrap()).unwrap();
            if let Placement::Hypoid { offset, .. } = &mut app.project.gears[r].placement {
                *offset = 0.0;
            }
            app.project.ball_end_mill = 2.0;
            app.project.normalize();
            app
        });
        let share = checked_reach(&mut harness);
        assert!(share < 0.02, "{share}");
        harness.get_by_label_contains("floors are raised");
        harness.get_by_label_contains("Slot floors raised");

        let mut p = Project::empty();
        let first = p.add_root(PitchUnit::Module.starter(24), 0.0, 0.0);
        p.ball_end_mill = 1.0;
        harness.state_mut().replace_project(p, None);
        harness.state_mut().switch_workspace(Workspace::Spur);
        harness.state_mut().selected = Some(first);
        assert_eq!(checked_reach(&mut harness), 0.0);
        assert!(harness.query_by_label_contains("can't reach").is_none());
    }

    /// Renders the window offscreen to GEARCRAFT_SHOT (a PNG path) so the
    /// layout can be looked at without opening a window. Skipped unless set.
    #[test]
    fn screenshot() {
        let Ok(out) = std::env::var("GEARCRAFT_SHOT") else { return };
        let file = std::env::var("GEARCRAFT_SHOT_FILE").ok().map(PathBuf::from);
        let dark = std::env::var("GEARCRAFT_SHOT_LIGHT").is_err();
        let mut harness = egui_kittest::Harness::builder()
            .with_size(Vec2::new(1400.0, if std::env::var("GEARCRAFT_SHOT_TALL").is_ok() { 1500.0 } else { 860.0 }))
            .with_theme(if dark { egui::Theme::Dark } else { egui::Theme::Light })
            .wgpu()
            .build_eframe(|cc| GearCraftApp::new(cc, file));
        for _ in 0..4 {
            harness.step();
        }
        if let Some(a) = std::env::var("GEARCRAFT_SHOT_ANGLE").ok().and_then(|a| a.parse::<f64>().ok()) {
            harness.state_mut().driver_angle = a.to_radians();
            harness.step();
        }
        if std::env::var("GEARCRAFT_SHOT_3D").is_ok() {
            harness.state_mut().three_d = true;
            harness.state_mut().fit_pending = true;
            harness.step();
            harness.step();
        }
        if std::env::var("GEARCRAFT_SHOT_PULLEYS").is_ok() {
            harness.state_mut().switch_workspace(Workspace::Pulleys);
            harness.step();
            harness.step();
        }
        if std::env::var("GEARCRAFT_SHOT_BEVEL").is_ok() {
            let app = harness.state_mut();
            app.switch_workspace(Workspace::Bevel);
            if app.first_shown().is_none() {
                app.add_bevel_pair();
            }
            app.fit_pending = true;
            harness.step();
            harness.step();
        }
        if std::env::var("GEARCRAFT_SHOT_HYPOID").is_ok() {
            let app = harness.state_mut();
            app.switch_workspace(Workspace::Bevel);
            if !app.project.gears.iter().any(|g| g.params.is_hypoid()) {
                app.add_hypoid_pair();
            }
            app.three_d = true;
            harness.step();
            let start = Instant::now();
            while harness.state().pair_pending() && start.elapsed() < Duration::from_secs(120) {
                std::thread::sleep(Duration::from_millis(100));
                harness.step();
            }
            harness.state_mut().fit_pending = true;
            harness.step();
            harness.step();
        }
        if std::env::var("GEARCRAFT_SHOT_WORMDRIVE").is_ok() {
            let app = harness.state_mut();
            app.switch_workspace(Workspace::Worm);
            if app.first_shown().is_none() {
                app.add_worm_drive();
            }
            // GEARCRAFT_SHOT_WORMDRIVE=worm selects the worm, =straight
            // makes the wheel a straight one.
            let how = std::env::var("GEARCRAFT_SHOT_WORMDRIVE").unwrap_or_default();
            if how == "worm" {
                app.selected = app.first_shown();
            }
            if how == "backdrive" {
                let wheel = app.project.gears.iter().position(|g| g.params.worm_drive == Some(worm::Role::Wheel));
                app.make_back_drivable(wheel.unwrap());
            }
            // =globoid makes it a globoid drive, =globoid_long one whose worm
            // is too long for its base circle, with the worm selected.
            if how.starts_with("globoid") {
                let wheel = app.project.gears.iter().position(|g| g.params.worm_drive == Some(worm::Role::Wheel));
                app.project.make_globoid(wheel.unwrap(), true);
            }
            if how == "globoid_long" {
                let worm = app.project.gears.iter().position(|g| g.params.worm_drive == Some(worm::Role::Worm)).unwrap();
                app.project.gears[worm].params.face_width = 30.0;
                app.selected = Some(app.project.gears[worm].id);
            }
            if how == "straight" {
                for g in &mut app.project.gears {
                    if let Placement::Worm { throated, .. } = &mut g.placement {
                        *throated = false;
                    }
                }
            }
            harness.step();
            let start = Instant::now();
            while harness.state().pair_pending() && start.elapsed() < Duration::from_secs(120) {
                std::thread::sleep(Duration::from_millis(100));
                harness.step();
            }
            harness.state_mut().fit_pending = true;
            harness.step();
            harness.step();
        }
        if std::env::var("GEARCRAFT_SHOT_WORM").is_ok() {
            let app = harness.state_mut();
            app.switch_workspace(Workspace::BevelWorm);
            if app.first_shown().is_none() {
                app.add_worm_pair();
            }
            // GEARCRAFT_SHOT_WORM=worm selects the worm rather than the wheel.
            if std::env::var("GEARCRAFT_SHOT_WORM").as_deref() == Ok("worm") {
                app.selected = app.first_shown();
            }
            harness.step();
            let start = Instant::now();
            while harness.state().pair_pending() && start.elapsed() < Duration::from_secs(120) {
                std::thread::sleep(Duration::from_millis(100));
                harness.step();
            }
            harness.state_mut().fit_pending = true;
            harness.step();
            harness.step();
        }
        // "yaw,pitch,zoom[,x,y,z]": degrees, a scale factor after fitting, and
        // optionally the point to look at.
        if let Ok(cam) = std::env::var("GEARCRAFT_SHOT_CAM") {
            let v: Vec<f64> = cam.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            let app = harness.state_mut();
            app.camera.yaw = v[0].to_radians();
            app.camera.pitch = v[1].to_radians();
            app.fit_pending = true;
            harness.step();
            let app = harness.state_mut();
            app.camera.scale *= v.get(2).copied().unwrap_or(1.0);
            if v.len() >= 6 {
                app.camera.target = [v[3], v[4], v[5]];
            }
            app.auto_fit = false;
            harness.step();
        }
        if let Ok(ws) = std::env::var("GEARCRAFT_SHOT_MENU") {
            harness.get_by_label(&ws).click();
            for _ in 0..4 {
                harness.step();
            }
        }
        harness.render().unwrap().save(out).unwrap();
    }
}

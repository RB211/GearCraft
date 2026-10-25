use crate::export::{self, place};
use crate::gear::{self, FillMesh, GearParams, Issue, Level, ToothStandard};
use crate::train::{Placement, PitchUnit, Project, STD_DPS, STD_MODULES, Solved};
use eframe::egui::{
    self, Align2, Color32, FontId, Key, KeyboardShortcut, Modifiers, Pos2, Rect, RichText, Sense, Shape, Stroke, Ui,
    Vec2,
};
use std::collections::HashMap;
use std::f64::consts::TAU;
use std::ops::RangeInclusive;
use std::path::PathBuf;
use std::time::{Duration, Instant};

const FILE_EXT: &str = "gearcraft";


struct Cached {
    params: GearParams,
    outline: Vec<[f64; 2]>,
    fill: FillMesh,
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
    /// Swinging a meshing gear around its parent.
    Swing(u64),
}

#[derive(Clone, Copy, PartialEq)]
enum Pending {
    New(PitchUnit),
    Open,
    Quit,
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
    /// Asking whether to convert the gears for a Metric/Inch switch.
    unit_switch: Option<PitchUnit>,
    title: String,
}

impl GearCraftApp {
    pub fn new(_cc: &eframe::CreationContext<'_>, file: Option<PathBuf>) -> Self {
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
            unit_switch: None,
            title: String::new(),
        };
        if let Some(f) = file {
            app.load(f);
        }
        app
    }

    fn flash(&mut self, msg: impl Into<String>) {
        self.status = Some((msg.into(), Instant::now()));
    }

    // ---- files -----------------------------------------------------------

    fn replace_project(&mut self, project: Project, path: Option<PathBuf>) {
        self.selected = project.gears.first().map(|g| g.id);
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

    fn open_dialog(&mut self) {
        if let Some(p) = rfd::FileDialog::new().add_filter("GearCraft project", &[FILE_EXT]).pick_file() {
            self.load(p);
        }
    }

    fn save(&mut self, save_as: bool) -> bool {
        let path = match (&self.path, save_as) {
            (Some(p), false) => p.clone(),
            _ => {
                let Some(mut p) = rfd::FileDialog::new()
                    .add_filter("GearCraft project", &[FILE_EXT])
                    .set_file_name(format!("train.{FILE_EXT}"))
                    .save_file()
                else {
                    return false;
                };
                if p.extension().is_none() {
                    p.set_extension(FILE_EXT);
                }
                p
            }
        };
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

    fn export(&mut self, whole_train: bool, dxf: bool) {
        let only = if whole_train { None } else { self.selected };
        let shapes = export::shapes(&self.project, self.driver_angle, only);
        if shapes.is_empty() {
            self.flash("Nothing to export");
            return;
        }
        let (ext, label) = if dxf { ("dxf", "DXF") } else { ("svg", "SVG") };
        let stem = if whole_train {
            self.path.as_ref().and_then(|p| p.file_stem()).map(|s| s.to_string_lossy().into_owned()).unwrap_or("train".into())
        } else {
            shapes[0].name.replace(' ', "_")
        };
        let Some(mut path) =
            rfd::FileDialog::new().add_filter(label, &[ext]).set_file_name(format!("{stem}.{ext}")).save_file()
        else {
            return;
        };
        if path.extension().is_none() {
            path.set_extension(ext);
        }
        match export::write(&path, &shapes, self.project.pitch_unit.is_inch()) {
            Ok(()) => self.flash(format!("Exported {}", path.display())),
            Err(e) => self.flash(format!("Could not write {}: {e}", path.display())),
        }
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

    fn add_gear(&mut self, on_axle: bool) {
        let Some(parent) = self.selected.and_then(|id| self.project.get(id)) else {
            self.add_train();
            return;
        };
        let pp = &parent.params;
        // A compound pinion is usually a fraction of its wheel; a meshing
        // gear starts as a copy so the pair is 1:1 until edited.
        let params = if on_axle {
            GearParams { teeth: (pp.teeth / 4).max(8), bore: pp.bore, ..pp.clone() }
        } else {
            pp.clone()
        };
        let id = self.project.add_child(parent.id, params, on_axle);
        self.selected = Some(id);
    }

    fn add_train(&mut self) {
        let solved = self.project.solve();
        let right = self
            .project
            .gears
            .iter()
            .zip(&solved)
            .map(|(g, s)| s.center[0] + g.params.tip_radius())
            .fold(f64::MIN, f64::max);
        let x = if right == f64::MIN { 0.0 } else { right + 40.0 };
        let id = self.project.add_root(self.project.pitch_unit.starter(24), x, 0.0);
        self.selected = Some(id);
    }

    fn delete_selected(&mut self) {
        if let Some(id) = self.selected {
            let i = self.project.index_of(id).unwrap_or(0);
            self.project.remove(id);
            self.selected =
                self.project.gears.get(i.saturating_sub(1)).or(self.project.gears.first()).map(|g| g.id);
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
        if self.selected.and_then(|id| self.project.get(id)).is_none() {
            self.selected = self.project.gears.first().map(|g| g.id);
        }
    }

    fn refresh_cache(&mut self) {
        let ids: Vec<u64> = self.project.gears.iter().map(|g| g.id).collect();
        self.cache.retain(|id, _| ids.contains(id));
        for g in &self.project.gears {
            if self.cache.get(&g.id).is_some_and(|c| c.params == g.params) {
                continue;
            }
            match gear::tooth_profile(&g.params) {
                Some(prof) => {
                    let outline = gear::outline(&g.params, &prof);
                    let fill = gear::fill_mesh(&g.params, &prof);
                    self.cache.insert(g.id, Cached { params: g.params.clone(), outline, fill });
                }
                None => {
                    self.cache.remove(&g.id);
                }
            }
        }
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
        for (g, s) in self.project.gears.iter().zip(solved) {
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
    fn hit(&self, solved: &[Solved], w: [f64; 2]) -> Option<u64> {
        self.project
            .gears
            .iter()
            .zip(solved)
            .filter(|(g, s)| (w[0] - s.center[0]).hypot(w[1] - s.center[1]) <= g.params.tip_radius())
            .min_by(|a, b| a.0.params.tip_radius().total_cmp(&b.0.params.tip_radius()))
            .map(|(g, _)| g.id)
    }

    fn canvas(&mut self, ui: &mut Ui) {
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
                        Placement::Mesh { .. } => self.drag = Some(Drag::Swing(id)),
                        Placement::Axle { .. } => {}
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
                    if let (Placement::Mesh { angle, .. }, Some(pi)) = (&mut self.project.gears[i].placement, pi) {
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
        self.refresh_cache();
        let collisions = self.project.collisions(&solved);
        self.draw_gears(&painter, rect, &visuals, &solved, &collisions);

        if self.project.gears.is_empty() {
            painter.text(
                rect.center(),
                Align2::CENTER_CENTER,
                "Empty project. Use \"New train\" to place a gear.",
                FontId::proportional(16.0),
                visuals.weak_text_color(),
            );
        }
        let hint = if rect.width() > 820.0 {
            "Scroll: zoom · Drag empty space: pan · Drag a gear: move it, or swing it round its \
             partner (Shift snaps 15°) · Double-click: fit"
        } else {
            "Scroll zoom · drag pan · drag gear to move/swing"
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
                painter.circle_stroke(center, (g.params.bore / 2.0 * self.view.scale) as f32, Stroke::new(1.0, edge));
            }
            // Arbor mark, plus a tick so rotation is visible on plain gears.
            let tick = (ra_px * 0.15).clamp(3.0, 10.0);
            painter.circle_filled(center, 2.0, edge);
            let dir = Vec2::new(rot.cos() as f32, -rot.sin() as f32);
            let r0 = (g.params.bore / 2.0 * self.view.scale) as f32 + 2.0;
            painter.line_segment([center + dir * r0, center + dir * (r0 + tick)], Stroke::new(1.5, edge));
        }

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
            for (g, s) in self.project.gears.iter().zip(solved) {
                let ra_px = (g.params.tip_radius() * self.view.scale) as f32;
                if ra_px < 40.0 {
                    continue;
                }
                let at = self.to_screen(rect, s.center) + Vec2::new(0.0, 14.0);
                let text = format!("{}  {}T", g.name, g.params.teeth);
                let font = FontId::proportional(12.0);
                let galley = painter.layout_no_wrap(text, font, visuals.text_color());
                let r = Align2::CENTER_TOP.anchor_size(at, galley.size()).expand(3.0);
                painter.rect_filled(r, 3.0, visuals.extreme_bg_color.gamma_multiply(0.8));
                painter.galley(r.min + Vec2::splat(3.0), galley, visuals.text_color());
            }
        }
    }

    // ---- panels ----------------------------------------------------------

    fn menu_bar(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();
        egui::MenuBar::new().ui(ui, |ui| {
            ui.menu_button("File", |ui| {
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
                    self.save(false);
                }
                if ui.add(egui::Button::new("Save As…").shortcut_text("Ctrl+Shift+S")).clicked() {
                    self.save(true);
                }
                ui.separator();
                ui.menu_button("Export", |ui| {
                    let has_sel = self.selected.is_some();
                    if ui.add_enabled(has_sel, egui::Button::new("Selected gear as DXF…")).clicked() {
                        self.export(false, true);
                    }
                    if ui.add_enabled(has_sel, egui::Button::new("Selected gear as SVG…")).clicked() {
                        self.export(false, false);
                    }
                    ui.separator();
                    if ui.button("Whole train as DXF…").clicked() {
                        self.export(true, true);
                    }
                    if ui.button("Whole train as SVG…").clicked() {
                        self.export(true, false);
                    }
                });
                ui.separator();
                if ui.add(egui::Button::new("Quit").shortcut_text("Ctrl+Q")).clicked() {
                    self.request(&ctx, Pending::Quit);
                }
            });
            ui.menu_button("Edit", |ui| {
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
                if ui.add_enabled(has_sel, egui::Button::new("Add meshing gear").shortcut_text("M")).clicked() {
                    self.add_gear(false);
                }
                if ui.add_enabled(has_sel, egui::Button::new("Add gear on same axle").shortcut_text("A")).clicked() {
                    self.add_gear(true);
                }
                if ui.button("New train").clicked() {
                    self.add_train();
                }
                ui.separator();
                if ui.add_enabled(has_sel, egui::Button::new("Delete gear").shortcut_text("Del")).clicked() {
                    self.delete_selected();
                }
            });
            ui.menu_button("View", |ui| {
                if ui.add(egui::Button::new("Fit to window").shortcut_text("F")).clicked() {
                    self.fit_pending = true;
                }
                ui.separator();
                ui.checkbox(&mut self.show.grid, "Grid");
                ui.checkbox(&mut self.show.pitch, "Pitch circle (selected)");
                ui.checkbox(&mut self.show.base, "Base circle (selected)");
                ui.checkbox(&mut self.show.labels, "Labels");
            });
        });
    }

    fn toolbar(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("Gears").strong());
            let current = self.project.pitch_unit;
            for (unit, label, hint) in [
                (PitchUnit::Module, "Metric", "Module pitch, dimensions in millimetres"),
                (PitchUnit::DiametralPitch, "Inch", "Diametral pitch, dimensions in inches"),
            ] {
                let text = RichText::new(label).strong();
                if ui.selectable_label(current == unit, text).on_hover_text(hint).clicked() && current != unit {
                    if self.project.gears.is_empty() {
                        self.project.pitch_unit = unit;
                    } else {
                        self.unit_switch = Some(unit);
                    }
                }
            }
            ui.separator();
            let label = if self.playing { "⏸ Pause" } else { "▶ Run" };
            if ui.button(label).on_hover_text("Space").clicked() {
                self.playing = !self.playing;
            }
            if ui.button("⏮").on_hover_text("Back to the start position").clicked() {
                self.driver_angle = 0.0;
            }
            ui.label("Driver");
            ui.add(egui::DragValue::new(&mut self.project.driver_rpm).speed(0.1).range(-600.0..=600.0).suffix(" rpm"));
            ui.separator();
            if ui.button("⚙ Mesh gear").on_hover_text("Add a gear meshing with the selected one (M)").clicked() {
                self.add_gear(false);
            }
            if ui.button("◎ Axle gear").on_hover_text("Add a gear on the selected gear's axle (A)").clicked() {
                self.add_gear(true);
            }
            if ui.button("➕ New train").on_hover_text("Add a free-standing driver gear").clicked() {
                self.add_train();
            }
            ui.separator();
            if ui.button("Fit").on_hover_text("F").clicked() {
                self.fit_pending = true;
            }
            ui.toggle_value(&mut self.show.pitch, "Pitch ○");
            ui.toggle_value(&mut self.show.base, "Base ○");
        });
    }

    fn tree(&mut self, ui: &mut Ui) {
        ui.heading("Train");
        ui.add_space(4.0);
        let solved = self.project.solve();
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            for (g, s) in self.project.gears.iter().zip(&solved) {
                let (icon, how) = match g.placement {
                    Placement::Root { .. } => ("⏺", "driver"),
                    Placement::Mesh { .. } => ("⚙", "meshes"),
                    Placement::Axle { .. } => ("◎", "same axle"),
                };
                ui.horizontal(|ui| {
                    ui.add_space(s.depth.min(12) as f32 * 14.0);
                    let text = RichText::new(format!("{icon}  {}", g.name));
                    let r = ui.selectable_label(self.selected == Some(g.id), text);
                    ui.label(RichText::new(format!("{}T", g.params.teeth)).weak());
                    let r = r.on_hover_text(format!("{how} · ratio {}", fmt_ratio(s.ratio)));
                    if r.clicked() {
                        self.selected = Some(g.id);
                    }
                });
            }
        });
    }

    fn properties(&mut self, ui: &mut Ui) {
        let Some(idx) = self.selected.and_then(|id| self.project.index_of(id)) else {
            ui.heading("Properties");
            ui.label(RichText::new("Select a gear on the canvas or in the train list.").weak());
            return;
        };
        let solved = self.project.solve();
        let collisions = self.project.collisions(&solved);
        let unit = self.project.pitch_unit;
        let rpm = self.project.driver_rpm;
        let earlier: Vec<(u64, String)> = self.project.gears[..idx].iter().map(|g| (g.id, g.name.clone())).collect();
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

        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            let g = &mut self.project.gears[idx];
            ui.horizontal(|ui| {
                ui.label("Name");
                ui.text_edit_singleline(&mut g.name);
            });
            ui.add_space(6.0);

            let locked = g.match_parent && matches!(g.placement, Placement::Mesh { .. });
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

                    if matches!(g.placement, Placement::Mesh { .. }) {
                        ui.label("");
                        ui.checkbox(&mut g.match_parent, "Same tooth system as partner").on_hover_text(
                            "Copy pitch, pressure angle and rack proportions from the meshing partner. \
                             Pitch and pressure angle must match for gears to mesh.",
                        );
                        ui.end_row();
                    }

                    ui.label("Profile shift x");
                    ui.add(egui::DragValue::new(&mut g.params.profile_shift).range(-1.0..=1.5).speed(0.005).max_decimals(3))
                        .on_hover_text("Moves the cutter out (+) or in (−) by x × module. Positive shift cures undercut on small pinions.");
                    ui.end_row();

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
                });
            });

            section(ui, "Blank", |ui| {
                egui::Grid::new("blank").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                    ui.label("Bore Ø");
                    len_drag(ui, &mut g.params.bore, unit, 0.0..=10000.0, 0.1);
                    ui.end_row();
                    ui.label("Face width");
                    len_drag(ui, &mut g.params.face_width, unit, 0.0..=10000.0, 0.1);
                    ui.end_row();
                });
            });

            section(ui, "Placement", |ui| {
                let kind = match g.placement {
                    Placement::Root { .. } => 0,
                    Placement::Mesh { .. } => 1,
                    Placement::Axle { .. } => 2,
                };
                let mut new_kind = kind;
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut new_kind, 0, "Fixed");
                    ui.add_enabled_ui(!earlier.is_empty(), |ui| {
                        ui.selectable_value(&mut new_kind, 1, "Meshes with");
                        ui.selectable_value(&mut new_kind, 2, "On axle of");
                    });
                });
                if new_kind != kind {
                    let parent = g.placement.parent().or(earlier.last().map(|e| e.0)).unwrap_or(0);
                    g.placement = match new_kind {
                        0 => Placement::Root { x: s.center[0], y: s.center[1], phase: 0.0 },
                        1 => Placement::Mesh { parent, angle: 0.0 },
                        _ => Placement::Axle { parent, phase: 0.0 },
                    };
                }
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
                    Placement::Axle { parent, phase } => {
                        ui.label("Shares axle with");
                        parent_combo(ui, parent, &earlier);
                        ui.end_row();
                        ui.label("Phase");
                        ui.add(egui::DragValue::new(phase).speed(0.5).range(-360.0..=360.0).suffix("°"));
                        ui.end_row();
                    }
                });
            });

            let p = g.params.clone();
            let fmt = |v: f64| len(v, unit);
            let mut issues = p.issues(fmt);
            section(ui, "Dimensions", |ui| {
                if !p.is_buildable() {
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
            for o in others {
                issues.push(Issue { level: Level::Error, text: format!("Overlaps {o} without meshing.") });
            }

            ui.add_space(6.0);
            if issues.is_empty() {
                ui.label(RichText::new("✔ No problems found").color(ui.visuals().weak_text_color()));
            }
            for is in issues {
                let (icon, color) = match is.level {
                    Level::Warning => ("⚠", ui.visuals().warn_fg_color),
                    Level::Error => ("⛔", ui.visuals().error_fg_color),
                };
                ui.label(RichText::new(format!("{icon} {}", is.text)).color(color));
            }
        });
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
            ui.label(format!("{} gears", self.project.gears.len()));
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
            self.save(true);
        } else if pressed(cmd(Key::S)) {
            self.save(false);
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
        if key(Key::M) {
            self.add_gear(false);
        }
        if key(Key::A) {
            self.add_gear(true);
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
                if self.save(false) {
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
        let name = self
            .path
            .as_ref()
            .and_then(|p| p.file_name())
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "Untitled".into());
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

        egui::Panel::top("menu").show(ui, |ui| self.menu_bar(ui));
        egui::Panel::top("toolbar").show(ui, |ui| {
            ui.add_space(2.0);
            self.toolbar(ui);
            ui.add_space(2.0);
        });
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

fn parent_combo(ui: &mut Ui, parent: &mut u64, choices: &[(u64, String)]) {
    let current = choices.iter().find(|c| c.0 == *parent).map(|c| c.1.as_str()).unwrap_or("?");
    egui::ComboBox::from_id_salt("parent").selected_text(current).show_ui(ui, |ui| {
        for (id, name) in choices {
            ui.selectable_value(parent, *id, name);
        }
    });
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

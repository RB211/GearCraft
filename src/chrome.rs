//! The window chrome, styled after SolidCraft: IBM Plex Sans, a dark
//! application bar with the brand, and a ribbon of large tool buttons
//! grouped under captions.

use eframe::egui::{
    self, Align2, Color32, CornerRadius, FontData, FontDefinitions, FontFamily, FontId, Pos2, Rect, Response, Sense,
    Shape, Stroke, StrokeKind, Ui, epaint::text::VariationCoords, pos2, vec2,
};
use std::sync::Arc;

const PLEX_SANS: &[u8] = include_bytes!("../assets/fonts/IBMPlexSans-Variable.ttf");

pub const APP_BAR_HEIGHT: f32 = 34.0;
pub const APP_BAR: Color32 = Color32::from_rgb(52, 56, 64);
pub const APP_BAR_TEXT: Color32 = Color32::from_rgb(230, 234, 242);
const APP_BAR_SEPARATOR: Color32 = Color32::from_rgb(96, 102, 114);
const APP_BAR_HOVER: Color32 = Color32::from_rgba_premultiplied(34, 35, 36, 34);
/// Ribbon height, so the workspace card lines up with it.
const CARD_HEIGHT: f32 = 86.0;
pub const ACCENT: Color32 = Color32::from_rgb(38, 120, 218);

/// Plex Sans at regular and semibold weights ahead of egui's own fonts,
/// which stay as fallbacks for the symbol glyphs used as icons.
pub fn install_fonts(ctx: &egui::Context) {
    let mut fonts = FontDefinitions::default();
    let fallbacks = fonts.families.get(&FontFamily::Proportional).cloned().unwrap_or_default();
    for (name, weight, family) in [("plex-sans", 400.0, FontFamily::Proportional), ("plex-semibold", 600.0, semibold_family())] {
        let mut data = FontData::from_static(PLEX_SANS);
        data.tweak.coords = VariationCoords::new([(b"wght", weight)]);
        fonts.font_data.insert(name.to_owned(), Arc::new(data));
        fonts.families.insert(family, std::iter::once(name.to_owned()).chain(fallbacks.iter().cloned()).collect());
    }
    ctx.set_fonts(fonts);
}

fn semibold_family() -> FontFamily {
    FontFamily::Name("semibold".into())
}

pub fn semibold(size: f32) -> FontId {
    FontId::new(size, semibold_family())
}

/// Light text and a translucent hover on the dark bar, in either theme.
pub fn app_bar_style(ui: &mut Ui) {
    let v = ui.visuals_mut();
    v.override_text_color = Some(APP_BAR_TEXT);
    for w in [&mut v.widgets.inactive, &mut v.widgets.noninteractive] {
        w.weak_bg_fill = Color32::TRANSPARENT;
        w.bg_fill = Color32::TRANSPARENT;
        w.fg_stroke.color = APP_BAR_TEXT;
    }
    for w in [&mut v.widgets.hovered, &mut v.widgets.active, &mut v.widgets.open] {
        w.weak_bg_fill = APP_BAR_HOVER;
        w.bg_fill = APP_BAR_HOVER;
        w.bg_stroke = Stroke::NONE;
        w.fg_stroke.color = APP_BAR_TEXT;
    }
}

pub fn app_bar_frame() -> egui::Frame {
    egui::Frame::new().fill(APP_BAR).inner_margin(egui::Margin::symmetric(10, 0))
}

/// The logo and product name at the left of the bar.
pub fn brand(ui: &mut Ui) {
    let galley = ui.painter().layout_no_wrap("GearCraft".into(), semibold(17.0), APP_BAR_TEXT);
    let (rect, _) = ui.allocate_exact_size(vec2(30.0 + galley.size().x, APP_BAR_HEIGHT), Sense::hover());
    gear_mark(ui.painter(), pos2(rect.left() + 11.0, rect.center().y), 11.0, ACCENT);
    ui.painter().galley(pos2(rect.left() + 30.0, rect.center().y - galley.size().y / 2.0), galley, APP_BAR_TEXT);
}

/// A small spur gear: the brand mark, also used as the workspace icon.
pub fn gear_mark(painter: &egui::Painter, c: Pos2, r: f32, color: Color32) {
    let teeth = 10;
    let pitch = std::f32::consts::TAU / teeth as f32;
    let at = |a: f32, rr: f32| c + vec2(a.cos(), a.sin()) * rr;
    for k in 0..teeth {
        let a = k as f32 * pitch;
        let (base, tip) = (pitch * 0.30, pitch * 0.20);
        let quad = vec![at(a - base, r * 0.7), at(a - tip, r), at(a + tip, r), at(a + base, r * 0.7)];
        painter.add(Shape::convex_polygon(quad, color, Stroke::NONE));
    }
    painter.circle_filled(c, r * 0.76, color);
    painter.circle_filled(c, r * 0.3, APP_BAR);
}

pub fn bar_separator(ui: &mut Ui) {
    let (rect, _) = ui.allocate_exact_size(vec2(17.0, APP_BAR_HEIGHT), Sense::hover());
    let x = rect.center().x;
    ui.painter().line_segment([pos2(x, rect.top() + 8.0), pos2(x, rect.bottom() - 8.0)], Stroke::new(1.0, APP_BAR_SEPARATOR));
}

/// A square icon button on the bar.
pub fn bar_icon(ui: &mut Ui, icon: &str, tip: &str, enabled: bool) -> bool {
    let (rect, resp) = ui.allocate_exact_size(vec2(30.0, 30.0), if enabled { Sense::click() } else { Sense::hover() });
    if enabled && resp.hovered() {
        ui.painter().rect_filled(rect.shrink(2.0), 4.0, APP_BAR_HOVER);
    }
    let color = if enabled { APP_BAR_TEXT } else { APP_BAR_TEXT.gamma_multiply(0.35) };
    ui.painter().text(rect.center(), Align2::CENTER_CENTER, icon, FontId::proportional(17.0), color);
    resp.on_hover_text(tip).clicked() && enabled
}

/// The open document, drawn like a tab.
pub fn doc_tab(ui: &mut Ui, name: &str, dirty: bool) {
    let text = if dirty { format!("{name} •") } else { name.to_owned() };
    let galley = ui.painter().layout_no_wrap(text, semibold(13.5), APP_BAR_TEXT);
    let (rect, _) = ui.allocate_exact_size(vec2(galley.size().x + 24.0, 26.0), Sense::hover());
    ui.painter().rect_filled(rect, 5.0, Color32::from_rgb(70, 75, 85));
    ui.painter().galley(rect.left_center() + vec2(12.0, -galley.size().y / 2.0), galley, APP_BAR_TEXT);
}

// ---- ribbon -------------------------------------------------------------

pub fn ribbon_frame(ui: &Ui) -> egui::Frame {
    let v = ui.visuals();
    egui::Frame::new()
        .fill(v.panel_fill)
        .stroke(Stroke::new(1.0, v.widgets.noninteractive.bg_stroke.color))
        .corner_radius(CornerRadius::same(6))
        .inner_margin(egui::Margin::symmetric(6, 4))
}

/// Tools above, a caption below, a rule after.
pub fn group(ui: &mut Ui, caption: &str, tools: impl FnOnce(&mut Ui)) {
    ui.vertical(|ui| {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 2.0;
            tools(ui)
        });
        let w = ui.min_rect().width();
        ui.allocate_ui(vec2(w, 16.0), |ui| {
            ui.centered_and_justified(|ui| {
                ui.label(egui::RichText::new(caption).size(12.0).color(ui.visuals().weak_text_color()));
            });
        });
    });
    let (rect, _) = ui.allocate_exact_size(vec2(9.0, 76.0), Sense::hover());
    let x = rect.center().x;
    let color = ui.visuals().widgets.noninteractive.bg_stroke.color;
    ui.painter().line_segment([pos2(x, rect.top() + 4.0), pos2(x, rect.bottom() - 4.0)], Stroke::new(1.0, color));
}

/// What sits in a big button's icon slot.
pub enum Icon<'a> {
    Glyph(&'a str),
    /// Short text such as "mm", drawn bold.
    Text(&'a str),
    Gear,
    /// Two pulleys and their belt.
    Pulley,
    /// A mitre pair, side on.
    Bevel,
}

/// A bevel gear in perspective: a cone frustum with its teeth running
/// down the slanted face from the small front face to the large back.
fn bevel_mark(painter: &egui::Painter, c: Pos2, color: Color32) {
    let (top, rt) = (c + vec2(0.0, -5.0), vec2(5.0, 2.0));
    let (bottom, rb) = (c + vec2(0.0, 5.0), vec2(11.5, 4.5));
    let on = |o: Pos2, r: egui::Vec2, a: f32| o + vec2(r.x * a.cos(), r.y * a.sin());
    let side = color.gamma_multiply(0.5);
    // Body: the back half of the front face and the front half of the back.
    let mut hull: Vec<Pos2> = (0..=16).map(|k| on(top, rt, std::f32::consts::PI * (1.0 + k as f32 / 16.0))).collect();
    hull.extend((0..=16).map(|k| on(bottom, rb, std::f32::consts::PI * k as f32 / 16.0)));
    painter.add(Shape::convex_polygon(hull, side, Stroke::NONE));
    let teeth = 14;
    let pitch = std::f32::consts::TAU / teeth as f32;
    for k in 0..teeth {
        let a = k as f32 * pitch + 0.2;
        if a.sin() < -0.05 {
            continue;
        }
        let w = pitch * 0.27;
        let quad = vec![on(top, rt, a - w), on(top, rt, a + w), on(bottom, rb, a + w), on(bottom, rb, a - w)];
        painter.add(Shape::convex_polygon(quad, color, Stroke::NONE));
    }
    let face: Vec<Pos2> = (0..24).map(|k| on(top, rt, std::f32::consts::TAU * k as f32 / 24.0)).collect();
    painter.add(Shape::convex_polygon(face, color, Stroke::NONE));
}

/// A belt drive: small and large pulley joined by the belt's straight runs.
fn pulley_mark(painter: &egui::Painter, c: Pos2, color: Color32) {
    let (c1, r1) = (c + vec2(-7.0, 2.0), 4.5);
    let (c2, r2) = (c + vec2(6.0, -2.0), 7.0);
    let stroke = Stroke::new(2.0, color);
    painter.circle_stroke(c1, r1, stroke);
    painter.circle_stroke(c2, r2, stroke);
    painter.circle_filled(c1, 1.5, color);
    painter.circle_filled(c2, 2.0, color);
    let d = c2 - c1;
    let theta = d.y.atan2(d.x);
    let alpha = ((r1 - r2) / d.length()).acos();
    for s in [1.0f32, -1.0] {
        let a = theta + s * alpha;
        let u = vec2(a.cos(), a.sin());
        painter.line_segment([c1 + u * r1, c2 + u * r2], stroke);
    }
}

fn paint_icon(painter: &egui::Painter, icon: Icon, at: Pos2, color: Color32) {
    match icon {
        Icon::Glyph(g) => {
            painter.text(at, Align2::CENTER_CENTER, g, FontId::proportional(24.0), color);
        }
        Icon::Text(t) => {
            painter.text(at, Align2::CENTER_CENTER, t, semibold(17.0), color);
        }
        Icon::Gear => gear_mark(painter, at, 11.0, color),
        Icon::Pulley => pulley_mark(painter, at, color),
        Icon::Bevel => bevel_mark(painter, at, color),
    }
}

/// A ribbon tool: icon above its label.
pub fn big_button(ui: &mut Ui, icon: Icon, label: &str, selected: bool, enabled: bool) -> Response {
    let font = FontId::proportional(13.0);
    let text = ui.visuals().text_color();
    let galley = ui.painter().layout_no_wrap(label.to_owned(), font, text);
    let size = vec2((galley.size().x + 16.0).max(58.0), 58.0);
    let (rect, resp) = ui.allocate_exact_size(size, if enabled { Sense::click() } else { Sense::hover() });
    let v = ui.visuals();
    if selected {
        ui.painter().rect_filled(rect, 5.0, ACCENT.gamma_multiply(0.22));
    } else if enabled && resp.hovered() {
        ui.painter().rect_filled(rect, 5.0, v.widgets.hovered.weak_bg_fill);
    }
    let dim = |c: Color32| if enabled { c } else { c.gamma_multiply(0.35) };
    let icon_at = pos2(rect.center().x, rect.top() + 19.0);
    paint_icon(ui.painter(), icon, icon_at, dim(ACCENT));
    let at = pos2(rect.center().x - galley.size().x / 2.0, rect.bottom() - galley.size().y - 4.0);
    ui.painter().galley(at, galley, dim(text));
    resp
}

/// The workspace card at the left of the ribbon: icon, workspace name and
/// a "Workspace ⏷" caption; clicking it opens the list.
pub fn workspace_card(ui: &mut Ui, icon: Icon, name: &str) -> Response {
    let size = vec2(118.0, CARD_HEIGHT);
    let (rect, resp) = ui.allocate_exact_size(size, Sense::click());
    resp.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, name));
    let v = ui.visuals();
    let border = v.widgets.noninteractive.bg_stroke.color;
    let fill = if resp.hovered() { v.widgets.hovered.weak_bg_fill } else { v.panel_fill };
    ui.painter().rect(rect, 6.0, fill, Stroke::new(1.0, border), StrokeKind::Inside);
    // SolidCraft's accent strip along the top of the active workspace.
    let strip = Rect::from_min_size(rect.left_top() + vec2(14.0, 0.0), vec2(rect.width() - 28.0, 3.0));
    ui.painter().rect_filled(strip, 1.5, ACCENT);
    let icon_at = pos2(rect.center().x, rect.top() + 25.0);
    paint_icon(ui.painter(), icon, icon_at, ACCENT);
    let text = v.text_color();
    ui.painter().text(pos2(rect.center().x, rect.top() + 52.0), Align2::CENTER_CENTER, name, semibold(14.0), text);
    ui.painter().text(
        pos2(rect.center().x, rect.top() + 70.0),
        Align2::CENTER_CENTER,
        "Workspace ⏷",
        FontId::proportional(12.0),
        v.weak_text_color(),
    );
    resp
}

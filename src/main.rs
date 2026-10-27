//! GearCraft: a gear and gear-train designer.
//!
//! One window, no modal parameter dialogs: the train tree on the left, a
//! live canvas in the middle, and the selected gear's parameters and
//! derived values on the right, all updating as you type.

mod app;
mod bevel;
mod chrome;
mod export;
mod gear;
mod hypoid;
mod mesh;
mod pulley;
mod reach;
mod step;
mod train;
mod view3d;
mod worm;

fn main() -> eframe::Result {
    if std::env::args().any(|a| a == "--version" || a == "-V") {
        println!("{} {}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));
        return Ok(());
    }
    let mut file = None;
    let mut export_to = None;
    let mut part = None;
    let mut list = false;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--export" => export_to = args.next().map(std::path::PathBuf::from),
            "--part" => part = Some(args.next().unwrap_or_else(|| fail("--part needs a part id or name"))),
            "--list" => list = true,
            _ => file = Some(std::path::PathBuf::from(a)),
        }
    }
    if list {
        for g in &load_project(file.as_deref()).gears {
            println!("{}\t{}", g.id, g.name);
        }
        return Ok(());
    }
    if let Some(out) = export_to {
        return headless_export(file, &out, part.as_deref());
    }
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_title("GearCraft")
            .with_app_id("gearcraft")
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([900.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native("GearCraft", options, Box::new(move |cc| Ok(Box::new(app::GearCraftApp::new(cc, file)))))
}

fn load_project(file: Option<&std::path::Path>) -> train::Project {
    let mut project = match file {
        Some(f) => {
            let text = std::fs::read_to_string(f).unwrap_or_else(|e| fail(&format!("{}: {e}", f.display())));
            serde_json::from_str(&text).unwrap_or_else(|e| fail(&format!("{}: {e}", f.display())))
        }
        None => train::Project::default(),
    };
    project.normalize();
    project
}

/// `gearcraft [project] --export out.dxf|svg|step|stl|3mf|obj [--part ID|NAME]`: write the
/// train without opening a window, or just one part, placed as File → Export places a
/// selected part (axis on Z, tooth 0 on +X, at its export origin).
/// `gearcraft project --list`: the parts, one `id<TAB>name` per line.
fn headless_export(file: Option<std::path::PathBuf>, out: &std::path::Path, part: Option<&str>) -> eframe::Result {
    let project = load_project(file.as_deref());
    let only = part.map(|p| {
        project
            .gears
            .iter()
            .find(|g| g.id.to_string() == p || g.name == p)
            .map(|g| g.id)
            .unwrap_or_else(|| fail(&format!("no part {p:?} in the project")))
    });
    let ext = out.extension().map(|e| e.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
    if matches!(ext.as_str(), "stl" | "3mf" | "obj") {
        let parts = mesh::parts(&project, 0.0, only, |_| true, |_| None);
        if parts.is_empty() {
            fail("nothing to export");
        }
        mesh::write(out, &parts).unwrap_or_else(|e| fail(&format!("{}: {e}", out.display())));
        return Ok(());
    }
    let shapes = export::shapes(&project, 0.0, only, |_| true);
    let bevels = export::bevels(&project, 0.0, only, |_| true);
    let hypoids = export::hypoids(&project, 0.0, only, |_| true, |_| None);
    if shapes.is_empty() && bevels.is_empty() && hypoids.is_empty() {
        fail("nothing to export");
    }
    export::write(out, &shapes, &bevels, &hypoids, project.pitch_unit.is_inch()).unwrap_or_else(|e| fail(&format!("{}: {e}", out.display())));
    Ok(())
}

fn fail(msg: &str) -> ! {
    eprintln!("gearcraft: {msg}");
    std::process::exit(1)
}

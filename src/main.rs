//! GearCraft: a gear and gear-train designer.
//!
//! One window, no modal parameter dialogs: the train tree on the left, a
//! live canvas in the middle, and the selected gear's parameters and
//! derived values on the right, all updating as you type.

mod app;
mod export;
mod gear;
mod train;

fn main() -> eframe::Result {
    if std::env::args().any(|a| a == "--version" || a == "-V") {
        println!("{} {}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));
        return Ok(());
    }
    let mut file = None;
    let mut export_to = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--export" => export_to = args.next().map(std::path::PathBuf::from),
            _ => file = Some(std::path::PathBuf::from(a)),
        }
    }
    if let Some(out) = export_to {
        return headless_export(file, &out);
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

/// `gearcraft [project] --export out.dxf|out.svg`: write the train without
/// opening a window.
fn headless_export(file: Option<std::path::PathBuf>, out: &std::path::Path) -> eframe::Result {
    let mut project = match &file {
        Some(f) => {
            let text = std::fs::read_to_string(f).unwrap_or_else(|e| fail(&format!("{}: {e}", f.display())));
            serde_json::from_str(&text).unwrap_or_else(|e| fail(&format!("{}: {e}", f.display())))
        }
        None => train::Project::default(),
    };
    project.normalize();
    let shapes = export::shapes(&project, 0.0, None);
    export::write(out, &shapes, project.pitch_unit.is_inch()).unwrap_or_else(|e| fail(&format!("{}: {e}", out.display())));
    Ok(())
}

fn fail(msg: &str) -> ! {
    eprintln!("gearcraft: {msg}");
    std::process::exit(1)
}

// #![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

pub mod cam_modeller;
pub mod gui;

use gui::app::CamApp;

fn main() -> Result<(), eframe::Error> {

    env_logger::init();
    let mut options = eframe::NativeOptions {
        follow_system_theme: false,
        default_theme: eframe::Theme::Dark,
        // vsync: false,
        ..Default::default()
    };
    let viewport = options.clone().viewport.with_min_inner_size([1200.0, 1000.0]);
    options.viewport = viewport;

    eframe::run_native(
        "Camulate v0.1.0 -- ALPHA",
        options,
        Box::new(|_cc| Box::new(CamApp::default())),
    )
}

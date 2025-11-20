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
    let icon = embedded_icon_data();
    let viewport = options.clone().viewport
        .with_min_inner_size([1200.0, 1000.0])
        .with_icon(icon);
    options.viewport = viewport;

    eframe::run_native(
        "Camulate v0.1.0 -- ALPHA",
        options,
        Box::new(|_cc| Box::new(CamApp::default())),
    )
}



/// Reads in the icon from the provided path and embeds it into the binary
/// The icon must be in the format 256 x 256 pixels
fn embedded_icon_data() -> eframe::egui::viewport::IconData {

    let bytes = include_bytes!(r"C:\Users\Dell\Desktop\dev\camulate\assetts\icon_camulate.bmp")
        .into_iter()
        .skip(54) // skips the .bmp header
        .collect::<Vec<&u8>>();

    let rgba = bytes.chunks_exact(3)
        .map(|chunk| {
            let mut alpha = 255;
            if chunk.iter().all(|x| **x == 255) { alpha = 0;}
            [*chunk[0], *chunk[1], *chunk[2], alpha]
        })
        .collect::<Vec<[u8; 4]>>();

    let rgba = rgba
        .into_iter()
        .flatten()
        .collect::<Vec<u8>>();

    let icon = eframe::egui::viewport::IconData {
        rgba: rgba,
        width: 256,
        height: 256,
    };

    icon
}

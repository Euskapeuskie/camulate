use crate::cam_modeller::cam_system::{CamSystem, CamType};
use super::ui_cam_system::PlotKind;

use eframe::egui::{self, Rect};
use serde::{Serialize, Deserialize};
use std::{error::Error, io::Write};
use std::fs::File;


const plot_types: [PlotKind; 13] = [
    PlotKind::Position, PlotKind::Velocity, PlotKind::Acceleration,
    PlotKind::ContactAngle, PlotKind::CurvatureHertz, PlotKind::CurvatureDisc,
    PlotKind::ForceY, PlotKind::NormalForce, PlotKind::HertzPressure, PlotKind::Torque,
    PlotKind::BearingForcesAxial, PlotKind::BearingForcesRadial, PlotKind::RPMFollower];


#[derive(Serialize, Deserialize, Debug)]
pub struct CamApp {
    cam_systems: Vec<CamSystem>,
    system_names: Vec<String>,
    selected_plots: Vec<Vec<Vec<PlotKind>>>,
    open_system_id: usize,

    name_edit: bool,

    plot_fullscreen: bool,
    plot_fullscreen_index: usize,
    animation_fullscreen: bool,

    save_path: Option<String>,
    export_accuracy: f64,
}


impl Default for CamApp {
    fn default() -> Self {
        Self {
            cam_systems: vec![CamSystem::default(), CamSystem::default()],
            system_names: vec![format!("Default"), format!("Summary")],
            selected_plots: vec![vec![vec![PlotKind::Position]]],
            open_system_id: 0,
            name_edit: false,
            plot_fullscreen: false,
            plot_fullscreen_index: 0,
            animation_fullscreen: false,
            save_path: None,
            export_accuracy: 0.1,
        }
    }
}

impl CamApp {

    /// UI for showing the stuff of a CAM system, e.g.
    /// * Modifying the cam system parameters (spring rate, follower radius, omega, ...)
    /// * Adding/modifying sections of the cam
    /// * Adding/modifying process forces
    /// * Visualisation of the animated cam
    /// * Plotting of different metrics
    fn ui_cam_system(&mut self, ctx: &eframe::egui::Context, ui: &mut egui::Ui, width: f32, height: f32) -> () {
        // --- CAM SYSTEM LAYER ---
        let system = &mut self.cam_systems[self.open_system_id];
        let name = &mut self.system_names[self.open_system_id];
        let plots = &mut self.selected_plots[self.open_system_id];

        let remaining_height = ui.available_height();

        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                let height_cam_mod = system.ui_cam_modification(ui, 700.0, height).rect.height();
                let height_res = system.ui_overview_results(ui, 700.0, height).rect.height();
                if system.plot_animation(ui, 700.0, remaining_height-height_cam_mod-height_res).middle_clicked() {
                    self.animation_fullscreen = !self.animation_fullscreen;
                };
            });

            let width_plot = ui.available_width();
            let mut removes = vec![];
            ui.vertical_centered(|ui| {
                for (i, multiplot) in plots.iter_mut().enumerate() {
                    let mut removes_multiplot = vec![];

                    ui.horizontal(|ui| {
                        ui.add_sized([65.0, 20.0], egui::Label::new("Plot type:"));
                        for (j, plot) in multiplot.iter_mut().enumerate() {

                            egui::ComboBox::from_id_source(format!("{plot}-{i}-{j}"))
                            .width(width*0.09)
                            .selected_text(format!("{plot}"))
                            .show_ui(ui, |ui| {
                                for plot_type in plot_types {
                                    ui.selectable_value(plot, plot_type, format!("{plot_type}"));
                                };
                            });
                            if ui.button("x").on_hover_text("Remove graph").clicked() {
                                removes_multiplot.push(j);
                            }
                        }
                        if ui.button("+").on_hover_text("Add graph to this plot").clicked() {
                            multiplot.push(PlotKind::Position);
                        }
                        if ui.button("Close").on_hover_text("Remove plot").clicked() {
                            removes.push(i);
                        }
                    });
                    for r in removes_multiplot {
                        multiplot.remove(r);
                    }
                    if system.multi_plot_graph(multiplot.to_vec(), ui, width_plot, remaining_height*0.33-20.0, true).middle_clicked() {
                        self.plot_fullscreen = !self.plot_fullscreen;
                        self.plot_fullscreen_index = i;
                    };
                }
                for r in removes {
                    plots.remove(r);
                }
                if plots.len() < 4 {
                    if ui.button("+").on_hover_text("Add another plot").clicked() {
                        plots.push(vec![PlotKind::Position]);
                    }
                }
            });
        }); 

        if self.plot_fullscreen {
            egui::Window::new("Fullscreen plot")
                .open(&mut self.plot_fullscreen)
                .default_size([width*0.8, height*0.8])
                .resizable(true)
                .vscroll(false)
                .show(ctx, |ui| {
                    if let Some(plot) = plots.get(self.plot_fullscreen_index) {
                        system.multi_plot_graph(plot.to_vec(), ui, ui.available_width(), ui.available_height(), true);
                    } else { }
                });
        }
        if self.animation_fullscreen {
            egui::Window::new("Fullscreen animation")
                .open(&mut self.animation_fullscreen)
                .default_size([width*0.8, height*0.8])
                .resizable(true)
                .vscroll(false)
                .show(ctx, |ui| {
                    system.plot_animation(ui, ui.available_width(), ui.available_height());
                });
        }            
        system.update_index();
    }


    /// Summary UI
    fn ui_summary(&mut self, ui: &mut egui::Ui, width: f32, height: f32) -> () {

        let n_cam_systems = self.cam_systems.len()-1;

        if let Some((summary, systems)) = self.cam_systems.split_last_mut() {

            let n_elements = summary.follower.phis.len();
            summary.ideal_torque = vec![0.0; n_elements];
            summary.real_torque = vec![0.0; n_elements];
            summary.ideal_bearing_forces_axial = vec![0.0; n_elements];
            summary.ideal_bearing_forces_radial = vec![0.0; n_elements];

            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    for (i, sys) in systems.iter().enumerate() {
                        summary.ideal_torque.iter_mut().zip(&sys.ideal_torque).for_each(|(a, b)| *a += b);
                        summary.real_torque.iter_mut().zip(&sys.real_torque).for_each(|(a, b)| *a += b);
                        summary.ideal_bearing_forces_axial.iter_mut().zip(&sys.ideal_bearing_forces_axial).for_each(|(a, b)| *a += b);
                        summary.ideal_bearing_forces_radial.iter_mut().zip(&sys.ideal_bearing_forces_radial).for_each(|(a, b)| *a += b);
                        ui.label(format!("{}", self.system_names[i]));
                        sys.plot_graph(PlotKind::Position, ui, width*0.45, height/n_cam_systems as f32 - 20.0, false);
                    }
                });
                ui.vertical(|ui| {
                    // ui.label("");
                    summary.plot_graph(PlotKind::Torque, ui, width*0.45, height*0.333, false);
                    // ui.label("");
                    summary.plot_graph(PlotKind::BearingForcesAxial, ui, width*0.45, height*0.333, false);
                    // ui.label("");
                    summary.plot_graph(PlotKind::BearingForcesRadial, ui, width*0.45, height*0.333, false);
                });
            });   
        }
    }

    fn save_cam_system(&self) -> Result<(), Box<dyn Error>> {
        let path = self.save_path.clone().unwrap();
        let mut f = File::create(format!("{}", path))?;
        let serialized: String = serde_json::to_string(self)?;
        write!(f, "{serialized}")?;
        Ok(())
    }

    fn open_cam_system(&mut self, path: String) -> Result<CamApp, Box<dyn Error>> {
        let input = std::fs::read_to_string(path)?;
        let t: CamApp = serde_json::from_str(&input)?;
        Ok(t)
    }

    fn export_pts(&self, export_path: String) -> Result<(), Box<dyn Error>> {
        // .pts-Datei für Zylindernutkurven 3 Spalten: [Radius, Drehwinkel, Hub] als Radius/Hub die Mittelpunktbahn des Followers
        // .pts-Datei für Planare Scheibe 3 Spalten: [Radius, Drehwinkel, z-Achse immer 0]
        let (base_path, extension) = export_path.split_once('.').ok_or_else(|| String::from("Invalid save path"))?;
        
        'sysloop: for (i, system) in self.cam_systems.iter().enumerate() {
            if self.system_names[i] == String::from("Summary") {
                continue 'sysloop;

            }
            let path_subsystem = format!("{}_{}.{}", base_path, self.system_names[i], extension);
            let mut f = csv::WriterBuilder::new().delimiter(b';').from_path(&export_path)?;

            let (rs, phis, zs) = match &system.cam_type {
                CamType::PlanarDisc => (system.follower.radius.clone(), system.follower.phis.clone(), vec![0.0; system.follower.phis.len()]),
                CamType::PlanarGroove(_) => (system.follower.radius.clone(), system.follower.phis.clone(), vec![0.0; system.follower.phis.len()]),
                CamType::CylindricBead(cylinder) => (vec![cylinder.base_radius; system.follower.phis.len()], system.follower.phis.clone(), system.follower.radius.clone()),
                CamType::CylindricGroove(cylinder) => (vec![cylinder.base_radius; system.follower.phis.len()], system.follower.phis.clone(), system.follower.radius.clone()),
            };

            let chunk_size = (self.export_accuracy/system.follower.accuracy) as usize;
            for ((r, phi), z) in rs.chunks(chunk_size).zip(phis.chunks(chunk_size)).zip(zs.chunks(chunk_size)) {
                f.write_record(&[format!("{:.3}\t{:.3}\t{:.3}", r[0]*1000.0, phi[0], z[0]*1000.0)])?
            }
        }
        Ok(())
    }

    fn export_follower_pos_csv(&self, export_path: String) -> Result<(), Box<dyn Error>> {
        let mut f = csv::WriterBuilder::new().delimiter(b';').from_path(export_path)?;
        f.write_field("Phi [°]")?;
        for name in self.system_names.iter() {
            if *name != String::from("Summary") {
                f.write_field(format!("{} [mm]", name))?;
            }
        }
        f.write_record(None::<&[u8]>)?; // Empty value as carriage return for new line

        let chunk_size = (self.export_accuracy/self.cam_systems[0].follower.accuracy) as usize;
        for (i, phis) in self.cam_systems[0].follower.phis.chunks(chunk_size).enumerate() {
            'sysloop: for (j, sys) in self.cam_systems.iter().enumerate() {
                if self.system_names[j] == String::from("Summary") {
                    continue 'sysloop;
                } else {
                    if j == 0 {
                        f.write_field(format!("{:.3}", phis[0]))?;
                    }
                    f.write_field(format!("{:.3}", sys.ideal_stroke.chunks(chunk_size).nth(i).unwrap()[0]*1000.0))?;
                }
            }
            f.write_record(None::<&[u8]>)?; // Empty value as carriage return for new line
        }
        Ok(())
    }


    fn export_torque_summary_csv(&self, export_path: String) -> Result<(), Box<dyn Error>> {
        let mut f = csv::WriterBuilder::new().delimiter(b';').from_path(export_path)?;
        f.write_record(&["Phi [°]", "Torque [Nm]"])?;
        for (i, sys) in self.cam_systems.iter().enumerate() {
            if self.system_names[i] == String::from("Summary") {
                for (j, phi) in sys.follower.phis.iter().enumerate() {
                    f.write_record(&[format!("{:.3}", phi), format!("{:.3}", sys.ideal_torque[j])])?;
                }
            }
        }

        Ok(())
    }
}

impl eframe::App for CamApp {

    fn update(&mut self, ctx: &eframe::egui::Context, frame: &mut eframe::Frame) {

        let ui_total_size = ctx.available_rect().size();
        let (ui_width, ui_height) = (ui_total_size.x, ui_total_size.y);
        
    	egui::CentralPanel::default().show(ctx, |ui| {
            
            // ----- MENU BAR -----
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File\t\t\t\t", |ui| {
                    if ui.button("New").clicked() {
                        *self = CamApp::default();
                    };

                    if ui.button("Open").on_hover_text("Open a saved .json file of a cam system").clicked() {
                        if let Some(open_path) = tinyfiledialogs::open_file_dialog("Open", "camulate.json", Some((&["*.json"], "JSON"))) {
                            match self.open_cam_system(open_path) {
                                Ok(t) => *self = t,
                                Err(e) => { println!("Unable to open system") },
                            };
                        }
                    }

                    if ui.button("Save").on_hover_text("Saves the cam system if a save path is known").clicked() {
                        if let Some(ref path) = self.save_path {
                            match self.save_cam_system() {
                                Ok(()) => (),
                                Err(e) => { println!("Unable to save system"); },
                            }
                        }
                        else {
                            if let Some(path) = tinyfiledialogs::save_file_dialog_with_filter("Save", "camulate.json", &["*.json"], "JSON") {
                                self.save_path = Some(path);
                                match self.save_cam_system() {
                                    Ok(()) => (),
                                    Err(e) => { println!("Unable to save system"); },
                                }
                            }
                        }
                    }

                    if ui.button("Save as").on_hover_text("Select the save path for this cam system").clicked() {
                        if let Some(path) = tinyfiledialogs::save_file_dialog_with_filter("Save", "camulate.json", &["*.json"], "JSON") {
                            self.save_path = Some(path);
                            match self.save_cam_system() {
                                Ok(()) => (),
                                Err(e) => { println!("Unable to save system"); },
                            }
                        }
                    }
                });
                ui.menu_button("Export\t\t\t\t", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Export accuracy:");
                        ui.add(egui::Slider::new(&mut self.export_accuracy, 0.01..=1.0).suffix(" °"))
                            .on_hover_text("Specify the accuracy in [°] in which the export\nof the points shall be performed");
                    });
                    
                    if ui.button("Export followers PTS").clicked() {
                        let export_path = tinyfiledialogs::save_file_dialog_with_filter("Save", "camulate.pts", &["*.pts"], "PTS");
                        if let Some(path) = export_path {
                            self.export_pts(path);
                        }
                    };

                    if ui.button("Export followers CSV").clicked() {
                        let export_path = tinyfiledialogs::save_file_dialog_with_filter("Save", "followers.csv", &["*.csv"], "CSV");
                        if let Some(path) = export_path {
                            self.export_follower_pos_csv(path);
                        }
                    }

                    if ui.button("Export torque summary CSV").clicked() {
                        let export_path = tinyfiledialogs::save_file_dialog_with_filter("Save", "torque_summary.csv", &["*.csv"], "CSV");
                        if let Some(path) = export_path {
                            self.export_torque_summary_csv(path);
                        }
                    }
                });

                // Switch light and dark mode button
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if let Some(v) = ui.visuals().clone().light_dark_small_toggle_button(ui) {
                        ctx.set_visuals(v);
                    };
                });

            });

            // --- TOP LAYER TO MANAGE DIFFERENT CAM SYSTEMS ---
            ui.horizontal( |ui| {
                let mut removes: Vec<usize> = vec![];
                for i in 0..self.cam_systems.len() {
                    ui.group(|ui| {
                        let label = ui.selectable_value(&mut self.open_system_id, i, &self.system_names[i]);
                        if label.double_clicked() & (self.system_names[i] != "Summary".to_string()) {
                            self.name_edit = !self.name_edit;
                        }
                        // Show a cross to remove the label if it is the selected one
                        if i == self.open_system_id {
                            let top_right = label.rect.right_top();
                            let mut bottom_left = label.rect.right_bottom();
                            bottom_left.x -= 10.0;
                            let rect = Rect::from_two_pos(bottom_left, top_right);
                            if ui.put(rect, egui::Button::new("x")).clicked() {
                                removes.push(i);
                            }
                            if self.name_edit {
                                if ui.put(label.rect, egui::TextEdit::singleline(&mut self.system_names[i])).lost_focus() {
                                    self.name_edit = !self.name_edit;
                                };
                            }
                        }
                    }).response;
                };
                // Remove the cam system
                for i in removes {
                    // If last element avoid that it can't be removed
                    if (i != 0) & (i<self.selected_plots.len()) {
                        // If system currently selected -> remove index
                        if self.open_system_id == i {
                            self.open_system_id -= 1;
                        }
                        self.cam_systems.remove(i);
                        self.system_names.remove(i);
                        self.selected_plots.remove(i);
                    }
                }
                // Add new section
                if ui.button("+").clicked() {
                    let insert_last = self.system_names.len()-1;
                    self.system_names.insert(insert_last, format!("Default_{}", self.system_names.len()));
                    self.cam_systems.insert(insert_last, CamSystem::default());
                    self.selected_plots.insert(insert_last, vec![vec![PlotKind::Position]]);
                    self.open_system_id = insert_last;
                }                     
            });

            // ----- PLOTS -----
            ui.vertical(|ui| {
                let remaining_height = ui.available_height();
                // UI for cam system if not summary
                if self.system_names[self.open_system_id] != "Summary" {
                    self.ui_cam_system(ctx, ui, ui_width, remaining_height);
                    // Only continuously redraw if animation speed > 0
                    if self.cam_systems[self.open_system_id].animation_speed > 0.0 {
                        ctx.request_repaint();
                    }
                } 
                // UI for summary of all system
                else {
                    self.ui_summary(ui, ui_width, remaining_height)
                }
            })
        });
        
    }
}
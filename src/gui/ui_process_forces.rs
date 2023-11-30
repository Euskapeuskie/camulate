use eframe::egui;

use crate::cam_modeller::process_forces::{ProcessForces, ProcessForceKind, ProcessForce};

const process_force_types: [ProcessForceKind; 3] = [ProcessForceKind::Cutting, ProcessForceKind::Stitching, ProcessForceKind::Friction];

impl ProcessForces {
    /// Return true if any parameter of the follower was changed, else returns false
    pub fn ui_process_force_modification(&mut self, ui: &mut eframe::egui::Ui, width: f32, height: f32) -> bool {
        let accuracy = self.accuracy;
        let mut rs = vec![];
        let mut remove_element = None;
    
        let response = egui::CollapsingHeader::new(egui::widget_text::RichText::new("Process forces").size(15.0).underline())
        .default_open(false)
        .show(ui, |ui| {
            let _response_inner = ui.vertical(|ui| {  
                egui::Grid::new("ProcessForceUI").show(ui, |ui| {
    
                    // Labels for type, deg_start, deg_end, ...
                    ui.add_sized([4.85*width/10.0, 20.0], egui::Label::new("Process force type"));
                    ui.add_sized([1.5*width/10.0, 20.0], egui::Label::new("Start"));
                    ui.add_sized([1.5*width/10.0, 20.0], egui::Label::new("End"));
                    ui.add_sized([1.5*width/10.0, 20.0], egui::Label::new("Peak force"));
                    ui.end_row();
    
                    for (i, process_force) in self.process_forces.iter_mut().enumerate() {       
                        // ComboBox for MovementType   
                        let res_force_kind = egui::ComboBox::from_id_source(format!("{:?}-{i}", process_force))
                            .selected_text(format!("{:?}", process_force.kind))
                            .width(4.85*width/10.0)
                            .show_ui(ui, |ui| {
                                for process_force_type in process_force_types {
                                    let x = ui.selectable_value(&mut process_force.kind, process_force_type, format!("{process_force_type:?}"));
                                    rs.push(x.changed());
                                };
                            }).response;
        
                        // Input fields for deg_start, deg_end and incline
                        let res_deg_start = ui.add_sized([1.5*width/10.0, 20.0], egui::DragValue::new(&mut process_force.deg_start).speed(accuracy).clamp_range(0.0..=std::f64::INFINITY).update_while_editing(false).suffix(" °"));
                        let res_deg_end = ui.add_sized([1.5*width/10.0, 20.0], egui::DragValue::new(&mut process_force.deg_end).speed(accuracy).clamp_range(process_force.deg_start..=std::f64::INFINITY).update_while_editing(false).suffix(" °"));
                        let res_peak_force = ui.add_sized([1.5*width/10.0, 20.0], egui::DragValue::new(&mut process_force.peak_force).speed(0.1).update_while_editing(false).suffix(" N"));
                        let r = res_force_kind.changed() | res_deg_start.changed() | res_deg_end.changed() | res_peak_force.changed();
                        rs.push(r);
        
                        // Button for removing a section
                        if ui.button("x").on_hover_text("Remove force").clicked() {
                            remove_element = Some(i);
                            rs.push(true);
                        };
                        ui.end_row();
                    }    
                });
                // Button for adding a section
                if ui.button("Add force").clicked() {
                    self.process_forces.push(ProcessForce::new(ProcessForceKind::Cutting, 0.0, 0.0, 10.0));
                    rs.push(true);
                }
            }).response;
        });
        ui.add_sized([width, 1.0], egui::Separator::default());

    
        if let Some(remove_element) = remove_element {
            self.process_forces.remove(remove_element);
            rs.push(true);
        };

        if rs.iter().any(|x| *x == true) {
            true
        }
        else {false}
    }
}
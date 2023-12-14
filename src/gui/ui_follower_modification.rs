use eframe::egui;

use crate::cam_modeller::cam_follower::{CamFollower, MovementType};
use crate::cam_modeller::movement_laws_rest::MovementLawsRest;
use crate::cam_modeller::movement_laws_rest_rest::MovementLawsRestRest;
use crate::cam_modeller::movement_laws_rest_return::{MovementLawsRestReturn, MovementLawsReturnRest};

const movement_types_list: [MovementType; 4] = [
    MovementType::Rest(MovementLawsRest::Rest),
    MovementType::RestRest(MovementLawsRestRest::__PlaceHolder),
    MovementType::RestReturn(MovementLawsRestReturn::__PlaceHolder),
    MovementType::ReturnRest(MovementLawsReturnRest::__PlaceHolder)];

const movement_laws_rest: [MovementLawsRest; 1] = [MovementLawsRest::Rest];

const movement_laws_rest_rest: [MovementLawsRestRest; 7] = [
    MovementLawsRestRest::AccelerationTrapezoid,
    MovementLawsRestRest::Linear,
    MovementLawsRestRest::ParabolaQuadratic,
    MovementLawsRestRest::Poly5,
    MovementLawsRestRest::SineInclined,
    MovementLawsRestRest::SineModified,
    MovementLawsRestRest::SineSimple];

const movement_laws_rest_return: [MovementLawsRestReturn; 2] = [
    MovementLawsRestReturn::AccelerationTrapezoid,
    MovementLawsRestReturn::HarmonicCombination];

const movement_laws_return_rest: [MovementLawsReturnRest; 2] = [
    MovementLawsReturnRest::AccelerationTrapezoid,
    MovementLawsReturnRest::HarmonicCombination];

impl CamFollower {
    /// Return true if any parameter of the follower was changed, else returns false
    pub fn ui_follower_modification(&mut self, ui: &mut eframe::egui::Ui, width: f32, height: f32) -> bool {
        let accuracy = self.accuracy;
        let mut rs = vec![];
        let mut remove_element = None;

        let respone = egui::CollapsingHeader::new(egui::widget_text::RichText::new("Follower modification").size(15.0).underline())
        .default_open(true)
        .show(ui, |ui| {
            ui.vertical(|ui| {  
                egui::Grid::new("FollowerUI").show(ui, |ui| {

                    // Labels for type, deg_start, deg_end, ...
                    ui.add_sized([2.1*width/10.0, 20.0], egui::Label::new("Movement type"));
                    ui.add_sized([2.1*width/10.0, 20.0], egui::Label::new("Movement law"));
                    ui.add_sized([1.5*width/10.0, 20.0], egui::Label::new("Start"));
                    ui.add_sized([1.5*width/10.0, 20.0], egui::Label::new("End"));
                    ui.add_sized([1.5*width/10.0, 20.0], egui::Label::new("Incline"));
                    ui.end_row();

                    for (i, section) in self.sections.iter_mut().enumerate() {       
                        // ComboBox for MovementType   
                        let res_movement_type = egui::ComboBox::from_id_source(&section)
                            .selected_text(format!("{}", section.movement_type))
                            .width(2.1*width/10.0)
                            .show_ui(ui, |ui| {
                                for movement_type in movement_types_list {
                                    let x = ui.selectable_value(&mut section.movement_type, movement_type, format!("{movement_type}"));
                                    rs.push(x.changed());
                                };
                            }).response;
                        
                        // Currently selected movement_law
                        let movement_type = match section.movement_type {
                            MovementType::Rest(x) => format!("{x}"),
                            MovementType::RestRest(x) => format!("{x}"),
                            MovementType::RestReturn(x) => format!("{x}"),
                            MovementType::ReturnRest(x) => format!("{x}"),
                        };

                        // ComboBox for MovementLaw
                        let res_movement_law = egui::ComboBox::from_id_source(format!("{:?}-{i}", section))
                            .selected_text(format!("{}", movement_type))
                            .width(2.1*width/10.0)
                            .show_ui(ui, |ui| {
                                match section.movement_type {
                                    MovementType::Rest(x) => {
                                        for movement_law in movement_laws_rest {
                                            let x = ui.selectable_value(&mut section.movement_type, MovementType::Rest(movement_law), format!("{movement_law}"));
                                            rs.push(x.changed());
                                        }
                                    }
                                    MovementType::RestRest(x) => {
                                        for movement_law in movement_laws_rest_rest {
                                            let x = ui.selectable_value(&mut section.movement_type, MovementType::RestRest(movement_law), format!("{movement_law}"));
                                            rs.push(x.changed());
                                        }
                                    }
                                    MovementType::RestReturn(x) => {
                                        for movement_law in movement_laws_rest_return {
                                            let x = ui.selectable_value(&mut section.movement_type, MovementType::RestReturn(movement_law), format!("{movement_law}"));
                                            rs.push(x.changed());
                                        }
                                    }
                                    MovementType::ReturnRest(x) => {
                                        for movement_law in movement_laws_return_rest {
                                            let x = ui.selectable_value(&mut section.movement_type, MovementType::ReturnRest(movement_law), format!("{movement_law}"));
                                            rs.push(x.changed());
                                        }
                                    }
                                }
                            }).response;
        
                        // Input fields for deg_start, deg_end and incline
                        let res_deg_start = ui.add_sized([1.5*width/10.0, 20.0], egui::DragValue::new(&mut section.deg_start).speed(accuracy).clamp_range(0.0..=360.0).update_while_editing(false).suffix(" °"));
                        let res_deg_end = ui.add_sized([1.5*width/10.0, 20.0], egui::DragValue::new(&mut section.deg_end).speed(accuracy).clamp_range(section.deg_start..=360.0).update_while_editing(false).suffix(" °"));
                        let res_incline = ui.add_sized([1.5*width/10.0, 20.0], egui::DragValue::new(&mut section.incline)
                                .speed(0.0001)
                                .update_while_editing(false)
                                .custom_parser(|s| {
                                    match s.parse::<f64>() {
                                        Ok(s) => Some(s/1000.0),
                                        Err(e) => Some(0.0),
                                    }
                                })
                                .custom_formatter(|n, _| format!("{:.2}", n*1000.0))
                                .suffix(" mm"));

                        let r = res_movement_type.changed() | res_movement_law.changed() | res_deg_start.changed() | res_deg_end.changed() | res_incline.changed();
                        rs.push(r);
        
                        // Button for removing a section
                        if ui.button("x").on_hover_text("Remove section").clicked() {
                            remove_element = Some(i);
                            rs.push(true);
                        };
                        ui.end_row();
                    }    
                });
                // Button for adding a section
                if ui.button("Add section").clicked() {
                    self.add_section(MovementType::Rest(MovementLawsRest::Rest), 0.0, 360.0 - 2.0*self.accuracy, 360.0, 0.5);
                    rs.push(true);
                }

            });
        });

        ui.add_sized([width, 1.0], egui::Separator::default());
    
        if let Some(remove_element) = remove_element {
            self.sections.remove(remove_element);
            rs.push(true);
        };

        if rs.iter().any(|x| *x == true) {
            true
        }
        else {false}
    }
}








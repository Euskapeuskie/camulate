use std::{fmt::Display, ops::RangeInclusive};
use std::f64::consts::PI;
use std::collections::HashMap;
use serde::{Serialize, Deserialize};

use eframe::egui;
use eframe::egui::Rect;
use eframe::egui::epaint::Rgba;
use egui_plot::{PlotPoints, PlotPoint, Plot, Legend, Line, VLine, PlotBounds, HLine, AxisHints};

use crate::cam_modeller::cam_system::{CamSystem, CamType, CylindricBead, CamFeasibility, CylindricGroove, PlanarGroove};
use super::helper_functions;

#[derive(Debug, Serialize, Deserialize, Clone, Copy, PartialEq)]
pub enum PlotKind {
    Position,
    Velocity,
    Acceleration,
    ContactAngle,
    CurvatureDisc,
    CurvatureHertz,
    ForceY,
    NormalForce,
    Torque,
    HertzPressure,
    BearingForcesAxial,
    BearingForcesRadial,
    RPMFollower,
}

impl Display for PlotKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Position => write!(f, "position"),
            Self::Velocity => write!(f, "velocity"),
            Self::Acceleration => write!(f, "acceleration"),
            Self::ContactAngle => write!(f, "contact angle"),
            Self::CurvatureDisc => write!(f, "disc curvature"),
            Self::CurvatureHertz => write!(f, "Ersatzkrümmung"),
            Self::ForceY => write!(f, "Force Y-Dir"),
            Self::NormalForce => write!(f, "normal force"),
            Self::Torque => write!(f, "torque"),
            Self::HertzPressure => write!(f, "hertz pressure"),
            Self::BearingForcesAxial => write!(f, "shaft bearing forces axial"),
            Self::BearingForcesRadial => write!(f, "shaft bearing forces radial"),
            Self::RPMFollower => write!(f, "RPM of follower")
        }
    }
}

impl From<String> for PlotKind {
    fn from(value: String) -> Self {
        let mut kind: String = value.clone();
        if value.contains("ideal") {
            kind = value.split("ideal ").collect::<Vec<_>>()[1..].join(" ");
        }
        if value.contains("real") {
            kind = value.split("ideal ").collect::<Vec<_>>()[1..].join(" ");
        }
        let kind = &*kind;

        match kind {
            "position" => Self::Position,
            "velocity" => Self::Velocity,
            "acceleration" => Self::Acceleration,
            "contact angle" => Self::ContactAngle,
            "disc curvature" => Self::CurvatureDisc,
            "Ersatzkrümmung" => Self::CurvatureHertz,
            "normal force" => Self::NormalForce,
            "torque" => Self::Torque,
            "hertz pressure" => Self::HertzPressure,
            "bearing forces axial" => Self::BearingForcesAxial,
            "bearing forces radial" => Self::BearingForcesRadial,
            "RPM of follower" => Self::RPMFollower,
            _ => Self::Position,
        }
    }
}


impl CamSystem {

    /// Plot the animated cam system with real values of the follower
    /// 
    /// Inputs
    /// 
    /// HAS TODO THE ROTATION
    pub fn plot_animation(&self, ui: &mut egui::Ui, width: f32, height: f32) -> egui::Response {
        // Generate the points for the animation of disc & follower
        let angle = -(self.index as f64 * self.follower.accuracy)/180.0 * PI + PI/2.0;
        let pos_y_follower = self.real_positions[self.index];
        let points_follower = helper_functions::plot_points_circle(self.follower.radius_follower, pos_y_follower);
        let points_second_follower = match &self.cam_type {
            CamType::CylindricBead(cylinder) => 
                Some(helper_functions::plot_points_circle(self.follower.radius_follower, self.follower.radius[self.index] - cylinder.min_bead_thickness - 2.0*self.follower.radius_follower)),
            _ => None
        };
        let (points_disc, points_second_disc) = match &self.cam_type {
            CamType::PlanarDisc => 
                (helper_functions::rotate_points_planar_xy(&self.disc_points_xyz, angle), None),
            CamType::PlanarGroove(groove) =>
                (helper_functions::rotate_points_planar_xy(&self.disc_points_xyz, angle), Some(helper_functions::rotate_points_planar_xy(&groove.second_disc, angle))),
            CamType::CylindricBead(cylinder) => 
                (helper_functions::rotate_points_cylinder_z(&self.disc_points_xyz, angle), Some(helper_functions::rotate_points_cylinder_z(&cylinder.second_disc, angle))),
            CamType::CylindricGroove(cylinder) => 
                (helper_functions::rotate_points_cylinder_z(&self.disc_points_xyz, angle), Some(helper_functions::rotate_points_cylinder_z(&cylinder.second_disc, angle))),
        };

        // Define axis limits for the animation
        let max_follower = self.follower.radius.iter().max_by(|a, b| a.total_cmp(b)).unwrap();
        let (x_min, x_max, y_min, y_max) = match &self.cam_type {
            CamType::PlanarDisc => (-*max_follower, *max_follower, -*max_follower, *max_follower+2.0*self.follower.radius_follower+0.01),
            CamType::PlanarGroove(_) => (-*max_follower, *max_follower, -*max_follower, *max_follower+2.0*self.follower.radius_follower+0.01),
            CamType::CylindricBead(cylinder) => (-cylinder.base_radius, cylinder.base_radius, -0.01, *max_follower+2.0*self.follower.radius_follower+0.01),
            CamType::CylindricGroove(cylinder) => (-cylinder.base_radius, cylinder.base_radius, -0.01, *max_follower+2.0*self.follower.radius_follower+0.01),
        };

        let plot_animation = Plot::new("animation")
            .legend(Legend::default())
            // .width(width);
            .data_aspect(1.0)
            .view_aspect(1.0)
            .height(height)
            .label_formatter(|name, value| format!("x: {:.2}\ny:{:.2}", value.x*1000.0, value.y*1000.0))
            .x_axis_formatter(|f, n, _| format!("{:.2}", f*1000.0))
            .y_axis_formatter(|f, n, _| format!("{:.2}", f*1000.0));

        let plot_response = plot_animation.show(ui, |plot_ui| {
            plot_ui.line(Line::new(points_disc).name("cam disc"));
            plot_ui.line(Line::new(points_follower).name("follower"));
            if let Some(second_disc) = points_second_disc {
                plot_ui.line(Line::new(second_disc).name("second disc"));
            }
            if let Some(second_follower) = points_second_follower {
                plot_ui.line(Line::new(second_follower).name("back follower"));
            }
            plot_ui.set_plot_bounds(PlotBounds::from_min_max([x_min, y_min], [x_max, y_max]));
        }).response;

        plot_response
    }


    /// Plot the graphs for position, velocity, etc.
    /// 
    /// # Inputs
    /// * `kind`: PlotKind to plot for, e.g. `PlotKind::Position`
    /// * `ui`: egui UI that should be plotted on
    /// * `width`: Total width of the plot in Pixels
    /// * `height`: Total height of the plot in Pixels
    /// 
    /// # Returns
    /// Response of the plot
    pub fn plot_graph(&self, kind: PlotKind, ui: &mut egui::Ui, width: f32, height: f32, plot_real_values: bool) -> egui::Response {

        // phis, ideal_values, real_values
        let (phis, ideal_values, real_values) = match kind {
            PlotKind::Position => (&self.follower.phis, &self.ideal_stroke, &self.real_stroke),
            PlotKind::Velocity => (&self.follower.phis, &self.ideal_velocities, &self.real_velocities),
            PlotKind::Acceleration => (&self.follower.phis, &self.ideal_accelerations, &self.real_accelerations),
            PlotKind::ContactAngle => (&self.follower.phis, &self.contact_angles, &self.contact_angles),
            PlotKind::CurvatureHertz => (&self.follower.phis, &self.curvature_total, &self.curvature_total),
            PlotKind::CurvatureDisc => (&self.follower.phis, &self.curvature_disc, &self.curvature_disc),
            PlotKind::ForceY => (&self.follower.phis, &self.ideal_force_y, &self.ideal_force_y),
            PlotKind::NormalForce => (&self.follower.phis, &self.ideal_force_normal, &self.real_force_normal),
            PlotKind::Torque => (&self.follower.phis, &self.ideal_torque, &self.real_torque),
            PlotKind::HertzPressure => (&self.follower.phis, &self.ideal_hertz_pressure, &self.real_hertz_pressure),
            PlotKind::BearingForcesAxial => (&self.follower.phis, &self.ideal_bearing_forces_axial, &self.ideal_bearing_forces_axial),
            PlotKind::BearingForcesRadial => (&self.follower.phis, &self.ideal_bearing_forces_radial, &self.ideal_bearing_forces_radial),
            PlotKind::RPMFollower => (&self.follower.phis, &self.ideal_ns_follower, &self.ideal_ns_follower),
        };

        // y-axis label
        let y_label = match kind {
            PlotKind::Position => format!("{kind} in [mm]"),
            PlotKind::Velocity => format!("{kind} in [mm/s]"),
            PlotKind::Acceleration => format!("{kind} in [mm/s²]"),
            PlotKind::ContactAngle => format!("{kind} in [rad]"),
            PlotKind::CurvatureHertz => format!("{kind} in mm"),
            PlotKind::CurvatureDisc => format!("{kind} in mm"),
            PlotKind::ForceY => format!("{kind} in [N]"),
            PlotKind::NormalForce => format!("{kind} in [N]"),
            PlotKind::Torque => format!("{kind} in [Nm]"),
            PlotKind::HertzPressure => format!("{kind} in [N/mm²]"),
            PlotKind::BearingForcesAxial => format!("{kind} in [N]"),
            PlotKind::BearingForcesRadial => format!("{kind} in [N]"),
            PlotKind::RPMFollower => format!("{kind} in [1/min]"),
        };
        let angle = self.index as f64 * self.follower.accuracy;

        let ideal_plot_points = phis
            .iter()
            .zip(ideal_values.iter())
            .map(|(phi, x)| [*phi, *x])
            .collect::<Vec<_>>();

        let real_plot_points = phis
            .iter()
            .zip(real_values.iter())
            .map(|(phi, x)| [*phi, *x])
            .collect::<Vec<_>>();

        let mut plot = Plot::new(format!("{kind}"))
            .x_axis_label("Rotation angle in [°]")
            .y_axis_label(y_label)
            .width(width)
            .height(height)
            .auto_bounds_x()
            .auto_bounds_y();

        if plot_real_values {
            plot = plot.legend(Legend::default());
        }

        let plot_response = plot.show(ui, |plot_ui| {
            plot_ui.line(Line::new(PlotPoints::new(ideal_plot_points)).name(format!("ideal {kind}", )));
            if plot_real_values {
                plot_ui.line(Line::new(PlotPoints::new(real_plot_points)).name(format!("real {kind}", )));
                plot_ui.vline(VLine::new(angle));
            }
            if kind == PlotKind::HertzPressure {
                plot_ui.hline(HLine::new(1500.0));
            }
        }).response;

        plot_response
    }



    /// Scale the graphs from -1 to 1 then plot with that
    pub fn multi_plot_graph(&self, kinds: Vec<PlotKind>, ui: &mut egui::Ui, width: f32, height: f32, plot_real_values: bool) -> egui::Response {

        let angle = self.index as f64 * self.follower.accuracy;

        let mut plot = Plot::new(format!("{kinds:?}"))
            .x_axis_label("Rotation angle in [°] / Time in [ms]")
            .width(width)
            .height(height)
            .auto_bounds_x()
            .auto_bounds_y();
        if plot_real_values {
            plot = plot.legend(Legend::default());
        }

        struct ToBePlotted {
            ideal_plot_points: Vec<[f64; 2]>,
            real_plot_points: Vec<[f64;2]>,
            kind: PlotKind,
            y_min: f64,
            y_max: f64,
        }

        let mut plots = vec![];
        let mut x_axes_formatter = AxisHints::default();
        let mut y_axes_formatter = vec![];
        let mut label_formatter_dict: HashMap<String, (f64, f64)> = HashMap::new();

        for kind in kinds {

            // phis, ideal_values, real_values
            let (phis, ideal_values, real_values) = match kind {
                PlotKind::Position => (&self.follower.phis, &self.ideal_stroke, &self.real_stroke),
                PlotKind::Velocity => (&self.follower.phis, &self.ideal_velocities, &self.real_velocities),
                PlotKind::Acceleration => (&self.follower.phis, &self.ideal_accelerations, &self.real_accelerations),
                PlotKind::ContactAngle => (&self.follower.phis, &self.contact_angles, &self.contact_angles),
                PlotKind::CurvatureHertz => (&self.follower.phis, &self.curvature_total, &self.curvature_total),
                PlotKind::CurvatureDisc => (&self.follower.phis, &self.curvature_disc, &self.curvature_disc),
                PlotKind::ForceY => (&self.follower.phis, &self.ideal_force_y, &self.ideal_force_y),
                PlotKind::NormalForce => (&self.follower.phis, &self.ideal_force_normal, &self.real_force_normal),
                PlotKind::Torque => (&self.follower.phis, &self.ideal_torque, &self.real_torque),
                PlotKind::HertzPressure => (&self.follower.phis, &self.ideal_hertz_pressure, &self.real_hertz_pressure),
                PlotKind::BearingForcesAxial => (&self.follower.phis, &self.ideal_bearing_forces_axial, &self.ideal_bearing_forces_axial),
                PlotKind::BearingForcesRadial => (&self.follower.phis, &self.ideal_bearing_forces_radial, &self.ideal_bearing_forces_radial),
                PlotKind::RPMFollower => (&self.follower.phis, &self.ideal_ns_follower, &self.ideal_ns_follower),
            };

            // y-axis label
            let y_label = match kind {
                PlotKind::Position => format!("{kind} in [mm]"),
                PlotKind::Velocity => format!("{kind} in [m/s]"),
                PlotKind::Acceleration => format!("{kind} in [m/s²]"),
                PlotKind::ContactAngle => format!("{kind} in [°]"),
                PlotKind::CurvatureHertz => format!("{kind} in mm"),
                PlotKind::CurvatureDisc => format!("{kind} in mm"),
                PlotKind::ForceY => format!("{kind} in [N]"),
                PlotKind::NormalForce => format!("{kind} in [N]"),
                PlotKind::Torque => format!("{kind} in [Nm]"),
                PlotKind::HertzPressure => format!("{kind} in [N/mm²]"),
                PlotKind::BearingForcesAxial => format!("{kind} in [N]"),
                PlotKind::BearingForcesRadial => format!("{kind} in [N]"),
                PlotKind::RPMFollower => format!("{kind} in [1/min]"),
            };

            let y_max_ideal = ideal_values.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
            let y_max_real = ideal_values.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
            let y_max = [y_max_ideal, y_max_real];
            let y_max = y_max.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
            let y_min_ideal = ideal_values.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
            let y_min_real = ideal_values.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
            let y_min = [y_min_ideal, y_min_real];
            let y_min = y_min.iter()
                .min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap()
                .max(0.0);
            let y_span = *y_max - y_min;

            let ideal_plot_points = phis
                .iter()
                .zip(ideal_values.iter())
                .map(|(phi, x)| [*phi, (*x-y_min)/y_span])
                .collect::<Vec<_>>();

            let real_plot_points = phis
                .iter()
                .zip(real_values.iter())
                .map(|(phi, x)| [*phi, (*x-y_min)/y_span])
                .collect::<Vec<_>>();

            let t = ToBePlotted {
                ideal_plot_points: ideal_plot_points,
                real_plot_points: real_plot_points,
                kind: kind,
                y_min: y_min,
                y_max: *(*y_max),
            };
            plots.push(t);
            let rpm = self.rpm;
            x_axes_formatter = x_axes_formatter.label("\n\n\n")
                .formatter(move |phi, _digits, _range: &RangeInclusive<f64>| {
                    let time_ms = phi/360.0 * 60.0/rpm * 1000.0;
                    format!("{:.1}°\n{:.1}ms", phi, time_ms)
                });
            y_axes_formatter.push(AxisHints::default()
                .label(format!("{y_label}"))
                .formatter(move |y, _digits, _range: &RangeInclusive<f64>| {
                    let mut formatted_y = y*y_span+y_min;
                    match kind {
                        PlotKind::Position => formatted_y *= 1000.0,
                        PlotKind::CurvatureDisc => formatted_y *= 1000.0,
                        PlotKind::CurvatureHertz => formatted_y *= 1000.0,
                        PlotKind::ContactAngle => formatted_y *= 180.0/PI,
                        _ => (),
                    }
                    format!("{:.1}", formatted_y)
                })
            );
            label_formatter_dict.insert(format!("{kind}"), (y_span, y_min));
        }

        plot = plot.custom_y_axes(y_axes_formatter);
        plot = plot.custom_x_axes(vec![x_axes_formatter]);
        plot = plot.label_formatter(move |s: &str, val: &PlotPoint| {
            let k: PlotKind = String::from(s).into();
            let k_string = format!("{k}");
            let (y_span, y_min) = label_formatter_dict.get(&k_string).unwrap_or_else(|| &(0.0, 0.0));
            match k {
                PlotKind::Position => format!("{s}\nx: {:.2}°\ny: {:.2}", val.x, (val.y*y_span+y_min)*1000.0),
                PlotKind::CurvatureDisc => format!("{s}\nx: {:.2}°\ny: {:.2}", val.x, (val.y*y_span+y_min)*1000.0),
                PlotKind::CurvatureHertz => format!("{s}\nx: {:.2}°\ny: {:.2}", val.x, (val.y*y_span+y_min)*1000.0),
                PlotKind::ContactAngle => format!("{s}\nx: {:.2}°\ny: {:.2}", val.x, (val.y*y_span+y_min)*180.0/PI),
                _ => format!("{s}\nx: {:.2}°\ny: {:.2}", val.x, val.y*y_span+y_min),
            }
            
        });
        let plot_response = plot.show(ui, |plot_ui| {
            for p in plots {
                plot_ui.line(Line::new(PlotPoints::new(p.ideal_plot_points)).name(format!("ideal {}", p.kind)));
                if plot_real_values {
                    plot_ui.line(Line::new(PlotPoints::new(p.real_plot_points)).name(format!("real {}", p.kind)));
                }
                if p.kind == PlotKind::HertzPressure {
                    plot_ui.hline(HLine::new((self.max_hertz-p.y_min)/(p.y_max-p.y_min)).name(format!("{} N/mm²", self.max_hertz)));
                }
            }
            plot_ui.vline(VLine::new(angle));
        }).response;

        plot_response
    }


    pub fn ui_cam_modification(&mut self, ui: &mut egui::Ui, width: f32, height: f32) -> egui::Response {
        let cam_type_list: [CamType; 4] = [CamType::PlanarDisc, CamType::PlanarGroove(PlanarGroove::default()), CamType::CylindricBead(CylindricBead::default()), CamType::CylindricGroove(CylindricGroove::default())];
        let mut rs = vec![];

        let response = ui.vertical(|ui| {

            // General cam inputs
            ui.add_sized([width, 1.0], egui::Separator::default());
            egui::Grid::new("GeneralCamInputs").show(ui, |ui| {
                let col_width = width/6.0;

                ui.label("Cam system type:");
                let res_cam_type = egui::ComboBox::from_id_source("Select value")
                    .selected_text(format!("{}", self.cam_type))
                    .width(col_width)
                    .show_ui(ui, |ui| {
                        for t in cam_type_list.iter() {
                            let r = ui.selectable_value(&mut self.cam_type, t.clone(), (format!("{}", t)));
                            rs.push(r.changed())
                        }
                    }).response;

                ui.label("Omega:");
                let res_rpm = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut self.rpm)
                    .speed(1.0)
                    .update_while_editing(false)
                    .clamp_range(0.0..=std::f64::INFINITY).suffix(" 1/min"));

                ui.label("Gravity:");
                let res_grav = ui.add(egui::Checkbox::without_text(&mut self.gravity))
                    .on_hover_text("Enable if vertical layout, disable if horizontal layout");
                ui.end_row();


                ui.label("Follower radius:");
                let res_radius = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut self.follower.radius_follower)
                    .speed(0.0001)
                    .update_while_editing(false)
                    .custom_parser(parser_m_to_mm)
                    .custom_formatter(formatter_mm_to_m)
                    .clamp_range(0.0..=std::f64::INFINITY)
                    .suffix(" mm"));

                ui.label("Contact length:");
                let res_contact_length = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut self.contact_length)
                        .speed(0.0001)
                        .update_while_editing(false)
                        .clamp_range(0.0..=std::f64::INFINITY)
                        .custom_parser(parser_m_to_mm)
                        .custom_formatter(formatter_mm_to_m)
                        .suffix(" mm"))
                    .on_hover_text("Contact length of the cam follower with the cam");

                ui.label("Follower mass:");
                let res_mass = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut self.mass)
                    .speed(0.1)
                    .update_while_editing(false)
                    .clamp_range(0.0..=std::f64::INFINITY)
                    .suffix(" kg"));
                ui.end_row();

                ui.label("Spring rate:");
                let res_spring_rate = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut self.spring_rate)
                    .speed(100)
                    .update_while_editing(false)
                    .clamp_range(0.0..=std::f64::INFINITY)
                    .custom_parser(|s| {
                        match s.parse::<f64>() {
                            Ok(s) => Some(s*1000.0),
                            Err(e) => Some(0.0)
                        }
                    }).custom_formatter(|n, _| format!("{:.2}", n/1000.0))
                    .suffix(" N/mm"));

                ui.label("Spring preload:");
                let res_spring_preload = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut self.spring_pretension)
                        .speed(1.0)
                        .update_while_editing(false)
                        .clamp_range(0.0..=std::f64::INFINITY)
                        .suffix(" N"))
                    .on_hover_text("Preload of the spring at the smallest diameter (ideally at 0°)");
                ui.end_row();

                ui.label("Max. Hertz:");
                let res_max_hertz = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut self.max_hertz)
                    .speed(1.0)
                    .update_while_editing(false)
                    .clamp_range(0.0..=std::f64::INFINITY)
                    .suffix(" N/mm²"))
                .on_hover_text("PMax. allowed hertzian pressure of either contact partner");

                ui.label("C_stat:");
                let res_c_stat = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut self.c_stat)
                    .speed(1.0)
                    .update_while_editing(false)
                    .clamp_range(0.0..=std::f64::INFINITY)
                    .suffix(" N"))
                .on_hover_text("C_stat according followers data sheet");
                ui.label("C_dyn:");
                let res_c_dyn = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut self.c_dyn)
                    .speed(1.0)
                    .update_while_editing(false)
                    .clamp_range(0.0..=std::f64::INFINITY)
                    .suffix(" N"))
                .on_hover_text("C_dyn according followers data sheet");

                ui.end_row();

                let r = res_cam_type.changed() | res_rpm.changed() | res_grav.changed() 
                | res_contact_length.changed() | res_mass.changed() | res_radius.changed()
                | res_spring_preload.changed() | res_spring_rate.changed() | res_max_hertz.changed()
                | res_c_stat.changed() | res_c_dyn.changed();
                rs.push(r);
                
                // Additional row only for cylindric bead cam
                if let CamType::CylindricBead(ref mut cylinder) = self.cam_type {
                    ui.label("Base radius:");
                    let res_base_diam = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut cylinder.base_radius)
                        .speed(0.0001)
                        .update_while_editing(false)
                        .clamp_range(0.0..=std::f64::INFINITY)
                        .custom_parser(parser_m_to_mm)
                        .custom_formatter(formatter_mm_to_m)
                        .suffix( "mm"));

                    ui.label("Bead separation:");
                    let res_bead_thickness = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut cylinder.min_bead_thickness)
                        .speed(0.0001)
                        .update_while_editing(false)
                        .clamp_range(0.0..=std::f64::INFINITY)
                        .custom_parser(parser_m_to_mm)
                        .custom_formatter(formatter_mm_to_m)
                        .suffix(" mm"));

                    let r = res_base_diam.changed() | res_bead_thickness.changed();
                    rs.push(r);
                }

                // Additional row only for cylindric groove cam
                if let CamType::CylindricGroove(ref mut cylinder) = self.cam_type {
                    ui.label("Base radius:");
                    let res_base_diam = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut cylinder.base_radius)
                        .speed(0.001)
                        .update_while_editing(false)
                        .clamp_range(0.0..=std::f64::INFINITY)
                        .custom_parser(parser_m_to_mm)
                        .custom_formatter(formatter_mm_to_m)
                        .suffix( "mm"));

                    ui.label("Groove play:");
                    let res_groove_play = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut cylinder.groove_play)
                        .speed(0.00001)
                        .update_while_editing(false)
                        .clamp_range(0.0..=std::f64::INFINITY)
                        .custom_parser(parser_m_to_mm)
                        .custom_formatter(formatter_mm_to_m)
                        .suffix( "mm"));
                    let r = res_base_diam.changed() | res_groove_play.changed();
                    rs.push(r);
                }

                // Additional row only for planar groove
                if let CamType::PlanarGroove(ref mut groove) = self.cam_type {
                    ui.label("Groove play:");
                    let res_groove_play = ui.add_sized([col_width, 20.0], egui::DragValue::new(&mut groove.groove_play)
                        .speed(0.00001)
                        .update_while_editing(false)
                        .clamp_range(0.0..=std::f64::INFINITY)
                        .custom_parser(parser_m_to_mm)
                        .custom_formatter(formatter_mm_to_m)
                        .suffix( "mm"));
                    let r = res_groove_play.changed();
                    rs.push(r);
                }
            });


            ui.horizontal(|ui| {
                ui.label("Animation speed:");
                ui.add(egui::Slider::new(&mut self.animation_speed, 0.0..=100.0).suffix(" %"));
            });
            ui.add_sized([width, 1.0], egui::Separator::default());
            
            if self.follower.ui_follower_modification(ui, width, height) {
                rs.push(true);
            };

            // UI for process forces
            if self.process_forces.ui_process_force_modification(ui, width, height) {
                rs.push(true);
            }

        }).response;

        // Update the planar_disc if any input parameter changed
        if rs.iter().any(|x| *x == true) {
            self.update_sections();
        }

        response
    }

    pub fn ui_overview_results(&mut self, ui: &mut egui::Ui, width: f32, height: f32) -> egui::Response {
        let min_v = self.ideal_velocities.iter().min_by(|a, b| a.partial_cmp(b).unwrap_or_else(|| std::cmp::Ordering::Equal))
            .unwrap_or_else(|| &std::f64::NAN);
        let max_v = self.ideal_velocities.iter().max_by(|a, b| a.partial_cmp(b).unwrap_or_else(|| std::cmp::Ordering::Equal))
            .unwrap_or_else(|| &std::f64::NAN);
        let min_a = self.ideal_accelerations.iter().min_by(|a, b| a.partial_cmp(b).unwrap_or_else(|| std::cmp::Ordering::Equal))
            .unwrap_or_else(|| &std::f64::NAN);
        let max_a = self.ideal_accelerations.iter().max_by(|a, b| a.partial_cmp(b).unwrap_or_else(|| std::cmp::Ordering::Equal))
            .unwrap_or_else(|| &std::f64::NAN);
        let max_hertz = self.ideal_hertz_pressure.iter().max_by(|a, b| a.partial_cmp(b).unwrap_or_else(|| std::cmp::Ordering::Equal))
        .unwrap_or_else(|| &std::f64::NAN);

        let mut dict_colormap: HashMap<CamFeasibility, [f32;4]> = HashMap::new();
        dict_colormap.insert(CamFeasibility::OK, [0.0, 255.0, 0.0, 0.001]);
        dict_colormap.insert(CamFeasibility::FollowerLiftOff, [255.0, 0.0, 0.0, 0.001]);
        dict_colormap.insert(CamFeasibility::HertzPressure, [255.0, 125.0, 0.0, 0.001]);
        dict_colormap.insert(CamFeasibility::Undercut, [255.0, 0.0, 0.0, 0.001]);

        let col_width = 1.5*width/10.0;

        let response = ui.vertical(|ui| {
            egui::CollapsingHeader::new(egui::widget_text::RichText::new("Results").size(15.0).underline())
            .default_open(false)
            .show(ui, |ui| {
                egui::Grid::new("Result summary").min_col_width(col_width).show(ui, |ui| {
                    
                    ui.label("Cam feasibility:").rect;
                    let [r, g, b, a] = dict_colormap[&self.cam_disc_feasible];
                    let rect = ui.add_sized([col_width, 20.0], egui::Label::new(
                        egui::RichText::new(format!("{:?}", self.cam_disc_feasible)).color(egui::Color32::from_rgb(r as u8, g as u8, b as u8))))
                        .on_hover_text( match self.cam_disc_feasible {
                            CamFeasibility::OK => "Cam system OK",
                            CamFeasibility::FollowerLiftOff => "Follower lifts off from cam disc, try increasing spring forces",
                            CamFeasibility::HertzPressure => "Hertzian pressure > p_max, try decreasing forces or change of follower radius/cam disc",
                            CamFeasibility::Undercut => "Cam disc not feasible, try to either change roller diameter or section definitions",
                        }).rect;  
                    ui
                        .painter()
                        .rect_filled(rect, 2.0, Rgba::from_rgba_unmultiplied(r, g, b, a));
                    ui.end_row();

                    ui.label("Min/Max velocity:\nMin/Max accel.:");
                    ui.label(format!("{:.2} / {:.2} m/s\n{:.2} / {:.2} m/s²", min_v, max_v, min_a, max_a));
                    ui.label("Max Hertz pressure:\nEquivalent load:");
                    ui.label(format!("{:.2} N/mm²\n{:.2} N", max_hertz, self.p_equiv)).on_hover_text(
                        if self.p_equiv.is_nan() {
                            "Make sure the follower doesn't lift off to allow for a calculation"
                        } else {
                            "Equivalent load for the calculation of L10 or L10_h"
                        }
                    );
                    ui.label("L_10:\nL_10_h:");
                    ui.label(format!("{:.0} x10e6\n{:.0} h", self.l_10, self.l_10_h)).on_hover_text(
                        if self.l_10.is_nan() {
                            "Make sure the follower doesn't lift off to allow for a calculation"
                        } else {
                            "L_10: Durability in million revolotions of the follower\nL_10_h: Durability in hours"
                        }
                    );
                    ui.end_row();
                });
            });
        }).response;

        response
    }
}


fn parser_m_to_mm(s: &str) -> Option<f64> {
    match s.parse::<f64>() {
        Ok(s) => Some(s/1000.0),
        Err(e) => Some(0.0),
    }
}

fn formatter_mm_to_m(n: f64, _: RangeInclusive<usize>) -> String {
    format!("{:.2}", n*1000.0)
}
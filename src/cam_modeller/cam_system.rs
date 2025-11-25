use std::f64::consts::PI;
use std::fmt::Display;
use std::mem::discriminant;
use serde::{Serialize, Deserialize};

use super::cam_follower::{CamFollower, MovementType};
use super::movement_laws_rest::MovementLawsRest;
use super::movement_laws_rest_rest::MovementLawsRestRest;
use super::helper_functions;
use super::process_forces::ProcessForces;

// ---------- CamType Enumerator ----------
#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum CamType {
    PlanarDisc,
    CylindricBead(CylindricBead),
    CylindricGroove(CylindricGroove),
    PlanarGroove(PlanarGroove),
}

impl Display for CamType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::PlanarDisc => write!(f, "PlanarDisc"),
            Self::CylindricBead(_) => write!(f, "CylindricBead"),
            Self::CylindricGroove(_) => write!(f, "CylindricGroove"),
            Self::PlanarGroove(_) => write!(f, "PlanarGroove"),
        }
    }
}

impl PartialEq for CamType {
    fn eq(&self, other: &Self) -> bool {
        discriminant(self) == discriminant(other)
    }
}

// ---------- Struct for CylindricBead --> is used in the CamType Enumerator ----------
#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct CylindricBead {
    pub base_radius: f64,
    pub second_disc: Vec<[f64; 3]>,
    pub min_bead_thickness: f64,
    pub smoothing_optimized_curve: usize,
    pub is_optimized: bool,
}

impl Default for CylindricBead {
    fn default() -> Self {
        Self {
            base_radius: 0.05,
            second_disc: vec![],
            min_bead_thickness: 0.006,
            smoothing_optimized_curve: 100,
            is_optimized: false
        }
    }
}


// ---------- Struct for CylindricGroove --> is used in the CamType Enumerator ----------
#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct CylindricGroove {
    pub base_radius: f64,
    pub groove_play: f64,
    pub second_disc: Vec<[f64; 3]>,
}

impl Default for CylindricGroove {
    fn default() -> Self {
        Self {
            base_radius: 0.05,
            groove_play: 0.0001,
            second_disc: vec![],
        }
    }
}


// ---------- Struct for PlanarGroove --> is used in the CamType Enumerator ----------
#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct PlanarGroove {
    pub groove_play: f64,
    pub second_disc: Vec<[f64; 3]>,
}

impl Default for PlanarGroove {
    fn default() -> Self {
        Self {
            groove_play: 0.0001,
            second_disc: vec![],
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CamFeasibility {
    OK,
    Undercut,
    FollowerLiftOff,
    HertzPressure,
}


// ---------- Struct for CamSystem ----------
#[derive(Debug, Serialize, Deserialize)]
pub struct CamSystem {
    pub cam_type: CamType,
    pub follower: CamFollower,
    pub process_forces: ProcessForces,
    pub disc_points_xyz: Vec<[f64; 3]>,
    pub normal_vec_follower: Vec<[f64;3]>,
    pub contact_angles: Vec<f64>,
    pub curvature_total: Vec<f64>,
    pub curvature_disc: Vec<f64>,
    pub rpm: f64,
    pub mass: f64,
    pub spring_rate: f64,
    pub spring_pretension: f64,
    pub contact_length: f64,
    pub gravity: bool,
    pub index: usize,
    pub animation_speed: f64,
    pub cam_disc_feasible: CamFeasibility,
    pub max_hertz: f64,
    pub c_stat: f64,
    pub c_dyn: f64,
    pub p_equiv: f64,
    pub l_10: f64,
    pub l_10_h: f64,

    pub ideal_stroke: Vec<f64>,
    pub ideal_velocities: Vec<f64>,
    pub ideal_accelerations: Vec<f64>,
    pub ideal_force_y: Vec<f64>,
    pub ideal_force_normal: Vec<f64>,
    pub ideal_torque: Vec<f64>,
    pub ideal_hertz_pressure: Vec<f64>,
    pub ideal_bearing_forces_radial: Vec<f64>,
    pub ideal_bearing_forces_axial: Vec<f64>,
    pub ideal_ns_follower: Vec<f64>,

    pub real_positions: Vec<f64>,
    pub real_stroke: Vec<f64>,
    pub real_velocities: Vec<f64>,
    pub real_accelerations: Vec<f64>,
    pub real_force_y: Vec<f64>,
    pub real_force_normal: Vec<f64>,
    pub real_torque: Vec<f64>,
    pub real_hertz_pressure: Vec<f64>,
}


impl CamSystem {
    pub fn new(mut cam_type: CamType, follower: CamFollower, process_forces: ProcessForces, rpm: f64, mass: f64, spring_rate: f64, spring_pretension: f64, gravity: bool) -> Self {
        let n_elements = follower.phis.len();
        let gravity_value = 9.81 * (gravity as usize as f64);
        let phis_radiant = follower.phis.iter().map(|phi| phi/180.0 * PI).collect::<Vec<_>>();

        let min_radius = follower.radius.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
        let ideal_stroke = follower.radius.iter().map(|r| r-min_radius).collect::<Vec<_>>();
        let (velocities, accelerations) = helper_functions::calc_velocities_accelerations(&follower.radius, &follower.phis, rpm);

        let movement_forces_y = calc_force_y(&cam_type, mass, spring_rate, spring_pretension, gravity_value, &follower.radius, &accelerations);
        let process_forces_y = process_forces.to_vec();
        let forces_y = movement_forces_y.iter()
            .zip(process_forces_y)
            .map(|(f_m, f_p)| f_m+f_p)
            .collect::<Vec<_>>();

        let vec_y_force = phis_radiant.iter().map(|phi|
            match cam_type {
                CamType::PlanarDisc => [-phi.cos(), -phi.sin(), 0.0],
                CamType::PlanarGroove(_) => [-phi.cos(), -phi.sin(), 0.0],
                CamType::CylindricBead(_) => [0.0, 0.0, -1.0],
                CamType::CylindricGroove(_) => [0.0, 0.0, -1.0],
            }).collect::<Vec<_>>();
        
        // Convert the r(phi) to [x,y,z]
        let xyz_follower = helper_functions::polar_2_cartesian_3d(&follower.radius, &phis_radiant, &cam_type);

        // Gradient of the follower curve in 3d cartesian coordinates
        let d_xyz_follower = helper_functions::gradient_wrapping_3d(&xyz_follower);

        // Helper vector to build the normal vector in 3d (we need 2 vectors to define a plane for the normal vector)
        let normal_vec_helper = helper_functions::normal_vec_helper_3d(&cam_type, n_elements);

        // Build the normal vector with the cross product
        let normal_vec = helper_functions::cross_product_3d(&d_xyz_follower, &normal_vec_helper, true);
        
        // Build the front disc
        let xyz_disc = xyz_follower.iter()
            .zip(&normal_vec)
            .map(|(x_i, n_i)| {
                [x_i[0] + follower.radius_follower*n_i[0],
                x_i[1] + follower.radius_follower*n_i[1],
                x_i[2] + follower.radius_follower*n_i[2]]
            }).collect::<Vec<_>>();
        
        // Build the secondary disc for the cylinder groove cam
        if let CamType::CylindricGroove(ref mut cylinder) = cam_type {
            let back_disc = xyz_follower.iter()
                .zip(&normal_vec)
                .map(|(x_i, n_i)| {
                    [x_i[0] - (follower.radius_follower+cylinder.groove_play)*n_i[0],
                    x_i[1] - (follower.radius_follower+cylinder.groove_play)*n_i[1],
                    x_i[2] - (follower.radius_follower+cylinder.groove_play)*n_i[2]]
                }).collect::<Vec<_>>();
            cylinder.second_disc = back_disc;
        }
        // Build the secondary disc for the cylindric bead cam
        if let CamType::CylindricBead(ref mut cylinder) = cam_type {
            let back_disc = xyz_follower.iter()
                .zip(&normal_vec)
                .map(|(x_i, n_i)| {
                    [x_i[0] - (follower.radius_follower)*n_i[0],
                    x_i[1] - (follower.radius_follower)*n_i[1],
                    x_i[2] - (follower.radius_follower)*n_i[2] - cylinder.min_bead_thickness - 2.0*follower.radius_follower]
                }).collect::<Vec<_>>();
            cylinder.second_disc = back_disc;
        }
        // Build the secondary disc for the planar groove
        if let CamType::PlanarGroove(ref mut groove) = cam_type {
            let back_disc = xyz_follower.iter()
                .zip(&normal_vec)
                .map(|(x_i, n_i)| {
                    [x_i[0] - (follower.radius_follower+groove.groove_play)*n_i[0],
                    x_i[1] - (follower.radius_follower+groove.groove_play)*n_i[1],
                    x_i[2] - (follower.radius_follower+groove.groove_play)*n_i[2]]
                }).collect::<Vec<_>>();
            groove.second_disc = back_disc;
        }
        
        // Calculate the curvatures
        let curvature_disc = helper_functions::calc_curvature_ers_3d(&xyz_disc, None, &cam_type);
        let curvature_total = helper_functions::calc_curvature_ers_3d(&xyz_disc, Some(follower.radius_follower), &cam_type);
        let mut cam_disc_feasible = CamFeasibility::OK;
        // Check if the curvatures are all feasible
        if curvature_total.iter().any(|r| *r<=0.001) {
            cam_disc_feasible = CamFeasibility::Undercut;
        }

        // Calculate the contact angles
        let phis_contact = helper_functions::angles_3d(&vec_y_force, &normal_vec);

        // Normal forces based on the angle between F_y and normal vector to the disc
        let forces_normal = forces_y.iter().zip(&phis_contact).map(|(f_y, phi)| *f_y/phi.cos()).collect::<Vec<_>>();

        // torque = F_xyz x r_xyz
        let norm_force_vec = normal_vec.clone().iter_mut().zip(&forces_normal)
            .map(|(v, f)| {
                for v_i in v.iter_mut() {
                    *v_i *= *f;
                }
                *v
            }).collect::<Vec<_>>();
        let torque = helper_functions::cross_product_3d(&xyz_disc, &norm_force_vec, false);
        let torque = torque.into_iter().map(|[x, y, z]| z).collect::<Vec<_>>();

        let bearing_forces_axial = norm_force_vec.iter()
            .map(|[f_x, f_y, f_z]| *f_z)
            .collect::<Vec<_>>();
        let bearing_forces_radial = norm_force_vec.iter()
            .map(|[f_x, f_y, f_z]| (f_x.powi(2)+f_y.powi(2)).sqrt())
            .collect::<Vec<_>>();

        Self {
            cam_type: cam_type,
            follower: follower,
            process_forces: process_forces,
            disc_points_xyz: xyz_disc,
            normal_vec_follower: normal_vec,
            contact_angles: phis_contact,
            curvature_total: curvature_total,
            curvature_disc: curvature_disc,
            rpm: rpm,
            mass: mass,
            spring_rate: spring_rate,
            spring_pretension: spring_pretension,
            contact_length: 0.0,
            gravity: gravity,
            cam_disc_feasible: cam_disc_feasible,
            index: 0,
            animation_speed: 20.0,
            max_hertz: 0.0,
            c_stat: 0.0,
            c_dyn: 0.0,
            p_equiv: 0.0,
            l_10: 0.0,
            l_10_h: 0.0,

            ideal_stroke: ideal_stroke,
            ideal_velocities: velocities,
            ideal_accelerations: accelerations,
            ideal_force_y: forces_y,
            ideal_force_normal: forces_normal,
            ideal_torque: torque,
            ideal_hertz_pressure: vec![0.0; n_elements],
            ideal_bearing_forces_axial: bearing_forces_axial,
            ideal_bearing_forces_radial: bearing_forces_radial,
            ideal_ns_follower: vec![0.0; n_elements],

            real_positions: vec![0.0; n_elements],
            real_stroke: vec![0.0; n_elements],
            real_velocities: vec![0.0; n_elements],
            real_accelerations: vec![0.0; n_elements],
            real_force_y: vec![0.0; n_elements],
            real_force_normal: vec![0.0; n_elements],
            real_torque: vec![0.0; n_elements],
            real_hertz_pressure: vec![0.0; n_elements],
        }
    }

    fn calc_durability_metrics(&mut self, contact_length: f64, max_hertz: f64, c_stat: f64, c_dyn: f64) -> () {

        // Hertzian pressure
        let hertz_pressure = self.ideal_force_normal.iter().zip(&self.curvature_total)
            .map(|(f, r)| {
                if *f >= 0.0 { (f/(2.0*PI*r*contact_length*1_000_000.0) * (210_000.0/((1.0_f64-0.3).powi(2)))).sqrt()}
                else { 0.0 }
            }).collect::<Vec<_>>();
        if (hertz_pressure.iter().any(|p| *p>=max_hertz)) & (self.cam_disc_feasible != CamFeasibility::Undercut) {
            self.cam_disc_feasible = CamFeasibility::HertzPressure;
        }

        // Equivalent Load p_equiv
        let omega_disc = (self.rpm/60.0)*360.0; // °/s
        let d_t = omega_disc/self.follower.phis.len() as f64;
        let ds = helper_functions::ds_wrapping(&self.disc_points_xyz);
        let (ns_follower, p): (Vec<f64>, Vec<(f64, f64)>) = self.ideal_force_normal.iter().zip(ds.iter()).map(|(f_n, s)| {
                // calc rpm of the follower based on the length of the disc element s
                let s_dot_disc = s*omega_disc/self.follower.accuracy; // m/s
                let n_follower = s_dot_disc/(2.0*PI*self.follower.radius_follower) * 60.0; // 1/min
                let nominator = n_follower * f_n.powf(10.0/3.0); 
                let denominator = n_follower;
                (n_follower, (nominator, denominator))
            }).unzip();

        let (nom, denom): (Vec<f64>, Vec<f64>) = p.into_iter().unzip();
        let nom = helper_functions::trapz(&nom, None, Some(d_t));
        let denom = helper_functions::trapz(&denom, None, Some(d_t));
        let mut p_equiv = (nom/denom).powf(3.0/10.0);
        match self.cam_type {
            CamType::PlanarDisc => (),
            CamType::PlanarGroove(_) => (),
            CamType::CylindricBead(_) => p_equiv /= 2.0, // see master's thesis Daniel Grum, division by 2 if two follower bearings
            CamType::CylindricGroove(_) => (),
        }

        // L10 & L10_h
        let l_10 = (c_dyn/p_equiv).powf(10.0/3.0);
        let n_mean = ns_follower.iter().sum::<f64>()/(ns_follower.len() as f64);
        let l_10_h = 16666.0/n_mean * l_10;

        self.contact_length = contact_length;
        self.max_hertz = max_hertz;
        self.c_stat = c_stat;
        self.c_dyn = c_dyn;
        self.p_equiv = p_equiv;
        self.ideal_hertz_pressure = hertz_pressure;
        self.ideal_ns_follower = ns_follower;
        self.l_10 = l_10;
        self.l_10_h = l_10_h;
    }

    pub fn update_follower_position(&mut self, index: usize) -> () {
        // time step size dt based on angular velocity in °/s and step size
        // e.g. if accuray = 1.0° and rpm = 300 1/min
        // dt = 1° / (300 1/min / 60 s/min * 360°) = 1° / (300 * 6)
        let dt = self.follower.accuracy/(self.rpm*6.0);
        
        // ideal_x, _v, _a of the theoretical follower curve
        let ideal_x = self.follower.radius[index];
        let ideal_v = self.ideal_velocities[index];
        let ideal_a = self.ideal_accelerations[index];

        // true x, v, a at the given index (known from previous calculation step)
        let mut x_true = self.real_positions[index];
        let mut v_true = self.real_velocities[index];
        let mut a_true = self.real_accelerations[index];

        if (index==0) & (x_true==0.0) & (v_true==0.0) & (a_true==0.0) {
            x_true = ideal_x;
            self.real_positions[index] = x_true;
            v_true = ideal_v;
            self.real_velocities[index] = v_true;
            a_true = ideal_a;
            self.real_accelerations[index] = a_true;
        }

        // Force on the follower is spring force, gravity and process force  --> FORCE BY THE CAM DISC IS NOT "really" ACTING
        // ideal_force_y = ideal_force_acc + ideal_force_spring + gravity + process_force
        // --> true force_y = ideal_force_y + additional_force_spring - ideal_force_acc
        // Revert the sign to have force in proper direction
        let f_follower = - (self.ideal_force_y[index] + (x_true-ideal_x)*self.spring_rate - ideal_a*self.mass);
        let a_free = f_follower/self.mass;
        // Theoretical force of the cam disc
        let f_disc = ideal_a * self.mass;


        let mut x_next = 0.0;
        let mut v_next = 0.0;
        let mut a_next = 0.0;
        let mut contact_disc = true;
        let mut contact_disc_second = false;
        let groove = match &self.cam_type {
            CamType::PlanarDisc => None,
            CamType::PlanarGroove(groove) => Some(groove.groove_play),
            CamType::CylindricBead(_) => None,
            CamType::CylindricGroove(cylinder) => Some(cylinder.groove_play),
        };

        // If the "true" calculated position of the follower (expressed in radius) is <= the theoretical position we know there must be contact
        if x_true <= ideal_x {
            contact_disc = true;
            contact_disc_second = false;

            // If true velocity of the follower < theoretical velocity and the follower is touching, we know there is elastic impact
            if v_true < ideal_v-0.01 {
                let v_diff = 0.8*(v_true - ideal_v);
                x_next = ideal_x + (ideal_v - v_diff)*dt;
                v_next = ideal_v - v_diff;
                a_next = 0.0; //roughly 0.5ms impact time estimation, i don't konw
                self.real_accelerations[index] = -v_diff/0.00055
            }

            // If the force of the follower < theoretical disc force, the follower stays on the disc
            else if f_follower < f_disc {
                x_next = *self.follower.radius.get(index+1).unwrap_or_else(|| &self.follower.radius[0]);
                v_next = *self.ideal_velocities.get(index+1).unwrap_or_else(|| &self.ideal_velocities[0]);
                a_next = *self.ideal_accelerations.get(index+1).unwrap_or_else(|| &self.ideal_accelerations[0]);
            }
            // Last case: Should basically never be triggered?
            else {
                contact_disc = false;
                contact_disc_second = false;
                x_next = x_true + v_true*dt + 0.5*a_free*(dt.powi(2));
                v_next = v_true + a_free*dt;
                a_next = 0.0; 
            }
        }

        // Contact for groove cam front curve
        else if let Some(groove) = groove {
            if x_true >= (ideal_x + groove) {
                contact_disc = false;
                contact_disc_second = true;
                if v_true > ideal_v+0.01 {
                    let v_diff = 0.8*(v_true - ideal_v);
                    x_next = x_true + (ideal_v - v_diff)*dt;
                    v_next = ideal_v - v_diff;
                    a_next = 0.0; //roughly 0.5ms impact time estimation, i don't konw
                    self.real_accelerations[index] = -v_diff/0.00055
                }
                else if f_follower > f_disc {
                    x_next = *self.follower.radius.get(index+1).unwrap_or_else(|| &self.follower.radius[0]) + groove;
                    v_next = *self.ideal_velocities.get(index+1).unwrap_or_else(|| &self.ideal_velocities[0]);
                    a_next = *self.ideal_accelerations.get(index+1).unwrap_or_else(|| &self.ideal_accelerations[0]);
                }
                else {
                    x_next = x_true + v_true*dt + 0.5*a_free*(dt.powi(2));
                    v_next = v_true + a_free*dt;
                    a_next = 0.0;
                    if self.cam_disc_feasible != CamFeasibility::Undercut {
                        self.cam_disc_feasible = CamFeasibility::FollowerLiftOff;
                    }  
                }
            }
            else {
                contact_disc = false;
                contact_disc_second = false;
                x_next = x_true + v_true*dt + 0.5*a_free*(dt.powi(2));
                v_next = v_true + a_free*dt;
                a_next = 0.0;
                if self.cam_disc_feasible != CamFeasibility::Undercut {
                    self.cam_disc_feasible = CamFeasibility::FollowerLiftOff;
                }  
            }
        }

        // If the follower has no contact just standard physics x = x + v*t + 1/2*a*t²
        else {
            contact_disc_second = false;
            contact_disc = false;
            x_next = x_true + v_true*dt + 0.5*a_free*(dt.powi(2));
            v_next = v_true + a_free*dt;
            a_next = 0.0;
            if self.cam_disc_feasible != CamFeasibility::Undercut {
                self.cam_disc_feasible = CamFeasibility::FollowerLiftOff;
            }  
        }

        // Modify the vectors for position, velocity and acceleration
        // If index+1 > 360° we have to wrap around back to 0
        let index_next = match self.real_positions.get(index+1) {
            Some(t) => index+1,
            None => 0,
        };
        self.real_positions[index_next] = x_next;
        self.real_stroke[index_next] = self.ideal_stroke[index_next] + x_next - self.follower.radius[index_next];
        self.real_velocities[index_next] = v_next;
        self.real_accelerations[index_next] = a_next;

        self.real_force_y[index] = -f_follower + self.real_accelerations[index]*self.mass;
        

        if contact_disc {  
            self.real_force_normal[index] = self.real_force_y[index]/(self.contact_angles[index].cos());  
            self.real_hertz_pressure[index] = (self.real_force_normal[index]/(2.0*PI*self.curvature_total[index]*self.contact_length*1_000_000.0) * 210_000.0/(1.0_f64-0.3).powi(2)).sqrt();
            let norm_force_vec = self.normal_vec_follower[index]
                .map(|x| x*self.real_force_normal[index]);
            self.real_torque[index] = helper_functions::cross_product_3d(&[self.disc_points_xyz[index]], &[norm_force_vec], false)[0][2];
        }
        else if contact_disc_second {
            self.real_force_normal[index] = self.real_force_y[index]/(self.contact_angles[index].cos());
            if let CamType::CylindricGroove(cylinder) = &self.cam_type {
                let norm_force_vec = self.normal_vec_follower[index]
                    .map(|x| x*self.real_force_normal[index]);  
                self.real_torque[index] = helper_functions::cross_product_3d(&[cylinder.second_disc[index]], &[norm_force_vec], false)[0][2];
            }
            if let CamType::PlanarGroove(groove) = &self.cam_type {
                let norm_force_vec = self.normal_vec_follower[index]
                    .map(|x| x*self.real_force_normal[index]);
                self.real_torque[index] = helper_functions::cross_product_3d(&[groove.second_disc[index]], &[norm_force_vec], false)[0][2];
            }
        }
        else {
            self.real_force_normal[index] = 0.0;
            self.real_hertz_pressure[index] = 0.0;
            self.real_torque[index] = 0.0;
        }
    }


    pub fn update_index(&mut self) -> () {
        let n_elements = self.follower.phis.len();
        let index_next = match self.animation_speed {
            0.0 => n_elements-1, // simulate everything at once if animation speed 0
            _ => self.index + ((self.rpm/3600.0)*n_elements as f64 * self.animation_speed/100.0) as usize // next step depends on animation speed
        };

        // Simulate only the next number of steps
        for i in self.index..index_next {
            self.update_follower_position(i % n_elements);
        };
        self.index = index_next % n_elements;

    }


    pub fn update_sections(&mut self) -> () {
        self.follower.sort_sections();

        // New follower and its dynamics
        let mut follower = CamFollower::new(self.follower.radius_follower, self.follower.accuracy);

        for (i, section) in self.follower.sections.iter().enumerate() {
            let lambda = 0.5;
            follower.add_section(section.movement_type, section.incline, section.deg_start, section.deg_end, lambda);
        }

        let mut cam = Self::new(self.cam_type.clone(), follower, self.process_forces.clone(), self.rpm, self.mass, self.spring_rate, self.spring_pretension, self.gravity);
        cam.animation_speed = self.animation_speed;
        cam.c_dyn = self.c_dyn;
        cam.c_stat = self.c_stat;

        // Calculate all points up to current index for smooth transition
        for i in 0..=self.index {
            cam.update_follower_position(i)
        };
        cam.calc_durability_metrics(self.contact_length, self.max_hertz, self.c_stat, self.c_dyn);
        *self = cam;
    }
}

/// Calculate the force in pure radial direction
pub fn calc_force_y(cam_type: &CamType, mass: f64, spring_rate: f64, spring_pretension: f64, gravity: f64, radius: &[f64], accelerations: &[f64]) -> Vec<f64> {
    assert_eq!(radius.len(), accelerations.len(), "Arrays must have same number of elements");
    let mut fs_y: Vec<f64> = Vec::with_capacity(radius.len());
    for (i, r) in radius.iter().enumerate() {
        let f_spring = match cam_type {
            CamType::PlanarDisc => (*r - radius[0]) * spring_rate + spring_pretension,
            CamType::PlanarGroove(groove) => (*r - radius[0]) * spring_rate + spring_pretension,
            CamType::CylindricBead(cylinder) => spring_pretension,
            CamType::CylindricGroove(cylinder) => (*r - radius[0]) * spring_rate + spring_pretension,
        };
        let f_a = mass * (accelerations[i]+gravity); //input accelerations are in mm/s²
        fs_y.push(f_spring+f_a);
    }
    fs_y
}

impl Default for CamSystem {
    fn default() -> Self {
        let mut follower = CamFollower::new(0.012, 0.1);
        follower.add_section(MovementType::Rest(MovementLawsRest::Rest), 0.025, 0.0, 150.0, 0.5).unwrap();
        follower.add_section(MovementType::RestRest(MovementLawsRestRest::SineModified), 0.006, 150.0, 200.0, 0.5).unwrap();
        follower.add_section(MovementType::RestRest(MovementLawsRestRest::SineModified), -0.006, 200.0, 250.0, 0.5).unwrap();
        follower.add_section(MovementType::Rest(MovementLawsRest::Rest), 0.025, 250.0, 360.0, 0.5).unwrap();

        let process_forces = ProcessForces::new(follower.accuracy);
        let mut disc = Self::new(CamType::PlanarDisc, follower, process_forces, 60.0, 1.0, 15000.0, 50.0, true);
        disc.calc_durability_metrics(0.01, 1500.0, 4100.0, 3500.0);
        disc
    }
}
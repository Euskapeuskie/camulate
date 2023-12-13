use std::{hash::Hash, fmt::Display};
use serde::{Serialize, Deserialize};

use super::movement_laws_rest::MovementLawsRest;
use super::movement_laws_rest_rest::MovementLawsRestRest;
use super::movement_laws_rest_return::{MovementLawsRestReturn, MovementLawsReturnRest};

/// Struct for building the follower movement according VDI 2143
/// 
/// * `phis`: Vector of the angles in [°], automatically calculated based on accuracy
/// * `radius`: Vector of the calculated radius based on the sections that were added
/// * `radius_follower`: Radius of the follower bearing
/// * `accuracy`: Accuracy in [°], typically something like 0.01. Big influence on computational effort
/// * `sections`: Vector of the sections that have been added
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CamFollower {
    pub phis: Vec<f64>,
    pub radius: Vec<f64>,
    pub radius_follower: f64,
    pub accuracy: f64,
    pub sections: Vec<Section>,
}


/// Movement types based on VDI 2143
/// * `Rest`: No stroke at all
/// * `RestRest`: Rest to rest movement
/// * `RestReturn`: Rest to return movement, usually followed by a return to rest movement
/// * `ReturnRest`: Return to rest movement, usually after a rest to return movement
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Copy)]
pub enum MovementType {
    Rest(MovementLawsRest),
    RestRest(MovementLawsRestRest),
    RestReturn(MovementLawsRestReturn),
    ReturnRest(MovementLawsReturnRest),
}

impl Display for MovementType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Rest(_) => write!(f, "Rest"),
            Self::RestRest(_) => write!(f, "Rest to Rest"),
            Self::RestReturn(_) => write!(f, "Rest to Return"),
            Self::ReturnRest(_) => write!(f, "Return to Rest"),
        }
    }
}

/// Struct that holds the relevant data about a specific section of the cam follower
/// * `deg_start`: Start value in [°] of this sections
/// * `deg_end`: End value in [°] of this sections
/// * `movement_type`: 'MovementType` of this section
/// * `incline`: Stroke in this section
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Section {
    pub deg_start: f64,
    pub deg_end: f64,
    pub movement_type: MovementType,
    pub incline: f64,
}

impl Section {
    pub fn new(deg_start: f64, deg_end: f64, movement_type: MovementType, incline: f64) -> Self {
        Section { deg_start: deg_start, deg_end: deg_end, movement_type: movement_type, incline: incline }
    }
}

impl Hash for Section {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        let a = self.deg_start as usize;
        let b = self.deg_end as usize;
        let c = format!("{:?}", self.movement_type);
        let d = self.incline as usize;

        a.hash(state);
        b.hash(state);
        c.hash(state);
        d.hash(state);
    }
}

impl CamFollower {

    /// Creates a new instance of a CamFollower
    /// 
    /// # Arguments
    /// * `radius_follower`: Radius of the follower bearing
    /// * `accuracy`: Accuracy in degrees, e.g. 0.1
    pub fn new(radius_follower: f64, accuracy: f64) -> Self {
        let n_steps = (360.0 / accuracy) as usize;
        let phis = (0..n_steps).map(|x| x as f64 * accuracy).collect::<Vec<f64>>();
        let n_elements = phis.len();

        CamFollower {
            phis: phis,
            radius: vec![0.0; n_elements],
            radius_follower: radius_follower,
            accuracy: accuracy,
            sections: vec![],
        }
    }

    /// Adds a section with the specified movement law to the profile of the cam follower. This function modifies `self.radius` in place.
    /// 
    /// # Arguments
    /// * `movement_law`: MovementLaw enumerator
    /// * `incline`: incline in mm from start to end. For constant radius optional
    /// (if no incline the value of the last position before deg_start will be taken as radius)
    /// * `deg_start`: start value in degrees for this section
    /// * `deg_end`: end value in degrees for this section
    /// 
    /// # Returns
    /// `None`
    pub fn add_section(&mut self, movement_law: MovementType, incline: f64, deg_start: f64, deg_end: f64) -> Result<(), String> {

        // Round degree inputs to the precision of the accuracy
        let deg_start = (deg_start / self.accuracy).round() * self.accuracy;
        let deg_end = (deg_end / self.accuracy).round() * self.accuracy;

        // Get indices of deg_start and deg_end
        let index_start = (deg_start / self.accuracy) as usize;
        let mut index_end = (deg_end / self.accuracy) as usize;
        // Reduce the final "wrapping value" of 360.0° by one index position to keep in bounds
        if index_end == self.phis.len() {
            index_end -= 1;
        }
        if index_start == index_end {
            return Err(String::from("Not enough elements to create the section, try increasing the angle range"))
        }

        let n_elements = index_end - index_start + 1;

        let last_element = *self.radius.get(index_start).ok_or_else(|| String::from("deg_start out of bounds"))?;

        let mut rs = match movement_law {
            MovementType::Rest(x) => x.calc_radius(incline-last_element+self.radius_follower, n_elements),
            MovementType::RestRest(x) => x.calc_radius(incline, n_elements),
            MovementType::RestReturn(x) => x.calc_radius(incline, n_elements, Some(0.5)),
            MovementType::ReturnRest(x) => x.calc_radius(incline, n_elements, Some(0.5)),
        };

        rs.iter_mut().for_each(|r| *r += last_element);

        // replace the existing radius with the calculated section and add the section
        if let Some(slice) = self.radius.get_mut(index_start..=index_end) {
            slice.copy_from_slice(&rs);
            self.sections.push(Section::new(deg_start, deg_end, movement_law, incline)); 
            return Ok(())
        }

        Err(String::from("deg_start or deg_end out of bounds, have to be between 0..360°"))
    }


    /// Sorts the sections of the follower by deg_start in ascending order
    /// Necessary to use before constructing a cam_system so that sections where only a stroke/incline
    /// is specified, we can find the radius of the previous section
    pub fn sort_sections(&mut self) -> () {
        self.sections.sort_by(|a, b| a.deg_start.total_cmp(&b.deg_start));
    }
}
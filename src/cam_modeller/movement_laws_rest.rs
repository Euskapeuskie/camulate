use std::fmt::Display;
use serde::{Serialize, Deserialize};


/// Rest Section
/// * for planar disc this means constant radius
/// * for cylindrical cams this means constant stroke
#[derive(Debug, Serialize, Deserialize, PartialEq, Clone, Copy)]
pub enum MovementLawsRest {
    Rest,
}


impl MovementLawsRest {

    /// * `incline`: Incline in mm of the section or absolute radius for ConstantRadius
    /// * `n_elements`: Number of elements for which the section shall be calculated
    pub fn calc_radius(&self, incline: f64, n_elements: usize) -> Vec<f64> {

        match self {
            MovementLawsRest::Rest => const_radius(incline, n_elements),
        }
    }
}


impl Display for MovementLawsRest {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MovementLawsRest::Rest => write!(f, "Rest"),
        }
    }
}


/// Section with constant radius
/// 
/// # Arguments
/// * `radius`: radius of the constant section in mm
/// * `n_elements`: Number of elements to generate this section for
/// 
/// # Returns
/// Vector of the radius elements from index_start..index_end
fn const_radius(radius: f64, n_elements: usize) -> Vec<f64> {
    vec![radius; n_elements]
}
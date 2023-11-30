use std::{f64::consts::PI, fmt::Display};
use serde::{Serialize, Deserialize};


/// Different movement laws for rest to rest movement according to VDI 2143
/// Note that not all of these are jerk free
/// * `ConstantRadius`: Simply a circle with constant radius
/// * `Linear`: Linear slope from start to end
/// * ...
/// * `__PlaceHolder`: This enum variant exists for properly displaying in the GUI
#[derive(Debug, Serialize, Deserialize, PartialEq, Clone, Copy)]
pub enum MovementLawsRestRest {
    Linear,
    ParabolaQuadratic,
    SineSimple,
    Poly5,
    SineInclined,
    AccelerationTrapezoid,
    SineModified,
    __PlaceHolder,
}


impl MovementLawsRestRest {

    /// * `incline`: Incline in mm of the section or absolute radius for ConstantRadius
    /// * `n_elements`: Number of elements for which the section shall be calculated
    pub fn calc_radius(&self, incline: f64, n_elements: usize) -> Vec<f64> {

        match self {
            MovementLawsRestRest::Linear => linear(incline, n_elements),
            MovementLawsRestRest::ParabolaQuadratic => parabola_quadratic(incline, n_elements),
            MovementLawsRestRest::SineSimple => sine_simple(incline, n_elements),
            MovementLawsRestRest::Poly5 => poly_5(incline, n_elements),
            MovementLawsRestRest::SineInclined => sine_inclined(incline, n_elements),
            MovementLawsRestRest::AccelerationTrapezoid => acceleration_trapezoid(incline, n_elements),
            MovementLawsRestRest::SineModified => modified_sine(incline, n_elements),
            MovementLawsRestRest::__PlaceHolder => vec![0.0; n_elements],
        }
    }
}


impl Display for MovementLawsRestRest {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MovementLawsRestRest::Linear => write!(f, "Linear"),
            MovementLawsRestRest::ParabolaQuadratic => write!(f, "Parab. quadratic"),
            MovementLawsRestRest::SineSimple => write!(f, "Sine simple"),
            MovementLawsRestRest::Poly5 => write!(f, "Poly5"),
            MovementLawsRestRest::SineInclined => write!(f, "Sine inclined"),
            MovementLawsRestRest::AccelerationTrapezoid => write!(f, "Accel. Trapezoid"),
            MovementLawsRestRest::SineModified => write!(f, "Sine modified"),
            MovementLawsRestRest::__PlaceHolder => write!(f, ""),
        }
    }
}


/// Section with linear slope
/// This movement type is NOT jerk free!
/// 
/// # Arguments
/// * `incline`: incline of the section in mm
/// * `n_elements`: Number of elements to generate this section for
/// 
/// # Returns
/// Vector of the radius elements from index_start..index_end
fn linear(incline: f64, n_elements: usize) -> Vec<f64> {
    let mut zs = (0..n_elements).map(|x| x as f64/(n_elements-1) as f64).collect::<Vec<_>>();
    for z in zs.iter_mut() {
        *z *= incline;
    };
    zs
}


/// Section of quadratic parabola
/// This movement type is NOT jerk free!
/// 
/// # Arguments
/// * `incline`: incline of the section in mm
/// * `n_elements`: Number of elements to generate this section for
/// 
/// # Returns
/// Vector of the radius elements from index_start..index_end
fn parabola_quadratic(incline: f64, n_elements: usize) -> Vec<f64> {
    let zs = (0..n_elements).map(|x| x as f64/(n_elements-1) as f64).collect::<Vec<_>>();

    fn f1(z: &f64) -> f64 {
        2.0 * (*z).powi(2)
    }

    fn f2(z: &f64) -> f64 {
        1.0 - 2.0*(*z-1.0).powi(2)
    }

    let rs = zs.iter().map(|z| {
        if *z <= 0.5 { incline * f1(z) }
        else { incline * f2(z) }
    }).collect::<Vec<_>>();
    rs
}


/// Section of standard sine
/// This movement type is NOT jerk free, but might be use for low torque
/// 
/// # Arguments
/// * `incline`: incline of the section in mm
/// * `n_elements`: Number of elements to generate this section for
/// 
/// # Returns
/// Vector of the radius elements from index_start..index_end
fn sine_simple(incline: f64, n_elements: usize) -> Vec<f64> {
    let zs = (0..n_elements).map(|x| x as f64/(n_elements-1) as f64).collect::<Vec<_>>();
    let rs = zs.iter().map(|z| { incline * (0.5 * (1.0 - (PI*(*z)).cos()))}).collect::<Vec<_>>();
    rs
}


/// Section of polynomial of 5th order
/// 
/// # Arguments
/// * `incline`: incline (positive) or decline (negative) in mm
/// * `n_elements`: number of elements to generate this section for
/// 
/// # Returns
/// Vector of the radius elements from index_start..index_end
fn poly_5(incline: f64, n_elements: usize) -> Vec<f64> {
    let zs = (0..n_elements).map(|x| x as f64/(n_elements-1) as f64).collect::<Vec<_>>();

    // poly-5 rest-to-rest has only one section
    fn f1(z: &f64) -> f64 {
        let r = 10.0*(z.powi(3)) - 15.0*(z.powi(4)) + 6.0*(z.powi(5));
        r
    }

    let rs = zs.iter().map(|z| incline*f1(z)).collect::<Vec<_>>();
    rs
}


/// Section of inclined sine
/// 
/// # Arguments
/// * `incline`: incline (positive) or decline (negative) in mm
/// * `n_elements`: number of elements to generate this section for
/// 
/// # Returns
/// Vector of the radius elements from index_start..index_end
fn sine_inclined(incline: f64, n_elements: usize) -> Vec<f64> {
    let zs = (0..n_elements).map(|x| x as f64/(n_elements-1) as f64).collect::<Vec<_>>();
    let rs = zs.iter().map(|z| incline * ((*z) - 1.0/(2.0*PI) * (2.0*PI*z).sin())).collect::<Vec<_>>();
    rs
}


/// Section of modified acceleration trapezoid
/// 
/// # Arguments
/// * `incline`: incline (positive) or decline (negative) in mm
/// * `n_elements`: number of elements to generate this section for
/// 
/// # Returns
/// Vector of the radius elements from index_start..index_end
fn acceleration_trapezoid(incline: f64, n_elements: usize) -> Vec<f64> {
    let zs = (0..n_elements).map(|x| x as f64/(n_elements-1) as f64).collect::<Vec<_>>();

    fn f1(z: &f64) -> f64 {
        2.0/(PI+2.0) * (z - (4.0*PI*z).sin()/(4.0*PI))
    }
    fn f2(z: &f64) -> f64 {
        (1.0 / (PI+2.0)) * (4.0*PI*z.powi(2) + (2.0 - PI)*z + (PI.powi(2)-8.0)/(16.0*PI))
    }
    fn f3(z: &f64) -> f64 {
        (2.0/(PI+2.0)) * ((PI+1.0)*z - 1.0/(4.0*PI) * (4.0*PI*(z-0.25)).sin() - PI/4.0)
    }
    fn f4(z: &f64) -> f64 {
        let diff = 1.0-z;
        1.0 - f3(&diff)
    }
    fn f5(z: &f64) -> f64 {
        let diff = 1.0-z;
        1.0 - f2(&diff)
    }
    fn f6(z: &f64) -> f64 {
        let diff = 1.0-z;
        1.0 - f1(&diff)
    }

    let rs = zs.iter().map(|z| {
        if *z <= 0.125 {
            incline * f1(z)
        }
        else if *z <= 0.375 {
            incline * f2(z)
        }
        else if *z <= 0.5 {
            incline * f3(z)
        }
        else if *z <= 0.625 {
            incline * f4(z)
        }
        else if *z <= 0.875 {
            incline * f5(z)
        }
        else {
            incline * f6(z)
        }
    }).collect::<Vec<_>>();
    rs
}


/// Section for modified sine, rest-to-rest
/// 
/// # Arguments
/// * `incline`: incline (positive) or decline (negative) in mm
/// * `n_elements`: number of elements to generate this section for
/// 
/// # Returns
/// Vector of the radius elements from index_start..index_end
fn modified_sine(incline: f64, n_elements: usize) -> Vec<f64> {
    let zs = (0..n_elements).map(|x| x as f64/(n_elements-1) as f64).collect::<Vec<_>>();

    // Functions for the different sections
    // f1: Between 0..1/8
    fn f1(z: &f64) -> f64 {
        let r = PI/(4.0+PI) * (z - 1.0/(4.0*PI) * (4.0*PI*z).sin());
        r
    }
    // f2: Between 1/8..7/8
    fn f2(z: &f64) -> f64 {
        let r = PI/(4.0+PI) * (2.0/PI + z - (9.0/(4.0*PI))*((PI/3.0)*(1.0 + 4.0*z)).sin());
        r
    }
    // f3: Between 7/8..8/8
    fn f3(z: &f64) -> f64 {
        let r = PI/(4.0+PI) * (4.0/PI + z - 1.0/(4.0*PI)*(4.0*PI*z).sin());
        r
    }

    let rs = zs.iter().map(|z| {
        if *z <= (1.0/8.0) {
            incline*f1(z)
        }
        else if (*z >= (1.0/8.0)) & (*z<= (7.0/8.0)) {
            incline*f2(z)
        }
        else {
            incline*f3(z)
        }
    }).collect();
    rs
}
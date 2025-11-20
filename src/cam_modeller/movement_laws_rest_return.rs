use std::{f64::consts::PI, fmt::Display};
use serde::{Serialize, Deserialize};


#[derive(Debug, Serialize, Deserialize, PartialEq, Clone, Copy)]
pub enum MovementLawsRestReturn {
    AccelerationTrapezoid,
    HarmonicCombination,
    __PlaceHolder,
}

#[derive(Debug, Serialize, Deserialize, PartialEq, Clone, Copy)]
pub enum MovementLawsReturnRest {
    AccelerationTrapezoid,
    HarmonicCombination,
    __PlaceHolder,
}


// --- Movement Laws REST TO RETURN ---
impl MovementLawsRestReturn {

    /// * `incline`: Incline of the section in mm
    /// * `n_elements`: Number of elements to calculate the section for
    /// * `lambda`: Wendepunktverschiebung
    pub fn calc_radius(&self, incline: f64, n_elements: usize, lambda: f64) -> Vec<f64> {
        match self {
            Self::AccelerationTrapezoid => acceleration_trapezoid_rest_return(incline, n_elements, lambda),
            Self::HarmonicCombination => harmonic_combination_rest_return(incline, n_elements, lambda),
            Self::__PlaceHolder => vec![0.0; n_elements],
        }
    }
}

impl Display for MovementLawsRestReturn {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::AccelerationTrapezoid => write!(f, "Accel. trapezoid"),
            Self::HarmonicCombination => write!(f, "Harmonic comb."),
            Self::__PlaceHolder => write!(f, ""),
        }
    }
}


// --- Movement Laws RETURN TO REST ---
impl MovementLawsReturnRest {
    pub fn calc_radius(&self, incline: f64, n_elements: usize, lambda: f64) -> Vec<f64> {
        match self {
            Self::AccelerationTrapezoid => acceleration_trapezoid_return_rest(incline, n_elements, lambda),
            Self::HarmonicCombination => harmonic_combination_return_rest(incline, n_elements, lambda),
            Self::__PlaceHolder => vec![0.0; n_elements],
        }
    }
}

impl Display for MovementLawsReturnRest {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::AccelerationTrapezoid => write!(f, "Accel. trapezoid"),
            Self::HarmonicCombination => write!(f, "Harmonic comb."),
            Self::__PlaceHolder => write!(f, ""),
        }
    }
}


fn acceleration_trapezoid_rest_return(incline: f64, n_elements: usize, lambda: f64) -> Vec<f64> {
    let mut zs = (0..n_elements).map(|x| x as f64/(n_elements-1) as f64).collect::<Vec<_>>();

    let c_a_star = (-32.0*PI.powi(2)) / (15.0*PI.powi(2) + 8.0 - 2.0*lambda*(9.0*PI.powi(2) - 4.0*PI + 8.0) + lambda.powi(2)*(3.0*PI.powi(2) - 8.0*PI + 8.0));
    let c_a = - (1.0 - lambda)/lambda * (3.0*PI+2.0)/(2.0*(2.0+PI)) * c_a_star;

    for z in zs.iter_mut() {
        if *z <= lambda/4.0 {
            let p1 = 2.0*PI*(*z)/lambda;
            let f1 = lambda/(2.0*PI) * c_a * (*z - lambda/(2.0*PI) * p1.sin());
            *z = f1;
        }
        else if *z <= (3.0*lambda)/4.0 {
            let f2 = c_a * (((*z - lambda/4.0).powi(2))/2.0 + lambda/(2.0*PI)*(*z) - lambda.powi(2)/(4.0*(PI.powi(2))));
            *z = f2;
        }
        else if *z <= lambda {
            let p3 = 2.0*PI/lambda * (*z - 3.0*lambda/4.0);
            let f3 = lambda/(2.0*PI) * c_a * ((1.0+PI)*(*z) - PI*lambda/2.0 - lambda/(2.0*PI) * p3.cos());
            *z = f3;
        }
        else if *z <= (3.0*lambda + 1.0)/4.0 {
            let p4 = 2.0*PI/(1.0-lambda) * (*z - lambda);
            let f4 = (1.0 - lambda)/(2.0*PI) * c_a_star * ((3.0*PI-2.0)/4.0 * lambda - 3.0*PI/2.0*(*z) - (1.0-lambda)/(2.0*PI) * p4.sin());
            *z = f4;
        }
        else {
            let f5 = c_a_star * ((*z-1.0).powi(2)/2.0 - 1.0/(c_a_star.abs()));
            *z = f5;
        }
    };

    let _ = zs.iter_mut().for_each(|z| *z *= incline);
    zs
}


fn harmonic_combination_rest_return(incline: f64, n_elements: usize, lambda: f64) -> Vec<f64> {
    let mut zs = (0..n_elements).map(|x| x as f64/(n_elements-1) as f64).collect::<Vec<_>>();
    
    let c_a_star = (-2.0 * PI.powi(2)) / (8.0 - lambda*(12.0-PI) + lambda.powi(2) * (4.0-PI));
    let c_a = - (1.0-lambda)/lambda * c_a_star;

    for z in zs.iter_mut() {
        if *z <= lambda/4.0 {
            let p1 = 2.0*PI/lambda * (*z);
            let f1 = lambda/(2.0*PI) * c_a * (*z - lambda/(2.0*PI) * p1.sin());
            *z = f1;
        }
        else if *z <= lambda {
            let p2 = 2.0*PI/(3.0*lambda) * (*z - lambda/4.0);
            let f2 = (lambda/(2.0*PI)).powi(2) * c_a * (8.0 + 2.0*PI/lambda * (*z) - 9.0 * p2.cos());
            *z = f2;
        }
        else {
            let p3 = PI/(2.0*(1.0-lambda)) * (*z-lambda);
            let f3 = - (1.0-lambda)/PI * c_a_star * ((4.0*(1.0-lambda))/PI * p3.sin() + lambda/(2.0*PI)*(4.0+PI));
            *z = f3;
        }
    }

    let _ = zs.iter_mut().for_each(|z| *z *= incline);
    zs
}


fn acceleration_trapezoid_return_rest(incline: f64, n_elements: usize, lambda: f64) -> Vec<f64> {
    let mut zs = (0..n_elements).map(|x| 1.0 - (x as f64/(n_elements-1) as f64)).collect::<Vec<_>>();

    let c_a_star = (-32.0*PI.powi(2)) / (15.0*PI.powi(2) + 8.0 - 2.0*lambda*(9.0*PI.powi(2) - 4.0*PI + 8.0) + lambda.powi(2)*(3.0*PI.powi(2) - 8.0*PI + 8.0));
    let c_a = - (1.0 - lambda)/lambda * (3.0*PI+2.0)/(2.0*(2.0+PI)) * c_a_star;

    for z in zs.iter_mut() {
        if *z <= lambda/4.0 {
            let p1 = 2.0*PI*(*z)/lambda;
            let f1 = lambda/(2.0*PI) * c_a * (*z - lambda/(2.0*PI) * p1.sin());
            *z = 1.0-f1;
        }
        else if *z <= (3.0*lambda)/4.0 {
            let f2 = c_a * (((*z - lambda/4.0).powi(2))/2.0 + lambda/(2.0*PI)*(*z) - lambda.powi(2)/(4.0*(PI.powi(2))));
            *z = 1.0-f2;
        }
        else if *z <= lambda {
            let p3 = 2.0*PI/lambda * (*z - 3.0*lambda/4.0);
            let f3 = lambda/(2.0*PI) * c_a * ((1.0+PI)*(*z) - PI*lambda/2.0 - lambda/(2.0*PI) * p3.cos());
            *z = 1.0-f3;
        }
        else if *z <= (3.0*lambda + 1.0)/4.0 {
            let p4 = 2.0*PI/(1.0-lambda) * (*z - lambda);
            let f4 = (1.0 - lambda)/(2.0*PI) * c_a_star * ((3.0*PI-2.0)/4.0 * lambda - 3.0*PI/2.0*(*z) - (1.0-lambda)/(2.0*PI) * p4.sin());
            *z = 1.0-f4;
        }
        else {
            let f5 = c_a_star * ((*z-1.0).powi(2)/2.0 - 1.0/(c_a_star.abs()));
            *z = 1.0-f5;
        }
    };

    let _ = zs.iter_mut().for_each(|z| *z *= incline);
    zs
}


fn harmonic_combination_return_rest(incline: f64, n_elements: usize, lambda: f64) -> Vec<f64> {
    let mut zs = (0..n_elements).map(|x| 1.0 - (x as f64/(n_elements-1) as f64)).collect::<Vec<_>>();
    
    let c_a_star = (-2.0 * PI.powi(2)) / (8.0 - lambda*(12.0-PI) + lambda.powi(2) * (4.0-PI));
    let c_a = - (1.0-lambda)/lambda * c_a_star;

    for z in zs.iter_mut() {
        if *z <= lambda/4.0 {
            let p1 = 2.0*PI/lambda * (*z);
            let f1 = lambda/(2.0*PI) * c_a * (*z - lambda/(2.0*PI) * p1.sin());
            *z = 1.0-f1;
        }
        else if *z <= lambda {
            let p2 = 2.0*PI/(3.0*lambda) * (*z - lambda/4.0);
            let f2 = (lambda/(2.0*PI)).powi(2) * c_a * (8.0 + 2.0*PI/lambda * (*z) - 9.0 * p2.cos());
            *z = 1.0-f2;
        }
        else {
            let p3 = PI/(2.0*(1.0-lambda)) * (*z-lambda);
            let f3 = - (1.0-lambda)/PI * c_a_star * ((4.0*(1.0-lambda))/PI * p3.sin() + lambda/(2.0*PI)*(4.0+PI));
            *z = 1.0-f3;
        }
    }
    let _ = zs.iter_mut().for_each(|z| *z *= incline);
    zs
}


/// Function to calculate the optimized lambda value for a return rest section
/// following a rest-return section
/// 
/// # Inputs
/// * `a_return`: The normed acceleration of the rest-return section at its return position
/// 
/// # Returns
/// * `lambda`: The shift parameter for the following harmonic combination
pub fn optimize_lambda_return_rest_harmonic(a_return: f64) -> f64 {
    0.0

}
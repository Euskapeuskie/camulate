use std::f64::consts::PI;
use egui_plot::PlotPoints;

use super::cam_system::CamType;

/// Convert polar to cartesian coordiante system
/// 
/// # Inputs
/// * `phis`: vector of angles in degrees
/// * `radius`: vector of corresponding radius
/// 
/// # Returns
/// Vector of (x, y) pairs
pub fn polar_2_cartesian(phis: &[f64], radius: &[f64]) -> Vec<(f64, f64)> {
    assert!(phis.len() == radius.len(), "Input vectors must have same size");

    // Convert degrees to radiants
    let phis = phis
        .into_iter()
        .map(|phi| phi/180.0*PI)
        .collect::<Vec<_>>();

    // Calculate x,y pairs
    let x = phis
        .iter()
        .zip(radius.iter())
        .map(|(phi, r)| (r*phi.cos(), r*phi.sin()))
        .collect();
    x
}

pub fn polar_2_cartesian_3d(radius: &[f64], phis: &[f64], cam_type: &CamType) -> Vec<[f64;3]> {
    assert_eq!(radius.len(), phis.len(), "Input arrays must have same size");

    match cam_type {
        // PlanarDisc -> [x, y, 0]
        CamType::PlanarDisc => phis.iter().zip(radius).map(|(phi, r)| [r*phi.cos(), r*phi.sin(), 0.0]).collect(),
        CamType::PlanarGroove(groove) => phis.iter().zip(radius).map(|(phi, r)| [r*phi.cos(), r*phi.sin(), 0.0]).collect(),
        // CylndricCurve -> [x, y, r]
        CamType::CylindricBead(cylinder) => phis.iter().zip(radius).map(|(phi, r)| [cylinder.base_radius*phi.cos(), cylinder.base_radius*phi.sin(), *r]).collect(),
        CamType::CylindricGroove(cylinder) => phis.iter().zip(radius).map(|(phi, r)| [cylinder.base_radius*phi.cos(), cylinder.base_radius*phi.sin(), *r]).collect(),
    }
}

pub fn gradient_wrapping_3d(points: &[[f64;3]]) -> Vec<[f64;3]> {
    // Calculate the difference of 2 n_dimensional vectors
    fn _normed_vec_diff(a: &[f64;3], b: &[f64;3]) -> [f64;3] {
        let res = [b[0]-a[0], b[1]-a[1], b[2]-a[2]];
        res
    }

    let res = points.iter().enumerate()
        .map(|(i, point)| {
            let mut i_next = 0;
            let mut i_prev = 0;
            if i == 0 {
                i_prev = points.len()-1;
                i_next = i+1;
            }
            else if i == points.len()-1 {
                i_prev = i-1;
                i_next = 0;
            }
            else {
                i_next = i+1;
                i_prev = i-1;
            }
            _normed_vec_diff(&points[i_prev], &points[i_next])
        }).collect::<Vec<_>>();
    res
}

pub fn normal_vec_helper_3d(cam_type: &CamType, n_elements: usize) -> Vec<[f64;3]> {

    match cam_type {
        // normal helper for planar disc is just in z-direction (disc is created in xy-plane)
        CamType::PlanarDisc => vec![[0.0, 0.0, -1.0]; n_elements],
        CamType::PlanarGroove(_) => vec![[0.0, 0.0, -1.0]; n_elements],
        // normal helper for cylindric bead is a concentric vector to the axis of rotation --> Create a PlanarDisc on the xy-system
        CamType::CylindricBead(_) => {
            let rs_concentric = vec![1.0; n_elements];
            let phis_radiant = (0..n_elements).map(|i| i as f64/(n_elements-1) as f64 * 2.0*PI).collect::<Vec<_>>();
            let xyz_helper = polar_2_cartesian_3d(&rs_concentric, &phis_radiant, &CamType::PlanarDisc);
            xyz_helper },  
        // normal helper for cylindric bead is a concentric vector to the axis of rotation --> Create a PlanarDisc on the xy-system
        CamType::CylindricGroove(_) => {
            let rs_concentric = vec![1.0; n_elements];
            let phis_radiant = (0..n_elements).map(|i| i as f64/(n_elements-1) as f64 * 2.0*PI).collect::<Vec<_>>();
            let xyz_helper = polar_2_cartesian_3d(&rs_concentric, &phis_radiant, &CamType::PlanarDisc);
            xyz_helper }, 
    }
}

pub fn cross_product_3d(vec_1: &[[f64;3]], vec_2: &[[f64;3]], normed: bool) -> Vec<[f64;3]> {
    let norm = vec_1.iter()
        .zip(vec_2)
        .map(|(v1, v2)| {
            let mut cross_product = [
                v1[1]*v2[2] - v1[2]*v2[1],
                v1[2]*v2[0] - v1[0]*v2[2],
                v1[0]*v2[1] - v1[1]*v2[0]];
            if normed {
                let len_res = cross_product.iter().map(|x| x.powi(2)).sum::<f64>().sqrt();
                cross_product.iter_mut().for_each(|x| {
                    *x /= len_res;
                });
            }
            cross_product
        }).collect::<Vec<_>>();
    norm
}

pub fn angles_3d(vec_1: &[[f64;3]], vec_2: &[[f64;3]]) -> Vec<f64> {
    let phis = vec_1.iter().zip(vec_2).map(|(v1, v2)| {
        let len_1 = v1.iter().map(|e| e.powi(2)).sum::<f64>().sqrt();
        let len_2 = v2.iter().map(|e| e.powi(2)).sum::<f64>().sqrt();
        let mut ans = ((v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]) / (len_1*len_2));
        if ans <= -1.0 {
            ans = -1.0;
        }
        else if ans >= 1.0 {
            ans = 1.0;
        }
        ans.acos()
    }).collect::<Vec<_>>();
    phis
}


/// See: https://de.wikipedia.org/wiki/Kr%C3%BCmmung -> Raumkurven with kappa = 1/r
pub fn calc_curvature_ers_3d(points: &[[f64;3]], radius_follower: Option<f64>, cam_type: &CamType) -> Vec<f64> {
    let helper = normal_vec_helper_3d(cam_type, points.len());
    let d_points = gradient_wrapping_3d(&points);
    let d2_points = gradient_wrapping_3d(&d_points);

    let curvature_sign = cross_product_3d(&d_points, &helper, false)
        .iter()
        .map(|[x, y, z]| {
            if *z <= 0.0 { -1.0 }
            else { 1.0 }
        }).collect::<Vec<f64>>();

    let numerator = cross_product_3d(&d_points, &d2_points, false)
        .iter()
        .map(|[x, y, z]| (x.powi(2)+y.powi(2)+z.powi(2)).sqrt())
        .collect::<Vec<_>>();
    let denominator = d_points
        .iter()
        .map(|[x, y, z]| (x.powi(2)+y.powi(2)+z.powi(2)).sqrt().powi(3))
        .collect::<Vec<_>>();

    let mut kappas_disc = numerator.iter().zip(denominator).map(|(n, d)| n/d).collect::<Vec<_>>();
    kappas_disc.iter_mut().zip(curvature_sign).for_each(|(k, s)| *k *= -s);

    // Berechnen des Ersatzradius
    match radius_follower {
        Some(radius_follower) => {
            match cam_type {
                CamType::PlanarDisc => kappas_disc.iter_mut().for_each(|k| {
                    let k_sum = *k + 1.0/radius_follower;
                    *k = 1.0/k_sum;
                }),
                CamType::PlanarGroove(_) => kappas_disc.iter_mut().for_each(|k| {
                    let k_sum = *k + 1.0/radius_follower;
                    *k = 1.0/k_sum;
                }),
                CamType::CylindricBead(cylinder) => kappas_disc.iter_mut().for_each(|k| {
                    let k_sum = *k + 1.0/cylinder.base_radius + 1.0/radius_follower;
                    *k = 1.0/k_sum;
                }),
                CamType::CylindricGroove(cylinder) => kappas_disc.iter_mut().for_each(|k| {
                    let k_sum = *k + 1.0/cylinder.base_radius + 1.0/radius_follower;
                    *k = 1.0/k_sum;
                }),
            }
        }
        None => {
            kappas_disc.iter_mut().for_each(|k| {
                *k = 1.0/(*k);
            });
        }
    }
    kappas_disc
}


/// Build the central difference (gradient) for a function f(x), or r(phi)
/// See: https://en.wikipedia.org/wiki/Finite_difference
/// 
/// The first and last element are treated as wrapping around, so at every position the 2nd order difference is possible
/// ## Make sure to pass in angles in radiants!
/// # Inputs
/// * `fs`: function values f(x)
/// * `xs`: variable x (make sure to pass in angles in radiant). Must be of same size as fs.
/// 
/// # Returns
/// * Vector of the numeric gradient d/dx*f(x). This Vec has the same size as the input arrays
pub fn gradient_wrapping(fs: &[f64], xs: &[f64]) -> Vec<f64> {
    assert!(fs.len() == xs.len(), "Input vectors must be of same size");

    let res = fs.iter().zip(xs.iter()).enumerate().map(
        |(i, _)| {
            // Wrap around to last element for first element
            if i == 0 {
                let i_prev = fs.len()-1;
                let i_next = i+1;
                return (fs[i_next]-fs[i_prev]) / (2.0*xs[i_next]-xs[i])
            }
            // Wrap around to first element for last element
            if i == fs.len()-1 {
                let i_prev = i-1;
                let i_next = 0;
                return (fs[i_next]-fs[i_prev]) / (2.0*xs[i]-xs[i_prev])

            } 
            (fs[i+1]-fs[i-1]) / (xs[i+1]-xs[i-1])
        }
    ).collect::<Vec<_>>();
    res
}



pub fn gradient_wrapping_ndim(points: Vec<Vec<f64>>) -> Vec<Vec<f64>> {
    // Calculate the difference of 2 n_dimensional vectors
    fn _normed_vec_diff<'a>(a: &'a [f64], b: &[f64]) -> Vec<f64> {
        let res = a.iter()
            .zip(b)
            .map(|(a_n, b_n)| b_n-a_n)
            .collect::<Vec<_>>();
        let len = res.iter().map(|x| x.powi(2)).sum::<f64>().sqrt();
        let normed_vec = res.iter().map(|x| x/len).collect::<Vec<_>>();
        normed_vec
    }

    let res = points.iter().enumerate()
        .map(|(i, point)| {
            let mut i_next = i+1;
            let mut i_prev = i-1;
            if i == 0 {
                i_prev = points.len()-1;
                i_next = i+1;
            }
            if i == points.len()-1 {
                i_prev = i-1;
                i_next = 0;
            }
            _normed_vec_diff(&points[i_prev], &points[i_next])
        }).collect::<Vec<_>>();
    res
}

/// Computes the gradient in x-y-direction based on polar coordinates. 
/// For more info see: https://mathepedia.de/Polarkoordinaten_Tangentialvektor_Ebene_Kurven.html
/// 
/// # Inputs
/// * `r_dots`: difference dr/dphi e.g. via the helper function `gradient_wrapping`
/// * `phis`: corresponding angles in radiants!
/// 
/// # Returns
/// Vector of normalized (x,y) gradient pairs (meaning lenght always 1)
pub fn gradient_polar_2_gradient_cartesian(r_dots: &[f64], rs: &[f64], phis: &[f64]) -> Vec<(f64, f64)> {

    let xy_dots = r_dots
        .iter()
        .zip(rs.iter())
        .zip(phis.iter())
        .map(|((r_dot, r), phi)| {
            let mut x_dot = (r_dot*(phi.cos()) - r*(phi.sin()));
            let mut y_dot = (r_dot*(phi.sin()) + r*(phi.cos()));
            let norm = (x_dot.powi(2) + y_dot.powi(2)).sqrt();
            x_dot /= norm;
            y_dot /= norm;
            (x_dot, y_dot)
        })
        .collect::<Vec<_>>();
    xy_dots
}

/// Calculate the velocities and accelerations for a given cam follower curve
/// 
/// # Inputs
/// * `radius`: Radiuses in mm
/// * `phis`: Corresponding angles in degrees
/// * `rpm`: angular velocity in 1/min
/// 
/// # Returns
/// * `(Vec<f64>, Vec<F64>)`: (velocities, accelerations)
pub fn calc_velocities_accelerations(radius: &[f64], phis: &[f64], rpm: f64) -> (Vec<f64>, Vec<f64>) {
    let omega = PI*rpm/30.0;
    let phis_radiant = phis.iter().map(|phi| phi/180.0 * PI).collect::<Vec<_>>();

    // Calculate the velocities and accelerations based on the contour and angular velocity
    let gradient_first = gradient_wrapping(&radius, &phis_radiant);
    let gradient_second = gradient_wrapping(&gradient_first, &phis_radiant);

    let velocities_follower = gradient_first
        .iter()
        .map(|gradient| gradient * omega)
        .collect::<Vec<_>>();
    let accelerations_follower = gradient_second
        .iter()
        .map(|gradient| gradient * omega.powi(2))
        .collect::<Vec<_>>();
    (velocities_follower, accelerations_follower)
}

pub fn centered_moving_average_wrapping(radius: &[f64], n_elements: &mut usize) -> Vec<f64> {
    // Make sure n_elements is even
    if *n_elements % 2 != 0 {
        *n_elements += 1;
    }
    let res = radius.iter().enumerate()
        .map(|(i, r)| {
            let mut ans = 0.0;
            for j in i-*n_elements/2..i+*n_elements/2 {
                let mut picker = j;
                if picker < 0 {
                    picker = radius.len() - picker;
                }
                else if picker >= radius.len() {
                    picker = picker - radius.len();
                }
                ans += radius[picker];
            }
            ans /= *n_elements as f64;
            ans
        }).collect::<Vec<_>>();
    res
}

/// Berechnung der Streckenabschnittlänge ds für den Drehwinkel dphi der Kurvenscheibe
/// Returns Unit [m/°]
pub fn ds_wrapping(points: &[[f64;3]]) -> Vec<f64> {

    let res = points.iter().enumerate()
        .map(|(i, p)| {
            let mut i_next = i+1;
            if i == points.len()-1 {
                i_next = 0
            };
            // Calculate the length of the streckenabschnitt dphi
            let ds = (
                (points[i_next][0] - points[i][0]).powi(2) + 
                (points[i_next][1] - points[i][1]).powi(2) +
                (points[i_next][2] - points[i][2]).powi(2)
            ).sqrt();
            ds
        }).collect::<Vec<_>>();
    res
}


/// Trapezoidal integration of a function f(x)
/// Specify dx if constant spacing, else pass in the xs
/// 
/// # Inputs
/// * `fs`: Function values (y-axis)
/// * `xs`: If uneven spacing the x-values where the function was samples
/// * `dx`: If even spacing the spacing between to function values
/// 
/// # Returns
/// * `trapz`: Trapezoidal integration of the function
pub fn trapz(fs: &[f64], xs: Option<&[f64]>, dx: Option<f64>) -> f64 {
    if (xs == None) & (dx == None) {
        panic!("One of xs or dx must be specified");
    }

    else if let Some(dx) = dx {
        let mut ans = 0.0;
        for i in 0..fs.len()-2 {
            ans += dx * 0.5 * (fs[i+1] + fs[i]);
        }
        return ans;
    }

    else if let Some(xs) = xs {
        let mut ans = 0.0;
        for i in 0..fs.len()-2 {
            ans += (xs[i+1] - xs[i]) * 0.5 * (fs[i+1] + fs[i]);
        }
        return ans;
    }

    else { std::f64::NAN }
}
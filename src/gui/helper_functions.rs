use std::f64::consts::PI;
use egui_plot::PlotPoints;


/// Function to get the plot points of a circle with radius `radius_follower` at offset `pos_y`
/// 
/// # Inputs:
/// * `pos_y`: Offset of the center of the circle to 0
/// 
/// # Returns:
/// * PlotPoints of the circle for plotting in egui. The plot points can then be used e.g. for a line plot like Line::new(PlotPoints)
pub fn plot_points_circle(radius: f64, offset_y: f64) -> PlotPoints {
    let raw_circle = PlotPoints::from_parametric_callback(|t| {(t.cos(), t.sin())}, 0.0..=2.0*PI, 100);
    let offset_circle = raw_circle
        .points()
        .iter()
        .map(|point| {
            let x = point.x * radius;
            let y = point.y * radius + offset_y;
            [x, y]
        })
        .collect::<PlotPoints>();

    offset_circle
}


/// Function to get the plot points of the disc of the camsystem if it is rotated by `angle`
/// 
/// # Inputs:
/// * `angle`: Rotational angle in radiants
/// 
/// # Returns:
/// * PlotPoints of the disc for plotting in egui. The plot points can then be used e.g. for a line plot like Line::new(PlotPoints)
pub fn rotate_points_planar_xy(points: &[[f64; 3]], angle: f64) -> PlotPoints {
    // 2d rotation matrix
    let rot_mat = [[angle.cos(), -angle.sin()], [angle.sin(), angle.cos()]];

    // Rotate every single point of the disc with the rotation matrix
    // Only take one point for every 1°
    let chunk_size = points.len() / 360;
    let disc_graph = points
        .chunks(chunk_size)
        .flat_map(|c| c.first())
        .map(|[x, y, z]| [x*rot_mat[0][0] + y*rot_mat[0][1], x*rot_mat[1][0]+y*rot_mat[1][1]])
        .collect::<PlotPoints>();

    disc_graph
}

pub fn rotate_points_cylinder_z(points: &[[f64;3]], angle: f64) -> PlotPoints {
    let rot_mat = [
        [angle.cos(), -angle.sin(), 0.0],
        [angle.sin(), angle.cos(), 0.0],
        [0.0, 0.0, 1.0]
    ];
    let chunk_size = points.len() / 360;
    let graph = points
        .chunks(chunk_size)
        .flat_map(|c| c.first())
        .map(|[x, y, z]| {
            [x*rot_mat[0][0] + y*rot_mat[0][1],
            *z]
        })
        .collect::<PlotPoints>();
    graph
}
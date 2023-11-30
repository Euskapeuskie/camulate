use serde::{Serialize, Deserialize};


#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ProcessForce {
    pub kind: ProcessForceKind,
    pub peak_force: f64,
    pub deg_start: f64,
    pub deg_end: f64,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ProcessForces {
    pub process_forces: Vec<ProcessForce>,
    pub accuracy: f64,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy, PartialEq)]
pub enum ProcessForceKind {
    Cutting,
    Stitching,
    Friction,
}


impl ProcessForce {
    pub fn new(kind: ProcessForceKind, peak_force: f64, deg_start: f64, deg_end: f64) -> Self {
        ProcessForce {
            kind: kind,
            peak_force: peak_force,
            deg_start: deg_start,
            deg_end: deg_end,
        }
    }

    pub fn to_vec(&self, accuracy: f64) -> Vec<f64> {
        let n_elements = ((self.deg_end - self.deg_start)/accuracy) as usize;
        match self.kind {
            ProcessForceKind::Cutting => self.process_forces_cutting(n_elements),
            ProcessForceKind::Stitching => self.process_forces_stitching(n_elements),
            ProcessForceKind::Friction => vec![self.peak_force; n_elements],
        }
    }

    /// Calculates a inverted parabola with peak = peak_force and 0 at start_deg & end_deg.
    /// This is roughly similar to 
    /// 
    /// # Inputs
    /// * `peak_force`: Expected maximum force of the cut in N
    /// * `start_deg`: Start of the cut in ° of the rotation angle of the cam system
    /// * `end_deg`: End of the cut in ° of the rotation angle of the cam system
    /// * `accuracy`: Accuracy of the calculation in ° --> determines how many points will be calculated.
    /// Has to be the same as the accuracy of the cam system
    /// 
    /// # Returns
    /// A vector of length (end_deg - start_deg)/accuracy, holding the forces at each index
    fn process_forces_cutting(&self, n_elements: usize) -> Vec<f64> {

        // 0-crossings of the parabola as indices
        let (x_0, x_1) = (0.0, n_elements as f64-1.0);

        // Nullstellenform der Parabel
        // Herleitung von a über...
        // f(x) = a* (x-x_0) * (x-x_1)
        // Nebenbediungung: f((x_0+x_1)/2) = peak_force --> Höchster Punkt der Parabel in der Mitte zwischen x_0 und x_1
        // --> Einsetzen und Umstellen nach a
        let a = 4.0*self.peak_force / ((x_1-x_0) * (x_0-x_1));

        let ys = (0..n_elements)
            .map(|x| a*(x as f64-x_0)*(x as f64-x_1))
            .collect::<Vec<_>>();
        ys  
    }


    fn process_forces_stitching(&self, n_elements: usize) -> Vec<f64> {
    
        todo!()
    }
}



impl ProcessForces {
    pub fn new(accuracy: f64) -> Self {
        Self {
            process_forces: vec![],
            accuracy: accuracy
        }
    }

    pub fn to_vec(&self) -> Vec<f64> {
        let mut process_forces_vec = vec![0.0; (360.0/self.accuracy) as usize];

        for process_force in &self.process_forces {
            let process_force_vec = process_force.to_vec(self.accuracy);
            let index_start = (process_force.deg_start/self.accuracy) as usize;
            let index_end = (process_force.deg_end/self.accuracy)as usize;
            if let Some(x) = process_forces_vec.get_mut(index_start..index_end) {
                for (i, x_i) in x.iter_mut().enumerate() {
                    *x_i += process_force_vec[i];
                }
            }
        }
        process_forces_vec
    }
}
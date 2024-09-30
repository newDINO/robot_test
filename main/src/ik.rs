use std::f32::consts::PI;

use rapier3d::prelude::nalgebra;
use nalgebra::{Rotation, Vector3};

#[derive(Debug)]
pub struct RobotNova {
    pub _a2: f32,
    pub _a3: f32,
    pub _d1: f32,
    pub d4: f32,
    pub _d5: f32,
    pub d6: f32,
    pub theta1: [f32; 2],
    pub theta5: [f32; 2],
    pub theta60: [f32; 4],
    pub theta61: [f32; 4],
}

impl Default for RobotNova {
    fn default() -> Self {
        Self {
            _a2: 0.4,
            _a3: 0.33,
            _d1: 0.24,
            d4: 0.135,
            _d5: 0.12,
            d6: 0.088,
            theta1: [0.0; 2],
            theta5: [0.0; 2],
            theta60: [0.0; 4],
            theta61: [0.0; 4],
        }
    }
}

impl RobotNova {
    pub fn solve(&mut self, r: &Rotation<f32, 3>, p: &Vector3<f32>) {
        // theta1
        self.theta1 = solve_asbc(
            self.d6 * r[(0, 2)] - p.x,
            p.y - self.d6 * r[(1, 2)],
            -self.d4,
        ).into();
        // theta5
        let cos_theta5_0 = r[(0, 2)] * self.theta1[0].sin() - r[(1, 2)] * self.theta1[0].cos();
        let cos_theta5_1 = r[(0, 2)] * self.theta1[1].sin() - r[(1, 2)] * self.theta1[1].cos();
        self.theta5[0] = cos_theta5_0.acos();
        // self.theta5[1] = -self.theta5[0]; // invalid because theta6 from these values are NaN or fixed
        // self.theta5[2] = cos_theta5_1.acos();
        self.theta5[1] = -cos_theta5_1.acos();
        
        // theta6
        for i in 0..self.theta5.len() {
            let theta5 = self.theta5[i];
            let theta1 = self.theta1[i % 2];
            let cos_theta6 = (-r[(0, 0)] * theta1.sin() + r[(1, 0)] * theta1.cos()) / (-theta5.sin());
            let theta6 = cos_theta6.acos();
            self.theta60[i * 2] = theta6;
            self.theta60[i * 2 + 1] = -theta6;
        }
        for i in 0..self.theta5.len() {
            let theta5 = self.theta5[i];
            let theta1 = self.theta1[i % 2];
            let sin_theta6 =  (-r[(0, 1)] * theta1.sin() + r[(1, 1)] * theta1.cos()) / theta5.sin();
            let theta6 = sin_theta6.asin();
            self.theta61[i * 2] = theta6;
            self.theta61[i * 2 + 1] = if theta6 > 0.0 {
                PI - theta6
            } else {
                -PI - theta6
            };
        }
    }
}
// solutions for
// a * sin(x) + b * cos(x) = c
fn solve_asbc(a: f32, b: f32, c: f32) -> (f32, f32) {
    let k1 = (a * a + b * b - c * c).sqrt();
    let k2 = b + c;
    let s1 = 2.0 * ((a - k1) / k2).atan();
    let s2 = 2.0 * ((a + k1) / k2).atan();
    // let s1 = b.atan2(-a) - c.atan2(k1);
    // let s2 = b.atan2(-a) - c.atan2(-k1);
    (s1, s2)
}

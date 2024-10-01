use std::f32::consts::{FRAC_PI_2, PI};

use argmin::{
    core::{CostFunction, Executor, State},
    solver::particleswarm::ParticleSwarm,
};
use rapier3d::{
    na::{matrix, Isometry3, Matrix4, Rotation, Translation3, UnitQuaternion, Vector3},
    prelude::nalgebra,
};

#[derive(Debug)]
pub struct RobotNova {
    // parameters
    pub a2: f32,
    pub a3: f32,
    pub d1: f32,
    pub d4: f32,
    pub d5: f32,
    pub d6: f32,
    pub limits: (Vec<f32>, Vec<f32>),
    // analytical solutions
    pub theta1: [f32; 2],
    pub theta5: [f32; 4],
    pub sin_theta6: [f32; 4],
    pub cos_theta6: [f32; 4],
    pub ssum_theta6: [f32; 4],
    pub theta6: [f32; 4],
    pub t14_left: Matrix4<f32>,
    // numerical solutions
    pub best_root: Vec<f32>,
    pub best_cost: f32,
    pub roots: Vec<[f32; 6]>,
}

impl Default for RobotNova {
    fn default() -> Self {
        Self {
            a2: -0.4,
            a3: -0.33,
            d1: -0.24,
            d4: 0.135,
            d5: -0.12,
            d6: 0.088,
            limits: (
                vec![-PI, -PI, -PI, -PI, -PI, -PI],
                vec![PI, PI, PI, PI, PI, PI],
            ),

            theta1: [0.0; 2],
            theta5: [0.0; 4],
            sin_theta6: [0.0; 4],
            cos_theta6: [0.0; 4],
            ssum_theta6: [0.0; 4],
            theta6: [0.0; 4],
            t14_left: Matrix4::identity(),

            best_root: Vec::new(),
            best_cost: 0.0,
            roots: Vec::new(),
        }
    }
}

impl RobotNova {
    pub fn solve(&mut self, rq: &UnitQuaternion<f32>, p: &Vector3<f32>) {
        let r = rq.to_rotation_matrix();
        // theta1
        self.theta1 = solve_asbc(
            self.d6 * r[(0, 2)] - p.x,
            p.y - self.d6 * r[(1, 2)],
            -self.d4,
        )
        .into();
        // theta5
        let cos_theta5_0 = r[(0, 2)] * self.theta1[0].sin() - r[(1, 2)] * self.theta1[0].cos();
        let cos_theta5_1 = r[(0, 2)] * self.theta1[1].sin() - r[(1, 2)] * self.theta1[1].cos();
        self.theta5[0] = cos_theta5_0.acos();
        self.theta5[1] = -self.theta5[0];
        self.theta5[2] = cos_theta5_1.acos();
        self.theta5[3] = -cos_theta5_1.acos();

        // theta6
        let mut theta6_cos = [0.0; 8];
        for i in 0..self.theta5.len() {
            let theta5 = self.theta5[i];
            let theta1 = self.theta1[i / 2];
            let cos_theta6 =
                (-r[(0, 0)] * theta1.sin() + r[(1, 0)] * theta1.cos()) / (-theta5.sin());
            self.cos_theta6[i] = cos_theta6;
            let theta6 = cos_theta6.acos();
            theta6_cos[i * 2] = theta6;
            theta6_cos[i * 2 + 1] = -theta6;
        }
        let mut theta6_sin = [0.0; 8];
        for i in 0..self.theta5.len() {
            let theta5 = self.theta5[i];
            let theta1 = self.theta1[i / 2];
            let sin_theta6 = (-r[(0, 1)] * theta1.sin() + r[(1, 1)] * theta1.cos()) / theta5.sin();
            self.sin_theta6[i] = sin_theta6;
            let theta6 = sin_theta6.asin();
            theta6_sin[i * 2] = theta6;
            theta6_sin[i * 2 + 1] = if theta6 > 0.0 {
                PI - theta6
            } else {
                -PI - theta6
            };
        }
        for i in 0..self.cos_theta6.len() {
            self.ssum_theta6[i] = self.cos_theta6[i].powi(2) + self.sin_theta6[i].powi(2);
        }
        let epsilon = 0.001;
        'outer: for i in 0..self.theta6.len() {
            let base = i * 2;
            for j in 0..2 {
                let c = theta6_cos[base + j];
                for k in 0..2 {
                    let s = theta6_sin[base + k];
                    if (c - s).abs() < epsilon {
                        self.theta6[i] = c;
                        continue 'outer;
                    }
                }
            }
            self.theta6[i] = f32::NAN;
        }

        // theta234
        let t_matrix = Isometry3 {
            translation: Translation3::from(*p),
            rotation: *rq,
        }
        .to_matrix();
        let get_t14_left = || -> Option<Matrix4<f32>> {
            Some(
                t_joint(self.theta1[0], 0.0, 0.0, self.d1).try_inverse()?
                    * t_matrix
                    * t_joint(self.theta6[0], -FRAC_PI_2, 0.0, self.d6).try_inverse()?
                    * t_joint(self.theta5[0], FRAC_PI_2, 0.0, self.d5).try_inverse()?,
            )
        };
        if let Some(t14_left) = get_t14_left() {
            self.t14_left = t14_left;
        } else {
            self.t14_left = Matrix4::identity();
        }
    }
    pub fn solve_num(&mut self, r: Rotation<f32, 3>, p: Vector3<f32>) {
        self.roots.clear();
        let problem = RobotProblem {
            a2: self.a2,
            a3: self.a3,
            d1: self.d1,
            d4: self.d4,
            d5: self.d5,
            d6: self.d6,
            target_pos: p,
            target_rot: r,
        };
        let solver = ParticleSwarm::new(self.limits.clone(), 200);
        let executor = Executor::new(problem, solver);
        let res = executor
            .configure(|state| state.max_iters(200).target_cost(0.0))
            .run()
            .unwrap();

        self.best_cost = res.state().get_best_cost();
        self.best_root = res.state().get_best_param().unwrap().position.clone();
        let particles = res.state().get_population().unwrap();
        let max_cost = 0.1;
        let max_difference = 0.1;
        for particle in particles {
            if particle.cost > max_cost {
                continue;
            }
            let mut has_similar = false;
            for root in &self.roots {
                let difference = slice_difference(root, &particle.position);
                if difference < max_difference {
                    has_similar = true;
                    break;
                }
            }
            if !has_similar {
                let mut new_root = [0.0; 6];
                new_root.copy_from_slice(&particle.position);
                self.roots.push(new_root);
            }
        }
    }
    pub fn print_roots(&self) -> String {
        let mut result = "Roots:".to_owned();
        for root in &self.roots {
            result += "\n";
            for angle in root {
                result += &format!("{:.2}, ", angle);
            }
        }
        result
    }
}

fn slice_difference(s1: &[f32], s2: &[f32]) -> f32 {
    let mut result: f32 = 0.0;
    for i in 0..s1.len() {
        result = result.max((s1[i] - s2[i]).abs())
    }
    result
}

struct RobotProblem {
    a2: f32,
    a3: f32,
    d1: f32,
    d4: f32,
    d5: f32,
    d6: f32,
    target_rot: Rotation<f32, 3>,
    target_pos: Vector3<f32>,
}

impl RobotProblem {
    fn end_transform(&self, thetas: &Vec<f32>) -> Matrix4<f32> {
        let t01 = t_joint(thetas[0], 0.0, 0.0, self.d1);
        let t12 = t_joint(thetas[1], FRAC_PI_2, 0.0, 0.0);
        let t23 = t_joint(thetas[2], 0.0, self.a2, 0.0);
        let t34 = t_joint(thetas[3], 0.0, self.a3, self.d4);
        let t45 = t_joint(thetas[4], FRAC_PI_2, 0.0, self.d5);
        let t56 = t_joint(thetas[5], -FRAC_PI_2, 0.0, self.d6);
        t01 * t12 * t23 * t34 * t45 * t56
    }
}

impl CostFunction for RobotProblem {
    type Output = f32;
    type Param = Vec<f32>;
    fn cost(&self, param: &Self::Param) -> Result<Self::Output, argmin_math::Error> {
        let transform = self.end_transform(param);
        let mut cost: f32 = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                let tij = transform[(i, j)];
                let rij = self.target_rot[(i, j)];
                cost = cost.max((tij - rij).abs())
            }
        }
        for i in 0..3 {
            let ti = transform[(i, 3)];
            let pi = self.target_pos[i];
            cost = cost.max((ti - pi).abs());
        }
        Ok(cost)
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

fn t_joint(theta: f32, alpha: f32, a: f32, d: f32) -> Matrix4<f32> {
    matrix![
        theta.cos(), -theta.sin(), 0.0, a;
        theta.sin() * alpha.cos(), theta.cos() * alpha.cos(), -alpha.sin(), -alpha.sin() * d;
        theta.sin() * alpha.sin(), theta.cos() * alpha.sin(), alpha.cos(), alpha.cos() * d;
        0.0, 0.0, 0.0, 1.0
    ]
}

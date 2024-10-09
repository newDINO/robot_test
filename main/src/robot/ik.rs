use std::f32::consts::{FRAC_PI_2, PI};

use rapier3d::na::{matrix, Matrix4};
use rapier3d::prelude::*;

// use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
// pub fn new_nova_fk(angles: &[f32]) -> Isometry3<f32> {
//     let d1 = 0.24;
//     let d4 = 0.135;
//     let d5 = 0.12;
//     let d6 = 0.088;
//     let a2 = -0.4;
//     let a3 = -0.33;
//     let t01 = new_t_joint(angles[0], 0.0, 0.0, d1);
//     let t12 = new_t_joint(angles[1], FRAC_PI_2, 0.0, 0.0);
//     let t23 = new_t_joint(angles[2], 0.0, a2, 0.0);
//     let t34 = new_t_joint(angles[3], 0.0, a3, d4);
//     let t45 = new_t_joint(angles[4], FRAC_PI_2, 0.0, d5);
//     let t56 = new_t_joint(angles[5], -FRAC_PI_2, 0.0, d6);
//     t01 * t12 * t23 * t34 * t45 * t56
// }
// fn new_t_joint(theta: f32, alpha: f32, a: f32, d: f32) -> Isometry3<f32> {
//     let ra = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), alpha);
//     let rt = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), theta);
//     let t = Translation3::new(a, -alpha.sin() * d, alpha.cos() * d);
//     Isometry3 {
//         rotation: ra * rt,
//         translation: t,
//     }
// }

pub fn solve_nova(
    limits: &[(f32, f32)],
    target: Matrix4<f32>,
) -> Vec<[f32; 6]> {
    solve(
        -0.4,
        -0.33,
        0.24,
        0.135,
        0.12,
        0.088,
        limits,
        target,
    )
}

pub fn solve(
    a2: f32,
    a3: f32,
    d1: f32,
    d4: f32,
    d5: f32,
    d6: f32,
    limits: &[(f32, f32)],
    target: Matrix4<f32>,
) -> Vec<[f32; 6]> {
    let epsilon = 1e-3;
    let mut result = Vec::with_capacity(8);

    for theta1 in solve_asbc(
        d6 * target[(0, 2)] - target[(0, 3)],
        target[(1, 3)] - d6 * target[(1, 2)],
        -d4,
    ) {
        if theta1.is_nan() || out_of_limits(theta1, limits[0]) {
            break;
        }
        let cos_theta5 = target[(0, 2)] * theta1.sin() - target[(1, 2)] * theta1.cos();
        let theta5_p = cos_theta5.acos();
        for theta5 in [theta5_p, -theta5_p] {
            if theta5.is_nan() || out_of_limits(theta5, limits[4]) {
                break;
            }
            let cos_theta6 =
                (-target[(0, 0)] * theta1.sin() + target[(1, 0)] * theta1.cos()) / (-theta5.sin());
            let sin_theta6 =
                (-target[(0, 1)] * theta1.sin() + target[(1, 1)] * theta1.cos()) / theta5.sin();
            let theta6 = theta_from_cos_sin(cos_theta6, sin_theta6, epsilon);
            let get_t14_left = || -> Option<Matrix4<f32>> {
                Some(
                    t_joint(theta1, 0.0, 0.0, d1).try_inverse()?
                        * target
                        * t_joint(theta6, -FRAC_PI_2, 0.0, d6).try_inverse()?
                        * t_joint(theta5, FRAC_PI_2, 0.0, d5).try_inverse()?,
                )
            };
            let t14_left = get_t14_left();
            if t14_left.is_none() {
                break;
            }
            let t14_left = t14_left.unwrap();

            let cos234 = (t14_left[(0, 0)] + t14_left[(2, 1)]) * 0.5;
            let sin234 = (t14_left[(2, 0)] - t14_left[(0, 1)]) * 0.5;
            let theta234 = theta_from_cos_sin(cos234, sin234, epsilon);
            if theta234.is_nan() {
                break;
            }

            let px = t14_left[(0, 3)];
            let pz = t14_left[(2, 3)];
            for theta2 in solve_asbc(
                2.0 * a2 * pz,
                2.0 * a2 * px,
                a2 * a2 - a3 * a3 + px * px + pz * pz,
            ) {
                if theta2.is_nan() || out_of_limits(theta2, limits[1]) {
                    break;
                }
                let cos_theta2 = theta2.cos();
                let sin_theta2 = theta2.sin();
                let cos_theta3 = (px * cos_theta2 + pz * sin_theta2 - a2) / a3;
                let sin_theta3 = (-px * sin_theta2 + pz * cos_theta2) / a3;
                let theta3 = theta_from_cos_sin(cos_theta3, sin_theta3, epsilon);
                if theta3.is_nan() || out_of_limits(theta3, limits[2]) {
                    break;
                }

                let theta4 = theta234 - theta3 - theta2;
                if out_of_limits(theta4, limits[3]) {
                    break;
                }
                result.push([-theta1, theta2, theta3, -theta4, theta5, theta6]);
            }
        }
    }

    result
}
fn out_of_limits(value: f32, limits: (f32, f32)) -> bool {
    value <= limits.0 || value >= limits.1
}
// solutions for
// a * sin(x) + b * cos(x) = c
fn solve_asbc(a: f32, b: f32, c: f32) -> [f32; 2] {
    let k1 = (a * a + b * b - c * c).sqrt();
    let k2 = b + c;
    let s1 = 2.0 * ((a - k1) / k2).atan();
    let s2 = 2.0 * ((a + k1) / k2).atan();
    // let s1 = b.atan2(-a) - c.atan2(k1);
    // let s2 = b.atan2(-a) - c.atan2(-k1);
    [s1, s2]
}
fn theta_from_cos_sin(c: f32, s: f32, epsilon: f32) -> f32 {
    let c0 = c.acos();
    let c1 = -c0;
    let s0 = s.asin();
    let s1 = if s0 > 0.0 { PI - s0 } else { -PI - s0 };
    if (c0 - s0).abs() < epsilon {
        (c0 + s0) * 0.5
    } else if (c0 - s1).abs() < epsilon {
        (c0 + s1) * 0.5
    } else if (c1 - s0).abs() < epsilon {
        (c1 + s0) * 0.5
    } else if (c1 - s1).abs() < epsilon {
        (c1 + s1) * 0.5
    } else {
        f32::NAN
    }
}
fn t_joint(theta: f32, alpha: f32, a: f32, d: f32) -> Matrix4<f32> {
    matrix![
        theta.cos(), -theta.sin(), 0.0, a;
        theta.sin() * alpha.cos(), theta.cos() * alpha.cos(), -alpha.sin(), -alpha.sin() * d;
        theta.sin() * alpha.sin(), theta.cos() * alpha.sin(), alpha.cos(), alpha.cos() * d;
        0.0, 0.0, 0.0, 1.0
    ]
}

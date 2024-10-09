use bevy::prelude::*;
use nalgebra::{Matrix4, Quaternion, Translation, UnitQuaternion};
use rapier3d::prelude::*;
use rapier3d::{
    math::Isometry,
    na::{vector, Vector4},
};

pub fn isometry_to_transform(isometry: &Isometry<f32>) -> Transform {
    let t1 = isometry.translation;
    let t2 = Vec3::new(t1.x, t1.y, t1.z);
    let r1 = isometry.rotation.as_vector();
    let r2 = Quat::from_xyzw(r1.x, r1.y, r1.z, r1.w);
    Transform {
        translation: t2,
        rotation: r2,
        scale: Vec3::ONE,
    }
}

pub fn transform_to_isometry(transfrom: &Transform) -> Isometry<f32> {
    let t1 = transfrom.translation;
    let t2 = vector![t1.x, t1.y, t1.z];
    let r1 = transfrom.rotation;
    let r2 = Quaternion::new(r1.w, r1.x, r1.y, r1.z);
    Isometry {
        rotation: UnitQuaternion::from_quaternion(r2),
        translation: Translation::from(t2),
    }
}

pub fn vec4_to_vector(v: Vec4) -> Vector4<f32> {
    vector![v.x, v.y, v.z, v.w]
}

pub fn mat4_to_matrix(m: Mat4) -> Matrix4<f32> {
    Matrix4::<f32>::from_columns(&[
        vec4_to_vector(m.col(0)),
        vec4_to_vector(m.col(1)),
        vec4_to_vector(m.col(2)),
        vec4_to_vector(m.col(3)),
    ])
}

pub fn transform_to_matrix(transform: Transform) -> Matrix4<f32> {
    mat4_to_matrix(transform.compute_matrix())
}

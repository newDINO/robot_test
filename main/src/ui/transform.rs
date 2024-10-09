use bevy::{
    math::{EulerRot, Quat},
    prelude::Transform,
};
use bevy_egui::egui;

pub fn rot_pos_control(transform: &mut Transform, ui: &mut egui::Ui) {
    egui::Grid::new("Transform").show(ui, |ui| {
        inner_rot_pos_control(transform, ui);
    });
}

// pub fn transform_control(transform: &mut Transform, ui: &mut egui::Ui) {
//     egui::Grid::new("Transform").show(ui, |ui| {
//         inner_rot_pos_control(transform, ui);

//         ui.label("Scale");
//         ui.end_row();

//         let scale = &mut transform.scale;
//         let speed = 0.01;
//         ui.label("x: ");
//         ui.add(egui::DragValue::new(&mut scale.x).speed(speed));
//         ui.end_row();

//         ui.label("y: ");
//         ui.add(egui::DragValue::new(&mut scale.y).speed(speed));
//         ui.end_row();

//         ui.label("z: ");
//         ui.add(egui::DragValue::new(&mut scale.z).speed(speed));
//         ui.end_row();
//     });
// }

fn inner_rot_pos_control(transform: &mut Transform, ui: &mut egui::Ui) {
    ui.label("Rotation");
    ui.end_row();

    let euler_rot = EulerRot::default();
    ui.label("Euler Rot");
    ui.label(format!("{:?}", euler_rot));
    ui.end_row();

    let (mut a, mut b, mut c) = transform.rotation.to_euler(euler_rot);

    let speed = 0.01;
    ui.label("a: ");
    ui.add(egui::DragValue::new(&mut a).speed(speed));
    ui.end_row();

    ui.label("b: ");
    ui.add(egui::DragValue::new(&mut b).speed(speed));
    ui.end_row();

    ui.label("c: ");
    ui.add(egui::DragValue::new(&mut c).speed(speed));
    ui.end_row();

    transform.rotation = Quat::from_euler(euler_rot, a, b, c);

    ui.label("Translation");
    ui.end_row();

    let pos = &mut transform.translation;
    let speed = 0.01;
    ui.label("x: ");
    ui.add(egui::DragValue::new(&mut pos.x).speed(speed));
    ui.end_row();

    ui.label("y: ");
    ui.add(egui::DragValue::new(&mut pos.y).speed(speed));
    ui.end_row();

    ui.label("z: ");
    ui.add(egui::DragValue::new(&mut pos.z).speed(speed));
    ui.end_row();
}

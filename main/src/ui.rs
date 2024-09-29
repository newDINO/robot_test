use rapier3d::prelude::*;

use bevy::diagnostic::DiagnosticsStore;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::input::{keyboard::KeyboardInput, mouse::MouseButtonInput, ButtonState};
use bevy::prelude::*;
use bevy::window::CursorGrabMode;
use bevy::window::PrimaryWindow;
use bevy_egui::{egui, EguiContexts};

use crate::robot::JointHandles;
use crate::robot::PhysicsSystem;
use crate::SystemState;

pub fn ui_plugin(app: &mut App) {
    app.add_plugins(FrameTimeDiagnosticsPlugin);
    app.add_systems(Update, ui_system_update);
}

fn ui_system_update(
    mut system_state: ResMut<SystemState>,
    mut camera_q: Query<&mut Transform, With<Camera3d>>,
    mut contexts: EguiContexts,
    diagnostics: Res<DiagnosticsStore>,
    mut keyboard_input_events: EventReader<KeyboardInput>,
    mut mouse_button_events: EventReader<MouseButtonInput>,
    mut main_window: Query<&mut Window, With<PrimaryWindow>>,
    mut physics_system: ResMut<PhysicsSystem>,
    joint_handles: Res<JointHandles>,
) {
    let inner_response = egui::Window::new("UI")
        .show(contexts.ctx_mut(), |ui| {
            ui.collapsing("Debug Info", |ui| {
                if let Some(value) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
                    if let Some(value) = value.smoothed() {
                        ui.label(format!("FPS: {:.2}", value));
                    } else {
                        ui.label("FPS: N/A");
                    }
                } else {
                    ui.label("FPS: N/A");
                };
                let camera_transform = camera_q.get_single_mut().unwrap();
                ui.label(format!("{:?}", camera_transform));
            });
            ui.collapsing("Joints", |ui| {
                for (multibody_id, link_id) in &joint_handles.idx {
                    let multibody = physics_system
                        .multibody_joint_set
                        .get_multibody_mut(*multibody_id)
                        .unwrap();
                    let link = multibody.link_mut(*link_id).unwrap();
                    let joint = &mut link.joint.data;
                    let axis = JointAxis::AngX;
                    joint.motor_axes |= axis.into();
                    let motor = &mut joint.motors[axis as usize];
                    motor.stiffness = 10.0;
                    motor.damping = 0.1;
                    let limit = joint.limits[axis as usize];
                    ui.add(egui::Slider::new(
                        &mut motor.target_pos,
                        limit.min..=limit.max,
                    ));
                }
            });
        })
        .unwrap();

    // Camera Controlling
    for event in mouse_button_events.read() {
        if event.state == ButtonState::Pressed {
            if event.button == MouseButton::Left {
                let mut window = main_window.get_mut(event.window).unwrap();
                let cursor_pos = window.cursor_position().unwrap();
                let cursor_pos_egui = egui::pos2(cursor_pos.x, cursor_pos.y);
                let ui_rect = inner_response.response.rect;
                if !ui_rect.contains(cursor_pos_egui) && !system_state.controlling_camera {
                    window.cursor.visible = false;
                    window.cursor.grab_mode = CursorGrabMode::Confined;
                    system_state.controlling_camera = true;
                }
            }
        }
    }
    for event in keyboard_input_events.read() {
        if event.key_code == KeyCode::Escape {
            if system_state.controlling_camera {
                system_state.controlling_camera = false;
                let mut window = main_window.get_mut(event.window).unwrap();
                window.cursor.visible = true;
                window.cursor.grab_mode = CursorGrabMode::None;
            }
        }
    }
}

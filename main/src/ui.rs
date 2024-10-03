use bevy::diagnostic::DiagnosticsStore;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::input::{keyboard::KeyboardInput, mouse::MouseButtonInput, ButtonState};
use bevy::prelude::*;
use bevy::window::CursorGrabMode;
use bevy::window::PrimaryWindow;
use bevy_egui::{egui, EguiContexts};
use rapier3d::prelude::*;

use crate::robot::RobotSystem;

pub fn ui_plugin(app: &mut App) {
    app.add_plugins(FrameTimeDiagnosticsPlugin);
    app.add_systems(Startup, ui_system_setup);
    app.add_systems(Update, ui_system_update);
}

#[derive(Resource)]
pub struct SystemState {
    pub controlling_camera: bool,
}
impl SystemState {
    fn new() -> Self {
        Self {
            controlling_camera: false,
        }
    }
}

fn ui_system_setup(mut commands: Commands) {
    commands.insert_resource(SystemState::new());
}

fn ui_system_update(
    mut system_state: ResMut<SystemState>,
    mut camera_q: Query<&mut Transform, With<Camera3d>>,
    mut contexts: EguiContexts,
    diagnostics: Res<DiagnosticsStore>,
    mut keyboard_input_events: EventReader<KeyboardInput>,
    mut mouse_button_events: EventReader<MouseButtonInput>,
    mut main_window: Query<&mut Window, With<PrimaryWindow>>,
    robot_system: ResMut<RobotSystem>,
) {
    let robot_system = robot_system.into_inner();
    let show_ui_window = |ui: &mut egui::Ui| {
        if system_state.controlling_camera {
            ui.disable();
        }
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
        ui.collapsing("Joint Control", |ui| {
            let index = 0;
            for i in 0..robot_system.displacements.len() {
                let joint = robot_system.get_link(i).joint.data;
                let limits = joint.limits[3];
                let slider = egui::Slider::new(
                    &mut robot_system.displacements[i][index],
                    limits.min..=limits.max,
                );
                ui.add(slider);
                let displacement = robot_system.displacements[i];
                let link = robot_system.get_link_mut(i);
                let mut new_joint = MultibodyJoint::new(link.joint.data, true);
                new_joint.apply_displacement(displacement.as_slice());
                link.joint = new_joint;
            }
            robot_system.update_multi_body();
        });
        ui.collapsing("Collision Detection", |ui| {
            if ui.button("detect").clicked() {
                robot_system.detect_collision();
            };
            for collision in &robot_system.last_collisions {
                ui.label(format!("{:?}", collision));
            }
        });
    };
    let inner_response = egui::Window::new("UI")
        .show(contexts.ctx_mut(), |ui| {
            egui::ScrollArea::vertical().show(ui, show_ui_window);
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

use std::fmt::Write;

use bevy::diagnostic::DiagnosticsStore;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::input::keyboard::KeyboardInput;
use bevy::prelude::*;
use bevy::window::CursorGrabMode;
use bevy::window::PrimaryWindow;
use bevy_egui::{egui, EguiContexts};
use rapier3d::na::Vector6;

use crate::math::transform_to_matrix;
use crate::robot;
use crate::robot::astar;
use crate::robot::astar::AStar;
use crate::robot::rtt::Rtt;
use crate::robot::RobotSystem;

mod transform;
pub use transform::*;

pub fn ui_plugin(app: &mut App) {
    app.add_plugins(FrameTimeDiagnosticsPlugin);
    app.add_systems(Startup, ui_system_setup);
    app.add_systems(Update, ui_system_update);
}

#[derive(Resource)]
pub struct SystemState {
    pub controlling_camera: bool,
    target_transform: Transform,

    rtt: Option<Rtt>,
    rtt_n_steps: usize,
    rtt_auto_step: bool,

    astar: Option<AStar<Vector6<i32>>>,
    astar_graph: astar::RobotGraph,
    astar_auto_step: bool,
}
impl SystemState {
    fn new() -> Self {
        Self {
            controlling_camera: false,
            target_transform: Transform::IDENTITY,

            rtt: None,
            rtt_n_steps: 1,
            rtt_auto_step: false,

            astar: None,
            astar_graph: astar::RobotGraph::new(0.01),
            astar_auto_step: false,
        }
    }
}

fn ui_system_setup(mut commands: Commands) {
    commands.insert_resource(SystemState::new());
}

fn ui_system_update(
    system_state: ResMut<SystemState>,
    mut camera_q: Query<&mut Transform, With<Camera3d>>,
    mut contexts: EguiContexts,
    diagnostics: Res<DiagnosticsStore>,
    mut keyboard_input_events: EventReader<KeyboardInput>,
    mut main_window: Query<&mut Window, With<PrimaryWindow>>,
    robot_system: ResMut<RobotSystem>,
    mut gizmos: Gizmos,
) {
    let robot_system = robot_system.into_inner();
    let system_state = system_state.into_inner();
    let show_ui_window = |ui: &mut egui::Ui| {
        if system_state.controlling_camera {
            ui.disable();
        }
        if ui.button("Control Camera").clicked() {
            let mut window = main_window.get_single_mut().unwrap();
            window.cursor.visible = false;
            window.cursor.grab_mode = CursorGrabMode::Confined;
            system_state.controlling_camera = true;
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
            for i in 0..robot_system.displacements.len() {
                let displacement = &mut robot_system.displacements[i];
                let limit = robot_system.limits[i];
                let slider = egui::Slider::new(displacement, limit.0..=limit.1);
                ui.add(slider);
            }
            robot_system.update_from_displacements();
        });
        ui.collapsing("Collision Detection", |ui| {
            if ui.button("detect").clicked() {
                robot_system.detect_collision();
            };
            for collision in &robot_system.last_collisions {
                ui.label(format!("{:?}", collision));
            }
        });
        gizmos.axes(system_state.target_transform, 0.1);
        ui.collapsing("Target Transform", |ui| {
            rot_pos_control(&mut system_state.target_transform, ui);
            let target_matrix = transform_to_matrix(system_state.target_transform);
            let solutions = robot::ik::solve_nova(&robot_system.limits, target_matrix);
            ui.label("Solutions: ");
            for solution in solutions {
                if ui.button("Apply").clicked() {
                    robot_system.displacements.copy_from_slice(&solution);
                    robot_system.update_from_displacements();
                }
                let mut solution_text = String::new();
                for a in solution {
                    write!(&mut solution_text, "{:.2}, ", a).unwrap();
                }
                ui.label(solution_text);
            }
        });
        ui.collapsing("Rtt", |ui| {
            if ui.button("New").clicked() {
                let target_matrix = transform_to_matrix(system_state.target_transform);
                system_state.rtt = Rtt::new_with_target_matrix(target_matrix, robot_system);
            }
            if let Some(ref mut rtt) = system_state.rtt {
                if ui.button("Step").clicked() {
                    for _ in 0..system_state.rtt_n_steps {
                        rtt.step(robot_system);
                    }
                }
                ui.checkbox(&mut system_state.rtt_auto_step, "Auto Step");
                if system_state.rtt_auto_step {
                    for _ in 0..system_state.rtt_n_steps {
                        rtt.step(robot_system);
                    }
                }
                ui.add(egui::Slider::new(&mut system_state.rtt_n_steps, 1..=100));
            }
            ui.label("Rtt State: ");
            ui.label(format!("{:?}", system_state.rtt));
        });
        ui.collapsing("AStar", |ui| {
            if ui.button("New").clicked() {
                let target_matrix = transform_to_matrix(system_state.target_transform);
                let startf = Vector6::from(robot_system.displacements);
                let solutions = robot_system.get_valid_target_solitions(target_matrix);
                let goalf = Vector6::from(solutions[0]);
                let step_size = system_state.astar_graph.step_size;
                let start = (startf / step_size).try_cast::<i32>().unwrap();
                let goal = (goalf / step_size).try_cast::<i32>().unwrap();
                system_state.astar = Some(AStar::new(start, goal));
            }
            if let Some(ref mut astar) = system_state.astar {
                if ui.button("Step").clicked() {
                    let mut graph = system_state.astar_graph.build(robot_system);
                    astar.step(
                        &mut graph,
                    );
                }
                ui.checkbox(&mut system_state.astar_auto_step, "Auto step");
                if system_state.astar_auto_step {
                    let mut graph = system_state.astar_graph.build(robot_system);
                    astar.step(
                        &mut graph,
                    );
                }
                ui.label(format!("finished: {}", astar.finished));
            }
        });
    };
    egui::Window::new("UI")
        .show(contexts.ctx_mut(), |ui| {
            egui::ScrollArea::vertical().show(ui, show_ui_window);
        })
        .unwrap();

    // Camera Controlling
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
// fn format_matrix<R, C, S>(name: &str, m: &rapier3d::na::Matrix<f32, R, C, S>) -> String
// where
//     R: rapier3d::na::Dim,
//     C: rapier3d::na::Dim,
//     S: rapier3d::na::RawStorage<f32, R, C>,
// {
//     let mut result = name.to_owned();
//     for row in m.row_iter() {
//         result += "\n";
//         for v in row {
//             result += &format!("{:.3}, ", v);
//         }
//     }
//     result
// }
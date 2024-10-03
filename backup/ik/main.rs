use bevy::prelude::*;
use bevy_egui::EguiPlugin;

mod camera;
mod ik;
mod robot;
mod ui;

fn main() {
    App::new()
        .insert_resource(ClearColor(
            bevy::color::palettes::tailwind::SLATE_950.into(),
        ))
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_systems(Startup, system_setup)
        .add_plugins(ui::ui_plugin)
        .add_plugins(camera::camera_plugin)
        .add_plugins(robot::robot_plugin)
        .run();
}

#[derive(Resource)]
pub struct SystemState {
    controlling_camera: bool,
}

fn system_setup(mut commands: Commands) {
    let ui_system = SystemState {
        controlling_camera: false,
    };
    commands.insert_resource(ui_system);

    // Simple Scene
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 200000.0,
            shadows_enabled: true,
            color: bevy::color::palettes::css::BISQUE.into(),
            ..default()
        },
        transform: Transform::from_xyz(-1.0, 3.0, -1.0),
        ..default()
    });
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 150000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(1.0, 1.0, 1.0),
        ..default()
    });
}

use bevy::prelude::*;
use bevy_egui::EguiPlugin;

mod camera;
mod math;
mod robot;
mod scene;
mod ui;

fn main() {
    App::new()
        .insert_resource(ClearColor(
            bevy::color::palettes::tailwind::SLATE_950.into(),
        ))
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_plugins(scene::scene_plugin)
        .add_plugins(ui::ui_plugin)
        .add_plugins(camera::camera_plugin)
        .add_plugins(robot::robot_plugin)
        .run();
}

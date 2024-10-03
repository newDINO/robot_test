use bevy::diagnostic::DiagnosticsStore;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::prelude::*;

#[derive(Component)]
pub struct DebugText;

#[derive(Resource)]
pub struct DebugTextData {
    pub text: String,
}

pub fn setup_debug(
    mut commands: Commands,
) {
    commands.insert_resource(DebugTextData {
        text: "".to_string(),
    });
    commands.spawn((
        TextBundle {
            text: Text::from_section("", TextStyle::default()),
            ..default()
        },
        DebugText
    ));
}

pub fn update_fps(
    mut text_data: ResMut<DebugTextData>,
    diagnostics: Res<DiagnosticsStore>,
) {
    if let Some(value) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
        if let Some(value) = value.smoothed() {
            let fps_text = format!("FPS: {:.2}", value);
            text_data.text.push_str(&fps_text);
        }
    } else {
        text_data.text.push_str("FPS: N/A");
    }
}

pub fn update_text(
    mut text_data: ResMut<DebugTextData>,
    mut query: Query<&mut Text, With<DebugText>>,
) {
    let mut text = query.get_single_mut().unwrap();
    text.sections[0].value.clear();
    text.sections[0].value.push_str(&text_data.text);
    text_data.text.clear();
    text_data.text.push_str("Debug:\n");
}

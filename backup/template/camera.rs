use bevy::{
    input::{keyboard::KeyboardInput, mouse::MouseMotion, ButtonState},
    prelude::*,
};

use crate::SystemState;

pub fn camera_plugin(app: &mut App) {
    app.add_systems(Startup, camera_setup);
    app.add_systems(Update, camera_update);
}

fn camera_setup(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-1.0, 0.0, 0.0),
        ..default()
    });
    let mut camera_controller = CameraController::new();
    camera_controller.speed = 1.0;
    commands.insert_resource(camera_controller);
}

fn camera_update(
    system_state: Res<SystemState>,
    mut camera_q: Query<&mut Transform, With<Camera3d>>,
    mut controller: ResMut<CameraController>,
    mut keyboard_input: EventReader<KeyboardInput>,
    mut mouse_motion: EventReader<MouseMotion>,
    time: Res<Time>,
) {
    for event in keyboard_input.read() {
        if system_state.controlling_camera {
            controller.keyboard_input(event);
        }
    }
    for event in mouse_motion.read() {
        if system_state.controlling_camera {
            controller.mouse_motion(event);
        }
    }
    let mut camera_transform = camera_q.get_single_mut().unwrap();
    controller.update_camera(&mut camera_transform, time.delta().as_secs_f32());
}

#[derive(Resource)]
struct CameraController {
    pub yaw: f32,
    pub pitch: f32,

    pub amount_left: f32,
    pub amount_right: f32,
    pub amount_forward: f32,
    pub amount_backward: f32,
    pub amount_up: f32,
    pub amount_down: f32,
    pub rotate_horizontal: f32,
    pub rotate_vertical: f32,
    pub speed: f32,
    pub sensitivity: f32,
}

impl CameraController {
    pub fn new() -> Self {
        Self {
            yaw: 0.0,
            pitch: 0.0,
            amount_backward: 0.0,
            amount_down: 0.0,
            amount_forward: 0.0,
            amount_left: 0.0,
            amount_right: 0.0,
            amount_up: 0.0,
            rotate_horizontal: 0.0,
            rotate_vertical: 0.0,
            speed: 5.0,
            sensitivity: 0.3,
        }
    }
    pub fn keyboard_input(&mut self, event: &KeyboardInput) {
        let amount = if event.state == ButtonState::Pressed {
            1.0
        } else {
            0.0
        };
        match event.key_code {
            KeyCode::KeyW => self.amount_forward = amount,
            KeyCode::KeyS => self.amount_backward = amount,
            KeyCode::KeyA => self.amount_left = amount,
            KeyCode::KeyD => self.amount_right = amount,
            KeyCode::Space => self.amount_up = amount,
            KeyCode::ShiftLeft => self.amount_down = amount,
            _ => {}
        }
    }
    pub fn mouse_motion(&mut self, event: &MouseMotion) {
        self.rotate_horizontal = event.delta.x;
        self.rotate_vertical = event.delta.y;
    }
    pub fn update_camera(&mut self, transform: &mut Transform, dt: f32) {
        let (sin_yaw, cos_yaw) = self.yaw.sin_cos();
        let forward = Vec3::new(cos_yaw, 0.0, sin_yaw).normalize();
        let right = Vec3::new(-sin_yaw, 0.0, cos_yaw).normalize();

        transform.translation +=
            forward * (self.amount_forward - self.amount_backward) * self.speed * dt;
        transform.translation += right * (self.amount_right - self.amount_left) * self.speed * dt;

        transform.translation.y += (self.amount_up - self.amount_down) * self.speed * dt;

        self.yaw += self.rotate_horizontal * self.sensitivity * dt;
        self.pitch += -self.rotate_vertical * self.sensitivity * dt;

        self.rotate_horizontal = 0.0;
        self.rotate_vertical = 0.0;

        const SAFE_FRAC_PI_2: f32 = std::f32::consts::FRAC_PI_2 - 0.0001;

        if self.pitch < -SAFE_FRAC_PI_2 {
            self.pitch = -SAFE_FRAC_PI_2;
        } else if self.pitch > SAFE_FRAC_PI_2 {
            self.pitch = SAFE_FRAC_PI_2;
        }
        let (sin_pitch, cos_pitch) = self.pitch.sin_cos();

        *transform = transform.looking_to(
            Vec3::new(cos_pitch * cos_yaw, sin_pitch, cos_pitch * sin_yaw).normalize(),
            Vec3::new(0.0, 1.0, 0.0),
        );
    }
}

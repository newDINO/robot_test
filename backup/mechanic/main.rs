mod camera;
mod debug_ui;

use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::ecs::system::EntityCommands;
use bevy::{
    gltf::{GltfMesh, GltfNode},
    input::{
        keyboard::KeyboardInput,
        mouse::{MouseButtonInput, MouseMotion},
        ButtonState,
    },
    prelude::*,
    window::{CursorGrabMode, PrimaryWindow},
};
use bevy_rapier3d::prelude::*;
use camera::CameraController;

fn main() {
    App::new()
        .insert_resource(ClearColor(
            bevy::color::palettes::tailwind::SLATE_950.into(),
        ))
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(FrameTimeDiagnosticsPlugin)
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, debug_ui::setup_debug)
        .add_systems(Startup, setup)
        .add_systems(Startup, setup_scene)
        .add_systems(Startup, load_gltf)
        .add_systems(Update, spawn_gltf_objects)
        .add_systems(Update, update)
        .add_systems(PreUpdate, debug_ui::update_text)
        .add_systems(PreUpdate, debug_ui::update_fps)
        .run();
}

#[derive(Resource)]
struct ArmScene(Handle<Gltf>);

fn load_gltf(mut commands: Commands, asset_server: Res<AssetServer>) {
    let gltf = asset_server.load("CR5-MODLE.gltf");
    // let gltf = asset_server.load("/home/zyh/Downloads/CR5-MODLE.gltf");
    commands.insert_resource(ArmScene(gltf));
}

fn spawn_gltf_objects(
    mut commands: Commands,
    arm_scene: Res<ArmScene>,
    gltf_assets: Res<Assets<Gltf>>,
    gltf_node_assets: Res<Assets<GltfNode>>,
    gltf_mesh_assets: Res<Assets<GltfMesh>>,
    mesh_assets: Res<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut loaded: Local<bool>,
) {
    // Only do this once. A bad implementation!
    if *loaded {
        return;
    }

    let Some(gltf) = gltf_assets.get(&arm_scene.0) else {
        return;
    };
    *loaded = true;

    let base_node = gltf_node_assets
        .get(&gltf.named_nodes["BASE__5^CR5-MODLE(2)_CR5-MODLE"])
        .unwrap();
    let mut entity_commands = commands.spawn(RigidBody::Fixed);
    spawn_as_a_rigid(
        &mut entity_commands,
        base_node,
        &gltf_mesh_assets,
        &mesh_assets,
        &mut materials,
    );
    let parent_id = entity_commands.id();

    let mut j1_base_joint = RevoluteJoint::new(Vec3::Y);
    j1_base_joint.set_motor_velocity(1.0, 1.0);
    j1_base_joint.set_contacts_enabled(false);

    let j1_node = gltf_node_assets
        .get(&gltf.named_nodes["25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE"])
        .unwrap();
    let mut entity_commands = commands.spawn(RigidBody::Dynamic);
    spawn_as_a_rigid(
        &mut entity_commands,
        j1_node,
        &gltf_mesh_assets,
        &mesh_assets,
        &mut materials,
    );
    entity_commands.insert(ImpulseJoint::new(parent_id, j1_base_joint));
    // let parent_id = entity_commands.id();

    // let mut j2_j1_joint = RevoluteJoint::new(Vec3::Z);
    // j2_j1_joint.set_local_anchor1(Vec3::new(1.0, 1.0, 1.0));
    // j2_j1_joint.set_local_anchor2(Vec3::new(1.0, 1.0, 1.0));
    // j2_j1_joint.set_motor_velocity(1.0, 1.0);
    // j2_j1_joint.set_contacts_enabled(false);

    // let j2_node = gltf_node_assets
    //     .get(&gltf.named_nodes["25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE001"])
    //     .unwrap();
    // let mut entity_commands = commands.spawn(RigidBody::Dynamic);
    // spawn_as_a_rigid(
    //     &mut entity_commands,
    //     j2_node,
    //     &gltf_mesh_assets,
    //     &mesh_assets,
    //     &mut materials,
    // );
    // entity_commands.insert(ImpulseJoint::new(parent_id, j2_j1_joint));
}

fn spawn_as_a_rigid(
    commands: &mut EntityCommands,
    gltf_node: &GltfNode,
    gltf_mesh_assets: &Res<Assets<GltfMesh>>,
    mesh_assets: &Res<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    if let Some(gltf_mesh_handle) = gltf_node.mesh.as_ref() {
        let gltf_mesh = gltf_mesh_assets.get(gltf_mesh_handle).unwrap();
        commands
            .insert(TransformBundle {
                local: gltf_node.transform,
                ..Default::default()
            })
            .with_children(|child_builder| {
                for primitive in &gltf_mesh.primitives {
                    let mesh = mesh_assets.get(&primitive.mesh).unwrap();
                    child_builder
                        .spawn(
                            Collider::from_bevy_mesh(mesh, &ComputedColliderShape::TriMesh)
                                .unwrap(),
                        )
                        .insert(PbrBundle {
                            mesh: primitive.mesh.clone(),
                            material: materials.add(StandardMaterial {
                                base_color: Color::WHITE,
                                metallic: 0.2,
                                ..Default::default()
                            }),
                            ..Default::default()
                        });
                }
            });
    } else {
        commands
            .insert(TransformBundle {
                local: gltf_node.transform,
                ..Default::default()
            })
            .with_children(|new_child_builder| {
                for child in &gltf_node.children {
                    spawn_none_rigid(
                        new_child_builder,
                        child,
                        gltf_mesh_assets,
                        mesh_assets,
                        materials,
                    );
                }
            });
    }
}

fn spawn_none_rigid(
    child_builder: &mut ChildBuilder,
    gltf_node: &GltfNode,
    gltf_mesh_assets: &Res<Assets<GltfMesh>>,
    mesh_assets: &Res<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    if let Some(gltf_mesh_handle) = gltf_node.mesh.as_ref() {
        let gltf_mesh = gltf_mesh_assets.get(gltf_mesh_handle).unwrap();
        child_builder
            .spawn(TransformBundle {
                local: gltf_node.transform,
                ..Default::default()
            })
            .with_children(|child_builder| {
                for primitive in &gltf_mesh.primitives {
                    let mesh = mesh_assets.get(&primitive.mesh).unwrap();
                    child_builder
                        .spawn(
                            Collider::from_bevy_mesh(mesh, &ComputedColliderShape::TriMesh)
                                .unwrap(),
                        )
                        .insert(PbrBundle {
                            mesh: primitive.mesh.clone(),
                            material: materials.add(StandardMaterial {
                                base_color: Color::WHITE,
                                metallic: 0.2,
                                ..Default::default()
                            }),
                            ..Default::default()
                        });
                }
            });
    } else {
        child_builder
            .spawn(TransformBundle {
                local: gltf_node.transform,
                ..Default::default()
            })
            .with_children(|new_child_builder| {
                for child in &gltf_node.children {
                    spawn_none_rigid(
                        new_child_builder,
                        child,
                        gltf_mesh_assets,
                        mesh_assets,
                        materials,
                    );
                }
            });
    }
}

fn setup_scene(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let material = materials.add(Color::WHITE);

    /* Create the ground. */
    commands
        .spawn(Collider::cuboid(100.0, 0.1, 100.0))
        .insert(Restitution::coefficient(1.0))
        .insert(PbrBundle {
            mesh: meshes.add(Cuboid::from_size(Vec3::new(100.0, 0.1, 100.0) * 2.0)),
            material: material.clone(),
            transform: Transform::from_xyz(0.0, -1.1, 0.0),
            ..Default::default()
        });

    // Add lights
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 150000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(-1.0, 1.0, 1.0),
        ..default()
    });
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 200000.0,
            shadows_enabled: true,
            color: bevy::color::palettes::css::BISQUE.into(),
            ..default()
        },
        transform: Transform::from_xyz(1.0, 3.0, 1.0),
        ..default()
    });
}
#[derive(Resource)]
struct SystemState {
    controlling: bool,
}
impl SystemState {
    fn new() -> Self {
        Self { controlling: false }
    }
}
fn setup(mut commands: Commands) {
    commands.insert_resource(SystemState::new());

    // Add a camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-1.0, 0.0, 0.0),
        ..default()
    });
    let mut camera_controller = CameraController::new();
    camera_controller.speed = 2.0;
    commands.insert_resource(camera_controller);
}

fn update(
    mut camera_q: Query<&mut Transform, With<Camera3d>>,
    mut controller: ResMut<CameraController>,
    mut system_state: ResMut<SystemState>,
    mut keyboard_input: EventReader<KeyboardInput>,
    mut mouse_motion: EventReader<MouseMotion>,
    mut mouse_button: EventReader<MouseButtonInput>,
    time: Res<Time>,
    mut windows: Query<&mut Window, With<PrimaryWindow>>,
) {
    for event in keyboard_input.read() {
        if event.key_code == KeyCode::Escape {
            if system_state.controlling {
                system_state.controlling = false;
                windows.iter_mut().for_each(|mut window| {
                    window.cursor.visible = true;
                    window.cursor.grab_mode = CursorGrabMode::None;
                });
            }
        }
        if system_state.controlling {
            controller.keyboard_input(event);
        }
    }
    for event in mouse_motion.read() {
        if system_state.controlling {
            controller.mouse_motion(event);
        }
    }
    for event in mouse_button.read() {
        if event.state == ButtonState::Pressed {
            if event.button == MouseButton::Left {
                if !system_state.controlling {
                    system_state.controlling = true;
                    windows.iter_mut().for_each(|mut window| {
                        window.cursor.visible = false;
                        window.cursor.grab_mode = CursorGrabMode::Confined;
                    });
                }
            }
        }
    }

    let mut camera_transform = camera_q.get_single_mut().unwrap();
    controller.update_camera(&mut camera_transform, time.delta().as_secs_f32());
}

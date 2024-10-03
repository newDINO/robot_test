mod camera;
mod debug_ui;

use std::collections::HashMap;

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
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy_rapier3d::prelude::*;
use camera::CameraController;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(ClearColor(
            bevy::color::palettes::tailwind::SLATE_950.into(),
        ))
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_systems(Startup, load_gltf)
        .add_systems(Startup, setup)
        .add_systems(Startup, debug_ui::setup_debug)
        .add_systems(Update, spawn_gltf_objects)
        .add_systems(Update, update)
        .add_systems(Update, rotate_arms)
        .add_systems(Update, show_intersected_objects)
        .add_systems(PreUpdate, debug_ui::update_text)
        .add_systems(PreUpdate, debug_ui::update_fps)
        .run();
}

#[derive(Resource)]
struct ArmScene(Handle<Gltf>);

fn load_gltf(mut commands: Commands, asset_server: Res<AssetServer>) {
    let gltf = asset_server.load("t.gltf");
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

    let root_node = gltf_node_assets
        .get(&gltf.named_nodes["CR5-MODLE(2)^CR5-MODLE"])
        .unwrap();
    // print_gltf_nodes(root_node, 0);

    let robot_transform = Transform::default();

    let arm_axis: HashMap<&str, Dir3> = HashMap::from([
        ("25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE", Dir3::Y),
        ("25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE001", Dir3::Y),
        ("U-ARM_ASM_3_ASM^CR5-MODLE(2)_CR5-MODLE", Dir3::X),
        ("14-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE001", Dir3::X),
        ("J6_ASM_7_ASM^CR5-MODLE(2)_CR5-MODLE", Dir3::X),
    ]);

    commands
        .spawn((
            PbrBundle {
                transform: robot_transform,
                ..Default::default()
            },
            Robot,
        ))
        .with_children(|child_builder| {
            spawn_robot(
                child_builder,
                root_node,
                &gltf_mesh_assets,
                &mesh_assets,
                &mut materials,
                &arm_axis,
            )
        });
}

// fn print_gltf_nodes(parent: &GltfNode, depth: usize) {
//     for node in &parent.children {
//         println!("{:indent$}{}", "", node.name, indent = depth * 4);
//         print_gltf_nodes(&node, depth + 1);
//     }
// }

#[derive(Component)]
struct Robot;

#[derive(Component)]
struct ArmJoint {
    axis: Dir3,
}

fn spawn_robot(
    child_builder: &mut ChildBuilder,
    node: &GltfNode,
    gltf_mesh_assets: &Res<Assets<GltfMesh>>,
    mesh_assets: &Res<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    arm_axis: &HashMap<&str, Dir3>,
) {
    if let Some(gltf_mesh_handle) = &node.mesh {
        let gltf_mesh = gltf_mesh_assets.get(gltf_mesh_handle).unwrap();
        child_builder.spawn(RigidBody::KinematicPositionBased)
            .insert(TransformBundle {
                local: node.transform,
                ..Default::default()
            })
            .with_children(|new_child_builder| {
                for primitive in &gltf_mesh.primitives {
                    let pbr_bundle = PbrBundle {
                        mesh: primitive.mesh.clone(),
                        material: materials.add(StandardMaterial {
                            base_color: Color::WHITE,
                            metallic: 0.2,
                            ..Default::default()
                        }),
                        ..Default::default()
                    };
                    // new_child_builder.spawn(pbr_bundle);
                    let mesh = mesh_assets.get(&primitive.mesh).unwrap();
                    let collider = Collider::from_bevy_mesh(mesh, &ComputedColliderShape::TriMesh).unwrap();
                    new_child_builder
                        .spawn(collider)
                        .insert(Sensor)
                        .insert(ActiveEvents::COLLISION_EVENTS)
                        .insert(pbr_bundle);
                }
            });
    } else {
        if let Some(axis) = arm_axis.get(node.name.as_str()) {
            child_builder.spawn((
                TransformBundle {
                    local: node.transform,
                    ..default()
                },
                ArmJoint { axis: *axis },
            ))
        } else {
            child_builder.spawn(TransformBundle {
                local: node.transform,
                ..default()
            })
        }.with_children(|new_child_builder| {
            for child in &node.children {
                spawn_robot(
                    new_child_builder,
                    child,
                    gltf_mesh_assets,
                    mesh_assets,
                    materials,
                    arm_axis,
                );
            }
        });
    }
}

fn rotate_arms(mut arm_joints: Query<(&mut Transform, &ArmJoint)>, time: Res<Time>) {
    for (mut transform, joint) in arm_joints.iter_mut() {
        transform.rotate_local_axis(joint.axis, time.delta_seconds());
    }
}

// fn show_intersected_objects(
//     mut collision_events: EventReader<CollisionEvent>,
//     mut materials: ResMut<Assets<StandardMaterial>>,
//     material_query: Query<&Handle<StandardMaterial>>,
// ) {
//     for collision_event in collision_events.read() {
//         match collision_event {
//             CollisionEvent::Started(entity1, entity2, _) => {
//                 let handle0 = material_query.get(*entity1).unwrap();
//                 let handle1 = material_query.get(*entity2).unwrap();
//                 materials.get_mut(handle0).unwrap().base_color = Color::srgb(1.0, 0.0, 0.0);
//                 materials.get_mut(handle1).unwrap().base_color = Color::srgb(1.0, 0.0, 0.0);
//             },
//             CollisionEvent::Stopped(entity1, entity2, _) => {
//                 let handle0 = material_query.get(*entity1).unwrap();
//                 let handle1 = material_query.get(*entity2).unwrap();
//                 materials.get_mut(handle0).unwrap().base_color = Color::WHITE;
//                 materials.get_mut(handle1).unwrap().base_color = Color::WHITE;
//             },
//         }
//     }
// }
// fn show_intersected_objects(
//     mut materials: ResMut<Assets<StandardMaterial>>,
//     rapier_context: Res<RapierContext>,
//     material_query: Query<&Handle<StandardMaterial>>,
// ) {
//     for (entity1, entity2, intersected) in rapier_context.intersection_pairs() {
//         if intersected {
//             let handle1 = material_query.get(entity1).unwrap();
//             let material1 = materials.get_mut(handle1).unwrap();
//             material1.base_color = Color::srgb(1.0, 0.7, 0.6);
//             let handle2 = material_query.get(entity2).unwrap();
//             let material2 = materials.get_mut(handle2).unwrap();
//             material2.base_color = Color::srgb(1.0, 0.7, 0.6);
//         }
//     }
// }
// fn show_intersected_objects(
//     mut materials: ResMut<Assets<StandardMaterial>>,
//     rapier_context: Res<RapierContext>,
//     material_query: Query<&Handle<StandardMaterial>>,
//     // mut debug_text_data: ResMut<DebugTextData>,
// ) {
//     for contact_pair in rapier_context.contact_pairs() {
//         if contact_pair.find_deepest_contact().is_some() {
//             let handle1 = material_query.get(contact_pair.collider1()).unwrap();
//             let material1 = materials.get_mut(handle1).unwrap();
//             material1.base_color = Color::srgb(1.0, 0.7, 0.6);
//             let handle2 = material_query.get(contact_pair.collider2()).unwrap();
//             let material2 = materials.get_mut(handle2).unwrap();
//             material2.base_color = Color::srgb(1.0, 0.7, 0.6);
//         }
//     }
// }

fn show_intersected_objects(
    mut materials: ResMut<Assets<StandardMaterial>>,
    material_query: Query<&Handle<StandardMaterial>>,
    rapier_context: Res<RapierContext>,
    query: Query<Entity, With<Collider>>,
) {
    for e1 in query.iter() {
        for e2 in query.iter() {
            if e1 == e2 {
                continue;
            }
            let handle1 = material_query.get(e1).unwrap();
            let material1 = materials.get_mut(handle1).unwrap();
            if rapier_context.intersection_pair(e1, e2).is_some() {
                material1.base_color = Color::srgb(1.0, 0.0, 0.0);
            } else {
                material1.base_color = Color::WHITE;
            }
        }
    }
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
    camera_controller.speed = 1.0;
    commands.insert_resource(camera_controller);

    // Add a light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 200000.0,
            shadows_enabled: true,
            color: bevy::color::palettes::css::BISQUE.into(),
            ..default()
        },
        transform: Transform::from_xyz(-1.0, 1.0, 1.0),
        ..default()
    });
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 150000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(1.0, 3.0, 1.0),
        ..default()
    });
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

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use rapier3d::prelude::*;

use crate::{
    math::transform_to_isometry,
    robot::RobotSystem,
    ui::{rot_pos_control, SystemState},
};

pub fn scene_plugin(app: &mut App) {
    app.add_systems(PostStartup, scene_setup);
    app.add_systems(Update, scene_update);
}

fn scene_setup(mut commands: Commands) {
    // Simple Scene
    commands
        .spawn(PointLightBundle {
            point_light: PointLight {
                intensity: 200000.0,
                shadows_enabled: true,
                color: bevy::color::palettes::css::BISQUE.into(),
                ..default()
            },
            transform: Transform::from_xyz(-1.0, 3.0, -1.0),
            ..default()
        })
        .insert(SceneObject {
            name: "Light1".to_owned(),
        });
    commands
        .spawn(PointLightBundle {
            point_light: PointLight {
                intensity: 150000.0,
                shadows_enabled: true,
                ..default()
            },
            transform: Transform::from_xyz(1.0, 1.0, 1.0),
            ..default()
        })
        .insert(SceneObject {
            name: "Light2".to_owned(),
        });
}

fn scene_update(
    mut commands: Commands,
    mut contexts: EguiContexts,
    system_state: ResMut<SystemState>,
    mut gizmos: Gizmos,

    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut objects: Query<(&mut Transform, &SceneObject, Entity, &SceneCollider)>,
    mesh_handles: Query<&Handle<Mesh>>,
    material_handles: Query<&Handle<StandardMaterial>>,
    collider_handles: Query<&SceneCollider>,

    robot_system: ResMut<RobotSystem>,
) {
    let robot_system = robot_system.into_inner();
    let show_ui = |ui: &mut egui::Ui| {
        if system_state.controlling_camera {
            ui.disable();
        }
        ui.collapsing("Add Object", |ui| {
            let mut shape = SpawnShape::Cuboid;
            egui::ComboBox::from_id_source("add object")
                .selected_text(format!("{:?}", shape))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut shape, SpawnShape::Cuboid, "Cuboid");
                });
            let (mesh, collider) = match shape {
                SpawnShape::Cuboid => (
                    Mesh::from(bevy::prelude::Cuboid::new(0.1, 0.1, 0.1)),
                    ColliderBuilder::cuboid(0.05, 0.05, 0.05),
                ),
            };
            let collider = collider
                .active_events(ActiveEvents::COLLISION_EVENTS)
                .sensor(true);
            if ui.button("Add").clicked() {
                commands.spawn((
                    SceneObject {
                        name: "Object".to_owned(),
                    },
                    PbrBundle {
                        mesh: meshes.add(mesh),
                        material: materials.add(Color::WHITE),
                        ..Default::default()
                    },
                    SceneCollider {
                        handle: robot_system.collider_set.insert(collider),
                    },
                ));
            }
        });

        for (transform, object, entity, scene_collider) in objects.iter_mut() {
            let transform = transform.into_inner();
            let openness = egui::CollapsingHeader::new(&object.name)
                .id_source(entity.to_bits())
                .show(ui, |ui| {
                    rot_pos_control(transform, ui);
                    if ui.button("Remove").clicked() {
                        if let Ok(mesh_handle) = mesh_handles.get(entity) {
                            meshes.remove(mesh_handle);
                        }
                        if let Ok(material_handle) = material_handles.get(entity) {
                            materials.remove(material_handle);
                        }
                        if let Ok(collider) = collider_handles.get(entity) {
                            let mut island_manager = IslandManager::new();
                            robot_system.collider_set.remove(
                                collider.handle,
                                &mut island_manager,
                                &mut robot_system.rigid_body_set,
                                true,
                            );
                        }
                        commands.entity(entity).despawn();
                    }
                })
                .openness;
            if openness > 0.5 {
                gizmos.axes(*transform, 0.1);
                if let Some(collider) = robot_system.collider_set.get_mut(scene_collider.handle) {
                    collider.set_position(transform_to_isometry(transform));
                };
            }
        }
        ui.collapsing("Debug", |ui| {
            ui.label("Meshes:");
            for (id, _) in meshes.iter() {
                ui.label(format!("{:?}", id));
            }
            ui.label("Materials:");
            for (id, _) in materials.iter() {
                ui.label(format!("{:?}", id));
            }
            ui.label("Colliders");
            for (handle, _) in robot_system.collider_set.iter() {
                ui.label(format!("{:?}", handle));
            }
        });
    };
    egui::Window::new("Scene")
        .default_pos((3000.0, 100.0))
        .show(contexts.ctx_mut(), |ui| {
            egui::ScrollArea::vertical().show(ui, show_ui);
        });
}

#[derive(Component)]
struct SceneObject {
    name: String,
}

#[derive(Component)]
struct SceneCollider {
    handle: ColliderHandle,
}

#[derive(PartialEq, Debug)]
enum SpawnShape {
    Cuboid,
}

use std::sync::Mutex;

pub mod ik;
pub mod rtt;
pub mod astar;

use bevy::{
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology, VertexAttributeValues},
};
use nalgebra::{Matrix4, Point3, Vector3, Vector6};
use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot, UrdfRobotHandles};

use crate::math::isometry_to_transform;

pub fn robot_plugin(app: &mut App) {
    app.add_systems(Startup, robot_setup);
    app.add_systems(Update, robot_update);
}

fn robot_setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();

    // load the robot to physics system
    let options = UrdfLoaderOptions {
        create_colliders_from_visual_shapes: true,
        create_colliders_from_collision_shapes: false,
        make_roots_fixed: true,
        // Z-up to Y-up.
        // shift: Isometry::rotation(-Vector::x() * std::f32::consts::FRAC_PI_2),
        ..Default::default()
    };
    let (robot, _) =
        UrdfRobot::from_file("assets/nova5_urdf/urdf/nova5.SLDASM.urdf", options, None).unwrap();
    let robot_handles = robot.insert_using_multibody_joints(
        &mut rigid_body_set,
        &mut collider_set,
        &mut multibody_joint_set,
        UrdfMultibodyOptions::empty(),
    );

    // robot system
    let mut robot_system = RobotSystem::new(
        rigid_body_set,
        collider_set,
        multibody_joint_set,
        robot_handles,
    );
    robot_system.update_multi_body();

    // spawn visual meshes
    let material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        metallic: 0.1,
        ..Default::default()
    });
    for (handle, collider) in robot_system.collider_set.iter() {
        if let Some(trimesh) = collider.shape().as_trimesh() {
            let mesh = mesh_from_trimesh(trimesh);
            commands
                .spawn(PbrBundle {
                    mesh: meshes.add(mesh),
                    transform: isometry_to_transform(collider.position()),
                    material: material.clone(),
                    ..Default::default()
                })
                .insert(ColliderId(handle));
        }
    }

    commands.insert_resource(robot_system);
}

fn robot_update(
    robot_system: Res<RobotSystem>,
    mut collider_transforms: Query<(&mut Transform, &ColliderId)>,
    mut gizmos: Gizmos,
) {
    // update graphics
    for (mut transform, collider_id) in collider_transforms.iter_mut() {
        let collider = robot_system.collider_set.get(collider_id.0).unwrap();
        *transform = isometry_to_transform(collider.position());
    }

    // gizmos
    gizmos.axes(Transform::IDENTITY, 0.1);
    let end_id = 6;
    let end_rigid_handle = robot_system.robot_handles.links[end_id].body;
    let end_isometry = robot_system
        .rigid_body_set
        .get(end_rigid_handle)
        .unwrap()
        .position();
    gizmos.axes(isometry_to_transform(end_isometry), 0.1);
}

#[derive(Component)]
struct ColliderId(ColliderHandle);

#[derive(Resource)]
pub struct RobotSystem {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub multibody_joint_set: MultibodyJointSet,

    pub robot_handles: UrdfRobotHandles<Option<MultibodyJointHandle>>,

    pub limits: [(f32, f32); 6],
    pub displacements: [f32; 6],
    pub last_collisions: Vec<(ColliderHandle, ColliderHandle)>,
}
impl RobotSystem {
    pub fn update_from_displacements(&mut self) {
        for i in 0..self.displacements.len() {
            let d = self.displacements[i];
            let limit = self.limits[i];
            if d < limit.0 || d > limit.1 {
                panic!("Displacement out of limit");
            }
            let link = self.get_link_mut(i);
            let mut new_joint = MultibodyJoint::new(link.joint.data, true);
            let displacement = [d, 0.0, 0.0, 0.0, 0.0, 0.0];
            new_joint.apply_displacement(&displacement);
            link.joint = new_joint;
        }
        self.update_multi_body();
    }
    fn new(
        rigid_body_set: RigidBodySet,
        mut collider_set: ColliderSet,
        multibody_joint_set: MultibodyJointSet,
        robot_handles: UrdfRobotHandles<Option<MultibodyJointHandle>>,
    ) -> Self {
        // setting collision flags
        for (_, collider) in collider_set.iter_mut() {
            collider.set_active_events(ActiveEvents::COLLISION_EVENTS);
            collider.set_sensor(true); // Using sensor for performance ? But contact detection also failed to filter joint for unknown reasons.
            collider.set_active_collision_types(
                ActiveCollisionTypes::DYNAMIC_DYNAMIC | ActiveCollisionTypes::DYNAMIC_FIXED,
            );
        }
        // limits
        let mut limits = [(0.0, 0.0); 6];
        for i in 0..6 {
            let link = robot_handles.joints[i].joint.unwrap();
            let (multi_body, id) = multibody_joint_set.get(link).unwrap();
            let joint = multi_body.link(id).unwrap();
            let limit = joint.joint.data.limits[3];
            limits[i] = (limit.min, limit.max);
        }
        Self {
            rigid_body_set,
            collider_set,
            multibody_joint_set,
            robot_handles,
            limits,
            displacements: [0.0; 6],
            last_collisions: Vec::new(),
        }
    }
    fn update_multi_body(&mut self) {
        for joint in &self.robot_handles.joints {
            let handle = joint.joint.unwrap();
            let (multi_body, _) = self.multibody_joint_set.get_mut(handle).unwrap();
            multi_body.forward_kinematics(&mut self.rigid_body_set, true);
            multi_body.update_rigid_bodies(&mut self.rigid_body_set, true);
        }
        for (_, body) in self.rigid_body_set.iter() {
            for handle in body.colliders() {
                if let Some(collider) = self.collider_set.get_mut(*handle) {
                    let new_pos = body.position() * collider.position_wrt_parent().unwrap();
                    collider.set_position(new_pos);
                }
            }
        }
    }
    fn get_link_mut(&mut self, i: usize) -> &mut MultibodyLink {
        let link = self.robot_handles.joints[i].joint.unwrap();
        let (multi_body, i) = self.multibody_joint_set.get_mut(link).unwrap();
        multi_body.link_mut(i).unwrap()
    }
    // pub fn get_link(&mut self, i: usize) -> &MultibodyLink {
    //     let link = self.robot_handles.joints[i].joint.unwrap();
    //     let (multi_body, i) = self.multibody_joint_set.get(link).unwrap();
    //     multi_body.link(i).unwrap()
    // }
    pub fn detect_collision(&mut self) {
        let event_handler = CollisionEventHandler::new();
        let mut broad_phase = EmptyBroadPhase;
        let mut narrow_phase = NarrowPhase::new();
        CollisionPipeline::new().step(
            0.0,
            &mut broad_phase,
            &mut narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            None,
            &(),
            &event_handler,
        );
        self.last_collisions = event_handler.collisions.into_inner().unwrap();
    }
    pub fn get_valid_target_solitions(&mut self, target_matrix: Matrix4<f32>) -> Vec<[f32; 6]> {
        let start = Vector6::from(self.displacements);
        let mut solutions = ik::solve_nova(&self.limits, target_matrix);
        solutions.retain(|solution| {
            self.displacements.copy_from_slice(solution);
            self.update_from_displacements();
            self.detect_collision();
            self.last_collisions.len() == 0
        });
        // self back to start
        self.displacements.copy_from_slice(start.as_slice());
        self.update_from_displacements();
        solutions
    }
}

struct EmptyBroadPhase;
impl BroadPhase for EmptyBroadPhase {
    fn update(
        &mut self,
        _dt: f32,
        _prediction_distance: f32,
        colliders: &mut ColliderSet,
        _bodies: &RigidBodySet,
        _modified_colliders: &[ColliderHandle],
        _removed_colliders: &[ColliderHandle],
        events: &mut Vec<BroadPhasePairEvent>,
    ) {
        for (handle1, collider1) in colliders.iter() {
            let aabb1 = collider1.compute_aabb();
            for (handle2, collider2) in colliders.iter() {
                if handle1 == handle2 {
                    continue;
                }
                let aabb2 = collider2.compute_aabb();
                if aabb1.intersects(&aabb2) {
                    events.push(BroadPhasePairEvent::AddPair(ColliderPair {
                        collider1: handle1,
                        collider2: handle2,
                    }));
                }
            }
        }
    }
}

struct CollisionEventHandler {
    collisions: Mutex<Vec<(ColliderHandle, ColliderHandle)>>,
}
impl CollisionEventHandler {
    fn new() -> Self {
        Self {
            collisions: Mutex::new(Vec::new()),
        }
    }
}
impl EventHandler for CollisionEventHandler {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        match event {
            CollisionEvent::Started(h1, h2, _) => {
                let h1_index = h1.0.into_raw_parts().0;
                let h2_index = h2.0.into_raw_parts().0;
                if !(h1_index < 6 && h2_index < 6 && h1_index.abs_diff(h2_index) <= 1) {
                    self.collisions.lock().unwrap().push((h1, h2));
                }
            }
            CollisionEvent::Stopped(_h1, _h2, _) => (),
        };
        // let (h1, h2) = match event {
        //     CollisionEvent::Started(h1, h2, _) => (h1, h2),
        //     CollisionEvent::Stopped(h1, h2, _) => (h1, h2),
        // };
        // if h1.0.into_raw_parts().0.abs_diff(h2.0.into_raw_parts().0) != 1 {
        //     self.collisions.lock().unwrap().push((h1, h2));
        // }
    }
    fn handle_contact_force_event(
        &self,
        _dt: f32,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        _contact_pair: &ContactPair,
        _total_force_magnitude: f32,
    ) {
    }
}

fn mesh_from_trimesh(trimesh: &TriMesh) -> Mesh {
    let vtx = trimesh.vertices();
    let idx = trimesh.indices();
    let mut normals: Vec<[f32; 3]> = vec![];
    let mut vertices: Vec<[f32; 3]> = vec![];

    for idx in idx {
        let a = vtx[idx[0] as usize];
        let b = vtx[idx[1] as usize];
        let c = vtx[idx[2] as usize];

        vertices.push(a.cast::<f32>().into());
        vertices.push(b.cast::<f32>().into());
        vertices.push(c.cast::<f32>().into());
    }

    for vtx in vertices.chunks(3) {
        let a = Point3::from(vtx[0]);
        let b = Point3::from(vtx[1]);
        let c = Point3::from(vtx[2]);
        let n = (b - a).cross(&(c - a)).normalize();
        normals.push(n.cast::<f32>().into());
        normals.push(n.cast::<f32>().into());
        normals.push(n.cast::<f32>().into());
    }

    normals
        .iter_mut()
        .for_each(|n| *n = Vector3::from(*n).normalize().into());
    let indices: Vec<_> = (0..vertices.len() as u32).collect();
    let uvs: Vec<_> = (0..vertices.len()).map(|_| [0.0, 0.0]).collect();

    // Generate the mesh
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, Default::default());
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        VertexAttributeValues::from(vertices),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, VertexAttributeValues::from(normals));
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, VertexAttributeValues::from(uvs));
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

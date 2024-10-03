use std::sync::Mutex;

use bevy::{
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology, VertexAttributeValues},
};
use nalgebra::{Point3, Vector3};
use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot, UrdfRobotHandles};

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
    robot_system: ResMut<RobotSystem>,
    mut collider_transforms: Query<(&mut Transform, &ColliderId)>,
) {
    // update graphics
    for (mut transform, collider_id) in collider_transforms.iter_mut() {
        let collider = robot_system.collider_set.get(collider_id.0).unwrap();
        *transform = isometry_to_transform(collider.position());
    }
}

#[derive(Component)]
struct ColliderId(ColliderHandle);

#[derive(Resource)]
pub struct RobotSystem {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub multibody_joint_set: MultibodyJointSet,

    pub robot_handles: UrdfRobotHandles<Option<MultibodyJointHandle>>,

    pub displacements: [SpatialVector<f32>; 6],
    pub last_collisions: Vec<(ColliderHandle, ColliderHandle)>,
}
impl RobotSystem {
    pub fn new(
        rigid_body_set: RigidBodySet,
        mut collider_set: ColliderSet,
        multibody_joint_set: MultibodyJointSet,
        robot_handles: UrdfRobotHandles<Option<MultibodyJointHandle>>,
    ) -> Self {
        // setting collision flags
        for (_, collider) in collider_set.iter_mut() {
            collider.set_active_events(ActiveEvents::COLLISION_EVENTS);
            collider.set_sensor(true); // Using sensor for performance ? But contact detection also failed to filter joint for unknown reasons.
        }
        Self {
            rigid_body_set,
            collider_set,
            multibody_joint_set,
            robot_handles,
            displacements: [nalgebra::zero(); 6],
            last_collisions: Vec::new(),
        }
    }
    pub fn update_multi_body(&mut self) {
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
    pub fn get_link_mut(&mut self, i: usize) -> &mut MultibodyLink {
        let link = self.robot_handles.joints[i].joint.unwrap();
        let (multi_body, i) = self.multibody_joint_set.get_mut(link).unwrap();
        multi_body.link_mut(i).unwrap()
    }
    pub fn get_link(&mut self, i: usize) -> &MultibodyLink {
        let link = self.robot_handles.joints[i].joint.unwrap();
        let (multi_body, i) = self.multibody_joint_set.get(link).unwrap();
        multi_body.link(i).unwrap()
    }
    pub fn detect_collision(&mut self) {
        let event_handler = CollisionEventHandler::new();
        let mut broad_phase = DefaultBroadPhase::new();
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
            CollisionEvent::Started(h1, h2, _) => self.collisions.lock().unwrap().push((h1, h2)),
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

fn isometry_to_transform(isometry: &Isometry<f32>) -> Transform {
    let t1 = isometry.translation;
    let t2 = Vec3::new(t1.x, t1.y, t1.z);
    let r1 = isometry.rotation.as_vector();
    let r2 = Quat::from_xyzw(r1.x, r1.y, r1.z, r1.w);
    Transform {
        translation: t2,
        rotation: r2,
        scale: Vec3::ONE,
    }
}

use bevy::{
    core::FrameCount,
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology, VertexAttributeValues},
};
use nalgebra::{Point3, Vector3};
use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot};

use crate::ik::RobotNova;

pub fn robot_plugin(app: &mut App) {
    app.add_systems(Startup, robot_setup);
    app.add_systems(Update, robot_update);
}

fn robot_setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // PHysics System
    let mut physics_system = PhysicsSystem::new();

    // Load URDF
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
    robot.insert_using_multibody_joints(
        &mut physics_system.rigid_body_set,
        &mut physics_system.collider_set,
        &mut physics_system.multibody_joint_set,
        UrdfMultibodyOptions::empty(),
    );

    // Get joints
    let joint_handles: Vec<_> = physics_system
        .multibody_joint_set
        .iter()
        .map(|(_, link_id, _, _)| (link_id.multibody, link_id.id))
        .collect();
    commands.insert_resource(JointHandles { idx: joint_handles });

    // spawn the solver
    commands.insert_resource(RobotSolver(RobotNova::default()));

    // Spawn Meshes
    let material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        metallic: 0.1,
        ..Default::default()
    });
    for (handle, collider) in physics_system.collider_set.iter() {
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

    commands.insert_resource(physics_system);
}

fn robot_update(
    physics_system: ResMut<PhysicsSystem>,
    joint_handles: Res<JointHandles>,
    mut collider_transforms: Query<(&mut Transform, &ColliderId)>,
    mut gizmos: Gizmos,
    mut robot_solver: ResMut<RobotSolver>,
    framecount: Res<FrameCount>,
) {
    let physics_system = physics_system.into_inner();
    // wake up all bodies for manual motor setting
    for (_, _, _, link) in physics_system.multibody_joint_set.iter() {
        physics_system.island_manager.wake_up(
            &mut physics_system.rigid_body_set,
            link.rigid_body_handle(),
            true,
        );
    }

    // physics step
    physics_system.step();

    // update graphics
    for (mut transform, collider_id) in collider_transforms.iter_mut() {
        let collider = physics_system.collider_set.get(collider_id.0).unwrap();
        *transform = isometry_to_transform(collider.position());
    }

    // Draw axis
    gizmos.axes(Transform::IDENTITY, 0.1);
    let end_index = 5;
    let (multibody_id, link_id) = joint_handles.idx[end_index];
    let multibody = physics_system
        .multibody_joint_set
        .get_multibody(multibody_id)
        .unwrap();
    let link = multibody.link(link_id).unwrap();
    let rigid_body = physics_system
        .rigid_body_set
        .get(link.rigid_body_handle())
        .unwrap();
    gizmos.axes(isometry_to_transform(rigid_body.position()), 0.1);

    // solve the robot
    let nova = &mut robot_solver.0;
    let end_rot = rigid_body.rotation().to_rotation_matrix();
    let end_pos = rigid_body.translation();
    nova.solve(&rigid_body.rotation(), end_pos);
    if framecount.0 % 60 == 0 {
        nova.solve_num(end_rot, *end_pos);
    }
}

#[derive(Resource)]
pub struct RobotSolver(pub RobotNova);

#[derive(Resource)]
pub struct JointHandles {
    pub idx: Vec<(MultibodyIndex, usize)>,
}

#[derive(Component)]
struct ColliderId(ColliderHandle);

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

#[derive(Resource)]
pub struct PhysicsSystem {
    pub gravity: Vector3<f32>,
    pub physics_pipeline: PhysicsPipeline,
    pub integration_parameters: IntegrationParameters,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,

    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
}

impl PhysicsSystem {
    fn step(&mut self) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &(),
            &(),
        );
    }
    fn new() -> Self {
        let integration_parameters = IntegrationParameters::default();
        let physics_pipeline = PhysicsPipeline::new();
        let island_manager = IslandManager::new();
        let broad_phase = DefaultBroadPhase::new();
        let narrow_phase = NarrowPhase::new();
        let ccd_solver = CCDSolver::new();
        let query_pipeline = QueryPipeline::new();
        let gravity = vector![0.0, -9.81, 0.0];

        let rigid_body_set = RigidBodySet::new();
        let collider_set = ColliderSet::new();
        let impulse_joint_set = ImpulseJointSet::new();
        let multibody_joint_set = MultibodyJointSet::new();
        Self {
            integration_parameters,
            physics_pipeline,
            island_manager,
            broad_phase,
            narrow_phase,
            ccd_solver,
            query_pipeline,
            gravity,
            rigid_body_set,
            collider_set,
            impulse_joint_set,
            multibody_joint_set,
        }
    }
}

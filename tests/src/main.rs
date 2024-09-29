use std::f32::consts::PI;

use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot};
use rapier_testbed3d::{Testbed, TestbedApp};

fn main() {
    let builders: Vec<(_, fn(&mut Testbed))> = vec![("robot", setup_testbed)];
    let testbed_app = TestbedApp::from_builders(0, builders);
    testbed_app.run();
}

fn setup_testbed(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    // Ground
    // colliders.insert(ColliderBuilder::cuboid(100.0, 0.01, 100.0).translation(vector![0.0, -0.2, 0.0]));

    // Robot
    let options = UrdfLoaderOptions {
        create_colliders_from_visual_shapes: true,
        create_colliders_from_collision_shapes: false,
        make_roots_fixed: true,
        // Z-up to Y-up.
        shift: Isometry::rotation(-Vector::x() * std::f32::consts::FRAC_PI_2),
        ..Default::default()
    };

    let (robot, _) = UrdfRobot::from_file(
        "assets/nova5_urdf/urdf/nova5.SLDASM.urdf",
        options,
        None,
        // Some(std::path::Path::new("/home/zyh/Downloads/DOBOT Nova 5_URDF/nova5_urdf/meshes")),
    )
    .unwrap();

    robot.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        UrdfMultibodyOptions::empty(),
    );

    let joint_handles: Vec<_> = multibody_joints
        .iter()
        .map(|(handle, _, _, _)| handle)
        .collect();

    let mut joint_counter = 0;
    for handle in joint_handles {
        let (body, i) = multibody_joints.get_mut(handle).unwrap();
        println!("body: {}", i);
        for link in body.links_mut() {
            println!("link: {}", link.link_id());
            if let Some(joint) = link.joint.data.as_revolute_mut() {
                joint.set_limits([-2.0 * PI, 2.0 * PI]);
                joint.set_motor_velocity(1.0, 1.0);
                println!("Joint: {}", joint_counter);
                joint_counter += 1;
            } else {
                println!("Not a revolute joint");
            }
        }
    }
    for (_, collider) in colliders.iter_mut() {
        collider.set_active_events(ActiveEvents::COLLISION_EVENTS);
    }

    /*
     * Set up the testbed.
     */
    testbed.add_callback(|_, _, event, _| {
        for event in event.collision_events.try_iter() {
            println!("{:?}", event);
        }
    });
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![1.0, 1.0, 1.0], point![0.0, 0.0, 0.0]);
}

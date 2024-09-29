use std::{
    collections::HashMap,
    fs::File,
    io::{Read, Write},
};

use nalgebra::{UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

fn main() {
    let gltf_string = read_file_to_string("/home/zyh/Downloads/CR5-MODLE.gltf");

    let mut gltf_json: serde_json::Value = serde_json::from_str(&gltf_string).unwrap();

    let mut nodes: Vec<GltfNode> = serde_json::from_value(gltf_json["nodes"].clone()).unwrap();
    let mut named_nodes = HashMap::<String, usize>::new();
    for i in 0..nodes.len() {
        named_nodes.insert(nodes[i].name.clone().unwrap(), i);
    }

    init_parent(&mut nodes);
    init_transform(&mut nodes);

    // 25JOINT z
    // 25JOINT001 y
    // UARM y
    // 14JOINT001 y
    // JS z

    append_child_conserve_global_transform(
        &mut nodes,
        *named_nodes
            .get("25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE")
            .unwrap(),
        *named_nodes
            .get("25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE001")
            .unwrap(),
    );
    append_child_conserve_global_transform(
        &mut nodes,
        *named_nodes
            .get("25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE001")
            .unwrap(),
        *named_nodes.get("L-ARM_3^CR5-MODLE(2)_CR5-MODLE").unwrap(),
    );
    append_child_conserve_global_transform(
        &mut nodes,
        *named_nodes
            .get("25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE001")
            .unwrap(),
        *named_nodes
            .get("25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE002")
            .unwrap(),
    );
    append_child_conserve_global_transform(
        &mut nodes,
        *named_nodes
            .get("25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE001")
            .unwrap(),
        *named_nodes
            .get("U-ARM_ASM_3_ASM^CR5-MODLE(2)_CR5-MODLE")
            .unwrap(),
    );
    append_child_conserve_global_transform(
        &mut nodes,
        *named_nodes
            .get("U-ARM_ASM_3_ASM^CR5-MODLE(2)_CR5-MODLE")
            .unwrap(),
        *named_nodes
            .get("14-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE")
            .unwrap(),
    );
    append_child_conserve_global_transform(
        &mut nodes,
        *named_nodes
            .get("U-ARM_ASM_3_ASM^CR5-MODLE(2)_CR5-MODLE")
            .unwrap(),
        *named_nodes
            .get("14-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE001")
            .unwrap(),
    );
    append_child_conserve_global_transform(
        &mut nodes,
        *named_nodes
            .get("14-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE001")
            .unwrap(),
        *named_nodes
            .get("J6_ASM_7_ASM^CR5-MODLE(2)_CR5-MODLE")
            .unwrap(),
    );

    let offset = 0.08;

    let u_arm = *named_nodes
        .get("U-ARM_ASM_3_ASM^CR5-MODLE(2)_CR5-MODLE")
        .unwrap();
    let (mut u_arm_origin, _) = get_global_transform(&mut nodes, u_arm);
    u_arm_origin.y = 0.574 - offset;
    set_global_origin(&mut nodes, u_arm, u_arm_origin);

    let joint = *named_nodes
        .get("14-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE001")
        .unwrap();
    let (mut joint_origin, _) = get_global_transform(&mut nodes, joint);
    joint_origin.y = 0.931 - offset;
    set_global_origin(&mut nodes, joint, joint_origin);

    let asm = *named_nodes
        .get("J6_ASM_7_ASM^CR5-MODLE(2)_CR5-MODLE")
        .unwrap();
    let (mut asm_origin, _) = get_global_transform(&mut nodes, asm);
    asm_origin.z = -0.141;
    set_global_origin(&mut nodes, asm, asm_origin);

    // struct_print(&nodes, 0, 0);

    gltf_json["nodes"] = serde_json::to_value(nodes).unwrap();
    let mut output_file = File::create("assets/t.gltf").unwrap();
    output_file
        .write_all(serde_json::to_string_pretty(&gltf_json).unwrap().as_bytes())
        .unwrap();
}

#[derive(Debug, Deserialize, Serialize)]
struct GltfNode {
    name: Option<String>,
    mesh: Option<usize>,
    children: Option<Vec<usize>>,
    translation: Option<Vector3<f32>>,
    rotation: Option<UnitQuaternion<f32>>,
    parent: Option<usize>,
}

fn append_child_conserve_global_transform(
    nodes: &mut Vec<GltfNode>,
    parent_index: usize,
    child_index: usize,
) {
    let (global_translation, global_rotation) = get_global_transform(nodes, child_index);
    append_child(nodes, parent_index, child_index);
    set_global_transform(nodes, child_index, global_translation, global_rotation);
}

fn append_child(nodes: &mut Vec<GltfNode>, parent_index: usize, child_index: usize) {
    let children = nodes[parent_index].children.as_mut().unwrap();
    children.push(child_index);

    if let Some(former_parent_index) = nodes[child_index].parent {
        let former_parent = &mut nodes[former_parent_index];
        former_parent
            .children
            .as_mut()
            .unwrap()
            .retain(|x| *x != child_index);
    }

    nodes[child_index].parent = Some(parent_index);
}

fn get_global_transform(
    nodes: &[GltfNode],
    node_index: usize,
) -> (Vector3<f32>, UnitQuaternion<f32>) {
    let mut translation = nodes[node_index].translation.unwrap();
    let mut rotation = nodes[node_index].rotation.unwrap();

    let parent_index = nodes[node_index].parent;
    if let Some(parent_index) = parent_index {
        let (parent_translation, parent_rotation) = get_global_transform(nodes, parent_index);
        translation = parent_rotation * translation + parent_translation;
        rotation = parent_rotation * rotation;
    }
    (translation, rotation)
}

fn global_to_local(
    nodes: &[GltfNode],
    parent_index: usize,
    global_pos: Vector3<f32>,
) -> Vector3<f32> {
    let (parent_translation, parent_rotation) = get_global_transform(nodes, parent_index);
    parent_rotation.inverse() * (global_pos - parent_translation)
}

// fn local_to_global(nodes: &[GltfNode], parent_index: usize, local_pos: Vector3<f32>) -> Vector3<f32> {
//     let parent_translation = nodes[parent_index].translation.unwrap();
//     let parent_rotation = nodes[parent_index].rotation.unwrap();
//     let parent_local = parent_rotation * local_pos + parent_translation;
//     if let Some(grand_parent_index) = nodes[parent_index].parent {
//         local_to_global(nodes, grand_parent_index, parent_local)
//     } else {
//         parent_local
//     }
// }

fn set_origin(nodes: &mut [GltfNode], node_index: usize, origin: Vector3<f32>) {
    let delta = nodes[node_index].translation.unwrap() - origin;
    let rotation = nodes[node_index].rotation.unwrap();
    nodes[node_index].translation = Some(origin);
    for child_index in nodes[node_index].children.as_ref().unwrap() {
        nodes[*child_index].translation =
            Some(rotation.inverse() * delta + nodes[*child_index].translation.unwrap());
    }
}

fn set_global_origin(nodes: &mut [GltfNode], node_index: usize, global_origin: Vector3<f32>) {
    if let Some(parent_index) = nodes[node_index].parent {
        let local_origin = global_to_local(nodes, parent_index, global_origin);
        set_origin(nodes, node_index, local_origin);
    } else {
        set_origin(nodes, node_index, global_origin);
    }
}

// Change the local transform to set the global transform, without changing parent transform.
fn set_global_transform(
    nodes: &mut [GltfNode],
    node_index: usize,
    global_translation: Vector3<f32>,
    global_rotation: UnitQuaternion<f32>,
) {
    let parent_index = nodes[node_index].parent;
    if let Some(parent_index) = parent_index {
        let (parent_translation, parent_rotation) = get_global_transform(nodes, parent_index);
        let local_translation =
            parent_rotation.inverse() * (global_translation - parent_translation);
        let local_rotation = parent_rotation.inverse() * global_rotation;
        nodes[node_index].translation = Some(local_translation);
        nodes[node_index].rotation = Some(local_rotation);
    } else {
        nodes[node_index].translation = Some(global_translation);
        nodes[node_index].rotation = Some(global_rotation);
    }
}

fn init_parent(nodes: &mut [GltfNode]) {
    for i in 0..nodes.len() {
        let children_clone = nodes[i].children.clone();
        if let Some(ref children) = children_clone {
            for child in children {
                if nodes[*child].parent.is_none() {
                    nodes[*child].parent = Some(i);
                } else {
                    panic!("child already has parent")
                }
            }
        }
    }
}

fn init_transform(nodes: &mut [GltfNode]) {
    for i in 0..nodes.len() {
        if nodes[i].rotation.is_none() {
            nodes[i].rotation = Some(UnitQuaternion::identity());
        }
        if nodes[i].translation.is_none() {
            nodes[i].translation = Some(Vector3::zeros());
        }
    }
}

// fn struct_print(nodes: &[GltfNode], node_index: usize, depth: usize) {
//     println!("{}{}", " ".repeat(depth * 4), nodes[node_index].name.as_ref().unwrap());
//     if let Some(ref children) = nodes[node_index].children {
//         for child in children {
//             struct_print(nodes, *child, depth + 1);
//         }
//     }
// }

fn read_file_to_string(path: &str) -> String {
    let mut string = String::new();
    let mut file = File::open(path).unwrap();
    file.read_to_string(&mut string).unwrap();
    string
}

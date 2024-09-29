use std::{collections::HashMap, fs::File, io::{Read, Write}};

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

    append_child_by_name(
        &mut nodes,
        named_nodes,
        "25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE",
        "25-JIONT_ASM_4_ASM^CR5-MODLE(2)_CR5-MODLE001"
    );
    struct_print(&nodes, 0, 0);

    gltf_json["nodes"] = serde_json::to_value(nodes).unwrap();
    let mut output_file = File::create("t.gltf").unwrap();
    output_file.write_all(serde_json::to_string_pretty(&gltf_json).unwrap().as_bytes()).unwrap();
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
fn append_child_by_name(nodes: &mut Vec<GltfNode>, named_nodes: HashMap<String, usize>, parent_name: &str, child_name: &str) {
    let parent_index = named_nodes.get(parent_name).unwrap();
    let child_index = named_nodes.get(child_name).unwrap();

    let children = nodes[*parent_index].children.as_mut().unwrap();
    children.push(*child_index);

    if let Some(former_parent_index) = nodes[*child_index].parent {
        let former_parent = &mut nodes[former_parent_index];
        former_parent.children.as_mut().unwrap().retain(|x| *x != *child_index);
    }
    
    nodes[*child_index].parent = Some(*parent_index);
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

fn struct_print(nodes: &[GltfNode], node_index: usize, depth: usize) {
    println!("{}{}", " ".repeat(depth * 4), nodes[node_index].name.as_ref().unwrap());
    if let Some(ref children) = nodes[node_index].children {
        for child in children {
            struct_print(nodes, *child, depth + 1);
        }
    }
}

fn read_file_to_string(path: &str) -> String {
    let mut string = String::new();
    let mut file = File::open(path).unwrap();
    file.read_to_string(&mut string).unwrap();
    string
}
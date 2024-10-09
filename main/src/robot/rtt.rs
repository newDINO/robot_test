use std::fmt::Debug;

use nalgebra::Matrix4;
use rand::prelude::*;
use rapier3d::na::{vector, Vector6};
use rapier3d::prelude::*;

use super::RobotSystem;

pub struct Rtt {
    min: Vector6<f32>,
    max: Vector6<f32>,
    root_index: usize,
    nodes: Vec<Node>,
    targets: Vec<Vector6<f32>>,
    rng: SmallRng,
    step_size: f32,
    current_best: usize,
    current_best_distance: f32,
    end_node_index: Option<usize>,

    path: Vec<Vector6<f32>>, // from end to start
}
impl Debug for Rtt {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "end node: {:?}", self.end_node_index)?;
        writeln!(f, "current best: {}", self.current_best_distance)?;
        writeln!(f, "number of nodes: {}", self.nodes.len())?;
        writeln!(f, "root node: {:?}", self.nodes[self.root_index])?;
        writeln!(f, "min: {:?}", self.min)?;
        write!(f, "max: {:?}", self.max)?;
        Ok(())
    }
}

impl Rtt {
    pub fn new_with_target_matrix(
        target: Matrix4<f32>,
        robot_system: &mut RobotSystem,
    ) -> Option<Self> {
        let start = Vector6::from(robot_system.displacements);
        let solutions = robot_system.get_valid_target_solitions(target);
        if solutions.len() == 0 {
            None
        } else {
            let targets = solutions
                .iter()
                .map(|solution| Vector6::from_row_slice(solution))
                .collect::<Vec<Vector6<f32>>>();
            let limtis = &robot_system.limits;
            let min = vector![
                limtis[0].0,
                limtis[1].0,
                limtis[2].0,
                limtis[3].0,
                limtis[4].0,
                limtis[5].0,
            ];
            let max = vector![
                limtis[0].1,
                limtis[1].1,
                limtis[2].1,
                limtis[3].1,
                limtis[4].1,
                limtis[5].1,
            ];
            Some(Self::new(min, max, start, targets))
        }
    }
    pub fn new(
        min: Vector6<f32>,
        max: Vector6<f32>,
        start: Vector6<f32>,
        targets: Vec<Vector6<f32>>,
    ) -> Self {
        let mut nodes = Vec::new();
        let root = Node {
            parent: u32::MAX,
            children: Vec::new(),
            pos: start,
        };
        nodes.push(root);
        Self {
            min,
            max,
            root_index: 0,
            nodes,
            targets,
            rng: SmallRng::from_entropy(),
            step_size: 0.1,
            end_node_index: None,
            current_best: 0,
            current_best_distance: f32::INFINITY,
            path: Vec::new(),
        }
    }
    pub fn step(&mut self, robot_system: &mut RobotSystem) {
        if self.end_node_index.is_some() {
            return;
        }
        let random_x = self.random_pos();
        let (near_index, distance) = self.nodes.iter().enumerate().fold(
            (
                self.root_index,
                (self.nodes[self.root_index].pos - random_x).magnitude(),
            ),
            |(last_index, last_distance), (current_index, node)| {
                let current_distance = (node.pos - random_x).magnitude();
                if current_distance < last_distance {
                    (current_index, current_distance)
                } else {
                    (last_index, last_distance)
                }
            },
        );
        let near_node = &self.nodes[near_index];
        let new_x = if distance == 0.0 {
            return;
        } else if distance < self.step_size {
            random_x
        } else {
            near_node.pos + (random_x - near_node.pos) / distance * self.step_size
        };
        robot_system.displacements.copy_from_slice(new_x.as_slice());
        robot_system.update_from_displacements();
        // robot_system.detect_collision();
        let new_index = if robot_system.last_collisions.len() == 0 {
            let new_index = self.nodes.len();
            self.nodes.push(Node {
                parent: near_index as u32,
                children: Vec::new(),
                pos: new_x,
            });
            self.nodes[near_index].children.push(new_index as u32);
            new_index
        } else {
            return;
        };
        for target in &self.targets {
            let target_distance = (new_x - target).magnitude();
            if target_distance < self.current_best_distance {
                self.current_best = new_index;
                self.current_best_distance = target_distance;
            }
            if (new_x - target).magnitude() < self.step_size {
                self.end_node_index = Some(new_index);

                self.path.push(*target);
                let mut current = new_index;
                loop {
                    let node = &self.nodes[current];
                    self.path.push(node.pos);
                    if current == self.root_index {
                        break;
                    }
                    current = node.parent as usize;
                }
                return;
            }
        }
    }
    fn random_pos(&mut self) -> Vector6<f32> {
        vector![
            self.rng.gen_range(self.min[0]..self.max[0]),
            self.rng.gen_range(self.min[1]..self.max[1]),
            self.rng.gen_range(self.min[2]..self.max[2]),
            self.rng.gen_range(self.min[3]..self.max[3]),
            self.rng.gen_range(self.min[4]..self.max[4]),
            self.rng.gen_range(self.min[5]..self.max[5]),
        ]
    }
}

#[derive(Debug)]
struct Node {
    children: Vec<u32>,
    parent: u32,
    pos: Vector6<f32>,
}

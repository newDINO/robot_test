use std::collections::HashSet;

use rapier3d::na::Vector6;

use crate::robot::RobotSystem;
use super::Graph;

const DIM: u32 = 6;
const SPAN: usize = 3;
const N: usize = SPAN.pow(DIM);
fn offsets() -> [Vector6<i32>; N] {
    let mut result = [Vector6::zeros(); N];
    for i in 0..N {
        for e in 0..DIM {
            result[i][e as usize] = i as i32 / SPAN.pow(e) as i32 % SPAN as i32;
        }
    }
    result
}

pub struct RobotGraph {
    offsets: [Vector6<i32>; N],
    pub step_size: f32,
    free_cells: HashSet<Vector6<i32>>,
    obstacles: HashSet<Vector6<i32>>,
}
impl RobotGraph {
    pub fn new(step_size: f32) -> Self {
        Self {
            offsets: offsets(),
            step_size,
            free_cells: HashSet::new(),
            obstacles: HashSet::new(),
        }
    }
    pub fn build<'a>(&'a mut self, robot_system: &'a mut RobotSystem) -> RobotGraphRef<'a> {
        let step_size = self.step_size;
        RobotGraphRef {
            robot_system, graph: self, step_size,
        }
    }
}

pub struct RobotGraphRef<'a> {
    robot_system: &'a mut RobotSystem,
    graph: &'a mut RobotGraph,
    step_size: f32,
}
impl<'a> Graph for RobotGraphRef<'a> {
    type Element = Vector6<i32>;
    fn neighbors(&mut self, x: Self::Element) -> impl Iterator<Item = Self::Element> {
        NeighborIter {
            center: x,
            robot_system: self.robot_system,
            graph: self.graph,
            index: 0,
            step_size: self.step_size,
        }
    }
    fn cost_fn(&self) -> Box<dyn Fn(Self::Element, Self::Element) -> f32> {
        // let step_size = self.step_size;
        Box::new(move |a: Vector6<i32>, b: Vector6<i32>| {
            // let ta = new_nova_fk((a.cast::<f32>() * step_size).as_slice());
            // let tb = new_nova_fk((b.cast::<f32>() * step_size).as_slice());
            // let dx = ta.translation.vector - tb.translation.vector;
            // let dr = 0.3 * ta.rotation.angle_to(&tb.rotation);
            // (dx.magnitude_squared() + dr * dr).sqrt()
            (a - b).cast::<f32>().magnitude()
        })
    }
    fn heuristic_fn(&self) -> Box<dyn Fn(Self::Element, Self::Element) -> f32> {
        // let step_size = self.step_size;
        Box::new(move |a: Vector6<i32>, b: Vector6<i32>| {
            // let ta = new_nova_fk((a.cast::<f32>() * step_size).as_slice());
            // let tb = new_nova_fk((b.cast::<f32>() * step_size).as_slice());
            // let dx = ta.translation.vector - tb.translation.vector;
            // let dr = 0.3 * ta.rotation.angle_to(&tb.rotation);
            // dx.abs().sum() + dr.abs()
            (a - b).abs().sum() as f32
        })
    }
}

pub struct NeighborIter<'a> {
    center: Vector6<i32>,
    robot_system: &'a mut RobotSystem,
    graph: &'a mut RobotGraph,
    index: usize,
    step_size: f32,
}

impl Iterator for NeighborIter<'_> {
    type Item = Vector6<i32>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.index == N.div_ceil(2) {
            self.index += 1;
            return self.next();
        }
        let offset = self.graph.offsets.get(self.index)?;
        self.index += 1;
        let result = self.center + offset;
        
        if self.graph.free_cells.contains(&result) {
            return Some(result);
        }
        if self.graph.obstacles.contains(&result) {
            return self.next();
        }

        let real_coord = result.cast::<f32>() * self.step_size;
        self.robot_system.displacements.copy_from_slice(real_coord.as_slice());
        self.robot_system.update_from_displacements();
        self.robot_system.detect_collision();

        if self.robot_system.last_collisions.len() > 0 {
            self.graph.obstacles.insert(result);
            self.next()
        } else {
            self.graph.free_cells.insert(result);
            Some(result)
        }
    }
}
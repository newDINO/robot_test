use rapier3d::na::Vector6;

use super::{astar::AStar, RobotSystem};

pub struct ArmPath {
    pub points: Vec<Vector6<f32>>, // from end to start
    pub index: usize,
}

impl ArmPath {
    pub fn new() -> Self {
        Self {
            points: vec![Vector6::zeros()],
            index: 0,
        }
    }
    pub fn from_to(from: Vector6<f32>, to: Vector6<f32>, step_size: f32) -> Self {
        let dist = (from - to).magnitude();
        let dir = (from - to) / dist;
        let mut a = 0.0;
        let mut path = Vec::new();
        while a < dist {
            path.push(to + a * dir);
            a += step_size;
        }
        path.push(from);
        let last_index = path.len() - 1;
        Self {
            points: path,
            index: last_index,
        }
    }
    pub fn from_astar(astar: &AStar<Vector6<i32>>, step_size: f32) -> Self {
        let mut last = astar.goal;
        let mut points = vec![last.cast::<f32>() * step_size];
        while let Some(came_from) = astar.came_from[&last] {
            last = came_from;
            points.push(last.cast::<f32>() * step_size);
        }
        let index = points.len() - 1;
        Self {
            points, index,
        }
    }
    pub fn update_robot(&self, robot_system: &mut RobotSystem) {
        robot_system
            .displacements
            .copy_from_slice(&self.points[self.index].as_slice());
        robot_system.update_from_displacements();
    }
}

use std::{collections::{BinaryHeap, HashMap}, hash::Hash};

mod robot;
pub use robot::*;

pub struct AStar<T: Eq + Hash + Copy> {
    frontier: BinaryHeap<Ranked<T>>,
    came_from: HashMap<T, Option<T>>,
    cost_so_far: HashMap<T, f32>,
    goal: T,
    pub finished: bool,
}

impl<T: Eq + Hash + Copy> AStar<T> {
    pub fn new(start: T, goal: T) -> Self {
        Self {
            frontier: BinaryHeap::from([Ranked::new(start, 0.0)]),
            came_from: HashMap::from([(start, None)]),
            cost_so_far: HashMap::from([(start, 0.0)]),
            goal,
            finished: false,
        }
    }
    pub fn step(
        &mut self,
        graph: &mut impl Graph<Element = T>,
    ) {
        if self.finished {
            return;
        }
        let current = self.frontier.pop().unwrap().value;
        if current == self.goal {
            self.finished = true;
            return;
        }
        let cost_fn = graph.cost_fn();
        let heuristic_fn = graph.heuristic_fn();
        for next in graph.neighbors(current) {
            let new_cost = self.cost_so_far[&current] + cost_fn(current, next);
            if let Some(former_cost) = self.cost_so_far.get_mut(&next) {
                if new_cost < *former_cost {
                    *former_cost = new_cost;
                    let priority = new_cost + heuristic_fn(self.goal, next);
                    self.frontier.push(Ranked::new(next, priority));
                    *self.came_from.get_mut(&next).unwrap() = Some(current);
                }
            } else {
                self.cost_so_far.insert(next, new_cost);
                self.came_from.insert(next, Some(current));
                let priority = new_cost + heuristic_fn(self.goal, next);
                self.frontier.push(Ranked::new(next, priority));
            }
        }
    }
}


pub trait Graph {
    type Element;
    fn neighbors(&mut self, x: Self::Element) -> impl Iterator<Item = Self::Element>;
    fn cost_fn(&self) -> Box<dyn Fn(Self::Element, Self::Element) -> f32>;
    fn heuristic_fn(&self) -> Box<dyn Fn(Self::Element, Self::Element) -> f32>;
}

struct Ranked<T> {
    value: T,
    priority: f32,
}
impl<T> Ranked<T> {
    fn new(value: T, priority: f32) -> Self {
        Self { value, priority }
    }
}

impl<T> PartialEq for Ranked<T> {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}

impl<T> Eq for Ranked<T> {}

impl<T> PartialOrd for Ranked<T> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T> Ord for Ranked<T> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        if self.priority > other.priority {
            std::cmp::Ordering::Less
        } else if self.priority == other.priority {
            std::cmp::Ordering::Equal
        } else {
            std::cmp::Ordering::Greater
        }
    }
}
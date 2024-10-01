use argmin::{
    core::{CostFunction, Executor, State},
    solver::particleswarm::ParticleSwarm,
};
use nalgebra::{matrix, vector, Vector2};

fn main() {
    let problem = TestProblem;
    let solver = ParticleSwarm::new((vector![-10.0, -10.0], vector![10.0, 10.0]), 100);
    let executor = Executor::new(problem, solver);
    let res = executor
        .configure(|state| state.max_iters(100).target_cost(0.0))
        .run()
        .unwrap();
    println!("{}", res.state().get_best_param().unwrap().position);
    println!("{}", res.state().get_best_cost());
    println!("{}", res.state().get_iter());
    println!("{}", res.state().get_last_best_iter());
}

struct TestProblem;

impl CostFunction for TestProblem {
    type Output = f32;
    type Param = Vector2<f32>;
    fn cost(&self, param: &Self::Param) -> Result<Self::Output, argmin_math::Error> {
        let m = matrix![
            1.0, 2.0;
            3.0, 4.0
        ];
        let cost = m * param;
        Ok(cost.amax())
    }
}

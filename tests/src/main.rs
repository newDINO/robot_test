use nalgebra::matrix;


fn main() {
    let m = matrix![
        1.0, 2.0;
        3.0, 4.0
    ];
    // let x = vector![1.0, 1.0];
    println!("{}", m[(0, 1)]);
}

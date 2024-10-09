use nalgebra::Vector6;

fn main() {
    let offsets = offsets();
    println!("length: {}", offsets.len());
}

const DIM: u32 = 6;
fn offsets() -> [Vector6<i32>; 3_usize.pow(DIM)] {
    const SPAN: usize = 3;
    const N: usize = SPAN.pow(DIM);
    let mut result = [Vector6::zeros(); N];
    for i in 0..N {
        for e in 0..DIM {
            result[i][e as usize] = i as i32 / SPAN.pow(e) as i32 % SPAN as i32;
        }
    }
    result
}
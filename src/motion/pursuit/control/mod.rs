use measurements::Length;

pub enum PursuitDirection {
    Forward,
    Backward,
    Auto,
}

pub mod basic;

pub trait PusuitControl {
    fn control(&self, x: Length, y: Length, lookahead: Length) -> ((f64, f64), bool);
}

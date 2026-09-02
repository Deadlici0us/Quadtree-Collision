//! Simulation dispatch. The crate exposes two parallel sims (boids, elastic
//! collisions) and the mode/view enums used by the WASM surface.

pub mod boids;
pub mod collisions;

/// Which simulation is active.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Mode {
    Boids = 0,
    Collisions = 1,
}

impl Mode {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Mode::Collisions,
            _ => Mode::Boids,
        }
    }
}

/// Which split-screen view the demo is showing.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum View {
    Split = 0,
    BoidsOnly = 1,
    CollisionsOnly = 2,
}

impl View {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => View::BoidsOnly,
            2 => View::CollisionsOnly,
            _ => View::Split,
        }
    }
}

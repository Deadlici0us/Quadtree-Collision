//! `#[wasm_bindgen]` surface — the only module that knows about
//! `wasm-bindgen`; everything else is pure Rust.
//!
//! Memory layout exposed to JS:
//!   - `positions_ptr()` returns a pointer into a stable `Vec<f32>` of
//!     length `4 * total_particles`, laid out as `[x, y, vx, vy]` per
//!     particle. In split view the left-side particles come first,
//!     followed by the right side; JS addresses the right half via
//!     `particle_count_per_side()`.
//!   - `left_qt_nodes_ptr()` / `right_qt_nodes_ptr()` return pointers to
//!     buffers of `[x, y, w, h, depth]` describing the bounding box of
//!     every node in the most-recently-built tree, for the debug overlay.
//!
//! JS wraps the raw pointer in a `Float32Array` over `wasm.memory.buffer`
//! (see `www/main.js`). The buffer is grown automatically by Rust; JS
//! must re-read `memory.buffer` after growth.

use crate::particle::Storage;
use crate::quadtree::{QuadTree, Rect};
use crate::sim::boids::{self, BoidsParams};
use crate::sim::collisions::{self, CollisionsParams};
use crate::sim::{Mode, View};
use wasm_bindgen::prelude::*;

const MAX_PARTICLES_PER_SIDE: usize = 10_000;
const DEFAULT_RADIUS: f32 = 3.0;

struct Side {
    storage: Storage,
    qt: QuadTree,
    qt_nodes: Vec<f32>,
    scratch: Vec<u32>,
    mode: Mode,
    boids: BoidsParams,
    coll: CollisionsParams,
}

impl Side {
    fn new(world: Rect, count: usize, mode: Mode, seed: u32) -> Self {
        let storage = Storage::randomize(count, world, DEFAULT_RADIUS, seed);
        let qt = QuadTree::new(world);
        Self {
            storage,
            qt,
            qt_nodes: Vec::new(),
            scratch: Vec::new(),
            mode,
            boids: BoidsParams::default(),
            coll: CollisionsParams::default(),
        }
    }

    fn rebuild_qt(&mut self) {
        self.qt.clear();
        for i in 0..self.storage.len() {
            self.qt.insert(crate::quadtree::Entry {
                idx: i as u32,
                aabb: self.storage.step_aabb(i),
            });
        }
        self.qt_nodes.clear();
        self.qt.collect_node_bounds(&mut self.qt_nodes);
    }

    fn step(&mut self, dt: f32) {
        match self.mode {
            Mode::Boids => boids::step(
                &mut self.storage,
                &self.qt,
                &self.boids,
                dt,
                &mut self.scratch,
            ),
            Mode::Collisions => collisions::step(
                &mut self.storage,
                &self.qt,
                &self.coll,
                dt,
                &mut self.scratch,
            ),
        }
        self.rebuild_qt();
    }

    fn write_positions(&self, dst: &mut [f32]) {
        for (i, p) in self.storage.parts.iter().enumerate() {
            let base = i * 4;
            dst[base] = p.pos.x();
            dst[base + 1] = p.pos.y();
            dst[base + 2] = p.vel.x();
            dst[base + 3] = p.vel.y();
        }
    }
}

/// Main sandbox handle exposed to JavaScript.
#[wasm_bindgen]
pub struct Sandbox {
    view: View,
    world_w: f32,
    world_h: f32,
    count_per_side: usize,
    left: Option<Side>,
    right: Option<Side>,
    positions: Vec<f32>,
}

#[wasm_bindgen]
impl Sandbox {
    /// Creates a sandbox in `Boids` mode, single view, with the given
    /// particle count (capped at 10_000).
    #[wasm_bindgen(constructor)]
    pub fn new(count: u32, w: f32, h: f32) -> Sandbox {
        let count = (count as usize).min(MAX_PARTICLES_PER_SIDE);
        let world = Rect::new(0.0, 0.0, w, h);
        let left = Side::new(world, count, Mode::Boids, 1);
        let positions = vec![0.0; count * 4];
        Sandbox {
            view: View::BoidsOnly,
            world_w: w,
            world_h: h,
            count_per_side: count,
            left: Some(left),
            right: None,
            positions,
        }
    }

    /// Switches to split view (boids left, collisions right). Always rebuilds
    /// both sides so this is also the canonical "reset" call from JS in
    /// split view.
    pub fn set_view_split(&mut self) {
        let half_w = self.world_w * 0.5;
        let left_world = Rect::new(0.0, 0.0, half_w, self.world_h);
        let right_world = Rect::new(0.0, 0.0, half_w, self.world_h);
        let left = Side::new(left_world, self.count_per_side, Mode::Boids, 1);
        let right = Side::new(right_world, self.count_per_side, Mode::Collisions, 2);
        self.left = Some(left);
        self.right = Some(right);
        self.view = View::Split;
        self.resize_positions();
    }

    /// Switches to a single mode (boids or collisions), full canvas.
    pub fn set_view_single(&mut self, mode: u8) {
        let mode = Mode::from_u8(mode);
        let world = Rect::new(0.0, 0.0, self.world_w, self.world_h);
        let left = Side::new(world, self.count_per_side, mode, 3);
        self.left = Some(left);
        self.right = None;
        self.view = match mode {
            Mode::Collisions => View::CollisionsOnly,
            _ => View::BoidsOnly,
        };
        self.resize_positions();
    }

    fn resize_positions(&mut self) {
        let len = self.total_particles() * 4;
        self.positions = vec![0.0; len];
    }

    /// Sets the world dimensions; re-allocates both sides.
    pub fn set_world(&mut self, w: f32, h: f32) {
        self.world_w = w;
        self.world_h = h;
        match self.view {
            View::Split => self.set_view_split(),
            View::BoidsOnly => self.set_view_single(0),
            View::CollisionsOnly => self.set_view_single(1),
        }
    }

    /// Sets the particle count per side (in split view) or total (in single
    /// view). Capped at 10_000.
    pub fn set_count(&mut self, count: u32) {
        let count = (count as usize).min(MAX_PARTICLES_PER_SIDE);
        if count == self.count_per_side {
            return;
        }
        self.count_per_side = count;
        match self.view {
            View::Split => self.set_view_split(),
            View::BoidsOnly => self.set_view_single(0),
            View::CollisionsOnly => self.set_view_single(1),
        }
    }

    /// Advances the simulation by `dt` seconds.
    pub fn step(&mut self, dt: f32) {
        match self.view {
            View::Split => {
                if let Some(s) = self.left.as_mut() {
                    s.step(dt);
                }
                if let Some(s) = self.right.as_mut() {
                    s.step(dt);
                }
                self.refresh_positions();
            }
            _ => {
                if let Some(s) = self.left.as_mut() {
                    s.step(dt);
                    self.refresh_positions();
                }
            }
        }
    }

    fn refresh_positions(&mut self) {
        match self.view {
            View::Split => {
                if let Some(s) = self.left.as_ref() {
                    s.write_positions(&mut self.positions);
                }
                let offset = self.count_per_side * 4;
                if let Some(s) = self.right.as_ref() {
                    s.write_positions(&mut self.positions[offset..]);
                }
            }
            _ => {
                if let Some(s) = self.left.as_ref() {
                    s.write_positions(&mut self.positions);
                }
            }
        }
    }

    /// Returns a pointer to the flat position buffer (`[x,y,vx,vy]` per
    /// particle, total = 4 * `total_particles`). The buffer is owned by the
    /// sandbox and re-allocated on view/count change.
    pub fn positions_ptr(&self) -> *const f32 {
        self.positions.as_ptr()
    }

    /// Number of f32 elements in the positions buffer.
    pub fn positions_len(&self) -> u32 {
        self.positions.len() as u32
    }

    /// Particle count for the LEFT side (in single mode this is the total).
    pub fn particle_count_per_side(&self) -> u32 {
        self.count_per_side as u32
    }

    /// Total particles across all sides.
    pub fn total_particles(&self) -> usize {
        match self.view {
            View::Split => self.count_per_side * 2,
            _ => self.count_per_side,
        }
    }

    /// Returns the QuadTree debug-overlay node buffer for the LEFT side.
    pub fn left_qt_nodes_ptr(&self) -> *const f32 {
        self.left
            .as_ref()
            .map(|s| s.qt_nodes.as_ptr())
            .unwrap_or(core::ptr::null())
    }

    pub fn left_qt_nodes_len(&self) -> u32 {
        self.left
            .as_ref()
            .map(|s| s.qt_nodes.len() as u32)
            .unwrap_or(0)
    }

    /// Returns the QuadTree debug-overlay node buffer for the RIGHT side
    /// (split view only). Empty in single view.
    pub fn right_qt_nodes_ptr(&self) -> *const f32 {
        self.right
            .as_ref()
            .map(|s| s.qt_nodes.as_ptr())
            .unwrap_or(core::ptr::null())
    }

    pub fn right_qt_nodes_len(&self) -> u32 {
        self.right
            .as_ref()
            .map(|s| s.qt_nodes.len() as u32)
            .unwrap_or(0)
    }

    /// Sets a tunable parameter. Supported names:
    ///   boids: perception, separation_radius, w_sep, w_ali, w_coh,
    ///          max_speed, max_force
    ///   collisions: damping, restitution
    pub fn set_param(&mut self, name: &str, value: f32) {
        let update = |s: &mut Side| match name {
            "perception" => s.boids.perception = value,
            "separation_radius" => s.boids.separation_radius = value,
            "w_sep" => s.boids.w_sep = value,
            "w_ali" => s.boids.w_ali = value,
            "w_coh" => s.boids.w_coh = value,
            "max_speed" => s.boids.max_speed = value,
            "max_force" => s.boids.max_force = value,
            "damping" => s.coll.damping = value,
            "restitution" => s.coll.restitution = value,
            _ => {}
        };
        if let Some(s) = self.left.as_mut() {
            update(s);
        }
        if let Some(s) = self.right.as_mut() {
            update(s);
        }
    }
}

/// Hook used by `wasm-pack test --node` to smoke-test the module without
/// spinning up a browser. Not exposed in the browser build.
#[cfg(all(test, not(target_arch = "wasm32")))]
mod native_tests {
    use super::*;

    #[test]
    fn sandbox_starts_and_steps() {
        let mut sb = Sandbox::new(200, 800.0, 600.0);
        for _ in 0..60 {
            sb.step(1.0 / 60.0);
        }
        // No assertion other than "did not panic" and positions filled.
        assert_eq!(sb.positions_len() as usize, 200 * 4);
    }

    #[test]
    fn split_view_runs_both_sides() {
        let mut sb = Sandbox::new(100, 800.0, 600.0);
        sb.set_view_split();
        for _ in 0..60 {
            sb.step(1.0 / 60.0);
        }
        assert!(sb.left_qt_nodes_len() > 0);
        assert!(sb.right_qt_nodes_len() > 0);
    }

    #[test]
    fn positions_buffer_has_no_nan() {
        let mut sb = Sandbox::new(200, 800.0, 600.0);
        for _ in 0..600 {
            sb.step(1.0 / 60.0);
        }
        let len = sb.positions_len() as usize;
        let slice = &sb.positions[..len];
        for (i, v) in slice.iter().enumerate() {
            assert!(v.is_finite(), "NaN/Inf at index {}", i);
        }
    }
}

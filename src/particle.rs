//! Particle data model and the structure-of-arrays-ish [`Storage`] used by
//! every simulation module.
//!
//! `Storage` owns a contiguous `Vec<Particle>` and a deterministic LCG so
//! tests are reproducible. Particles are addressed by stable `u32` index,
//! which is what the [`crate::quadtree::QuadTree`] references.

use crate::math::Vec2;
use crate::quadtree::Rect;

/// A single simulated agent.
#[derive(Copy, Clone, Debug)]
pub struct Particle {
    pub pos: Vec2,
    pub vel: Vec2,
    pub acc: Vec2,
    pub radius: f32,
}

impl Particle {
    #[inline]
    pub fn new(pos: Vec2, vel: Vec2, radius: f32) -> Self {
        Self {
            pos,
            vel,
            acc: Vec2::zero(),
            radius,
        }
    }
}

/// Owns the particle array plus the world bounds the sim runs in.
pub struct Storage {
    pub parts: Vec<Particle>,
    pub world: Rect,
}

impl Storage {
    /// Creates a new storage with `count` particles uniformly distributed
    /// inside `world`, with random velocities and the given per-particle
    /// radius. `seed` makes the distribution reproducible.
    pub fn randomize(count: usize, world: Rect, radius: f32, seed: u32) -> Self {
        let mut parts = Vec::with_capacity(count);
        let mut rng = Lcg::new(seed);
        for _ in 0..count {
            let px = world.x + rng.next_f32() * world.w;
            let py = world.y + rng.next_f32() * world.h;
            let angle = rng.next_f32() * core::f32::consts::TAU;
            let speed = 20.0 + rng.next_f32() * 40.0;
            let vel = Vec2::new(angle.cos() * speed, angle.sin() * speed);
            parts.push(Particle::new(Vec2::new(px, py), vel, radius));
        }
        Self { parts, world }
    }

    /// Number of particles.
    #[inline]
    pub fn len(&self) -> usize {
        self.parts.len()
    }

    /// True when the storage holds no particles.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.parts.is_empty()
    }

    /// AABB enclosing the particle at index `i` (used as the QuadTree key).
    #[inline]
    pub fn step_aabb(&self, i: usize) -> Rect {
        let p = &self.parts[i];
        Rect::from_center_half(p.pos.x(), p.pos.y(), p.radius, p.radius)
    }

    /// Resets all accumulators to zero. Called at the top of every step.
    #[inline]
    pub fn reset_accumulators(&mut self) {
        for p in self.parts.iter_mut() {
            p.acc = Vec2::zero();
        }
    }

    /// Wraps a particle back into the world if it crossed a boundary.
    pub fn wrap(&mut self, i: usize) {
        let p = &mut self.parts[i];
        let w = self.world;
        if p.pos.x() < w.x {
            p.pos = Vec2::new(p.pos.x() + w.w, p.pos.y());
        } else if p.pos.x() >= w.x + w.w {
            p.pos = Vec2::new(p.pos.x() - w.w, p.pos.y());
        }
        if p.pos.y() < w.y {
            p.pos = Vec2::new(p.pos.x(), p.pos.y() + w.h);
        } else if p.pos.y() >= w.y + w.h {
            p.pos = Vec2::new(p.pos.x(), p.pos.y() - w.h);
        }
    }

    /// Bounces a particle off the world walls (used by elastic collisions).
    pub fn bounce(&mut self, i: usize, restitution: f32) {
        let p = &mut self.parts[i];
        let w = self.world;
        let r = p.radius;
        if p.pos.x() - r < w.x {
            p.pos = Vec2::new(w.x + r, p.pos.y());
            if p.vel.x() < 0.0 {
                p.vel = Vec2::new(-p.vel.x() * restitution, p.vel.y());
            }
        } else if p.pos.x() + r > w.x + w.w {
            p.pos = Vec2::new(w.x + w.w - r, p.pos.y());
            if p.vel.x() > 0.0 {
                p.vel = Vec2::new(-p.vel.x() * restitution, p.vel.y());
            }
        }
        if p.pos.y() - r < w.y {
            p.pos = Vec2::new(p.pos.x(), w.y + r);
            if p.vel.y() < 0.0 {
                p.vel = Vec2::new(p.vel.x(), -p.vel.y() * restitution);
            }
        } else if p.pos.y() + r > w.y + w.h {
            p.pos = Vec2::new(p.pos.x(), w.y + w.h - r);
            if p.vel.y() > 0.0 {
                p.vel = Vec2::new(p.vel.x(), -p.vel.y() * restitution);
            }
        }
    }
}

/// Tiny, deterministic LCG. Avoids pulling in the `rand` crate just to seed
/// initial particle positions.
pub struct Lcg {
    state: u32,
}

impl Lcg {
    pub fn new(seed: u32) -> Self {
        Self { state: seed }
    }

    pub fn next_u32(&mut self) -> u32 {
        self.state = self.state.wrapping_mul(1664525).wrapping_add(1013904223);
        self.state
    }

    pub fn next_f32(&mut self) -> f32 {
        (self.next_u32() as f32) / (u32::MAX as f32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn randomize_count_matches() {
        let s = Storage::randomize(50, Rect::new(0.0, 0.0, 100.0, 100.0), 1.0, 42);
        assert_eq!(s.len(), 50);
    }

    #[test]
    fn randomize_seed_is_deterministic() {
        let a = Storage::randomize(10, Rect::new(0.0, 0.0, 100.0, 100.0), 1.0, 7);
        let b = Storage::randomize(10, Rect::new(0.0, 0.0, 100.0, 100.0), 1.0, 7);
        for (pa, pb) in a.parts.iter().zip(b.parts.iter()) {
            assert_eq!(pa.pos, pb.pos);
            assert_eq!(pa.vel, pb.vel);
        }
    }

    #[test]
    fn randomize_stays_in_bounds() {
        let w = Rect::new(0.0, 0.0, 200.0, 100.0);
        let s = Storage::randomize(500, w, 1.0, 99);
        for p in &s.parts {
            assert!(p.pos.x() >= w.x && p.pos.x() < w.x + w.w);
            assert!(p.pos.y() >= w.y && p.pos.y() < w.y + w.h);
        }
    }

    #[test]
    fn step_aabb_contains_center() {
        let s = Storage::randomize(1, Rect::new(0.0, 0.0, 100.0, 100.0), 5.0, 1);
        let r = s.step_aabb(0);
        let p = s.parts[0].pos;
        assert!(r.contains(p.x(), p.y()));
        assert!(r.w >= 10.0);
    }

    #[test]
    fn no_nan_after_1k_step_in_place() {
        let mut s = Storage::randomize(50, Rect::new(0.0, 0.0, 100.0, 100.0), 1.0, 5);
        // Manually nudge each particle to ensure velocities are not pathological.
        for p in s.parts.iter_mut() {
            p.vel = p.vel.limit(100.0);
        }
        for _ in 0..1000 {
            s.reset_accumulators();
            for i in 0..s.len() {
                let p = s.parts[i];
                s.parts[i].pos = p.pos + p.vel.scale(1.0 / 60.0);
                s.wrap(i);
            }
            for p in s.parts.iter() {
                assert!(p.pos.x().is_finite());
                assert!(p.pos.y().is_finite());
            }
        }
    }

    #[test]
    fn wrap_brings_particle_back() {
        let mut s = Storage::randomize(1, Rect::new(0.0, 0.0, 100.0, 100.0), 1.0, 0);
        s.parts[0].pos = Vec2::new(-5.0, 50.0);
        s.wrap(0);
        let w = s.world;
        assert!(s.parts[0].pos.x() >= w.x && s.parts[0].pos.x() < w.x + w.w);
    }

    #[test]
    fn bounce_flips_velocity_component() {
        let mut s = Storage::randomize(1, Rect::new(0.0, 0.0, 100.0, 100.0), 1.0, 0);
        s.parts[0].pos = Vec2::new(0.0, 50.0);
        s.parts[0].vel = Vec2::new(-50.0, 0.0);
        s.bounce(0, 1.0);
        assert!(s.parts[0].vel.x() > 0.0);
    }
}

//! Equal-mass elastic particle collisions.
//!
//! Each frame:
//!   1. Build a [`QuadTree`] from the current AABBs.
//!   2. For every particle, query overlapping pairs (j > i so each pair is
//!      visited once).
//!   3. Resolve penetration by pushing the two particles apart along their
//!      contact normal, then exchange velocity components along that normal
//!      (equal-mass elastic impulse).
//!   4. Apply a small damping factor so the system doesn't thermalize to a
//!      uniform velocity field too quickly.
//!   5. Bounce off the world walls with `restitution`.

use crate::particle::Storage;
use crate::quadtree::QuadTree;
#[derive(Copy, Clone, Debug)]
pub struct CollisionsParams {
    pub damping: f32,
    pub restitution: f32,
}

impl Default for CollisionsParams {
    fn default() -> Self {
        Self {
            damping: 0.998,
            restitution: 1.0,
        }
    }
}

/// One simulation step. `scratch` is a caller-owned buffer reused across
/// frames so the hot path performs no allocation.
pub fn step(
    storage: &mut Storage,
    qt: &QuadTree,
    params: &CollisionsParams,
    dt: f32,
    scratch: &mut Vec<u32>,
) {
    storage.reset_accumulators();

    for i in 0..storage.len() {
        let me = storage.parts[i];
        let my_aabb = storage.step_aabb(i);
        scratch.clear();
        qt.collect(&my_aabb, scratch);
        for &j in scratch.iter() {
            let j = j as usize;
            if j <= i {
                continue; // visit each pair once
            }
            let other = storage.parts[j];
            let diff = other.pos - me.pos;
            let dist_sq = diff.length_sq();
            let min_dist = me.radius + other.radius;
            if dist_sq >= min_dist * min_dist || dist_sq < 1e-8 {
                continue;
            }
            let dist = dist_sq.sqrt();
            let n = diff.scale(1.0 / dist);
            // Positional correction: push the two apart by the full
            // penetration (50/50 split) so a single step clears overlap.
            let penetration = min_dist - dist;
            let correction = n.scale(0.5 * penetration);
            storage.parts[i].pos = me.pos - correction;
            storage.parts[j].pos = other.pos + correction;

            // Equal-mass elastic impulse along the normal.
            //
            // `n` points from particle i toward particle j. The relative
            // velocity `v1 - v2` projected onto `n` is:
            //   > 0  → particles are approaching (i moving toward j faster
            //           than j is moving away from i).
            //   < 0  → already separating; skip the impulse.
            let v1 = storage.parts[i].vel;
            let v2 = storage.parts[j].vel;
            let rel = v1 - v2;
            let vel_along = rel.dot(n);
            if vel_along <= 0.0 {
                continue;
            }
            let impulse = n.scale(vel_along);
            storage.parts[i].vel = v1 - impulse;
            storage.parts[j].vel = v2 + impulse;
        }
    }

    // Integrate + wall bounce + damping.
    for i in 0..storage.len() {
        let p = storage.parts[i];
        let new_vel = p.vel.scale(params.damping);
        let new_pos = p.pos + new_vel * dt;
        storage.parts[i].vel = new_vel;
        storage.parts[i].pos = new_pos;
        if new_pos.x().is_nan() || new_pos.y().is_nan() {
            storage.parts[i].pos = p.pos;
        }
        storage.bounce(i, params.restitution);
    }
}

/// Build a fresh QuadTree from a storage. Convenience for tests and the
/// WASM surface.
pub fn build_qt(storage: &Storage) -> QuadTree {
    let mut qt = QuadTree::new(storage.world);
    for i in 0..storage.len() {
        qt.insert(crate::quadtree::Entry {
            idx: i as u32,
            aabb: storage.step_aabb(i),
        });
    }
    qt
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec2;
    use crate::particle::Particle;
    use crate::quadtree::Rect;

    fn two_particle_storage() -> Storage {
        Storage {
            parts: vec![
                Particle::new(Vec2::new(50.0, 50.0), Vec2::new(100.0, 0.0), 4.0),
                Particle::new(Vec2::new(57.0, 50.0), Vec2::new(-100.0, 0.0), 4.0),
            ],
            world: Rect::new(0.0, 0.0, 1000.0, 1000.0),
        }
    }

    #[test]
    fn head_on_equal_mass_exchanges_velocities() {
        let mut s = two_particle_storage();
        let qt = build_qt(&s);
        let params = CollisionsParams::default();
        step(&mut s, &qt, &params, 1.0 / 60.0, &mut Vec::new());
        // After a head-on equal-mass elastic collision, the velocities swap.
        let v0x = s.parts[0].vel.x();
        let v1x = s.parts[1].vel.x();
        // Damping factor 0.998 reduces magnitudes by ~0.2% per step.
        assert!(v0x < 0.0, "particle 0 should now move left, got {}", v0x);
        assert!(v1x > 0.0, "particle 1 should now move right, got {}", v1x);
        // Magnitudes approximately equal (damping affects both equally).
        assert!(
            (v0x.abs() - v1x.abs()).abs() < 0.5,
            "|v0|={} |v1|={}",
            v0x.abs(),
            v1x.abs()
        );
    }

    #[test]
    fn no_overlap_remains_after_step() {
        let mut s = two_particle_storage();
        // Push them even closer.
        s.parts[0].pos = Vec2::new(50.0, 50.0);
        s.parts[1].pos = Vec2::new(53.0, 50.0);
        s.parts[0].vel = Vec2::new(0.0, 0.0);
        s.parts[1].vel = Vec2::new(0.0, 0.0);
        let qt = build_qt(&s);
        let params = CollisionsParams::default();
        step(&mut s, &qt, &params, 1.0 / 60.0, &mut Vec::new());
        // The full positional correction is 50/50 each, so a single step
        // should already clear the overlap. Allow a tiny epsilon for floats.
        const EPS: f32 = 0.01;
        for pair in s.parts.windows(2) {
            let d = (pair[1].pos - pair[0].pos).length();
            let min = pair[0].radius + pair[1].radius - EPS;
            assert!(d >= min, "particles still overlap: d={}, min={}", d, min);
        }
    }

    #[test]
    fn two_particle_system_conserves_momentum() {
        let mut s = two_particle_storage();
        // Use a very large world so neither particle can hit a wall during
        // the test. This isolates the conservation check to the elastic
        // collision impulse alone.
        s.world = Rect::new(-1_000_000.0, -1_000_000.0, 2_000_000.0, 2_000_000.0);
        s.parts[0].vel = Vec2::new(120.0, 30.0);
        s.parts[1].vel = Vec2::new(-80.0, -20.0);
        let initial_px = s.parts[0].vel.x() + s.parts[1].vel.x();
        let initial_py = s.parts[0].vel.y() + s.parts[1].vel.y();
        assert!(initial_px.abs() > 1.0);
        let params = CollisionsParams {
            damping: 1.0,
            restitution: 1.0,
        };
        for _ in 0..1000 {
            let qt = build_qt(&s);
            step(&mut s, &qt, &params, 1.0 / 60.0, &mut Vec::new());
        }
        let final_px = s.parts[0].vel.x() + s.parts[1].vel.x();
        let final_py = s.parts[0].vel.y() + s.parts[1].vel.y();
        assert!(
            (initial_px - final_px).abs() < 1.0,
            "x-momentum drift: {} -> {}",
            initial_px,
            final_px
        );
        assert!(
            (initial_py - final_py).abs() < 1.0,
            "y-momentum drift: {} -> {}",
            initial_py,
            final_py
        );
    }

    #[test]
    fn no_nan_over_1000_steps_with_200_particles() {
        let mut s = Storage::randomize(200, Rect::new(0.0, 0.0, 500.0, 500.0), 2.0, 11);
        let params = CollisionsParams::default();
        let mut scratch = Vec::new();
        for _ in 0..1000 {
            let qt = build_qt(&s);
            step(&mut s, &qt, &params, 1.0 / 60.0, &mut scratch);
            for p in s.parts.iter() {
                assert!(p.pos.x().is_finite());
                assert!(p.pos.y().is_finite());
                assert!(p.vel.x().is_finite());
                assert!(p.vel.y().is_finite());
            }
        }
    }
}

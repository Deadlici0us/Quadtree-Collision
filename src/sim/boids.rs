//! Boids flocking simulation.
//!
//! Each particle queries the [`QuadTree`] within `perception_radius` and
//! computes three steering contributions:
//!   - **Separation** — push away from neighbours closer than
//!     `separation_radius` (inverse-distance repulsion).
//!   - **Alignment** — steer toward the average velocity of neighbours.
//!   - **Cohesion** — steer toward the average position of neighbours.
//!
//! All three are weighted, summed, and clamped to `max_force`. The world
//! wraps toroidally so agents stay visible at all times; the per-boid
//! perception query is tiled across the 3x3 wrap grid so a boid near an
//! edge still "sees" the neighbours that physically wrap to the other
//! side. Without this, edge boids lose alignment and cohesion because
//! the tree only stores positions in the central tile, and they would
//! drift along the wall.

use crate::math::Vec2;
use crate::particle::Storage;
use crate::quadtree::{QuadTree, Rect};

#[derive(Copy, Clone, Debug)]
pub struct BoidsParams {
    pub perception: f32,
    pub separation_radius: f32,
    pub w_sep: f32,
    pub w_ali: f32,
    pub w_coh: f32,
    pub max_speed: f32,
    pub max_force: f32,
}

impl Default for BoidsParams {
    fn default() -> Self {
        Self {
            perception: 35.0,
            separation_radius: 5.0,
            w_sep: 1.8,
            w_ali: 1.0,
            w_coh: 0.8,
            max_speed: 70.0,
            max_force: 180.0,
        }
    }
}

/// One simulation step. `scratch` is a caller-owned buffer reused across
/// frames so the hot path performs no allocation.
pub fn step(
    storage: &mut Storage,
    qt: &QuadTree,
    params: &BoidsParams,
    dt: f32,
    scratch: &mut Vec<u32>,
) {
    storage.reset_accumulators();

    let world = storage.world;
    let perception_sq = params.perception * params.perception;
    let separation_sq = params.separation_radius * params.separation_radius;

    for i in 0..storage.len() {
        let me = storage.parts[i];
        // Build the 3x3 wrap-shift offsets for the perception AABB. The
        // central tile is always queried; the eight neighbours are
        // included only when the perception AABB crosses the world edge.
        //
        // Wrap trick: if the AABB extends past the *left* edge (base.x <
        // 0), the boid's local perception includes positions in
        // [base.x, 0). Those positions in the *world* wrap from
        // [world.w + base.x, world.w). To find the relevant candidates
        // in the QuadTree, we query the AABB shifted by +world.w (the
        // "right tile") and then unwrap each candidate by subtracting
        // world.w from its world position so the diff vector is in the
        // boid's local frame.
        let base_rect =
            Rect::from_center_half(me.pos.x(), me.pos.y(), params.perception, params.perception);
        let ox_pos = if base_rect.x < world.x { world.w } else { 0.0 };
        let ox_neg = if base_rect.x + base_rect.w > world.x + world.w {
            -world.w
        } else {
            0.0
        };
        let oy_pos = if base_rect.y < world.y { world.h } else { 0.0 };
        let oy_neg = if base_rect.y + base_rect.h > world.y + world.h {
            -world.h
        } else {
            0.0
        };

        let mut sep = Vec2::zero();
        let mut ali = Vec2::zero();
        let mut coh = Vec2::zero();
        let mut sep_count = 0u32;
        let mut ali_count = 0u32;
        let mut coh_count = 0u32;

        // Iterate the 3x3 tile grid. In the interior this is a single
        // tile; near edges we query up to 4 (corner), 6 (edge), or 9 tiles.
        // We use the `ox_neg, 0, ox_pos` pattern to enumerate the three
        // x-offsets and the analogous three y-offsets. Each query shifts
        // the *tree-side* AABB by `dx, dy`; the candidate's local-frame
        // position is unwrapped by the *opposite* shift (`-dx, -dy`).
        let x_offsets = [ox_neg, 0.0, ox_pos];
        let y_offsets = [oy_neg, 0.0, oy_pos];
        for &dy in y_offsets.iter() {
            for &dx in x_offsets.iter() {
                scratch.clear();
                qt.collect(
                    &Rect::new(base_rect.x + dx, base_rect.y + dy, base_rect.w, base_rect.h),
                    scratch,
                );
                for &j in scratch.iter() {
                    if j as usize == i {
                        continue;
                    }
                    let other = storage.parts[j as usize];
                    // Unwrap the candidate into the boid's local frame.
                    // If we queried the right tile (dx = +world.w), the
                    // candidate's local x is `other.x - world.w`; the
                    // central tile (dx = 0) leaves it as-is.
                    let other_pos = Vec2::new(other.pos.x() - dx, other.pos.y() - dy);
                    let diff = me.pos - other_pos;
                    let d_sq = diff.length_sq();
                    if d_sq > perception_sq {
                        continue;
                    }
                    ali += other.vel;
                    ali_count += 1;
                    coh += other_pos;
                    coh_count += 1;
                    if d_sq < separation_sq && d_sq > 0.0 {
                        let inv = 1.0 / d_sq.sqrt();
                        sep += diff.scale(inv);
                        sep_count += 1;
                    }
                }
            }
        }

        let mut acc = Vec2::zero();
        if sep_count > 0 {
            let avg = sep.scale(1.0 / sep_count as f32);
            let steer = (avg.normalize_or_zero() * params.max_speed) - me.vel;
            acc += steer.limit(params.max_force) * params.w_sep;
        }
        if ali_count > 0 {
            let avg = ali.scale(1.0 / ali_count as f32);
            let steer = (avg.normalize_or_zero() * params.max_speed) - me.vel;
            acc += steer.limit(params.max_force) * params.w_ali;
        }
        if coh_count > 0 {
            let target = coh.scale(1.0 / coh_count as f32);
            let desired = target - me.pos;
            let steer = (desired.normalize_or_zero() * params.max_speed) - me.vel;
            acc += steer.limit(params.max_force) * params.w_coh;
        }
        storage.parts[i].acc = acc;
    }

    // Integrate.
    for i in 0..storage.len() {
        let p = storage.parts[i];
        let new_vel = (p.vel + p.acc * dt).limit(params.max_speed);
        let new_pos = p.pos + new_vel * dt;
        storage.parts[i].vel = new_vel;
        storage.parts[i].pos = new_pos;
        storage.wrap(i);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quadtree::Rect;

    fn make_storage_with_centers(centers: &[(f32, f32)], radius: f32) -> Storage {
        let s = Storage {
            parts: centers
                .iter()
                .map(|&(x, y)| {
                    crate::particle::Particle::new(Vec2::new(x, y), Vec2::new(0.0, 0.0), radius)
                })
                .collect(),
            world: Rect::new(0.0, 0.0, 1000.0, 1000.0),
        };
        s
    }

    fn make_qt(storage: &Storage) -> QuadTree {
        let mut qt = QuadTree::new(storage.world);
        for i in 0..storage.len() {
            qt.insert(crate::quadtree::Entry {
                idx: i as u32,
                aabb: storage.step_aabb(i),
            });
        }
        qt
    }

    #[test]
    #[allow(clippy::field_reassign_with_default)]
    fn zero_perception_yields_no_acceleration() {
        let mut s = make_storage_with_centers(&[(50.0, 50.0), (52.0, 50.0)], 1.0);
        let qt = make_qt(&s);
        let mut p = BoidsParams::default();
        p.perception = 0.0;
        step(&mut s, &qt, &p, 1.0 / 60.0, &mut Vec::new());
        for particle in s.parts.iter() {
            assert_eq!(particle.acc, Vec2::zero());
        }
    }

    #[test]
    #[allow(clippy::field_reassign_with_default)]
    fn separation_pushes_close_neighbours_apart() {
        // Two particles, very close, separation radius encloses them.
        let mut s = make_storage_with_centers(&[(50.0, 50.0), (52.0, 50.0)], 1.0);
        let qt = make_qt(&s);
        let mut p = BoidsParams::default();
        p.perception = 20.0;
        p.separation_radius = 10.0;
        p.w_sep = 1.0;
        p.w_ali = 0.0;
        p.w_coh = 0.0;
        step(&mut s, &qt, &p, 1.0 / 60.0, &mut Vec::new());
        // Particle 0 should have negative-x acceleration (push left).
        assert!(s.parts[0].acc.x() < 0.0, "got {:?}", s.parts[0].acc);
        // Particle 1 should have positive-x acceleration.
        assert!(s.parts[1].acc.x() > 0.0, "got {:?}", s.parts[1].acc);
    }

    #[test]
    #[allow(clippy::field_reassign_with_default)]
    fn cohesion_pulls_lone_particle_toward_neighbours() {
        // One particle alone at origin, another at (100, 0). With high cohesion
        // weight the lone one should accelerate in +x.
        let mut s = make_storage_with_centers(&[(50.0, 50.0), (150.0, 50.0)], 1.0);
        let qt = make_qt(&s);
        let mut p = BoidsParams::default();
        p.perception = 200.0;
        p.w_sep = 0.0;
        p.w_ali = 0.0;
        p.w_coh = 1.0;
        step(&mut s, &qt, &p, 1.0 / 60.0, &mut Vec::new());
        assert!(s.parts[0].acc.x() > 0.0);
    }

    #[test]
    #[allow(clippy::field_reassign_with_default)]
    fn alignment_torques_others_toward_avg_velocity() {
        // Particle 0 stationary, particle 1 moving in +x within perception.
        let mut s = make_storage_with_centers(&[(50.0, 50.0), (55.0, 50.0)], 1.0);
        s.parts[1].vel = Vec2::new(10.0, 0.0);
        let qt = make_qt(&s);
        let mut p = BoidsParams::default();
        p.perception = 50.0;
        p.w_sep = 0.0;
        p.w_ali = 1.0;
        p.w_coh = 0.0;
        step(&mut s, &qt, &p, 1.0 / 60.0, &mut Vec::new());
        assert!(s.parts[0].acc.x() > 0.0);
    }

    #[test]
    #[allow(clippy::field_reassign_with_default)]
    fn boid_at_edge_sees_wrapped_neighbour() {
        // Two boids in a 100x100 world. Boid 0 is at the top-left corner
        // (5, 5); boid 1 is at the bottom-right corner (95, 95). Without
        // the 3x3 wrap-shift, the distance is ~127 and the perception
        // query (35 px) misses boid 1. With wrap-shift, the toroidal
        // distance is ~9.8 (boid 1 wraps to (95-100, 95-100) = (-5, -5))
        // and boid 0 should see it.
        let mut s = make_storage_with_centers(&[(5.0, 5.0), (95.0, 95.0)], 1.0);
        s.world = Rect::new(0.0, 0.0, 100.0, 100.0);
        let qt = make_qt(&s);
        let mut p = BoidsParams::default();
        p.perception = 35.0;
        p.w_sep = 0.0;
        p.w_ali = 0.0;
        p.w_coh = 1.0;
        step(&mut s, &qt, &p, 1.0 / 60.0, &mut Vec::new());
        // Without the wrap fix this would be Vec2::zero(). With the fix,
        // boid 0 sees the wrapped neighbour at (-5, -5) and its average
        // position is (-5, -5) — target minus me.pos is (-10, -10),
        // normalised * max_speed - me.vel. Since me.vel is zero, the
        // desired direction is (-1, -1) and acceleration is non-zero
        // along the negative diagonal.
        let a = s.parts[0].acc;
        assert!(
            a.x() < -0.0 && a.y() < -0.0,
            "expected non-zero acceleration toward the wrapped neighbour, got {:?}",
            a
        );
    }

    #[test]
    fn no_nan_over_1000_steps_with_100_boids() {
        let mut s = Storage::randomize(100, Rect::new(0.0, 0.0, 1000.0, 1000.0), 2.0, 1);
        let params = BoidsParams::default();
        let mut scratch = Vec::new();
        for _ in 0..1000 {
            let mut qt = QuadTree::new(s.world);
            for i in 0..s.len() {
                qt.insert(crate::quadtree::Entry {
                    idx: i as u32,
                    aabb: s.step_aabb(i),
                });
            }
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

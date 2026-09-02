//! Dynamic axis-aligned bounding-box [`Rect`] and a per-frame-rebuilt
//! [`QuadTree`] spatial index.
//!
//! The tree is intentionally kept simple: every frame we call
//! [`QuadTree::clear`] and re-insert all live particles. This is faster than
//! incremental updates for fields where most particles move every tick
//! (boids, elastic collisions) because the amortized cost of full rebuilds
//! is lower than tracking per-entry moves across a deep tree.
//!
//! # Capacity tuning
//!
//! - `capacity` (default 8): how many entries a node holds before subdividing.
//! - `max_depth` (default 8): hard cap on subdivision depth so degenerate
//!   clusters (e.g. 1000 particles stacked at the same point) cannot blow
//!   the recursion stack.

/// Axis-aligned bounding box. Half-open on the max edges
/// (`x <= px < x + w` and `y <= py < y + h`) so that adjacent quadrant
/// boundaries don't double-count shared edges.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Rect {
    /// Minimum x.
    pub x: f32,
    /// Minimum y.
    pub y: f32,
    /// Width.
    pub w: f32,
    /// Height.
    pub h: f32,
}

impl Rect {
    /// Constructs a rect from `(x, y, w, h)`.
    #[inline]
    pub const fn new(x: f32, y: f32, w: f32, h: f32) -> Self {
        Self { x, y, w, h }
    }

    /// Constructs a rect enclosing a point with the given half-extents.
    #[inline]
    pub fn from_center_half(cx: f32, cy: f32, hw: f32, hh: f32) -> Self {
        Self {
            x: cx - hw,
            y: cy - hh,
            w: hw * 2.0,
            h: hh * 2.0,
        }
    }

    /// Tests whether a point lies inside (or on the min edge of) the rect.
    #[inline]
    pub fn contains(&self, px: f32, py: f32) -> bool {
        px >= self.x && px < self.x + self.w && py >= self.y && py < self.y + self.h
    }

    /// Tests whether two rects overlap. Touching edges count as not-overlapping
    /// (same half-open convention as [`contains`]).
    #[inline]
    pub fn intersects(&self, other: &Rect) -> bool {
        self.x < other.x + other.w
            && self.x + self.w > other.x
            && self.y < other.y + other.h
            && self.y + self.h > other.y
    }

    /// Returns the four quadrant rects `[NW, NE, SW, SE]` that exactly
    /// partition this rect. Quadrants share the centre line; ordering matches
    /// the order [`QuadTree`] uses for child storage.
    pub fn quadrants(&self) -> [Rect; 4] {
        let hw = self.w * 0.5;
        let hh = self.h * 0.5;
        let cx = self.x + hw;
        let cy = self.y + hh;
        [
            Rect::new(self.x, self.y, hw, hh), // NW
            Rect::new(cx, self.y, hw, hh),     // NE
            Rect::new(self.x, cy, hw, hh),     // SW
            Rect::new(cx, cy, hw, hh),         // SE
        ]
    }
}

/// An entry stored in the tree: a stable particle index plus its AABB.
#[derive(Copy, Clone, Debug)]
pub struct Entry {
    pub idx: u32,
    pub aabb: Rect,
}

const DEFAULT_CAPACITY: usize = 8;
const DEFAULT_MAX_DEPTH: usize = 8;

/// Dynamic quadtree. Holds particle indices keyed by their world-space AABB.
pub struct QuadTree {
    bounds: Rect,
    capacity: usize,
    max_depth: usize,
    depth: usize,
    entries: Vec<Entry>,
    children: Option<Box<[QuadTree; 4]>>,
}

impl QuadTree {
    /// Creates a new tree over the given world bounds. The default capacity
    /// (8) and max depth (8) are tuned for 10–20k particles in a 1080p world.
    pub fn new(bounds: Rect) -> Self {
        Self::with_params(bounds, DEFAULT_CAPACITY, DEFAULT_MAX_DEPTH)
    }

    /// Creates a tree with custom capacity and max depth. Useful for tests
    /// and the public `set_param` knob.
    pub fn with_params(bounds: Rect, capacity: usize, max_depth: usize) -> Self {
        Self {
            bounds,
            capacity: capacity.max(1),
            max_depth,
            depth: 0,
            entries: Vec::with_capacity(capacity),
            children: None,
        }
    }

    /// World bounds the tree covers.
    #[inline]
    pub fn bounds(&self) -> Rect {
        self.bounds
    }

    /// Number of entries currently held directly at this node (not children).
    #[inline]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// `true` if this node holds no entries and has no children.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty() && self.children.is_none()
    }

    /// Empties the tree, keeping allocated memory and child boxes around so
    /// subsequent frames can reuse them.
    pub fn clear(&mut self) {
        self.entries.clear();
        if let Some(children) = self.children.as_mut() {
            for child in children.iter_mut() {
                child.clear();
            }
        }
    }

    /// Inserts an entry. Returns `true` if the entry's AABB intersected the
    /// tree bounds (and was therefore stored). Out-of-bounds entries are
    /// rejected so the tree never holds invisible data.
    pub fn insert(&mut self, entry: Entry) -> bool {
        self.insert_entry(entry)
    }

    fn insert_entry(&mut self, entry: Entry) -> bool {
        if !self.bounds.intersects(&entry.aabb) {
            return false;
        }

        // If we have children, decide which child(ren) the entry belongs in:
        //   - 1 quadrant:  store only in that child (clean partition).
        //   - 2+ quadrants: store only at this node (straddles; pushing to
        //     multiple children would duplicate the entry and corrupt
        //     counts / queries).
        if let Some(children) = self.children.as_mut() {
            let quads = self.bounds.quadrants();
            let mut hit_idx: Option<usize> = None;
            let mut hit_count = 0u32;
            for (i, _child) in children.iter_mut().enumerate() {
                if quads[i].intersects(&entry.aabb) {
                    hit_count += 1;
                    hit_idx = Some(i);
                }
            }
            if hit_count == 1 {
                let idx = hit_idx.unwrap();
                return children[idx].insert_entry(entry);
            } else {
                self.entries.push(entry);
                return true;
            }
        }

        // Leaf path.
        if self.entries.len() < self.capacity || self.depth >= self.max_depth {
            self.entries.push(entry);
        } else {
            self.subdivide();
            // Redistribute the existing entries into the new children, then
            // insert the new entry (which goes through `insert_entry` again
            // and will either land in a child or stay at this node if it
            // straddles a boundary).
            let existing = core::mem::take(&mut self.entries);
            for e in existing {
                self.insert_entry(e);
            }
            self.insert_entry(entry);
        }
        true
    }

    fn subdivide(&mut self) {
        let quads = self.bounds.quadrants();
        let next_depth = self.depth + 1;
        let mut kids: [QuadTree; 4] = [
            QuadTree::with_params(quads[0], self.capacity, self.max_depth),
            QuadTree::with_params(quads[1], self.capacity, self.max_depth),
            QuadTree::with_params(quads[2], self.capacity, self.max_depth),
            QuadTree::with_params(quads[3], self.capacity, self.max_depth),
        ];
        for child in kids.iter_mut() {
            child.depth = next_depth;
        }
        self.children = Some(Box::new(kids));
    }

    /// Visits every entry whose AABB intersects `range`. `visit` is called
    /// once per matching index. Pass a `&mut Vec<u32>` as `scratch` to reuse
    /// allocation across frames.
    pub fn query_range<F>(&self, range: &Rect, scratch: &mut Vec<u32>, mut visit: F)
    where
        F: FnMut(u32),
    {
        scratch.clear();
        self.collect(range, scratch);
        for &idx in scratch.iter() {
            visit(idx);
        }
    }

    /// Same as [`query_range`] but writes matching indices directly into
    /// `out` without a scratch buffer. Used by the hot path in the sim
    /// modules where the scratch clear/fill is a non-trivial cost.
    pub fn collect(&self, range: &Rect, out: &mut Vec<u32>) {
        out.clear();
        self.collect_into(range, out);
    }

    fn collect_into(&self, range: &Rect, out: &mut Vec<u32>) {
        if !self.bounds.intersects(range) {
            return;
        }
        for entry in &self.entries {
            if entry.aabb.intersects(range) {
                out.push(entry.idx);
            }
        }
        if let Some(children) = self.children.as_ref() {
            for child in children.iter() {
                child.collect_into(range, out);
            }
        }
    }

    /// Total number of entries across this node and all descendants.
    pub fn total_entries(&self) -> usize {
        let own = self.entries.len();
        match &self.children {
            None => own,
            Some(c) => own + c.iter().map(|ch| ch.total_entries()).sum::<usize>(),
        }
    }

    /// For debugging: collects the bounds of every internal node and leaf.
    /// Used by the WASM "show quadtree" overlay.
    pub fn collect_node_bounds(&self, out: &mut Vec<f32>) {
        out.push(self.bounds.x);
        out.push(self.bounds.y);
        out.push(self.bounds.w);
        out.push(self.bounds.h);
        out.push(self.depth as f32);
        if let Some(children) = self.children.as_ref() {
            for child in children.iter() {
                child.collect_node_bounds(out);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    // ---- Rect tests ----

    #[test]
    fn rect_contains_inside() {
        let r = Rect::new(0.0, 0.0, 10.0, 10.0);
        assert!(r.contains(5.0, 5.0));
        assert!(r.contains(0.0, 0.0));
        assert!(!r.contains(10.0, 5.0), "max edge is exclusive");
        assert!(!r.contains(15.0, 5.0));
    }

    #[test]
    fn rect_intersects_overlap() {
        let a = Rect::new(0.0, 0.0, 10.0, 10.0);
        let b = Rect::new(5.0, 5.0, 10.0, 10.0);
        assert!(a.intersects(&b));
        assert!(b.intersects(&a));
    }

    #[test]
    fn rect_intersects_touching_edges_do_not_count() {
        let a = Rect::new(0.0, 0.0, 10.0, 10.0);
        let b = Rect::new(10.0, 0.0, 10.0, 10.0);
        assert!(!a.intersects(&b));
    }

    #[test]
    fn rect_intersects_disjoint() {
        let a = Rect::new(0.0, 0.0, 10.0, 10.0);
        let b = Rect::new(20.0, 20.0, 10.0, 10.0);
        assert!(!a.intersects(&b));
    }

    #[test]
    fn quadrants_partition_exactly() {
        let r = Rect::new(0.0, 0.0, 100.0, 100.0);
        let quads = r.quadrants();
        // Every quadrant should have half width/height.
        for q in &quads {
            assert!(approx_eq(q.w, 50.0));
            assert!(approx_eq(q.h, 50.0));
        }
        // Pick 9 sample points; each must lie in exactly one quadrant.
        let points = [
            (1.0, 1.0),
            (50.0, 1.0),
            (99.0, 1.0),
            (1.0, 50.0),
            (50.0, 50.0),
            (99.0, 50.0),
            (1.0, 99.0),
            (50.0, 99.0),
            (99.0, 99.0),
        ];
        for p in points {
            let count = quads.iter().filter(|q| q.contains(p.0, p.1)).count();
            assert_eq!(count, 1, "point {:?} hit {} quadrants", p, count);
        }
    }

    #[test]
    fn from_center_half() {
        let r = Rect::from_center_half(10.0, 20.0, 5.0, 3.0);
        assert!(approx_eq(r.x, 5.0));
        assert!(approx_eq(r.y, 17.0));
        assert!(approx_eq(r.w, 10.0));
        assert!(approx_eq(r.h, 6.0));
    }

    // ---- QuadTree tests ----

    fn collect_all(qt: &QuadTree, range: Rect) -> Vec<u32> {
        let mut out = Vec::new();
        let mut scratch = Vec::new();
        qt.query_range(&range, &mut scratch, |i| out.push(i));
        out
    }

    #[test]
    fn insert_rejects_out_of_bounds() {
        let mut qt = QuadTree::new(Rect::new(0.0, 0.0, 100.0, 100.0));
        let ok = qt.insert(Entry {
            idx: 1,
            aabb: Rect::from_center_half(200.0, 200.0, 1.0, 1.0),
        });
        assert!(!ok);
        assert_eq!(qt.total_entries(), 0);
    }

    #[test]
    fn insert_stores_in_bounds() {
        let mut qt = QuadTree::new(Rect::new(0.0, 0.0, 100.0, 100.0));
        assert!(qt.insert(Entry {
            idx: 42,
            aabb: Rect::from_center_half(10.0, 10.0, 1.0, 1.0),
        }));
        assert_eq!(qt.total_entries(), 1);
    }

    #[test]
    fn subdivision_caps_at_max_depth() {
        let mut qt = QuadTree::with_params(Rect::new(0.0, 0.0, 100.0, 100.0), 2, 2);
        for i in 0..50u32 {
            qt.insert(Entry {
                idx: i,
                aabb: Rect::from_center_half((i as f32) * 0.1, 0.0, 0.1, 0.1),
            });
        }
        assert_eq!(qt.total_entries(), 50);
    }

    #[test]
    fn query_finds_overlapping_entries() {
        let mut qt = QuadTree::new(Rect::new(0.0, 0.0, 100.0, 100.0));
        for i in 0..5u32 {
            qt.insert(Entry {
                idx: i,
                aabb: Rect::from_center_half(10.0 + i as f32, 10.0, 1.0, 1.0),
            });
        }
        // Range (8, 8, 10, 5) covers x = [8, 18), y = [8, 13). All 5 AABBs are
        // inside (their x range is [9.5, 14.5], all < 18).
        let hits = collect_all(&qt, Rect::new(8.0, 8.0, 10.0, 5.0));
        assert_eq!(hits.len(), 5);
    }

    #[test]
    fn query_excludes_distant_entries() {
        let mut qt = QuadTree::new(Rect::new(0.0, 0.0, 100.0, 100.0));
        qt.insert(Entry {
            idx: 1,
            aabb: Rect::from_center_half(10.0, 10.0, 1.0, 1.0),
        });
        qt.insert(Entry {
            idx: 2,
            aabb: Rect::from_center_half(90.0, 90.0, 1.0, 1.0),
        });
        let hits = collect_all(&qt, Rect::new(0.0, 0.0, 20.0, 20.0));
        assert_eq!(hits, vec![1]);
    }

    #[test]
    fn query_covering_whole_world_returns_all() {
        let mut qt = QuadTree::new(Rect::new(0.0, 0.0, 100.0, 100.0));
        for i in 0..20u32 {
            qt.insert(Entry {
                idx: i,
                aabb: Rect::from_center_half((i as f32) * 4.0, 50.0, 1.0, 1.0),
            });
        }
        // Use a query slightly larger than world so every AABB intersects.
        let hits = collect_all(&qt, Rect::new(-1.0, -1.0, 102.0, 102.0));
        assert_eq!(hits.len(), 20);
    }

    #[test]
    fn clear_empties_tree_but_keeps_capacity() {
        let mut qt = QuadTree::new(Rect::new(0.0, 0.0, 100.0, 100.0));
        for i in 0..100u32 {
            qt.insert(Entry {
                idx: i,
                aabb: Rect::from_center_half((i as f32) % 100.0, 0.0, 0.5, 0.5),
            });
        }
        assert!(qt.total_entries() > 0);
        qt.clear();
        assert_eq!(qt.total_entries(), 0);
        // Re-insert still works.
        assert!(qt.insert(Entry {
            idx: 999,
            aabb: Rect::from_center_half(50.0, 50.0, 1.0, 1.0),
        }));
        assert_eq!(qt.total_entries(), 1);
    }

    #[test]
    fn collect_node_bounds_visits_every_internal_node() {
        let mut qt = QuadTree::new(Rect::new(0.0, 0.0, 100.0, 100.0));
        for i in 0..50u32 {
            qt.insert(Entry {
                idx: i,
                aabb: Rect::from_center_half((i as f32) * 1.5, 50.0, 0.5, 0.5),
            });
        }
        let mut out = Vec::new();
        qt.collect_node_bounds(&mut out);
        assert_eq!(out.len() % 5, 0);
        // First node is the root at depth 0.
        assert!(approx_eq(out[4], 0.0));
    }

    #[test]
    fn stress_neighbour_query_matches_brute_force() {
        use crate::math::Vec2;
        // Deterministic LCG for reproducibility.
        let mut state: u32 = 0x1234_5678;
        let mut next = || -> f32 {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            (state as f32) / (u32::MAX as f32)
        };
        let n = 1000;
        let mut pts: Vec<Vec2> = Vec::with_capacity(n);
        for _ in 0..n {
            pts.push(Vec2::new(next() * 1000.0, next() * 1000.0));
        }
        let mut qt = QuadTree::new(Rect::new(0.0, 0.0, 1000.0, 1000.0));
        for (i, p) in pts.iter().enumerate() {
            qt.insert(Entry {
                idx: i as u32,
                aabb: Rect::from_center_half(p.x(), p.y(), 0.5, 0.5),
            });
        }
        assert_eq!(qt.total_entries(), n);

        // For a few sample points, dump QT results to inspect.
        let radius = 10.0;
        let mut scratch = Vec::new();
        for &idx in &[0u32, 100, 500, 999] {
            let p = pts[idx as usize];
            scratch.clear();
            qt.collect(
                &Rect::from_center_half(p.x(), p.y(), radius, radius),
                &mut scratch,
            );
            // BF neighbours of p.
            let mut bf = Vec::new();
            for (j, q) in pts.iter().enumerate() {
                if j as u32 == idx {
                    continue;
                }
                let dx = p.x() - q.x();
                let dy = p.y() - q.y();
                if dx * dx + dy * dy <= radius * radius {
                    bf.push(j as u32);
                }
            }
            // Compare (both should be the same set).
            let mut qts: Vec<u32> = scratch.iter().copied().filter(|&i| i != idx).collect();
            qts.sort();
            let mut bfs = bf.clone();
            bfs.sort();
            assert_eq!(
                qts, bfs,
                "mismatch for point {}: QT={:?}, BF={:?}",
                idx, qts, bfs
            );
        }
    }

    #[test]
    fn stress_50k_random_points_matches_brute_force() {
        use crate::math::Vec2;
        // Deterministic LCG for reproducibility.
        let mut state: u32 = 0x1234_5678;
        let mut next = || -> f32 {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            (state as f32) / (u32::MAX as f32)
        };
        let mut pts: Vec<Vec2> = Vec::with_capacity(50_000);
        for _ in 0..50_000 {
            pts.push(Vec2::new(next() * 1000.0, next() * 1000.0));
        }
        let mut qt = QuadTree::new(Rect::new(0.0, 0.0, 1000.0, 1000.0));
        for (i, p) in pts.iter().enumerate() {
            qt.insert(Entry {
                idx: i as u32,
                aabb: Rect::from_center_half(p.x(), p.y(), 0.5, 0.5),
            });
        }
        assert_eq!(qt.total_entries(), 50_000);

        // Ground truth: count points whose AABB intersects a 20x20 box
        // around each point (same semantic as the QT query). Exclude self.
        let half = 10.0;
        let mut qt_total = 0u64;
        let mut bf_total = 0u64;
        let mut scratch = Vec::new();
        for (i, p) in pts.iter().enumerate() {
            let query = Rect::from_center_half(p.x(), p.y(), half, half);
            let mut qt_seen = std::collections::HashSet::new();
            qt.collect(&query, &mut scratch);
            for &idx in &scratch {
                qt_seen.insert(idx);
            }
            qt_seen.remove(&(i as u32));
            qt_total += qt_seen.len() as u64;

            for (j, q) in pts.iter().enumerate() {
                if i == j {
                    continue;
                }
                let aabb = Rect::from_center_half(q.x(), q.y(), 0.5, 0.5);
                if aabb.intersects(&query) {
                    bf_total += 1;
                }
            }
        }
        assert_eq!(
            qt_total, bf_total,
            "QT AABB-intersection counts must match brute force (QT={}, BF={})",
            qt_total, bf_total
        );
    }
}

//! Benchmarks for the QuadTree vs brute-force neighbour queries.
//!
//! Run with `cargo bench`; full results live in `docs/benchmarks.md`.

use criterion::{criterion_group, criterion_main, Criterion};
use quadtree_collision::particle::Storage;
use quadtree_collision::quadtree::{Entry, QuadTree, Rect};
use quadtree_collision::sim::collisions;

fn make_storage(n: usize, world: Rect, seed: u32) -> Storage {
    Storage::randomize(n, world, 2.0, seed)
}

fn build_qt(storage: &Storage) -> QuadTree {
    let mut qt = QuadTree::new(storage.world);
    for i in 0..storage.len() {
        qt.insert(Entry {
            idx: i as u32,
            aabb: storage.step_aabb(i),
        });
    }
    qt
}

fn qt_neighbour_count(storage: &Storage, qt: &QuadTree, radius: f32) -> u64 {
    let mut total = 0u64;
    let mut scratch = Vec::new();
    for i in 0..storage.len() {
        let p = storage.parts[i].pos;
        scratch.clear();
        qt.collect(
            &Rect::from_center_half(p.x(), p.y(), radius, radius),
            &mut scratch,
        );
        total += scratch.iter().filter(|&&j| j as usize != i).count() as u64;
    }
    total
}

fn brute_neighbour_count(storage: &Storage, radius: f32) -> u64 {
    let mut total = 0u64;
    for i in 0..storage.len() {
        let p = storage.parts[i].pos;
        for j in 0..storage.len() {
            if i == j {
                continue;
            }
            let q = storage.parts[j].pos;
            let dx = p.x() - q.x();
            let dy = p.y() - q.y();
            if dx * dx + dy * dy <= radius * radius {
                total += 1;
            }
        }
    }
    total
}

fn bench_n(c: &mut Criterion, n: usize) {
    let world = Rect::new(0.0, 0.0, 1000.0, 1000.0);
    let storage = make_storage(n, world, 42);
    let qt = build_qt(&storage);
    let radius = 50.0;

    // For larger n the brute-force O(n^2) sweep dominates wall clock;
    // keep sample counts modest so the bench finishes in a reasonable
    // time. The speedup ratio is what matters; absolute timing
    // precision is less important.
    let samples = if n <= 2_000 {
        50
    } else if n <= 10_000 {
        20
    } else {
        10
    };
    let mut group = c.benchmark_group(format!("n={n}"));
    group.sample_size(samples);
    group.bench_function("quadtree_query", |b| {
        b.iter(|| {
            let total = qt_neighbour_count(&storage, &qt, radius);
            criterion::black_box(total);
        });
    });
    group.bench_function("brute_force_query", |b| {
        b.iter(|| {
            let total = brute_neighbour_count(&storage, radius);
            criterion::black_box(total);
        });
    });
    group.bench_function("full_collisions_step", |b| {
        b.iter(|| {
            let qt = build_qt(&storage);
            let mut s = storage_clone(&storage);
            let mut scratch = Vec::new();
            collisions::step(&mut s, &qt, &Default::default(), 1.0 / 60.0, &mut scratch);
            criterion::black_box(s);
        });
    });
    group.finish();
}

fn storage_clone(s: &Storage) -> Storage {
    Storage {
        parts: s.parts.clone(),
        world: s.world,
    }
}

fn bench_100(c: &mut Criterion) {
    bench_n(c, 100);
}
fn bench_500(c: &mut Criterion) {
    bench_n(c, 500);
}
fn bench_1k(c: &mut Criterion) {
    bench_n(c, 1_000);
}
fn bench_2k(c: &mut Criterion) {
    bench_n(c, 2_000);
}
fn bench_5k(c: &mut Criterion) {
    bench_n(c, 5_000);
}
fn bench_10k(c: &mut Criterion) {
    bench_n(c, 10_000);
}

criterion_group!(benches, bench_100, bench_500, bench_1k, bench_2k, bench_5k, bench_10k);
criterion_main!(benches);

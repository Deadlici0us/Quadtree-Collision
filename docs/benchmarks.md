# Benchmarks

Measured with `cargo bench` (Criterion 0.5) on a single core, Rust 1.98
release build. Numbers are wall-clock per full neighbour query sweep over
all `n` particles, plus a full `collisions::step` (build QT + resolve +
integrate).

| n particles | Brute-force query | QuadTree query | Speedup | Full collisions step |
| ----------- | ----------------: | -------------: | ------: | -------------------: |
| 100         |          0.021 ms |       0.007 ms |    2.9× |              0.012 ms |
| 500         |          0.518 ms |       0.097 ms |    5.3× |              0.106 ms |
| 1,000       |          2.085 ms |       0.378 ms |    5.5× |              0.311 ms |
| 2,000       |          8.359 ms |       1.232 ms |    6.8× |              0.775 ms |
| 5,000       |         51.67  ms |       6.22  ms |    8.3× |              2.97  ms |
| 10,000      |        205.68  ms |      21.19  ms |    9.7× |              8.40  ms |

A 60 FPS budget is 16.6 ms per frame. Within the demo's slider range
(100–10,000 per side):

- **Neighbour query only** stays under the budget at every n ≤ 5,000,
  climbs to 21 ms at n = 10,000 (FPS ≈ 47).
- **Full collisions step** (QT rebuild + resolve + integrate + bounce)
  stays under the budget at every n ≤ 10,000 — even at the slider max
  the sim finishes in 8.4 ms, leaving ~8 ms for the Canvas2D draw loop.
- **Split view** runs two sims sequentially on a single thread, so the
  effective budget per side is half the frame. At the default 500+500
  the total sim work is ~0.2 ms — comfortably within budget.

The QuadTree is faster than brute force at every n tested. The
speedup ratio is roughly 3× at very small n (constant overhead of tree
construction dominates) and grows toward 10× as n climbs.

## Reproduction

```bash
cargo bench
```

Sample sizes drop with `n` (50 at n ≤ 2k, 20 at 5k–10k) so the full
sweep finishes in a couple of minutes; the speedup ratios are stable
across sample sizes.

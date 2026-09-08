#pragma once

#include <cmath>

namespace routing {

// Physical cost of a diagonal grid move relative to a cardinal move. On a
// 4-connected grid a diagonal "staircase" of two cardinal moves would cost 2.0,
// but a true Euclidean diagonal trace is sqrt(2) ~= 1.414. Used consistently by
// the A* step cost and the heuristic so they can never drift apart.
// ---- Fixed-point cost quantum (optimization plan, Stage A1) --------------------
// Every cost SOURCE is an exact multiple of 1/1024 stored in a double. Integers
// (and their dyadic products/sums) are exact in doubles, so all hot-path cost
// arithmetic becomes exact and associative: immune to FMA contraction,
// instruction reordering, and compiler flags. This is the determinism
// foundation for -march=native, cloud fleets, and the integer/bucket-queue and
// GPU stages (which reuse the same quantized values as true integers).
constexpr double COST_QUANTUM = 1.0 / 1024.0;

inline double quantize_cost(double v) {
    // std::round: deterministic half-away-from-zero, no rounding-mode dependence
    return std::round(v * 1024.0) * COST_QUANTUM;
}

// Diagonal move cost, pre-quantized (1448/1024 = 1.4140625; rel. error 7e-5 vs
// sqrt(2) — a 1000-step diagonal path drifts ~0.07 base units, far below any
// meaningful routing decision threshold). Heuristic and step cost share this
// exact constant so admissibility/consistency hold EXACTLY in the quantized
// metric.
constexpr double DIAG_FACTOR = 1448.0 / 1024.0;

// Default influence radius (in cells) of an avoidance object. The kernel is exactly
// zero beyond this, so penalties only reach where they still matter.
// k/(1+R^2) at R=16 is ~0.4% of k — negligible vs. base cost.
constexpr double DEFAULT_FALLOFF_RADIUS = 16.0;

// Parameters controlling the A* cost model.
//
// Cost of entering a cell = base_cost
//     + pad_avoid_mult   * pad_avoidance(cell)
//     + trace_avoid_mult * trace_avoidance(cell)
//
// pad_avoidance / trace_avoidance use a bounded-radius, distance-squared falloff:
// they bite hard near the object and are exactly zero beyond `falloff_radius`.
struct CostParams {
    double base_cost = 1.0;

    // Pad avoidance: cost grows near component pads/obstacles.
    double pad_avoid_mult = 0.0;   // multiplier for the pad-avoidance term
    // Trace avoidance: cost grows near already-routed traces (other nets).
    double trace_avoid_mult = 0.0; // multiplier for the trace-avoidance term

    // Falloff scale in cells: cost(d) = k / (1 + d^2) for d < falloff_radius.
    double pad_falloff_k = 8.0;      // sharpness of pad avoidance
    double trace_falloff_k = 8.0;    // sharpness of trace avoidance

    // Influence radius (cells): contribution is 0 beyond this distance.
    double falloff_radius = DEFAULT_FALLOFF_RADIUS;

    // Via transition cost (multi-layer).
    double via_cost = 10.0;
};

// Euclidean distance in cells between two integer cell coordinates.
inline double euclid_dist(double dx, double dy) {
    return std::sqrt(dx * dx + dy * dy);
}

// Bounded-radius, distance-squared falloff kernel.
//   falloff(d) = k / (1 + d^2)      for d < radius   (bites hard close, ~1/d^2 decay)
//              = 0                  for d >= radius  (only extends where it matters)
inline double falloff_cost(double distance, double k, double radius = DEFAULT_FALLOFF_RADIUS) {
    if (distance >= radius) return 0.0;
    // Quantized at the single shared source: baked fields and the on-the-fly
    // self-exemption sums (self_pad_cost) produce IDENTICAL values, so
    // (field - self) stays exact and the self-exemption identity is preserved
    // to the bit.
    return quantize_cost(k / (1.0 + distance * distance));
}

// Total per-cell cost given the distance-to-nearest-pad and
// distance-to-nearest-trace, using the supplied parameters.
inline double cell_cost(const CostParams& p, double pad_dist, double trace_dist) {
    double c = p.base_cost;
    c += p.pad_avoid_mult * falloff_cost(pad_dist, p.pad_falloff_k, p.falloff_radius);
    c += p.trace_avoid_mult * falloff_cost(trace_dist, p.trace_falloff_k, p.falloff_radius);
    return c;
}

} // namespace routing

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "routing/cost_model.hpp"

using namespace routing;

TEST_CASE("falloff_sharp_and_decaying") {
    // Bounded distance-squared kernel k/(1+d^2): distance 0 is the most expensive
    // (== k), decays ~1/d^2, and is exactly 0 beyond the radius.
    CHECK(falloff_cost(0, 8.0) == doctest::Approx(8.0));
    CHECK(falloff_cost(0, 8.0) > falloff_cost(1, 8.0));
    CHECK(falloff_cost(1, 8.0) == doctest::Approx(8.0 / 2.0));     // k/(1+1)
    // values are quantized to 1/1024 cost units (Stage A1 determinism)
    CHECK(falloff_cost(3, 8.0) == doctest::Approx(8.0 / 10.0).epsilon(0.001));  // k/(1+9)
    // Larger k -> sharper
    CHECK(falloff_cost(1, 16.0) > falloff_cost(1, 8.0));
}

TEST_CASE("falloff_bounded_radius") {
    // The kernel must be exactly zero beyond the chosen radius and positive within it
    // (penalties only extend where they have significant effect).
    double R = 16.0;
    CHECK(falloff_cost(R, 8.0, R) == 0.0);
    CHECK(falloff_cost(R + 1.0, 8.0, R) == 0.0);
    CHECK(falloff_cost(100.0, 8.0, R) == 0.0);
    CHECK(falloff_cost(0.0, 8.0, R) > 0.0);
    CHECK(falloff_cost(R - 0.001, 8.0, R) > 0.0);
    // Strictly decreasing within the radius.
    CHECK(falloff_cost(1.0, 8.0, R) > falloff_cost(2.0, 8.0, R));
    CHECK(falloff_cost(2.0, 8.0, R) > falloff_cost(8.0, 8.0, R));
}

TEST_CASE("base_only_when_multipliers_zero") {
    CostParams p;
    p.base_cost = 1.0;
    p.pad_avoid_mult = 0.0;
    p.trace_avoid_mult = 0.0;
    // independent of distance when multipliers are 0
    CHECK(cell_cost(p, 0, 0) == doctest::Approx(1.0));
    CHECK(cell_cost(p, 5, 5) == doctest::Approx(1.0));
}

TEST_CASE("pad_avoidance_increases_cost") {
    CostParams p;
    p.base_cost = 1.0;
    p.pad_avoid_mult = 1.0;
    p.trace_avoid_mult = 0.0;
    p.pad_falloff_k = 8.0;
    double on_pad = cell_cost(p, 0, 99);   // pad_dist 0
    double far = cell_cost(p, 8, 99);      // pad_dist far from pad
    CHECK(on_pad > far);
    // And raising the multiplier scales the pad term only.
    CostParams p2 = p;
    p2.pad_avoid_mult = 2.0;
    CHECK(cell_cost(p2, 2, 99) > cell_cost(p, 2, 99));
}

TEST_CASE("trace_avoidance_increases_cost") {
    CostParams p;
    p.pad_avoid_mult = 0.0;
    p.trace_avoid_mult = 1.0;
    p.trace_falloff_k = 8.0;
    double on_trace = cell_cost(p, 99, 0);
    double far = cell_cost(p, 99, 8);
    CHECK(on_trace > far);
    CostParams p2 = p;
    p2.trace_avoid_mult = 2.0;
    CHECK(cell_cost(p2, 99, 2) > cell_cost(p, 99, 2));
}

TEST_CASE("independence_of_terms") {
    CostParams p;
    p.base_cost = 1.0;
    p.pad_avoid_mult = 0.0;
    p.trace_avoid_mult = 0.0;
    double v0 = cell_cost(p, 3, 4);
    // Toggling pad only changes pad term:
    p.pad_avoid_mult = 1.0;
    double v1 = cell_cost(p, 3, 4);
    CHECK(v1 > v0);
    // Toggling trace only changes trace term:
    CostParams q;
    q.base_cost = 1.0;
    q.pad_avoid_mult = 0.0;
    q.trace_avoid_mult = 0.0;
    double w0 = cell_cost(q, 3, 4);
    q.trace_avoid_mult = 1.0;
    double w1 = cell_cost(q, 3, 4);
    CHECK(w1 > w0);
    // Sum is additive: pad-only + trace-only == both
    double combined = p.pad_avoid_mult * falloff_cost(3, p.pad_falloff_k)
                    + 1.0 * falloff_cost(4, q.trace_falloff_k)
                    + 1.0;
    CostParams both;
    both.base_cost = 1.0;
    both.pad_avoid_mult = 1.0;
    both.trace_avoid_mult = 1.0;
    CHECK(cell_cost(both, 3, 4) == doctest::Approx(combined));
}

TEST_CASE("multiplier_scales_linearly") {
    CostParams p1, p2;
    p1.base_cost = p2.base_cost = 1.0;
    p1.trace_avoid_mult = 1.0;
    p2.trace_avoid_mult = 2.0;
    p1.pad_avoid_mult = p2.pad_avoid_mult = 0.5;
    // doubling trace mult adds +trace_falloff_cost(d)
    double d1 = cell_cost(p1, 4, 4);
    double d2 = cell_cost(p2, 4, 4);
    double diff = d2 - d1;
    CHECK(diff == doctest::Approx(falloff_cost(4, p1.trace_falloff_k)));
}

// ---------------------------------------------------------------------------
// STAGE 1b: Euclidean avoidance distance
// ---------------------------------------------------------------------------

TEST_CASE("avoidance_euclidean") {
    // With the physical Euclidean distance model, a (1,1) diagonal neighbor is at
    // distance sqrt(2) and a (1,0) cardinal neighbor at distance 1, so the squared
    // kernel differs: k/(1+2) = k/3 vs k/(1+1) = k/2.
    double k = 8.0;
    double diag = falloff_cost(euclid_dist(1, 1), k);   // k/(1+2) = k/3
    double cardinal = falloff_cost(euclid_dist(1, 0), k); // k/(1+1) = k/2
    CHECK(cardinal == doctest::Approx(k / 2.0));
    CHECK(diag == doctest::Approx(k / 3.0).epsilon(0.001));  // 1/1024-quantized
    // The diagonal neighbor is closer in Euclidean terms, so its avoidance is higher
    // than a naive Manhattan(2) cell would suggest, but farther than the cardinal.
    CHECK(diag > falloff_cost(2, k));       // 8/5 = 1.6 < 8/3
    CHECK(diag < cardinal);                 // the cardinal neighbor is closer
}

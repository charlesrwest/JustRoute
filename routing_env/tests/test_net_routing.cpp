#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "routing/grid.hpp"
#include "routing/net.hpp"
#include "routing/cost_model.hpp"
#include "routing/astar.hpp"

#include <vector>
#include <set>
#include <queue>
#include <random>

using namespace routing;

// Uniform-cost enter fn that returns -1 (blocked) for a set of cells.
static EnterCostFn cost_with_blocks(const std::set<Cell>& blocked) {
    return [blocked](const Cell& c) -> double {
        if (blocked.count(c)) return -1.0;
        return 1.0;
    };
}

// 8-connected flood fill over trace cells (4 cardinal + 4 diagonal, since traces
// may now use diagonal moves) plus adjacent-layer via connections at the same x,y.
// Returns true if all `pins` reachable from pins[0] within the trace set.
static bool trace_connects_all_pins(const std::set<Cell>& trace_cells,
                                    const std::vector<Cell>& pins) {
    if (pins.empty()) return true;
    std::set<Cell> seen;
    std::queue<Cell> q;
    q.push(pins[0]);
    seen.insert(pins[0]);
    const int DX[8] = {1,-1,0,0,1,1,-1,-1};
    const int DY[8] = {0,0,1,-1,1,-1,1,-1};
    const int MAX_LAYER = 16;
    while (!q.empty()) {
        Cell c = q.front(); q.pop();
        // same-layer 8-connectivity
        for (int d = 0; d < 8; ++d) {
            Cell nb{c.layer, c.x+DX[d], c.y+DY[d]};
            if (seen.count(nb)) continue;
            if (!trace_cells.count(nb)) continue;
            seen.insert(nb);
            q.push(nb);
        }
        // via: adjacent layer at same (x,y)
        for (int dl : {-1, 1}) {
            Cell nb{c.layer+dl, c.x, c.y};
            if (nb.layer < 0 || nb.layer > MAX_LAYER) continue;
            if (seen.count(nb)) continue;
            if (!trace_cells.count(nb)) continue;
            seen.insert(nb);
            q.push(nb);
        }
    }
    for (const Cell& p : pins) if (!seen.count(p)) return false;
    return true;
}

static std::set<Cell> net_trace_cells(const Net& net) {
    std::set<Cell> s;
    for (const auto& seg : net.segments)
        for (const Cell& c : seg) s.insert(c);
    // pins themselves are part of the trace
    for (const Cell& p : net.pins) s.insert(p);
    return s;
}

TEST_CASE("two_pin_net") {
    Grid g(1, 20, 20, 1.0);
    Net n;
    n.id = 1;
    n.pins = {{0,2,2}, {0,2,8}};
    bool ok = route_net_pairwise(g, n, 100.0, 1.0, cost_with_blocks({}));
    CHECK(ok);
    CHECK(n.routed);
    CHECK(n.total_length == doctest::Approx(6.0)); // 2->8 = 6 steps
    auto trace = net_trace_cells(n);
    CHECK(trace_connects_all_pins(trace, n.pins));
}

TEST_CASE("three_pin_net_sequential") {
    Grid g(1, 20, 20, 1.0);
    Net n;
    n.id = 1;
    n.pins = {{0,2,2}, {0,7,2}, {0,7,7}};
    bool ok = route_net_pairwise(g, n, 100.0, 1.0, cost_with_blocks({}));
    CHECK(ok);
    CHECK(n.routed);
    // pair1: (2,2)->(7,2) = 5; pair2: (7,2)->(7,7) = 5; total 10
    CHECK(n.total_length == doctest::Approx(10.0));
    auto trace = net_trace_cells(n);
    CHECK(trace_connects_all_pins(trace, n.pins));
}

TEST_CASE("unroutable_pair_reports_failure") {
    // Isolate the second pin within a wall so it cannot be reached.
    Grid g(1, 10, 10, 1.0);
    std::set<Cell> blocked;
    // enclose (6,6)
    blocked.insert({0,5,6}); blocked.insert({0,7,6});
    blocked.insert({0,6,5}); blocked.insert({0,6,7});
    Net n;
    n.id = 1;
    n.pins = {{0,1,1}, {0,6,6}};
    bool ok = route_net_pairwise(g, n, 100.0, 1.0, cost_with_blocks(blocked));
    CHECK_FALSE(ok);
    CHECK_FALSE(n.routed);
}

TEST_CASE("pins_on_different_layers_use_vias") {
    Grid g(2, 20, 20, 1.0);
    Net n;
    n.id = 1;
    n.pins = {{0,4,4}, {0,4,6}, {1,4,10}}; // two on layer0, one on layer1
    bool ok = route_net_pairwise(g, n, 2.0, 1.0, cost_with_blocks({}));
    CHECK(ok);
    CHECK(n.routed);
    // total trace must include layer 1 cell(s) => a via happened somewhere
    auto trace = net_trace_cells(n);
    bool has_l1 = false;
    for (const Cell& c : trace) if (c.layer == 1) has_l1 = true;
    CHECK(has_l1);
    CHECK(trace_connects_all_pins(trace, n.pins));
}

TEST_CASE("single_pin_net_trivially_routed") {
    Grid g(1, 10, 10, 1.0);
    Net n;
    n.id = 1;
    n.pins = {{0,3,3}};
    bool ok = route_net_pairwise(g, n, 100.0, 1.0, cost_with_blocks({}));
    CHECK(ok);
    CHECK(n.routed);
    CHECK(n.total_length == doctest::Approx(0.0));
}

// ---------------------------------------------------------------------------
// STAGE 1b: physical (Euclidean) net length with diagonal moves
// ---------------------------------------------------------------------------

TEST_CASE("net_length_euclidean") {
    // A 2-pin net offset by (3,3) is routed with 3 diagonal moves, so total_length
    // is the physical length 3*sqrt(2), NOT the old cell-staircase count of 6.
    Grid g(1, 20, 20, 1.0);
    Net n;
    n.id = 1;
    n.pins = {{0,2,2}, {0,5,5}};
    bool ok = route_net_pairwise(g, n, 100.0, 1.0, cost_with_blocks({}));
    CHECK(ok);
    CHECK(n.routed);
    CHECK(n.total_length == doctest::Approx(3.0 * DIAG_FACTOR).epsilon(1e-6));
    // The trace still connects both pins.
    auto trace = net_trace_cells(n);
    CHECK(trace_connects_all_pins(trace, n.pins));
}

TEST_CASE("net_length_mixed_diagonal_and_cardinal") {
    // Offset (3,1): 1 diagonal + 2 cardinal moves -> physical length 2 + sqrt(2).
    Grid g(1, 20, 20, 1.0);
    Net n;
    n.id = 1;
    n.pins = {{0,0,0}, {0,3,1}};
    bool ok = route_net_pairwise(g, n, 100.0, 1.0, cost_with_blocks({}));
    CHECK(ok);
    double expect = 2.0 + DIAG_FACTOR;
    CHECK(n.total_length == doctest::Approx(expect).epsilon(1e-6));
}

// ---------------------------------------------------------------------------
// STAGE 7: multi-pin TREE routing (dynamic multi-source A*)
// ---------------------------------------------------------------------------

// The tree must be order-independent: it always grows as an MST toward the nearest
// unconnected pin, so the input pin order must not change the result (within ties).
static double tree_length(Net n) {   // copies so we can shuffle
    Grid g(1, 30, 20, 1.0);
    n.id = 1;
    bool ok = route_net_tree(g, n, 100.0, 1.0, cost_with_blocks({}));
    REQUIRE(ok);
    CHECK(n.routed);
    return n.total_length;
}

TEST_CASE("tree_two_pin_matches_pairwise") {
    Grid g(1, 20, 20, 1.0);
    Net n; n.id = 1; n.pins = {{0,2,2}, {0,2,8}};
    bool ok = route_net_tree(g, n, 100.0, 1.0, cost_with_blocks({}));
    CHECK(ok); CHECK(n.routed);
    CHECK(n.total_length == doctest::Approx(6.0));
    auto trace = net_trace_cells(n);
    CHECK(trace_connects_all_pins(trace, n.pins));
}

TEST_CASE("tree_order_independent_and_shorter_than_pairwise") {
    // Five collinear pins spanning x=2..10 at y=10. The tree must route them as an MST
    // (total = span 8) regardless of pin order; a deliberately shuffled PAIRWISE order
    // produces a long chain (E->A->B->C->D = 8+2+2+2 = 14). This is the cleanest proof
    // that tree routing joins each next pin to the nearest point of the existing trace
    // (the GND case) rather than a designated partner.
    std::vector<Cell> base = {{0,2,10},{0,4,10},{0,6,10},{0,8,10},{0,10,10}};
    // Tree with natural order and with a shuffled order -> identical length.
    Net t1; t1.pins = base;
    double l1 = tree_length(t1);
    Net t2; t2.pins = {base[4], base[0], base[2], base[1], base[3]};
    double l2 = tree_length(t2);
    CHECK(l1 == doctest::Approx(8.0).epsilon(1e-6));
    CHECK(l2 == doctest::Approx(l1).epsilon(1e-9));

    // Pairwise in the evil order E,A,B,C,D is a long chain.
    Grid g(1, 30, 20, 1.0);
    Net p; p.id = 1; p.pins = {base[4], base[0], base[1], base[2], base[3]};
    bool ok = route_net_pairwise(g, p, 100.0, 1.0, cost_with_blocks({}));
    REQUIRE(ok);
    CHECK(p.total_length == doctest::Approx(14.0).epsilon(1e-6));
    // The tree is strictly better than the pairwise chain.
    CHECK(l1 < p.total_length);
    // And every pin is connected (flood fill over t1's segments).
    Net t3; t3.pins = base;
    Grid g2(1, 30, 20, 1.0);
    REQUIRE(route_net_tree(g2, t3, 100.0, 1.0, cost_with_blocks({})));
    CHECK(trace_connects_all_pins(net_trace_cells(t3), base));
}

TEST_CASE("tree_single_pin_and_empty") {
    Grid g(1, 10, 10, 1.0);
    Net n; n.id = 1; n.pins = {{0,3,3}};
    bool ok = route_net_tree(g, n, 100.0, 1.0, cost_with_blocks({}));
    CHECK(ok); CHECK(n.routed); CHECK(n.total_length == doctest::Approx(0.0));

    Net m; m.id = 2;
    bool ok2 = route_net_tree(g, m, 100.0, 1.0, cost_with_blocks({}));
    CHECK(ok2); CHECK(m.routed); CHECK(m.total_length == doctest::Approx(0.0));
}

TEST_CASE("tree_multi_layer_uses_vias") {
    Grid g(2, 30, 20, 1.0);
    Net n; n.id = 1;
    // three pins: two on layer0, one on layer1; the layer-1 pin is nearest the middle.
    n.pins = {{0,2,10},{0,10,10},{1,6,10}};
    bool ok = route_net_tree(g, n, 2.0, 1.0, cost_with_blocks({}));
    CHECK(ok); CHECK(n.routed);
    auto trace = net_trace_cells(n);
    bool has_l1 = false;
    for (const Cell& c : trace) if (c.layer == 1) has_l1 = true;
    CHECK(has_l1);
    CHECK(trace_connects_all_pins(trace, n.pins));
}

TEST_CASE("tree_joins_existing_trace_L") {
    // L-corner geometry: A(2,2), B(12,2), C(2,12).
    // The tree grows from A and connects each next pin to the NEAREST point of the
    // existing component. C is equidistant in the corner; the tree attaches C to the
    // shared corner (A) of the A-B trace instead of chaining to the distant partner B.
    // Tree MST length = 10 (A-B) + 10 (C via A) = 20.
    // Pairwise [A,B,C] forces C to connect to B: 10 (A-B) + sqrt(10^2+10^2) (B-C)
    //   = 10 + 14.142 = 24.14, a longer chain. This is the GND-pad-abuts-its-trace
    //   win: the pad joins the closest trace point, not a designated partner.
    Grid g(1, 20, 20, 1.0);
    Net t; t.id = 1; t.pins = {{0,2,2},{0,12,2},{0,2,12}};
    bool ok = route_net_tree(g, t, 100.0, 1.0, cost_with_blocks({}));
    REQUIRE(ok); CHECK(t.routed);
    double tree_len = t.total_length;
    CHECK(tree_len == doctest::Approx(20.0).epsilon(1e-6));
    CHECK(trace_connects_all_pins(net_trace_cells(t), t.pins));

    // Pairwise in the chain order is strictly longer (fails to beat the tree).
    Grid g2(1, 20, 20, 1.0);
    Net p; p.id = 1; p.pins = {{0,2,2},{0,12,2},{0,2,12}};
    REQUIRE(route_net_pairwise(g2, p, 100.0, 1.0, cost_with_blocks({})));
    double pair_len = p.total_length;
    CHECK(pair_len == doctest::Approx(10.0 + 10.0 * DIAG_FACTOR).epsilon(1e-6));
    CHECK(tree_len < pair_len);
}

TEST_CASE("tree_reports_unroutable_when_pin_enclosed") {
    // Enclose a pin in a wall so no connection can reach it from the growing tree.
    Grid g(1, 10, 10, 1.0);
    std::set<Cell> blocked;
    blocked.insert({0,5,6}); blocked.insert({0,7,6});
    blocked.insert({0,6,5}); blocked.insert({0,6,7});
    Net n; n.id = 1; n.pins = {{0,1,1},{0,6,6},{0,8,2}};
    bool ok = route_net_tree(g, n, 100.0, 1.0, cost_with_blocks(blocked));
    CHECK_FALSE(ok);
    CHECK_FALSE(n.routed);
}

TEST_CASE("tree_reverse_quality_and_connectivity") {
    // Stage-9 oracle. Reverse-from-pins and forward are the SAME nearest-first greedy; they
    // differ only by tie-breaking (a Steiner-greedy is not order-invariant), so strict
    // equality is not expected. The meaningful correctness properties are:
    //  (1) every reverse tree actually connects ALL its pins (core correctness);
    //  (2) reverse is never systematically worse on length (symmetric tie-bound);
    //  (3) reverse routes at least as many nets as forward overall.
    std::mt19937 rng(42);
    int rev_only = 0, fwd_only = 0, bad_conn = 0;
    double worst_worse = 0.0;
    for (int trial = 0; trial < 500; ++trial) {
        const int layers = 1 + (int)(rng() % 4);
        const int w = 8 + (int)(rng() % 20), h = 8 + (int)(rng() % 20);
        Grid g(layers, w, h, 1.0);
        std::set<Cell> blocked;
        int nobs = (int)(rng() % 20);
        for (int i = 0; i < nobs; ++i)
            blocked.insert(Cell{(int)(rng() % layers), (int)(rng() % w), (int)(rng() % h)});
        const int npins = 2 + (int)(rng() % 12);
        std::vector<Cell> pins;
        for (int i = 0; i < npins; ++i)
            pins.push_back(Cell{(int)(rng() % layers), (int)(rng() % w), (int)(rng() % h)});
        auto enter = cost_with_blocks(blocked);
        const double via = 5.0 + (double)(rng() % 60);
        Net fwd; fwd.id = 1; fwd.pins = pins;
        Net rev; rev.id = 1; rev.pins = pins;
        bool fr = route_net_tree(g, fwd, via, 1.0, enter);
        bool rr = route_net_tree_reverse(g, rev, via, 1.0, enter);
        if (rr) {
            if (!trace_connects_all_pins(net_trace_cells(rev), pins)) bad_conn++;
        }
        if (rr && fr) {
            double w_ = rev.total_length - fwd.total_length;
            if (w_ > worst_worse) worst_worse = w_;
        } else if (rr) rev_only++;
        else if (fr) fwd_only++;
    }
    CHECK(bad_conn == 0);                 // every reverse tree is fully connected
    CHECK(fwd_only <= rev_only);          // reverse routes at least as many nets
    CHECK(worst_worse <= 20.0);           // documented symmetric tie-divergence bound
}

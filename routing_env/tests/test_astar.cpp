#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "routing/grid.hpp"
#include "routing/cost_model.hpp"
#include "routing/astar.hpp"

#include <vector>
#include <set>

using namespace routing;

// Simple uniform-cost enter-cost fn (same value for all cells).
static EnterCostFn uniform_cost(double base) {
    return [base](const Cell&) { return base; };
}

TEST_CASE("straight_line") {
    Grid g(1, 10, 10, 1.0);
    Cell s{0, 1, 1}, t{0, 1, 5};
    auto r = astar_route(g, s, t, 100.0, 1.0, uniform_cost(1.0));
    REQUIRE(r.found);
    // Manhattan distance = 4 + horizontal 0.
    CHECK(r.cost == doctest::Approx(4.0));
    CHECK(r.path.front() == s);
    CHECK(r.path.back() == t);
    CHECK(r.path.size() == 5); // 4 steps + start
}

TEST_CASE("around_obstacle") {
    // Build a grid; block a vertical wall between start and goal forcing a detour.
    // Leave a gap at the top row (y=9) so the path can go around.
    Grid g(1, 10, 10, 1.0);
    std::set<Cell> blocked;
    // wall at x=5, y=0..8 blocked -> cannot cross at y<=8; gap at y=9
    for (int y = 0; y < 9; ++y) blocked.insert(Cell{0, 5, y});
    auto ent = [&](const Cell& c) -> double {
        if (blocked.count(c)) return -1.0; // blocked
        return 1.0;
    };
    Cell s{0, 2, 2}, t{0, 7, 2};
    auto r = astar_route(g, s, t, 100.0, 1.0, ent);
    REQUIRE(r.found);
    // Path must go around the wall. Ensure it avoids blocked cells.
    for (const Cell& c : r.path) {
        CHECK_FALSE(blocked.count(c));
    }
    CHECK(r.path.front() == s);
    CHECK(r.path.back() == t);
    // Must pass through the gap: the path must include the top row (y=9).
    bool used_gap = false;
    for (const Cell& c : r.path) if (c.y == 9) used_gap = true;
    CHECK(used_gap);
}

TEST_CASE("no_path_enclosed") {
    Grid g(1, 10, 10, 1.0);
    // Surround the goal with blocked cells.
    Cell t{0, 3, 3};
    std::set<Cell> blocked;
    blocked.insert({0,2,3}); blocked.insert({0,4,3});
    blocked.insert({0,3,2}); blocked.insert({0,3,4});
    auto ent = [&](const Cell& c) -> double {
        if (blocked.count(c)) return -1.0;
        return 1.0;
    };
    Cell s{0, 0, 0};
    auto r = astar_route(g, s, t, 100.0, 1.0, ent);
    CHECK_FALSE(r.found);
}

TEST_CASE("heuristic_admissible_under_variable_enter_cost") {
    // Regression for a critical bug: the heuristic must use the true global base
    // cost floor, NOT enter_cost(start). If it used start's (inflated) cost, the
    // heuristic would overestimate and A* could return a SUBOPTIMAL path.
    Grid g(1, 5, 5, 1.0);
    // Start cell reports a huge entry cost; the direct-path cell (1,0) also huge.
    auto ent = [](const Cell& c) -> double {
        if (c.x == 0 && c.y == 0) return 100.0; // start (never paid, g=0)
        if (c.x == 1 && c.y == 0) return 100.0; // expensive direct cell
        return 1.0;
    };
    Cell s{0, 0, 0}, t{0, 2, 0};
    auto r = astar_route(g, s, t, 100.0, /*base_cost=*/1.0, ent);
    REQUIRE(r.found);
    // True optimal avoids (1,0): go down/around. Cost must be well below 101.
    // Expected detour: down to (0,1), across to (2,1), up to (2,0) = 4 steps * 1 = 4.
    CHECK(r.cost < 10.0);
}

TEST_CASE("shortest_octile") {
    // On an 8-connected grid with cardinal=1 / diagonal=sqrt(2), the shortest path
    // from (0,0) to (13,7) is not a Manhattan staircase (20) but uses 7 diagonal + 6
    // cardinal moves (13 cells) at true Euclidean cost ~= 15.9.
    Grid g(1, 20, 20, 1.0);
    Cell s{0, 0, 0}, t{0, 13, 7};
    auto r = astar_route(g, s, t, 100.0, 1.0, uniform_cost(1.0));
    REQUIRE(r.found);
    double dx = 13.0, dy = 7.0;
    double mn = std::min(dx, dy), mx = std::max(dx, dy);
    double octile = (mx - mn) + mn * DIAG_FACTOR;
    CHECK(r.cost == doctest::Approx(octile).epsilon(1e-6));
    // Path uses mn diagonals + (mx-mn) cardinals
    CHECK(r.path.size() == static_cast<size_t>(mx + 1));
    // Physical length matches octile distance too.
    CHECK(path_euclidean_length(r.path) == doctest::Approx(octile).epsilon(1e-6));
}

TEST_CASE("determinism") {
    Grid g(1, 15, 15, 1.0);
    std::set<Cell> blocked{{0,5,5},{0,5,6},{0,6,5}};
    auto ent = [&](const Cell& c) -> double { return blocked.count(c) ? -1.0 : 1.0; };
    Cell s{0, 0, 0}, t{0, 14, 14};
    auto r1 = astar_route(g, s, t, 100.0, 1.0, ent);
    auto r2 = astar_route(g, s, t, 100.0, 1.0, ent);
    REQUIRE(r1.found);
    REQUIRE(r2.found);
    bool same_path = (r1.path == r2.path);
    CHECK(same_path);
    CHECK(r1.cost == doctest::Approx(r2.cost));
}

TEST_CASE("same_layer_when_via_cost_high") {
    Grid g(3, 10, 10, 1.0);
    Cell s{0, 1, 1}, t{1, 1, 5}; // goal on layer 1
    // Very high via cost -> A* should prefer not to change layers if it can avoid
    // reaching a different layer. But the goal IS on layer 1, so at least one via is
    // required. Test: with a huge via cost, path uses the minimal number of vias (1).
    auto r = astar_route(g, s, t, 1e6, 1.0, uniform_cost(1.0));
    REQUIRE(r.found);
    int layer_changes = 0;
    for (size_t i = 1; i < r.path.size(); ++i)
        if (r.path[i].layer != r.path[i-1].layer) layer_changes++;
    CHECK(layer_changes == 1); // exactly one transition to reach the goal layer
}

TEST_CASE("flux_via_uses_layer_switch") {
    // Start on layer 0, goal on layer 1 a short horizontal distance away but with a
    // wall on layer 0 between them; low via cost should let it switch layers to detour.
    Grid g(2, 10, 10, 1.0);
    std::set<Cell> blocked0;
    for (int y = 0; y < 10; ++y) blocked0.insert(Cell{0, 5, y});
    auto ent = [&](const Cell& c) -> double {
        if (c.layer == 0 && blocked0.count(c)) return -1.0;
        if (c.layer == 1 && c.x >= 0 && c.x < 10 && c.y >= 0 && c.y < 10) return 1.0;
        return 1.0;
    };
    Cell s{0, 2, 2}, t{1, 7, 2};
    auto r = astar_route(g, s, t, 3.0, 1.0, ent); // cheap vias
    REQUIRE(r.found);
    // Expect the path to have used layer 1 (layer changes > 0).
    bool used_layer1 = false;
    for (const Cell& c : r.path) if (c.layer == 1) used_layer1 = true;
    CHECK(used_layer1);
    // And it should not pass through the wall on layer 0.
    for (const Cell& c : r.path) {
        bool on_wall = (c.layer == 0 && blocked0.count(c));
        REQUIRE_FALSE(on_wall);
    }
}

TEST_CASE("via_adjacent_only") {
    // A* cannot jump directly from layer 0 to layer 2; must pass through layer 1.
    Grid g(3, 10, 10, 1.0);
    Cell s{0, 0, 0}, t{2, 0, 0};
    auto r = astar_route(g, s, t, 2.0, 1.0, uniform_cost(1.0));
    REQUIRE(r.found);
    std::set<int> layers;
    for (const Cell& c : r.path) layers.insert(c.layer);
    // To get from 0 to 2 with ±1 transitions, layer 1 must be visited.
    CHECK(layers.count(1) == 1);
    // Ensure no non-adjacent layer jumps: verify each consecutive transition is ±1.
    for (size_t i = 1; i < r.path.size(); ++i) {
        int dl = r.path[i].layer - r.path[i-1].layer;
        if (dl != 0) CHECK(std::abs(dl) == 1);
    }
}

TEST_CASE("multi_layer_cost_reflects_vias") {
    Grid g(2, 10, 10, 1.0);
    Cell s{0, 1, 1}, t{1, 1, 2};
    // each horizontal step costs 1; need at least one via (cost via_cost) to layer 1.
    double via = 5.0;
    auto r = astar_route(g, s, t, via, 1.0, uniform_cost(1.0));
    REQUIRE(r.found);
    // min cost = 1 horizontal step (to reach layer-0 cell under goal) + via + ... 
    // More robustly: the cost must be >= via (at least one via required).
    CHECK(r.cost >= via - 1e-9);
}

// ---------------------------------------------------------------------------
// STAGE 1b: 8-connectivity diagonal moves
// ---------------------------------------------------------------------------

TEST_CASE("diagonal_length_is_sqrt2") {
    // Two pins offset by (3,3): the 8-connected router should take 3 diagonal moves
    // (no staircase), so physical length == 3*sqrt(2) and A* cost == that.
    Grid g(1, 20, 20, 1.0);
    Cell s{0, 0, 0}, t{0, 3, 3};
    auto r = astar_route(g, s, t, 100.0, 1.0, uniform_cost(1.0));
    REQUIRE(r.found);
    double expect = 3.0 * DIAG_FACTOR;
    CHECK(r.cost == doctest::Approx(expect).epsilon(1e-6));
    CHECK(path_euclidean_length(r.path) == doctest::Approx(expect).epsilon(1e-6));
    // Path is 4 cells: start + 3 diagonal steps.
    CHECK(r.path.size() == 4);
}

TEST_CASE("corner_cut_forbidden") {
    // Start (0,0), goal (1,1). Block (1,0) (one orthogonal neighbor of the diagonal).
    // The direct diagonal (0,0)->(1,1) would cut the corner at (1,0), so it must be
    // disallowed; the path must detour via (0,1) using two cardinal moves (cost 2),
    // not the sqrt(2) diagonal.
    Grid g(1, 10, 10, 1.0);
    auto ent = [](const Cell& c) -> double {
        if (c.layer == 0 && c.x == 1 && c.y == 0) return -1.0; // blocked
        return 1.0;
    };
    Cell s{0, 0, 0}, t{0, 1, 1};
    auto r = astar_route(g, s, t, 100.0, 1.0, ent);
    REQUIRE(r.found);
    // Must NOT be the sqrt(2) cut-corner diagonal; cost must be >= 2 (two cardinals).
    CHECK(r.cost >= 2.0 - 1e-9);
    CHECK(r.path.size() == 3); // (0,0) -> (0,1) -> (1,1)
}

TEST_CASE("diagonal_detour_used") {
    // Start (0,0), goal (2,2). Block (0,1) (one orthogonal neighbor of the first
    // diagonal). The direct first diagonal (0,0)->(1,1) is corner-cut-forbidden, so
    // A* detours east first then uses a diagonal: (0,0)->(1,0)->(2,1)->(2,2).
    // Cost = 1 + sqrt(2) + 1 = 2 + sqrt(2); a diagonal IS still used in the detour.
    Grid g(1, 10, 10, 1.0);
    auto ent = [](const Cell& c) -> double {
        if (c.layer == 0 && c.x == 0 && c.y == 1) return -1.0; // blocked
        return 1.0;
    };
    Cell s{0, 0, 0}, t{0, 2, 2};
    auto r = astar_route(g, s, t, 100.0, 1.0, ent);
    REQUIRE(r.found);
    double expect = 2.0 + DIAG_FACTOR;
    CHECK(r.cost == doctest::Approx(expect).epsilon(1e-6));
    CHECK(path_euclidean_length(r.path) == doctest::Approx(expect).epsilon(1e-6));
}

TEST_CASE("octile_admissible_under_variable_enter_cost") {
    // Regression: with the octile heuristic the A* result must still be minimal under
    // elevated per-cell entry costs. Start (0,0), goal (2,2); make the diagonal
    // neighbor (1,1) very expensive so A* must route around it cheaply.
    Grid g(1, 6, 6, 1.0);
    auto ent = [](const Cell& c) -> double {
        if (c.layer == 0 && c.x == 1 && c.y == 1) return 100.0; // expensive diagonal
        return 1.0;
    };
    Cell s{0, 0, 0}, t{0, 2, 2};
    auto r = astar_route(g, s, t, 100.0, /*base_cost=*/1.0, ent);
    REQUIRE(r.found);
    // True optimum avoids (1,1): e.g. (0,0)->(1,0)->(2,1)->(2,2) or the mirror = 3.
    // Suboptimal greedy diagonal through (1,1) would cost ~= 100*sqrt(2).
    CHECK(r.cost < 10.0);
    // And the found path must not pass through the expensive (1,1).
    bool visited_expensive = false;
    for (const Cell& c : r.path) {
        if (c.layer == 0 && c.x == 1 && c.y == 1) visited_expensive = true;
    }
    CHECK_FALSE(visited_expensive);
}

TEST_CASE("heuristic_sqrt2_matches_move") {
    // Consistency smoke: for several offsets on a uniform grid, A* cost equals the
    // sum of per-move physical costs (cardinal*1 + diagonal*sqrt(2)) and equals the
    // octile distance -- i.e. the heuristic never over/under-shoots on uniform cost.
    const int offsets[][2] = {{5,3},{8,1},{0,6},{7,7},{4,9},{12,4}};
    for (auto& off : offsets) {
        int dx = off[0], dy = off[1];
        Grid g(1, 25, 25, 1.0);
        Cell s{0, 0, 0}, t{0, dx, dy};
        auto r = astar_route(g, s, t, 100.0, 1.0, uniform_cost(1.0));
        REQUIRE(r.found);
        double mn = (double)std::min(dx, dy), mx = (double)std::max(dx, dy);
        double octile = (mx - mn) + mn * DIAG_FACTOR;
        CHECK(r.cost == doctest::Approx(octile).epsilon(1e-6));
        CHECK(path_euclidean_length(r.path) == doctest::Approx(r.cost).epsilon(1e-6));
    }
}

// ---------------------------------------------------------------------------
// Regression: via transitions must respect the enter_cost blocked sentinel.
// Before the fix, `step = via_cost + enter_cost(nb)` swallowed -1 whenever
// via_cost > 1 (e.g. 5 + (-1) = 4), letting A* tunnel through hard-blocked
// cells at layer transitions in all three search variants.
// ---------------------------------------------------------------------------
TEST_CASE("via_transition_respects_blocked_sentinel") {
    Grid g(2, 5, 5, 1.0);
    Cell s{0, 2, 2}, t{1, 2, 2};
    EnterCostFn enter = [](const Cell& c) -> double {
        return c.layer == 1 ? -1.0 : 1.0;   // whole target layer hard-blocked
    };
    auto r = astar_route(g, s, t, 5.0, 1.0, enter);
    CHECK_FALSE(r.found);

    auto rm = astar_route_multi(g, {s}, {t}, 5.0, 1.0, enter);
    CHECK_FALSE(rm.found);

    auto rt = astar_route_tree(g, {s}, [t](const Cell& c) { return c == t; },
                               5.0, 1.0, enter);
    CHECK_FALSE(rt.found);
}

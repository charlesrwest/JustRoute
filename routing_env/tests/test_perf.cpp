#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "routing/astar.hpp"
#include "routing/cost_model.hpp"
#include "routing/grid.hpp"

#include <cmath>
#include <queue>
#include <random>
#include <vector>

using namespace routing;

// Self-contained REFERENCE A* — the exact PRE-optimization algorithm (fresh local
// `g`/`parent` arrays, lazy-deletion priority queue). Used as the ground truth to prove
// the current A* should produce bit-for-bit identical results to this reference
// (regression guard: repeated searches must stay deterministic).
namespace {

constexpr int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
constexpr bool IS_DIAG[8] = {false, false, false, false, true, true, true, true};

struct RefEntry { double f; double g; Cell cell; };
struct RefCmp { bool operator()(const RefEntry& a, const RefEntry& b) const { return a.f > b.f; } };

double ref_heuristic(const Cell& c, const Cell& goal, double via, double base) {
    double dx = std::abs((double)(c.x - goal.x));
    double dy = std::abs((double)(c.y - goal.y));
    double mn = std::min(dx, dy), mx = std::max(dx, dy);
    return ((mx - mn) + mn * DIAG_FACTOR) * base + via * std::abs(c.layer - goal.layer);
}

AStarResult reference_astar(const Grid& grid, const Cell& start, const Cell& goal,
                            double via, double base, const EnterCostFn& enter) {
    AStarResult r;
    if (!grid.valid(start) || !grid.valid(goal)) return r;
    if (start == goal) { r.found = true; r.path = {start}; r.cost = 0.0; return r; }

    const size_t n = grid.size();
    std::vector<double> g(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    std::priority_queue<RefEntry, std::vector<RefEntry>, RefCmp> open;

    const size_t start_idx = grid.index(start);
    g[start_idx] = 0.0;
    open.push({ref_heuristic(start, goal, via, base), 0.0, start});

    size_t expanded = 0; bool done = false;
    while (!open.empty()) {
        RefEntry cur = open.top(); open.pop();
        const size_t cur_idx = grid.index(cur.cell);
        if (cur.g > g[cur_idx] + 1e-9) continue;
        if (cur.cell == goal) { done = true; break; }
        expanded++;
        if (expanded > 2000000) break;
        for (int d = 0; d < 8; ++d) {
            Cell nb{cur.cell.layer, cur.cell.x + DX[d], cur.cell.y + DY[d]};
            if (!grid.valid(nb)) continue;
            if (IS_DIAG[d]) {
                Cell ax{cur.cell.layer, cur.cell.x + DX[d], cur.cell.y};
                Cell ay{cur.cell.layer, cur.cell.x, cur.cell.y + DY[d]};
                if (enter(ax) < 0 || enter(ay) < 0) continue;
            }
            double step = enter(nb);
            if (step < 0) continue;
            if (IS_DIAG[d]) step *= DIAG_FACTOR;
            double ng = cur.g + step;
            size_t nb_idx = grid.index(nb);
            if (ng < g[nb_idx] - 1e-9) {
                g[nb_idx] = ng; parent[nb_idx] = (int)cur_idx;
                open.push({ng + ref_heuristic(nb, goal, via, base), ng, nb});
            }
        }
        for (int dl : {-1, 1}) {
            int nl = cur.cell.layer + dl;
            if (nl < 0 || nl >= grid.layers()) continue;
            Cell nb{nl, cur.cell.x, cur.cell.y};
            double ec = enter(nb);
            if (ec < 0) continue; // blocked: check before adding via cost (sentinel)
            double ng = cur.g + via + ec;
            size_t nb_idx = grid.index(nb);
            if (ng < g[nb_idx] - 1e-9) {
                g[nb_idx] = ng; parent[nb_idx] = (int)cur_idx;
                open.push({ng + ref_heuristic(nb, goal, via, base), ng, nb});
            }
        }
    }
    if (!done) { r.nodes_expanded = expanded; return r; }
    std::vector<Cell> path;
    size_t ci = grid.index(goal);
    while (true) {
        path.push_back(grid.unindex(ci));
        if (ci == start_idx) break;
        int p = parent[ci]; if (p < 0) break;
        ci = (size_t)p;
    }
    std::reverse(path.begin(), path.end());
    r.found = true; r.path = std::move(path); r.cost = g[grid.index(goal)];
    r.nodes_expanded = expanded;
    return r;
}

// Build a small random grid with a handful of random obstacles, then a constant-cost
// enter function (uniform, no avoidance). Deterministic per seed.
struct Case { Grid grid; Cell start; Cell goal; double via; double base; };

Case make_case(int seed, int layers, int w, int h, bool obstacles, bool multi_via) {
    std::mt19937 rng((unsigned)seed);
    Grid grid(layers, w, h, 1.0);
    if (obstacles) {
        int ob = 6;
        for (int i = 0; i < ob; ++i) {
            int x = (int)(rng() % (unsigned)w);
            int y = (int)(rng() % (unsigned)h);
            int l = (int)(rng() % (unsigned)layers);
            if (grid.valid(l, x, y)) grid.at(l, x, y) = -1.0;  // blocked marker
        }
    }
    auto pick = [&](int lo, int hi) { return (int)(lo + rng() % (unsigned)(hi - lo)); };
    Cell start{pick(0, layers), pick(1, w - 2), pick(1, h - 2)};
    Cell goal{pick(0, layers), pick(1, w - 2), pick(1, h - 2)};
    double via = multi_via ? 5.0 : 1000.0;  // 1000 => stay on one layer
    return Case{std::move(grid), start, goal, via, 1.0};
}

} // namespace

TEST_CASE("astar_matches_reference") {
    // Varied cases: obstacles on/off, single/multi-layer, with/without cheap vias.
    for (int seed = 0; seed < 40; ++seed) {
        Case c = make_case(seed, /*layers*/ (seed % 3 == 0 ? 3 : 2),
                           /*w*/ 12 + (seed % 5), /*h*/ 12 + (seed % 4),
                           /*obstacles*/ (seed % 2 == 0), /*multi_via*/ (seed % 4 != 1));
        EnterCostFn enter = [&](const Cell& cell) -> double {
            if (!c.grid.valid(cell)) return -1.0;
            double marker = c.grid.at(cell);
            return marker < 0.0 ? -1.0 : 1.0;
        };
        AStarResult got = astar_route(c.grid, c.start, c.goal, c.via, c.base, enter);
        AStarResult ref = reference_astar(c.grid, c.start, c.goal, c.via, c.base, enter);
        CHECK(got.found == ref.found);
        if (got.found != ref.found) continue;
        CHECK(got.cost == doctest::Approx(ref.cost).epsilon(1e-12));
        CHECK(got.path == ref.path);  // bit-for-bit
    }
}

TEST_CASE("astar_repeated_search_deterministic") {
    // Run MANY successive searches on the same board/thread with different sources, goals,
    // and orderings. Any dependence of one search on the state of a prior one (stale
    // g/parent) would surface as a path that differs from the fresh reference. We
    // interleave very short, very long, and unroutable searches to exercise reuse.
    Grid grid(3, 16, 16, 1.0);
    // hard obstacles in the middle to force detours / block some paths
    for (int x = 5; x <= 10; ++x) { grid.at(0, x, 8) = -1.0; grid.at(1, x, 8) = -1.0; }
    auto enter = [&](const Cell& cell) -> double {
        if (!grid.valid(cell)) return -1.0;
        return grid.at(cell) < 0.0 ? -1.0 : 1.0;
    };
    struct P { Cell a; Cell b; } pairs[] = {
        {{0, 1, 1}, {0, 14, 1}},
        {{1, 15, 1}, {1, 1, 2}},
        {{2, 1, 15}, {2, 14, 15}},
        {{0, 12, 8}, {0, 1, 8}},   // blocked straight through => detour
        {{0, 7, 8}, {0, 8, 8}},    // both in obstacle line => unroutable
        {{2, 14, 14}, {2, 1, 1}},
        {{0, 1, 1}, {0, 1, 1}},    // same-cell
    };
    for (int rep = 0; rep < 5; ++rep) {           // repeat to hammer strictly-sequential determinism
        for (auto& p : pairs) {
            AStarResult got = astar_route(grid, p.a, p.b, 5.0, 1.0, enter);
            AStarResult ref = reference_astar(grid, p.a, p.b, 5.0, 1.0, enter);
            CHECK(got.found == ref.found);
            CHECK(got.cost == doctest::Approx(ref.cost).epsilon(1e-12));
            CHECK(got.path == ref.path);
        }
    }
}

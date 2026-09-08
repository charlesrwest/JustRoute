#include "routing/astar.hpp"
#include "routing/compat.hpp"
#include "routing/cost_model.hpp"

#include <queue>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <atomic>

namespace routing {

namespace {

// 8-connectivity within a layer: 4 cardinal + 4 diagonal.
constexpr int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
// Whether a neighbor offset is a diagonal move (both dx and dy nonzero).
constexpr bool IS_DIAG[8] = {false, false, false, false, true, true, true, true};

struct PQEntry {
    double f;      // g + h
    double g;      // cost so far
    Cell cell;
};

struct GreaterByF {
    bool operator()(const PQEntry& a, const PQEntry& b) const { return a.f > b.f; }
};

// Consistent, admissible heuristic for multi-layer grid A* with 8-connectivity
// (cardinal=1, diagonal=sqrt(2)) and per-layer-via transitions.
//   octile = (max-min)*1 + min*sqrt(2)   [min number of diagonal moves]
//   h = octile * base_cost + via_cost*|layer - goal.layer|
double heuristic(const Cell& c, const Cell& goal, double via_cost, double base_cost) {
    double dx = std::abs(c.x - goal.x);
    double dy = std::abs(c.y - goal.y);
    double mn = std::min(dx, dy);
    double mx = std::max(dx, dy);
    double octile = (mx - mn) + mn * DIAG_FACTOR;
    double h_xy = octile * base_cost;
    double h_layer = via_cost * std::abs(c.layer - goal.layer);
    return h_xy + h_layer;
}

// Reusable per-thread search buffers with epoch-stamped lazy reset: a fresh
// A* pass costs O(1) setup instead of allocating and infinity-filling two
// O(grid) vectors per call (tens of MB on fine-resolution boards). A slot is
// normalized to its init value on first touch of the current epoch, so the
// views below behave EXACTLY like freshly-initialized vectors.
struct StampedScratch {
    std::vector<double> g;
    std::vector<uint32_t> gs;
    std::vector<int> parent;
    std::vector<uint32_t> ps;
    uint32_t ep = 0;
    void begin(size_t n) {
        if (g.size() < n) {
            g.resize(n);
            gs.assign(n, 0);
            parent.resize(n);
            ps.assign(n, 0);
            ep = 0;
        }
        if (++ep == 0) {   // epoch wrap: clear stamps once every 2^32 passes
            std::fill(gs.begin(), gs.end(), 0);
            std::fill(ps.begin(), ps.end(), 0);
            ep = 1;
        }
    }
    double& G(size_t i) {
        if (gs[i] != ep) { gs[i] = ep; g[i] = std::numeric_limits<double>::infinity(); }
        return g[i];
    }
    int& P(size_t i) {
        if (ps[i] != ep) { ps[i] = ep; parent[i] = -1; }
        return parent[i];
    }
};
static thread_local StampedScratch tls_scratch;

// Early-UNSAT probe scratch (optimization plan C2): visited stamps + frontier.
// The frontier is a min-heap on an integer octile distance-to-target-bbox when
// a bbox is supplied (greedy best-first: routable probes beeline like A*), or
// FIFO order without one. Verdicts are order-independent either way.
struct FloodScratch {
    std::vector<uint32_t> vis;
    uint32_t ep = 0;
    std::vector<std::pair<int, int>> heap;   // (h, cell index)
    void begin(size_t n) {
        if (vis.size() < n) { vis.assign(n, 0); ep = 0; }
        if (++ep == 0) { std::fill(vis.begin(), vis.end(), 0); ep = 1; }
        heap.clear();
    }
};
static thread_local FloodScratch tls_flood;
std::atomic<bool> g_unsat_check{true};

// Monotone radix heap over int64 keys (M2): for Dijkstra/A* with consistent
// heuristics the popped key sequence is non-decreasing, so buckets indexed by
// the highest bit differing from the last popped minimum give amortized-O(1)
// push/pop with no span limit (quantized costs make keys exact integers).
struct RadixHeap {
    std::vector<std::pair<int64_t, int32_t>> bucket[65];
    int64_t last = 0;
    size_t count = 0;
    void reset() {
        for (auto& b : bucket) b.clear();
        last = 0;
        count = 0;
    }
    static int idx_of(int64_t key, int64_t last) {
        const uint64_t x = (uint64_t)key ^ (uint64_t)last;
        return x ? 64 - rt_clzll(x) : 0;
    }
    void push(int64_t key, int32_t v) {
        bucket[idx_of(key, last)].emplace_back(key, v);
        ++count;
    }
    std::pair<int64_t, int32_t> pop() {
        if (bucket[0].empty()) {
            int b = 1;
            while (bucket[b].empty()) ++b;
            int64_t mn = bucket[b][0].first;
            for (auto& e : bucket[b]) mn = std::min(mn, e.first);
            last = mn;
            for (auto& e : bucket[b])
                bucket[idx_of(e.first, last)].push_back(e);
            bucket[b].clear();
        }
        auto out = bucket[0].back();
        bucket[0].pop_back();
        --count;
        return out;
    }
    bool empty() const { return count == 0; }
};
static thread_local RadixHeap tls_radix;

struct GView {
    StampedScratch& s;
    double& operator[](size_t i) const { return s.G(i); }
};
struct PView {
    StampedScratch& s;
    int& operator[](size_t i) const { return s.P(i); }
};

} // namespace

void set_unsat_check_enabled(bool on) {
    g_unsat_check.store(on, std::memory_order_relaxed);
}
bool unsat_check_enabled() {
    return g_unsat_check.load(std::memory_order_relaxed);
}

// Early-UNSAT reachability flood (plan Stage C2). Seeds from the unconnected
// pins and BFS-floods toward the component using a strict SUPERSET of A*'s
// move rules (target-touch tested before any gate; via transitions allowed if
// EITHER endpoint passes via_ok — forward search gates the destination only).
// Therefore: flood says "unreachable" => the weighted search would provably
// exhaust and fail; flood says "reachable" may still fail (search decides).
// Cost shape: routable nets early-exit at ~distance^2 cells; sealed pins flood
// only their pocket — both are tiny next to a failed exhaustive A*.
bool flood_goals_reach_sources(const Grid& grid,
                               const std::vector<Cell>& seeds,
                               const ComponentGoalFn& is_target,
                               const EnterCostFn& enter_cost,
                               const ViaEligibilityFn& via_ok,
                               const int* target_bbox) {
    // NOTE: no distance gate. A near-the-component pin can still own a huge
    // sealed pocket (pocket size, not pin distance, is what a failed search
    // costs) — a bbox-distance gate was measured to destroy the sealed-board
    // win (9.4x -> 1.4x on AS5043) while saving only ~1ms/net on easy boards.
    auto& S = tls_flood;
    S.begin(grid.size());
    // Integer octile distance from (x,y) to the target bbox; 0 without a bbox
    // (degrades to insertion order — verdict identical, just undirected).
    auto hdist = [&](int x, int y) -> int {
        if (!target_bbox) return 0;
        int dx = x < target_bbox[0] ? target_bbox[0] - x
                 : (x > target_bbox[2] ? x - target_bbox[2] : 0);
        int dy = y < target_bbox[1] ? target_bbox[1] - y
                 : (y > target_bbox[3] ? y - target_bbox[3] : 0);
        int mn = dx < dy ? dx : dy;
        int mx = dx < dy ? dy : dx;
        return 2 * mx + mn;                 // ~octile in half-units, ints only
    };
    auto cmp = [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return a.first > b.first;
    };
    auto push = [&](size_t idx, int h) {
        S.heap.emplace_back(h, (int)idx);
        std::push_heap(S.heap.begin(), S.heap.end(), cmp);
    };
    for (const Cell& c : seeds) {
        if (!grid.valid(c)) continue;
        if (is_target(c)) return true;
        size_t i = grid.index(c);
        if (S.vis[i] == S.ep) continue;
        S.vis[i] = S.ep;
        push(i, hdist(c.x, c.y));
    }
    while (!S.heap.empty()) {
        std::pop_heap(S.heap.begin(), S.heap.end(), cmp);
        const size_t cur_idx = (size_t)S.heap.back().second;
        S.heap.pop_back();
        const Cell cur = grid.unindex(cur_idx);
        const int l = cur.layer, cx = cur.x, cy = cur.y;
        for (int d = 0; d < 8; ++d) {
            Cell nb{l, cx + DX[d], cy + DY[d]};
            if (!grid.valid(nb)) continue;
            size_t ni = grid.index(nb);
            if (S.vis[ni] == S.ep) continue;
            if (is_target(nb)) return true;          // superset: no gates on touch
            if (IS_DIAG[d]) {
                Cell ax{l, cx + DX[d], cy};
                Cell ay{l, cx, cy + DY[d]};
                if (enter_cost(ax) < 0 || enter_cost(ay) < 0) continue;
            }
            if (enter_cost(nb) < 0) continue;
            S.vis[ni] = S.ep;
            push(ni, hdist(nb.x, nb.y));
        }
        for (int dl : {-1, 1}) {
            int nl = l + dl;
            if (nl < 0 || nl >= grid.layers()) continue;
            Cell nb{nl, cx, cy};
            size_t ni = grid.index(nb);
            if (S.vis[ni] == S.ep) continue;
            if (is_target(nb)) return true;
            // EXACT backward mirror of the forward via edge nb->cur, which A*
            // gates on via_ok(cur) && enter(cur): enter(cur) held when the
            // flood arrived at cur, so gate via_ok(cur) here plus enter(nb)
            // below. (The earlier either-endpoint superset let the flood escape
            // sealed pockets through vias A* forbids, then the search exhausted
            // anyway — paying flood AND exhaust.)
            if (via_ok && !via_ok(cur)) continue;
            if (enter_cost(nb) < 0) continue;
            S.vis[ni] = S.ep;
            push(ni, hdist(nb.x, nb.y));
        }
    }
    return false;
}

double path_euclidean_length(const std::vector<Cell>& path) {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        const Cell& a = path[i - 1];
        const Cell& b = path[i];
        if (a.layer != b.layer) continue;   // via transition: 0 horizontal length
        int dx = std::abs(b.x - a.x);
        int dy = std::abs(b.y - a.y);
        if (dx == 1 && dy == 1) {
            len += DIAG_FACTOR;             // diagonal move
        } else if (dx + dy >= 1) {
            len += 1.0;                     // cardinal move
        }
        // dx==0 && dy==0: self-loop, contributes nothing
    }
    return len;
}

AStarResult astar_route(const Grid& grid,
                        const Cell& start,
                        const Cell& goal,
                        double via_cost,
                        double base_cost,
                        const EnterCostFn& enter_cost,
                        const ViaEligibilityFn& via_ok,
                        RouteDeadline deadline) {
    AStarResult result;

    if (!grid.valid(start) || !grid.valid(goal)) {
        return result;
    }
    if (start == goal) {
        result.found = true;
        result.path = {start};
        result.cost = 0.0;
        return result;
    }

    // Bookkeeping arrays indexed by linear cell index.
    const size_t n = grid.size();
    tls_scratch.begin(n);
    GView g{tls_scratch};             // acts as vector<double>(n, inf)
    PView parent{tls_scratch};        // acts as vector<int>(n, -1)

    std::priority_queue<PQEntry, std::vector<PQEntry>, GreaterByF> open;

    // base_cost is the true global per-step floor (from CostParams.base_cost).
    // The heuristic must use this floor so it stays admissible even when the
    // start cell or other cells have elevated entry cost from avoidance terms.
    const size_t start_idx = grid.index(start);
    g[start_idx] = 0.0;
    open.push({heuristic(start, goal, via_cost, base_cost), 0.0, start});

    size_t expanded = 0;
    bool done = false;

    while (!open.empty()) {
        PQEntry cur = open.top();
        open.pop();
        const size_t cur_idx = grid.index(cur.cell);

        // Lazy deletion: skip stale entries.
        if (cur.g > g[cur_idx] + 1e-9) {
            continue;
        }

        if (cur.cell == goal) {
            done = true;
            break;
        }
        expanded++;
        if (expanded > 2000000) {
            break; // safety bound
        }
        if ((expanded & 1023) == 0 && deadline_passed(deadline)) break; // wall cap

        const int l = cur.cell.layer;
        const int cx = cur.cell.x;
        const int cy = cur.cell.y;

        for (int d = 0; d < 8; ++d) {
            Cell nb{l, cx + DX[d], cy + DY[d]};
            if (!grid.valid(nb)) continue;

            if (IS_DIAG[d]) {
                // Corner-cut guard: a diagonal move into a corner is only allowed
                // if the two orthogonally adjacent cells are not blocked.
                Cell ax{l, cx + DX[d], cy};
                Cell ay{l, cx, cy + DY[d]};
                if (enter_cost(ax) < 0 || enter_cost(ay) < 0) continue;
            }

            double step = enter_cost(nb);
            if (step < 0) continue; // blocked
            // Apply the physical move weight: diagonals cost sqrt(2) x cardinal.
            if (IS_DIAG[d]) step *= DIAG_FACTOR;

            double ng = cur.g + step;
            size_t nb_idx = grid.index(nb);
            if (ng < g[nb_idx] - 1e-9) {
                g[nb_idx] = ng;
                parent[nb_idx] = static_cast<int>(cur_idx);
                double f = ng + heuristic(nb, goal, via_cost, base_cost);
                open.push({f, ng, nb});
            }
        }

        // Via transitions to adjacent layers (±1).
        for (int dl : {-1, 1}) {
            int nl = cur.cell.layer + dl;
            if (nl < 0 || nl >= grid.layers()) continue;
            Cell nb{nl, cx, cy};
            if (via_ok && !via_ok(nb)) continue;   // Stage 8.4 via-emergence keepout
            // entering the new layer's cell costs via_cost + its entry cost
            double ec = enter_cost(nb);
            if (ec < 0) continue; // blocked: check before adding via_cost, which would mask the sentinel
            double step = via_cost + ec;
            double ng = cur.g + step;
            size_t nb_idx = grid.index(nb);
            if (ng < g[nb_idx] - 1e-9) {
                g[nb_idx] = ng;
                parent[nb_idx] = static_cast<int>(cur_idx);
                double f = ng + heuristic(nb, goal, via_cost, base_cost);
                open.push({f, ng, nb});
            }
        }
    }

    if (!done) {
        result.nodes_expanded = expanded;
        return result; // not found
    }

    // Reconstruct path.
    std::vector<Cell> path;
    size_t cur_idx = grid.index(goal);
    while (true) {
        Cell c = grid.unindex(cur_idx);
        path.push_back(c);
        if (cur_idx == start_idx) break;
        int p = parent[cur_idx];
        if (p < 0) break;
        cur_idx = static_cast<size_t>(p);
    }
    std::reverse(path.begin(), path.end());

    result.found = true;
    result.path = std::move(path);
    result.cost = g[grid.index(goal)];
    result.nodes_expanded = expanded;
    return result;
}

AStarResult astar_route_multi(const Grid& grid,
                              const std::vector<Cell>& sources_in,
                              const std::vector<Cell>& goals_in,
                              double via_cost,
                              double base_cost,
                              const EnterCostFn& enter_cost,
                              const ViaEligibilityFn& via_ok,
                              RouteDeadline deadline) {
    AStarResult result;

    // Dedup + validate sources and goals on the grid.
    std::vector<size_t> sources;
    {
        std::vector<bool> seen(grid.size(), false);
        for (const Cell& c : sources_in) {
            if (!grid.valid(c)) continue;
            size_t idx = grid.index(c);
            if (!seen[idx]) { seen[idx] = true; sources.push_back(idx); }
        }
    }
    std::vector<size_t> goals;
    {
        std::vector<bool> seen(grid.size(), false);
        for (const Cell& c : goals_in) {
            if (!grid.valid(c)) continue;
            size_t idx = grid.index(c);
            if (!seen[idx]) { seen[idx] = true; goals.push_back(idx); }
        }
    }
    if (sources.empty() || goals.empty()) return result;

    // Membership in the goal set by linear index (goals are few; linear scan is fine).
    auto is_goal = [&](size_t idx) {
        for (size_t gd : goals) if (gd == idx) return true;
        return false;
    };
    // Defensive: if a source cell is also a goal, it is already connected at 0 cost.
    // (Unreachable via route_net_tree, which keeps sources and goals disjoint, but this
    // hardens the public API against a self-connecting caller.)
    for (size_t s : sources) {
        if (is_goal(s)) {
            result.found = true;
            result.path = {grid.unindex(s)};
            result.cost = 0.0;
            return result;
        }
    }

    const size_t n = grid.size();
    tls_scratch.begin(n);
    GView g{tls_scratch};             // acts as vector<double>(n, inf)
    PView parent{tls_scratch};        // acts as vector<int>(n, -1)

    std::priority_queue<PQEntry, std::vector<PQEntry>, GreaterByF> open;

    auto h_to_goals = [&](const Cell& c) -> double {
        double best = std::numeric_limits<double>::infinity();
        for (size_t gd : goals) {
            double h = heuristic(c, grid.unindex(gd), via_cost, base_cost);
            if (h < best) best = h;
        }
        return best;
    };

    // Seed open with every source cell at g=0.
    for (size_t s : sources) {
        g[s] = 0.0;
        open.push({h_to_goals(grid.unindex(s)), 0.0, grid.unindex(s)});
    }

    size_t expanded = 0;
    bool done = false;
    size_t goal_idx = SIZE_MAX;

    while (!open.empty()) {
        PQEntry cur = open.top();
        open.pop();
        const size_t cur_idx = grid.index(cur.cell);
        if (cur.g > g[cur_idx] + 1e-9) continue;   // lazy deletion
        if (is_goal(cur_idx)) { done = true; goal_idx = cur_idx; break; }
        expanded++;
        if (expanded > 2000000) break;
        if ((expanded & 1023) == 0 && deadline_passed(deadline)) break; // wall cap

        const int l = cur.cell.layer, cx = cur.cell.x, cy = cur.cell.y;
        for (int d = 0; d < 8; ++d) {
            Cell nb{l, cx + DX[d], cy + DY[d]};
            if (!grid.valid(nb)) continue;
            if (IS_DIAG[d]) {
                Cell ax{l, cx + DX[d], cy}, ay{l, cx, cy + DY[d]};
                if (enter_cost(ax) < 0 || enter_cost(ay) < 0) continue;
            }
            double step = enter_cost(nb);
            if (step < 0) continue;
            if (IS_DIAG[d]) step *= DIAG_FACTOR;
            double ng = cur.g + step;
            size_t nb_idx = grid.index(nb);
            if (ng < g[nb_idx] - 1e-9) {
                g[nb_idx] = ng; parent[nb_idx] = (int)cur_idx;
                open.push({ng + h_to_goals(nb), ng, nb});
            }
        }
        for (int dl : {-1, 1}) {
            int nl = cur.cell.layer + dl;
            if (nl < 0 || nl >= grid.layers()) continue;
            Cell nb{nl, cx, cy};
            if (via_ok && !via_ok(nb)) continue;   // Stage 8.4 via-emergence keepout
            double ec = enter_cost(nb);
            if (ec < 0) continue; // blocked: check before adding via_cost, which would mask the sentinel
            double ng = cur.g + via_cost + ec;
            size_t nb_idx = grid.index(nb);
            if (ng < g[nb_idx] - 1e-9) {
                g[nb_idx] = ng; parent[nb_idx] = (int)cur_idx;
                open.push({ng + h_to_goals(nb), ng, nb});
            }
        }
    }

    if (!done) { result.nodes_expanded = expanded; return result; }

    // Reconstruct path from the reached goal back to a source (parent == -1).
    std::vector<Cell> path;
    size_t cur_idx = goal_idx;
    while (true) {
        path.push_back(grid.unindex(cur_idx));
        if (parent[cur_idx] == -1) break;
        cur_idx = (size_t)parent[cur_idx];
    }
    std::reverse(path.begin(), path.end());

    result.found = true;
    result.path = std::move(path);
    result.cost = g[goal_idx];
    result.nodes_expanded = expanded;
    return result;
}

AStarResult astar_route_tree(const Grid& grid,
                             const std::vector<Cell>& sources_in,
                             const ComponentGoalFn& in_component,
                             double via_cost,
                             double base_cost,
                             const EnterCostFn& enter_cost,
                             const ViaEligibilityFn& via_ok,
                             RouteDeadline deadline,
                             const uint16_t* wave_h,
                             size_t max_expansions,
                             const int* target_bbox) {
    AStarResult result;
    // Without a wave map this is h=0 (Dijkstra): with a predicate goal there is
    // no single octile target. With one, h = (wave-1)*base_cost is an
    // admissible, consistent, obstacle-aware bound (each move costs >= base),
    // and wave==0 cells are pruned (provably cannot reach the component).
    // h = max(octile-to-component-bbox, wave-distance) * base_cost: both are
    // admissible and consistent for the predicate goal (every component cell
    // lies inside the bbox; wave counts moves through real free space), and
    // the max of consistent heuristics is consistent. bbox guidance costs
    // nothing and applies to EVERY phase-1 search; the wave term joins on
    // escalation and additionally prunes provably-unreachable cells.
    auto WH = [&](size_t idx) -> double {
        double h = 0.0;
        if (target_bbox) {
            const Cell c = grid.unindex(idx);
            const int dx = c.x < target_bbox[0] ? target_bbox[0] - c.x
                           : (c.x > target_bbox[2] ? c.x - target_bbox[2] : 0);
            const int dy = c.y < target_bbox[1] ? target_bbox[1] - c.y
                           : (c.y > target_bbox[3] ? c.y - target_bbox[3] : 0);
            const int mn = dx < dy ? dx : dy;
            const int mx = dx < dy ? dy : dx;
            h = ((double)(mx - mn) + (double)mn * DIAG_FACTOR) * base_cost;
        }
        if (wave_h) {
            if (!wave_h[idx]) return -1.0;             // provably unreachable
            const double wh = (double)(wave_h[idx] - 1) * base_cost;
            if (wh > h) h = wh;
        }
        return h;
    };

    // Dedup + validate sources (the remaining unconnected pins).
    std::vector<size_t> sources;
    {
        std::vector<bool> seen(grid.size(), false);
        for (const Cell& c : sources_in) {
            if (!grid.valid(c)) continue;
            size_t idx = grid.index(c);
            if (!seen[idx]) { seen[idx] = true; sources.push_back(idx); }
        }
    }
    if (sources.empty()) return result;

    const size_t n = grid.size();
    tls_scratch.begin(n);
    GView g{tls_scratch};             // acts as vector<double>(n, inf)
    PView parent{tls_scratch};        // acts as vector<int>(n, -1)

    // binary heap replaced by tls_radix (monotone radix heap, M2)

    // Seed open with every source pin at g=0. h=0 (Dijkstra): with a predicate goal (the
    // whole component) the octile heuristic is not a single distance, and the searched ball
    // is small (pin-to-component), so bounding with h=0 stays optimal and simple.
    auto qz = [](double v) -> int64_t { return llround(v * 1024.0); };
    tls_radix.reset();
    for (size_t s : sources) {
        const double h0 = WH(s);
        if (h0 < 0.0) continue;                    // wave-unreachable pin
        g[s] = 0.0;
        tls_radix.push(qz(h0), (int32_t)s);
    }

    size_t expanded = 0;
    bool done = false;
    size_t goal_idx = SIZE_MAX;

    while (!tls_radix.empty()) {
        const auto [fq, ci32] = tls_radix.pop();
        const size_t cur_idx = (size_t)ci32;
        const double gc = g[cur_idx];
        const double hc = WH(cur_idx);
        if (hc < 0.0 || qz(gc + hc) != fq) continue;   // stale entry
        PQEntry cur{gc + hc, gc, grid.unindex(cur_idx)};
        if (in_component(cur.cell)) { done = true; goal_idx = cur_idx; break; }
        expanded++;
        if (max_expansions && expanded > max_expansions) {
            result.capped = true;
            break;
        }
        if (expanded > 2000000) break;
        if ((expanded & 1023) == 0 && deadline_passed(deadline)) break; // wall cap

        const int l = cur.cell.layer, cx = cur.cell.x, cy = cur.cell.y;
        for (int d = 0; d < 8; ++d) {
            Cell nb{l, cx + DX[d], cy + DY[d]};
            if (!grid.valid(nb)) continue;
            if (IS_DIAG[d]) {
                Cell ax{l, cx + DX[d], cy}, ay{l, cx, cy + DY[d]};
                if (enter_cost(ax) < 0 || enter_cost(ay) < 0) continue;
            }
            double step = enter_cost(nb);
            if (step < 0) continue;
            if (IS_DIAG[d]) step *= DIAG_FACTOR;
            double ng = cur.g + step;
            size_t nb_idx = grid.index(nb);
            if (ng < g[nb_idx] - 1e-9) {
                const double h = WH(nb_idx);
                if (h < 0.0) continue;             // wave-pruned
                g[nb_idx] = ng; parent[nb_idx] = (int)cur_idx;
                tls_radix.push(qz(ng + h), (int32_t)nb_idx);
            }
        }
        for (int dl : {-1, 1}) {
            int nl = cur.cell.layer + dl;
            if (nl < 0 || nl >= grid.layers()) continue;
            Cell nb{nl, cx, cy};
            if (via_ok && !via_ok(nb)) continue;   // via-emergence keepout
            double ec = enter_cost(nb);
            if (ec < 0) continue; // blocked: check before adding via_cost, which would mask the sentinel
            double ng = cur.g + via_cost + ec;
            size_t nb_idx = grid.index(nb);
            if (ng < g[nb_idx] - 1e-9) {
                const double h = WH(nb_idx);
                if (h < 0.0) continue;             // wave-pruned
                g[nb_idx] = ng; parent[nb_idx] = (int)cur_idx;
                tls_radix.push(qz(ng + h), (int32_t)nb_idx);
            }
        }
    }

    if (!done) { result.nodes_expanded = expanded; return result; }

    // Reconstruct path from the reached component cell back to a source pin (parent==-1),
    // so path[0] is the connected unconnected pin and path.back() is the component cell.
    std::vector<Cell> path;
    size_t cur_idx = goal_idx;
    while (true) {
        path.push_back(grid.unindex(cur_idx));
        if (parent[cur_idx] == -1) break;
        cur_idx = (size_t)parent[cur_idx];
    }
    std::reverse(path.begin(), path.end());

    result.found = true;
    result.path = std::move(path);
    result.cost = g[goal_idx];
    result.nodes_expanded = expanded;
    return result;
}

std::vector<uint64_t> dijkstra_field(int W, int H, int L,
                                     const std::vector<uint32_t>& cost,
                                     const std::vector<uint8_t>& via_ok,
                                     uint32_t via_q,
                                     const std::vector<int64_t>& seeds) {
    const size_t cells = (size_t)L * H * W;
    constexpr uint64_t INF = ~0ull;
    constexpr uint32_t BLOCK = 0xFFFFFFFFu;
    std::vector<uint64_t> d(cells, INF);
    using QE = std::pair<uint64_t, uint32_t>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
    for (int64_t s : seeds) {
        if (s < 0 || (size_t)s >= cells || cost[(size_t)s] == BLOCK) continue;
        d[(size_t)s] = 0;
        pq.push({0, (uint32_t)s});
    }
    const int plane = H * W;
    static const int DX[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    static const int DY[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    while (!pq.empty()) {
        auto [dc, i] = pq.top();
        pq.pop();
        if (dc != d[i]) continue;                    // stale entry
        const int l = (int)(i / (uint32_t)plane);
        const int rem = (int)(i % (uint32_t)plane);
        const int y = rem / W, x = rem % W;
        for (int k = 0; k < 8; ++k) {
            const int nx = x + DX[k], ny = y + DY[k];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            const size_t j = (size_t)l * plane + (size_t)ny * W + nx;
            const uint32_t c = cost[j];
            if (c == BLOCK) continue;
            const bool diag = k >= 4;
            if (diag) {                              // corner-cut rule
                const size_t ax = (size_t)l * plane + (size_t)y * W + nx;
                const size_t ay = (size_t)l * plane + (size_t)ny * W + x;
                if (cost[ax] == BLOCK || cost[ay] == BLOCK) continue;
            }
            const uint64_t nd = dc + (uint64_t)c * (diag ? 181ull : 128ull);
            if (nd < d[j]) {
                d[j] = nd;
                pq.push({nd, (uint32_t)j});
            }
        }
        for (int dl : {-1, 1}) {
            const int nl = l + dl;
            if (nl < 0 || nl >= L) continue;
            const size_t j = (size_t)nl * plane + rem;
            const uint32_t c = cost[j];
            if (c == BLOCK || !via_ok[j]) continue;
            const uint64_t nd = dc + ((uint64_t)via_q + c) * 128ull;
            if (nd < d[j]) {
                d[j] = nd;
                pq.push({nd, (uint32_t)j});
            }
        }
    }
    return d;
}

std::vector<int64_t> walk_field_descent(int W, int H, int L,
                                        const std::vector<uint32_t>& cost,
                                        const std::vector<uint8_t>& via_ok,
                                        uint32_t via_q,
                                        const std::vector<uint64_t>& dist,
                                        int64_t start) {
    constexpr uint64_t INF = ~0ull;
    constexpr uint32_t BLOCK = 0xFFFFFFFFu;
    const int plane = H * W;
    static const int DX[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    static const int DY[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    std::vector<int64_t> path;
    if (start < 0 || (size_t)start >= dist.size() || dist[(size_t)start] == INF)
        return path;
    size_t cur = (size_t)start;
    const size_t max_steps = dist.size() + 8;   // inconsistency guard
    while (path.size() < max_steps) {
        path.push_back((int64_t)cur);
        const uint64_t dc = dist[cur];
        if (dc == 0) return path;
        const int l = (int)(cur / (size_t)plane);
        const int rem = (int)(cur % (size_t)plane);
        const int y = rem / W, x = rem % W;
        // step INTO cur cost: straight cost[cur]*1024, diag cost[cur]*1448,
        // via (via_q+cost[cur])*1024 — predecessor n satisfies
        // dist[n] + step == dc exactly.
        const uint64_t cc = cost[cur];
        size_t next = SIZE_MAX;
        for (int k = 0; k < 8 && next == SIZE_MAX; ++k) {
            const int nx = x + DX[k], ny = y + DY[k];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            const size_t j = (size_t)l * plane + (size_t)ny * W + nx;
            if (dist[j] == INF) continue;
            const bool diag = k >= 4;
            if (diag) {
                const size_t ax = (size_t)l * plane + (size_t)y * W + nx;
                const size_t ay = (size_t)l * plane + (size_t)ny * W + x;
                if (cost[ax] == BLOCK || cost[ay] == BLOCK) continue;
            }
            if (dist[j] + cc * (diag ? 181ull : 128ull) == dc) next = j;
        }
        if (next == SIZE_MAX && via_ok[cur]) {
            for (int dl : {-1, 1}) {
                const int nl = l + dl;
                if (nl < 0 || nl >= L) continue;
                const size_t j = (size_t)nl * plane + rem;
                if (dist[j] == INF) continue;
                if (dist[j] + ((uint64_t)via_q + cc) * 128ull == dc) {
                    next = j;
                    break;
                }
            }
        }
        if (next == SIZE_MAX) return {};        // inconsistent field
        cur = next;
    }
    return {};
}

} // namespace routing

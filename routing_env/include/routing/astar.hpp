#pragma once

#include "routing/grid.hpp"
#include "routing/cost_model.hpp"

#include <atomic>
#include <vector>
#include <functional>
#include <cstddef>
#include <chrono>

namespace routing {

// Wall-clock deadline for a routing call. RouteDeadline::max() (the default) means no
// limit. The A* expansion loops poll it every 1024 expansions, so a single search on a
// huge grid is interruptible; an expired deadline makes the search report "not found"
// and the callers above it stop starting new work. NOTE: a wall-time budget makes
// results depend on machine speed and load — use it for harness/step caps, not for
// anything that must be reproducible bit-for-bit.
using RouteDeadline = std::chrono::steady_clock::time_point;

// Cooperative cancel: set from ANY thread (e.g. a GUI) to make every
// deadline check report expired — searches abort within one poll interval
// and route loops stop starting nets, leaving a valid partial result.
// route_all()/route_from() clear it on entry, so only the call in flight
// when the request lands is affected (callers gate new passes themselves).
inline std::atomic<bool>& route_stop_flag() {
    static std::atomic<bool> f{false};
    return f;
}

// Coarse progress for UIs: the route loops publish nets-done / nets-total
// for the pass in flight (route_all/route_from reset it on entry). Global
// like the stop flag — one routing job per consumer process; readers poll
// from any thread without touching the GIL-released routing thread.
inline std::atomic<int>& route_progress_done() {
    static std::atomic<int> v{0};
    return v;
}
inline std::atomic<int>& route_progress_total() {
    static std::atomic<int> v{0};
    return v;
}
inline bool deadline_passed(const RouteDeadline& d) {
    if (route_stop_flag().load(std::memory_order_relaxed)) return true;
    return d != RouteDeadline::max() && std::chrono::steady_clock::now() >= d;
}

struct AStarResult {
    bool capped = false;   // hit max_expansions without exhausting
    bool found = false;
    std::vector<Cell> path;   // ordered start..goal (start inclusive, goal inclusive)
    double cost = 0.0;
    size_t nodes_expanded = 0;
};

// Callback: returns the cost to ENTER cell `c` (base + avoidance terms).
// The caller (env) is responsible for exemption of the net being routed.
// A return value < 0 marks the cell BLOCKED (impassable).
using EnterCostFn = std::function<double(const Cell&)>;

// Callback: true iff `c` is a cell of the tree component we are growing toward (an
// O(1) membership test over the laid trace + connected pins).
using ComponentGoalFn = std::function<bool(const Cell&)>;

// Callback: returns false to forbid CREATING a via that enters `c` (the emergence
// cell on the other layer). Used to enforce a via-emergence keepout (Stage 8.4): a
// via may only be created where its keepout disc is free of other nets' conductors.
// An empty callback means "always allowed" (default).
using ViaEligibilityFn = std::function<bool(const Cell&)>;

// 8-connectivity multi-layer A* over a grid with 4 cardinal + 4 diagonal moves
// per layer and ±1-layer via transitions (cost = via_cost). `enter_cost` provides
// per-cell entry cost; a negative return marks the cell blocked.
// `start` and `goal` are inclusive in the returned path.
//
// Move costs:
//   cardinal step : enter_cost(neighbor) * 1.0
//   diagonal step : enter_cost(neighbor) * sqrt(2)   (corner-cut guarded)
//   via           : via_cost + enter_cost(neighbor)
// A diagonal move into (x+dx, y+dy) is only allowed when the two orthogonally
// adjacent cells (x+dx, y) and (x, y+dy) are not blocked, so the trace never cuts
// through a corner-adjacent obstacle.
//
// base_cost MUST be the true global per-step floor: every enter_cost(cell) >= base_cost
// and every via costs >= via_cost. The heuristic uses the octile distance times
// base_cost so that it stays admissible even when individual cells (including the
// start cell) have elevated entry cost due to avoidance terms. Compute it as
// CostParams.base_cost.
AStarResult astar_route(const Grid& grid,
                        const Cell& start,
                        const Cell& goal,
                        double via_cost,
                        double base_cost,
                        const EnterCostFn& enter_cost,
                        const ViaEligibilityFn& via_ok = ViaEligibilityFn(),
                        RouteDeadline deadline = RouteDeadline::max());

// Dynamic multi-source A* used for multi-pin (tree) routing. The open set is seeded with
// every cell in `sources` at g=0 (the already-connected component — pins + laid trace), and
// the search expands until it reaches ANY cell in `goals` (the unconnected pins). This is
// the correct primitive for growing a net toward its nearest unconnected pin (it naturally
// joins a pad that abuts the net's own existing trace).
//
// Returns the path from some component source cell to the reached goal cell (start on the
// component boundary, end at the goal). Admissible heuristic = min over goals of the octile+
// via heuristic, so it stays admissible with multiple targets.
AStarResult astar_route_multi(const Grid& grid,
                              const std::vector<Cell>& sources,
                              const std::vector<Cell>& goals,
                              double via_cost,
                              double base_cost,
                              const EnterCostFn& enter_cost,
                              const ViaEligibilityFn& via_ok = ViaEligibilityFn(),
                              RouteDeadline deadline = RouteDeadline::max());

// Reverse-from-pins tree growth primitive (Stage 9): multi-source A* seeded with `sources`
// (the remaining unconnected pins, few) that expands until it reaches ANY cell satisfying
// `in_component` (the existing trace/pins). This avoids re-seeding the whole (large) trace
// into the heap on every connection, and the searched balls are tiny for a pad that abuts
// its own trace. Returns the path from one source pin (path[0]) to the reached component
// cell (path.back()). Min-over-goals is not needed: `in_component` is a predicate, so the
// admissible/consistent octile+via heuristic still applies to whichever goal is hit first.
// wave_h: optional per-cell wave-index map (0 = unreachable, else wave+1) from
// a flood seeded at the component; h(c) = (wave_h[c]-1)*base_cost is
// admissible+consistent (every move costs >= base_cost), and wave_h[c]==0
// cells are pruned outright (provably cannot reach the component).
// max_expansions: 0 = unlimited; when hit, returns with .capped = true so the
// caller can escalate (build the wave map, re-search guided).
AStarResult astar_route_tree(const Grid& grid,
                             const std::vector<Cell>& sources,
                             const ComponentGoalFn& in_component,
                             double via_cost,
                             double base_cost,
                             const EnterCostFn& enter_cost,
                             const ViaEligibilityFn& via_ok = ViaEligibilityFn(),
                             RouteDeadline deadline = RouteDeadline::max(),
                             const uint16_t* wave_h = nullptr,
                             size_t max_expansions = 0,
                             const int* target_bbox = nullptr);

// Physical (Euclidean) length of a returned path: sum of 1.0 per cardinal move,
// sqrt(2) per diagonal move, and 0 for via transitions (layer changes), which are
// accounted separately as `total_vias`. This is the true geometric trace length,
// independent of avoidance-weighted `cost`.
double path_euclidean_length(const std::vector<Cell>& path);

// Early-UNSAT reachability flood (optimization plan C2): BFS from `seeds`
// toward `is_target` under a strict SUPERSET of A*'s move rules. Returns false
// only when the weighted search provably cannot connect (it would exhaust).
// Toggleable for equivalence testing; enabled by default.
// target_bbox = {x0, y0, x1, y1} over the target cells (any layer), used as a
// greedy best-first heuristic so ROUTABLE probes cost ~O(distance) like A*
// instead of an undirected O(distance^2) flood; UNSAT probes still exhaust
// only the sealed pocket. Pass nullptr for plain BFS order (same verdicts).
bool flood_goals_reach_sources(const Grid& grid,
                               const std::vector<Cell>& seeds,
                               const ComponentGoalFn& is_target,
                               const EnterCostFn& enter_cost,
                               const ViaEligibilityFn& via_ok = ViaEligibilityFn(),
                               const int* target_bbox = nullptr);
void set_unsat_check_enabled(bool on);
bool unsat_check_enabled();

// Exact integer Dijkstra field over an explicit cost grid (CPU reference for
// the GPU field router; see docs/project/gpu-field-router.md). cost is in
// COST_QUANTUM units per cell (0xFFFFFFFF = blocked), layout ((l*H)+y)*W+x.
// Returns per-cell distance in microquanta (COST_QUANTUM^2 units; ~0ull =
// unreached): straight steps cost[c]*1024, diagonals cost[c]*1448 with the
// corner-cut rule, vias (via_q + cost[c])*1024 into adjacent layers gated on
// via_ok[c] (dest-side rule) — the same edge model as astar_route_tree.
std::vector<uint64_t> dijkstra_field(int W, int H, int L,
                                     const std::vector<uint32_t>& cost,
                                     const std::vector<uint8_t>& via_ok,
                                     uint32_t via_q,
                                     const std::vector<int64_t>& seeds);

// Deterministic greedy descent from `start` to any dist==0 seed: at each cell
// pick the FIRST predecessor (fixed neighbor order: 8 planar then vias) whose
// distance + edge cost equals the current distance exactly. Returns cell
// indices from start down to a seed (inclusive); empty if start is unreached
// or the field is inconsistent. Path cost == dist[start] by construction.
std::vector<int64_t> walk_field_descent(int W, int H, int L,
                                        const std::vector<uint32_t>& cost,
                                        const std::vector<uint8_t>& via_ok,
                                        uint32_t via_q,
                                        const std::vector<uint64_t>& dist,
                                        int64_t start);

} // namespace routing

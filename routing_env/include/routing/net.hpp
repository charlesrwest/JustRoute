#pragma once

#include "routing/grid.hpp"
#include "routing/astar.hpp"

#include <vector>

namespace routing {

// A net: a set of pins (cells) that must all be electrically connected by one trace.
struct Net {
    int id = 0;
    std::vector<Cell> pins;        // all pin cells of this net
    std::vector<std::vector<Cell>> segments; // per-pair route segments (filled by routing)
    double total_length = 0.0;     // sum of segment lengths (in cell steps)
    bool routed = false;
    // Pins NOT yet connected to the net's seed component (pins[0] counts as connected).
    // Partial connection is FIRST-CLASS: a routing failure keeps the successful
    // connections in `segments` (they are committed as real copper) and records how
    // many pins remain, so 4-of-5 is distinguishable from 0-of-5. An unattempted
    // P-pin net has P-1. routed == (unconnected_pins == 0).
    int unconnected_pins = 0;
    // Connection-mode bookkeeping (maintained by Board::route_one_connection; empty in
    // net-mode routing). pin_connected[i]: pins[i] is joined to the seed component.
    // committed_cells: exact cells of every committed path, in commit order — the A*
    // source component for the next connection (identical to tree growth's component).
    std::vector<uint8_t> pin_connected;
    std::vector<Cell> committed_cells;
};

// Route a net's pins pairwise-sequentially using A*, appending segments to `net`.
// The `enter_cost` callback must return the entry cost for a cell; cells belonging
// to THIS net are exempted (so routing through one's own pads is free of self-avoid).
// `base_cost` is the global per-step floor (CostParams.base_cost) used for the
// admissible heuristic.
//
// Returns the net's total routed length (sum of per-pair path steps). If any pair
// cannot be routed, returns false and leaves `routed=false`.
bool route_net_pairwise(Grid& grid,
                        Net& net,
                        double via_cost,
                        double base_cost,
                        const EnterCostFn& enter_cost,
                        const ViaEligibilityFn& via_ok = ViaEligibilityFn(),
                        RouteDeadline deadline = RouteDeadline::max());

// Route a net's pins as a growing tree using dynamic multi-source A* (see astar_route_multi).
// The component starts at one pin and grows toward the nearest unconnected pin, allowing a
// pad to join an already-laid trace of the same net (the GND-pad-abutting-its-trace case).
// Returns true if all pins became connected; on any failure sets net.routed=false.
// Fills net.segments / net.total_length.
// Tree-growth strategy (Stage 9). FORWARD seeds A* with the whole component (default,
// exact, and fastest on small/medium boards). REVERSE seeds A* with the few unconnected
// pins (astar_route_tree) — dramatically faster on LARGE fine-grid boards with long
// traces (measured ~60x in the synthetic 300x300 case) but measurably slower on small
// boards (~5x on the 1Bitsy fixture). Results are tie-equivalent (same nearest-first
// greedy; ties-only divergence).
enum class TreeStrategy { forward, reverse };

// Forward tree growth (DEFAULT): seeds A* with the full component and expands to the
// nearest unconnected pin. This is the validated Stage-7 reference.
bool route_net_tree(Grid& grid,
                    Net& net,
                    double via_cost,
                    double base_cost,
                    const EnterCostFn& enter_cost,
                    const ViaEligibilityFn& via_ok = ViaEligibilityFn(),
                    RouteDeadline deadline = RouteDeadline::max());

// Reverse-from-pins tree growth (opt-in for large boards): see astar_route_tree.
// build_wave: optional Board-bound wave-map builder (seeds = component cells).
// When provided, each connection attempt runs with an expansion cap; a capped
// (not exhausted) search escalates: build the obstacle-aware wave map, then
// re-search guided by h = (wave-1)*base_cost (admissible & consistent) with
// wave-unreachable cells pruned. Optimal paths in the same cost model.
using WaveMapFn = std::function<const uint16_t*(const std::vector<Cell>&)>;
// Escalation threshold (expansions of uncapped-Dijkstra before building the
// wave map and re-searching guided). Runtime-tunable for calibration sweeps.
void set_escalation_cap(size_t cap);
size_t escalation_cap();
bool route_net_tree_reverse(Grid& grid,
                            Net& net,
                            double via_cost,
                            double base_cost,
                            const EnterCostFn& enter_cost,
                            const ViaEligibilityFn& via_ok = ViaEligibilityFn(),
                            RouteDeadline deadline = RouteDeadline::max(),
                            const WaveMapFn& build_wave = WaveMapFn(),
                            std::size_t expansion_budget = 0);

} // namespace routing

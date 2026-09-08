#include <climits>
#include <atomic>
#include "routing/net.hpp"

#include <limits>
#include <cstdlib>
#include <unordered_set>

namespace routing {

namespace { std::atomic<size_t> g_escalation_cap{100000}; }
void set_escalation_cap(size_t cap) {
    g_escalation_cap.store(cap ? cap : 100000, std::memory_order_relaxed);
}
size_t escalation_cap() {
    return g_escalation_cap.load(std::memory_order_relaxed);
}

bool route_net_pairwise(Grid& grid,
                        Net& net,
                        double via_cost,
                        double base_cost,
                        const EnterCostFn& enter_cost,
                        const ViaEligibilityFn& via_ok,
                        RouteDeadline deadline) {
    net.unconnected_pins = 0;
    if (net.pins.empty()) {
        net.routed = true;
        net.total_length = 0.0;
        return true;
    }
    if (net.pins.size() == 1) {
        net.routed = true;
        net.total_length = 0.0;
        return true;
    }

    net.segments.clear();
    net.total_length = 0.0;

    // Pairwise-sequential: connect pin[i] to pin[i+1] for i in [0..n-2]. A failure
    // KEEPS the connections made so far (partial nets are first-class: the committed
    // prefix is real copper) and records the pins left unconnected.
    for (size_t i = 0; i + 1 < net.pins.size(); ++i) {
        if (deadline_passed(deadline)) {
            net.unconnected_pins = (int)net.pins.size() - (int)(i + 1);
            net.routed = false;
            return false;
        }
        Cell a = net.pins[i];
        Cell b = net.pins[i + 1];
        if (a == b) {
            continue; // degenerate pair
        }
        auto res = astar_route(grid, a, b, via_cost, base_cost, enter_cost, via_ok, deadline);
        if (!res.found) {
            net.unconnected_pins = (int)net.pins.size() - (int)(i + 1);
            net.routed = false;
            return false;
        }
        // Physical (Euclidean) trace length: 1.0 per cardinal move, sqrt(2) per
        // diagonal move, 0 for via transitions.
        double seg_len = path_euclidean_length(res.path);
        net.total_length += seg_len;
        net.segments.push_back(std::move(res.path));
    }

    net.routed = true;
    return true;
}

bool route_net_tree(Grid& grid,
                            Net& net,
                            double via_cost,
                            double base_cost,
                            const EnterCostFn& enter_cost,
                            const ViaEligibilityFn& via_ok,
                            RouteDeadline deadline) {
    net.unconnected_pins = 0;
    if (net.pins.empty() || net.pins.size() == 1) {
        net.routed = true;
        net.total_length = 0.0;
        net.segments.clear();
        return true;
    }

    net.segments.clear();
    net.total_length = 0.0;

    // Component sources: all cells of the connected part (seeded pins + laid trace cells).
    std::vector<Cell> component;
    // Remaining unconnected pins (goal set).
    std::vector<Cell> remaining(net.pins.begin(), net.pins.end());

    // A helper to mark a set of cells (with dedup) as part of the component.
    auto add_component = [&](const std::vector<Cell>& cells) {
        for (const Cell& c : cells) component.push_back(c);
    };
    // Remove from `remaining` any pin cell that is already among the laid cells / seeded.
    // (Handles a trace crossing an unconnected pin, connecting it trivially.)
    auto prune_remaining = [&]() {
        size_t w = 0;
        for (size_t i = 0; i < remaining.size(); ++i) {
            bool covered = false;
            for (const Cell& c : component) {
                if (c == remaining[i]) { covered = true; break; }
            }
            if (!covered) remaining[w++] = remaining[i];
        }
        remaining.resize(w);
    };

    // Seed the component with the first pin and mark it connected.
    add_component({net.pins[0]});
    remaining.erase(remaining.begin());
    prune_remaining();

    int guard = 0;
    while (!remaining.empty()) {
        if (++guard > (int)net.pins.size() * 4 + 16) {
            net.unconnected_pins = (int)remaining.size();
            net.routed = false;
            return false; // safety (keep the connections already made)
        }
        if (deadline_passed(deadline)) {
            net.unconnected_pins = (int)remaining.size();
            net.routed = false;
            return false;
        }
        if (unsat_check_enabled()) {
            auto in_comp = [&](const Cell& c) -> bool {
                for (const Cell& cc : component)
                    if (cc == c) return true;
                return false;
            };
            int bb[4] = {INT_MAX, INT_MAX, INT_MIN, INT_MIN};
            for (const Cell& cc : component) {
                bb[0] = std::min(bb[0], cc.x); bb[1] = std::min(bb[1], cc.y);
                bb[2] = std::max(bb[2], cc.x); bb[3] = std::max(bb[3], cc.y);
            }
            if (!flood_goals_reach_sources(grid, remaining, in_comp,
                                           enter_cost, via_ok, bb)) {
                net.unconnected_pins = (int)remaining.size();
                net.routed = false;
                return false;
            }
        }
        auto res = astar_route_multi(grid, component, remaining,
                                     via_cost, base_cost, enter_cost, via_ok, deadline);
        if (!res.found) {
            // Partial nets are first-class: keep the successful connections (real
            // copper, committed by the caller) and record what remains.
            net.unconnected_pins = (int)remaining.size();
            net.routed = false;
            return false;
        }
        net.total_length += path_euclidean_length(res.path);
        net.segments.push_back(res.path);

        // Add the new trace cells to the component, then drop the pin we just reached
        // (the path endpoint is the reached goal pin) and re-prune.
        add_component(res.path);
        Cell reached = res.path.back();
        for (size_t i = 0; i < remaining.size(); ++i) {
            if (remaining[i] == reached) { remaining.erase(remaining.begin() + (long)i); break; }
        }
        prune_remaining();
    }

    net.routed = true;
    return true;
}

bool route_net_tree_reverse(Grid& grid,
                    Net& net,
                    double via_cost,
                    double base_cost,
                    const EnterCostFn& enter_cost,
                    const ViaEligibilityFn& via_ok,
                    RouteDeadline deadline,
                    const WaveMapFn& build_wave,
                    std::size_t expansion_budget) {
    net.unconnected_pins = 0;
    if (net.pins.empty() || net.pins.size() == 1) {
        net.routed = true;
        net.total_length = 0.0;
        net.segments.clear();
        return true;
    }

    net.segments.clear();
    net.total_length = 0.0;

    // Component membership (O(1)) over linear indices of connected pins + laid trace cells.
    std::unordered_set<size_t> component;
    std::vector<Cell> comp_cells;                 // wave-map seeds (escalation)
    int comp_bb[4] = {INT_MAX, INT_MAX, INT_MIN, INT_MIN};   // x0,y0,x1,y1
    auto add_component = [&](const Cell& c) {
        if (!grid.valid(c)) return;
        if (component.count(grid.index(c)) == 0) comp_cells.push_back(c);
        component.insert(grid.index(c));
        comp_bb[0] = std::min(comp_bb[0], c.x);
        comp_bb[1] = std::min(comp_bb[1], c.y);
        comp_bb[2] = std::max(comp_bb[2], c.x);
        comp_bb[3] = std::max(comp_bb[3], c.y);
    };
    auto in_component = [&](const Cell& c) -> bool {
        return grid.valid(c) && component.count(grid.index(c)) != 0;
    };

    // Seed the component with the first pin; the REST are the unconnected source pins.
    add_component(net.pins[0]);
    std::vector<Cell> remaining;
    for (const Cell& p : net.pins)
        if (!in_component(p)) remaining.push_back(p);

    int guard = 0;
    while (!remaining.empty()) {
        if (++guard > (int)net.pins.size() * 4 + 16) {
            net.unconnected_pins = (int)remaining.size();
            net.routed = false;
            return false; // safety (keep the connections already made)
        }
        if (deadline_passed(deadline)) {
            net.unconnected_pins = (int)remaining.size();
            net.routed = false;
            return false;
        }
        // Reverse growth: A* from the few unconnected pins toward the existing component.
        // Early-UNSAT (plan C2): if no remaining pin can even REACH the
        // component under the hard-block rules, the weighted search would
        // exhaust and fail — skip it and fail with identical bookkeeping.
        if (unsat_check_enabled() &&
            !flood_goals_reach_sources(grid, remaining, in_component,
                                       enter_cost, via_ok, comp_bb)) {
            net.unconnected_pins = (int)remaining.size();
            net.routed = false;
            return false;
        }
        // Two-phase: uncapped Dijkstra is fine for small searches; monsters hit
        // the cap and escalate to a wave-guided re-search (same optimal cost
        // model, obstacle-aware h, unreachable cells pruned).
        // Phase-1 cap: escalation threshold, tightened by the deterministic
        // expansion budget when one is set; the guided retry gets the full
        // remaining budget (capped-at-budget => deterministic failure).
        size_t cap1 = build_wave ? escalation_cap() : 0;
        if (expansion_budget && (cap1 == 0 || expansion_budget < cap1))
            cap1 = expansion_budget;
        auto res = astar_route_tree(grid, remaining, in_component,
                                    via_cost, base_cost, enter_cost, via_ok,
                                    deadline, nullptr, cap1, comp_bb);
        if (!res.found && res.capped && build_wave &&
            (!expansion_budget || cap1 < expansion_budget)) {
            const uint16_t* wmap = build_wave(comp_cells);
            res = astar_route_tree(grid, remaining, in_component,
                                   via_cost, base_cost, enter_cost, via_ok,
                                   deadline, wmap, expansion_budget, comp_bb);
        }
        if (!res.found) {
            net.unconnected_pins = (int)remaining.size();
            net.routed = false;
            return false;
        }
        net.total_length += path_euclidean_length(res.path);
        net.segments.push_back(res.path);
        // Add the new trace to the component, then drop every pin it now covers (the
        // connected pin is path[0]; the trace may also cross other remaining pins).
        for (const Cell& c : res.path) add_component(c);
        std::vector<Cell> keep;
        for (const Cell& p : remaining)
            if (!in_component(p)) keep.push_back(p);
        remaining = std::move(keep);
    }

    net.routed = true;
    return true;
}

} // namespace routing

#include "routing/env.hpp"

#include <algorithm>

namespace routing {

RoutingEnv::RoutingEnv(int layers, int width, int height, double resolution,
                       double via_cost, double base_cost, double design_rule_clearance)
    : board_(layers, width, height, resolution, via_cost, base_cost, design_rule_clearance) {}

// Physical/avoidance config (trace width, via radius, falloff radius/sharpness) are
// ENV-level constants, not per-board data: preserve them across a fresh load so they
// survive to the newly built board (otherwise set-before-load would be silently lost).
// trace/via widths are resolution-derived: transfer them as mm and re-derive cells for
// the destination grid (the placeholder board may be at a different resolution).
namespace {
void keep_physics(Board& dst, const Board& src) {
    // Trace/via physics transfer only when a caller EXPLICITLY configured them on the
    // env: a loaded board's own design rules (set_rule_physics) otherwise win over the
    // placeholder's defaults.
    if (src.physics_explicit()) {
        dst.set_trace_half_width_cells(src.trace_half_width_mm() / dst.grid().resolution());
        dst.set_via_radius_cells(src.via_radius_mm() / dst.grid().resolution());
    }
    dst.set_falloff_radius(src.falloff_radius());
    dst.set_pad_sharp(src.pad_sharp());
    dst.set_trace_sharp(src.trace_sharp());
}
}

int RoutingEnv::load_synthetic(const SyntheticSpec& spec) {
    Board nb = build_synthetic(spec);
    keep_physics(nb, board_);
    board_ = std::move(nb);
    last_ = RouteStats{};
    return board_.num_nets();
}

int RoutingEnv::load_pcb_rdl(const std::string& json, double resolution, PcbRdlInfo* info) {
    Board nb = routing::load_pcb_rdl(json, resolution, info);
    keep_physics(nb, board_);
    board_ = std::move(nb);
    last_ = RouteStats{};
    return board_.num_nets();
}

int RoutingEnv::load_kicad_pcb(const std::string& contents, double resolution,
                               KicadPcbInfo* info, bool skip_poured, int max_fanout) {
    Board nb = routing::load_kicad_pcb(contents, resolution, info, true,
                                       skip_poured, max_fanout);
    keep_physics(nb, board_);
    board_ = std::move(nb);
    last_ = RouteStats{};
    return board_.num_nets();
}

RouteStats RoutingEnv::reset_empty() {
    board_.clear_routing();
    last_ = board_.collect_stats();
    return last_;
}

RouteStats RoutingEnv::step_connect(int net_id, int pin_idx) {
    const auto& nets = board_.nets();
    int pos = -1;
    for (size_t i = 0; i < nets.size(); ++i)
        if (nets[i].id == net_id) { pos = (int)i; break; }
    if (pos >= 0) {
        auto r = board_.route_one_connection(pos, pin_idx);
        // No DRC in the step loop: with hard clearance the mid-episode DRC count is
        // exactly unrouted_count. The gym runs one verifying pass at episode end.
        last_ = board_.collect_stats(/*with_drc=*/false);
        last_.time_budget_exceeded = r.time_budget_exceeded;
    }
    return last_;
}

RouteStats RoutingEnv::reset() {
    last_ = board_.route_all();
    return last_;
}

int RoutingEnv::position_of(int net_id) const {
    const auto& nets = board_.nets();
    for (size_t i = 0; i < nets.size(); ++i)
        if (nets[i].id == net_id) return (int)i;
    return -1;
}

RouteStats RoutingEnv::step(int net_idx, int target_pos, double pad_avoid, double trace_avoid) {
    const int n = board_.num_nets();
    if (n == 0) { last_ = RouteStats{}; return last_; }

    // If the board was never routed, establish a valid baseline first so the [0,lo)
    // prefix is present before the incremental re-route (route_from's precondition).
    if (last_.per_net_length.empty()) {
        reset();
    }

    // Find the current array position of the net with the given stable id.
    int pos = -1;
    const auto& nets = board_.nets();
    for (int i = 0; i < n; ++i) if (nets[(size_t)i].id == net_idx) { pos = i; break; }
    if (pos < 0) { last_ = board_.route_all(); return last_; } // unknown net -> full route

    // Set the moved net's avoidance multipliers at its CURRENT position (avoid_ is reordered
    // in lockstep with nets_ by move_net, so set before moving).
    board_.set_avoidance((size_t)pos, pad_avoid, trace_avoid);

    int to = std::max(0, std::min(target_pos, n - 1));
    int actual = board_.move_net(pos, to);
    int lo = std::min(pos, actual);

    // Incremental re-route of only the affected span [lo, n); equals a full route by the
    // Stage-4 `incremental == full` property.
    last_ = board_.route_from(lo);
    return last_;
}

std::vector<double> RoutingEnv::get_observation() {
    // Ensure we have metrics to report.
    if (last_.per_net_length.empty() && board_.num_nets() > 0) {
        reset();
    }
    const int n = board_.num_nets();
    std::vector<double> obs;
    obs.reserve(2 * (size_t)n);
    for (int i = 0; i < n; ++i) {
        double len = (size_t)i < last_.per_net_length.size() ? last_.per_net_length[(size_t)i] : 0.0;
        obs.push_back(len);
    }
    for (int i = 0; i < n; ++i) {
        bool unr = (size_t)i < last_.unrouted.size() ? last_.unrouted[(size_t)i] : true;
        obs.push_back(unr ? 1.0 : 0.0);
    }
    return obs;
}

} // namespace routing

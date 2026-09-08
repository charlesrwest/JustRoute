#pragma once

#include "routing/board.hpp"
#include "routing/synthetic.hpp"
#include "routing/pcb_rdl.hpp"
#include "routing/kicad_pcb.hpp"

#include <memory>
#include <vector>
#include <string>

namespace routing {

// RL-facing environment wrapper around a single Board. Owns the board and exposes the
// sequential-decision RL loop as `step(net_idx, target_pos, pad_avoid, trace_avoid)`:
// set that net's avoidance multipliers, move it to a new order position, and incrementally
// re-route (only the affected span) to get the resulting per-net metrics.
//
// This is the "one seam out to Python": the decision-maker (RL/LLM/rule) never touches the
// router internals, only this API. Keep it small and stable.
class RoutingEnv {
public:
    // Construct an empty env with a Board of the given size (pads/nets added via loaders).
    RoutingEnv(int layers, int width, int height, double resolution,
               double via_cost, double base_cost, double design_rule_clearance);

    // Loaders: (re)build the board. Returns the number of nets.
    // `resolution` (board units/cell, mm) defaults to DEFAULT_RESOLUTION_MM (0.05); a too-
    // coarse resolution that collapses distinct pads throws (Stage-10 fidelity guard).
    int load_synthetic(const SyntheticSpec& spec);
    int load_pcb_rdl(const std::string& json, double resolution = DEFAULT_RESOLUTION_MM,
                     PcbRdlInfo* info = nullptr);
    int load_kicad_pcb(const std::string& contents, double resolution = DEFAULT_RESOLUTION_MM,
                       KicadPcbInfo* info = nullptr, bool skip_poured = false);

    // Route the whole board in its current net order (full pass). Returns the metrics.
    //
    // NOTE: this is a re-route of the CURRENT board state, so per-net avoidance
    // multipliers set by prior step() calls are retained (penalties persist across
    // reset). For a genuinely clean episode, call board().clear_avoidance() afterwards
    // (or before the first step). This design lets reset() double as the reference
    // full route for verifying the env's incremental step (incremental == full).
    RouteStats reset();

    // Connection-mode episode init: clear routing to bare pads (all nets unattempted;
    // 0/1-pin nets trivially routed). The connection-mode analog of reset().
    RouteStats reset_empty();

    // Connection-mode RL step: connect ONE pin of net `net_id` (its nearest unconnected
    // pin when pin_idx = -1, else that specific pin) to the net's committed component,
    // committing the path immediately. No reorder, no rip-up. Returns snapshot stats.
    RouteStats step_connect(int net_id, int pin_idx = -1);

    // RL step: set net[net_idx]'s avoidance multipliers, bump it to target_pos, then
    // incrementally re-route the affected span. Returns the full-board metrics after the move.
    // Requires the board to have been routed at least once (call reset() first, or this
    // falls back to a full route to establish a valid [0,lo) prefix).
    RouteStats step(int net_idx, int target_pos, double pad_avoid, double trace_avoid);

    // Compact observation for the agent: [per-net routed length... | per-net unrouted(0/1)...]
    // as a contiguous std::vector<double>. Shape is deterministic given the net count.
    // Non-const: lazily ensures the board has been routed at least once if never routed.
    std::vector<double> get_observation();

    // Current array position of the net with the given stable id in nets(), or -1 if
    // unknown. Lets the agent compute target_pos / deltas without scanning nets().
    int position_of(int net_id) const;

    // Convenient pass-throughs.
    Board& board() { return board_; }
    const Board& board() const { return board_; }
    int num_nets() const { return board_.num_nets(); }
    const std::vector<Net>& nets() const { return board_.nets(); }
    RouteStats last_stats() const { return last_; }

    // Best-state checkpoint: the Board IS the complete routing state, so a
    // copy is a restorable snapshot. Rip-up is non-monotone (measured: rk86
    // converges to 28 unrouted after visiting 13) — callers snapshot at each
    // new best and restore at budget end, making any episode an anytime
    // algorithm whose result is its best visited state, not its last.
    void save_checkpoint() { ckpt_ = std::make_shared<Board>(board_); }
    bool restore_checkpoint() {
        if (!ckpt_) return false;
        board_ = *ckpt_;
        return true;
    }
    bool has_checkpoint() const { return ckpt_ != nullptr; }

private:
    Board board_;
    RouteStats last_;
    std::shared_ptr<Board> ckpt_;
};

} // namespace routing

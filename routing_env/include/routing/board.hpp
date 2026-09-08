#pragma once

#include "routing/grid.hpp"
#include "routing/net.hpp"
#include "routing/cost_model.hpp"
#include "routing/astar.hpp"

#include <vector>
#include <string>
#include <cstdint>
#include <tuple>
#include <functional>
#include <stdexcept>

namespace routing {

// Physical-fidelity defaults (KiCad default netclass, netclass.cpp): these are ALWAYS
// on and cannot be disabled (setters reject <= 0). Units are millimetres.
constexpr double kKiCadClearanceMM = 0.2;        // copper clearance (DRC)
constexpr double kKiCadEdgeClearanceMM = 0.5;    // copper to board edge (Edge.Cuts)
constexpr double kKiCadTraceHalfWidthMM = 0.1;   // 0.2 mm full trace width
constexpr double kKiCadViaRadiusMM = 0.3;        // 0.6 mm outer via diameter

// Per-net avoidance multipliers. Set per net; applied when routing that net.
// Row-parallel wavefront control (single-board latency): 1 = serial (default,
// right for sweeps/training which parallelize across boards); >1 = split each
// wave step's rows across threads (interactive single-board use).
void set_wave_threads(int n);
int wave_threads();
// Stage-2 field router (opt-in, default off): route nets from exact GPU cost
// fields instead of A*. Serial-only in v1; falls back to A* per net when the
// backend is unavailable. Tie-breaks/topology differ from A* (Steiner MST) —
// outcome-panel gated before any epoch default.
void set_field_router(bool on);
bool field_router_enabled();
// Coarse-to-fine corridor stage (opt-in speed knob: Aleste 920s -> 800s,
// quality drifts chaotically per board — panel-adjudicated, never default).
void set_field_corridor(bool on);
bool field_corridor_enabled();
// Field-router phase telemetry (microseconds): [bake, field, labels, scan,
// walks, misc, unsat, total-unused]. Reset between measured runs.
std::vector<int64_t> field_phase_stats();
void field_phase_reset();

struct NetAvoidance {
    double pad_avoid_mult = 0.0;
    double trace_avoid_mult = 0.0;
};

// A pad's REAL copper footprint on one layer, in cell units with a fractional center.
// Keepouts are the exact Minkowski sum shape ⊕ disc(margin), computed by thresholding
// the analytic signed distance — no circumscribed-circle overreach, so fine-pitch
// escape corridors stay open exactly when they are physically legal.
struct PadShape {
    int32_t tok = 0;         // net token (net_idx + 1)
    int layer = 0;
    double cx = 0.0, cy = 0.0;       // center, CELLS (fractional)
    double half_w = 0.0, half_h = 0.0; // half extents, CELLS, in the pad-local frame
    double cos_r = 1.0, sin_r = 0.0;   // pad rotation
    bool oval = false;       // capsule (circle when half_w == half_h) vs rectangle
    // Via-only keepout (KiCad rule area with vias not_allowed): stamps ONLY the
    // via-placement mask, never copper keepout, and is invisible to copper DRC.
    bool via_only = false;
    // Signed distance from point (px, py) to the pad copper boundary (< 0 inside).
    double distance(double px, double py) const;
};

struct RouteStats {
    std::vector<double> per_net_length;   // length per net (in cell steps)
    std::vector<bool> unrouted;           // true if net could not be fully routed
    int drc_violations = 0;               // total DRC violations on final board
    int unrouted_count = 0;
    double total_length = 0.0;
    int total_vias = 0;
    // Partial connection is first-class: per net, the pins not yet connected to its
    // seed component (0 = fully routed; an unattempted P-pin net reports P-1). The
    // committed copper of partial nets IS on the board and counted in lengths/vias.
    std::vector<int> per_net_unconnected;
    int total_unconnected_pins = 0;
    // True iff the route call's wall-time budget expired mid-pass: nets routed before
    // the cutoff keep their (valid, committed) results; the rest report unrouted.
    bool time_budget_exceeded = false;
    // Resume checkpoint: the net POSITION where the budget cut the pass short, or -1 if
    // the pass ran to completion. The Board object itself is the checkpoint state —
    // committed prefix traces are still on the board — so calling route_from(resume_from)
    // (with a fresh or no budget) continues exactly where the pass stopped. By the
    // incremental contract, partial-pass + resume produces the same result as one
    // uninterrupted pass, which makes progress-based gating safe: route a slice, inspect
    // stats, then continue, reorder, or abandon.
    //
    // Gating policy note: route_from(resume_from) RETRIES the interrupted net from
    // scratch. If a single net needs more time than the slice budget, fixed slices +
    // retry make no progress (measured: 20 x 10s slices stuck on one net). On a
    // no-progress slice either grow the budget or call route_from(resume_from + 1) to
    // skip past the stuck net — skipping measured both faster AND equal-quality on a
    // corpus board (42/46 in 66s sliced vs 119s monolithic; the skipped nets were
    // exactly the unroutable ones).
    int resume_from = -1;
};

// A multi-layer PCB board model with sequential A* routing.
//
// Occupancy:
//   - pad_owner_  : permanent pad ownership per cell (0 = none)
//   - owner_      : full occupancy (pads + routed traces); 0 = free
//   - trace_owner_: routed-trace ownership (0 = none), reset each route_all
//
// Avoidance (incremental, as the user requested — cost grids are baked, not BFS):
//   - pad_avoid_cost_  : per-cell cost = sum over all placed pads of
//                         pad_sharp/(dist+1). Updated in add_pad.
//   - trace_avoid_cost_: per-cell cost = sum over all placed trace cells of
//                         trace_sharp/(dist+1). Updated as traces are routed.
//   When routing net k, the enter cost is
//       base + pad_mult*(pad_avoid_cost - self_pad_k)
//             + trace_mult*(trace_avoid_cost - self_trace_k)
//   where self_*_k subtracts net k's OWN contributions so the net being routed is
//   exempt from repelling itself (per the source/target exemption requirement).
class Board {
public:
    Board(int layers, int width, int height, double resolution,
          double via_cost, double base_cost, double design_rule_clearance);

    Grid& grid() { return grid_; }
    const Grid& grid() const { return grid_; }

    int num_nets() const { return static_cast<int>(nets_.size()); }
    std::vector<Net>& nets() { return nets_; }
    const std::vector<Net>& nets() const { return nets_; }

    double base_cost() const { return base_cost_; }
    double via_cost() const { return via_cost_; }
    // GPU field router stage 1: bake net k's enter-cost/via closures into an
    // explicit integer grid — (cost in q20 (2^-20) units, 0xFFFFFFFF =
    // blocked; via_ok 0/1; via_cost in q20 (2^-20) units). Full-grid layout
    // ((l*H)+y)*W+x; windowed form covers [x0,x1]x[y0,y1] (all layers) with
    // window-local layout ((l*wh)+wy)*ww+wx. OpenMP-parallel over rows (the
    // net context closures are thread-safe by the C3 design).
    // See docs/project/gpu-field-router.md.
    std::tuple<std::vector<uint32_t>, std::vector<uint8_t>, uint32_t>
    bake_net_cost_grid(int k);
    // fast_via replaces the closure's per-cell (2R+1)^2 scan with three exact
    // Chebyshev box dilations (token fixed per bake); false = closure path
    // (the equality-gate reference).
    std::tuple<std::vector<uint32_t>, std::vector<uint8_t>, uint32_t>
    bake_net_cost_window(int k, int x0, int y0, int x1, int y1,
                         bool fast_via = true,
                         const std::vector<uint8_t>* via_precomputed = nullptr);
    // Rip the committed traces of the given net positions (tokens, copper,
    // vias, segments, bookkeeping). Avoidance and cl_zone are rebuilt by the
    // caller when needed (route_from does its own sequencing).
    void rip_net_positions(const std::vector<int>& positions);

    // PathFinder-style windowed negotiation (McMurchie-Ebeling FPGA'95;
    // TritonRoute-shaped): rip the participant nets, then iteratively route
    // them over a shared-overlay cost field
    //     eff = (base + hist) * (1 + pres_fac * other_claims)
    // where claims are CLEARANCE-DILATED footprints (trace half-width +
    // clearance/2; via radius likewise), so a zero-sharing fixpoint is
    // geometrically DRC-clean within discretization. hist accrues on every
    // cell that stays shared each iteration and is never forgiven — this is
    // what resolves second-order congestion (a bystander net reroutes because
    // its cells became expensive, even though it never touched a conflict).
    // pres_fac escalates geometrically per iteration (gradual, per the
    // paper's warning that abrupt penalties reintroduce order dependence).
    // On convergence the negotiated paths are committed through the normal
    // commit_path physics and zones are rebuilt; on failure participants are
    // left ripped (caller restores its checkpoint).
    // Returns {converged, iterations, shared_cells_final, per-net routed}.
    struct NegotiateResult {
        bool converged = false;
        int iterations = 0;
        int shared_final = -1;
        std::vector<uint8_t> routed;
    };
    NegotiateResult negotiate_window(const std::vector<int>& positions,
                                     int x0, int y0, int x1, int y1,
                                     int max_iters = 40,
                                     double pres_init = 0.5,
                                     double pres_mult = 1.7,
                                     double hist_gain = 0.4);

    // Route net k RESTRICTED to a corridor (cells dilated by radius): bake
    // the hard world, BLOCK everything outside the dilated corridor, route
    // per connection with the field oracle, commit through commit_path.
    // Serves the exact-endgame handoff: lattice corridors are mutually
    // pitch-separated by construction, so sequential corridor commits stay
    // legal. Returns true iff the net fully routed and committed.
    bool route_in_corridor(int k, const std::vector<Cell>& corridor, double radius);

    // Blame probe: route net k over a SOFT cost field where foreign-blocked
    // cells are passable at a penalty (trace copper/halo: soft_mult x base
    // cell cost; foreign pads: 50x that, crossed only when nothing else
    // exists). The min-cost descent path then crosses the CHEAPEST blocker
    // set; returns their net IDS (deduped). crossed_pad reports the
    // path had to cross a foreign pad — rip-up alone cannot fix that net.
    // CPU-only (dijkstra_field oracle); intended for endgame repair.
    std::vector<int> probe_blockers(int k, double soft_mult, bool* crossed_pad);
    // Field-router backend (stage 2): route net k from one all-pins GPU field
    // (windowed, 64-cell margin -> full-grid ladder) via label pass, boundary
    // MST and truncated descents. Fills the same Net fields as
    // route_net_tree_reverse. Returns 1 routed, 0 valid-partial/fail, -1
    // backend unavailable/inconsistent (caller uses the A* path).
    int route_net_field(int k, RouteDeadline deadline);
    // Frozen-board variant for C3 batching: routes into a LOCAL Net copy
    // (board state is read-only during the batch; commit happens later with
    // the exact legality re-walk).
    int route_net_field_into(int k, Net& net, RouteDeadline deadline);
    // Closure-free via-eligibility mask for net k over a window (Chebyshev
    // dilation fast path; shared by CPU and GPU bakes).
    std::vector<uint8_t> fast_via_mask_window(int k, int x0, int y0, int x1,
                                              int y1);
    // Via transition cost (A* heuristic uses it consistently). Loaders default 5.0;
    // measured: at 5.0 we place 24% fewer vias/net than KRT on boards both solve —
    // this knob exists to sweep that conservatism against the gap cohort.
    void set_via_cost(double v) { if (v > 0.0) via_cost_ = quantize_cost(v); }
    void set_parallel_nets(int p) { parallel_nets_ = p > 0 ? p : 1; }
    // C1: deterministic per-search expansion budget (0 = unlimited). Unlike
    // wall clocks this is machine- and epoch-independent: step-budgeted
    // experiments become bit-reproducible anywhere. A capped search fails
    // that connection deterministically (same bookkeeping as exhaustion).
    void set_expansion_budget(std::size_t per_search) { expansion_budget_ = per_search; }
    std::size_t expansion_budget() const { return expansion_budget_; }
    int parallel_nets() const { return parallel_nets_; }
    double design_rule_clearance() const { return drc_clearance_; }

    // Falloff sharpness used when baking avoidance. k=8 by default.
    // Changing pad sharpness rebuilds the baked pad-avoidance grid so the
    // self-exemption stays exact (baked cost and self_pad_cost must use the same k).
    double pad_sharp() const { return pad_sharp_; }
    double trace_sharp() const { return trace_sharp_; }
    void set_pad_sharp(double k) { if (k > 0 && k != pad_sharp_) { pad_sharp_ = k; rebuild_avoidance(); } }
    void set_trace_sharp(double k) { if (k > 0) { trace_sharp_ = k; } }

    // Bounded, distance-squared falloff influence radius (cells). Changing it rebuilds
    // the baked avoidance so baked cost and self_pad_cost stay consistent (exact
    // self-exemption). Beyond `falloff_radius` cells an avoidance source contributes 0.
    double falloff_radius() const { return falloff_radius_; }
    void set_falloff_radius(double r) {
        if (r > 0 && r != falloff_radius_) {
            falloff_radius_ = r;
            rebuild_avoidance();
        }
    }

    // --- Physical keepout coating (Stage 8.2/8.3) -------------------------------
    // When a trace is placed it additionally claims cells within `trace_hw` of its
    // centerline as its own (so a later net cannot route within width/2 of it); when a
    // via is placed it claims cells within `via_radius` of its center (on every layer),
    // enforcing via-to-trace / via-to-via spacing. Default 0 = disabled, so existing
    // boards/tests are byte-identical unless these are set.
    double trace_half_width_cells() const { return trace_hw_cells_; }
    double via_radius_cells() const { return via_radius_cells_; }
    // Physical values in mm. physics is resolution-derived, so when a Board is rebuilt at
    // a different resolution (e.g. a res-1.0 placeholder replaced by a real res-0.05 load),
    // config must move as mm and be re-derived for the new grid, not copied as raw cells.
    double trace_half_width_mm() const { return trace_hw_cells_ * grid_.resolution(); }
    double via_radius_mm() const { return via_radius_cells_ * grid_.resolution(); }
    void set_trace_half_width_cells(double w) {
        if (w <= 0.0)
            throw std::invalid_argument("trace half-width must be > 0; physics cannot be disabled");
        trace_hw_cells_ = w;
        physics_explicit_ = true;
    }
    void set_via_radius_cells(double r) {
        if (r <= 0.0)
            throw std::invalid_argument("via radius must be > 0; physics cannot be disabled");
        via_radius_cells_ = r;
        physics_explicit_ = true;
    }
    // Physics derived from the BOARD'S OWN design rules (loaders): sets values without
    // marking them user-explicit, so precedence is user config > board rules > defaults
    // (RoutingEnv transfers a prior board's physics only when user-explicit).
    void set_rule_physics(double tw_cells, double vr_cells) {
        if (tw_cells > 0.0) trace_hw_cells_ = tw_cells;
        if (vr_cells > 0.0) via_radius_cells_ = vr_cells;
    }
    // Per-net-class physics: assign net `net_id` a (half-width, clearance) pair
    // in CELLS. Values matching the board globals keep the net on tier 0 (free);
    // new values allocate a tier with its own claim masks. Call after loading,
    // before any routing. See PhysTier above for the pair-exactness argument.
    void set_net_class_physics(int net_id, double hw_cells, double clr_cells);
    int tier_of_id(int id) const {
        return (id >= 0 && (size_t)id < tier_of_id_.size()) ? tier_of_id_[(size_t)id] : 0;
    }
    int tier_of_tok(int32_t tok) const { return tier_of_id(tok - 1); }
    double tier_hw(int t) const {
        return t <= 0 ? trace_hw_cells_ : extra_tiers_[(size_t)t - 1].hw;
    }
    double tier_clr(int t) const {
        return t <= 0 ? (double)drc_clearance_cells() : extra_tiers_[(size_t)t - 1].clr;
    }
    // Maxima over tiers: the safe values for machinery not yet pair-aware
    // (via emergence, negotiation claims, probes).
    double max_hw_cells() const {
        double v = trace_hw_cells_;
        for (const auto& t : extra_tiers_) v = std::max(v, t.hw);
        return v;
    }
    double max_clr_cells() const {
        double v = (double)drc_clearance_cells();
        for (const auto& t : extra_tiers_) v = std::max(v, t.clr);
        return v;
    }
    // Query-side mask selection: the routing net's tier decides which zone /
    // pad-keepout mask gates its entry.
    const int32_t* zone_data_for(int32_t tok) const {
        const int t = tier_of_tok(tok);
        return t <= 0 ? cl_zone_.data() : extra_tiers_[(size_t)t - 1].zone.data();
    }
    const int32_t* pad_infl_data_for(int32_t tok) const {
        const int t = tier_of_tok(tok);
        return t <= 0 ? pad_inflated_.data() : extra_tiers_[(size_t)t - 1].pad_infl.data();
    }
    // Exact pair radius: owner copper (hw_o, c_o) vs tier-t traffic.
    double pair_zone_radius(double hw_o, double c_o, int t) const {
        return hw_o + tier_hw(t) + std::max(c_o, tier_clr(t));
    }
    bool physics_explicit() const { return physics_explicit_; }
    double pad_radius_cells() const { return pad_radius_cells_; }
    void set_pad_radius_cells(double r) { pad_radius_cells_ = r > 0.0 ? r : via_radius_cells_; }

    // Tree-growth strategy (Stage 9): forward (default) or reverse-from-pins. Reverse is
    // much faster on large fine-grid boards but slower on small boards; both produce
    // tie-equivalent trees (see net.hpp).
    TreeStrategy tree_strategy() const { return tree_strategy_; }
    void set_tree_strategy(TreeStrategy s) { tree_strategy_ = s; }

    // --- Congestion prior (RUDY over flight lines) -------------------------------
    // Bake a shared 2D routing-demand field: per net, an MST over its pins is
    // decomposed into two-pin flight segments; each smears density (w+h)/(w*h) over
    // its bounding box (RUDY, Spindler&Johannes DATE'07). Normalized to max 1.
    // enter_cost adds congest_weight * base_cost * field so every net softly avoids
    // predicted-congested regions; 0 weight (default) disables — bit-identical costs.
    void bake_congestion_rudy();
    double congestion_weight() const { return congest_weight_; }
    void set_congestion_weight(double w) { congest_weight_ = w > 0.0 ? quantize_cost(w) : 0.0; }
    // Field shape: cost term uses field^gamma (gamma 2 penalizes only hotspots).
    double congestion_gamma() const { return congest_gamma_; }
    void set_congestion_gamma(double g) { congest_gamma_ = g >= 1.0 ? g : 1.0; }
    // Diagnostic/observation access: normalized demand at (x,y), 0 if not baked.
    double congestion_at(int x, int y) const {
        if (congest_cost_.empty() || x < 0 || y < 0 || x >= grid_.width() || y >= grid_.height())
            return 0.0;
        return congest_cost_[(size_t)y * (size_t)grid_.width() + (size_t)x];
    }

    // Wall-time budget (seconds) for a single route_all()/route_from() call; 0 (default)
    // = unlimited. When it expires, the pass stops starting new work and returns PARTIAL
    // results: already-routed nets keep their committed traces, the rest report unrouted,
    // and stats.time_budget_exceeded is set. The A* inner loop polls the deadline, so
    // even one search on a huge grid is interruptible. NOTE: makes results machine/load-
    // dependent — a harness knob, not part of any reproducibility contract.
    double route_time_budget_s() const { return route_time_budget_s_; }
    void set_route_time_budget_s(double s) { route_time_budget_s_ = s > 0.0 ? s : 0.0; }

    void set_avoidance(std::size_t net_idx, double pad_mult, double trace_mult);
    // Reset all per-net avoidance multipliers to 0 (clean episode). Does not re-route.
    void clear_avoidance();
    // Add a pad. radius_cells is the pad's PHYSICAL radius (cells) used for clearance so a
    // foreign trace never overlaps the pad's real footprint (0 = point-pad, e.g. synthetic).
    // Registers a circular PadShape of that radius; all clearance rules run on shapes.
    void add_pad(std::size_t net_idx, const Cell& c, double radius_cells = 0.0);
    // add_pad + register the cell as a routing TERMINAL of the net (Net::pins).
    // Basis for waypoint-guided routing: a point pad placed on a planned escape
    // ladder or trunk lane becomes a mandatory tree terminal, forcing the net's
    // route through the planned structure with the ordinary router.
    void add_pin_pad(std::size_t net_idx, const Cell& c, double radius_cells = 0.0);
    // Add a no-net obstacle footprint (an unconnected pad / keepout): its copper blocks
    // every net's routing (clearance-dilated like any pad) but belongs to no net.
    void add_keepout_shape(int layer, double cx, double cy, double half_w, double half_h,
                           double rot_deg, bool oval);
    // Rule area banning vias only (tracks stay legal): blocks via emergence inside
    // the shape (dilated by the via radius) through the via_ok mask.
    void add_via_keepout_shape(int layer, double cx, double cy, double half_w,
                               double half_h, double rot_deg, bool oval);
    // Add a pad with its REAL copper footprint: pin cell `c` (electrical identity) plus the
    // exact shape — fractional center, per-axis half extents (cells), rotation, and
    // rect-vs-oval. Keepout/DRC/via rules use the shape's true Minkowski dilation.
    void add_pad_shape(std::size_t net_idx, const Cell& c,
                       double cx, double cy, double half_w, double half_h,
                       double rot_deg, bool oval);
    // Test/validation hook: mark `c` as real copper owned by `net_id` (sets
    // trace_owner_/owner_/copper_). Lets DRC rules (e.g. trace-trace clearance) be
    // exercised on hand-built layouts without routing.
    void add_manual_trace(const Cell& c, int net_id);
    // Declare column (x,y) a thru-hole pad barrel of `net_idx`: its pre-drilled copper
    // already connects every layer, so a layer transition riding it is NOT a router-
    // placed via (no via cost accounting, no via keepout, exempt from via_ok). Loaders
    // call this when fanning a thru-hole pad out to one pin per copper layer; stacked
    // same-net SMD pads must NOT be declared (connecting those needs a real via).
    void mark_thru_pad(std::size_t net_idx, int x, int y);

    // Move net [from_idx] to [to_idx] (bump-down semantics). Returns new index.
    int move_net(int from_idx, int to_idx);

    RouteStats route_all();
    // Clear all routing state back to pads-only (fresh episode; extracted route_all
    // preamble): keepouts prepared, traces/copper/avoidance cleared, zones rebuilt,
    // every net reset to unattempted. Connection-mode episodes start here.
    void clear_routing();

    // --- Connection-level routing (pad-at-a-time RL mode) -----------------------
    // Connect ONE unconnected pin of the net at `position` to its committed component
    // and commit the path immediately (real copper, zones, avoidance -- identical
    // machinery to net-mode commits). pin_idx = -1 connects the NEAREST unconnected
    // pin (the exact tie-breaking of tree growth: multi-source A* finds it); a
    // specific pin index restricts the goal set to that pin. The board is the only
    // state: partial nets, checkpoints, and stats all read straight from it.
    // A net's own committed copper NEVER repels it: the self-avoidance scratch
    // subtracts the active net's own contributions from the shared avoidance field.
    struct ConnectResult {
        bool ok = false;                  // a pin was connected (or nothing left to do)
        bool progress = false;            // a NEW pin became connected this call
        bool time_budget_exceeded = false;
        int unconnected_pins = 0;         // net's remaining count after the call
        double added_length = 0.0;
    };
    ConnectResult route_one_connection(int position, int pin_idx = -1);
    // Unconnected pins of the net at `position` (indices into net.pins).
    std::vector<int> unconnected_pin_indices(int position) const;
    // Snapshot stats of the CURRENT board state (no routing): per-net lengths,
    // unconnected counts, vias. with_drc runs a full DRC pass; without it,
    // drc_violations is unrouted_count — EXACT for this router mid-episode, since
    // hard clearance makes copper violations impossible by construction. Connection-
    // mode steps use with_drc=false and verify with a real pass at episode end.
    RouteStats collect_stats(bool with_drc = true) const;
    // Incremental re-route: assumes nets [0, lo) are already routed with their traces
    // still in the board state (e.g. from a prior route_all/route_from). Rips up and
    // re-routes nets [lo, n). Preserving the [0, lo) prefix is valid because those nets'
    // order and predecessors are unchanged by any move at index >= lo. Per-net results
    // equal a fresh route_all over the same board with the same net order.
    RouteStats route_from(int lo);
    RouteStats check_drc() const;

    // ---- post-route trace smoothing (grid staircase -> straight segments) ----
    // A committed grid path is 8-connected, so a shallow slope renders as a
    // staircase of tiny H/diagonal steps. smooth_paths() replaces each net's
    // per-layer vertex chain with the fewest straight segments that (a) stay
    // within `max_dev_cells` of the original centerline (so a load-bearing
    // detour is never shortcut away) and (b) whose rasterized centerline is
    // DRC-clean against foreign copper/pads with a `max_dev_cells` safety
    // margin (so two neighbouring nets smoothing at once can never collide).
    // Endpoints (pads/vias) are preserved exactly. The result is a per-net
    // list of float-cell segments for the writer; the grid model is untouched
    // (validation reads it, never mutates it). Returns segments removed.
    // A smoothed run: a straight segment, or (is_arc) a circular arc from
    // (x0,y0) through (mx,my) to (x1,y1) — a corner fillet. All in float cells.
    struct SmoothSeg {
        int layer; double x0, y0, x1, y1;
        double mx = 0.0, my = 0.0; bool is_arc = false;
    };
    // fillet_radius_cells > 0 rounds interior corners with tangent arcs (the
    // KiCad-native smooth-curve primitive); each arc is validated exactly like
    // a straight smoothed segment, so it never introduces a DRC violation.
    int smooth_paths(double max_dev_cells = 1.5, double fillet_radius_cells = 0.0);
    const std::vector<std::vector<SmoothSeg>>& smoothed_paths() const {
        return smoothed_paths_;
    }
    bool has_smoothed() const { return !smoothed_paths_.empty(); }
    // Static pad-vs-pad design-rule check on the UNROUTED board: counts cells where
    // two different nets' pad copper, each dilated by clearance/2, overlap — i.e. the
    // board's own pad geometry already violates its clearance rule before any routing.
    // A positive count means uniform-rule full routing is unattainable (the original
    // design relies on local rule overrides); use it to flag/skip such boards.
    int count_static_pad_conflicts() const;

    const std::vector<PadShape>& pad_shapes() const { return pad_shapes_; }
    // Diagnostic: baked pad-keepout token at a cell (0 free, -1 multi-net CONFLICT).
    // Valid after prepare_pad_keepout (any route call); for inspection/tests.
    int32_t pad_inflated_token(const Cell& c) const {
        return valid(c) ? pad_inflated_[grid_.index(c)] : 0;
    }

    // Thru-hole barrel token at column (x,y): net_id+1 if a thru-hole pad pre-connects
    // every layer there, 0 otherwise. See mark_thru_pad().
    int32_t thru_pad_token(int x, int y) const {
        if (x < 0 || y < 0 || x >= grid_.width() || y >= grid_.height()) return 0;
        return thru_pad_col_[(size_t)y * (size_t)grid_.width() + (size_t)x];
    }
    // Number of router-placed vias in a net's committed paths: layer transitions
    // excluding those riding the net's own thru-hole pad barrels (pre-drilled pad
    // copper is not a via). The single source of truth for via accounting — Python
    // consumers (stats, RL observation, exporters, renderers) must use this rather
    // than re-counting raw layer transitions.
    int count_net_vias(const Net& net) const;

    // --- occupancy exports. RAW TOKENS: 0 = free, else (net id + 1). This is
    // unambiguous even for net id 0 (its cells report 1). Use is_free() /
    // owner_of() for the decoded view. ---
    const std::vector<int32_t>& owner() const { return owner_; }
    int32_t owner_token(const Cell& c) const { return valid(c) ? owner_[grid_.index(c)] : 0; }
    const std::vector<int32_t>& pads() const { return pad_owner_; }
    int32_t pad_token(const Cell& c) const { return valid(c) ? pad_owner_[grid_.index(c)] : 0; }
    const std::vector<int32_t>& traces() const { return trace_owner_; }
    int32_t trace_token(const Cell& c) const { return valid(c) ? trace_owner_[grid_.index(c)] : 0; }

    // Decoded net id (0 = free). NOTE: net id 0 == free == 0 here; prefer
    // owner_token()/is_free() when the distinction matters.
    int owner_at(const Cell& c) const { int32_t t = owner_token(c); return t == 0 ? 0 : (int)(t - 1); }
    int pad_owner(const Cell& c) const { int32_t t = pad_token(c); return t == 0 ? 0 : (int)(t - 1); }
    int trace_owner(const Cell& c) const { int32_t t = trace_token(c); return t == 0 ? 0 : (int)(t - 1); }
    // True iff this cell is free (not owned by any net, pad, or trace).
    bool is_free(const Cell& c) const { return owner_token(c) == 0; }

    // Avoidance cost at a cell (gross, includes ALL objects incl. self).
    double pad_avoid_cost_at(const Cell& c) const {
        return valid(c) ? pad_avoid_cost_[grid_.index(c)] : 0.0;
    }
    double trace_avoid_cost_at(const Cell& c) const {
        return valid(c) ? trace_avoid_cost_[grid_.index(c)] : 0.0;
    }

    double width_units() const;
    double height_units() const;

    // Recompute both avoidance grids from scratch (used by loaders / after manual
    // pin edits). Exposed for correctness/testing.
    void rebuild_avoidance();
    void ensure_avoid_baked();
    // C3: MIS-batched optimistic parallel nets. Batches are maximal CONSECUTIVE
    // runs of nets whose flight lines don't cross (preserves order semantics
    // exactly; composes with bundle ordering, whose bus runs are precisely
    // such prefixes). Members route concurrently against the frozen board,
    // commit strictly in net order with exact per-path revalidation; any
    // member invalidated by an earlier commit re-routes serially in place.
    // Deterministic by construction. Default 1 = serial (zero change).
    void route_nets_from_parallel(int lo, RouteStats& stats, RouteDeadline deadline);
    bool wave_flood_impl(int position, const std::vector<Cell>& seeds,
                         const std::vector<Cell>& targets, uint16_t* map,
                         const uint8_t* via_col_override = nullptr) const;
    void build_masks32(int position, const std::vector<Cell>& seeds,
                       uint32_t* ent, uint32_t* via, uint32_t* cur) const;

    // Bitplane wavefront reachability (optimization plan C2b/D0): 64-cell-per-
    // word wave propagation over the net's exact enter mask with a SUPERSET
    // via plane (column pz + own barrel; the R-neighborhood via keepout is
    // skipped — superset edges keep UNSAT verdicts sound and wave distances
    // admissible). Returns whether any seed reaches any target. This is the
    // same data-parallel algorithm the Vulkan stage runs; here it executes on
    // CPU words (auto-vectorizable).
    bool wave_probe_superset(int position, const std::vector<Cell>& seeds,
                             const std::vector<Cell>& targets);
    bool wave_probe(int position, const std::vector<Cell>& seeds,
                    const std::vector<Cell>& targets) const;
    // Full flood recording per-cell wave indices (0 = unreached, else wave+1).
    // Returns a thread-local grid-sized buffer valid until the next wave call
    // on this thread. h(c) = (map[c]-1) * base_cost is an admissible,
    // consistent, obstacle-aware heuristic for searches TOWARD `seeds`.
    const uint16_t* wave_map(int position, const std::vector<Cell>& seeds) const;
    // Scalar-probe twin with identical call shape (for parity tests/benchmarks)
    bool scalar_probe(int position, const std::vector<Cell>& seeds,
                      const std::vector<Cell>& targets);
    // Visited-cell count of a full wave flood (equivalence key for the GPU
    // implementation: same masks => identical count) — floods to exhaustion.
    size_t wave_reach_count(int position, const std::vector<Cell>& seeds) const;
    // Dump the net's wave masks + seeds as 32-bit-packed planes (the exact
    // input contract of tools/wave_vk): header i32[4]={W,H,L,S32}, then ent
    // (L*H*S32), via (H*S32), cur seeds (L*H*S32). GPU and CPU consume
    // identical bytes => visited-count equality is a real equivalence test.
    void dump_wave_masks(int position, const std::vector<Cell>& seeds,
                         const std::string& path) const;

private:
    bool valid(const Cell& c) const { return grid_.valid(c); }
    int drc_clearance_cells() const;
    // True iff the layer transition a->b (same x,y) rides this net's own thru-hole pad
    // barrel rather than a router-placed via.
    bool own_pad_barrel(const Cell& a, const Cell& b, int32_t tok) const;
    // Per-net routing context: the enter-cost and via-eligibility closures for the net
    // at array position k (extracted from the net-mode loop; shared by both modes).
    struct NetContext { EnterCostFn enter; ViaEligibilityFn via_ok; };
    NetContext make_net_context(int k);
    // Commit ONE path of `net` to the board: real copper, swath coating, via zones,
    // avoidance baking (the net-mode commit-loop body, shared by both modes). Also
    // bakes into the self-avoidance scratch when `net` is the active scratch net.
    void commit_path(Net& net, const std::vector<Cell>& seg);
    // Make `nid`'s own committed contributions subtractable from trace avoidance
    // (rebuilds the scratch from net.committed_cells when the active net changes).
    void ensure_self_scratch(int nid);
    // Route nets [lo, n) using the CURRENT board state. Precondition: the traces of
    // nets [0, lo) are already present in trace_owner_/owner_ and baked into
    // trace_avoid_cost_; the traces of nets [lo, n) must already be cleared. Fills stats.
    void route_nets_from(int lo, RouteStats& stats, RouteDeadline deadline);
    // Claim the (axis-aligned-square) neighborhood of radius `r` around `c` on layers
    // [lmin,lmax] as owned by `tok` (Stage 8.2/8.3 keepout coating). Skips foreign pads
    // and already-owned cells; bakes avoidance for consistency with route_from's rebuild.
    // `claimed`, when non-null, collects every cell whose trace_owner_ this call
    // newly set (avoidance was baked there) — for incremental self-scratch updates.
    void coat(const Cell& c, int lmin, int lmax, double radius, int32_t tok,
              std::vector<Cell>* claimed = nullptr);
    // Write the UNCONDITIONAL clearance keepout (cl_zone_) for every cell within `radius`
    // on layers [lmin,lmax] as `tok`. Used by coat/pad-keepout and to rebuild cl_zone_ from
    // the preserved prefix (pads + real copper) so full == incremental.
    void set_zone(const Cell& c, int lmin, int lmax, double radius, int32_t tok);
    // set_zone into an arbitrary tier mask (per-net-pair claims).
    void set_zone_mask(std::vector<int32_t>& mask, const Cell& c, int lmin,
                       int lmax, double radius, int32_t tok);
    void rebuild_cl_zone();
    // Bake pad_inflated_ from all pads (post-load; pads are static). No-op when physics off.
    void prepare_pad_keepout();
    // Sum of falloff from a single source cell `src` to the whole grid (baked).
    void add_source_avoidance(const Cell& src, std::vector<double>& cost, double sharp);
    // Self-contribution of net k's own pads at a given cell (for exemption).
    double self_pad_cost(const Cell& c, int k) const;

    Grid grid_;
    double via_cost_;
    double base_cost_;
    double drc_clearance_;
    double pad_sharp_ = 8.0;
    double trace_sharp_ = 8.0;
    double falloff_radius_ = DEFAULT_FALLOFF_RADIUS;
    double trace_hw_cells_ = 0.0;   // trace half-width in cells (default set in ctor from KiCad 0.1mm)
    double via_radius_cells_ = 0.0; // via keepout radius in cells (default set in ctor from KiCad 0.3mm)
    double pad_radius_cells_ = 0.0; // pad physical radius in cells (default = via radius); pad clearance
                                    // is padded by this so a trace never overlaps a pad's real footprint
    double max_pad_radius_cells_ = 0.0; // largest per-pad radius on the board (via_ok scan bound)
    TreeStrategy tree_strategy_ = TreeStrategy::forward;
    double route_time_budget_s_ = 0.0;   // 0 = unlimited
    bool physics_explicit_ = false;      // trace/via set by a caller (not ctor/rules)
    std::vector<int32_t> owner_;
    std::vector<int32_t> pad_owner_;
    std::vector<int32_t> trace_owner_;
    // Thru-hole pad barrel columns, token-keyed by (x,y) (layer-independent): the pad's
    // drilled barrel pre-connects every layer for its net. See mark_thru_pad().
    std::vector<int32_t> thru_pad_col_;
    // Pad physical radius (cells) keyed by cell; 0 = point-pad. Parallel to pad_owner_.
    std::vector<double> pad_radius_cell_;
    // Sparse marker of REAL copper cells (actual trace path + via barrel), distinct from
    // keepout-coating margin (Stage 8.2/8.3) which is ownership-only. copper_.size()==grid.
    // Used so DRC (padding/trace clearance) is evaluated only against real copper, avoiding
    // false positives from the squared keepout margin. Keyed by trace_owner_ token.
    std::vector<uint8_t> copper_;
    // Stage 8.4c pad-inflation keepout (routing-side trace-width pad spacing). For each cell
    // within `pad_inflation = clr + tw` (cells) of a pad, stores that pad's net token; if
    // pads of MORE than one net cover the same cell, stores PAD_CONFLICT (-1). Routing a net
    // blocks any cell whose token != 0 and != the net's own token, so a net is exempt from
    // its own pads' inflation (it must be able to reach them) but must keep trace width +
    // clearance from foreign pads. Pads are static; baked once per route.
    std::vector<int32_t> pad_inflated_;
    // Via-emergence pad keepout, token-keyed like pad_inflated_: cells whose center is
    // within (via_radius + trace_hw + clearance) of any pad's REAL copper. Baked by
    // prepare_pad_keepout from pad_shapes_; via_ok tests it with one lookup instead of
    // scanning a neighborhood per candidate cell.
    std::vector<int32_t> via_pad_zone_;
    // Real pad footprints (one entry per pad per copper layer). The single geometry
    // source for pad_inflated_, via_pad_zone_, and the DRC pad-trace rule.
    std::vector<PadShape> pad_shapes_;
    // Post-route smoothed geometry: per-net float-cell segments (see
    // smooth_paths). Empty until smooth_paths() runs; the writer emits it in
    // place of the raw grid runs when present.
    std::vector<std::vector<SmoothSeg>> smoothed_paths_;
    // Stage 16: UNCONDITIONAL clearance keepout, token-keyed (0 = none). coat() and pad-
    // keepout write it for EVERY cell within the spacing radius, regardless of the cell's
    // current ownership. enter_cost uses it as a HARD rejection, so a net never drops its
    // real trace inside another net's clearance zone even when the owned swath is truncated
    // under congestion (edge-to-edge DRC-clean guarantee).
    std::vector<int32_t> cl_zone_;
    // --- Per-net-class physics tiers (per-net-PAIR spacing) ------------------
    // Tier 0 is the board's global (trace_hw_cells_, drc_clearance_cells());
    // extra tiers hold wider classes. Claims are stamped per tier at the exact
    // pair radius (owner hw + tier hw + max(owner clr, tier clr)); a routing
    // net queries ITS tier's masks, so center-center >= hw_i + hw_j +
    // max(c_i, c_j) holds EXACTLY for every pair. Single-class boards have no
    // extra tiers and take today's paths bit-identically.
    struct PhysTier {
        double hw = 0.0, clr = 0.0;                  // cells
        std::vector<int32_t> zone;                   // per-tier cl_zone
        std::vector<int32_t> pad_infl;               // per-tier pad keepout
    };
    std::vector<PhysTier> extra_tiers_;              // tiers 1..K-1
    std::vector<int> tier_of_id_;                    // net id -> tier (empty = all 0)
    // Per-type occupancy masks (the user-desired separate images): pads (pad_owner_), real
    // trace copper (copper_ keyed by trace_owner_), and via barrels (via_owner_, 0 = not a
    // via). via_owner_ makes the via-emergence distance check (via-pad / via-trace / via-via)
    // a plain neighborhood scan over the right mask.
    std::vector<int32_t> via_owner_;
    std::vector<double> pad_avoid_cost_;
    std::vector<double> trace_avoid_cost_;
    // Self-avoidance exemption: the ACTIVE net's own contributions to
    // trace_avoid_cost_, subtracted in enter_cost so a net never repels itself.
    // All-zero whenever the active net has no committed copper -- always true in
    // net-mode routing, which keeps net-mode arithmetic bit-identical.
    std::vector<double> self_trace_scratch_;
    int active_self_net_ = -1;   // net id the scratch currently describes
    // Shared RUDY demand field (2D, W*H, normalized max 1; empty = not baked) and its
    // global weight in units of base_cost. See bake_congestion_rudy().
    std::vector<double> congest_cost_;
    double congest_weight_ = 0.0;
    double congest_gamma_ = 1.0;
    double congest_norm_ = 1.0;   // pre-normalization max (self-exemption uses it)
    // Per-net MST flight edges (x0,y0,x1,y1 bboxes) recorded by bake_congestion_rudy,
    // keyed by net id: a net NEVER pays for its own predicted demand (self-exemption,
    // same principle as pad/trace avoidance) — subtracted via a per-net scratch.
    std::vector<std::vector<std::array<int, 4>>> congest_edges_;
    std::vector<double> self_congest_scratch_;
    int active_congest_net_ = -1;
    void ensure_self_congest(int nid);
    std::vector<NetAvoidance> avoid_;
    int parallel_nets_ = 1;
    std::size_t expansion_budget_ = 0;
    // Lazy avoidance baking: fields are baked on the FIRST nonzero avoidance
    // multiplier (rebuild_avoidance from owner grids reproduces the exact
    // incremental state). Until then all avoidance/scratch baking is skipped —
    // profiling showed it dominating boards routed with multipliers at 0.
    bool avoid_baked_ = false;
    std::vector<Net> nets_;
};

} // namespace routing

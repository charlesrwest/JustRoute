#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "routing/env.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace routing;

static std::string read_file(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
static std::string data_file(const std::string& name) {
    return std::string(ROUTING_TEST_DATA_DIR) + "/boards/" + name;
}

static SyntheticSpec small_board() {
    SyntheticSpec spec;
    spec.layers = 1; spec.width = 20; spec.height = 20; spec.resolution = 1.0;
    spec.via_cost = 5.0; spec.base_cost = 1.0; spec.design_rule_clearance = 0.0;
    spec.nets = {{{0,2,2},{0,2,8}}, {{0,9,3},{0,9,11}}, {{0,5,15},{0,12,15}}};
    return spec;
}

TEST_CASE("reset_routes_board") {
    RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    REQUIRE(env.load_synthetic(small_board()) == 3);
    RouteStats st = env.reset();
    CHECK(st.unrouted_count == 0);
    CHECK(st.per_net_length.size() == 3);
    for (double len : st.per_net_length) CHECK(len > 0.0);
}

TEST_CASE("position_of_returns_position_or_minus_one") {
    RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    REQUIRE(env.load_synthetic(small_board()) == 3);
    // ids 0,1,2 start at positions 0,1,2.
    CHECK(env.position_of(0) == 0);
    CHECK(env.position_of(1) == 1);
    CHECK(env.position_of(2) == 2);
    CHECK(env.position_of(99) == -1);
    // After moving net id 2 to the front, its position updates and others shift.
    env.step(2, 0, 0.0, 0.0);
    CHECK(env.position_of(2) == 0);
    CHECK(env.position_of(0) == 1);
    CHECK(env.position_of(1) == 2);
}

TEST_CASE("step_matches_incremental_equals_full") {
    RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    env.load_synthetic(small_board());
    env.reset();
    // Move net id 2 to the front with a trace penalty; the env's incremental step must
    // equal a fresh full route over the same final order.
    RouteStats stepped = env.step(/*net_idx=*/2, /*target_pos=*/0, /*pad_avoid=*/1.0,
                                  /*trace_avoid=*/2.0);
    REQUIRE(stepped.per_net_length.size() == 3);

    // Rebuild the final order by hand and route fully via a fresh Board to compare.
    SyntheticSpec spec = small_board();
    // step(2,0) relocated the net with id==2 from position 2 to position 0: order -> {2,0,1}.
    std::vector<std::vector<Cell>> ordered = {spec.nets[2], spec.nets[0], spec.nets[1]};
    Board fresh(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    for (size_t i = 0; i < ordered.size(); ++i) {
        Net net; net.id = (int)i; net.pins = ordered[i];
        fresh.nets().push_back(std::move(net));
        for (const Cell& p : ordered[i]) fresh.add_pad((size_t)i, p);
    }
    // NOTE: avoidance (trace 2.0) was set on the moved net; replicate on the fresh net 0.
    fresh.set_avoidance(0, 1.0, 2.0);
    RouteStats full = fresh.route_all();

    // Per-net lengths and unrouted must match the env's incremental step.
    CHECK(stepped.unrouted_count == full.unrouted_count);
    REQUIRE(stepped.per_net_length.size() == full.per_net_length.size());
    for (size_t i = 0; i < stepped.per_net_length.size(); ++i)
        CHECK(stepped.per_net_length[i] == doctest::Approx(full.per_net_length[i]));
}

TEST_CASE("step_incremental_positive_lo_equals_full") {
    // A move that lands strictly inside the board (lo > 0) exercises the true
    // incremental span — only [lo, n) is re-routed. The env's incremental step must
    // still equal a full route over the same final order (Stage-4 property).
    RoutingEnv a(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    a.load_synthetic(small_board()); a.reset();
    // Move net id 2 from position 2 to position 1 -> final order {0,2,1}, lo = min(2,1) = 1.
    RouteStats stepped = a.step(/*net_idx=*/2, /*target_pos=*/1, /*pad_avoid=*/1.0,
                                /*trace_avoid=*/2.0);
    // reset() re-routes the current (final) order while retaining the avoidance the step
    // set on the moved net -> this is the reference full route (incremental == full).
    RouteStats full = a.reset();
    REQUIRE(stepped.per_net_length.size() == full.per_net_length.size());
    CHECK(stepped.unrouted_count == full.unrouted_count);
    CHECK(stepped.total_length == doctest::Approx(full.total_length));
    for (size_t i = 0; i < stepped.per_net_length.size(); ++i)
        CHECK(stepped.per_net_length[i] == doctest::Approx(full.per_net_length[i]));
}

TEST_CASE("step_respects_penalty") {
    RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    env.load_synthetic(small_board());
    env.reset();
    // Baseline total length with no penalty vs a strong trace-avoidance penalty on the
    // moved net: applying a penalty must not shorten the board (it can only detour/lengthen).
    RouteStats base = env.reset();
    // reset moved nothing; now apply a penalty by stepping a net in place (same position).
    // step requires a move, so instead compare two identical moves with/without penalty.
    RoutingEnv a(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    a.load_synthetic(small_board()); a.reset();
    RouteStats s_low = a.step(/*net*/2, /*to*/1, 0.0, 0.0);
    RoutingEnv b(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    b.load_synthetic(small_board()); b.reset();
    RouteStats s_hi = b.step(/*net*/2, /*to*/1, 0.0, 30.0);
    // Same move; the strong trace penalty should not REDUCE total length.
    CHECK(s_hi.total_length >= s_low.total_length - 1e-9);
    CHECK(base.unrouted_count == 0);
}

TEST_CASE("observation_shape_and_content") {
    RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    env.load_synthetic(small_board());
    env.reset();
    std::vector<double> obs = env.get_observation();
    // 3 nets -> [len0,len1,len2, unr0,unr1,unr2] = 6 entries.
    CHECK(obs.size() == 6);
    // All routed -> all unrouted flags are 0.
    for (size_t i = 3; i < obs.size(); ++i) CHECK(obs[i] == 0.0);
    // Lengths are positive.
    for (size_t i = 0; i < 3; ++i) CHECK(obs[i] > 0.0);
}

TEST_CASE("loaders_round_trip") {
    RoutingEnv env(2, 1, 1, 1.0, 5.0, 1.0, 0.0);
    int rdl_nets = env.load_pcb_rdl(read_file(data_file("CherryMxBitboard.rdl.json")), 1.0);
    CHECK(rdl_nets == 3);
    CHECK(env.reset().unrouted_count < rdl_nets);

    RoutingEnv env2(2, 1, 1, 1.0, 5.0, 1.0, 0.0);
    int kicad_nets = env2.load_kicad_pcb(
        read_file(data_file("KicadV5_xover4schiit.kicad_pcb")), 1.0);
    CHECK(kicad_nets == 3);
    CHECK(env2.reset().unrouted_count == 0);
}

TEST_CASE("thread_isolated_instances") {
    // Two independent env instances over the same board must produce identical results
    // (no shared global mutable state) -- precursor to Stage-6 concurrency.
    SyntheticSpec spec = small_board();
    auto run = [&]() {
        RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
        env.load_synthetic(spec);
        env.reset();
        return env.step(2, 1, 0.0, 0.0);
    };
    RouteStats r1 = run();
    RouteStats r2 = run();
    REQUIRE(r1.per_net_length.size() == r2.per_net_length.size());
    for (size_t i = 0; i < r1.per_net_length.size(); ++i)
        CHECK(r1.per_net_length[i] == doctest::Approx(r2.per_net_length[i]));
    CHECK(r1.unrouted_count == r2.unrouted_count);
}

TEST_CASE("self_exemption_exact_under_bounded_kernel") {
    // A net routed with a LARGE pad-avoidance multiplier must NOT repel itself: its own
    // pads' contribution is subtracted exactly (bake and self_pad_cost use the same
    // bounded kernel + radius + sharp). If self-exemption leaked, the straight path
    // between its own pins would be pushed into a long detour; if exact (as required),
    // the direct path is chosen at minimal cost.
    RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    SyntheticSpec spec = small_board();
    spec.nets = {{{0,2,2},{0,12,2}}};   // single net, straight horizontal, length 10
    env.load_synthetic(spec);
    env.board().set_avoidance(0, 300.0, 0.0);   // huge pad-avoidance on its own net
    RouteStats st = env.reset();
    CHECK(st.unrouted_count == 0);
    CHECK(st.per_net_length[0] == doctest::Approx(10.0).epsilon(1e-6)); // no self-detour
    CHECK(st.drc_violations == 0);
}

TEST_CASE("cross_net_pad_is_respected") {
    // A net must never route over ANOTHER net's pad (its cells are hard-blocked by the
    // owner token). Net0 owns pad (10,10); net1 routed (2,10)->(18,10) must detour around
    // it rather than route through it, leaving net0's pad owned by net0.
    RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    SyntheticSpec spec = small_board();
    spec.nets = {{{0,10,10},{0,10,10}}, {{0,2,10},{0,18,10}}};
    env.load_synthetic(spec);
    RouteStats st = env.reset();
    CHECK(st.unrouted_count == 0);
    CHECK(st.drc_violations == 0);
    // net0 id0 -> token 1 still owns its pad; net1 (token 2) must not have taken it.
    CHECK(env.board().owner_token({0,10,10}) == 1);
}

TEST_CASE("via_column_blocks_unused_layer") {
    // Stage 8.1: a via (through-hole, layers 0->1) at (5,5) must make the drill column
    // impassable on ALL layers — including layer 2, which net0's trace never touches.
    RoutingEnv env(3, 20, 20, 1.0, 2.0, 1.0, 0.0);
    SyntheticSpec spec;
    spec.layers = 3; spec.width = 20; spec.height = 20; spec.resolution = 1.0;
    spec.via_cost = 2.0; spec.base_cost = 1.0; spec.design_rule_clearance = 0.0;
    // net0: through-via at (5,5) spanning layers 0 and 1.
    // net1: horizontal trace on layer 2, y=5, x in [4,6] -> crosses x=5.
    spec.nets = {{{0,5,5},{1,5,5}}, {{2,4,5},{2,6,5}}};
    env.load_synthetic(spec);
    RouteStats st = env.reset();
    CHECK(st.unrouted_count == 0);
    CHECK(st.drc_violations == 0);
    CHECK(st.total_vias == 1);
    // The via column must be impassable on ALL layers, including layer 2 which net0's
    // copper never touches. Physics detours can move the via, so locate net0's layer
    // transition and assert that cell is net0-owned (token 1) on layer 2 and NOT net1's.
    auto via_col = [&]() {  // returns (x, y) of net0's layer-0->1 transition, or (-1,-1)
        for (const auto& path : env.board().nets()[0].segments)
            for (size_t k = 0; k + 1 < path.size(); ++k)
                if (path[k].layer != path[k + 1].layer)
                    return std::pair<int, int>{path[k].x, path[k].y};
        return std::pair<int, int>{-1, -1};
    };
    auto [vx, vy] = via_col();
    REQUIRE(vx >= 0);
    CHECK(env.board().owner_token(Cell{2, vx, vy}) == 1);  // via column blocks layer 2
    CHECK(env.board().owner_token(Cell{2, vx, vy}) != 2);  // net1 did not take it

    // Incremental re-route must preserve net0's via column after a step moves net1.
    RouteStats st2 = env.step(/*net_idx=*/1, /*target_pos=*/1, 0.0, 0.0);
    CHECK(st2.drc_violations == 0);
    CHECK(st2.unrouted_count == 0);
    auto [vx2, vy2] = via_col();
    if (vx2 >= 0 && vy2 >= 0)
        CHECK(env.board().owner_token(Cell{2, vx2, vy2}) == 1);
}

TEST_CASE("via_column_preserved_on_tail_ripup") {
    // net0 = a layer-0 prefix net; net1 = a via net at the tail. Re-routing from index 1
    // (route_from(1)) rips up net1 and re-lays its via; its drill column must still be
    // owned by net1 (token 2) on the untouched layer 2 after the incremental re-route.
    RoutingEnv env(3, 20, 20, 1.0, 2.0, 1.0, 0.0);
    SyntheticSpec spec;
    spec.layers = 3; spec.width = 20; spec.height = 20; spec.resolution = 1.0;
    spec.via_cost = 2.0; spec.base_cost = 1.0; spec.design_rule_clearance = 0.0;
    spec.nets = {{{0,2,2},{0,2,8}}, {{0,5,5},{1,5,5}}};
    env.load_synthetic(spec);
    env.reset();
    CHECK(env.board().owner_token(Cell{2,5,5}) == 2);
    RouteStats st = env.board().route_from(1);
    CHECK(st.unrouted_count == 0);
    CHECK(env.board().owner_token(Cell{2,5,5}) == 2);
}

TEST_CASE("thru_hole_pad_blocks_other_layer") {
    // A thru-hole pad spans EVERY copper layer, so its barrel must block a foreign net's
    // copper on a layer we registered it on. net0 pad occupies the same (5,5) on F.Cu and
    // B.Cu; net1 routes a B.Cu horizontal trace that would cross x=5 at y=5 -> it must
    // detour around the pad column instead of shorting through it.
    RoutingEnv env(2, 20, 20, 1.0, 5.0, 1.0, 0.0);
    SyntheticSpec spec;
    spec.layers = 2; spec.width = 20; spec.height = 20; spec.resolution = 1.0;
    spec.via_cost = 5.0; spec.base_cost = 1.0; spec.design_rule_clearance = 0.0;
    // net0: a thru-hole pad (both layers) at (5,5). net1: horizontal trace on B.Cu.
    spec.nets = {{{0,5,5},{1,5,5}}, {{1,1,5},{1,9,5}}};
    env.load_synthetic(spec);
    RouteStats st = env.reset();
    CHECK(st.unrouted_count == 0);
    CHECK(st.drc_violations == 0);
    // The pad column (5,5) is owned by net0 on BOTH layers (token 1) -> not net1's.
    CHECK(env.board().owner_token(Cell{0,5,5}) == 1);
    CHECK(env.board().owner_token(Cell{1,5,5}) == 1);
    CHECK(env.board().owner_token(Cell{1,5,5}) != 2);
    // net1 (B.Cu) must detour around the pad column, not pass through (5,5).
    bool net1_on_pad_col = false;
    for (const auto& path : env.board().nets()[1].segments)
        for (const Cell& c : path) if (c.layer == 1 && c.x == 5 && c.y == 5) net1_on_pad_col = true;
    CHECK_FALSE(net1_on_pad_col);
}

TEST_CASE("via_keepout_spacing") {
    // Stage 8.2: a via claims its keepout (radius-2 axis-aligned square) on every layer, so a later
    // net cannot route through it or place a via/trace within the spacing radius.
    RoutingEnv env(3, 30, 30, 1.0, 2.0, 1.0, 0.0);
    env.board().set_via_radius_cells(2.0);
    SyntheticSpec spec;
    spec.layers = 3; spec.width = 30; spec.height = 30; spec.resolution = 1.0;
    spec.via_cost = 2.0; spec.base_cost = 1.0; spec.design_rule_clearance = 0.0;
    // net0 via at (5,5) spanning layers 0-1; net1 on layer 2 must cross near it.
    spec.nets = {{{0,5,5},{1,5,5}}, {{2,1,5},{2,9,5}}};
    env.load_synthetic(spec);
    RouteStats st = env.reset();
    CHECK(st.unrouted_count == 0);
    CHECK(st.drc_violations == 0);
    // net0 keeps out cells within radius 2 (Chebyshev square) of (5,5) on layer 2.
    CHECK(env.board().owner_token(Cell{2,6,5}) == 1);
    CHECK(env.board().owner_token(Cell{2,6,6}) == 1);
    // net1 must have detoured, not taken the keepout cell.
    CHECK(env.board().owner_token(Cell{2,6,5}) != 2);
}

TEST_CASE("trace_width_swath_blocks_nearby") {
    // Stage 8.3: a trace claims a swath of width/2 around its centerline, so a later
    // net cannot route within its width (a parallel trace is kept apart).
    RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    env.board().set_trace_half_width_cells(1.0);
    SyntheticSpec spec;
    spec.layers = 1; spec.width = 20; spec.height = 20; spec.resolution = 1.0;
    spec.via_cost = 5.0; spec.base_cost = 1.0; spec.design_rule_clearance = 0.0;
    // net0 horizontal trace at y=10 (x 2..8); net1 must cross y=10 near x 4..6.
    spec.nets = {{{0,2,10},{0,8,10}}, {{0,4,6},{0,6,14}}};
    env.load_synthetic(spec);
    RouteStats st = env.reset();
    CHECK(st.unrouted_count == 0);
    CHECK(st.drc_violations == 0);
    // (0,5,9) is one above net0's centerline -> claimed by net0 (token 1).
    CHECK(env.board().owner_token(Cell{0,5,9}) == 1);
    // net1 (token 2) must have detoured around the swath, not taken it.
    CHECK(env.board().owner_token(Cell{0,5,9}) != 2);
}

TEST_CASE("via_emergence_keeps_clear_of_foreign_crossing") {
    // Stage 8.4: a via may only be CREATED where its emergence keepout (radius 2) is
    // free of other nets' copper. net0 lays a copper trace on layer 0 at y=10, x in
    // [2,10]. net1 must connect layer 0 -> layer 1; its via (if it emerged near the
    // trace) would overlap net0 — so the emergence check must push net1's via at least
    // 3 cells away from net0's copper (Chebyshev).
    RoutingEnv env(3, 30, 30, 1.0, 2.0, 1.0, 0.0);
    env.board().set_via_radius_cells(2.0);
    SyntheticSpec spec;
    spec.layers = 3; spec.width = 30; spec.height = 30; spec.resolution = 1.0;
    spec.via_cost = 2.0; spec.base_cost = 1.0; spec.design_rule_clearance = 0.0;
    // net0: layer-0 trace y=10, x 2..10. net1: layer0 y=3 -> layer1 y=17 (needs a via).
    spec.nets = {{{0,2,10},{0,10,10}}, {{0,10,3},{1,10,17}}};
    env.load_synthetic(spec);
    RouteStats st = env.reset();
    CHECK(st.unrouted_count == 0);
    CHECK(st.drc_violations == 0);

    // net0 copper cells: (0, x, 10) for x in [2,10].
    // Every net1 via column must be >= 3 cells (Chebyshev) from this copper.
    const auto& segs = env.board().nets()[1].segments;
    bool via_too_close = false;
    for (const auto& seg : segs) {
        for (size_t i = 1; i < seg.size(); ++i) {
            if (seg[i].layer == seg[i-1].layer) continue; // not a via
            const Cell& v = seg[i]; // via site at (v.x,v.y)
            for (int x = 2; x <= 10; ++x) {
                int ch = std::max(std::abs(x - v.x), std::abs(10 - v.y));
                if (ch <= 2) { via_too_close = true; }
            }
        }
    }
    CHECK_FALSE(via_too_close);
}

TEST_CASE("drc_trace_trace_clean_or_flagged") {
    // Stage 8.4b: trace-vs-trace spacing driven by the copper_ flag (not keepout margin).
    // Two real-copper cells of different nets within 2*tw+clr => DRC flags; properly
    // spaced => clean. Builds the layout by hand via add_manual_trace.
    // clr = drc_clearance_cells() = max(1, 0) = 1; tw=1 => spacing = 2*1 + 1 = 3.
    {
        Board b(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
        b.set_trace_half_width_cells(1.0);
        b.add_manual_trace({0,5,5}, 0);   // net0 copper
        b.add_manual_trace({0,6,5}, 1);   // net1 copper 1 cell away (< spacing)
        RouteStats s = b.check_drc();
        CHECK(s.drc_violations >= 1);
    }
    {
        Board b(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
        b.set_trace_half_width_cells(1.0);
        b.add_manual_trace({0,5,5}, 0);   // net0 copper
        b.add_manual_trace({0,8,5}, 1);   // net1 copper 3 cells away (>= spacing)
        RouteStats s = b.check_drc();
        CHECK(s.drc_violations == 0);
    }
}

TEST_CASE("drc_trace_trace_catches_routed_copper") {
    // BLOCKER-1 regression: copper_ must be set on EVERY real path cell (not only the
    // first, which was the effect of the self-coating guard). Route a real net with
    // tw=1 (spacing 2*1+1=3), then place a foreign copper cell near the MIDDLE of its
    // trace (distance 2 < 3) and require DRC to flag it. Under the bug, mid-trace
    // copper_ was 0, so this was missed.
    Board b(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    b.set_trace_half_width_cells(1.0);
    Net n0; n0.id = 0; n0.pins = {{0,2,10},{0,8,10}};
    b.nets().push_back(n0);
    b.add_pad(0, {0,2,10}); b.add_pad(0, {0,8,10});
    RouteStats st0 = b.route_all();
    CHECK(st0.unrouted_count == 0);
    // foreign net1 copper 2 cells above the MIDDLE (x=5) of net0's trace
    b.add_manual_trace({0,5,8}, 1);
    RouteStats s = b.check_drc();
    CHECK(s.drc_violations >= 1);
}

TEST_CASE("drc_own_pad_not_flagged_as_trace") {
    // MAJOR regression: a segment-endpoint pad cell must NOT be judged by the trace-trace
    // rule (2*tw+clr) — it has its own pad-trace threshold (clr+tw+pad_radius). A foreign
    // copper cell at the legal pad-trace distance (3 here, with pad radius 0.3 @ res1.0)
    // must be clean; only closer-than-that must flag.
    Board b(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    b.set_trace_half_width_cells(1.0);
    b.add_manual_trace({0,5,5}, 0);  // pad cell of net0 (pad_owner set by add_pad)
    b.add_pad(0, {0,5,5});
    b.add_manual_trace({0,5,8}, 1);  // foreign net1 copper at Euclidean distance 3
    RouteStats s = b.check_drc();
    CHECK(s.drc_violations == 0);   // pad-trace: 3 >= clr+tw+pad_radius(2.3)
}

TEST_CASE("pad_inflation_routing_spacing") {
    // Stage 8.4c: with physics on (tw=1), the router itself must keep trace-width +
    // clearance from a FOREIGN pad at search time, not just be flagged by DRC afterwards.
    // net0 has a pad at (5,5); net1 routes at y=6 (passing 1 below — strictly inside the
    // clr + tw = 2 Euclidean keepout). With physics on net1 must DETOUR around net0's pad
    // (longer total length and its real trace stays off the inflated cell). A crossing at
    // EXACTLY distance 2 is legal: the shape-true keepout is Euclidean and agrees with
    // DRC's strict-less threshold (the old Chebyshev square over-blocked the boundary).
    auto run = [](double tw) {
        RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
        SyntheticSpec spec;
        spec.layers = 1; spec.width = 20; spec.height = 20; spec.resolution = 1.0;
        spec.via_cost = 5.0; spec.base_cost = 1.0; spec.design_rule_clearance = 0.0;
        spec.nets = {{{0,5,5}}, {{0,1,6},{0,9,6}}};
        env.load_synthetic(spec);
        env.board().set_trace_half_width_cells(tw);
        RouteStats st = env.reset();
        return std::make_pair(st, env);
    };
    // Physics ON: net1 detours (length > straight-line 8) and its real copper avoids (5,6).
    {
        auto [st, env] = run(1.0);
        CHECK(st.unrouted_count == 0);
        CHECK(st.drc_violations == 0);
        CHECK(st.per_net_length[1] > 8.0 + 1e-9);  // detoured around net0's pad
        bool on_inflated = false;
        for (const auto& seg : env.board().nets()[1].segments)
            for (const Cell& cc : seg) if (cc == Cell{0,5,6}) on_inflated = true;
        CHECK_FALSE(on_inflated);
    }
}

TEST_CASE("pad_inflation_reaches_own_pad") {
    // Regression: a net's OWN pad inside a FOREIGN pad's inflation must still be reachable
    // (its exact pad cell is exempt from pad-inflation blocking). net0 pad at (5,5) inflates
    // a radius-3 square (incl. pad radius). net1 has a pad at (8,5) on the inflation edge
    // and must reach it from the far pad (10,10); (8,5) exits via x=9 (outside), so net1
    // must route. Without the own-pad exemption the goal cell is blocked -> unroutable.
    RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
    env.board().set_trace_half_width_cells(1.0);
    SyntheticSpec spec;
    spec.layers = 1; spec.width = 20; spec.height = 20; spec.resolution = 1.0;
    spec.via_cost = 5.0; spec.base_cost = 1.0; spec.design_rule_clearance = 0.0;
    spec.nets = {{{0,5,5}}, {{0,8,5},{0,10,10}}};
    env.load_synthetic(spec);
    RouteStats st = env.reset();
    CHECK(st.unrouted_count == 0);
}

TEST_CASE("parallel_envs_identical") {
    // Stage 6.3: many independent env instances over the SAME board run on separate
    // threads must each reproduce the single-threaded result (parallel-session
    // readiness). Any shared global mutable state (or the removed thread-local A*
    // scratch) would break this.
    SyntheticSpec spec = small_board();
    // Reference: sequential result.
    auto run = [&]() {
        RoutingEnv env(1, 20, 20, 1.0, 5.0, 1.0, 0.0);
        env.load_synthetic(spec);
        env.reset();
        return env.step(2, 1, 1.0, 2.0);
    };
    RouteStats ref = run();
    std::vector<RouteStats> got;
    const int N = 8;
    got.resize((size_t)N);
    std::vector<std::thread> threads;
    for (int t = 0; t < N; ++t) {
        threads.emplace_back([&, t]() { got[(size_t)t] = run(); });
    }
    for (auto& th : threads) th.join();
    for (int t = 0; t < N; ++t) {
        REQUIRE(got[(size_t)t].per_net_length.size() == ref.per_net_length.size());
        CHECK(got[(size_t)t].unrouted_count == ref.unrouted_count);
        for (size_t i = 0; i < ref.per_net_length.size(); ++i)
            CHECK(got[(size_t)t].per_net_length[i] == doctest::Approx(ref.per_net_length[i]));
    }
}

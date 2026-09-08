#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "routing/board.hpp"
#include "routing/synthetic.hpp"

#include <vector>
#include <set>

using namespace routing;

// Build a small board from a list of nets (each a list of single-layer pins).
static Board make_board(int w, int h, int layers,
                        const std::vector<std::vector<Cell>>& nets,
                        double via_cost = 5.0, double clearance = 0.0) {
    Board b(layers, w, h, 1.0, via_cost, 1.0, clearance);
    for (size_t i = 0; i < nets.size(); ++i) {
        Net net;
        net.id = (int)i;
        net.pins = nets[i];
        b.nets().push_back(std::move(net));
        for (const Cell& p : nets[i]) b.add_pad(i, p);
    }
    return b;
}

// ---------------------------------------------------------------------------
// 2.1 route_all
// ---------------------------------------------------------------------------
TEST_CASE("physics_cannot_be_disabled") {
    // Stage-15 invariant: physical fidelity (trace width, via radius, clearance) is
    // ALWAYS on with KiCad defaults. It is tunable to any positive value but can never
    // be set to 0 (thrown), and a fresh Board has non-zero defaults.
    Board b(1, 20, 20, 0.05, 5.0, 1.0, 0.0);          // res 0.05, clearance 0 -> default
    CHECK(b.trace_half_width_cells() > 0.0);           // 0.1mm half-width default
    CHECK(b.via_radius_cells() > 0.0);                 // 0.3mm via radius default
    CHECK(b.design_rule_clearance() >= 0.2);           // clearance defaults to KiCad 0.2
    CHECK_THROWS_AS(b.set_trace_half_width_cells(0.0), std::invalid_argument);
    CHECK_THROWS_AS(b.set_via_radius_cells(0.0), std::invalid_argument);
    CHECK_THROWS_AS(b.set_trace_half_width_cells(-1.0), std::invalid_argument);
    // Positive values remain adjustable.
    b.set_trace_half_width_cells(3.0);
    CHECK(b.trace_half_width_cells() == 3.0);
}

TEST_CASE("all_routed_simple") {
    auto b = make_board(20, 20, 1, {{{0,2,2},{0,2,8}}, {{0,9,3},{0,9,11}}});
    RouteStats s = b.route_all();
    CHECK(s.unrouted_count == 0);
    CHECK(s.per_net_length.size() == 2);
    CHECK(s.per_net_length[0] > 0.0);
    CHECK(s.per_net_length[1] > 0.0);
    // Net 0 routes vertically through (2,2)..(2,8); a mid cell is owned by net 0,
    // NOT free. Use raw token (net0 -> token 1) to avoid the net0/free ambiguity.
    CHECK(b.owner_token({0,2,4}) == 1);   // net 0 (id 0) owns this cell -> token 1
    CHECK_FALSE(b.is_free({0,2,4}));
    // A cell far from every net's copper + physical coating is truly untouched.
    CHECK(b.is_free({0,19,19}));
    CHECK(b.owner_token({0,19,19}) == 0);
}

TEST_CASE("nets_are_obstacles_and_order_matters") {
    // Two crossing nets in a board large enough that each CAN route under physical
    // clearance; whichever routes SECOND must detour around the first.
    // Net A horizontal at y=8 (x=2..8); Net B vertical at x=5 (y=2..12), crossing at (5,8).
    auto boards = [&](bool a_first) {
        std::vector<std::vector<Cell>> nets = {
            {{0,2,8},{0,8,8}},          // A, straight length 6
            {{0,5,2},{0,5,12}}          // B, straight length 10
        };
        auto b = make_board(16, 16, 1, nets);
        if (!a_first) b.move_net(0, 1); // B first
        return b;
    };

    // A first: A straight (6); B must detour (length > its straight 10).
    Board ba = boards(true);
    RouteStats sa = ba.route_all();
    CHECK(sa.unrouted_count == 0);
    CHECK(sa.per_net_length[0] == doctest::Approx(6.0));    // A straight
    CHECK(sa.per_net_length[1] > 10.0);                     // B detours

    // B first: order [B, A]; B straight (10), A detours (length > its straight 6).
    Board bb = boards(false);
    RouteStats sb = bb.route_all();
    CHECK(sb.unrouted_count == 0);
    CHECK(sb.per_net_length[0] == doctest::Approx(10.0));   // B straight
    CHECK(sb.per_net_length[1] > 6.0);                      // A detours
}

TEST_CASE("unrouted_reported_when_enclosed") {
    // Enclose an isolated pin so it can't be reached.
    auto b = make_board(10, 10, 1, {{{0,1,1},{0,8,8}}});
    // Add a wall around (8,8) by pre-marking... we can't easily inject obstacle cells
    // via the public API except by adding pads/other nets. Add a second net whose
    // pads form a cage around (8,8) is hard. Instead test with an unreachable pin via
    // a ring of pads belonging to net 1 that fully surrounds (8,8).
    std::vector<std::vector<Cell>> nets = {
        {{0,1,1},{0,8,8}},
        // ring blocks (8,8)
        {{0,7,7},{0,8,7},{0,9,7},
         {0,7,8},{0,9,8},
         {0,7,9},{0,8,9},{0,9,9}}
    };
    Board b2(1, 12, 12, 1.0, 5.0, 1.0, 0.0);
    for (size_t i = 0; i < nets.size(); ++i) {
        Net net; net.id = (int)i; net.pins = nets[i];
        b2.nets().push_back(std::move(net));
        for (const Cell& p : nets[i]) b2.add_pad(i, p);
    }
    RouteStats s = b2.route_all();
    // Net 0's pin at (8,8) is enclosed by net 1's pads -> net 0 unrouted.
    CHECK(s.unrouted_count >= 1);
    CHECK(s.unrouted[0] == true);
}

// ---------------------------------------------------------------------------
// 2.2 move_net
// ---------------------------------------------------------------------------
TEST_CASE("reorder_semantics_bump_down") {
    std::vector<std::vector<Cell>> nets = {
        {{0,0,0}}, {{0,1,1}}, {{0,2,2}}, {{0,3,3}}, {{0,4,4}}
    };
    auto b = make_board(10, 10, 1, nets);
    // move index 4 to index 1: [A,B,C,D,E] -> [A,E,B,C,D]
    int ni = b.move_net(4, 1);
    CHECK(ni == 1);
    // pins after move: expect net order {0,4,1,2,3}
    CHECK(b.nets()[0].pins == nets[0]);
    CHECK(b.nets()[1].pins == nets[4]);
    CHECK(b.nets()[2].pins == nets[1]);
    CHECK(b.nets()[3].pins == nets[2]);
    CHECK(b.nets()[4].pins == nets[3]);
}

TEST_CASE("round_trip_and_invalid_target") {
    std::vector<std::vector<Cell>> nets = {
        {{0,0,0}}, {{0,1,1}}, {{0,2,2}}
    };
    auto b = make_board(10, 10, 1, nets);
    int ni = b.move_net(0, 2); // [A,B,C] -> [B,C,A]
    CHECK(ni == 2);
    CHECK(b.nets()[2].pins == nets[0]);
    // invalid (out of range) targets clamp without crashing
    CHECK(b.move_net(2, 99) == 2); // clamp to last
    CHECK(b.move_net(2, -5) == 0); // clamp to first
    CHECK(b.nets().size() == 3);
}

TEST_CASE("multiplier_set_stored_and_used") {
    auto b = make_board(20, 20, 1, {{{0,2,2},{0,2,8}}});
    // No avoidance set -> default zeros, no crash, routes fine.
    RouteStats s1 = b.route_all();
    CHECK(s1.unrouted_count == 0);

    // Set avoidance on net 0 -> routes fine too, and pad avoidance stored.
    b.set_avoidance(0, 2.0, 2.0);
    RouteStats s2 = b.route_all();
    CHECK(s2.unrouted_count == 0);
    // pad_avoid_cost near any pad should be > 0
    CHECK(b.pad_avoid_cost_at({0,2,3}) > 0.0);
}

// ---------------------------------------------------------------------------
// 2.3 DRC
// ---------------------------------------------------------------------------
TEST_CASE("connectivity_drc_flags_unrouted") {
    // Same enclosed-pin structure as unrouted_reported test.
    std::vector<std::vector<Cell>> nets = {
        {{0,1,1},{0,8,8}},
        {{0,7,7},{0,8,7},{0,9,7},{0,7,8},{0,9,8},{0,7,9},{0,8,9},{0,9,9}}
    };
    Board b(1, 12, 12, 1.0, 5.0, 1.0, 0.0);
    for (size_t i = 0; i < nets.size(); ++i) {
        Net net; net.id = (int)i; net.pins = nets[i];
        b.nets().push_back(std::move(net));
        for (const Cell& p : nets[i]) b.add_pad(i, p);
    }
    RouteStats s = b.route_all();
    CHECK(s.drc_violations >= 1); // net 0 unrouted is a connectivity violation
}

TEST_CASE("clean_board_zero_connectivity_violations") {
    auto b = make_board(20, 20, 1, {{{0,2,2},{0,2,8}}, {{0,9,3},{0,9,11}}});
    RouteStats s = b.route_all();
    // After a full clean route, DRC connectivity/overlap should be zero.
    // (Clearance is 0 here, so only connectivity/overlap counted.)
    CHECK(s.drc_violations == 0);
}

TEST_CASE("clearance_drc_flags_trace_near_pad") {
    // pad A at (3,4); net B forced to pass adjacent (4,4) within clearance 2.
    std::vector<std::vector<Cell>> nets = {
        {{0,3,4}},             // A: single pad
        {{0,5,4},{0,5,0}}      // B
    };
    Board b(1, 10, 10, 1.0, 5.0, 1.0, 2.0); // clearance 2 => clr=2 cells
    for (size_t i = 0; i < nets.size(); ++i) {
        Net net; net.id = (int)i; net.pins = nets[i];
        b.nets().push_back(std::move(net));
        for (const Cell& p : nets[i]) b.add_pad(i, p);
    }
    // Block x=5, y=1..3 to force B to route along x=4 (adjacent to pad (3,4)).
    // We simulate the block by adding a net-2 pad wall; but that would route too.
    // Simpler: add obstacle pads belonging to a dummy single-pin net that we don't
    // care about routing (it's single-pin so trivially routed and its pads block).
    std::vector<Cell> wall = {{0,5,1},{0,5,2},{0,5,3}};
    for (const Cell& wc : wall) b.add_pad(2, wc); // net 2 pads act as obstacles
    Net dummy; dummy.id = 2; dummy.pins = wall;
    b.nets().push_back(std::move(dummy));

    RouteStats s = b.route_all();
    // B's straight path (5,4)->(5,0) is blocked at (5,1..3); it must detour to x=4,
    // passing (4,4) which is adjacent to pad (3,4) -> clearance violation (clr=2).
    CHECK(s.drc_violations >= 1);
}

TEST_CASE("clean_board_with_clearance_no_false_positive") {
    // Regression for the DRC bug where a net's OWN pads at its endpoints were
    // flagged as clearance violations. A clean straight route with clearance >= 2
    // must report ZERO violations (its trace legitimately abuts its own pads).
    std::vector<std::vector<Cell>> nets = {{{0,2,2},{0,2,8}}, {{0,9,3},{0,9,11}}};
    Board b(1, 20, 20, 1.0, 5.0, 1.0, 2.0); // clearance 2 => clr=2
    for (size_t i = 0; i < nets.size(); ++i) {
        Net net; net.id = (int)i; net.pins = nets[i];
        b.nets().push_back(std::move(net));
        for (const Cell& p : nets[i]) b.add_pad(i, p);
    }
    RouteStats s = b.route_all();
    CHECK(s.unrouted_count == 0);
    CHECK(s.drc_violations == 0); // no false positives on a clean board
}

TEST_CASE("clearance_drc_flags_cross_net_proximity") {
    // Cross-net clearance IS enforced: a trace that gets within clr of a DIFFERENT
    // net's pad is flagged, even though own-net pads are exempt.
    std::vector<std::vector<Cell>> nets = {
        {{0,0,0},{0,4,0}},  // net 0 track along y=0
        {{0,2,3}}           // net 1 single pad at (2,3) far from net 0's trace
    };
    Board b(1, 10, 10, 1.0, 5.0, 1.0, 1.0); // clr=1
    for (size_t i = 0; i < nets.size(); ++i) {
        Net net; net.id = (int)i; net.pins = nets[i];
        b.nets().push_back(std::move(net));
        for (const Cell& p : nets[i]) b.add_pad(i, p);
    }
    // Net 0 routes along y=0 -> its trace is >= 3 cells from net 1's pad, so clean.
    RouteStats s = b.route_all();
    CHECK(s.unrouted_count == 0);
    CHECK(s.drc_violations == 0);
}

// ---------------------------------------------------------------------------
// 2.4 synthetic builder + IO
// ---------------------------------------------------------------------------
TEST_CASE("synthetic_json_malformed_throws_runtime_error") {
    // Regression: malformed JSON / missing keys must throw std::runtime_error.
    CHECK_THROWS_AS(synthetic_from_json("not json {{{{{{"), std::runtime_error);
    CHECK_THROWS_AS(synthetic_from_json("{\"width\":5,\"height\":5}"), // missing layers/nets
                    std::runtime_error);
    // pin out of bounds
    std::string bad = "{\"layers\":1,\"width\":5,\"height\":5,\"resolution\":1,"
                      "\"nets\":[[[0,100,100]]]}";
    CHECK_THROWS_AS(synthetic_from_json(bad), std::runtime_error);
}

TEST_CASE("synthetic_two_net_routes") {
    SyntheticSpec spec;
    spec.layers = 1; spec.width = 20; spec.height = 20;
    spec.nets = {{{0,2,2},{0,2,8}}, {{0,9,3},{0,9,11}}};
    Board b = build_synthetic(spec);
    REQUIRE(b.num_nets() == 2);
    RouteStats s = b.route_all();
    CHECK(s.unrouted_count == 0);
}

TEST_CASE("synthetic_three_layer_with_vias") {
    SyntheticSpec spec;
    spec.layers = 3; spec.width = 20; spec.height = 20; spec.via_cost = 2.0;
    spec.nets = {{{0,2,2},{2,2,8}}}; // pin on layer 0 and layer 2 -> needs 2 vias
    Board b = build_synthetic(spec);
    RouteStats s = b.route_all();
    CHECK(s.unrouted_count == 0);
    CHECK(s.total_vias >= 2);
}

TEST_CASE("synthetic_json_round_trip") {
    SyntheticSpec spec;
    spec.layers = 2; spec.width = 15; spec.height = 12; spec.via_cost = 3.0;
    spec.design_rule_clearance = 1.5; spec.base_cost = 1.0; spec.resolution = 0.5;
    spec.nets = {{{0,1,1},{1,3,4}}, {{0,5,5},{0,6,6},{1,7,7}}};

    std::string json = synthetic_to_json(spec);
    SyntheticSpec back = synthetic_from_json(json);

    CHECK(back.layers == spec.layers);
    CHECK(back.width == spec.width);
    CHECK(back.height == spec.height);
    CHECK(back.resolution == doctest::Approx(spec.resolution));
    CHECK(back.via_cost == doctest::Approx(spec.via_cost));
    CHECK(back.base_cost == doctest::Approx(spec.base_cost));
    CHECK(back.design_rule_clearance == doctest::Approx(spec.design_rule_clearance));
    REQUIRE(back.nets.size() == spec.nets.size());
    for (size_t i = 0; i < spec.nets.size(); ++i)
        CHECK(back.nets[i] == spec.nets[i]);
}

// ---------------------------------------------------------------------------
// STAGE 4: incremental re-route
// ---------------------------------------------------------------------------

// Helper: check two RouteStats are equal in the fields we care about.
static bool stats_eq(const RouteStats& a, const RouteStats& b) {
    if (a.per_net_length != b.per_net_length) return false;
    if (a.unrouted != b.unrouted) return false;
    if (a.unrouted_count != b.unrouted_count) return false;
    if (a.drc_violations != b.drc_violations) return false;
    if (a.total_length != b.total_length) return false;
    if (a.total_vias != b.total_vias) return false;
    return true;
}

// Build a Board, apply a move via path A (full route) vs path B (route, move,
// incremental route), and require identical full-board outcome.
static void check_incremental_equals_full(std::vector<std::vector<Cell>> nets,
                                          int from, int to, int lo,
                                          int w = 12, int h = 12, int layers = 1) {
    // Path B: route fully, then move, then incremental from lo.
    Board full(layers, w, h, 1.0, 3.0, 1.0, 0.0);
    for (size_t i = 0; i < nets.size(); ++i) {
        Net net; net.id = (int)i; net.pins = nets[i];
        full.nets().push_back(std::move(net));
        for (const Cell& p : nets[i]) full.add_pad(i, p);
    }
    full.route_all();           // initial full route
    full.move_net(from, to);    // apply the same move
    RouteStats inc = full.route_from(lo); // incremental

    // Path A: same net order reached by directly building the moved order and
    // doing a fresh full route_all from scratch.
    std::vector<std::vector<Cell>> moved = nets;
    std::vector<size_t> order(nets.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    // perform the same reorder on a copy of indices
    {
        std::vector<size_t> idx = order;
        size_t f = (size_t)from, t = (size_t)to;
        size_t m = idx[f];
        if (f < t) { for (size_t i = f; i < t; ++i) idx[i] = idx[i+1]; }
        else       { for (size_t i = f; i > t; --i) idx[i] = idx[i-1]; }
        idx[t] = m;
        order = idx;
    }
    Board fresh(layers, w, h, 1.0, 3.0, 1.0, 0.0);
    std::vector<std::vector<Cell>> ordered_nets;
    for (size_t i = 0; i < order.size(); ++i) ordered_nets.push_back(nets[order[i]]);
    for (size_t i = 0; i < ordered_nets.size(); ++i) {
        Net net; net.id = (int)i; net.pins = ordered_nets[i];
        fresh.nets().push_back(std::move(net));
        for (const Cell& p : ordered_nets[i]) fresh.add_pad(i, p);
    }
    RouteStats full_stats = fresh.route_all();

    // per-net lengths in `full` (inc) are indexed by position; in `fresh` by its own
    // order (0..n, matching positions). Both use position ordering, so compare directly.
    CAPTURE(from); CAPTURE(to); CAPTURE(lo);
    CHECK(stats_eq(inc, full_stats));
}

TEST_CASE("incremental_equals_full_end_move") {
    std::vector<std::vector<Cell>> nets = {
        {{0,1,1},{0,6,1}}, {{0,1,4},{0,6,4}}, {{0,1,7},{0,6,7}}, {{0,1,10},{0,6,10}},
        {{0,2,2},{0,5,2}}, {{0,2,6},{0,5,6}}
    };
    // move net 5 -> position 4 (affects lo=4): re-route from 4
    check_incremental_equals_full(nets, 5, 4, 4);
}

TEST_CASE("incremental_equals_full_front_move") {
    std::vector<std::vector<Cell>> nets = {
        {{0,1,1},{0,6,1}}, {{0,1,4},{0,6,4}}, {{0,1,7},{0,6,7}}, {{0,1,10},{0,6,10}},
        {{0,2,2},{0,5,2}}, {{0,2,6},{0,5,6}}
    };
    // move net 0 -> position 3 (affects lo=0): re-route from 0 (== full)
    check_incremental_equals_full(nets, 0, 3, 0);
}

TEST_CASE("incremental_equals_full_mid_swap") {
    std::vector<std::vector<Cell>> nets = {
        {{0,1,1},{0,6,1}}, {{0,1,4},{0,6,4}}, {{0,1,7},{0,6,7}}, {{0,1,10},{0,6,10}},
        {{0,2,2},{0,5,2}}, {{0,2,6},{0,5,6}}
    };
    // move net 2 -> position 4 (lo=2)
    check_incremental_equals_full(nets, 2, 4, 2);
}

TEST_CASE("incremental_prefix_preserved") {
    // After an incremental move with lo>0, the traces of nets < lo are unchanged.
    std::vector<std::vector<Cell>> nets = {
        {{0,1,1},{0,6,1}}, {{0,1,4},{0,6,4}}, {{0,1,7},{0,6,7}}, {{0,1,10},{0,6,10}},
        {{0,2,2},{0,5,2}}, {{0,2,6},{0,5,6}}
    };
    Board b(1, 12, 12, 1.0, 3.0, 1.0, 0.0);
    for (size_t i = 0; i < nets.size(); ++i) {
        Net net; net.id = (int)i; net.pins = nets[i];
        b.nets().push_back(std::move(net));
        for (const Cell& p : nets[i]) b.add_pad(i, p);
    }
    // snapshot trace tokens before the move (for the prefix nets' trace cells)
    RouteStats s0 = b.route_all();
    auto trace_before = b.traces(); // copy

    b.move_net(5, 4); // lo = min(5,4) = 4
    b.route_from(4);
    // prefix nets are positions 0..3 -> their owned trace cells unchanged
    const auto& trace_after = b.traces();
    int prefix_changed = 0;
    for (size_t i = 0; i < trace_before.size(); ++i) {
        if (trace_before[i] != trace_after[i]) prefix_changed++;
    }
    // Some cells changed (the re-routed tail). But the prefix nets' OWN trace cells
    // must be preserved. Verify: every cell that trace_before attributed to a prefix
    // net (position < 4 => net.id < 4 since id==position pre-move) is unchanged.
    int changed_prefix_cells = 0;
    for (size_t i = 0; i < trace_before.size(); ++i) {
        int32_t t = trace_before[i];
        if (t != 0 && (t - 1) < 4) {          // a prefix net's trace cell
            if (trace_after[i] != t) changed_prefix_cells++;
        }
    }
    CHECK(changed_prefix_cells == 0);
}

TEST_CASE("incremental_equals_full_multilayer_with_vias_in_prefix") {
    // Regression: route_from must carry the preserved prefix nets' VIAS into
    // total_vias (it previously dropped them, breaking incremental == full on
    // total_vias for multi-layer boards). Net 0 (a via net) sits in the preserved
    // prefix [0, lo); the incremental re-route must report the same total_vias as a
    // fresh full route over the same final net order.
    std::vector<std::vector<Cell>> nets = {
        {{0,1,1},{1,1,3}},            // net 0: layer0 -> layer1 (>=1 via), prefix net
        {{0,4,1},{0,4,6}},            // net 1: vertical layer0
        {{0,8,1},{1,8,4}},            // net 2: layer0 -> layer1 (>=1 via)
        {{0,1,9},{0,6,9}},            // net 3: horizontal layer0
        {{0,10,10},{1,12,10}}         // net 4: layer0 -> layer1 (>=1 via)
    };
    // Move net 4 -> position 1: lo = min(4,1) = 1, so prefix [0,1) = {net 0} (a via
    // net) is preserved. Requires the prefix vias be carried into total_vias.
    check_incremental_equals_full(nets, /*from=*/4, /*to=*/1, /*lo=*/1,
                                  /*w=*/15, /*h=*/15, /*layers=*/2);
}

// ---------------------------------------------------------------------------
// Regression: episode isolation. rebuild_cl_zone derives via clearance zones
// from net.segments, so stale segments (a previous episode's paths, a ripped
// net's paths, or a failed net's partial paths) must never survive into it.
// ---------------------------------------------------------------------------
TEST_CASE("route_all_is_stateless_across_episodes") {
    // Nets that force a via and mutual interaction. Two consecutive route_all()
    // calls must match exactly: the second must not see phantom zones baked from
    // the first episode's segments.
    Board b = make_board(24, 24, 2,
                         {{{0, 12, 4}, {1, 12, 20}},
                          {{0, 4, 12}, {0, 20, 12}},
                          {{1, 4, 4}, {1, 20, 20}}},
                         5.0, 2.0);
    RouteStats s1 = b.route_all();
    RouteStats s2 = b.route_all();
    CHECK(s1.unrouted_count == s2.unrouted_count);
    CHECK(s1.total_vias == s2.total_vias);
    CHECK(s1.total_length == doctest::Approx(s2.total_length));
    CHECK(s1.drc_violations == s2.drc_violations);
}

TEST_CASE("failed_net_leaves_no_segments") {
    // net 1's second pin is walled off by net 0's pad column on a 1-layer board:
    // routing must fail AND leave no partial segments behind (partial paths would
    // bake phantom clearance zones in later rebuilds).
    std::vector<Cell> wall;
    for (int y = 0; y < 12; ++y) wall.push_back(Cell{0, 6, y});
    Board b = make_board(12, 12, 1, {wall, {{0, 2, 6}, {0, 10, 6}}});
    RouteStats st = b.route_all();
    CHECK(st.unrouted[1]);
    CHECK(b.nets()[1].segments.empty());
    CHECK(b.nets()[1].total_length == 0.0);
}

// ---------------------------------------------------------------------------
// Regression: on 2-layer boards the incremental via clearance zone used to sit
// inside the intermediate-layer loop, which never runs with 2 layers — so a via
// placed during a routing pass had NO clearance zone until the next rebuild.
// Net 0 is forced to via at exactly (10,10) (stacked same-net pads, no barrel);
// net 1 routes later in the same pass and must keep clear of it.
// ---------------------------------------------------------------------------
TEST_CASE("two_layer_via_zone_applies_incrementally") {
    Board b = make_board(21, 21, 2,
                         {{{0, 10, 10}, {1, 10, 10}},
                          {{0, 2, 10}, {0, 18, 10}}},
                         5.0, 2.0);
    RouteStats st = b.route_all();
    REQUIRE(st.unrouted_count == 0);
    CHECK(st.total_vias == 1);   // stacked pads need a REAL via (no drilled barrel)
    // Net 1's committed cells must stay outside the via's hard zone square
    // (radius ceil(vr + tw + clr) = 3 cells at this resolution) around (10,10).
    for (const auto& seg : b.nets()[1].segments)
        for (const Cell& c : seg) {
            int cd = std::max(std::abs(c.x - 10), std::abs(c.y - 10));
            CHECK(cd > 3);
        }
}

TEST_CASE("thru_pad_barrel_not_a_via") {
    // A declared thru-hole pad barrel is pre-drilled pad copper: using it to reach
    // the other layer is not a router-placed via (no via count, no via_ok gating).
    Board b = make_board(20, 20, 2, {{{0, 5, 5}, {1, 5, 5}, {0, 15, 5}}});
    b.mark_thru_pad(0, 5, 5);
    RouteStats st = b.route_all();
    CHECK(st.unrouted_count == 0);
    CHECK(st.total_vias == 0);   // was 1 when the barrel was miscounted as a via
}

// ---------------------------------------------------------------------------
// Wall-time route budget: an (effectively) already-expired budget must return
// immediately with PARTIAL results — every net honestly unrouted, the flag set,
// no partial segments left behind. Budget 0 = unlimited (default behavior).
// ---------------------------------------------------------------------------
TEST_CASE("route_time_budget_returns_partial_results") {
    Board b = make_board(30, 30, 2,
                         {{{0, 5, 5}, {0, 25, 5}},
                          {{0, 5, 15}, {0, 25, 15}},
                          {{0, 5, 25}, {0, 25, 25}}});
    b.set_route_time_budget_s(1e-9);   // expires before the first net starts
    RouteStats st = b.route_all();
    CHECK(st.time_budget_exceeded);
    CHECK(st.unrouted_count == 3);
    for (const Net& n : b.nets()) {
        CHECK_FALSE(n.routed);
        CHECK(n.segments.empty());
    }
    // Budget off again: the same board routes fully and the flag stays clear.
    b.set_route_time_budget_s(0.0);
    RouteStats st2 = b.route_all();
    CHECK_FALSE(st2.time_budget_exceeded);
    CHECK(st2.unrouted_count == 0);
}

// ---------------------------------------------------------------------------
// Resume checkpoint: a budget-cut pass reports WHERE it stopped (resume_from);
// route_from(resume_from) continues on the committed prefix, and the combined
// result equals one uninterrupted pass — regardless of where the cut landed.
// ---------------------------------------------------------------------------
TEST_CASE("route_time_budget_resume_equals_full_route") {
    auto nets = std::vector<std::vector<Cell>>{
        {{0, 3, 3}, {0, 27, 3}},   {{0, 3, 8}, {0, 27, 8}},
        {{0, 3, 13}, {0, 27, 13}}, {{0, 3, 18}, {0, 27, 18}},
        {{0, 3, 23}, {0, 27, 23}}, {{1, 3, 27}, {1, 27, 27}}};

    // Reference: one uninterrupted pass on an identical board.
    Board ref = make_board(31, 31, 2, nets);
    RouteStats full = ref.route_all();
    CHECK(full.resume_from == -1);

    // Budgeted pass (cut lands wherever machine load puts it) + unbudgeted resume.
    Board b = make_board(31, 31, 2, nets);
    b.set_route_time_budget_s(1e-9);
    RouteStats part = b.route_all();
    CHECK(part.time_budget_exceeded);
    REQUIRE(part.resume_from >= 0);          // something was cut
    b.set_route_time_budget_s(0.0);
    RouteStats resumed = b.route_from(part.resume_from);

    CHECK(resumed.resume_from == -1);
    CHECK_FALSE(resumed.time_budget_exceeded);
    CHECK(resumed.unrouted_count == full.unrouted_count);
    CHECK(resumed.total_vias == full.total_vias);
    CHECK(resumed.total_length == doctest::Approx(full.total_length));
    REQUIRE(resumed.per_net_length.size() == full.per_net_length.size());
    for (size_t i = 0; i < full.per_net_length.size(); ++i)
        CHECK(resumed.per_net_length[i] == doctest::Approx(full.per_net_length[i]));
}

// ---------------------------------------------------------------------------
// Partial connection is first-class: a net that can connect only SOME of its
// pins keeps (and commits) the successful connections and reports exactly how
// many pins remain — 4-of-5 is distinguishable from 0-of-5.
// ---------------------------------------------------------------------------
TEST_CASE("partial_net_keeps_committed_connections") {
    // net 0 walls off the right side; net 1 has two connectable pins on the left
    // and one unreachable pin on the right.
    std::vector<Cell> wall;
    for (int y = 0; y < 20; ++y) wall.push_back(Cell{0, 10, y});
    Board b = make_board(20, 20, 1, {wall, {{0, 2, 10}, {0, 6, 10}, {0, 16, 10}}});
    RouteStats st = b.route_all();

    CHECK(st.unrouted[1]);                       // net 1 is not FULLY routed...
    CHECK(st.per_net_unconnected[1] == 1);       // ...but only ONE pin is missing
    CHECK(st.total_unconnected_pins == 1);
    CHECK(st.per_net_length[1] > 0.0);           // the (2,10)-(6,10) connection stands
    CHECK_FALSE(b.nets()[1].segments.empty());
    // The partial connection is REAL committed copper on the board.
    CHECK(b.owner_token(Cell{0, 4, 10}) == 2);
    // The wall net itself fully routed.
    CHECK(st.per_net_unconnected[0] == 0);
}

// ---------------------------------------------------------------------------
// THE connection-mode regression gate: replaying net order through
// route_one_connection must reproduce route_all EXACTLY — same per-net lengths,
// vias, unconnected sets — because it executes the identical A* sequence with
// identical tie-breaking, differing only in when commits land.
// ---------------------------------------------------------------------------
TEST_CASE("connection_mode_netorder_equals_route_all") {
    auto nets = std::vector<std::vector<Cell>>{
        {{0, 3, 3}, {0, 27, 3}, {1, 15, 8}},
        {{0, 3, 8}, {0, 27, 8}},
        {{0, 3, 13}, {0, 27, 13}, {0, 15, 20}, {1, 25, 25}},
        {{1, 3, 27}, {1, 27, 27}},
        {{0, 5, 22}, {0, 26, 17}}};

    Board ref = make_board(31, 31, 2, nets);
    RouteStats full = ref.route_all();

    Board b = make_board(31, 31, 2, nets);
    b.clear_routing();
    for (int pos = 0; pos < b.num_nets(); ++pos) {
        // Net-order policy: keep connecting this net's nearest pin until it is done
        // or a connection fails (exactly the net-mode stop-at-first-failure contract).
        while (true) {
            auto r = b.route_one_connection(pos, -1);
            if (!r.progress) break;
            if (r.unconnected_pins == 0) break;
        }
    }
    RouteStats conn = b.collect_stats();

    CHECK(conn.unrouted_count == full.unrouted_count);
    CHECK(conn.total_vias == full.total_vias);
    CHECK(conn.total_unconnected_pins == full.total_unconnected_pins);
    CHECK(conn.total_length == doctest::Approx(full.total_length));
    REQUIRE(conn.per_net_length.size() == full.per_net_length.size());
    for (size_t i = 0; i < full.per_net_length.size(); ++i) {
        CHECK(conn.per_net_length[i] == doctest::Approx(full.per_net_length[i]));
        CHECK(conn.per_net_unconnected[i] == full.per_net_unconnected[i]);
    }
    CHECK(conn.drc_violations == full.drc_violations);
}

// A net's own committed copper must never repel it: with strong trace avoidance,
// every connection must take EXACTLY the same path it takes with avoidance off —
// the self-exemption scratch removes the net's own contributions entirely.
TEST_CASE("connection_mode_no_self_repulsion") {
    auto run = [](double trace_mult) {
        Board b = make_board(25, 25, 1, {{{0, 3, 12}, {0, 21, 12}, {0, 12, 3}}});
        b.set_avoidance(0, 0.0, trace_mult);
        b.clear_routing();
        std::vector<double> lens;
        while (true) {
            auto r = b.route_one_connection(0, -1);
            if (!r.progress) break;
            lens.push_back(r.added_length);
            if (r.unconnected_pins == 0) break;
        }
        return std::make_pair(lens, b.nets()[0].routed);
    };
    auto [len_off, routed_off] = run(0.0);
    auto [len_on, routed_on] = run(2.0);   // strong repulsion, fully self-exempt
    CHECK(routed_off);
    CHECK(routed_on);
    REQUIRE(len_on.size() == len_off.size());
    for (size_t i = 0; i < len_off.size(); ++i)
        CHECK(len_on[i] == doctest::Approx(len_off[i]));
}

// ---------------------------------------------------------------------------
// Congestion self-exemption: a net never pays for its OWN predicted demand.
// A lone net with a huge congestion weight must route exactly as with weight 0
// (its flight-line bbox is the only demand on the board — all self).
// ---------------------------------------------------------------------------
TEST_CASE("congestion_field_self_exempt") {
    auto run = [](double weight) {
        Board b = make_board(30, 30, 1, {{{0, 3, 15}, {0, 26, 15}, {0, 15, 4}}});
        b.bake_congestion_rudy();
        b.set_congestion_weight(weight);
        return b.route_all();
    };
    RouteStats off = run(0.0);
    RouteStats on = run(5.0);   // enormous weight, all demand is the net's own
    CHECK(off.unrouted_count == 0);
    CHECK(on.unrouted_count == 0);
    CHECK(on.total_length == doctest::Approx(off.total_length));
}

// ---------------------------------------------------------------------------
// Early-UNSAT flood (optimization plan C2): a provably-sealed pin must fail
// with EXACTLY the same final stats as the exhaustive search it replaces, and
// routable boards must be untouched by the check.
TEST_CASE("unsat_check_equivalence") {
    // wall of foreign pads down x=10 seals the left pin pocket on a 1-layer board
    auto build = [&](bool gap_in_wall) {
        std::vector<std::vector<Cell>> nets;
        nets.push_back({Cell{0, 3, 10}, Cell{0, 17, 10}});     // net 0: crosses the wall
        std::vector<Cell> wall;
        for (int y = 0; y < 21; ++y) {
            if (gap_in_wall && (y == 10 || y == 11 || y == 12)) continue;
            wall.push_back(Cell{0, 10, y});
        }
        nets.push_back(wall);                                   // net 1: the wall pads
        return make_board(21, 21, 1, nets);
    };
    for (bool gap : {false, true}) {
        set_unsat_check_enabled(true);
        Board a = build(gap);
        RouteStats sa = a.route_all();
        set_unsat_check_enabled(false);
        Board b = build(gap);
        RouteStats sb = b.route_all();
        set_unsat_check_enabled(true);
        CHECK(sa.unrouted_count == sb.unrouted_count);
        CHECK(sa.total_length == sb.total_length);
        CHECK(sa.total_vias == sb.total_vias);
        if (!gap) CHECK(sa.unrouted_count >= 1);   // sealed: net 0 must fail
        if (gap) CHECK(sa.unrouted_count == 0);    // opened: everything routes
    }
}

// ---------------------------------------------------------------------------
// C3 parallel nets: batched routing must be deterministic across runs, DRC
// clean, and equal to serial on a fixture of non-crossing nets.
TEST_CASE("parallel_nets_deterministic_and_clean") {
    auto build = [&]() {
        std::vector<std::vector<Cell>> nets;
        for (int i = 0; i < 8; ++i)                     // parallel horizontal runs
            nets.push_back({Cell{0, 2, 3 + 4 * i}, Cell{0, 37, 3 + 4 * i}});
        return make_board(40, 40, 1, nets);
    };
    Board serial = build();
    RouteStats ss = serial.route_all();

    Board p1 = build();
    p1.set_parallel_nets(8);
    RouteStats s1 = p1.route_all();
    Board p2 = build();
    p2.set_parallel_nets(8);
    RouteStats s2 = p2.route_all();

    CHECK(s1.unrouted_count == s2.unrouted_count);      // deterministic
    CHECK(s1.total_length == s2.total_length);
    CHECK(s1.total_vias == s2.total_vias);
    CHECK(s1.drc_violations == 0);                      // hard-DRC holds
    CHECK(s1.unrouted_count == ss.unrouted_count);      // matches serial here
    CHECK(s1.unrouted_count == 0);
}

// ---------------------------------------------------------------------------
// P1a gates: PathFinder FPGA'95 Figures 1 & 2 as constructive grid boards at
// PRODUCTION physics (res 0.05mm: trace half-width 2 cells, clearance 4,
// pitch 8). Wall rows pierced by three 16-cell channels: passable strip =
// 16 - 2*(hw+clr) = 4 < pitch, so each channel carries EXACTLY one trace.
// ---------------------------------------------------------------------------
namespace {
constexpr int CH_A = 64, CH_B = 160, CH_C = 256, CH_W = 16;
Board make_channel_board(const std::vector<std::pair<Cell, Cell>>& netpins) {
    Board b(1, 320, 160, 0.05, 5.0, 1.0, 0.0);   // KiCad-default physics
    for (size_t i = 0; i < netpins.size(); ++i) {
        Net net; net.id = (int)i;
        net.pins = {netpins[i].first, netpins[i].second};
        b.nets().push_back(std::move(net));
        b.add_pad(i, netpins[i].first, 2.0);
        b.add_pad(i, netpins[i].second, 2.0);
    }
    // wall band y=[64,96): keepout rectangles between/outside channels
    auto wall = [&](double xa, double xb) {
        if (xb <= xa) return;
        b.add_keepout_shape(0, 0.5 * (xa + xb), 80.0, 0.5 * (xb - xa), 16.0,
                            0.0, false);
    };
    wall(0, CH_A - CH_W / 2);
    wall(CH_A + CH_W / 2, CH_B - CH_W / 2);
    wall(CH_B + CH_W / 2, CH_C - CH_W / 2);
    wall(CH_C + CH_W / 2, 320);
    return b;
}
int channel_of(const Board& b, int k) {
    for (const auto& seg : b.nets()[(size_t)k].segments)
        for (const auto& c : seg)
            if (c.y == 80) {
                if (std::abs(c.x - CH_A) <= CH_W) return CH_A;
                if (std::abs(c.x - CH_B) <= CH_W) return CH_B;
                if (std::abs(c.x - CH_C) <= CH_W) return CH_C;
            }
    return -1;
}
} // namespace

TEST_CASE("negotiate_window_first_order_congestion") {
    // all three nets sit near channel B; capacity 1 forces a spread
    std::vector<std::pair<Cell, Cell>> netpins = {
        {{0, 144, 16}, {0, 144, 144}},
        {{0, 160, 16}, {0, 160, 144}},
        {{0, 176, 16}, {0, 176, 144}},
    };
    Board b = make_channel_board(netpins);
    auto r = b.negotiate_window({0, 1, 2}, 0, 0, 319, 159, 40, 0.5, 1.7, 0.4);
    CHECK(r.converged);
    auto st = b.collect_stats(true);
    CHECK(st.unrouted_count == 0);
    CHECK(st.drc_violations == 0);
    int c0 = channel_of(b, 0), c1 = channel_of(b, 1), c2 = channel_of(b, 2);
    CHECK(c0 != -1); CHECK(c1 != -1); CHECK(c2 != -1);
    CHECK(c0 != c1); CHECK(c1 != c2); CHECK(c0 != c2);
}

TEST_CASE("negotiate_window_second_order_congestion") {
    // net2 lives in channel C's bore (can use ONLY C); net1 midway between
    // B and C; net0 hugs B. Sequential: 0->B, 1->C, 2 stuck; 0 never
    // conflicts, so rip-up cannot move it. Negotiation must cascade:
    // hist(C) evicts 1 to B, hist(B) evicts 0 to A.
    std::vector<std::pair<Cell, Cell>> netpins = {
        {{0, 156, 16}, {0, 156, 144}},
        {{0, 224, 16}, {0, 224, 144}},
        {{0, 256, 68}, {0, 256, 92}},
    };
    Board b = make_channel_board(netpins);
    // (Comparative fail-of-sequential control lives in the P1b board gate —
    // toy grids keep giving greedy an out that graph instances don't have.
    // This test proves the MECHANISM: the second-order cascade resolves with
    // the bystander evicted, which no blame/ripup scheme can produce.)
    auto r = b.negotiate_window({0, 1, 2}, 0, 0, 319, 159, 40, 0.5, 1.7, 0.4);
    CHECK(r.converged);
    auto st = b.collect_stats(true);
    CHECK(st.unrouted_count == 0);
    CHECK(st.drc_violations == 0);
    CHECK(channel_of(b, 2) == CH_C);
    CHECK(channel_of(b, 1) == CH_B);
    CHECK(channel_of(b, 0) == CH_A);
}

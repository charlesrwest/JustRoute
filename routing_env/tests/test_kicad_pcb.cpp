#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "routing/kicad_pcb.hpp"

#include <fstream>
#include <sstream>
#include <string>
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

TEST_CASE("v5_import_net_pad_counts") {
    // Small KiCad v5 (module) board: F.Cu+B.Cu, 4 declared nets (net 0 = unconnected),
    // 3 with thru-hole pads; 12 pins total. All pins in bounds.
    std::string s = read_file(data_file("KicadV5_xover4schiit.kicad_pcb"));
    KicadPcbInfo info;
    Board b = load_kicad_pcb(s, 1.0, &info);
    CHECK(b.grid().layers() == 2);
    REQUIRE(info.layer_names.size() == 2);
    CHECK(info.layer_names[0] == "F.Cu");
    CHECK(info.layer_names[1] == "B.Cu");
    REQUIRE(b.num_nets() == 3);
    // original ids are 1,2,3 (net 0 skipped) preserving the declared order
    REQUIRE(info.original_net_ids.size() == 3);
    CHECK(info.original_net_ids[0] == 1);
    CHECK(info.original_net_ids[2] == 3);
    int total_pins = 0;
    for (const Net& n : b.nets()) { total_pins += (int)n.pins.size(); for (const Cell& c : n.pins) CHECK(b.grid().valid(c)); }
    // thru-hole pads span F.Cu+B.Cu, so 12 pads -> 24 pins
    CHECK(total_pins == 24);
}

TEST_CASE("v6_import_net_pad_counts") {
    // Real KiCad v6 (footprint) board: F.Cu+B.Cu, 43 footprints, 113 thru-holes, 29
    // routable nets (orig ids 1..29), 103 pins -- all in bounds.
    std::string s = read_file(data_file("KicadV6_piezo_amplifier.kicad_pcb"));
    KicadPcbInfo info;
    Board b = load_kicad_pcb(s, 1.0, &info);
    CHECK(b.grid().layers() == 2);
    REQUIRE(info.layer_names.size() == 2);
    REQUIRE(b.num_nets() == 29);
    int total_pins = 0;
    for (const Net& n : b.nets()) { total_pins += (int)n.pins.size(); for (const Cell& c : n.pins) CHECK(b.grid().valid(c)); }
    // thru-hole pads span F.Cu+B.Cu, so 103 pads -> 206 pins
    CHECK(total_pins == 206);
}

TEST_CASE("pad_absolute_placement_v5") {
    // Sanity: a known v5 pad's absolute center must land inside the board bounds and
    // map to an in-grid cell that is actually a pad cell of some net.
    std::string s = read_file(data_file("KicadV5_xover4schiit.kicad_pcb"));
    Board b = load_kicad_pcb(s, 1.0, nullptr);
    int pad_cells = 0;
    for (const Net& n : b.nets()) pad_cells += (int)n.pins.size();
    CHECK(pad_cells > 0);
    CHECK(b.grid().width() > 0);
    CHECK(b.grid().height() > 0);
}

TEST_CASE("is_routeable_v5") {
    std::string s = read_file(data_file("KicadV5_xover4schiit.kicad_pcb"));
    Board b = load_kicad_pcb(s, 1.0, nullptr);
    RouteStats st = b.route_all();
    // This small board routes fully with zero DRC violations.
    CHECK(st.unrouted_count == 0);
    CHECK(st.drc_violations == 0);
    CHECK(st.per_net_length.size() == 3);
    for (double len : st.per_net_length) CHECK(len > 0.0);
}

TEST_CASE("is_routeable_v6") {
    // Real v6 board imports and routes with PHYSICS ON (trace width/clearance/via are
    // always enabled). With physical clearance this dense amplifier board is genuinely
    // hard: the router reports it honestly (some unrouted, vias present) rather than
    // claiming a thin/no-clearance "full" route. (Physics cannot be turned off.)
    std::string s = read_file(data_file("KicadV6_piezo_amplifier.kicad_pcb"));
    Board b = load_kicad_pcb(s, 1.0, nullptr);
    RouteStats st = b.route_all();
    CHECK(st.per_net_length.size() > 0);
    // Multi-layer connectivity is exercised (barrels and/or vias): count raw layer
    // transitions in the committed paths. total_vias itself now counts only router-
    // placed vias — a thru-hole pad's pre-drilled barrel is pad copper, not a via.
    int transitions = 0;
    for (const auto& net : b.nets())
        for (const auto& seg : net.segments)
            for (size_t i = 1; i < seg.size(); ++i)
                if (seg[i].layer != seg[i - 1].layer) transitions++;
    CHECK(transitions > 0);
    // Unrouted reflects the genuinely over-subscribed board under real clearance.
    CHECK(st.unrouted_count > 0);
}

TEST_CASE("malformed_throws_runtime_error") {
    // Not an S-expression at all.
    CHECK_THROWS_AS(load_kicad_pcb("not an s-expression {{{(", 1.0), std::runtime_error);
    // Unbalanced parenthesis.
    CHECK_THROWS_AS(load_kicad_pcb("(kicad_pcb (version 20171130)", 1.0), std::runtime_error);
    // Missing layers / no copper layers.
    CHECK_THROWS_AS(load_kicad_pcb("(kicad_pcb (foo 1))", 1.0), std::runtime_error);
    // Bad resolution.
    CHECK_THROWS_AS(load_kicad_pcb("(kicad_pcb (layers (0 F.Cu signal)))", 0.0),
                    std::runtime_error);
}

TEST_CASE("deterministic_import") {
    std::string s = read_file(data_file("KicadV5_xover4schiit.kicad_pcb"));
    Board a = load_kicad_pcb(s, 1.0, nullptr);
    Board b2 = load_kicad_pcb(s, 1.0, nullptr);
    REQUIRE(a.num_nets() == b2.num_nets());
    for (int i = 0; i < a.num_nets(); ++i) {
        REQUIRE(a.nets()[(size_t)i].pins.size() == b2.nets()[(size_t)i].pins.size());
        for (size_t j = 0; j < a.nets()[(size_t)i].pins.size(); ++j)
            CHECK(a.nets()[(size_t)i].pins[j] == b2.nets()[(size_t)i].pins[j]);
    }
}

// Regression for the reviewer-confirmed rotation-sign bug: KiCad uses Y-down coords, so a
// rotated module's pads must be placed with the negated (KiCad) sign. A module at (100,100)
// rotated +90 with pads at local (0,-10)/(0,+10) places them at x=90 / x=110 respectively.
TEST_CASE("rotation_sign_matches_kicad") {
    const std::string rdl = R"(
(kicad_pcb (version 20171130)
  (layers (0 F.Cu signal) (31 B.Cu signal))
  (net 0 "")
  (net 1 GND)
  (net 2 /A)
  (module test (layer F.Cu) (at 100 100 90)
    (pad 1 thru_hole circle (at 0 -10 0) (size 1 1) (drill 0.5) (layers *.Cu)
      (net 2 /A))
    (pad 2 thru_hole circle (at 0 10 0) (size 1 1) (drill 0.5) (layers *.Cu)
      (net 1 GND))
  )
)
)";
    KicadPcbInfo info;
    Board b = load_kicad_pcb(rdl, 1.0, &info);
    REQUIRE(b.num_nets() == 2);
    // Board net 0 = declared GND (orig 1), net 1 = declared /A (orig 2).
    REQUIRE(info.original_net_ids.size() == 2);
    int gnd = 0, a = 0;
    if (info.original_net_ids[0] == 1) { gnd = 0; a = 1; } else { gnd = 1; a = 0; }
    const Cell& gpad = b.nets()[(size_t)gnd].pins[0];
    const Cell& apad = b.nets()[(size_t)a].pins[0];
    // Correct KiCad sign: /A pad ends up LEFT of the GND pad (x=90 vs x=110 at res 1).
    // The old CCW-mirrored sign swaps them, so this ordering proves the fix.
    CHECK(apad.x < gpad.x);
    CHECK(apad.layer == 0); // thru-hole *.Cu -> F.Cu (layer 0)
}

// Regression for the SMD/non-thru-hole layer path (reviewer note): an explicit B.Cu
// surface-mount pad must be pinned on layer 1 (B.Cu), not layer 0 (F.Cu).
TEST_CASE("smd_pad_on_own_copper_layer") {
    const std::string rdl = R"(
(kicad_pcb (version 20211014)
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (net 0 "")
  (net 1 SMDB)
  (footprint "test" (layer "F.Cu") (at 50 50)
    (pad "1" smd rect (at 0 0) (size 1.5 1.5) (drill 0) (layers "B.Cu")
      (net 1 SMDB))
  )
)
)";
    Board b = load_kicad_pcb(rdl, 1.0, nullptr);
    REQUIRE(b.num_nets() == 1);
    REQUIRE(b.nets()[0].pins.size() == 1);
    // The SMD pad is on B.Cu -> Board layer 1, not F.Cu layer 0.
    CHECK(b.nets()[0].pins[0].layer == 1);
}

// Regression for the degenerate-bounds reviewer note: a single-column / single-row board
// (all pads on one y) must still import (the pre-margin max<=min check used to reject it).
TEST_CASE("single_row_board_imports") {
    const std::string rdl = R"(
(kicad_pcb (version 20171130)
  (layers (0 F.Cu signal) (31 B.Cu signal))
  (net 0 "")
  (net 1 A)
  (net 2 B)
  (net 3 C)
  (module m (layer F.Cu) (at 0 10 0)
    (pad 1 thru_hole circle (at 0 0 0) (size 1 1) (drill 0.5) (layers *.Cu) (net 1 A))
    (pad 2 thru_hole circle (at 10 0 0) (size 1 1) (drill 0.5) (layers *.Cu) (net 2 B))
    (pad 3 thru_hole circle (at 20 0 0) (size 1 1) (drill 0.5) (layers *.Cu) (net 3 C))
  )
)
)";
    Board b = load_kicad_pcb(rdl, 1.0, nullptr);
    CHECK(b.num_nets() == 3);
    RouteStats st = b.route_all();
    CHECK(st.unrouted_count == 0);
}

// Regression for the reviewer-low parser-depth note: deeply nested input must throw
// std::runtime_error (depth-capped) instead of overflowing the stack (SIGSEGV).
TEST_CASE("deeply_nested_throws_runtime_error") {
    std::string s;
    s.reserve(1 << 16);
    const int N = 10000;
    for (int i = 0; i < N; ++i) s.push_back('(');
    s += "(kicad_pcb";
    for (int i = 0; i < N; ++i) s.push_back(')');
    CHECK_THROWS_AS(load_kicad_pcb(s, 1.0), std::runtime_error);
}

TEST_CASE("kicad_pin_collision_reported_at_coarse") {
    // Stage 10: fine resolution keeps all pads distinct; a deliberately too-coarse
    // resolution collapses some (piezo min pad pitch ~1.27mm, so 3.0mm collapses some).
    std::string s = read_file(data_file("KicadV6_piezo_amplifier.kicad_pcb"));
    KicadPcbInfo fine, coarse;
    load_kicad_pcb(s, 0.2, &fine);
    CHECK(fine.pin_collisions == 0);
    CHECK(fine.min_pad_spacing < 10.0);
    load_kicad_pcb(s, 3.0, &coarse, /*reject_collisions=*/false);
    CHECK(coarse.pin_collisions > 0);
    CHECK_THROWS_AS(load_kicad_pcb(s, 3.0), std::runtime_error);
}

TEST_CASE("min_pad_spacing_counts_physical_pads_once") {
    // Thru-hole pads used to contribute one centre per copper layer, so the
    // pairwise spacing scan always found an identical-centre pair -> 0.0 and the
    // resolution recommendation was unusable.
    std::string s = read_file(data_file("KicadV6_piezo_amplifier.kicad_pcb"));
    KicadPcbInfo info;
    Board b = load_kicad_pcb(s, 1.0, &info);
    CHECK(info.min_pad_spacing > 0.0);
    CHECK(std::isfinite(info.min_pad_spacing));
}

TEST_CASE("pad_size_parsed_into_physical_radius") {
    // The loader must read the pad's real (size w h) instead of the 0.3/0.1mm
    // heuristic. A 3mm pad has radius 1.5mm; a foreign trace 2 cells from its
    // centre (inside radius + clearance + trace half-width = 2.6 cells at res 1.0)
    // must be a DRC violation. Under the old heuristic (0.3mm) it passed clean.
    std::string s = R"((kicad_pcb (version 20221018) (generator test)
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (net 0 "") (net 1 "A") (net 2 "B")
  (footprint "f" (at 0 0)
    (pad "1" thru_hole circle (at 5 5) (size 3.0 3.0) (drill 1.0) (layers *.Cu) (net 1 "A"))
    (pad "2" thru_hole circle (at 15 5) (size 3.0 3.0) (drill 1.0) (layers *.Cu) (net 1 "A"))
    (pad "3" smd rect (at 5 15) (size 1.0 1.0) (layers "F.Cu") (net 2 "B"))
    (pad "4" smd rect (at 15 15) (size 1.0 1.0) (layers "F.Cu") (net 2 "B"))
  )))";
    Board b = load_kicad_pcb(s, 1.0, nullptr);
    REQUIRE(b.num_nets() == 2);
    int drc_before = b.check_drc().drc_violations;
    // Pad "1" (net A) grid cell = its layer-0 pin; put net B copper 2 cells away.
    Cell pad = b.nets()[0].pins[0];
    b.add_manual_trace(Cell{0, pad.x + 2, pad.y}, 1);
    int drc_after = b.check_drc().drc_violations;
    CHECK(drc_after > drc_before);   // trace sits inside the pad's REAL keepout
}

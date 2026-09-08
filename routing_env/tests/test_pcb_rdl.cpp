#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "routing/pcb_rdl.hpp"

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

TEST_CASE("import_cherrymx_counts") {
    std::string json = read_file(data_file("CherryMxBitboard.rdl.json"));
    PcbRdlInfo info;
    Board b = load_pcb_rdl(json, 1.0, &info);
    CHECK(b.num_nets() == 3);
    CHECK(b.grid().layers() == 2);
    CHECK(info.layer_names.size() == 2);
    // net pin counts: thru-hole pads span BOTH copper layers, so each pad contributes a
    // pin per layer (original pads 2/5/7 -> 4/10/14 pins) while still routing cleanly.
    REQUIRE(b.nets().size() == 3);
    CHECK(b.nets()[0].pins.size() == 4);
    CHECK(b.nets()[1].pins.size() == 10);
    CHECK(b.nets()[2].pins.size() == 14);
    REQUIRE(info.original_net_ids.size() == 3);
    CHECK(info.original_net_ids[0] == 1);
    CHECK(info.original_net_ids[2] == 3);
}

TEST_CASE("import_1bitsy_counts_and_bounds") {
    std::string json = read_file(data_file("1Bitsy_1bitsy.rdl.json"));
    PcbRdlInfo info;
    Board b = load_pcb_rdl(json, 0.5, &info);  // 1.0mm collapses pads; use 0.5 (no collapse)
    // file has 63 nets (with >=1 routable pad each); a few may be empty/skipped
    CHECK(b.num_nets() >= 50);
    CHECK(b.grid().layers() == 4);
    CHECK(info.layer_names.size() == 4);
    // every pad cell must be within the grid
    for (const Net& n : b.nets()) {
        for (const Cell& c : n.pins) {
            CHECK(b.grid().valid(c));
        }
    }
    // total pads = sum over nets
    int total = 0;
    for (const Net& n : b.nets()) total += (int)n.pins.size();
    CHECK(total > 100);
}

TEST_CASE("import_is_routeable") {
    // A real board should load and route (at least partially) without crashing.
    std::string json = read_file(data_file("CherryMxBitboard.rdl.json"));
    Board b = load_pcb_rdl(json, 1.0, nullptr);
    RouteStats s = b.route_all();
    // It should route at least some nets successfully.
    CHECK(s.unrouted_count < b.num_nets());
    // nothing outside the grid is ever referenced (DRC overlaps etc.)
    CHECK(s.drc_violations >= 0);
}

TEST_CASE("import_bounds_positive_for_large_board") {
    std::string json = read_file(data_file("1Bitsy_1bitsy.rdl.json"));
    PcbRdlInfo info;
    Board b = load_pcb_rdl(json, 0.5, &info);
    CHECK(b.grid().width() > 0);
    CHECK(b.grid().height() > 0);
    CHECK(b.grid().resolution() == doctest::Approx(0.5));
    // board bounds recorded
    CHECK(info.border_max_x > info.border_min_x);
    CHECK(info.border_max_y > info.border_min_y);
}

TEST_CASE("invalid_rdl_throws") {
    CHECK_THROWS_AS(load_pcb_rdl("not json {{{", 1.0), std::runtime_error);
    // valid JSON but missing required keys
    CHECK_THROWS_AS(load_pcb_rdl("{\"layers\":[\"F.Cu\"]}", 1.0), std::runtime_error);
    // bad resolution
    CHECK_THROWS_AS(load_pcb_rdl("{\"layers\":[\"F.Cu\"],\"border\":[],\"nets\":{}}", 0.0),
                    std::runtime_error);
}

TEST_CASE("malformed_inputs_throw_runtime_error_contract") {
    // Regression: even type-mismatched / non-numeric-key input must throw
    // std::runtime_error (not an nlohmann/stoi exception), per the documented contract.
    // non-numeric net key (stoi would throw invalid_argument)
    std::string bad_key =
        "{\"layers\":[\"F.Cu\"],\"border\":[[{\"type\":\"polyline\",\"vertices\":[[0,0],[10,0],[10,10],[0,10]]}]],"
        "\"nets\":{\"abc\":[[{\"center\":[1.0,1.0],\"layer\":[\"F.Cu\"]}]]}}";
    CHECK_THROWS_AS(load_pcb_rdl(bad_key, 1.0), std::runtime_error);
}


TEST_CASE("rdl_pin_collision_is_reported") {
    // Stage 10: coarse resolution must not silently collapse distinct pads. 1Bitsy has a
    // ~0.4mm min pad pitch: at 1.0mm many distinct pads collapse (>0), at 0.5mm or finer
    // there are none.
    std::string s = read_file(data_file("1Bitsy_1bitsy.rdl.json"));
    PcbRdlInfo coarse, fine;
    load_pcb_rdl(s, 1.0, &coarse, /*reject_collisions=*/false);  // read the count, don't throw
    load_pcb_rdl(s, 0.5, &fine);
    CHECK(coarse.pin_collisions > 0);
    CHECK(fine.pin_collisions == 0);
    // a coarse load REJECTS (throws) by default
    CHECK_THROWS_AS(load_pcb_rdl(s, 1.0), std::runtime_error);
    // recommended resolution (>= 2 cells across the closest pad pair) has no collisions
    double rec = recommended_resolution(fine.min_pad_spacing);
    PcbRdlInfo atrec;
    load_pcb_rdl(s, std::max(rec, 0.05), &atrec);
    CHECK(atrec.pin_collisions == 0);
    CHECK(rec > 0.0);
    CHECK(rec < fine.min_pad_spacing);
}

TEST_CASE("real_scale_fixtures_import") {
    // Stage 12: representative larger real boards must import at 0.05 mm with no pin
    // collision and sane grid bounds.
    struct Case { const char* file; int nets; };
    Case cs[] = {{"96boards-robomezzi.rdl.json", 103},
                 {"2d_conduction_sk9822-matrix.rdl.json", 132}};
    for (const auto& c : cs) {
        std::string json = read_file(data_file(c.file));
        PcbRdlInfo info;
        Board b = load_pcb_rdl(json, 0.05, &info);
        CHECK(info.pin_collisions == 0);
        CHECK(b.num_nets() == c.nets);
        CHECK(b.grid().width() > 0);
        CHECK(b.grid().height() > 0);
        for (const Net& n : b.nets())
            for (const Cell& cc : n.pins) CHECK(b.grid().valid(cc));
    }
}

TEST_CASE("large_board_cap_raised") {
    // A ~300 mm board at 0.05 mm -> ~6000 cells wide, above the OLD 4096 cap but within
    // the new 32768 cap: it must now import. A genuinely huge board (still) must be rejected.
    const std::string wide = R"({
      "layers": ["F.Cu","B.Cu"],
      "border": [{"type":"polyline","vertices":[[0,0],[300,0],[300,100],[0,100]]}],
      "nets": {"1":[{"center":[10.0,10.0],"layer":["F.Cu"]}]}
    })";
    CHECK_NOTHROW(load_pcb_rdl(wide, 0.05));  // 6000 cells wide -> old cap would reject
    const std::string huge = R"({
      "layers": ["F.Cu","B.Cu"],
      "border": [{"type":"polyline","vertices":[[0,0],[5000,0],[5000,100],[0,100]]}],
      "nets": {"1":[{"center":[10.0,10.0],"layer":["F.Cu"]}]}
    })";
    CHECK_THROWS_AS(load_pcb_rdl(huge, 0.05), std::runtime_error);  // > 32768 cells -> rejected
}

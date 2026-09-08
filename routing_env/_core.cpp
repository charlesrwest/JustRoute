#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>

#include "routing/grid.hpp"
#include "routing/cost_model.hpp"
#include "routing/astar.hpp"
#include "routing/board.hpp"
#include "routing/wave_gpu.hpp"
#include "routing/env.hpp"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace routing;

PYBIND11_MODULE(routing_env, m) {
    m.doc() = "C++ PCB routing environment (Grid + A* core + RoutingEnv)";
    m.attr("DEFAULT_RESOLUTION_MM") = routing::DEFAULT_RESOLUTION_MM;

    py::class_<Coord>(m, "Coord")
        .def(py::init<>())
        .def_readwrite("x", &Coord::x)
        .def_readwrite("y", &Coord::y);

    py::class_<Cell>(m, "Cell")
        .def(py::init<>())
        .def(py::init<int, int, int>(), py::arg("layer"), py::arg("x"), py::arg("y"))
        .def_readwrite("layer", &Cell::layer)
        .def_readwrite("x", &Cell::x)
        .def_readwrite("y", &Cell::y)
        .def("__eq__", [](const Cell& a, const Cell& b) { return a == b; });

    py::class_<Grid>(m, "Grid")
        .def(py::init<int, int, int, double>(),
             py::arg("layers"), py::arg("width"), py::arg("height"), py::arg("resolution"))
        .def("layers", &Grid::layers)
        .def("width", &Grid::width)
        .def("height", &Grid::height)
        .def("resolution", &Grid::resolution)
        .def("index", [](const Grid& g, const Cell& c){ return g.index(c); }, py::arg("cell"))
        .def("unindex", &Grid::unindex, py::arg("i"))
        .def("to_cell", [](const Grid& g, double x, double y){ return g.to_cell(x, y, 0); },
             py::arg("x"), py::arg("y"))
        .def("valid", [](const Grid& g, const Cell& c){ return g.valid(c); }, py::arg("cell"))
        .def("at", [](const Grid& g, int layer, int x, int y){ return g.at(layer, x, y); },
             py::arg("layer"), py::arg("x"), py::arg("y"))
        .def("set", [](Grid& g, int layer, int x, int y, double v){ g.at(layer, x, y) = v; },
             py::arg("layer"), py::arg("x"), py::arg("y"), py::arg("v"))
        .def("fill_all", &Grid::fill_all, py::arg("v"));

    py::class_<CostParams>(m, "CostParams")
        .def(py::init<>())
        .def_readwrite("base_cost", &CostParams::base_cost)
        .def_readwrite("pad_avoid_mult", &CostParams::pad_avoid_mult)
        .def_readwrite("trace_avoid_mult", &CostParams::trace_avoid_mult)
        .def_readwrite("pad_falloff_k", &CostParams::pad_falloff_k)
        .def_readwrite("trace_falloff_k", &CostParams::trace_falloff_k)
        .def_readwrite("via_cost", &CostParams::via_cost);

    m.def("cell_cost", &cell_cost,
          py::arg("params"), py::arg("pad_dist"), py::arg("trace_dist"));

    // ---- RouteStats ----
    py::class_<RouteStats>(m, "RouteStats")
        .def(py::init<>())
        .def_readwrite("per_net_length", &RouteStats::per_net_length)
        .def_readwrite("unrouted", &RouteStats::unrouted)
        .def_readwrite("drc_violations", &RouteStats::drc_violations)
        .def_readwrite("unrouted_count", &RouteStats::unrouted_count)
        .def_readwrite("total_length", &RouteStats::total_length)
        .def_readwrite("total_vias", &RouteStats::total_vias)
        .def_readwrite("time_budget_exceeded", &RouteStats::time_budget_exceeded)
        .def_readwrite("resume_from", &RouteStats::resume_from)
        .def_readwrite("per_net_unconnected", &RouteStats::per_net_unconnected)
        .def_readwrite("total_unconnected_pins", &RouteStats::total_unconnected_pins);

    // ---- Net ----
    py::class_<Net>(m, "Net")
        .def(py::init<>())
        .def_readwrite("id", &Net::id)
        .def_readwrite("pins", &Net::pins)
        .def_readwrite("total_length", &Net::total_length)
        .def_readwrite("routed", &Net::routed)
        .def_readwrite("unconnected_pins", &Net::unconnected_pins)
        .def_property_readonly("pin_connected", [](const Net& n) {
            py::list out;
            for (size_t i = 0; i < n.pins.size(); ++i)
                out.append(i < n.pin_connected.size() ? n.pin_connected[i] != 0 : i == 0);
            return out;
        })
        .def_readwrite("segments", &Net::segments);

    // ---- Board ----
    py::class_<Board>(m, "Board")
        .def(py::init<int, int, int, double, double, double, double>(),
             py::arg("layers"), py::arg("width"), py::arg("height"), py::arg("resolution"),
             py::arg("via_cost"), py::arg("base_cost"), py::arg("design_rule_clearance"))
        .def("num_nets", &Board::num_nets)
        .def("route_all", &Board::route_all, py::call_guard<py::gil_scoped_release>())
        .def("grid", [](Board& b) -> Grid& { return b.grid(); }, py::return_value_policy::reference)
        .def("owner", [](Board& b) { return b.owner(); })  // occupancy tokens (net_id+1), layer-major
        .def("bake_net_cost_grid",
             [](Board& b, int k) {
                 auto t = b.bake_net_cost_grid(k);
                 return py::make_tuple(
                     py::bytes((const char*)std::get<0>(t).data(),
                               std::get<0>(t).size() * 4),
                     py::bytes((const char*)std::get<1>(t).data(),
                               std::get<1>(t).size()),
                     std::get<2>(t));
             },
             py::arg("k"))
        .def("bake_net_cost_window",
             [](Board& b, int k, int x0, int y0, int x1, int y1,
                bool fast_via) {
                 auto t = b.bake_net_cost_window(k, x0, y0, x1, y1, fast_via);
                 return py::make_tuple(
                     py::bytes((const char*)std::get<0>(t).data(),
                               std::get<0>(t).size() * 4),
                     py::bytes((const char*)std::get<1>(t).data(),
                               std::get<1>(t).size()),
                     std::get<2>(t));
             },
             py::arg("k"), py::arg("x0"), py::arg("y0"), py::arg("x1"),
             py::arg("y1"), py::arg("fast_via") = true)
        .def("thru_pad_token", &Board::thru_pad_token, py::arg("x"), py::arg("y"))
        .def("pad_shape_info", [](const Board& b) {
                 py::list out;
                 for (const auto& ps : b.pad_shapes())
                     out.append(py::make_tuple(ps.tok, ps.layer, ps.cx, ps.cy));
                 return out;
             },
             "Pad shapes as (token, layer, cx, cy) with fractional cell centers "
             "(keepout obstacles report token INT32_MAX)")
        .def("pad_inflated_token", &Board::pad_inflated_token, py::arg("c"))
        .def("pad_shapes", [](const Board& b) {
            py::list out;
            for (const auto& ps : b.pad_shapes()) {
                py::dict d;
                d["tok"] = ps.tok; d["layer"] = ps.layer;
                d["cx"] = ps.cx; d["cy"] = ps.cy;
                d["half_w"] = ps.half_w; d["half_h"] = ps.half_h;
                d["cos_r"] = ps.cos_r; d["sin_r"] = ps.sin_r; d["oval"] = ps.oval;
                out.append(d);
            }
            return out;
        })
        .def("count_net_vias", &Board::count_net_vias, py::arg("net"))
        .def("unconnected_pin_indices", &Board::unconnected_pin_indices, py::arg("position"))
        .def("clear_routing", &Board::clear_routing)
        .def("collect_stats", &Board::collect_stats, py::arg("with_drc") = true, py::call_guard<py::gil_scoped_release>())
        .def("wave_reach_count", &Board::wave_reach_count, py::arg("position"),
             py::arg("seeds"), py::call_guard<py::gil_scoped_release>())
        .def("dump_wave_masks", &Board::dump_wave_masks, py::arg("position"),
             py::arg("seeds"), py::arg("path"))
        .def("wave_probe", &Board::wave_probe, py::arg("position"),
             py::arg("seeds"), py::arg("targets"),
             py::call_guard<py::gil_scoped_release>())
        .def("wave_probe_superset", &Board::wave_probe_superset,
             py::arg("position"), py::arg("seeds"), py::arg("targets"),
             py::call_guard<py::gil_scoped_release>())
        .def("scalar_probe", &Board::scalar_probe, py::arg("position"),
             py::arg("seeds"), py::arg("targets"),
             py::call_guard<py::gil_scoped_release>())
        .def("route_from", &Board::route_from, py::arg("lo"), py::call_guard<py::gil_scoped_release>())
        .def("move_net", &Board::move_net, py::arg("from_idx"), py::arg("to_idx"))
        .def("set_parallel_nets", &Board::set_parallel_nets, py::arg("n"))
        .def("set_expansion_budget", &Board::set_expansion_budget, py::arg("per_search"))
        .def("set_avoidance", &Board::set_avoidance, py::arg("net_idx"),
             py::arg("pad_mult"), py::arg("trace_mult"))
        .def("clear_avoidance", &Board::clear_avoidance)
        .def("set_tree_strategy", &Board::set_tree_strategy, py::arg("strategy"))
        .def("tree_strategy", &Board::tree_strategy)
        .def("bake_congestion_rudy", &Board::bake_congestion_rudy)
        .def("set_congestion_weight", &Board::set_congestion_weight, py::arg("w"))
        .def("set_congestion_gamma", &Board::set_congestion_gamma, py::arg("g"))
        .def("congestion_weight", &Board::congestion_weight)
        .def("congestion_at", &Board::congestion_at, py::arg("x"), py::arg("y"))
        .def("set_route_time_budget_s", &Board::set_route_time_budget_s, py::arg("seconds"))
        .def("route_time_budget_s", &Board::route_time_budget_s)
        .def("falloff_radius", &Board::falloff_radius)
        .def("set_falloff_radius", &Board::set_falloff_radius, py::arg("radius"))
        .def("trace_half_width_cells", &Board::trace_half_width_cells)
        .def("set_trace_half_width_cells", &Board::set_trace_half_width_cells, py::arg("cells"))
        .def("via_radius_cells", &Board::via_radius_cells)
        .def("set_via_radius_cells", &Board::set_via_radius_cells, py::arg("cells"))
        .def("set_net_class_physics", &Board::set_net_class_physics,
             py::arg("net_id"), py::arg("hw_cells"), py::arg("clr_cells"),
             "Per-net-pair physics tier: assign this net its own trace half-width "
             "and clearance (cells). Call after loading, before routing.")
        .def("add_pad", &Board::add_pad, py::arg("net_idx"), py::arg("cell"),
             py::arg("radius_cells") = 0.0)
        .def("add_pin_pad", &Board::add_pin_pad, py::arg("net_idx"), py::arg("cell"),
             py::arg("radius_cells") = 0.0)
        .def("rip_net_positions", &Board::rip_net_positions, py::arg("positions"))
        .def("trace_half_width_cells", &Board::trace_half_width_cells)
        .def("via_radius_cells", &Board::via_radius_cells)
        .def("drc_clearance_cells", [](const Board& b) {
                 return b.design_rule_clearance() / b.grid().resolution();
             })
        .def("route_in_corridor", &Board::route_in_corridor, py::arg("k"),
             py::arg("corridor"), py::arg("radius"),
             py::call_guard<py::gil_scoped_release>())
        .def("negotiate_window", [](Board& b, const std::vector<int>& positions,
                                    int x0, int y0, int x1, int y1, int max_iters,
                                    double pres_init, double pres_mult, double hist_gain) {
                 Board::NegotiateResult r;
                 {
                     py::gil_scoped_release rel;
                     r = b.negotiate_window(positions, x0, y0, x1, y1, max_iters,
                                            pres_init, pres_mult, hist_gain);
                 }
                 py::list routed;
                 for (auto v : r.routed) routed.append((bool)v);
                 return py::make_tuple(r.converged, r.iterations, r.shared_final, routed);
             }, py::arg("positions"), py::arg("x0"), py::arg("y0"),
             py::arg("x1"), py::arg("y1"), py::arg("max_iters") = 40,
             py::arg("pres_init") = 0.5, py::arg("pres_mult") = 1.7,
             py::arg("hist_gain") = 0.4)
        .def("probe_blockers", [](Board& b, int k, double soft_mult) {
                 bool crossed_pad = false;
                 auto v = b.probe_blockers(k, soft_mult, &crossed_pad);
                 return py::make_tuple(v, crossed_pad);
             }, py::arg("k"), py::arg("soft_mult") = 200.0)
        .def("check_drc", &Board::check_drc)
        .def("smooth_paths", &Board::smooth_paths, py::arg("max_dev_cells") = 1.5,
             py::arg("fillet_radius_cells") = 0.0,
             py::call_guard<py::gil_scoped_release>(),
             "Post-route: replace each net's grid staircase with the fewest "
             "DRC-clean straight segments. fillet_radius_cells>0 also rounds "
             "corners with validated tangent arcs. Returns vertices removed.")
        .def("has_smoothed", &Board::has_smoothed)
        .def("smoothed_paths", [](const Board& b) {
            py::list out;
            const auto& sp = b.smoothed_paths();
            for (size_t k = 0; k < sp.size(); ++k) {
                py::list segs;
                for (const auto& s : sp[k])
                    segs.append(py::make_tuple(s.layer, s.x0, s.y0, s.x1, s.y1,
                                               s.mx, s.my, s.is_arc));
                out.append(segs);
            }
            return out;
        }, "Per-net list of (layer, x0, y0, x1, y1, mx, my, is_arc) float-cell "
           "runs; is_arc segments are circular arcs through (mx,my).")
        .def("count_static_pad_conflicts", &Board::count_static_pad_conflicts)
        .def("nets", [](const Board& b) { return b.nets(); })
        .def("base_cost", &Board::base_cost)
        .def("via_cost", &Board::via_cost)
        .def("set_via_cost", &Board::set_via_cost, py::arg("v"));

    py::enum_<routing::TreeStrategy>(m, "TreeStrategy")
        .value("forward", routing::TreeStrategy::forward)
        .value("reverse", routing::TreeStrategy::reverse);

    // ---- RoutingEnv ----
    py::class_<RoutingEnv>(m, "RoutingEnv")
        .def(py::init<int, int, int, double, double, double, double>(),
             py::arg("layers"), py::arg("width"), py::arg("height"), py::arg("resolution"),
             py::arg("via_cost"), py::arg("base_cost"), py::arg("design_rule_clearance"))
        .def("load_synthetic",
             [](RoutingEnv& e, const SyntheticSpec& s) { return e.load_synthetic(s); },
             py::arg("spec"))
        .def("load_pcb_rdl",
             [](RoutingEnv& e, const std::string& j, double r) { return e.load_pcb_rdl(j, r); },
             py::arg("json"), py::arg("resolution") = routing::DEFAULT_RESOLUTION_MM)
        .def("load_kicad_pcb",
             [](RoutingEnv& e, const std::string& t, double r) {
                 return e.load_kicad_pcb(t, r);
             },
             py::arg("contents"), py::arg("resolution") = routing::DEFAULT_RESOLUTION_MM)
        .def("load_kicad_pcb_info",
             [](RoutingEnv& e, const std::string& t, double r, bool skip_poured) {
                 KicadPcbInfo info;
                 int nets = e.load_kicad_pcb(t, r, &info, skip_poured);
                 return py::dict(
                     "nets"_a = nets,
                     "layer_names"_a = info.layer_names,
                     "original_net_ids"_a = info.original_net_ids,
                     "net_names"_a = info.net_names,
                     "origin_x"_a = info.border_min_x,
                     "origin_y"_a = info.border_min_y,
                     "max_x"_a = info.border_max_x,
                     "max_y"_a = info.border_max_y,
                     "pre_routed_nets"_a = info.pre_routed_nets,
                     "partial_nets"_a = info.partial_nets,
                     "pour_fed_nets"_a = info.pour_fed_nets,
                     "min_pad_spacing"_a = info.min_pad_spacing,
                     "rule_clearance"_a = info.rule_clearance,
                     "default_track_width"_a = info.default_track_width,
                     "default_via_dia"_a = info.default_via_dia,
                     "default_via_drill"_a = info.default_via_drill,
                     "net_widths"_a = info.net_widths,
                     "net_via_dias"_a = info.net_via_dias,
                     "net_via_drills"_a = info.net_via_drills,
                     "rule_track_width"_a = info.rule_track_width,
                     "rule_via_dia"_a = info.rule_via_dia,
                     "rule_via_drill"_a = info.rule_via_drill);
             },
             py::arg("contents"), py::arg("resolution") = routing::DEFAULT_RESOLUTION_MM,
             py::arg("skip_poured") = false,
             "Load a .kicad_pcb like load_kicad_pcb, but also return the file-frame "
             "metadata the writer needs: copper layer names in Board layer order, the "
             "original KiCad net id per Board net index, and the grid origin in mm "
             "(cell -> mm is origin + cell*resolution; the margin is already folded in)")
        .def("reset", &RoutingEnv::reset, py::call_guard<py::gil_scoped_release>())
        .def("reset_empty", &RoutingEnv::reset_empty, py::call_guard<py::gil_scoped_release>())
        .def("save_checkpoint", &RoutingEnv::save_checkpoint)
        .def("restore_checkpoint", &RoutingEnv::restore_checkpoint)
        .def("has_checkpoint", &RoutingEnv::has_checkpoint)
        .def("step_connect", &RoutingEnv::step_connect,
             py::arg("net_id"), py::arg("pin_idx") = -1, py::call_guard<py::gil_scoped_release>())
        .def("step", &RoutingEnv::step,
             py::arg("net_idx"), py::arg("target_pos"), py::arg("pad_avoid"),
             py::arg("trace_avoid"), py::call_guard<py::gil_scoped_release>())
        .def("get_observation", &RoutingEnv::get_observation)
        .def("position_of", &RoutingEnv::position_of, py::arg("net_id"))
        .def("num_nets", &RoutingEnv::num_nets)
        .def("last_stats", &RoutingEnv::last_stats)
        .def("board", [](RoutingEnv& e) -> Board& { return e.board(); },
             py::return_value_policy::reference);

    // ---- synthetic spec (for load_synthetic) ----
    py::class_<SyntheticSpec>(m, "SyntheticSpec")
        .def(py::init<>())
        .def_readwrite("layers", &SyntheticSpec::layers)
        .def_readwrite("width", &SyntheticSpec::width)
        .def_readwrite("height", &SyntheticSpec::height)
        .def_readwrite("resolution", &SyntheticSpec::resolution)
        .def_readwrite("via_cost", &SyntheticSpec::via_cost)
        .def_readwrite("base_cost", &SyntheticSpec::base_cost)
        .def_readwrite("design_rule_clearance", &SyntheticSpec::design_rule_clearance)
        .def_readwrite("nets", &SyntheticSpec::nets);

    m.def("request_route_stop", [] { route_stop_flag().store(true); },
          "Cooperative cancel from any thread: the in-flight routing pass "
          "aborts within milliseconds, keeping its valid partial result.");
    m.def("clear_route_stop", [] { route_stop_flag().store(false); });
    m.def("route_progress", [] {
        return py::make_tuple(route_progress_done().load(),
                              route_progress_total().load());
    }, "(nets_done, nets_total) of the routing pass in flight — poll from "
       "any thread for UI progress.");
    m.def("build_synthetic", &build_synthetic, py::arg("spec"));
    m.def("wave_gpu_available", &wave_gpu_available);
    m.def("set_wave_gpu_enabled", &set_wave_gpu_enabled, py::arg("on"));
    m.def("gpu_field_available", &gpu_field_available);
    m.def("set_field_router", &set_field_router, py::arg("on"));
    m.def("set_field_corridor", &set_field_corridor, py::arg("on"));
    m.def("field_phase_stats", &field_phase_stats);
    m.def("field_phase_reset", &field_phase_reset);
    m.def("field_router_enabled", &field_router_enabled);
    m.def("gpu_cost_field_warm",
          [](int W, int H, int L, uint32_t via_q,
             const std::vector<int64_t>& seeds) -> py::object {
              const size_t cells = (size_t)L * H * W;
              std::vector<uint64_t> d(cells);
              bool ok;
              {
                  py::gil_scoped_release rel;
                  ok = gpu_cost_field_warm(W, H, L, via_q, seeds.data(),
                                           seeds.size(), d.data());
              }
              if (!ok) return py::none();
              return py::bytes((const char*)d.data(), d.size() * 8);
          },
          py::arg("W"), py::arg("H"), py::arg("L"), py::arg("via_q"),
          py::arg("seeds"));
    // GPU field router stage 1 (docs/project/gpu-field-router.md): explicit
    // cost grids as bytes (u32 le / u8), fields as bytes (u64 le microquanta).
    m.def("cpu_dijkstra_field",
          [](int W, int H, int L, py::bytes cost_b, py::bytes via_b,
             uint32_t via_q, const std::vector<int64_t>& seeds) {
              std::string cs = cost_b, vs = via_b;
              std::vector<uint32_t> cost(cs.size() / 4);
              std::memcpy(cost.data(), cs.data(), cs.size());
              std::vector<uint8_t> via(vs.begin(), vs.end());
              std::vector<uint64_t> d;
              {
                  py::gil_scoped_release rel;
                  d = dijkstra_field(W, H, L, cost, via, via_q, seeds);
              }
              return py::bytes((const char*)d.data(), d.size() * 8);
          },
          py::arg("W"), py::arg("H"), py::arg("L"), py::arg("cost"),
          py::arg("via_ok"), py::arg("via_q"), py::arg("seeds"));
    m.def("walk_field_descent",
          [](int W, int H, int L, py::bytes cost_b, py::bytes via_b,
             uint32_t via_q, py::bytes dist_b, int64_t start) {
              std::string cs = cost_b, vs = via_b, ds = dist_b;
              std::vector<uint32_t> cost(cs.size() / 4);
              std::memcpy(cost.data(), cs.data(), cs.size());
              std::vector<uint8_t> via(vs.begin(), vs.end());
              std::vector<uint64_t> dist(ds.size() / 8);
              std::memcpy(dist.data(), ds.data(), ds.size());
              return walk_field_descent(W, H, L, cost, via, via_q, dist,
                                        start);
          },
          py::arg("W"), py::arg("H"), py::arg("L"), py::arg("cost"),
          py::arg("via_ok"), py::arg("via_q"), py::arg("dist"),
          py::arg("start"));
    m.def("gpu_cost_field",
          [](int W, int H, int L, py::bytes cost_b, py::bytes via_b,
             uint32_t via_q, const std::vector<int64_t>& seeds) -> py::object {
              std::string cs = cost_b, vs = via_b;
              const size_t cells = (size_t)L * H * W;
              std::vector<uint64_t> d(cells);
              bool ok;
              {
                  py::gil_scoped_release rel;
                  ok = gpu_cost_field(W, H, L, (const uint32_t*)cs.data(),
                                      (const uint8_t*)vs.data(), via_q,
                                      seeds.data(), seeds.size(), d.data());
              }
              if (!ok) return py::none();
              return py::bytes((const char*)d.data(), d.size() * 8);
          },
          py::arg("W"), py::arg("H"), py::arg("L"), py::arg("cost"),
          py::arg("via_ok"), py::arg("via_q"), py::arg("seeds"));
    m.def("set_escalation_cap", &set_escalation_cap, py::arg("cap"),
          "Dijkstra expansions before wave-map escalation (default 100000)");
    m.def("set_wave_threads", &set_wave_threads, py::arg("n"),
          "Row-parallel wavefront threads (1 = serial; use >1 for interactive "
          "single-board routing where cores are otherwise idle)");
    m.def("set_unsat_check_enabled", &set_unsat_check_enabled, py::arg("on"),
          "Toggle the early-UNSAT reachability flood (default on; off = always "
          "run the full weighted search — for equivalence testing)");
    m.def("load_pcb_rdl",
          [](const std::string& json, double res) { return load_pcb_rdl(json, res); },
          py::arg("json"), py::arg("resolution"));
    m.def("load_kicad_pcb",
          [](const std::string& text, double res) { return load_kicad_pcb(text, res); },
          py::arg("contents"), py::arg("resolution"));

    // ---- Stage 10: resolution fidelity diagnostics ----
    m.def("recommended_resolution", &routing::recommended_resolution, py::arg("min_pad_spacing"));
    m.def("load_pcb_rdl_diagnostics",
          [](const std::string& json, double res) {
              PcbRdlInfo info;
              load_pcb_rdl(json, res, &info, /*reject_collisions=*/false);
              return py::dict("pin_collisions"_a=info.pin_collisions,
                              "min_pad_spacing"_a=info.min_pad_spacing,
                              "nets"_a=info.original_net_ids.size());
          },
          py::arg("json"), py::arg("resolution"));
    m.def("load_kicad_pcb_diagnostics",
          [](const std::string& text, double res) {
              KicadPcbInfo info;
              load_kicad_pcb(text, res, &info, /*reject_collisions=*/false);
              return py::dict("pin_collisions"_a=info.pin_collisions,
                              "min_pad_spacing"_a=info.min_pad_spacing);
          },
          py::arg("contents"), py::arg("resolution"));
}

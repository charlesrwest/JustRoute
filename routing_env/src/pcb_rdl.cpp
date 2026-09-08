#include "routing/pcb_rdl.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <stdexcept>

namespace routing {

namespace {

// Layer-name -> index mapping for the nets' pads.
struct RdlParse {
    double min_x, min_y, max_x, max_y;
    int width, height;
    std::map<std::string, int> layer_index; // layer name -> index
};

double border_min_x(const nlohmann::json& border) {
    double m = std::numeric_limits<double>::infinity();
    for (const auto& o : border) {
        if (!o.contains("vertices") || !o["vertices"].is_array()) continue;
        for (const auto& v : o["vertices"]) {
            if (v.is_array() && v.size() >= 1) m = std::min(m, v[0].get<double>());
        }
    }
    return m;
}
double border_max_x(const nlohmann::json& border) {
    double m = -std::numeric_limits<double>::infinity();
    for (const auto& o : border) {
        if (!o.contains("vertices") || !o["vertices"].is_array()) continue;
        for (const auto& v : o["vertices"]) if (v.is_array() && v.size() >= 1) m = std::max(m, v[0].get<double>());
    }
    return m;
}
double border_min_y(const nlohmann::json& border) {
    double m = std::numeric_limits<double>::infinity();
    for (const auto& o : border) {
        if (!o.contains("vertices") || !o["vertices"].is_array()) continue;
        for (const auto& v : o["vertices"]) if (v.is_array() && v.size() >= 2) m = std::min(m, v[1].get<double>());
    }
    return m;
}
double border_max_y(const nlohmann::json& border) {
    double m = -std::numeric_limits<double>::infinity();
    for (const auto& o : border) {
        if (!o.contains("vertices") || !o["vertices"].is_array()) continue;
        for (const auto& v : o["vertices"]) if (v.is_array() && v.size() >= 2) m = std::max(m, v[1].get<double>());
    }
    return m;
}

// A single pad, with the cells (layer,x,y) it occupies as pins.
struct Pin {
    Cell cell;
    int src_net; // original PCB-RDL net id (for info)
};
} // namespace

Board load_pcb_rdl(const std::string& json, double resolution, PcbRdlInfo* info,
                   bool reject_collisions) {
    if (resolution <= 0.0) throw std::runtime_error("resolution must be > 0");

    auto body = [&]() -> Board {
    nlohmann::json j = nlohmann::json::parse(json);

    // layers
    if (!j.contains("layers") || !j["layers"].is_array() || j["layers"].empty())
        throw std::runtime_error("PCB-RDL missing non-empty 'layers'");
    std::vector<std::string> layer_names;
    for (const auto& l : j["layers"]) layer_names.push_back(l.get<std::string>());
    std::map<std::string, int> layer_index;
    for (size_t i = 0; i < layer_names.size(); ++i) layer_index[layer_names[i]] = (int)i;
    // The Board uses consecutive copper layers; shift so F.Cu == 0 if present.
    // (PCB-RDL lists copper layers first; we use index as-is -> layer 0 = layer_names[0].)
    int n_layers = (int)layer_names.size();

    // border->bounds
    const nlohmann::json& border = j.contains("border") ? j["border"] : nlohmann::json::array();
    double min_x = border_min_x(border), max_x = border_max_x(border);
    double min_y = border_min_y(border), max_y = border_max_y(border);
    if (!std::isfinite(min_x) || !std::isfinite(max_x) ||
        !std::isfinite(min_y) || !std::isfinite(max_y) ||
        max_x <= min_x || max_y <= min_y) {
        throw std::runtime_error("PCB-RDL border does not define valid bounds");
    }

    if ((max_x - min_x) / resolution >= 32768.0 || (max_y - min_y) / resolution >= 32768.0)
        throw std::runtime_error("grid out of supported bounds");
    int width = (int)std::ceil((max_x - min_x) / resolution) + 1;
    int height = (int)std::ceil((max_y - min_y) / resolution) + 1;
    if (width <= 0 || height <= 0 || width > 32768 || height > 32768)
        throw std::runtime_error("PCB-RDL grid out of supported bounds");

    // nets
    if (!j.contains("nets") || !j["nets"].is_object())
        throw std::runtime_error("PCB-RDL missing 'nets' object");

    // Build temporary per-original-net pin lists.
    struct PadPin {
        Cell c; double radius; bool thru;
        // real footprint (cells, fractional center): rect or capsule, rotated
        double cx, cy, hw, hh, rot; bool oval;
    };
    std::vector<std::pair<int, std::vector<PadPin>>> net_pins; // (orig_net_id, pins)
    // Stage-10 fidelity diagnostics: cell key (layer*height+y)*width+x -> first original
    // centre; count DISTINCT centres that collapse to the same cell; collect one centre
    // per PHYSICAL pad (not per copper layer) plus its layer mask, so spacing compares
    // only pads that share a layer (a thru pad no longer measures 0 against itself and
    // stacked opposite-side pads no longer measure 0 against each other).
    std::map<size_t, std::pair<double, double>> cell_to_centre;
    std::vector<std::pair<double, double>> all_centres;
    std::vector<uint64_t> centre_layer_masks;
    int collisions = 0;
    for (auto it = j["nets"].begin(); it != j["nets"].end(); ++it) {
        int orig_id = std::stoi(it.key());
        const nlohmann::json& pads = it.value();
        if (!pads.is_array()) continue;
        std::vector<PadPin> pins;
        for (const auto& p : pads) {
            if (!p.contains("center") || !p["center"].is_array() || p["center"].size() < 2)
                continue;
            double cx = p["center"][0].get<double>();
            double cy = p["center"][1].get<double>();
            if (!p.contains("layer") || !p["layer"].is_array() || p["layer"].empty())
                continue; // no routable layer
            int x = (int)std::floor((cx - min_x) / resolution);
            int y = (int)std::floor((cy - min_y) / resolution);
            if (x < 0) { x = 0; }
            if (y < 0) { y = 0; }
            if (x >= width) { x = width - 1; }
            if (y >= height) { y = height - 1; }
            // Resolve the MAPPED copper layers first: the thru-hole heuristic and the
            // spacing diagnostics must count only real copper layers, not mask-only or
            // mechanical names (which produce no pins).
            std::vector<int> mapped;
            for (const auto& lname : p["layer"]) {
                auto it2 = layer_index.find(lname.get<std::string>());
                if (it2 == layer_index.end()) continue;
                int layer = it2->second;
                if (layer < 0 || layer >= n_layers) continue;
                mapped.push_back(layer);
            }
            if (mapped.empty()) continue; // no copper: no pins, no diagnostics
            // A pad's layer[] lists the copper layers it occupies; a thru-hole pad lists
            // several (or all). Add a pin on EVERY mapped copper layer so its barrel is
            // blocked on all layers (foreign copper cannot cross the column on any layer).
            const bool thru = mapped.size() > 1;
            // Real footprint: PCB-RDL 'radii' holds FULL extents [w, h] (verified: 0.2mm
            // FPC pads at 0.4mm pitch would overlap under a half-extent reading).
            double hw = 0.0, hh = 0.0, rot = 0.0;
            bool oval = true;
            if (p.contains("radii") && p["radii"].is_array() && p["radii"].size() >= 2) {
                hw = p["radii"][0].get<double>() * 0.5 / resolution;
                hh = p["radii"][1].get<double>() * 0.5 / resolution;
            }
            // NOTE: 'radii' are BOARD-FRAME extents — the RDL exporter already baked the
            // pad rotation in (the same physical pad appears as [0.25,0.875] or
            // [0.875,0.25] depending on orientation). The 'rotation' field is
            // informational; applying it again would double-rotate the footprint.
            (void)rot;
            if (p.contains("shape") && p["shape"].is_string()) {
                const std::string sh = p["shape"].get<std::string>();
                oval = (sh == "circle" || sh == "oval"); // rect/roundrect -> rectangle
            }
            const double pr_fallback = thru ? (kKiCadViaRadiusMM / resolution)
                                            : (0.1 / resolution);
            const double pr = std::max(hw, hh) > 0.0 ? std::max(hw, hh) : pr_fallback;
            if (hw <= 0.0 && hh <= 0.0) { hw = hh = pr_fallback; oval = true; }
            const double cxc = (cx - min_x) / resolution;   // fractional cell center
            const double cyc = (cy - min_y) / resolution;
            all_centres.emplace_back(cx, cy);   // once per PHYSICAL pad (diagnostics)
            uint64_t lmask = 0;
            for (int layer : mapped) {
                // fidelity: detect distinct centres collapsing to the same cell (per layer)
                const size_t key = ((size_t)layer * height + y) * width + x;
                auto inserted = cell_to_centre.emplace(key, std::make_pair(cx, cy));
                if (!inserted.second) {
                    const auto& prev = inserted.first->second;
                    if ((prev.first != cx || prev.second != cy) &&
                        (cx - prev.first) * (cx - prev.first) +
                            (cy - prev.second) * (cy - prev.second) > 1e-12)
                        collisions++;
                }
                pins.push_back(PadPin{Cell{layer, x, y}, pr, thru,
                                      cxc, cyc, hw, hh, rot, oval});
                lmask |= 1ull << std::min(layer, 63);
            }
            centre_layer_masks.push_back(lmask);
        }
        // de-duplicate identical pins within a net (same cell); several pins collapsing
        // to one cell keep the LARGEST radius (keepout/DRC must cover the biggest pad)
        std::vector<PadPin> uniq;
        for (const auto& c : pins) {
            bool has = false;
            for (auto& u : uniq)
                if (u.c == c.c) {
                    u.radius = std::max(u.radius, c.radius);
                    u.thru = u.thru || c.thru;
                    has = true; break;
                }
            if (!has) uniq.push_back(c);
        }
        if (uniq.size() >= 1) net_pins.emplace_back(orig_id, uniq);
    }

    // Construct the Board under the BOARD'S OWN design rules (rules.net_classes):
    // trace width, clearance, and via diameter. Imposing KiCad defaults instead was
    // measurably wrong in both directions — 0.2mm clearance is STRICTER than most
    // corpus boards' own rules (0.15-0.18mm), which made legal fine-pitch escapes
    // (0.4/0.5mm pitch QFN + FPC) provably unroutable. Multiple classes take the
    // most permissive value per rule; per-net class application is a future step.
    const double via_cost = 5.0, base_cost = 1.0;
    double rule_clr = 0.0, rule_width = 0.0, rule_via_d = 0.0;
    if (j.contains("rules") && j["rules"].is_object() &&
        j["rules"].contains("net_classes") && j["rules"]["net_classes"].is_array()) {
        for (const auto& nc : j["rules"]["net_classes"]) {
            auto take = [&](const char* key, double& dst) {
                if (nc.contains(key) && nc[key].is_number()) {
                    double v = nc[key].get<double>();
                    if (v > 0.0 && (dst <= 0.0 || v < dst)) dst = v;
                }
            };
            take("clearance", rule_clr);
            take("width", rule_width);
            take("via_diameter", rule_via_d);
        }
    }
    Board b(n_layers, width, height, resolution, via_cost, base_cost, rule_clr);
    b.set_rule_physics(rule_width > 0.0 ? rule_width * 0.5 / resolution : 0.0,
                       rule_via_d > 0.0 ? rule_via_d * 0.5 / resolution : 0.0);

    // No-net keepout pads (same schema as net pads): real copper obstacles that block
    // every net's routing but belong to none.
    if (j.contains("keepouts") && j["keepouts"].is_array()) {
        for (const auto& p : j["keepouts"]) {
            if (!p.contains("center") || !p["center"].is_array() || p["center"].size() < 2)
                continue;
            if (!p.contains("layer") || !p["layer"].is_array()) continue;
            double kcx = p["center"][0].get<double>();
            double kcy = p["center"][1].get<double>();
            double hw = 0.1 / resolution, hh = 0.1 / resolution;
            bool oval = true;
            if (p.contains("radii") && p["radii"].is_array() && p["radii"].size() >= 2) {
                hw = p["radii"][0].get<double>() * 0.5 / resolution;
                hh = p["radii"][1].get<double>() * 0.5 / resolution;
            }
            if (p.contains("shape") && p["shape"].is_string()) {
                const std::string sh = p["shape"].get<std::string>();
                oval = (sh == "circle" || sh == "oval");
            }
            for (const auto& lname : p["layer"]) {
                auto it2 = layer_index.find(lname.get<std::string>());
                if (it2 == layer_index.end()) continue;
                b.add_keepout_shape(it2->second, (kcx - min_x) / resolution,
                                    (kcy - min_y) / resolution, hw, hh, 0.0, oval);
            }
        }
    }

    for (size_t i = 0; i < net_pins.size(); ++i) {
        Net net;
        net.id = (int)i;
        net.pins.reserve(net_pins[i].second.size());
        for (const auto& pc : net_pins[i].second) net.pins.push_back(pc.c);
        b.nets().push_back(std::move(net));
        for (const auto& pc : net_pins[i].second) {
            b.add_pad_shape((size_t)i, pc.c, pc.cx, pc.cy, pc.hw, pc.hh, pc.rot, pc.oval);
            // Thru-hole pads pre-connect their column on every layer (drilled barrel):
            // layer transitions riding it are not router-placed vias.
            if (pc.thru) b.mark_thru_pad(i, pc.c.x, pc.c.y);
        }
    }

    // Stage-10 fidelity: report collapses and minimum distinct-pad spacing. Only pads
    // sharing at least one copper layer are compared (pads on disjoint layers cannot
    // collapse into the same cell).
    double min_sp = std::numeric_limits<double>::infinity();
    for (size_t a = 0; a < all_centres.size(); ++a)
        for (size_t q = a + 1; q < all_centres.size(); ++q) {
            if ((centre_layer_masks[a] & centre_layer_masks[q]) == 0) continue;
            double dx = all_centres[a].first - all_centres[q].first;
            double dy = all_centres[a].second - all_centres[q].second;
            double d = std::sqrt(dx * dx + dy * dy);
            if (d < min_sp) min_sp = d;
        }
    if (info) {
        info->layer_names = layer_names;
        info->original_net_ids.clear();
        for (const auto& np : net_pins) info->original_net_ids.push_back(np.first);
        info->border_min_x = min_x; info->border_min_y = min_y;
        info->border_max_x = max_x; info->border_max_y = max_y;
        info->pin_collisions = collisions;
        info->min_pad_spacing = min_sp;
    }
    // Stage-10 fidelity guard: coarser than the board's pad pitch silently collapses
    // distinct pads into one cell (loses pins); refuse rather than route a collapsed board.
    if (reject_collisions && collisions > 0) {
        throw std::runtime_error(
            std::string("load_pcb_rdl: resolution too coarse: ") + std::to_string(collisions) +
            " distinct pad(s) collapsed into the same cell; use a finer resolution (<= " +
            std::to_string(recommended_resolution(min_sp)) + " board units/cell)");
    }
    return b;
    };
    try {
        return body();
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error(std::string("malformed PCB-RDL JSON: ") + e.what());
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error(std::string("malformed PCB-RDL JSON: ") + e.what());
    }
}

} // namespace routing

#include "routing/synthetic.hpp"

#include <nlohmann/json.hpp>

#include <sstream>
#include <stdexcept>

namespace routing {

Board build_synthetic(const SyntheticSpec& spec) {
    Board b(spec.layers, spec.width, spec.height, spec.resolution,
            spec.via_cost, spec.base_cost, spec.design_rule_clearance);
    b.nets().reserve(spec.nets.size());
    for (size_t i = 0; i < spec.nets.size(); ++i) {
        Net net;
        net.id = (int)i;
        net.pins = spec.nets[i];
        b.nets().push_back(std::move(net));
        for (const Cell& p : spec.nets[i]) b.add_pad(i, p);
    }
    return b;
}

// Serialize a SyntheticSpec to a compact JSON string using nlohmann::json.
std::string synthetic_to_json(const SyntheticSpec& spec) {
    nlohmann::json j;
    j["layers"] = spec.layers;
    j["width"] = spec.width;
    j["height"] = spec.height;
    j["resolution"] = spec.resolution;
    j["via_cost"] = spec.via_cost;
    j["base_cost"] = spec.base_cost;
    j["design_rule_clearance"] = spec.design_rule_clearance;
    j["nets"] = nlohmann::json::array();
    for (const auto& net : spec.nets) {
        nlohmann::json pins = nlohmann::json::array();
        for (const Cell& c : net) {
            pins.push_back({c.layer, c.x, c.y});
        }
        j["nets"].push_back(std::move(pins));
    }
    return j.dump();
}

// Parse a SyntheticSpec from JSON, validating structure.
// Throws std::runtime_error on any malformed/out-of-contract input.
SyntheticSpec synthetic_from_json(const std::string& json) {
    try {
        nlohmann::json j = nlohmann::json::parse(json);

        if (!j.is_object()) throw std::runtime_error("synthetic spec must be a JSON object");
        auto need = [&](const char* k) -> const nlohmann::json& {
            if (!j.contains(k)) throw std::runtime_error(std::string("missing key '") + k + "'");
            return j[k];
        };

        SyntheticSpec spec;
        spec.layers = need("layers").get<int>();
        spec.width = need("width").get<int>();
        spec.height = need("height").get<int>();
        spec.resolution = need("resolution").get<double>();
        if (j.contains("via_cost")) spec.via_cost = j["via_cost"].get<double>();
        if (j.contains("base_cost")) spec.base_cost = j["base_cost"].get<double>();
        if (j.contains("design_rule_clearance")) spec.design_rule_clearance = j["design_rule_clearance"].get<double>();

        if (spec.layers <= 0 || spec.width <= 0 || spec.height <= 0 || spec.resolution <= 0.0)
            throw std::runtime_error("invalid synthetic spec dims");

        const nlohmann::json& nets = need("nets");
        if (!nets.is_array()) throw std::runtime_error("nets must be an array");
        for (const auto& net : nets) {
            if (!net.is_array()) throw std::runtime_error("net must be an array of pins");
            std::vector<Cell> pins;
            for (const auto& item : net) {
                if (!item.is_array() || item.size() != 3)
                    throw std::runtime_error("pin must be [layer,x,y]");
                Cell c;
                c.layer = item[0].get<int>();
                c.x = item[1].get<int>();
                c.y = item[2].get<int>();
                if (c.layer < 0 || c.layer >= spec.layers ||
                    c.x < 0 || c.x >= spec.width || c.y < 0 || c.y >= spec.height)
                    throw std::runtime_error("pin out of board bounds");
                pins.push_back(c);
            }
            spec.nets.push_back(std::move(pins));
        }
        return spec;
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error(std::string("malformed synthetic JSON: ") + e.what());
    }
}

} // namespace routing

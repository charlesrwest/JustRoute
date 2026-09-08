#pragma once

#include "routing/board.hpp"

#include <string>
#include <vector>

namespace routing {

// A small, self-contained description of a synthetic board used for tests and RL
// experimentation (before real benchmark import in Stage 3).
struct SyntheticSpec {
    int layers = 1;
    int width = 20;       // in cells
    int height = 20;      // in cells
    double resolution = 1.0;
    double via_cost = 5.0;
    double base_cost = 1.0;
    double design_rule_clearance = 0.0;

    // per net: list of pins (layer,x,y)
    std::vector<std::vector<Cell>> nets;
};

// Build a Board from a SyntheticSpec.
// Returns a Board with nets_ populated (in spec order) and pads marked.
Board build_synthetic(const SyntheticSpec& spec);

// Serialize a SyntheticSpec to a compact JSON string.
std::string synthetic_to_json(const SyntheticSpec& spec);

// Parse a SyntheticSpec from a JSON string produced by synthetic_to_json.
// Throws std::runtime_error on malformed input.
SyntheticSpec synthetic_from_json(const std::string& json);

} // namespace routing

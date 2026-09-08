#pragma once

#include "routing/board.hpp"

#include <string>
#include <limits>
#include <vector>

namespace routing {

// Parsed metadata from a PCB-RDL file, for validation / downstream use.
struct PcbRdlInfo {
    std::vector<std::string> layer_names; // index == layer index used by Board
    std::vector<int> original_net_ids;    // original PCB-RDL net id per Board net index
    double border_min_x = 0.0, border_min_y = 0.0;
    double border_max_x = 0.0, border_max_y = 0.0;
    // Fidelity diagnostics (Stage 10): pin_collisions is the number of DISTINCT physical
    // pad centers that quantized to the same grid cell (must be 0 for real routing).
    // min_pad_spacing is the min Euclidean distance between distinct pad centers.
    int pin_collisions = 0;
    double min_pad_spacing = std::numeric_limits<double>::infinity();
};

// Default routing resolution (board units per cell, mm): 0.05 mm ≈ fine fidelity. Use this
// (or finer) for real boards; do not use 1.0 mm (too coarse, collapses pads).
constexpr double DEFAULT_RESOLUTION_MM = 0.05;

// Load a PCB-RDL JSON string (PCBench `final.json`) into a Board.
// `resolution` is the grid resolution in board units (the PCB-RDL `unit`, e.g. mm).
// When `reject_collisions` is true (default) and coarser than the board's pad pitch causes
// distinct pads to quantize into the same cell, an std::runtime_error is thrown (fidelity
// guard). Pass false only for diagnostics/testing of the collision count itself.
// Nets are assigned sequential Board net ids; their original PCB-RDL ids are returned
// (if `info` is non-null). A thru-hole pad is represented by a single pin on its first
// copper layer (the pad's hole connects its layers), so it is NOT over-counted into one
// pin per layer.
//
// Assumptions / documented behaviors:
//  - PCBench `layers[0]` is a copper layer (F.Cu) and is mapped to Board layer 0.
//    Non-copper-first layer lists are not handled (routing lives on copper).
//  - `original_net_ids` are in the JSON object's (lexicographic by net-key) order, NOT
//    numeric order. Board net i preserves the pairing with original_net_ids[i].
//  - Distinct pads closer than `resolution` may quantize to the same cell and are
//    de-duplicated (a self-connection is meaningless). For boards with closely-spaced
//    pads, use a finer resolution than the pad spacing to avoid losing pins.
//  - Out-of-bounds pads (falling outside the border box) are clamped to the grid edge
//    rather than discarded; for well-formed boards this never occurs.
//
// Throws std::runtime_error on any malformed/out-of-contract input (including
// type-mismatched fields and non-numeric net keys).
Board load_pcb_rdl(const std::string& json, double resolution, PcbRdlInfo* info = nullptr,
                   bool reject_collisions = true);

// Stage-10: board-units-per-cell so the closest distinct pad centers span >= 2 cells
// (avoids pin collapse). Pass a bound like min(min_pad_spacing, min feature size * 2);
// the caller then usually snaps to a standard design grid >= this value.
inline double recommended_resolution(double min_pad_spacing) {
    return std::max(min_pad_spacing / 2.0, 1e-12);
}

} // namespace routing

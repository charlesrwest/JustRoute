#pragma once

#include "routing/board.hpp"

#include <string>
#include <limits>
#include <vector>

namespace routing {

// Parsed metadata from a KiCad .kicad_pcb file, for validation / downstream use.
struct KicadPcbInfo {
    std::vector<std::string> layer_names;       // board copper layer names in Board layer order
    std::vector<int> original_net_ids;          // original KiCad net id per Board net index
    // v11 name-keyed dialect (no net codes in the file): the net NAME per Board
    // net index; empty for numeric-coded files. Writers must emit (net "NAME")
    // when nonempty (original_net_ids then holds synthetic negative codes).
    std::vector<std::string> net_names;
    double border_min_x = 0.0, border_min_y = 0.0;
    double border_max_x = 0.0, border_max_y = 0.0;
    // Nets with existing routed copper in the file (root segment/via/arc). They are
    // NOT board nets: their copper and pads load as fixed obstacles, the file's
    // routing stays in place, and only the remaining nets are routed (incremental).
    int pre_routed_nets = 0;
    int partial_nets = 0;   // nets with copper that does NOT connect all pads
    std::vector<std::string> pour_fed_nets; // skipped as pour-fed (skip_poured)
    // Effective design rules parsed from the file (net classes + setup minima,
    // v4/v5 dialect), mm; 0 = not present (v6+ keeps them in the project file).
    // rule_* are the MODEL values (max across populated classes — the uniform
    // swath must satisfy every pair). default_* are the Default class's own
    // values; net_widths / net_via_* give each net's OWN class geometry (0 =
    // Default/unknown) so writers emit proper per-net tracks and vias.
    double rule_clearance = 0.0, rule_track_width = 0.0;
    double rule_via_dia = 0.0, rule_via_drill = 0.0;
    double default_track_width = 0.0, default_via_dia = 0.0, default_via_drill = 0.0;
    std::vector<double> net_widths, net_via_dias, net_via_drills;
    // Fidelity diagnostics (Stage 10): see PcbRdlInfo.
    int pin_collisions = 0;
    double min_pad_spacing = std::numeric_limits<double>::infinity();
};

// Load a KiCad .kicad_pcb (S-expression) file into a Board and autoroute-capable model.
// Supports the two dialects found in the corpus:
//   - KiCad v4/5:  (module ...) with unquoted layer names and (at x y rot) on module+pad.
//   - KiCad v6+:   (footprint ...) with (possibly quoted) layer names; pads may put their
//                  (net id "name") on a continuation line, so a real S-expression tokenizer
//                  is used (not line-based parsing).
//
// `resolution` is the grid resolution in board units (KiCad uses mm). Nets are assigned
// sequential Board net ids; their original KiCad net ids are returned (if `info` is non-null).
//
// Behavior / documented choices:
//  - Only COPPER layers become routable Board layers, in listed order (F.Cu -> 0, B.Cu -> next,
//    then inner In*.Cu). Non-copper layers (silk/paste/mask/Edge.Cuts, etc.) are ignored.
//  - `(net 0 "")` is the unconnected net and is skipped. Only nets with >= 1 pad become Board
//    nets (mirrors the PCB-RDL importer). Nets that exist but have no pad are dropped.
//  - Pad absolute center = module/footprint origin (at mx my [rot]) + the pad's (at px py [rot])
//    rotated by the module rotation. Thru-hole pads contribute a pin on their first copper layer
//    (the hole connects layers, consistent with the PCB-RDL convention); SMD pads use their
//    single copper layer. Pads with no (net ...) or a net id of 0 are treated as unconnected and
//    skipped.
//  - Board bounds are taken from `(general (area ...))` when present, else from pad extents,
//    plus a margin; the grid is sized from bounds/resolution.
//  - Existing `(segment ...)` / `(via ...)` / `(arc ...)` routing marks its net as
//    PRE-ROUTED: that net is dropped from the routable set and its copper + pads load
//    as fixed no-net obstacles (incremental routing honors them; the writer appends).
//    Copper pours are not obstacles — KiCad refills them around new tracks (judge
//    with kicad-cli drc --refill-zones).
//  - Out-of-bounds pads (outside the computed bounds) are clamped to the grid edge rather than
//    discarded (mirrors the RDL importer's documented choice).
//
// Throws std::runtime_error on any malformed/out-of-contract input (unbalanced S-expressions,
// missing required sections, type mismatches) -- normalized like the PCB-RDL importer. When
// `reject_collisions` is true (default) and the resolution collapses distinct pads, it also
// throws (fidelity guard); pass false for diagnostics/testing only.
// skip_poured: nets owning a copper pour are treated as pre-routed even
// without tracks — the refilled zone is trusted to connect them (the
// KiCad-plugin default; campaigns/gates load with false).
Board load_kicad_pcb(const std::string& file_contents, double resolution,
                     KicadPcbInfo* info = nullptr, bool reject_collisions = true,
                     bool skip_poured = false);

} // namespace routing

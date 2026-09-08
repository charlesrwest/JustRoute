#include "routing/kicad_pcb.hpp"
#include "routing/newstroke_ascii.hpp"
#include "routing/pcb_rdl.hpp"  // recommended_resolution (Stage-10 fidelity)

#include <algorithm>
#include <cmath>
#include <functional>
#include <cstring>
#include <cctype>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace routing {

namespace {

// -------- lightweight S-expression model --------
struct SExpr {
    std::string atom;          // value when this is an atom (empty for a list)
    std::vector<SExpr> kids;   // children when is_list
    bool is_list = false;
    bool is_quoted = false;    // atom came from a "..." string
};

// -------- tokenizer / parser --------
// Tokenizes `(`, `)`, plain atoms, and quoted strings; builds a nested tree.
// Throws std::runtime_error on unbalanced or malformed input (normalized below).
class SExprParser {
public:
    explicit SExprParser(const std::string& s) : s_(s) {}

    // Parse a whole document; returns the top-level list that must enclose everything.
    SExpr parse() {
        skip_ws();
        if (pos_ >= s_.size() || s_[pos_] != '(')
            throw std::runtime_error("KiCad PCB: expected '(' at top level");
        SExpr root = parse_list();
        skip_ws();
        if (pos_ < s_.size())
            throw std::runtime_error("KiCad PCB: trailing content after top-level list");
        return root;
    }

private:
    const std::string& s_;
    size_t pos_ = 0;
    // Cap list nesting so adversarial deeply-nested input throws std::runtime_error
    // instead of overflowing the stack (real KiCad files nest only a handful of levels).
    static constexpr size_t kMaxDepth = 512;
    size_t depth_ = 0;

    void skip_ws() {
        while (pos_ < s_.size()) {
            char c = s_[pos_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') { pos_++; continue; }
            if (c == ';') { while (pos_ < s_.size() && s_[pos_] != '\n') pos_++; continue; }
            break;
        }
    }

    SExpr parse_list() {
        if (depth_ >= kMaxDepth)
            throw std::runtime_error("KiCad PCB: S-expression nesting too deep");
        depth_++;
        SExpr n; n.is_list = true;
        if (pos_ >= s_.size() || s_[pos_] != '(')
            throw std::runtime_error("KiCad PCB: expected '('");
        pos_++; // consume '('
        for (;;) {
            skip_ws();
            if (pos_ >= s_.size()) {
                depth_--;
                throw std::runtime_error("KiCad PCB: unbalanced ')' (EOF inside list)");
            }
            char c = s_[pos_];
            if (c == ')') { pos_++; break; }
            if (c == '(') { n.kids.push_back(parse_list()); continue; }
            n.kids.push_back(parse_atom_or_string());
        }
        depth_--;
        return n;
    }

    SExpr parse_atom_or_string() {
        SExpr a; a.is_list = false;
        if (s_[pos_] == '"') {
            a.is_quoted = true;
            pos_++;
            std::string v;
            while (pos_ < s_.size() && s_[pos_] != '"') { v.push_back(s_[pos_]); pos_++; }
            if (pos_ >= s_.size())
                throw std::runtime_error("KiCad PCB: unterminated quoted string");
            pos_++; // closing quote
            a.atom = v;
            return a;
        }
        std::string v;
        while (pos_ < s_.size()) {
            char c = s_[pos_];
            if (c==' '||c=='\t'||c=='\n'||c=='\r'||c=='('||c==')'||c=='"'||c==';') break;
            v.push_back(c); pos_++;
        }
        if (v.empty())
            throw std::runtime_error("KiCad PCB: empty atom");
        a.atom = v;
        return a;
    }
};

// -------- navigation helpers --------
// The child list of `n` whose first atom equals `name`, or nullptr.
const SExpr* find_child(const SExpr& n, const std::string& name) {
    for (const SExpr& k : n.kids) {
        if (k.is_list && !k.kids.empty() && !k.kids[0].is_list && k.kids[0].atom == name)
            return &k;
    }
    return nullptr;
}

// All child lists of `n` whose first atom equals `name`.
std::vector<const SExpr*> find_children(const SExpr& n, const std::string& name) {
    std::vector<const SExpr*> out;
    for (const SExpr& k : n.kids) {
        if (k.is_list && !k.kids.empty() && !k.kids[0].is_list && k.kids[0].atom == name)
            out.push_back(&k);
    }
    return out;
}

// Returns the pad's (net id) if present, else 0 (unconnected). Throws nothing.
int pad_net_id(const SExpr& pad) {
    const SExpr* net = find_child(pad, "net");
    if (!net) return 0;
    if (net->kids.size() < 2) return 0;
    const SExpr& id = net->kids[1];
    if (id.is_list) return 0;
    try { return (int)std::lround(std::stod(id.atom)); }
    catch (...) { return 0; }
}

// Pad type token: (pad "name" thru_hole|np_thru_hole|smd|connect shape ...). Drilled
// pads (thru_hole/np_thru_hole) have a plated barrel pre-connecting every copper layer.
bool pad_is_thru(const SExpr& pad) {
    if (pad.kids.size() >= 3 && !pad.kids[2].is_list) {
        const std::string& t = pad.kids[2].atom;
        return t == "thru_hole" || t == "np_thru_hole";
    }
    return false;
}

// Pad footprint in mm: half extents from the real (size w h), shape kind from the
// shape token (kids[3]: circle|oval|rect|roundrect|trapezoid|custom), and the pad's
// own rotation (3rd value of its (at ...), which in KiCad files already includes the
// parent footprint rotation). Rect-like shapes map to rectangles (roundrect/trapezoid
// slightly conservatively); circle/oval map to capsules. Falls back to the legacy
// two-value circle heuristic when (size ...) is absent or malformed.
struct PadGeom {
    double half_w = 0.0, half_h = 0.0, rot = 0.0;
    bool oval = true;
    // (drill (offset ox oy)): KiCad shifts the COPPER SHAPE relative to the pad
    // position (the pin/hole stays at (at)). Local frame, mm; rotated with the pad.
    double off_x = 0.0, off_y = 0.0;
};
PadGeom pad_geom_mm(const SExpr& pad, bool thru) {
    PadGeom g;
    if (pad.kids.size() >= 4 && !pad.kids[3].is_list) {
        const std::string& sh = pad.kids[3].atom;
        g.oval = (sh == "circle" || sh == "oval");
    }
    if (const SExpr* at = find_child(pad, "at")) {
        try {
            if (at->kids.size() >= 4 && !at->kids[3].is_list)
                // Same Y-down sign flip as rotate_deg(): KiCad angles are CCW on
                // screen, which is CW in raw (x, y-down) math. Without this, a
                // rotated pad's model copper leans on the WRONG diagonal — invisible
                // at 0/90/180 (mirror symmetric) but real at 45 (via-pad clearance
                // violations against slanted fine-pitch pads, jackco ESP32 module).
                g.rot = -std::stod(at->kids[3].atom);
        } catch (...) {}
    }
    if (const SExpr* dr = find_child(pad, "drill")) {
        if (const SExpr* off = find_child(*dr, "offset")) {
            if (off->kids.size() >= 3 && !off->kids[1].is_list && !off->kids[2].is_list) {
                try {
                    g.off_x = std::stod(off->kids[1].atom);
                    g.off_y = std::stod(off->kids[2].atom);
                } catch (...) {}
            }
        }
    }
    if (const SExpr* size = find_child(pad, "size")) {
        try {
            if (size->kids.size() >= 2 && !size->kids[1].is_list) {
                double w = std::stod(size->kids[1].atom);
                double h = (size->kids.size() >= 3 && !size->kids[2].is_list)
                               ? std::stod(size->kids[2].atom) : w;
                if (w > 0.0 && h > 0.0) {
                    g.half_w = w * 0.5;
                    g.half_h = h * 0.5;
                    // Trapezoid pads: (rect_delta a b) widens one edge by the
                    // delta — the conservative rect grows by half the max delta
                    // per axis (measured 0.047mm shortfall on solder-jumper
                    // trapezoids without it).
                    if (const SExpr* rd = find_child(pad, "rect_delta")) {
                        double dmax = 0.0;
                        for (size_t k2 = 1; k2 < rd->kids.size(); ++k2) {
                            if (rd->kids[k2].is_list) continue;
                            try {
                                dmax = std::max(dmax,
                                                std::abs(std::stod(rd->kids[k2].atom)));
                            } catch (...) {}
                        }
                        g.half_w += dmax * 0.5;
                        g.half_h += dmax * 0.5;
                        g.oval = false;
                    }
                    // Custom pads: (size) is only the ANCHOR; real copper is the
                    // (primitives ...) block (solder jumpers, RF stubs). Expand to
                    // the primitives' bbox, symmetric per axis (conservative rect) —
                    // else the router crosses copper it cannot see (shorting_items).
                    if (const SExpr* prim = find_child(pad, "primitives")) {
                        g.oval = false;
                        double ex = g.half_w, ey = g.half_h;
                        auto grow = [&](double x, double y, double pad2) {
                            ex = std::max(ex, std::abs(x) + pad2);
                            ey = std::max(ey, std::abs(y) + pad2);
                        };
                        for (const SExpr& pe : prim->kids) {
                            if (!pe.is_list || pe.kids.empty() || pe.kids[0].is_list)
                                continue;
                            double pw = 0.0;
                            if (const SExpr* wd = find_child(pe, "width")) {
                                if (wd->kids.size() >= 2 && !wd->kids[1].is_list) {
                                    try { pw = std::stod(wd->kids[1].atom) * 0.5; }
                                    catch (...) {}
                                }
                            }
                            if (const SExpr* pts = find_child(pe, "pts")) {
                                for (const SExpr& pt : pts->kids) {
                                    if (!pt.is_list || pt.kids.size() < 3 ||
                                        pt.kids[0].is_list || pt.kids[0].atom != "xy")
                                        continue;
                                    try {
                                        grow(std::stod(pt.kids[1].atom),
                                             std::stod(pt.kids[2].atom), pw);
                                    } catch (...) {}
                                }
                            }
                            for (const char* key : {"start", "end", "mid", "center"}) {
                                if (const SExpr* c = find_child(pe, key)) {
                                    if (c->kids.size() >= 3 && !c->kids[1].is_list &&
                                        !c->kids[2].is_list) {
                                        try {
                                            grow(std::stod(c->kids[1].atom),
                                                 std::stod(c->kids[2].atom), pw);
                                        } catch (...) {}
                                    }
                                }
                            }
                        }
                        g.half_w = ex;
                        g.half_h = ey;
                    }
                    return g;
                }
            }
        } catch (...) { /* fall through to heuristic */ }
    }
    g.half_w = g.half_h = thru ? kKiCadViaRadiusMM : 0.1;
    g.oval = true;
    return g;
}

// True iff the pad has any copper presence at all. Paste/mask-only aperture pads
// (common as no-net stencil apertures) must not become phantom copper obstacles.
// A missing (layers ...) means the old v4 dialect's default: a copper pad.
bool pad_on_copper(const SExpr& pad) {
    const SExpr* layers = find_child(pad, "layers");
    if (!layers) return true;
    for (const SExpr& l : layers->kids) {
        if (l.is_list) continue;
        const std::string& nm = l.atom;
        if (nm == "*.Cu" || nm == "F&B.Cu" ||
            (nm.size() > 3 && nm.compare(nm.size() - 3, 3, ".Cu") == 0))
            return true;
    }
    return false;
}

// The pad's copper Board layer for its pin. Reads the pad's `(layers ...)` copper
// All Board copper layers a pad occupies. A thru-hole pad (layers `*.Cu` / `F&B.Cu`, or
// explicit multiple copper names) touches EVERY copper layer so its barrel is blocked on
// all of them (a foreign net cannot cross the column on a layer we didn't register).
std::vector<int> pad_copper_layers(const SExpr& pad,
                                   const std::map<std::string, int>& layer_index,
                                   int n_layers) {
    const SExpr* layers = find_child(pad, "layers");
    std::vector<int> out;
    if (!layers) { out.push_back(0); return out; }
    bool all = false;
    std::set<int> named;
    for (const SExpr& l : layers->kids) {
        if (l.is_list) continue;
        const std::string& nm = l.atom;
        if (nm == "*.Cu" || nm == "F&B.Cu") all = true;      // thru-hole / all-copper
        auto it = layer_index.find(nm);
        if (it != layer_index.end()) named.insert(it->second);
    }
    if (all)
        for (int i = 0; i < n_layers; ++i) out.push_back(i);
    else if (!named.empty())
        for (int li : named) out.push_back(li);
    else
        out.push_back(0);
    return out;
}

// Is a layer name a copper layer?
inline bool is_copper_name(const std::string& name) {
    if (name == "F.Cu" || name == "B.Cu" || name == "F&B.Cu" || name == "*.Cu") return true;
    // inner layers In1.Cu .. In30.Cu
    if (name.size() >= 4 && name.compare(0, 2, "In") == 0 && name.size() > 2 &&
        name.compare(name.size() - 3, 3, ".Cu") == 0) {
        // In<N>.Cu
        std::string idx = name.substr(2, name.size() - 5);
        if (!idx.empty() && std::all_of(idx.begin(), idx.end(),
                                        [](char c){ return c >= '0' && c <= '9'; }))
            return true;
    }
    return false;
}

// Rotate a local (x,y) by `deg` degrees about the origin. KiCad board coordinates are
// Y-DOWN (positive y = down), so KiCad's positive rotation direction is the OPPOSITE of
// standard math CCW (Y-up). We therefore negate the angle so that a (module 'at ... rot')
// rotation places pads on the same side as KiCad (verified against fixture segment traces).
void rotate_deg(double& x, double& y, double deg) {
    if (deg == 0.0) return;
    double r = -deg * M_PI / 180.0;   // note the negation (Y-down sign convention)
    double c = std::cos(r), s = std::sin(r);
    double nx = x * c - y * s;
    double ny = x * s + y * c;
    x = nx; y = ny;
}

} // namespace

Board load_kicad_pcb(const std::string& contents, double resolution, KicadPcbInfo* info,
                     bool reject_collisions, bool skip_poured) {
    if (resolution <= 0.0) throw std::runtime_error("KiCad PCB: resolution must be > 0");

    auto body = [&]() -> Board {
        SExpr root = SExprParser(contents).parse();
        // File format version (root (version N)): some conventions are
        // dialect-dependent (text vertical anchor, below).
        long file_version = 0;
        if (const SExpr* ver = find_child(root, "version")) {
            if (ver->kids.size() >= 2 && !ver->kids[1].is_list) {
                try { file_version = std::lround(std::stod(ver->kids[1].atom)); }
                catch (...) {}
            }
        }

        // ---- copper layer mapping (in listed order) ----
        const SExpr* layers = find_child(root, "layers");
        if (!layers) throw std::runtime_error("KiCad PCB: missing 'layers'");
        std::vector<std::string> copper_names;         // in Board layer order
        std::map<std::string, int> layer_index;        // copper name -> Board layer
        for (const SExpr& ent : layers->kids) {
            if (!ent.is_list || ent.kids.size() < 3) continue;
            // (idx name type [alias]) -> name = kids[1], type = kids[2]. Copper
            // layers carry type signal|power|mixed|jumper (power planes are
            // 'power', NOT 'signal' — a 4-layer board with power inners loaded
            // as 2-layer before this). A renamed layer keeps its canonical name
            // in kids[1] with the user alias in kids[3]; tracks may reference
            // either, so both map to the same Board layer. The alias (KiCad's
            // display/serialization name when present) is what the writer emits.
            const SExpr& name = ent.kids[1];
            const SExpr& type = ent.kids[2];
            if (name.is_list || type.is_list) continue;
            if (type.atom != "signal" && type.atom != "power" &&
                type.atom != "mixed" && type.atom != "jumper")
                continue;
            const std::string nm = name.atom;
            if (!is_copper_name(nm)) continue;
            std::string alias;
            if (ent.kids.size() >= 4 && !ent.kids[3].is_list)
                alias = ent.kids[3].atom;
            if (layer_index.find(nm) == layer_index.end()) {
                int board_layer = (int)copper_names.size();
                copper_names.push_back(alias.empty() ? nm : alias);
                layer_index[nm] = board_layer;
                if (!alias.empty()) layer_index[alias] = board_layer;
            }
        }
        if (copper_names.empty())
            throw std::runtime_error("KiCad PCB: board has no copper layers");
        const int n_layers = (int)copper_names.size();

        // ---- design rules: Default net class + setup minima (v4/v5 dialect) ----
        // v6+ keeps net classes in the project file, not the pcb; there these stay 0
        // and the loader's KiCad-default constants apply. Values are mm; 0 = absent.
        // Routing/emission both use them: routing via Board physics (set_rule_physics /
        // ctor clearance), emission via KicadPcbInfo so the writer emits legal widths
        // (kicad-cli flags track_width/via_diameter/drill_out_of_range otherwise).
        double rule_clearance = 0.0, rule_tw = 0.0, rule_via = 0.0, rule_drill = 0.0;
        auto num_child = [](const SExpr& e, const char* key) -> double {
            if (const SExpr* c = find_child(e, key)) {
                if (c->kids.size() >= 2 && !c->kids[1].is_list) {
                    try { return std::stod(c->kids[1].atom); } catch (...) {}
                }
            }
            return 0.0;
        };
        // Per-net-class physics, uniform-model projection:
        //  - model clearance/width/via = MAX across Default and populated classes
        //    (the single swath must satisfy every pair; conservative for
        //    thin-class nets on mixed boards — true per-net-pair physics is a
        //    later core stage);
        //  - each net's OWN class width/via sizes are recorded by NAME so the
        //    writer emits proper per-net geometry (a power net gets its 0.6mm
        //    track). Emitted copper never exceeds the modeled swath, so every
        //    model-verified clearance holds a fortiori.
        struct ClassRule { double tw = 0.0, via = 0.0, drill = 0.0, clr = 0.0; };
        std::map<std::string, ClassRule> member_rules;                 // net name -> class
        double default_tw = 0.0, default_via = 0.0, default_drill = 0.0;
        double default_clr = 0.0;
        for (const SExpr& ent : root.kids) {
            if (!ent.is_list || ent.kids.size() < 2 || ent.kids[0].is_list) continue;
            if (ent.kids[0].atom != "net_class") continue;
            const bool is_default = !ent.kids[1].is_list && ent.kids[1].atom == "Default";
            const double cls_clr = num_child(ent, "clearance");
            const double cls_tw = num_child(ent, "trace_width");
            const double cls_via = num_child(ent, "via_dia");
            const double cls_drill = num_child(ent, "via_drill");
            const auto members = find_children(ent, "add_net");
            if (is_default || !members.empty()) {
                rule_clearance = std::max(rule_clearance, cls_clr);
                rule_tw = std::max(rule_tw, cls_tw);
                rule_via = std::max(rule_via, cls_via);
                rule_drill = std::max(rule_drill, cls_drill);
            }
            if (is_default) {
                default_tw = cls_tw;
                default_via = cls_via;
                default_drill = cls_drill;
                default_clr = cls_clr;
            }
            for (const SExpr* mem : members) {
                if (mem->kids.size() >= 2 && !mem->kids[1].is_list)
                    member_rules[mem->kids[1].atom] =
                        ClassRule{cls_tw, cls_via, cls_drill, cls_clr};
            }
        }
        // The model's ENFORCED MINIMUM clearance is the Default class (tier 0);
        // every growth/deficit computed against "what the model guarantees"
        // must use this, not the class max (a max-based growth under-protects
        // when Default < max: stompboxes guard rings regressed 0 -> 56).
        const double enforced_clr_mm =
            (default_clr > 0.0 ? default_clr
                               : (rule_clearance > 0.0 ? rule_clearance
                                                       : kKiCadClearanceMM));
        double setup_mask_margin = 0.0;
        if (const SExpr* setup = find_child(root, "setup")) {
            rule_tw = std::max(rule_tw, num_child(*setup, "trace_min"));
            rule_via = std::max(rule_via, num_child(*setup, "via_min_size"));
            rule_drill = std::max(rule_drill, num_child(*setup, "via_min_drill"));
            setup_mask_margin = num_child(*setup, "pad_to_mask_clearance");
        }
        if (rule_drill > 0.0 && rule_via > 0.0 && rule_drill > rule_via)
            rule_drill = rule_via * 0.5;   // malformed file; keep drill < diameter

        // ---- net id/name table ----
        // (net id name) at top level. We only need ids; net 0 = unconnected.
        // Build declared-order list of net ids (nonzero) with pads as we collect them.

        // ---- collect pads via modules / footprints ----
        // absolute pad position (x,y, board units), copper layer, and net id
        struct AbsPin { double x, y; int src_net; int layer; double radius; bool thru;
                        double half_w, half_h, rot; bool oval;
                        double shape_dx = 0.0, shape_dy = 0.0; };  // copper offset, world mm
        std::vector<AbsPin> all_pins;
        // Diagnostics: one centre per PHYSICAL pad plus its copper-layer mask, so min
        // spacing compares only pads sharing a layer (a thru pad no longer measures 0
        // against its own per-layer copies).
        std::vector<std::pair<double, double>> all_centres;
        std::vector<uint64_t> centre_layer_masks;
        // No-net pads (mounting holes, NPTH, fiducials, unconnected pins): not
        // routable, but their copper/holes are obstacles every net must clear.
        struct AbsObstacle { double x, y; int layer; double half_w, half_h, rot;
                             bool oval; bool via_only = false; };
        std::vector<AbsObstacle> obstacles;

        // ---- net key resolver ----
        // v4..v10: every net reference is (net CODE [name]) — key by CODE.
        // v11 (version 2026xxxx): no net table, references are (net "NAME") —
        // key by NAME through synthetic negative codes (stable within a load).
        std::map<std::string, int> name_codes;
        std::vector<std::string> code_names;   // synthetic code -k -> name (idx k-1)
        // Seed declared names with their POSITIVE codes so a name-keyed ref
        // and a numeric ref to the same net resolve to the SAME key (a board
        // may mix dialects; keys must unify or pre-routed accounting splits).
        for (const SExpr& ent : root.kids) {
            if (!ent.is_list || ent.kids.size() < 3 || ent.kids[0].is_list) continue;
            if (ent.kids[0].atom != "net" || ent.kids[1].is_list) continue;
            if (ent.kids[2].is_list || ent.kids[2].atom.empty()) continue;
            try {
                name_codes.emplace(ent.kids[2].atom,
                                   (int)std::lround(std::stod(ent.kids[1].atom)));
            } catch (...) { /* not a numeric decl */ }
        }
        auto net_key = [&](const SExpr& form) -> int {
            const SExpr* net = find_child(form, "net");
            if (!net || net->kids.size() < 2 || net->kids[1].is_list) return 0;
            const SExpr& a = net->kids[1];
            if (!a.is_quoted) {
                try { return (int)std::lround(std::stod(a.atom)); }
                catch (...) { /* unquoted name: fall through */ }
            }
            if (a.atom.empty()) return 0;
            auto ins = name_codes.emplace(a.atom, -(int)(code_names.size() + 1));
            if (ins.second) code_names.push_back(a.atom);
            return ins.first->second;
        };

        // ---- existing routing pre-scan ----
        // Root-level (segment)/(via)/(arc) forms are the board's already-routed
        // copper. A net is PRE-ROUTED — dropped from the routable set, its file
        // copper left in place (the writer only appends), its pads and tracks
        // fixed obstacles — only when that copper CONNECTS ALL ITS PADS. A net
        // with dangling partial copper (a cancelled run, a user's partial
        // rip-up) stays ROUTABLE: its stubs remain obstacles (safe: same-net
        // detours, never shorts) and the router completes it pad-to-pad.
        // Copper POURS ((zone ... (filled_polygon ...))) are deliberately NOT
        // obstacles: KiCad refills pours around new tracks (certification runs
        // kicad-cli drc --refill-zones so the judge sees refreshed fills).
        // n < 0 keys are name-keyed nets (KiCad 10+ saves (net "NAME") on
        // copper): every nonzero key is a real net owning copper.
        struct CuSeg { double x0, y0, x1, y1, w; int layer; };
        struct CuVia { double x, y, sz; };
        std::map<int, std::vector<CuSeg>> cu_segs;
        std::map<int, std::vector<CuVia>> cu_vias;
        std::set<int> zone_glue_nets;   // nets whose POUR may finish the job
        for (const SExpr& ent : root.kids) {
            if (!ent.is_list || ent.kids.empty() || ent.kids[0].is_list) continue;
            const std::string& tag = ent.kids[0].atom;
            if (tag == "zone" && !find_child(ent, "keepout")) {
                // a net-owning pour connects what tracks alone may not —
                // KiCad refills it around anything we add, so its verdict on
                // "all pads reached" belongs to the pour, not to us
                int zn = net_key(ent);
                if (zn != 0) zone_glue_nets.insert(zn);
                continue;
            }
            if (tag != "segment" && tag != "via" && tag != "arc") continue;
            int n = net_key(ent);   // reads the (net ...) child of any form
            if (n == 0) continue;
            auto num = [&](const char* key, int idx, double dflt) {
                const SExpr* c = find_child(ent, key);
                if (!c || (int)c->kids.size() <= idx || c->kids[(size_t)idx].is_list)
                    return dflt;
                try { return std::stod(c->kids[(size_t)idx].atom); }
                catch (...) { return dflt; }
            };
            if (tag == "via") {
                cu_vias[n].push_back({num("at", 1, 0.0), num("at", 2, 0.0),
                                      num("size", 1, 0.6)});
                continue;
            }
            int lay = 0;
            if (const SExpr* l = find_child(ent, "layer")) {
                if (l->kids.size() >= 2 && !l->kids[1].is_list) {
                    auto it = layer_index.find(l->kids[1].atom);
                    if (it != layer_index.end()) lay = it->second;
                }
            }
            double w = num("width", 1, 0.2);
            double sx = num("start", 1, 0.0), sy = num("start", 2, 0.0);
            double ex = num("end", 1, 0.0), ey = num("end", 2, 0.0);
            if (tag == "arc") {
                // connectivity only cares where an arc can be TOUCHED: its
                // endpoints and midpoint — two chords approximate that.
                double mx2 = num("mid", 1, (sx + ex) * 0.5);
                double my2 = num("mid", 2, (sy + ey) * 0.5);
                cu_segs[n].push_back({sx, sy, mx2, my2, w, lay});
                cu_segs[n].push_back({mx2, my2, ex, ey, w, lay});
            } else {
                cu_segs[n].push_back({sx, sy, ex, ey, w, lay});
            }
        }

        // gather module/footprint lists
        std::vector<const SExpr*> bods = find_children(root, "module");
        {
            std::vector<const SExpr*> fps = find_children(root, "footprint");
            bods.insert(bods.end(), fps.begin(), fps.end());
        }

        // ---- pre-routed = fully-connected: per-net union-find over
        // pads + copper with STRICT touch criteria (a doubtful touch counts
        // as NOT connected — the safe direction is re-routing the net).
        std::set<int> prerouted_nets;
        int partial_net_count = 0;
        {
            struct CuPad { double x, y, half_w, half_h, rot; bool oval;
                           uint32_t lmask; };
            std::map<int, std::vector<CuPad>> cu_pads;
            for (const SExpr* mod : bods) {
                double mx = 0.0, my = 0.0, mrot = 0.0;
                if (const SExpr* at = find_child(*mod, "at")) {
                    try {
                        if (at->kids.size() >= 2) mx = std::stod(at->kids[1].atom);
                        if (at->kids.size() >= 3) my = std::stod(at->kids[2].atom);
                        if (at->kids.size() >= 4) mrot = std::stod(at->kids[3].atom);
                    } catch (...) { continue; }
                }
                for (const SExpr* pad : find_children(*mod, "pad")) {
                    int nid = net_key(*pad);
                    if (nid == 0 || (!cu_segs.count(nid) && !cu_vias.count(nid)))
                        continue;   // only nets owning copper need the check
                    const SExpr* at = find_child(*pad, "at");
                    if (!at || at->kids.size() < 3) continue;
                    double px, py;
                    try {
                        px = std::stod(at->kids[1].atom);
                        py = std::stod(at->kids[2].atom);
                    } catch (...) { continue; }
                    rotate_deg(px, py, mrot);
                    const bool thru = pad_is_thru(*pad);
                    PadGeom pg = pad_geom_mm(*pad, thru);
                    double odx = pg.off_x, ody = pg.off_y;
                    if (odx != 0.0 || ody != 0.0) rotate_deg(odx, ody, -pg.rot);
                    uint32_t lm = 0;
                    if (thru) lm = ~0u;
                    else for (int q : pad_copper_layers(*pad, layer_index, n_layers))
                        if (q >= 0 && q < 32) lm |= (1u << q);
                    if (lm == 0) continue;
                    cu_pads[nid].push_back({mx + px + odx, my + py + ody,
                                            pg.half_w, pg.half_h, pg.rot,
                                            pg.oval, lm});
                }
            }

            // distance from a point to the pad's copper OUTLINE (0 inside);
            // rect in the pad frame, oval treated as its inscribed stadium
            auto pad_dist = [](const CuPad& pd, double qx, double qy) {
                double lx = qx - pd.x, ly = qy - pd.y;
                rotate_deg(lx, ly, -pd.rot);
                if (pd.oval) {
                    double r = std::min(pd.half_w, pd.half_h);
                    double ax = std::max(0.0, std::abs(lx) - (pd.half_w - r));
                    double ay = std::max(0.0, std::abs(ly) - (pd.half_h - r));
                    return std::max(0.0, std::sqrt(ax * ax + ay * ay) - r);
                }
                double dx = std::max(0.0, std::abs(lx) - pd.half_w);
                double dy = std::max(0.0, std::abs(ly) - pd.half_h);
                return std::sqrt(dx * dx + dy * dy);
            };
            auto p2s = [](double px2, double py2, const CuSeg& s) {
                double dx = s.x1 - s.x0, dy = s.y1 - s.y0;
                double L2 = dx * dx + dy * dy;
                double t = L2 > 0.0
                    ? std::max(0.0, std::min(1.0, ((px2 - s.x0) * dx +
                                                   (py2 - s.y0) * dy) / L2))
                    : 0.0;
                double cx = s.x0 + t * dx - px2, cy = s.y0 + t * dy - py2;
                return std::sqrt(cx * cx + cy * cy);
            };
            auto crosses = [](const CuSeg& a, const CuSeg& b) {
                auto orient = [](double ax, double ay, double bx, double by,
                                 double cx, double cy) {
                    double v = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
                    return v > 1e-12 ? 1 : (v < -1e-12 ? -1 : 0);
                };
                int o1 = orient(a.x0, a.y0, a.x1, a.y1, b.x0, b.y0);
                int o2 = orient(a.x0, a.y0, a.x1, a.y1, b.x1, b.y1);
                int o3 = orient(b.x0, b.y0, b.x1, b.y1, a.x0, a.y0);
                int o4 = orient(b.x0, b.y0, b.x1, b.y1, a.x1, a.y1);
                return o1 != o2 && o3 != o4 && o1 && o2 && o3 && o4;
            };
            const double STRICT = 0.9;   // of the electrical-touch threshold

            std::set<int> cu_nets;
            for (auto& kv : cu_segs) cu_nets.insert(kv.first);
            for (auto& kv : cu_vias) cu_nets.insert(kv.first);
            for (int nid : cu_nets) {
                if (zone_glue_nets.count(nid)) {   // pour completes the net
                    prerouted_nets.insert(nid);
                    continue;
                }
                auto& pads = cu_pads[nid];
                auto& segs = cu_segs[nid];
                auto& vias = cu_vias[nid];
                if (pads.size() < 2) {   // nothing left to connect
                    prerouted_nets.insert(nid);
                    continue;
                }
                size_t np = pads.size(), ns = segs.size(), nv = vias.size();
                std::vector<int> uf(np + ns + nv);
                for (size_t i = 0; i < uf.size(); ++i) uf[i] = (int)i;
                std::function<int(int)> find = [&](int a) {
                    while (uf[(size_t)a] != a) {
                        uf[(size_t)a] = uf[(size_t)uf[(size_t)a]];
                        a = uf[(size_t)a];
                    }
                    return a;
                };
                auto join = [&](size_t a, size_t b) {
                    uf[(size_t)find((int)a)] = find((int)b);
                };
                auto smask = [](int lay) {
                    return (lay >= 0 && lay < 32) ? (1u << lay) : 0u;
                };
                for (size_t i = 0; i < ns; ++i)
                    for (size_t j = i + 1; j < ns; ++j) {
                        if ((smask(segs[i].layer) & smask(segs[j].layer)) == 0)
                            continue;
                        double tol = STRICT * 0.5 * (segs[i].w + segs[j].w);
                        if (crosses(segs[i], segs[j]) ||
                            p2s(segs[i].x0, segs[i].y0, segs[j]) <= tol ||
                            p2s(segs[i].x1, segs[i].y1, segs[j]) <= tol ||
                            p2s(segs[j].x0, segs[j].y0, segs[i]) <= tol ||
                            p2s(segs[j].x1, segs[j].y1, segs[i]) <= tol)
                            join(np + i, np + j);
                    }
                for (size_t v = 0; v < nv; ++v) {
                    for (size_t i = 0; i < ns; ++i)
                        if (p2s(vias[v].x, vias[v].y, segs[i]) <=
                            STRICT * 0.5 * (vias[v].sz + segs[i].w))
                            join(np + ns + v, np + i);
                    for (size_t u = v + 1; u < nv; ++u) {
                        double dx = vias[v].x - vias[u].x,
                               dy = vias[v].y - vias[u].y;
                        if (std::sqrt(dx * dx + dy * dy) <=
                            STRICT * 0.5 * (vias[v].sz + vias[u].sz))
                            join(np + ns + v, np + ns + u);
                    }
                }
                for (size_t p = 0; p < np; ++p) {
                    for (size_t i = 0; i < ns; ++i) {
                        if ((pads[p].lmask & smask(segs[i].layer)) == 0)
                            continue;
                        // a track joins a pad when either endpoint lands on
                        // (or within a strict half-width of) the pad copper,
                        // or its span passes through the pad center region
                        double tol = STRICT * 0.5 * segs[i].w;
                        if (pad_dist(pads[p], segs[i].x0, segs[i].y0) <= tol ||
                            pad_dist(pads[p], segs[i].x1, segs[i].y1) <= tol ||
                            p2s(pads[p].x, pads[p].y, segs[i]) <=
                                STRICT * (std::min(pads[p].half_w,
                                                   pads[p].half_h) +
                                          0.5 * segs[i].w))
                            join(p, np + i);
                    }
                    for (size_t v = 0; v < nv; ++v)
                        if (pad_dist(pads[p], vias[v].x, vias[v].y) <=
                            STRICT * 0.5 * vias[v].sz)
                            join(p, np + ns + v);
                }
                bool one = true;
                int r0 = find(0);
                for (size_t p = 1; p < np; ++p)
                    if (find((int)p) != r0) { one = false; break; }
                if (one) prerouted_nets.insert(nid);
                else ++partial_net_count;
            }
        }

        // skip_poured (KiCad-plugin default): a net owning a pour is left to
        // its zone even when it has no tracks yet — routing tracks all over a
        // poured net is the classic autorouter surprise. Newly-skipped keys
        // are reported by name so the user is told exactly what was left out.
        std::vector<int> pour_skipped_keys;
        if (skip_poured)
            for (int zn : zone_glue_nets)
                if (prerouted_nets.insert(zn).second)
                    pour_skipped_keys.push_back(zn);

        for (const SExpr* mod : bods) {
            // module absolute (at mx my [rot])
            double mx = 0.0, my = 0.0, mrot = 0.0;
            if (const SExpr* at = find_child(*mod, "at")) {
                try {
                    if (at->kids.size() >= 2) { mx = std::stod(at->kids[1].atom); }
                    if (at->kids.size() >= 3) { my = std::stod(at->kids[2].atom); }
                    if (at->kids.size() >= 4) { mrot = std::stod(at->kids[3].atom); }
                } catch (...) {
                    throw std::runtime_error("KiCad PCB: non-numeric module 'at'");
                }
            }
            for (const SExpr* pad : find_children(*mod, "pad")) {
                int nid = net_key(*pad);
                const SExpr* at = find_child(*pad, "at");
                if (!at || at->kids.size() < 3) continue;
                double px, py;
                try {
                    px = std::stod(at->kids[1].atom);
                    py = std::stod(at->kids[2].atom);
                } catch (...) {
                    throw std::runtime_error("KiCad PCB: non-numeric pad 'at'");
                }
                rotate_deg(px, py, mrot);
                auto pls = pad_copper_layers(*pad, layer_index, n_layers);
                // Drilled barrel from the pad's TYPE token, not its layer count (a
                // thru-hole pad on a 1-layer board still has a barrel; stacked SMD
                // pads on several layers do not).
                const bool thru = pad_is_thru(*pad);
                PadGeom pg = pad_geom_mm(*pad, thru);
                // Drill dims (mm): (drill D) or (drill oval W H); 0 = none.
                double drill_w = 0.0, drill_h = 0.0;
                if (const SExpr* dr0 = find_child(*pad, "drill")) {
                    std::vector<double> dv0;
                    for (size_t k2 = 1; k2 < dr0->kids.size() && dv0.size() < 2; ++k2) {
                        if (dr0->kids[k2].is_list) continue;
                        try { dv0.push_back(std::stod(dr0->kids[k2].atom)); }
                        catch (...) {}
                    }
                    if (!dv0.empty() && dv0[0] > 0.0) {
                        drill_w = dv0[0];
                        drill_h = dv0.size() > 1 ? dv0[1] : dv0[0];
                    }
                }
                // Local rule overrides grow the modeled copper so the GLOBAL
                // clearance still satisfies them:
                //  - (clearance C) on the pad/footprint replaces netclass
                //    clearance for that pad (1.1mm guard rings on stompboxes);
                //  - (solder_mask_margin M) / setup pad_to_mask_clearance opens
                //    a mask aperture beyond the copper — foreign tracks inside
                //    it are exposed -> solder_mask_bridge (fiducials, keyboard
                //    switch pads). Foreign copper must clear the APERTURE.
                {
                    auto local_num = [&](const SExpr& e, const char* key) {
                        if (const SExpr* c = find_child(e, key)) {
                            if (c->kids.size() >= 2 && !c->kids[1].is_list) {
                                try { return std::stod(c->kids[1].atom); }
                                catch (...) {}
                            }
                        }
                        return 0.0;
                    };
                    const double eff_clr2 = enforced_clr_mm;
                    double c_over = std::max(local_num(*pad, "clearance"),
                                             local_num(*mod, "clearance"));
                    double m_over = std::max({local_num(*pad, "solder_mask_margin"),
                                              local_num(*mod, "solder_mask_margin"),
                                              setup_mask_margin});
                    const double grow = std::max({0.0, c_over - eff_clr2,
                                                  m_over - eff_clr2});
                    if (grow > 0.0) {
                        pg.half_w += grow;
                        pg.half_h += grow;
                    }
                }
                // Mask aperture on an outer side WITHOUT copper there (back-side
                // test points with a front window, etc.): an open-mask window —
                // any copper routed under it is exposed and bridges to the pad
                // (solder_mask_bridge). Block that outer layer under the pad.
                {
                    bool fmask = false, bmask = false;
                    if (const SExpr* lay = find_child(*pad, "layers"))
                        for (const SExpr& l : lay->kids) {
                            if (l.is_list) continue;
                            if (l.atom == "F.Mask" || l.atom == "*.Mask") fmask = true;
                            if (l.atom == "B.Mask" || l.atom == "*.Mask") bmask = true;
                        }
                    auto on_layer = [&](int l) {
                        for (int q : pls) if (q == l) return true;
                        return false;
                    };
                    if ((fmask && !on_layer(0)) ||
                        (bmask && n_layers > 1 && !on_layer(n_layers - 1))) {
                        double odx = pg.off_x, ody = pg.off_y;
                        if (odx != 0.0 || ody != 0.0) rotate_deg(odx, ody, -pg.rot);
                        if (fmask && !on_layer(0))
                            obstacles.push_back({mx + px + odx, my + py + ody, 0,
                                                 pg.half_w, pg.half_h, pg.rot, pg.oval});
                        if (bmask && n_layers > 1 && !on_layer(n_layers - 1))
                            obstacles.push_back({mx + px + odx, my + py + ody,
                                                 n_layers - 1, pg.half_w, pg.half_h,
                                                 pg.rot, pg.oval});
                    }
                }
                if (const SExpr* dr = find_child(*pad, "drill")) {
                    // (drill D) / (drill oval W H): hole half extents in the pad frame.
                    // Foreign copper keeps `clearance` from the pad COPPER edge, but
                    // KiCad's hole_clearance rule (0.25mm default) measures from the
                    // HOLE edge — a small annular ring under-protects the hole. Floor
                    // the model extents so hole + deficit is always covered.
                    std::vector<double> dv;
                    for (size_t k = 1; k < dr->kids.size() && dv.size() < 2; ++k) {
                        if (dr->kids[k].is_list) continue;
                        try { dv.push_back(std::stod(dr->kids[k].atom)); }
                        catch (...) {}   // skips the 'oval' token
                    }
                    if (!dv.empty() && dv[0] > 0.0) {
                        const double deficit = std::max(0.0, 0.25 - enforced_clr_mm);
                        // hole center sits at -offset in the shape frame
                        const double hw2 = std::abs(pg.off_x) + dv[0] * 0.5 + deficit;
                        const double hh2 = std::abs(pg.off_y)
                                           + (dv.size() > 1 ? dv[1] : dv[0]) * 0.5 + deficit;
                        pg.half_w = std::max(pg.half_w, hw2);
                        pg.half_h = std::max(pg.half_h, hh2);
                    }
                }
                if (nid == 0 || prerouted_nets.count(nid)) {
                    // Unconnected pad, or a pad of a PRE-ROUTED net: no pin to route,
                    // but its copper/hole is a hard obstacle — a trace through it is
                    // a short KiCad flags (shorting_items / clearance / hole_clearance).
                    // A drilled barrel blocks EVERY layer regardless of listed copper;
                    // paste/mask-only apertures without a hole block nothing.
                    if (thru) {
                        pls.clear();
                        for (int i = 0; i < n_layers; ++i) pls.push_back(i);
                    } else if (!pad_on_copper(*pad)) {
                        continue;
                    }
                    double odx = pg.off_x, ody = pg.off_y;
                    if (odx != 0.0 || ody != 0.0) rotate_deg(odx, ody, -pg.rot);
                    for (int layer : pls)
                        obstacles.push_back({mx + px + odx, my + py + ody, layer,
                                             pg.half_w, pg.half_h, pg.rot, pg.oval});
                    continue;
                }
                const double pr = std::max(pg.half_w, pg.half_h) / resolution;
                // world-frame copper offset: rotate the local offset by the pad's
                // file rotation (g.rot stores the NEGATED angle; undo for rotate_deg)
                double sdx = pg.off_x, sdy = pg.off_y;
                if (sdx != 0.0 || sdy != 0.0) rotate_deg(sdx, sdy, -pg.rot);
                for (int layer : pls)
                    all_pins.push_back({mx + px, my + py, nid, layer, pr, thru,
                                        pg.half_w / resolution, pg.half_h / resolution,
                                        pg.rot, pg.oval, sdx, sdy});
                // Single-sided THT: a drilled pad listing copper on SOME layers
                // still pierces EVERY layer — on copper-less layers the HOLE
                // (dilated to the 0.25mm hole-clearance rule) blocks foreign
                // tracks (measured: F.Cu tracks straight through B.Cu-only pad
                // holes, actual 0.0). Same-net pins, so the plated barrel stays
                // legal for its own net; the (drill offset) moves copper, never
                // the hole, so hole pins sit at the pad position.
                if (thru && drill_w > 0.0 && (int)pls.size() < n_layers) {
                    const double hdef = std::max(0.0, 0.25 - enforced_clr_mm);
                    const double hw3 = (drill_w * 0.5 + hdef) / resolution;
                    const double hh3 = (drill_h * 0.5 + hdef) / resolution;
                    std::set<int> have(pls.begin(), pls.end());
                    for (int l = 0; l < n_layers; ++l) {
                        if (have.count(l)) continue;
                        all_pins.push_back({mx + px, my + py, nid, l,
                                            std::max(hw3, hh3), thru, hw3, hh3,
                                            pg.rot, true, 0.0, 0.0});
                    }
                }
                all_centres.emplace_back(mx + px, my + py); // once per PHYSICAL pad
                uint64_t lmask = 0;
                for (int layer : pls) lmask |= 1ull << std::min(layer, 63);
                centre_layer_masks.push_back(lmask);
            }
        }

        if (all_pins.empty()) {
            // distinguishable: callers treat a fully-routed board as a no-op
            if (!prerouted_nets.empty())
                throw std::runtime_error("KiCad PCB: all nets already routed");
            throw std::runtime_error("KiCad PCB: no nets/pads to route (all unconnected or none)");
        }

        // ---- bounds: union of (general (area ...)) and pad extents ----
        double min_x =  std::numeric_limits<double>::infinity();
        double min_y =  std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        for (const AbsPin& p : all_pins) {
            min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
        }
        if (const SExpr* gen = find_child(root, "general")) {
            if (const SExpr* area = find_child(*gen, "area")) {
                try {
                    if (area->kids.size() >= 5) {
                        double ax1 = std::stod(area->kids[1].atom);
                        double ay1 = std::stod(area->kids[2].atom);
                        double ax2 = std::stod(area->kids[3].atom);
                        double ay2 = std::stod(area->kids[4].atom);
                        min_x = std::min(min_x, std::min(ax1, ax2));
                        max_x = std::max(max_x, std::max(ax1, ax2));
                        min_y = std::min(min_y, std::min(ay1, ay2));
                        max_y = std::max(max_y, std::max(ay1, ay2));
                    }
                } catch (...) { /* area optional / malformed; fall back to pad extents */ }
            }
        }
        // ---- graphic primitives -> obstacles ----
        // Two families, one walk:
        //  - Edge.Cuts / Margin primitives: the copper-to-edge rule (0.5mm default)
        //    is wider than copper clearance, so each becomes a capsule on EVERY
        //    copper layer inflated by the difference; the ordinary pad-keepout
        //    stamp (margin = clearance + trace half-width) then enforces it.
        //  - Copper-layer graphics (root gr_* or footprint fp_* — card-edge
        //    fingers, antennas, logos on copper): real copper belonging to no
        //    routable net; a capsule at the stroke half-width blocks routing
        //    through them (KiCad flags such contact as shorting_items).
        {
            const double eff_clr = enforced_clr_mm;
            const double edge_extra = std::max(0.0, kKiCadEdgeClearanceMM - eff_clr);
            // rad: capsule radius (mm). layer < 0 = every copper layer.
            auto add_capsule = [&](double x1, double y1, double x2, double y2,
                                   double rad, int layer) {
                const double dx = x2 - x1, dy = y2 - y1;
                const double len = std::hypot(dx, dy);
                const double rot = std::atan2(dy, dx) * 180.0 / M_PI;
                const double hw2 = len * 0.5 + rad;
                if (layer >= 0) {
                    obstacles.push_back({(x1 + x2) * 0.5, (y1 + y2) * 0.5, layer,
                                         hw2, rad, rot, true});
                    return;
                }
                for (int l = 0; l < n_layers; ++l)
                    obstacles.push_back({(x1 + x2) * 0.5, (y1 + y2) * 0.5, l,
                                         hw2, rad, rot, true});
            };
            auto xy_of = [](const SExpr* e, double& x, double& y) -> bool {
                if (!e || e->kids.size() < 3 || e->kids[1].is_list || e->kids[2].is_list)
                    return false;
                try { x = std::stod(e->kids[1].atom); y = std::stod(e->kids[2].atom); }
                catch (...) { return false; }
                return true;
            };
            // Stroke width: v4/5 (width w), v6+ (stroke (width w)).
            auto stroke_w = [&](const SExpr& ent) -> double {
                if (const SExpr* w = find_child(ent, "width")) {
                    if (w->kids.size() >= 2 && !w->kids[1].is_list) {
                        try { return std::stod(w->kids[1].atom); } catch (...) {}
                    }
                }
                if (const SExpr* st = find_child(ent, "stroke")) {
                    if (const SExpr* w = find_child(*st, "width")) {
                        if (w->kids.size() >= 2 && !w->kids[1].is_list) {
                            try { return std::stod(w->kids[1].atom); } catch (...) {}
                        }
                    }
                }
                return 0.0;
            };
            // Scanline-fill a closed polygon into row capsules on the given layers.
            auto fill_poly = [&](const std::vector<std::pair<double, double>>& v,
                                 const std::vector<int>& fl, bool via_only) {
                if (v.size() < 3) return;
                double ymin = v[0].second, ymax = v[0].second;
                for (const auto& q : v) {
                    ymin = std::min(ymin, q.second);
                    ymax = std::max(ymax, q.second);
                }
                // Row radius = full step: rows are step apart, so any interior
                // point — including polygon TIPS past the last scanline (up to a
                // whole step away) — stays within rrad of a row. Keepouts may
                // overreach outward by up to one step; the safe direction.
                const double step = resolution * 2.0, rrad = step;
                for (double y = ymin; y <= ymax + 1e-12; y += step) {
                    std::vector<double> xs;
                    for (size_t k = 0; k < v.size(); ++k) {
                        const auto& a = v[k];
                        const auto& b2 = v[(k + 1) % v.size()];
                        if ((a.second <= y) == (b2.second <= y)) continue;
                        xs.push_back(a.first + (y - a.second) * (b2.first - a.first) /
                                                   (b2.second - a.second));
                    }
                    std::sort(xs.begin(), xs.end());
                    for (size_t k = 0; k + 1 < xs.size(); k += 2)
                        for (int l : fl)
                            obstacles.push_back({(xs[k] + xs[k+1]) * 0.5, y, l,
                                                 (xs[k+1] - xs[k]) * 0.5 + rrad, rrad,
                                                 0.0, true, via_only});
                }
            };
            // Emit one primitive. `xf` maps its local coords to world (identity for
            // root graphics; module translate+rotate for fp_*). tag_base: "line",
            // "arc", "rect", "circle", "poly".
            auto emit_shape = [&](const SExpr& ent, const std::string& tag_base,
                                  auto&& xf) {
                int layer = -2;          // -2 skip, -1 all-copper edge band
                double rad = 0.0;
                bool fill = false;       // mask windows block their INTERIOR
                bool back_layer = false; // text on B.* copper is MIRRORED
                {
                    const SExpr* lay = find_child(ent, "layer");
                    if (!lay || lay->kids.size() < 2 || lay->kids[1].is_list) return;
                    const std::string& lname = lay->kids[1].atom;
                    if (lname == "Edge.Cuts" || lname == "Margin") {
                        layer = -1;
                        // KiCad measures edge clearance from the STROKE edge, not
                        // the centerline (seen on Margin outlines: 0.1 leak = the
                        // stroke half-width) — include it in the band.
                        rad = edge_extra + stroke_w(ent) * 0.5;
                    } else if (lname == "F.Mask" || lname == "B.Mask") {
                        // Graphic mask aperture: an open-mask window over the outer
                        // copper layer. Any routed copper inside is exposed beside
                        // foreign exposed copper -> solder_mask_bridge. Conservative
                        // model: no routing under graphic mask windows at all.
                        layer = (lname == "F.Mask") ? 0 : n_layers - 1;
                        rad = std::max(0.05, stroke_w(ent) * 0.5);
                        fill = true;
                    } else {
                        auto it = layer_index.find(lname);
                        if (it == layer_index.end()) return;   // silk/fab/etc
                        layer = it->second;
                        rad = std::max(0.05, stroke_w(ent) * 0.5);
                        back_layer = lname.size() >= 2 && lname[0] == 'B' &&
                                     lname[1] == '.';
                        // Solid copper interiors are real copper, not outlines:
                        // fp_poly/gr_poly on copper (filled by default in v5;
                        // explicit (fill solid|yes) in v6+), rect/circle when
                        // explicitly filled.
                        if (tag_base == "poly") fill = true;
                        else if (const SExpr* f = find_child(ent, "fill")) {
                            if (f->kids.size() >= 2 && !f->kids[1].is_list &&
                                (f->kids[1].atom == "solid" || f->kids[1].atom == "yes"))
                                fill = true;
                        }
                    }
                }
                auto seg = [&](double x1, double y1, double x2, double y2) {
                    xf(x1, y1); xf(x2, y2);
                    add_capsule(x1, y1, x2, y2, rad, layer == -1 ? -1 : layer);
                };
                double sx, sy, ex, ey, mx2, my2;
                if (tag_base == "line") {
                    if (xy_of(find_child(ent, "start"), sx, sy) &&
                        xy_of(find_child(ent, "end"), ex, ey))
                        seg(sx, sy, ex, ey);
                } else if (tag_base == "rect") {
                    if (xy_of(find_child(ent, "start"), sx, sy) &&
                        xy_of(find_child(ent, "end"), ex, ey)) {
                        if (fill) {
                            double ax = sx, ay = sy, bx2 = ex, by2 = ey;
                            xf(ax, ay); xf(bx2, by2);
                            fill_poly({{ax, ay}, {bx2, ay}, {bx2, by2}, {ax, by2}},
                                      {layer}, false);
                        }
                        seg(sx, sy, ex, sy); seg(ex, sy, ex, ey);
                        seg(ex, ey, sx, ey); seg(sx, ey, sx, sy);
                    }
                } else if (tag_base == "circle") {
                    double ccx, ccy;
                    if (xy_of(find_child(ent, "center"), ccx, ccy) &&
                        xy_of(find_child(ent, "end"), ex, ey)) {
                        const double r = std::hypot(ex - ccx, ey - ccy);
                        // Sagitta-bounded chords (see the arc branch).
                        const double sag = 0.5 * resolution;
                        const double th = (sag < r)
                            ? 2.0 * std::acos(std::max(0.0, 1.0 - sag / r))
                            : (M_PI / 12);
                        const int N = std::max(12, (int)std::ceil(2.0 * M_PI /
                                                                  std::max(1e-3, th)));
                        rad += sag;
                        if (fill) {
                            std::vector<std::pair<double, double>> ring;
                            for (int k = 0; k < N; ++k) {
                                const double a = 2.0 * M_PI * k / N;
                                double qx = ccx + r * std::cos(a), qy = ccy + r * std::sin(a);
                                xf(qx, qy);
                                ring.emplace_back(qx, qy);
                            }
                            fill_poly(ring, {layer}, false);
                        }
                        double px = ccx + r, py = ccy;
                        for (int k = 1; k <= N; ++k) {
                            const double a = 2.0 * M_PI * k / N;
                            seg(px, py, ccx + r * std::cos(a), ccy + r * std::sin(a));
                            px = ccx + r * std::cos(a); py = ccy + r * std::sin(a);
                        }
                        rad -= sag;
                    }
                } else if (tag_base == "arc") {
                    // v6+: (start)(mid)(end) ON the arc; v4/5: (start)=center,
                    // (end)=arc start point, (angle)=sweep. Chord-sampled.
                    const SExpr* mid = find_child(ent, "mid");
                    double ccx, ccy, r = 0.0, a0 = 0.0, a1 = 0.0;
                    bool ok = false;
                    if (mid && xy_of(find_child(ent, "start"), sx, sy) &&
                        xy_of(mid, mx2, my2) && xy_of(find_child(ent, "end"), ex, ey)) {
                        const double d = 2.0 * (sx * (my2 - ey) + mx2 * (ey - sy) +
                                                ex * (sy - my2));
                        if (std::abs(d) > 1e-9) {
                            const double s2 = sx*sx + sy*sy, m2 = mx2*mx2 + my2*my2,
                                         e2 = ex*ex + ey*ey;
                            ccx = (s2*(my2-ey) + m2*(ey-sy) + e2*(sy-my2)) / d;
                            ccy = (s2*(ex-mx2) + m2*(sx-ex) + e2*(mx2-sx)) / d;
                            r = std::hypot(sx - ccx, sy - ccy);
                            a0 = std::atan2(sy - ccy, sx - ccx);
                            const double am = std::atan2(my2 - ccy, mx2 - ccx);
                            a1 = std::atan2(ey - ccy, ex - ccx);
                            auto norm = [](double a) {
                                while (a < 0) a += 2.0 * M_PI;
                                while (a >= 2.0 * M_PI) a -= 2.0 * M_PI;
                                return a;
                            };
                            const double sw_ccw = norm(a1 - a0);
                            a1 = (norm(am - a0) <= sw_ccw) ? a0 + sw_ccw
                                                           : a0 - (2.0 * M_PI - sw_ccw);
                            ok = true;
                        }
                    } else if (xy_of(find_child(ent, "start"), ccx, ccy) &&
                               xy_of(find_child(ent, "end"), sx, sy)) {
                        double ang = 90.0;
                        if (const SExpr* an = find_child(ent, "angle")) {
                            if (an->kids.size() >= 2 && !an->kids[1].is_list) {
                                try { ang = std::stod(an->kids[1].atom); } catch (...) {}
                            }
                        }
                        r = std::hypot(sx - ccx, sy - ccy);
                        a0 = std::atan2(sy - ccy, sx - ccx);
                        a1 = a0 + ang * M_PI / 180.0;
                        ok = true;
                    }
                    if (ok && r > 1e-9) {
                        // Chord count bounds the SAGITTA to half a cell: a fixed
                        // 15-degree step on a 60mm-radius board outline cuts 0.5mm
                        // inside the true curve (58 edge violations, acorn). The
                        // residual sagitta is folded into the band radius.
                        const double sag = 0.5 * resolution;
                        const double th = (sag < r)
                            ? 2.0 * std::acos(std::max(0.0, 1.0 - sag / r))
                            : (M_PI / 12);
                        const int N = std::max(2, (int)std::ceil(std::abs(a1 - a0) /
                                                                 std::max(1e-3, th)));
                        rad += sag;
                        double px = ccx + r * std::cos(a0), py = ccy + r * std::sin(a0);
                        for (int k = 1; k <= N; ++k) {
                            const double a = a0 + (a1 - a0) * k / N;
                            seg(px, py, ccx + r * std::cos(a), ccy + r * std::sin(a));
                            px = ccx + r * std::cos(a); py = ccy + r * std::sin(a);
                        }
                        rad -= sag;
                    }
                } else if (tag_base == "poly") {
                    if (const SExpr* pts = find_child(ent, "pts")) {
                        std::vector<std::pair<double, double>> v;
                        for (const SExpr& pt : pts->kids) {
                            if (!pt.is_list || pt.kids.size() < 3 || pt.kids[0].is_list ||
                                pt.kids[0].atom != "xy")
                                continue;
                            try {
                                v.emplace_back(std::stod(pt.kids[1].atom),
                                               std::stod(pt.kids[2].atom));
                            } catch (...) {}
                        }
                        for (size_t k = 0; k + 1 < v.size(); ++k)
                            seg(v[k].first, v[k].second, v[k+1].first, v[k+1].second);
                        if (v.size() > 2) {
                            seg(v.back().first, v.back().second, v[0].first, v[0].second);
                            if (fill) {
                                std::vector<std::pair<double, double>> w = v;
                                for (auto& q : w) xf(q.first, q.second);
                                fill_poly(w, {layer}, false);
                            }
                        }
                    }
                } else if (tag_base == "text") {
                    // Copper text: REAL stroke geometry from KiCad's newstroke
                    // font (ASCII; non-ASCII falls back to a full-advance box).
                    // gr_text: kids[1] = string; fp_text: kids[1] = kind, [2] =
                    // string. Hidden text has no copper.
                    if (layer < 0) return;   // text on Edge.Cuts/Margin: no band
                    // (layer "F.Cu" knockout): INVERTED text — the copper is a
                    // filled plate with the glyphs cut out, far larger than the
                    // strokes. Blocked as its padded box below.
                    bool knockout = false;
                    if (const SExpr* lay2 = find_child(ent, "layer"))
                        for (const SExpr& lk : lay2->kids)
                            if (!lk.is_list && lk.atom == "knockout")
                                knockout = true;
                    std::string txt;
                    if (ent.kids.size() >= 3 && !ent.kids[1].is_list && !ent.kids[2].is_list &&
                        (ent.kids[1].atom == "reference" || ent.kids[1].atom == "value" ||
                         ent.kids[1].atom == "user"))
                        txt = ent.kids[2].atom;
                    else if (ent.kids.size() >= 2 && !ent.kids[1].is_list)
                        txt = ent.kids[1].atom;
                    if (txt.empty()) return;
                    for (const SExpr& k : ent.kids)
                        if (!k.is_list && k.atom == "hide") return;
                    if (const SExpr* h = find_child(ent, "hide"))
                        if (h->kids.size() >= 2 && !h->kids[1].is_list &&
                            h->kids[1].atom == "yes")
                            return;
                    double tx = 0.0, ty = 0.0, trot = 0.0;
                    if (const SExpr* at = find_child(ent, "at")) {
                        if (at->kids.size() < 3 || at->kids[1].is_list) return;
                        try {
                            tx = std::stod(at->kids[1].atom);
                            ty = std::stod(at->kids[2].atom);
                            if (at->kids.size() >= 4 && !at->kids[3].is_list)
                                trot = std::stod(at->kids[3].atom);
                        } catch (...) { return; }
                    } else return;
                    double fh = 1.0, fw = 1.0, thick = 0.15;
                    int hj = 0, vj = 0;      // -1 left/top, 0 center, +1 right/bottom
                    // KiCad serializes the mirror state EXPLICITLY as a justify
                    // token — back-copper text carries (justify ... mirror) in the
                    // file. Auto-flipping by layer double-mirrors (measured on a
                    // rotated left-justified B.Cu label).
                    (void)back_layer;
                    bool mirror = false;
                    if (const SExpr* eff = find_child(ent, "effects")) {
                        if (const SExpr* font = find_child(*eff, "font")) {
                            if (const SExpr* sz = find_child(*font, "size"))
                                if (sz->kids.size() >= 3 && !sz->kids[1].is_list &&
                                    !sz->kids[2].is_list) {
                                    try {
                                        fh = std::stod(sz->kids[1].atom);
                                        fw = std::stod(sz->kids[2].atom);
                                    } catch (...) {}
                                }
                            if (const SExpr* th = find_child(*font, "thickness"))
                                if (th->kids.size() >= 2 && !th->kids[1].is_list) {
                                    try { thick = std::stod(th->kids[1].atom); }
                                    catch (...) {}
                                }
                        }
                        if (const SExpr* just = find_child(*eff, "justify"))
                            for (const SExpr& jk : just->kids) {
                                if (jk.is_list) continue;
                                if (jk.atom == "left") hj = -1;
                                else if (jk.atom == "right") hj = 1;
                                else if (jk.atom == "top") vj = -1;
                                else if (jk.atom == "bottom") vj = 1;
                                else if (jk.atom == "mirror") mirror = !mirror;
                            }
                    }
                    // ---- stroke the text (newstroke units: /21, y offset -10) ----
                    const double S21 = 1.0 / 21.0;
                    std::vector<std::string> lines;
                    {
                        std::string cur;
                        for (size_t k2 = 0; k2 < txt.size(); ++k2) {
                            if (txt[k2] == '\\' && k2 + 1 < txt.size() &&
                                txt[k2 + 1] == 'n') {
                                lines.push_back(cur); cur.clear(); k2++;
                            } else cur.push_back(txt[k2]);
                        }
                        lines.push_back(cur);
                    }
                    auto advance_of = [&](unsigned char c) -> double {
                        if (c < 32 || c > 126) return 0.76;   // non-ASCII: safe box
                        const char* g = kNewstrokeAscii[c - 32];
                        return (double)(g[1] - g[0]) * S21;
                    };
                    const double interline = 1.62 * fh;
                    const double block_h = fh + interline * (double)(lines.size() - 1);
                    // vertical: default anchor at the block CENTER; top/bottom put
                    // the anchor on that edge of the block.
                    double y0 = -block_h * 0.5 + fh * 0.5;   // first line center
                    if (vj < 0) y0 += block_h * 0.5;         // top: block below anchor
                    else if (vj > 0) y0 -= block_h * 0.5;    // bottom: block above
                    auto emit_seg_world = [&](double ax, double ay, double bx,
                                              double by) {
                        // local text frame -> mirror -> rotate -> translate
                        if (mirror) { ax = -ax; bx = -bx; }
                        rotate_deg(ax, ay, trot);
                        rotate_deg(bx, by, trot);
                        double wx1 = tx + ax, wy1 = ty + ay;
                        double wx2 = tx + bx, wy2 = ty + by;
                        const double dx = wx2 - wx1, dy = wy2 - wy1;
                        const double len = std::hypot(dx, dy);
                        const double rot2 = std::atan2(dy, dx) * 180.0 / M_PI;
                        const double r_st = thick * 0.5 + 0.02;  // end-cap slack
                        obstacles.push_back({(wx1 + wx2) * 0.5, (wy1 + wy2) * 0.5,
                                             layer, len * 0.5 + r_st,
                                             r_st, rot2, true});
                    };
                    for (size_t li = 0; li < lines.size(); ++li) {
                        const std::string& ln = lines[li];
                        double lw = 0.0;
                        for (unsigned char c : ln) lw += advance_of(c) * fw;
                        double x0 = -lw * 0.5;               // center justify
                        if (hj < 0) x0 = 0.0;                // left: text after anchor
                        else if (hj > 0) x0 = -lw;           // right: text before
                        if (knockout) {
                            // plate box: line span padded ~0.6*fh per side (the
                            // knockout border), full plate height as the capsule
                            // radius (thick temporarily widened for the emit).
                            const double pad2 = 0.6 * fh;
                            const double ly2 = y0 + interline * (double)li - 0.64 * fh;
                            const double keep = thick;
                            thick = 2.2 * fh;    // plate half-height ~1.1*fh
                            emit_seg_world(x0 - pad2, ly2, x0 + lw + pad2, ly2);
                            thick = keep;
                            continue;
                        }
                        // Baseline calibration measured against kicad-cli SVG
                        // renders: v6+ needs -0.64*fh (uniform offset across
                        // center/bottom/rot90/multiline WWWW cases). v5-era
                        // files: plain-mirror text measured -0.265*fh, but a
                        // ROTATED mirror text on the same dialect measured
                        // -0.64 — the v5 convention is ambiguous per transform,
                        // so rotated/mirrored v5 text strokes BOTH candidates
                        // (cheap union; rare objects, safe direction).
                        // v5 verdict after three boards: the offset depends on
                        // transform combinations in ways one constant can't fit
                        // (plain-mirror fit -0.265, rot90-mirror fit -0.64, and
                        // PLAIN text on a third board matched neither band
                        // alone) — ALL v5 text gets the dual band.
                        const bool v5 = file_version < 20200000;
                        const double v_cal = v5 ? 0.265 : 0.64;
                        const bool dual = v5;
                        // v5 + rotation: the transform ORDER (rotate/mirror/
                        // justify composition) is ambiguous too — also stroke
                        // the 180-flipped reading direction. Four cheap passes
                        // beat another week of dialect archaeology.
                        const bool quad = v5 && trot != 0.0;
                        const double ly = y0 + interline * (double)li - v_cal * fh;
                        const double ly_alt = ly - 0.375 * fh;
                        double pen = x0;
                        for (unsigned char c : ln) {
                            const double adv = advance_of(c) * fw;
                            if (c < 32 || c > 126) {
                                // unknown glyph: block its full cell
                                emit_seg_world(pen, ly - fh * 0.5, pen + adv,
                                               ly - fh * 0.5);
                                emit_seg_world(pen, ly + fh * 0.5, pen + adv,
                                               ly + fh * 0.5);
                                pen += adv;
                                continue;
                            }
                            const char* g = kNewstrokeAscii[c - 32];
                            const int gl = (int)std::strlen(g);
                            const int startx = g[0] - 'R';
                            const int npass = quad ? 4 : (dual ? 2 : 1);
                            for (int pass = 0; pass < npass; ++pass) {
                                const double lyp = (pass & 1) == 0 ? ly : ly_alt;
                                const bool flip = pass >= 2;   // 180-flipped read
                                double px = 0.0, py = 0.0;
                                bool pen_down = false;
                                for (int t2 = 2; t2 + 1 < gl; t2 += 2) {
                                    if (g[t2] == ' ' && g[t2 + 1] == 'R') {
                                        pen_down = false;
                                        continue;
                                    }
                                    double gx =
                                        pen + ((double)(g[t2] - 'R' - startx)) * S21 * fw;
                                    // y NEGATED: newstroke raw y grows UP, board
                                    // y grows down (SVG-calibrated).
                                    double gy =
                                        lyp - ((double)(g[t2 + 1] - 'R' - 10)) * S21 * fh;
                                    if (flip) { gx = -gx; gy = -gy; }
                                    if (pen_down) emit_seg_world(px, py, gx, gy);
                                    else emit_seg_world(gx, gy, gx, gy);  // dot
                                    px = gx; py = gy;
                                    pen_down = true;
                                }
                            }
                            pen += adv;
                        }
                    }
                }
            };
            auto identity = [](double&, double&) {};
            // Dimensions (measurement annotations) placed on COPPER are real
            // copper: their nested gr_text strokes plus every line primitive
            // (feature1/feature2/crossbar/arrows, each a (pts (xy)(xy)) chain).
            auto emit_dimension = [&](const SExpr& dim) {
                const SExpr* lay = find_child(dim, "layer");
                if (!lay || lay->kids.size() < 2 || lay->kids[1].is_list) return;
                auto itl = layer_index.find(lay->kids[1].atom);
                if (itl == layer_index.end()) return;      // Dwgs.User etc: not copper
                const int dl = itl->second;
                // v4/5: (width W); v6+: (style (thickness T)).
                double dw = 0.1;
                if (const SExpr* w = find_child(dim, "width")) {
                    if (w->kids.size() >= 2 && !w->kids[1].is_list) {
                        try { dw = std::stod(w->kids[1].atom); } catch (...) {}
                    }
                }
                if (const SExpr* st = find_child(dim, "style")) {
                    if (const SExpr* th = find_child(*st, "thickness")) {
                        if (th->kids.size() >= 2 && !th->kids[1].is_list) {
                            try { dw = std::stod(th->kids[1].atom); } catch (...) {}
                        }
                    }
                }
                // arrows/extensions stick out ~arrow_length beyond the chord —
                // fold a generous pad into the stroke radius.
                const double dim_r = std::max(0.05, dw * 0.5) + 0.7;
                auto walk_pts = [&](const SExpr& pts) {
                    double lx = 0.0, lyv = 0.0;
                    bool have = false;
                    for (const SExpr& pt : pts.kids) {
                        if (!pt.is_list || pt.kids.size() < 3 || pt.kids[0].is_list ||
                            pt.kids[0].atom != "xy")
                            continue;
                        double x2, y2;
                        try {
                            x2 = std::stod(pt.kids[1].atom);
                            y2 = std::stod(pt.kids[2].atom);
                        } catch (...) { continue; }
                        if (have) add_capsule(lx, lyv, x2, y2, dim_r, dl);
                        lx = x2; lyv = y2; have = true;
                    }
                };
                for (const SExpr& k : dim.kids) {
                    if (!k.is_list || k.kids.empty() || k.kids[0].is_list) continue;
                    if (k.kids[0].atom == "gr_text") {
                        emit_shape(k, "text", identity);
                        continue;
                    }
                    if (k.kids[0].atom == "pts") {
                        // v6+: the dimension LINE is derived from these anchor
                        // points (offset by (height h) perpendicular; the chord
                        // + generous radius covers the common small heights).
                        walk_pts(k);
                        continue;
                    }
                    if (const SExpr* pts = find_child(k, "pts"))
                        walk_pts(*pts);   // v4/5 feature1/feature2/crossbar/arrows
                }
            };
            for (const SExpr& ent : root.kids) {
                if (!ent.is_list || ent.kids.empty() || ent.kids[0].is_list) continue;
                const std::string& tag = ent.kids[0].atom;
                if (tag == "dimension") { emit_dimension(ent); continue; }
                if (tag.rfind("gr_", 0) == 0)
                    emit_shape(ent, tag.substr(3), identity);
                // Existing routed copper (see pre-scan above): tracks/track-arcs have
                // the same (start/end/width/layer) grammar as graphics strokes.
                else if (tag == "segment" || tag == "arc")
                    emit_shape(ent, tag == "segment" ? "line" : "arc", identity);
                else if (tag == "via") {
                    // (via (at x y) (size s) (drill d) ...): barrel blocks every
                    // copper layer. Radius covers both the copper annulus and the
                    // hole + its stricter 0.25mm hole-clearance rule.
                    const SExpr* at = find_child(ent, "at");
                    if (!at || at->kids.size() < 3 || at->kids[1].is_list) continue;
                    double vx2, vy2, sz = 0.6, drl = 0.0;
                    try {
                        vx2 = std::stod(at->kids[1].atom);
                        vy2 = std::stod(at->kids[2].atom);
                    } catch (...) { continue; }
                    if (const SExpr* z = find_child(ent, "size")) {
                        if (z->kids.size() >= 2 && !z->kids[1].is_list) {
                            try { sz = std::stod(z->kids[1].atom); } catch (...) {}
                        }
                    }
                    if (const SExpr* dchild = find_child(ent, "drill")) {
                        if (dchild->kids.size() >= 2 && !dchild->kids[1].is_list) {
                            try { drl = std::stod(dchild->kids[1].atom); } catch (...) {}
                        }
                    }
                    const double deficit = std::max(0.0, 0.25 - eff_clr);
                    const double r = std::max(sz * 0.5, drl * 0.5 + deficit);
                    for (int l = 0; l < n_layers; ++l)
                        obstacles.push_back({vx2, vy2, l, r, r, 0.0, true});
                }
            }
            for (const SExpr* mod : bods) {
                double mx = 0.0, my = 0.0, mrot = 0.0;
                if (const SExpr* at = find_child(*mod, "at")) {
                    try {
                        if (at->kids.size() >= 2) mx = std::stod(at->kids[1].atom);
                        if (at->kids.size() >= 3) my = std::stod(at->kids[2].atom);
                        if (at->kids.size() >= 4) mrot = std::stod(at->kids[3].atom);
                    } catch (...) { continue; }
                }
                auto mod_xf = [&](double& x, double& y) {
                    rotate_deg(x, y, mrot);
                    x += mx; y += my;
                };
                for (const SExpr& ent : mod->kids) {
                    if (!ent.is_list || ent.kids.empty() || ent.kids[0].is_list) continue;
                    const std::string& tag = ent.kids[0].atom;
                    if (tag.rfind("fp_", 0) == 0)
                        emit_shape(ent, tag.substr(3), mod_xf);
                }
            }

            // ---- rule areas (keepout zones), root-level and footprint-embedded ----
            // (zone (keepout (tracks not_allowed)|(vias not_allowed) ...) (polygon
            // (pts ...))): tracks-banned interiors become copper keepout rows;
            // vias-banned interiors become via-only rows (blocks via emergence
            // through the via_ok mask, tracks unaffected). Interiors are filled by
            // scanline rows of thin capsules — stamp margins over-dilate slightly
            // outward, the safe direction.
            auto zone_layers = [&](const SExpr& z) -> std::vector<int> {
                bool all = false;
                std::set<int> named;
                auto handle = [&](const std::string& nm) {
                    if (nm == "*.Cu" || nm == "F&B.Cu") all = true;
                    else {
                        auto it = layer_index.find(nm);
                        if (it != layer_index.end()) named.insert(it->second);
                    }
                };
                if (const SExpr* l = find_child(z, "layer"))
                    if (l->kids.size() >= 2 && !l->kids[1].is_list) handle(l->kids[1].atom);
                if (const SExpr* ls = find_child(z, "layers"))
                    for (size_t k = 1; k < ls->kids.size(); ++k)
                        if (!ls->kids[k].is_list) handle(ls->kids[k].atom);
                std::vector<int> out;
                if (all) for (int i = 0; i < n_layers; ++i) out.push_back(i);
                else out.assign(named.begin(), named.end());
                return out;
            };
            auto emit_zone = [&](const SExpr& z, auto&& xf) {
                const SExpr* ko = find_child(z, "keepout");
                if (!ko) return;   // copper pours: Phase A step 2 (obstacle loader)
                auto banned = [&](const char* what) {
                    const SExpr* w = find_child(*ko, what);
                    return w && w->kids.size() >= 2 && !w->kids[1].is_list &&
                           w->kids[1].atom == "not_allowed";
                };
                const bool no_tracks = banned("tracks");
                const bool no_vias = banned("vias");
                if (!no_tracks && !no_vias) return;
                const std::vector<int> zl = zone_layers(z);
                if (zl.empty()) return;
                for (const SExpr* poly : find_children(z, "polygon")) {
                    const SExpr* pts = find_child(*poly, "pts");
                    if (!pts) continue;
                    std::vector<std::pair<double, double>> v;
                    for (const SExpr& pt : pts->kids) {
                        if (!pt.is_list || pt.kids.size() < 3 || pt.kids[0].is_list ||
                            pt.kids[0].atom != "xy")
                            continue;
                        try {
                            double x = std::stod(pt.kids[1].atom);
                            double y = std::stod(pt.kids[2].atom);
                            xf(x, y);
                            v.emplace_back(x, y);
                        } catch (...) {}
                    }
                    fill_poly(v, zl, /*via_only=*/!no_tracks);
                }
            };
            // Zone polygon pts are serialized in BOARD coordinates at every
            // nesting level — including rule areas embedded in footprints
            // (verified: footprint at (152.45,85.52), its zone pts within 2mm of
            // it in absolute frame; applying the module transform would fling
            // them off-board). Walk the whole tree, always identity.
            auto identity2 = [](double&, double&) {};
            auto walk_zones = [&](const SExpr& node, auto&& self) -> void {
                for (const SExpr& ent : node.kids) {
                    if (!ent.is_list || ent.kids.empty() || ent.kids[0].is_list)
                        continue;
                    if (ent.kids[0].atom == "zone")
                        emit_zone(ent, identity2);
                    else
                        self(ent, self);
                }
            };
            walk_zones(root, walk_zones);
        }

        // No-net obstacles count toward bounds too: a mounting hole at the board edge
        // must land on the grid to block anything.
        for (const AbsObstacle& ob : obstacles) {
            min_x = std::min(min_x, ob.x); max_x = std::max(max_x, ob.x);
            min_y = std::min(min_y, ob.y); max_y = std::max(max_y, ob.y);
        }
        if (!std::isfinite(min_x) || !std::isfinite(max_x) ||
            !std::isfinite(min_y) || !std::isfinite(max_y)) {
            throw std::runtime_error("KiCad PCB: could not determine board bounds");
        }
        // margin so the edge pins are not clamped (also lets a degenerate single-row/
        // single-column board pass, since the margin gives it nonzero extent)
        min_x -= resolution; min_y -= resolution;
        max_x += resolution; max_y += resolution;
        if (max_x <= min_x || max_y <= min_y) {
            throw std::runtime_error("KiCad PCB: could not determine board bounds");
        }

        if ((max_x - min_x) / resolution >= 32768.0 || (max_y - min_y) / resolution >= 32768.0)
            throw std::runtime_error("KiCad PCB: grid out of supported bounds");
        int width  = (int)std::ceil((max_x - min_x) / resolution) + 1;
        int height = (int)std::ceil((max_y - min_y) / resolution) + 1;
        if (width <= 0 || height <= 0)
            throw std::runtime_error("KiCad PCB: grid out of supported bounds");

        // ---- group pins by net (declared order) ----
        std::vector<int> declared_nets;   // nonzero net ids in (net ...) declaration order
        std::set<int> added;
        std::map<std::string, int> name_to_code;   // mangled-table fidelity guard
        std::map<int, std::string> decl_code_names; // code -> declared name (class lookup)
        for (const SExpr& ent : root.kids) {
            if (ent.is_list && ent.kids.size() >= 2 && !ent.kids[0].is_list &&
                ent.kids[0].atom == "net") {
                int nid = 0;
                try { nid = (int)std::lround(std::stod(ent.kids[1].atom)); }
                catch (...) { continue; }
                if (nid <= 0) continue;
                // Fidelity guard: KiCad writes each net code exactly once, and net
                // NAMES are unique. A file redeclaring a code, or two codes sharing
                // a name, has a mangled table — KiCad reconciles pad nets by name
                // on load and can disagree with file order (seen: a pad declared
                // (net 134 PG5) rendered as /PG13 -> the model faithfully routes
                // the FILE and shorts KiCad's reinterpretation). Refuse to guess.
                if (reject_collisions) {
                    if (!added.insert(nid).second)
                        throw std::runtime_error(
                            "KiCad PCB: net code " + std::to_string(nid) +
                            " declared more than once (mangled net table)");
                    if (ent.kids.size() >= 3 && !ent.kids[2].is_list &&
                        !ent.kids[2].atom.empty()) {
                        auto ins = name_to_code.emplace(ent.kids[2].atom, nid);
                        if (!ins.second && ins.first->second != nid)
                            throw std::runtime_error(
                                "KiCad PCB: net name '" + ent.kids[2].atom +
                                "' maps to multiple codes (mangled net table)");
                        decl_code_names[nid] = ent.kids[2].atom;
                    }
                    declared_nets.push_back(nid);
                } else if (added.insert(nid).second) {
                    if (ent.kids.size() >= 3 && !ent.kids[2].is_list)
                        decl_code_names[nid] = ent.kids[2].atom;
                    declared_nets.push_back(nid);
                }
            }
        }
        // Also ensure any net seen on a pad but not declared is included (safety).
        for (const AbsPin& p : all_pins)
            if (added.insert(p.src_net).second) declared_nets.push_back(p.src_net);

        std::map<int, std::vector<AbsPin>> net_pins; // net id -> its pins (with layer)
        for (const AbsPin& p : all_pins) net_pins[p.src_net].push_back(p);

        const double via_cost = 5.0, base_cost = 1.0;
        // Half-cell safety margin: the grid model enforces clearance at CELL
        // CENTERS; continuous emitted geometry (pad-center snap stubs, diagonal
        // cuts) can dip up to resolution/2 closer between centers — measured as
        // exactly-half-a-cell violations (0.225 vs 0.250, SegMan). Routing half
        // a cell wider makes the emitted copper continuously rule-clean.
        // Base (tier 0) physics = the DEFAULT class; wider classes become
        // per-net-pair tiers below. Via radius stays the class MAX: the via
        // machinery is not pair-aware, so max is the safe direction.
        const double base_clr_mm =
            (default_clr > 0.0 ? default_clr
                               : (rule_clearance > 0.0 ? rule_clearance
                                                       : kKiCadClearanceMM));
        // Diagonal-safe: a snap stub's endpoint can sit half a cell off in BOTH
        // axes, dipping res*sqrt(2)/2 (~0.035mm) below the cell-center guarantee
        // — measured as 2um violations. Half-diagonal covers the worst case.
        const double kSnapMargin = 0.7072 * resolution;
        const double model_clearance = base_clr_mm + kSnapMargin;
        Board b(n_layers, width, height, resolution, via_cost, base_cost, model_clearance);
        // Board's own rules drive physics (not user-explicit: env overrides still win).
        const double base_tw_mm = default_tw > 0.0 ? default_tw : rule_tw;
        b.set_rule_physics(base_tw_mm > 0.0 ? base_tw_mm * 0.5 / resolution : 0.0,
                           rule_via > 0.0 ? rule_via * 0.5 / resolution : 0.0);

        // original_net_ids in the same order as Board nets
        int bidx = 0;
        std::vector<int> orig_ids;
        // Stage-10 fidelity: cell key -> first centre; count DISTINCT centres collapsing
        // to the same cell. (all_centres/min spacing were collected per physical pad in
        // the module loop above.)
        struct CellClaim { double x, y; int src_net; };
        std::map<size_t, CellClaim> cell_to_centre;
        int collisions = 0;
        for (int nid : declared_nets) {
            auto it = net_pins.find(nid);
            if (it == net_pins.end()) continue;
            const std::vector<AbsPin>& pins = it->second;
            if (pins.empty()) continue;
            // de-dup identical pins within a net (same cell) and clamp to grid; several
            // pins collapsing to one cell keep the LARGEST radius (keepout/DRC must
            // cover the biggest pad, not whichever pin was parsed last)
            struct CellPad { double radius; bool thru;
                             double cx, cy, hw, hh, rot; bool oval; };
            std::map<Cell, CellPad> cells;
            for (const AbsPin& p : pins) {
                int x = (int)std::floor((p.x - min_x) / resolution);
                int y = (int)std::floor((p.y - min_y) / resolution);
                if (x < 0) { x = 0; }
                if (y < 0) { y = 0; }
                if (x >= width) { x = width - 1; }
                if (y >= height) { y = height - 1; }
                int l = (p.layer >= 0 && p.layer < n_layers) ? p.layer : 0;
                const size_t key = ((size_t)l * height + y) * width + x;
                auto inserted = cell_to_centre.emplace(key, CellClaim{p.x, p.y, nid});
                if (!inserted.second) {
                    const auto& prev = inserted.first->second;
                    double dd = (p.x - prev.x) * (p.x - prev.x) +
                                (p.y - prev.y) * (p.y - prev.y);
                    // Distinct centres collapsing (resolution too coarse) OR pads
                    // of DIFFERENT nets claiming one cell (duplicate overlapping
                    // modules — seen on 'stacked footprint' boards): the model can
                    // hold one owner per cell, so routing such a board silently
                    // shorts whichever net loses the overwrite. Fidelity-reject.
                    if (dd > 1e-12 || prev.src_net != nid) collisions++;
                }
                const double cxf = (p.x + p.shape_dx - min_x) / resolution;  // copper center
                const double cyf = (p.y + p.shape_dy - min_y) / resolution;  // (pin stays at at)
                auto ins = cells.emplace(Cell{l, x, y},
                                         CellPad{p.radius, p.thru, cxf, cyf,
                                                 p.half_w, p.half_h, p.rot, p.oval});
                if (!ins.second) {
                    // same-net pins collapsed to one cell: keep the LARGER footprint
                    if (p.radius > ins.first->second.radius) {
                        ins.first->second = CellPad{p.radius, ins.first->second.thru || p.thru,
                                                    cxf, cyf, p.half_w, p.half_h, p.rot, p.oval};
                    } else {
                        ins.first->second.thru = ins.first->second.thru || p.thru;
                    }
                }
            }
            if (cells.empty()) continue;
            // Structurally-connected net: every pin shares ONE (x,y) column with
            // a drilled barrel — the plating already connects all layers, no
            // copper is needed or emitted. Leaving it routable makes reloads of
            // our own output re-route it REDUNDANTLY (gate A3: +514 segments on
            // a second pass). Its pads become fixed obstacles instead.
            {
                bool same_col = true, any_thru = false;
                const double ccx0 = cells.begin()->second.cx;
                const double ccy0 = cells.begin()->second.cy;
                for (const auto& cv : cells) {
                    // TRUE centers must coincide (a stacked thru pad), not merely
                    // collapse to one cell at this resolution — distinct pads
                    // within a cell are a REAL net to route, and dropping them
                    // as "barrel-connected" orphaned whole nets (71 violations
                    // on a coarse-gridded THT board).
                    if (std::abs(cv.second.cx - ccx0) > 0.11 ||
                        std::abs(cv.second.cy - ccy0) > 0.11)
                        same_col = false;
                    if (cv.second.thru) any_thru = true;
                }
                if (same_col && any_thru && cells.size() > 1) {
                    for (const auto& cv : cells)
                        obstacles.push_back({min_x + cv.second.cx * resolution,
                                             min_y + cv.second.cy * resolution,
                                             cv.first.layer,
                                             cv.second.hw * resolution,
                                             cv.second.hh * resolution,
                                             cv.second.rot, cv.second.oval});
                    prerouted_nets.insert(nid);
                    continue;
                }
            }
            Net net;
            net.id = bidx;
            for (const auto& cv : cells) net.pins.push_back(cv.first);
            b.nets().push_back(std::move(net));
            for (const auto& cv : cells) {
                b.add_pad_shape((size_t)bidx, cv.first, cv.second.cx, cv.second.cy,
                                cv.second.hw, cv.second.hh, cv.second.rot, cv.second.oval);
                // Drilled barrel pre-connects the column: layer transitions riding it
                // are not router-placed vias.
                if (cv.second.thru) b.mark_thru_pad((size_t)bidx, cv.first.x, cv.first.y);
            }
            orig_ids.push_back(nid);
            bidx++;
        }
        if (b.num_nets() == 0)
            throw std::runtime_error("KiCad PCB: no routable nets after padding");

        // Per-net-pair physics tiers: nets whose class differs from Default get
        // their own (half-width, clearance) tier — pair spacing becomes exact
        // instead of the conservative class-max projection.
        {
            const double half_cell = kSnapMargin;
            for (size_t bi = 0; bi < b.nets().size(); ++bi) {
                const int oid = orig_ids[bi];
                std::string nm;
                if (oid < 0 && -oid <= (int)code_names.size())
                    nm = code_names[(size_t)(-oid) - 1];
                else {
                    auto it = decl_code_names.find(oid);
                    if (it != decl_code_names.end()) nm = it->second;
                }
                auto mr = member_rules.find(nm);
                if (mr == member_rules.end() && nm.size() > 1 && nm[0] == '/')
                    mr = member_rules.find(nm.substr(1));
                if (mr == member_rules.end()) continue;
                const double cls_tw = mr->second.tw > 0.0 ? mr->second.tw : base_tw_mm;
                const double cls_clr = mr->second.clr > 0.0 ? mr->second.clr : base_clr_mm;
                if (cls_tw == base_tw_mm && cls_clr == base_clr_mm)
                    continue;   // Default-equal class: stays tier 0
                // Ceil like drc_clearance_cells(): the rule is a MINIMUM — a
                // fractional tier would sit LOOSER than the ceiled tier 0.
                const double clr_cells = std::max(
                    1.0, std::ceil((cls_clr + half_cell) / resolution - 1e-9));
                b.set_net_class_physics((int)bi, cls_tw * 0.5 / resolution,
                                        clr_cells);
            }
        }

        // No-net pad copper/holes become keepout shapes: clearance-dilated obstacles
        // owned by no net (same coordinate frame as pads: fractional cells).
        for (const AbsObstacle& ob : obstacles) {
            if (ob.via_only)
                b.add_via_keepout_shape(ob.layer, (ob.x - min_x) / resolution,
                                        (ob.y - min_y) / resolution,
                                        ob.half_w / resolution, ob.half_h / resolution,
                                        ob.rot, ob.oval);
            else
                b.add_keepout_shape(ob.layer, (ob.x - min_x) / resolution,
                                    (ob.y - min_y) / resolution,
                                    ob.half_w / resolution, ob.half_h / resolution,
                                    ob.rot, ob.oval);
        }

        // Stage-10 fidelity: report collapses + minimum distinct-pad spacing, and refuse
        // coarser-than-pad-pitch resolution (collapsed pins -> lost nets).
        double min_sp = std::numeric_limits<double>::infinity();
        for (size_t a = 0; a < all_centres.size(); ++a)
            for (size_t q = a + 1; q < all_centres.size(); ++q) {
                // Only pads sharing a copper layer can collapse into the same cell.
                if ((centre_layer_masks[a] & centre_layer_masks[q]) == 0) continue;
                double dx = all_centres[a].first - all_centres[q].first;
                double dy = all_centres[a].second - all_centres[q].second;
                double d = std::sqrt(dx * dx + dy * dy);
                if (d < min_sp) min_sp = d;
            }
        if (info) {
            info->layer_names = copper_names;
            info->net_names.clear();
            for (int oid : orig_ids) {
                if (oid < 0 && -oid <= (int)code_names.size())
                    info->net_names.push_back(code_names[(size_t)(-oid) - 1]);
                else {
                    // numeric-coded boards: name from the root net table, so
                    // project-file (.kicad_pro) netclass matching works on
                    // every dialect, not just v11 name-keyed files
                    auto it = decl_code_names.find(oid);
                    info->net_names.push_back(
                        it != decl_code_names.end() ? it->second : std::string());
                }
            }
            // Per-net emission geometry from the net's own class (name-matched;
            // 0 = Default/unknown -> writer falls back to Default's values).
            info->default_track_width = default_tw;
            info->default_via_dia = default_via;
            info->default_via_drill = default_drill;
            info->net_widths.clear();
            info->net_via_dias.clear();
            info->net_via_drills.clear();
            for (int oid : orig_ids) {
                std::string nm;
                if (oid < 0 && -oid <= (int)code_names.size())
                    nm = code_names[(size_t)(-oid) - 1];
                else {
                    auto it = decl_code_names.find(oid);
                    if (it != decl_code_names.end()) nm = it->second;
                }
                auto mr = member_rules.find(nm);
                if (mr == member_rules.end() && nm.size() > 1 && nm[0] == '/')
                    mr = member_rules.find(nm.substr(1));
                if (mr != member_rules.end()) {
                    info->net_widths.push_back(mr->second.tw);
                    info->net_via_dias.push_back(mr->second.via);
                    info->net_via_drills.push_back(mr->second.drill);
                } else {
                    info->net_widths.push_back(0.0);
                    info->net_via_dias.push_back(0.0);
                    info->net_via_drills.push_back(0.0);
                }
            }
            info->original_net_ids = std::move(orig_ids);   // AFTER the fills read it
            info->pre_routed_nets = (int)prerouted_nets.size();
            info->partial_nets = partial_net_count;
            info->pour_fed_nets.clear();
            for (int oid : pour_skipped_keys) {
                if (oid < 0 && -oid <= (int)code_names.size())
                    info->pour_fed_nets.push_back(code_names[(size_t)(-oid) - 1]);
                else {
                    auto it = decl_code_names.find(oid);
                    info->pour_fed_nets.push_back(
                        it != decl_code_names.end() ? it->second
                                                    : "net#" + std::to_string(oid));
                }
            }
            info->rule_clearance = rule_clearance;
            info->rule_track_width = rule_tw;
            info->rule_via_dia = rule_via;
            info->rule_via_drill = rule_drill;
            info->border_min_x = min_x; info->border_min_y = min_y;
            info->border_max_x = max_x; info->border_max_y = max_y;
            info->pin_collisions = collisions;
            info->min_pad_spacing = min_sp;
        }
        if (reject_collisions && collisions > 0) {
            throw std::runtime_error(
                std::string("load_kicad_pcb: resolution too coarse: ") +
                std::to_string(collisions) +
                " distinct pad(s) collapsed into the same cell; use a finer resolution (<= " +
                std::to_string(recommended_resolution(min_sp)) + " board units/cell)");
        }
        return b;
    };

    try {
        return body();
    } catch (const std::runtime_error& e) {
        throw; // already normalized
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("malformed KiCad PCB: ") + e.what());
    }
}

} // namespace routing

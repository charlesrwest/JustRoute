#include "routing/astar.hpp"
#include "routing/compat.hpp"
#include "routing/board.hpp"
#include "routing/wave_gpu.hpp"

#include <algorithm>
#include <limits>
#include <cstdint>
#include <unordered_set>
#include <climits>
#include <atomic>
#include <cstring>
#include <functional>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <map>
#include <memory>
#include <mutex>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>

namespace routing {

double PadShape::distance(double px, double py) const {
    const double dx = px - cx, dy = py - cy;
    // into the pad-local frame
    const double lx = dx * cos_r + dy * sin_r;
    const double ly = -dx * sin_r + dy * cos_r;
    if (oval) {
        // Capsule: a segment along the longer half-extent with radius min(half_w, half_h).
        const double r = std::min(half_w, half_h);
        const double ax = std::max(0.0, half_w - r), ay = std::max(0.0, half_h - r);
        const double qx = std::max(0.0, std::abs(lx) - ax);
        const double qy = std::max(0.0, std::abs(ly) - ay);
        return std::sqrt(qx * qx + qy * qy) - r;
    }
    // Rectangle SDF (negative inside).
    const double qx = std::abs(lx) - half_w, qy = std::abs(ly) - half_h;
    const double ox = std::max(qx, 0.0), oy = std::max(qy, 0.0);
    return std::sqrt(ox * ox + oy * oy) + std::min(std::max(qx, qy), 0.0);
}

void Board::mark_thru_pad(std::size_t net_idx, int x, int y) {
    if (x < 0 || y < 0 || x >= grid_.width() || y >= grid_.height()) return;
    thru_pad_col_[(size_t)y * (size_t)grid_.width() + (size_t)x] = (int32_t)net_idx + 1;
}

bool Board::own_pad_barrel(const Cell& a, const Cell& b, int32_t tok) const {
    // a and b share (x,y) (a via transition only changes layer).
    if (!valid(a) || !valid(b)) return false;
    return thru_pad_col_[(size_t)a.y * (size_t)grid_.width() + (size_t)a.x] == tok;
}

// Number of via transitions in a routed net: consecutive cells within a segment that
// differ in layer (a layer change is a via). A transition riding the net's own thru-hole
// pad barrel is pre-drilled pad copper, not a placed via, and is not counted. Used both
// when re-routing the tail and when accounting for preserved prefix nets whose traces
// are kept unchanged.
int Board::count_net_vias(const Net& net) const {
    const int32_t tok = (int32_t)net.id + 1;
    int vias = 0;
    for (const auto& seg : net.segments)
        for (size_t i = 1; i < seg.size(); ++i)
            if (seg[i].layer != seg[i - 1].layer && !own_pad_barrel(seg[i - 1], seg[i], tok))
                vias++;
    return vias;
}

Board::Board(int layers, int width, int height, double resolution,
             double via_cost, double base_cost, double design_rule_clearance)
    : grid_(layers, width, height, resolution),
      via_cost_(quantize_cost(via_cost)),
      base_cost_(quantize_cost(base_cost > 0 ? base_cost : 1.0)),
      drc_clearance_(design_rule_clearance > 0.0 ? design_rule_clearance : kKiCadClearanceMM),
      trace_hw_cells_(kKiCadTraceHalfWidthMM / resolution),
      via_radius_cells_(kKiCadViaRadiusMM / resolution),
      pad_radius_cells_(kKiCadViaRadiusMM / resolution),
      owner_(grid_.size(), 0),
      pad_owner_(grid_.size(), 0),
      trace_owner_(grid_.size(), 0),
      thru_pad_col_((size_t)width * (size_t)height, 0),
      pad_radius_cell_(grid_.size(), 0.0),
      copper_(grid_.size(), 0),
      pad_inflated_(grid_.size(), 0),
      via_pad_zone_(grid_.size(), 0),
      cl_zone_(grid_.size(), 0),
      via_owner_(grid_.size(), 0),
      pad_avoid_cost_(grid_.size(), 0.0),
      trace_avoid_cost_(grid_.size(), 0.0) {}

void Board::set_avoidance(std::size_t net_idx, double pad_mult, double trace_mult) {
    if (avoid_.size() <= net_idx) avoid_.resize(net_idx + 1);
    avoid_[net_idx].pad_avoid_mult = std::max(0.0, pad_mult);
    avoid_[net_idx].trace_avoid_mult = std::max(0.0, trace_mult);
    if (pad_mult > 0.0 || trace_mult > 0.0) ensure_avoid_baked();
}

void Board::clear_avoidance() {
    avoid_.assign(nets_.size(), NetAvoidance{});
    halo_cost_.clear();
    halo_sources_.clear();
}

void Board::add_manual_trace(const Cell& c, int net_id) {
    if (!valid(c) || net_id < 0) return;
    size_t i = grid_.index(c);
    const int32_t tok = (int32_t)net_id + 1;
    trace_owner_[i] = tok;
    copper_[i] = 1;
    owner_[i] = tok;
}

// Bake the falloff shape from a single source cell into a cost grid: for every
// cell on the source's layer, add sharp/(dist+1) where dist is the Euclidean
// cell distance within that layer.
void Board::prepare_pad_keepout() {
    std::fill(pad_inflated_.begin(), pad_inflated_.end(), 0);
    std::fill(via_pad_zone_.begin(), via_pad_zone_.end(), 0);
    // Routing-side pad inflation applies only when physical trace width is enabled (it is a
    // Stage-8.4c fidelity feature; with tw=0 the prior routing behavior is preserved). The
    // margin matches the DRC pad-trace threshold clr + tw, measured from the pad's REAL
    // copper boundary (exact Minkowski dilation via the shape SDF) — not from a
    // circumscribed circle, which sealed physically-legal fine-pitch escape corridors.
    const double infl = (trace_hw_cells_ > 0.0)
                            ? ((double)drc_clearance_cells() + trace_hw_cells_)
                            : 0.0;
    if (infl <= 0.0) return;
    // Via emergence must keep (via_radius + tw + clearance) from pad copper: both the
    // physical via-pad clearance and enough margin that a via's own hard zone cannot
    // seal a pad's approach (see via_ok). Max-tier values: via machinery is not
    // pair-aware, so it stays safe for every class.
    const double vinfl = via_radius_cells_ > 0.0
                             ? via_radius_cells_ + max_hw_cells() + max_clr_cells()
                             : 0.0;
    const int W = grid_.width(), H = grid_.height();
    static constexpr int32_t CONFLICT = -1;
    auto stamp = [&](std::vector<int32_t>& mask, const PadShape& ps, double margin) {
        // Rotation-safe bound: a rotated rect's corner reaches up to
        // hypot(half_w, half_h) from center — max(hw, hh) truncates the scan box
        // and leaves the tips of 45-degree pads unstamped (jackco thermal diamond).
        const double reach = std::hypot(ps.half_w, ps.half_h) + margin;
        const int x0 = std::max(0, (int)std::floor(ps.cx - reach));
        const int x1 = std::min(W - 1, (int)std::ceil(ps.cx + reach));
        const int y0 = std::max(0, (int)std::floor(ps.cy - reach));
        const int y1 = std::min(H - 1, (int)std::ceil(ps.cy + reach));
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                if (ps.distance((double)x, (double)y) >= margin) continue;
                size_t j = grid_.index(ps.layer, x, y);
                int32_t cur = mask[j];
                if (cur == 0) mask[j] = ps.tok;
                else if (cur != ps.tok) mask[j] = CONFLICT; // pads of >=2 nets cover it
            }
    };
    for (auto& t : extra_tiers_) std::fill(t.pad_infl.begin(), t.pad_infl.end(), 0);
    for (const PadShape& ps : pad_shapes_) {
        if (ps.via_only) {
            // Via-only rule area: only via emergence is banned; margin is the via
            // radius (via copper may not enter the area), not the full pad vinfl.
            if (via_radius_cells_ > 0.0)
                stamp(via_pad_zone_, ps, via_radius_cells_);
            continue;
        }
        // Per-tier margins: pad of class (c_pad) vs tier-t traffic needs
        // max(c_pad, tier clr) + tier hw from the pad edge. Obstacle shapes
        // (tok = INT32_MAX) have no class: c_pad = tier clr (today's rule).
        const int pt = (ps.tok > 0 && ps.tok < INT32_MAX) ? tier_of_tok(ps.tok) : 0;
        const double c_pad = tier_clr(pt);
        stamp(pad_inflated_, ps, std::max(c_pad, tier_clr(0)) + tier_hw(0));
        for (size_t t = 0; t < extra_tiers_.size(); ++t)
            stamp(extra_tiers_[t].pad_infl, ps,
                  std::max(c_pad, tier_clr((int)t + 1)) + tier_hw((int)t + 1));
        if (vinfl > 0.0) stamp(via_pad_zone_, ps, vinfl);
    }
}

void Board::add_source_avoidance(const Cell& src, std::vector<double>& cost, double sharp) {
    // Lazy baking: every consumer of these fields (incl. self-exemption
    // scratches) is guarded by `mult > 0`, and the first nonzero multiplier
    // triggers ensure_avoid_baked() -> rebuild_avoidance(), which reconstructs
    // the exact field state from pad_owner_/trace_owner_. Profiling showed
    // 65% of easy-board instructions baking fields nothing reads.
    if (!avoid_baked_) return;
    const int l = src.layer;
    const int w = grid_.width();
    const int h = grid_.height();
    // Bounded radius: the kernel is exactly 0 beyond `falloff_radius_`, so only the
    // O((2R+1)^2) neighborhood of the source needs visiting (was O(w*h) per source).
    const double R = falloff_radius_;
    const int r = std::max(0, (int)std::ceil(R));
    const int x0 = std::max(0, src.x - r), x1 = std::min(w - 1, src.x + r);
    const int y0 = std::max(0, src.y - r), y1 = std::min(h - 1, src.y + r);
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            double dx = (double)(x - src.x), dy = (double)(y - src.y);
            double dist = euclid_dist(dx, dy);
            if (dist >= R) continue;      // exactly 0 beyond radius
            size_t i = grid_.index(l, x, y);
            cost[i] += falloff_cost(dist, sharp, R);
        }
    }
}

// Sum of falloff from net nid's own pads to a given cell (for self-exemption).
// `nid` is the net's stable id (not its array position).
double Board::self_pad_cost(const Cell& c, int nid) const {
    double sum = 0.0;
    const double R = falloff_radius_;
    for (const Net& n : nets_) {
        if (n.id != nid) continue;
        for (const Cell& pad : n.pins) {
            if (pad.layer != c.layer) continue; // within-layer avoidance
            // Euclidean cell distance.
            double dist = euclid_dist((double)(pad.x - c.x), (double)(pad.y - c.y));
            if (dist >= R) continue;            // contribution is 0 beyond radius
            sum += falloff_cost(dist, pad_sharp_, R);
        }
    }
    return sum;
}

void Board::add_repel_halo(int owner_nid, const std::vector<Cell>& cells,
                           double weight) {
    ensure_avoid_baked();                    // add_source_avoidance gates on it
    if (halo_cost_.empty()) halo_cost_.assign(grid_.size(), 0.0);
    halo_sharp_ = pad_sharp_ * std::max(1e-6, weight);
    for (const Cell& c : cells) {
        if (!valid(c)) continue;
        halo_sources_.emplace_back(owner_nid, c);
        add_source_avoidance(c, halo_cost_, halo_sharp_);
    }
}

// Sum of halo falloff from net nid's OWN halo seeds to a cell (self-exemption,
// so the blocked net can still reach the very pad its halo protects). Mirrors
// self_pad_cost; halo_sources_ is small (only blocked pads), so this is cheap.
double Board::self_halo_cost(const Cell& c, int nid) const {
    if (halo_sources_.empty()) return 0.0;
    double sum = 0.0;
    const double R = falloff_radius_;
    for (const auto& hs : halo_sources_) {
        if (hs.first != nid) continue;
        const Cell& s = hs.second;
        if (s.layer != c.layer) continue;
        double dist = euclid_dist((double)(s.x - c.x), (double)(s.y - c.y));
        if (dist >= R) continue;
        sum += falloff_cost(dist, halo_sharp_, R);
    }
    return sum;
}

void Board::add_pad(std::size_t net_idx, const Cell& c, double radius_cells) {
    if (!valid(c)) return;
    size_t i = grid_.index(c);
    // 1-based so 0 always means free. Token derives from the STABLE net id
    // (commit paths use net.id+1); position==id only until the first
    // move_net, so a position-based token corrupts reordered boards.
    int32_t token = (net_idx < nets_.size() ? nets_[net_idx].id
                                            : (int32_t)net_idx) + 1;
    pad_owner_[i] = token;
    owner_[i] = token;
    pad_radius_cell_[i] = radius_cells > 0.0 ? radius_cells : 0.0;
    max_pad_radius_cells_ = std::max(max_pad_radius_cells_, pad_radius_cell_[i]);
    // All clearance rules run on shapes: a legacy radius pad is a circle (a point pad
    // when radius 0, which reproduces the old center-distance semantics exactly).
    const double r = pad_radius_cell_[i];
    pad_shapes_.push_back(PadShape{token, c.layer, (double)c.x, (double)c.y,
                                   r, r, 1.0, 0.0, /*oval=*/true});
    add_source_avoidance(c, pad_avoid_cost_, pad_sharp_);
}

void Board::add_pin_pad(std::size_t net_idx, const Cell& c, double radius_cells) {
    if (!valid(c) || net_idx >= nets_.size()) return;
    if (owner_[grid_.index(c)] != 0) return;   // never clobber existing copper/pads
    add_pad(net_idx, c, radius_cells);
    nets_[net_idx].pins.push_back(c);
}

void Board::add_keepout_shape(int layer, double cx, double cy, double half_w,
                              double half_h, double rot_deg, bool oval) {
    if (layer < 0 || layer >= grid_.layers()) return;
    // Obstacle token: matches no net (net tokens are net_idx + 1, far below this), so
    // every net treats the dilated footprint as foreign and it never conflicts with the
    // pad-inflation exemptions.
    static constexpr int32_t kObstacleTok = INT32_MAX;
    const double rad = rot_deg * M_PI / 180.0;
    pad_shapes_.push_back(PadShape{kObstacleTok, layer, cx, cy,
                                   std::max(0.0, half_w), std::max(0.0, half_h),
                                   std::cos(rad), std::sin(rad), oval});
}

void Board::add_via_keepout_shape(int layer, double cx, double cy, double half_w,
                                  double half_h, double rot_deg, bool oval) {
    if (layer < 0 || layer >= grid_.layers()) return;
    static constexpr int32_t kObstacleTok = INT32_MAX;
    const double rad = rot_deg * M_PI / 180.0;
    pad_shapes_.push_back(PadShape{kObstacleTok, layer, cx, cy,
                                   std::max(0.0, half_w), std::max(0.0, half_h),
                                   std::cos(rad), std::sin(rad), oval,
                                   /*via_only=*/true});
}

void Board::add_pad_shape(std::size_t net_idx, const Cell& c,
                          double cx, double cy, double half_w, double half_h,
                          double rot_deg, bool oval) {
    if (!valid(c)) return;
    size_t i = grid_.index(c);
    int32_t token = (int32_t)net_idx + 1;
    pad_owner_[i] = token;
    owner_[i] = token;
    const double rmax = std::max(half_w, half_h);
    pad_radius_cell_[i] = rmax;   // legacy accessor only; rules use the shape
    max_pad_radius_cells_ = std::max(max_pad_radius_cells_, rmax);
    const double rad = rot_deg * M_PI / 180.0;
    pad_shapes_.push_back(PadShape{token, c.layer, cx, cy,
                                   std::max(0.0, half_w), std::max(0.0, half_h),
                                   std::cos(rad), std::sin(rad), oval});
    add_source_avoidance(c, pad_avoid_cost_, pad_sharp_);
}

int Board::move_net(int from_idx, int to_idx) {
    int n = (int)nets_.size();
    if (n == 0) return 0;
    from_idx = std::max(0, std::min(from_idx, n - 1));
    to_idx = std::max(0, std::min(to_idx, n - 1));
    if (from_idx == to_idx) return from_idx;

    if (avoid_.size() < (size_t)n) avoid_.resize((size_t)n);

    Net moved = std::move(nets_[(size_t)from_idx]);
    NetAvoidance am = avoid_[(size_t)from_idx];
    if (from_idx < to_idx) {
        for (int i = from_idx; i < to_idx; ++i) {
            nets_[(size_t)i] = std::move(nets_[(size_t)(i + 1)]);
            avoid_[(size_t)i] = avoid_[(size_t)(i + 1)];
        }
    } else {
        for (int i = from_idx; i > to_idx; --i) {
            nets_[(size_t)i] = std::move(nets_[(size_t)(i - 1)]);
            avoid_[(size_t)i] = avoid_[(size_t)(i - 1)];
        }
    }
    nets_[(size_t)to_idx] = std::move(moved);
    avoid_[(size_t)to_idx] = am;
    return to_idx;
}

void Board::coat(const Cell& c, int lmin, int lmax, double radius, int32_t tok,
                 std::vector<Cell>* claimed) {
    if (radius <= 0.0) return;
    const int r = (int)std::ceil(radius);
    const int w = grid_.width(), h = grid_.height();
    const int x0 = std::max(0, c.x - r), x1 = std::min(w - 1, c.x + r);
    const int y0 = std::max(0, c.y - r), y1 = std::min(h - 1, c.y + r);
    for (int l = lmin; l <= lmax && l < grid_.layers(); ++l) {
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                size_t vi = grid_.index(l, x, y);
                if (pad_owner_[vi] != 0 && pad_owner_[vi] != tok) continue; // foreign pad
                if (trace_owner_[vi] == 0) {
                    trace_owner_[vi] = tok;
                    owner_[vi] = tok;
                    // Keep soft avoidance consistent with route_from's rebuild.
                    add_source_avoidance(Cell{l, x, y}, trace_avoid_cost_, trace_sharp_);
                    if (claimed) claimed->push_back(Cell{l, x, y});
                }
                cl_zone_[vi] = tok; // unconditional clearance zone
            }
    }
}

void Board::set_zone(const Cell& c, int lmin, int lmax, double radius, int32_t tok) {
    if (radius <= 0.0) return;
    const int r = (int)std::ceil(radius);
    const int w = grid_.width(), h = grid_.height();
    const int x0 = std::max(0, c.x - r), x1 = std::min(w - 1, c.x + r);
    const int y0 = std::max(0, c.y - r), y1 = std::min(h - 1, c.y + r);
    for (int l = lmin; l <= lmax && l < grid_.layers(); ++l)
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                size_t vi = grid_.index(l, x, y);
                if (pad_owner_[vi] != 0 && pad_owner_[vi] != tok) continue; // foreign pad
                cl_zone_[vi] = tok;
            }
}

void Board::set_net_class_physics(int net_id, double hw_cells, double clr_cells) {
    if (net_id < 0) return;
    if (tier_of_id_.size() < nets_.size()) tier_of_id_.assign(nets_.size(), 0);
    if ((size_t)net_id >= tier_of_id_.size()) tier_of_id_.resize((size_t)net_id + 1, 0);
    const double eps = 1e-9;
    if (std::abs(hw_cells - trace_hw_cells_) < eps &&
        std::abs(clr_cells - (double)drc_clearance_cells()) < eps) {
        tier_of_id_[(size_t)net_id] = 0;
        return;
    }
    for (size_t t = 0; t < extra_tiers_.size(); ++t) {
        if (std::abs(hw_cells - extra_tiers_[t].hw) < eps &&
            std::abs(clr_cells - extra_tiers_[t].clr) < eps) {
            tier_of_id_[(size_t)net_id] = (int)t + 1;
            return;
        }
    }
    PhysTier nt;
    nt.hw = hw_cells;
    nt.clr = clr_cells;
    nt.zone.assign(grid_.size(), 0);
    nt.pad_infl.assign(grid_.size(), 0);
    extra_tiers_.push_back(std::move(nt));
    tier_of_id_[(size_t)net_id] = (int)extra_tiers_.size();
}

void Board::set_zone_mask(std::vector<int32_t>& mask, const Cell& c, int lmin,
                          int lmax, double radius, int32_t tok) {
    if (radius <= 0.0) return;
    const int r = (int)std::ceil(radius);
    const int w = grid_.width(), h = grid_.height();
    const int x0 = std::max(0, c.x - r), x1 = std::min(w - 1, c.x + r);
    const int y0 = std::max(0, c.y - r), y1 = std::min(h - 1, c.y + r);
    for (int l = lmin; l <= lmax && l < grid_.layers(); ++l)
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                size_t vi = grid_.index(l, x, y);
                if (pad_owner_[vi] != 0 && pad_owner_[vi] != tok) continue;
                mask[vi] = tok;
            }
}

void Board::rebuild_cl_zone() {
    std::fill(cl_zone_.begin(), cl_zone_.end(), 0);
    for (auto& t : extra_tiers_) std::fill(t.zone.begin(), t.zone.end(), 0);
    std::fill(via_owner_.begin(), via_owner_.end(), 0);
    const double tw = trace_hw_cells_;
    const double clr = (double)drc_clearance_cells();
    if (tw <= 0.0 && clr <= 0.0) return;
    // (pads are NOT baked here: enter_cost already hard-blocks the identical radius
    // clr + tw + padR via pad_inflated_, which additionally represents multi-net
    // overlap cells (CONFLICT) that this token-keyed zone cannot express.)
    // real trace copper: each tier's zone at the exact pair radius
    // (owner hw + tier hw + max(owner clr, tier clr)); tier 0 reduces to the
    // legacy 2*tw + clr on single-class boards.
    if (tw > 0.0) {
        for (size_t i = 0; i < copper_.size(); ++i) {
            if (!copper_[i]) continue;
            int32_t tok = trace_owner_[i];
            if (tok == 0) continue;
            Cell c = grid_.unindex(i);
            const int ot = tier_of_tok(tok);
            const double hw_o = tier_hw(ot), c_o = tier_clr(ot);
            set_zone(c, c.layer, c.layer, pair_zone_radius(hw_o, c_o, 0), tok);
            for (size_t t = 0; t < extra_tiers_.size(); ++t)
                set_zone_mask(extra_tiers_[t].zone, c, c.layer, c.layer,
                              pair_zone_radius(hw_o, c_o, (int)t + 1), tok);
        }
    }
    // vias: hard via-clearance zone covering via-trace/pad spacing, per tier.
    if (via_radius_cells_ > 0.0) {
        for (const Net& net : nets_)
            for (const auto& path : net.segments)
                for (size_t k = 0; k + 1 < path.size(); ++k)
                    if (path[k].layer != path[k + 1].layer &&
                        !own_pad_barrel(path[k], path[k + 1], (int32_t)net.id + 1)) {
                        int32_t tok = (int32_t)net.id + 1;
                        const int ot = tier_of_tok(tok);
                        const double c_o = tier_clr(ot);
                        Cell vc{path[k].layer, path[k].x, path[k].y};
                        set_zone(vc, 0, grid_.layers() - 1,
                                 via_radius_cells_ + tier_hw(0) +
                                     std::max(c_o, tier_clr(0)),
                                 tok);
                        for (size_t t = 0; t < extra_tiers_.size(); ++t)
                            set_zone_mask(extra_tiers_[t].zone, vc, 0,
                                          grid_.layers() - 1,
                                          via_radius_cells_ + tier_hw((int)t + 1) +
                                              std::max(c_o, tier_clr((int)t + 1)),
                                          tok);
                        // via barrel occupies (x,y) on every layer
                        for (int l = 0; l < grid_.layers(); ++l)
                            via_owner_[grid_.index(l, vc.x, vc.y)] = tok;
                    }
    }
}

namespace {
// Bitplane scratch: 5 planes of (layers*H*S) 64-bit words, reused per thread.
struct WaveScratch {
    std::vector<uint64_t> ent, via, vis, cur, nxt;
    std::vector<uint16_t> map;                     // wave indices (wave_map)
    // Slab-lazy masks: ent/via are built 64-row slabs at a time, on first
    // touch, per probe (epoch-stamped) — the per-cell rule evaluation is the
    // dominant cost and becomes proportional to the region actually flooded.
    std::vector<uint32_t> ent_slab, via_slab;
    uint32_t slab_ep = 0;
    void begin(size_t words, size_t ent_slabs, size_t via_slabs) {
        if (ent.size() < words) {
            ent.resize(words); via.resize(words); vis.resize(words);
            cur.resize(words); nxt.resize(words);
        }
        if (ent_slab.size() < ent_slabs) ent_slab.assign(ent_slabs, 0);
        if (via_slab.size() < via_slabs) via_slab.assign(via_slabs, 0);
        if (++slab_ep == 0) {
            std::fill(ent_slab.begin(), ent_slab.end(), 0);
            std::fill(via_slab.begin(), via_slab.end(), 0);
            slab_ep = 1;
        }
    }
};
thread_local WaveScratch tls_wave;
std::atomic<int> g_wave_threads{1};
} // namespace

void set_wave_threads(int n) {
    g_wave_threads.store(n > 0 ? n : 1, std::memory_order_relaxed);
}
int wave_threads() { return g_wave_threads.load(std::memory_order_relaxed); }

bool Board::wave_probe(int position, const std::vector<Cell>& seeds,
                       const std::vector<Cell>& targets) const {
    return wave_flood_impl(position, seeds, targets, nullptr);
}

bool Board::wave_probe_superset(int position, const std::vector<Cell>& seeds,
                                const std::vector<Cell>& targets) {
    // Escalation-screen semantics (the scalar probe's SUPERSET rules) at
    // bitplane speed: via transitions gated by the column-OR of the EXACT
    // per-cell via_ok (Chebyshev fast path) — "either endpoint passes" on
    // 2-layer boards, a safe over-approximation beyond. The screen may only
    // SKIP escalation on a False verdict, so over-approximation is the safe
    // direction; the equality gate vs the scalar probe is the acceptance
    // test (never False where scalar is True).
    const int W = grid_.width(), H = grid_.height(), L = grid_.layers();
    std::vector<uint8_t> cellv =
        fast_via_mask_window(position, 0, 0, W - 1, H - 1);
    std::vector<uint8_t> colv((size_t)W * H, 0);
    const size_t plane = (size_t)W * H;
    for (int l = 0; l < L; ++l) {
        const uint8_t* src = cellv.data() + (size_t)l * plane;
        for (size_t i = 0; i < plane; ++i) colv[i] |= src[i];
    }
    // Gate-free target touch (the scalar probe counts adjacency to a target
    // BEFORE any gate — through blocked cells, illegal vias, cut corners):
    // expand targets to their 10-neighborhood; reaching a neighbor legally
    // IS the scalar's touch condition.
    std::unordered_set<size_t> texp;
    std::vector<Cell> targets2;
    auto add_tc = [&](const Cell& c) {
        if (!valid(c)) return;
        if (texp.insert(grid_.index(c)).second) targets2.push_back(c);
    };
    static const int TDX[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    static const int TDY[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    for (const Cell& t : targets) {
        add_tc(t);
        for (int d = 0; d < 8; ++d)
            add_tc(Cell{t.layer, t.x + TDX[d], t.y + TDY[d]});
        add_tc(Cell{t.layer - 1, t.x, t.y});
        add_tc(Cell{t.layer + 1, t.x, t.y});
    }
    return wave_flood_impl(position, seeds, targets2, nullptr, colv.data());
}

const uint16_t* Board::wave_map(int position,
                                const std::vector<Cell>& seeds) const {
    auto& Z = tls_wave;
    const int W = grid_.width(), H = grid_.height(), L = grid_.layers();
    const size_t cells = (size_t)L * W * H;
    if (Z.map.size() < cells) Z.map.resize(cells);
    // GPU first (full floods are its natural shape); CPU flood on any failure.
    if (wave_gpu_available() && wave_gpu_enabled()) {
        const int S32 = (W + 31) >> 5;
        const size_t p32 = (size_t)H * S32, t32 = (size_t)L * p32;
        static thread_local std::vector<uint32_t> e32, v32, c32;
        if (e32.size() < t32) { e32.resize(t32); c32.resize(t32); }
        if (v32.size() < p32) v32.resize(p32);
        build_masks32(position, seeds, e32.data(), v32.data(), c32.data());
        if (wave_gpu_map(W, H, L, e32.data(), v32.data(), c32.data(),
                         Z.map.data()))
            return Z.map.data();
    }
    std::fill(Z.map.begin(), Z.map.begin() + (long)cells, (uint16_t)0);
    wave_flood_impl(position, seeds, {}, Z.map.data());
    return Z.map.data();
}

// Branchless 32-bit-packed mask builder shared by the GPU path and
// dump_wave_masks (identical rules to the 64-bit slab builder).
void Board::build_masks32(int position, const std::vector<Cell>& seeds,
                          uint32_t* ent, uint32_t* via, uint32_t* cur) const {
    const int W = grid_.width(), H = grid_.height(), L = grid_.layers();
    const int S32 = (W + 31) >> 5;
    const size_t p32 = (size_t)H * S32, t32 = (size_t)L * p32;
    const Net& net = nets_[(size_t)position];
    const int32_t tok = (int32_t)net.id + 1;
    std::memset(cur, 0, t32 * 4);
    for (int l = 0; l < L; ++l)
        for (int y = 0; y < H; ++y) {
            const size_t cb = ((size_t)l * H + y) * (size_t)W;
            const size_t wb = ((size_t)l * H + y) * (size_t)S32;
            for (int wx = 0; wx < S32; ++wx) {
                const int x0 = wx << 5;
                const int xmax = std::min(32, W - x0);
                const int32_t* op = owner_.data() + cb + x0;
                const int32_t* cp = zone_data_for(tok) + cb + x0;
                const int32_t* pp = pad_infl_data_for(tok) + cb + x0;
                const int32_t* wp = pad_owner_.data() + cb + x0;
                uint32_t bits = 0;
                for (int b = 0; b < xmax; ++b)
                    bits |= (uint32_t)(((op[b] == 0) | (op[b] == tok)) &
                                       ((cp[b] == 0) | (cp[b] == tok)) &
                                       ((pp[b] == 0) | (pp[b] == tok) |
                                        (wp[b] == tok))) << b;
                ent[wb + wx] = bits;
            }
        }
    for (int y = 0; y < H; ++y)
        for (int wx = 0; wx < S32; ++wx) {
            const int x0 = wx << 5;
            const int xmax = std::min(32, W - x0);
            uint32_t bits = 0;
            for (int b = 0; b < xmax; ++b) {
                const int x = x0 + b;
                const size_t col = (size_t)y * W + x;
                bool ok = thru_pad_col_[col] == tok;
                if (!ok) {
                    ok = true;
                    for (int l = 0; l < L; ++l) {
                        const int32_t pz =
                            via_pad_zone_[(size_t)l * W * H + col];
                        if (pz != 0 && pz != tok) { ok = false; break; }
                    }
                }
                bits |= (uint32_t)ok << b;
            }
            via[(size_t)y * S32 + wx] = bits;
        }
    for (const Cell& s : seeds)
        if (valid(s))
            cur[((size_t)s.layer * H + s.y) * S32 + (s.x >> 5)] |=
                1u << (s.x & 31);
}

bool Board::wave_flood_impl(int position, const std::vector<Cell>& seeds,
                            const std::vector<Cell>& targets,
                            uint16_t* map,
                            const uint8_t* via_col_override) const {
    const int W = grid_.width(), H = grid_.height(), L = grid_.layers();
    const int S = (W + 63) >> 6;                     // words per row
    const size_t layer_words = (size_t)H * (size_t)S;
    const size_t words = (size_t)L * layer_words;
    const int n_yslabs = (H + 63) >> 6;
    auto& Z = tls_wave;
    Z.begin(words, (size_t)L * (size_t)n_yslabs, (size_t)n_yslabs);

    const Net& net = nets_[(size_t)position];
    const int32_t tok = (int32_t)net.id + 1;
    const uint64_t last_mask = (W & 63) ? ((1ULL << (W & 63)) - 1) : ~0ULL;
    for (size_t wi = 0; wi < words; ++wi) { Z.vis[wi] = 0; Z.cur[wi] = 0; }

    // --- slab-lazy mask builders (64 rows at a time, on first touch) --------
    auto build_ent_slab = [&](int l, int sb) {
        const int ya = sb << 6, yb = std::min(H - 1, (sb << 6) + 63);
        const size_t cell_base = (size_t)l * (size_t)W * (size_t)H;
        const size_t word_base = (size_t)l * layer_words;
        for (int y = ya; y <= yb; ++y) {
            const size_t cb = cell_base + (size_t)y * (size_t)W;
            const size_t wb = word_base + (size_t)y * (size_t)S;
            for (int wx = 0; wx < S; ++wx) {
                const int x0 = wx << 6;
                const int xmax = std::min(64, W - x0);
                // branchless, auto-vectorizable: one 0/1 byte per cell, packed
                uint8_t okb[64];
                const int32_t* op = owner_.data() + cb + x0;
                const int32_t* cp = zone_data_for(tok) + cb + x0;
                const int32_t* pp = pad_infl_data_for(tok) + cb + x0;
                const int32_t* wp = pad_owner_.data() + cb + x0;
                for (int b = 0; b < xmax; ++b) {
                    okb[b] = (uint8_t)(((op[b] == 0) | (op[b] == tok)) &
                                       ((cp[b] == 0) | (cp[b] == tok)) &
                                       ((pp[b] == 0) | (pp[b] == tok) |
                                        (wp[b] == tok)));
                }
                uint64_t bits = 0;
                for (int b = 0; b < xmax; ++b)
                    bits |= (uint64_t)okb[b] << b;
                Z.ent[wb + (size_t)wx] = bits;
            }
        }
    };
    auto ensure_ent = [&](int l, int y) {
        const int sb = y >> 6;
        uint32_t& st = Z.ent_slab[(size_t)l * (size_t)n_yslabs + (size_t)sb];
        if (st != Z.slab_ep) { st = Z.slab_ep; build_ent_slab(l, sb); }
    };
    auto build_via_slab = [&](int sb) {
        const int ya = sb << 6, yb = std::min(H - 1, (sb << 6) + 63);
        for (int y = ya; y <= yb; ++y) {
            const size_t wb = (size_t)y * (size_t)S;
            for (int wx = 0; wx < S; ++wx) {
                uint64_t bits = 0;
                const int x0 = wx << 6;
                const int xmax = std::min(64, W - x0);
                if (via_col_override) {   // superset probe: caller-built mask
                    const uint8_t* src =
                        via_col_override + (size_t)y * (size_t)W + x0;
                    for (int b = 0; b < xmax; ++b)
                        if (src[b]) bits |= 1ULL << b;
                    Z.via[wb + (size_t)wx] = bits;
                    continue;
                }
                for (int b = 0; b < xmax; ++b) {
                    const int x = x0 + b;
                    const size_t col = (size_t)y * (size_t)W + (size_t)x;
                    if (thru_pad_col_[col] == tok) { bits |= 1ULL << b; continue; }
                    bool ok = true;
                    for (int l = 0; l < L; ++l) {
                        const int32_t pz =
                            via_pad_zone_[(size_t)l * (size_t)W * (size_t)H + col];
                        if (pz != 0 && pz != tok) { ok = false; break; }
                    }
                    if (ok) bits |= 1ULL << b;
                }
                Z.via[wb + (size_t)wx] = bits;
            }
        }
    };
    auto ensure_via = [&](int y) {
        const int sb = y >> 6;
        uint32_t& st = Z.via_slab[(size_t)sb];
        if (st != Z.slab_ep) { st = Z.slab_ep; build_via_slab(sb); }
    };
    auto ensure_ent_range = [&](int l, int ya, int yb) {
        for (int sb = std::max(0, ya) >> 6; sb <= std::min(H - 1, yb) >> 6; ++sb)
            ensure_ent(l, sb << 6);
    };

    // --- seed / target bit helpers ----------------------------------------
    auto word_of = [&](const Cell& c) -> size_t {
        return (size_t)c.layer * layer_words + (size_t)c.y * (size_t)S
             + (size_t)(c.x >> 6);
    };
    auto bit_of = [&](const Cell& c) -> uint64_t { return 1ULL << (c.x & 63); };
    for (const Cell& t : targets)
        if (valid(t))
            for (const Cell& s : seeds)
                if (valid(s) && s == t) return true;
    for (const Cell& s : seeds) {
        if (!valid(s)) continue;
        Z.cur[word_of(s)] |= bit_of(s);
        Z.vis[word_of(s)] |= bit_of(s);
        if (map)
            map[((size_t)s.layer * H + s.y) * (size_t)W + s.x] = 1;
    }

    // --- wave loop ---------------------------------------------------------
    // row shift helpers operate per (layer,row): E/W with cross-word carry.
    const int wave_nt = wave_threads();
    auto spread_layer = [&](const uint64_t* f, const uint64_t* ent,
                            uint64_t* out, int yA, int yB) {
        // out = gated 8-neighbor spread of f, rows [yA, yB] of one layer.
        // Rows are independent (pure gather) -> optionally row-parallel.
        const bool par = wave_nt > 1 && (yB - yA + 1) * S > 4096;
        (void)par;
#ifdef ROUTING_HAVE_OPENMP
#pragma omp parallel for schedule(static) num_threads(wave_nt) if (par)
#endif
        for (int y = yA; y <= yB; ++y) {
            const size_t r = (size_t)y * (size_t)S;
            const size_t rN = (size_t)(y - 1) * (size_t)S;   // row above (y-1)
            const size_t rS = (size_t)(y + 1) * (size_t)S;   // row below (y+1)
            for (int wx = 0; wx < S; ++wx) {
                const uint64_t colmask = (wx == S - 1) ? last_mask : ~0ULL;
                auto Ee = [&](const uint64_t* p, size_t row) -> uint64_t {
                    uint64_t v = p[row + (size_t)wx] << 1;
                    if (wx > 0) v |= p[row + (size_t)wx - 1] >> 63;
                    return v;
                };
                auto Ww = [&](const uint64_t* p, size_t row) -> uint64_t {
                    uint64_t v = p[row + (size_t)wx] >> 1;
                    if (wx < S - 1) v |= p[row + (size_t)wx + 1] << 63;
                    return v;
                };
                uint64_t nw = 0;
                // cardinals: E, W same row; N (from y+1... careful): a cell is
                // reached from its N neighbor (y-1) moving S, etc. Gather form:
                // new |= f(neighbor) for each of 8 neighbors.
                nw |= Ee(f, r);                        // from west neighbor
                nw |= Ww(f, r);                        // from east neighbor
                if (y > 0) nw |= f[rN + (size_t)wx];   // from north
                if (y < H - 1) nw |= f[rS + (size_t)wx]; // from south
                // diagonals with the corner-cut rule: reaching nb from its
                // NW/NE/SW/SE neighbor requires BOTH orthogonal cells of that
                // corner enterable: ent at (nb shifted toward neighbor in x)
                // and ent at (nb shifted toward neighbor in y).
                if (y > 0) {
                    uint64_t dNW = Ee(f, rN) & Ee(ent, r) & ent[rN + (size_t)wx];
                    uint64_t dNE = Ww(f, rN) & Ww(ent, r) & ent[rN + (size_t)wx];
                    nw |= dNW | dNE;
                }
                if (y < H - 1) {
                    uint64_t dSW = Ee(f, rS) & Ee(ent, r) & ent[rS + (size_t)wx];
                    uint64_t dSE = Ww(f, rS) & Ww(ent, r) & ent[rS + (size_t)wx];
                    nw |= dSW | dSE;
                }
                out[r + (size_t)wx] = (nw & colmask & ent[r + (size_t)wx]);
            }
        }
    };

    // targets are always touchable (matches the scalar probe's ungated touch;
    // real targets are the net's own copper — enterable anyway)
    std::vector<std::pair<size_t, uint64_t>> tbits;
    tbits.reserve(targets.size());
    for (const Cell& t : targets)
        if (valid(t)) {
            ensure_ent(t.layer, t.y);            // build slab BEFORE the OR-in
            tbits.emplace_back(word_of(t), bit_of(t));
            Z.ent[word_of(t)] |= bit_of(t);
        }

    // per-layer active frontier band [lo, hi] — each wave only touches the
    // band's neighborhood, so total work ~ cells reached, not waves x grid
    std::vector<int> lo(L, H), hi(L, -1);
    for (const Cell& s : seeds) {
        if (!valid(s)) continue;
        lo[s.layer] = std::min(lo[s.layer], s.y);
        hi[s.layer] = std::max(hi[s.layer], s.y);
    }
    for (size_t wi = 0; wi < words; ++wi) Z.nxt[wi] = 0;

    // scan band for layer l: union of the frontier bands of layers l-1, l, l+1
    // (a via can spawn frontier on a neighbor layer's rows), expanded by 2
    auto scan_band = [&](const std::vector<int>& blo, const std::vector<int>& bhi,
                         int l) -> std::pair<int, int> {
        int a = H, b = -1;
        for (int m = std::max(0, l - 1); m <= std::min(L - 1, l + 1); ++m) {
            if (bhi[m] < blo[m]) continue;
            a = std::min(a, blo[m]);
            b = std::max(b, bhi[m]);
        }
        if (b < a) return {1, 0};                          // empty
        return {std::max(0, a - 2), std::min(H - 1, b + 2)};
    };

    uint16_t wave_no = 1;
    while (true) {
        ++wave_no;
        for (int l = 0; l < L; ++l) {
            if (hi[l] < lo[l]) continue;
            const size_t wb = (size_t)l * layer_words;
            const int ya = std::max(0, lo[l] - 1), yb = std::min(H - 1, hi[l] + 1);
            ensure_ent_range(l, std::max(0, ya - 1), std::min(H - 1, yb + 1));
            spread_layer(Z.cur.data() + wb, Z.ent.data() + wb, Z.nxt.data() + wb,
                         ya, yb);
        }
        for (int l = 0; l < L; ++l) {
            if (hi[l] < lo[l]) continue;
            const size_t wb = (size_t)l * layer_words;
            for (int y = lo[l] >> 6; y <= hi[l] >> 6; ++y) ensure_via(y << 6);
            if (l > 0) ensure_ent_range(l - 1, lo[l], hi[l]);
            if (l < L - 1) ensure_ent_range(l + 1, lo[l], hi[l]);
            const size_t w0 = (size_t)lo[l] * (size_t)S;
            const size_t w1 = (size_t)(hi[l] + 1) * (size_t)S;
            for (size_t wi = w0; wi < w1; ++wi) {
                const uint64_t v = Z.cur[wb + wi] & Z.via[wi];
                if (!v) continue;
                if (l > 0)
                    Z.nxt[wb - layer_words + wi] |= v & Z.ent[wb - layer_words + wi];
                if (l < L - 1)
                    Z.nxt[wb + layer_words + wi] |= v & Z.ent[wb + layer_words + wi];
            }
        }
        bool any = false;
        std::vector<int> nlo(L, H), nhi(L, -1);
        for (int l = 0; l < L; ++l) {
            auto [y0, y1] = scan_band(lo, hi, l);
            const size_t wb = (size_t)l * layer_words;
            for (int y = y0; y <= y1; ++y) {
                const size_t r = wb + (size_t)y * (size_t)S;
                bool row_fresh = false;
                for (int wx = 0; wx < S; ++wx) {
                    uint64_t fresh = Z.nxt[r + wx] & ~Z.vis[r + wx];
                    Z.nxt[r + wx] = fresh;
                    Z.vis[r + wx] |= fresh;
                    row_fresh |= (fresh != 0);
                    if (map) {
                        uint64_t bits = fresh;
                        const size_t cbase =
                            ((size_t)l * H + y) * (size_t)W + ((size_t)wx << 6);
                        while (bits) {
                            const int b = rt_ctzll(bits);
                            bits &= bits - 1;
                            map[cbase + b] = wave_no;
                        }
                    }
                }
                if (row_fresh) {
                    nlo[l] = std::min(nlo[l], y);
                    nhi[l] = std::max(nhi[l], y);
                    any = true;
                }
            }
        }
        for (const auto& [twi, tb] : tbits)
            if (Z.vis[twi] & tb) return true;
        if (!any) return false;
        std::swap(Z.cur, Z.nxt);
        // the OLD frontier (now in nxt) lives within the same scan bands: clear
        for (int l = 0; l < L; ++l) {
            auto [y0, y1] = scan_band(lo, hi, l);
            const size_t wb = (size_t)l * layer_words;
            for (int y = y0; y <= y1; ++y) {
                const size_t r = wb + (size_t)y * (size_t)S;
                for (int wx = 0; wx < S; ++wx) Z.nxt[r + wx] = 0;
            }
        }
        lo = nlo;
        hi = nhi;
    }
}

size_t Board::wave_reach_count(int position, const std::vector<Cell>& seeds) const {
    wave_probe(position, seeds, {});
    const size_t words = (size_t)grid_.layers() * (size_t)grid_.height()
                       * (size_t)((grid_.width() + 63) >> 6);
    size_t n = 0;
    for (size_t i = 0; i < words; ++i)
        n += (size_t)rt_popcountll(tls_wave.vis[i]);
    return n;
}

void Board::dump_wave_masks(int position, const std::vector<Cell>& seeds,
                            const std::string& path) const {
    const int W = grid_.width(), H = grid_.height(), L = grid_.layers();
    const int S32 = (W + 31) >> 5;
    const Net& net = nets_[(size_t)position];
    const int32_t tok = (int32_t)net.id + 1;
    std::vector<uint32_t> ent((size_t)L * H * S32, 0), via((size_t)H * S32, 0),
        cur((size_t)L * H * S32, 0);
    for (int l = 0; l < L; ++l)
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) {
                const size_t i = ((size_t)l * H + y) * (size_t)W + x;
                const int32_t o = owner_[i];
                if (o != 0 && o != tok) continue;
                const int32_t cz = zone_data_for(tok)[i];
                if (cz != 0 && cz != tok) continue;
                const int32_t pi2 = pad_infl_data_for(tok)[i];
                if (pi2 != 0 && pi2 != tok && pad_owner_[i] != tok) continue;
                ent[((size_t)l * H + y) * S32 + (x >> 5)] |= 1u << (x & 31);
            }
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            const size_t col = (size_t)y * W + x;
            bool ok = thru_pad_col_[col] == tok;
            if (!ok) {
                ok = true;
                for (int l = 0; l < L; ++l) {
                    const int32_t pz = via_pad_zone_[(size_t)l * W * H + col];
                    if (pz != 0 && pz != tok) { ok = false; break; }
                }
            }
            if (ok) via[(size_t)y * S32 + (x >> 5)] |= 1u << (x & 31);
        }
    for (const Cell& s : seeds)
        if (valid(s))
            cur[((size_t)s.layer * H + s.y) * S32 + (s.x >> 5)] |= 1u << (s.x & 31);
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) throw std::runtime_error("dump_wave_masks: cannot open " + path);
    const int32_t hdr[4] = {W, H, L, S32};
    std::fwrite(hdr, sizeof(hdr), 1, f);
    std::fwrite(ent.data(), 4, ent.size(), f);
    std::fwrite(via.data(), 4, via.size(), f);
    std::fwrite(cur.data(), 4, cur.size(), f);
    std::fclose(f);
}

bool Board::scalar_probe(int position, const std::vector<Cell>& seeds,
                         const std::vector<Cell>& targets) {
    NetContext ctx = make_net_context(position);
    std::unordered_set<size_t> tset;
    int bb[4] = {INT_MAX, INT_MAX, INT_MIN, INT_MIN};
    for (const Cell& t : targets) {
        if (!valid(t)) continue;
        tset.insert(grid_.index(t));
        bb[0] = std::min(bb[0], t.x); bb[1] = std::min(bb[1], t.y);
        bb[2] = std::max(bb[2], t.x); bb[3] = std::max(bb[3], t.y);
    }
    auto is_t = [&](const Cell& c) -> bool {
        return valid(c) && tset.count(grid_.index(c)) != 0;
    };
    return flood_goals_reach_sources(grid_, seeds, is_t, ctx.enter, ctx.via_ok, bb);
}

void Board::ensure_avoid_baked() {
    if (avoid_baked_) return;
    avoid_baked_ = true;     // before rebuild: add_source_avoidance gates on it
    rebuild_avoidance();
}

void Board::rebuild_avoidance() {
    if (!avoid_baked_) return;
    active_self_net_ = -1;   // scratch describes contributions this wipes
    std::fill(pad_avoid_cost_.begin(), pad_avoid_cost_.end(), 0.0);
    std::fill(trace_avoid_cost_.begin(), trace_avoid_cost_.end(), 0.0);
    for (size_t i = 0; i < pad_owner_.size(); ++i) {
        if (pad_owner_[i] == 0) continue;
        Cell c = grid_.unindex(i);
        add_source_avoidance(c, pad_avoid_cost_, pad_sharp_);
    }
    for (size_t i = 0; i < trace_owner_.size(); ++i) {
        if (trace_owner_[i] == 0) continue;
        Cell c = grid_.unindex(i);
        add_source_avoidance(c, trace_avoid_cost_, trace_sharp_);
    }
    // Re-bake the targeted halo overlay so it stays consistent with the current
    // falloff radius / sharpness (a set_falloff_radius after seeding halos would
    // otherwise leave halo_cost_ baked at the old radius).
    if (!halo_sources_.empty()) {
        if (halo_cost_.empty()) halo_cost_.assign(grid_.size(), 0.0);
        else std::fill(halo_cost_.begin(), halo_cost_.end(), 0.0);
        for (const auto& hs : halo_sources_)
            add_source_avoidance(hs.second, halo_cost_, halo_sharp_);
    }
}

int Board::drc_clearance_cells() const {
    // Ceil, not truncate: the rule is a MINIMUM. Plain (int) turned 0.2mm/0.05mm
    // (= 3.999999... in doubles) into 3 cells — the whole model quietly routed at
    // 0.15mm effective clearance while our DRC (same truncation) agreed with it;
    // kicad-cli caught it. The 1e-9 slack keeps exact ratios from over-rounding.
    return std::max(1, (int)std::ceil(drc_clearance_ / grid_.resolution() - 1e-9));
}

void Board::clear_routing() {
    prepare_pad_keepout(); // route-side pad inflation (8.4c)
    // Reset traces: no traces yet, full occupancy = pads only.
    std::fill(trace_owner_.begin(), trace_owner_.end(), 0);
    std::fill(copper_.begin(), copper_.end(), 0);
    if (avoid_baked_)
        std::fill(trace_avoid_cost_.begin(), trace_avoid_cost_.end(), 0.0);
    owner_ = pad_owner_;
    // Drop the previous episode's paths BEFORE rebuilding zones: rebuild_cl_zone derives
    // via zones from net.segments, and stale segments would bake phantom clearance.
    for (Net& net : nets_) {
        net.segments.clear();
        net.total_length = 0.0;
        net.committed_cells.clear();
        net.pin_connected.assign(net.pins.size(), 0);
        if (!net.pins.empty()) net.pin_connected[0] = 1;   // the seed is connected
        net.unconnected_pins = std::max(0, (int)net.pins.size() - 1); // unattempted
        net.routed = (net.unconnected_pins == 0);          // 0/1-pin nets are trivial
    }
    active_self_net_ = -1;
    rebuild_cl_zone(); // pads only (no traces yet)
}

std::vector<int> Board::unconnected_pin_indices(int position) const {
    std::vector<int> out;
    if (position < 0 || position >= (int)nets_.size()) return out;
    const Net& net = nets_[(size_t)position];
    for (size_t i = 0; i < net.pins.size(); ++i) {
        const bool connected = i < net.pin_connected.size() ? net.pin_connected[i] != 0
                                                            : i == 0; // uninitialized = unattempted
        if (!connected) out.push_back((int)i);
    }
    return out;
}

Board::ConnectResult Board::route_one_connection(int position, int pin_idx) {
    ConnectResult r;
    const int n = (int)nets_.size();
    if (position < 0 || position >= n) return r;
    Net& net = nets_[(size_t)position];
    if (net.pin_connected.size() != net.pins.size()) {
        net.pin_connected.assign(net.pins.size(), 0);
        if (!net.pins.empty()) net.pin_connected[0] = 1;
        net.unconnected_pins = std::max(0, (int)net.pins.size() - 1);
    }
    r.unconnected_pins = net.unconnected_pins;
    if (net.unconnected_pins == 0) {
        net.routed = true;
        r.ok = true;   // nothing left to do
        return r;
    }

    // Goal set: one chosen pin, or every unconnected pin (multi-source A* then reaches
    // the NEAREST one first — the exact tie-breaking of net-mode tree growth).
    std::vector<Cell> goals;
    if (pin_idx >= 0) {
        if (pin_idx >= (int)net.pins.size() || net.pin_connected[(size_t)pin_idx]) return r;
        goals.push_back(net.pins[(size_t)pin_idx]);
    } else {
        for (size_t i = 0; i < net.pins.size(); ++i)
            if (!net.pin_connected[i]) goals.push_back(net.pins[i]);
    }

    // Source component in tree growth's EXACT order — seed pin, then committed path
    // cells in commit order. Order matters: A* seeds the open heap from this list, so
    // a different order changes tie-breaking and would break bit-equivalence with
    // net-mode routing. Pins connected later are path cells, so they are included.
    std::vector<Cell> sources;
    if (!net.pins.empty()) sources.push_back(net.pins[0]);
    sources.insert(sources.end(), net.committed_cells.begin(), net.committed_cells.end());

    NetContext ctx = make_net_context(position);
    const RouteDeadline dl = route_time_budget_s_ > 0.0
        ? std::chrono::steady_clock::now()
              + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(route_time_budget_s_))
        : RouteDeadline::max();
    if (unsat_check_enabled()) {
        std::unordered_set<size_t> src_idx;
        src_idx.reserve(sources.size() * 2);
        for (const Cell& c : sources)
            if (grid_.valid(c)) src_idx.insert(grid_.index(c));
        auto in_src = [&](const Cell& c) -> bool {
            return grid_.valid(c) && src_idx.count(grid_.index(c)) != 0;
        };
        int bb[4] = {INT_MAX, INT_MAX, INT_MIN, INT_MIN};
        for (const Cell& c : sources) {
            if (!grid_.valid(c)) continue;
            bb[0] = std::min(bb[0], c.x); bb[1] = std::min(bb[1], c.y);
            bb[2] = std::max(bb[2], c.x); bb[3] = std::max(bb[3], c.y);
        }
        if (!flood_goals_reach_sources(grid_, goals, in_src, ctx.enter, ctx.via_ok, bb))
            return r;           // provably unreachable — identical outcome, no search
    }
    auto res = astar_route_multi(grid_, sources, goals, via_cost_, base_cost_,
                                 ctx.enter, ctx.via_ok, dl);
    r.time_budget_exceeded = deadline_passed(dl);
    if (!res.found) return r;   // pin(s) unreachable right now — board unchanged

    const double len = path_euclidean_length(res.path);
    net.total_length += len;
    net.segments.push_back(res.path);
    commit_path(net, net.segments.back());
    net.committed_cells.insert(net.committed_cells.end(),
                               res.path.begin(), res.path.end());
    // The reached goal — and any other unconnected pin the path happens to cross —
    // is now connected (same as tree growth's prune_remaining).
    for (const Cell& c : res.path)
        for (size_t i = 0; i < net.pins.size(); ++i)
            if (!net.pin_connected[i] && net.pins[i] == c) net.pin_connected[i] = 1;
    int unconnected = 0;
    for (uint8_t f : net.pin_connected)
        if (!f) unconnected++;
    net.unconnected_pins = unconnected;
    net.routed = (unconnected == 0);
    r.ok = true;
    r.progress = true;
    r.added_length = len;
    r.unconnected_pins = unconnected;
    return r;
}

RouteStats Board::collect_stats(bool with_drc) const {
    RouteStats stats;
    const int n = (int)nets_.size();
    stats.per_net_length.assign((size_t)n, 0.0);
    stats.unrouted.assign((size_t)n, false);
    stats.per_net_unconnected.assign((size_t)n, 0);
    for (int k = 0; k < n; ++k) {
        const Net& net = nets_[(size_t)k];
        stats.per_net_length[(size_t)k] = net.total_length;
        stats.total_length += net.total_length;
        stats.total_vias += count_net_vias(net);
        stats.per_net_unconnected[(size_t)k] = net.unconnected_pins;
        stats.total_unconnected_pins += net.unconnected_pins;
        if (!net.routed) { stats.unrouted[(size_t)k] = true; stats.unrouted_count++; }
    }
    if (with_drc) {
        stats.drc_violations = check_drc().drc_violations;
    } else {
        // Exact mid-episode substitute: hard clearance makes copper violations
        // impossible, so a DRC pass can only ever report the unrouted count.
        stats.drc_violations = stats.unrouted_count;
    }
    return stats;
}

RouteStats Board::route_all() {
    route_stop_flag().store(false);
    RouteStats stats;
    int n = (int)nets_.size();
    stats.per_net_length.assign((size_t)n, 0.0);
    stats.unrouted.assign((size_t)n, false);
    stats.per_net_unconnected.assign((size_t)n, 0);
    clear_routing();

    const RouteDeadline dl = route_time_budget_s_ > 0.0
        ? std::chrono::steady_clock::now()
              + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(route_time_budget_s_))
        : RouteDeadline::max();
    route_nets_from(0, stats, dl);
    stats.time_budget_exceeded = deadline_passed(dl);

    RouteStats drc = check_drc();
    stats.drc_violations = drc.drc_violations;
    return stats;
}

RouteStats Board::route_from(int lo) {
    route_stop_flag().store(false);
    RouteStats stats;
    prepare_pad_keepout(); // route-side pad inflation (8.4c)
    int n = (int)nets_.size();
    if (lo < 0) lo = 0;
    if (lo > n) lo = n;
    stats.per_net_length.assign((size_t)n, 0.0);
    stats.unrouted.assign((size_t)n, false);
    stats.per_net_unconnected.assign((size_t)n, 0);

    // Nets [0, lo) keep their (already routed) per-net lengths where they are < lo.
    // We re-route only [lo, n), so recompute those; preserve [0,lo) values. Their via
    // transitions are also preserved (count them from the already-routed segments, which
    // are unchanged for the prefix) so total_vias matches a fresh full route.
    for (int k = 0; k < lo && k < n; ++k) {
        const Net& net = nets_[(size_t)k];
        stats.per_net_length[(size_t)k] = net.total_length;
        stats.total_length += net.total_length;
        stats.total_vias += count_net_vias(net);
        stats.per_net_unconnected[(size_t)k] = net.unconnected_pins;
        stats.total_unconnected_pins += net.unconnected_pins;
        if (!net.routed) { stats.unrouted[(size_t)k] = true; stats.unrouted_count++; }
    }

    // Rip up traces of nets at positions [lo, n). Ownership tokens are net.id+1.
    // Their segments must go too: rebuild_cl_zone below derives via zones from
    // net.segments, and a ripped net's stale paths would bake phantom clearance.
    {
        std::vector<int32_t> rip_tokens;
        rip_tokens.reserve((size_t)(n - lo));
        for (int k = lo; k < n; ++k) {
            Net& net = nets_[(size_t)k];
            rip_tokens.push_back((int32_t)net.id + 1);
            net.segments.clear();
            net.total_length = 0.0;
            net.routed = false;
            net.unconnected_pins = std::max(0, (int)net.pins.size() - 1); // unattempted
        }
        for (size_t i = 0; i < trace_owner_.size(); ++i) {
            int32_t t = trace_owner_[i];
            if (t == 0) continue;
            bool found = false;
            for (int32_t rt : rip_tokens) if (rt == t) { found = true; break; }
            if (found) {
                trace_owner_[i] = 0;
                copper_[i] = 0;
                owner_[i] = pad_owner_[i]; // restore pad occupancy (or 0 if none)
            }
        }
    }

    // Rebuild trace-avoidance from the surviving [0, lo) traces. The self-scratch
    // may describe a net whose contributions were just ripped: invalidate it.
    active_self_net_ = -1;
    if (avoid_baked_) {
        std::fill(trace_avoid_cost_.begin(), trace_avoid_cost_.end(), 0.0);
        for (size_t i = 0; i < trace_owner_.size(); ++i) {
            if (trace_owner_[i] != 0) {
                Cell c = grid_.unindex(i);
                add_source_avoidance(c, trace_avoid_cost_, trace_sharp_);
            }
        }
    }

    rebuild_cl_zone(); // cl_zone_ from the preserved (pads + [0,lo) copper) prefix

    const RouteDeadline dl = route_time_budget_s_ > 0.0
        ? std::chrono::steady_clock::now()
              + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(route_time_budget_s_))
        : RouteDeadline::max();
    route_nets_from(lo, stats, dl);
    stats.time_budget_exceeded = deadline_passed(dl);

    RouteStats drc = check_drc();
    stats.drc_violations = drc.drc_violations;
    return stats;
}

void Board::ensure_self_scratch(int nid) {
    if (active_self_net_ == nid && self_trace_scratch_.size() == grid_.size()) return;
    self_trace_scratch_.assign(grid_.size(), 0.0);
    active_self_net_ = nid;
    // Rebuild from every cell this net has CLAIMED (trace_owner_ == tok): avoidance is
    // baked exactly once per claimed cell (path, coat swath, via barrel), so summing
    // falloff over claimed cells reproduces the net's own contribution exactly.
    const int32_t tok = (int32_t)nid + 1;
    for (size_t i = 0; i < trace_owner_.size(); ++i) {
        if (trace_owner_[i] != tok) continue;
        add_source_avoidance(grid_.unindex(i), self_trace_scratch_, trace_sharp_);
    }
}

void Board::bake_congestion_rudy() {
    const int W = grid_.width(), H = grid_.height();
    congest_cost_.assign((size_t)W * (size_t)H, 0.0);
    congest_edges_.clear();
    for (const Net& net : nets_) {
        const auto& pins = net.pins;
        const size_t P = pins.size();
        if (P < 2) continue;
        // Prim MST over pin (x,y) with octile distance — the flight-line decomposition
        // of a multi-pin net into two-pin segments.
        std::vector<bool> in_tree(P, false);
        std::vector<double> best(P, 1e18);
        std::vector<int> from(P, 0);
        in_tree[0] = true;
        for (size_t i = 1; i < P; ++i) {
            double dx = std::abs(pins[i].x - pins[0].x), dy = std::abs(pins[i].y - pins[0].y);
            best[i] = std::max(dx, dy) + (DIAG_FACTOR - 1.0) * std::min(dx, dy);
        }
        for (size_t added = 1; added < P; ++added) {
            int b = -1;
            for (size_t i = 0; i < P; ++i)
                if (!in_tree[i] && (b < 0 || best[i] < best[(size_t)b])) b = (int)i;
            if (b < 0) break;
            in_tree[(size_t)b] = true;
            // smear this MST edge's RUDY density over its bbox
            const Cell& a = pins[(size_t)from[(size_t)b]];
            const Cell& c = pins[(size_t)b];
            const int x0 = std::min(a.x, c.x), x1 = std::max(a.x, c.x);
            const int y0 = std::min(a.y, c.y), y1 = std::max(a.y, c.y);
            const double w = (double)(x1 - x0 + 1), h = (double)(y1 - y0 + 1);
            const double d = (w + h) / (w * h);
            for (int y = std::max(0, y0); y <= std::min(H - 1, y1); ++y)
                for (int x = std::max(0, x0); x <= std::min(W - 1, x1); ++x)
                    congest_cost_[(size_t)y * (size_t)W + (size_t)x] += d;
            if ((size_t)net.id >= congest_edges_.size())
                congest_edges_.resize((size_t)net.id + 1);
            congest_edges_[(size_t)net.id].push_back({x0, y0, x1, y1});
            for (size_t i = 0; i < P; ++i) {
                if (in_tree[i]) continue;
                double dx = std::abs(pins[i].x - c.x), dy = std::abs(pins[i].y - c.y);
                double nd = std::max(dx, dy) + (DIAG_FACTOR - 1.0) * std::min(dx, dy);
                if (nd < best[i]) { best[i] = nd; from[i] = b; }
            }
        }
    }
    double mx = 0.0;
    for (double v : congest_cost_) mx = std::max(mx, v);
    congest_norm_ = mx > 0.0 ? mx : 1.0;
    // Re-accumulate the normalized field from per-edge QUANTIZED contributions —
    // term-for-term identical to ensure_self_congest, so (field - self_scratch)
    // is exact and the congestion self-exemption identity holds to the bit.
    std::fill(congest_cost_.begin(), congest_cost_.end(), 0.0);
    for (const auto& per_net : congest_edges_) {
        for (const auto& e : per_net) {
            const int x0 = e[0], y0 = e[1], x1 = e[2], y1 = e[3];
            const double w = (double)(x1 - x0 + 1), h = (double)(y1 - y0 + 1);
            const double d = quantize_cost(((w + h) / (w * h)) / congest_norm_);
            for (int y = std::max(0, y0); y <= std::min(H - 1, y1); ++y)
                for (int x = std::max(0, x0); x <= std::min(W - 1, x1); ++x)
                    congest_cost_[(size_t)y * (size_t)W + (size_t)x] += d;
        }
    }
    active_congest_net_ = -1;
}

void Board::ensure_self_congest(int nid) {
    if (active_congest_net_ == nid &&
        self_congest_scratch_.size() == congest_cost_.size()) return;
    const int W = grid_.width(), H = grid_.height();
    self_congest_scratch_.assign((size_t)W * (size_t)H, 0.0);
    active_congest_net_ = nid;
    for (const Net& net : nets_) {
        if (net.id != nid) continue;
        if ((size_t)nid < congest_edges_.size()) {
            for (const auto& e : congest_edges_[(size_t)nid]) {
                const int x0 = e[0], y0 = e[1], x1 = e[2], y1 = e[3];
                const double w = (double)(x1 - x0 + 1), h = (double)(y1 - y0 + 1);
                const double d = quantize_cost(((w + h) / (w * h)) / congest_norm_);
                for (int y = std::max(0, y0); y <= std::min(H - 1, y1); ++y)
                    for (int x = std::max(0, x0); x <= std::min(W - 1, x1); ++x)
                        self_congest_scratch_[(size_t)y * (size_t)W + (size_t)x] += d;
            }
        }
        break;
    }
}

Board::NetContext Board::make_net_context(int k) {
    Net& net = nets_[(size_t)k];
    const int nid = net.id; // stable identity; ownership tokens use nid+1
    NetAvoidance av;
    if ((size_t)k < avoid_.size()) av = avoid_[(size_t)k];
    av.pad_avoid_mult = std::max(0.0, av.pad_avoid_mult);
    av.trace_avoid_mult = std::max(0.0, av.trace_avoid_mult);
    // defensive: any nonzero multiplier requires baked fields (normally done
    // in set_avoidance; this covers direct avoid_ writers)
    if (av.pad_avoid_mult > 0.0 || av.trace_avoid_mult > 0.0)
        ensure_avoid_baked();

    // A net must NEVER repel itself (the entire goal is to unite its parts): the
    // scratch holds the active net's own baked contributions and is subtracted from
    // the shared field below. In net-mode routing a net has no committed copper while
    // it routes, so the scratch is all zeros and the arithmetic is bit-identical.
    ensure_self_scratch(nid);
    // Congestion self-exemption via per-context own-edge summation (identical
    // quantized per-edge terms to the field bake, so field-minus-self stays
    // exact) — replaces the shared self-scratch, making contexts thread-safe
    // for parallel-net routing (plan C3).
    struct OwnEdge { int x0, y0, x1, y1; double d; };
    auto own_edges = std::make_shared<std::vector<OwnEdge>>();
    if (congest_weight_ > 0.0 && !congest_cost_.empty() &&
        (size_t)nid < congest_edges_.size()) {
        for (const auto& e : congest_edges_[(size_t)nid]) {
            const double w = (double)(e[2] - e[0] + 1), h = (double)(e[3] - e[1] + 1);
            own_edges->push_back(
                {e[0], e[1], e[2], e[3],
                 quantize_cost(((w + h) / (w * h)) / congest_norm_)});
        }
    }
    // Big nets (bus fanouts): the per-cell own-edge scan is O(E) per enter_cost
    // CALL — 487-pin nets paid ~486 rect tests per A* expansion (measured: the
    // dominant cost on monster boards). Rasterize the edges once into a
    // self-demand plane (2D difference array + prefix sums). All terms are
    // exact quantized lattice values, so double addition is associative here
    // and the plane is BIT-IDENTICAL to the per-cell loop. Small nets keep the
    // loop (plane memory ~8B/cell is only worth it when E is large).
    std::shared_ptr<std::vector<double>> self_plane;
    if (own_edges->size() >= 16) {
        const int W = grid_.width(), H = grid_.height();
        self_plane = std::make_shared<std::vector<double>>((size_t)W * H, 0.0);
        auto& P = *self_plane;
        for (const auto& e : *own_edges) {
            P[(size_t)e.y0 * W + e.x0] += e.d;
            if (e.x1 + 1 < W) P[(size_t)e.y0 * W + (e.x1 + 1)] -= e.d;
            if (e.y1 + 1 < H) P[(size_t)(e.y1 + 1) * W + e.x0] -= e.d;
            if (e.x1 + 1 < W && e.y1 + 1 < H)
                P[(size_t)(e.y1 + 1) * W + (e.x1 + 1)] += e.d;
        }
        for (int y = 0; y < H; ++y) {
            double* row = P.data() + (size_t)y * W;
            for (int x = 1; x < W; ++x) row[x] += row[x - 1];
        }
        for (int y = 1; y < H; ++y) {
            double* row = P.data() + (size_t)y * W;
            const double* prev = row - W;
            for (int x = 0; x < W; ++x) row[x] += prev[x];
        }
    }

    NetContext ctx;
    // Per-net-pair physics: this net's tier decides which zone / pad-keepout
    // masks gate its entry (tier 0 == the legacy masks).
    const int32_t* zone_mask = zone_data_for((int32_t)nid + 1);
    const int32_t* padk_mask = pad_infl_data_for((int32_t)nid + 1);
    ctx.enter = [this, nid, av, own_edges, self_plane, zone_mask,
                 padk_mask](const Cell& c) -> double {
        if (!valid(c)) return -1.0;
        size_t i = grid_.index(c);
        int32_t tok = (int32_t)nid + 1;
        int32_t o = owner_[i];
        if (o != 0 && o != tok) return -1.0; // blocked by another net/pad/trace
        // Stage 16: HARD clearance. A net must never drop its real copper inside another
        // net's clearance zone (edge-to-edge >= clearance), even where ownership swaths
        // are truncated under congestion.
        int32_t cz = zone_mask[i];
        if (cz != 0 && cz != tok) return -1.0;
        // Stage 8.4c: keep clear of foreign pads by (trace width + clearance). A cell
        // whose pad_inflated_ token is foreign (or a multi-net conflict) is blocked;
        // a net's OWN pads (token match) are exempt so it can reach them (and its exact
        // own-pad cell stays enterable even when a foreign pad's inflation overlaps it).
        int32_t pi2 = padk_mask[i];
        if (pi2 != 0 && pi2 != tok && pad_owner_[i] != tok) return -1.0;

        double cost = base_cost_;
        if (av.pad_avoid_mult > 0.0) {
            double pad_avoid = pad_avoid_cost_[i] - self_pad_cost(c, nid);
            if (pad_avoid < 0.0) pad_avoid = 0.0;
            cost += av.pad_avoid_mult * pad_avoid;
        }
        // Targeted repulsion halo (add_repel_halo): ALWAYS on (no per-net mult)
        // but self-exempt for the owning net, so blocked pads shove every OTHER
        // net aside while their own net still reaches them. Empty = no overhead.
        if (!halo_cost_.empty()) {
            double hv = halo_cost_[i] - self_halo_cost(c, nid);
            if (hv < 0.0) hv = 0.0;
            cost += hv;
        }
        if (av.trace_avoid_mult > 0.0) {
            // Self-exemption: subtract the net's OWN baked contribution.
            double tv = trace_avoid_cost_[i] - self_trace_scratch_[i];
            if (tv < 0.0) tv = 0.0;
            cost += av.trace_avoid_mult * tv;
        }
        // Congestion prior: softly steer every net away from predicted-demand
        // hotspots (shared field; 0 weight = disabled). SELF-EXEMPT: a net never
        // pays for its own predicted demand — its flight-line bbox is exactly
        // where it must route (same principle as pad/trace avoidance).
        if (congest_weight_ > 0.0 && !congest_cost_.empty()) {
            const size_t ci = (size_t)c.y * (size_t)grid_.width() + (size_t)c.x;
            double self = 0.0;
            if (self_plane) {
                self = (*self_plane)[ci];
            } else {
                for (const auto& e : *own_edges)
                    if (c.x >= e.x0 && c.x <= e.x1 && c.y >= e.y0 && c.y <= e.y1)
                        self += e.d;
            }
            double v = congest_cost_[ci] - self;
            if (v > 0.0) {
                // gamma==2 as exact dyadic square; other gammas quantize the
                // libm pow result so cross-libm ulp variance rounds away
                if (congest_gamma_ == 2.0) v = v * v;
                else if (congest_gamma_ != 1.0)
                    v = quantize_cost(std::pow(v, congest_gamma_));
                cost += congest_weight_ * base_cost_ * v;
            }
        }
        return cost;
    };

    // Stage 8.4/16 via-emergence: a via is only created where it stays physically clear
    // of other nets' features. Via-pad spacing is a single lookup into via_pad_zone_
    // (the pads' real copper dilated by vr + tw + clr, which both enforces physical
    // via-pad clearance and keeps a via's own hard zone from sealing a pad's
    // approach). Remaining scans over the per-type masks (self token exempt):
    //   via-trace : vr + tw + clr      via-via : 2*vr + clr
    if (via_radius_cells_ > 0.0) {
        const double clr = max_clr_cells();   // via machinery is not pair-aware:
        const double vzone = via_radius_cells_ + max_hw_cells() + clr;   // max = safe
        const int r_keep = (int)std::ceil(via_radius_cells_);
        const int r_trc = (int)std::ceil(vzone);
        const int r_via = (int)std::ceil(2.0 * via_radius_cells_ + clr);
        // Hole-to-hole (KiCad board constraint, 0.25mm default): applies to SAME-net
        // vias too — a drill is a drill. With the typical drill = dia/2, required
        // center distance 2*drill_r + 0.25 == via_radius + 0.25mm.
        const int r_hh = (int)std::ceil(via_radius_cells_ +
                                        0.25 / grid_.resolution());
        const int R = std::max({r_via, r_trc, r_hh}); // outer square bound
        ctx.via_ok = [this, nid, r_keep, r_trc, r_via, r_hh, R](const Cell& e) -> bool {
            const int32_t tok = (int32_t)nid + 1;
            const int w = grid_.width(), h = grid_.height();
            // The net's own thru-hole pad barrel is pre-drilled copper, not a via
            // placement: never veto it (foreign features nearby are the FOREIGN
            // object's spacing problem, checked when that object was placed).
            if (thru_pad_col_[(size_t)e.y * (size_t)w + (size_t)e.x] == tok) return true;
            // via-pad: one lookup into the shape-true dilated mask (all layers of the
            // barrel column must be clear of foreign pad keepout).
            for (int l = 0; l < grid_.layers(); ++l) {
                int32_t pz = via_pad_zone_[grid_.index(l, e.x, e.y)];
                if (pz != 0 && pz != tok) return false;
            }
            const int x0 = std::max(0, e.x - R), x1 = std::min(w - 1, e.x + R);
            const int y0 = std::max(0, e.y - R), y1 = std::min(h - 1, e.y + R);
            for (int l = 0; l < grid_.layers(); ++l)
                for (int y = y0; y <= y1; ++y)
                    for (int x = x0; x <= x1; ++x) {
                        const size_t idx = grid_.index(l, x, y);
                        const int cd = std::max(std::abs(x - e.x), std::abs(y - e.y));
                        // keepout: any foreign conductor (including a net's coating)
                        if (cd <= r_keep) {
                            int32_t o = owner_[idx];
                            if (o != 0 && o != tok) return false;
                        }
                        // via-trace (real copper)
                        if (cd <= r_trc && copper_[idx]) {
                            int32_t t = trace_owner_[idx];
                            if (t != 0 && t != tok) return false;
                        }
                        // via-via (foreign)
                        if (cd <= r_via) {
                            int32_t v = via_owner_[idx];
                            if (v != 0 && v != tok) return false;
                        }
                        // hole-to-hole: OWN vias too (cd 0 = reusing the same
                        // site, legal; anything nearer than the drill rule not)
                        if (cd > 0 && cd <= r_hh && via_owner_[idx] == tok)
                            return false;
                    }
            return true;
        };
    }
    return ctx;
}

void Board::commit_path(Net& net, const std::vector<Cell>& seg) {
    const int32_t tok = (int32_t)net.id + 1;
    const double td_sharp = trace_sharp_;
    // Every cell whose avoidance this commit bakes, for the incremental self-scratch
    // update below (a net must never repel itself).
    std::vector<Cell> claimed;
    const Cell* prev = nullptr;
    for (const Cell& c : seg) {
        if (!valid(c)) continue;
        size_t i = grid_.index(c);
        if (pad_owner_[i] != 0 && pad_owner_[i] != tok) { prev = &c; continue; } // another net's pad
        // Real trace copper, set regardless of trace_owner_ (which coat() may already
        // have set on an adjacent path cell via self-coating). Pad cells are handled
        // by the separate pad-trace DRC (threshold clr+tw), so they are NOT marked
        // copper_ — otherwise the trace-trace rule would judge them at the wrong
        // (2*tw+clr) threshold and false-positive.
        if (pad_owner_[i] == 0) copper_[i] = 1;
        if (trace_owner_[i] == 0) {
            trace_owner_[i] = tok;
            owner_[i] = tok;
            add_source_avoidance(c, trace_avoid_cost_, td_sharp);
            claimed.push_back(c);
        }
        // Stage 8.3: claim the trace's width swath. The keepout radius equals the
        // DRC pair spacing (owner hw + tier hw + max clearances) so placement
        // and DRC agree; tier 0 reduces to the legacy 2*tw + clr.
        if (trace_hw_cells_ > 0.0) {
            const int ot = tier_of_tok(tok);
            const double hw_o = tier_hw(ot), c_o = tier_clr(ot);
            coat(c, c.layer, c.layer, pair_zone_radius(hw_o, c_o, 0), tok, &claimed);
            for (size_t t = 0; t < extra_tiers_.size(); ++t)
                set_zone_mask(extra_tiers_[t].zone, c, c.layer, c.layer,
                              pair_zone_radius(hw_o, c_o, (int)t + 1), tok);
        }
        // A layer change riding the net's own thru-hole pad barrel is pre-drilled
        // pad copper, not a placed via: no via_owner_, no via keepout/zone (the
        // pad's own keepout already covers its physical extent).
        if (prev && prev->layer != c.layer && !own_pad_barrel(*prev, c, tok)) {
            // via barrel occupies (c.x, c.y) on EVERY layer
            for (int l = 0; l < grid_.layers(); ++l)
                via_owner_[grid_.index(l, c.x, c.y)] = tok;
            // Claim barrel ownership on the intermediate layers (the endpoint
            // layers are already path cells).
            for (int l = 0; l < grid_.layers(); ++l) {
                if (l == prev->layer || l == c.layer) continue; // already in path
                Cell vc{l, c.x, c.y};
                if (!valid(vc)) continue;
                size_t vi = grid_.index(vc);
                if (pad_owner_[vi] != 0 && pad_owner_[vi] != tok) continue;
                if (trace_owner_[vi] == 0) {
                    trace_owner_[vi] = tok;
                    owner_[vi] = tok;
                    // (via barrels are NOT copper_ for trace-trace DRC: via spacing is
                    // enforced by the via keepout coating + via-emergence check below,
                    // avoiding a via-vs-trace formula conflict.)
                    // Keep the soft avoidance field consistent with route_from's
                    // rebuild (which bakes prefix via cells), so full == incremental.
                    add_source_avoidance(vc, trace_avoid_cost_, td_sharp);
                    claimed.push_back(vc);
                }
            }
            // Keepout + hard clearance for the whole barrel, ONCE per transition.
            // coat/set_zone span [0, layers) themselves, so they must NOT sit inside
            // the intermediate-layer loop above: that loop never runs on a 2-layer
            // board, which left vias with no clearance zone during the routing pass
            // (only rebuild_cl_zone wrote it, after the fact).
            if (via_radius_cells_ > 0.0) {
                Cell vc{c.layer, c.x, c.y};
                // Stage 8.2: claim the via keepout disc on every layer.
                coat(vc, 0, grid_.layers() - 1, via_radius_cells_, tok, &claimed);
                // Stage 16: hard via clearance zone so a foreign trace's edge
                // keeps >= max(pair clearances) from the via edge, per tier.
                {
                    const int ot = tier_of_tok(tok);
                    const double c_o = tier_clr(ot);
                    set_zone(vc, 0, grid_.layers() - 1,
                             via_radius_cells_ + tier_hw(0) +
                                 std::max(c_o, tier_clr(0)), tok);
                    for (size_t t = 0; t < extra_tiers_.size(); ++t)
                        set_zone_mask(extra_tiers_[t].zone, vc, 0,
                                      grid_.layers() - 1,
                                      via_radius_cells_ + tier_hw((int)t + 1) +
                                          std::max(c_o, tier_clr((int)t + 1)),
                                      tok);
                }
            }
        }
        prev = &c;
    }
    // Keep the self-avoidance scratch exact WITHOUT an O(grid) rebuild: when this
    // net is the active scratch net (always true in connection mode, where context
    // is built immediately before commit), bake the newly claimed cells into the
    // scratch incrementally. For any other committing net, invalidate.
    if (net.id == active_self_net_ && self_trace_scratch_.size() == grid_.size()) {
        for (const Cell& cc : claimed)
            add_source_avoidance(cc, self_trace_scratch_, td_sharp);
    } else {
        active_self_net_ = -1;
    }
}

namespace {
// Flight-line decomposition for the crossing test: octile Prim MST edges over
// a net's pins, as (x1,y1,x2,y2) in cell coordinates.
std::vector<std::array<int, 4>> flight_segs(const Net& net) {
    std::vector<std::array<int, 4>> out;
    const auto& pins = net.pins;
    const size_t P = pins.size();
    if (P < 2) return out;
    std::vector<bool> in_tree(P, false);
    std::vector<double> best(P, 1e18);
    std::vector<int> from(P, 0);
    in_tree[0] = true;
    auto octile = [&](size_t a, size_t b) {
        const double dx = std::abs(pins[a].x - pins[b].x);
        const double dy = std::abs(pins[a].y - pins[b].y);
        return std::max(dx, dy) + (DIAG_FACTOR - 1.0) * std::min(dx, dy);
    };
    for (size_t i = 1; i < P; ++i) best[i] = octile(i, 0);
    for (size_t added = 1; added < P; ++added) {
        int b = -1;
        for (size_t i = 0; i < P; ++i)
            if (!in_tree[i] && (b < 0 || best[i] < best[(size_t)b])) b = (int)i;
        if (b < 0) break;
        in_tree[(size_t)b] = true;
        out.push_back({pins[(size_t)from[(size_t)b]].x, pins[(size_t)from[(size_t)b]].y,
                       pins[(size_t)b].x, pins[(size_t)b].y});
        for (size_t i = 0; i < P; ++i)
            if (!in_tree[i]) {
                const double d = octile(i, (size_t)b);
                if (d < best[i]) { best[i] = d; from[i] = b; }
            }
    }
    return out;
}
bool segs_cross(const std::array<int, 4>& a, const std::array<int, 4>& b) {
    auto ccw = [](double px, double py, double qx, double qy, double rx, double ry) {
        return (ry - py) * (qx - px) > (qy - py) * (rx - px);
    };
    return ccw(a[0], a[1], b[0], b[1], b[2], b[3]) !=
               ccw(a[2], a[3], b[0], b[1], b[2], b[3]) &&
           ccw(a[0], a[1], a[2], a[3], b[0], b[1]) !=
               ccw(a[0], a[1], a[2], a[3], b[2], b[3]);
}
} // namespace

void Board::route_nets_from_parallel(int lo, RouteStats& stats,
                                     RouteDeadline deadline) {
    const int n = (int)nets_.size();
    const int B = parallel_nets_;
    // any nonzero avoidance multiplier: bake ONCE serially so parallel
    // contexts are pure readers
    for (const auto& a : avoid_)
        if (a.pad_avoid_mult > 0.0 || a.trace_avoid_mult > 0.0) {
            ensure_avoid_baked();
            break;
        }

    // per-net bookkeeping identical to the serial loop
    auto account_and_commit = [&](int k, bool ok) {
        Net& net = nets_[(size_t)k];
        stats.per_net_length[(size_t)k] = net.total_length;
        stats.total_length += net.total_length;
        if (!ok || !net.routed) {
            if (deadline_passed(deadline) && stats.resume_from < 0)
                stats.resume_from = k;
            stats.unrouted[(size_t)k] = true;
            stats.unrouted_count++;
            net.routed = false;
        }
        stats.per_net_unconnected[(size_t)k] = net.unconnected_pins;
        stats.total_unconnected_pins += net.unconnected_pins;
        stats.total_vias += count_net_vias(net);
        for (const auto& seg : net.segments) commit_path(net, seg);
        route_progress_done().fetch_add(1);
    };
    auto mark_deadline_skip = [&](int k) {
        Net& net = nets_[(size_t)k];
        if (stats.resume_from < 0) stats.resume_from = k;
        net.segments.clear();
        net.total_length = 0.0;
        net.routed = false;
        net.unconnected_pins = std::max(0, (int)net.pins.size() - 1);
        stats.unrouted[(size_t)k] = true;
        stats.unrouted_count++;
        stats.per_net_unconnected[(size_t)k] = net.unconnected_pins;
        stats.total_unconnected_pins += net.unconnected_pins;
        route_progress_done().fetch_add(1);
    };
    auto route_serial_one = [&](int k) {
        Net& net = nets_[(size_t)k];
        net.segments.clear();
        net.total_length = 0.0;
        bool ok;
        const int fr = field_router_enabled() ? route_net_field(k, deadline)
                                              : -1;
        if (fr >= 0) {
            ok = (fr == 1);
        } else {
            NetContext ctx = make_net_context(k);
            WaveMapFn bw = [this, k](const std::vector<Cell>& comp)
                -> const uint16_t* { return wave_map(k, comp); };
            ok = tree_strategy_ == TreeStrategy::reverse
                     ? route_net_tree_reverse(grid_, net, via_cost_,
                                              base_cost_, ctx.enter,
                                              ctx.via_ok, deadline, bw,
                                              expansion_budget_)
                     : route_net_tree(grid_, net, via_cost_, base_cost_,
                                      ctx.enter, ctx.via_ok, deadline);
        }
        account_and_commit(k, ok);
    };
    // exact legality walk of a routed path set under the CURRENT board state
    auto still_legal = [&](const Net& net, const NetContext& ctx) -> bool {
        for (const auto& seg : net.segments)
            for (size_t i = 1; i < seg.size(); ++i) {
                const Cell& a = seg[i - 1];
                const Cell& b = seg[i];
                if (a.layer != b.layer) {
                    if (ctx.via_ok && !ctx.via_ok(b)) return false;
                    if (ctx.enter(b) < 0) return false;
                    continue;
                }
                const int dx = b.x - a.x, dy = b.y - a.y;
                if (dx != 0 && dy != 0) {
                    if (ctx.enter(Cell{a.layer, a.x + dx, a.y}) < 0) return false;
                    if (ctx.enter(Cell{a.layer, a.x, a.y + dy}) < 0) return false;
                }
                if (ctx.enter(b) < 0) return false;
            }
        return true;
    };

    int k = lo;
    while (k < n) {
        if (deadline_passed(deadline)) { mark_deadline_skip(k); ++k; continue; }
        // maximal consecutive non-crossing batch starting at k
        std::vector<int> batch{k};
        std::vector<std::vector<std::array<int, 4>>> fl;
        fl.push_back(flight_segs(nets_[(size_t)k]));
        int j = k + 1;
        while (j < n && (int)batch.size() < B) {
            auto fj = flight_segs(nets_[(size_t)j]);
            bool crosses = false;
            for (size_t m = 0; m < batch.size() && !crosses; ++m)
                for (const auto& sa : fl[m]) {
                    for (const auto& sb : fj)
                        if (segs_cross(sa, sb)) { crosses = true; break; }
                    if (crosses) break;
                }
            if (crosses) break;                 // prefix rule: stop at first crosser
            batch.push_back(j);
            fl.push_back(std::move(fj));
            ++j;
        }
        if (getenv("ROUTING_FIELD_DEBUG"))
            fprintf(stderr, "[batch] k=%d size=%zu\n", k, batch.size());
        if (batch.size() == 1) {
            route_serial_one(k);
            k = j;
            continue;
        }
        // parallel phase: route local copies against the frozen board
        std::vector<Net> locals((size_t)batch.size());
        std::vector<char> okv(batch.size(), 0);
#ifdef ROUTING_HAVE_OPENMP
#pragma omp parallel for schedule(dynamic) num_threads(B)
#endif
        for (int bi = 0; bi < (int)batch.size(); ++bi) {
            const int kk = batch[bi];
            Net local = nets_[(size_t)kk];
            local.segments.clear();
            local.total_length = 0.0;
            const int fr = field_router_enabled()
                               ? route_net_field_into(kk, local, deadline)
                               : -1;
            if (fr >= 0) {
                okv[bi] = (fr == 1) ? 1 : 0;
            } else {
                NetContext ctx = make_net_context(kk);
                WaveMapFn bw = [this, kk](const std::vector<Cell>& comp)
                    -> const uint16_t* { return wave_map(kk, comp); };
                okv[bi] = (tree_strategy_ == TreeStrategy::reverse
                               ? route_net_tree_reverse(
                                     grid_, local, via_cost_, base_cost_,
                                     ctx.enter, ctx.via_ok, deadline, bw,
                                     expansion_budget_)
                               : route_net_tree(grid_, local, via_cost_,
                                                base_cost_, ctx.enter,
                                                ctx.via_ok, deadline))
                              ? 1
                              : 0;
            }
            locals[bi] = std::move(local);
        }
        // commit phase: strict net order, exact revalidation, serial re-route
        // for any member an earlier commit invalidated
        for (size_t bi = 0; bi < batch.size(); ++bi) {
            const int kk = batch[bi];
            NetContext vctx = make_net_context(kk);
            if (okv[bi] && (bi == 0 || still_legal(locals[bi], vctx))) {
                nets_[(size_t)kk] = std::move(locals[bi]);
                account_and_commit(kk, true);
            } else {
                route_serial_one(kk);           // fresh state, normal path
            }
        }
        k = j;
    }
}

void Board::route_nets_from(int lo, RouteStats& stats, RouteDeadline deadline) {
    int n = (int)nets_.size();
    route_progress_done().store(0);
    route_progress_total().store(n - lo);
    if (getenv("ROUTING_FIELD_DEBUG")) {
        static std::atomic<int> once{0};
        if (once.fetch_add(1) < 3)
            fprintf(stderr, "[route] parallel_nets=%d strategy=%d field=%d\n",
                    parallel_nets_, (int)tree_strategy_,
                    (int)field_router_enabled());
    }
    if (parallel_nets_ > 1 && (tree_strategy_ == TreeStrategy::reverse ||
                               field_router_enabled())) {
        // C3 batching: native for reverse-strategy A*; the field router is
        // strategy-independent (its A* fallback respects the strategy below).
        route_nets_from_parallel(lo, stats, deadline);
        return;
    }

    for (int k = lo; k < n; ++k) {
        Net& net = nets_[(size_t)k];
        // Wall-time budget: once expired, stop starting nets — the remaining tail
        // reports unrouted and the already-committed prefix stands (partial result).
        if (deadline_passed(deadline)) {
            if (stats.resume_from < 0) stats.resume_from = k; // checkpoint: continue here
            net.segments.clear();
            net.total_length = 0.0;
            net.routed = false;
            net.unconnected_pins = std::max(0, (int)net.pins.size() - 1);
            stats.unrouted[(size_t)k] = true;
            stats.unrouted_count++;
            stats.per_net_unconnected[(size_t)k] = net.unconnected_pins;
            stats.total_unconnected_pins += net.unconnected_pins;
            route_progress_done().fetch_add(1);
            continue;
        }
        net.segments.clear();
        net.total_length = 0.0;
        bool ok;
        // Field router (stage 2, opt-in): single all-pins field + boundary
        // MST. -1 = backend unavailable/inconsistent -> normal A* path.
        const int fr = field_router_enabled() ? route_net_field(k, deadline)
                                              : -1;
        if (fr >= 0) {
            ok = (fr == 1);
        } else {
            NetContext ctx = make_net_context(k);
            WaveMapFn build_wave = [this, k](const std::vector<Cell>& comp)
                -> const uint16_t* { return wave_map(k, comp); };
            ok = (tree_strategy_ == TreeStrategy::reverse)
                     ? route_net_tree_reverse(grid_, net, via_cost_, base_cost_,
                                              ctx.enter, ctx.via_ok, deadline,
                                              build_wave, expansion_budget_)
                     : route_net_tree(grid_, net, via_cost_, base_cost_,
                                      ctx.enter, ctx.via_ok, deadline);
        }

        stats.per_net_length[(size_t)k] = net.total_length;
        stats.total_length += net.total_length;
        if (!ok || !net.routed) {
            // A failure with the deadline already expired is (conservatively) an
            // interruption, not proof of unroutability: resume retries this net.
            if (deadline_passed(deadline) && stats.resume_from < 0) stats.resume_from = k;
            stats.unrouted[(size_t)k] = true;
            stats.unrouted_count++;
            net.routed = false;
            // NO `continue`: a partial net's successful connections are real copper —
            // they commit below exactly like a routed net's (4-of-5 beats 0-of-5, and
            // committed partials keep segments == committed copper for zone rebuilds).
        }
        stats.per_net_unconnected[(size_t)k] = net.unconnected_pins;
        stats.total_unconnected_pins += net.unconnected_pins;

        stats.total_vias += count_net_vias(net);

        // Commit every returned path — PARTIAL nets included (their successful
        // connections are real copper). Shared with connection mode via commit_path().
        for (const auto& seg : net.segments) commit_path(net, seg);
        route_progress_done().fetch_add(1);
    }
}

int Board::count_static_pad_conflicts() const {
    // Rasterized shape-shape gap test: stamp every pad dilated by clearance/2 into a
    // scratch mask; a cell claimed by two different tokens means the copper-to-copper
    // gap there is below the clearance rule (within one cell of discretization).
    const double margin = 0.5 * (double)drc_clearance_cells();
    const int W = grid_.width(), H = grid_.height();
    std::vector<int32_t> mask((size_t)grid_.size(), 0);
    int conflicts = 0;
    for (const PadShape& ps : pad_shapes_) {
        if (ps.via_only) continue;   // via rule area: not copper
        if (ps.layer < 0 || ps.layer >= grid_.layers()) continue;
        // Rotation-safe bound: a rotated rect's corner reaches up to
        // hypot(half_w, half_h) from center — max(hw, hh) truncates the scan box
        // and leaves the tips of 45-degree pads unstamped (jackco thermal diamond).
        const double reach = std::hypot(ps.half_w, ps.half_h) + margin;
        const int x0 = std::max(0, (int)std::floor(ps.cx - reach));
        const int x1 = std::min(W - 1, (int)std::ceil(ps.cx + reach));
        const int y0 = std::max(0, (int)std::floor(ps.cy - reach));
        const int y1 = std::min(H - 1, (int)std::ceil(ps.cy + reach));
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                if (ps.distance((double)x, (double)y) >= margin) continue;
                size_t j = grid_.index(ps.layer, x, y);
                if (mask[j] == 0) mask[j] = ps.tok;
                else if (mask[j] != ps.tok) conflicts++;
            }
    }
    return conflicts;
}

RouteStats Board::check_drc() const {
    RouteStats s;
    size_t n = nets_.size();
    s.unrouted.assign(n, false);

    for (size_t i = 0; i < n; ++i) {
        if (!nets_[i].routed) {
            s.unrouted[i] = true;
            s.unrouted_count++;
            s.drc_violations++;
        }
    }

    const int clr = drc_clearance_cells();
    // Physical keepout is enforced by coating/via-emergence at routing; DRC backstops the
    // pad-to-trace rule. With no clearance and no trace width there is nothing to check.
    if (clr < 1 && trace_hw_cells_ <= 0.0) {
        return s;
    }

    // Pad-trace clearance runs on the REAL pad shapes (exact Minkowski threshold):
    // a foreign trace centerline within clr + tw of a pad's copper boundary violates.
    // Own pads are exempt (a trace touches its own pads at its endpoints). Shapes are
    // bucketed by layer with a cheap bounding-circle reject before the SDF call.
    // Per-net-pair thresholds: trace of tier i vs pad of tier p needs
    // max(c_p, c_i) + hw_i from the pad edge; single-class reduces to clr + tw.
    std::vector<std::vector<const PadShape*>> shapes_by_layer(grid_.layers());
    for (const PadShape& ps : pad_shapes_)
        if (!ps.via_only && ps.layer >= 0 && ps.layer < grid_.layers())
            shapes_by_layer[(size_t)ps.layer].push_back(&ps);
    auto pad_trace_margin = [this](int32_t trace_tok, int32_t pad_tok) {
        const int ti = tier_of_tok(trace_tok);
        const int tp = (pad_tok > 0 && pad_tok < INT32_MAX) ? tier_of_tok(pad_tok) : 0;
        return std::max(tier_clr(tp), tier_clr(ti)) + tier_hw(ti);
    };

    // Iterate REAL copper cells (copper_ flag, keyed by trace_owner_ token). This excludes
    // the Stage 8.2/8.3 keepout-coating margin and non-committed via-transition artifacts
    // (e.g. a via cell landing on a foreign through-hole pad), so overlap/clearance is
    // judged only against actual copper — no false positives.
    const bool check_tt = trace_hw_cells_ > 0.0; // trace-vs-trace with physics on
    // Scan box must cover the widest possible pair; the exact per-pair
    // threshold is applied inside (single-class: both equal 2*tw + clr).
    const int sr = (int)std::ceil(2.0 * max_hw_cells() + max_clr_cells());
    auto tt_spacing = [this](int32_t a, int32_t b2) {
        const int ta = tier_of_tok(a), tb = tier_of_tok(b2);
        return tier_hw(ta) + tier_hw(tb) + std::max(tier_clr(ta), tier_clr(tb));
    };
    const int W = grid_.width(), H = grid_.height();

    for (size_t i = 0; i < copper_.size(); ++i) {
        if (!copper_[i]) continue;
        int32_t tn = trace_owner_[i];
        if (tn == 0) continue;
        int32_t pn = pad_owner_[i];
        if (pn != 0 && pn != tn) {
            s.drc_violations++;
            if (std::getenv("DRC_DEBUG") && s.drc_violations <= 30) {
                Cell cc = grid_.unindex(i);
                fprintf(stderr, "[drc] OVERLAP l=%d x=%d y=%d trace_tok=%d pad_tok=%d\n",
                        cc.layer, cc.x, cc.y, tn, pn);
            }
            continue;
        } // overlap
        if (pn != 0) continue; // a (own) pad/barrel cell: pad-to-pad spacing is placement
        Cell c = grid_.unindex(i);
        // pad-trace clearance (within-layer, own pads allowed at trace endpoints)
        bool too_close = false;
        for (const PadShape* ps : shapes_by_layer[(size_t)c.layer]) {
            if (ps->tok == tn) continue;            // own pad at its endpoint is fine
            const double reach = std::max(ps->half_w, ps->half_h) +
                                 pad_trace_margin(tn, ps->tok);
            const double ddx = (double)c.x - ps->cx, ddy = (double)c.y - ps->cy;
            if (std::abs(ddx) > reach || std::abs(ddy) > reach) continue; // bbox reject
            if (ps->distance((double)c.x, (double)c.y) <
                pad_trace_margin(tn, ps->tok)) {
                too_close = true; break;
            }
        }
        if (too_close) {
            s.drc_violations++;
            if (std::getenv("DRC_DEBUG") && s.drc_violations <= 30)
                fprintf(stderr, "[drc] PAD-TRACE l=%d x=%d y=%d trace_tok=%d\n",
                        c.layer, c.x, c.y, tn);
            continue;
        }
        // trace-vs-trace side clearance (Stage 8.4b) — only when physical trace width is on
        if (check_tt && sr >= 1) {
            int x0 = std::max(0, c.x - sr), x1 = std::min(W - 1, c.x + sr);
            int y0 = std::max(0, c.y - sr), y1 = std::min(H - 1, c.y + sr);
            bool tt = false;
            for (int y = y0; y <= y1 && !tt; ++y)
                for (int x = x0; x <= x1; ++x) {
                    size_t j = grid_.index(c.layer, x, y);
                    if (j <= i || !copper_[j]) continue; // dedup unordered pairs + self
                    int32_t tn2 = trace_owner_[j];
                    if (tn2 == 0 || tn2 == tn) continue; // free or same net
                    Cell q = grid_.unindex(j);
                    if (euclid_dist((double)(q.x - c.x), (double)(q.y - c.y)) <
                        tt_spacing(tn, tn2)) {
                        if (std::getenv("DRC_DEBUG") && s.drc_violations < 30)
                            fprintf(stderr, "[drc-tt] l=%d (%d,%d)tok%d vs (%d,%d)tok%d d=%.2f\n",
                                    c.layer, c.x, c.y, tn, q.x, q.y, tn2,
                                    euclid_dist((double)(q.x - c.x), (double)(q.y - c.y)));
                        tt = true; break;
                    }
                }
            if (tt) {
                s.drc_violations++;
                if (std::getenv("DRC_DEBUG") && s.drc_violations <= 30)
                    fprintf(stderr, "[drc] TRACE-TRACE l=%d x=%d y=%d tok=%d\n",
                            c.layer, c.x, c.y, tn);
            }
        }
    }
    return s;
}

// ---- post-route trace smoothing -------------------------------------------
// A chord replacing a staircase is accepted only when every cell its copper
// (the hw disk swept along the centreline) covers is one of:
//   (a) a cell THIS net's original copper already occupied (own-footprint), or
//   (b) a genuinely free cell — outside every FOREIGN clearance zone and
//       foreign pad keepout, and not already reserved by another net's
//       smoothing this pass.
// (a) is what lets a straight run alongside a neighbour survive: at minimum
// spacing a trace's own copper edge lies on the neighbour's zone boundary, so a
// pure "avoid foreign zones" test would wrongly reject the net's own geometry.
// (b) keeps any NEW copper >= clearance from all foreign copper (zones are baked
// at foreign_hw + clearance) and, via the shared `reserved` mask, stops two
// nets corner-cutting into the same gap. No grid mutation; the original clean
// copper plus only-into-free moves can never raise a DRC violation.
int Board::smooth_paths(double max_dev_cells, double fillet_radius_cells) {
    smoothed_paths_.assign(nets_.size(), {});
    int removed = 0;
    const size_t NG = grid_.size();
    const int W = grid_.width(), H = grid_.height();
    std::vector<int32_t> own(NG, 0);        // this net's original footprint (by gen)
    std::vector<int32_t> reserved(NG, 0);   // cells claimed by accepted smoothing
    int32_t gen = 0;

    auto seg_dist = [](double px, double py, double ax, double ay,
                       double bx, double by) -> double {
        const double vx = bx - ax, vy = by - ay;
        const double L2 = vx * vx + vy * vy;
        double t = L2 > 0.0 ? ((px - ax) * vx + (py - ay) * vy) / L2 : 0.0;
        t = std::max(0.0, std::min(1.0, t));
        const double qx = ax + t * vx - px, qy = ay + t * vy - py;
        return std::sqrt(qx * qx + qy * qy);
    };

    for (size_t k = 0; k < nets_.size(); ++k) {
        const int32_t tok = (int32_t)nets_[k].id + 1;
        const double hw = std::max(0.5, tier_hw(tier_of_tok(tok)));
        const int hwi = (int)std::ceil(hw);
        const double hw2 = hw * hw;
        const int32_t* zone = zone_data_for(tok);
        const int32_t* padk = pad_infl_data_for(tok);
        auto& out = smoothed_paths_[k];

        // 1) stamp this net's original copper footprint (owned cells, hw disk)
        ++gen;
        std::vector<Cell> owned_cells;
        for (const auto& path : nets_[k].segments)
            for (const Cell& c : path) {
                if (!valid(c) || owner_[grid_.index(c)] != tok) continue;
                owned_cells.push_back(c);
                for (int dy = -hwi; dy <= hwi; ++dy) {
                    const int y = c.y + dy; if (y < 0 || y >= H) continue;
                    for (int dx = -hwi; dx <= hwi; ++dx) {
                        if (dx * dx + dy * dy > hwi * hwi) continue;
                        const int x = c.x + dx; if (x < 0 || x >= W) continue;
                        own[grid_.index(c.layer, x, y)] = gen;
                    }
                }
            }

        // per-cell test: covered cell is own-footprint, or free & unreserved
        auto disk_ok = [&](int layer, int cx, int cy) -> bool {
            for (int dy = -hwi; dy <= hwi; ++dy) {
                const int y = cy + dy; if (y < 0 || y >= H) return false;
                for (int dx = -hwi; dx <= hwi; ++dx) {
                    if ((double)(dx * dx + dy * dy) > hw2) continue;
                    const int x = cx + dx; if (x < 0 || x >= W) return false;
                    const size_t i = grid_.index(layer, x, y);
                    if (own[i] == gen) continue;                    // our own copper
                    if (zone[i] != 0 && zone[i] != tok) return false;   // foreign zone
                    if (padk[i] != 0 && padk[i] != tok && pad_owner_[i] != tok)
                        return false;                               // foreign pad
                    if (reserved[i] != 0 && reserved[i] != tok) return false;
                }
            }
            return true;
        };
        auto seg_ok = [&](int layer, double x0, double y0, double x1, double y1) {
            const double dx = x1 - x0, dy = y1 - y0;
            const int steps = std::max(1, (int)std::ceil(std::sqrt(dx*dx+dy*dy) * 2.0));
            int px = INT_MIN, py = INT_MIN;
            for (int s = 0; s <= steps; ++s) {
                const double t = (double)s / (double)steps;
                const int cx = (int)std::lround(x0 + t * dx);
                const int cy = (int)std::lround(y0 + t * dy);
                if (cx == px && cy == py) continue;
                px = cx; py = cy;
                if (!disk_ok(layer, cx, cy)) return false;
            }
            return true;
        };
        auto stamp_cell = [&](int layer, int cx, int cy) {
            for (int ddy = -hwi; ddy <= hwi; ++ddy) {
                const int y = cy + ddy; if (y < 0 || y >= H) continue;
                for (int ddx = -hwi; ddx <= hwi; ++ddx) {
                    if ((double)(ddx*ddx+ddy*ddy) > hw2) continue;
                    const int x = cx + ddx; if (x < 0 || x >= W) continue;
                    reserved[grid_.index(layer, x, y)] = tok;
                }
            }
        };
        auto reserve = [&](int layer, double x0, double y0, double x1, double y1) {
            const double dx = x1 - x0, dy = y1 - y0;
            const int steps = std::max(1, (int)std::ceil(std::sqrt(dx*dx+dy*dy) * 2.0));
            for (int s = 0; s <= steps; ++s) {
                const double t = (double)s / (double)steps;
                stamp_cell(layer, (int)std::lround(x0 + t * dx),
                           (int)std::lround(y0 + t * dy));
            }
        };
        // sample an arc (centre cx,cy, radius r, angles a0->a1 the short way)
        // into `fn` at <= 0.5-cell spacing.
        auto arc_walk = [&](double ccx, double ccy, double r, double a0,
                            double a1, const std::function<bool(int,int)>& fn) {
            double d = a1 - a0;
            while (d >  M_PI) d -= 2.0 * M_PI;
            while (d < -M_PI) d += 2.0 * M_PI;
            const int steps = std::max(1, (int)std::ceil(std::fabs(d) * r * 2.0));
            int px = INT_MIN, py = INT_MIN;
            for (int s = 0; s <= steps; ++s) {
                const double a = a0 + d * (double)s / (double)steps;
                const int cx = (int)std::lround(ccx + r * std::cos(a));
                const int cy = (int)std::lround(ccy + r * std::sin(a));
                if (cx == px && cy == py) continue;
                px = cx; py = cy;
                if (!fn(cx, cy)) return false;
            }
            return true;
        };

        // 2) per-layer polylines -> simplified vertex list -> emit lines and
        //    (when fillet_radius_cells > 0) validated corner arcs.
        auto simplify = [&](const std::vector<Cell>& poly,
                            std::vector<std::pair<double,double>>& verts) {
            verts.clear();
            verts.push_back({(double)poly[0].x, (double)poly[0].y});
            size_t a = 0;
            while (a + 1 < poly.size()) {
                size_t best = a + 1;
                for (size_t b = a + 2; b < poly.size(); ++b) {
                    bool corridor = true;
                    for (size_t m = a + 1; m < b && corridor; ++m)
                        if (seg_dist((double)poly[m].x, (double)poly[m].y,
                                     (double)poly[a].x, (double)poly[a].y,
                                     (double)poly[b].x, (double)poly[b].y) > max_dev_cells)
                            corridor = false;
                    if (!corridor) break;   // vertices only get farther from the chord
                    if (seg_ok((int)poly[0].layer, (double)poly[a].x, (double)poly[a].y,
                               (double)poly[b].x, (double)poly[b].y))
                        best = b;
                }
                verts.push_back({(double)poly[best].x, (double)poly[best].y});
                removed += (int)(best - a) - 1;
                a = best;
            }
        };
        auto emit_verts = [&](int layer, const std::vector<std::pair<double,double>>& V) {
            if (V.size() < 2) return;
            if (fillet_radius_cells <= 0.0 || V.size() < 3) {
                for (size_t i = 0; i + 1 < V.size(); ++i) {
                    out.push_back(SmoothSeg{layer, V[i].first, V[i].second,
                                            V[i+1].first, V[i+1].second});
                    reserve(layer, V[i].first, V[i].second, V[i+1].first, V[i+1].second);
                }
                return;
            }
            double cx = V[0].first, cy = V[0].second;   // running start of next line
            for (size_t i = 1; i + 1 < V.size(); ++i) {
                const double vx = V[i].first, vy = V[i].second;
                const double bx = V[i+1].first, by = V[i+1].second;
                double iax = cx - vx, iay = cy - vy;          // toward previous
                double obx = bx - vx, oby = by - vy;          // toward next
                const double li = std::sqrt(iax*iax + iay*iay);
                const double lo = std::sqrt(obx*obx + oby*oby);
                bool done = false;
                if (li > 1e-6 && lo > 1e-6) {
                    iax /= li; iay /= li; obx /= lo; oby /= lo;
                    double cosphi = std::max(-1.0, std::min(1.0, iax*obx + iay*oby));
                    const double phi = std::acos(cosphi);      // interior angle
                    // skip near-straight (no visible corner) and near-reversal
                    if (phi > 0.20 && phi < M_PI - 0.05) {
                        const double halfp = phi * 0.5;
                        double bxs = iax + obx, bys = iay + oby;
                        const double bl = std::sqrt(bxs*bxs + bys*bys);
                        if (bl > 1e-6) {
                            bxs /= bl; bys /= bl;              // bisector (into corner)
                            const double tmax = 0.45 * std::min(li, lo);
                            double r = fillet_radius_cells;
                            for (int attempt = 0; attempt < 5 && !done; ++attempt) {
                                double t = r / std::tan(halfp);
                                if (t > tmax) { r = tmax * std::tan(halfp); t = tmax; }
                                if (r < std::max(1.0, hw)) break;   // too small to bother
                                const double t1x = vx + iax*t, t1y = vy + iay*t;
                                const double t2x = vx + obx*t, t2y = vy + oby*t;
                                const double D = r / std::sin(halfp);
                                const double ctx = vx + bxs*D, cty = vy + bys*D;
                                double mvx = vx - ctx, mvy = vy - cty;
                                const double ml = std::sqrt(mvx*mvx + mvy*mvy);
                                const double midx = ctx + mvx/ml*r, midy = cty + mvy/ml*r;
                                const double a1 = std::atan2(t1y - cty, t1x - ctx);
                                const double a2 = std::atan2(t2y - cty, t2x - ctx);
                                if (arc_walk(ctx, cty, r, a1, a2,
                                        [&](int gx, int gy){ return disk_ok(layer, gx, gy); })) {
                                    // line cur->T1 (sub-segment of a validated seg), arc T1->T2
                                    out.push_back(SmoothSeg{layer, cx, cy, t1x, t1y});
                                    reserve(layer, cx, cy, t1x, t1y);
                                    SmoothSeg arc{layer, t1x, t1y, t2x, t2y};
                                    arc.mx = midx; arc.my = midy; arc.is_arc = true;
                                    out.push_back(arc);
                                    arc_walk(ctx, cty, r, a1, a2,
                                             [&](int gx, int gy){ stamp_cell(layer, gx, gy); return true; });
                                    cx = t2x; cy = t2y;
                                    done = true;
                                }
                                r *= 0.55;
                            }
                        }
                    }
                }
                if (!done) {   // no fillet: plain corner at V
                    out.push_back(SmoothSeg{layer, cx, cy, vx, vy});
                    reserve(layer, cx, cy, vx, vy);
                    cx = vx; cy = vy;
                }
            }
            out.push_back(SmoothSeg{layer, cx, cy, V.back().first, V.back().second});
            reserve(layer, cx, cy, V.back().first, V.back().second);
        };

        std::vector<std::pair<double,double>> verts;
        for (const auto& path : nets_[k].segments) {
            std::vector<Cell> poly;
            auto flush = [&]() {
                if (poly.size() >= 2) { simplify(poly, verts); emit_verts(poly[0].layer, verts); }
                poly.clear();
            };
            for (const Cell& c : path) {
                if (!valid(c) || owner_[grid_.index(c)] != tok) { flush(); continue; }
                if (!poly.empty() && poly.back().layer != c.layer) flush();
                if (poly.empty() || poly.back().x != c.x || poly.back().y != c.y)
                    poly.push_back(c);
            }
            flush();
        }
    }
    return removed;
}

double Board::width_units() const {
    return (double)grid_.width() * grid_.resolution();
}
double Board::height_units() const {
    return (double)grid_.height() * grid_.resolution();
}

namespace {
// Binary Chebyshev box dilation by radius r over a 2D u8 mask, separable
// (horizontal any-in-window then vertical). Exact: dilated[x,y] = 1 iff any
// set cell within Chebyshev distance r.
void dilate_cheby(std::vector<uint8_t>& m, int W, int H, int r,
                  std::vector<uint8_t>& tmp) {
    if (r <= 0) return;
    tmp.assign(m.size(), 0);
    for (int y = 0; y < H; ++y) {           // horizontal pass
        const uint8_t* src = m.data() + (size_t)y * W;
        uint8_t* dst = tmp.data() + (size_t)y * W;
        int cnt = 0;
        for (int x = -r; x < W; ++x) {
            if (x + r < W && src[x + r]) ++cnt;
            if (x - r - 1 >= 0 && src[x - r - 1]) --cnt;
            if (x >= 0) dst[x] = cnt > 0;
        }
    }
    for (int x = 0; x < W; ++x) {           // vertical pass
        int cnt = 0;
        for (int y = -r; y < H; ++y) {
            if (y + r < H && tmp[(size_t)(y + r) * W + x]) ++cnt;
            if (y - r - 1 >= 0 && tmp[(size_t)(y - r - 1) * W + x]) --cnt;
            if (y >= 0) m[(size_t)y * W + x] = cnt > 0;
        }
    }
}
} // namespace


std::vector<uint8_t> Board::fast_via_mask_window(int k, int x0, int y0,
                                                 int x1, int y1) {
    const int L = grid_.layers(), W = grid_.width(), H = grid_.height();
    x0 = std::max(0, x0); y0 = std::max(0, y0);
    x1 = std::min(W - 1, x1); y1 = std::min(H - 1, y1);
    const int ww = x1 - x0 + 1, wh = y1 - y0 + 1;
    const int32_t tok = nets_[(size_t)k].id + 1;
    // Masks are built over window+R only (NOT the full board): a mask cell
    // can influence the window by at most its dilation radius.
    const double clr = (double)drc_clearance_cells();
    const int r_keep = (int)std::ceil(via_radius_cells_);
    const int r_trc = (int)std::ceil(via_radius_cells_ + trace_hw_cells_ + clr);
    const int r_via = (int)std::ceil(2.0 * via_radius_cells_ + clr);
    const int R = std::max(r_via, r_trc);
    const int mx0 = std::max(0, x0 - R), my0 = std::max(0, y0 - R);
    const int mx1 = std::min(W - 1, x1 + R), my1 = std::min(H - 1, y1 + R);
    const int mw = mx1 - mx0 + 1, mh = my1 - my0 + 1;
    std::vector<uint8_t> d_own, d_trc, d_via;
    {
        const size_t mplane = (size_t)mw * mh;
        d_own.assign(mplane, 0); d_trc.assign(mplane, 0); d_via.assign(mplane, 0);
        for (int l = 0; l < L; ++l)
            for (int my = 0; my < mh; ++my) {
                const size_t off = grid_.index(l, mx0, my0 + my);
                const size_t mo = (size_t)my * mw;
                for (int mx = 0; mx < mw; ++mx) {
                    const int32_t o = owner_[off + mx];
                    if (o != 0 && o != tok) d_own[mo + mx] = 1;
                    if (copper_[off + mx]) {
                        const int32_t t = trace_owner_[off + mx];
                        if (t != 0 && t != tok) d_trc[mo + mx] = 1;
                    }
                    const int32_t v = via_owner_[off + mx];
                    if (v != 0 && v != tok) d_via[mo + mx] = 1;
                }
            }
        std::vector<uint8_t> tmp;
        dilate_cheby(d_own, mw, mh, r_keep, tmp);
        dilate_cheby(d_trc, mw, mh, r_trc, tmp);
        dilate_cheby(d_via, mw, mh, r_via, tmp);
    }

    std::vector<uint8_t> out((size_t)L * wh * ww, 0);
    for (int row = 0; row < L * wh; ++row) {
        const int wy = row % wh;
        const size_t base = (size_t)row * ww;
        for (int wx = 0; wx < ww; ++wx) {
            const int gx = x0 + wx, gy = y0 + wy;
            const size_t i2 = (size_t)gy * W + gx;
            const size_t im = (size_t)(gy - my0) * mw + (gx - mx0);
            uint8_t ok;
            if (thru_pad_col_[i2] == tok) {
                ok = 1;                     // own barrel: never vetoed
            } else {
                ok = 1;
                for (int pl = 0; pl < L && ok; ++pl) {
                    const int32_t pz = via_pad_zone_[grid_.index(pl, gx, gy)];
                    if (pz != 0 && pz != tok) ok = 0;
                }
                if (ok && (d_own[im] || d_trc[im] || d_via[im])) ok = 0;
            }
            out[base + wx] = ok;
        }
    }
    return out;
}

std::tuple<std::vector<uint32_t>, std::vector<uint8_t>, uint32_t>
Board::bake_net_cost_window(int k, int x0, int y0, int x1, int y1,
                            bool fast_via,
                            const std::vector<uint8_t>* via_precomputed) {
    NetContext ctx = make_net_context(k);
    const int L = grid_.layers(), W = grid_.width(), H = grid_.height();
    x0 = std::max(0, x0); y0 = std::max(0, y0);
    x1 = std::min(W - 1, x1); y1 = std::min(H - 1, y1);
    const int ww = x1 - x0 + 1, wh = y1 - y0 + 1;
    std::vector<uint32_t> cost((size_t)L * wh * ww, 0xFFFFFFFFu);
    std::vector<uint8_t> via((size_t)L * wh * ww, 0);

    const int32_t tok = (k >= 0 && (size_t)k < nets_.size())
                            ? nets_[(size_t)k].id + 1 : -1;
    const bool use_fast = fast_via && via_radius_cells_ > 0.0 && tok > 0;
    std::vector<uint8_t> fastv;
    if (via_precomputed) fastv = *via_precomputed;
    else if (use_fast) fastv = fast_via_mask_window(k, x0, y0, x1, y1);
    const bool have_via = via_precomputed != nullptr || use_fast;

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int row = 0; row < L * wh; ++row) {
        const int l = row / wh, wy = row - l * wh;
        const size_t base = (size_t)row * ww;
        for (int wx = 0; wx < ww; ++wx) {
            const Cell c{l, x0 + wx, y0 + wy};
            const double ec = ctx.enter(c);
            // q20 units (2^-20): the exact lattice of enter costs with
            // congestion gamma==1 (quantized weight x quantized field =
            // 2^-20 dyadics). Exact for dyadic ec; llround guards drift.
            if (ec >= 0.0)
                cost[base + wx] = (uint32_t)std::llround(ec * 1048576.0);
            via[base + wx] = have_via
                ? fastv[base + wx]
                : ((!ctx.via_ok || ctx.via_ok(c)) ? 1 : 0);
        }
    }
    return {std::move(cost), std::move(via),
            (uint32_t)std::llround(via_cost_ * 1048576.0)};
}

std::tuple<std::vector<uint32_t>, std::vector<uint8_t>, uint32_t>
Board::bake_net_cost_grid(int k) {
    return bake_net_cost_window(k, 0, 0, grid_.width() - 1,
                                grid_.height() - 1, true);
}

// ---- field router (stage 2): single all-pins field + boundary-MST tree ----
namespace {
std::atomic<bool> g_field_router{false};
// per-phase wall accumulators (seconds*1e6), reset via field_phase_reset()
std::array<std::atomic<int64_t>, 8> g_fphase{};  // bake,field,labels,scan,walk,commit_misc,unsat,total
struct FTimer {
    int slot; std::chrono::steady_clock::time_point t0;
    explicit FTimer(int s) : slot(s), t0(std::chrono::steady_clock::now()) {}
    ~FTimer() {
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        g_fphase[(size_t)slot].fetch_add(us, std::memory_order_relaxed);
    }
};
}
std::vector<int64_t> field_phase_stats() {
    std::vector<int64_t> out;
    for (auto& a : g_fphase) out.push_back(a.load());
    return out;
}
void field_phase_reset() { for (auto& a : g_fphase) a.store(0); }
void set_field_router(bool on) { g_field_router.store(on); }
bool field_router_enabled() { return g_field_router.load(); }


void Board::rip_net_positions(const std::vector<int>& positions) {
    std::vector<int32_t> rip_tokens;
    rip_tokens.reserve(positions.size());
    for (int k : positions) {
        if (k < 0 || (size_t)k >= nets_.size()) continue;
        Net& net = nets_[(size_t)k];
        rip_tokens.push_back((int32_t)net.id + 1);
        net.segments.clear();
        net.total_length = 0.0;
        net.routed = false;
        net.unconnected_pins = std::max(0, (int)net.pins.size() - 1);
        net.pin_connected.assign(net.pins.size(), 0);
        if (!net.pin_connected.empty()) net.pin_connected[0] = 1;
    }
    auto in_rip = [&](int32_t t) {
        for (int32_t rt : rip_tokens) if (rt == t) return true;
        return false;
    };
    for (size_t i = 0; i < trace_owner_.size(); ++i) {
        if (trace_owner_[i] != 0 && in_rip(trace_owner_[i])) {
            trace_owner_[i] = 0;
            copper_[i] = 0;
            owner_[i] = pad_owner_[i];
        }
        if (via_owner_[i] != 0 && in_rip(via_owner_[i])) via_owner_[i] = 0;
    }
    // Rebuild trace avoidance from surviving copper; invalidate self-scratch.
    active_self_net_ = -1;
    if (avoid_baked_) {
        std::fill(trace_avoid_cost_.begin(), trace_avoid_cost_.end(), 0.0);
        for (size_t i = 0; i < trace_owner_.size(); ++i)
            if (trace_owner_[i] != 0)
                add_source_avoidance(grid_.unindex(i), trace_avoid_cost_, trace_sharp_);
    }
    rebuild_cl_zone();
}

Board::NegotiateResult Board::negotiate_window(const std::vector<int>& positions,
                                               int x0, int y0, int x1, int y1,
                                               int max_iters, double pres_init,
                                               double pres_mult, double hist_gain) {
    NegotiateResult res;
    const int W = grid_.width(), H = grid_.height(), L = grid_.layers();
    x0 = std::max(0, x0); y0 = std::max(0, y0);
    x1 = std::min(W - 1, x1); y1 = std::min(H - 1, y1);
    const int ww = x1 - x0 + 1, wh = y1 - y0 + 1;
    const size_t wcells = (size_t)L * wh * ww;
    const int K = (int)positions.size();
    if (K == 0 || ww <= 2 || wh <= 2) return res;
    res.routed.assign((size_t)K, 0);

    // All participant pins must sit inside the window.
    for (int k : positions) {
        if (k < 0 || (size_t)k >= nets_.size()) return res;
        for (const Cell& c : nets_[(size_t)k].pins)
            if (c.x < x0 || c.x > x1 || c.y < y0 || c.y > y1) return res;
    }

    prepare_pad_keepout();
    rip_net_positions(positions);

    // Per-participant hard-world base bakes (participants absent from board).
    constexpr uint32_t BLOCK = 0xFFFFFFFFu;
    constexpr uint32_t CAP = 0x3FFFFFFFu;
    std::vector<std::vector<uint32_t>> base((size_t)K);
    std::vector<std::vector<uint8_t>> viaok((size_t)K);
    std::vector<uint32_t> viaq((size_t)K, 0);
    uint32_t min_base = BLOCK;
    for (int i = 0; i < K; ++i) {
        auto baked = bake_net_cost_window(positions[(size_t)i], x0, y0, x1, y1);
        base[(size_t)i] = std::move(std::get<0>(baked));
        viaok[(size_t)i] = std::move(std::get<1>(baked));
        viaq[(size_t)i] = std::get<2>(baked);
        for (uint32_t c : base[(size_t)i])
            if (c != BLOCK && c > 0 && c < min_base) min_base = c;
    }
    if (min_base == BLOCK) min_base = 1u << 20;
    const double hist_add = hist_gain * (double)(min_base >> 3);

    // Claim dilation radii (cells): disjoint claims => DRC-clean separation.
    // STRICT inequality: at radius exactly pitch/2, two nets at legal minimum
    // pitch share their boundary cells FOREVER (measured: 11.5k phantom
    // shared cells that no penalty can resolve — the nets are already
    // legal). Half-cell shrink makes legally-packed claims disjoint; the
    // final collect_stats(true) DRC pass owns exactness.
    // Metric-honest conflict test: DRC distances are EUCLIDEAN; separable
    // box dilation is Chebyshev and under-detects diagonal-parallel runs by
    // sqrt(2) (measured: 141 residual DRCs, diagonal-heavy routes). So the
    // CLAIMS carry nearly the whole radius as true Euclidean discs
    // (pitch-2) and the penalty dilation is a single cell; the residual
    // detection band is sub-cell and the polish pass owns it.
    const double tr_pitch = 2.0 * trace_hw_cells_ + (double)drc_clearance_cells();
    const double vt_pitch = via_radius_cells_ + trace_hw_cells_ + (double)drc_clearance_cells();
    const double vv_pitch = 2.0 * via_radius_cells_ + (double)drc_clearance_cells();
    // Claim radius derived NUMERICALLY per pitch: the largest r such that
    // (a) every integer pair at distance < pitch has a 3x3 neighbor within
    // r (detected), and (b) no pair at distance >= pitch does (no false
    // positive). A fixed shrink is only exact for one pitch (measured:
    // pitch-1.5 exact at pitch 8, blind to the whole [8,9) band at the
    // board's ACTUAL pitch 9 from its 0.25mm trace class).
    auto exact_claim_radius = [](double pitch) {
        const int R = (int)std::ceil(pitch) + 2;
        double need = 0.0;        // max over illegal pairs of nearest-neighbor dist
        double cap = 1e18;        // min over legal pairs of nearest-neighbor dist
        for (int dx = -R; dx <= R; ++dx)
            for (int dy = -R; dy <= R; ++dy) {
                if (dx == 0 && dy == 0) continue;
                const double d = std::sqrt((double)(dx * dx + dy * dy));
                double best = 1e18;
                for (int nx = -1; nx <= 1; ++nx)
                    for (int ny = -1; ny <= 1; ++ny) {
                        const double nd = std::sqrt((double)((dx + nx) * (dx + nx)
                                                            + (dy + ny) * (dy + ny)));
                        best = std::min(best, nd);
                    }
                if (d < pitch - 1e-9) need = std::max(need, best);
                else cap = std::min(cap, best);
            }
        // exact iff need < cap; midpoint is safely both
        return 0.5 * (need + std::min(cap, need + 1.0));
    };
    const double r_tr = exact_claim_radius(tr_pitch);
    const double r_via = exact_claim_radius(vt_pitch);

    std::vector<uint16_t> usage(wcells, 0);       // trace-class claims
    std::vector<uint16_t> usage_v(wcells, 0);     // via-class claims
    std::vector<float> hist(wcells, 0.f);
    std::vector<std::vector<int64_t>> claims((size_t)K);
    std::vector<std::vector<int64_t>> claims_v((size_t)K);
    std::vector<std::vector<std::vector<int64_t>>> conns((size_t)K);
    std::vector<int32_t> stamp(wcells, -1);
    int stamp_id = 0;

    auto widx = [&](int l, int x, int y) -> int64_t {
        return ((int64_t)l * wh + (y - y0)) * ww + (x - x0);
    };
    auto dilate_path = [&](const std::vector<int64_t>& path,
                           std::vector<int64_t>& out, std::vector<int64_t>& out_v) {
        ++stamp_id;
        out.clear();
        auto claim_disc = [&](int l, int cx, int cy, double r) {
            const int ir = (int)std::ceil(r);
            for (int dy = -ir; dy <= ir; ++dy)
                for (int dx = -ir; dx <= ir; ++dx) {
                    if ((double)(dx * dx + dy * dy) > r * r + 1e-9) continue;
                    const int nx = cx + dx, ny = cy + dy;
                    if (nx < x0 || nx > x1 || ny < y0 || ny > y1) continue;
                    const int64_t wi = widx(l, nx, ny);
                    if (stamp[(size_t)wi] != stamp_id) {
                        stamp[(size_t)wi] = stamp_id;
                        out.push_back(wi);
                    }
                }
        };
        for (size_t i = 0; i < path.size(); ++i) {
            const int64_t ci = path[i];
            const int l = (int)(ci / ((int64_t)wh * ww));
            const int64_t rem = ci % ((int64_t)wh * ww);
            const int y = (int)(rem / ww) + y0, x = (int)(rem % ww) + x0;
            claim_disc(l, x, y, r_tr);
        }
        // via-class claims in their own field (via pitch needs its own
        // asymmetric dilation; measured 198 slop DRCs with trace-sized rr)
        ++stamp_id;
        out_v.clear();
        std::swap(out, out_v);      // reuse claim_disc's sink
        for (size_t i = 1; i < path.size(); ++i) {
            const int64_t ci = path[i];
            const int l = (int)(ci / ((int64_t)wh * ww));
            const int pl = (int)(path[i - 1] / ((int64_t)wh * ww));
            if (pl != l) {
                const int64_t rem = ci % ((int64_t)wh * ww);
                const int y = (int)(rem / ww) + y0, x = (int)(rem % ww) + x0;
                for (int vl = 0; vl < L; ++vl) claim_disc(vl, x, y, r_via);
            }
        }
        std::swap(out, out_v);
    };

    // Separable Chebyshev max-dilation by rr cells, per layer: a net entering
    // cell c creates a claim disc; it conflicts iff any OTHER claim lies
    // within the claim radius — so the penalty (and history) fields must be
    // the dilation of claim-space, not raw cells (measured failure mode:
    // two paths packed 3 cells apart, claims overlapping BETWEEN them,
    // every path cell penalty-free, stable illegal fixpoint forever).
    const int rr = 1;          // 1-cell penalty dilation on top of big claims
    const int rr_v = 1;
    // Via placement is enforced HARD through the via_ok mask (soft costs
    // cannot reach via edges through the oracle): forbid vias within
    // (vv_pitch-1 - r_via) of via claims and (vt_pitch-1 - r_tr) of trace
    // claims — both evaluate to the same small radius at default rules.
    const int rr_vv = std::max(1, (int)std::floor(vv_pitch - 1.0 - r_via));
    const int rr_vt = std::max(1, (int)std::floor(vt_pitch - 1.0 - r_tr));
    auto dilmax_r = [&](const std::vector<float>& in, std::vector<float>& out,
                        std::vector<float>& tmp, int rad) {
        for (int l = 0; l < L; ++l) {
            const size_t base_i = (size_t)l * wh * ww;
            for (int y = 0; y < wh; ++y) {
                const size_t row = base_i + (size_t)y * ww;
                for (int x = 0; x < ww; ++x) {
                    float m = 0.f;
                    const int a = std::max(0, x - rad), bx = std::min(ww - 1, x + rad);
                    for (int u = a; u <= bx; ++u) m = std::max(m, in[row + u]);
                    tmp[row + x] = m;
                }
            }
            for (int x = 0; x < ww; ++x)
                for (int y = 0; y < wh; ++y) {
                    float m = 0.f;
                    const int a = std::max(0, y - rad), by = std::min(wh - 1, y + rad);
                    for (int v = a; v <= by; ++v) m = std::max(m, tmp[base_i + (size_t)v * ww + x]);
                    out[base_i + (size_t)y * ww + x] = m;
                }
        }
    };
    std::vector<float> fu(wcells), fdil(wcells), fdil_v(wcells), fvv(wcells), ftmp(wcells), hdil(wcells);
    std::vector<uint8_t> veff(wcells);
    auto dilmax = [&](const std::vector<float>& in, std::vector<float>& out,
                      std::vector<float>& tmp) { dilmax_r(in, out, tmp, rr); };
    std::vector<int> conflict_count((size_t)K, 1 << 20);
    double pres = pres_init;
    std::vector<uint32_t> eff(wcells);
    bool all_routed = false;
    int shared = -1;
    for (int it = 0; it < max_iters; ++it) {
        res.iterations = it + 1;
        for (int i = 0; i < K; ++i) {
            const int k = positions[(size_t)i];
            const Net& net = nets_[(size_t)k];
            for (int64_t wi : claims[(size_t)i]) --usage[(size_t)wi];
            for (int64_t wi : claims_v[(size_t)i]) --usage_v[(size_t)wi];
            const auto& b = base[(size_t)i];
            // others' claim-space and history, dilated by the claim radius
            for (size_t c = 0; c < wcells; ++c) fu[c] = (float)usage[c];
            dilmax(fu, fdil, ftmp);
            for (size_t c = 0; c < wcells; ++c) fu[c] = (float)usage_v[c];
            dilmax_r(fu, fdil_v, ftmp, rr_v);
            dilmax_r(fu, fvv, ftmp, rr_vv);
            for (size_t c = 0; c < wcells; ++c) fdil[c] += fdil_v[c];
            dilmax(hist, hdil, ftmp);
            for (size_t c = 0; c < wcells; ++c) fu[c] = (float)usage[c];
            dilmax_r(fu, ftmp, fdil_v, rr_vt);   // trace claims vs via sites
            const auto& vok = viaok[(size_t)i];
            for (size_t c = 0; c < wcells; ++c)
                veff[c] = (uint8_t)(vok[c] && fvv[c] == 0.f && ftmp[c] == 0.f);
            // base >> 3: headroom under the u32 CAP so penalties outrank
            // multi-thousand-cell detours instead of saturating.
            for (size_t c = 0; c < wcells; ++c) {
                if (b[c] == BLOCK) { eff[c] = BLOCK; continue; }
                const double e = ((double)(b[c] >> 3) + (double)hdil[c])
                                 * (1.0 + pres * (double)fdil[c]);
                eff[c] = (uint32_t)std::max<double>(1.0, std::min<double>(e, (double)CAP));
            }
            // Prim-style multi-pin: grow the tree connection by connection.
            std::vector<int64_t> tree;
            tree.push_back(widx(net.pins[0].layer, net.pins[0].x, net.pins[0].y));
            std::vector<int64_t> pend;
            for (size_t p = 1; p < net.pins.size(); ++p)
                pend.push_back(widx(net.pins[p].layer, net.pins[p].x, net.pins[p].y));
            conns[(size_t)i].clear();
            bool ok = true;
            while (!pend.empty()) {
                auto dist = dijkstra_field(ww, wh, L, eff, veff,
                                           viaq[(size_t)i], tree);
                size_t bi = 0; uint64_t bd = UINT64_MAX;
                for (size_t p = 0; p < pend.size(); ++p)
                    if (dist[(size_t)pend[p]] < bd) { bd = dist[(size_t)pend[p]]; bi = p; }
                if (bd >= (uint64_t)1 << 62) { ok = false; break; }
                auto path = walk_field_descent(ww, wh, L, eff, veff,
                                               viaq[(size_t)i], dist, pend[bi]);
                if (path.empty()) { ok = false; break; }
                conns[(size_t)i].push_back(path);
                for (int64_t ci : path) tree.push_back(ci);
                pend.erase(pend.begin() + (long)bi);
            }
            res.routed[(size_t)i] = ok ? 1 : 0;
            if (ok) {
                int nc = 0;
                for (const auto& cn : conns[(size_t)i])
                    for (int64_t ci : cn)
                        if (fdil[(size_t)ci] > 0.f) {
                            ++nc;
                            hist[(size_t)ci] += (float)hist_add;
                        }
                conflict_count[(size_t)i] = nc;
            } else conflict_count[(size_t)i] = 1 << 20;
            if (std::getenv("NEGOTIATE_DEBUG") && it == 0) {
                size_t nb = 0; for (size_t c = 0; c < wcells; ++c) if (eff[c] != BLOCK) ++nb;
                const int64_t s0 = widx(net.pins[0].layer, net.pins[0].x, net.pins[0].y);
                fprintf(stderr, "[neg:n%d] passable=%zu/%zu eff[pin0]=%u ok=%d conns=%zu "
                        "chan(8,20,32)@y7=%u,%u,%u\n",
                        k, nb, wcells, eff[(size_t)s0], (int)ok, conns[(size_t)i].size(),
                        eff[(size_t)widx(0, 8, 7)], eff[(size_t)widx(0, 20, 7)],
                        eff[(size_t)widx(0, 32, 7)]);
            }
            if (ok) {
                std::vector<int64_t> flat;
                for (const auto& cn : conns[(size_t)i])
                    for (int64_t ci : cn) flat.push_back(ci);
                dilate_path(flat, claims[(size_t)i], claims_v[(size_t)i]);
            } else {
                claims[(size_t)i].clear();
                claims_v[(size_t)i].clear();
            }
            for (int64_t wi : claims[(size_t)i]) ++usage[(size_t)wi];
            for (int64_t wi : claims_v[(size_t)i]) ++usage_v[(size_t)wi];
        }
        shared = 0;
        for (int i = 0; i < K; ++i)
            if (conflict_count[(size_t)i] < (1 << 20)) shared += conflict_count[(size_t)i];
        all_routed = true;
        for (int i = 0; i < K; ++i) if (!res.routed[(size_t)i]) all_routed = false;
        if (std::getenv("NEGOTIATE_DEBUG")) {
            fprintf(stderr, "[neg] it=%d shared=%d pres=%.2f routed=", it, shared, pres);
            for (int i = 0; i < K; ++i) fprintf(stderr, "%d", (int)res.routed[(size_t)i]);
            fprintf(stderr, "\n");
        }
        if (shared == 0 && all_routed) { res.converged = true; break; }
        if (std::getenv("NEGOTIATE_DEBUG") && it == 6) {
            for (int i = 0; i < K; ++i) {
                int minx = 1 << 30, maxx = -1, miny = 1 << 30, maxy = -1;
                size_t npc = 0;
                for (const auto& cn : conns[(size_t)i])
                    for (int64_t ci : cn) {
                        const int64_t rem = ci % ((int64_t)wh * ww);
                        const int xx = (int)(rem % ww) + x0, yy = (int)(rem / ww) + y0;
                        minx = std::min(minx, xx); maxx = std::max(maxx, xx);
                        miny = std::min(miny, yy); maxy = std::max(maxy, yy);
                        ++npc;
                    }
                size_t on_shared = 0;
                for (const auto& cn : conns[(size_t)i])
                    for (int64_t ci : cn) if (usage[(size_t)ci] > 1) ++on_shared;
                fprintf(stderr, "[neg-path] net%d cells=%zu bbox=(%d,%d)-(%d,%d) on_shared=%zu\n",
                        positions[(size_t)i], npc, minx, miny, maxx, maxy, on_shared);
            }
        }
        if (false) {
            int printed = 0;
            for (size_t c = 0; c < wcells && printed < 12; ++c)
                if (usage[c] > 1) {
                    const int l = (int)(c / ((size_t)wh * ww));
                    const size_t rem = c % ((size_t)wh * ww);
                    fprintf(stderr, "[neg-shared] l=%d x=%d y=%d usage=%d owners:",
                            l, (int)(rem % ww) + x0, (int)(rem / ww) + y0, (int)usage[c]);
                    for (int i = 0; i < K; ++i)
                        for (int64_t wi : claims[(size_t)i])
                            if ((size_t)wi == c) { fprintf(stderr, " %d", positions[(size_t)i]); break; }
                    fprintf(stderr, "\n");
                    ++printed;
                }
        }
        pres *= pres_mult;
    }
    res.shared_final = shared;
    if (!res.converged) return res;   // participants left ripped; caller restores

    // Commit negotiated paths through the normal physics.
    for (int i = 0; i < K; ++i) {
        const int k = positions[(size_t)i];
        Net& net = nets_[(size_t)k];
        net.segments.clear();
        net.total_length = 0.0;
        for (const auto& cn : conns[(size_t)i]) {
            std::vector<Cell> cells;
            cells.reserve(cn.size());
            for (int64_t ci : cn) {
                const int l = (int)(ci / ((int64_t)wh * ww));
                const int64_t rem = ci % ((int64_t)wh * ww);
                cells.push_back(Cell{l, (int)(rem % ww) + x0, (int)(rem / ww) + y0});
            }
            for (size_t j = 1; j < cells.size(); ++j) {
                const int ddx = std::abs(cells[j].x - cells[j-1].x);
                const int ddy = std::abs(cells[j].y - cells[j-1].y);
                net.total_length += (ddx && ddy) ? 1.41421356 : 1.0;
            }
            net.segments.push_back(cells);
        }
        net.routed = true;
        net.unconnected_pins = 0;
        net.pin_connected.assign(net.pins.size(), 1);
        for (const auto& seg : net.segments) commit_path(net, seg);
    }
    active_self_net_ = -1;
    rebuild_cl_zone();

    // Violation-driven iterative polish. Rerouting EVERY participant is
    // wrong: a failed reroute restores negotiated geometry that was never
    // checked against neighbors already MOVED by their own polish (measured:
    // 238 parallel-run DRCs between restored and polished participants).
    // Instead: find the participants actually involved in violations,
    // reroute only those, in rounds, until clean or no progress.
    auto violating = [&]() {
        std::vector<uint8_t> hit((size_t)K, 0);
        const double spacing_tt = 2.0 * trace_hw_cells_ + (double)drc_clearance_cells();
        const int sr = (int)std::ceil(spacing_tt);
        std::vector<int32_t> tok2i((size_t)nets_.size() + 2, -1);
        for (int i = 0; i < K; ++i)
            tok2i[(size_t)nets_[(size_t)positions[(size_t)i]].id + 1] = i;
        const double pad_margin = (double)drc_clearance_cells() + trace_hw_cells_;
        for (int i = 0; i < K; ++i) {
            const Net& net = nets_[(size_t)positions[(size_t)i]];
            const int32_t tn = (int32_t)net.id + 1;
            for (const auto& seg : net.segments)
                for (const Cell& c : seg) {
                    if (!valid(c)) continue;
                    const size_t ci = grid_.index(c);
                    if (!copper_[ci] || trace_owner_[ci] != tn) continue;
                    // pad-trace class (was unscanned: polish couldn't see it)
                    for (const PadShape& ps : pad_shapes_) {
                        if (ps.via_only) continue;
                        if (ps.tok == tn || ps.layer != c.layer) continue;
                        const double reach = std::hypot(ps.half_w, ps.half_h) + pad_margin;
                        if (std::abs((double)c.x - ps.cx) > reach
                            || std::abs((double)c.y - ps.cy) > reach) continue;
                        if (ps.distance((double)c.x, (double)c.y) < pad_margin) {
                            hit[(size_t)i] = 1; break;
                        }
                    }
                    const int xa = std::max(0, c.x - sr), xb = std::min(grid_.width() - 1, c.x + sr);
                    const int ya = std::max(0, c.y - sr), yb = std::min(grid_.height() - 1, c.y + sr);
                    for (int y = ya; y <= yb; ++y)
                        for (int x = xa; x <= xb; ++x) {
                            const size_t j = grid_.index(c.layer, x, y);
                            if (!copper_[j]) continue;
                            const int32_t t2 = trace_owner_[j];
                            if (t2 == 0 || t2 == tn) continue;
                            const double dx = (double)(x - c.x), dy = (double)(y - c.y);
                            if (dx * dx + dy * dy < spacing_tt * spacing_tt - 1e-9) {
                                hit[(size_t)i] = 1;
                                if (t2 < (int32_t)tok2i.size() && tok2i[(size_t)t2] >= 0)
                                    hit[(size_t)tok2i[(size_t)t2]] = 1;
                            }
                        }
                }
        }
        return hit;
    };
    // rounds scale with reshuffle size (measured: 22-participant reshuffle
    // exhausted 4 rounds with 39 violations left, forfeiting a 6->2 gain)
    const int polish_rounds = 4 + K / 4;
    for (int round = 0; round < polish_rounds; ++round) {
        auto hit = violating();
        int nv = 0;
        for (int i = 0; i < K; ++i) nv += hit[(size_t)i];
        if (std::getenv("NEGOTIATE_DEBUG"))
            fprintf(stderr, "[neg-polish] round %d: %d violating participants\n", round, nv);
        if (nv == 0) break;
        bool any_change = false;
        for (int i = 0; i < K; ++i) {
            if (!hit[(size_t)i]) continue;
            const int k = positions[(size_t)i];
            auto negotiated = nets_[(size_t)k].segments;
            rip_net_positions({k});
            prepare_pad_keepout();
            bool progress = true;
            while (progress && nets_[(size_t)k].unconnected_pins > 0) {
                auto cr = route_one_connection(k, -1);
                progress = cr.progress;
            }
            if (nets_[(size_t)k].unconnected_pins > 0) {
                rip_net_positions({k});
                Net& net = nets_[(size_t)k];
                net.segments = negotiated;
                net.routed = true;
                net.unconnected_pins = 0;
                net.pin_connected.assign(net.pins.size(), 1);
                for (const auto& seg : net.segments) commit_path(net, seg);
                rebuild_cl_zone();
            } else any_change = true;
        }
        if (!any_change) break;
    }
    for (int i = 0; i < K; ++i) {
        const int k = positions[(size_t)i];
        if (std::getenv("NEGOTIATE_DEBUG")) {
            fprintf(stderr, "[neg-polish] net%d unconn=%d routed=%d | y80 owners:", k,
                    nets_[(size_t)k].unconnected_pins, (int)nets_[(size_t)k].routed);
            for (int cx : {64, 160, 256}) {
                fprintf(stderr, " ch%d[", cx);
                for (int dx = -4; dx <= 4; ++dx) {
                    const size_t ii = grid_.index(0, cx + dx, 80);
                    fprintf(stderr, "%d", owner_[ii]);
                }
                fprintf(stderr, "]");
            }
            fprintf(stderr, "\n");
        }
    }
    active_self_net_ = -1;
    rebuild_cl_zone();
    return res;
}

bool Board::route_in_corridor(int k, const std::vector<Cell>& corridor, double radius) {
    if (k < 0 || (size_t)k >= nets_.size() || corridor.empty()) return false;
    const int W = grid_.width(), H = grid_.height(), L = grid_.layers();
    Net& net = nets_[(size_t)k];
    prepare_pad_keepout();
    rip_net_positions({k});
    auto baked = bake_net_cost_grid(k);
    std::vector<uint32_t> cost = std::move(std::get<0>(baked));
    const std::vector<uint8_t>& via_ok = std::get<1>(baked);
    const uint32_t via_q = std::get<2>(baked);
    constexpr uint32_t BLOCK = 0xFFFFFFFFu;
    // corridor mask: dilated Euclidean discs, all listed layers
    std::vector<uint8_t> in_cor((size_t)L * H * W, 0);
    const int ir = (int)std::ceil(radius);
    for (const Cell& c0 : corridor) {
        for (int dy = -ir; dy <= ir; ++dy)
            for (int dx = -ir; dx <= ir; ++dx) {
                if ((double)(dx * dx + dy * dy) > radius * radius + 1e-9) continue;
                const int x = c0.x + dx, y = c0.y + dy;
                if (x < 0 || y < 0 || x >= W || y >= H) continue;
                if (c0.layer < 0 || c0.layer >= L) continue;
                in_cor[((size_t)c0.layer * H + y) * W + x] = 1;
            }
    }
    // pins always usable
    for (const Cell& p : net.pins)
        if (valid(p)) in_cor[grid_.index(p)] = 1;
    for (size_t i = 0; i < cost.size(); ++i)
        if (!in_cor[i]) cost[i] = BLOCK;
    // Prim-style connections
    std::vector<int64_t> tree;
    if (!valid(net.pins[0])) return false;
    tree.push_back((int64_t)grid_.index(net.pins[0]));
    std::vector<int64_t> pend;
    for (size_t p = 1; p < net.pins.size(); ++p)
        pend.push_back((int64_t)grid_.index(net.pins[p]));
    std::vector<std::vector<Cell>> segs;
    while (!pend.empty()) {
        auto dist = dijkstra_field(W, H, L, cost, via_ok, via_q, tree);
        size_t bi = 0; uint64_t bd = UINT64_MAX;
        for (size_t p = 0; p < pend.size(); ++p)
            if (dist[(size_t)pend[p]] < bd) { bd = dist[(size_t)pend[p]]; bi = p; }
        if (bd >= (uint64_t)1 << 62) return false;
        auto path = walk_field_descent(W, H, L, cost, via_ok, via_q, dist, pend[bi]);
        if (path.empty()) return false;
        std::vector<Cell> cells;
        for (int64_t ci : path) cells.push_back(grid_.unindex((size_t)ci));
        for (int64_t ci : path) tree.push_back(ci);
        segs.push_back(std::move(cells));
        pend.erase(pend.begin() + (long)bi);
    }
    net.segments = segs;
    net.total_length = 0.0;
    for (const auto& seg : net.segments)
        for (size_t j = 1; j < seg.size(); ++j) {
            const int ddx = std::abs(seg[j].x - seg[j-1].x);
            const int ddy = std::abs(seg[j].y - seg[j-1].y);
            net.total_length += (ddx && ddy) ? 1.41421356 : 1.0;
        }
    net.routed = true;
    net.unconnected_pins = 0;
    net.pin_connected.assign(net.pins.size(), 1);
    for (const auto& seg : net.segments) commit_path(net, seg);
    active_self_net_ = -1;
    rebuild_cl_zone();
    return true;
}

std::vector<int> Board::probe_blockers(int k, double soft_mult, bool* crossed_pad) {
    if (crossed_pad) *crossed_pad = false;
    if (k < 0 || (size_t)k >= nets_.size()) return {};
    const Net& net = nets_[(size_t)k];
    if (net.pins.size() < 2) return {};
    auto baked = bake_net_cost_grid(k);
    std::vector<uint32_t> cost = std::move(std::get<0>(baked));
    const std::vector<uint8_t>& via_ok = std::get<1>(baked);
    const uint32_t via_q = std::get<2>(baked);
    const int W = grid_.width(), H = grid_.height(), L = grid_.layers();
    constexpr uint32_t BLOCK = 0xFFFFFFFFu;
    const int32_t tok = (int32_t)net.id + 1;   // owner grids hold net.id+1

    uint32_t base_q = BLOCK;
    for (uint32_t c : cost) if (c != BLOCK && c < base_q) base_q = c;
    if (base_q == BLOCK || base_q == 0) base_q = 1024;
    const uint64_t soft_q = (uint64_t)(soft_mult * (double)base_q);
    constexpr uint32_t CAP = 0x3FFFFFFFu;
    std::vector<uint8_t> softened(cost.size(), 0);
    for (size_t i = 0; i < cost.size(); ++i) {
        if (cost[i] != BLOCK) continue;
        const int32_t po = pad_owner_[i], o = owner_[i];
        const bool foreign_pad = (po != 0 && po != tok);
        const bool foreign_trace = (o != 0 && o != tok && po == 0);
        uint64_t c = foreign_pad ? (uint64_t)base_q + 50u * soft_q
                   : foreign_trace ? (uint64_t)base_q + soft_q
                   : (uint64_t)base_q + soft_q;   // clearance halo: like trace
        cost[i] = (uint32_t)std::min<uint64_t>(c, (uint64_t)CAP);
        softened[i] = 1;
    }

    // Seeds = the net's ROOT copper component only (flood from pin 0):
    // unconnected pins can sit on committed ISLAND copper, and seeding all
    // own cells merges the islands, making every pin look reached (measured:
    // dist[start]=0 on every residual). The probe's real question is how to
    // JOIN components; start = an own cell outside the root component.
    std::vector<int64_t> seeds;
    if (!valid(net.pins[0])) return {};
    std::vector<uint8_t> in_root(owner_.size(), 0);
    {
        std::vector<int64_t> stack;
        const Cell& c0 = net.pins[0];
        const int64_t i0 = (int64_t)(((size_t)c0.layer * H + c0.y) * W + c0.x);
        stack.push_back(i0);
        if ((size_t)i0 < in_root.size()) in_root[(size_t)i0] = 1;
        while (!stack.empty()) {
            const int64_t ci = stack.back(); stack.pop_back();
            const int lay = (int)(ci / ((int64_t)W * H));
            const int y = (int)((ci / W) % H), x = (int)(ci % W);
            const int dx8[] = {1,-1,0,0,1,1,-1,-1}, dy8[] = {0,0,1,-1,1,-1,1,-1};
            for (int d = 0; d < 8; ++d) {
                const int nx = x + dx8[d], ny = y + dy8[d];
                if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                const size_t ni = ((size_t)lay * H + ny) * W + nx;
                if (!in_root[ni] && owner_[ni] == tok) { in_root[ni] = 1; stack.push_back((int64_t)ni); }
            }
            for (int dl = -1; dl <= 1; dl += 2) {
                const int nl = lay + dl;
                if (nl < 0 || nl >= L) continue;
                const size_t ni = ((size_t)nl * H + y) * W + x;
                if (!in_root[ni] && owner_[ni] == tok) { in_root[ni] = 1; stack.push_back((int64_t)ni); }
            }
        }
        for (size_t i = 0; i < owner_.size(); ++i)
            if (in_root[i]) seeds.push_back((int64_t)i);
    }
    // start: prefer an unconnected PIN cell outside the root component, else
    // any island cell.
    int64_t start = -1;
    for (size_t p = 1; p < net.pins.size(); ++p) {
        const Cell& c = net.pins[p];
        if (!valid(c)) continue;
        const size_t idx = ((size_t)c.layer * H + c.y) * W + c.x;
        if (!in_root[idx]) { start = (int64_t)idx; break; }
    }
    if (start < 0) {
        for (size_t i = 0; i < owner_.size(); ++i)
            if (owner_[i] == tok && !in_root[i]) { start = (int64_t)i; break; }
    }
    if (start < 0 || seeds.empty()) return {};

    auto dist = dijkstra_field(W, H, L, cost, via_ok, via_q, seeds);
    auto path = walk_field_descent(W, H, L, cost, via_ok, via_q, dist, start);

    std::vector<int> out;                       // returns net IDS, not positions
    int32_t max_id = 0;
    for (const auto& nn : nets_) max_id = std::max(max_id, nn.id);
    std::vector<char> seen((size_t)max_id + 2, 0);
    for (int64_t ci : path) {
        if (ci < 0 || (size_t)ci >= softened.size() || !softened[(size_t)ci])
            continue;
        const int layer = (int)(ci / ((int64_t)W * H));
        const int y = (int)((ci / W) % H), x = (int)(ci % W);
        // clearance halos extend well beyond the owning copper (~6 cells at
        // 0.05mm): scan wide enough to attribute halo crossings to their net.
        for (int dy = -8; dy <= 8; ++dy)
            for (int dx = -8; dx <= 8; ++dx) {
                const int nx = x + dx, ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                const size_t ni = ((size_t)layer * H + ny) * W + nx;
                const int32_t o = owner_[ni];
                if (o != 0 && o != tok && (size_t)(o - 1) < seen.size()
                        && !seen[(size_t)(o - 1)] && pad_owner_[ni] == 0) {
                    seen[(size_t)(o - 1)] = 1;
                    out.push_back(o - 1);      // net ID of the blocking net
                }
                const int32_t po = pad_owner_[ni];
                if (po != 0 && po != tok && crossed_pad
                        && dx >= -2 && dx <= 2 && dy >= -2 && dy <= 2)
                    *crossed_pad = true;
            }
    }
    return out;
}

int Board::route_net_field(int k, RouteDeadline deadline) {
    return route_net_field_into(k, nets_[(size_t)k], deadline);
}

namespace {
// The resident GPU pipeline (field -> labels -> boundary -> walks) is
// stateful across calls: one net's sequence must not interleave with
// another's. Batch threads serialize here; CPU phases overlap outside it.
std::mutex g_field_gpu_session;
std::atomic<bool> g_field_corridor{false};
}
void set_field_corridor(bool on) { g_field_corridor.store(on); }
bool field_corridor_enabled() { return g_field_corridor.load(); }

int Board::route_net_field_into(int k, Net& net, RouteDeadline deadline) {
    net.segments.clear();
    net.total_length = 0.0;
    net.unconnected_pins = 0;
    if (net.pins.empty() || net.pins.size() == 1) {
        net.routed = true;
        return 1;
    }
    if (!gpu_field_available() || !wave_gpu_enabled()) return -1;

    const int W = grid_.width(), H = grid_.height(), L = grid_.layers();
    constexpr uint32_t BLOCK = 0xFFFFFFFFu;
    constexpr uint64_t INF = ~0ull;

    int px0 = INT_MAX, py0 = INT_MAX, px1 = INT_MIN, py1 = INT_MIN;
    for (const Cell& p : net.pins) {
        px0 = std::min(px0, p.x); py0 = std::min(py0, p.y);
        px1 = std::max(px1, p.x); py1 = std::max(py1, p.y);
    }

    int probe_verdict = -1;   // stage-2 probe cache: board is frozen during
                              // this net's routing, so the verdict can't change
    bool corridor_failed = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
        net.segments.clear();          // escalation restarts the extraction
        net.total_length = 0.0;
        if (deadline_passed(deadline)) { net.routed = false;
            net.unconnected_pins = (int)net.pins.size() - 1; return 0; }
        const int margin = attempt == 0 ? 64 : 512;
        const bool full = attempt == 2;
        int x0 = full ? 0 : px0 - margin, y0 = full ? 0 : py0 - margin;
        int x1 = full ? W - 1 : px1 + margin, y1 = full ? H - 1 : py1 + margin;
        x0 = std::max(0, x0); y0 = std::max(0, y0);
        x1 = std::min(W - 1, x1); y1 = std::min(H - 1, y1);
        const int ww = x1 - x0 + 1, wh = y1 - y0 + 1;
        const size_t wcells = (size_t)L * wh * ww;

        // GPU bake when the exact fast-path preconditions hold (mults 0,
        // gamma 1, base 1): windowed occupancy slices -> on-device cost grid
        // (the field pass then reuses it without re-upload). Falls back to
        // the CPU bake on any failure (VRAM, preconditions, no GPU).
        std::vector<uint32_t> cost;
        std::vector<uint8_t> viaok;
        uint32_t via_q = (uint32_t)std::llround(via_cost_ * 1048576.0);
        bool cost_resident = false;
        {
            FTimer ft(0);
            NetAvoidance av;
            if ((size_t)k < avoid_.size()) av = avoid_[(size_t)k];
            const bool pre = av.pad_avoid_mult <= 0.0 &&
                             av.trace_avoid_mult <= 0.0 &&
                             base_cost_ == 1.0 &&
                             (congest_weight_ <= 0.0 || congest_gamma_ == 1.0) &&
                             via_radius_cells_ > 0.0;
            if (pre && gpu_bake_prealloc(ww, wh, L)) {
                const int32_t tok = net.id + 1;
                viaok = fast_via_mask_window(k, x0, y0, x1, y1);
                auto slice_i32 = [&](const std::vector<int32_t>& src) {
                    std::vector<int32_t> out((size_t)L * wh * ww);
                    for (int l = 0; l < L; ++l)
                        for (int wy = 0; wy < wh; ++wy)
                            std::memcpy(out.data() + ((size_t)l * wh + wy) * ww,
                                        src.data() + grid_.index(l, x0, y0 + wy),
                                        (size_t)ww * 4);
                    return out;
                };
                std::vector<int32_t> so = slice_i32(owner_);
                std::vector<int32_t> sc = slice_i32(cl_zone_);
                std::vector<int32_t> si = slice_i32(pad_inflated_);
                std::vector<int32_t> sp = slice_i32(pad_owner_);
                const size_t plane2 = (size_t)wh * ww;
                std::vector<uint32_t> cg(plane2, 0), sf(plane2, 0);
                uint32_t wq = 0;
                if (congest_weight_ > 0.0 && !congest_cost_.empty()) {
                    wq = (uint32_t)std::llround(congest_weight_ * 1024.0);
                    for (int wy = 0; wy < wh; ++wy)
                        for (int wx = 0; wx < ww; ++wx)
                            cg[(size_t)wy * ww + wx] = (uint32_t)std::llround(
                                congest_cost_[(size_t)(y0 + wy) * W + x0 + wx]
                                * 1024.0);
                    if ((size_t)net.id < congest_edges_.size()) {
                        std::vector<int64_t> df(plane2, 0);
                        for (const auto& e : congest_edges_[(size_t)net.id]) {
                            const double wd = (double)(e[2] - e[0] + 1),
                                         hd = (double)(e[3] - e[1] + 1);
                            const int64_t d = std::llround(
                                quantize_cost(((wd + hd) / (wd * hd)) /
                                              congest_norm_) * 1024.0);
                            const int ex0 = std::max(e[0], x0) - x0;
                            const int ey0 = std::max(e[1], y0) - y0;
                            const int ex1 = std::min(e[2], x1) - x0;
                            const int ey1 = std::min(e[3], y1) - y0;
                            if (ex0 > ex1 || ey0 > ey1) continue;
                            df[(size_t)ey0 * ww + ex0] += d;
                            if (ex1 + 1 < ww) df[(size_t)ey0 * ww + ex1 + 1] -= d;
                            if (ey1 + 1 < wh) df[(size_t)(ey1 + 1) * ww + ex0] -= d;
                            if (ex1 + 1 < ww && ey1 + 1 < wh)
                                df[(size_t)(ey1 + 1) * ww + ex1 + 1] += d;
                        }
                        for (int wy = 0; wy < wh; ++wy)
                            for (int wx = 1; wx < ww; ++wx)
                                df[(size_t)wy * ww + wx] += df[(size_t)wy * ww + wx - 1];
                        for (int wy = 1; wy < wh; ++wy)
                            for (int wx = 0; wx < ww; ++wx)
                                df[(size_t)wy * ww + wx] += df[(size_t)(wy - 1) * ww + wx];
                        for (size_t i = 0; i < plane2; ++i)
                            sf[i] = (uint32_t)std::max<int64_t>(0, df[i]);
                    }
                }
                cost.resize(wcells);
                if (gpu_bake_cost(ww, wh, L, tok, wq, 1048576u, so.data(),
                                  sc.data(), si.data(), sp.data(), cg.data(),
                                  sf.data(), cost.data()))
                    cost_resident = true;
            }
        }
        if (!cost_resident) {
            FTimer ft(0);
            auto baked = bake_net_cost_window(k, x0, y0, x1, y1, true,
                                              viaok.empty() ? nullptr : &viaok);
            cost = std::move(std::get<0>(baked));
            if (viaok.empty()) viaok = std::move(std::get<1>(baked));
            via_q = std::get<2>(baked);
        }
        auto lidx = [&](int l, int gx, int gy) -> size_t {
            return ((size_t)l * wh + (gy - y0)) * ww + (gx - x0);
        };

        // seeds: one label per PIN (label = pin index); unreachable-at-bake
        // pins are simply unconnected (same as A*: their searches exhaust).
        std::vector<int64_t> seeds;
        std::vector<int32_t> seed_label;
        for (size_t pi = 0; pi < net.pins.size(); ++pi) {
            const Cell& p = net.pins[pi];
            const size_t i = lidx(p.layer, p.x, p.y);
            if (cost[i] != BLOCK) {
                seeds.push_back((int64_t)i);
                seed_label.push_back((int32_t)pi);
            }
        }
        if (seeds.empty()) { net.routed = false;
            net.unconnected_pins = (int)net.pins.size() - 1; return 0; }

        // ---- coarse-to-fine corridor (attempt 0, large windows) ----------
        // Coarse grid: pessimistic max-pool blockage/cost, OR-pooled vias,
        // optimistic free disks at terminals. One coarse field from pin 0;
        // descent walks from every pin form the SSSP tree = the corridor
        // skeleton, dilated by the collar. Failures retry attempt 0 plain.
        std::vector<uint32_t> corridor_bits;
        const bool try_corridor = field_corridor_enabled() && attempt == 0 &&
                                  !corridor_failed && wcells > 2000000 &&
                                  net.pins.size() >= 2;
        if (try_corridor) {
            FTimer ft(5);
            const int F = 4, COLLAR = 20;
            const int cw = (ww + F - 1) / F, ch2 = (wh + F - 1) / F;
            const size_t cplane = (size_t)ch2 * cw;
            const size_t ccells = (size_t)L * cplane;
            std::vector<uint32_t> ccost(ccells, 0);
            std::vector<uint8_t> cbad(ccells, 0), cvia(ccells, 0);
            for (int l = 0; l < L; ++l)
                for (int wy = 0; wy < wh; ++wy) {
                    const size_t fro = ((size_t)l * wh + wy) * ww;
                    const size_t cro = (size_t)l * cplane +
                                       (size_t)(wy / F) * cw;
                    for (int wx = 0; wx < ww; ++wx) {
                        const size_t ci = cro + wx / F;
                        const uint32_t fc = cost[fro + wx];
                        if (fc == BLOCK) cbad[ci] = 1;
                        else if (fc > ccost[ci]) ccost[ci] = fc;
                        cvia[ci] |= viaok[fro + wx];
                    }
                }
            for (size_t i = 0; i < ccells; ++i)
                if (cbad[i] || ccost[i] == 0) ccost[i] = cbad[i] ? BLOCK
                                                                : 1048576u;
            const int rterm = COLLAR / F + 1;
            for (const Cell& pcell : net.pins) {
                const int cx = (pcell.x - x0) / F, cy = (pcell.y - y0) / F;
                for (int dy = -rterm; dy <= rterm; ++dy)
                    for (int dx = -rterm; dx <= rterm; ++dx) {
                        const int nx = cx + dx, ny = cy + dy;
                        if (nx < 0 || nx >= cw || ny < 0 || ny >= ch2)
                            continue;
                        for (int l = 0; l < L; ++l) {
                            const size_t ci = (size_t)l * cplane +
                                              (size_t)ny * cw + nx;
                            ccost[ci] = 1048576u;
                            cvia[ci] = 1;
                        }
                    }
            }
            std::vector<int64_t> cseeds;
            {
                const Cell& p0 = net.pins[0];
                for (int l = 0; l < L; ++l)
                    cseeds.push_back((int64_t)((size_t)l * cplane +
                                               (size_t)((p0.y - y0) / F) * cw +
                                               (p0.x - x0) / F));
            }
            std::vector<uint64_t> cdist(ccells);
            if (gpu_cost_field(cw, ch2, L, ccost.data(), cvia.data(), via_q,
                               cseeds.data(), cseeds.size(), cdist.data())) {
                std::vector<uint8_t> cmark(cplane, 0);
                bool all_reached = true;
                for (size_t pi = 1; pi < net.pins.size() && all_reached;
                     ++pi) {
                    const Cell& pp = net.pins[pi];
                    const int64_t st =
                        (int64_t)((size_t)pp.layer * cplane +
                                  (size_t)((pp.y - y0) / F) * cw +
                                  (pp.x - x0) / F);
                    auto path = walk_field_descent(cw, ch2, L, ccost, cvia,
                                                   via_q, cdist, st);
                    if (path.empty()) { all_reached = false; break; }
                    for (int64_t ci : path)
                        cmark[(size_t)ci % cplane] = 1;
                }
                if (all_reached) {
                    for (int64_t ci : cseeds) cmark[(size_t)ci % cplane] = 1;
                    std::vector<uint8_t> tmp;
                    dilate_cheby(cmark, cw, ch2, rterm + 1, tmp);
                    const size_t nwords = (wcells + 31) / 32;
                    corridor_bits.assign(nwords, 0);
                    for (int l = 0; l < L; ++l)
                        for (int wy = 0; wy < wh; ++wy) {
                            const size_t fro = ((size_t)l * wh + wy) * ww;
                            const size_t cro = (size_t)(wy / F) * cw;
                            for (int wx = 0; wx < ww; ++wx)
                                if (cmark[cro + wx / F])
                                    corridor_bits[(fro + wx) >> 5] |=
                                        1u << ((fro + wx) & 31);
                        }
                    cost_resident = false;   // coarse field clobbered fCost
                }
            }
        }

        // ---- field + labels (GPU resident-first; CPU fallback) -----------
        std::vector<uint64_t> dist;         // filled only on the CPU path
        std::vector<uint32_t> gpred;
        std::vector<int32_t> label;
        bool resident = true;               // full on-device tree pipeline
        std::unique_lock<std::mutex> gpu_session(g_field_gpu_session);
        {
            FTimer ft(1);
            if (!gpu_cost_field(ww, wh, L,
                                cost_resident ? nullptr : cost.data(),
                                viaok.data(), via_q, seeds.data(),
                                seeds.size(), nullptr,
                                corridor_bits.empty() ? nullptr
                                                      : corridor_bits.data())) {
                if (getenv("ROUTING_FIELD_DEBUG"))
                    fprintf(stderr, "[field] net %d: cost_field FAIL (%dx%dx%d)\n",
                            k, ww, wh, L);
                return -1;
            }
        }
        {
            FTimer ft(2);
            if (!gpu_field_labels(ww, wh, L, via_q, seeds.data(),
                                  seed_label.data(), seeds.size(), nullptr,
                                  nullptr)) {
                if (getenv("ROUTING_FIELD_DEBUG"))
                    fprintf(stderr, "[field] net %d: labels FAIL\n", k);
                return -1;
            }
        }
        const int plane = wh * ww;
        static const int DXs[8] = {1, -1, 0, 0, 1, -1, 1, -1};
        static const int DYs[8] = {0, 0, 1, -1, 1, -1, -1, 1};
        // ---- basin-boundary pair reduction --------------------------------
        struct PairE { uint64_t w; int32_t la, lb; uint32_t c, n; };
        std::vector<PairE> pairs;
        {
            FTimer ft(3);
            std::vector<std::array<uint32_t, 5>> rows;
            if (gpu_boundary_pairs(ww, wh, L, via_q, rows)) {
                pairs.reserve(rows.size());
                for (auto& r : rows) {
                    if (r[3] == 0xFFFFFFFFu) continue;
                    pairs.push_back({((uint64_t)r[1] << 32) | r[2],
                                     (int32_t)(r[0] >> 16),
                                     (int32_t)(r[0] & 0xFFFFu), r[3], r[4]});
                }
            } else {
                resident = false;
                if (getenv("ROUTING_FIELD_DEBUG"))
                    fprintf(stderr, "[field] net %d: boundary FAIL -> CPU\n", k);
            }
        }
        auto pred_of_cpu = [&](uint32_t i) -> int64_t {
            const uint64_t dc = dist[i];
            const int l = (int)(i / (uint32_t)plane);
            const int rem = (int)(i % (uint32_t)plane);
            const int y = rem / ww, x = rem % ww;
            const uint64_t cc = cost[i];
            for (int d = 0; d < 8; ++d) {
                const int nx = x + DXs[d], ny = y + DYs[d];
                if (nx < 0 || nx >= ww || ny < 0 || ny >= wh) continue;
                const size_t j = (size_t)l * plane + (size_t)ny * ww + nx;
                if (dist[j] == INF) continue;
                const bool dg = d >= 4;
                if (dg) {
                    const size_t ax = (size_t)l * plane + (size_t)y * ww + nx;
                    const size_t ay = (size_t)l * plane + (size_t)ny * ww + x;
                    if (cost[ax] == BLOCK || cost[ay] == BLOCK) continue;
                }
                if (dist[j] + cc * (dg ? 181ull : 128ull) == dc)
                    return (int64_t)j;
            }
            if (viaok[i]) {
                for (int dl : {-1, 1}) {
                    const int nl = l + dl;
                    if (nl < 0 || nl >= L) continue;
                    const size_t j = (size_t)nl * plane + rem;
                    if (dist[j] == INF) continue;
                    if (dist[j] + ((uint64_t)via_q + cc) * 128ull == dc)
                        return (int64_t)j;
                }
            }
            return -1;
        };
        if (!resident) {
            // fetch the resident buffers and rebuild pairs on the CPU
            FTimer ft(3);
            dist.resize(wcells);
            gpred.assign(wcells, 0xFFFFFFFFu);
            label.assign(wcells, -1);
            std::vector<uint32_t> glab(wcells);
            if (!gpu_cost_field(ww, wh, L,
                                cost_resident ? nullptr : cost.data(),
                                viaok.data(), via_q, seeds.data(),
                                seeds.size(), dist.data()))
                return -1;
            if (gpu_field_labels(ww, wh, L, via_q, seeds.data(),
                                 seed_label.data(), seeds.size(), glab.data(),
                                 gpred.data())) {
                for (size_t i = 0; i < wcells; ++i)
                    if (glab[i] != 0xFFFFFFFFu) label[i] = (int32_t)glab[i];
            } else {
                for (size_t s2 = 0; s2 < seeds.size(); ++s2)
                    if (label[(size_t)seeds[s2]] < 0)
                        label[(size_t)seeds[s2]] = seed_label[s2];
                std::vector<uint32_t> order;
                order.reserve(wcells / 4);
                for (uint32_t i = 0; i < (uint32_t)wcells; ++i)
                    if (dist[i] != INF) order.push_back(i);
                std::sort(order.begin(), order.end(),
                          [&](uint32_t a, uint32_t b) {
                              return dist[a] < dist[b];
                          });
                for (uint32_t i : order) {
                    if (label[i] >= 0) continue;
                    const int64_t p = pred_of_cpu(i);
                    if (p >= 0) label[i] = label[(size_t)p];
                }
            }
            // CPU boundary scan with the SAME deterministic (w, i, j) rule
            std::map<std::pair<int32_t, int32_t>, PairE> best;
            static const int FDX[4] = {1, 0, 1, 1};
            static const int FDY[4] = {0, 1, 1, -1};
            for (uint32_t i = 0; i < (uint32_t)wcells; ++i) {
                if (dist[i] == INF) continue;
                const int32_t la = label[i];
                if (la < 0) continue;
                const int l = (int)(i / (uint32_t)plane);
                const int rem = (int)(i % (uint32_t)plane);
                const int y = rem / ww, x = rem % ww;
                auto consider = [&](size_t j, uint64_t st) {
                    const int32_t lb = label[j];
                    if (lb < 0 || lb == la) return;
                    const uint64_t w = dist[i] + dist[j] + st;
                    auto key = std::minmax(la, lb);
                    auto it = best.find({key.first, key.second});
                    if (it == best.end() || w < it->second.w ||
                        (w == it->second.w &&
                         ((uint32_t)i < it->second.c ||
                          ((uint32_t)i == it->second.c &&
                           (uint32_t)j < it->second.n))))
                        best[{key.first, key.second}] =
                            {w, key.first, key.second, (uint32_t)i,
                             (uint32_t)j};
                };
                for (int d = 0; d < 4; ++d) {
                    const int nx = x + FDX[d], ny = y + FDY[d];
                    if (nx < 0 || nx >= ww || ny < 0 || ny >= wh) continue;
                    const size_t j = (size_t)l * plane + (size_t)ny * ww + nx;
                    const bool dg = d >= 2;
                    if (dg) {
                        const size_t ax = (size_t)l * plane + (size_t)y * ww + nx;
                        const size_t ay = (size_t)l * plane + (size_t)ny * ww + x;
                        if (cost[ax] == BLOCK || cost[ay] == BLOCK) continue;
                    }
                    if (cost[j] == BLOCK || dist[j] == INF) continue;
                    consider(j, (uint64_t)cost[j] * (dg ? 181ull : 128ull));
                }
                if (viaok[i] && l + 1 < L) {
                    const size_t j = (size_t)(l + 1) * plane + rem;
                    if (viaok[j] && cost[j] != BLOCK && dist[j] != INF)
                        consider(j, ((uint64_t)via_q + cost[j]) * 128ull);
                }
            }
            pairs.clear();
            for (auto& kv : best) pairs.push_back(kv.second);
        }
        std::sort(pairs.begin(), pairs.end(), [](const PairE& a,
                                                 const PairE& b) {
            if (a.w != b.w) return a.w < b.w;
            if (a.la != b.la) return a.la < b.la;
            return a.lb < b.lb;
        });
        std::vector<int32_t> dsu((size_t)net.pins.size());
        for (size_t i = 0; i < dsu.size(); ++i) dsu[i] = (int32_t)i;
        std::function<int32_t(int32_t)> find = [&](int32_t a) -> int32_t {
            while (dsu[(size_t)a] != a) {
                dsu[(size_t)a] = dsu[(size_t)dsu[(size_t)a]];
                a = dsu[(size_t)a];
            }
            return a;
        };
        std::vector<const PairE*> accepted;
        for (auto& e : pairs) {
            const int32_t ra = find(e.la), rb = find(e.lb);
            if (ra == rb) continue;
            dsu[(size_t)ra] = rb;
            accepted.push_back(&e);
        }
        const int32_t root = find(0);
        auto to_global = [&](uint32_t i) -> Cell {
            const int l = (int)(i / (uint32_t)plane);
            const int rem = (int)(i % (uint32_t)plane);
            return Cell{l, x0 + rem % ww, y0 + rem / ww};
        };
        bool any_fail = false;
        {
            FTimer ftw(4);
            std::vector<const PairE*> rooted;
            for (auto* e : accepted)
                if (find(e->la) == root) rooted.push_back(e);
            if (resident) {
                std::vector<std::pair<uint32_t, uint32_t>> elist;
                elist.reserve(rooted.size());
                for (auto* e : rooted) elist.push_back({e->c, e->n});
                std::vector<uint32_t> pcs, eoff;
                if (!elist.empty() &&
                    !gpu_walk_edges(ww, wh, L, elist, pcs, eoff)) {
                    any_fail = true;
                } else {
                    for (size_t ei = 0; ei + 1 < eoff.size() ||
                                        (eoff.size() == 1 && ei < 0);
                         ++ei) {
                        std::vector<Cell> pa, pb;
                        for (uint32_t t = eoff[ei]; t < eoff[ei + 1]; ++t) {
                            const uint32_t v = pcs[t];
                            if (v & 0x80000000u)
                                pb.push_back(to_global(v & 0x7FFFFFFFu));
                            else
                                pa.push_back(to_global(v & 0x7FFFFFFFu));
                        }
                        std::vector<Cell> seg;
                        seg.reserve(pa.size() + pb.size());
                        for (auto it = pa.rbegin(); it != pa.rend(); ++it)
                            seg.push_back(*it);
                        for (auto& c2 : pb) seg.push_back(c2);
                        if (seg.empty()) { any_fail = true; continue; }
                        net.total_length += path_euclidean_length(seg);
                        net.segments.push_back(std::move(seg));
                    }
                }
            } else {
                std::vector<uint8_t> in_tree(wcells, 0);
                auto predf = [&](uint32_t i) -> int64_t {
                    if (!gpred.empty() && gpred[i] != 0xFFFFFFFFu &&
                        gpred[i] != i)
                        return (int64_t)gpred[i];
                    if (!gpred.empty() && gpred[i] == i) return (int64_t)i;
                    return pred_of_cpu(i);
                };
                auto walk_trunc = [&](uint32_t from) -> std::vector<uint32_t> {
                    std::vector<uint32_t> path;
                    uint32_t cur = from;
                    while (true) {
                        path.push_back(cur);
                        if (in_tree[cur] || dist[cur] == 0) break;
                        const int64_t p = predf(cur);
                        if (p < 0 || p == (int64_t)cur) return {};
                        cur = (uint32_t)p;
                    }
                    return path;
                };
                for (auto* e : rooted) {
                    std::vector<uint32_t> pa = walk_trunc(e->c);
                    std::vector<uint32_t> pb = walk_trunc(e->n);
                    if (pa.empty() || pb.empty()) { any_fail = true; continue; }
                    std::vector<Cell> seg;
                    seg.reserve(pa.size() + pb.size());
                    for (auto it = pa.rbegin(); it != pa.rend(); ++it)
                        seg.push_back(to_global(*it));
                    for (uint32_t i2 : pb) seg.push_back(to_global(i2));
                    for (uint32_t i2 : pa) in_tree[i2] = 1;
                    for (uint32_t i2 : pb) in_tree[i2] = 1;
                    net.total_length += path_euclidean_length(seg);
                    net.segments.push_back(std::move(seg));
                }
            }
        }

        gpu_session.unlock();
        // connectivity accounting relative to pin 0 (A* convention)
        int unconnected = 0;
        for (size_t pi = 1; pi < net.pins.size(); ++pi)
            if (find((int32_t)pi) != root) ++unconnected;
        if (unconnected > 0 && !corridor_bits.empty()) {
            // corridor was too tight: retry this attempt WITHOUT it (the
            // screen's sealed verdicts are not valid under a corridor mask)
            corridor_failed = true;
            --attempt;
            continue;
        }
        if (unconnected > 0 && attempt < 2) {
            // Window-local exact escalation screen: flood each unconnected
            // pin's PASSABLE region (8-conn + vias, a superset of legal
            // moves) with early exit at the window border. Border touched ->
            // an outside detour might connect it: escalate. Exhausted
            // interior -> every full-grid path from this pin lives inside
            // the enclosed region the window already fielded exactly: skip.
            // Handles both unreachable pins AND reachable-but-disconnected
            // components (the v5 full-grid probe cost ~450s on Aleste;
            // open regions here exit at the border almost immediately).
            FTimer ftu(6);
            bool must_escalate = false;
            std::vector<uint8_t> seen(wcells, 0);
            std::vector<uint32_t> stack;
            for (size_t pi = 1; pi < net.pins.size() && !must_escalate; ++pi) {
                if (find((int32_t)pi) == root) continue;
                const Cell& p = net.pins[pi];
                stack.clear();
                for (int l = 0; l < L; ++l) {
                    const size_t i = lidx(l, p.x, p.y);
                    if (cost[i] != BLOCK && !seen[i]) {
                        seen[i] = 1;
                        stack.push_back((uint32_t)i);
                    }
                }
                while (!stack.empty() && !must_escalate) {
                    const uint32_t i = stack.back();
                    stack.pop_back();
                    const int l = (int)(i / (uint32_t)plane);
                    const int rem = (int)(i % (uint32_t)plane);
                    const int y = rem / ww, x = rem % ww;
                    if (x == 0 || x == ww - 1 || y == 0 || y == wh - 1) {
                        // border reached: the pocket may continue outside
                        // (unless the window IS the full grid)
                        if (ww != W || wh != H) { must_escalate = true; break; }
                    }
                    static const int PDX[8] = {1, -1, 0, 0, 1, -1, 1, -1};
                    static const int PDY[8] = {0, 0, 1, -1, 1, -1, -1, 1};
                    for (int d = 0; d < 8; ++d) {
                        const int nx = x + PDX[d], ny = y + PDY[d];
                        if (nx < 0 || nx >= ww || ny < 0 || ny >= wh) continue;
                        const size_t j = (size_t)l * plane + (size_t)ny * ww + nx;
                        if (seen[j] || cost[j] == BLOCK)
                            continue;
                        seen[j] = 1;
                        stack.push_back((uint32_t)j);
                    }
                    for (int dl : {-1, 1}) {
                        const int nl = l + dl;
                        if (nl < 0 || nl >= L) continue;
                        const size_t j = (size_t)nl * plane + rem;
                        if (seen[j] || cost[j] == BLOCK)
                            continue;
                        seen[j] = 1;
                        stack.push_back((uint32_t)j);
                    }
                }
            }
            if (must_escalate) {
                // stage 2: the border test is necessary-but-weak (open
                // regions always touch the border). Ask the exact directed
                // probe whether any unconnected pin can actually REACH the
                // rooted tree before paying an escalation round.
                std::unordered_set<size_t> tset;
                int bb[4] = {INT_MAX, INT_MAX, INT_MIN, INT_MIN};
                auto add_t = [&](const Cell& c) {
                    if (!valid(c)) return;
                    tset.insert(grid_.index(c));
                    bb[0] = std::min(bb[0], c.x); bb[1] = std::min(bb[1], c.y);
                    bb[2] = std::max(bb[2], c.x); bb[3] = std::max(bb[3], c.y);
                };
                add_t(net.pins[0]);
                for (const auto& seg : net.segments)
                    for (const Cell& c : seg) add_t(c);
                std::vector<Cell> probe_seeds;
                for (size_t pi = 1; pi < net.pins.size(); ++pi)
                    if (find((int32_t)pi) != root)
                        probe_seeds.push_back(net.pins[pi]);
                if (probe_verdict < 0) {
                    // scalar probe kept: the verdict-identical bitplane
                    // variant (wave_probe_superset, 692/692 gate) measured
                    // SLOWER here (v16: screen 413s -> 450s) — its per-probe
                    // full-grid via-mask build doesn't amortize. Revisit with
                    // lazy slab-wise mask dilation.
                    NetContext pctx = make_net_context(k);
                    auto is_t = [&](const Cell& c) -> bool {
                        return valid(c) && tset.count(grid_.index(c)) != 0;
                    };
                    probe_verdict = flood_goals_reach_sources(
                                        grid_, probe_seeds, is_t, pctx.enter,
                                        pctx.via_ok, bb)
                                        ? 1 : 0;
                }
                if (probe_verdict == 1)
                    continue;               // genuinely connectable: escalate
            }
            // sealed (or probe-refuted): the windowed partial is final
        }
        if (any_fail) return -1;            // inconsistent walk: fall back
        net.unconnected_pins = unconnected;
        net.routed = (unconnected == 0);
        return net.routed ? 1 : 0;
    }
    return -1;
}

} // namespace routing

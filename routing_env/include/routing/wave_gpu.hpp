#pragma once
// Optional in-process GPU wavefront (optimization plan Stage D integration).
// Vulkan is loaded via dlopen at first use — NO link-time dependency: machines
// without a Vulkan loader (or without a compute device) simply report
// unavailable and the CPU path serves everything (portability contract).
#include <cstddef>
#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace routing {

bool wave_gpu_available();               // lazy init; cached verdict
void set_wave_gpu_enabled(bool on);      // default on (used only if available)
bool wave_gpu_enabled();

// Full flood with wave-index recording on the GPU. Inputs are 32-bit-packed
// planes (S32 words per row): ent (L*H*S32), via (H*S32), seeds (L*H*S32).
// On success fills map_out (L*H*W u16 cells; 0 = unreached, seeds = 1,
// first ring = 2, ...) and returns true. Any failure returns false (caller
// falls back to the CPU flood). Thread-safe (serialized on the device queue).
bool wave_gpu_map(int W, int H, int L, const uint32_t* ent,
                  const uint32_t* via, const uint32_t* seeds,
                  uint16_t* map_out);

// Exact weighted cost field (GPU field router stage 1). cost: per-cell
// enter cost in q20 (2^-20) units (0xFFFFFFFF = blocked), layer-major
// ((l*H + y)*W + x). via_ok: 0/1 per cell (dest-side rule). seeds: cell
// indices with distance 0. On success fills dist_out (L*H*W uint64
// 2^-27 units; ~0 = unreached) with the EXACT
// integer Dijkstra field and returns true. Requires shaderInt64.
bool gpu_cost_field(int W, int H, int L, const uint32_t* cost,
                    const uint8_t* via_ok, uint32_t via_q,
                    const int64_t* seeds, std::size_t n_seeds,
                    uint64_t* dist_out,
                    const uint32_t* corridor_bits = nullptr);
// Warm re-flood: the device dist buffer PERSISTS between calls; adding seeds
// only lowers distances, so iterating from the previous fixpoint converges
// locally (the multi-pin tree-growth pattern: component grows -> add its new
// cells as seeds). Skips cost/via upload and the INF fill — caller guarantees
// same net + same window as the immediately preceding cold call. Result is
// EXACTLY the field of the union seed set.
bool gpu_cost_field_warm(int W, int H, int L, uint32_t via_q,
                         const int64_t* seeds, std::size_t n_seeds,
                         uint64_t* dist_out);
// After gpu_cost_field on the SAME window: compute per-cell descent
// predecessors (fixed neighbor order — identical to walk_field_descent) and
// basin labels (seed i gets seed_labels[i]) by GPU pointer-jumping. Fills
// labels_out and pred_out (both L*H*W u32; 0xFFFFFFFF = unset/unreached).
bool gpu_field_labels(int W, int H, int L, uint32_t via_q,
                      const int64_t* seeds, const int32_t* seed_labels,
                      std::size_t n_seeds, uint32_t* labels_out,
                      uint32_t* pred_out);
// GPU enter-cost bake (stage 3): windowed slices of the board occupancy
// grids (owner / clearance-zone / pad-inflation / pad-owner, layer-major)
// plus 2D congestion and per-net self-demand planes (q10 ints) -> the net's
// cost grid, baked ON DEVICE into the field cost buffer (pass cost=nullptr
// to gpu_cost_field to reuse it) and copied back to cost_out for CPU-side
// boundary checks. Exact only under the fast-path preconditions (avoidance
// mults 0, gamma 1, base 1) — caller guards and falls back to CPU bake.
// Cheap allocation pre-flight for gpu_bake_cost (buffers are persistent, so
// success costs nothing on repeat calls; failure means the caller should
// skip slicing and use the CPU bake).
bool gpu_bake_prealloc(int W, int H, int L);
bool gpu_bake_cost(int W, int H, int L, int32_t tok, uint32_t wq,
                   uint32_t base_q20, const int32_t* own, const int32_t* clz,
                   const int32_t* pin, const int32_t* pad,
                   const uint32_t* cong, const uint32_t* selfq,
                   uint32_t* cost_out);
// On-device boundary reduction: per basin pair, the deterministic minimum
// (w, i, j) connecting edge. out rows: {key = (min<<16|max), w_hi, w_lo,
// cell_i, cell_j}. Requires resident dist/labels from the preceding field +
// labels calls (pass nullptr outputs to keep them on device).
bool gpu_boundary_pairs(int W, int H, int L, uint32_t via_q,
                        std::vector<std::array<uint32_t, 5>>& out);
// Walk MST edges on-device (predO chains, predJ reused as the in-tree mask,
// exact truncation, sequential per edge). Returns concatenated window cell
// indices (bit31 = second endpoint's half) and per-edge offsets.
bool gpu_walk_edges(int W, int H, int L,
                    const std::vector<std::pair<uint32_t, uint32_t>>& edges,
                    std::vector<uint32_t>& path_cells,
                    std::vector<uint32_t>& edge_off);
bool gpu_field_available();

} // namespace routing

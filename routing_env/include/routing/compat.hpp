#pragma once
// Cross-compiler shims for MSVC vs GCC/Clang. The core is C++17 (no <bit>),
// so bit scan/popcount go through __builtin_* on GCC/Clang and <intrin.h>
// intrinsics on MSVC. Also defines M_PI, which MSVC does not provide without
// _USE_MATH_DEFINES. Include before any use of M_PI or the rt_* helpers.
#include <cstdint>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if defined(_MSC_VER)
#include <intrin.h>
inline int rt_clzll(uint64_t x) { unsigned long i; return _BitScanReverse64(&i, x) ? 63 - (int)i : 64; }
inline int rt_ctzll(uint64_t x) { unsigned long i; return _BitScanForward64(&i, x) ? (int)i : 64; }
inline int rt_ctz(uint32_t x)   { unsigned long i; return _BitScanForward(&i, x) ? (int)i : 32; }
inline int rt_popcountll(uint64_t x) { return (int)__popcnt64(x); }
#else
inline int rt_clzll(uint64_t x) { return x ? __builtin_clzll(x) : 64; }
inline int rt_ctzll(uint64_t x) { return x ? __builtin_ctzll(x) : 64; }
inline int rt_ctz(uint32_t x)   { return x ? __builtin_ctz(x) : 32; }
inline int rt_popcountll(uint64_t x) { return __builtin_popcountll(x); }
#endif

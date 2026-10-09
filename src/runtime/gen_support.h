// Support for recompiled i960 code (tools/m2recomp output). Generated code is
// native C++: every instruction's semantics are emitted inline with operands
// resolved at recompile time. It calls into the runtime only for memory
// access, call/return frame management and interrupt entry, as a CPU would.
#pragma once
#include <limits>

#include "runtime/cpu.h"
#include "runtime/enhance.h"
#include "runtime/lockstep.h"

#include <bit>
#include <cmath>
#include <cstdint>

namespace gen {

struct Env {
    rt::Cpu &c;
    rt::Lockstep &ls;
};

// Condition codes, as MAME's cmp_u / cmp_s / concmp_u / concmp_s.
inline uint32_t cc_u(uint32_t a, uint32_t b) { return a < b ? 4u : a == b ? 2u : 1u; }
inline uint32_t cc_s(int32_t a, int32_t b) { return a < b ? 4u : a == b ? 2u : 1u; }
inline uint32_t cc_d(double a, double b) { return a < b ? 4u : a == b ? 2u : a > b ? 1u : 0u; }

// FP with MAME's host-double semantics (lockstep against MAME). The shipped
// path will use the proven native paths in i960/fp.h instead.
inline double round_to_int(double v, uint32_t ac) {
    switch ((ac >> 30) & 3) {
    case 0: return std::round(v);
    case 1: return std::floor(v);
    case 2: return std::ceil(v);
    default: return std::trunc(v);
    }
}
// MAME's s32(double) on its x86-64 build (cvttsd2si): NaN and out-of-range
// give 0x80000000. A plain cast is undefined there and ARM64 saturates (NaN
// to 0).
inline uint32_t d2i(double v) {
    if (!(v >= -2147483648.0 && v < 2147483648.0)) return 0x80000000u;
    return uint32_t(int32_t(v));
}
inline uint32_t f2u(float f) { return std::bit_cast<uint32_t>(f); }
inline float u2f(uint32_t u) { return std::bit_cast<float>(u); }
inline uint64_t d2u(double d) { return std::bit_cast<uint64_t>(d); }
inline double u2d(uint64_t u) { return std::bit_cast<double>(u); }

// (int64_t)x as MAME's x86-64 build computes it: out of range or NaN gives
// 0x8000000000000000 (cvttsd2si); ARM64 would saturate.
inline int64_t d2i64(double x) {
    if (!(x >= -9223372036854775808.0 && x < 9223372036854775808.0)) return INT64_MIN;
    return int64_t(x);
}

// i960 extended real (movre), as src/refcore/i960_ref.cpp (MAME i960.cpp):
// words 0/1 the 64-bit fraction with an explicit integer bit, word 2 the sign
// (bit 15) and the 15-bit biased exponent.
inline void double_to_extended(double val, uint32_t *words) {
    const uint64_t bits = d2u(val);
    const uint32_t sign = uint32_t(bits >> 63) << 15;
    uint64_t frac;
    uint32_t e;
    if (std::isnan(val) || std::isinf(val)) {
        frac = (1ULL << 63) | ((bits & 0x000fffffffffffffULL) << 11);
        e = 0x7fff;
    } else if (val == 0.0) {
        frac = 0;
        e = 0;
    } else {
        int exp2;
        const double m = std::frexp(std::fabs(val), &exp2);
        frac = uint64_t(std::ldexp(m, 64));
        e = uint32_t(exp2 - 1 + 16383);
    }
    words[0] = uint32_t(frac);
    words[1] = uint32_t(frac >> 32);
    words[2] = sign | e;
}

inline double extended_to_double(const uint32_t *words) {
    const uint64_t frac = words[0] | (uint64_t(words[1]) << 32);
    const uint32_t e = words[2] & 0x7fff;
    double val;
    if (e == 0x7fff) {
        if (!(frac << 1)) val = std::numeric_limits<double>::infinity();
        else val = u2d(0x7ff8000000000000ULL | ((frac & 0x7fffffffffffffffULL) >> 11));
    } else {
        val = std::ldexp(double(frac), int(e ? e : 1) - 16383 - 63);
    }
    return (words[2] >> 15 & 1) ? -val : val;
}

// Recompiled code entry points (generated): run from c.m_IP until it leaves
// recompiled code or the lockstep run ends.
bool has_code(uint32_t addr);   // generated
void run(Env &e);               // generated: dispatches into the chunk holding c.m_IP
uint64_t native_instructions(); // generated: instructions recompiled (all of them: there is no fallback)

} // namespace gen

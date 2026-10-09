// The few MAME helpers the transplanted SHARC sources use (emu.h stand-ins).
#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace rt::sharc {

using offs_t = uint32_t;
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;

// MAME emucore.h BIT(), named sharc_bit: libnx has a BIT macro of its own.
// sharc.cpp maps BIT to it for the transplanted files.
template <typename T, typename U> constexpr T sharc_bit(T x, U n) noexcept { return (x >> n) & T(1); }
template <typename T, typename U, typename V> constexpr T sharc_bit(T x, U n, V w) {
    return (x >> n) & ((T(1) << w) - 1);
}

namespace util {
// MAME util/coretmpl.h
template <typename T, typename U> constexpr T make_bitmask(U n) {
    return T((n < (8 * sizeof(T)) ? (std::make_unsigned_t<T>(1) << n) : std::make_unsigned_t<T>(0)) - 1);
}
template <typename T, typename U> constexpr std::make_signed_t<T> sext(T value, U width) {
    return std::make_signed_t<T>(value << (8 * sizeof(T) - width)) >> (8 * sizeof(T) - width);
}
} // namespace util

class emu_fatalerror : public std::runtime_error {
public:
    template <typename... A> explicit emu_fatalerror(const char *fmt, A... args) : std::runtime_error(format(fmt, args...)) {}
private:
    template <typename... A> static std::string format(const char *fmt, A... args) {
        char b[512];
        if constexpr (sizeof...(A) == 0) std::snprintf(b, sizeof b, "%s", fmt);
        else std::snprintf(b, sizeof b, fmt, args...);
        return b;
    }
};
#define fatalerror(...) throw ::rt::sharc::emu_fatalerror(__VA_ARGS__)
#define LOG(...) do {} while (0)
#define TIMER_CALLBACK_MEMBER(name) void name(int param)

#define ATTR_COLD

// MAME eminline.h
constexpr uint32_t rotl_32(uint32_t v, int s) { s &= 31; return s ? (v << s) | (v >> (32 - s)) : v; }
constexpr uint32_t rotr_32(uint32_t v, int s) { s &= 31; return s ? (v >> s) | (v << (32 - s)) : v; }
constexpr int64_t mul_32x32(int32_t a, int32_t b) { return int64_t(a) * int64_t(b); }
constexpr uint64_t mulu_32x32(uint32_t a, uint32_t b) { return uint64_t(a) * uint64_t(b); }
constexpr int32_t mul_32x32_shift(int32_t a, int32_t b, uint8_t shift) { return int32_t((int64_t(a) * int64_t(b)) >> shift); }

// DMA timers only exist for MAME's DRC (m_enable_drc is false here).
struct attotime { static constexpr int never = 0; };
struct emu_timer { template <typename... A> void adjust(A...) {} };
inline int cycles_to_attotime(int c) { return c; }

} // namespace rt::sharc

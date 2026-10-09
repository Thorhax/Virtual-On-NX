// license:BSD-3-Clause
// copyright-holders:R. Belmont, Olivier Galibert, ElSemi, Angelo Salese
//
// Model 2B coprocessor ports, from MAME's src/mame/sega/model2.cpp
// (model2b_state: machine_reset, copro_function_port_w, copro_fifo_r/w,
// copro_ctl1_w, copro_status_r, fifo_control_r, copro_sharc_map, the FIFO
// flag hookup; BSD-3-Clause, notice above kept). MAME's device FIFOs and
// scheduler are replaced by running the SHARC on demand. See THIRD_PARTY.md.

#include "runtime/m2_sharc_board.h"

#include <cstring>

namespace rt {

SharcBoard::SharcBoard(const std::vector<uint8_t> &copro_data, uint32_t *buffer) : buffer_(buffer) {
    copro_data_.resize(copro_data.size() / 4);
    std::memcpy(copro_data_.data(), copro_data.data(), copro_data_.size() * 4);
    if (copro_data_.empty()) copro_data_.assign(1, 0);
    sharc_.ext_dm_read = [this](uint32_t a) { return ext_read(a); };
    sharc_.ext_dm_write = [this](uint32_t a, uint32_t v) { ext_write(a, v); };
    reset();
}

void SharcBoard::reset() {
    sharc_.reset();
    sharc_.set_halt(true);      // held until the program is uploaded
    in_.clear();
    out_.clear();
    coproctl_ = coprocnt_ = 0;
    starved_ = false;
    since_bus_ = 0;
    update_flags();
}

void SharcBoard::update_flags() {
    sharc_.set_flag_input(0, in_.empty() ? 1 : 0); // FIFOIN empty
    sharc_.set_flag_input(1, 0);                    // FIFOOUT full: never (unbounded here)
}

// copro_sharc_map
uint32_t SharcBoard::ext_read(uint32_t a) {
    since_bus_ = 0;
    if (a >= 0x0400000 && a <= 0x0bfffff) { // FIFO in: an empty pop reads 0 (MAME's generic FIFO)
        if (in_.empty()) return 0;
        const uint32_t v = in_.front();
        in_.pop_front();
        update_flags();
        return v;
    }
    if (a >= 0x1400000 && a <= 0x1bfffff) return buffer_[a & 0x7fff];
    if (a >= 0x1c00000 && a <= 0x1dfffff) return copro_data_[(a - 0x1c00000) % copro_data_.size()];
    return 0;
}

void SharcBoard::ext_write(uint32_t a, uint32_t v) {
    since_bus_ = 0;
    if (a >= 0x0c00000 && a <= 0x13fffff) { out_.push_back(v); return; }
    if (a >= 0x1400000 && a <= 0x1bfffff) { buffer_[a & 0x7fff] = v; return; }
}

void SharcBoard::run(uint64_t limit, bool until_output) {
    if (sharc_.halted()) return;
    uint64_t done = 0;
    while (done < limit) {
        if (until_output && !out_.empty()) return;
        const int step = 64;
        const uint64_t before = sharc_.instructions();
        sharc_.run(step);
        const uint64_t ran = sharc_.instructions() - before;
        done += ran ? ran : step;
        since_bus_ += ran;
        if (!until_output && in_.empty() && since_bus_ >= kIdleRun) { starved_ = true; return; }
        if (sharc_.idle() && ran == 0) { starved_ = true; return; }
    }
}

void SharcBoard::sync() {
    if (!starved_) run(kOutputLimit, false);
}

void SharcBoard::coproctl_w(uint32_t data, uint32_t mask) {
    if ((data ^ coproctl_) == 0x80000000) {
        if (data & 0x80000000) {
            coprocnt_ = 0; // start upload (copro_halt: nothing on 2B)
        } else {
            sharc_.set_halt(false); // copro_boot
            starved_ = false;
        }
    }
    coproctl_ = (coproctl_ & ~mask) | (data & mask);
}

void SharcBoard::function_port_w(uint32_t offset, uint32_t data) {
    uint32_t d = data & 0x800fffff;
    const uint32_t a = (offset >> 2) & 0xff;
    d |= a << 23;
    in_.push_back(d);
    starved_ = false;
    update_flags();
}

void SharcBoard::fifo_w(uint32_t data) {
    if (coproctl_ & 0x80000000) {
        sharc_.external_dma_write(coprocnt_, data & 0xffff);
        coprocnt_++;
        return;
    }
    in_.push_back(data);
    starved_ = false;
    update_flags();
}

uint32_t SharcBoard::fifo_r() {
    if (out_.empty()) {
        starved_ = false;
        run(kOutputLimit, true); // the i960 waits for it (MAME stalls the i960)
        if (out_.empty()) throw SharcFatal("the i960 read the SHARC's output FIFO, but the SHARC produced nothing");
    }
    const uint32_t v = out_.front();
    out_.pop_front();
    return v;
}

bool SharcBoard::fifo_out_empty() {
    if (out_.empty()) sync();
    return out_.empty();
}

void SharcBoard::iop_w(uint32_t offset, uint32_t data) {
    sharc_.external_iop_write(offset, data);
    starved_ = false;
}

} // namespace rt

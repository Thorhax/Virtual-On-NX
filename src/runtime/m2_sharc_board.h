// The i960's side of the Model 2B coprocessor: the ADSP-21062 SHARC's FIFOs,
// control and status registers, IOP window and program upload (MAME
// model2.cpp: model2b_state copro_*, copro_sharc_map). Buffer RAM is the
// geometrizer's (TgpBoard), shared. The SHARC runs clockless, only when the
// i960 needs its output: it pauses once its input FIFO is empty and it has
// gone kIdleRun instructions without touching the external bus (its FLAG0
// poll loop).
#pragma once

#include "runtime/sharc/sharc.h"

#include <cstdint>
#include <deque>
#include <stdexcept>
#include <vector>

namespace rt {

struct SharcFatal : std::runtime_error {
    using std::runtime_error::runtime_error;
};

class SharcBoard {
public:
    // copro_data: 0x800000 bytes (the SHARC reads it at 0x1c00000); buffer:
    // buffer RAM (0x8000 dwords), owned by the TgpBoard.
    SharcBoard(const std::vector<uint8_t> &copro_data, uint32_t *buffer);

    void reset();                                        // MAME model2b_state::machine_reset
    void function_port_w(uint32_t offset, uint32_t data); // 0x00880000 (dword offset)
    void fifo_w(uint32_t data);                          // 0x00884000
    uint32_t fifo_r();                                   // 0x00884000
    bool fifo_out_empty();                               // 0x00980004 fifo_control_r
    void coproctl_w(uint32_t data, uint32_t mask);       // 0x00980000
    uint32_t coproctl_r() const { return coproctl_; }
    uint32_t status_r() const { return coprocnt_ == 0 ? 0xffffffffu : 0; } // 0x00980014
    void iop_w(uint32_t offset, uint32_t data);           // 0x008c0000 (dword offset)
    // The game polls buffer RAM for the SHARC's results: let it catch up.
    void sync();
    uint64_t instructions() const { return sharc_.instructions(); }

private:
    void run(uint64_t limit, bool until_output);
    uint32_t ext_read(uint32_t a);
    void ext_write(uint32_t a, uint32_t v);
    void update_flags();

    static constexpr uint64_t kIdleRun = 256;
    static constexpr uint64_t kOutputLimit = 50'000'000; // a SHARC that never answers is a fault
    sharc::adsp21062_device sharc_;
    std::vector<uint32_t> copro_data_;
    uint32_t *buffer_;
    std::deque<uint32_t> in_, out_;
    uint32_t coproctl_ = 0, coprocnt_ = 0;
    uint64_t since_bus_ = 0;  // instructions since the SHARC last touched the external bus
    bool starved_ = false;    // paused on an empty input FIFO; nothing new since
};

} // namespace rt

// license:BSD-3-Clause
// copyright-holders:Ville Linde
//
// Analog Devices ADSP-21062 SHARC: MAME's interpreter (src/devices/cpu/sharc,
// sharc.h/sharc.cpp, sharcops.hxx, compute.hxx, sharcdma.hxx,
// sharcinternal.ipp, sharcops_table.cpp), lifted out of MAME's device
// framework. The opcode, compute and DMA files are MAME's, unmodified; this
// header and sharc.cpp replace the device, address spaces and timers (see
// THIRD_PARTY.md). The DRC is not included.
#pragma once

#include "runtime/sharc/mame_shim.h"

#include <cstdint>
#include <functional>
#include <string>
#include <memory>
#include <utility>

namespace rt::sharc {

class adsp21062_device
{
public:
	enum sharc_boot_mode { BOOT_MODE_EPROM, BOOT_MODE_HOST, BOOT_MODE_LINK, BOOT_MODE_NOBOOT };

	adsp21062_device();
	~adsp21062_device();

	// The external data bus (addresses at or above 0x400000 in data space).
	std::function<uint32_t(uint32_t)> ext_dm_read;
	std::function<void(uint32_t, uint32_t)> ext_dm_write;
	// FLAG outputs (MAME's flag_out callbacks).
	std::function<void(int)> m_flag_out_cb[4];

	void reset();                       // device_reset (boot mode host)
	int run(int cycles);                // execute_run for up to `cycles`; returns cycles used
	void stop_run() { m_stop = true; }  // from a bus callback: end run() after this instruction
	void set_halt(bool on) { m_halted = on; }
	bool halted() const { return m_halted; }
	uint64_t instructions() const { return m_instructions; }
	uint32_t pc() const;
	bool idle() const;

	void external_iop_write(offs_t address, uint32_t data);
	void external_dma_write(offs_t address, uint64_t data);
	void set_flag_input(int flag_num, int state);
	void set_input_line(int irqline, int state); // execute_set_input
	void write_stall(int state);

	uint32_t iop_r(offs_t offset);
	void iop_w(offs_t offset, uint32_t data);

	enum ASTAT_SHIFT
	{
		AZ_SHIFT = 0,
		AV_SHIFT = 1,
		AN_SHIFT = 2,
		AC_SHIFT = 3,
		AS_SHIFT = 4,
		AI_SHIFT = 5,
		MN_SHIFT = 6,
		MV_SHIFT = 7,
		MU_SHIFT = 8,
		MI_SHIFT = 9,
		AF_SHIFT = 10,
		SV_SHIFT = 11,
		SZ_SHIFT = 12,
		SS_SHIFT = 13,
		BTF_SHIFT = 18,
		FLG0_SHIFT = 19,
		FLG1_SHIFT = 20,
		FLG2_SHIFT = 21,
		FLG3_SHIFT = 22
	};

	enum MODE1_DELAY_MODE
	{
		MODE1_WRITE_IMM,
		MODE1_WRITE_REG,
		MODE1_SET,
		MODE1_CLEAR,
		MODE1_TOGGLE,
	};

	enum
	{
		EXCEPTION_INTERRUPT = 0,
		EXCEPTION_COUNT
	};


	const char *tag() const { return "sharc"; }
	static std::string disassemble_one(uint32_t, uint64_t) { return "(no disassembler)"; }
	void build_opcode_table();
	// ASTAT flags
	static constexpr uint32_t AZ =              0x0000'0001;    // ALU result zero
	static constexpr uint32_t AV =              0x0000'0002;    // ALU overflow
	static constexpr uint32_t AN =              0x0000'0004;    // ALU result negative
	static constexpr uint32_t AC =              0x0000'0008;    // ALU fixed-point carry
	static constexpr uint32_t AS =              0x0000'0010;    // ALU X input sign
	static constexpr uint32_t AI =              0x0000'0020;    // ALU floating-point invalid operation
	static constexpr uint32_t MN =              0x0000'0040;    // Multiplier result negative
	static constexpr uint32_t MV =              0x0000'0080;    // Multiplier overflow
	static constexpr uint32_t MU =              0x0000'0100;    // Multiplier underflow
	static constexpr uint32_t MI =              0x0000'0200;    // Multiplier floating-point invalid operation
	static constexpr uint32_t AF =              0x0000'0400;    // Last ALU operation was a floating-point operation
	static constexpr uint32_t SV =              0x0000'0800;    // Shifter overflow
	static constexpr uint32_t SZ =              0x0000'1000;    // Shifter result zero
	static constexpr uint32_t SS =              0x0000'2000;    // Shifter input sign
	static constexpr uint32_t BTF =             0x0004'0000;    // Bit Test Flag
	static constexpr uint32_t FLG0 =            0x0008'0000;    // FLAG0
	static constexpr uint32_t FLG1 =            0x0010'0000;    // FLAG1
	static constexpr uint32_t FLG2 =            0x0020'0000;    // FLAG2
	static constexpr uint32_t FLG3 =            0x0040'0000;    // FLAG3

	// STKY flags
	static constexpr uint32_t AUS =             0x0000'0001;    // ALU floating-point underflow
	static constexpr uint32_t AVS =             0x0000'0002;    // ALU floating-point overflow
	static constexpr uint32_t AOS =             0x0000'0004;    // ALU fixed-point overflow
	static constexpr uint32_t AIS =             0x0000'0020;    // ALU floating-point invalid operation
	static constexpr uint32_t MOS =             0x0000'0040;    // Multiplier fixed-point overflow
	static constexpr uint32_t MVS =             0x0000'0080;    // Multiplier floating-point overflow
	static constexpr uint32_t MUS =             0x0000'0100;    // Multiplier underflow
	static constexpr uint32_t MIS =             0x0000'0200;    // Multiplier floating-point invalid operation
	static constexpr uint32_t PCFL =            0x0020'0000;    // PC stack full
	static constexpr uint32_t PCEM =            0x0040'0000;    // PC stack empty
	static constexpr uint32_t SSOV =            0x0080'0000;    // Status stack overflow
	static constexpr uint32_t SSEM =            0x0100'0000;    // Status stack empty
	static constexpr uint32_t LSOV =            0x0200'0000;    // Loop stacks overflow
	static constexpr uint32_t LSEM =            0x0400'0000;    // Loop stacks empty

	// MODE1 flags
	static constexpr uint32_t MODE1_BR8 =       0x0000'0001;    // Bit-reverse for I8
	static constexpr uint32_t MODE1_BR0 =       0x0000'0002;    // Bit-reverse for I0
	static constexpr uint32_t MODE1_SRCU =      0x0000'0004;    // Alternate register select for computational units
	static constexpr uint32_t MODE1_SRD1H =     0x0000'0008;    // DAG alternate register select (7-4)
	static constexpr uint32_t MODE1_SRD1L =     0x0000'0010;    // DAG alternate register select (3-0)
	static constexpr uint32_t MODE1_SRD2H =     0x0000'0020;    // DAG alternate register select (15-12)
	static constexpr uint32_t MODE1_SRD2L =     0x0000'0040;    // DAG alternate register select (11-8)
	static constexpr uint32_t MODE1_SRRFH =     0x0000'0080;    // Register file alternate select for R(15-8)
	static constexpr uint32_t MODE1_SRRFL =     0x0000'0400;    // Register file alternate select for R(7-0)
	static constexpr uint32_t MODE1_NESTM =     0x0000'0800;    // Interrupt nesting enable
	static constexpr uint32_t MODE1_IRPTEN =    0x0000'1000;    // Global interrupt enable
	static constexpr uint32_t MODE1_ALUSAT =    0x0000'2000;    // Enable ALU fixed-point saturation
	static constexpr uint32_t MODE1_SSE =       0x0000'4000;    // Enable short word sign extension
	static constexpr uint32_t MODE1_TRUNCATE =  0x0000'8000;    // (1) Floating-point truncation / (0) round to nearest
	static constexpr uint32_t MODE1_RND32 =     0x0001'0000;    // (1) 32-bit floating-point rounding / (0) 40-bit rounding
	static constexpr uint32_t MODE1_CSEL =      0x0006'0000;    // CSelect

	// MODE2 flags
	static constexpr uint32_t MODE2_IRQ0E =     0x0000'0001;    // IRQ0 (1) Edge sens. / (0) Level sens.
	static constexpr uint32_t MODE2_IRQ1E =     0x0000'0002;    // IRQ1 (1) Edge sens. / (0) Level sens.
	static constexpr uint32_t MODE2_IRQ2E =     0x0000'0004;    // IRQ2 (1) Edge sens. / (0) Level sens.
	static constexpr uint32_t MODE2_CADIS =     0x0000'0010;    // Cache disable
	static constexpr uint32_t MODE2_TIMEN =     0x0000'0020;    // Timer enable
	static constexpr uint32_t MODE2_BUSLK =     0x0000'0040;    // External bus lock
	static constexpr uint32_t MODE2_FLG0O =     0x0000'8000;    // FLAG0 (1) Output / (0) Input
	static constexpr uint32_t MODE2_FLG1O =     0x0001'0000;    // FLAG1 (1) Output / (0) Input
	static constexpr uint32_t MODE2_FLG2O =     0x0002'0000;    // FLAG2 (1) Output / (0) Input
	static constexpr uint32_t MODE2_FLG3O =     0x0004'0000;    // FLAG3 (1) Output / (0) Input
	static constexpr uint32_t MODE2_CAFRZ =     0x0008'0000;    // Cache freeze


	struct cfuncs;

	using opcode_func = void (adsp21062_device::*)();
	struct SHARC_OP
	{
		uint32_t op_mask;
		uint32_t op_bits;
		opcode_func handler;
	};


	union SHARC_REG
	{
		int32_t r;
		float f;
	};

	struct SHARC_DMA_REGS
	{
		uint32_t control;
		uint32_t int_index;
		uint32_t int_modifier;
		uint32_t int_count;
		uint32_t chain_ptr;
		uint32_t gen_purpose;
		uint32_t ext_index;
		uint32_t ext_modifier;
		uint32_t ext_count;
	};

	struct SHARC_DMA_OP
	{
		uint32_t src;
		uint32_t dst;
		uint32_t chain_ptr;
		int32_t src_modifier;
		int32_t dst_modifier;
		int32_t src_count;
		int32_t dst_count;
		int32_t pmode;
		int32_t chained_direction;
		emu_timer *timer;
		bool active;
		bool chained;
	};

	struct alignas(16) sharc_internal_state;


	static const SHARC_OP s_sharc_opcode_table[];
	static const size_t s_num_ops;

	static const uint32_t recips_mantissa_lookup[128];
	static const uint32_t rsqrts_mantissa_lookup[128];



	sharc_internal_state *m_core;


	// UML stuff





	opcode_func m_sharc_op[512];




	inline void CHANGE_PC(uint32_t newpc);
	inline void CHANGE_PC_DELAYED(uint32_t newpc);
	void sharc_iop_delayed_w(uint32_t reg, uint32_t data, int cycles);
	uint32_t pm_read32(uint32_t address);
	void pm_write32(uint32_t address, uint32_t data);
	uint64_t pm_read48(uint32_t address);
	void pm_write48(uint32_t address, uint64_t data);
	uint32_t dm_read32(uint32_t address);
	void dm_write32(uint32_t address, uint32_t data);
	void schedule_chained_dma_op(int channel, uint32_t dma_chain_ptr, int chained_direction);
	void schedule_dma_op(int channel, uint32_t src, uint32_t dst, int src_modifier, int dst_modifier, int src_count, int dst_count, int pmode);
	void dma_op(int channel);
	void sharc_dma_exec(int channel);
	void dma_run_cycle(int channel);
	void add_systemreg_write_latency_effect(int sysreg, uint32_t data, uint32_t prev_data);
	void systemreg_write_latency_effect();
	uint32_t GET_UREG(int ureg);
	void SET_UREG(int ureg, uint32_t data);
	void SHIFT_OPERATION_IMM(int shiftop, int data, int rn, int rx);
	void COMPUTE(uint32_t opcode);
	void check_interrupts();
	void PUSH_PC();
	uint32_t POP_PC();
	uint32_t TOP_PC();
	void PUSH_LOOP();
	void POP_LOOP();
	void PUSH_STATUS_STACK();
	void POP_STATUS_STACK();
	int IF_CONDITION_CODE(int cond);
	int DO_CONDITION_CODE(int cond);
	void sharcop_compute_dreg_dm_dreg_pm();
	void sharcop_compute();
	void sharcop_compute_ureg_dmpm_premod();
	void sharcop_compute_ureg_dmpm_postmod();
	void sharcop_compute_dm_to_dreg_immmod();
	void sharcop_compute_dreg_to_dm_immmod();
	void sharcop_compute_pm_to_dreg_immmod();
	void sharcop_compute_dreg_to_pm_immmod();
	void sharcop_compute_ureg_to_ureg();
	void sharcop_imm_shift_dreg_dmpm();
	void sharcop_imm_shift();
	void sharcop_compute_modify();
	void sharcop_direct_call();
	void sharcop_direct_jump();
	void sharcop_relative_call();
	void sharcop_relative_jump();
	void sharcop_indirect_jump();
	void sharcop_indirect_call();
	void sharcop_relative_jump_compute();
	void sharcop_relative_call_compute();
	void sharcop_indirect_jump_compute_dreg_dm();
	void sharcop_relative_jump_compute_dreg_dm();
	void sharcop_rts();
	void sharcop_rti();
	void sharcop_do_until_counter_imm();
	void sharcop_do_until_counter_ureg();
	void sharcop_do_until();
	void sharcop_dm_to_ureg_direct();
	void sharcop_ureg_to_dm_direct();
	void sharcop_pm_to_ureg_direct();
	void sharcop_ureg_to_pm_direct();
	void sharcop_dm_to_ureg_indirect();
	void sharcop_ureg_to_dm_indirect();
	void sharcop_pm_to_ureg_indirect();
	void sharcop_ureg_to_pm_indirect();
	void sharcop_imm_to_dmpm();
	void sharcop_imm_to_ureg();
	void sharcop_sysreg_bitop();
	void sharcop_modify();
	void sharcop_bit_reverse();
	void sharcop_push_pop_stacks();
	void sharcop_nop();
	void sharcop_idle();
	[[noreturn]] void sharcop_unimplemented() ATTR_COLD;

	// ALU helpers
	inline SHARC_REG FADD(int fx, int fy);
	inline SHARC_REG FSUB(int fx, int fy);
	inline SHARC_REG FAVG(int fx, int fy);
	inline SHARC_REG FABS(int fx);
	inline SHARC_REG FMIN(int fx, int fy);
	inline SHARC_REG FMAX(int fx, int fy);
	inline std::pair<SHARC_REG, SHARC_REG> FADD_FSUB(int fx, int fy);
	inline uint32_t SCALB(SHARC_REG fx, int ry);

	// Multiplier helpers
	inline SHARC_REG FMUL(int fx, int fy);

	// ALU fixed-point
	inline void compute_add(int rn, int rx, int ry);
	inline void compute_sub(int rn, int rx, int ry);
	inline void compute_add_ci(int rn, int rx, int ry);
	inline void compute_sub_ci(int rn, int rx, int ry);
	inline void compute_comp(int rx, int ry);
	inline void compute_add_ci(int rn, int rx);
	inline void compute_sub_ci(int rn, int rx);
	inline void compute_inc(int rn, int rx);
	inline void compute_dec(int rn, int rx);
	inline void compute_neg(int rn, int rx);
	inline void compute_abs(int rn, int rx);
	inline void compute_pass(int rn, int rx);
	inline void compute_and(int rn, int rx, int ry);
	inline void compute_or(int rn, int rx, int ry);
	inline void compute_xor(int rn, int rx, int ry);
	inline void compute_not(int rn, int rx);
	inline void compute_min(int rn, int rx, int ry);
	inline void compute_max(int rn, int rx, int ry);
	inline void compute_clip(int rn, int rx, int ry);

	// ALU floating-point
	inline void compute_fadd(int fn, int fx, int fy);
	inline void compute_fsub(int fn, int fx, int fy);
	inline void compute_fadd_abs(int fn, int fx, int fy);
	inline void compute_fsub_abs(int fn, int fx, int fy);
	inline void compute_favg(int fn, int fx, int fy);
	inline void compute_fcomp(int fx, int fy);
	inline void compute_fneg(int fn, int fx);
	inline void compute_fabs(int fn, int fx);
	inline void compute_fpass(int fn, int fx);
	inline void compute_scalb(int fn, int fx, int ry);
	inline void compute_logb(int rn, int fx);
	inline void compute_fix_scaled(int rn, int fx, int ry);
	inline void compute_fix(int rn, int fx);
	inline void compute_float_scaled(int fn, int rx, int ry);
	inline void compute_float(int fn, int rx);
	inline void compute_recips(int fn, int fx);
	inline void compute_rsqrts(int fn, int fx);
	inline void compute_fcopysign(int fn, int fx, int fy);
	inline void compute_fmin(int fn, int fx, int fy);
	inline void compute_fmax(int fn, int fx, int fy);
	inline void compute_fclip(int fn, int fx, int fy);

	// Multiplier
	inline void compute_mul_uuin(int rn, int rx, int ry);
	inline void compute_mul_ssin(int rn, int rx, int ry);
	inline uint32_t compute_mrf_plus_mul_ssin(int rx, int ry);
	inline uint32_t compute_mrb_plus_mul_ssin(int rx, int ry);
	inline void compute_fmul(int fn, int fx, int fy);

	// Dual add/subtract
	inline void compute_dual_add_sub(int ra, int rs, int rx, int ry);
	inline void compute_dual_fadd_fsub(int fa, int fs, int fx, int fy);

	// Fixed-Point multiply/accumulate and add, subtract or average
	inline void compute_mul_ssfr_add(int rm, int rxm, int rym, int ra, int rxa, int rya);
	inline void compute_mul_ssfr_sub(int rm, int rxm, int rym, int ra, int rxa, int rya);

	// Floating-Point multiplication and ALU operation
	inline void compute_fmul_fadd(int fm, int fxm, int fym, int fa, int fxa, int fya);
	inline void compute_fmul_fsub(int fm, int fxm, int fym, int fa, int fxa, int fya);
	inline void compute_fmul_float_scaled(int fm, int fxm, int fym, int fa, int rxa, int rya);
	inline void compute_fmul_fix_scaled(int fm, int fxm, int fym, int ra, int fxa, int rya);
	inline void compute_fmul_favg(int fm, int fxm, int fym, int fa, int fxa, int fya);
	inline void compute_fmul_fabs(int fm, int fxm, int fym, int fa, int fxa);
	inline void compute_fmul_fmax(int fm, int fxm, int fym, int fa, int fxa, int fya);
	inline void compute_fmul_fmin(int fm, int fxm, int fym, int fa, int fxa, int fya);

	// Multiplication and dual add/subtract
	inline void compute_fmul_dual_fadd_fsub(int fm, int fxm, int fym, int fa, int fs, int fxa, int fya);

	inline void compute_multi_mr_to_reg(int ai, int rk);
	inline void compute_multi_reg_to_mr(int ai, int rk);

	// Bus (MAME's pgm_2m / data_2m maps for the ADSP-21062).
	template <unsigned N> uint64_t pm_r(offs_t offset);
	template <unsigned N> void pm_w(offs_t offset, uint64_t data, uint64_t mem_mask = ~uint64_t(0));
	uint64_t program_read(uint32_t address);
	void program_write(uint32_t address, uint64_t data, uint64_t mem_mask = ~uint64_t(0));
	uint32_t data_read(uint32_t address);
	void data_write(uint32_t address, uint32_t data);
	struct short_view { bool sign_extend = false; void select(int) { sign_extend = true; } void disable() { sign_extend = false; } } m_dm_short_view;

	static constexpr bool m_enable_drc = false;
	struct program_space { adsp21062_device *d; uint64_t read_qword(uint32_t a); void write_qword(uint32_t a, uint64_t v, uint64_t m = ~uint64_t(0)); } m_program;
	struct data_space { adsp21062_device *d; uint32_t read_dword(uint32_t a); void write_dword(uint32_t a, uint32_t v); } m_data;
	std::unique_ptr<sharc_internal_state> m_heap_core;
	uint32_t m_blocks[2][0x8000];
	bool m_stop = false, m_halted = true;
	uint64_t m_instructions = 0;
	int m_iop_delay = 0; // cycles until the delayed IOP write (MAME's delayed_iop_timer)
	void sharc_iop_delayed_write_callback();
	void sharc_dma_callback(int channel);
};

} // namespace rt::sharc

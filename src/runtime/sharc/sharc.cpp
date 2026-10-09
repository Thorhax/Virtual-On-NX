// license:BSD-3-Clause
// copyright-holders:Ville Linde
//
// ADSP-21062 SHARC: MAME's sharc.cpp interpreter paths (device_start/reset,
// IOP registers, internal memory, external host DMA, interrupts and the
// execute loop) without the device framework or the DRC. The MAME files
// sharcops.hxx, compute.hxx, sharcdma.hxx, sharcinternal.ipp and
// sharcops_table.cpp are included unmodified. Changes here: memory maps are
// explicit functions (pgm_2m/data_2m); the delayed IOP write counts cycles
// instead of using a timer; run() stops early when a bus callback asks
// (stop_run) and does nothing while halted.

#include "runtime/sharc/sharc.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#undef BIT
#define BIT(...) sharc_bit(__VA_ARGS__)

namespace rt::sharc {

#include "sharcinternal.ipp"

void adsp21062_device::CHANGE_PC(uint32_t newpc)
{
	m_core->pc = newpc;
	m_core->daddr = newpc;
	m_core->faddr = newpc+1;
	m_core->nfaddr = newpc+2;
}

void adsp21062_device::CHANGE_PC_DELAYED(uint32_t newpc)
{
	m_core->nfaddr = newpc;

	m_core->delay_slot1 = m_core->pc;
	m_core->delay_slot2 = m_core->daddr;
}

void adsp21062_device::sharc_iop_delayed_write_callback()
{
	switch (m_core->iop_delayed_reg)
	{
		case 0x1c:
			if (m_core->iop_delayed_data & 0x1)
				sharc_dma_exec(6);
			break;
		case 0x1d:
			if (m_core->iop_delayed_data & 0x1)
				sharc_dma_exec(7);
			break;
		default:
			throw emu_fatalerror("SHARC: sharc_iop_delayed_write: unknown IOP register %02X\n", m_core->iop_delayed_reg);
	}
}

void adsp21062_device::sharc_iop_delayed_w(uint32_t reg, uint32_t data, int cycles)
{
	m_core->iop_delayed_reg = reg;
	m_core->iop_delayed_data = data;
	m_iop_delay = std::max(cycles, 1);
}

template <unsigned N>
uint64_t adsp21062_device::pm_r(offs_t offset)
{
	offs_t slot = offset >> 12;
	offs_t base = (offset & 0xfff) + (slot >> 1) * (3<<12);
	if (slot & 1)
		return (uint64_t(m_blocks[N][base + 0x2000]) << 16) | (m_blocks[N][base + 0x1000] & 0xffff);
	else
		return (uint64_t(m_blocks[N][base         ]) << 16) | (m_blocks[N][base + 0x1000] >> 16);
}

template <unsigned N>
void adsp21062_device::pm_w(offs_t offset, uint64_t data, uint64_t mem_mask)
{
	offs_t slot = offset >> 12;
	offs_t base = (offset & 0xfff) + (slot >> 1) * (3<<12);
	const bool low16 = (mem_mask & 0xffff) != 0; // ACCESSING_BITS_0_15
	if (slot & 1)
	{
		if (low16)
			m_blocks[N][base + 0x1000] = (m_blocks[N][base + 0x1000] & 0xffff0000) | (data & 0xffff);
		m_blocks[N][base + 0x2000] = uint32_t((m_blocks[N][base + 0x2000] & ~(mem_mask >> 16)) | ((data & mem_mask) >> 16));
	}
	else
	{
		m_blocks[N][base + 0x0000] = uint32_t((m_blocks[N][base + 0x0000] & ~(mem_mask >> 16)) | ((data & mem_mask) >> 16));
		if (low16)
			m_blocks[N][base + 0x1000] = uint32_t((m_blocks[N][base + 0x1000] & 0xffff) | ((data & 0xffff) << 16));
	}
}

// pgm_2m: 0x20000-0x24fff block 0; block 1 mirrored at 0x28000, 0x30000, 0x38000.
uint64_t adsp21062_device::program_read(uint32_t address)
{
	address &= 0xffffff;
	if (address >= 0x20000 && address <= 0x3ffff)
	{
		const uint32_t o = address & 0x7fff;
		if (o > 0x4fff) return 0;
		return (address & 0x18000) ? pm_r<1>(o) : pm_r<0>(o);
	}
	return 0;
}

void adsp21062_device::program_write(uint32_t address, uint64_t data, uint64_t mem_mask)
{
	address &= 0xffffff;
	if (address >= 0x20000 && address <= 0x3ffff)
	{
		const uint32_t o = address & 0x7fff;
		if (o > 0x4fff) return;
		if (address & 0x18000) pm_w<1>(o, data, mem_mask);
		else pm_w<0>(o, data, mem_mask);
	}
}

// data_2m: IOP registers, the two blocks (block 1 mirrored), short words
// (block 0 at 0x40000, block 1 mirrored above), the external bus above.
uint32_t adsp21062_device::data_read(uint32_t address)
{
	if (address < 0x100) return iop_r(address);
	if (address >= 0x20000 && address <= 0x3ffff)
		return m_blocks[(address & 0x18000) ? 1 : 0][address & 0x7fff];
	if (address >= 0x40000 && address <= 0x7ffff)
	{
		const uint16_t *b = reinterpret_cast<const uint16_t *>(m_blocks[(address & 0x30000) ? 1 : 0]);
		const uint16_t v = b[address & 0xffff];
		return m_dm_short_view.sign_extend ? uint32_t(int32_t(int16_t(v))) : v;
	}
	if (address >= 0x400000 && ext_dm_read) return ext_dm_read(address);
	return 0;
}

void adsp21062_device::data_write(uint32_t address, uint32_t data)
{
	if (address < 0x100) { iop_w(address, data); return; }
	if (address >= 0x20000 && address <= 0x3ffff)
	{
		m_blocks[(address & 0x18000) ? 1 : 0][address & 0x7fff] = data;
		return;
	}
	if (address >= 0x40000 && address <= 0x7ffff)
	{
		uint16_t *b = reinterpret_cast<uint16_t *>(m_blocks[(address & 0x30000) ? 1 : 0]);
		b[address & 0xffff] = uint16_t(data);
		return;
	}
	if (address >= 0x400000 && ext_dm_write) ext_dm_write(address, data);
}

// sharcinternal.ipp's accessors go through m_program / m_data.
uint64_t adsp21062_device::program_space::read_qword(uint32_t a) { return d->program_read(a); }
void adsp21062_device::program_space::write_qword(uint32_t a, uint64_t v, uint64_t m) { d->program_write(a, v, m); }
uint32_t adsp21062_device::data_space::read_dword(uint32_t a) { return d->data_read(a); }
void adsp21062_device::data_space::write_dword(uint32_t a, uint32_t v) { d->data_write(a, v); }

/* IOP registers */
uint32_t adsp21062_device::iop_r(offs_t offset)
{
	switch (offset)
	{
		case 0x00: return 0;    // System configuration

		case 0x37:      // DMA status
		{
			return m_core->dma_status;
		}
		default:
			throw emu_fatalerror("sharc_iop_r: Unimplemented IOP reg %02X at %08X\n", offset, m_core->pc);
	}
}

void adsp21062_device::iop_w(offs_t offset, uint32_t data)
{
	switch (offset)
	{
		case 0x00: m_core->syscon = data; break;
		case 0x02: break;       // External Memory Wait State Configuration
		case 0x04: // External port DMA buffer 0
		{
			external_dma_write(m_core->extdma_shift, data);
			m_core->extdma_shift++;
			if (m_core->extdma_shift == 3)
				m_core->extdma_shift = 0;
			break;
		}

		case 0x08: break;       // Message Register 0
		case 0x09: break;       // Message Register 1
		case 0x0a: break;       // Message Register 2
		case 0x0b: break;       // Message Register 3
		case 0x0c: break;       // Message Register 4
		case 0x0d: break;       // Message Register 5
		case 0x0e: break;       // Message Register 6
		case 0x0f: break;       // Message Register 7

		case 0x14: // reserved??? written by Last Bronx
		case 0x17: break;

		// DMA 6
		case 0x1c:
		{
			m_core->dma[6].control = data;
			if (data & 0x1)
			{
				sharc_iop_delayed_w(0x1c, data, 1);
			}
			break;
		}

		case 0x20: break;

		case 0x40: m_core->dma[6].int_index = data; return;
		case 0x41: m_core->dma[6].int_modifier = data; return;
		case 0x42: m_core->dma[6].int_count = data; return;
		case 0x43: m_core->dma[6].chain_ptr = data; return;
		case 0x44: m_core->dma[6].gen_purpose = data; return;
		case 0x45: m_core->dma[6].ext_index = data; return;
		case 0x46: m_core->dma[6].ext_modifier = data; return;
		case 0x47: m_core->dma[6].ext_count = data; return;

		// DMA 7
		case 0x1d:
		{
			m_core->dma[7].control = data;
			if (data & 0x1)
			{
				sharc_iop_delayed_w(0x1d, data, 30);
			}
			break;
		}

		case 0x48: m_core->dma[7].int_index = data; return;
		case 0x49: m_core->dma[7].int_modifier = data; return;
		case 0x4a: m_core->dma[7].int_count = data; return;
		case 0x4b: m_core->dma[7].chain_ptr = data; return;
		case 0x4c: m_core->dma[7].gen_purpose = data; return;
		case 0x4d: m_core->dma[7].ext_index = data; return;
		case 0x4e: m_core->dma[7].ext_modifier = data; return;
		case 0x4f: m_core->dma[7].ext_count = data; return;

		default:
			throw emu_fatalerror("sharc_iop_w: Unimplemented IOP reg %02X, %08X at %08X\n", offset, data, m_core->pc);
	}
}

#include "sharcdma.hxx"
#include "sharcops.hxx"
#include "sharcops_table.cpp"

void adsp21062_device::build_opcode_table()
{
	for (size_t i = 0; i < std::size(m_sharc_op); i++)
	{
		m_sharc_op[i] = &adsp21062_device::sharcop_unimplemented;

		const uint16_t op = uint16_t(i << 7);

		size_t j = 0;
		while (j < s_num_ops)
		{
			auto &opcode = s_sharc_opcode_table[j++];
			if ((opcode.op_mask & op) == opcode.op_bits)
			{
				m_sharc_op[i] = opcode.handler;
				break;
			}
		}
		while (j < s_num_ops)
		{
			auto &opcode = s_sharc_opcode_table[j++];
			if ((opcode.op_mask & op) == opcode.op_bits)
			{
				throw emu_fatalerror("build_opcode_table: table already filled! (i=%04X, j=%d)\n", unsigned(i), int(j));
			}
		}
	}
}

void adsp21062_device::external_iop_write(offs_t address, uint32_t data)
{
	// host packing mode used to determine width of external host bus width (16/32)
	// if writing to external port DMA buffer, just write the data directly;
	// external_dma_write() handles packed data
	if (m_core->syscon & 0x10 && address != 0x04)
	{
		if ((m_core->iop_write_num++ & 1) == 0)
		{
			m_core->iop_data = data & 0xffff;
			return;
		}
		else
		{
			m_core->iop_data |= (data & 0xffff) << 16;
		}
	}
	else
	{
		m_core->iop_data = data;
	}

	if (address == 0x1c)
	{
		m_core->dma[6].control = m_core->iop_data;
	}
	else
	{
		data_write(address, m_core->iop_data);
	}
}

void adsp21062_device::external_dma_write(offs_t address, uint64_t data)
{
	/*
	All addresses in the 17-bit index registers are offset by 0x0002 0000, the
	first internal RAM location, before they are used by the DMA controller.
	*/

	offs_t const index = (m_core->dma[6].int_index & 0x1ffff) | 0x20000;
	unsigned const mswf = BIT(m_core->dma[6].control, 8);
	unsigned const pmode = BIT(m_core->dma[6].control, 6, 2);
	unsigned const dtype = BIT(m_core->dma[6].control, 5);
	switch (pmode)
	{
		case 0:         // no packing
		{
			if (dtype)
				pm_write32(index, uint32_t(data));
			else
				dm_write32(index, uint32_t(data));

			m_core->dma[6].int_index += m_core->dma[6].int_modifier;
			break;
		}
		case 2:         // 16/48 packing
		{
			// FIXME: honour DTYPE
			unsigned const word = address % 3;
			unsigned const shift = (mswf ? (2 - word) : word) * 16;

			uint64_t r = pm_read48(index);
			r &= ~(uint64_t(0xffff) << shift);
			r |= (data & 0xffff) << shift;

			pm_write48(index, r);

			if (word == 2)
			{
				m_core->dma[6].int_index += m_core->dma[6].int_modifier;
			}
			break;
		}
		default:
		{
			throw emu_fatalerror("sharc_external_dma_write: unimplemented packing mode %d\n", pmode);
		}
	}
}

adsp21062_device::adsp21062_device()
	: m_program{this}, m_data{this}
{
	m_heap_core = std::make_unique<sharc_internal_state>();
	m_core = m_heap_core.get();
	std::memset(m_core, 0, sizeof(sharc_internal_state));
	build_opcode_table();
	for (auto &elem : m_core->dma_op)
	{
		elem = SHARC_DMA_OP{};
		elem.timer = nullptr;
	}
	m_core->fp_const.k0_0 = 0.0F;
	m_core->fp_const.k0_5 = 0.5F;
	m_core->fp_const.k1_0 = 1.0F;
	m_core->fp_const.k2_0 = 2.0F;
	m_core->delayed_iop_timer = nullptr;
	for (auto &cb : m_flag_out_cb) cb = [](int) {}; // unconnected unless the board sets them
}

adsp21062_device::~adsp21062_device() = default;

uint32_t adsp21062_device::pc() const { return m_core->pc; }
bool adsp21062_device::idle() const { return m_core->idle != 0; }

// device_reset, boot mode host.
void adsp21062_device::reset()
{
	for (auto &block : m_blocks)
		std::fill(std::begin(block), std::end(block), 0);

	m_core->dma[6].int_index      = 0x20000;
	m_core->dma[6].int_modifier   = 1;
	m_core->dma[6].int_count      = 0x100;
	m_core->dma[6].ext_index      = 0x400000;
	m_core->dma[6].ext_modifier   = 1;
	m_core->dma[6].ext_count      = 0x600;
	m_core->dma[6].control        = 0xa1;

	m_core->pc = 0x20004;
	m_core->extdma_shift = 0;
	m_core->daddr = m_core->pc + 1;
	m_core->faddr = m_core->daddr + 1;
	m_core->nfaddr = m_core->faddr+1;

	m_core->idle = 0;
	m_core->mode1 = 0x00000000;
	m_core->mode2 &= 0xf0000000;
	m_core->astat &= FLG0 | FLG1 | FLG2 | FLG3;
	m_core->stky = PCEM | SSEM | LSEM;
	m_core->irptl = 0x0000;
	m_core->imask = 0x0003;
	m_core->ustat1 = 0x0000;
	m_core->ustat2 = 0x0000;

	m_core->pcstkp = 0;
	m_core->lstkp = 0;
	m_core->pcstk = 0x00ffffff;
	m_core->curlcntr = 0xffffffff;
	m_core->lcntr = m_core->lcstack[0];
	m_core->laddr.unpack(0xffffffff);
	m_core->status_stkp = 0;
	m_core->interrupt_active = 0;

	m_core->syscon = 0x00000010;
	m_core->sysstat &= 0x00000ff0;
	m_core->iop_write_num = 0;
	m_core->iop_data = 0;
	m_iop_delay = 0;
}

void adsp21062_device::set_input_line(int irqline, int state)
{
	if (irqline >= 0 && irqline <= 2)
	{
		if (state)
			m_core->irq_pending |= 1 << (8-irqline);
		else
			m_core->irq_pending &= ~(1 << (8-irqline));
	}
}

// Clockless here: the caller sets inputs between runs, so they apply at once.
void adsp21062_device::set_flag_input(int flag_num, int state)
{
	if (BIT(m_core->mode2, flag_num + 15))
		throw emu_fatalerror("sharc_set_flag_input: flag %d is set output!\n", flag_num);
	m_core->flag[flag_num] = state ? 1 : 0;
}

void adsp21062_device::write_stall(int state)
{
	m_core->write_stalled = state != 0;
}

void adsp21062_device::check_interrupts()
{
	if ((m_core->imask & m_core->irq_pending) && (m_core->mode1 & MODE1_IRPTEN) && !m_core->interrupt_active &&
		m_core->pc != m_core->delay_slot1 && m_core->pc != m_core->delay_slot2)
	{
		int which = 0;
		for (int i = 0; i < 32; i++)
		{
			if (BIT(m_core->irq_pending, i))
				break;
			which++;
		}

		PUSH_PC();
		if (m_core->idle)
			m_core->pcstk = m_core->pc + 1;
		else
			m_core->pcstk = m_core->daddr;

		m_core->irptl |= 1 << which;

		// TODO: timer and VIRPT interrupts also push the status stack
		if (which >= 6 && which <= 8)
			PUSH_STATUS_STACK();

		CHANGE_PC(0x20000 + (which * 0x4));

		/* TODO: alter IMASKP */

		m_core->active_irq_num = which;
		m_core->irq_pending &= ~(1 << which);

		m_core->interrupt_active = 1;
	}
}

// execute_run, interpreter path. Returns the cycles used.
int adsp21062_device::run(int cycles)
{
	if (m_halted) return 0;
	m_core->icount = cycles;
	m_stop = false;

	if (m_core->write_stalled)
		return cycles;

	if (m_core->idle && m_core->irq_pending == 0)
	{
		int dma_count = m_core->icount;

		// run active DMAs even while idling
		while ((dma_count > 0) && (m_core->dma_status & ((1 << 6) | (1 << 7))))
		{
			if (!m_core->write_stalled)
			{
				dma_run_cycle(6);
				dma_run_cycle(7);
			}
			dma_count--;
		}

		m_core->icount = 0;
	}
	if (m_core->irq_pending != 0)
	{
		check_interrupts();
		m_core->idle = 0;
	}

	while (m_core->icount > 0 && !m_core->idle && !m_core->write_stalled && !m_stop)
	{
		m_core->pc = m_core->daddr;
		m_core->daddr = m_core->faddr;
		m_core->faddr = m_core->nfaddr;
		m_core->nfaddr++;

		m_core->astat_old_old_old = m_core->astat_old_old;
		m_core->astat_old_old = m_core->astat_old;
		m_core->astat_old = m_core->astat;

		m_core->opcode = program_read(m_core->pc);

		// handle looping
		if (!(m_core->stky & LSEM) && (m_core->pc == m_core->laddr.addr))
		{
			switch (m_core->laddr.loop_type)
			{
			case 0:     // arithmetic condition-based
			{
				if ((m_core->pc - TOP_PC()) > 2)
				{
					m_core->astat = m_core->astat_old_old_old;
				}

				if (DO_CONDITION_CODE(m_core->laddr.code))
				{
					POP_LOOP();
					POP_PC();
				}
				else
				{
					CHANGE_PC(TOP_PC());
				}

				m_core->astat = m_core->astat_old;
				break;
			}
			case 1:     // counter-based, length 1
			case 2:     // counter-based, length 2
			case 3:     // counter-based, length >2
			{
				--m_core->curlcntr;
				if (m_core->curlcntr == 0)
				{
					POP_LOOP();
					POP_PC();
				}
				else
				{
					CHANGE_PC(TOP_PC());
				}
			}
			}
		}

		(this->*m_sharc_op[(m_core->opcode >> 39) & 0x1ff])();
		++m_instructions;

		// System register latency effect
		if (m_core->systemreg_latency_cycles > 0)
		{
			--m_core->systemreg_latency_cycles;
			if (m_core->systemreg_latency_cycles <= 0)
			{
				systemreg_write_latency_effect();
			}
		}

		if (m_iop_delay > 0 && --m_iop_delay == 0)
			sharc_iop_delayed_write_callback();

		if (!m_core->write_stalled)
		{
			dma_run_cycle(6);
			dma_run_cycle(7);
		}

		--m_core->icount;
	}
	return cycles - std::max(m_core->icount, 0);
}

} // namespace rt::sharc

from __future__ import annotations

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import NextTimeStep, ReadOnly, RisingEdge

from verification.driver import ProtocolDriver
from verification.isa import ISA
from verification.monitor import (
    CoreMonitor,
    ST_EXEC,
    ST_POST_DELAY,
    ST_WAIT_PIN,
    ST_WAIT_TICKS,
)

ISA_DEF = ISA()


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    drv = ProtocolDriver(dut)
    await drv.reset()
    return drv


async def run_trace(dut, drv, words, tx=None, cfg=None, timeout=2000):
    if cfg:
        for addr, value in cfg:
            await drv.write_cfg(addr, value)
    if tx:
        for value in tx:
            await drv.push_tx(value)
    await drv.load_program(words)
    mon = CoreMonitor(dut)
    await drv.start()
    await mon.capture_until_idle(max_cycles=timeout)
    return mon

async def dump_waitpin_cycles(dut, cycles=20):
    core = dut.u_dut.u_core
    gpio = dut.u_dut.u_gpio

    print(
        "cyc  pad  s1  s2  prev  gin  rise fall "
        "pc state   ir     cond pin wp_true"
    )

    for cyc in range(cycles):
        await RisingEdge(dut.clk)
        await ReadOnly()

        print(
            f"{cyc:3d}  "
            f"{int(dut.pad_in.value):02x}   "
            f"{int(gpio.sync1.value):02x}  "
            f"{int(gpio.sync2.value):02x}  "
            f"{int(gpio.prev.value):02x}    "
            f"{int(dut.u_dut.gpio_in.value):02x}   "
            f"{int(dut.u_dut.gpio_rise.value):02x}   "
            f"{int(dut.u_dut.gpio_fall.value):02x}   "
            f"{int(core.pc.value):02x}    "
            f"{int(core.state.value)}    "
            f"{int(core.ir.value):04x}   "
            f"{int(core.cond.value)}    "
            f"{int(core.waitpin_pin.value)}   "
            f"{int(core.waitpin_true.value)}"
        )

    await NextTimeStep()

@cocotb.test()
async def test_reset_defaults_nop_halt(dut):
    drv = await setup(dut)
    assert int(dut.busy.value) == 0
    assert int(dut.fault.value) == 0
    assert int(dut.pad_out.value) == 0
    assert int(dut.pad_oe.value) == 0
    assert await drv.read_cfg(ISA_DEF.d("CFG_CLKDIV_LO")) == 1
    assert await drv.read_cfg(ISA_DEF.d("CFG_CLKDIV_HI")) == 0

    mon = await run_trace(dut, drv, [ISA_DEF.nop(), ISA_DEF.halt()])
    assert int(dut.fault.value) == 0
    assert int(dut.u_dut.u_core.pc.value) == 1
    assert len(mon.exec_samples(ISA_DEF.d("OP_NOP"))) == 1
    assert len(mon.exec_samples(ISA_DEF.d("OP_HALT"))) == 1


@cocotb.test()
async def test_gpio_and_output_enable_instructions(dut):
    drv = await setup(dut)
    words = [
        ISA_DEF.gpio_write(0xA5),
        ISA_DEF.gpio_set(0x12),
        ISA_DEF.gpio_clr(0x84),
        ISA_DEF.oe_write(0xF0),
        ISA_DEF.halt(),
    ]
    await run_trace(dut, drv, words)
    assert int(dut.u_dut.gpio_out.value) == 0x33
    assert int(dut.u_dut.gpio_oe.value) == 0xF0
    assert int(dut.pad_out.value) == 0x33
    assert int(dut.pad_oe.value) == 0xF0


@cocotb.test()
async def test_ldi_mov_and_zero_flag(dut):
    drv = await setup(dut)
    d = ISA_DEF.defs
    words = [
        ISA_DEF.ldi(d["REG_X"], 0x5A),
        ISA_DEF.mov(d["REG_Y"], d["REG_X"]),
        ISA_DEF.mov(d["REG_LC"], d["REG_Y"]),
        ISA_DEF.halt(),
    ]
    await run_trace(dut, drv, words)
    core = dut.u_dut.u_core
    assert int(core.x_reg.value) == 0x5A
    assert int(core.y_reg.value) == 0x5A
    assert int(core.loop_count.value) == 0x5A
    assert int(core.zero_flag.value) == 0

    await drv.reset()
    await run_trace(dut, drv, [ISA_DEF.ldi(d["REG_X"], 0), ISA_DEF.halt()])
    assert int(dut.u_dut.u_core.zero_flag.value) == 1


@cocotb.test()
async def test_jmp_and_jcc_taken_not_taken(dut):
    drv = await setup(dut)
    d = ISA_DEF.defs
    # JMP must skip the 0xEE marker.
    words = [
        ISA_DEF.jmp(3),
        ISA_DEF.gpio_write(0xEE),
        ISA_DEF.halt(),
        ISA_DEF.gpio_write(0x5A),
        ISA_DEF.halt(),
    ]
    await run_trace(dut, drv, words)
    assert int(dut.u_dut.gpio_out.value) == 0x5A

    # First JCC is not taken (Z=1, condition NZERO), second is taken (ZERO).
    await drv.reset()
    words = [
        ISA_DEF.ldi(d["REG_X"], 0),
        ISA_DEF.jcc(d["COND_NZERO"], 4),
        ISA_DEF.gpio_write(0x11),
        ISA_DEF.jcc(d["COND_ZERO"], 6),
        ISA_DEF.gpio_write(0xEE),
        ISA_DEF.halt(),
        ISA_DEF.gpio_write(0x22),
        ISA_DEF.halt(),
    ]
    await run_trace(dut, drv, words)
    assert int(dut.u_dut.gpio_out.value) == 0x22


@cocotb.test()
async def test_loop_semantics(dut):
    drv = await setup(dut)
    d = ISA_DEF.defs
    words = [
        ISA_DEF.ldi(d["REG_LC"], 3),
        ISA_DEF.nop(),
        ISA_DEF.loop(1),
        ISA_DEF.halt(),
    ]
    mon = await run_trace(dut, drv, words)
    executions_pc1 = [s for s in mon.samples if s.state == ST_EXEC and s.pc == 1]
    assert len(executions_pc1) == 3
    assert int(dut.u_dut.u_core.loop_count.value) == 0
    assert int(dut.u_dut.u_core.zero_flag.value) == 1


@cocotb.test()
async def test_wait_exact_tick_count_and_wait_zero(dut):
    drv = await setup(dut)
    mon = await run_trace(dut, drv, [ISA_DEF.wait(3), ISA_DEF.halt()])
    wait_samples = mon.state_samples(ST_WAIT_TICKS)
    assert [s.wait_remaining for s in wait_samples] == [3, 2, 1]
    assert sum(s.tick for s in wait_samples) == 3

    await drv.reset()
    mon = await run_trace(dut, drv, [ISA_DEF.wait(0), ISA_DEF.halt()])
    assert len(mon.state_samples(ST_WAIT_TICKS)) == 0


@cocotb.test()
async def test_nop_post_delay(dut):
    drv = await setup(dut)
    mon = await run_trace(dut, drv, [ISA_DEF.nop(aux=3), ISA_DEF.halt()])
    delay = mon.state_samples(ST_POST_DELAY)
    assert [s.delay_remaining for s in delay] == [3, 2, 1]


@cocotb.test()
async def test_waitpin_level_and_rising_edge_not_latched_early(dut):
    drv = await setup(dut)
    d = ISA_DEF.defs
    pin = 2

    # Level wait.
    await drv.set_pad(0, settle_cycles=3)
    await drv.load_program([ISA_DEF.waitpin(d["COND_PIN_HIGH"], pin), ISA_DEF.halt()])
    await drv.start()
    await drv.wait_for_state(ST_WAIT_PIN)
    #await drv.cycle(3)
    assert int(dut.u_dut.u_core.pc.value) == 0
    print("---- BEFORE INPUT CHANGE ----")
    await dump_waitpin_cycles(dut, 5)
    await drv.set_pad(1 << pin)
    print("---- AFTER INPUT CHANGE ----")
    await dump_waitpin_cycles(dut, 12)
    #await drv.wait_idle()
    assert int(dut.fault.value) == 0

    # An edge that happened before WAITPIN became active must not be remembered.
    await drv.reset()
    await drv.set_pad(0, settle_cycles=3)
    await drv.set_pad(1 << pin, settle_cycles=3)  # rising edge before start
    await drv.load_program([ISA_DEF.waitpin(d["COND_PIN_RISE"], pin), ISA_DEF.halt()])
    await drv.start()
    await drv.wait_for_state(ST_WAIT_PIN)
    await drv.cycle(3)
    assert int(dut.u_dut.u_core.pc.value) == 0
    await drv.set_pad(0, settle_cycles=3)
    assert int(dut.u_dut.u_core.pc.value) == 0
    await drv.set_pad(1 << pin)
    await drv.wait_idle()
    assert int(dut.fault.value) == 0


@cocotb.test()
async def test_pull_success_and_empty_stall(dut):
    drv = await setup(dut)
    await drv.push_tx(0x96)
    await run_trace(dut, drv, [ISA_DEF.pull(), ISA_DEF.halt()])
    assert int(dut.u_dut.osr_value.value) == 0x96
    assert int(dut.u_dut.tx_empty.value) == 1

    await drv.reset()
    await drv.load_program([ISA_DEF.pull(), ISA_DEF.halt()])
    await drv.start()
    await drv.wait_for_state(ST_EXEC)
    await drv.cycle(4)
    assert int(dut.u_dut.u_core.pc.value) == 0
    assert int(dut.busy.value) == 1
    await drv.push_tx(0x3C)
    await drv.wait_idle()
    assert int(dut.u_dut.osr_value.value) == 0x3C


@cocotb.test()
async def test_push_and_full_stall(dut):
    drv = await setup(dut)
    # ISR resets to zero. Four pushes fill the V0 four-entry FIFO; the fifth stalls.
    words = [ISA_DEF.push()] * 5 + [ISA_DEF.halt()]
    await drv.load_program(words)
    await drv.start()

    core = dut.u_dut.u_core
    for _ in range(300):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(core.state.value) == ST_EXEC and int(core.pc.value) == 4:
            break
    else:
        raise AssertionError("fifth PUSH was never reached")

    await drv.cycle(3)
    assert int(core.pc.value) == 4
    assert int(dut.u_dut.rx_full.value) == 1

    assert await drv.pop_rx() == 0
    await drv.wait_idle()
    assert int(dut.fault.value) == 0


@cocotb.test()
async def test_out_shift_right_and_left(dut):
    drv = await setup(dut)
    d = ISA_DEF.defs
    out_pin = 1
    value = 0x96

    # PINMAP0 [2:0]=out pin, bit6=OUT shift right.
    pinmap_right = out_pin | (1 << 6)
    words = [ISA_DEF.pull()] + [ISA_DEF.out() for _ in range(8)] + [ISA_DEF.halt()]
    mon = await run_trace(
        dut,
        drv,
        words,
        tx=[value],
        cfg=[(d["CFG_PINMAP0"], pinmap_right)],
    )
    out_exec = mon.exec_samples(d["OP_OUT"])
    right_bits = [s.osr & 1 for s in out_exec]
    assert right_bits == [(value >> i) & 1 for i in range(8)]
    assert int(dut.u_dut.osr_value.value) == 0

    await drv.reset()
    pinmap_left = out_pin
    mon = await run_trace(
        dut,
        drv,
        words,
        tx=[value],
        cfg=[(d["CFG_PINMAP0"], pinmap_left)],
    )
    out_exec = mon.exec_samples(d["OP_OUT"])
    left_bits = [(s.osr >> 7) & 1 for s in out_exec]
    assert left_bits == [(value >> i) & 1 for i in range(7, -1, -1)]


@cocotb.test()
async def test_in_shift_right_and_left(dut):
    drv = await setup(dut)
    d = ISA_DEF.defs
    pin = 0
    bits = [1, 0, 1, 1, 0, 0, 1, 0]

    async def run_direction(shift_right: bool):
        await drv.reset()
        pinmap = (pin << 3) | ((1 if shift_right else 0) << 7)
        await drv.write_cfg(d["CFG_PINMAP0"], pinmap)

        words = []
        wait_pcs = []
        for bit in bits:
            wait_pcs.append(len(words))
            cond = d["COND_PIN_HIGH"] if bit else d["COND_PIN_LOW"]
            words += [ISA_DEF.waitpin(cond, pin), ISA_DEF.inp()]
        words.append(ISA_DEF.halt())
        await drv.load_program(words)
        await drv.start()

        core = dut.u_dut.u_core
        for bit, wait_pc in zip(bits, wait_pcs):
            # Wait until this WAITPIN is active, then present the requested level.
            for _ in range(300):
                await RisingEdge(dut.clk)
                await ReadOnly()
                if int(core.pc.value) == wait_pc and int(core.state.value) in (ST_EXEC, ST_WAIT_PIN):
                    break
            else:
                raise AssertionError(f"WAITPIN at PC {wait_pc} not reached")
            # The search loop breaks in ReadOnly.  Move to the next writable
            # timestep before presenting the next input bit.
            await NextTimeStep()
            await drv.set_pad(bit << pin)

        await drv.wait_idle(max_cycles=2000)
        return int(dut.u_dut.isr_value.value)

    right_result = await run_direction(True)
    expected_right = 0
    for bit in bits:
        expected_right = ((bit << 7) | (expected_right >> 1)) & 0xFF
    assert right_result == expected_right

    left_result = await run_direction(False)
    expected_left = 0
    for bit in bits:
        expected_left = ((expected_left << 1) | bit) & 0xFF
    assert left_result == expected_left


@cocotb.test()
async def test_xfer_shifts_both_directions_same_instruction(dut):
    drv = await setup(dut)
    d = ISA_DEF.defs
    # OUT pin 1, IN pin 0, both shift right.
    pinmap = 1 | (0 << 3) | (1 << 6) | (1 << 7)
    await drv.write_cfg(d["CFG_PINMAP0"], pinmap)
    await drv.push_tx(0x03)
    await drv.set_pad(1, settle_cycles=3)
    mon = await run_trace(dut, drv, [ISA_DEF.pull(), ISA_DEF.xfer(), ISA_DEF.halt()])
    xfers = mon.exec_samples(d["OP_XFER"])
    assert len(xfers) == 1
    assert (xfers[0].osr & 1) == 1
    assert int(dut.u_dut.osr_value.value) == 0x01
    assert int(dut.u_dut.isr_value.value) == 0x80


@cocotb.test()
async def test_cfg_instruction_updates_configuration_bank(dut):
    drv = await setup(dut)
    d = ISA_DEF.defs
    words = [
        ISA_DEF.cfg(d["CFG_CLKDIV_LO"], 7),
        ISA_DEF.cfg(d["CFG_OD_MASK"], 0xA5),
        ISA_DEF.cfg(d["CFG_JMP_PIN"], 5),
        ISA_DEF.halt(),
    ]
    await run_trace(dut, drv, words)
    assert await drv.read_cfg(d["CFG_CLKDIV_LO"]) == 7
    assert await drv.read_cfg(d["CFG_OD_MASK"]) == 0xA5
    assert (await drv.read_cfg(d["CFG_JMP_PIN"]) & 0x7) == 5


@cocotb.test()
async def test_side_set_output_and_side_set_delay(dut):
    drv = await setup(dut)
    d = ISA_DEF.defs
    side_pin = 3
    # PINMAP1: [2:0] side pin, [3] enable, [4] target OE.
    side_cfg = side_pin | (1 << 3)
    await drv.write_cfg(d["CFG_PINMAP1"], side_cfg)
    aux = ISA_DEF.side_aux(1, delay=2)
    mon = await run_trace(dut, drv, [ISA_DEF.nop(aux=aux), ISA_DEF.halt()])
    assert (int(dut.u_dut.gpio_out.value) >> side_pin) & 1 == 1
    delay = mon.state_samples(ST_POST_DELAY)
    assert [s.delay_remaining for s in delay] == [2, 1]

    # Target OE instead of GPIO_OUT.
    await drv.reset()
    side_cfg = side_pin | (1 << 3) | (1 << 4)
    await drv.write_cfg(d["CFG_PINMAP1"], side_cfg)
    await run_trace(dut, drv, [ISA_DEF.nop(aux=ISA_DEF.side_aux(1, 0)), ISA_DEF.halt()])
    assert (int(dut.u_dut.gpio_oe.value) >> side_pin) & 1 == 1


@cocotb.test()
async def test_reserved_and_unknown_opcodes_fault(dut):
    drv = await setup(dut)
    reserved = ["OP_ALU", "OP_EVENT", "OP_CRC", "OP_STREAM", "OP_LOAD", "OP_STORE"]
    for name in reserved:
        await drv.reset()
        await drv.load_program([ISA_DEF.reserved(name), ISA_DEF.halt()])
        await drv.start()
        await drv.wait_idle()
        assert int(dut.fault.value) == 1, f"{name} should fault until implemented"

    await drv.reset()
    unknown_word = ISA_DEF.raw(0x1F, 0)
    await drv.load_program([unknown_word])
    await drv.start()
    await drv.wait_idle()
    assert int(dut.fault.value) == 1

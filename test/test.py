# Gate-level-safe top-level smoke test for tt_um_protocol_top.
#
# This test deliberately uses only Tiny Tapeout-visible pins.  It works against
# both RTL and the flattened gate-level netlist, and exercises:
#   reset -> host loader -> instruction SRAM -> core -> GPIO/OE -> HALT.

from pathlib import Path
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

# Allow importing the repository's verification package when cocotb runs from test/.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from verification.isa import ISA

I = ISA()

# tt_um_protocol_top host ABI
CMD_PROG_LO  = 0b000
CMD_PROG_HI  = 0b001
CMD_CFG      = 0b010
CMD_TX_PUSH  = 0b011
CMD_RX_POP   = 0b100
CMD_START    = 0b101
CMD_ADDR_RST = 0b110


def ui_word(cmd: int, arg: int = 0, strobe: int = 0, show_rx: int = 0) -> int:
    return (
        ((strobe & 1) << 7)
        | ((cmd & 0x7) << 4)
        | ((show_rx & 1) << 3)
        | (arg & 0x7)
    )


async def host_command(dut, cmd: int, *, arg: int = 0, data: int = 0):
    """Issue one host command using the wrapper's rising-edge HOST_STROBE."""
    dut.uio_in.value = data & 0xFF

    # Present command/data with strobe low.
    dut.ui_in.value = ui_word(cmd, arg, 0)
    await RisingEdge(dut.clk)

    # Rising HOST_STROBE executes the command.
    dut.ui_in.value = ui_word(cmd, arg, 1)
    await RisingEdge(dut.clk)

    # Return low so a later command can generate another rising edge.
    dut.ui_in.value = ui_word(cmd, arg, 0)
    await RisingEdge(dut.clk)


async def load_program(dut, words):
    await host_command(dut, CMD_ADDR_RST)
    for word in words:
        await host_command(dut, CMD_PROG_LO, data=word & 0xFF)
        await host_command(dut, CMD_PROG_HI, data=(word >> 8) & 0xFF)


@cocotb.test()
async def test_gate_level_smoke(dut):
    # Match the ASIC target clock: 50 MHz.
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0

    dut.rst_n.value = 0
    for _ in range(8):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    for _ in range(4):
        await RisingEdge(dut.clk)

    # When idle, the wrapper must release all bidirectional protocol pins.
    assert int(dut.uio_oe.value) == 0x00

    # Keep a visible GPIO value active for several cycles before HALT.
    words = [
        I.gpio_write(0xA5),
        I.oe_write(0xFF),
        I.wait(8),
        I.halt(),
    ]
    await load_program(dut, words)
    await host_command(dut, CMD_START)

    # Wait until the core executes GPIO_WRITE/OE_WRITE.
    saw_gpio = False
    for _ in range(200):
        await RisingEdge(dut.clk)
        if int(dut.uio_oe.value) == 0xFF:
            assert int(dut.uio_out.value) == 0xA5
            saw_gpio = True
            break
    assert saw_gpio, "program never drove the expected GPIO/OE value"

    # HALT returns the wrapper to host ownership and releases uio_oe.
    halted = False
    for _ in range(300):
        await RisingEdge(dut.clk)
        if int(dut.uio_oe.value) == 0x00:
            halted = True
            break
    assert halted, "core did not return to idle after HALT"

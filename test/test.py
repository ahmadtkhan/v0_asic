# Gate-level-safe top-level smoke test for tt_um_protocol_top.
#
# This test intentionally depends only on:
#   * cocotb
#   * test/tb.v
#   * src/proto_isa.vh
#
# It does NOT import verification/ so the TinyTapeout gl_test remains
# self-contained and robust even if PYTHONPATH differs in CI.

from pathlib import Path
import re

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge


def _repo_root() -> Path:
    """Find the repository root by locating src/proto_isa.vh."""
    here = Path(__file__).resolve()
    for parent in (here.parent, *here.parents):
        if (parent / "src" / "proto_isa.vh").is_file():
            return parent
    raise RuntimeError("Could not locate src/proto_isa.vh from test.py")


REPO_ROOT = _repo_root()
ISA_TEXT = (REPO_ROOT / "src" / "proto_isa.vh").read_text()


def _define_hex5(name: str) -> int:
    m = re.search(
        rf"(?m)^\s*`define\s+{re.escape(name)}\s+5'h([0-9a-fA-F]+)\b",
        ISA_TEXT,
    )
    if not m:
        raise RuntimeError(f"{name} not found in src/proto_isa.vh")
    return int(m.group(1), 16)


OP_HALT       = _define_hex5("OP_HALT")
OP_WAIT       = _define_hex5("OP_WAIT")
OP_GPIO_WRITE = _define_hex5("OP_GPIO_WRITE")
OP_OE_WRITE   = _define_hex5("OP_OE_WRITE")


def op(opcode: int, payload11: int = 0) -> int:
    return ((opcode & 0x1F) << 11) | (payload11 & 0x7FF)


def gpio_write(value: int) -> int:
    return op(OP_GPIO_WRITE, value & 0xFF)


def oe_write(value: int) -> int:
    return op(OP_OE_WRITE, value & 0xFF)


def wait_ticks(ticks: int) -> int:
    return op(OP_WAIT, ticks)


def halt() -> int:
    return op(OP_HALT)


# tt_um_protocol_top host ABI.
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
    """Issue one command using the wrapper's rising-edge HOST_STROBE."""
    dut.uio_in.value = data & 0xFF

    dut.ui_in.value = ui_word(cmd, arg, 0)
    await RisingEdge(dut.clk)

    dut.ui_in.value = ui_word(cmd, arg, 1)
    await RisingEdge(dut.clk)

    dut.ui_in.value = ui_word(cmd, arg, 0)
    await RisingEdge(dut.clk)


async def load_program(dut, words):
    await host_command(dut, CMD_ADDR_RST)
    for word in words:
        await host_command(dut, CMD_PROG_LO, data=word & 0xFF)
        await host_command(dut, CMD_PROG_HI, data=(word >> 8) & 0xFF)


@cocotb.test()
async def test_gate_level_smoke(dut):
    # ASIC target clock: 20 ns period = 50 MHz.
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

    # Idle wrapper releases protocol GPIOs.
    assert int(dut.uio_oe.value) == 0x00

    # Program is assembled locally from src/proto_isa.vh opcode definitions.
    words = [
        gpio_write(0xA5),
        oe_write(0xFF),
        wait_ticks(8),
        halt(),
    ]

    await load_program(dut, words)
    await host_command(dut, CMD_START)

    # Confirm the program reached GPIO_WRITE/OE_WRITE.
    saw_gpio = False
    for _ in range(200):
        await RisingEdge(dut.clk)
        if int(dut.uio_oe.value) == 0xFF:
            assert int(dut.uio_out.value) == 0xA5
            saw_gpio = True
            break
    assert saw_gpio, "program never drove expected GPIO/OE value"

    # HALT should return the wrapper to host ownership.
    halted = False
    for _ in range(300):
        await RisingEdge(dut.clk)
        if int(dut.uio_oe.value) == 0x00:
            halted = True
            break
    assert halted, "core did not return to idle after HALT"

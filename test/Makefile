# Tiny Tapeout RTL / gate-level cocotb Makefile

SIM ?= icarus
FST ?= -fst
TOPLEVEL_LANG ?= verilog

SRC_DIR = $(PWD)/../src
REPO_ROOT := $(abspath ..)

# Make repository-level Python packages visible to cocotb.
export PYTHONPATH := $(REPO_ROOT):$(PYTHONPATH)

PROJECT_SOURCES = \
    tt_um_protocol_top.v \
    protocol_top.v \
    protocol_core.v \
    proto_tick.v \
    proto_shift.v \
    proto_gpio.v \
    proto_fifo.v \
    instruction_memory.v \
    instruction_decoder.v \
    config_regs.v \
    data_memory.v \
    proto_alu.v \
    proto_bitstuff.v \
    proto_crc_lfsr.v \
    proto_manchester.v \
    proto_nrzi.v \
    proto_stream.v \
    proto_can_timing.v

ifneq ($(GATES),yes)

SIM_BUILD = sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

SIM_BUILD = sim_build/gl
COMPILE_ARGS += -DGL_TEST
COMPILE_ARGS += -DFUNCTIONAL
COMPILE_ARGS += -DSIM

# Current Tiny Tapeout IHP gate-level setup.
PDK ?= ihp-sg13cmos5l
PDK_CELL_PREFIX = $(patsubst ihp-%,%,$(PDK))
PDK_LIBS_REF = $(PDK_ROOT)/$(PDK)/libs.ref

VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_io/verilog/$(PDK_CELL_PREFIX)_io.v

# Newer CMOS5L PDKs place user-defined primitives such as ihp_mux2 here.
# wildcard keeps compatibility with PDK revisions that do not split this file.
VERILOG_SOURCES += $(wildcard $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_stdcell/verilog/$(PDK_CELL_PREFIX)_udp.v)

VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_stdcell/verilog/$(PDK_CELL_PREFIX)_stdcell.v

# Functional models for the two IHP SRAM hard macros.
# SG13CMOS5L reuses the SG13G2 SRAM library views.
SRAM_VERILOG_DIR = $(PDK_ROOT)/ihp-sg13g2/libs.ref/sg13g2_sram/verilog
VERILOG_SOURCES += $(wildcard $(SRAM_VERILOG_DIR)/RM_IHPSG13_1P_core_behavioral_bm_bist.v)
VERILOG_SOURCES += $(wildcard $(SRAM_VERILOG_DIR)/RM_IHPSG13_1P_256x16_c2_bm_bist.v)
VERILOG_SOURCES += $(wildcard $(SRAM_VERILOG_DIR)/RM_IHPSG13_1P_256x8_c3_bm_bist.v)

# The Tiny Tapeout gl_test action copies the hardened netlist here.
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

COMPILE_ARGS += -I$(SRC_DIR)

VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

COCOTB_TEST_MODULES = test

include $(shell cocotb-config --makefiles)/Makefile.sim

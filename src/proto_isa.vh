`ifndef PROTO_ISA_VH
`define PROTO_ISA_VH

// Protocol-emulator V0 ISA constants.
// The ISA is deliberately separate from proto_params.vh: changing a FIFO or
// memory depth must not silently change instruction encodings.

// Opcodes: instr[15:11]
`define OP_NOP         5'h00
`define OP_HALT        5'h01
`define OP_WAIT        5'h02
`define OP_WAITPIN     5'h03
`define OP_GPIO_SET    5'h04
`define OP_GPIO_CLR    5'h05
`define OP_GPIO_WRITE  5'h06
`define OP_OE_WRITE    5'h07
`define OP_OUT         5'h08
`define OP_IN          5'h09
`define OP_XFER        5'h0A
`define OP_PULL        5'h0B
`define OP_PUSH        5'h0C
`define OP_JMP         5'h0D
`define OP_JCC         5'h0E
`define OP_LOOP        5'h0F
`define OP_LDI         5'h10
`define OP_MOV         5'h11
`define OP_CFG         5'h12

// Extension points. ALU and LOAD/STORE are implemented in the current architecture.
`define OP_ALU         5'h13
`define OP_EVENT       5'h14
`define OP_CRC         5'h15
`define OP_STREAM      5'h16
`define OP_LOAD        5'h17
`define OP_STORE       5'h18


// OP_ALU payload:
//   [10:7] ALU operation
//   [6:5]  destination: X/Y/OSR/ISR
//   [4:3]  source A:    X/Y/OSR/ISR
//   [2:1]  source B:    X/Y/OSR/ISR
//   [0]    reserved, encode as zero
//
// CMP and TEST are flag-only and ignore the destination write.  All ALU
// operations update zero_flag from the computed result.
`define ALU_ADD         4'h0
`define ALU_SUB         4'h1
`define ALU_AND         4'h2
`define ALU_OR          4'h3
`define ALU_XOR         4'h4
`define ALU_SHL         4'h5
`define ALU_SHR         4'h6
`define ALU_NOT         4'h7
`define ALU_INC         4'h8
`define ALU_DEC         4'h9
`define ALU_CMP         4'hA
`define ALU_TEST        4'hB
`define ALU_ROL         4'hC
`define ALU_ROR         4'hD
`define ALU_PASS_A      4'hE
`define ALU_PASS_B      4'hF

// The ALU and LOAD/STORE intentionally share the same four byte-register
// selectors so data can move naturally between computation, DMEM and shifters.
`define ALU_REG_X       2'd0
`define ALU_REG_Y       2'd1
`define ALU_REG_OSR     2'd2
`define ALU_REG_ISR     2'd3

// Conditions used by JCC and WAITPIN.
`define COND_ZERO      3'd0
`define COND_NZERO     3'd1
`define COND_PIN_LOW   3'd2
`define COND_PIN_HIGH  3'd3
`define COND_PIN_RISE  3'd4
`define COND_PIN_FALL  3'd5
`define COND_TX_EMPTY  3'd6
`define COND_RX_FULL   3'd7

// General register selectors used by LDI/MOV.
`define REG_X          3'd0
`define REG_Y          3'd1
`define REG_LC         3'd2
`define REG_ZERO       3'd7

// LOAD/STORE payload:
//   [10:9] data register: X, Y, OSR, ISR
//   [8]    address base: 0=X, 1=Y
//   [7:0]  signed 8-bit offset (two's-complement)
//
// Effective address is (base + offset) modulo 256.  Modulo arithmetic is
// intentional because V0 DMEM is exactly 256 bytes.
`define MEM_REG_X      2'd0
`define MEM_REG_Y      2'd1
`define MEM_REG_OSR    2'd2
`define MEM_REG_ISR    2'd3

`define MEM_BASE_X     1'b0
`define MEM_BASE_Y     1'b1

// Byte-addressed configuration space used by OP_CFG.
`define CFG_CLKDIV_LO  3'd0
`define CFG_CLKDIV_HI  3'd1
`define CFG_PINMAP0    3'd2
`define CFG_PINMAP1    3'd3
`define CFG_OD_MASK    3'd4
`define CFG_JMP_PIN    3'd5
`define CFG_RSVD6      3'd6
`define CFG_RSVD7      3'd7

`endif

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

// Reserved extension points.
`define OP_ALU         5'h13
`define OP_EVENT       5'h14
`define OP_CRC         5'h15
`define OP_STREAM      5'h16
`define OP_LOAD        5'h17
`define OP_STORE       5'h18

// Conditions used by JCC and WAITPIN.
`define COND_ZERO      3'd0
`define COND_NZERO     3'd1
`define COND_PIN_LOW   3'd2
`define COND_PIN_HIGH  3'd3
`define COND_PIN_RISE  3'd4
`define COND_PIN_FALL  3'd5
`define COND_TX_EMPTY  3'd6
`define COND_RX_FULL   3'd7

// General register selectors.
`define REG_X          3'd0
`define REG_Y          3'd1
`define REG_LC         3'd2
`define REG_ZERO       3'd7

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

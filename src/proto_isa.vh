`ifndef PROTO_ISA_VH
`define PROTO_ISA_VH

// Protocol-emulator V1 ISA constants.
//
// Fixed architectural contract:
//   instruction width = 16 bits
//   opcode            = instr[15:11]
//   payload           = instr[10:0]
//
// Unused/reserved bits in V1 encodings must be zero.  instruction_decoder.v
// exposes instr_legal so malformed encodings trap instead of being silently
// accepted.  OP_CAN=0x19 is the CAN nominal-bit-timing extension; opcodes
// 0x1A..0x1F remain reserved for future architectural work.

// -----------------------------------------------------------------------------
// Primary opcodes
// -----------------------------------------------------------------------------
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
`define OP_ALU         5'h13
`define OP_EVENT       5'h14   // architecturally reserved, not implemented
`define OP_CRC         5'h15
`define OP_STREAM      5'h16
`define OP_LOAD        5'h17
`define OP_STORE       5'h18
`define OP_CAN         5'h19
`define OP_RSVD1A      5'h1A
`define OP_RSVD1B      5'h1B
`define OP_RSVD1C      5'h1C
`define OP_RSVD1D      5'h1D
`define OP_RSVD1E      5'h1E
`define OP_RSVD1F      5'h1F

// -----------------------------------------------------------------------------
// Shared byte-register selector used by new extensions
// -----------------------------------------------------------------------------
`define REG8_X          2'd0
`define REG8_Y          2'd1
`define REG8_OSR        2'd2
`define REG8_ISR        2'd3

// Backward-compatible selector names used by the ALU and LOAD/STORE formats.
`define ALU_REG_X       2'd0
`define ALU_REG_Y       2'd1
`define ALU_REG_OSR     2'd2
`define ALU_REG_ISR     2'd3
`define MEM_REG_X       2'd0
`define MEM_REG_Y       2'd1
`define MEM_REG_OSR     2'd2
`define MEM_REG_ISR     2'd3

// -----------------------------------------------------------------------------
// OP_ALU payload
//   [10:7] operation
//   [6:5]  destination REG8
//   [4:3]  source A REG8
//   [2:1]  source B REG8
//   [0]    reserved = 0
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Conditions used by JCC and WAITPIN
// -----------------------------------------------------------------------------
`define COND_ZERO       3'd0
`define COND_NZERO      3'd1
`define COND_PIN_LOW    3'd2
`define COND_PIN_HIGH   3'd3
`define COND_PIN_RISE   3'd4
`define COND_PIN_FALL   3'd5
`define COND_TX_EMPTY   3'd6
`define COND_RX_FULL    3'd7

// General register selectors used by LDI/MOV.
`define REG_X           3'd0
`define REG_Y           3'd1
`define REG_LC          3'd2
`define REG_ZERO        3'd7

// -----------------------------------------------------------------------------
// LOAD/STORE payload
//   [10:9] data register REG8
//   [8]    address base: 0=X, 1=Y
//   [7:0]  signed 8-bit offset
// Effective address = base + offset modulo 256.
// -----------------------------------------------------------------------------
`define MEM_BASE_X      1'b0
`define MEM_BASE_Y      1'b1

// -----------------------------------------------------------------------------
// OP_CRC payload
//   [10:8] CRC sub-operation
//   [7:0]  sub-operation argument
//
// Byte index convention for POLY/INIT/XOR/READ:
//   byte 0 = bits [7:0], byte 3 = bits [31:24].
// -----------------------------------------------------------------------------
`define CRC_CFG         3'd0
// arg[4:0] = width_minus_1 (0..31 => width 1..32)
// arg[5]   = reserved = 0
// arg[6]   = lsb_first (right-shifting/reflected polynomial convention)
// arg[7]   = reflect_out before xor_out
`define CRC_CTRL_LSB_FIRST    8'h40
`define CRC_CTRL_REFLECT_OUT  8'h80

`define CRC_POLY        3'd1
`define CRC_INIT        3'd2
`define CRC_XOR         3'd3
// POLY/INIT/XOR arg: [7:6]=REG8 source, [5:4]=byte index, [3:0]=0.

`define CRC_RESET       3'd4
// arg = 0.  Active state <= configured init value.

`define CRC_FEED8       3'd5
// arg: [7:6]=REG8 source, [5:0]=0.  Blocking; processes all 8 bits.

`define CRC_FEED1       3'd6
// arg: [7:6]=REG8 source, [5:3]=bit index 0..7, [2:0]=0.

`define CRC_READ        3'd7
// arg: [7:6]=REG8 destination, [5:4]=result byte index, [3:0]=0.
// READ returns finalized result = optional reflected state XOR xor_out.

// -----------------------------------------------------------------------------
// OP_STREAM payload
//   [10:8] STREAM sub-operation
//   [7:0]  sub-operation argument
//
// STREAM TX/RX are blocking and use X as a byte-addressed DMEM base pointer.
// DMEM bytes are packed according to STREAM_F_LSB_FIRST. X is NOT modified.
// TX/RX arg is logical bit_count_minus_1: 0=>1 bit, 255=>256 bits.
// -----------------------------------------------------------------------------
`define STREAM_CFG      3'd0
// Full 8-bit flag byte:
`define STREAM_F_LSB_FIRST       8'h01
`define STREAM_F_STUFF_ENABLE    8'h02
`define STREAM_F_STUFF_ANY       8'h04  // 1=CAN-style any-run, 0=USB ones-only
`define STREAM_F_NRZI_ENABLE     8'h08
`define STREAM_F_NRZI_INITIAL    8'h10
`define STREAM_F_MANCHESTER      8'h20
`define STREAM_F_MANCHESTER_INV  8'h40
`define STREAM_F_CRC_AUTO        8'h80

`define STREAM_STUFF_CFG 3'd1
// arg[2:0] = actual threshold 1..7; arg[7:3] = 0.

`define STREAM_TX       3'd2
// arg = logical bit_count_minus_1, source starts at DMEM[X].

`define STREAM_RX       3'd3
// arg = logical bit_count_minus_1, destination starts at DMEM[X].

`define STREAM_STATUS   3'd4
// arg[7:6] = REG8 destination; arg[5:0] = 0.
// status bits:
//   [0] busy
//   [1] error
//   [2] bit-stuff/destuff error
//   [3] Manchester decode error
//   [4] last command was RX
//   [5] CRC-auto configured
//   [6] Manchester configured
//   [7] NRZI configured

`define STREAM_CLEAR    3'd5
// arg = 0.  Clears sticky stream errors and packet codec state.

`define STREAM_RSVD6    3'd6
`define STREAM_RSVD7    3'd7



// -----------------------------------------------------------------------------
// OP_CAN -- CAN 2.0 nominal bit timing / synchronization
//   [10:8] sub-operation
//   [7:0]  argument
//
// cfg_in_pin is CAN RX/bus monitor. cfg_out_pin is CAN TX logical state.
// With open drain configured: 0=dominant, 1=recessive.
// -----------------------------------------------------------------------------
`define CAN_BRP          3'd0
// arg = clocks-per-TQ minus one, 0=>1 ... 255=>256.

`define CAN_SEG1         3'd1
// arg[7:4]=PROP_SEG-1 (0..7), arg[3:0]=PHASE_SEG1-1 (0..7).

`define CAN_SEG2         3'd2
// arg[7:4]=PHASE_SEG2-2 (0..6), arg[3:0]=SJW-1 (0..3).

`define CAN_CTRL         3'd3
`define CAN_F_ENABLE          8'h01
`define CAN_F_RESYNC_ENABLE   8'h02
`define CAN_F_RX_HARD_SYNC    8'h04
`define CAN_F_ARB_MONITOR     8'h08
// arg[7:4]=0.

`define CAN_ARB          3'd4
// arg = arbitration wire-bit count minus one, 0=>1 ... 255=>256.

`define CAN_STATUS       3'd5
// arg[7:6]=REG8 destination; arg[5:0]=0.
// [0] active
// [1] synchronized
// [2] config valid
// [3] last sampled bus bit
// [4] resync seen
// [5] RX hard sync seen
// [6] arbitration lost
// [7] waiting for hard-sync edge

`define CAN_PHASE        3'd6
// arg[7:6]=REG8 destination; arg[5:0]=0.
// signed last phase error in whole TQ.

`define CAN_CLEAR        3'd7
// arg=0; clears sticky timing/arbitration status.

// -----------------------------------------------------------------------------
// Byte-addressed configuration space used by OP_CFG
// -----------------------------------------------------------------------------
`define CFG_CLKDIV_LO   3'd0
`define CFG_CLKDIV_HI   3'd1
`define CFG_PINMAP0     3'd2
`define CFG_PINMAP1     3'd3
`define CFG_OD_MASK     3'd4
`define CFG_JMP_PIN     3'd5
`define CFG_RSVD6       3'd6
`define CFG_RSVD7       3'd7

`endif

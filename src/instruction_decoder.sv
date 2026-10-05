`default_nettype none

`include "proto_pkg.sv"

module instruction_decoder (
    input  wire [15:0] instr,

    output reg  [4:0]  opcode,
    output reg  [10:0] payload11,
    output reg  [2:0]  aux3,
    output reg  [7:0]  arg8,

    output reg  [2:0]  cond,
    output reg  [7:0]  branch_addr,
    output reg  [2:0]  waitpin_pin,

    output reg  [2:0]  reg_dst,
    output reg  [2:0]  reg_src,
    output reg  [7:0]  imm8,

    output reg  [2:0]  cfg_addr,
    output reg  [7:0]  cfg_data,

    output reg         opcode_known
);

    always @(*) begin
        opcode      = instr[15:11];
        payload11   = instr[10:0];
        aux3        = instr[10:8];
        arg8        = instr[7:0];

        // JCC: [10:8] condition, [7:0] target.
        cond        = instr[10:8];
        branch_addr = instr[7:0];

        // WAITPIN: [10:8] condition, [2:0] pin index.
        waitpin_pin = instr[2:0];

        // LDI: [10:8] destination, [7:0] immediate.
        // MOV: [10:8] destination, [7:5] source.
        reg_dst     = instr[10:8];
        reg_src     = instr[7:5];
        imm8        = instr[7:0];

        // CFG: [10:8] config byte address, [7:0] data.
        cfg_addr    = instr[10:8];
        cfg_data    = instr[7:0];

        case (opcode)
            `OP_NOP, `OP_HALT, `OP_WAIT, `OP_WAITPIN,
            `OP_GPIO_SET, `OP_GPIO_CLR, `OP_GPIO_WRITE, `OP_OE_WRITE,
            `OP_OUT, `OP_IN, `OP_XFER, `OP_PULL, `OP_PUSH,
            `OP_JMP, `OP_JCC, `OP_LOOP, `OP_LDI, `OP_MOV, `OP_CFG,
            `OP_ALU, `OP_EVENT, `OP_CRC, `OP_STREAM, `OP_LOAD, `OP_STORE:
                opcode_known = 1'b1;
            default:
                opcode_known = 1'b0;
        endcase
    end

endmodule

`default_nettype wire

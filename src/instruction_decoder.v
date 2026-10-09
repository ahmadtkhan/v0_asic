`default_nettype none
`include "proto_params.vh"
`include "proto_isa.vh"

// Pure combinational field decoder for the fixed 16-bit V0 ISA.
module instruction_decoder (
    input  wire [`PROTO_INSTR_W-1:0] instr,

    output wire [4:0]  opcode,
    output wire [10:0] payload11,
    output wire [2:0]  aux3,
    output wire [7:0]  arg8,

    output wire [2:0]  cond,
    output wire [7:0]  branch_addr,
    output wire [2:0]  waitpin_pin,

    output wire [2:0]  reg_dst,
    output wire [2:0]  reg_src,
    output wire [7:0]  imm8,

    output wire [2:0]  cfg_addr,
    output wire [7:0]  cfg_data,

    output reg          opcode_known
);

    assign opcode      = instr[15:11];
    assign payload11   = instr[10:0];
    assign aux3        = instr[10:8];
    assign arg8        = instr[7:0];

    // JCC: [10:8] condition, [7:0] target.
    assign cond        = instr[10:8];
    assign branch_addr = instr[7:0];

    // WAITPIN: [10:8] condition, [2:0] pin index.
    assign waitpin_pin = instr[2:0];

    // LDI: [10:8] destination, [7:0] immediate.
    // MOV: [10:8] destination, [7:5] source.
    assign reg_dst = instr[10:8];
    assign reg_src = instr[7:5];
    assign imm8    = instr[7:0];

    // CFG: [10:8] byte address, [7:0] data.
    assign cfg_addr = instr[10:8];
    assign cfg_data = instr[7:0];

    always @(*) begin
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

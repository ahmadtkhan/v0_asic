`default_nettype none
`include "proto_params.vh"
`include "proto_isa.vh"

// Pure combinational decoder for the fixed 16-bit V1 ISA.
// opcode_known answers whether the primary opcode is architecturally assigned.
// instr_legal additionally checks V1 reserved-bit/sub-operation rules.
module instruction_decoder (
    input  wire [`PROTO_INSTR_W-1:0] instr,

    output wire [4:0]  opcode,
    output wire [10:0] payload11,
    output wire [2:0]  aux3,
    output wire [2:0]  subop3,
    output wire [7:0]  arg8,

    output wire [2:0]  cond,
    output wire [7:0]  branch_addr,
    output wire [2:0]  waitpin_pin,

    output wire [2:0]  reg_dst,
    output wire [2:0]  reg_src,
    output wire [7:0]  imm8,

    output wire [2:0]  cfg_addr,
    output wire [7:0]  cfg_data,

    output wire [1:0]  mem_reg,
    output wire        mem_base,
    output wire [7:0]  mem_offset,

    output wire [3:0]  alu_op,
    output wire [1:0]  alu_dst,
    output wire [1:0]  alu_src_a,
    output wire [1:0]  alu_src_b,

    output reg          opcode_known,
    output reg          instr_legal
);

    assign opcode      = instr[15:11];
    assign payload11   = instr[10:0];
    assign aux3        = instr[10:8];
    assign subop3      = instr[10:8];
    assign arg8        = instr[7:0];

    assign cond        = instr[10:8];
    assign branch_addr = instr[7:0];
    assign waitpin_pin = instr[2:0];

    assign reg_dst = instr[10:8];
    assign reg_src = instr[7:5];
    assign imm8    = instr[7:0];

    assign cfg_addr = instr[10:8];
    assign cfg_data = instr[7:0];

    assign mem_reg    = instr[10:9];
    assign mem_base   = instr[8];
    assign mem_offset = instr[7:0];

    assign alu_op    = instr[10:7];
    assign alu_dst   = instr[6:5];
    assign alu_src_a = instr[4:3];
    assign alu_src_b = instr[2:1];

    always @(*) begin
        opcode_known = 1'b1;
        instr_legal  = 1'b1;

        case (opcode)
            `OP_NOP: begin
                // aux3 is delay/side-set; the lower byte is unused.
                instr_legal = (arg8 == 8'h00);
            end

            `OP_HALT: begin
                instr_legal = (payload11 == 11'h000);
            end

            `OP_WAIT: begin
            end

            `OP_WAITPIN: begin
                instr_legal = (instr[7:3] == 5'b00000);
            end

            `OP_GPIO_SET, `OP_GPIO_CLR, `OP_GPIO_WRITE, `OP_OE_WRITE: begin
                // All 11 payload bits are meaningful through arg8 + aux3.
            end

            `OP_OUT, `OP_IN, `OP_XFER: begin
                instr_legal = (arg8 == 8'h00);
            end

            `OP_PULL, `OP_PUSH: begin
                instr_legal = (payload11 == 11'h000);
            end

            `OP_JMP, `OP_LOOP: begin
                instr_legal = (instr[10:8] == 3'b000);
            end

            `OP_JCC: begin
            end

            `OP_LDI: begin
                instr_legal = (reg_dst == `REG_X) ||
                              (reg_dst == `REG_Y) ||
                              (reg_dst == `REG_LC);
            end

            `OP_MOV: begin
                instr_legal = ((reg_dst == `REG_X) ||
                               (reg_dst == `REG_Y) ||
                               (reg_dst == `REG_LC)) &&
                              ((reg_src == `REG_X) ||
                               (reg_src == `REG_Y) ||
                               (reg_src == `REG_LC)) &&
                              (instr[4:0] == 5'b00000);
            end

            `OP_CFG: begin
            end

            `OP_ALU: begin
                instr_legal = (instr[0] == 1'b0);
            end

            `OP_EVENT: begin
                // Assigned but not implemented.  Keep one canonical encoding.
                instr_legal = (payload11 == 11'h000);
            end

            `OP_CRC: begin
                case (subop3)
                    `CRC_CFG:   instr_legal = (arg8[5] == 1'b0);
                    `CRC_POLY,
                    `CRC_INIT,
                    `CRC_XOR:   instr_legal = (arg8[3:0] == 4'h0);
                    `CRC_RESET: instr_legal = (arg8 == 8'h00);
                    `CRC_FEED8: instr_legal = (arg8[5:0] == 6'b000000);
                    `CRC_FEED1: instr_legal = (arg8[2:0] == 3'b000);
                    `CRC_READ:  instr_legal = (arg8[3:0] == 4'h0);
                    default:    instr_legal = 1'b0;
                endcase
            end

            `OP_STREAM: begin
                case (subop3)
                    `STREAM_CFG: begin
                    end
                    `STREAM_STUFF_CFG:
                        instr_legal = (arg8[7:3] == 5'b00000) &&
                                      (arg8[2:0] != 3'b000);
                    `STREAM_TX, `STREAM_RX: begin
                    end
                    `STREAM_STATUS:
                        instr_legal = (arg8[5:0] == 6'b000000);
                    `STREAM_CLEAR:
                        instr_legal = (arg8 == 8'h00);
                    default:
                        instr_legal = 1'b0;
                endcase
            end

            `OP_LOAD, `OP_STORE: begin
            end

            default: begin
                opcode_known = 1'b0;
                instr_legal  = 1'b0;
            end
        endcase
    end

endmodule

`default_nettype wire

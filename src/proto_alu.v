`default_nettype none
`include "proto_params.vh"
`include "proto_isa.vh"

// Small combinational ALU for protocol firmware.
//
// Operands and result are one protocol data byte.  CMP and TEST update flags
// but do not write a destination register.  SHL/SHR use B[2:0] as a variable
// shift amount.  ROL/ROR are one-bit rotates, useful for bit-oriented firmware.
//
// V0 exposes only a zero flag architecturally.  Carry/overflow are deliberately
// not added here because the current JCC condition space has no spare encoding;
// protocol CRC/polynomial work is intended for the later CRC/LFSR block.
module proto_alu (
    input  wire [3:0]                   op,
    input  wire [`PROTO_DATA_W-1:0]     a,
    input  wire [`PROTO_DATA_W-1:0]     b,

    output reg  [`PROTO_DATA_W-1:0]     result,
    output wire                         zero,
    output reg                          write_result
);

    always @(*) begin
        result       = {`PROTO_DATA_W{1'b0}};
        write_result = 1'b1;

        case (op)
            `ALU_ADD:   result = a + b;
            `ALU_SUB:   result = a - b;
            `ALU_AND:   result = a & b;
            `ALU_OR:    result = a | b;
            `ALU_XOR:   result = a ^ b;

            // Variable logical shifts.  Only the low three bits are used for
            // an 8-bit datapath, so the shift amount is 0..7.
            `ALU_SHL:   result = a << b[2:0];
            `ALU_SHR:   result = a >> b[2:0];

            `ALU_NOT:   result = ~a;
            `ALU_INC:   result = a + {{(`PROTO_DATA_W-1){1'b0}}, 1'b1};
            `ALU_DEC:   result = a - {{(`PROTO_DATA_W-1){1'b0}}, 1'b1};

            // Flag-only operations.
            `ALU_CMP: begin
                result       = a - b;
                write_result = 1'b0;
            end

            `ALU_TEST: begin
                result       = a & b;
                write_result = 1'b0;
            end

            `ALU_ROL:   result = {a[`PROTO_DATA_W-2:0], a[`PROTO_DATA_W-1]};
            `ALU_ROR:   result = {a[0], a[`PROTO_DATA_W-1:1]};
            `ALU_PASS_A: result = a;
            `ALU_PASS_B: result = b;

            default: begin
                result       = {`PROTO_DATA_W{1'b0}};
                write_result = 1'b0;
            end
        endcase
    end

    assign zero = (result == {`PROTO_DATA_W{1'b0}});

endmodule

`default_nettype wire

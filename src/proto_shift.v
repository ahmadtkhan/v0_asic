`default_nettype none
`include "proto_params.vh"

// Shared input/output shift engine used by UART, SPI, I2C and future protocols.
module proto_shift #(
    parameter integer WIDTH = `PROTO_SHIFT_W
) (
    input  wire             clk,
    input  wire             rst_n,

    input  wire             osr_load,
    input  wire [WIDTH-1:0] osr_load_data,
    input  wire             osr_shift,
    input  wire             osr_shift_right,

    input  wire             isr_clear,
    input  wire             isr_shift,
    input  wire             isr_shift_right,
    input  wire             serial_in,

    output wire             serial_out,
    output reg  [WIDTH-1:0] osr,
    output reg  [WIDTH-1:0] isr
);

    assign serial_out = osr_shift_right ? osr[0] : osr[WIDTH-1];

    always @(posedge clk) begin
        if (!rst_n) begin
            osr <= {WIDTH{1'b0}};
            isr <= {WIDTH{1'b0}};
        end else begin
            if (osr_load) begin
                osr <= osr_load_data;
            end else if (osr_shift) begin
                if (osr_shift_right)
                    osr <= {1'b0, osr[WIDTH-1:1]};
                else
                    osr <= {osr[WIDTH-2:0], 1'b0};
            end

            if (isr_clear) begin
                isr <= {WIDTH{1'b0}};
            end else if (isr_shift) begin
                if (isr_shift_right)
                    isr <= {serial_in, isr[WIDTH-1:1]};
                else
                    isr <= {isr[WIDTH-2:0], serial_in};
            end
        end
    end

endmodule

`default_nettype wire

`default_nettype none
`include "proto_params.vh"

// Generates a clock-enable pulse; it does not create a derived clock domain.
module proto_tick #(
    parameter integer WIDTH = `PROTO_TICK_W
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire [WIDTH-1:0] divisor,
    output reg              tick
);

    reg [WIDTH-1:0] count;
    localparam [WIDTH-1:0] ZERO = {WIDTH{1'b0}};
    localparam [WIDTH-1:0] ONE  = {{(WIDTH-1){1'b0}}, 1'b1};

    always @(posedge clk) begin
        if (!rst_n) begin
            count <= ZERO;
            tick  <= 1'b0;
        end else begin
            tick <= 1'b0;

            // Divisors 0 and 1 intentionally mean one tick per system clock.
            if ((divisor == ZERO) || (divisor == ONE)) begin
                count <= ZERO;
                tick  <= 1'b1;
            end else if (count == (divisor - ONE)) begin
                count <= ZERO;
                tick  <= 1'b1;
            end else begin
                count <= count + ONE;
            end
        end
    end

endmodule

`default_nettype wire

//a clk enable program instead of a physical program
module proto_tick #(
    parameter int WIDTH = 16
) (
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire [WIDTH-1:0]     divisor,

    output reg                 tick
);

    reg [WIDTH-1:0] count;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            count <= '0;
            tick  <= 1'b0;
        end else begin
            tick <= 1'b0;

            // divisor = 0 or 1 means run every system clock
            if (divisor <= 1) begin
                count <= '0;
                tick  <= 1'b1;
            end else if (count == divisor - 1'b1) begin
                count <= '0;
                tick  <= 1'b1;
            end else begin
                count <= count + 1'b1;
            end
        end
    end

endmodule

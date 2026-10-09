`default_nettype none
`include "proto_params.vh"

// Common GPIO block for all emulated protocols.
// External inputs are synchronized into the core clock domain.  Output value
// and output-enable state are retained in registers.  Open-drain pins are
// represented logically as 0=drive low, 1=release.
module proto_gpio #(
    parameter integer N = `PROTO_GPIO_W
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire [N-1:0] pad_in,

    input  wire         out_we,
    input  wire [N-1:0] out_wdata,
    input  wire         oe_we,
    input  wire [N-1:0] oe_wdata,
    input  wire [N-1:0] open_drain_mask,

    output wire [N-1:0] pad_out,
    output wire [N-1:0] pad_oe,

    output wire [N-1:0] gpio_in,
    output reg  [N-1:0] gpio_out,
    output reg  [N-1:0] gpio_oe,
    output wire [N-1:0] rise,
    output wire [N-1:0] fall
);

    reg [N-1:0] sync1;
    reg [N-1:0] sync2;
    reg [N-1:0] prev;

    always @(posedge clk) begin
        if (!rst_n) begin
            sync1 <= {N{1'b0}};
            sync2 <= {N{1'b0}};
            prev  <= {N{1'b0}};
        end else begin
            sync1 <= pad_in;
            sync2 <= sync1;
            prev  <= sync2;
        end
    end

    assign gpio_in = sync2;
    assign rise    =  sync2 & ~prev;
    assign fall    = ~sync2 &  prev;

    always @(posedge clk) begin
        if (!rst_n) begin
            gpio_out <= {N{1'b0}};
            gpio_oe  <= {N{1'b0}};
        end else begin
            if (out_we)
                gpio_out <= out_wdata;
            if (oe_we)
                gpio_oe <= oe_wdata;
        end
    end

    // Push-pull pins pass the registered value/OE directly.
    // Open-drain pins always drive a physical 0 when enabled, and release the
    // line when their logical value is 1.
    assign pad_out = gpio_out & ~open_drain_mask;
    assign pad_oe  = gpio_oe & (~open_drain_mask | ~gpio_out);

endmodule

`default_nettype wire

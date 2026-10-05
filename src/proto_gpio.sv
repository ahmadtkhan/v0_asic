//a general purpose gpio program for synchronizing and managing IOs
module proto_gpio #(
    parameter int N = 8
) (
    input wire clk,
    input wire rst_n,

    input wire [N-1:0] pad_in,

    input wire out_we,
    input wire [N-1:0] out_wdata,

    input wire         oe_we,
    input wire [N-1:0] oe_wdata,

    input wire [N-1:0] open_drain_mask,

    // To physical Tiny Tapeout interface
    output reg [N-1:0] pad_out,
    output reg [N-1:0] pad_oe,

    // To protocol engine
    output wire [N-1:0] gpio_in,
    output reg [N-1:0] gpio_out,
    output reg [N-1:0] gpio_oe,
    output wire [N-1:0] rise,
    output wire [N-1:0] fall
);
  reg [N-1:0] sync1;
  reg [N-1:0] sync2;
  reg [N-1:0] prev;

  // Synchronize all external inputs.
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      sync1 <= '0;
      sync2 <= '0;
      prev  <= '0;
    end else begin
      sync1 <= pad_in;
      sync2 <= sync1;
      prev  <= sync2;
    end
  end

  assign gpio_in = sync2;

  assign rise = sync2 & ~prev;
  assign fall = ~sync2 & prev;

  // Logical GPIO state.
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      gpio_out <= '0;
      gpio_oe  <= '0;
    end else begin
      if (out_we) gpio_out <= out_wdata;

      if (oe_we) gpio_oe <= oe_wdata;
    end
  end
  // Physical output conversion.
  //
  // Push-pull:
  //    pad_out = requested value
  //    pad_oe  = requested OE
  //
  // Open drain:
  //    logical 0 -> drive 0

  always_comb begin
    for (int i = 0; i < N; i++) begin
      if (open_drain_mask[i]) begin
        pad_out[i] = 1'b0;
        pad_oe[i]  = gpio_oe[i] & ~gpio_out[i];
      end else begin
        pad_out[i] = gpio_out[i];
        pad_oe[i]  = gpio_oe[i];
      end
    end
  end
endmodule

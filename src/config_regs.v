`default_nettype none
`include "proto_params.vh"
`include "proto_isa.vh"

// Byte-addressed configuration bank.  Host writes are intended while the core
// is halted; firmware can modify the same bank through OP_CFG while running.
module config_regs (
    input  wire                      clk,
    input  wire                      rst_n,

    input  wire                      host_we,
    input  wire [`PROTO_CFG_AW-1:0]  host_addr,
    input  wire [`PROTO_DATA_W-1:0]  host_wdata,
    output wire [`PROTO_DATA_W-1:0]  host_rdata,

    input  wire                      core_we,
    input  wire [`PROTO_CFG_AW-1:0]  core_addr,
    input  wire [`PROTO_DATA_W-1:0]  core_wdata,

    output wire [`PROTO_TICK_W-1:0]  clkdiv,
    output wire [2:0]                out_pin,
    output wire [2:0]                in_pin,
    output wire [2:0]                side_pin,
    output wire                      out_shift_right,
    output wire                      in_shift_right,
    output wire                      side_enable,
    output wire                      side_target_oe,
    output wire [`PROTO_GPIO_W-1:0]  open_drain_mask,
    output wire [2:0]                jmp_pin
);

    reg [`PROTO_DATA_W-1:0] cfg [0:`PROTO_CFG_COUNT-1];

    wire                     wr_en;
    wire [`PROTO_CFG_AW-1:0] wr_addr;
    wire [`PROTO_DATA_W-1:0] wr_data;
    integer i;

    // Core writes take priority if both are asserted accidentally.
    assign wr_en   = core_we | host_we;
    assign wr_addr = core_we ? core_addr  : host_addr;
    assign wr_data = core_we ? core_wdata : host_wdata;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < `PROTO_CFG_COUNT; i = i + 1)
                cfg[i] <= {`PROTO_DATA_W{1'b0}};

            // Divider 1 = one protocol tick per system clock.
            cfg[`CFG_CLKDIV_LO] <= 8'h01;
            cfg[`CFG_CLKDIV_HI] <= 8'h00;
        end else if (wr_en) begin
            cfg[wr_addr] <= wr_data;
        end
    end

    assign host_rdata = cfg[host_addr];

    assign clkdiv = {cfg[`CFG_CLKDIV_HI], cfg[`CFG_CLKDIV_LO]};

    // CFG_PINMAP0:
    // [2:0] OUT pin, [5:3] IN pin, [6] OUT shift right, [7] IN shift right.
    assign out_pin         = cfg[`CFG_PINMAP0][2:0];
    assign in_pin          = cfg[`CFG_PINMAP0][5:3];
    assign out_shift_right = cfg[`CFG_PINMAP0][6];
    assign in_shift_right  = cfg[`CFG_PINMAP0][7];

    // CFG_PINMAP1:
    // [2:0] side-set pin, [3] side enable, [4] side targets OE instead of OUT.
    assign side_pin       = cfg[`CFG_PINMAP1][2:0];
    assign side_enable    = cfg[`CFG_PINMAP1][3];
    assign side_target_oe = cfg[`CFG_PINMAP1][4];

    assign open_drain_mask = cfg[`CFG_OD_MASK];
    assign jmp_pin         = cfg[`CFG_JMP_PIN][2:0];

endmodule

`default_nettype wire

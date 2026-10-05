`default_nettype none

`include "proto_pkg.sv"

module config_regs (
    input  wire        clk,
    input  wire        rst_n,

    // Host writes are intended while the core is halted.
    input  wire        host_we,
    input  wire [2:0]  host_addr,
    input  wire [7:0]  host_wdata,
    output wire [7:0]  host_rdata,

    // OP_CFG writes while firmware is running.
    input  wire        core_we,
    input  wire [2:0]  core_addr,
    input  wire [7:0]  core_wdata,

    output wire [15:0] clkdiv,
    output wire [2:0]  out_pin,
    output wire [2:0]  in_pin,
    output wire [2:0]  side_pin,
    output wire        out_shift_right,
    output wire        in_shift_right,
    output wire        side_enable,
    output wire        side_target_oe,
    output wire [7:0]  open_drain_mask,
    output wire [2:0]  jmp_pin
);

    reg [7:0] cfg [0:7];

    // Core has priority. Top-level should normally gate host writes while busy.
    wire       wr_en;
    wire [2:0] wr_addr;
    wire [7:0] wr_data;

    assign wr_en   = core_we ? 1'b1       : host_we;
    assign wr_addr = core_we ? core_addr  : host_addr;
    assign wr_data = core_we ? core_wdata : host_wdata;

    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 8; i = i + 1) begin
                cfg[i] <= 8'h00;
            end

            // Divider of one means one protocol tick per system clock.
            cfg[`CFG_CLKDIV_LO] <= 8'h01;
            cfg[`CFG_CLKDIV_HI] <= 8'h00;
        end else if (wr_en) begin
            cfg[wr_addr] <= wr_data;
        end
    end

    assign host_rdata = cfg[host_addr];

    assign clkdiv = {
        cfg[`CFG_CLKDIV_HI],
        cfg[`CFG_CLKDIV_LO]
    };

    // CFG_PINMAP0:
    // [2:0] OUT pin
    // [5:3] IN pin
    // [6]   OUT shift right
    // [7]   IN shift right
    assign out_pin         = cfg[`CFG_PINMAP0][2:0];
    assign in_pin          = cfg[`CFG_PINMAP0][5:3];
    assign out_shift_right = cfg[`CFG_PINMAP0][6];
    assign in_shift_right  = cfg[`CFG_PINMAP0][7];

    // CFG_PINMAP1:
    // [2:0] side-set pin
    // [3]   side enable
    // [4]   target OE rather than OUT
    assign side_pin       = cfg[`CFG_PINMAP1][2:0];
    assign side_enable    = cfg[`CFG_PINMAP1][3];
    assign side_target_oe = cfg[`CFG_PINMAP1][4];

    assign open_drain_mask = cfg[`CFG_OD_MASK];
    assign jmp_pin         = cfg[`CFG_JMP_PIN][2:0];

endmodule

`default_nettype wire

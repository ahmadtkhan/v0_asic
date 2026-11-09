`default_nettype none

// Test-only top level.  It bypasses the temporary Tiny Tapeout loader shim and
// exposes protocol_top's architectural host interface directly to cocotb.
module isa_tb;
    reg clk;
    reg rst_n;

    reg start;
    wire busy;
    wire fault;

    reg         prog_we;
    reg  [7:0]  prog_addr;
    reg  [15:0] prog_wdata;
    wire        prog_ready;

    reg         cfg_host_we;
    reg  [2:0]  cfg_host_addr;
    reg  [7:0]  cfg_host_wdata;
    wire [7:0]  cfg_host_rdata;
    wire        cfg_host_ready;

    reg         tx_push;
    reg  [7:0]  tx_data;
    wire        tx_full;

    reg         rx_pop;
    wire [7:0]  rx_data;
    wire        rx_empty;

    reg  [7:0]  pad_in;
    wire [7:0]  pad_out;
    wire [7:0]  pad_oe;

    protocol_top u_dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .busy           (busy),
        .fault          (fault),
        .prog_we        (prog_we),
        .prog_addr      (prog_addr),
        .prog_wdata     (prog_wdata),
        .prog_ready     (prog_ready),
        .cfg_host_we    (cfg_host_we),
        .cfg_host_addr  (cfg_host_addr),
        .cfg_host_wdata (cfg_host_wdata),
        .cfg_host_rdata (cfg_host_rdata),
        .cfg_host_ready (cfg_host_ready),
        .tx_push        (tx_push),
        .tx_data        (tx_data),
        .tx_full        (tx_full),
        .rx_pop         (rx_pop),
        .rx_data        (rx_data),
        .rx_empty       (rx_empty),
        .pad_in         (pad_in),
        .pad_out        (pad_out),
        .pad_oe         (pad_oe)
    );

`ifdef COCOTB_SIM
    initial begin
        $dumpfile("isa_tb.vcd");
        $dumpvars(0, isa_tb);
    end
`endif
endmodule

`default_nettype wire

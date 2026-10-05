`default_nettype none

module tt_um_protocol_top #(
    parameter integer IMEM_AW    = 8,
    parameter integer FIFO_DEPTH = 4
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // Core control/status.
    input  wire                 start,
    output wire                 busy,
    output wire                 fault,

    // Program loader. Writes are accepted only while !busy.
    input  wire                 prog_we,
    input  wire [IMEM_AW-1:0]   prog_addr,
    input  wire [15:0]          prog_wdata,
    output wire                 prog_ready,

    // Host configuration access. Writes are accepted only while !busy.
    input  wire                 cfg_host_we,
    input  wire [2:0]           cfg_host_addr,
    input  wire [7:0]           cfg_host_wdata,
    output wire [7:0]           cfg_host_rdata,
    output wire                 cfg_host_ready,

    // Host-facing data queues.
    input  wire                 tx_push,
    input  wire [7:0]           tx_data,
    output wire                 tx_full,

    input  wire                 rx_pop,
    output wire [7:0]           rx_data,
    output wire                 rx_empty,

    // Eight protocol GPIOs. Map these onto Tiny Tapeout uio_* in project.v.
    input  wire [7:0]           pad_in,
    output wire [7:0]           pad_out,
    output wire [7:0]           pad_oe
);

    // Instruction memory.
    wire                 imem_req;
    wire [IMEM_AW-1:0]   imem_addr;
    wire [15:0]          imem_rdata;
    wire                 imem_rvalid;

    // Configuration.
    wire                 core_cfg_we;
    wire [2:0]           core_cfg_addr;
    wire [7:0]           core_cfg_wdata;
    wire [15:0]          clkdiv;
    wire [2:0]           out_pin;
    wire [2:0]           in_pin;
    wire [2:0]           side_pin;
    wire                 out_shift_right;
    wire                 in_shift_right;
    wire                 side_enable;
    wire                 side_target_oe;
    wire [7:0]           open_drain_mask;
    wire [2:0]           jmp_pin;

    // Timing.
    wire tick;

    // GPIO internal signals.
    wire [7:0] gpio_in;
    wire [7:0] gpio_out;
    wire [7:0] gpio_oe;
    wire [7:0] gpio_rise;
    wire [7:0] gpio_fall;
    wire       gpio_out_we;
    wire [7:0] gpio_out_wdata;
    wire       gpio_oe_we;
    wire [7:0] gpio_oe_wdata;

    // Shift engine.
    wire       osr_load;
    wire [7:0] osr_load_data;
    wire       osr_shift;
    wire       isr_clear;
    wire       isr_shift;
    wire       shift_serial_in;
    wire       shift_serial_out;
    wire [7:0] osr_value;
    wire [7:0] isr_value;

    // TX FIFO.
    wire       tx_pop;
    wire [7:0] tx_pop_data;
    wire       tx_empty;

    // RX FIFO.
    wire       rx_push;
    wire [7:0] rx_push_data;
    wire       rx_full;

    assign prog_ready     = ~busy;
    assign cfg_host_ready = ~busy;

    instruction_memory #(
        .AW(IMEM_AW),
        .DEPTH(1 << IMEM_AW)
    ) u_imem (
        .clk         (clk),
        .rst_n       (rst_n),
        .prog_we     (prog_we & prog_ready),
        .prog_addr   (prog_addr),
        .prog_wdata  (prog_wdata),
        .core_req    (imem_req),
        .core_addr   (imem_addr),
        .core_rdata  (imem_rdata),
        .core_rvalid (imem_rvalid)
    );

    config_regs u_cfg (
        .clk             (clk),
        .rst_n           (rst_n),
        .host_we         (cfg_host_we & cfg_host_ready),
        .host_addr       (cfg_host_addr),
        .host_wdata      (cfg_host_wdata),
        .host_rdata      (cfg_host_rdata),
        .core_we         (core_cfg_we),
        .core_addr       (core_cfg_addr),
        .core_wdata      (core_cfg_wdata),
        .clkdiv          (clkdiv),
        .out_pin         (out_pin),
        .in_pin          (in_pin),
        .side_pin        (side_pin),
        .out_shift_right (out_shift_right),
        .in_shift_right  (in_shift_right),
        .side_enable     (side_enable),
        .side_target_oe  (side_target_oe),
        .open_drain_mask (open_drain_mask),
        .jmp_pin         (jmp_pin)
    );

    proto_tick #(
        .WIDTH(16)
    ) u_tick (
        .clk     (clk),
        .rst_n   (rst_n),
        .divisor (clkdiv),
        .tick    (tick)
    );

    proto_gpio #(
        .N(8)
    ) u_gpio (
        .clk             (clk),
        .rst_n           (rst_n),
        .pad_in          (pad_in),
        .out_we          (gpio_out_we),
        .out_wdata       (gpio_out_wdata),
        .oe_we           (gpio_oe_we),
        .oe_wdata        (gpio_oe_wdata),
        .open_drain_mask (open_drain_mask),
        .pad_out         (pad_out),
        .pad_oe          (pad_oe),
        .gpio_in         (gpio_in),
        .gpio_out        (gpio_out),
        .gpio_oe         (gpio_oe),
        .rise            (gpio_rise),
        .fall            (gpio_fall)
    );

    proto_shift #(
        .WIDTH(8)
    ) u_shift (
        .clk             (clk),
        .rst_n           (rst_n),
        .osr_load        (osr_load),
        .osr_load_data   (osr_load_data),
        .osr_shift       (osr_shift),
        .osr_shift_right (out_shift_right),
        .isr_clear       (isr_clear),
        .isr_shift       (isr_shift),
        .isr_shift_right (in_shift_right),
        .serial_in       (shift_serial_in),
        .serial_out      (shift_serial_out),
        .osr             (osr_value),
        .isr             (isr_value)
    );

    proto_fifo #(
        .WIDTH(8),
        .DEPTH(FIFO_DEPTH)
    ) u_tx_fifo (
        .clk       (clk),
        .rst_n     (rst_n),
        .push      (tx_push),
        .push_data (tx_data),
        .pop       (tx_pop),
        .pop_data  (tx_pop_data),
        .full      (tx_full),
        .empty     (tx_empty)
    );

    proto_fifo #(
        .WIDTH(8),
        .DEPTH(FIFO_DEPTH)
    ) u_rx_fifo (
        .clk       (clk),
        .rst_n     (rst_n),
        .push      (rx_push),
        .push_data (rx_push_data),
        .pop       (rx_pop),
        .pop_data  (rx_data),
        .full      (rx_full),
        .empty     (rx_empty)
    );

    protocol_core #(
        .IMEM_AW(IMEM_AW)
    ) u_core (
        .clk                 (clk),
        .rst_n               (rst_n),
        .start               (start),
        .busy                (busy),
        .fault               (fault),
        .imem_req            (imem_req),
        .imem_addr           (imem_addr),
        .imem_rdata          (imem_rdata),
        .imem_rvalid         (imem_rvalid),
        .tick                (tick),
        .cfg_out_pin         (out_pin),
        .cfg_in_pin          (in_pin),
        .cfg_side_pin        (side_pin),
        .cfg_out_shift_right (out_shift_right),
        .cfg_in_shift_right  (in_shift_right),
        .cfg_side_enable     (side_enable),
        .cfg_side_target_oe  (side_target_oe),
        .cfg_jmp_pin         (jmp_pin),
        .cfg_we              (core_cfg_we),
        .cfg_addr            (core_cfg_addr),
        .cfg_wdata           (core_cfg_wdata),
        .gpio_in             (gpio_in),
        .gpio_out            (gpio_out),
        .gpio_oe             (gpio_oe),
        .gpio_rise           (gpio_rise),
        .gpio_fall           (gpio_fall),
        .gpio_out_we         (gpio_out_we),
        .gpio_out_wdata      (gpio_out_wdata),
        .gpio_oe_we          (gpio_oe_we),
        .gpio_oe_wdata       (gpio_oe_wdata),
        .shift_serial_out    (shift_serial_out),
        .shift_isr           (isr_value),
        .osr_load            (osr_load),
        .osr_load_data       (osr_load_data),
        .osr_shift           (osr_shift),
        .osr_shift_right     (),
        .isr_clear           (isr_clear),
        .isr_shift           (isr_shift),
        .isr_shift_right     (),
        .shift_serial_in     (shift_serial_in),
        .tx_empty            (tx_empty),
        .tx_pop_data         (tx_pop_data),
        .tx_pop              (tx_pop),
        .rx_full             (rx_full),
        .rx_push             (rx_push),
        .rx_push_data        (rx_push_data)
    );

endmodule

`default_nettype wire

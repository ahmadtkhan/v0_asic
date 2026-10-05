`default_nettype none

module tt_um_protocol_top (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high)
    input  wire       ena,      // design enable (can connect to internal logic or leave unused)
    input  wire       clk,      // clock
    input  wire       rst_n     // reset (active low)
);

    // 1. Declare internal wires to bridge the pins to your core
    wire        start;
    wire        busy;
    wire        fault;
    wire        prog_we;
    reg [7:0]   pad_in;
    reg [7:0]   pad_out;
    reg [7:0]   pad_oe;

    // ... declare other internal signals here (prog_we, cfg_host_we, etc.) ...

    // 2. Map Tiny Tapeout's fixed 8-bit buses to your named signals
    // (Example pin assignment mapping - you must decide how to multiplex these!)
    assign start   = ui_in[0];
    assign prog_we = ui_in[1];

    assign uo_out[0] = busy;
    assign uo_out[1] = fault;

    // For the bidirectional pins (uio_*), map them to your pad wires:
    assign pad_in   = uio_in;   // Inputs coming from the external pads
    assign uio_out  = pad_out;  // Outputs going to the external pads
    assign uio_oe   = pad_oe;   // Output enables dictating input/output direction

    // 3. Instantiate your actual design block
    protocol_top#(
        .IMEM_AW(8),
        .FIFO_DEPTH(4)
    ) core_inst (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .busy           (busy),
        .fault          (fault),
        .prog_we        (prog_we),
        // ... connect all other pins ...
        .pad_in         (pad_in),
        .pad_out        (pad_out),
        .pad_oe         (pad_oe)
    );

endmodule

`default_nettype none
`include "proto_params.vh"

// Tiny Tapeout physical wrapper.
//
// This wrapper deliberately keeps the emulator core independent from the final
// host-loader protocol.  For the first GDS/bring-up flow it supplies a compact
// byte-command interface that reuses uio[7:0] as loader data while the core is
// idle, and releases those same pins to the protocol engine while it is busy.
//
// Host command format (sampled on a rising edge of ui_in[7]):
//   ui_in[6:4] command
//   ui_in[3]   output-select (0=status on uo_out, 1=last RX byte)
//   ui_in[2:0] command argument (CFG address for CFG_WRITE)
//   uio_in[7:0] command data
//
// Commands:
//   000 latch program low byte
//   001 write program high byte, commit 16-bit word, auto-increment address
//   010 write configuration byte (address=ui_in[2:0])
//   011 push one TX FIFO byte
//   100 pop/latch one RX FIFO byte
//   101 start core
//   110 reset program write address to zero
//   111 reserved/no-op
module tt_um_protocol_top (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    localparam [2:0] CMD_PROG_LO  = 3'b000;
    localparam [2:0] CMD_PROG_HI  = 3'b001;
    localparam [2:0] CMD_CFG      = 3'b010;
    localparam [2:0] CMD_TX_PUSH  = 3'b011;
    localparam [2:0] CMD_RX_POP   = 3'b100;
    localparam [2:0] CMD_START    = 3'b101;
    localparam [2:0] CMD_ADDR_RST = 3'b110;

    wire [2:0] host_cmd;
    wire [2:0] host_arg;
    wire       host_strobe;
    wire       show_rx_data;
    wire       host_pulse;

    reg host_strobe_d;

    reg                         start_r;
    reg                         prog_we_r;
    reg [`PROTO_IMEM_AW-1:0]    prog_addr_ptr;
    reg [`PROTO_IMEM_AW-1:0]    prog_addr_commit;
    reg [7:0]                   prog_low_byte;
    reg [`PROTO_INSTR_W-1:0]    prog_wdata_r;

    reg                         cfg_we_r;
    reg [`PROTO_CFG_AW-1:0]     cfg_addr_r;
    reg [`PROTO_DATA_W-1:0]     cfg_wdata_r;

    reg                         tx_push_r;
    reg [`PROTO_DATA_W-1:0]     tx_data_r;
    reg                         rx_pop_r;
    reg [`PROTO_DATA_W-1:0]     rx_latched;

    wire                        busy;
    wire                        fault;
    wire                        prog_ready;
    wire                        cfg_ready;
    wire [`PROTO_DATA_W-1:0]    cfg_rdata;
    wire                        tx_full;
    wire [`PROTO_DATA_W-1:0]    rx_data;
    wire                        rx_empty;
    wire [`PROTO_GPIO_W-1:0]    pad_out;
    wire [`PROTO_GPIO_W-1:0]    pad_oe;
    wire                        _unused;

    assign host_strobe = ui_in[7];
    assign host_cmd    = ui_in[6:4];
    assign show_rx_data = ui_in[3];
    assign host_arg    = ui_in[2:0];
    assign host_pulse  = host_strobe & ~host_strobe_d;

    // During idle/loading the bidirectional pins are released so an external
    // host may drive command data.  During execution they become protocol GPIO.
    assign uio_out = pad_out;
    assign uio_oe  = busy ? pad_oe : 8'h00;

    // Status format when ui_in[3]=0:
    // [7:6] reserved, [5] RX empty, [4] TX full, [3] CFG ready,
    // [2] program ready, [1] fault, [0] busy.
    assign uo_out = show_rx_data ? rx_latched :
                    {2'b00, rx_empty, tx_full, cfg_ready, prog_ready, fault, busy};

    always @(posedge clk) begin
        if (!rst_n) begin
            host_strobe_d   <= 1'b0;
            start_r          <= 1'b0;
            prog_we_r        <= 1'b0;
            prog_addr_ptr    <= {`PROTO_IMEM_AW{1'b0}};
            prog_addr_commit <= {`PROTO_IMEM_AW{1'b0}};
            prog_low_byte    <= 8'h00;
            prog_wdata_r     <= {`PROTO_INSTR_W{1'b0}};
            cfg_we_r         <= 1'b0;
            cfg_addr_r       <= {`PROTO_CFG_AW{1'b0}};
            cfg_wdata_r      <= {`PROTO_DATA_W{1'b0}};
            tx_push_r        <= 1'b0;
            tx_data_r        <= {`PROTO_DATA_W{1'b0}};
            rx_pop_r         <= 1'b0;
            rx_latched       <= {`PROTO_DATA_W{1'b0}};
        end else begin
            host_strobe_d <= host_strobe;

            // Command pulses default low and are asserted for one core clock.
            start_r   <= 1'b0;
            prog_we_r <= 1'b0;
            cfg_we_r  <= 1'b0;
            tx_push_r <= 1'b0;
            rx_pop_r  <= 1'b0;

            if (ena && host_pulse) begin
                case (host_cmd)
                    CMD_PROG_LO: begin
                        if (prog_ready)
                            prog_low_byte <= uio_in;
                    end

                    CMD_PROG_HI: begin
                        if (prog_ready) begin
                            prog_addr_commit <= prog_addr_ptr;
                            prog_wdata_r     <= {uio_in, prog_low_byte};
                            prog_we_r        <= 1'b1;
                            prog_addr_ptr    <= prog_addr_ptr + 1'b1;
                        end
                    end

                    CMD_CFG: begin
                        if (cfg_ready) begin
                            cfg_addr_r  <= host_arg;
                            cfg_wdata_r <= uio_in;
                            cfg_we_r    <= 1'b1;
                        end
                    end

                    CMD_TX_PUSH: begin
                        if (!tx_full) begin
                            tx_data_r <= uio_in;
                            tx_push_r <= 1'b1;
                        end
                    end

                    CMD_RX_POP: begin
                        if (!rx_empty) begin
                            rx_latched <= rx_data;
                            rx_pop_r   <= 1'b1;
                        end
                    end

                    CMD_START: begin
                        if (prog_ready)
                            start_r <= 1'b1;
                    end

                    CMD_ADDR_RST: begin
                        if (prog_ready)
                            prog_addr_ptr <= {`PROTO_IMEM_AW{1'b0}};
                    end

                    default: begin
                    end
                endcase
            end
        end
    end

    protocol_top u_protocol_top (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start_r),
        .busy           (busy),
        .fault          (fault),
        .prog_we        (prog_we_r),
        .prog_addr      (prog_addr_commit),
        .prog_wdata     (prog_wdata_r),
        .prog_ready     (prog_ready),
        .cfg_host_we    (cfg_we_r),
        .cfg_host_addr  (cfg_addr_r),
        .cfg_host_wdata (cfg_wdata_r),
        .cfg_host_rdata (cfg_rdata),
        .cfg_host_ready (cfg_ready),
        .tx_push        (tx_push_r),
        .tx_data        (tx_data_r),
        .tx_full        (tx_full),
        .rx_pop         (rx_pop_r),
        .rx_data        (rx_data),
        .rx_empty       (rx_empty),
        .pad_in         (uio_in),
        .pad_out        (pad_out),
        .pad_oe         (pad_oe)
    );

    // Readback is currently available through rx_latched; cfg_rdata is retained
    // for the internal host interface but not multiplexed onto the V0 TT pins.
    assign _unused = &{cfg_rdata, 1'b0};

endmodule

`default_nettype wire

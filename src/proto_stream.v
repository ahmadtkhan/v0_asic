`default_nettype none
`include "proto_params.vh"

// Blocking, DMEM-backed serial stream engine.
//
// STREAM TX reads bytes beginning at base_addr and emits them on line_bit.
// STREAM RX samples gpio_in_bit and writes bytes beginning at base_addr.
// bit_count_m1 encodes logical payload length minus one: 0 => 1 bit, 255 => 256 bits.
//
// Transform order is fixed and architectural:
//   TX: payload bit -> stuff -> NRZI -> Manchester -> pin
//   RX: pin -> Manchester -> NRZI -> destuff -> payload bit
//
// If crc_auto is configured, crc_bit_valid marks logical payload bits:
// before stuffing on TX and after destuffing on RX.  The stream engine does not
// reset/configure the CRC block; firmware controls CRC packet boundaries.
//
// Timing:
//   - Manchester disabled: one tick = one wire bit.
//   - Manchester enabled : one tick = one half-bit.
//
// Every start is packet-scoped: bit-stuff state and NRZI state are reset to the
// configured defaults.  The configured GPIO output-enable/open-drain behavior
// remains owned by proto_gpio and must be set by firmware before STREAM TX.
module proto_stream #(
    parameter integer AW = `PROTO_DMEM_AW
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    tick,

    input  wire                    cfg_we,
    input  wire [7:0]              cfg_data,
    input  wire                    stuff_cfg_we,
    input  wire [7:0]              stuff_cfg_data,
    input  wire                    clear_status,

    input  wire                    start_tx,
    input  wire                    start_rx,
    input  wire [AW-1:0]           base_addr,
    input  wire [7:0]              bit_count_m1,

    input  wire                    gpio_in_bit,
    output reg                     line_we,
    output reg                     line_bit,

    output reg                     dmem_req,
    output reg                     dmem_we,
    output reg  [AW-1:0]           dmem_addr,
    output reg  [`PROTO_DATA_W-1:0] dmem_wdata,
    input  wire [`PROTO_DATA_W-1:0] dmem_rdata,
    input  wire                    dmem_rvalid,

    output wire                    busy,
    output reg                     done,
    output wire                    error,
    output wire [7:0]              status,

    output reg                     crc_bit_valid,
    output reg                     crc_bit
);

    localparam [2:0] S_IDLE         = 3'd0;
    localparam [2:0] S_TX_REQ       = 3'd1;
    localparam [2:0] S_TX_WAIT      = 3'd2;
    localparam [2:0] S_TX_ACTIVE    = 3'd3;
    localparam [2:0] S_TX_NEXT_WAIT = 3'd4;
    localparam [2:0] S_RX_ACTIVE    = 3'd5;

    reg [2:0] state;

    reg [7:0] stream_cfg;
    reg [2:0] stuff_threshold;

    wire cfg_lsb_first;
    wire cfg_stuff_enable;
    wire cfg_stuff_any;
    wire cfg_nrzi_enable;
    wire cfg_nrzi_initial;
    wire cfg_manchester_enable;
    wire cfg_manchester_invert;
    wire cfg_crc_auto;

    assign cfg_lsb_first        = stream_cfg[0];
    assign cfg_stuff_enable     = stream_cfg[1];
    assign cfg_stuff_any        = stream_cfg[2];
    assign cfg_nrzi_enable      = stream_cfg[3];
    assign cfg_nrzi_initial     = stream_cfg[4];
    assign cfg_manchester_enable= stream_cfg[5];
    assign cfg_manchester_invert= stream_cfg[6];
    assign cfg_crc_auto         = stream_cfg[7];

    reg [AW-1:0] cur_addr;
    reg [8:0]    bits_remaining;
    reg [2:0]    bit_index;

    reg [7:0] tx_cur_byte;
    reg [7:0] tx_next_byte;
    reg       tx_next_valid;
    reg       tx_prefetch_pending;

    reg [7:0] rx_byte;
    reg       man_phase;
    reg       man_second_hold;
    reg       man_consume_hold;
    reg       rx_first_hold;

    reg error_sticky;
    reg stuff_error_sticky;
    reg man_error_sticky;
    reg last_was_rx;

    wire codec_clear;
    assign codec_clear = clear_status | start_tx | start_rx;

    wire tx_payload_bit;
    assign tx_payload_bit = cfg_lsb_first ? tx_cur_byte[bit_index]
                                          : tx_cur_byte[3'd7 - bit_index];

    wire is_rx;
    assign is_rx = (state == S_RX_ACTIVE);

    // Manchester RX produces one coded line bit after every pair of half-bits.
    wire man_rx_data_bit;
    wire man_rx_error;
    wire man_tx_first;
    wire man_tx_second;

    wire rx_wire_step;
    assign rx_wire_step = (state == S_RX_ACTIVE) && tick &&
                          (!cfg_manchester_enable || man_phase);

    wire rx_line_bit;
    assign rx_line_bit = cfg_manchester_enable ? man_rx_data_bit : gpio_in_bit;

    // Stuffing and NRZI share one packet-scoped state because STREAM is either
    // TX or RX, never both simultaneously.
    wire bs_step;
    wire bs_in_bit;
    wire bs_out_valid;
    wire bs_out_bit;
    wire bs_consume;
    wire bs_error_now;

    wire tx_new_wire_bit_step;
    assign tx_new_wire_bit_step = (state == S_TX_ACTIVE) && tick &&
                                  (!cfg_manchester_enable || !man_phase);

    wire nrzi_step;
    wire nrzi_in_bit;
    wire nrzi_out_bit;

    assign nrzi_step   = tx_new_wire_bit_step | rx_wire_step;
    assign nrzi_in_bit = is_rx ? rx_line_bit : bs_out_bit;

    assign bs_step   = tx_new_wire_bit_step | rx_wire_step;
    assign bs_in_bit = is_rx ? (cfg_nrzi_enable ? nrzi_out_bit : rx_line_bit)
                             : tx_payload_bit;

    proto_bitstuff u_bitstuff (
        .clk           (clk),
        .rst_n         (rst_n),
        .clear         (codec_clear),
        .enable        (cfg_stuff_enable),
        .destuff       (is_rx),
        .mode_any      (cfg_stuff_any),
        .threshold     (stuff_threshold),
        .step          (bs_step),
        .in_bit        (bs_in_bit),
        .out_valid     (bs_out_valid),
        .out_bit       (bs_out_bit),
        .consume_input (bs_consume),
        .inserted      (),
        .dropped       (),
        .error_now     (bs_error_now),
        .error         ()
    );

    proto_nrzi u_nrzi (
        .clk           (clk),
        .rst_n         (rst_n),
        .clear         (codec_clear),
        .decode        (is_rx),
        .initial_level (cfg_nrzi_initial),
        .step          (nrzi_step & cfg_nrzi_enable),
        .in_bit        (nrzi_in_bit),
        .out_bit       (nrzi_out_bit),
        .level_state   ()
    );

    wire tx_line_coded_bit;
    assign tx_line_coded_bit = cfg_nrzi_enable ? nrzi_out_bit : bs_out_bit;

    proto_manchester u_manchester (
        .invert         (cfg_manchester_invert),
        .tx_data_bit    (tx_line_coded_bit),
        .tx_first_half  (man_tx_first),
        .tx_second_half (man_tx_second),
        .rx_first_half  (rx_first_hold),
        .rx_second_half (gpio_in_bit),
        .rx_data_bit    (man_rx_data_bit),
        .rx_error       (man_rx_error)
    );

    wire rx_payload_event;
    assign rx_payload_event = rx_wire_step && bs_out_valid &&
                              !bs_error_now &&
                              !(cfg_manchester_enable && man_rx_error);

    reg [7:0] rx_byte_with_bit;
    always @(*) begin
        rx_byte_with_bit = rx_byte;
        if (cfg_lsb_first)
            rx_byte_with_bit[bit_index] = bs_out_bit;
        else
            rx_byte_with_bit[3'd7 - bit_index] = bs_out_bit;
    end

    wire rx_complete_byte;
    assign rx_complete_byte = rx_payload_event && (bit_index == 3'd7);

    // A payload bit advances only when the stuffer consumed it.  An inserted
    // wire bit intentionally leaves bit_index unchanged.
    wire tx_payload_advance;
    assign tx_payload_advance = (state == S_TX_ACTIVE) && tick &&
        ((!cfg_manchester_enable && bs_consume) ||
         ( cfg_manchester_enable && man_phase && man_consume_hold));

    wire tx_complete_byte;
    assign tx_complete_byte = tx_payload_advance && (bit_index == 3'd7);

    wire need_prefetch;
    assign need_prefetch = (state == S_TX_ACTIVE) &&
                           (bits_remaining > 9'd8) &&
                           !tx_next_valid && !tx_prefetch_pending;

    assign busy  = (state != S_IDLE);
    assign error = error_sticky;
    assign status = {
        cfg_nrzi_enable,          // [7]
        cfg_manchester_enable,    // [6]
        cfg_crc_auto,             // [5]
        last_was_rx,              // [4]
        man_error_sticky,         // [3]
        stuff_error_sticky,       // [2]
        error_sticky,             // [1]
        busy                      // [0]
    };

    // Combinational side effects to GPIO, DMEM and CRC.
    always @(*) begin
        line_we       = 1'b0;
        line_bit      = 1'b0;

        dmem_req      = 1'b0;
        dmem_we       = 1'b0;
        dmem_addr     = cur_addr;
        dmem_wdata    = rx_byte_with_bit;

        crc_bit_valid = 1'b0;
        crc_bit       = 1'b0;

        if (state == S_TX_REQ) begin
            dmem_req  = 1'b1;
            dmem_we   = 1'b0;
            dmem_addr = cur_addr;
        end else if (need_prefetch) begin
            dmem_req  = 1'b1;
            dmem_we   = 1'b0;
            dmem_addr = cur_addr + 1'b1;
        end

        if ((state == S_TX_ACTIVE) && tick) begin
            if (cfg_manchester_enable) begin
                line_we = 1'b1;
                if (!man_phase)
                    line_bit = man_tx_first;
                else
                    line_bit = man_second_hold;
            end else begin
                line_we  = 1'b1;
                line_bit = tx_line_coded_bit;
            end

            if (cfg_crc_auto && tx_new_wire_bit_step && bs_consume) begin
                crc_bit_valid = 1'b1;
                crc_bit       = tx_payload_bit;
            end
        end

        if (state == S_RX_ACTIVE) begin
            if (rx_payload_event && ((bit_index == 3'd7) || (bits_remaining == 9'd1))) begin
                // STORE full bytes and also the final partial byte on this same
                // clock edge. Unused bits in a partial final byte remain zero.
                dmem_req   = 1'b1;
                dmem_we    = 1'b1;
                dmem_addr  = cur_addr;
                dmem_wdata = rx_byte_with_bit;
            end

            if (cfg_crc_auto && rx_payload_event) begin
                crc_bit_valid = 1'b1;
                crc_bit       = bs_out_bit;
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            state               <= S_IDLE;
            stream_cfg          <= 8'h10; // NRZI initial level=1, all transforms disabled
            stuff_threshold     <= 3'd5;
            cur_addr            <= {AW{1'b0}};
            bits_remaining      <= 9'd0;
            bit_index           <= 3'd0;
            tx_cur_byte         <= 8'h00;
            tx_next_byte        <= 8'h00;
            tx_next_valid       <= 1'b0;
            tx_prefetch_pending <= 1'b0;
            rx_byte             <= 8'h00;
            man_phase           <= 1'b0;
            man_second_hold     <= 1'b0;
            man_consume_hold    <= 1'b0;
            rx_first_hold       <= 1'b0;
            error_sticky        <= 1'b0;
            stuff_error_sticky  <= 1'b0;
            man_error_sticky    <= 1'b0;
            last_was_rx         <= 1'b0;
            done                <= 1'b0;
        end else begin
            done <= 1'b0;

            if ((state == S_IDLE) && cfg_we)
                stream_cfg <= cfg_data;

            if ((state == S_IDLE) && stuff_cfg_we)
                stuff_threshold <= stuff_cfg_data[2:0];

            if ((state == S_IDLE) && clear_status) begin
                error_sticky       <= 1'b0;
                stuff_error_sticky <= 1'b0;
                man_error_sticky   <= 1'b0;
            end

            if ((state == S_IDLE) && (start_tx || start_rx)) begin
                cur_addr            <= base_addr;
                bits_remaining      <= {1'b0, bit_count_m1} + 9'd1;
                bit_index           <= 3'd0;
                tx_next_valid       <= 1'b0;
                tx_prefetch_pending <= 1'b0;
                rx_byte             <= 8'h00;
                man_phase           <= 1'b0;
                man_second_hold     <= 1'b0;
                man_consume_hold    <= 1'b0;
                rx_first_hold       <= 1'b0;
                error_sticky        <= 1'b0;
                stuff_error_sticky  <= 1'b0;
                man_error_sticky    <= 1'b0;
                last_was_rx         <= start_rx;

                if (start_tx)
                    state <= S_TX_REQ;
                else
                    state <= S_RX_ACTIVE;
            end else begin
                case (state)
                    S_IDLE: begin
                    end

                    S_TX_REQ: begin
                        // Combinational dmem_req above is accepted here.
                        state <= S_TX_WAIT;
                    end

                    S_TX_WAIT: begin
                        if (dmem_rvalid) begin
                            tx_cur_byte <= dmem_rdata;
                            bit_index   <= 3'd0;
                            state       <= S_TX_ACTIVE;
                        end
                    end

                    S_TX_ACTIVE: begin
                        // Capture an outstanding prefetch response.
                        if (dmem_rvalid && tx_prefetch_pending) begin
                            tx_next_byte        <= dmem_rdata;
                            tx_next_valid       <= 1'b1;
                            tx_prefetch_pending <= 1'b0;
                        end

                        if (need_prefetch)
                            tx_prefetch_pending <= 1'b1;

                        if (tick && cfg_manchester_enable) begin
                            if (!man_phase) begin
                                man_second_hold  <= man_tx_second;
                                man_consume_hold <= bs_consume;
                                man_phase        <= 1'b1;
                            end else begin
                                man_phase <= 1'b0;
                            end
                        end

                        if (tx_payload_advance) begin
                            if (bits_remaining == 9'd1) begin
                                bits_remaining <= 9'd0;
                                state          <= S_IDLE;
                                done           <= 1'b1;
                            end else begin
                                bits_remaining <= bits_remaining - 1'b1;
                                if (tx_complete_byte) begin
                                    cur_addr  <= cur_addr + 1'b1;
                                    bit_index <= 3'd0;

                                    if (tx_next_valid) begin
                                        tx_cur_byte   <= tx_next_byte;
                                        tx_next_valid <= 1'b0;
                                    end else if (dmem_rvalid && tx_prefetch_pending) begin
                                        // Response arrived exactly on the byte boundary.
                                        tx_cur_byte         <= dmem_rdata;
                                        tx_next_valid       <= 1'b0;
                                        tx_prefetch_pending <= 1'b0;
                                    end else begin
                                        // Should be rare because one byte gives the SRAM
                                        // several clocks to satisfy the prefetch.
                                        state <= S_TX_NEXT_WAIT;
                                    end
                                end else begin
                                    bit_index <= bit_index + 1'b1;
                                end
                            end
                        end
                    end

                    S_TX_NEXT_WAIT: begin
                        if (dmem_rvalid && tx_prefetch_pending) begin
                            tx_cur_byte         <= dmem_rdata;
                            tx_prefetch_pending <= 1'b0;
                            bit_index           <= 3'd0;
                            state               <= S_TX_ACTIVE;
                        end
                    end

                    S_RX_ACTIVE: begin
                        if (tick && cfg_manchester_enable) begin
                            if (!man_phase) begin
                                rx_first_hold <= gpio_in_bit;
                                man_phase     <= 1'b1;
                            end else begin
                                man_phase <= 1'b0;
                                if (man_rx_error) begin
                                    error_sticky     <= 1'b1;
                                    man_error_sticky <= 1'b1;
                                    state            <= S_IDLE;
                                    done             <= 1'b1;
                                end
                            end
                        end

                        if (rx_wire_step && bs_error_now) begin
                            error_sticky       <= 1'b1;
                            stuff_error_sticky <= 1'b1;
                            state              <= S_IDLE;
                            done               <= 1'b1;
                        end else if (rx_payload_event) begin
                            if (bits_remaining == 9'd1) begin
                                bits_remaining <= 9'd0;
                                rx_byte        <= 8'h00;
                                bit_index      <= 3'd0;
                                state          <= S_IDLE;
                                done           <= 1'b1;
                            end else begin
                                bits_remaining <= bits_remaining - 1'b1;
                                if (bit_index == 3'd7) begin
                                    rx_byte   <= 8'h00;
                                    bit_index <= 3'd0;
                                    cur_addr  <= cur_addr + 1'b1;
                                end else begin
                                    rx_byte   <= rx_byte_with_bit;
                                    bit_index <= bit_index + 1'b1;
                                end
                            end
                        end
                    end

                    default: begin
                        error_sticky <= 1'b1;
                        state        <= S_IDLE;
                        done         <= 1'b1;
                    end
                endcase
            end
        end
    end

endmodule

`default_nettype wire

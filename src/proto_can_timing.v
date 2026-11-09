`default_nettype none

// CAN 2.0 nominal bit-timing engine.
//
// Implements the timing layer:
//   * programmable time-quantum prescaler
//   * SYNC_SEG fixed at 1 TQ
//   * PROP_SEG / PHASE_SEG1 / PHASE_SEG2
//   * sample point at the end of PHASE_SEG1
//   * RX hard synchronization
//   * recessive-to-dominant resynchronization
//   * SJW-limited PHASE_SEG1 extension / PHASE_SEG2 shortening
//   * optional arbitration-window monitoring
//
// Frame parsing, ACK/error-frame generation and TEC/REC confinement are not
// implemented here.
//
// Encoding:
//   BRP      : clocks-per-TQ minus one, 0=>1 ... 255=>256
//   SEG1     : [7:4]=PROP_SEG-1,   [3:0]=PHASE_SEG1-1
//   SEG2     : [7:4]=PHASE_SEG2-2, [3:0]=SJW-1
//   CTRL[0]  : enable
//   CTRL[1]  : resynchronization enable
//   CTRL[2]  : STREAM_RX waits for SOF recessive->dominant hard sync
//   CTRL[3]  : arbitration monitor enable
//
// Nominal bitrate:
//   Fbit = Fclk / ((BRP+1) * (1 + PROP + PH1 + PH2))
module proto_can_timing (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       brp_we,
    input  wire [7:0] brp_data,
    input  wire       seg1_we,
    input  wire [7:0] seg1_data,
    input  wire       seg2_we,
    input  wire [7:0] seg2_data,
    input  wire       ctrl_we,
    input  wire [7:0] ctrl_data,
    input  wire       arb_we,
    input  wire [7:0] arb_bits_m1_data,
    input  wire       clear_status,

    input  wire       tx_start,
    input  wire       rx_start,
    input  wire       stream_stop,

    input  wire       bus_level,
    input  wire       bus_fall,
    input  wire       tx_level,

    output wire       can_enable,
    output wire       cfg_valid,
    output wire       bit_tick,
    output wire       sample_strobe,
    output reg        sampled_bit,
    output wire       arbitration_lost_now,
    output wire [7:0] status,
    output reg  [7:0] last_phase_error
);

    reg [7:0] brp_m1;
    reg [3:0] prop_m1;
    reg [3:0] phase1_m1;
    reg [3:0] phase2_m2;
    reg [3:0] sjw_m1;
    reg [3:0] ctrl;
    reg [7:0] arb_bits_m1;

    wire [4:0] prop_seg;
    wire [4:0] phase1_seg;
    wire [4:0] phase2_seg;
    wire [4:0] sjw;
    wire [5:0] nominal_sample_index;
    wire [5:0] nominal_end_index;
    wire [5:0] total_tq;

    assign prop_seg   = {1'b0, prop_m1}   + 5'd1;
    assign phase1_seg = {1'b0, phase1_m1} + 5'd1;
    assign phase2_seg = {1'b0, phase2_m2} + 5'd2;
    assign sjw        = {1'b0, sjw_m1}    + 5'd1;

    // 0-based TQ index:
    // 0 is SYNC_SEG.  Sample occurs after PROP + PH1.  Bit end occurs after
    // PROP + PH1 + PH2.
    assign nominal_sample_index = prop_seg + phase1_seg;
    assign nominal_end_index    = prop_seg + phase1_seg + phase2_seg;
    assign total_tq             = nominal_end_index + 6'd1;

    assign can_enable = ctrl[0];

    // CTRL[7:4] are architecturally reserved.
    wire _unused_ctrl_reserved = 1'b0 && &{1'b0, ctrl_data[7:4]};

    assign cfg_valid =
        (prop_seg   >= 5'd1) && (prop_seg   <= 5'd8) &&
        (phase1_seg >= 5'd1) && (phase1_seg <= 5'd8) &&
        (phase2_seg >= 5'd2) && (phase2_seg <= 5'd8) &&
        (sjw        >= 5'd1) && (sjw        <= 5'd4) &&
        (sjw <= phase1_seg) &&
        (sjw <= phase2_seg) &&
        (total_tq >= 6'd8) && (total_tq <= 6'd25);

    reg       active;
    reg       synced;
    reg       waiting_hard_sync;
    reg       tx_monitor_active;

    reg [7:0] brp_count;
    reg [5:0] tq_index;
    reg [4:0] phase1_extend;
    reg [4:0] phase2_shorten;
    reg       resync_used;

    reg [8:0] tx_wire_index;

    reg resync_seen;
    reg hard_sync_seen;
    reg arbitration_lost;

    wire tq_expire;
    assign tq_expire = active && (brp_count == brp_m1);

    wire [5:0] cur_sample_index;
    wire [5:0] cur_end_index;

    assign cur_sample_index =
        nominal_sample_index + {1'b0, phase1_extend};
    assign cur_end_index =
        nominal_end_index + {1'b0, phase1_extend} - {1'b0, phase2_shorten};

    // One synchronization is allowed between two sample points.  Only
    // recessive->dominant edges are supplied through bus_fall.
    wire resync_common;
    assign resync_common =
        can_enable && cfg_valid && ctrl[1] &&
        active && synced && !waiting_hard_sync &&
        !resync_used && bus_fall && !tx_start;

    wire positive_edge;
    wire negative_edge;

    // A transmitter already driving dominant ignores a positive edge because
    // it can simply be its own bus transition.
    assign positive_edge =
        resync_common &&
        (tq_index != 6'd0) &&
        (tq_index <= cur_sample_index) &&
        !(tx_monitor_active && (tx_level == 1'b0));

    assign negative_edge =
        resync_common &&
        (tq_index > cur_sample_index) &&
        (tq_index <= cur_end_index);

    reg [5:0] positive_error_mag;
    reg [5:0] negative_error_mag;
    reg [4:0] positive_adjust;
    reg [4:0] negative_adjust;
    reg [5:0] phase2_short_limit;

    always @(*) begin
        positive_error_mag = tq_index;

        if (cur_end_index >= tq_index)
            negative_error_mag = (cur_end_index - tq_index) + 6'd1;
        else
            negative_error_mag = 6'd0;

        if (positive_error_mag > {1'b0, sjw})
            positive_adjust = sjw;
        else
            positive_adjust = positive_error_mag[4:0];

        if (phase2_seg > 5'd1)
            phase2_short_limit = {1'b0, (phase2_seg - 5'd1)};
        else
            phase2_short_limit = 6'd0;

        if (negative_error_mag > {1'b0, sjw})
            negative_adjust = sjw;
        else
            negative_adjust = negative_error_mag[4:0];

        if ({1'b0, negative_adjust} > phase2_short_limit)
            negative_adjust = phase2_short_limit[4:0];
    end

    wire [5:0] sample_index_with_edge;
    wire [5:0] end_index_with_edge;

    assign sample_index_with_edge =
        cur_sample_index + (positive_edge ? {1'b0, positive_adjust} : 6'd0);

    assign end_index_with_edge =
        cur_end_index
        + (positive_edge ? {1'b0, positive_adjust} : 6'd0)
        - (negative_edge ? {1'b0, negative_adjust} : 6'd0);

    assign sample_strobe =
        tq_expire && (tq_index == sample_index_with_edge);

    wire bit_boundary;
    assign bit_boundary =
        tq_expire && (tq_index == end_index_with_edge);

    // tx_start is the synthetic first boundary.  proto_stream raises it only
    // after the first DMEM byte has been fetched.
    assign bit_tick =
        (tx_start && can_enable && cfg_valid) || bit_boundary;

    // CAN arbitration loss: transmit recessive, sample dominant.
    assign arbitration_lost_now =
        sample_strobe &&
        tx_monitor_active &&
        ctrl[3] &&
        (tx_wire_index <= {1'b0, arb_bits_m1}) &&
        (tx_level == 1'b1) &&
        (bus_level == 1'b0);

    assign status = {
        waiting_hard_sync,   // [7]
        arbitration_lost,    // [6]
        hard_sync_seen,      // [5]
        resync_seen,         // [4]
        sampled_bit,         // [3]
        cfg_valid,           // [2]
        synced,              // [1]
        active               // [0]
    };

    always @(posedge clk) begin
        if (!rst_n) begin
            // Valid reset geometry: 8 TQ/bit, sample point 75%.
            brp_m1            <= 8'd0; // 1 clk/TQ
            prop_m1           <= 4'd0; // 1 TQ
            phase1_m1         <= 4'd3; // 4 TQ
            phase2_m2         <= 4'd0; // 2 TQ
            sjw_m1            <= 4'd0; // 1 TQ
            ctrl              <= 4'b0000;
            arb_bits_m1       <= 8'd0;

            active            <= 1'b0;
            synced            <= 1'b0;
            waiting_hard_sync <= 1'b0;
            tx_monitor_active <= 1'b0;
            brp_count         <= 8'd0;
            tq_index          <= 6'd0;
            phase1_extend     <= 5'd0;
            phase2_shorten    <= 5'd0;
            resync_used       <= 1'b0;
            tx_wire_index     <= 9'd0;

            sampled_bit       <= 1'b1;
            resync_seen       <= 1'b0;
            hard_sync_seen    <= 1'b0;
            arbitration_lost  <= 1'b0;
            last_phase_error  <= 8'h00;
        end else begin
            if (brp_we)
                brp_m1 <= brp_data;

            if (seg1_we) begin
                prop_m1   <= seg1_data[7:4];
                phase1_m1 <= seg1_data[3:0];
            end

            if (seg2_we) begin
                phase2_m2 <= seg2_data[7:4];
                sjw_m1    <= seg2_data[3:0];
            end

            if (arb_we)
                arb_bits_m1 <= arb_bits_m1_data;

            if (ctrl_we) begin
                ctrl <= ctrl_data[3:0];
                if (!ctrl_data[0]) begin
                    active            <= 1'b0;
                    synced            <= 1'b0;
                    waiting_hard_sync <= 1'b0;
                    tx_monitor_active <= 1'b0;
                    brp_count         <= 8'd0;
                    tq_index          <= 6'd0;
                end
            end

            if (clear_status) begin
                resync_seen      <= 1'b0;
                hard_sync_seen   <= 1'b0;
                arbitration_lost <= 1'b0;
                last_phase_error <= 8'h00;
            end

            if (sample_strobe) begin
                sampled_bit <= bus_level;
                resync_used <= 1'b0;
            end

            if (arbitration_lost_now) begin
                arbitration_lost  <= 1'b1;
                tx_monitor_active <= 1'b0;
            end

            if (stream_stop) begin
                active            <= 1'b0;
                synced            <= 1'b0;
                waiting_hard_sync <= 1'b0;
                tx_monitor_active <= 1'b0;
                brp_count         <= 8'd0;
                tq_index          <= 6'd0;
                phase1_extend     <= 5'd0;
                phase2_shorten    <= 5'd0;
                resync_used       <= 1'b0;
            end

            // TX starts on a local bit boundary.
            if (tx_start && can_enable && cfg_valid) begin
                active            <= 1'b1;
                synced            <= 1'b1;
                waiting_hard_sync <= 1'b0;
                tx_monitor_active <= 1'b1;
                brp_count         <= 8'd0;
                tq_index          <= 6'd0;
                phase1_extend     <= 5'd0;
                phase2_shorten    <= 5'd0;
                resync_used       <= 1'b0;
                tx_wire_index     <= 9'd0;
            end else if (rx_start && can_enable && cfg_valid) begin
                tx_monitor_active <= 1'b0;
                phase1_extend     <= 5'd0;
                phase2_shorten    <= 5'd0;
                resync_used       <= 1'b0;
                tx_wire_index     <= 9'd0;

                if (ctrl[2]) begin
                    active            <= 1'b0;
                    synced            <= 1'b0;
                    waiting_hard_sync <= 1'b1;
                    brp_count         <= 8'd0;
                    tq_index          <= 6'd0;
                end else begin
                    active            <= 1'b1;
                    synced            <= 1'b1;
                    waiting_hard_sync <= 1'b0;
                    brp_count         <= 8'd0;
                    tq_index          <= 6'd0;
                end
            end

            // SOF hard sync.
            if (can_enable && cfg_valid && bus_fall &&
                (waiting_hard_sync || (rx_start && ctrl[2]))) begin
                active            <= 1'b1;
                synced            <= 1'b1;
                waiting_hard_sync <= 1'b0;
                brp_count         <= 8'd0;
                tq_index          <= 6'd0;
                phase1_extend     <= 5'd0;
                phase2_shorten    <= 5'd0;
                resync_used       <= 1'b0;
                hard_sync_seen    <= 1'b1;
                last_phase_error  <= 8'h00;
            end else if (active && can_enable && cfg_valid &&
                         !stream_stop && !tx_start) begin
                if (tq_expire) begin
                    brp_count <= 8'd0;

                    if (bit_boundary) begin
                        tq_index       <= 6'd0;
                        phase1_extend  <= 5'd0;
                        phase2_shorten <= 5'd0;

                        if (tx_monitor_active)
                            tx_wire_index <= tx_wire_index + 1'b1;
                    end else begin
                        tq_index <= tq_index + 1'b1;
                    end
                end else begin
                    brp_count <= brp_count + 1'b1;
                end

                if (positive_edge) begin
                    // If an adjustment itself creates a boundary, the boundary
                    // reset above owns the next-bit segment state.
                    if (!bit_boundary)
                        phase1_extend <= phase1_extend + positive_adjust;
                    resync_used      <= 1'b1;
                    resync_seen      <= 1'b1;
                    last_phase_error <= {2'b00, positive_error_mag};
                end else if (negative_edge) begin
                    if (!bit_boundary)
                        phase2_shorten <= phase2_shorten + negative_adjust;
                    resync_used       <= 1'b1;
                    resync_seen       <= 1'b1;
                    last_phase_error  <= (~{2'b00, negative_error_mag}) + 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire

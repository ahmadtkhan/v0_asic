`default_nettype none

// CAN 2.0 nominal bit-timing engine.
//
// V1.2 timing-closure architecture:
//   * CAN_BRP / CAN_SEG1 / CAN_SEG2 write staging registers.
//   * CAN_CTRL commits the staged configuration into runtime registers.
//   * The hot timing loop uses registered sample_target_q/end_target_q.
//   * sample_strobe and bit_boundary are only counter/target comparisons.
//   * Resynchronization updates the targets; it is not in the output compare
//     path.
//
// CAN timing model:
//   SYNC_SEG  = 1 TQ
//   sample    = end of PHASE_SEG1
//   bit end   = end of PHASE_SEG2
//   positive phase error -> lengthen PHASE_SEG1
//   negative phase error -> shorten PHASE_SEG2
//   correction magnitude <= SJW
//
// This block remains a timing layer, not a complete CAN controller.
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

    // ---------------------------------------------------------------------
    // Staged firmware-visible configuration.
    // ---------------------------------------------------------------------
    reg [7:0] brp_m1;
    reg [3:0] prop_m1;
    reg [3:0] phase1_m1;
    reg [3:0] phase2_m2;
    reg [3:0] sjw_m1;
    reg [3:0] ctrl;
    reg [7:0] arb_bits_m1;

    wire [4:0] staged_prop_seg;
    wire [4:0] staged_phase1_seg;
    wire [4:0] staged_phase2_seg;
    wire [4:0] staged_sjw;
    wire [5:0] staged_sample_index;
    wire [5:0] staged_end_index;
    wire [5:0] staged_total_tq;
    wire       staged_cfg_valid;

    assign staged_prop_seg   = {1'b0, prop_m1}   + 5'd1;
    assign staged_phase1_seg = {1'b0, phase1_m1} + 5'd1;
    assign staged_phase2_seg = {1'b0, phase2_m2} + 5'd2;
    assign staged_sjw        = {1'b0, sjw_m1}    + 5'd1;

    // tq_index is 0-based:
    //   0                  = SYNC_SEG
    //   sample_index       = last TQ before sample point
    //   end_index          = last TQ of the bit
    assign staged_sample_index =
        {1'b0, staged_prop_seg} + {1'b0, staged_phase1_seg};

    assign staged_end_index =
        {1'b0, staged_prop_seg}
        + {1'b0, staged_phase1_seg}
        + {1'b0, staged_phase2_seg};

    assign staged_total_tq = staged_end_index + 6'd1;

    assign staged_cfg_valid =
        (staged_prop_seg   >= 5'd1) && (staged_prop_seg   <= 5'd8) &&
        (staged_phase1_seg >= 5'd1) && (staged_phase1_seg <= 5'd8) &&
        (staged_phase2_seg >= 5'd2) && (staged_phase2_seg <= 5'd8) &&
        (staged_sjw        >= 5'd1) && (staged_sjw        <= 5'd4) &&
        (staged_sjw <= staged_phase1_seg) &&
        (staged_sjw <= staged_phase2_seg) &&
        (staged_total_tq >= 6'd8) &&
        (staged_total_tq <= 6'd25);

    // ---------------------------------------------------------------------
    // Committed runtime configuration.
    //
    // These are the only timing-geometry values used while a stream runs.
    // Static staging fields no longer feed the hot timing path.
    // ---------------------------------------------------------------------
    reg [7:0] brp_active_m1;
    reg [4:0] nominal_sample_target;
    reg [4:0] nominal_end_target;
    reg [4:0] phase2_seg_r;
    reg [4:0] sjw_r;
    reg       cfg_valid_r;

    assign can_enable = ctrl[0];
    assign cfg_valid  = cfg_valid_r;

    // Reserved upper control bits are intentionally ignored.
    wire _unused_ctrl_reserved = 1'b0 && &{1'b0, ctrl_data[7:4]};

    // ---------------------------------------------------------------------
    // Runtime state.
    // ---------------------------------------------------------------------
    reg       active;
    reg       synced;
    reg       waiting_hard_sync;
    reg       tx_monitor_active;

    reg [7:0] brp_count;
    reg [4:0] tq_index;

    // Absolute targets for the current bit.
    // These replace phase1_extend / phase2_shorten.
    reg [4:0] sample_target_q;
    reg [4:0] end_target_q;

    // CAN permits only one synchronization within one bit time.
    reg       resync_used;

    reg [8:0] tx_wire_index;

    reg resync_seen;
    reg hard_sync_seen;
    reg arbitration_lost;

    // ---------------------------------------------------------------------
    // Hot timing path.
    //
    // This is intentionally small:
    //   BRP compare -> TQ expiry
    //   6-bit target compare -> sample/boundary
    // ---------------------------------------------------------------------
    wire tq_expire;
    wire bit_boundary;

    assign tq_expire =
        active && (brp_count == brp_active_m1);

    assign sample_strobe =
        tq_expire && (tq_index == sample_target_q);

    assign bit_boundary =
        tq_expire && (tq_index == end_target_q);

    // tx_start is a synthetic first bit boundary. proto_stream already has
    // the first transmit byte available when it raises tx_start.
    assign bit_tick =
        (tx_start && can_enable && cfg_valid) || bit_boundary;

    // ---------------------------------------------------------------------
    // Resynchronization event classification.
    //
    // Resynchronization arithmetic only feeds the target registers.  It is
    // deliberately not part of sample_strobe / bit_boundary.
    //
    // Edges exactly on sample_target_q or end_target_q are treated as zero
    // phase error in this whole-TQ digital model and do not cause correction.
    // ---------------------------------------------------------------------
    wire resync_common;
    wire positive_edge;
    wire negative_edge;

    assign resync_common =
        can_enable &&
        cfg_valid &&
        ctrl[1] &&
        active &&
        synced &&
        !waiting_hard_sync &&
        !resync_used &&
        bus_fall &&
        sampled_bit &&
        !tx_start;

    // Positive phase error: edge is late relative to SYNC and before sample.
    // A transmitter already driving dominant ignores this positive edge.
    assign positive_edge =
        resync_common &&
        (tq_index != 5'd0) &&
        (tq_index < sample_target_q) &&
        !(tx_monitor_active && (tx_level == 1'b0));

    // Negative phase error: edge is after the sample point but before bit end.
    assign negative_edge =
        resync_common &&
        (tq_index > sample_target_q) &&
        (tq_index < end_target_q);

    reg [4:0] positive_error_mag;
    reg [4:0] negative_error_mag;
    reg [4:0] negative_future_room;
    reg [4:0] phase2_short_limit;
    reg [4:0] positive_adjust;
    reg [4:0] negative_adjust;

    always @(*) begin
        // Positive error is measured from the expected SYNC position.
        positive_error_mag = tq_index;

        // Negative error is the distance from the observed edge to the
        // currently scheduled bit end. No +1 is used: an edge one TQ before
        // bit end has a magnitude of exactly one TQ.
        if (end_target_q > tq_index)
            negative_error_mag = end_target_q - tq_index;
        else
            negative_error_mag = 5'd0;

        // If the edge is being processed on a TQ-expiry clock, tq_index will
        // advance on this same clock edge. Keep the new end target at or ahead
        // of that next index so the boundary cannot be moved behind the
        // counter and missed.
        negative_future_room = negative_error_mag;
        if (tq_expire && (negative_future_room != 5'd0))
            negative_future_room = negative_future_room - 5'd1;

        // Preserve at least one TQ of PHASE_SEG2 after shortening.
        if (phase2_seg_r > 5'd1)
            phase2_short_limit = phase2_seg_r - 5'd1;
        else
            phase2_short_limit = 5'd0;

        if (positive_error_mag > sjw_r)
            positive_adjust = sjw_r;
        else
            positive_adjust = positive_error_mag;

        if (negative_error_mag > sjw_r)
            negative_adjust = sjw_r;
        else
            negative_adjust = negative_error_mag;

        if (negative_adjust > phase2_short_limit)
            negative_adjust = phase2_short_limit;

        if (negative_adjust > negative_future_room)
            negative_adjust = negative_future_room;
    end

    // ---------------------------------------------------------------------
    // Arbitration monitor.
    // ---------------------------------------------------------------------
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

    // ---------------------------------------------------------------------
    // Sequential state.
    // ---------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            // Staged reset geometry:
            // clocks/TQ=1, PROP=1, PH1=4, PH2=2, SJW=1.
            brp_m1      <= 8'd0;
            prop_m1     <= 4'd0;
            phase1_m1   <= 4'd3;
            phase2_m2   <= 4'd0;
            sjw_m1      <= 4'd0;
            ctrl        <= 4'b0000;
            arb_bits_m1 <= 8'd0;

            // Matching committed reset geometry:
            // sample target = 1 + 4 = 5
            // end target    = 1 + 4 + 2 = 7
            brp_active_m1       <= 8'd0;
            nominal_sample_target <= 5'd5;
            nominal_end_target    <= 5'd7;
            phase2_seg_r          <= 5'd2;
            sjw_r                 <= 5'd1;
            cfg_valid_r           <= 1'b1;

            active            <= 1'b0;
            synced            <= 1'b0;
            waiting_hard_sync <= 1'b0;
            tx_monitor_active <= 1'b0;

            brp_count       <= 8'd0;
            tq_index        <= 5'd0;
            sample_target_q <= 5'd5;
            end_target_q    <= 5'd7;
            resync_used     <= 1'b0;
            tx_wire_index   <= 9'd0;

            sampled_bit      <= 1'b1;
            resync_seen      <= 1'b0;
            hard_sync_seen   <= 1'b0;
            arbitration_lost <= 1'b0;
            last_phase_error <= 8'h00;
        end else begin
            // -------------------------------------------------------------
            // Staging-register writes.
            // -------------------------------------------------------------
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

            // CAN_CTRL is the configuration commit point.
            if (ctrl_we) begin
                brp_active_m1        <= brp_m1;
                nominal_sample_target <= staged_sample_index[4:0];
                nominal_end_target    <= staged_end_index[4:0];
                phase2_seg_r          <= staged_phase2_seg;
                sjw_r                 <= staged_sjw;
                cfg_valid_r           <= staged_cfg_valid;
                ctrl                  <= ctrl_data[3:0];

                // Also prime current targets with the committed geometry.
                sample_target_q <= staged_sample_index[4:0];
                end_target_q    <= staged_end_index[4:0];

                if (!ctrl_data[0]) begin
                    active            <= 1'b0;
                    synced            <= 1'b0;
                    waiting_hard_sync <= 1'b0;
                    tx_monitor_active <= 1'b0;
                    brp_count         <= 8'd0;
                    tq_index          <= 5'd0;
                    resync_used       <= 1'b0;
                end
            end

            if (clear_status) begin
                resync_seen      <= 1'b0;
                hard_sync_seen   <= 1'b0;
                arbitration_lost <= 1'b0;
                last_phase_error <= 8'h00;
            end

            if (sample_strobe)
                sampled_bit <= bus_level;

            if (arbitration_lost_now) begin
                arbitration_lost  <= 1'b1;
                tx_monitor_active <= 1'b0;
            end

            // -------------------------------------------------------------
            // Timing-engine lifecycle.
            // Priority: stop > TX start > RX hard sync > RX arm > run.
            // -------------------------------------------------------------
            if (stream_stop) begin
                active            <= 1'b0;
                synced            <= 1'b0;
                waiting_hard_sync <= 1'b0;
                tx_monitor_active <= 1'b0;

                brp_count       <= 8'd0;
                tq_index        <= 5'd0;
                sample_target_q <= nominal_sample_target;
                end_target_q    <= nominal_end_target;
                resync_used     <= 1'b0;
            end else if (tx_start && can_enable && cfg_valid) begin
                active            <= 1'b1;
                synced            <= 1'b1;
                waiting_hard_sync <= 1'b0;
                tx_monitor_active <= 1'b1;

                brp_count       <= 8'd0;
                tq_index        <= 5'd0;
                sample_target_q <= nominal_sample_target;
                end_target_q    <= nominal_end_target;
                resync_used     <= 1'b0;
                tx_wire_index   <= 9'd0;
            end else if (
                can_enable &&
                cfg_valid &&
                bus_fall &&
                (waiting_hard_sync || (rx_start && ctrl[2]))
            ) begin
                // Hard synchronization: restart the bit time at SOF.
                active            <= 1'b1;
                synced            <= 1'b1;
                waiting_hard_sync <= 1'b0;
                tx_monitor_active <= 1'b0;

                brp_count       <= 8'd0;
                tq_index        <= 5'd0;
                sample_target_q <= nominal_sample_target;
                end_target_q    <= nominal_end_target;

                // Hard sync consumes the synchronization opportunity for this
                // bit. It is cleared at the next bit boundary.
                resync_used      <= 1'b1;
                tx_wire_index    <= 9'd0;
                hard_sync_seen   <= 1'b1;
                last_phase_error <= 8'h00;
            end else if (rx_start && can_enable && cfg_valid) begin
                tx_monitor_active <= 1'b0;
                tx_wire_index     <= 9'd0;

                brp_count       <= 8'd0;
                tq_index        <= 5'd0;
                sample_target_q <= nominal_sample_target;
                end_target_q    <= nominal_end_target;
                resync_used     <= 1'b0;

                if (ctrl[2]) begin
                    active            <= 1'b0;
                    synced            <= 1'b0;
                    waiting_hard_sync <= 1'b1;
                end else begin
                    active            <= 1'b1;
                    synced            <= 1'b1;
                    waiting_hard_sync <= 1'b0;
                end
            end else if (active && can_enable && cfg_valid) begin
                // ---------------------------------------------------------
                // Base TQ progression.
                // ---------------------------------------------------------
                if (tq_expire) begin
                    brp_count <= 8'd0;

                    if (bit_boundary) begin
                        tq_index        <= 5'd0;
                        sample_target_q <= nominal_sample_target;
                        end_target_q    <= nominal_end_target;

                        // Exactly one synchronization is permitted per bit.
                        resync_used <= 1'b0;

                        if (tx_monitor_active)
                            tx_wire_index <= tx_wire_index + 1'b1;
                    end else begin
                        tq_index <= tq_index + 5'd1;
                    end
                end else begin
                    brp_count <= brp_count + 1'b1;
                end

                // ---------------------------------------------------------
                // Resynchronization target updates.
                //
                // These register writes do not feed the current-cycle
                // sample_strobe or bit_boundary combinationally.
                // ---------------------------------------------------------
                if (positive_edge) begin
                    sample_target_q <=
                        sample_target_q + positive_adjust;
                    end_target_q <=
                        end_target_q + positive_adjust;

                    resync_used      <= 1'b1;
                    resync_seen      <= 1'b1;
                    last_phase_error <= {3'b000, positive_error_mag};
                end else if (negative_edge) begin
                    if (negative_adjust != 5'd0)
                        end_target_q <=
                            end_target_q - negative_adjust;

                    resync_used <= 1'b1;
                    resync_seen <= 1'b1;

                    // Two's-complement signed whole-TQ phase error.
                    last_phase_error <=
                        (~{3'b000, negative_error_mag}) + 8'd1;
                end
            end
        end
    end

endmodule

`default_nettype wire

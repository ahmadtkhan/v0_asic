`default_nettype none

// Configurable 1..32-bit CRC/LFSR engine.
//
// Polynomial convention:
//   lsb_first=0: POLY is the normal, top-bit-omitted polynomial and the LFSR
//                shifts left (MSB-first input).
//   lsb_first=1: POLY is the reflected polynomial and the LFSR shifts right
//                (LSB-first input).
//
// Byte writes are little-endian: byte_sel 0 targets bits [7:0], byte_sel 3
// targets bits [31:24].  Only the configured width participates in updates and
// in the final result.  result = optional bit-reflection(state) XOR xor_out.
module proto_crc_lfsr (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        ctrl_we,
    input  wire [7:0]  ctrl_data,

    input  wire        poly_we,
    input  wire        init_we,
    input  wire        xor_we,
    input  wire [1:0]  byte_sel,
    input  wire [7:0]  cfg_data,

    input  wire        state_reset,

    input  wire        feed_byte_start,
    input  wire [7:0]  feed_byte_data,

    input  wire        feed_bit_valid,
    input  wire        feed_bit,

    output reg         busy,
    output reg         done,
    output wire [31:0] state_value,
    output wire [31:0] result_value
);

    reg [4:0]  width_m1;
    reg        lsb_first;
    reg        reflect_out;
    reg [31:0] polynomial;
    reg [31:0] init_value;
    reg [31:0] xor_out;
    reg [31:0] lfsr_state;

    reg [7:0] byte_shift;
    reg [3:0] bits_left;

    wire [31:0] active_mask;
    wire [31:0] reflected_state;

    function [31:0] width_mask;
        input [4:0] wm1;
        begin
            if (wm1 == 5'd31)
                width_mask = 32'hFFFF_FFFF;
            else
                width_mask = (32'h0000_0001 << (wm1 + 1'b1)) - 1'b1;
        end
    endfunction

    function [31:0] crc_step;
        input [31:0] cur;
        input        data_bit;
        input [31:0] poly;
        input [4:0]  wm1;
        input        shift_lsb;
        reg          feedback;
        reg [31:0]   nxt;
        reg [31:0]   mask;
        begin
            mask = width_mask(wm1);
            if (shift_lsb) begin
                feedback = cur[0] ^ data_bit;
                nxt      = cur >> 1;
                if (feedback)
                    nxt = nxt ^ poly;
            end else begin
                feedback = cur[wm1] ^ data_bit;
                nxt      = (cur << 1) & mask;
                if (feedback)
                    nxt = nxt ^ poly;
            end
            crc_step = nxt & mask;
        end
    endfunction

    function [31:0] reflect_active;
        input [31:0] value;
        input [4:0]  wm1;
        integer i;
        begin
            reflect_active = 32'h0000_0000;
            for (i = 0; i < 32; i = i + 1) begin
                if (i <= wm1)
                    reflect_active[i] = value[wm1 - i];
            end
        end
    endfunction

    assign active_mask     = width_mask(width_m1);
    assign reflected_state = reflect_active(lfsr_state, width_m1);
    assign state_value     = lfsr_state & active_mask;
    assign result_value    = ((reflect_out ? reflected_state : lfsr_state) ^ xor_out)
                           & active_mask;

    always @(posedge clk) begin
        if (!rst_n) begin
            width_m1    <= 5'd7;           // width = 8
            lsb_first   <= 1'b0;
            reflect_out <= 1'b0;
            polynomial  <= 32'h0000_0007;  // CRC-8 style default
            init_value  <= 32'h0000_0000;
            xor_out     <= 32'h0000_0000;
            lfsr_state  <= 32'h0000_0000;
            byte_shift  <= 8'h00;
            bits_left   <= 4'd0;
            busy        <= 1'b0;
            done        <= 1'b0;
        end else begin
            done <= 1'b0;

            // Configuration is only issued by firmware while the engine is idle.
            if (!busy && ctrl_we) begin
                width_m1    <= ctrl_data[4:0];
                lsb_first   <= ctrl_data[6];
                reflect_out <= ctrl_data[7];
                lfsr_state  <= lfsr_state & width_mask(ctrl_data[4:0]);
            end

            if (!busy && poly_we) begin
                case (byte_sel)
                    2'd0: polynomial[7:0]   <= cfg_data;
                    2'd1: polynomial[15:8]  <= cfg_data;
                    2'd2: polynomial[23:16] <= cfg_data;
                    2'd3: polynomial[31:24] <= cfg_data;
                endcase
            end

            if (!busy && init_we) begin
                case (byte_sel)
                    2'd0: init_value[7:0]   <= cfg_data;
                    2'd1: init_value[15:8]  <= cfg_data;
                    2'd2: init_value[23:16] <= cfg_data;
                    2'd3: init_value[31:24] <= cfg_data;
                endcase
            end

            if (!busy && xor_we) begin
                case (byte_sel)
                    2'd0: xor_out[7:0]   <= cfg_data;
                    2'd1: xor_out[15:8]  <= cfg_data;
                    2'd2: xor_out[23:16] <= cfg_data;
                    2'd3: xor_out[31:24] <= cfg_data;
                endcase
            end

            if (!busy && state_reset) begin
                lfsr_state <= init_value & active_mask;
            end else if (!busy && feed_byte_start) begin
                byte_shift <= feed_byte_data;
                bits_left  <= 4'd8;
                busy       <= 1'b1;
            end else if (busy) begin
                if (lsb_first) begin
                    lfsr_state <= crc_step(lfsr_state, byte_shift[0], polynomial,
                                           width_m1, 1'b1);
                    byte_shift <= {1'b0, byte_shift[7:1]};
                end else begin
                    lfsr_state <= crc_step(lfsr_state, byte_shift[7], polynomial,
                                           width_m1, 1'b0);
                    byte_shift <= {byte_shift[6:0], 1'b0};
                end

                if (bits_left == 4'd1) begin
                    bits_left <= 4'd0;
                    busy      <= 1'b0;
                    done      <= 1'b1;
                end else begin
                    bits_left <= bits_left - 1'b1;
                end
            end else if (feed_bit_valid) begin
                lfsr_state <= crc_step(lfsr_state, feed_bit, polynomial,
                                       width_m1, lsb_first);
            end
        end
    end

endmodule

`default_nettype wire

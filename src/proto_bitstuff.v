`default_nettype none

// Generic serial bit stuffer / destuffer.
//
// mode_any = 1:
//   CAN-style run-length stuffing.  After THRESHOLD identical wire bits, the
//   encoder inserts the complement.  The decoder expects/drops that complement.
//
// mode_any = 0:
//   USB-style ones stuffing.  After THRESHOLD consecutive ones, the encoder
//   inserts zero.  The decoder expects/drops zero.
//
// The block advances only when step=1.  In encode mode consume_input=0 while
// an automatically inserted bit is being emitted, so the caller must keep the
// payload bit stable until consume_input=1.
module proto_bitstuff (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       clear,

    input  wire       enable,
    input  wire       destuff,
    input  wire       mode_any,
    input  wire [2:0] threshold,

    input  wire       step,
    input  wire       in_bit,

    output reg        out_valid,
    output reg        out_bit,
    output reg        consume_input,
    output reg        inserted,
    output reg        dropped,
    output reg        error_now,
    output reg        error
);

    reg       have_last;
    reg       last_bit;
    reg [2:0] run_count;
    reg       insert_pending;
    reg       expect_stuff;

    reg       expected_stuff_bit;

    always @(*) begin
        out_valid          = 1'b1;
        out_bit            = in_bit;
        consume_input      = 1'b1;
        inserted           = 1'b0;
        dropped            = 1'b0;
        error_now          = 1'b0;
        expected_stuff_bit = mode_any ? ~last_bit : 1'b0;

        if (enable) begin
            if (!destuff) begin
                if (insert_pending) begin
                    out_valid     = 1'b1;
                    out_bit       = expected_stuff_bit;
                    consume_input = 1'b0;
                    inserted      = 1'b1;
                end
            end else begin
                consume_input = 1'b1;
                if (expect_stuff) begin
                    out_valid = 1'b0;
                    dropped   = 1'b1;
                    if (in_bit != expected_stuff_bit)
                        error_now = 1'b1;
                end
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            have_last      <= 1'b0;
            last_bit       <= 1'b0;
            run_count      <= 3'd0;
            insert_pending <= 1'b0;
            expect_stuff   <= 1'b0;
            error          <= 1'b0;
        end else if (clear) begin
            have_last      <= 1'b0;
            last_bit       <= 1'b0;
            run_count      <= 3'd0;
            insert_pending <= 1'b0;
            expect_stuff   <= 1'b0;
            error          <= 1'b0;
        end else if (step) begin
            if (!enable) begin
                have_last      <= 1'b0;
                run_count      <= 3'd0;
                insert_pending <= 1'b0;
                expect_stuff   <= 1'b0;
            end else if (!destuff) begin
                if (insert_pending) begin
                    insert_pending <= 1'b0;
                    have_last      <= 1'b1;
                    if (mode_any) begin
                        last_bit  <= ~last_bit;
                        run_count <= 3'd1;
                    end else begin
                        last_bit  <= 1'b0;
                        run_count <= 3'd0;
                    end
                end else if (mode_any) begin
                    if (!have_last || (in_bit != last_bit)) begin
                        have_last <= 1'b1;
                        last_bit  <= in_bit;
                        run_count <= 3'd1;
                        if (threshold <= 3'd1)
                            insert_pending <= 1'b1;
                    end else begin
                        run_count <= run_count + 1'b1;
                        if ((run_count + 1'b1) >= threshold)
                            insert_pending <= 1'b1;
                    end
                end else begin
                    // Ones-only stuffing.
                    if (in_bit) begin
                        run_count <= run_count + 1'b1;
                        if ((run_count + 1'b1) >= threshold)
                            insert_pending <= 1'b1;
                    end else begin
                        run_count <= 3'd0;
                    end
                    have_last <= 1'b1;
                    last_bit  <= in_bit;
                end
            end else begin
                // Destuff mode consumes every wire bit but suppresses the
                // automatically inserted one from the payload output.
                if (expect_stuff) begin
                    if (error_now)
                        error <= 1'b1;
                    expect_stuff <= 1'b0;
                    have_last    <= 1'b1;
                    if (mode_any) begin
                        last_bit  <= in_bit;
                        run_count <= 3'd1;
                    end else begin
                        last_bit  <= in_bit;
                        run_count <= 3'd0;
                    end
                end else if (mode_any) begin
                    if (!have_last || (in_bit != last_bit)) begin
                        have_last <= 1'b1;
                        last_bit  <= in_bit;
                        run_count <= 3'd1;
                        if (threshold <= 3'd1)
                            expect_stuff <= 1'b1;
                    end else begin
                        run_count <= run_count + 1'b1;
                        if ((run_count + 1'b1) >= threshold)
                            expect_stuff <= 1'b1;
                    end
                end else begin
                    if (in_bit) begin
                        run_count <= run_count + 1'b1;
                        if ((run_count + 1'b1) >= threshold)
                            expect_stuff <= 1'b1;
                    end else begin
                        run_count <= 3'd0;
                    end
                    have_last <= 1'b1;
                    last_bit  <= in_bit;
                end
            end
        end
    end

endmodule

`default_nettype wire

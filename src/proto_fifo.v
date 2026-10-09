`default_nettype none
`include "proto_params.vh"

// Small synchronous register/DFF FIFO.  This is intentionally not the bulk
// SRAM: it is the elastic buffer between the protocol shifter and host/data
// path, where independent push/pop behavior is more useful than density.
module proto_fifo #(
    parameter integer WIDTH = `PROTO_DATA_W,
    parameter integer DEPTH = `PROTO_FIFO_DEPTH
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             push,
    input  wire [WIDTH-1:0] push_data,
    input  wire             pop,
    output wire [WIDTH-1:0] pop_data,
    output wire             full,
    output wire             empty
);

    function integer clog2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            clog2 = 0;
            while (v > 0) begin
                v = v >> 1;
                clog2 = clog2 + 1;
            end
            if (clog2 == 0)
                clog2 = 1;
        end
    endfunction

    localparam integer AW      = clog2(DEPTH);
    localparam integer COUNT_W = clog2(DEPTH + 1);
    localparam [AW-1:0] LAST_ADDR = DEPTH - 1;
    localparam [COUNT_W-1:0] DEPTH_COUNT = DEPTH;
    localparam [COUNT_W-1:0] ZERO_COUNT  = {COUNT_W{1'b0}};

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [AW-1:0] rd_ptr;
    reg [AW-1:0] wr_ptr;
    reg [COUNT_W-1:0] count;

    wire do_pop;
    wire do_push;

    assign empty = (count == ZERO_COUNT);
    assign full  = (count == DEPTH_COUNT);
    assign pop_data = mem[rd_ptr];

    // A simultaneous pop permits a write even when the FIFO starts full.
    assign do_pop  = pop  && !empty;
    assign do_push = push && (!full || do_pop);

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr <= {AW{1'b0}};
            wr_ptr <= {AW{1'b0}};
            count  <= ZERO_COUNT;
        end else begin
            if (do_push) begin
                mem[wr_ptr] <= push_data;
                if (wr_ptr == LAST_ADDR)
                    wr_ptr <= {AW{1'b0}};
                else
                    wr_ptr <= wr_ptr + 1'b1;
            end

            if (do_pop) begin
                if (rd_ptr == LAST_ADDR)
                    rd_ptr <= {AW{1'b0}};
                else
                    rd_ptr <= rd_ptr + 1'b1;
            end

            case ({do_push, do_pop})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule

`default_nettype wire

`default_nettype none

// Synchronous first-word-fall-through FIFO.
// DEPTH may be any value >= 2.
module proto_fifo #(
    parameter integer WIDTH = 8,
    parameter integer DEPTH = 4,
    localparam integer AW = (DEPTH <= 2) ? 1 : $clog2(DEPTH),
    localparam integer COUNT_W = $clog2(DEPTH + 1)
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

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [AW-1:0] rd_ptr;
    reg [AW-1:0] wr_ptr;
    reg [COUNT_W-1:0] count;

    // Explicitly size integer parameters before assigning to vectors.
    localparam [AW-1:0] LAST_ADDR =
        DEPTH[AW-1:0] - {{(AW-1){1'b0}}, 1'b1};
    localparam [COUNT_W-1:0] DEPTH_COUNT =
        DEPTH[COUNT_W-1:0];
    localparam [COUNT_W-1:0] ZERO_COUNT =
        {COUNT_W{1'b0}};

    assign empty = (count == ZERO_COUNT);
    assign full  = (count == DEPTH_COUNT);

    assign pop_data = mem[rd_ptr];

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr <= {AW{1'b0}};
            wr_ptr <= {AW{1'b0}};
            count  <= ZERO_COUNT;
        end else begin
            if (push && !full) begin
                mem[wr_ptr] <= push_data;
                if (wr_ptr == LAST_ADDR)
                    wr_ptr <= {AW{1'b0}};
                else
                    wr_ptr <= wr_ptr + {{(AW-1){1'b0}}, 1'b1};
            end

            if (pop && !empty) begin
                if (rd_ptr == LAST_ADDR)
                    rd_ptr <= {AW{1'b0}};
                else
                    rd_ptr <= rd_ptr + {{(AW-1){1'b0}}, 1'b1};
            end

            case ({push && !full, pop && !empty})
                2'b10: count <= count + {{(COUNT_W-1){1'b0}}, 1'b1};
                2'b01: count <= count - {{(COUNT_W-1){1'b0}}, 1'b1};
                default: count <= count;
            endcase
        end
    end

endmodule

`default_nettype wire

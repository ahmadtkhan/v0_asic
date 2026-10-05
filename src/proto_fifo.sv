`default_nettype none

// Minimal synchronous FIFO.
module proto_fifo #(
    parameter integer WIDTH = 8,
    parameter integer DEPTH = 4,
    localparam integer AW = $clog2(DEPTH)
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
    reg [AW:0]   count;

    // Explicitly size DEPTH to match count. This avoids comparing the unsigned
    // count vector against the signed integer parameter DEPTH.
    localparam [AW:0] DEPTH_COUNT = DEPTH;
    localparam [AW:0] ZERO_COUNT  = {(AW+1){1'b0}};

    assign empty = (count == ZERO_COUNT);
    assign full  = (count == DEPTH_COUNT);

    assign pop_data = mem[rd_ptr];

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr <= {AW{1'b0}};
            wr_ptr <= {AW{1'b0}};
            count  <= {(AW+1){1'b0}};
        end else begin
            if (push && !full) begin
                mem[wr_ptr] <= push_data;
                wr_ptr      <= wr_ptr + {{(AW-1){1'b0}}, 1'b1};
            end

            if (pop && !empty) begin
                rd_ptr <= rd_ptr + {{(AW-1){1'b0}}, 1'b1};
            end

            case ({push && !full, pop && !empty})
                2'b10: count <= count + {{AW{1'b0}}, 1'b1};
                2'b01: count <= count - {{AW{1'b0}}, 1'b1};
                default: count <= count;
            endcase
        end
    end

endmodule

`default_nettype wire

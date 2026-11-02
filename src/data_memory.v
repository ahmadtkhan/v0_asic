`default_nettype none
`include "proto_params.vh"

// V0 byte-addressed data-memory abstraction.
//
// Default path:
//   behavioral synchronous 1RW 256x8 memory.
//
// Hard-macro path:
//   define PROTO_USE_IHP_DMEM_SRAM and compile/include
//   RM_IHPSG13_1P_256x8_c3_bm_bist.
//
// The core issues one-cycle read or write requests.  Reads return rvalid one
// cycle after the request.  Stores are accepted on the request clock edge and
// require no extra completion state.
module data_memory #(
    parameter integer AW    = `PROTO_DMEM_AW,
    parameter integer DEPTH = `PROTO_DMEM_DEPTH
) (
    input  wire                    clk,
    input  wire                    rst_n,

    input  wire                    req,
    input  wire                    we,
    input  wire [AW-1:0]           addr,
    input  wire [`PROTO_DATA_W-1:0] wdata,

    output wire [`PROTO_DATA_W-1:0] rdata,
    output reg                     rvalid
);

    wire read_accept;
    assign read_accept = req & ~we;

    // Registered read-valid matches the synchronous SRAM contract.
    always @(posedge clk) begin
        if (!rst_n)
            rvalid <= 1'b0;
        else
            rvalid <= read_accept;
    end

`ifdef PROTO_USE_IHP_DMEM_SRAM

    // V0 hard macro is fixed at 256x8.  Keep AW=8, DEPTH=256 and
    // PROTO_DATA_W=8 when this branch is enabled.

    wire [7:0] sram_dout;

    RM_IHPSG13_1P_256x8_c3_bm_bist u_dmem_sram (
        .A_CLK       (clk),
        .A_MEN       (req),
        .A_WEN       (req & we),
        .A_REN       (read_accept),
        .A_ADDR      (addr[7:0]),
        .A_DIN       (wdata),
        .A_DLY       (1'b1),
        .A_DOUT      (sram_dout),
        .A_BM        (8'hFF),
        .A_BIST_CLK  (1'b0),
        .A_BIST_EN   (1'b0),
        .A_BIST_MEN  (1'b0),
        .A_BIST_WEN  (1'b0),
        .A_BIST_REN  (1'b0),
        .A_BIST_ADDR (8'h00),
        .A_BIST_DIN  (8'h00),
        .A_BIST_BM   (8'h00)
    );

    assign rdata = sram_dout;

`else

    reg [`PROTO_DATA_W-1:0] mem [0:DEPTH-1];
    reg [`PROTO_DATA_W-1:0] rdata_reg;

    assign rdata = rdata_reg;

    always @(posedge clk) begin
        if (!rst_n) begin
            rdata_reg <= {`PROTO_DATA_W{1'b0}};
        end else if (req) begin
            if (we)
                mem[addr] <= wdata;
            else
                rdata_reg <= mem[addr];
        end
    end

`endif

endmodule

`default_nettype wire

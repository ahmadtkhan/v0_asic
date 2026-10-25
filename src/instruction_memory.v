`default_nettype none
`include "proto_params.vh"

// 256x16 instruction-memory abstraction.
//
// Physical TinyTapeout/LibreLane builds define PROTO_USE_IHP_SRAM and bind
// u_imem_sram to RM_IHPSG13_1P_256x16_c2_bm_bist through src/config.json.
// RTL simulation may either use the behavioral fallback below or compile the
// official IHP functional SRAM model with PROTO_USE_IHP_SRAM + FUNCTIONAL.
module instruction_memory #(
    parameter integer AW    = `PROTO_IMEM_AW,
    parameter integer DEPTH = `PROTO_IMEM_DEPTH
) (
    input  wire                      clk,
    input  wire                      rst_n,

    input  wire                      prog_we,
    input  wire [AW-1:0]             prog_addr,
    input  wire [`PROTO_INSTR_W-1:0] prog_wdata,

    input  wire                      core_req,
    input  wire [AW-1:0]             core_addr,
    output wire [`PROTO_INSTR_W-1:0] core_rdata,
    output reg                       core_rvalid
);

    wire core_read_accept;
    assign core_read_accept = core_req & ~prog_we;

    // Both implementations below are synchronous 1RW memories.  The core's
    // FETCH_REQ -> FETCH_WAIT sequence expects this one-cycle response pulse.
    always @(posedge clk) begin
        if (!rst_n)
            core_rvalid <= 1'b0;
        else
            core_rvalid <= core_read_accept;
    end

`ifdef PROTO_USE_IHP_SRAM

    // V0 hard-SRAM mode is intentionally exact: 256 words x 16 bits.
`ifndef SYNTHESIS
    initial begin
        if (AW != 8 || DEPTH != 256 || `PROTO_INSTR_W != 16) begin
            $error("IHP IMEM macro requires AW=8, DEPTH=256, INSTR_W=16");
        end
    end
`endif

    wire [15:0] sram_dout;
    wire [7:0]  mem_addr;

    // Programming has priority over instruction fetch.  protocol_top already
    // exposes prog_ready only while the core is idle, so normal operation does
    // not request a write and fetch in the same cycle.
    assign mem_addr   = prog_we ? prog_addr[7:0] : core_addr[7:0];
    assign core_rdata = sram_dout;

    RM_IHPSG13_1P_256x16_c2_bm_bist u_imem_sram (
        .A_CLK       (clk),
        .A_MEN       (prog_we | core_read_accept),
        .A_WEN       (prog_we),
        .A_REN       (core_read_accept),
        .A_ADDR      (mem_addr),
        .A_DIN       (prog_wdata),
        .A_DLY       (1'b1),
        .A_DOUT      (sram_dout),
        .A_BM        (16'hFFFF),

        .A_BIST_CLK  (1'b0),
        .A_BIST_EN   (1'b0),
        .A_BIST_MEN  (1'b0),
        .A_BIST_WEN  (1'b0),
        .A_BIST_REN  (1'b0),
        .A_BIST_ADDR (8'h00),
        .A_BIST_DIN  (16'h0000),
        .A_BIST_BM   (16'h0000)
    );

`else

    reg [`PROTO_INSTR_W-1:0] mem [0:DEPTH-1];
    reg [`PROTO_INSTR_W-1:0] core_rdata_reg;

    assign core_rdata = core_rdata_reg;

    always @(posedge clk) begin
        if (!rst_n) begin
            core_rdata_reg <= {`PROTO_INSTR_W{1'b0}};
        end else begin
            if (prog_we)
                mem[prog_addr] <= prog_wdata;
            else if (core_req)
                core_rdata_reg <= mem[core_addr];
        end
    end

`endif

endmodule

`default_nettype wire

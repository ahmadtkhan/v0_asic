`default_nettype none
`include "proto_params.vh"

// Instruction-memory abstraction.
//
// Default simulation/synthesis path: behavioral synchronous 1RW memory.
// For the IHP hard macro, define PROTO_USE_IHP_SRAM and include the selected
// SRAM Verilog model in the build.  The hard-macro branch below is specifically
// for RM_IHPSG13_1P_256x16_c2_bm_bist and therefore requires the V0 256x16
// architecture values from proto_params.vh.
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

    // rvalid is deliberately registered.  This matches the core's conservative
    // FETCH_REQ -> FETCH_WAIT sequencing and the synchronous SRAM contract.
    always @(posedge clk) begin
        if (!rst_n)
            core_rvalid <= 1'b0;
        else
            core_rvalid <= core_read_accept;
    end

`ifdef PROTO_USE_IHP_SRAM

    wire [`PROTO_INSTR_W-1:0] sram_dout;
    wire [AW-1:0] mem_addr;

    assign mem_addr   = prog_we ? prog_addr : core_addr;
    assign core_rdata = sram_dout;

    // Current IHP SRAM functional interface follows the A_* convention used by
    // Tiny Tapeout's IHP SRAM example.  BIST is disabled for normal operation.
    RM_IHPSG13_1P_256x16_c2_bm_bist u_imem_sram (
        .A_CLK       (clk),
        .A_MEN       (prog_we | core_req),
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

`default_nettype none

module instruction_memory #(
    parameter integer AW    = 8,
    parameter integer DEPTH = (1 << AW)
) (
    input  wire          clk,
    input  wire          rst_n,

    // Host/program-loader write port. The top-level only accepts writes while
    // the protocol core is halted, so this and core_req never need arbitration
    // during normal operation.
    input  wire          prog_we,
    input  wire [AW-1:0] prog_addr,
    input  wire [15:0]   prog_wdata,

    // Core synchronous read port.
    input  wire          core_req,
    input  wire [AW-1:0] core_addr,
    output reg  [15:0]   core_rdata,
    output reg           core_rvalid
);

`ifdef PROTO_USE_IHP_SRAM
    // Replace this section only after checking the exact Verilog model for the
    // selected IHP macro in the CMOS5L flow. Signal names, BIST pins, masks,
    // and power pins must match that model. The logical behavior expected by
    // the core is a single-port synchronous memory with one-cycle read latency.
    //
    // Suggested physical V0 macro: RM_IHPSG13_1P_256x16_c2_bm_bist
    //
    // Intentionally left as an integration error rather than guessing a macro
    // pinout that may differ between PDK/Tiny-Tapeout releases.
    initial begin
        $error("PROTO_USE_IHP_SRAM requires the project-specific IHP SRAM wrapper implementation");
    end
`else
    reg [15:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (!rst_n) begin
            core_rdata  <= 16'h0000;
            core_rvalid <= 1'b0;
        end else begin
            core_rvalid <= 1'b0;

            if (prog_we) begin
                mem[prog_addr] <= prog_wdata;
            end else if (core_req) begin
                core_rdata  <= mem[core_addr];
                core_rvalid <= 1'b1;
            end
        end
    end
`endif

endmodule

`default_nettype wire

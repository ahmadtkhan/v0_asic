`default_nettype none

// Synthesis blackbox for the IHP 256x8 SRAM macro.
// Do not compile this together with the official functional SRAM model.
(* blackbox *)
module RM_IHPSG13_1P_256x8_c3_bm_bist (
    input  wire       A_CLK,
    input  wire       A_MEN,
    input  wire       A_WEN,
    input  wire       A_REN,
    input  wire [7:0] A_ADDR,
    input  wire [7:0] A_DIN,
    input  wire       A_DLY,
    output wire [7:0] A_DOUT,
    input  wire [7:0] A_BM,
    input  wire       A_BIST_CLK,
    input  wire       A_BIST_EN,
    input  wire       A_BIST_MEN,
    input  wire       A_BIST_WEN,
    input  wire       A_BIST_REN,
    input  wire [7:0] A_BIST_ADDR,
    input  wire [7:0] A_BIST_DIN,
    input  wire [7:0] A_BIST_BM
);
endmodule

`default_nettype wire

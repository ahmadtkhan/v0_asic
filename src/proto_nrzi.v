`default_nettype none

// Stateful NRZI codec using the USB convention:
//   data 0 -> transition
//   data 1 -> no transition
//
// In encode mode in_bit is data and out_bit is line level.
// In decode mode in_bit is line level and out_bit is decoded data.
module proto_nrzi (
    input  wire clk,
    input  wire rst_n,
    input  wire clear,
    input  wire decode,
    input  wire initial_level,
    input  wire step,
    input  wire in_bit,
    output wire out_bit,
    output wire level_state
);

    reg level;

    assign out_bit     = decode ? (in_bit == level)
                                : (in_bit ? level : ~level);
    assign level_state = level;

    always @(posedge clk) begin
        if (!rst_n)
            level <= 1'b1;
        else if (clear)
            level <= initial_level;
        else if (step) begin
            if (decode)
                level <= in_bit;
            else
                level <= out_bit;
        end
    end

endmodule

`default_nettype wire

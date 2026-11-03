`default_nettype none

// Stateless Manchester pair encoder/decoder.
//
// invert=0 canonical mapping:
//   data 0 -> 0,1
//   data 1 -> 1,0
//
// invert=1 swaps both line levels.  Decoder flags 00 and 11 as invalid pairs.
module proto_manchester (
    input  wire invert,

    input  wire tx_data_bit,
    output wire tx_first_half,
    output wire tx_second_half,

    input  wire rx_first_half,
    input  wire rx_second_half,
    output wire rx_data_bit,
    output wire rx_error
);

    assign tx_first_half  = invert ? ~tx_data_bit : tx_data_bit;
    assign tx_second_half = ~tx_first_half;

    assign rx_error    = (rx_first_half == rx_second_half);
    assign rx_data_bit = invert ? rx_second_half : rx_first_half;

endmodule

`default_nettype wire

//multi-purpose shifting module which supports OSR and ISR
module proto_shift #(
    parameter integer WIDTH = 8
) (
    input  wire                 clk,
    input  wire                 rst_n,

    // OSR
    input  wire                 osr_load,
    input  wire [WIDTH-1:0]     osr_load_data,
    input  wire                 osr_shift,
    input  wire                 osr_shift_right,

    // ISR
    input  wire                 isr_clear,
    input  wire                 isr_shift,
    input  wire                 isr_shift_right,
    input  wire                 serial_in,

    output reg                 serial_out,
    output reg [WIDTH-1:0]     osr,
    output reg [WIDTH-1:0]     isr
);

    always_comb begin
        if (osr_shift_right)
            serial_out = osr[0];
        else
            serial_out = osr[WIDTH-1];
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            osr <= '0;
            isr <= '0;
        end else begin

            if (osr_load) begin
                osr <= osr_load_data;
            end else if (osr_shift) begin
                if (osr_shift_right)
                    osr <= {1'b0, osr[WIDTH-1:1]};
                else
                    osr <= {osr[WIDTH-2:0], 1'b0};
            end

            if (isr_clear) begin
                isr <= '0;
            end else if (isr_shift) begin
                if (isr_shift_right)
                    isr <= {serial_in, isr[WIDTH-1:1]};
                else
                    isr <= {isr[WIDTH-2:0], serial_in};
            end

        end
    end

endmodule

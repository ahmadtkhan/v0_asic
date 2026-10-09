`default_nettype none
`include "proto_params.vh"
`include "proto_isa.vh"

// V0 protocol execution core.  It intentionally uses a conservative
// request/wait/execute fetch sequence so the first implementation has simple,
// deterministic synchronous-SRAM behavior.  A later prefetch optimization can
// remove fetch bubbles without changing this ISA.
module protocol_core #(
    parameter integer IMEM_AW = `PROTO_IMEM_AW
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         start,

    output wire                         busy,
    output reg                          fault,

    output wire                         imem_req,
    output wire [IMEM_AW-1:0]           imem_addr,
    input  wire [`PROTO_INSTR_W-1:0]    imem_rdata,
    input  wire                         imem_rvalid,

    input  wire                         tick,

    input  wire [2:0]                   cfg_out_pin,
    input  wire [2:0]                   cfg_in_pin,
    input  wire [2:0]                   cfg_side_pin,
    input  wire                         cfg_out_shift_right,
    input  wire                         cfg_in_shift_right,
    input  wire                         cfg_side_enable,
    input  wire                         cfg_side_target_oe,
    input  wire [2:0]                   cfg_jmp_pin,

    output reg                          cfg_we,
    output reg  [`PROTO_CFG_AW-1:0]     cfg_addr,
    output reg  [`PROTO_DATA_W-1:0]     cfg_wdata,

    input  wire [`PROTO_GPIO_W-1:0]     gpio_in,
    input  wire [`PROTO_GPIO_W-1:0]     gpio_out,
    input  wire [`PROTO_GPIO_W-1:0]     gpio_oe,
    input  wire [`PROTO_GPIO_W-1:0]     gpio_rise,
    input  wire [`PROTO_GPIO_W-1:0]     gpio_fall,
    output reg                          gpio_out_we,
    output reg  [`PROTO_GPIO_W-1:0]     gpio_out_wdata,
    output reg                          gpio_oe_we,
    output reg  [`PROTO_GPIO_W-1:0]     gpio_oe_wdata,

    input  wire                         shift_serial_out,
    input  wire [`PROTO_SHIFT_W-1:0]    shift_isr,
    output reg                          osr_load,
    output reg  [`PROTO_SHIFT_W-1:0]    osr_load_data,
    output reg                          osr_shift,
    output reg                          isr_clear,
    output reg                          isr_shift,
    output reg                          shift_serial_in,

    input  wire                         tx_empty,
    input  wire [`PROTO_DATA_W-1:0]     tx_pop_data,
    output reg                          tx_pop,

    input  wire                         rx_full,
    output reg                          rx_push,
    output reg  [`PROTO_DATA_W-1:0]     rx_push_data
);

    localparam [2:0] ST_IDLE       = 3'd0;
    localparam [2:0] ST_FETCH_REQ  = 3'd1;
    localparam [2:0] ST_FETCH_WAIT = 3'd2;
    localparam [2:0] ST_EXEC       = 3'd3;
    localparam [2:0] ST_WAIT_TICKS = 3'd4;
    localparam [2:0] ST_WAIT_PIN   = 3'd5;
    localparam [2:0] ST_POST_DELAY = 3'd6;

    reg [2:0] state;
    reg [IMEM_AW-1:0] pc;
    reg [`PROTO_INSTR_W-1:0] ir;

    reg [`PROTO_DATA_W-1:0] x_reg;
    reg [`PROTO_DATA_W-1:0] y_reg;
    reg [`PROTO_DATA_W-1:0] loop_count;
    reg                     zero_flag;

    // WAIT uses the full 11-bit payload.  Side-set/post-delay uses three bits.
    reg [10:0] wait_remaining;
    reg [2:0]  delay_remaining;

    wire [4:0]  opcode;
    wire [10:0] payload11;
    wire [2:0]  aux3;
    wire [7:0]  arg8;
    wire [2:0]  cond;
    wire [7:0]  branch_addr;
    wire [2:0]  waitpin_pin;
    wire [2:0]  reg_dst;
    wire [2:0]  reg_src;
    wire [7:0]  imm8;
    wire [2:0]  dec_cfg_addr;
    wire [7:0]  dec_cfg_data;
    wire        opcode_known;

    reg                     side_capable;
    reg                     side_value;
    reg [2:0]               post_delay;
    reg                     jcc_true;
    reg                     waitpin_true;
    reg [`PROTO_DATA_W-1:0] selected_src;

    // Combinational datapath temporaries.
    reg [`PROTO_GPIO_W-1:0] next_out;
    reg [`PROTO_GPIO_W-1:0] next_oe;
    reg                     out_changed;
    reg                     oe_changed;

    instruction_decoder u_decode (
        .instr        (ir),
        .opcode       (opcode),
        .payload11    (payload11),
        .aux3         (aux3),
        .arg8         (arg8),
        .cond         (cond),
        .branch_addr  (branch_addr),
        .waitpin_pin  (waitpin_pin),
        .reg_dst      (reg_dst),
        .reg_src      (reg_src),
        .imm8         (imm8),
        .cfg_addr     (dec_cfg_addr),
        .cfg_data     (dec_cfg_data),
        .opcode_known (opcode_known)
    );

    function eval_cond;
        input [2:0] c;
        input [2:0] pin_index;
        begin
            case (c)
                `COND_ZERO:     eval_cond = zero_flag;
                `COND_NZERO:    eval_cond = ~zero_flag;
                `COND_PIN_LOW:  eval_cond = ~gpio_in[pin_index];
                `COND_PIN_HIGH: eval_cond =  gpio_in[pin_index];
                `COND_PIN_RISE: eval_cond =  gpio_rise[pin_index];
                `COND_PIN_FALL: eval_cond =  gpio_fall[pin_index];
                `COND_TX_EMPTY: eval_cond =  tx_empty;
                `COND_RX_FULL:  eval_cond =  rx_full;
                default:        eval_cond = 1'b0;
            endcase
        end
    endfunction

    always @(*) begin
        case (reg_src)
            `REG_X:   selected_src = x_reg;
            `REG_Y:   selected_src = y_reg;
            `REG_LC:  selected_src = loop_count;
            default:  selected_src = {`PROTO_DATA_W{1'b0}};
        endcase
    end

    always @(*) begin
        side_capable = 1'b0;
        case (opcode)
            `OP_NOP,
            `OP_GPIO_SET, `OP_GPIO_CLR, `OP_GPIO_WRITE, `OP_OE_WRITE,
            `OP_OUT, `OP_IN, `OP_XFER:
                side_capable = 1'b1;
            default:
                side_capable = 1'b0;
        endcase

        side_value = aux3[2];
        if (cfg_side_enable)
            post_delay = {1'b0, aux3[1:0]};
        else
            post_delay = aux3;

        jcc_true     = eval_cond(cond, cfg_jmp_pin);
        waitpin_true = eval_cond(cond, waitpin_pin);
    end

    assign busy      = (state != ST_IDLE);
    assign imem_req  = (state == ST_FETCH_REQ);
    assign imem_addr = pc;

    // One-cycle control pulses and GPIO next-state calculation.
    always @(*) begin
        cfg_we          = 1'b0;
        cfg_addr        = dec_cfg_addr;
        cfg_wdata       = dec_cfg_data;

        gpio_out_we     = 1'b0;
        gpio_out_wdata  = gpio_out;
        gpio_oe_we      = 1'b0;
        gpio_oe_wdata   = gpio_oe;

        osr_load        = 1'b0;
        osr_load_data   = tx_pop_data;
        osr_shift       = 1'b0;
        isr_clear       = 1'b0;
        isr_shift       = 1'b0;
        shift_serial_in = gpio_in[cfg_in_pin];

        tx_pop          = 1'b0;
        rx_push         = 1'b0;
        rx_push_data    = shift_isr;

        next_out        = gpio_out;
        next_oe         = gpio_oe;
        out_changed     = 1'b0;
        oe_changed      = 1'b0;

        if (state == ST_EXEC) begin
            case (opcode)
                `OP_GPIO_SET: begin
                    next_out    = gpio_out | arg8;
                    out_changed = 1'b1;
                end

                `OP_GPIO_CLR: begin
                    next_out    = gpio_out & ~arg8;
                    out_changed = 1'b1;
                end

                `OP_GPIO_WRITE: begin
                    next_out    = arg8;
                    out_changed = 1'b1;
                end

                `OP_OE_WRITE: begin
                    next_oe    = arg8;
                    oe_changed = 1'b1;
                end

                `OP_OUT: begin
                    next_out[cfg_out_pin] = shift_serial_out;
                    out_changed           = 1'b1;
                    osr_shift             = 1'b1;
                end

                `OP_IN: begin
                    isr_shift = 1'b1;
                end

                `OP_XFER: begin
                    next_out[cfg_out_pin] = shift_serial_out;
                    out_changed           = 1'b1;
                    osr_shift             = 1'b1;
                    isr_shift             = 1'b1;
                end

                `OP_PULL: begin
                    if (!tx_empty) begin
                        tx_pop   = 1'b1;
                        osr_load = 1'b1;
                    end
                end

                `OP_PUSH: begin
                    if (!rx_full) begin
                        rx_push      = 1'b1;
                        rx_push_data = shift_isr;
                    end
                end

                `OP_CFG: begin
                    cfg_we    = 1'b1;
                    cfg_addr  = dec_cfg_addr;
                    cfg_wdata = dec_cfg_data;
                end

                default: begin
                end
            endcase

            // Side-set is atomic with the primary action and wins if both target
            // the same GPIO bit.
            if (side_capable && cfg_side_enable) begin
                if (cfg_side_target_oe) begin
                    next_oe[cfg_side_pin] = side_value;
                    oe_changed            = 1'b1;
                end else begin
                    next_out[cfg_side_pin] = side_value;
                    out_changed            = 1'b1;
                end
            end
        end

        gpio_out_we    = out_changed;
        gpio_out_wdata = next_out;
        gpio_oe_we     = oe_changed;
        gpio_oe_wdata  = next_oe;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            pc              <= {IMEM_AW{1'b0}};
            ir              <= {`PROTO_INSTR_W{1'b0}};
            x_reg           <= {`PROTO_DATA_W{1'b0}};
            y_reg           <= {`PROTO_DATA_W{1'b0}};
            loop_count      <= {`PROTO_DATA_W{1'b0}};
            zero_flag       <= 1'b1;
            wait_remaining  <= 11'h000;
            delay_remaining <= 3'b000;
            fault           <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (start) begin
                        pc    <= {IMEM_AW{1'b0}};
                        fault <= 1'b0;
                        state <= ST_FETCH_REQ;
                    end
                end

                ST_FETCH_REQ: begin
                    state <= ST_FETCH_WAIT;
                end

                ST_FETCH_WAIT: begin
                    if (imem_rvalid) begin
                        ir    <= imem_rdata;
                        state <= ST_EXEC;
                    end
                end

                ST_EXEC: begin
                    if (!opcode_known) begin
                        fault <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        case (opcode)
                            `OP_HALT: begin
                                state <= ST_IDLE;
                            end

                            `OP_WAIT: begin
                                if (payload11 == 11'h000) begin
                                    pc    <= pc + 1'b1;
                                    state <= ST_FETCH_REQ;
                                end else begin
                                    wait_remaining <= payload11;
                                    state          <= ST_WAIT_TICKS;
                                end
                            end

                            `OP_WAITPIN: begin
                                if (waitpin_true) begin
                                    pc    <= pc + 1'b1;
                                    state <= ST_FETCH_REQ;
                                end else begin
                                    state <= ST_WAIT_PIN;
                                end
                            end

                            `OP_PULL: begin
                                if (!tx_empty) begin
                                    pc    <= pc + 1'b1;
                                    state <= ST_FETCH_REQ;
                                end
                            end

                            `OP_PUSH: begin
                                if (!rx_full) begin
                                    pc    <= pc + 1'b1;
                                    state <= ST_FETCH_REQ;
                                end
                            end

                            `OP_JMP: begin
                                pc    <= branch_addr;
                                state <= ST_FETCH_REQ;
                            end

                            `OP_JCC: begin
                                if (jcc_true)
                                    pc <= branch_addr;
                                else
                                    pc <= pc + 1'b1;
                                state <= ST_FETCH_REQ;
                            end

                            `OP_LOOP: begin
                                if (loop_count > 8'd1) begin
                                    loop_count <= loop_count - 1'b1;
                                    zero_flag  <= 1'b0;
                                    pc         <= branch_addr;
                                end else begin
                                    loop_count <= {`PROTO_DATA_W{1'b0}};
                                    zero_flag  <= 1'b1;
                                    pc         <= pc + 1'b1;
                                end
                                state <= ST_FETCH_REQ;
                            end

                            `OP_LDI: begin
                                case (reg_dst)
                                    `REG_X:  x_reg      <= imm8;
                                    `REG_Y:  y_reg      <= imm8;
                                    `REG_LC: loop_count <= imm8;
                                    default: begin end
                                endcase
                                zero_flag <= (imm8 == 8'h00);
                                pc        <= pc + 1'b1;
                                state     <= ST_FETCH_REQ;
                            end

                            `OP_MOV: begin
                                case (reg_dst)
                                    `REG_X:  x_reg      <= selected_src;
                                    `REG_Y:  y_reg      <= selected_src;
                                    `REG_LC: loop_count <= selected_src;
                                    default: begin end
                                endcase
                                zero_flag <= (selected_src == {`PROTO_DATA_W{1'b0}});
                                pc        <= pc + 1'b1;
                                state     <= ST_FETCH_REQ;
                            end

                            // Reserved known encodings fault until their hardware
                            // semantics are implemented.
                            `OP_ALU, `OP_EVENT, `OP_CRC, `OP_STREAM,
                            `OP_LOAD, `OP_STORE: begin
                                fault <= 1'b1;
                                state <= ST_IDLE;
                            end

                            default: begin
                                // NOP, GPIO, OUT/IN/XFER and CFG complete here.
                                if (side_capable && (post_delay != 3'b000)) begin
                                    delay_remaining <= post_delay;
                                    state           <= ST_POST_DELAY;
                                end else begin
                                    pc    <= pc + 1'b1;
                                    state <= ST_FETCH_REQ;
                                end
                            end
                        endcase
                    end
                end

                ST_WAIT_TICKS: begin
                    if (tick) begin
                        if (wait_remaining <= 11'd1) begin
                            wait_remaining <= 11'h000;
                            pc             <= pc + 1'b1;
                            state          <= ST_FETCH_REQ;
                        end else begin
                            wait_remaining <= wait_remaining - 1'b1;
                        end
                    end
                end

                ST_WAIT_PIN: begin
                    if (waitpin_true) begin
                        pc    <= pc + 1'b1;
                        state <= ST_FETCH_REQ;
                    end
                end

                ST_POST_DELAY: begin
                    if (tick) begin
                        if (delay_remaining <= 3'd1) begin
                            delay_remaining <= 3'b000;
                            pc              <= pc + 1'b1;
                            state           <= ST_FETCH_REQ;
                        end else begin
                            delay_remaining <= delay_remaining - 1'b1;
                        end
                    end
                end

                default: begin
                    fault <= 1'b1;
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire

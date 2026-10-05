`default_nettype none

`include "proto_pkg.sv"

module protocol_core #(
    parameter integer IMEM_AW = 8
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 start,

    output wire                busy,
    output reg                 fault,

    // Instruction-memory interface.
    output reg                 imem_req,
    output reg [IMEM_AW-1:0]   imem_addr,
    input  wire [15:0]          imem_rdata,
    input  wire                 imem_rvalid,

    // Protocol timing tick from proto_tick.
    input  wire                 tick,

    // Configuration values.
    input  wire [2:0]           cfg_out_pin,
    input  wire [2:0]           cfg_in_pin,
    input  wire [2:0]           cfg_side_pin,
    input  wire                 cfg_out_shift_right,
    input  wire                 cfg_in_shift_right,
    input  wire                 cfg_side_enable,
    input  wire                 cfg_side_target_oe,
    input  wire [2:0]           cfg_jmp_pin,

    // OP_CFG writeback.
    output reg                 cfg_we,
    output reg [2:0]           cfg_addr,
    output reg [7:0]           cfg_wdata,

    // GPIO state and write controls.
    input  wire [7:0]           gpio_in,
    input  wire [7:0]           gpio_out,
    input  wire [7:0]           gpio_oe,
    input  wire [7:0]           gpio_rise,
    input  wire [7:0]           gpio_fall,
    output reg                 gpio_out_we,
    output reg [7:0]           gpio_out_wdata,
    output reg                 gpio_oe_we,
    output reg [7:0]           gpio_oe_wdata,

    // Shared shift engine.
    input  wire                 shift_serial_out,
    input  wire [7:0]           shift_isr,
    output reg                 osr_load,
    output reg [7:0]           osr_load_data,
    output reg                 osr_shift,
    output reg                 osr_shift_right,
    output reg                 isr_clear,
    output reg                 isr_shift,
    output reg                 isr_shift_right,
    output reg                 shift_serial_in,

    // TX FIFO: host pushes, core pops.
    input  wire                 tx_empty,
    input  wire [7:0]           tx_pop_data,
    output reg                 tx_pop,

    // RX FIFO: core pushes, host pops.
    input  wire                 rx_full,
    output reg                 rx_push,
    output reg [7:0]           rx_push_data
);
    // Explicit FSM encoding avoids SystemVerilog enum/package dependencies.
    localparam [2:0] ST_IDLE       = 3'd0;
    localparam [2:0] ST_FETCH_REQ  = 3'd1;
    localparam [2:0] ST_FETCH_WAIT = 3'd2;
    localparam [2:0] ST_EXEC       = 3'd3;
    localparam [2:0] ST_WAIT_TICKS = 3'd4;
    localparam [2:0] ST_WAIT_PIN   = 3'd5;
    localparam [2:0] ST_POST_DELAY = 3'd6;

    reg [2:0] state;

    reg [IMEM_AW-1:0] pc;
    reg [15:0]        ir;

    reg [7:0] x_reg;
    reg [7:0] y_reg;
    reg [7:0] loop_count;
    reg       zero_flag;

    reg [10:0] wait_remaining;
    reg [2:0]  delay_remaining;

    wire [4:0] opcode;
    wire [10:0] payload11;
    wire [2:0] aux3;
    wire [7:0] arg8;
    wire [2:0] cond;
    wire [7:0] branch_addr;
    wire [2:0] waitpin_pin;
    wire [2:0] reg_dst;
    wire [2:0] reg_src;
    wire [7:0] imm8;
    wire [2:0] dec_cfg_addr;
    wire [7:0] dec_cfg_data;
    wire opcode_known;

    reg side_capable;
    reg side_value;
    reg [2:0] post_delay;
    reg jcc_true;
    reg waitpin_true;

    reg [7:0] selected_src;

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
                default:       eval_cond = 1'b0;
            endcase
        end
    endfunction

    always @(*) begin
        case (reg_src)
            `REG_X:   selected_src = x_reg;
            `REG_Y:   selected_src = y_reg;
            `REG_LC:  selected_src = loop_count;
            default: selected_src = 8'h00;
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

    assign busy = (state != ST_IDLE);

    // Memory request phase. This conservative V0 sequencer intentionally uses
    // explicit request/wait/execute states. It is easy to verify against a
    // one-cycle synchronous SRAM. A prefetch pipeline can later remove fetch
    // bubbles without changing the ISA.
    always @(*) begin
        imem_req  = (state == ST_FETCH_REQ);
        imem_addr = pc;
    end

    // Datapath control pulses generated in ST_EXEC.
    always @(*) begin : exec_controls
        reg [7:0] next_out;
        reg [7:0] next_oe;
        reg       out_changed;
        reg       oe_changed;

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
        osr_shift_right = cfg_out_shift_right;
        isr_clear       = 1'b0;
        isr_shift       = 1'b0;
        isr_shift_right = cfg_in_shift_right;
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

            // Side-set happens atomically with the primary instruction action.
            // If side-set targets the same bit as the primary action, side-set
            // wins because it is applied last.
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
            ir              <= 16'h0000;
            x_reg           <= 8'h00;
            y_reg           <= 8'h00;
            loop_count      <= 8'h00;
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
                                // else remain in ST_EXEC until host/data path supplies data.
                            end

                            `OP_PUSH: begin
                                if (!rx_full) begin
                                    pc    <= pc + 1'b1;
                                    state <= ST_FETCH_REQ;
                                end
                                // else remain in ST_EXEC until space is available.
                            end

                            `OP_JMP: begin
                                pc    <= branch_addr[IMEM_AW-1:0];
                                state <= ST_FETCH_REQ;
                            end

                            `OP_JCC: begin
                                if (jcc_true)
                                    pc <= branch_addr[IMEM_AW-1:0];
                                else
                                    pc <= pc + 1'b1;
                                state <= ST_FETCH_REQ;
                            end

                            `OP_LOOP: begin
                                if (loop_count > 8'd1) begin
                                    loop_count <= loop_count - 1'b1;
                                    zero_flag  <= 1'b0;
                                    pc         <= branch_addr[IMEM_AW-1:0];
                                end else begin
                                    loop_count <= 8'h00;
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
                                zero_flag <= (selected_src == 8'h00);
                                pc        <= pc + 1'b1;
                                state     <= ST_FETCH_REQ;
                            end

                            // Reserved instructions are known encodings but are
                            // intentionally faults until their hardware semantics
                            // are implemented.
                            `OP_ALU, `OP_EVENT, `OP_CRC, `OP_STREAM, `OP_LOAD, `OP_STORE: begin
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

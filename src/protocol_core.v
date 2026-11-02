`default_nettype none
`include "proto_params.vh"
`include "proto_isa.vh"

// V0 protocol execution core.
//
// Instruction fetch remains conservative and synchronous:
// FETCH_REQ -> FETCH_WAIT -> EXEC.
//
// LOAD/STORE use a separate 256-byte Harvard data-memory port.  STORE commits
// on the EXEC clock edge.  LOAD enters ST_DMEM_WAIT and completes when the
// synchronous data memory raises dmem_rvalid.
module protocol_core #(
    parameter integer IMEM_AW = `PROTO_IMEM_AW,
    parameter integer DMEM_AW = `PROTO_DMEM_AW
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

    output reg                          dmem_req,
    output reg                          dmem_we,
    output reg  [DMEM_AW-1:0]           dmem_addr,
    output reg  [`PROTO_DATA_W-1:0]     dmem_wdata,
    input  wire [`PROTO_DATA_W-1:0]     dmem_rdata,
    input  wire                         dmem_rvalid,

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
    input  wire [`PROTO_SHIFT_W-1:0]    shift_osr,
    input  wire [`PROTO_SHIFT_W-1:0]    shift_isr,
    output reg                          osr_load,
    output reg  [`PROTO_SHIFT_W-1:0]    osr_load_data,
    output reg                          osr_shift,
    output reg                          isr_load,
    output reg  [`PROTO_SHIFT_W-1:0]    isr_load_data,
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

    // All eight encodings fit in the existing 3-bit state field.
    localparam [2:0] ST_IDLE       = 3'd0;
    localparam [2:0] ST_FETCH_REQ  = 3'd1;
    localparam [2:0] ST_FETCH_WAIT = 3'd2;
    localparam [2:0] ST_EXEC       = 3'd3;
    localparam [2:0] ST_WAIT_TICKS = 3'd4;
    localparam [2:0] ST_WAIT_PIN   = 3'd5;
    localparam [2:0] ST_POST_DELAY = 3'd6;
    localparam [2:0] ST_DMEM_WAIT  = 3'd7;

    reg [2:0] state;
    reg [IMEM_AW-1:0] pc;
    reg [`PROTO_INSTR_W-1:0] ir;

    reg [`PROTO_DATA_W-1:0] x_reg;
    reg [`PROTO_DATA_W-1:0] y_reg;
    reg [`PROTO_DATA_W-1:0] loop_count;
    reg                     zero_flag;

    // LOAD destination is latched when the request is issued.  This makes the
    // memory transaction independent of IR remaining stable in future fetch
    // optimizations.
    reg [1:0] load_dst;

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
    wire [1:0]  mem_reg;
    wire        mem_base;
    wire [7:0]  mem_offset;
    wire [3:0]  alu_op;
    wire [1:0]  alu_dst;
    wire [1:0]  alu_src_a;
    wire [1:0]  alu_src_b;
    wire        opcode_known;

    reg                     side_capable;
    reg                     side_value;
    reg [2:0]               post_delay;
    reg                     jcc_true;
    reg                     waitpin_true;
    reg [`PROTO_DATA_W-1:0] selected_src;

    reg [`PROTO_DATA_W-1:0] mem_base_value;
    reg [`PROTO_DATA_W-1:0] mem_store_value;
    reg [`PROTO_DATA_W-1:0] mem_effective_addr;

    reg  [`PROTO_DATA_W-1:0] alu_a_value;
    reg  [`PROTO_DATA_W-1:0] alu_b_value;
    wire [`PROTO_DATA_W-1:0] alu_result;
    wire                     alu_zero;
    wire                     alu_write_result;

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
        .mem_reg      (mem_reg),
        .mem_base     (mem_base),
        .mem_offset   (mem_offset),
        .alu_op       (alu_op),
        .alu_dst      (alu_dst),
        .alu_src_a    (alu_src_a),
        .alu_src_b    (alu_src_b),
        .opcode_known (opcode_known)
    );

    // Condition evaluation is explicit so Verilog always @(*) sensitivity
    // includes the runtime GPIO/flag dependencies.
    always @(*) begin
        jcc_true     = 1'b0;
        waitpin_true = 1'b0;

        case (cond)
            `COND_ZERO: begin
                jcc_true     = zero_flag;
                waitpin_true = zero_flag;
            end

            `COND_NZERO: begin
                jcc_true     = ~zero_flag;
                waitpin_true = ~zero_flag;
            end

            `COND_PIN_LOW: begin
                jcc_true     = ~gpio_in[cfg_jmp_pin];
                waitpin_true = ~gpio_in[waitpin_pin];
            end

            `COND_PIN_HIGH: begin
                jcc_true     = gpio_in[cfg_jmp_pin];
                waitpin_true = gpio_in[waitpin_pin];
            end

            `COND_PIN_RISE: begin
                jcc_true     = gpio_rise[cfg_jmp_pin];
                waitpin_true = gpio_rise[waitpin_pin];
            end

            `COND_PIN_FALL: begin
                jcc_true     = gpio_fall[cfg_jmp_pin];
                waitpin_true = gpio_fall[waitpin_pin];
            end

            `COND_TX_EMPTY: begin
                jcc_true     = tx_empty;
                waitpin_true = tx_empty;
            end

            `COND_RX_FULL: begin
                jcc_true     = rx_full;
                waitpin_true = rx_full;
            end

            default: begin
                jcc_true     = 1'b0;
                waitpin_true = 1'b0;
            end
        endcase
    end

    // MOV source selector remains the original X/Y/LC register bank.
    always @(*) begin
        case (reg_src)
            `REG_X:   selected_src = x_reg;
            `REG_Y:   selected_src = y_reg;
            `REG_LC:  selected_src = loop_count;
            default:  selected_src = {`PROTO_DATA_W{1'b0}};
        endcase
    end

    // DMEM address and STORE source selection.
    //
    // mem_offset is encoded as signed two's-complement, but because both the
    // base register and DMEM address are 8 bits, ordinary 8-bit addition gives
    // the desired modulo-256 result for both positive and negative offsets.
    always @(*) begin
        case (mem_base)
            `MEM_BASE_Y: mem_base_value = y_reg;
            default:     mem_base_value = x_reg;
        endcase

        mem_effective_addr = mem_base_value + mem_offset;

        case (mem_reg)
            `MEM_REG_X:   mem_store_value = x_reg;
            `MEM_REG_Y:   mem_store_value = y_reg;
            `MEM_REG_OSR: mem_store_value = shift_osr;
            `MEM_REG_ISR: mem_store_value = shift_isr;
            default:      mem_store_value = {`PROTO_DATA_W{1'b0}};
        endcase
    end


    // ALU source operands come from the same byte registers used by DMEM:
    // X, Y, OSR and ISR.
    always @(*) begin
        case (alu_src_a)
            `ALU_REG_X:   alu_a_value = x_reg;
            `ALU_REG_Y:   alu_a_value = y_reg;
            `ALU_REG_OSR: alu_a_value = shift_osr;
            `ALU_REG_ISR: alu_a_value = shift_isr;
            default:      alu_a_value = {`PROTO_DATA_W{1'b0}};
        endcase

        case (alu_src_b)
            `ALU_REG_X:   alu_b_value = x_reg;
            `ALU_REG_Y:   alu_b_value = y_reg;
            `ALU_REG_OSR: alu_b_value = shift_osr;
            `ALU_REG_ISR: alu_b_value = shift_isr;
            default:      alu_b_value = {`PROTO_DATA_W{1'b0}};
        endcase
    end

    proto_alu u_alu (
        .op           (alu_op),
        .a            (alu_a_value),
        .b            (alu_b_value),
        .result       (alu_result),
        .zero         (alu_zero),
        .write_result (alu_write_result)
    );

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
    end

    assign busy      = (state != ST_IDLE);
    assign imem_req  = (state == ST_FETCH_REQ);
    assign imem_addr = pc;

    // One-cycle control pulses, memory request generation, and GPIO next-state
    // calculation.
    always @(*) begin
        cfg_we          = 1'b0;
        cfg_addr        = dec_cfg_addr;
        cfg_wdata       = dec_cfg_data;

        dmem_req        = 1'b0;
        dmem_we         = 1'b0;
        dmem_addr       = mem_effective_addr[DMEM_AW-1:0];
        dmem_wdata      = mem_store_value;

        gpio_out_we     = 1'b0;
        gpio_out_wdata  = gpio_out;
        gpio_oe_we      = 1'b0;
        gpio_oe_wdata   = gpio_oe;

        osr_load        = 1'b0;
        osr_load_data   = tx_pop_data;
        osr_shift       = 1'b0;
        isr_load        = 1'b0;
        isr_load_data   = dmem_rdata;
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

                `OP_LOAD: begin
                    dmem_req   = 1'b1;
                    dmem_we    = 1'b0;
                    dmem_addr  = mem_effective_addr[DMEM_AW-1:0];
                end

                `OP_STORE: begin
                    dmem_req   = 1'b1;
                    dmem_we    = 1'b1;
                    dmem_addr  = mem_effective_addr[DMEM_AW-1:0];
                    dmem_wdata = mem_store_value;
                end

                `OP_ALU: begin
                    // X/Y destinations are written in the sequential block.
                    // OSR/ISR are external to the core, so generate the same
                    // one-cycle load pulses used by PULL/DMEM LOAD.
                    if (alu_write_result) begin
                        case (alu_dst)
                            `ALU_REG_OSR: begin
                                osr_load      = 1'b1;
                                osr_load_data = alu_result;
                            end

                            `ALU_REG_ISR: begin
                                isr_load      = 1'b1;
                                isr_load_data = alu_result;
                            end

                            default: begin
                            end
                        endcase
                    end
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

        // LOAD to OSR/ISR completes when synchronous DMEM data is valid.
        // X/Y destinations are written in the sequential state machine below.
        if ((state == ST_DMEM_WAIT) && dmem_rvalid) begin
            case (load_dst)
                `MEM_REG_OSR: begin
                    osr_load      = 1'b1;
                    osr_load_data = dmem_rdata;
                end

                `MEM_REG_ISR: begin
                    isr_load      = 1'b1;
                    isr_load_data = dmem_rdata;
                end

                default: begin
                end
            endcase
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
            load_dst        <= `MEM_REG_X;
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

                            `OP_LOAD: begin
                                load_dst <= mem_reg;
                                state    <= ST_DMEM_WAIT;
                            end

                            `OP_STORE: begin
                                // The synchronous SRAM accepts the write on
                                // this EXEC clock edge; no response is needed.
                                pc    <= pc + 1'b1;
                                state <= ST_FETCH_REQ;
                            end

                            `OP_ALU: begin
                                if (alu_write_result) begin
                                    case (alu_dst)
                                        `ALU_REG_X: x_reg <= alu_result;
                                        `ALU_REG_Y: y_reg <= alu_result;
                                        // OSR/ISR are updated by combinational
                                        // load pulses on this same clock edge.
                                        `ALU_REG_OSR: begin end
                                        `ALU_REG_ISR: begin end
                                        default: begin end
                                    endcase
                                end

                                zero_flag <= alu_zero;
                                pc        <= pc + 1'b1;
                                state     <= ST_FETCH_REQ;
                            end

                            // Reserved known encodings fault until their hardware
                            // semantics are implemented.
                            `OP_EVENT, `OP_CRC, `OP_STREAM: begin
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

                ST_DMEM_WAIT: begin
                    if (dmem_rvalid) begin
                        case (load_dst)
                            `MEM_REG_X: x_reg <= dmem_rdata;
                            `MEM_REG_Y: y_reg <= dmem_rdata;
                            // OSR/ISR are loaded by combinational pulses into
                            // proto_shift on this same clock edge.
                            `MEM_REG_OSR: begin end
                            `MEM_REG_ISR: begin end
                            default: begin end
                        endcase

                        zero_flag <= (dmem_rdata == {`PROTO_DATA_W{1'b0}});
                        pc        <= pc + 1'b1;
                        state     <= ST_FETCH_REQ;
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

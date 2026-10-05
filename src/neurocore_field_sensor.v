// ============================================================================
// NeuroCore Field Sensor -- event-driven digital core
// ============================================================================
// Copyright (c) 2024-2026 Christoph Kassir, Ben Liu, Daniel Zheng, Michael Lander
// SPDX-License-Identifier: Apache-2.0
//
// Overview
// --------
// An external always-on comparator (not part of this chip) raises `wake` when
// the sensed signal crosses a coarse threshold. Each wake runs one pass of a
// fixed-function pipeline, sends the resulting 3-bit command back over an LSK
// (load-shift-keying) backscatter link, and returns the core to sleep.
//
//   adc_data -> FIR filter -> 3-level Haar DWT -> |x| -> x^2 per band
//                                                              |
//   target_idx ------------------> dominant-band search -------+
//                                          |
//                                    3-bit command -> 14-bit Manchester LSK packet
//
// Input timing contract
// ---------------------
//   wake        Any pulse >= 1 clk. Sampled only while idle: a wake that
//               arrives while the pipeline is busy is dropped, not queued.
//   adc_valid   Exactly 1 clk wide per sample. The sample shift register
//               advances once per clock that the synchronised strobe is high.
//   adc_data    NOT synchronised. It is captured two clocks after adc_valid is
//               first sampled, so the source must hold it stable for at least
//               3 clocks starting at the adc_valid assertion.
//   target_idx  Quasi-static frequency-bin index the external TMS should match.
//               Read by the encoder on the last cycle of its scan.
//
// Main FSM
// --------
//   IDLE -wake-> WAKE -> FIR -> WAIT_FIR -> DWT -> WAIT_DWT -> ABS -> WAIT_ABS
//    ^                                                                    |
//    |     +--------------------------------------------------------------+
//    |     v
//    |    BAND -> WAIT_BAND -> ENCODE -> LSK_TX -> LSK_ACK -> LSK_WAIT
//    |                                                            |
//    +--- SLEEP <-------------------------------------------------+
//
//   Every WAIT_* state advances on its block's `valid` strobe. A watchdog
//   forces SLEEP if a wait state stalls (see the watchdog comment below).
// ============================================================================

`default_nettype none
/* verilator lint_off DECLFILENAME */

module neurocore_field_sensor #(
    parameter ADC_BITS  = 4,    // ADC sample width (two's complement)
    parameter FIR_WIDTH = 8,    // FIR datapath width
    parameter DWT_WIDTH = 12,   // DWT and magnitude datapath width
    parameter WDT_BITS  = 16    // Watchdog counter width; timeout = 2^(WDT_BITS-1) clks
) (
    input  wire                clk,
    input  wire                rst_n,

    // ADC interface
    input  wire [ADC_BITS-1:0] adc_data,
    input  wire                adc_valid,

    // Event detection
    input  wire                wake,

    // Frequency-bin index the external TMS should match
    input  wire [2:0]          target_idx,

    // Command output
    output wire [2:0]          cmd_out,
    output wire                cmd_valid,

    // LSK backscatter modulator
    output wire                lsk_ctrl,
    output wire                lsk_tx,

    // Power gating and status
    output wire                pwr_gate_ctrl,
    output wire                fir_busy,
    output wire                dwt_busy,
    output wire                processing
);

    localparam PWR_WIDTH = 16;   // per-band power width

    // ------------------------------------------------------------------------
    // Input synchronisers (2-FF) for wake and adc_valid
    // ------------------------------------------------------------------------
    reg wake_meta,      wake_sync;
    reg adc_valid_meta, adc_valid_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wake_meta      <= 1'b0;
            wake_sync      <= 1'b0;
            adc_valid_meta <= 1'b0;
            adc_valid_sync <= 1'b0;
        end else begin
            wake_meta      <= wake;
            wake_sync      <= wake_meta;
            adc_valid_meta <= adc_valid;
            adc_valid_sync <= adc_valid_meta;
        end
    end

    // ------------------------------------------------------------------------
    // Inter-block signals
    // ------------------------------------------------------------------------
    wire [FIR_WIDTH-1:0]       fir_out;
    wire                       fir_start, fir_valid;

    wire [8*DWT_WIDTH-1:0]     dwt_bus;     // {sub_7 .. sub_0}, see dwt_haar_lift
    wire                       dwt_start, dwt_valid;

    wire [8*DWT_WIDTH-1:0]     abs_bus;     // |sub_n|, same lane order
    wire                       abs_start, abs_valid;

    wire [8*PWR_WIDTH-1:0]     power_bus;   // sub_n^2, same lane order
    wire                       bp_start, bp_valid;

    wire [2:0]                 cmd_encoded;
    wire                       cmd_ready;

    // ------------------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------------------
    localparam [3:0] S_IDLE      = 4'd0,
                     S_WAKE      = 4'd1,
                     S_FIR       = 4'd2,
                     S_WAIT_FIR  = 4'd3,
                     S_DWT       = 4'd4,
                     S_WAIT_DWT  = 4'd5,
                     S_ABS       = 4'd6,
                     S_WAIT_ABS  = 4'd7,
                     S_BAND      = 4'd8,
                     S_WAIT_BAND = 4'd9,
                     S_ENCODE    = 4'd10,
                     S_LSK_TX    = 4'd11,
                     S_LSK_WAIT  = 4'd12,
                     S_SLEEP     = 4'd13,
                     S_LSK_ACK   = 4'd14;

    reg [3:0] state, next_state;

    // Watchdog: counts consecutive clocks spent in the states listed below. When
    // the MSB sets (2^(WDT_BITS-1) clocks, 32768 at the default) the FSM is
    // forced to SLEEP.
    //   - Covers every state that waits on a done strobe: WAIT_FIR, WAIT_DWT,
    //     WAIT_ABS, WAIT_BAND and ENCODE.
    //   - S_ABS and S_LSK_TX are also listed. Each lasts exactly one clock, so
    //     the only effect is that the count carries straight through them.
    //   - S_LSK_WAIT is not covered: its length is set by the LSK packet
    //     (14 * BIT_PERIOD clocks), and lsk_modulator always finishes.
    //   - The count restarts whenever the FSM passes through a state outside the
    //     list (S_DWT and S_BAND, for example), so back-to-back covered states
    //     (WAIT_DWT, ABS, WAIT_ABS and WAIT_BAND, ENCODE, LSK_TX) share one
    //     timeout.
    reg  [WDT_BITS-1:0] wdt_cnt;

    wire in_watchdog_state = (state == S_WAIT_FIR)  || (state == S_WAIT_DWT)  ||
                             (state == S_ABS)       || (state == S_WAIT_ABS)  ||
                             (state == S_WAIT_BAND) || (state == S_ENCODE)    ||
                             (state == S_LSK_TX);

    wire wdt_timeout = in_watchdog_state && wdt_cnt[WDT_BITS-1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            wdt_cnt <= {WDT_BITS{1'b0}};
        end else begin
            state   <= next_state;
            wdt_cnt <= in_watchdog_state ? wdt_cnt + 1'b1 : {WDT_BITS{1'b0}};
        end
    end

    always @(*) begin
        next_state = state;
        if (wdt_timeout) begin
            next_state = S_SLEEP;
        end else begin
            case (state)
                S_IDLE:       if (wake_sync) next_state = S_WAKE;
                S_WAKE:                      next_state = S_FIR;
                S_FIR:                       next_state = S_WAIT_FIR;
                S_WAIT_FIR:   if (fir_valid) next_state = S_DWT;
                S_DWT:                       next_state = S_WAIT_DWT;
                S_WAIT_DWT:   if (dwt_valid) next_state = S_ABS;
                S_ABS:                       next_state = S_WAIT_ABS;
                S_WAIT_ABS:   if (abs_valid) next_state = S_BAND;
                S_BAND:                      next_state = S_WAIT_BAND;
                S_WAIT_BAND:  if (bp_valid)  next_state = S_ENCODE;
                S_ENCODE:     if (cmd_ready) next_state = S_LSK_TX;
                S_LSK_TX:                    next_state = S_LSK_ACK;
                S_LSK_ACK:                   next_state = S_LSK_WAIT;
                S_LSK_WAIT:   if (!lsk_tx)   next_state = S_SLEEP;
                S_SLEEP:                     next_state = S_IDLE;
                default:                     next_state = S_IDLE;
            endcase
        end
    end

    // Block start strobes
    assign fir_start = (state == S_FIR);
    assign dwt_start = (state == S_DWT);
    assign abs_start = (state == S_ABS);
    assign bp_start  = (state == S_BAND);

    // Active in every state except IDLE and SLEEP. pwr_gate_ctrl and processing
    // are currently the same signal. pwr_gate_ctrl is a control output for an
    // external power switch; nothing inside this module is power-gated.
    wire core_active = (state != S_IDLE) && (state != S_SLEEP);
    assign pwr_gate_ctrl = core_active;
    assign processing    = core_active;

    // One-clock start pulse for the modulator, registered one clock after the
    // FSM enters S_LSK_TX. S_LSK_ACK then gives tx_active time to rise before
    // S_LSK_WAIT starts checking it.
    reg tx_start_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tx_start_r <= 1'b0;
        else        tx_start_r <= (state == S_LSK_TX);
    end

    // ------------------------------------------------------------------------
    // Datapath
    // ------------------------------------------------------------------------
    fir_filter #(
        .WIDTH (FIR_WIDTH)
    ) u_fir (
        .clk        (clk),
        .rst_n      (rst_n),
        .data_in    ({{(FIR_WIDTH-ADC_BITS){adc_data[ADC_BITS-1]}}, adc_data}),
        .data_valid (adc_valid_sync),
        .start      (fir_start),
        .data_out   (fir_out),
        .out_valid  (fir_valid),
        .busy       (fir_busy)
    );

    dwt_haar_lift #(
        .WIDTH (DWT_WIDTH)
    ) u_dwt (
        .clk        (clk),
        .rst_n      (rst_n),
        .data_in    ({{(DWT_WIDTH-FIR_WIDTH){fir_out[FIR_WIDTH-1]}}, fir_out}),
        .data_valid (fir_valid),
        .start      (dwt_start),
        .sub_0      (dwt_bus[0*DWT_WIDTH +: DWT_WIDTH]),
        .sub_1      (dwt_bus[1*DWT_WIDTH +: DWT_WIDTH]),
        .sub_2      (dwt_bus[2*DWT_WIDTH +: DWT_WIDTH]),
        .sub_3      (dwt_bus[3*DWT_WIDTH +: DWT_WIDTH]),
        .sub_4      (dwt_bus[4*DWT_WIDTH +: DWT_WIDTH]),
        .sub_5      (dwt_bus[5*DWT_WIDTH +: DWT_WIDTH]),
        .sub_6      (dwt_bus[6*DWT_WIDTH +: DWT_WIDTH]),
        .sub_7      (dwt_bus[7*DWT_WIDTH +: DWT_WIDTH]),
        .out_valid  (dwt_valid),
        .busy       (dwt_busy)
    );

    abs_mag_bank #(
        .WIDTH (DWT_WIDTH)
    ) u_abs (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (abs_start),
        .x          (dwt_bus),
        .mag        (abs_bus),
        .out_valid  (abs_valid)
    );

    band_power_ts #(
        .IN_WIDTH  (DWT_WIDTH),
        .OUT_WIDTH (PWR_WIDTH)
    ) u_band_power (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (bp_start),
        .mag        (abs_bus),
        .power      (power_bus),
        .out_valid  (bp_valid)
    );

    command_encoder #(
        .BIN_WIDTH (PWR_WIDTH)
    ) u_cmd_encoder (
        .clk        (clk),
        .rst_n      (rst_n),
        .power_in   (power_bus),
        .target_idx (target_idx),
        .encode_en  (state == S_ENCODE),
        .cmd_out    (cmd_encoded),
        .cmd_ready  (cmd_ready)
    );

    lsk_modulator u_lsk_mod (
        .clk        (clk),
        .rst_n      (rst_n),
        .cmd_in     (cmd_encoded),
        .tx_start   (tx_start_r),
        .lsk_ctrl   (lsk_ctrl),
        .tx_active  (lsk_tx)
    );

    assign cmd_out   = cmd_encoded;
    assign cmd_valid = cmd_ready;

endmodule


// ############################################################################
//  Sub-modules
// ############################################################################

// ============================================================================
// 8-tap FIR filter (fixed coefficients, shift-add, no multipliers)
// ============================================================================
// Smooths the ADC stream before the wavelet stage. The sample shift register
// advances on every `data_valid`, independent of `start`. A `start` pulse runs
// one 8-clock shift-and-accumulate pass over the current register contents.
//
//   tap       0     1     2     3     4     5     6     7
//   weight   1/4   1/8   1/16  1/16  1/16  1/16  1/8   1/4      (sum = 1)
//   shift    >>>2  >>>3  >>>4  >>>4  >>>4  >>>4  >>>3  >>>2
//
// All arithmetic is two's complement; the shifts are arithmetic.
//
// Each tap is shifted before it is summed, so every term rounds toward minus
// infinity, and the 4-bit ADC input has no fractional bits to absorb that. The
// effective DC gain is therefore far from the nominal 1: a constant +7 in gives
// +2 out, a constant -1 gives -8. Changing this changes every downstream value.
// ============================================================================
module fir_filter #(
    parameter WIDTH = 8
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire [WIDTH-1:0]        data_in,
    input  wire                    data_valid,   // shift data_in into the delay line
    input  wire                    start,        // begin one filter pass
    output reg  signed [WIDTH-1:0] data_out,
    output reg                     out_valid,    // 1-clk pulse when data_out is final
    output wire                    busy
);

    localparam TAPS = 8;
    localparam [2:0] LAST_TAP = 3'd7;

    reg signed [WIDTH-1:0] delay_line [0:TAPS-1];
    reg [2:0] tap_count;
    reg       running;
    integer   i;

    assign busy = running;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap_count <= 3'd0;
            running   <= 1'b0;
            out_valid <= 1'b0;
            data_out  <= {WIDTH{1'b0}};
            for (i = 0; i < TAPS; i = i + 1)
                delay_line[i] <= {WIDTH{1'b0}};
        end else begin
            out_valid <= 1'b0;

            // Sample shift register
            if (data_valid) begin
                delay_line[0] <= data_in;
                for (i = 1; i < TAPS; i = i + 1)
                    delay_line[i] <= delay_line[i-1];
            end

            // Start a pass
            if (start && !running) begin
                running   <= 1'b1;
                tap_count <= 3'd0;
                data_out  <= {WIDTH{1'b0}};
            end

            // One tap per clock
            if (running) begin
                case (tap_count)
                    3'd0: data_out <= data_out + (delay_line[0] >>> 2);
                    3'd1: data_out <= data_out + (delay_line[1] >>> 3);
                    3'd2: data_out <= data_out + (delay_line[2] >>> 4);
                    3'd3: data_out <= data_out + (delay_line[3] >>> 4);
                    3'd4: data_out <= data_out + (delay_line[4] >>> 4);
                    3'd5: data_out <= data_out + (delay_line[5] >>> 4);
                    3'd6: data_out <= data_out + (delay_line[6] >>> 3);
                    3'd7: data_out <= data_out + (delay_line[7] >>> 2);
                endcase

                if (tap_count == LAST_TAP) begin
                    running   <= 1'b0;
                    out_valid <= 1'b1;
                end else begin
                    tap_count <= tap_count + 3'd1;
                end
            end
        end
    end

endmodule


// ============================================================================
// Haar lifting DWT, 3 levels over an 8-sample window
// ============================================================================
// `data_valid` writes one sample into an 8-entry circular buffer. Here it is
// driven by the FIR's `out_valid`, which pulses once per wake, so the buffer
// holds the FIR output from the last 8 wakes (zeros until it has filled).
// The write pointer is never reset on `start`. The transform always reads fixed
// buffer positions [0]..[7]; once the buffer has wrapped, those positions are
// not in age order.
// `start` runs the transform over the buffer in four clocks (L1, L2, L3, OUT).
//
// Lifting step on an (even, odd) pair:   d = odd - even;   a = even + d/2
//
//   output   contents                  derived from
//   ------   -----------------------   --------------------------
//   sub_0    L3 approximation          L2 approx pair
//   sub_1    L3 detail                 L2 approx pair
//   sub_2,3  L2 details                L1 approx pairs (0,1), (2,3)
//   sub_4-7  L1 details                buffer pairs (0,1) .. (6,7)
// ============================================================================
module dwt_haar_lift #(
    parameter WIDTH = 12
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire [WIDTH-1:0]        data_in,
    input  wire                    data_valid,
    input  wire                    start,
    output reg  signed [WIDTH-1:0] sub_0,
    output reg  signed [WIDTH-1:0] sub_1,
    output reg  signed [WIDTH-1:0] sub_2,
    output reg  signed [WIDTH-1:0] sub_3,
    output reg  signed [WIDTH-1:0] sub_4,
    output reg  signed [WIDTH-1:0] sub_5,
    output reg  signed [WIDTH-1:0] sub_6,
    output reg  signed [WIDTH-1:0] sub_7,
    output reg                     out_valid,
    output wire                    busy
);

    localparam [2:0] ST_IDLE = 3'd0,
                     ST_L1   = 3'd1,
                     ST_L2   = 3'd2,
                     ST_L3   = 3'd3,
                     ST_OUT  = 3'd4;

    function signed [WIDTH-1:0] haar_d(input signed [WIDTH-1:0] even,
                                       input signed [WIDTH-1:0] odd);
        haar_d = odd - even;
    endfunction

    function signed [WIDTH-1:0] haar_a(input signed [WIDTH-1:0] even,
                                       input signed [WIDTH-1:0] odd);
        haar_a = even + ((odd - even) >>> 1);
    endfunction

    reg signed [WIDTH-1:0] buf_r [0:7];
    reg [2:0] wr_ptr;
    reg [2:0] step;
    reg       proc;
    integer   i;

    // Level-1 results (4 pairs), level-2 results (2 pairs)
    reg signed [WIDTH-1:0] l1_a0, l1_a1, l1_a2, l1_a3;
    reg signed [WIDTH-1:0] l1_d0, l1_d1, l1_d2, l1_d3;
    reg signed [WIDTH-1:0] l2_a0, l2_a1;
    reg signed [WIDTH-1:0] l2_d0, l2_d1;

    assign busy = proc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr    <= 3'd0;
            step      <= ST_IDLE;
            proc      <= 1'b0;
            out_valid <= 1'b0;
            sub_0 <= 0; sub_1 <= 0; sub_2 <= 0; sub_3 <= 0;
            sub_4 <= 0; sub_5 <= 0; sub_6 <= 0; sub_7 <= 0;
            l1_a0 <= 0; l1_a1 <= 0; l1_a2 <= 0; l1_a3 <= 0;
            l1_d0 <= 0; l1_d1 <= 0; l1_d2 <= 0; l1_d3 <= 0;
            l2_a0 <= 0; l2_a1 <= 0;
            l2_d0 <= 0; l2_d1 <= 0;
            for (i = 0; i < 8; i = i + 1)
                buf_r[i] <= {WIDTH{1'b0}};
        end else begin
            out_valid <= 1'b0;

            // Collect samples into the circular buffer
            if (data_valid) begin
                buf_r[wr_ptr] <= data_in;
                wr_ptr        <= wr_ptr + 3'd1;
            end

            case (step)
                ST_IDLE: begin
                    if (start && !proc) begin
                        proc <= 1'b1;
                        step <= ST_L1;
                    end
                end

                // Level 1: four pairs from the 8-sample buffer
                ST_L1: begin
                    l1_d0 <= haar_d(buf_r[0], buf_r[1]);  l1_a0 <= haar_a(buf_r[0], buf_r[1]);
                    l1_d1 <= haar_d(buf_r[2], buf_r[3]);  l1_a1 <= haar_a(buf_r[2], buf_r[3]);
                    l1_d2 <= haar_d(buf_r[4], buf_r[5]);  l1_a2 <= haar_a(buf_r[4], buf_r[5]);
                    l1_d3 <= haar_d(buf_r[6], buf_r[7]);  l1_a3 <= haar_a(buf_r[6], buf_r[7]);
                    step  <= ST_L2;
                end

                // Level 2: two pairs from the four L1 approximations
                ST_L2: begin
                    l2_d0 <= haar_d(l1_a0, l1_a1);        l2_a0 <= haar_a(l1_a0, l1_a1);
                    l2_d1 <= haar_d(l1_a2, l1_a3);        l2_a1 <= haar_a(l1_a2, l1_a3);
                    step  <= ST_L3;
                end

                // Level 3: one pair from the two L2 approximations
                ST_L3: begin
                    sub_0 <= haar_a(l2_a0, l2_a1);
                    sub_1 <= haar_d(l2_a0, l2_a1);
                    sub_2 <= l2_d0;
                    sub_3 <= l2_d1;
                    sub_4 <= l1_d0;
                    sub_5 <= l1_d1;
                    sub_6 <= l1_d2;
                    sub_7 <= l1_d3;
                    step  <= ST_OUT;
                end

                ST_OUT: begin
                    out_valid <= 1'b1;
                    proc      <= 1'b0;
                    step      <= ST_IDLE;
                end

                default: step <= ST_IDLE;
            endcase
        end
    end

endmodule


// ============================================================================
// Absolute-value bank (8 lanes)
// ============================================================================
// Lanes are packed LSB-first: lane n occupies bits [n*WIDTH +: WIDTH].
// Combinational |x| with a registered output; `out_valid` follows `start` by
// one clock.
// ============================================================================
module abs_mag_bank #(
    parameter WIDTH = 12
) (
    input  wire                clk,
    input  wire                rst_n,
    input  wire                start,
    input  wire [8*WIDTH-1:0]  x,
    output reg  [8*WIDTH-1:0]  mag,
    output reg                 out_valid
);

    localparam LANES = 8;

    function [WIDTH-1:0] abs_val(input [WIDTH-1:0] v);
        abs_val = v[WIDTH-1] ? (~v + 1'b1) : v;
    endfunction

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag       <= {(8*WIDTH){1'b0}};
            out_valid <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            if (start) begin
                for (i = 0; i < LANES; i = i + 1)
                    mag[i*WIDTH +: WIDTH] <= abs_val(x[i*WIDTH +: WIDTH]);
                out_valid <= 1'b1;
            end
        end
    end

endmodule


// ============================================================================
// Per-band power, one time-shared multiplier
// ============================================================================
// Squares each of the 8 magnitudes in turn (one lane per clock) and stores the
// result; each pass overwrites the previous one. Results saturate at
// 2^OUT_WIDTH - 1. Lanes are packed LSB-first.
// ============================================================================
module band_power_ts #(
    parameter IN_WIDTH  = 12,
    parameter OUT_WIDTH = 16
) (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   start,
    input  wire [8*IN_WIDTH-1:0]  mag,
    output reg  [8*OUT_WIDTH-1:0] power,
    output reg                    out_valid
);

    reg [2:0] idx;
    reg       running;
    integer   j;

    // Lane select. Loop bounds and offsets are constants, so this elaborates to
    // an 8:1 mux and per-lane write enables. (Variable part-selects on idx, for
    // reads or writes, synthesise to barrel shifters instead; in Yosys that
    // version came out about 1.8x larger.)
    reg [IN_WIDTH-1:0] cur_mag;
    always @(*) begin
        cur_mag = mag[0 +: IN_WIDTH];
        for (j = 1; j < 8; j = j + 1)
            if (idx == j[2:0])
                cur_mag = mag[j*IN_WIDTH +: IN_WIDTH];
    end

    wire [2*IN_WIDTH-1:0] sq_full = cur_mag * cur_mag;

    // Keep the low OUT_WIDTH bits; saturate if the square needs more.
    wire [OUT_WIDTH-1:0] sq_sat = (|sq_full[2*IN_WIDTH-1:OUT_WIDTH])
                                  ? {OUT_WIDTH{1'b1}}
                                  : sq_full[OUT_WIDTH-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            power     <= {(8*OUT_WIDTH){1'b0}};
            out_valid <= 1'b0;
            idx       <= 3'd0;
            running   <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            if (start && !running) begin
                running <= 1'b1;
                idx     <= 3'd0;
            end else if (running) begin
                case (idx)
                    3'd0: power <= {power[8*OUT_WIDTH-1:OUT_WIDTH], sq_sat};
                    3'd1: power <= {power[8*OUT_WIDTH-1:2*OUT_WIDTH], sq_sat,
                                    power[OUT_WIDTH-1:0]};
                    3'd2: power <= {power[8*OUT_WIDTH-1:3*OUT_WIDTH], sq_sat,
                                    power[2*OUT_WIDTH-1:0]};
                    3'd3: power <= {power[8*OUT_WIDTH-1:4*OUT_WIDTH], sq_sat,
                                    power[3*OUT_WIDTH-1:0]};
                    3'd4: power <= {power[8*OUT_WIDTH-1:5*OUT_WIDTH], sq_sat,
                                    power[4*OUT_WIDTH-1:0]};
                    3'd5: power <= {power[8*OUT_WIDTH-1:6*OUT_WIDTH], sq_sat,
                                    power[5*OUT_WIDTH-1:0]};
                    3'd6: power <= {power[8*OUT_WIDTH-1:7*OUT_WIDTH], sq_sat,
                                    power[6*OUT_WIDTH-1:0]};
                    3'd7: power <= {sq_sat, power[7*OUT_WIDTH-1:0]};
                    default: power <= power;
                endcase

                if (idx == 3'd7) begin
                    running   <= 1'b0;
                    out_valid <= 1'b1;
                end else begin
                    idx <= idx + 3'd1;
                end
            end
        end
    end

endmodule


// ============================================================================
// Command encoder (closed-loop frequency controller)
// ============================================================================
// Scans the 8 band powers for the dominant band, compares its index to
// `target_idx`, and emits a command telling the external TMS which way to move.
//
//   cmd   name          meaning
//   ---   -----------   -------------------------------------------
//   000   HOLD          dominant band == target
//   001   INC_1HZ       dominant band is 1-2 bins below target
//   010   DEC_1HZ       dominant band is 1-2 bins above target
//   011   INC_FAST      dominant band is 3+ bins below target
//   100   DEC_FAST      dominant band is 3+ bins above target
//   111   (reserved)    safety stop; not generated by this encoder
//
// Ties go to the lowest band index. `cmd_ready` rises when `cmd_out` is valid
// and drops one clock after `encode_en` falls. `cmd_out` holds its value until
// the next scan completes.
// ============================================================================
module command_encoder #(
    parameter BIN_WIDTH = 16
) (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire [8*BIN_WIDTH-1:0] power_in,    // lane n at [n*BIN_WIDTH +: BIN_WIDTH]
    input  wire [2:0]             target_idx,
    input  wire                   encode_en,
    output reg  [2:0]             cmd_out,
    output reg                    cmd_ready
);

    localparam [2:0] CMD_HOLD     = 3'b000,
                     CMD_INC_1HZ  = 3'b001,
                     CMD_DEC_1HZ  = 3'b010,
                     CMD_INC_FAST = 3'b011,
                     CMD_DEC_FAST = 3'b100;

    reg [BIN_WIDTH-1:0] max_bin;
    reg [2:0]           max_idx;
    reg [3:0]           scan_idx;   // 1..7 = compare lane, 8 = decide
    reg                 scanning;

    wire [BIN_WIDTH-1:0] cur_bin  = power_in[scan_idx[2:0]*BIN_WIDTH +: BIN_WIDTH];
    wire [2:0]           above_by = max_idx - target_idx;   // valid if max_idx > target_idx
    wire [2:0]           below_by = target_idx - max_idx;   // valid if max_idx < target_idx

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_out   <= 3'b000;
            cmd_ready <= 1'b0;
            max_bin   <= {BIN_WIDTH{1'b0}};
            max_idx   <= 3'd0;
            scan_idx  <= 4'd0;
            scanning  <= 1'b0;
        end else begin
            if (!encode_en)
                cmd_ready <= 1'b0;

            if (encode_en && !scanning && !cmd_ready) begin
                // Seed the search with lane 0
                max_bin  <= power_in[BIN_WIDTH-1:0];
                max_idx  <= 3'd0;
                scan_idx <= 4'd1;
                scanning <= 1'b1;
            end else if (scanning) begin
                if (scan_idx == 4'd8) begin
                    // Decide: compare the dominant band with the target
                    if (max_idx == target_idx)
                        cmd_out <= CMD_HOLD;
                    else if (max_idx > target_idx)
                        cmd_out <= (above_by > 3'd2) ? CMD_DEC_FAST : CMD_DEC_1HZ;
                    else
                        cmd_out <= (below_by > 3'd2) ? CMD_INC_FAST : CMD_INC_1HZ;

                    cmd_ready <= 1'b1;
                    scanning  <= 1'b0;
                end else begin
                    if (cur_bin > max_bin) begin
                        max_bin <= cur_bin;
                        max_idx <= scan_idx[2:0];
                    end
                    scan_idx <= scan_idx + 4'd1;
                end
            end
        end
    end

endmodule


// ============================================================================
// LSK modulator -- 14-bit Manchester packet, MSB first
// ============================================================================
// Packet (bit 13 sent first):
//
//   [13:10]  [9:6]  [5:3]  [2]     [1:0]
//   1010     1100   cmd    parity  11
//   preamble sync   3 bits XOR(cmd) postamble
//
// Each bit lasts BIT_PERIOD clocks: the first half carries the bit, the second
// half carries its complement. A full packet is 14 * BIT_PERIOD = 2800 clocks
// at the default. `tx_active` is high for the whole packet and `lsk_ctrl`
// returns low afterwards.
// ============================================================================
module lsk_modulator #(
    parameter BIT_PERIOD = 200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [2:0] cmd_in,
    input  wire       tx_start,     // 1-clk pulse; ignored while transmitting
    output reg        lsk_ctrl,
    output reg        tx_active
);

    localparam [3:0]  LAST_BIT    = 4'd13;
    localparam [10:0] LAST_TICK   = BIT_PERIOD[10:0] - 11'd1;
    localparam [10:0] HALF_PERIOD = BIT_PERIOD[10:0] / 2;

    reg [13:0] tx_shift_reg;
    reg [3:0]  bit_count;
    reg [10:0] bit_timer;
    reg        transmitting;

    wire parity = ^cmd_in;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lsk_ctrl     <= 1'b0;
            tx_active    <= 1'b0;
            transmitting <= 1'b0;
            bit_count    <= 4'd0;
            bit_timer    <= 11'd0;
            tx_shift_reg <= 14'd0;
        end else begin
            if (tx_start && !transmitting) begin
                transmitting <= 1'b1;
                tx_active    <= 1'b1;
                tx_shift_reg <= {4'b1010,    // preamble
                                 4'b1100,    // sync
                                 cmd_in,     // command
                                 parity,     // even-parity bit over cmd
                                 2'b11};     // postamble
                bit_count    <= LAST_BIT;
                bit_timer    <= 11'd0;
            end

            if (transmitting) begin
                if (bit_timer < LAST_TICK) begin
                    bit_timer <= bit_timer + 11'd1;
                    // Manchester: first half = bit, second half = ~bit
                    lsk_ctrl <= (bit_timer < HALF_PERIOD) ?  tx_shift_reg[bit_count]
                                                          : ~tx_shift_reg[bit_count];
                end else begin
                    bit_timer <= 11'd0;
                    if (bit_count == 4'd0) begin
                        transmitting <= 1'b0;
                        tx_active    <= 1'b0;
                        lsk_ctrl     <= 1'b0;
                    end else begin
                        bit_count <= bit_count - 4'd1;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
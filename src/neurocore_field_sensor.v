// ============================================================================
// NeuroCore Field Sensor - Ultra-Low-Power Digital Core  (Expert Refactor v2)
// ============================================================================
// Refactors applied:
//   1. DWT: proper Haar-lifting on 8-sample shift-register buffer
//   2. CORDIC replaced with combinational absolute-value (real-valued subbands)
//   3. Power accumulator: single time-shared multiplier (8-cycle scan)
//   4. Signal integrity: 2-FF synchronizers, sign-extension, 10-bit watchdog
//
// Copyright (c) 2024 Design Team
// SPDX-License-Identifier: Apache-2.0
// ============================================================================

`default_nettype none

// ============================================================================
// Top-Level Module
// ============================================================================
module neurocore_field_sensor #(
    parameter LMS_TAPS      = 8,
    parameter LMS_WIDTH     = 8,
    /* verilator lint_off UNUSEDPARAM */
    parameter DWT_LEVELS    = 3,
    parameter DWT_WIDTH     = 12,
    parameter NUM_BINS      = 8,
    /* verilator lint_on UNUSEDPARAM */
    parameter CMD_WIDTH     = 3,
    parameter ADC_BITS      = 4,
    parameter WDT_BITS      = 10  // Watchdog width (1024 cycles max)
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // ADC Interface
    input  wire [ADC_BITS-1:0]   adc_data,
    input  wire                  adc_valid,

    // Event Detection
    input  wire                  wake,

    // Command Output
    output wire [CMD_WIDTH-1:0]  cmd_out,
    output wire                  cmd_valid,

    // LSK Modulator
    output wire                  lsk_ctrl,
    output wire                  lsk_tx,

    // Power Gating
    output wire                  pwr_gate_ctrl,

    // Status
    output wire                  lms_busy,
    output wire                  dwt_busy,
    output wire                  cordic_busy,
    output wire                  processing
);

    // ========================================================================
    // 4. Two-Flip-Flop Synchronizers for async inputs (wake, adc_valid)
    // ========================================================================
    reg wake_meta, wake_sync;
    reg adc_valid_meta, adc_valid_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wake_meta      <= 0;
            wake_sync      <= 0;
            adc_valid_meta <= 0;
            adc_valid_sync <= 0;
        end else begin
            wake_meta      <= wake;
            wake_sync      <= wake_meta;
            adc_valid_meta <= adc_valid;
            adc_valid_sync <= adc_valid_meta;
        end
    end

    // ========================================================================
    // Internal Signals
    // ========================================================================

    // LMS Filter
    wire [LMS_WIDTH-1:0] lms_out;
    wire                 lms_valid;
    wire                 lms_start;

    // DWT Engine (Haar lifting, 8 subbands)
    wire [DWT_WIDTH-1:0] dwt_out_0, dwt_out_1, dwt_out_2, dwt_out_3;
    wire [DWT_WIDTH-1:0] dwt_out_4, dwt_out_5, dwt_out_6, dwt_out_7;
    wire                 dwt_valid;
    wire                 dwt_start;

    // Absolute-value magnitude (replaces CORDIC)
    wire [DWT_WIDTH-1:0] abs_mag_0, abs_mag_1, abs_mag_2, abs_mag_3;
    wire [DWT_WIDTH-1:0] abs_mag_4, abs_mag_5, abs_mag_6, abs_mag_7;
    wire                 abs_valid;
    wire                 abs_start;

    // Power Accumulator (time-shared single multiplier)
    wire [15:0] power_bins_0, power_bins_1, power_bins_2, power_bins_3;
    wire [15:0] power_bins_4, power_bins_5, power_bins_6, power_bins_7;
    wire        acc_valid;
    wire        acc_start;

    // Command Encoder
    wire [CMD_WIDTH-1:0] cmd_encoded;
    wire                 cmd_ready;

    // ========================================================================
    // Main FSM with Watchdog
    // ========================================================================
    reg  [3:0] state, next_state;
    reg  [WDT_BITS-1:0] wdt_cnt;

    localparam S_IDLE        = 4'd0;
    localparam S_WAKE        = 4'd1;
    localparam S_LMS         = 4'd2;
    localparam S_WAIT_LMS    = 4'd3;
    localparam S_DWT         = 4'd4;
    localparam S_WAIT_DWT    = 4'd5;
    localparam S_ABS         = 4'd6;
    localparam S_WAIT_ABS    = 4'd7;
    localparam S_ACCUM       = 4'd8;
    localparam S_WAIT_ACCUM  = 4'd9;
    localparam S_ENCODE      = 4'd10;
    localparam S_LSK_TX      = 4'd11;
    localparam S_SLEEP       = 4'd12;

    // Watchdog: any WAIT state that exceeds 2^WDT_BITS cycles -> force IDLE
    wire in_wait_state = (state == S_WAIT_LMS)  || (state == S_WAIT_DWT) ||
                         (state == S_WAIT_ABS)   || (state == S_WAIT_ACCUM);
    wire wdt_timeout   = in_wait_state && (&wdt_cnt);  // all-ones

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            wdt_cnt <= {WDT_BITS{1'b0}};
        end else begin
            state <= next_state;
            if (in_wait_state)
                wdt_cnt <= wdt_cnt + {{(WDT_BITS-1){1'b0}}, 1'b1};
            else
                wdt_cnt <= {WDT_BITS{1'b0}};
        end
    end

    always @(*) begin
        next_state = state;
        if (wdt_timeout) begin
            next_state = S_IDLE;
        end else begin
            case (state)
                S_IDLE:       if (wake_sync)  next_state = S_WAKE;
                S_WAKE:                       next_state = S_LMS;
                S_LMS:                        next_state = S_WAIT_LMS;
                S_WAIT_LMS:   if (lms_valid)  next_state = S_DWT;
                S_DWT:                        next_state = S_WAIT_DWT;
                S_WAIT_DWT:   if (dwt_valid)  next_state = S_ABS;
                S_ABS:                        next_state = S_WAIT_ABS;
                S_WAIT_ABS:   if (abs_valid)  next_state = S_ACCUM;
                S_ACCUM:                      next_state = S_WAIT_ACCUM;
                S_WAIT_ACCUM: if (acc_valid)  next_state = S_ENCODE;
                S_ENCODE:     if (cmd_ready)  next_state = S_LSK_TX;
                S_LSK_TX:     if (!lsk_tx)    next_state = S_SLEEP;
                S_SLEEP:                      next_state = S_IDLE;
                default:                      next_state = S_IDLE;
            endcase
        end
    end

    // Control pulses
    assign lms_start    = (state == S_LMS);
    assign dwt_start    = (state == S_DWT);
    assign abs_start    = (state == S_ABS);
    assign acc_start    = (state == S_ACCUM);
    assign pwr_gate_ctrl = (state != S_IDLE) && (state != S_SLEEP);
    assign processing    = lms_busy | dwt_busy;

    // cordic_busy is kept for pin compatibility but always 0 now
    assign cordic_busy = 1'b0;

    // ========================================================================
    // LMS Artifact Filter (8-tap, shift-add, sign-extended input)
    // ========================================================================
    lms_filter #(
        .TAPS  (LMS_TAPS),
        .WIDTH (LMS_WIDTH)
    ) u_lms_filter (
        .clk       (clk),
        .rst_n     (rst_n),
        // 4. Sign-extend ADC data (bipolar signals)
        .data_in   ({{(LMS_WIDTH-ADC_BITS){adc_data[ADC_BITS-1]}}, adc_data}),
        .data_valid(adc_valid_sync),
        .start     (lms_start),
        .data_out  (lms_out),
        .out_valid (lms_valid),
        .busy      (lms_busy)
    );

    // ========================================================================
    // 1. DWT: Haar Lifting on 8-sample buffer (3 levels)
    // ========================================================================
    dwt_haar_lift #(
        .WIDTH (DWT_WIDTH)
    ) u_dwt (
        .clk       (clk),
        .rst_n     (rst_n),
        // Sign-extend LMS output into DWT width
        .data_in   ({{(DWT_WIDTH-LMS_WIDTH){lms_out[LMS_WIDTH-1]}}, lms_out}),
        .data_valid(lms_valid),
        .start     (dwt_start),
        .sub_0     (dwt_out_0),
        .sub_1     (dwt_out_1),
        .sub_2     (dwt_out_2),
        .sub_3     (dwt_out_3),
        .sub_4     (dwt_out_4),
        .sub_5     (dwt_out_5),
        .sub_6     (dwt_out_6),
        .sub_7     (dwt_out_7),
        .out_valid (dwt_valid),
        .busy      (dwt_busy)
    );

    // ========================================================================
    // 2. Absolute-Value Magnitude (replaces CORDIC)
    // ========================================================================
    abs_mag_bank #(
        .WIDTH (DWT_WIDTH)
    ) u_abs (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (abs_start),
        .x_0(dwt_out_0), .x_1(dwt_out_1), .x_2(dwt_out_2), .x_3(dwt_out_3),
        .x_4(dwt_out_4), .x_5(dwt_out_5), .x_6(dwt_out_6), .x_7(dwt_out_7),
        .mag_0(abs_mag_0), .mag_1(abs_mag_1), .mag_2(abs_mag_2), .mag_3(abs_mag_3),
        .mag_4(abs_mag_4), .mag_5(abs_mag_5), .mag_6(abs_mag_6), .mag_7(abs_mag_7),
        .out_valid (abs_valid)
    );

    // ========================================================================
    // 3. Power Accumulator — single time-shared multiplier
    // ========================================================================
    power_accumulator_ts #(
        .IN_WIDTH (DWT_WIDTH),
        .OUT_WIDTH(16)
    ) u_power_acc (
        .clk   (clk),
        .rst_n (rst_n),
        .start (acc_start),
        .mag_0(abs_mag_0), .mag_1(abs_mag_1), .mag_2(abs_mag_2), .mag_3(abs_mag_3),
        .mag_4(abs_mag_4), .mag_5(abs_mag_5), .mag_6(abs_mag_6), .mag_7(abs_mag_7),
        .bin_0(power_bins_0), .bin_1(power_bins_1), .bin_2(power_bins_2), .bin_3(power_bins_3),
        .bin_4(power_bins_4), .bin_5(power_bins_5), .bin_6(power_bins_6), .bin_7(power_bins_7),
        .out_valid(acc_valid)
    );

    // ========================================================================
    // Command Encoder (sequential scan, unchanged)
    // ========================================================================
    command_encoder #(
        .CMD_WIDTH(CMD_WIDTH)
    ) u_cmd_encoder (
        .clk      (clk),
        .rst_n    (rst_n),
        .bin_0(power_bins_0), .bin_1(power_bins_1), .bin_2(power_bins_2), .bin_3(power_bins_3),
        .bin_4(power_bins_4), .bin_5(power_bins_5), .bin_6(power_bins_6), .bin_7(power_bins_7),
        .encode_en(state == S_ENCODE),
        .cmd_out  (cmd_encoded),
        .cmd_ready(cmd_ready)
    );

    // ========================================================================
    // LSK Modulator (unchanged)
    // ========================================================================
    lsk_modulator #(
        .CMD_WIDTH(CMD_WIDTH)
    ) u_lsk_mod (
        .clk      (clk),
        .rst_n    (rst_n),
        .cmd_in   (cmd_encoded),
        .tx_start (state == S_LSK_TX),
        .lsk_ctrl (lsk_ctrl),
        .tx_active(lsk_tx)
    );

    assign cmd_out   = cmd_encoded;
    assign cmd_valid = cmd_ready;

endmodule


// ############################################################################
//  SUB-MODULES
// ############################################################################

// ============================================================================
// LMS Artifact Filter (8-tap, shift-add — unchanged from previous revision)
// ============================================================================
module lms_filter #(
    parameter TAPS  = 8,
    parameter WIDTH = 8
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [WIDTH-1:0]     data_in,
    input  wire                 data_valid,
    input  wire                 start,
    output reg  [WIDTH-1:0]     data_out,
    output reg                  out_valid,
    output wire                 busy
);

    reg [WIDTH-1:0] delay_line [0:TAPS-1];
    reg [2:0] tap_count;
    reg       processing;
    integer   i;

    assign busy = processing;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap_count  <= 0;
            processing <= 0;
            out_valid  <= 0;
            data_out   <= 0;
        end else begin
            out_valid <= 0;

            if (data_valid) begin
                delay_line[0] <= data_in;
                for (i = 1; i < TAPS; i = i + 1)
                    delay_line[i] <= delay_line[i-1];
            end

            if (start && !processing) begin
                processing <= 1;
                tap_count  <= 0;
                data_out   <= 0;
            end

            if (processing) begin
                case (tap_count)
                    3'd0: data_out <= data_out + (delay_line[0] >>> 1);
                    3'd1: data_out <= data_out + (delay_line[1] >>> 2);
                    3'd2: data_out <= data_out + (delay_line[2] >>> 3);
                    3'd3: data_out <= data_out + (delay_line[3] >>> 4);
                    3'd4: data_out <= data_out + (delay_line[4] >>> 4);
                    3'd5: data_out <= data_out + (delay_line[5] >>> 3);
                    3'd6: data_out <= data_out + (delay_line[6] >>> 2);
                    3'd7: data_out <= data_out + (delay_line[7] >>> 1);
                endcase

                if (tap_count == TAPS[2:0] - 3'd1) begin
                    processing <= 0;
                    out_valid  <= 1;
                end else begin
                    tap_count <= tap_count + 3'd1;
                end
            end
        end
    end

endmodule


// ============================================================================
// 1. Haar-Lifting DWT Engine (3 levels on 8-sample shift-register buffer)
// ============================================================================
// Collects 8 samples via data_valid, then on 'start' performs 3-level Haar:
//   Level 1 (4 pairs): d[n] = x[2n+1] - x[2n],  s[n] = x[2n] + (d[n]>>>1)
//   Level 2 (2 pairs of s): repeat
//   Level 3 (1 pair of s):  repeat
// Outputs: sub_0 = final approx, sub_1..sub_3 = details L3-L1,
//          sub_4..sub_7 = level-1 details (all 4)
// Uses a single sequential FSM — no nesting.
// ============================================================================
module dwt_haar_lift #(
    parameter WIDTH = 12
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [WIDTH-1:0]     data_in,
    input  wire                 data_valid,
    input  wire                 start,
    output reg  [WIDTH-1:0]     sub_0,
    output reg  [WIDTH-1:0]     sub_1,
    output reg  [WIDTH-1:0]     sub_2,
    output reg  [WIDTH-1:0]     sub_3,
    output reg  [WIDTH-1:0]     sub_4,
    output reg  [WIDTH-1:0]     sub_5,
    output reg  [WIDTH-1:0]     sub_6,
    output reg  [WIDTH-1:0]     sub_7,
    output reg                  out_valid,
    output wire                 busy
);

    // 8-sample shift register buffer
    reg [WIDTH-1:0] buf_r [0:7];
    reg [2:0]       wr_ptr;

    // Working registers for lifting (4 approx + 4 detail per level)
    reg [WIDTH-1:0] a0, a1, a2, a3;
    reg [WIDTH-1:0] d0, d1, d2, d3;
    // Level-2 intermediates
    reg [WIDTH-1:0] a2_0, a2_1;
    reg [WIDTH-1:0] d2_0, d2_1;

    reg [2:0] step;
    reg       proc;

    localparam ST_IDLE = 3'd0,
               ST_L1   = 3'd1,  // level-1 pairs
               ST_L2   = 3'd2,  // level-2 pairs
               ST_L3   = 3'd3,  // level-3 pair
               ST_OUT  = 3'd4;

    assign busy = proc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr    <= 0;
            step      <= ST_IDLE;
            proc      <= 0;
            out_valid <= 0;
            sub_0 <= 0; sub_1 <= 0; sub_2 <= 0; sub_3 <= 0;
            sub_4 <= 0; sub_5 <= 0; sub_6 <= 0; sub_7 <= 0;
            a0 <= 0; a1 <= 0; a2 <= 0; a3 <= 0;
            d0 <= 0; d1 <= 0; d2 <= 0; d3 <= 0;
            a2_0 <= 0; a2_1 <= 0; d2_0 <= 0; d2_1 <= 0;
        end else begin
            out_valid <= 0;

            // Collect samples into circular buffer
            if (data_valid) begin
                buf_r[wr_ptr] <= data_in;
                wr_ptr <= wr_ptr + 3'd1;
            end

            case (step)
                ST_IDLE: begin
                    if (start && !proc) begin
                        proc <= 1;
                        step <= ST_L1;
                    end
                end

                // Level 1: 4 Haar pairs from 8 samples
                ST_L1: begin
                    d0 <= buf_r[1] - buf_r[0];
                    a0 <= buf_r[0] + ((buf_r[1] - buf_r[0]) >>> 1);
                    d1 <= buf_r[3] - buf_r[2];
                    a1 <= buf_r[2] + ((buf_r[3] - buf_r[2]) >>> 1);
                    d2 <= buf_r[5] - buf_r[4];
                    a2 <= buf_r[4] + ((buf_r[5] - buf_r[4]) >>> 1);
                    d3 <= buf_r[7] - buf_r[6];
                    a3 <= buf_r[6] + ((buf_r[7] - buf_r[6]) >>> 1);
                    step <= ST_L2;
                end

                // Level 2: 2 Haar pairs from 4 approx coefficients
                ST_L2: begin
                    d2_0 <= a1 - a0;
                    a2_0 <= a0 + ((a1 - a0) >>> 1);
                    d2_1 <= a3 - a2;
                    a2_1 <= a2 + ((a3 - a2) >>> 1);
                    step <= ST_L3;
                end

                // Level 3: 1 Haar pair from 2 level-2 approx
                ST_L3: begin
                    // Final outputs:
                    // sub_0 = L3 approx, sub_1 = L3 detail
                    // sub_2 = L2 detail 0, sub_3 = L2 detail 1
                    // sub_4..sub_7 = L1 details d0..d3
                    sub_1 <= a2_1 - a2_0;                          // L3 detail
                    sub_0 <= a2_0 + ((a2_1 - a2_0) >>> 1);        // L3 approx
                    sub_2 <= d2_0;                                  // L2 detail 0
                    sub_3 <= d2_1;                                  // L2 detail 1
                    sub_4 <= d0;                                    // L1 detail 0
                    sub_5 <= d1;                                    // L1 detail 1
                    sub_6 <= d2;                                    // L1 detail 2
                    sub_7 <= d3;                                    // L1 detail 3
                    step  <= ST_OUT;
                end

                ST_OUT: begin
                    out_valid <= 1;
                    proc      <= 0;
                    step      <= ST_IDLE;
                end

                default: step <= ST_IDLE;
            endcase
        end
    end

endmodule


// ============================================================================
// 2. Absolute-Value Magnitude Bank (replaces CORDIC)
// ============================================================================
// For real-valued DWT subbands, |x| is the mathematically correct magnitude.
// Single-cycle combinational with registered output.
// ============================================================================
module abs_mag_bank #(
    parameter WIDTH = 12
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             start,
    input  wire [WIDTH-1:0] x_0, x_1, x_2, x_3,
    input  wire [WIDTH-1:0] x_4, x_5, x_6, x_7,
    output reg  [WIDTH-1:0] mag_0, mag_1, mag_2, mag_3,
    output reg  [WIDTH-1:0] mag_4, mag_5, mag_6, mag_7,
    output reg              out_valid
);

    // Combinational absolute value
    wire [WIDTH-1:0] a0 = x_0[WIDTH-1] ? (~x_0 + {{(WIDTH-1){1'b0}}, 1'b1}) : x_0;
    wire [WIDTH-1:0] a1 = x_1[WIDTH-1] ? (~x_1 + {{(WIDTH-1){1'b0}}, 1'b1}) : x_1;
    wire [WIDTH-1:0] a2 = x_2[WIDTH-1] ? (~x_2 + {{(WIDTH-1){1'b0}}, 1'b1}) : x_2;
    wire [WIDTH-1:0] a3 = x_3[WIDTH-1] ? (~x_3 + {{(WIDTH-1){1'b0}}, 1'b1}) : x_3;
    wire [WIDTH-1:0] a4 = x_4[WIDTH-1] ? (~x_4 + {{(WIDTH-1){1'b0}}, 1'b1}) : x_4;
    wire [WIDTH-1:0] a5 = x_5[WIDTH-1] ? (~x_5 + {{(WIDTH-1){1'b0}}, 1'b1}) : x_5;
    wire [WIDTH-1:0] a6 = x_6[WIDTH-1] ? (~x_6 + {{(WIDTH-1){1'b0}}, 1'b1}) : x_6;
    wire [WIDTH-1:0] a7 = x_7[WIDTH-1] ? (~x_7 + {{(WIDTH-1){1'b0}}, 1'b1}) : x_7;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_0 <= 0; mag_1 <= 0; mag_2 <= 0; mag_3 <= 0;
            mag_4 <= 0; mag_5 <= 0; mag_6 <= 0; mag_7 <= 0;
            out_valid <= 0;
        end else begin
            out_valid <= 0;
            if (start) begin
                mag_0 <= a0; mag_1 <= a1; mag_2 <= a2; mag_3 <= a3;
                mag_4 <= a4; mag_5 <= a5; mag_6 <= a6; mag_7 <= a7;
                out_valid <= 1;
            end
        end
    end

endmodule


// ============================================================================
// 3. Power Accumulator — Single Time-Shared Multiplier
// ============================================================================
// One multiplier cycles through 8 magnitude inputs (scan_idx 0..7).
// Total latency: 9 cycles (1 start + 8 multiply-accumulate).
// Saves ~2000+ gates vs. 8 parallel multipliers.
// ============================================================================
module power_accumulator_ts #(
    parameter IN_WIDTH  = 12,
    parameter OUT_WIDTH = 16
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  start,
    input  wire [IN_WIDTH-1:0]   mag_0, mag_1, mag_2, mag_3,
    input  wire [IN_WIDTH-1:0]   mag_4, mag_5, mag_6, mag_7,
    output reg  [OUT_WIDTH-1:0]  bin_0, bin_1, bin_2, bin_3,
    output reg  [OUT_WIDTH-1:0]  bin_4, bin_5, bin_6, bin_7,
    output reg                   out_valid
);

    reg [3:0] scan_idx;
    reg       running;

    // Mux: select current magnitude based on scan_idx
    reg [IN_WIDTH-1:0] cur_mag;
    always @(*) begin
        case (scan_idx[2:0])
            3'd0: cur_mag = mag_0;
            3'd1: cur_mag = mag_1;
            3'd2: cur_mag = mag_2;
            3'd3: cur_mag = mag_3;
            3'd4: cur_mag = mag_4;
            3'd5: cur_mag = mag_5;
            3'd6: cur_mag = mag_6;
            default: cur_mag = mag_7;
        endcase
    end

    // Single shared multiplier: mag^2, right-shift to fit OUT_WIDTH
    wire [2*IN_WIDTH-1:0] sq_full = cur_mag * cur_mag;
    wire [OUT_WIDTH-1:0]  sq_trunc = sq_full[2*IN_WIDTH-1 : 2*IN_WIDTH-OUT_WIDTH];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bin_0 <= 0; bin_1 <= 0; bin_2 <= 0; bin_3 <= 0;
            bin_4 <= 0; bin_5 <= 0; bin_6 <= 0; bin_7 <= 0;
            out_valid <= 0;
            scan_idx  <= 0;
            running   <= 0;
        end else begin
            out_valid <= 0;

            if (start && !running) begin
                running  <= 1;
                scan_idx <= 0;
            end else if (running) begin
                // Store result into corresponding bin
                case (scan_idx[2:0])
                    3'd0: bin_0 <= sq_trunc;
                    3'd1: bin_1 <= sq_trunc;
                    3'd2: bin_2 <= sq_trunc;
                    3'd3: bin_3 <= sq_trunc;
                    3'd4: bin_4 <= sq_trunc;
                    3'd5: bin_5 <= sq_trunc;
                    3'd6: bin_6 <= sq_trunc;
                    3'd7: bin_7 <= sq_trunc;
                endcase

                if (scan_idx == 4'd7) begin
                    running   <= 0;
                    out_valid <= 1;
                end else begin
                    scan_idx <= scan_idx + 4'd1;
                end
            end
        end
    end

endmodule


// ============================================================================
// Command Encoder (sequential scan — structurally unchanged)
// ============================================================================
module command_encoder #(
    parameter CMD_WIDTH = 3
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [15:0]          bin_0, bin_1, bin_2, bin_3,
    input  wire [15:0]          bin_4, bin_5, bin_6, bin_7,
    input  wire                 encode_en,
    output reg  [CMD_WIDTH-1:0] cmd_out,
    output reg                  cmd_ready
);

    reg [15:0] max_bin;
    reg [2:0]  max_idx;
    reg [3:0]  scan_idx;
    reg        scanning;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_out   <= 0;
            cmd_ready <= 0;
            max_bin   <= 0;
            max_idx   <= 0;
            scan_idx  <= 0;
            scanning  <= 0;
        end else begin
            cmd_ready <= 0;

            if (encode_en && !scanning) begin
                max_bin  <= bin_0;
                max_idx  <= 3'd0;
                scan_idx <= 4'd1;
                scanning <= 1;
            end else if (scanning) begin
                case (scan_idx)
                    4'd1: begin if (bin_1 > max_bin) begin max_bin <= bin_1; max_idx <= 3'd1; end end
                    4'd2: begin if (bin_2 > max_bin) begin max_bin <= bin_2; max_idx <= 3'd2; end end
                    4'd3: begin if (bin_3 > max_bin) begin max_bin <= bin_3; max_idx <= 3'd3; end end
                    4'd4: begin if (bin_4 > max_bin) begin max_bin <= bin_4; max_idx <= 3'd4; end end
                    4'd5: begin if (bin_5 > max_bin) begin max_bin <= bin_5; max_idx <= 3'd5; end end
                    4'd6: begin if (bin_6 > max_bin) begin max_bin <= bin_6; max_idx <= 3'd6; end end
                    4'd7: begin if (bin_7 > max_bin) begin max_bin <= bin_7; max_idx <= 3'd7; end end
                    default: begin end
                endcase

                if (scan_idx == 4'd7) begin
                    cmd_out   <= max_idx[CMD_WIDTH-1:0];
                    cmd_ready <= 1;
                    scanning  <= 0;
                end else begin
                    scan_idx <= scan_idx + 4'd1;
                end
            end
        end
    end

endmodule


// ============================================================================
// LSK Modulator (structurally unchanged)
// ============================================================================
module lsk_modulator #(
    parameter CMD_WIDTH  = 3,
    parameter BIT_PERIOD = 1000
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [CMD_WIDTH-1:0] cmd_in,
    input  wire                 tx_start,
    output reg                  lsk_ctrl,
    output reg                  tx_active
);

    reg [CMD_WIDTH-1:0] tx_shift_reg;
    reg [2:0]           bit_count;
    reg [10:0]          bit_timer;
    reg                 transmitting;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lsk_ctrl     <= 0;
            tx_active    <= 0;
            transmitting <= 0;
            bit_count    <= 0;
            bit_timer    <= 0;
        end else begin
            if (tx_start && !transmitting) begin
                transmitting <= 1;
                tx_active    <= 1;
                tx_shift_reg <= cmd_in;
                bit_count    <= CMD_WIDTH[2:0];
                bit_timer    <= 0;
            end

            if (transmitting) begin
                if (bit_timer < BIT_PERIOD[10:0] - 11'd1) begin
                    bit_timer <= bit_timer + 11'd1;
                    if (bit_timer < BIT_PERIOD[10:0] / 2)
                        lsk_ctrl <= tx_shift_reg[0];
                    else
                        lsk_ctrl <= ~tx_shift_reg[0];
                end else begin
                    bit_timer    <= 0;
                    tx_shift_reg <= {1'b0, tx_shift_reg[CMD_WIDTH-1:1]};

                    if (bit_count == 3'd1) begin
                        transmitting <= 0;
                        tx_active    <= 0;
                        lsk_ctrl     <= 0;
                    end else begin
                        bit_count <= bit_count - 3'd1;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire

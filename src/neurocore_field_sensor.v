// ============================================================================
// NeuroCore Field Sensor - Ultra-Low-Power Digital Core (v2)
// ============================================================================
// Ultra-low-power magnetic field sensing system with closed-loop feedback
//
// Pipeline:
//   1. FIR artifact filter (8-tap, fixed shift-add coefficients)
//   2. 3-level Haar DWT (8-sample window, in-place lifting)
//   3. Absolute-value magnitude extraction
//   4. Power accumulator (single time-shared squarer)
//   5. Command encoder (sequential max-finder, 3-bit output)
//   6. LSK Manchester-encoded packet transmitter
//
// Copyright (c) 2024 Design Team
// SPDX-License-Identifier: Apache-2.0
// ============================================================================

`default_nettype none

// ============================================================================
// Top-Level Module
// ============================================================================
module neurocore_field_sensor #(
    parameter FIR_TAPS      = 8,
    parameter FIR_WIDTH     = 8,
    parameter DWT_LEVELS    = 3,
    parameter DWT_WIDTH     = 12,
    parameter NUM_BINS      = 8,
    parameter CMD_WIDTH     = 3,
    parameter ADC_BITS      = 4,
    parameter MAG_WIDTH     = 12,
    parameter POWER_WIDTH   = 16,
    parameter WATCHDOG_BITS = 16
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [ADC_BITS-1:0]   adc_data,
    input  wire                  adc_valid,
    input  wire                  wake,
    output wire [CMD_WIDTH-1:0]  cmd_out,
    output wire                  cmd_valid,
    output wire                  lsk_ctrl,
    output wire                  lsk_tx,
    output wire                  pwr_gate_ctrl,
    output wire                  fir_busy,
    output wire                  dwt_busy,
    output wire                  acc_busy,
    output wire                  processing
);

    // ========================================================================
    // Synchronizers for async inputs
    // ========================================================================
    reg wake_meta, wake_sync;
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

    // ========================================================================
    // Internal Signals
    // ========================================================================
    wire [FIR_WIDTH-1:0]     fir_out;
    wire                     fir_valid;

    wire [DWT_WIDTH-1:0]     dwt_out_0, dwt_out_1, dwt_out_2, dwt_out_3;
    wire [DWT_WIDTH-1:0]     dwt_out_4, dwt_out_5, dwt_out_6, dwt_out_7;
    wire                     dwt_valid;

    wire [MAG_WIDTH-1:0]     mag_0, mag_1, mag_2, mag_3;
    wire [MAG_WIDTH-1:0]     mag_4, mag_5, mag_6, mag_7;
    wire                     mag_valid;

    wire [POWER_WIDTH-1:0]   power_bins_0, power_bins_1, power_bins_2, power_bins_3;
    wire [POWER_WIDTH-1:0]   power_bins_4, power_bins_5, power_bins_6, power_bins_7;
    wire                     acc_valid;

    wire [CMD_WIDTH-1:0]     cmd_encoded;
    wire                     cmd_ready;

    // ========================================================================
    // Main FSM
    // ========================================================================
    reg  [3:0] state;
    reg  [3:0] next_state;

    localparam S_IDLE        = 4'd0;
    localparam S_WAKE        = 4'd1;
    localparam S_COLLECT     = 4'd2;
    localparam S_WAIT_DWT    = 4'd3;
    localparam S_MAG         = 4'd4;
    localparam S_ACCUM       = 4'd5;
    localparam S_WAIT_ACCUM  = 4'd6;
    localparam S_ENCODE      = 4'd7;
    localparam S_LSK_TX      = 4'd8;
    localparam S_SLEEP       = 4'd9;

    // Control registers
    reg                         fir_start_reg;
    reg                         sample_pending;
    reg                         lsk_start_pulse;
    reg [WATCHDOG_BITS-1:0]     watchdog;

    // State register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // Next-state logic
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (wake_sync)
                    next_state = S_WAKE;
            end
            S_WAKE: begin
                next_state = S_COLLECT;
            end
            S_COLLECT: begin
                if (dwt_valid)
                    next_state = S_MAG;
                else if (watchdog[WATCHDOG_BITS-1])
                    next_state = S_SLEEP;
            end
            S_WAIT_DWT: begin
                if (dwt_valid)
                    next_state = S_MAG;
                else if (watchdog[WATCHDOG_BITS-1])
                    next_state = S_SLEEP;
            end
            S_MAG: begin
                if (mag_valid)
                    next_state = S_ACCUM;
                else if (watchdog[WATCHDOG_BITS-1])
                    next_state = S_SLEEP;
            end
            S_WAIT_ACCUM: begin
                if (acc_valid)
                    next_state = S_ENCODE;
                else if (watchdog[WATCHDOG_BITS-1])
                    next_state = S_SLEEP;
            end
            S_ENCODE: begin
                if (cmd_ready)
                    next_state = S_LSK_TX;
            end
            S_LSK_TX: begin
                if (!lsk_tx)
                    next_state = S_SLEEP;
                else if (watchdog[WATCHDOG_BITS-1])
                    next_state = S_SLEEP;
            end
            S_SLEEP: begin
                next_state = S_IDLE;
            end
            default: next_state = S_IDLE;
        endcase
    end

    // ========================================================================
    // FSM Sequential Control
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            watchdog        <= {WATCHDOG_BITS{1'b0}};
            fir_start_reg   <= 1'b0;
            sample_pending  <= 1'b0;
            lsk_start_pulse <= 1'b0;
        end else begin
            fir_start_reg   <= 1'b0;
            lsk_start_pulse <= 1'b0;

            // Watchdog
            if (state != next_state)
                watchdog <= {WATCHDOG_BITS{1'b0}};
            else begin
                case (state)
                    S_COLLECT, S_WAIT_DWT, S_WAIT_ACCUM, S_LSK_TX, S_ENCODE, S_MAG:
                        watchdog <= watchdog + 1'b1;
                    default:
                        watchdog <= {WATCHDOG_BITS{1'b0}};
                endcase
            end

            // Sample collection during S_COLLECT
            if (state == S_WAKE)
                sample_pending <= 1'b0;

            if (state == S_COLLECT) begin
                if (adc_valid_sync && !sample_pending && !fir_busy) begin
                    fir_start_reg  <= 1'b1;
                    sample_pending <= 1'b1;
                end
                if (fir_valid)
                    sample_pending <= 1'b0;
            end

            // LSK start pulse
            if (state == S_ENCODE && cmd_ready)
                lsk_start_pulse <= 1'b1;
        end
    end

    // Control signal assignments
    wire dwt_start     = (state == S_WAKE);
    wire dwt_data_valid = fir_valid && (state == S_COLLECT);
    wire mag_start     = (state == S_MAG);
    wire acc_start     = (state == S_ACCUM);

    assign pwr_gate_ctrl = (state != S_IDLE) && (state != S_SLEEP);
    assign processing    = (state != S_IDLE) && (state != S_SLEEP);

    // ========================================================================
    // FIR Artifact Filter
    // ========================================================================
    fir_filter #(
        .TAPS(FIR_TAPS),
        .WIDTH(FIR_WIDTH)
    ) u_fir_filter (
        .clk(clk),
        .rst_n(rst_n),
        .data_in({{(FIR_WIDTH-ADC_BITS){1'b0}}, adc_data}),
        .start(fir_start_reg),
        .data_out(fir_out),
        .out_valid(fir_valid),
        .busy(fir_busy)
    );

    // ========================================================================
    // DWT Engine
    // ========================================================================
    dwt_engine #(
        .LEVELS(DWT_LEVELS),
        .WIDTH(DWT_WIDTH)
    ) u_dwt_engine (
        .clk(clk),
        .rst_n(rst_n),
        .data_in({{(DWT_WIDTH-FIR_WIDTH){fir_out[FIR_WIDTH-1]}}, fir_out}),
        .data_valid(dwt_data_valid),
        .start(dwt_start),
        .subband_0(dwt_out_0),
        .subband_1(dwt_out_1),
        .subband_2(dwt_out_2),
        .subband_3(dwt_out_3),
        .subband_4(dwt_out_4),
        .subband_5(dwt_out_5),
        .subband_6(dwt_out_6),
        .subband_7(dwt_out_7),
        .out_valid(dwt_valid),
        .busy(dwt_busy)
    );

    // ========================================================================
    // Magnitude Extraction
    // ========================================================================
    magnitude_extract #(
        .WIDTH(MAG_WIDTH)
    ) u_mag_extract (
        .clk(clk),
        .rst_n(rst_n),
        .start(mag_start),
        .in_0(dwt_out_0),
        .in_1(dwt_out_1),
        .in_2(dwt_out_2),
        .in_3(dwt_out_3),
        .in_4(dwt_out_4),
        .in_5(dwt_out_5),
        .in_6(dwt_out_6),
        .in_7(dwt_out_7),
        .mag_0(mag_0),
        .mag_1(mag_1),
        .mag_2(mag_2),
        .mag_3(mag_3),
        .mag_4(mag_4),
        .mag_5(mag_5),
        .mag_6(mag_6),
        .mag_7(mag_7),
        .out_valid(mag_valid)
    );

    // ========================================================================
    // Power Accumulator
    // ========================================================================
    power_accumulator #(
        .IN_WIDTH(MAG_WIDTH),
        .OUT_WIDTH(POWER_WIDTH),
        .NUM_BINS(NUM_BINS)
    ) u_power_acc (
        .clk(clk),
        .rst_n(rst_n),
        .start(acc_start),
        .mag_in_0(mag_0),
        .mag_in_1(mag_1),
        .mag_in_2(mag_2),
        .mag_in_3(mag_3),
        .mag_in_4(mag_4),
        .mag_in_5(mag_5),
        .mag_in_6(mag_6),
        .mag_in_7(mag_7),
        .bin_0(power_bins_0),
        .bin_1(power_bins_1),
        .bin_2(power_bins_2),
        .bin_3(power_bins_3),
        .bin_4(power_bins_4),
        .bin_5(power_bins_5),
        .bin_6(power_bins_6),
        .bin_7(power_bins_7),
        .out_valid(acc_valid),
        .busy(acc_busy)
    );

    // ========================================================================
    // Command Encoder
    // ========================================================================
    command_encoder #(
        .NUM_BINS(NUM_BINS),
        .CMD_WIDTH(CMD_WIDTH),
        .BIN_WIDTH(POWER_WIDTH)
    ) u_cmd_encoder (
        .clk(clk),
        .rst_n(rst_n),
        .bin_0(power_bins_0),
        .bin_1(power_bins_1),
        .bin_2(power_bins_2),
        .bin_3(power_bins_3),
        .bin_4(power_bins_4),
        .bin_5(power_bins_5),
        .bin_6(power_bins_6),
        .bin_7(power_bins_7),
        .encode_en(state == S_ENCODE),
        .cmd_out(cmd_encoded),
        .cmd_ready(cmd_ready)
    );

    // ========================================================================
    // LSK Modulator
    // ========================================================================
    lsk_modulator #(
        .CMD_WIDTH(CMD_WIDTH)
    ) u_lsk_mod (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_in(cmd_encoded),
        .tx_start(lsk_start_pulse),
        .lsk_ctrl(lsk_ctrl),
        .tx_active(lsk_tx)
    );

    assign cmd_out   = cmd_encoded;
    assign cmd_valid = cmd_ready;

endmodule


// ============================================================================
// FIR Artifact Filter (8-tap, fixed shift-add coefficients)
// ============================================================================
// Coefficients (symmetric, sum to 1.0):
//   [0.25, 0.125, 0.0625, 0.0625, 0.0625, 0.0625, 0.125, 0.25]
//   Implemented as right-shifts: [2, 3, 4, 4, 4, 4, 3, 2]
// ============================================================================
module fir_filter #(
    parameter TAPS  = 8,
    parameter WIDTH = 8
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [WIDTH-1:0]     data_in,
    input  wire                 start,
    output reg  [WIDTH-1:0]     data_out,
    output reg                  out_valid,
    output wire                 busy
);

    reg [WIDTH-1:0] delay_line [0:TAPS-1];
    reg [2:0]       tap_count;
    reg             processing;

    // Accumulator with extra bits to prevent overflow
    // Max accumulation: 8 terms of WIDTH-bit values shifted right by at least 2
    // Worst case sum ~ 1.0 * max_value, so WIDTH+1 bits is sufficient
    reg signed [WIDTH+2:0] accum;

    // Sign-extended and shifted tap value (computed combinationally for clarity)
    reg signed [WIDTH+2:0] tap_contribution;

    integer i;

    assign busy = processing;

    always @(*) begin
        // Default
        tap_contribution = {(WIDTH+3){1'b0}};
        case (tap_count)
            3'd0: tap_contribution = {{3{delay_line[0][WIDTH-1]}}, delay_line[0]} >>> 2;
            3'd1: tap_contribution = {{3{delay_line[1][WIDTH-1]}}, delay_line[1]} >>> 3;
            3'd2: tap_contribution = {{3{delay_line[2][WIDTH-1]}}, delay_line[2]} >>> 4;
            3'd3: tap_contribution = {{3{delay_line[3][WIDTH-1]}}, delay_line[3]} >>> 4;
            3'd4: tap_contribution = {{3{delay_line[4][WIDTH-1]}}, delay_line[4]} >>> 4;
            3'd5: tap_contribution = {{3{delay_line[5][WIDTH-1]}}, delay_line[5]} >>> 4;
            3'd6: tap_contribution = {{3{delay_line[6][WIDTH-1]}}, delay_line[6]} >>> 3;
            3'd7: tap_contribution = {{3{delay_line[7][WIDTH-1]}}, delay_line[7]} >>> 2;
            default: tap_contribution = {(WIDTH+3){1'b0}};
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap_count  <= 3'd0;
            processing <= 1'b0;
            out_valid  <= 1'b0;
            data_out   <= {WIDTH{1'b0}};
            accum      <= {(WIDTH+3){1'b0}};
            for (i = 0; i < TAPS; i = i + 1)
                delay_line[i] <= {WIDTH{1'b0}};
        end else begin
            out_valid <= 1'b0;

            if (start && !processing) begin
                // Shift in new data
                delay_line[0] <= data_in;
                for (i = 1; i < TAPS; i = i + 1)
                    delay_line[i] <= delay_line[i-1];

                processing <= 1'b1;
                tap_count  <= 3'd0;
                accum      <= {(WIDTH+3){1'b0}};
            end

            if (processing) begin
                accum <= accum + tap_contribution;

                if (tap_count == 3'd7) begin
                    processing <= 1'b0;
                    out_valid  <= 1'b1;
                    data_out   <= accum[WIDTH-1:0] + tap_contribution[WIDTH-1:0];
                end else begin
                    tap_count <= tap_count + 3'd1;
                end
            end
        end
    end

endmodule


// ============================================================================
// Lifting-Scheme DWT Engine (3-level Haar, 8-sample window)
// ============================================================================
// Collects 8 input samples, then performs 3-level in-place Haar lifting.
//
// Haar lifting per level (operating on pairs):
//   detail  = even_sample - odd_sample
//   approx  = (even_sample + odd_sample) >>> 1
//
// Output mapping:
//   subband_0 = cA3       (DC / lowest frequency)
//   subband_1 = cD3       (lowest detail)
//   subband_2 = cD2[0]    (mid detail)
//   subband_3 = cD2[1]    (mid detail)
//   subband_4 = cD1[0]    (high detail)
//   subband_5 = cD1[1]    (high detail)
//   subband_6 = cD1[2]    (high detail)
//   subband_7 = cD1[3]    (high detail)
// ============================================================================
module dwt_engine #(
    parameter LEVELS = 3,
    parameter WIDTH  = 12
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire signed [WIDTH-1:0] data_in,
    input  wire                 data_valid,
    input  wire                 start,
    output reg  signed [WIDTH-1:0] subband_0,
    output reg  signed [WIDTH-1:0] subband_1,
    output reg  signed [WIDTH-1:0] subband_2,
    output reg  signed [WIDTH-1:0] subband_3,
    output reg  signed [WIDTH-1:0] subband_4,
    output reg  signed [WIDTH-1:0] subband_5,
    output reg  signed [WIDTH-1:0] subband_6,
    output reg  signed [WIDTH-1:0] subband_7,
    output reg                  out_valid,
    output wire                 busy
);

    localparam NUM_SAMPLES = 8;

    // Working registers
    reg signed [WIDTH-1:0] w [0:NUM_SAMPLES-1];

    // Snapshot registers to avoid read-before-write hazards
    reg signed [WIDTH-1:0] s [0:NUM_SAMPLES-1];

    reg [2:0] sample_cnt;
    reg [2:0] proc_level;
    reg       collecting;
    reg       proc_active;

    integer i;

    assign busy = collecting | proc_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            collecting  <= 1'b0;
            proc_active <= 1'b0;
            out_valid   <= 1'b0;
            sample_cnt  <= 3'd0;
            proc_level  <= 2'd0;
            subband_0   <= {WIDTH{1'b0}};
            subband_1   <= {WIDTH{1'b0}};
            subband_2   <= {WIDTH{1'b0}};
            subband_3   <= {WIDTH{1'b0}};
            subband_4   <= {WIDTH{1'b0}};
            subband_5   <= {WIDTH{1'b0}};
            subband_6   <= {WIDTH{1'b0}};
            subband_7   <= {WIDTH{1'b0}};
            for (i = 0; i < NUM_SAMPLES; i = i + 1) begin
                w[i] <= {WIDTH{1'b0}};
                s[i] <= {WIDTH{1'b0}};
            end
        end else begin
            out_valid <= 1'b0;

            // ---- Phase 1: Arm collection ----
            if (start && !collecting && !proc_active) begin
                collecting <= 1'b1;
                sample_cnt <= 3'd0;
            end

            // ---- Phase 2: Collect 8 samples ----
            if (collecting && data_valid) begin
                w[sample_cnt] <= data_in;
                if (sample_cnt == 3'd7) begin
                    collecting  <= 1'b0;
                    proc_active <= 1'b1;
                    proc_level  <= 2'd0;
                end else begin
                    sample_cnt <= sample_cnt + 3'd1;
                end
            end

            // ---- Phase 3: Haar lifting (snapshot-based) ----
            if (proc_active) begin
                case (proc_level)
                    2'd0: begin
                        // Snapshot current values
                        for (i = 0; i < NUM_SAMPLES; i = i + 1)
                            s[i] <= w[i];
                        proc_level <= 2'd1;
                    end
                    2'd1: begin
                        // Level 1: 8 -> 4 approx + 4 detail
                        // Uses snapshot s[] so no hazards
                        w[0] <= (s[0] + s[1]) >>> 1;  // approx
                        w[1] <= (s[2] + s[3]) >>> 1;
                        w[2] <= (s[4] + s[5]) >>> 1;
                        w[3] <= (s[6] + s[7]) >>> 1;
                        w[4] <= s[0] - s[1];           // detail (cD1)
                        w[5] <= s[2] - s[3];
                        w[6] <= s[4] - s[5];
                        w[7] <= s[6] - s[7];
                        proc_level <= 2'd2;
                    end
                    2'd2: begin
                        // Level 2: 4 approx -> 2 approx + 2 detail
                        // w[0..3] hold level-1 approximations
                        // Snapshot not needed: we only read w[0..3], write w[0..3]
                        // But use s[] for safety
                        s[0] <= w[0];
                        s[1] <= w[1];
                        s[2] <= w[2];
                        s[3] <= w[3];
                        proc_level <= 2'd3;
                    end
                    2'd3: begin
                        // Apply level 2 from snapshot
                        w[0] <= (s[0] + s[1]) >>> 1;
                        w[1] <= (s[2] + s[3]) >>> 1;
                        w[2] <= s[0] - s[1];           // cD2[0]
                        w[3] <= s[2] - s[3];           // cD2[1]
                        proc_level <= 2'd4;
                    end
                    2'd4: begin
                        // Level 3: 2 approx -> 1 approx + 1 detail
                        // Snapshot w[0], w[1]
                        s[0] <= w[0];
                        s[1] <= w[1];
                        proc_level <= 2'd5;
                    end
                    2'd5: begin
                        // Apply level 3 from snapshot
                        w[0] <= (s[0] + s[1]) >>> 1;   // cA3
                        w[1] <= s[0] - s[1];            // cD3
                        proc_level <= 2'd6;
                    end
                    default: begin
                        // 2'd6 and above: Output results
                        subband_0 <= w[0];  // cA3
                        subband_1 <= w[1];  // cD3
                        subband_2 <= w[2];  // cD2[0]
                        subband_3 <= w[3];  // cD2[1]
                        subband_4 <= w[4];  // cD1[0]
                        subband_5 <= w[5];  // cD1[1]
                        subband_6 <= w[6];  // cD1[2]
                        subband_7 <= w[7];  // cD1[3]
                        proc_active <= 1'b0;
                        out_valid   <= 1'b1;
                    end
                endcase
            end
        end
    end

endmodule


// ============================================================================
// Magnitude Extraction (Absolute Value)
// ============================================================================
// Single-cycle registered absolute value. Replaces CORDIC vectoring since
// Haar DWT coefficients are real-valued (not complex I/Q pairs).
// ============================================================================
module magnitude_extract #(
    parameter WIDTH = 12
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    input  wire signed [WIDTH-1:0] in_0, in_1, in_2, in_3,
    input  wire signed [WIDTH-1:0] in_4, in_5, in_6, in_7,
    output reg  [WIDTH-1:0]        mag_0, mag_1, mag_2, mag_3,
    output reg  [WIDTH-1:0]        mag_4, mag_5, mag_6, mag_7,
    output reg                     out_valid
);

    function [WIDTH-1:0] abs_val;
        input signed [WIDTH-1:0] val;
        abs_val = val[WIDTH-1] ? (~val + {{(WIDTH-1){1'b0}}, 1'b1}) : val;
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            mag_0 <= {WIDTH{1'b0}};
            mag_1 <= {WIDTH{1'b0}};
            mag_2 <= {WIDTH{1'b0}};
            mag_3 <= {WIDTH{1'b0}};
            mag_4 <= {WIDTH{1'b0}};
            mag_5 <= {WIDTH{1'b0}};
            mag_6 <= {WIDTH{1'b0}};
            mag_7 <= {WIDTH{1'b0}};
        end else begin
            out_valid <= 1'b0;
            if (start) begin
                mag_0 <= abs_val(in_0);
                mag_1 <= abs_val(in_1);
                mag_2 <= abs_val(in_2);
                mag_3 <= abs_val(in_3);
                mag_4 <= abs_val(in_4);
                mag_5 <= abs_val(in_5);
                mag_6 <= abs_val(in_6);
                mag_7 <= abs_val(in_7);
                out_valid <= 1'b1;
            end
        end
    end

endmodule


// ============================================================================
// Power Bin Accumulator (Single Time-Shared Squarer)
// ============================================================================
// Iterates one multiplier over 8 channels. Takes 8 cycles.
// Saves ~7 multipliers (~2000-3500 gates on 135nm).
// ============================================================================
module power_accumulator #(
    parameter IN_WIDTH  = 12,
    parameter OUT_WIDTH = 16,
    parameter NUM_BINS  = 8
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  start,
    input  wire [IN_WIDTH-1:0]   mag_in_0, mag_in_1, mag_in_2, mag_in_3,
    input  wire [IN_WIDTH-1:0]   mag_in_4, mag_in_5, mag_in_6, mag_in_7,
    output reg  [OUT_WIDTH-1:0]  bin_0, bin_1, bin_2, bin_3,
    output reg  [OUT_WIDTH-1:0]  bin_4, bin_5, bin_6, bin_7,
    output reg                   out_valid,
    output wire                  busy
);

    reg [2:0]          bin_idx;
    reg                processing;
    reg [IN_WIDTH-1:0] current_mag;

    wire [2*IN_WIDTH-1:0] squared = current_mag * current_mag;

    // Truncation: take upper OUT_WIDTH bits if result is wider
    wire [OUT_WIDTH-1:0] squared_out;
    generate
        if (2*IN_WIDTH > OUT_WIDTH)
            assign squared_out = squared[2*IN_WIDTH-1 -: OUT_WIDTH];
        else
            assign squared_out = {{(OUT_WIDTH-2*IN_WIDTH){1'b0}}, squared};
    endgenerate

    assign busy = processing;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bin_0       <= {OUT_WIDTH{1'b0}};
            bin_1       <= {OUT_WIDTH{1'b0}};
            bin_2       <= {OUT_WIDTH{1'b0}};
            bin_3       <= {OUT_WIDTH{1'b0}};
            bin_4       <= {OUT_WIDTH{1'b0}};
            bin_5       <= {OUT_WIDTH{1'b0}};
            bin_6       <= {OUT_WIDTH{1'b0}};
            bin_7       <= {OUT_WIDTH{1'b0}};
            out_valid   <= 1'b0;
            processing  <= 1'b0;
            bin_idx     <= 3'd0;
            current_mag <= {IN_WIDTH{1'b0}};
        end else begin
            out_valid <= 1'b0;

            if (start && !processing) begin
                processing  <= 1'b1;
                bin_idx     <= 3'd0;
                current_mag <= mag_in_0;
            end else if (processing) begin
                // Store squared result
                case (bin_idx)
                    3'd0: bin_0 <= squared_out;
                    3'd1: bin_1 <= squared_out;
                    3'd2: bin_2 <= squared_out;
                    3'd3: bin_3 <= squared_out;
                    3'd4: bin_4 <= squared_out;
                    3'd5: bin_5 <= squared_out;
                    3'd6: bin_6 <= squared_out;
                    3'd7: bin_7 <= squared_out;
                endcase

                if (bin_idx == 3'd7) begin
                    processing <= 1'b0;
                    out_valid  <= 1'b1;
                end else begin
                    bin_idx <= bin_idx + 3'd1;
                    case (bin_idx + 3'd1)
                        3'd1: current_mag <= mag_in_1;
                        3'd2: current_mag <= mag_in_2;
                        3'd3: current_mag <= mag_in_3;
                        3'd4: current_mag <= mag_in_4;
                        3'd5: current_mag <= mag_in_5;
                        3'd6: current_mag <= mag_in_6;
                        3'd7: current_mag <= mag_in_7;
                        default: current_mag <= {IN_WIDTH{1'b0}};
                    endcase
                end
            end
        end
    end

endmodule


// ============================================================================
// Command Encoder (Sequential Max-Finder)
// ============================================================================
// 9 cycles: 8 compare + 1 latch
// ============================================================================
module command_encoder #(
    parameter NUM_BINS  = 8,
    parameter CMD_WIDTH = 3,
    parameter BIN_WIDTH = 16
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [BIN_WIDTH-1:0]  bin_0, bin_1, bin_2, bin_3,
    input  wire [BIN_WIDTH-1:0]  bin_4, bin_5, bin_6, bin_7,
    input  wire                  encode_en,
    output reg  [CMD_WIDTH-1:0]  cmd_out,
    output reg                   cmd_ready
);

    reg [BIN_WIDTH-1:0] max_bin;
    reg [CMD_WIDTH-1:0] max_idx;
    reg [3:0]           scan_idx;
    reg                 scanning;

    // Mux to select current bin
    reg [BIN_WIDTH-1:0] current_bin;
    always @(*) begin
        case (scan_idx)
            4'd1:    current_bin = bin_1;
            4'd2:    current_bin = bin_2;
            4'd3:    current_bin = bin_3;
            4'd4:    current_bin = bin_4;
            4'd5:    current_bin = bin_5;
            4'd6:    current_bin = bin_6;
            4'd7:    current_bin = bin_7;
            default: current_bin = {BIN_WIDTH{1'b0}};
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_out   <= {CMD_WIDTH{1'b0}};
            cmd_ready <= 1'b0;
            max_bin   <= {BIN_WIDTH{1'b0}};
            max_idx   <= {CMD_WIDTH{1'b0}};
            scan_idx  <= 4'd0;
            scanning  <= 1'b0;
        end else begin
            cmd_ready <= 1'b0;

            if (encode_en && !scanning) begin
                max_bin  <= bin_0;
                max_idx  <= {CMD_WIDTH{1'b0}};
                scan_idx <= 4'd1;
                scanning <= 1'b1;
            end else if (scanning) begin
                if (scan_idx <= 4'd7) begin
                    if (current_bin > max_bin) begin
                        max_bin <= current_bin;
                        max_idx <= scan_idx[CMD_WIDTH-1:0];
                    end
                    scan_idx <= scan_idx + 4'd1;
                end else begin
                    // scan_idx == 8: latch final result
                    cmd_out   <= max_idx;
                    cmd_ready <= 1'b1;
                    scanning  <= 1'b0;
                end
            end
        end
    end

endmodule


// ============================================================================
// LSK Modulator (Manchester-Encoded 14-bit Packet)
// ============================================================================
// Packet: [1010][1100][cmd2 cmd1 cmd0][parity][11]
// Manchester: bit=1 -> HIGH then LOW; bit=0 -> LOW then HIGH
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

    localparam PACKET_LEN  = 14;
    localparam HALF_PERIOD = BIT_PERIOD / 2;
    localparam TIMER_W     = $clog2(BIT_PERIOD);

    reg [PACKET_LEN-1:0] packet_reg;
    reg [3:0]            bit_count;
    reg [TIMER_W-1:0]    bit_timer;
    reg                  transmitting;

    wire parity = cmd_in[2] ^ cmd_in[1] ^ cmd_in[0];

    wire [PACKET_LEN-1:0] full_packet = {
        4'b1010,
        4'b1100,
        cmd_in,
        parity,
        2'b11
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lsk_ctrl     <= 1'b0;
            tx_active    <= 1'b0;
            transmitting <= 1'b0;
            packet_reg   <= {PACKET_LEN{1'b0}};
            bit_count    <= 4'd0;
            bit_timer    <= {TIMER_W{1'b0}};
        end else begin

            if (tx_start && !transmitting) begin
                transmitting <= 1'b1;
                tx_active    <= 1'b1;
                packet_reg   <= full_packet;
                bit_count    <= PACKET_LEN[3:0] - 4'd1;
                bit_timer    <= {TIMER_W{1'b0}};
                lsk_ctrl     <= full_packet[PACKET_LEN-1];
            end

            if (transmitting) begin
                bit_timer <= bit_timer + {{(TIMER_W-1){1'b0}}, 1'b1};

                // Manchester encoding
                if (bit_timer < HALF_PERIOD[TIMER_W-1:0])
                    lsk_ctrl <= packet_reg[bit_count];
                else
                    lsk_ctrl <= ~packet_reg[bit_count];

                // End of bit period
                if (bit_timer == BIT_PERIOD[TIMER_W-1:0] - {{(TIMER_W-1){1'b0}}, 1'b1}) begin
                    bit_timer <= {TIMER_W{1'b0}};
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
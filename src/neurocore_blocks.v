/*
 * NeuroCore Field Sensor – Complete Block Diagram Implementation
 * Copyright (c) 2024 Design Team
 * SPDX-License-Identifier: Apache-2.0
 *
 * Block Diagram:
 *   Analog Front-End  →  Event Detection  →  Digital Core (power-gated)  →  Output
 *       ↑                                                                      |
 *       └──────────────── Power Subsystem ←────────────────────────────────────┘
 *
 * Spiral Inductor → Passive Integrator → Level-Crossing ADC (4-bit)
 * Coarse Threshold Comparator (~1nW always-on) → WAKE pulse
 * LMS Filter → DWT Engine → CORDIC → Power Bins → 3-bit Cmd → LSK Modulator
 * Energy Harvesting → Rectifier → Storage Cap → Dual-Rail Regulator
 */

`default_nettype none

// ============================================================================
// Module 1: Level-Crossing ADC (4-bit, inverter-based, event-driven)
// ============================================================================
// Models the dynamic inverter-based level-crossing ADC.
// Non-uniform continuous-time sampling: output valid only on threshold crossing.
// Active power ~4nW during crossing, ~0nW idle.
// ============================================================================
module level_crossing_adc #(
    parameter ADC_BITS = 4
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  adc_en,        // ADC enable from control register
    input  wire [ADC_BITS-1:0]   analog_in,     // Digitised proxy for analog signal
    output reg  [ADC_BITS-1:0]   adc_data,      // 4-bit level code
    output reg                   adc_valid,     // Pulse high on each crossing
    output wire                  adc_active     // 1 while conversion in progress
);

    reg [ADC_BITS-1:0] prev_sample;
    wire crossing_detected;

    // A crossing is detected when the current sample differs from the previous
    assign crossing_detected = adc_en && (analog_in != prev_sample);
    assign adc_active        = adc_valid;  // Active only during valid output cycle

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_sample <= {ADC_BITS{1'b0}};
            adc_data    <= {ADC_BITS{1'b0}};
            adc_valid   <= 1'b0;
        end else if (adc_en) begin
            prev_sample <= analog_in;
            if (crossing_detected) begin
                adc_data  <= analog_in;
                adc_valid <= 1'b1;
            end else begin
                adc_valid <= 1'b0;
            end
        end else begin
            adc_valid <= 1'b0;
        end
    end

endmodule


// ============================================================================
// Module 2: Event Detection Comparator (~1nW always-on) + WAKE Generator
// ============================================================================
// Coarse threshold comparator. Always-on at ~1nW.
// Generates a WAKE pulse to bring the digital core out of power-gated sleep.
// ============================================================================
module event_detector #(
    parameter ADC_BITS   = 4,
    parameter THRESH_W   = 8
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                comp_en,          // Comparator enable (CTRL[1])
    input  wire [ADC_BITS-1:0] analog_in,        // Same integrated signal as ADC input
    input  wire [THRESH_W-1:0] threshold,        // Programmable threshold (THRESH reg)
    output reg                 wake,             // WAKE pulse to digital core
    output reg                 event_detected    // Sticky status bit
);

    // Compare upper bits of threshold against the 4-bit input (scaled)
    wire exceeds = comp_en && ({analog_in, 4'b0} >= threshold);

    reg prev_exceeds;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wake           <= 1'b0;
            event_detected <= 1'b0;
            prev_exceeds   <= 1'b0;
        end else begin
            prev_exceeds <= exceeds;
            // Rising-edge detect on threshold crossing → WAKE pulse
            if (exceeds && !prev_exceeds) begin
                wake           <= 1'b1;
                event_detected <= 1'b1;
            end else begin
                wake <= 1'b0;
            end
        end
    end

endmodule


// ============================================================================
// Module 3: Power Gate Controller
// ============================================================================
// Manages power gating of the digital core.
// Asserts pwr_gate_ctrl when WAKE received; de-asserts after processing done.
// ============================================================================
module power_gate_ctrl (
    input  wire clk,
    input  wire rst_n,
    input  wire wake,              // From event detector
    input  wire force_wake,        // CTRL[3] — test override
    input  wire processing_done,   // Digital pipeline finished
    output reg  pwr_gate_active,   // 1 = digital core powered
    output reg  clk_en             // Gated clock enable
);

    // Small counter for power-up sequencing (~wake latency)
    reg [2:0] seq_cnt;

    localparam IDLE    = 2'd0,
               RAMP_UP = 2'd1,
               ACTIVE  = 2'd2,
               SHUT    = 2'd3;
    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= IDLE;
            pwr_gate_active <= 1'b0;
            clk_en          <= 1'b0;
            seq_cnt         <= 3'd0;
        end else begin
            case (state)
                IDLE: begin
                    pwr_gate_active <= 1'b0;
                    clk_en          <= 1'b0;
                    if (wake || force_wake) begin
                        state   <= RAMP_UP;
                        seq_cnt <= 3'd0;
                    end
                end
                RAMP_UP: begin
                    pwr_gate_active <= 1'b1;  // Rail begins charging
                    seq_cnt <= seq_cnt + 3'd1;
                    if (seq_cnt == 3'd4) begin // ~4 cycles settle time
                        clk_en <= 1'b1;
                        state  <= ACTIVE;
                    end
                end
                ACTIVE: begin
                    if (processing_done && !force_wake) begin
                        state <= SHUT;
                    end
                end
                SHUT: begin
                    clk_en          <= 1'b0;
                    pwr_gate_active <= 1'b0;
                    state           <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end

endmodule


// ============================================================================
// Module 4: LMS Artifact Filter (shift-add, 8 taps, no multipliers)
// ============================================================================
// Adaptive FIR filter using Least-Mean-Squares algorithm.
// All coefficient multiplications replaced by shift-add operations.
// ============================================================================
module lms_filter #(
    parameter TAPS     = 8,
    parameter WIDTH    = 8,
    parameter MU_SHIFT = 4   // Step-size µ = 2^{-MU_SHIFT}
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    en,
    input  wire                    data_valid,
    input  wire signed [WIDTH-1:0] data_in,      // From ADC (sign-extended)
    input  wire signed [WIDTH-1:0] ref_in,        // Reference / desired
    output reg  signed [WIDTH-1:0] data_out,
    output reg                     done
);

    // Tap delay line
    reg signed [WIDTH-1:0] tap_dl [0:TAPS-1];
    // Coefficients (adaptive)
    reg signed [WIDTH-1:0] coeff  [0:TAPS-1];

    integer i;

    // Shift-add multiply: coeff * tap  ≈  sign-preserving shift approximation
    // For ultra-low-power we approximate: coeff*x ≈ (x >>> (leading-zero count))
    // Here we use fixed-point shift-add: result = (coeff_positive ? x : -x) >>> |coeff| clamp
    // Simplified: accumulate using direct add of sign-selected shifts
    reg signed [WIDTH+3:0] acc;       // Extra bits to prevent overflow (TAPS=8 → +3)
    reg signed [WIDTH-1:0] error;

    reg [3:0] step;

    localparam S_IDLE   = 4'd0,
               S_SHIFT  = 4'd1,
               S_MAC    = 4'd2,
               S_OUTPUT = 4'd3,
               S_ADAPT  = 4'd4,
               S_DONE   = 4'd5;

    reg [3:0] tap_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            step     <= S_IDLE;
            data_out <= {WIDTH{1'b0}};
            done     <= 1'b0;
            acc      <= 0;
            error    <= 0;
            tap_idx  <= 0;
            for (i = 0; i < TAPS; i = i + 1) begin
                tap_dl[i] <= 0;
                coeff[i]  <= 0;
            end
        end else if (en) begin
            case (step)
                S_IDLE: begin
                    done <= 1'b0;
                    if (data_valid) begin
                        // Shift delay line
                        for (i = TAPS-1; i > 0; i = i - 1)
                            tap_dl[i] <= tap_dl[i-1];
                        tap_dl[0] <= data_in;
                        acc      <= 0;
                        tap_idx  <= 0;
                        step     <= S_MAC;
                    end
                end
                S_MAC: begin
                    // Shift-add multiply-accumulate: coeff[i] * tap_dl[i]
                    // Approximate: add arithmetic-right-shifted tap weighted by coeff sign
                    acc <= acc + ((coeff[tap_idx][WIDTH-1]) ?
                            -({{4{tap_dl[tap_idx][WIDTH-1]}}, tap_dl[tap_idx]} >>> 1) :
                             ({{4{tap_dl[tap_idx][WIDTH-1]}}, tap_dl[tap_idx]} >>> 1));
                    if (tap_idx == TAPS-1)
                        step <= S_OUTPUT;
                    else
                        tap_idx <= tap_idx + 4'd1;
                end
                S_OUTPUT: begin
                    // Clamp accumulator to WIDTH bits
                    data_out <= acc[WIDTH-1:0];
                    error    <= ref_in - acc[WIDTH-1:0];
                    tap_idx  <= 0;
                    step     <= S_ADAPT;
                end
                S_ADAPT: begin
                    // LMS update: coeff[i] += µ * error * tap_dl[i]
                    // µ * error ≈ error >>> MU_SHIFT (shift-add, no multiply)
                    coeff[tap_idx] <= coeff[tap_idx] +
                        ((error >>> MU_SHIFT) + (tap_dl[tap_idx] >>> (MU_SHIFT+1)));
                    if (tap_idx == TAPS-1)
                        step <= S_DONE;
                    else
                        tap_idx <= tap_idx + 4'd1;
                end
                S_DONE: begin
                    done <= 1'b1;
                    step <= S_IDLE;
                end
                default: step <= S_IDLE;
            endcase
        end
    end

endmodule


// ============================================================================
// Module 5: Lifting-Scheme DWT Engine (distributed arithmetic, no multipliers)
// ============================================================================
// 3-level wavelet decomposition using the lifting scheme (Haar basis).
// Predict step:  d[n] = x[2n+1] - x[2n]            (detail)
// Update step:   a[n] = x[2n]   + (d[n] >>> 1)      (approximation)
// All operations: add/subtract/shift — zero multipliers.
// ============================================================================
module dwt_engine #(
    parameter WIDTH  = 12,
    parameter LEVELS = 3
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    en,
    input  wire                    data_valid,
    input  wire signed [WIDTH-1:0] data_in,
    // Output: 4 sub-band coefficients per decomposition (approx3, detail3, detail2, detail1)
    output reg  signed [WIDTH-1:0] approx_out,    // Lowest-frequency approximation
    output reg  signed [WIDTH-1:0] detail1_out,   // Level-1 detail
    output reg  signed [WIDTH-1:0] detail2_out,   // Level-2 detail
    output reg  signed [WIDTH-1:0] detail3_out,   // Level-3 detail
    output reg                     done
);

    // Pair collection for each level
    reg signed [WIDTH-1:0] even_sample;
    reg        pair_phase;   // 0 = collecting even, 1 = collecting odd

    // Three-level pipeline state
    reg [2:0] level;
    reg signed [WIDTH-1:0] l1_approx, l1_detail;
    reg signed [WIDTH-1:0] l2_approx, l2_detail;
    reg signed [WIDTH-1:0] l3_approx, l3_detail;

    // Collecting state per level
    reg l1_have_even, l2_have_even, l3_have_even;
    reg signed [WIDTH-1:0] l1_even, l2_even, l3_even;

    reg l1_done, l2_done, l3_done;

    // Lifting step helper wires
    wire signed [WIDTH-1:0] lift_detail = data_in - even_sample;                    // predict
    wire signed [WIDTH-1:0] lift_approx = even_sample + (lift_detail >>> 1);        // update

    // Level-2 lifting on l1 approx stream
    wire signed [WIDTH-1:0] l2_lift_detail = l1_approx - l2_even;
    wire signed [WIDTH-1:0] l2_lift_approx = l2_even + (l2_lift_detail >>> 1);

    // Level-3 lifting on l2 approx stream
    wire signed [WIDTH-1:0] l3_lift_detail = l2_approx - l3_even;
    wire signed [WIDTH-1:0] l3_lift_approx = l3_even + (l3_lift_detail >>> 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pair_phase  <= 1'b0;
            even_sample <= 0;
            l1_approx   <= 0; l1_detail <= 0; l1_done <= 0; l1_have_even <= 0; l1_even <= 0;
            l2_approx   <= 0; l2_detail <= 0; l2_done <= 0; l2_have_even <= 0; l2_even <= 0;
            l3_approx   <= 0; l3_detail <= 0; l3_done <= 0; l3_have_even <= 0; l3_even <= 0;
            approx_out  <= 0; detail1_out <= 0; detail2_out <= 0; detail3_out <= 0;
            done        <= 0;
        end else if (en && data_valid) begin
            done <= 1'b0;

            // --- Level 1 Lifting ---
            if (!pair_phase) begin
                // Collecting even sample
                even_sample <= data_in;
                pair_phase  <= 1'b1;
                l1_done     <= 1'b0;
            end else begin
                // Odd sample arrived → compute lifting
                l1_detail   <= lift_detail;
                l1_approx   <= lift_approx;
                detail1_out <= lift_detail;
                pair_phase  <= 1'b0;
                l1_done     <= 1'b1;
            end

            // --- Level 2 Lifting (operates on L1 approx stream) ---
            if (l1_done) begin
                if (!l2_have_even) begin
                    l2_even     <= l1_approx;
                    l2_have_even <= 1'b1;
                    l2_done     <= 1'b0;
                end else begin
                    l2_detail    <= l2_lift_detail;
                    l2_approx    <= l2_lift_approx;
                    detail2_out  <= l2_lift_detail;
                    l2_have_even <= 1'b0;
                    l2_done      <= 1'b1;
                end
            end

            // --- Level 3 Lifting (operates on L2 approx stream) ---
            if (l2_done) begin
                if (!l3_have_even) begin
                    l3_even     <= l2_approx;
                    l3_have_even <= 1'b1;
                    l3_done     <= 1'b0;
                end else begin
                    l3_detail    <= l3_lift_detail;
                    l3_approx    <= l3_lift_approx;
                    detail3_out  <= l3_lift_detail;
                    approx_out   <= l3_lift_approx;
                    l3_have_even <= 1'b0;
                    l3_done      <= 1'b1;
                    done         <= 1'b1;  // Full decomposition complete
                end
            end
        end
    end

endmodule


// ============================================================================
// Module 6: CORDIC Phase Extractor (vectoring mode, 12 iterations)
// ============================================================================
// Extracts phase (angle) and magnitude from I/Q-style input.
// All operations are shift-add (no multipliers).
// Precision: ~10 bits from 12 iterations.
// ============================================================================
module cordic_phase #(
    parameter WIDTH = 12,
    parameter ITERS = 12
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    en,
    input  wire                    start,
    input  wire signed [WIDTH-1:0] x_in,     // Real / magnitude-like
    input  wire signed [WIDTH-1:0] y_in,     // Imaginary / detail-like
    output reg  signed [WIDTH-1:0] phase,    // Output angle
    output reg  signed [WIDTH-1:0] magnitude,// Output magnitude
    output reg                     done
);

    // Pre-computed arctangent table: atan(2^{-i}) in fixed-point (scaled by 2^(WIDTH-2))
    // For WIDTH=12: full-scale angle = ±π  → 1 LSB ≈ π/2048
    // atan table values (degrees → fixed): 45.0, 26.57, 14.04, 7.13, 3.58, 1.79, 0.90, ...
    reg signed [WIDTH-1:0] atan_table [0:ITERS-1];

    // Initialise table with compile-time constants (scaled for 12-bit)
    // atan(2^-i) * 2^(WIDTH-2) / π
    initial begin
        atan_table[0]  = 12'sd512;   // 45.000°
        atan_table[1]  = 12'sd302;   // 26.565°
        atan_table[2]  = 12'sd160;   // 14.036°
        atan_table[3]  = 12'sd081;   //  7.125°
        atan_table[4]  = 12'sd041;   //  3.576°
        atan_table[5]  = 12'sd020;   //  1.790°
        atan_table[6]  = 12'sd010;   //  0.895°
        atan_table[7]  = 12'sd005;   //  0.448°
        atan_table[8]  = 12'sd003;   //  0.224°
        atan_table[9]  = 12'sd001;   //  0.112°
        atan_table[10] = 12'sd001;   //  0.056°
        atan_table[11] = 12'sd000;   //  0.028°
    end

    reg signed [WIDTH-1:0] x_reg, y_reg, z_reg;
    reg [3:0] iter;

    localparam S_IDLE = 2'd0,
               S_RUN  = 2'd1,
               S_OUT  = 2'd2;
    reg [1:0] state;

    wire signed [WIDTH-1:0] x_shifted = x_reg >>> iter;
    wire signed [WIDTH-1:0] y_shifted = y_reg >>> iter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            x_reg     <= 0; y_reg <= 0; z_reg <= 0;
            iter      <= 0;
            phase     <= 0; magnitude <= 0;
            done      <= 0;
        end else if (en) begin
            case (state)
                S_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        // Pre-rotate to first quadrant
                        x_reg <= (x_in[WIDTH-1]) ? -x_in : x_in;
                        y_reg <= (x_in[WIDTH-1]) ? -y_in : y_in;
                        z_reg <= 0;
                        iter  <= 0;
                        state <= S_RUN;
                    end
                end
                S_RUN: begin
                    // Vectoring mode: drive y toward zero
                    if (y_reg[WIDTH-1]) begin
                        // y < 0: rotate counter-clockwise
                        x_reg <= x_reg - y_shifted;
                        y_reg <= y_reg + x_shifted;
                        z_reg <= z_reg - atan_table[iter];
                    end else begin
                        // y >= 0: rotate clockwise
                        x_reg <= x_reg + y_shifted;
                        y_reg <= y_reg - x_shifted;
                        z_reg <= z_reg + atan_table[iter];
                    end
                    if (iter == ITERS - 1)
                        state <= S_OUT;
                    else
                        iter <= iter + 4'd1;
                end
                S_OUT: begin
                    magnitude <= x_reg;   // ≈ K * sqrt(x² + y²), K ≈ 1.647
                    phase     <= z_reg;   // Accumulated angle
                    done      <= 1'b1;
                    state     <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule


// ============================================================================
// Module 7: Power Bin Accumulator (8 bins)
// ============================================================================
// Accumulates squared power-magnitude values into 8 frequency bins.
// Maps DWT sub-band energies to bins for command generation.
// ============================================================================
module power_bin_accumulator #(
    parameter WIDTH    = 12,
    parameter NUM_BINS = 8,
    parameter ACC_W    = 16
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    en,
    input  wire                    clear,           // Clear all bins
    input  wire [2:0]              bin_sel,          // Which bin to accumulate into
    input  wire                    acc_valid,        // Data ready to accumulate
    input  wire signed [WIDTH-1:0] magnitude,        // From CORDIC
    output reg  [ACC_W-1:0]        bin_values [0:NUM_BINS-1],
    output reg                     done
);

    // Squared-magnitude approximation: |mag|² ≈ mag * mag
    // Shift-add approx: |mag|² ≈ (mag <<< 1) + mag (for small dynamic range)
    // More accurate: use the unsigned absolute value shift-add
    wire [WIDTH-1:0] abs_mag = magnitude[WIDTH-1] ? (~magnitude + 1'b1) : magnitude;
    wire [ACC_W-1:0] sq_approx = {{(ACC_W-WIDTH){1'b0}}, abs_mag} *
                                  {{(ACC_W-WIDTH){1'b0}}, abs_mag};
    // Note: synthesis tools will map this to shift-add for small widths

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear) begin
            for (i = 0; i < NUM_BINS; i = i + 1)
                bin_values[i] <= 0;
            done <= 1'b0;
        end else if (en && acc_valid) begin
            bin_values[bin_sel] <= bin_values[bin_sel] + sq_approx;
            done <= 1'b1;
        end else begin
            done <= 1'b0;
        end
    end

endmodule


// ============================================================================
// Module 8: 3-bit Command Encoder
// ============================================================================
// Maps power-bin vector → 3-bit discrete command.
// Encoding:
//   000: No activity          001: Low-freq dominant
//   010: Mid-freq dominant    011: High-freq dominant
//   100: Artifact detected    101: Motion detected
//   110: Reserved             111: Emergency / high-priority
// ============================================================================
module command_encoder #(
    parameter CMD_WIDTH = 3,
    parameter NUM_BINS  = 8,
    parameter ACC_W     = 16
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                en,
    input  wire                start,
    input  wire [ACC_W-1:0]    bin_values [0:NUM_BINS-1],
    output reg  [CMD_WIDTH-1:0] cmd_out,
    output reg                  cmd_valid
);

    // Find the dominant bin (max energy) using sequential comparison
    reg [2:0] max_idx;
    reg [ACC_W-1:0] max_val;
    reg [3:0] scan_idx;
    reg scanning;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_out   <= 3'b000;
            cmd_valid <= 1'b0;
            max_idx   <= 0;
            max_val   <= 0;
            scan_idx  <= 0;
            scanning  <= 1'b0;
        end else if (en) begin
            cmd_valid <= 1'b0;
            if (start && !scanning) begin
                scanning <= 1'b1;
                scan_idx <= 0;
                max_idx  <= 0;
                max_val  <= 0;
            end else if (scanning) begin
                if (scan_idx < NUM_BINS) begin
                    if (bin_values[scan_idx[2:0]] > max_val) begin
                        max_val <= bin_values[scan_idx[2:0]];
                        max_idx <= scan_idx[2:0];
                    end
                    scan_idx <= scan_idx + 4'd1;
                end else begin
                    // Encode dominant bin → command
                    // Bins 0-1 → low-freq, 2-3 → mid, 4-5 → high, 6 → artifact, 7 → emergency
                    case (max_idx)
                        3'd0, 3'd1: cmd_out <= 3'b001;  // Low-freq
                        3'd2, 3'd3: cmd_out <= 3'b010;  // Mid-freq
                        3'd4, 3'd5: cmd_out <= 3'b011;  // High-freq
                        3'd6:       cmd_out <= 3'b100;  // Artifact
                        3'd7:       cmd_out <= 3'b111;  // Emergency
                        default:    cmd_out <= 3'b000;
                    endcase
                    // Override to 000 if no significant energy
                    if (max_val == 0)
                        cmd_out <= 3'b000;
                    cmd_valid <= 1'b1;
                    scanning  <= 1'b0;
                end
            end
        end
    end

endmodule


// ============================================================================
// Module 9: LSK Modulator (Load Shift Keying, single MOSFET)
// ============================================================================
// Serially transmits a 3-bit command by toggling inductor impedance.
// Manchester encoding for DC balance.
// Bit period configurable; default ~100 µs at 100kHz clock → 10 cycles/bit.
// ============================================================================
module lsk_modulator #(
    parameter CMD_WIDTH     = 3,
    parameter CLKS_PER_HALF = 5   // Half-bit period in clock cycles
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  en,
    input  wire                  cmd_valid,         // Latch and begin TX
    input  wire [CMD_WIDTH-1:0]  cmd_in,
    output reg                   lsk_ctrl,          // MOSFET gate — toggles inductor Z
    output reg                   lsk_tx_active      // Transmission in progress
);

    reg [CMD_WIDTH-1:0] shift_reg;
    reg [3:0] bit_idx;
    reg [7:0] half_cnt;
    reg       manchester_phase;  // 0 = first half, 1 = second half
    reg       tx_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lsk_ctrl      <= 1'b0;
            lsk_tx_active <= 1'b0;
            shift_reg     <= 0;
            bit_idx       <= 0;
            half_cnt      <= 0;
            manchester_phase <= 1'b0;
            tx_busy       <= 1'b0;
        end else if (en) begin
            if (cmd_valid && !tx_busy) begin
                // Latch command, start transmission
                shift_reg     <= cmd_in;
                bit_idx       <= CMD_WIDTH - 1;
                half_cnt      <= 0;
                manchester_phase <= 1'b0;
                tx_busy       <= 1'b1;
                lsk_tx_active <= 1'b1;
            end else if (tx_busy) begin
                // Manchester encoding:
                //   data=1 → high-then-low in bit period
                //   data=0 → low-then-high in bit period
                if (half_cnt < CLKS_PER_HALF - 1) begin
                    half_cnt <= half_cnt + 8'd1;
                end else begin
                    half_cnt <= 0;
                    if (!manchester_phase) begin
                        // First half of bit
                        lsk_ctrl <= shift_reg[bit_idx];
                        manchester_phase <= 1'b1;
                    end else begin
                        // Second half of bit (inverted)
                        lsk_ctrl <= ~shift_reg[bit_idx];
                        manchester_phase <= 1'b0;
                        if (bit_idx == 0) begin
                            // All bits transmitted
                            tx_busy       <= 1'b0;
                            lsk_tx_active <= 1'b0;
                            lsk_ctrl      <= 1'b0;
                        end else begin
                            bit_idx <= bit_idx - 4'd1;
                        end
                    end
                end
            end else begin
                lsk_ctrl      <= 1'b0;
                lsk_tx_active <= 1'b0;
            end
        end
    end

endmodule


// ============================================================================
// Module 10: Power Manager (energy harvesting + dual-rail regulator model)
// ============================================================================
// Behavioral model of the power subsystem:
//   Inductor harvest → rectifier → storage cap → dual-rail regulator
// Tracks capacitor voltage and generates supply-good indicators.
// ============================================================================
module power_manager #(
    parameter CAP_BITS   = 8,       // Storage cap voltage resolution
    parameter HARVEST_INC = 8'd1,   // Charge increment per harvest cycle
    parameter DRAIN_DIG   = 8'd2,   // Digital core drain per active cycle
    parameter DRAIN_ANA   = 8'd0,   // Analog drain (near-zero at sub-threshold)
    parameter THRESH_OK   = 8'd40,  // V_cap threshold for "OK" status (~200mV)
    parameter THRESH_LOW  = 8'd20   // V_cap low warning
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              harvest_active,   // Inductor field present
    input  wire              dig_core_active,  // Digital core consuming power
    output reg [CAP_BITS-1:0] vcap,            // Storage capacitor voltage proxy
    output wire              vrect_ok,         // Rectified voltage sufficient
    output wire              vcap_ok,          // Cap voltage sufficient for operation
    output wire              harvesting        // Currently harvesting
);

    assign vrect_ok   = harvest_active;
    assign vcap_ok    = (vcap >= THRESH_OK);
    assign harvesting = harvest_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vcap <= 8'd0;
        end else begin
            // Charge from harvesting
            if (harvest_active && vcap < 8'hFF)
                vcap <= vcap + HARVEST_INC;

            // Drain from analog (always-on comparator ~1nW → negligible per cycle)
            // Drain from digital core when active
            if (dig_core_active && vcap > DRAIN_DIG)
                vcap <= vcap - DRAIN_DIG;
        end
    end

endmodule


// ============================================================================
// Module 11: Register File (STATUS, CTRL, THRESH, CMD_OUT, PWR_STAT)
// ============================================================================
module register_file (
    input  wire        clk,
    input  wire        rst_n,
    // Write interface (directly from configuration — no serial bus yet)
    input  wire [7:0]  ctrl_wr_data,
    input  wire        ctrl_wr_en,
    input  wire [7:0]  thresh_wr_data,
    input  wire        thresh_wr_en,
    // Hardware status inputs
    input  wire        harvesting,
    input  wire        vrect_ok,
    input  wire        vcap_ok,
    input  wire        adc_active,
    input  wire        dig_awake,
    input  wire        event_det,
    input  wire        cmd_pend,
    input  wire        lsk_tx,
    input  wire [2:0]  cmd_code,
    input  wire [7:0]  vcap_level,
    // Register outputs
    output wire [7:0]  status_reg,     // 0x00
    output reg  [7:0]  ctrl_reg,       // 0x04
    output reg  [7:0]  thresh_reg,     // 0x08
    output wire [7:0]  cmd_out_reg,    // 0x0C
    output wire [7:0]  pwr_stat_reg    // 0x10
);

    // STATUS — read-only, assembled from hardware
    assign status_reg = {harvesting, vrect_ok, vcap_ok, adc_active,
                         dig_awake, event_det, cmd_pend, lsk_tx};

    // CMD_OUT — read-only
    assign cmd_out_reg = {5'b00000, cmd_code};

    // PWR_STAT — read-only
    assign pwr_stat_reg = vcap_level;

    // CTRL — read-write
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ctrl_reg <= 8'h01;   // SYS_EN = 1 on reset
        else if (ctrl_wr_en)
            ctrl_reg <= ctrl_wr_data;
    end

    // THRESH — read-write
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            thresh_reg <= 8'h32; // 50 mV default
        else if (thresh_wr_en)
            thresh_reg <= thresh_wr_data;
    end

endmodule


// ============================================================================
// Module 12: NeuroCore Top-Level Integration
// ============================================================================
// Connects all blocks into the full signal chain:
//   Analog Front-End → Event Detection → Digital Core → LSK Output
//   Power subsystem provides energy management.
//
// Port convention follows Tiny Tapeout standard (optional wrapper in
// tt_um_NeuroCore.v maps these to tt_um_example ports).
// ============================================================================
module neurocore_top #(
    parameter ADC_BITS    = 4,
    parameter LMS_TAPS    = 8,
    parameter LMS_WIDTH   = 8,
    parameter DWT_WIDTH   = 12,
    parameter DWT_LEVELS  = 3,
    parameter CORDIC_W    = 12,
    parameter CORDIC_ITER = 12,
    parameter NUM_BINS    = 8,
    parameter CMD_WIDTH   = 3,
    parameter ACC_W       = 16,
    parameter LSK_HALF    = 5
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- External / Tiny-Tapeout IO mapping ---
    input  wire [3:0]  analog_in,       // Digitised proxy of inductor signal
    input  wire        harvest_field,   // External magnetic field present
    input  wire [7:0]  cfg_data,        // Configuration write data
    input  wire        cfg_ctrl_wr,     // Write-enable for CTRL register
    input  wire        cfg_thresh_wr,   // Write-enable for THRESH register

    output wire [2:0]  cmd_out,         // Final 3-bit command
    output wire        cmd_valid,       // Command valid strobe
    output wire        lsk_ctrl,        // MOSFET gate for LSK
    output wire        lsk_tx_active,   // LSK transmission flag
    output wire [7:0]  status_out,      // STATUS register readback
    output wire [7:0]  pwr_stat_out     // PWR_STAT register readback
);

    // ----------------------------------------------------------------
    // Internal wires
    // ----------------------------------------------------------------

    // Register file outputs
    wire [7:0] ctrl_reg, thresh_reg, status_reg, cmd_out_reg, pwr_stat_reg;
    wire sys_en      = ctrl_reg[0];
    wire comp_en     = ctrl_reg[1];
    wire adc_en_cfg  = ctrl_reg[2];
    wire force_wake  = ctrl_reg[3];

    // ADC
    wire [ADC_BITS-1:0] adc_data;
    wire                adc_valid;
    wire                adc_active;

    // Event detector
    wire wake_pulse;
    wire event_det;

    // Power gate
    wire pwr_gate_active;
    wire dig_clk_en;

    // Gated clock for digital core
    wire dig_clk = clk & dig_clk_en;

    // LMS
    wire signed [LMS_WIDTH-1:0] lms_out;
    wire                        lms_done;

    // Widen ADC data for LMS
    wire signed [LMS_WIDTH-1:0] adc_signed = {{(LMS_WIDTH-ADC_BITS){1'b0}}, adc_data};

    // DWT
    wire signed [DWT_WIDTH-1:0] dwt_approx, dwt_d1, dwt_d2, dwt_d3;
    wire                        dwt_done;

    // Widen LMS output for DWT
    wire signed [DWT_WIDTH-1:0] lms_to_dwt = {{(DWT_WIDTH-LMS_WIDTH){lms_out[LMS_WIDTH-1]}}, lms_out};

    // CORDIC
    wire signed [CORDIC_W-1:0] cordic_phase, cordic_mag;
    wire                       cordic_done;

    // Widen DWT outputs for CORDIC
    wire signed [CORDIC_W-1:0] cordic_x = dwt_approx;  // Same width
    wire signed [CORDIC_W-1:0] cordic_y = dwt_d1;       // Detail as Q-component

    // Power bins
    wire [ACC_W-1:0] bin_vals [0:NUM_BINS-1];
    wire             bins_done;

    // Command encoder
    wire [CMD_WIDTH-1:0] enc_cmd;
    wire                 enc_valid;

    // Pipeline sequencing FSM
    reg [3:0] pipe_state;
    localparam P_IDLE       = 4'd0,
               P_LMS        = 4'd1,
               P_WAIT_LMS   = 4'd2,
               P_DWT        = 4'd3,
               P_WAIT_DWT   = 4'd4,
               P_CORDIC     = 4'd5,
               P_WAIT_CORD  = 4'd6,
               P_ACCUM      = 4'd7,
               P_WAIT_ACC   = 4'd8,
               P_ENCODE     = 4'd9,
               P_WAIT_ENC   = 4'd10,
               P_LSK        = 4'd11,
               P_WAIT_LSK   = 4'd12,
               P_DONE       = 4'd13;

    reg lms_start, dwt_start, cordic_start, acc_start, enc_start, lsk_start;
    reg processing_done;
    reg [2:0] bin_sel_r;

    // Power manager
    wire [7:0] vcap_level;
    wire       vrect_ok, vcap_ok, harvesting_flag;

    // ----------------------------------------------------------------
    // Instantiations
    // ----------------------------------------------------------------

    // --- Analog Front-End ---
    level_crossing_adc #(.ADC_BITS(ADC_BITS)) u_adc (
        .clk        (clk),
        .rst_n      (rst_n),
        .adc_en     (adc_en_cfg & sys_en),
        .analog_in  (analog_in),
        .adc_data   (adc_data),
        .adc_valid  (adc_valid),
        .adc_active (adc_active)
    );

    // --- Event Detector ---
    event_detector #(.ADC_BITS(ADC_BITS)) u_evt (
        .clk            (clk),
        .rst_n          (rst_n),
        .comp_en        (comp_en & sys_en),
        .analog_in      (analog_in),
        .threshold      (thresh_reg),
        .wake           (wake_pulse),
        .event_detected (event_det)
    );

    // --- Power Gate Controller ---
    power_gate_ctrl u_pg (
        .clk             (clk),
        .rst_n           (rst_n),
        .wake            (wake_pulse),
        .force_wake      (force_wake),
        .processing_done (processing_done),
        .pwr_gate_active (pwr_gate_active),
        .clk_en          (dig_clk_en)
    );

    // --- LMS Filter ---
    lms_filter #(.TAPS(LMS_TAPS), .WIDTH(LMS_WIDTH)) u_lms (
        .clk        (dig_clk),
        .rst_n      (rst_n),
        .en         (pwr_gate_active),
        .data_valid (lms_start),
        .data_in    (adc_signed),
        .ref_in     ({LMS_WIDTH{1'b0}}),  // Zero reference for artifact removal
        .data_out   (lms_out),
        .done       (lms_done)
    );

    // --- DWT Engine ---
    dwt_engine #(.WIDTH(DWT_WIDTH), .LEVELS(DWT_LEVELS)) u_dwt (
        .clk         (dig_clk),
        .rst_n       (rst_n),
        .en          (pwr_gate_active),
        .data_valid  (dwt_start),
        .data_in     (lms_to_dwt),
        .approx_out  (dwt_approx),
        .detail1_out (dwt_d1),
        .detail2_out (dwt_d2),
        .detail3_out (dwt_d3),
        .done        (dwt_done)
    );

    // --- CORDIC Phase Extractor ---
    cordic_phase #(.WIDTH(CORDIC_W), .ITERS(CORDIC_ITER)) u_cordic (
        .clk       (dig_clk),
        .rst_n     (rst_n),
        .en        (pwr_gate_active),
        .start     (cordic_start),
        .x_in      (cordic_x),
        .y_in      (cordic_y),
        .phase     (cordic_phase),
        .magnitude (cordic_mag),
        .done      (cordic_done)
    );

    // --- Power Bin Accumulator ---
    power_bin_accumulator #(.WIDTH(CORDIC_W), .NUM_BINS(NUM_BINS), .ACC_W(ACC_W)) u_bins (
        .clk        (dig_clk),
        .rst_n      (rst_n),
        .en         (pwr_gate_active),
        .clear      (~pwr_gate_active),
        .bin_sel    (bin_sel_r),
        .acc_valid  (acc_start),
        .magnitude  (cordic_mag),
        .bin_values (bin_vals),
        .done       (bins_done)
    );

    // --- Command Encoder ---
    command_encoder #(.CMD_WIDTH(CMD_WIDTH), .NUM_BINS(NUM_BINS), .ACC_W(ACC_W)) u_enc (
        .clk        (dig_clk),
        .rst_n      (rst_n),
        .en         (pwr_gate_active),
        .start      (enc_start),
        .bin_values (bin_vals),
        .cmd_out    (enc_cmd),
        .cmd_valid  (enc_valid)
    );

    // --- LSK Modulator ---
    lsk_modulator #(.CMD_WIDTH(CMD_WIDTH), .CLKS_PER_HALF(LSK_HALF)) u_lsk (
        .clk           (clk),         // Runs on ungated clock
        .rst_n         (rst_n),
        .en            (sys_en),
        .cmd_valid     (lsk_start),
        .cmd_in        (enc_cmd),
        .lsk_ctrl      (lsk_ctrl),
        .lsk_tx_active (lsk_tx_active)
    );

    // --- Power Manager ---
    power_manager u_pwr (
        .clk             (clk),
        .rst_n           (rst_n),
        .harvest_active  (harvest_field),
        .dig_core_active (pwr_gate_active),
        .vcap            (vcap_level),
        .vrect_ok        (vrect_ok),
        .vcap_ok         (vcap_ok),
        .harvesting      (harvesting_flag)
    );

    // --- Register File ---
    register_file u_regs (
        .clk            (clk),
        .rst_n          (rst_n),
        .ctrl_wr_data   (cfg_data),
        .ctrl_wr_en     (cfg_ctrl_wr),
        .thresh_wr_data (cfg_data),
        .thresh_wr_en   (cfg_thresh_wr),
        .harvesting     (harvesting_flag),
        .vrect_ok       (vrect_ok),
        .vcap_ok        (vcap_ok),
        .adc_active     (adc_active),
        .dig_awake      (pwr_gate_active),
        .event_det      (event_det),
        .cmd_pend       (enc_valid),
        .lsk_tx         (lsk_tx_active),
        .cmd_code       (enc_cmd),
        .vcap_level     (vcap_level),
        .status_reg     (status_reg),
        .ctrl_reg       (ctrl_reg),
        .thresh_reg     (thresh_reg),
        .cmd_out_reg    (cmd_out_reg),
        .pwr_stat_reg   (pwr_stat_reg)
    );

    // ----------------------------------------------------------------
    // Output assignments
    // ----------------------------------------------------------------
    assign cmd_out      = enc_cmd;
    assign cmd_valid    = enc_valid;
    assign status_out   = status_reg;
    assign pwr_stat_out = pwr_stat_reg;

    // ----------------------------------------------------------------
    // Digital Processing Pipeline FSM
    // ----------------------------------------------------------------
    // Sequences the power-gated digital blocks after WAKE.
    // Each stage waits for its done signal before advancing.
    // ----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_state      <= P_IDLE;
            lms_start       <= 1'b0;
            dwt_start       <= 1'b0;
            cordic_start    <= 1'b0;
            acc_start       <= 1'b0;
            enc_start       <= 1'b0;
            lsk_start       <= 1'b0;
            processing_done <= 1'b0;
            bin_sel_r       <= 3'd0;
        end else begin
            // Default de-assert single-cycle strobes
            lms_start    <= 1'b0;
            dwt_start    <= 1'b0;
            cordic_start <= 1'b0;
            acc_start    <= 1'b0;
            enc_start    <= 1'b0;
            lsk_start    <= 1'b0;
            processing_done <= 1'b0;

            case (pipe_state)
                P_IDLE: begin
                    if (dig_clk_en && adc_valid) begin
                        lms_start  <= 1'b1;
                        pipe_state <= P_WAIT_LMS;
                    end
                end

                P_WAIT_LMS: begin
                    if (lms_done) begin
                        dwt_start  <= 1'b1;
                        pipe_state <= P_WAIT_DWT;
                    end
                end

                P_WAIT_DWT: begin
                    if (dwt_done) begin
                        cordic_start <= 1'b1;
                        pipe_state   <= P_WAIT_CORD;
                    end
                end

                P_WAIT_CORD: begin
                    if (cordic_done) begin
                        acc_start  <= 1'b1;
                        bin_sel_r  <= 3'd0;    // Accumulate into bin 0 (round-robin can be added)
                        pipe_state <= P_WAIT_ACC;
                    end
                end

                P_WAIT_ACC: begin
                    if (bins_done) begin
                        enc_start  <= 1'b1;
                        pipe_state <= P_WAIT_ENC;
                    end
                end

                P_WAIT_ENC: begin
                    if (enc_valid) begin
                        lsk_start  <= 1'b1;
                        pipe_state <= P_WAIT_LSK;
                    end
                end

                P_WAIT_LSK: begin
                    if (!lsk_tx_active) begin
                        processing_done <= 1'b1;
                        pipe_state      <= P_DONE;
                    end
                end

                P_DONE: begin
                    pipe_state <= P_IDLE;
                end

                default: pipe_state <= P_IDLE;
            endcase
        end
    end

endmodule

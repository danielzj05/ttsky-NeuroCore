// ============================================================================
// NeuroCore Field Sensor - Ultra-Low-Power Digital Core
// ============================================================================
// Ultra-low-power magnetic field sensing system with closed-loop feedback
// Target: <20nW peak active, ~1nW always-on (analog only)
//
// Key Features:
// - Event-driven processing (power-gated when idle)
// - 8-tap LMS artifact filter (shift-add, no multipliers)
// - 3-level lifting DWT (distributed arithmetic)
// - 12-iteration CORDIC phase extractor
// - 8-bin power accumulator
// - 3-bit command encoder
// - LSK modulation output
//
// Copyright (c) 2024 Design Team
// SPDX-License-Identifier: Apache-2.0
// ============================================================================

`default_nettype none

// ============================================================================
// Top-Level Module
// ============================================================================
module neurocore_field_sensor #(
    parameter LMS_TAPS      = 8,      // LMS filter tap count
    parameter LMS_WIDTH     = 8,      // LMS data width
    parameter DWT_LEVELS    = 3,      // DWT decomposition levels
    parameter DWT_WIDTH     = 12,     // DWT coefficient width
    parameter CORDIC_ITERS  = 12,     // CORDIC iterations
    parameter CORDIC_WIDTH  = 12,     // CORDIC data width
    parameter NUM_BINS      = 8,      // Power accumulator bins
    parameter CMD_WIDTH     = 3,      // Command width (3-bit = 8 commands)
    parameter ADC_BITS      = 4       // ADC resolution
) (
    // Clock and Reset
    input  wire                  clk,          // Digital core clock
    input  wire                  rst_n,        // Active-low reset
    
    // ADC Interface (from analog front-end)
    input  wire [ADC_BITS-1:0]   adc_data,     // ADC level code
    input  wire                  adc_valid,    // ADC data valid
    
    // Event Detection Interface
    input  wire                  wake,         // Wake pulse from comparator
    
    // Command Output Interface
    output wire [CMD_WIDTH-1:0]  cmd_out,      // 3-bit command
    output wire                  cmd_valid,    // Command valid
    
    // LSK Modulator Interface
    output wire                  lsk_ctrl,     // LSK MOSFET control
    output wire                  lsk_tx,       // LSK transmission active
    
    // Power Gating Control
    output wire                  pwr_gate_ctrl,// Power gate control (1=active)
    
    // Status Outputs
    output wire                  lms_busy,     // LMS processing
    output wire                  dwt_busy,     // DWT processing
    output wire                  cordic_busy,  // CORDIC processing
    output wire                  processing    // Any block active
);

    // ========================================================================
    // Internal Signals
    // ========================================================================
    
    // LMS Filter signals
    wire [LMS_WIDTH-1:0]     lms_out;
    wire                     lms_valid;
    wire                     lms_start;
    
    // DWT Engine signals
    wire [DWT_WIDTH-1:0]     dwt_out_0, dwt_out_1, dwt_out_2, dwt_out_3;
    wire [DWT_WIDTH-1:0]     dwt_out_4, dwt_out_5, dwt_out_6, dwt_out_7;
    wire                     dwt_valid;
    wire                     dwt_start;
    
    // CORDIC signals
    wire [CORDIC_WIDTH-1:0]  cordic_mag_0, cordic_mag_1, cordic_mag_2, cordic_mag_3;
    wire [CORDIC_WIDTH-1:0]  cordic_mag_4, cordic_mag_5, cordic_mag_6, cordic_mag_7;
    wire                     cordic_valid;
    wire                     cordic_start;
    
    // Power Accumulator signals
    wire [15:0]              power_bins_0, power_bins_1, power_bins_2, power_bins_3;
    wire [15:0]              power_bins_4, power_bins_5, power_bins_6, power_bins_7;
    wire                     acc_valid;
    wire                     acc_start;
    
    // Command Encoder signals
    wire [CMD_WIDTH-1:0]     cmd_encoded;
    wire                     cmd_ready;
    
    // Main FSM signals
    reg  [3:0]               state;
    reg  [3:0]               next_state;
    
    // FSM States
    localparam S_IDLE       = 4'd0;
    localparam S_WAKE       = 4'd1;
    localparam S_LMS        = 4'd2;
    localparam S_WAIT_LMS   = 4'd3;
    localparam S_DWT        = 4'd4;
    localparam S_WAIT_DWT   = 4'd5;
    localparam S_CORDIC     = 4'd6;
    localparam S_WAIT_CORDIC= 4'd7;
    localparam S_ACCUM      = 4'd8;
    localparam S_WAIT_ACCUM = 4'd9;
    localparam S_ENCODE     = 4'd10;
    localparam S_LSK_TX     = 4'd11;
    localparam S_SLEEP      = 4'd12;
    
    // ========================================================================
    // Main Control FSM
    // ========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else
            state <= next_state;
    end
    
    always @(*) begin
        next_state = state;
        
        case (state)
            S_IDLE: begin
                if (wake)
                    next_state = S_WAKE;
            end
            
            S_WAKE: begin
                next_state = S_LMS;
            end
            
            S_LMS: begin
                next_state = S_WAIT_LMS;
            end
            
            S_WAIT_LMS: begin
                if (lms_valid)
                    next_state = S_DWT;
            end
            
            S_DWT: begin
                next_state = S_WAIT_DWT;
            end
            
            S_WAIT_DWT: begin
                if (dwt_valid)
                    next_state = S_CORDIC;
            end
            
            S_CORDIC: begin
                next_state = S_WAIT_CORDIC;
            end
            
            S_WAIT_CORDIC: begin
                if (cordic_valid)
                    next_state = S_ACCUM;
            end
            
            S_ACCUM: begin
                next_state = S_WAIT_ACCUM;
            end
            
            S_WAIT_ACCUM: begin
                if (acc_valid)
                    next_state = S_ENCODE;
            end
            
            S_ENCODE: begin
                if (cmd_ready)
                    next_state = S_LSK_TX;
            end
            
            S_LSK_TX: begin
                if (!lsk_tx)  // Wait for LSK transmission complete
                    next_state = S_SLEEP;
            end
            
            S_SLEEP: begin
                next_state = S_IDLE;
            end
            
            default: next_state = S_IDLE;
        endcase
    end
    
    // Control signals
    assign lms_start    = (state == S_LMS);
    assign dwt_start    = (state == S_DWT);
    assign cordic_start = (state == S_CORDIC);
    assign acc_start    = (state == S_ACCUM);
    
    // Power gating control
    assign pwr_gate_ctrl = (state != S_IDLE) && (state != S_SLEEP);
    
    // Processing status
    assign processing = lms_busy | dwt_busy | cordic_busy;
    
    // ========================================================================
    // LMS Artifact Filter (8-tap)
    // ========================================================================
    lms_filter #(
        .TAPS(LMS_TAPS),
        .WIDTH(LMS_WIDTH)
    ) u_lms_filter (
        .clk(clk),
        .rst_n(rst_n),
        .data_in({{(LMS_WIDTH-ADC_BITS){1'b0}}, adc_data}),
        .data_valid(adc_valid),
        .start(lms_start),
        .data_out(lms_out),
        .out_valid(lms_valid),
        .busy(lms_busy)
    );
    
    // ========================================================================
    // Lifting-Scheme DWT Engine (3-level)
    // ========================================================================
    dwt_engine #(
        .LEVELS(DWT_LEVELS),
        .WIDTH(DWT_WIDTH)
    ) u_dwt_engine (
        .clk(clk),
        .rst_n(rst_n),
        .data_in({{(DWT_WIDTH-LMS_WIDTH){1'b0}}, lms_out}),
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
    // CORDIC Phase Extractor (12 iterations)
    // ========================================================================
    cordic_vectoring #(
        .ITERATIONS(CORDIC_ITERS),
        .WIDTH(CORDIC_WIDTH)
    ) u_cordic (
        .clk(clk),
        .rst_n(rst_n),
        .start(cordic_start),
        .x_in_0(dwt_out_0),
        .x_in_1(dwt_out_1),
        .x_in_2(dwt_out_2),
        .x_in_3(dwt_out_3),
        .x_in_4(dwt_out_4),
        .x_in_5(dwt_out_5),
        .x_in_6(dwt_out_6),
        .x_in_7(dwt_out_7),
        .mag_out_0(cordic_mag_0),
        .mag_out_1(cordic_mag_1),
        .mag_out_2(cordic_mag_2),
        .mag_out_3(cordic_mag_3),
        .mag_out_4(cordic_mag_4),
        .mag_out_5(cordic_mag_5),
        .mag_out_6(cordic_mag_6),
        .mag_out_7(cordic_mag_7),
        .out_valid(cordic_valid),
        .busy(cordic_busy)
    );
    
    // ========================================================================
    // Power Bin Accumulator (8 bins)
    // ========================================================================
    power_accumulator #(
        .NUM_BINS(NUM_BINS),
        .WIDTH(16)
    ) u_power_acc (
        .clk(clk),
        .rst_n(rst_n),
        .start(acc_start),
        .mag_in_0({{4{1'b0}}, cordic_mag_0}),
        .mag_in_1({{4{1'b0}}, cordic_mag_1}),
        .mag_in_2({{4{1'b0}}, cordic_mag_2}),
        .mag_in_3({{4{1'b0}}, cordic_mag_3}),
        .mag_in_4({{4{1'b0}}, cordic_mag_4}),
        .mag_in_5({{4{1'b0}}, cordic_mag_5}),
        .mag_in_6({{4{1'b0}}, cordic_mag_6}),
        .mag_in_7({{4{1'b0}}, cordic_mag_7}),
        .bin_0(power_bins_0),
        .bin_1(power_bins_1),
        .bin_2(power_bins_2),
        .bin_3(power_bins_3),
        .bin_4(power_bins_4),
        .bin_5(power_bins_5),
        .bin_6(power_bins_6),
        .bin_7(power_bins_7),
        .out_valid(acc_valid)
    );
    
    // ========================================================================
    // 3-bit Command Encoder
    // ========================================================================
    command_encoder #(
        .NUM_BINS(NUM_BINS),
        .CMD_WIDTH(CMD_WIDTH)
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
        .tx_start(state == S_LSK_TX),
        .lsk_ctrl(lsk_ctrl),
        .tx_active(lsk_tx)
    );
    
    // Output assignments
    assign cmd_out   = cmd_encoded;
    assign cmd_valid = cmd_ready;

endmodule

// ============================================================================
// LMS Artifact Filter Module
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

    // Shift register for delay line
    reg [WIDTH-1:0] delay_line [0:TAPS-1];
    
    // Fixed coefficients (power-of-2 for shift-add)
    // Example: [0.5, 0.25, 0.125, 0.0625, 0.0625, 0.125, 0.25, 0.5]
    // Implemented as right-shifts: [1, 2, 3, 4, 4, 3, 2, 1]
    
    reg [2:0] tap_count;
    reg       processing;
    integer   i;
    
    assign busy = processing;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap_count <= 0;
            processing <= 0;
            out_valid <= 0;
            data_out <= 0;
        end else begin
            out_valid <= 0;
            
            if (data_valid) begin
                // Shift in new data
                delay_line[0] <= data_in;
                for (i = 1; i < TAPS; i = i + 1)
                    delay_line[i] <= delay_line[i-1];
            end
            
            if (start && !processing) begin
                processing <= 1;
                tap_count <= 0;
                data_out <= 0;
            end
            
            if (processing) begin
                // Accumulate filtered output (shift-add implementation)
                case (tap_count)
                    0: data_out <= data_out + (delay_line[0] >>> 1);  // 0.5
                    1: data_out <= data_out + (delay_line[1] >>> 2);  // 0.25
                    2: data_out <= data_out + (delay_line[2] >>> 3);  // 0.125
                    3: data_out <= data_out + (delay_line[3] >>> 4);  // 0.0625
                    4: data_out <= data_out + (delay_line[4] >>> 4);  // 0.0625
                    5: data_out <= data_out + (delay_line[5] >>> 3);  // 0.125
                    6: data_out <= data_out + (delay_line[6] >>> 2);  // 0.25
                    7: data_out <= data_out + (delay_line[7] >>> 1);  // 0.5
                endcase
                
                if (tap_count == TAPS[2:0] - 3'd1) begin
                    processing <= 0;
                    out_valid <= 1;
                end else begin
                    tap_count <= tap_count + 1;
                end
            end
        end
    end

endmodule

// ============================================================================
// Lifting-Scheme DWT Engine Module
// ============================================================================
module dwt_engine #(
    parameter LEVELS = 3,  /* verilator lint_off UNUSEDPARAM */
    parameter WIDTH  = 12
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [WIDTH-1:0]     data_in,
    input  wire                 start,
    output reg  [WIDTH-1:0]     subband_0,
    output reg  [WIDTH-1:0]     subband_1,
    output reg  [WIDTH-1:0]     subband_2,
    output reg  [WIDTH-1:0]     subband_3,
    output reg  [WIDTH-1:0]     subband_4,
    output reg  [WIDTH-1:0]     subband_5,
    output reg  [WIDTH-1:0]     subband_6,
    output reg  [WIDTH-1:0]     subband_7,
    output reg                  out_valid,
    output wire                 busy
);

    reg [2:0] level;
    reg       processing;
    
    // Temporary storage for decomposition
    reg [WIDTH-1:0] approx, detail;
    
    assign busy = processing;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            level <= 0;
            processing <= 0;
            out_valid <= 0;
            subband_0 <= 0;
            subband_1 <= 0;
            subband_2 <= 0;
            subband_3 <= 0;
            subband_4 <= 0;
            subband_5 <= 0;
            subband_6 <= 0;
            subband_7 <= 0;
        end else begin
            out_valid <= 0;
            
            if (start && !processing) begin
                processing <= 1;
                level <= 0;
                approx <= data_in;
            end
            
            if (processing) begin
                case (level)
                    0: begin
                        // Level 1: Split into approx and detail
                        detail <= approx - (approx >>> 1);
                        approx <= approx >>> 1;
                        subband_1 <= detail;
                        level <= 1;
                    end
                    1: begin
                        // Level 2: Split approx again
                        detail <= approx - (approx >>> 1);
                        approx <= approx >>> 1;
                        subband_2 <= detail;
                        level <= 2;
                    end
                    2: begin
                        // Level 3: Final split
                        detail <= approx - (approx >>> 1);
                        approx <= approx >>> 1;
                        subband_3 <= detail;
                        subband_0 <= approx;  // Final approximation
                        
                        // Populate remaining subbands (simplified)
                        subband_4 <= subband_1 >>> 1;
                        subband_5 <= subband_2 >>> 1;
                        subband_6 <= subband_3 >>> 1;
                        subband_7 <= approx >>> 2;
                        
                        level <= 3;
                    end
                    3: begin
                        processing <= 0;
                        out_valid <= 1;
                    end
                endcase
            end
        end
    end

endmodule

// ============================================================================
// CORDIC Vectoring Mode Module
// ============================================================================
module cordic_vectoring #(
    parameter ITERATIONS = 12,
    parameter WIDTH      = 12
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 start,
    input  wire [WIDTH-1:0]     x_in_0,
    input  wire [WIDTH-1:0]     x_in_1,
    input  wire [WIDTH-1:0]     x_in_2,
    input  wire [WIDTH-1:0]     x_in_3,
    input  wire [WIDTH-1:0]     x_in_4,
    input  wire [WIDTH-1:0]     x_in_5,
    input  wire [WIDTH-1:0]     x_in_6,
    input  wire [WIDTH-1:0]     x_in_7,
    output reg  [WIDTH-1:0]     mag_out_0,
    output reg  [WIDTH-1:0]     mag_out_1,
    output reg  [WIDTH-1:0]     mag_out_2,
    output reg  [WIDTH-1:0]     mag_out_3,
    output reg  [WIDTH-1:0]     mag_out_4,
    output reg  [WIDTH-1:0]     mag_out_5,
    output reg  [WIDTH-1:0]     mag_out_6,
    output reg  [WIDTH-1:0]     mag_out_7,
    output reg                  out_valid,
    output wire                 busy
);

    reg [3:0] iter_count;
    reg [2:0] channel;
    reg       processing;
    
    reg [WIDTH-1:0] x_temp, y_temp;
    
    assign busy = processing;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            iter_count <= 0;
            channel <= 0;
            processing <= 0;
            out_valid <= 0;
        end else begin
            out_valid <= 0;
            
            if (start && !processing) begin
                processing <= 1;
                iter_count <= 0;
                channel <= 0;
                x_temp <= x_in_0;
                y_temp <= x_in_0 >>> 2;  // Simplified initial condition
            end
            
            if (processing) begin
                if (iter_count < ITERATIONS) begin
                    // CORDIC iteration (simplified)
                    if (y_temp[WIDTH-1]) begin  // y < 0
                        x_temp <= x_temp - (y_temp >>> iter_count);
                        y_temp <= y_temp + (x_temp >>> iter_count);
                    end else begin
                        x_temp <= x_temp + (y_temp >>> iter_count);
                        y_temp <= y_temp - (x_temp >>> iter_count);
                    end
                    iter_count <= iter_count + 1;
                end else begin
                    // Store magnitude result
                    case (channel)
                        0: mag_out_0 <= x_temp;
                        1: mag_out_1 <= x_temp;
                        2: mag_out_2 <= x_temp;
                        3: mag_out_3 <= x_temp;
                        4: mag_out_4 <= x_temp;
                        5: mag_out_5 <= x_temp;
                        6: mag_out_6 <= x_temp;
                        7: mag_out_7 <= x_temp;
                    endcase
                    
                    if (channel == 7) begin
                        processing <= 0;
                        out_valid <= 1;
                    end else begin
                        channel <= channel + 1;
                        iter_count <= 0;
                        // Load next channel data
                        case (channel + 1)
                            1: x_temp <= x_in_1;
                            2: x_temp <= x_in_2;
                            3: x_temp <= x_in_3;
                            4: x_temp <= x_in_4;
                            5: x_temp <= x_in_5;
                            6: x_temp <= x_in_6;
                            7: x_temp <= x_in_7;
                        endcase
                    end
                end
            end
        end
    end

endmodule

// ============================================================================
// Power Bin Accumulator Module
// ============================================================================
module power_accumulator #(
    parameter NUM_BINS = 8,  /* verilator lint_off UNUSEDPARAM */
    parameter WIDTH    = 16
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 start,
    input  wire [WIDTH-1:0]     mag_in_0,
    input  wire [WIDTH-1:0]     mag_in_1,
    input  wire [WIDTH-1:0]     mag_in_2,
    input  wire [WIDTH-1:0]     mag_in_3,
    input  wire [WIDTH-1:0]     mag_in_4,
    input  wire [WIDTH-1:0]     mag_in_5,
    input  wire [WIDTH-1:0]     mag_in_6,
    input  wire [WIDTH-1:0]     mag_in_7,
    output reg  [WIDTH-1:0]     bin_0,
    output reg  [WIDTH-1:0]     bin_1,
    output reg  [WIDTH-1:0]     bin_2,
    output reg  [WIDTH-1:0]     bin_3,
    output reg  [WIDTH-1:0]     bin_4,
    output reg  [WIDTH-1:0]     bin_5,
    output reg  [WIDTH-1:0]     bin_6,
    output reg  [WIDTH-1:0]     bin_7,
    output reg                  out_valid
);

    reg processing;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bin_0 <= 0;
            bin_1 <= 0;
            bin_2 <= 0;
            bin_3 <= 0;
            bin_4 <= 0;
            bin_5 <= 0;
            bin_6 <= 0;
            bin_7 <= 0;
            out_valid <= 0;
            processing <= 0;
        end else begin
            out_valid <= 0;
            
            if (start && !processing) begin
                processing <= 1;
                // Compute power (magnitude squared approximation)
                bin_0 <= (mag_in_0 * mag_in_0) >>> 8;
                bin_1 <= (mag_in_1 * mag_in_1) >>> 8;
                bin_2 <= (mag_in_2 * mag_in_2) >>> 8;
                bin_3 <= (mag_in_3 * mag_in_3) >>> 8;
                bin_4 <= (mag_in_4 * mag_in_4) >>> 8;
                bin_5 <= (mag_in_5 * mag_in_5) >>> 8;
                bin_6 <= (mag_in_6 * mag_in_6) >>> 8;
                bin_7 <= (mag_in_7 * mag_in_7) >>> 8;
            end else if (processing) begin
                processing <= 0;
                out_valid <= 1;
            end
        end
    end

endmodule

// ============================================================================
// Command Encoder Module
// ============================================================================
module command_encoder #(
    parameter NUM_BINS  = 8,  /* verilator lint_off UNUSEDPARAM */
    parameter CMD_WIDTH = 3
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [15:0]          bin_0,
    input  wire [15:0]          bin_1,
    input  wire [15:0]          bin_2,
    input  wire [15:0]          bin_3,
    input  wire [15:0]          bin_4,
    input  wire [15:0]          bin_5,
    input  wire [15:0]          bin_6,
    input  wire [15:0]          bin_7,
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
                    4'd1: begin 
                        if (bin_1 > max_bin) begin 
                            max_bin <= bin_1; max_idx <= 3'd1; 
                        end 
                    end
                    4'd2: begin 
                        if (bin_2 > max_bin) begin 
                            max_bin <= bin_2; max_idx <= 3'd2; 
                        end 
                    end
                    4'd3: begin 
                        if (bin_3 > max_bin) begin 
                            max_bin <= bin_3; max_idx <= 3'd3; 
                        end 
                    end
                    4'd4: begin 
                        if (bin_4 > max_bin) begin 
                            max_bin <= bin_4; max_idx <= 3'd4; 
                        end 
                    end
                    4'd5: begin 
                        if (bin_5 > max_bin) begin 
                            max_bin <= bin_5; max_idx <= 3'd5; 
                        end 
                    end
                    4'd6: begin 
                        if (bin_6 > max_bin) begin 
                            max_bin <= bin_6; max_idx <= 3'd6; 
                        end 
                    end
                    4'd7: begin 
                        if (bin_7 > max_bin) begin 
                            max_bin <= bin_7; max_idx <= 3'd7; 
                        end 
                    end
                    // NEW: extra cycle to latch the final result
                    4'd8: begin
                        cmd_out   <= max_idx;
                        cmd_ready <= 1;
                        scanning  <= 0;
                    end
                    default: begin end
                endcase

                if (scan_idx <= 4'd8) begin
                    scan_idx <= scan_idx + 4'd1;
                end
            end
        end
    end

endmodule

// ============================================================================
// LSK Modulator Module
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

    // ================================================================
    // Packet format (MSB transmitted first):
    //   [1010]    4-bit preamble   — receiver clock sync
    //   [1100]    4-bit sync word  — marks start of data
    //   [c2 c1 c0] 3-bit command  — payload
    //   [p]       1-bit parity    — XOR of command bits
    //   [11]      2-bit postamble — clean end marker
    //   Total: 14 bits
    // ================================================================

    localparam PACKET_LEN   = 14;
    localparam HALF_PERIOD  = BIT_PERIOD / 2;

    // Packet shift register and control
    reg [PACKET_LEN-1:0] packet_reg;
    reg [3:0]            bit_count;     // counts down from PACKET_LEN-1
    reg [10:0]           bit_timer;
    reg                  transmitting;

    // Parity computation
    wire parity = cmd_in[2] ^ cmd_in[1] ^ cmd_in[0];

    // Full packet assembly
    wire [PACKET_LEN-1:0] full_packet = {
        4'b1010,     // preamble
        4'b1100,     // sync word
        cmd_in,      // 3-bit command (MSB first)
        parity,      // parity bit
        2'b11        // postamble
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lsk_ctrl     <= 1'b0;
            tx_active    <= 1'b0;
            transmitting <= 1'b0;
            packet_reg   <= {PACKET_LEN{1'b0}};
            bit_count    <= 4'd0;
            bit_timer    <= 11'd0;
        end else begin

            if (tx_start && !transmitting) begin
                // Load packet and begin transmission
                transmitting <= 1'b1;
                tx_active    <= 1'b1;
                packet_reg   <= full_packet;
                bit_count    <= PACKET_LEN[3:0] - 4'd1;  // start at MSB
                bit_timer    <= 11'd0;

                // Drive first half of first bit immediately
                lsk_ctrl <= full_packet[PACKET_LEN-1];
            end

            if (transmitting) begin
                bit_timer <= bit_timer + 11'd1;

                // Manchester encoding:
                //   First half of bit period:  output = data bit
                //   Second half of bit period: output = ~data bit
                if (bit_timer < HALF_PERIOD[10:0]) begin
                    lsk_ctrl <= packet_reg[bit_count];
                end else begin
                    lsk_ctrl <= ~packet_reg[bit_count];
                end

                // End of bit period
                if (bit_timer == BIT_PERIOD[10:0] - 11'd1) begin
                    bit_timer <= 11'd0;

                    if (bit_count == 4'd0) begin
                        // All bits transmitted
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

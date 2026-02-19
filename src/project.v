/*
 * Copyright (c) 2024 Design Team
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_NeuroCore (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // ---- Input mapping ----
    // ui_in[3:0] = adc_data  (4-bit ADC level code)
    // ui_in[4]   = adc_valid (ADC data valid pulse)
    // ui_in[5]   = wake      (wake pulse from comparator)
    // ui_in[7:6] = unused

    // ---- Output mapping ----
    // uo_out[2:0] = cmd_out       (3-bit command)
    // uo_out[3]   = cmd_valid
    // uo_out[4]   = lsk_ctrl      (LSK MOSFET control)
    // uo_out[5]   = lsk_tx        (LSK transmission active)
    // uo_out[6]   = pwr_gate_ctrl (power gate control)
    // uo_out[7]   = processing    (any block active)

    // ---- Bidirectional mapping (outputs) ----
    // uio_out[0] = lms_busy
    // uio_out[1] = dwt_busy
    // uio_out[2] = cordic_busy
    // uio_out[7:3] = 0

    // Internal wires
    wire [2:0] cmd_out;
    wire       cmd_valid;
    wire       lsk_ctrl;
    wire       lsk_tx;
    wire       pwr_gate_ctrl;
    wire       lms_busy;
    wire       dwt_busy;
    wire       cordic_busy;
    wire       processing;

    neurocore_field_sensor sensor (
        .clk          (clk),
        .rst_n        (rst_n),

        // ADC interface
        .adc_data     (ui_in[3:0]),
        .adc_valid    (ui_in[4]),

        // Event detection
        .wake         (ui_in[5]),

        // Command output
        .cmd_out      (cmd_out),
        .cmd_valid    (cmd_valid),

        // LSK modulator
        .lsk_ctrl     (lsk_ctrl),
        .lsk_tx       (lsk_tx),

        // Power gating
        .pwr_gate_ctrl(pwr_gate_ctrl),

        // Status
        .lms_busy     (lms_busy),
        .dwt_busy     (dwt_busy),
        .cordic_busy  (cordic_busy),
        .processing   (processing)
    );

    // Dedicated outputs
    assign uo_out[2:0] = cmd_out;
    assign uo_out[3]   = cmd_valid;
    assign uo_out[4]   = lsk_ctrl;
    assign uo_out[5]   = lsk_tx;
    assign uo_out[6]   = pwr_gate_ctrl;
    assign uo_out[7]   = processing;

    // Bidirectional outputs — expose status signals for debug
    assign uio_out[0]  = lms_busy;
    assign uio_out[1]  = dwt_busy;
    assign uio_out[2]  = cordic_busy;
    assign uio_out[7:3] = 5'b0;

    // uio pins 0-2 are outputs, rest are inputs
    assign uio_oe = 8'b0000_0111;

    // Suppress warnings for unused inputs
    wire _unused = &{ena, ui_in[7:6], uio_in, 1'b0};

endmodule

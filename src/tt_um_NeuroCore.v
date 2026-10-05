/*
 * Copyright (c) 2024-2026 Christoph Kassir, Ben Liu, Daniel Zheng, Michael Lander
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
/* verilator lint_off DECLFILENAME */

// Tiny Tapeout wrapper around neurocore_field_sensor.
//
// Pin map
// -------
//   ui_in[3:0]    adc_data      4-bit ADC level code (two's complement)
//   ui_in[4]      adc_valid     ADC sample strobe, 1 clk wide
//   ui_in[5]      wake          wake pulse from an external threshold comparator
//   ui_in[7:6]    unused
//
//   uio_in[5:3]   target_idx    frequency bin the external TMS should match
//   uio_in[7:6],
//   uio_in[2:0]   unused
//
//   uo_out[2:0]   cmd_out       3-bit command
//   uo_out[3]     cmd_valid     2-clk pulse; cmd_out holds until the next command
//   uo_out[4]     lsk_ctrl      LSK modulator MOSFET gate
//   uo_out[5]     lsk_tx        LSK packet in flight
//   uo_out[6]     pwr_gate_ctrl high while the pipeline is active (for an external switch)
//   uo_out[7]     processing    pipeline active
//
//   uio_out[0]    fir_busy      (output)
//   uio_out[1]    dwt_busy      (output)
//   uio_out[2]    reserved      (output, driven low)
//   uio_out[7:3]  not driven    (inputs; see uio_in above)

module tt_um_NeuroCore (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: input path
    output wire [7:0] uio_out,  // IOs: output path
    output wire [7:0] uio_oe,   // IOs: output enable (1 = output, 0 = input)
    input  wire       ena,      // High whenever the design is powered; unused
    input  wire       clk,      // Clock
    input  wire       rst_n     // Active-low reset
);

    wire [2:0] cmd_out;
    wire       cmd_valid;
    wire       lsk_ctrl;
    wire       lsk_tx;
    wire       pwr_gate_ctrl;
    wire       fir_busy;
    wire       dwt_busy;
    wire       processing;

    neurocore_field_sensor sensor (
        .clk           (clk),
        .rst_n         (rst_n),

        .adc_data      (ui_in[3:0]),
        .adc_valid     (ui_in[4]),
        .wake          (ui_in[5]),
        .target_idx    (uio_in[5:3]),

        .cmd_out       (cmd_out),
        .cmd_valid     (cmd_valid),
        .lsk_ctrl      (lsk_ctrl),
        .lsk_tx        (lsk_tx),
        .pwr_gate_ctrl (pwr_gate_ctrl),

        .fir_busy      (fir_busy),
        .dwt_busy      (dwt_busy),
        .processing    (processing)
    );

    assign uo_out = {processing, pwr_gate_ctrl, lsk_tx, lsk_ctrl, cmd_valid, cmd_out};

    // uio[2:0] are outputs: {reserved = 0, dwt_busy, fir_busy}. uio[7:3] are inputs.
    assign uio_out = {5'b0, 1'b0, dwt_busy, fir_busy};
    assign uio_oe  = 8'b0000_0111;

    // Unused inputs (keeps lint quiet)
    wire _unused = &{ena, ui_in[7:6], uio_in[7:6], uio_in[2:0], 1'b0};

endmodule

`default_nettype wire
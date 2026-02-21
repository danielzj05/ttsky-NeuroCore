// tb.v - Testbench wrapper for cocotb
`default_nettype none
`timescale 1ns / 1ps

module tb ();

    // Dump waveforms
    initial begin
        $dumpfile("tb.vcd");
        $dumpvars(0, tb);
        #1;
    end

    // Clock and reset
    reg clk;
    reg rst_n;
    reg ena;

    // Inputs
    reg  [7:0] ui_in;
    reg  [7:0] uio_in;

    // Outputs
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // Instantiate the Tiny Tapeout wrapper
    tt_um_NeuroCore u_dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .ena     (ena),
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe)
    );

endmodule
`default_nettype wire
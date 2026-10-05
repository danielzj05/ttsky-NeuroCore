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

`ifdef GL_TEST
    // Gate-level only: the powered sky130 cell models return X unless
    // VPWR = 1 and VGND = 0, so the testbench must drive them.
    wire VPWR = 1'b1;
    wire VGND = 1'b0;
`endif

    // Instantiate the Tiny Tapeout wrapper.
    // Instance name must stay u_dut: test.py uses dut.u_dut in RTL mode.
    tt_um_NeuroCore u_dut (
`ifdef GL_TEST
        .VPWR    (VPWR),
        .VGND    (VGND),
`endif
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
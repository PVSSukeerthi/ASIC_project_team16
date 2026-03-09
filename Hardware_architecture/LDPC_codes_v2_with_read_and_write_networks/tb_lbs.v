// =============================================================================
// tb_lbs.v  —  Testbench for Left Barrel Shifter
// =============================================================================
`timescale 1ns/1ps
`include "pkg_iams.vh"

module tb_lbs;

    localparam Z   = `LIFTING_Z;   // 52
    localparam QW  = `QTW;         // 8
    localparam SW  = 6;

    reg  [Z*QW-1:0] data_in;
    reg  [SW-1:0]   shift;
    wire [Z*QW-1:0] data_out;

    // DUT
    lbs #(.Z(Z), .QW(QW), .SW(SW)) dut (
        .data_in  (data_in),
        .shift    (shift),
        .data_out (data_out)
    );

    // Golden model
    function automatic [Z*QW-1:0] ref_lbs;
        input [Z*QW-1:0] din;
        input [SW-1:0]   sh;
        integer fe;
        reg [SW:0] raw;
        begin
            for (fe = 0; fe < Z; fe = fe + 1) begin
                raw = fe + sh;
                if (raw >= Z) raw = raw - Z;
                ref_lbs[fe*QW +: QW] = din[raw*QW +: QW];
            end
        end
    endfunction

    integer i, s, errors;
    reg [Z*QW-1:0] expected;

    initial begin
        errors = 0;
        $display("=== LBS Testbench (Z=%0d, QW=%0d) ===", Z, QW);

        // Test 1: shift = 0 — identity
        data_in = 0;
        for (i = 0; i < Z; i = i + 1)
            data_in[i*QW +: QW] = i[QW-1:0] + 8'd1;
        shift = 6'd0;
        #1;
        expected = ref_lbs(data_in, shift);
        if (data_out !== expected) begin
            $display("FAIL shift=0: got %h exp %h", data_out, expected);
            errors = errors + 1;
        end else
            $display("PASS shift=0");

        // Test 2: shift = 1
        shift = 6'd1; #1;
        expected = ref_lbs(data_in, shift);
        if (data_out !== expected) begin
            $display("FAIL shift=1"); errors = errors + 1;
        end else $display("PASS shift=1");

        // Test 3: shift = Z-1
        shift = Z - 1; #1;
        expected = ref_lbs(data_in, shift);
        if (data_out !== expected) begin
            $display("FAIL shift=Z-1"); errors = errors + 1;
        end else $display("PASS shift=Z-1");

        // Test 4: shift = Z/2
        shift = Z / 2; #1;
        expected = ref_lbs(data_in, shift);
        if (data_out !== expected) begin
            $display("FAIL shift=Z/2"); errors = errors + 1;
        end else $display("PASS shift=Z/2");

        // Test 5: random inputs and shifts
        for (s = 0; s < 200; s = s + 1) begin
            data_in = $random;
            data_in[Z*QW-1 -: (Z*QW - (Z/8)*8*QW)] = 0; // clear upper garbage
            shift   = $urandom % Z;
            #1;
            expected = ref_lbs(data_in, shift);
            if (data_out !== expected) begin
                $display("FAIL random s=%0d shift=%0d", s, shift);
                errors = errors + 1;
            end
        end
        $display("Random: 200 tests");

        if (errors == 0)
            $display("ALL PASSED (%0d errors)", errors);
        else
            $display("FAILED: %0d errors", errors);
        $finish;
    end

endmodule

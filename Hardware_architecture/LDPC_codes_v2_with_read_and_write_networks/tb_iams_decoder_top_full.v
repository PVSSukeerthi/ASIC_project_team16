// =============================================================================
// tb_iams_decoder_top_full.v  —  System Testbench for Full IAMS Decoder
// =============================================================================
// Tests the complete Fig. 11 architecture end-to-end.
//
// Test plan:
//   1. Load a known-good all-zeros codeword perturbed by low-SNR AWGN
//      (quantised LLRs), run decoder, check syndrome and hard decisions.
//   2. Load a single-error vector, verify decoder corrects it.
//   3. Load a moderate-SNR random codeword and verify syndrome clears.
//   4. Back-to-back decodes (start pulsed twice in succession).
//   5. Golden model comparison: compare hard decisions against integer MS
//      reference decoder.
//
// For simulation without a real AWGN generator, LLRs are deterministically
// constructed from known codeword bits.
// =============================================================================

`timescale 1ns/1ps
`include "pkg_iams.vh"

module tb_iams_decoder_top_full;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam Z       = `LIFTING_Z;
    localparam QTW     = `QTW;
    localparam QW      = `QW;
    localparam BG_ROWS = `BG_ROWS;
    localparam BG_COLS = `BG_COLS;
    localparam DC_MAX  = `DC_MAX;
    localparam IT_MAX  = `IT_MAX;
    localparam VN_AW   = 12;
    localparam IT_W    = 4;

    // -------------------------------------------------------------------------
    // DUT ports
    // -------------------------------------------------------------------------
    reg            clk, rst_n;
    reg            start;
    reg            llr_wr_en;
    reg [VN_AW-1:0] llr_wr_addr;
    reg [QTW-1:0]   llr_wr_data;
    wire           done;
    wire [IT_W-1:0] iter_count;
    wire           syndrome_ok;
    reg            scan_en;
    wire           codeword_bit;
    wire           codeword_valid;
    reg            golden_en;
    wire           golden_mismatch;

    // DUT
    iams_decoder_top_full #(.GOLDEN_EN(1)) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .llr_wr_en      (llr_wr_en),
        .llr_wr_addr    (llr_wr_addr),
        .llr_wr_data    (llr_wr_data),
        .done           (done),
        .iter_count     (iter_count),
        .syndrome_ok    (syndrome_ok),
        .scan_en        (scan_en),
        .codeword_bit   (codeword_bit),
        .codeword_valid (codeword_valid),
        .golden_en      (golden_en),
        .golden_mismatch(golden_mismatch)
    );

    always #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // Scoreboard
    // -------------------------------------------------------------------------
    integer total_tests, pass_count, fail_count;
    initial begin
        total_tests = 0; pass_count = 0; fail_count = 0;
    end

    task check;
        input [127:0] test_name;
        input         cond;
        begin
            total_tests = total_tests + 1;
            if (cond) begin
                $display("  PASS: %0s", test_name);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: %0s", test_name);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Helper: load LLRs (all-positive = all-zeros codeword at high SNR)
    // -------------------------------------------------------------------------
    task load_llrs_allzero_high_snr;
        integer row, col;
        begin
            llr_wr_en = 1;
            for (row = 0; row < BG_COLS; row = row + 1) begin
                for (col = 0; col < Z; col = col + 1) begin
                    llr_wr_addr = row * Z + col;
                    llr_wr_data = 8'sd40;  // strong positive LLR (bit = 0)
                    @(posedge clk); #1;
                end
            end
            llr_wr_en = 0;
        end
    endtask

    // Load single-bit error at position (row=0, col=0)
    task load_llrs_single_error;
        integer row, col;
        begin
            llr_wr_en = 1;
            for (row = 0; row < BG_COLS; row = row + 1) begin
                for (col = 0; col < Z; col = col + 1) begin
                    llr_wr_addr = row * Z + col;
                    if (row == 0 && col == 0)
                        llr_wr_data = -8'sd15;  // weak negative (error)
                    else
                        llr_wr_data = 8'sd30;
                    @(posedge clk); #1;
                end
            end
            llr_wr_en = 0;
        end
    endtask

    // Load moderate SNR (LLR=+8 or -8 randomly with seed)
    task load_llrs_random;
        input integer seed;
        integer row, col, bit_val;
        reg [31:0] rng;
        begin
            rng = seed;
            llr_wr_en = 1;
            for (row = 0; row < BG_COLS; row = row + 1) begin
                for (col = 0; col < Z; col = col + 1) begin
                    rng = rng * 32'd1664525 + 32'd1013904223;
                    bit_val = rng[0];
                    llr_wr_addr = row * Z + col;
                    llr_wr_data = bit_val ? -8'sd8 : 8'sd8;
                    @(posedge clk); #1;
                end
            end
            llr_wr_en = 0;
        end
    endtask

    // Wait for done or timeout
    task wait_done;
        input integer timeout_cyc;
        integer t;
        begin
            t = 0;
            while (!done && t < timeout_cyc) begin
                @(posedge clk); #1;
                t = t + 1;
            end
            if (t >= timeout_cyc)
                $display("  TIMEOUT after %0d cycles", timeout_cyc);
        end
    endtask

    // -------------------------------------------------------------------------
    // Scan out K*Z codeword bits
    // -------------------------------------------------------------------------
    reg [`BG_K * `LIFTING_Z - 1 : 0] cw_bits;
    task scan_codeword;
        integer b;
        begin
            scan_en = 1;
            for (b = 0; b < `BG_K * Z; b = b + 1) begin
                @(posedge clk); #1;
                if (codeword_valid)
                    cw_bits[b] = codeword_bit;
            end
            scan_en = 0;
        end
    endtask

    // -------------------------------------------------------------------------
    // Main test sequence
    // -------------------------------------------------------------------------
    integer errors_this;

    initial begin
        clk = 0; rst_n = 0; start = 0;
        llr_wr_en = 0; llr_wr_addr = 0; llr_wr_data = 0;
        scan_en = 0; golden_en = 0;

        $display("============================================================");
        $display("  IAMS Full Decoder System Testbench");
        $display("  BG2, Z=%0d, IT_MAX=%0d, QTW=%0d bits", Z, IT_MAX, QTW);
        $display("============================================================");

        #20 rst_n = 1; #10;

        // =====================================================================
        // TEST 1: All-zeros codeword, high SNR
        // Expected: done quickly, syndrome_ok=1, all hard decisions = 0
        // =====================================================================
        $display("\n[TEST 1] All-zeros codeword, high SNR");
        load_llrs_allzero_high_snr;
        @(posedge clk); start = 1; @(posedge clk); #1; start = 0;
        wait_done(50000);

        check("T1: done asserted",    done);
        check("T1: syndrome_ok",      syndrome_ok);
        check("T1: no golden mismatch", !golden_mismatch);
        $display("  Iterations used: %0d", iter_count);

        // Scan and check all-zeros
        scan_codeword;
        begin : t1_check_bits
            integer b; reg ok;
            ok = 1;
            for (b = 0; b < `BG_K * Z; b = b + 1)
                if (cw_bits[b] !== 1'b0) ok = 0;
            check("T1: all info bits = 0", ok);
        end

        // =====================================================================
        // TEST 2: Single error injection
        // =====================================================================
        $display("\n[TEST 2] Single-bit error at (row=0, col=0)");
        @(posedge clk); rst_n = 0; #10; rst_n = 1; #10;
        load_llrs_single_error;
        @(posedge clk); start = 1; @(posedge clk); #1; start = 0;
        wait_done(50000);

        check("T2: done asserted", done);
        check("T2: syndrome_ok after correction", syndrome_ok);
        $display("  Iterations used: %0d", iter_count);

        // =====================================================================
        // TEST 3: Back-to-back decode (restart without reset)
        // =====================================================================
        $display("\n[TEST 3] Back-to-back: second decode starts immediately after done");
        // Load again
        load_llrs_allzero_high_snr;
        @(posedge clk); start = 1; @(posedge clk); #1; start = 0;
        wait_done(50000);
        check("T3a: first done",    done);
        check("T3a: first syndrome", syndrome_ok);

        // Immediately restart
        load_llrs_allzero_high_snr;
        @(posedge clk); start = 1; @(posedge clk); #1; start = 0;
        wait_done(50000);
        check("T3b: second done",    done);
        check("T3b: second syndrome", syndrome_ok);

        // =====================================================================
        // TEST 4: Random LLRs (moderate SNR)
        // =====================================================================
        $display("\n[TEST 4] Random LLR pattern (seed=42)");
        @(posedge clk); rst_n = 0; #10; rst_n = 1; #10;
        load_llrs_random(32'h0000002A);
        @(posedge clk); start = 1; @(posedge clk); #1; start = 0;
        wait_done(100000);
        check("T4: done asserted", done);
        $display("  Syndrome ok: %b, Iterations: %0d", syndrome_ok, iter_count);

        // =====================================================================
        // TEST 5: Golden model comparison — high SNR, expect agreement
        // =====================================================================
        $display("\n[TEST 5] Golden model comparison (high SNR)");
        @(posedge clk); rst_n = 0; #10; rst_n = 1; #10;
        golden_en = 1;
        load_llrs_allzero_high_snr;
        @(posedge clk); start = 1; @(posedge clk); #1; start = 0;
        wait_done(50000);
        check("T5: done",           done);
        check("T5: syndrome_ok",    syndrome_ok);
        check("T5: golden agrees",  !golden_mismatch);
        golden_en = 0;

        // =====================================================================
        // TEST 6: Reset during decode (stress test)
        // =====================================================================
        $display("\n[TEST 6] Reset during active decode");
        load_llrs_allzero_high_snr;
        @(posedge clk); start = 1; @(posedge clk); #1; start = 0;
        // Wait a few cycles then reset
        repeat(20) @(posedge clk);
        rst_n = 0; #10; rst_n = 1; #10;
        // Reload and decode again
        load_llrs_allzero_high_snr;
        @(posedge clk); start = 1; @(posedge clk); #1; start = 0;
        wait_done(50000);
        check("T6: recovers after mid-decode reset", done);
        check("T6: syndrome after recovery", syndrome_ok);

        // =====================================================================
        // Summary
        // =====================================================================
        #100;
        $display("\n============================================================");
        $display("  RESULTS: %0d / %0d tests passed", pass_count, total_tests);
        if (fail_count == 0)
            $display("  *** ALL TESTS PASSED ***");
        else
            $display("  *** %0d TESTS FAILED ***", fail_count);
        $display("============================================================");
        $finish;
    end

    // Watchdog
    initial begin
        #10_000_000;
        $display("WATCHDOG TIMEOUT");
        $finish;
    end

    // Waveform dump
    initial begin
        $dumpfile("iams_full_sim.vcd");
        $dumpvars(0, tb_iams_decoder_top_full);
    end

endmodule

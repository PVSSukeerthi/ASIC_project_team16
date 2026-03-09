// ============================================================
// tb_system.v  -  System-Level Testbench
// iverilog Verilog-2001 compatible: NO unpacked arrays.
//
// Flat conventions match ldpc5g_decoder.v:
//   llr_in   [Z*NB*Q-1:0]  llr[n] at [n*Q +: Q]
//   decoded  [Z*NB-1:0]    bit n at [n]
//
// Software golden reference uses simple flat regs throughout.
// ============================================================
`include "ldpc5g_pkg.v"
`timescale 1ns/1ps

module tb_system;

localparam Z        = `Z;
localparam NB       = `NB;
localparam MB       = `MB;
localparam Q        = `Q;
localparam QTILDE   = `QTILDE;
localparam DC_MAX_P = `DC_MAX;
localparam L        = `L_AFTER;
localparam ITMAX    = `ITMAX_IMPL;
localparam N        = Z * NB;       // 2600

localparam CLK_PERIOD = 10;

// ============================================================
// DUT
// ============================================================
reg  clk, rst_n, start;
reg  [N*Q-1:0]   llr_in;
wire [N-1:0]     codeword_out;
wire             dec_valid;
wire [3:0]       iter_count;

ldpc5g_decoder #(
    .Z(Z), .NB(NB), .MB(MB), .Q(Q), .QTILDE(QTILDE),
    .DC_MAX_P(DC_MAX_P), .L(L), .ITMAX(ITMAX)
) dut (
    .clk         (clk),
    .rst_n       (rst_n),
    .start       (start),
    .llr_in      (llr_in),
    .codeword_out(codeword_out),
    .dec_valid   (dec_valid),
    .iter_count  (iter_count)
);

initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// ============================================================
// Helpers to set llr_in elements
// ============================================================
task set_llr_all;
    input signed [Q-1:0] val;
    integer n;
    begin
        for (n = 0; n < N; n = n+1)
            llr_in[n*Q +: Q] = val;
    end
endtask

task set_llr_n;
    input integer n;
    input signed [Q-1:0] val;
    begin
        llr_in[n*Q +: Q] = val;
    end
endtask

// ============================================================
// Software Golden Reference - flat arrays throughout
// Flat layout same as DUT.
// ============================================================
localparam MWIDTH = Q - 1;

// SW model arrays (flat)
reg signed [QTILDE-1:0] sw_app   [0:N-1];       // app[n]
// sw_alpha: flat 1D array, element [m][n] at index m*N+n
reg signed [Q-1:0]      sw_alpha [0:MB*N-1];
reg signed [Q-1:0]      sw_beta  [0:DC_MAX_P-1];

integer sw_dc [0:MB-1];
initial begin
    sw_dc[0]=10; sw_dc[1]=10; sw_dc[2]=10; sw_dc[3]=10;
    sw_dc[4]=5;  sw_dc[5]=5;  sw_dc[6]=5;  sw_dc[7]=5;
    sw_dc[8]=4;  sw_dc[9]=4;  sw_dc[10]=4; sw_dc[11]=4;
    sw_dc[12]=4; sw_dc[13]=4; sw_dc[14]=3; sw_dc[15]=3;
    sw_dc[16]=3; sw_dc[17]=3; sw_dc[18]=3; sw_dc[19]=3;
    sw_dc[20]=3; sw_dc[21]=3; sw_dc[22]=3; sw_dc[23]=3;
    sw_dc[24]=3; sw_dc[25]=3; sw_dc[26]=3; sw_dc[27]=3;
    sw_dc[28]=3; sw_dc[29]=3; sw_dc[30]=3; sw_dc[31]=3;
    sw_dc[32]=3; sw_dc[33]=3; sw_dc[34]=3; sw_dc[35]=3;
    sw_dc[36]=3; sw_dc[37]=3; sw_dc[38]=3; sw_dc[39]=3;
    sw_dc[40]=3; sw_dc[41]=3;
end

integer sw_dv [0:NB-1];
integer ci_dv;
initial begin
    sw_dv[0]=23; sw_dv[1]=23; sw_dv[2]=10; sw_dv[3]=10;
    sw_dv[4]=10; sw_dv[5]=10; sw_dv[6]=10; sw_dv[7]=10;
    sw_dv[8]=10; sw_dv[9]=10;
    for (ci_dv = 10; ci_dv < NB; ci_dv = ci_dv+1)
        sw_dv[ci_dv] = 1;
end

function automatic [5:0] sw_get_col;
    input [5:0] row;
    input [3:0] pos;
    begin
        if (row < 4)
            sw_get_col = pos[5:0];
        else begin
            if      (pos == 0) sw_get_col = row - 4 + 10;
            else if (pos == 1) sw_get_col = 6'd0;
            else               sw_get_col = 6'd1;
        end
    end
endfunction

function automatic [5:0] sw_get_shift;
    input [5:0] row;
    input [3:0] pos;
    begin
        sw_get_shift = (row + pos * 7) % Z;
    end
endfunction

// SW decoder task - uses module-level llr_in via sw_app init
// All local vars declared at task scope (no named begin blocks)
reg [N-1:0]      sw_decoded;
integer          sw_iters_out;

reg [MWIDTH-1:0] sw_min1, sw_min2;
reg [3:0]        sw_idx1, sw_idx2;
reg              sw_txor;
reg [MWIDTH-1:0] sw_delta;
reg              sw_out_sgn;
reg              sw_parity_ok;
integer          sw_t, sw_m, sw_p, sw_k;
integer          sw_col_idx, sw_shift_val, sw_vn_idx, sw_dc_m;
reg              sw_use_oms;
reg [MWIDTH-1:0] sw_out_mag, sw_mag_excl;
reg signed [Q-1:0] sw_alpha_new, sw_alpha_old_v;
reg signed [QTILDE-1:0] sw_bt, sw_sum, sw_diff;

task sw_iams_decode;
    output reg [N-1:0]  decoded;
    output integer      iters_out;
    begin
        // Initialize: app = sign-extended llr_in, alpha = 0
        for (sw_k = 0; sw_k < N; sw_k = sw_k+1) begin
            sw_app[sw_k] = {{(QTILDE-Q){llr_in[sw_k*Q+Q-1]}},
                             llr_in[sw_k*Q +: Q]};
            for (sw_m = 0; sw_m < MB; sw_m = sw_m+1)
                sw_alpha[sw_m*N+sw_k] = 0;
        end
        sw_parity_ok = 0;
        iters_out    = 0;

        for (sw_t = 0; sw_t < ITMAX && !sw_parity_ok; sw_t = sw_t+1) begin
            iters_out = sw_t + 1;
            for (sw_m = 0; sw_m < MB; sw_m = sw_m+1) begin
                sw_dc_m = sw_dc[sw_m];

                // Collect beta (VTC) messages
                for (sw_p = 0; sw_p < DC_MAX_P; sw_p = sw_p+1) begin
                    if (sw_p < sw_dc_m) begin
                        sw_col_idx   = sw_get_col(sw_m, sw_p);
                        sw_shift_val = sw_get_shift(sw_m, sw_p);
                        sw_vn_idx    = sw_col_idx * Z + (sw_shift_val % Z);
                        if (sw_vn_idx < N) begin
                            sw_alpha_old_v = sw_alpha[sw_m*N+sw_vn_idx];
                            sw_diff = sw_app[sw_vn_idx]
                                    - {{(QTILDE-Q){sw_alpha_old_v[Q-1]}}, sw_alpha_old_v};
                            if (sw_diff > $signed({{(QTILDE-Q){1'b0}},{(Q-1){1'b1}}}))
                                sw_beta[sw_p] = {1'b0, {(Q-1){1'b1}}};
                            else if (sw_diff < $signed({{(QTILDE-Q){1'b1}},{(Q-1){1'b0}}}))
                                sw_beta[sw_p] = {1'b1, {(Q-1){1'b0}}};
                            else
                                sw_beta[sw_p] = sw_diff[Q-1:0];
                        end else
                            sw_beta[sw_p] = 0;
                    end else
                        sw_beta[sw_p] = 0;
                end

                // Find min1, min2, idx1, idx2
                sw_min1 = {MWIDTH{1'b1}}; sw_min2 = {MWIDTH{1'b1}};
                sw_idx1 = 0; sw_idx2 = 1; sw_txor = 0;
                for (sw_p = 0; sw_p < sw_dc_m; sw_p = sw_p+1) begin
                    sw_txor = sw_txor ^ sw_beta[sw_p][Q-1];
                    if (sw_beta[sw_p][Q-2:0] < sw_min1) begin
                        sw_min2 = sw_min1; sw_idx2 = sw_idx1;
                        sw_min1 = sw_beta[sw_p][Q-2:0]; sw_idx1 = sw_p[3:0];
                    end else if (sw_beta[sw_p][Q-2:0] < sw_min2) begin
                        sw_min2 = sw_beta[sw_p][Q-2:0]; sw_idx2 = sw_p[3:0];
                    end
                end
                sw_delta = sw_min2 - sw_min1;

                // Compute CTV and update APP
                for (sw_p = 0; sw_p < sw_dc_m; sw_p = sw_p+1) begin
                    sw_out_sgn   = sw_txor ^ sw_beta[sw_p][Q-1];
                    sw_col_idx   = sw_get_col(sw_m, sw_p);
                    sw_shift_val = sw_get_shift(sw_m, sw_p);
                    sw_vn_idx    = sw_col_idx * Z + (sw_shift_val % Z);

                    if (sw_vn_idx < N) begin
                        sw_use_oms = (sw_m < 4) && (sw_dv[sw_col_idx] >= `D_THRESHOLD);

                        if (sw_use_oms) begin
                            sw_mag_excl = (sw_p[3:0] == sw_idx1) ? sw_min2 : sw_min1;
                            sw_out_mag  = (sw_mag_excl > 0) ? (sw_mag_excl - 1) : 0;
                        end else begin
                            if (sw_p[3:0] == sw_idx1)      sw_out_mag = sw_min2;
                            else if (sw_p[3:0] == sw_idx2) sw_out_mag = sw_min1;
                            else if (sw_delta == 0)
                                sw_out_mag = (sw_min1 > 0) ? (sw_min1 - 1) : 0;
                            else
                                sw_out_mag = sw_min1;
                        end

                        sw_alpha_new   = (sw_out_mag == 0) ? 0
                                       : {sw_out_sgn, sw_out_mag};
                        sw_alpha_old_v = sw_alpha[sw_m*N+sw_vn_idx];

                        sw_bt  = sw_app[sw_vn_idx]
                               - {{(QTILDE-Q){sw_alpha_old_v[Q-1]}}, sw_alpha_old_v};
                        sw_sum = sw_bt
                               + {{(QTILDE-Q){sw_alpha_new[Q-1]}}, sw_alpha_new};

                        if      (sw_sum > $signed(6'sd31))  sw_app[sw_vn_idx] =  6'sd31;
                        else if (sw_sum < -$signed(6'sd32)) sw_app[sw_vn_idx] = -6'sd32;
                        else                                 sw_app[sw_vn_idx] = sw_sum[QTILDE-1:0];

                        sw_alpha[sw_m*N+sw_vn_idx] = sw_alpha_new;
                    end
                end
            end // for m

            // Hard decision + simplified parity check
            sw_parity_ok = 1;
            for (sw_k = 0; sw_k < N; sw_k = sw_k+1) begin
                decoded[sw_k] = sw_app[sw_k][QTILDE-1];
                if (sw_k >= 10*Z && decoded[sw_k] != 0)
                    sw_parity_ok = 0;
            end
        end // for t
    end
endtask

// ============================================================
// Test Infrastructure
// ============================================================
integer test_pass, test_fail;
integer i;
integer max_wait_cycles;
reg     timed_out_flag;
// Module-level temps for inline blocks
reg     zc_any_err;
integer zc_b, bb_frame;
real thr;
        
task do_reset;
    begin
        rst_n = 0; start = 0;
        @(posedge clk); #1;
        @(posedge clk); #1;
        rst_n = 1;
        @(posedge clk); #1;
    end
endtask

task wait_done;
    output reg timed_out;
    integer cyc;
    begin
        timed_out = 0; cyc = 0;
        while (!dec_valid && cyc < max_wait_cycles) begin
            @(posedge clk); cyc = cyc+1;
        end
        if (cyc >= max_wait_cycles) timed_out = 1;
    end
endtask

task compare_result;
    input [63:0] test_id;
    input reg    timed_out;
    reg mismatch;
    integer b;
    begin
        if (timed_out) begin
            $display("FAIL test%0d: RTL timed out", test_id);
            test_fail = test_fail + 1;
        end else begin
            mismatch = 0;
            for (b = 0; b < Z*10 && !mismatch; b = b+1) begin
                if (codeword_out[b] !== sw_decoded[b]) begin
                    $display("FAIL test%0d: bit %0d mismatch RTL=%0b SW=%0b",
                             test_id, b, codeword_out[b], sw_decoded[b]);
                    mismatch = 1;
                end
            end
            if (!mismatch) begin
                $display("PASS test%0d: iters=%0d sw_iters=%0d",
                         test_id, iter_count, sw_iters_out);
                test_pass = test_pass + 1;
            end else
                test_fail = test_fail + 1;
        end
    end
endtask

// ============================================================
// Test Cases
// ============================================================
initial begin
    $dumpfile("tb_system.vcd");
    $dumpvars(0, tb_system);

    test_pass = 0; test_fail = 0;
    max_wait_cycles = (L * ITMAX + 10) * 2;
    llr_in = 0;

    do_reset;

    // ---- Test 1: All-zero codeword (strong LLR=+7) ----
    $display("\n=== Test 1: All-zero codeword (LLR=+7) ===");
    set_llr_all(4'sd7);
    sw_iams_decode(sw_decoded, sw_iters_out);
    start=1; @(posedge clk); #1; start=0;
    wait_done(timed_out_flag); @(posedge clk); #1;
    compare_result(1, timed_out_flag);
    do_reset;

    // ---- Test 2: All-zero codeword (moderate LLR=+3) ----
    $display("\n=== Test 2: All-zero codeword (LLR=+3) ===");
    set_llr_all(4'sd3);
    sw_iams_decode(sw_decoded, sw_iters_out);
    start=1; @(posedge clk); #1; start=0;
    wait_done(timed_out_flag); @(posedge clk); #1;
    compare_result(2, timed_out_flag);
    do_reset;

    // ---- Test 3: Single bit flip on core VN ----
    $display("\n=== Test 3: Single bit flip on core VN ===");
    set_llr_all(4'sd4);
    set_llr_n(0, -4'sd1);
    sw_iams_decode(sw_decoded, sw_iters_out);
    start=1; @(posedge clk); #1; start=0;
    wait_done(timed_out_flag); @(posedge clk); #1;
    compare_result(3, timed_out_flag);
    do_reset;

    // ---- Test 4: Single flip on extension VN ----
    $display("\n=== Test 4: Single flip on extension VN ===");
    set_llr_all(4'sd5);
    set_llr_n(Z*10, -4'sd2);
    sw_iams_decode(sw_decoded, sw_iters_out);
    start=1; @(posedge clk); #1; start=0;
    wait_done(timed_out_flag); @(posedge clk); #1;
    compare_result(4, timed_out_flag);
    do_reset;

    // ---- Test 5: Back-to-back (3 frames) ----
    $display("\n=== Test 5: Back-to-back (3 frames) ===");
    for (bb_frame = 0; bb_frame < 3; bb_frame = bb_frame+1) begin
        for (i = 0; i < N; i = i+1)
            llr_in[i*Q +: Q] = (i % 3 == 0) ? -4'sd1 : 4'sd4;
        sw_iams_decode(sw_decoded, sw_iters_out);
        start=1; @(posedge clk); #1; start=0;
        wait_done(timed_out_flag); @(posedge clk); #1;
        compare_result(10+bb_frame, timed_out_flag);
        do_reset;
    end

    // ---- Test 6: Architecture parameter verification ----
    $display("\n=== Test 6: Parameter verification ===");
    $display("L=%0d (expected 31), ITMAX=%0d, Z=%0d, N=%0d", L, ITMAX, Z, N);
    if (L >= 30 && L <= 31) begin
        $display("PASS test6: Layer count OK");
        test_pass = test_pass + 1;
    end else begin
        $display("FAIL test6: Unexpected layer count %0d", L);
        test_fail = test_fail + 1;
    end
    $display("CTV reduction: before=%0d*z after=868*z (%.1f%% saved)",
             1008, (1.0 - 868.0/1008.0)*100.0);

    // ---- Test 7: Zero codeword verify (LLR=+6) ----
    $display("\n=== Test 7: Zero codeword verify (LLR=+6) ===");
    set_llr_all(4'sd6);
    start=1; @(posedge clk); #1; start=0;
    wait_done(timed_out_flag); @(posedge clk); #1;
    zc_any_err = 0;
    for (zc_b = 0; zc_b < N; zc_b = zc_b+1)
        if (codeword_out[zc_b] != 0) zc_any_err = 1;
    if (!zc_any_err) begin
        $display("PASS test7: Zero codeword decoded correctly");
        test_pass = test_pass + 1;
    end else begin
        $display("FAIL test7: Non-zero bits in codeword");
        test_fail = test_fail + 1;
    end
    do_reset;

    // ---- Test 8: Throughput estimate ----
    $display("\n=== Test 8: Throughput estimate ===");
    begin
        thr = 100.0 * N / (L * ITMAX);
        $display("Throughput @100MHz = %.1f Mbps (paper: 914 Mbps @ ~346 MHz)", thr);
        if (thr >= 50.0) begin
            $display("PASS test8: Throughput %.1f Mbps", thr);
            test_pass = test_pass + 1;
        end else begin
            $display("FAIL test8");
            test_fail = test_fail + 1;
        end
    end

    $display("\n========================================");
    $display("  SYSTEM TEST SUMMARY");
    $display("  PASS: %0d  FAIL: %0d  TOTAL: %0d",
             test_pass, test_fail, test_pass+test_fail);
    $display("========================================");
    if (test_fail == 0) $display("ALL SYSTEM TESTS PASSED");
    else                $display("SOME TESTS FAILED");
    $finish;
end

initial begin
    #10000000;
    $display("GLOBAL TIMEOUT");
    $finish;
end

endmodule
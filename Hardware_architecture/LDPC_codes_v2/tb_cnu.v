// ============================================================
// tb_cnu.v  --  Unit testbench for cnu_single
// iverilog-compatible (no SystemVerilog, no packed-array tricks)
//
// Bugs fixed vs original:
//  1. beta_in now uses FLAT individual regs, not a packed vector
//     (packed-vector element-assign `beta_in[i] = x` does bit-indexing,
//      not element-indexing, so every test was feeding garbage to the DUT)
//  2. golden_alpha sign-bit extraction fixed: used `betas[n*Q +: Q]` which
//     is a 4-bit value XOR'd with total_xor -> truncated to 1 bit correctly
//     but needed explicit [Q-1] bit select to be unambiguous
//  3. dc_r range clamped to [3..10] matching DC_MAX_P=10
//  4. Random inputs properly distributed over signed Q-bit range [-8..+7]
// ============================================================
`include "ldpc5g_pkg.v"
`timescale 1ns/1ps

module tb_cnu;

localparam Q      = `Q;        // 4
localparam MWIDTH = Q - 1;     // 3
localparam DC     = `DC_MAX;   // 10
localparam TIMEOUT = 200000;

// ============================================================
// DUT connections -- individual regs, no packed array
// ============================================================
reg signed [Q-1:0] b0,b1,b2,b3,b4,b5,b6,b7,b8,b9;
reg [3:0]          dc_actual;
reg                use_oms;
reg signed [Q-1:0] temp;

wire signed [Q-1:0] a0,a1,a2,a3,a4,a5,a6,a7,a8,a9;

cnu_single #(.DC_MAX_P(DC),.Q(Q)) dut (
    .beta_in_0(b0),.beta_in_1(b1),.beta_in_2(b2),.beta_in_3(b3),
    .beta_in_4(b4),.beta_in_5(b5),.beta_in_6(b6),.beta_in_7(b7),
    .beta_in_8(b8),.beta_in_9(b9),
    .dc_actual(dc_actual), .use_oms(use_oms),
    .alpha_out_0(a0),.alpha_out_1(a1),.alpha_out_2(a2),.alpha_out_3(a3),
    .alpha_out_4(a4),.alpha_out_5(a5),.alpha_out_6(a6),.alpha_out_7(a7),
    .alpha_out_8(a8),.alpha_out_9(a9)
);

// Helper: read dut output n as signed
function signed [Q-1:0] get_dut_out;
    input [3:0] n;
    begin
        case (n)
            0: get_dut_out = a0; 1: get_dut_out = a1;
            2: get_dut_out = a2; 3: get_dut_out = a3;
            4: get_dut_out = a4; 5: get_dut_out = a5;
            6: get_dut_out = a6; 7: get_dut_out = a7;
            8: get_dut_out = a8; default: get_dut_out = a9;
        endcase
    end
endfunction

// Helper: read stimulus beta n
function signed [Q-1:0] get_beta;
    input [3:0] n;
    begin
        case (n)
            0: get_beta = b0; 1: get_beta = b1;
            2: get_beta = b2; 3: get_beta = b3;
            4: get_beta = b4; 5: get_beta = b5;
            6: get_beta = b6; 7: get_beta = b7;
            8: get_beta = b8; default: get_beta = b9;
        endcase
    end
endfunction

// ============================================================
// Golden reference -- pure Verilog, no packed array tricks
// Implements eq.(10) IAMS / eq.(5) OMS exactly as paper describes
// ============================================================
function signed [Q-1:0] golden_alpha;
    input [3:0]  n;
    input [3:0]  dc_in;
    input        oms_mode;
    // uses get_beta() to read current stimulus
    reg [MWIDTH-1:0] m1, m2;
    reg [3:0]        i1, i2;
    reg              txor;
    reg [MWIDTH-1:0] delta_v, out_mag, mag_excl;
    reg              out_sgn;
    reg signed [Q-1:0] bk;
    integer k;
    begin
        m1 = {MWIDTH{1'b1}};
        m2 = {MWIDTH{1'b1}};
        i1 = 4'd0; i2 = 4'd1;
        txor = 1'b0;

        for (k = 0; k < DC; k = k+1) begin
            if (k < dc_in) begin
                bk = get_beta(k[3:0]);
                txor = txor ^ bk[Q-1];                // sign bit
                if (bk[Q-2:0] < m1) begin
                    m2 = m1; i2 = i1;
                    m1 = bk[Q-2:0]; i1 = k[3:0];
                end else if (bk[Q-2:0] < m2) begin
                    m2 = bk[Q-2:0]; i2 = k[3:0];
                end
            end
        end

        // Output for node n
        if (n >= dc_in) begin
            golden_alpha = {Q{1'b0}};
        end else begin
            temp = get_beta(n);
            out_sgn   = txor ^ temp[Q-1];      // extrinsic sign
            delta_v   = m2 - m1;
            mag_excl  = (n[3:0] == i1) ? m2 : m1;

            if (oms_mode) begin
                // OMS eq.(5): max(mag_excl - 1, 0)
                out_mag = (mag_excl > 0) ? (mag_excl - 1'b1) : {MWIDTH{1'b0}};
            end else begin
                // IAMS eq.(10)
                if (n[3:0] == i1)
                    out_mag = m2;
                else if (n[3:0] == i2)
                    out_mag = m1;
                else if (delta_v == {MWIDTH{1'b0}})
                    out_mag = (m1 > 0) ? (m1 - 1'b1) : {MWIDTH{1'b0}};
                else
                    out_mag = m1;
            end

            golden_alpha = (out_mag == 0) ? {Q{1'b0}} : {out_sgn, out_mag};
        end
    end
endfunction

// ============================================================
// Test task
// ============================================================
integer pass_cnt, fail_cnt;
reg failed;

task apply_and_check;
    input [3:0]  dc_in;
    input        oms_in;
    input [63:0] test_id;
    reg signed [Q-1:0] gold;
    reg signed [Q-1:0] dut_v;
    integer k;
    begin
        dc_actual = dc_in;
        use_oms   = oms_in;
        #10;                    // combinational settle
        failed = 1'b0;
        for (k = 0; k < DC; k = k+1) begin
            if (k < dc_in) begin
                gold  = golden_alpha(k[3:0], dc_in, oms_in);
                dut_v = get_dut_out(k[3:0]);
                if (dut_v !== gold) begin
                    $display("FAIL test%0d n=%0d: got %0d, expected %0d (dc=%0d, oms=%0d)",
                             test_id, k, $signed(dut_v), $signed(gold), dc_in, oms_in);
                    failed = 1'b1;
                end
            end
        end
        if (!failed) begin
            $display("PASS test%0d (dc=%0d, oms=%0d)", test_id, dc_in, oms_in);
            pass_cnt = pass_cnt + 1;
        end else
            fail_cnt = fail_cnt + 1;
    end
endtask

// Helper: set all betas past index k to 0
task zero_rest;
    input integer from;
    begin
        if (from <= 0) b0 = 0; if (from <= 1) b1 = 0;
        if (from <= 2) b2 = 0; if (from <= 3) b3 = 0;
        if (from <= 4) b4 = 0; if (from <= 5) b5 = 0;
        if (from <= 6) b6 = 0; if (from <= 7) b7 = 0;
        if (from <= 8) b8 = 0; if (from <= 9) b9 = 0;
    end
endtask

reg signed [Q-1:0] r_iams, r_oms;
// ============================================================
initial begin
    $dumpfile("tb_cnu.vcd");
    $dumpvars(0, tb_cnu);
    pass_cnt = 0; fail_cnt = 0;
    b0=0;b1=0;b2=0;b3=0;b4=0;b5=0;b6=0;b7=0;b8=0;b9=0;

    // ----------------------------------------------------------
    // T1: All zeros, dc=5, IAMS
    // All outputs must be 0
    // ----------------------------------------------------------
    b0=0;b1=0;b2=0;b3=0;b4=0; zero_rest(5);
    apply_and_check(5, 0, 1);

    // ----------------------------------------------------------
    // T2: dc=3, IAMS, positive -- delta != 0
    // beta=[+2,+3,+5], all positive, total_xor=0
    // min1=2(idx1=0), min2=3(idx2=1), delta=1
    // n=0(idx1): out=+3   n=1(idx2): out=+2   n=2(ibar): out=+2
    // ----------------------------------------------------------
    b0=2; b1=3; b2=5; zero_rest(3);
    apply_and_check(3, 0, 2);

    // ----------------------------------------------------------
    // T3: dc=3, IAMS -- delta=0 (lambda=1 for ibar)
    // beta=[+3,+3,+5]: min1=3(idx=0), min2=3(idx=1), delta=0
    // n=0: +3  n=1: +3  n=2(ibar): max(3-1,0)=+2
    // ----------------------------------------------------------
    b0=3; b1=3; b2=5; zero_rest(3);
    apply_and_check(3, 0, 3);

    // ----------------------------------------------------------
    // T4: dc=4, IAMS, mixed signs
    // beta=[-2,+3,-5,+4]: mag=[2,3,5,4]
    // min1=2(idx1=0), min2=3(idx2=1), delta=1, total_xor=1^0^1^0=0
    // n=0: extr=0^1=1(neg) mag=min2=3  -> -3
    // n=1: extr=0^0=0(pos) mag=min1=2  -> +2
    // n=2: extr=0^1=1(neg) mag=min1=2  -> -2
    // n=3: extr=0^0=0(pos) mag=min1=2  -> +2
    // ----------------------------------------------------------
    b0=-2; b1=3; b2=-5; b3=4; zero_rest(4);
    apply_and_check(4, 0, 4);

    // ----------------------------------------------------------
    // T5: OMS mode, dc=3, same inputs as T2
    // n=0: max(min2-1,0)=max(3-1,0)=2  -> +2
    // n=1: max(min1-1,0)=max(2-1,0)=1  -> +1
    // n=2: max(min1-1,0)=1             -> +1
    // ----------------------------------------------------------
    b0=2; b1=3; b2=5; zero_rest(3);
    apply_and_check(3, 1, 5);

    // ----------------------------------------------------------
    // T6: dc=3, IAMS, min1=0 (Property Case 1 -> lambda=0)
    // beta=[0,3,5]: min1=0(idx1=0), min2=3(idx2=1), delta=3
    // n=0(idx1): mag=min2=3, extr=0^0=0 -> +3
    // n=1(idx2): mag=min1=0             -> 0
    // n=2(ibar,delta!=0): mag=min1=0   -> 0
    // ----------------------------------------------------------
    b0=0; b1=3; b2=5; zero_rest(3);
    apply_and_check(3, 0, 6);

    // ----------------------------------------------------------
    // T7: dc=10, IAMS, varied positive magnitudes
    // beta=[1,2,3,4,5,6,7,-1,-2,-3]
    // mags=[1,2,3,4,5,6,7,1,2,3]
    // min1=1(idx1=0 or 7), min2=1 or 2 -- let golden decide
    // ----------------------------------------------------------
    b0=1; b1=2; b2=3; b3=4; b4=5; b5=6; b6=7; b7=-1; b8=-2; b9=-3;
    apply_and_check(10, 0, 7);

    // ----------------------------------------------------------
    // T8: dc=10, OMS
    // ----------------------------------------------------------
    apply_and_check(10, 1, 8);

    // ----------------------------------------------------------
    // T9: dc=1 -- only one input, idx1=0
    // IAMS: n=0(idx1): mag=min2=max_val (since no second min),
    //   but with dc=1 there is no idx2. min2 stays all-ones.
    //   n=0 is idx1 -> output min2 (all-ones = 7 for MWIDTH=3)
    //   But sign: extr_sign = total_xor ^ sbit[0] = sbit[0] ^ sbit[0] = 0
    //   So output = +7 if b0 negative? No: extrinsic sign for dc=1
    //   means the "product of all OTHER signs" which is empty -> 0 (positive).
    //   n=0(idx1): mag=min2={1,1,1}=7, sign=0 -> +7
    // OMS: n=0(idx1): max(min2-1,0)=6, sign=0 -> +6
    // ----------------------------------------------------------
    b0=-4'sd8; zero_rest(1);   // any nonzero negative
    apply_and_check(1, 0, 9);  // IAMS dc=1
    apply_and_check(1, 1, 10); // OMS  dc=1

    // ----------------------------------------------------------
    // T11: dc=2, positive
    // beta=[3,5]: min1=3(idx1=0), min2=5(idx2=1), delta=2
    // IAMS: n=0: mag=min2=5,extr=0^0=0->+5  n=1: mag=min1=3,extr=0^0=0->+3
    // OMS:  n=0: max(5-1,0)=4->+4            n=1: max(3-1,0)=2->+2
    // ----------------------------------------------------------
    b0=3; b1=5; zero_rest(2);
    apply_and_check(2, 0, 11);
    apply_and_check(2, 1, 12);

    // ----------------------------------------------------------
    // T13: dc=3, all same magnitude, delta=0 -> lambda=1 for ibar
    // beta=[4,4,4]: min1=4(idx1=0), min2=4(idx2=1)
    // n=0(idx1): mag=4  n=1(idx2): mag=4  n=2(ibar,delta=0): max(4-1,0)=3
    // ----------------------------------------------------------
    b0=4; b1=4; b2=4; zero_rest(3);
    apply_and_check(3, 0, 13);

    // ----------------------------------------------------------
    // T14: dc=4, one zero magnitude
    // beta=[0,0,3,5]: min1=0(idx1=0), min2=0(idx2=1), delta=0
    // n=0(idx1): mag=min2=0->0
    // n=1(idx2): mag=min1=0->0
    // n=2(ibar,delta=0): max(min1-1,0)=max(0-1,0)=0 -> 0
    // n=3(ibar): same -> 0
    // ----------------------------------------------------------
    b0=0; b1=0; b2=3; b3=5; zero_rest(4);
    apply_and_check(4, 0, 14);

    // ----------------------------------------------------------
    // T15: Sign stress -- all negative inputs, dc=5
    // beta=[-1,-2,-3,-4,-5]
    // mags=[1,2,3,4,5], min1=1(idx1=0), min2=2(idx2=1), delta=1
    // total_xor=1^1^1^1^1=1
    // n=0(idx1): extr=1^1=0(pos), mag=min2=2 -> +2
    // n=1(idx2): extr=1^1=0(pos), mag=min1=1 -> +1
    // n=2..4(ibar,delta!=0): extr=1^1=0(pos), mag=min1=1 -> +1
    // ----------------------------------------------------------
    b0=-1; b1=-2; b2=-3; b3=-4; b4=-5; zero_rest(5);
    apply_and_check(5, 0, 15);
    apply_and_check(5, 1, 16);

    // ----------------------------------------------------------
    // T17: IAMS vs OMS difference check at idx1
    // For any input with delta != 0, IAMS gives min2 at idx1 (no offset),
    // OMS gives max(min2-1,0). Verify IAMS >= OMS at idx1 when min2>0.
    // ----------------------------------------------------------
    b0=2; b1=4; b2=6; zero_rest(3);
    begin
        dc_actual=3; use_oms=0; #10; r_iams=a0;  // IAMS at idx1(n=0)
        dc_actual=3; use_oms=1; #10; r_oms =a0;  // OMS  at idx1(n=0)
        if ($signed(r_iams) >= $signed(r_oms)) begin
            $display("PASS test17: IAMS(%0d) >= OMS(%0d) at idx1",
                     $signed(r_iams), $signed(r_oms));
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("FAIL test17: IAMS(%0d) < OMS(%0d) at idx1",
                     $signed(r_iams), $signed(r_oms));
            fail_cnt = fail_cnt + 1;
        end
    end

    // ----------------------------------------------------------
    // T18-T67: Randomised tests (50 IAMS + 50 OMS)
    // ----------------------------------------------------------
    begin : RAND_TESTS
        integer t;
        reg [3:0] dc_r;
        reg signed [Q-1:0] rval;
        for (t = 0; t < 50; t = t+1) begin
            // dc in [3..10]
            dc_r = ($random % 8) + 3;
            if (dc_r > 10) dc_r = 10;

            // random Q-bit signed values in [-8..+7]
            rval = $random; b0 = (dc_r>0) ? rval[Q-1:0] : 0;
            rval = $random; b1 = (dc_r>1) ? rval[Q-1:0] : 0;
            rval = $random; b2 = (dc_r>2) ? rval[Q-1:0] : 0;
            rval = $random; b3 = (dc_r>3) ? rval[Q-1:0] : 0;
            rval = $random; b4 = (dc_r>4) ? rval[Q-1:0] : 0;
            rval = $random; b5 = (dc_r>5) ? rval[Q-1:0] : 0;
            rval = $random; b6 = (dc_r>6) ? rval[Q-1:0] : 0;
            rval = $random; b7 = (dc_r>7) ? rval[Q-1:0] : 0;
            rval = $random; b8 = (dc_r>8) ? rval[Q-1:0] : 0;
            rval = $random; b9 = (dc_r>9) ? rval[Q-1:0] : 0;

            apply_and_check(dc_r, 0, 18+t);
            apply_and_check(dc_r, 1, 68+t);
        end
    end

    // ---- Summary ----
    $display("\n=== CNU Unit Test Summary ===");
    $display("PASS: %0d  FAIL: %0d  TOTAL: %0d",
             pass_cnt, fail_cnt, pass_cnt+fail_cnt);
    if (fail_cnt == 0)
        $display("ALL TESTS PASSED");
    else
        $display("SOME TESTS FAILED");
    $finish;
end

initial begin
    #TIMEOUT;
    $display("TIMEOUT at %0t", $time);
    $finish;
end

endmodule
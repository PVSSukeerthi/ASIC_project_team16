// ============================================================
// tb_vnu_app.v  –  Testbench for VNU and APP Update
// Verifies:
//   VNU:  beta = clamp(APP - alpha_old) to Q-bit range
//   APP:  new_APP = clamp(alpha_new + (APP - alpha_old)) to QTILDE range
// ============================================================
`include "ldpc5g_pkg.v"
`timescale 1ns/1ps

module tb_vnu_app;

localparam Q      = `Q;       // 4
localparam QTILDE = `QTILDE;  // 6

// ---- VNU DUT ----
reg signed [QTILDE-1:0] app_in;
reg signed [Q-1:0]      ctv_old;
wire signed [Q-1:0]     vtc_out;

vnu #(.Q(Q), .QTILDE(QTILDE)) dut_vnu (
    .app_in  (app_in),
    .ctv_old (ctv_old),
    .vtc_out (vtc_out)
);

// ---- APP Update DUT ----
reg signed [Q-1:0]      ctv_new;
wire signed [QTILDE-1:0] app_out;

app_update #(.Q(Q), .QTILDE(QTILDE)) dut_app (
    .app_in  (app_in),
    .ctv_old (ctv_old),
    .ctv_new (ctv_new),
    .app_out (app_out)
);

// ---- Golden Reference ----
localparam signed [Q-1:0]      MSG_MAX =  (1 << (Q-1)) - 1;    // +7
localparam signed [Q-1:0]      MSG_MIN = -(1 << (Q-1));         // -8
localparam signed [QTILDE-1:0] APP_MAX =  (1 << (QTILDE-1))-1; // +31
localparam signed [QTILDE-1:0] APP_MIN = -(1 << (QTILDE-1));   // -32

function automatic signed [Q-1:0] golden_vnu;
    input signed [QTILDE-1:0] app;
    input signed [Q-1:0]      alpha_old;
    reg signed [QTILDE-1:0] diff;
    begin
        diff = app - {{(QTILDE-Q){alpha_old[Q-1]}}, alpha_old};
        if (diff > MSG_MAX) golden_vnu = MSG_MAX;
        else if (diff < MSG_MIN) golden_vnu = MSG_MIN;
        else golden_vnu = diff[Q-1:0];
    end
endfunction

function automatic signed [QTILDE-1:0] golden_app;
    input signed [QTILDE-1:0] app;
    input signed [Q-1:0]      alpha_old, alpha_new;
    reg signed [QTILDE-1:0] beta_tilde, sum;
    begin
        beta_tilde = app - {{(QTILDE-Q){alpha_old[Q-1]}}, alpha_old};
        sum = beta_tilde + {{(QTILDE-Q){alpha_new[Q-1]}}, alpha_new};
        if (sum > APP_MAX) golden_app = APP_MAX;
        else if (sum < APP_MIN) golden_app = APP_MIN;
        else golden_app = sum;
    end
endfunction

integer pass_cnt, fail_cnt;
reg failed;

task check_vnu;
    input signed [QTILDE-1:0] app;
    input signed [Q-1:0] alpha;
    input [31:0] tid;
    reg signed [Q-1:0] gold;
    begin
        app_in  = app;
        ctv_old = alpha;
        #5;
        gold = golden_vnu(app, alpha);
        if (vtc_out !== gold) begin
            $display("VNU FAIL t%0d: app=%0d alpha=%0d got=%0d exp=%0d",
                     tid, $signed(app), $signed(alpha), $signed(vtc_out), $signed(gold));
            fail_cnt = fail_cnt + 1;
        end else begin
            $display("VNU PASS t%0d", tid);
            pass_cnt = pass_cnt + 1;
        end
    end
endtask

task check_app;
    input signed [QTILDE-1:0] app;
    input signed [Q-1:0] a_old, a_new;
    input [31:0] tid;
    reg signed [QTILDE-1:0] gold;
    begin
        app_in  = app;
        ctv_old = a_old;
        ctv_new = a_new;
        #5;
        gold = golden_app(app, a_old, a_new);
        if (app_out !== gold) begin
            $display("APP FAIL t%0d: app=%0d aold=%0d anew=%0d got=%0d exp=%0d",
                     tid, $signed(app), $signed(a_old), $signed(a_new),
                     $signed(app_out), $signed(gold));
            fail_cnt = fail_cnt + 1;
        end else begin
            $display("APP PASS t%0d", tid);
            pass_cnt = pass_cnt + 1;
        end
    end
endtask

initial begin
    $dumpfile("tb_vnu_app.vcd");
    $dumpvars(0, tb_vnu_app);
    pass_cnt = 0; fail_cnt = 0;
    ctv_new = 0;

    // VNU tests
    check_vnu(6'sd10,  4'sd2,  1);   // 10-2=8 -> clamp to +7
    check_vnu(6'sd5,   4'sd2,  2);   // 5-2=3
    check_vnu(-6'sd10, 4'sd2,  3);   // -10-2=-12 -> clamp to -8
    check_vnu(6'sd0,   4'sd0,  4);   // 0
    check_vnu(6'sd31,  4'sd7,  5);   // 31-7=24 -> clamp +7
    check_vnu(-6'sd32,-4'sd8,  6);   // -32-(-8)=-24 -> clamp -8
    check_vnu(6'sd3,  -4'sd4,  7);   // 3-(-4)=7 -> +7
    check_vnu(6'sd3,  -4'sd5,  8);   // 3-(-5)=8 -> clamp +7

    // Random VNU
    begin : rvnu
        integer t;
        for (t = 0; t < 30; t = t+1) begin
            check_vnu($random % 64 - 32,
                      $random % 16 - 8,
                      10 + t);
        end
    end

    // APP update tests
    ctv_new = 0;
    check_app(6'sd10, 4'sd2, 4'sd3, 100);  // beta=8, new=3, sum=11
    check_app(6'sd5,  4'sd2, 4'sd2, 101);  // beta=3, new=2, sum=5
    check_app(6'sd31, 4'sd0, 4'sd7, 102);  // beta=31, new=7 -> clamp 31
    check_app(-6'sd32,4'sd0,-4'sd8,103);   // -32+(-8)=-40 -> clamp -32
    check_app(6'sd0,  4'sd3,-4'sd3, 104);  // beta=-3, new=-3 -> -6

    begin : rapp
        integer t;
        for (t = 0; t < 30; t = t+1) begin
            check_app($random % 64 - 32,
                      $random % 16 - 8,
                      $random % 16 - 8,
                      200 + t);
        end
    end

    $display("\n=== VNU/APP Test Summary ===");
    $display("PASS: %0d  FAIL: %0d", pass_cnt, fail_cnt);
    if (fail_cnt == 0) $display("ALL PASS");
    $finish;
end

initial begin
    #50000;
    $display("TIMEOUT");
    $finish;
end

endmodule

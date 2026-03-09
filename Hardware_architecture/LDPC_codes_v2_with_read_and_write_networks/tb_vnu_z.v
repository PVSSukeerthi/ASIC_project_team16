// =============================================================================
// tb_vnu_z.v  —  Testbench for Z-Parallel VNU
// =============================================================================
`timescale 1ns/1ps
`include "pkg_iams.vh"

module tb_vnu_z;

    localparam Z   = `LIFTING_Z;
    localparam QTW = `QTW;
    localparam QW  = `QW;

    reg                clk, rst_n;
    reg  [Z*QTW-1:0]  app_in;
    reg  [Z*QW-1:0]   alpha_old;
    wire [Z*QTW-1:0]  beta_out;
    reg                upd_en;
    reg  [Z*QW-1:0]   alpha_new;
    wire [Z*QTW-1:0]  app_upd_out;
    wire               app_upd_valid;

    vnu_z dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .app_in        (app_in),
        .alpha_old     (alpha_old),
        .beta_out      (beta_out),
        .upd_en        (upd_en),
        .alpha_new     (alpha_new),
        .app_upd_out   (app_upd_out),
        .app_upd_valid (app_upd_valid)
    );

    always #5 clk = ~clk;

    // Saturation
    function signed [QTW-1:0] sat;
        input integer val;
        begin
            if      (val >  127) sat =  127;
            else if (val < -128) sat = -128;
            else                 sat = val[QTW-1:0];
        end
    endfunction

    integer errors, i;
    reg signed [QTW-1:0] a, ao, an;
    integer exp_beta, exp_app;

    initial begin
        clk = 0; rst_n = 0; upd_en = 0;
        app_in = 0; alpha_old = 0; alpha_new = 0;
        #12 rst_n = 1;
        errors = 0;
        $display("=== VNU_Z Testbench ===");

        // Test 1: β = APP - α_old, no update
        // Set uniform values
        for (i = 0; i < Z; i = i + 1) begin
            app_in   [i*QTW +: QTW] = 8'sd20;
            alpha_old[i*QW  +: QW]  = 7'sd5;
        end
        #1;
        // Check beta (combinational)
        begin : t1
            integer ok; ok = 1;
            for (i = 0; i < Z; i = i + 1) begin
                exp_beta = 20 - 5;  // = 15
                if ($signed(beta_out[i*QTW +: QTW]) !== exp_beta) begin
                    $display("FAIL T1 z=%0d beta=%0d exp=%0d",
                             i, $signed(beta_out[i*QTW +: QTW]), exp_beta);
                    ok = 0; errors = errors + 1;
                end
            end
            if (ok) $display("PASS T1 (beta = APP - alpha_old = 15)");
        end

        // Test 2: APP update with upd_en
        for (i = 0; i < Z; i = i + 1)
            alpha_new[i*QW +: QW] = 7'sd8;
        @(posedge clk); #1;
        upd_en = 1;
        @(posedge clk); #1;   // app_upd_valid now HIGH (= upd_en registered)
        // Expected: app + α_new - α_old = 20 + 8 - 5 = 23
        begin : t2
            integer ok; ok = 1;
            if (app_upd_valid) begin
                for (i = 0; i < Z; i = i + 1) begin
                    exp_app = 23;
                    if ($signed(app_upd_out[i*QTW +: QTW]) !== exp_app) begin
                        $display("FAIL T2 z=%0d app_upd=%0d exp=%0d",
                                 i, $signed(app_upd_out[i*QTW +: QTW]), exp_app);
                        ok = 0; errors = errors + 1;
                    end
                end
                if (ok) $display("PASS T2 (APP update = 23)");
            end else $display("WARN T2 upd_valid not asserted");
        end
        upd_en = 0;

        // Test 3: Saturation — positive overflow
        for (i = 0; i < Z; i = i + 1) begin
            app_in   [i*QTW +: QTW] = 8'sd100;
            alpha_old[i*QW  +: QW]  = -7'sd20;
            alpha_new[i*QW  +: QW]  = 7'sd30;
        end
        @(posedge clk); #1;
        // Beta = 100 - (-20) = 120
        begin : t3a
            integer ok; ok = 1;
            for (i = 0; i < Z; i = i + 1) begin
                if ($signed(beta_out[i*QTW +: QTW]) !== 120) begin
                    ok = 0; errors = errors + 1;
                end
            end
            if (ok) $display("PASS T3a (beta=120 no saturation)");
        end
        upd_en = 1; @(posedge clk); #1;   // valid now HIGH
        // app_upd = 100 + 30 - (-20) = 150 > 127 → saturate to 127
        begin : t3b
            integer ok; ok = 1;
            if (app_upd_valid) begin
                for (i = 0; i < Z; i = i + 1) begin
                    if ($signed(app_upd_out[i*QTW +: QTW]) !== 127) begin
                        $display("FAIL T3b z=%0d exp=127 got=%0d",
                                 i, $signed(app_upd_out[i*QTW +: QTW]));
                        ok = 0; errors = errors + 1;
                    end
                end
                if (ok) $display("PASS T3b (positive saturation to 127)");
            end
        end
        upd_en = 0;

        // Test 4: Saturation — negative overflow
        for (i = 0; i < Z; i = i + 1) begin
            app_in   [i*QTW +: QTW] = -8'sd100;
            alpha_old[i*QW  +: QW]  = 7'sd20;
            alpha_new[i*QW  +: QW]  = -7'sd30;
        end
        @(posedge clk); #1;
        upd_en = 1; @(posedge clk); #1;   // valid now HIGH
        // app_upd = -100 + (-30) - 20 = -150 < -128 → saturate to -128
        begin : t4
            integer ok; ok = 1;
            if (app_upd_valid) begin
                for (i = 0; i < Z; i = i + 1) begin
                    if ($signed(app_upd_out[i*QTW +: QTW]) !== -128) begin
                        $display("FAIL T4 z=%0d exp=-128 got=%0d",
                                 i, $signed(app_upd_out[i*QTW +: QTW]));
                        ok = 0; errors = errors + 1;
                    end
                end
                if (ok) $display("PASS T4 (negative saturation to -128)");
            end
        end
        upd_en = 0;

        if (errors == 0) $display("ALL PASSED");
        else             $display("FAILED: %0d errors", errors);
        $finish;
    end

endmodule
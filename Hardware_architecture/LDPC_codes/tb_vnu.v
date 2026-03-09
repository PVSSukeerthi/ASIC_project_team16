// =============================================================================
// tb_vnu.v  —  Testbench for vnu.v (Variable Node Unit)
// =============================================================================
// Tests the layered VN update:
//   beta_out = sat(app_in − alpha_prev)
//   app_out  = sat(app_in + alpha_new − alpha_prev)
//            = sat(beta_out + alpha_new)
// =============================================================================

`timescale 1ns / 1ps

module tb_vnu;

    localparam QW  = 7;
    localparam QTW = 8;

    localparam signed [QTW-1:0] SAT_APP_MAX =  (1 <<< (QTW-1)) - 1;  // +127
    localparam signed [QTW-1:0] SAT_APP_MIN = -(1 <<< (QTW-1));       // -128
    localparam signed [QW-1:0]  SAT_MSG_MAX =  (1 <<< (QW-1)) - 1;   // +63
    localparam signed [QW-1:0]  SAT_MSG_MIN = -(1 <<< (QW-1));        // -64

    // ---- Clock / reset -------------------------------------------------------
    reg clk = 0;
    always #5 clk = ~clk;

    reg rst_n;
    initial begin rst_n = 0; #25 rst_n = 1; end

    // ---- DUT ports ----------------------------------------------------------
    reg              valid_in;
    reg  signed [QTW-1:0] app_in;
    reg  signed [QW-1:0]  alpha_new;
    reg  signed [QW-1:0]  alpha_prev;
    wire signed [QW-1:0]  beta_out;
    wire signed [QTW-1:0] app_out;
    wire                  valid_out;

    vnu #(.QW(QW), .QTW(QTW)) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (valid_in),
        .app_in    (app_in),
        .alpha_new (alpha_new),
        .alpha_prev(alpha_prev),
        .beta_out  (beta_out),
        .app_out   (app_out),
        .valid_out (valid_out)
    );

    // ---- Golden model --------------------------------------------------------
    function signed [QW-1:0] sat_msg;
        input signed [QTW+1:0] x;
        begin
            if (x >  $signed(SAT_MSG_MAX)) sat_msg = SAT_MSG_MAX;
            else if (x < $signed(SAT_MSG_MIN)) sat_msg = SAT_MSG_MIN;
            else sat_msg = x[QW-1:0];
        end
    endfunction

    function signed [QTW-1:0] sat_app;
        input signed [QTW+1:0] x;
        begin
            if (x >  $signed(SAT_APP_MAX)) sat_app = SAT_APP_MAX;
            else if (x < $signed(SAT_APP_MIN)) sat_app = SAT_APP_MIN;
            else sat_app = x[QTW-1:0];
        end
    endfunction

    integer pass_cnt = 0, fail_cnt = 0;

    task apply_and_check;
        input signed [QTW-1:0] a_in;
        input signed [QW-1:0]  a_new, a_prev;
        input integer           test_id;
        reg signed [QTW+1:0]   beta_ref, app_ref;
        reg signed [QW-1:0]    exp_beta;
        reg signed [QTW-1:0]   exp_app;
        begin
            app_in     = a_in;
            alpha_new  = a_new;
            alpha_prev = a_prev;
            valid_in   = 1;
            @(posedge clk); #1;
            valid_in   = 0;
            // beta is combinational — read immediately
            beta_ref   = $signed(a_in) - $signed(a_prev);
            exp_beta   = sat_msg(beta_ref);
            // app is registered — wait one cycle
            @(posedge clk); #1;
            app_ref    = beta_ref + $signed(a_new);
            exp_app    = sat_app(app_ref);

            if (beta_out === exp_beta && app_out === exp_app) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] T%0d  app_in=%0d  an=%0d  ap=%0d  beta=%0d  app=%0d",
                         test_id, a_in, a_new, a_prev, exp_beta, exp_app);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] T%0d  app_in=%0d  an=%0d  ap=%0d  beta: got=%0d exp=%0d  app: got=%0d exp=%0d",
                         test_id, a_in, a_new, a_prev,
                         beta_out, exp_beta, app_out, exp_app);
            end
        end
    endtask

    integer seed = 7;
    integer trial;
    reg signed [QTW-1:0] r_app;
    reg signed [QW-1:0]  r_an, r_ap;

    initial begin
        valid_in   = 0;
        app_in     = 0;
        alpha_new  = 0;
        alpha_prev = 0;
        @(posedge rst_n); @(posedge clk); #1;

        $display("=================================================================");
        $display("  VNU Testbench  (QW=%0d, QTW=%0d)", QW, QTW);
        $display("=================================================================");

        // --- Directed tests ---
        apply_and_check(  8'd10,  7'd3,  7'd2, 1);
        apply_and_check(  8'd0,   7'd0,  7'd0, 2);
        apply_and_check( -8'd20,  7'd5, -7'd3, 3);
        apply_and_check(  8'd100, 7'd10, 7'd8, 4);  // near saturation
        apply_and_check( -8'd100,-7'd10,-7'd8, 5);
        // Saturation: large positive APP + positive alpha_new
        apply_and_check( 8'd127, 7'd63, 7'd0, 6);  // app_out should saturate at +127
        apply_and_check(-8'd128,-7'd64, 7'd0, 7);  // app_out should saturate at -128

        // --- Random trials ---
        for (trial = 8; trial < 508; trial = trial + 1) begin
            r_app = $random(seed) % 128;
            r_an  = $random(seed) % 64;
            r_ap  = $random(seed) % 64;
            apply_and_check(r_app, r_an, r_ap, trial);
        end

        $display("");
        $display("=================================================================");
        $display("  Results: %0d PASS,  %0d FAIL", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display("  *** ALL TESTS PASSED ***");
        $display("=================================================================");
        $finish;
    end

    initial begin
        #200_000; $display("TIMEOUT"); $finish;
    end

    initial begin
        $dumpfile("tb_vnu.vcd");
        $dumpvars(0, tb_vnu);
    end

endmodule

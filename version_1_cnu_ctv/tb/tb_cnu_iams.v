`timescale 1ns / 1ps

module tb_cnu_iams;

    localparam DC      = 10;
    localparam QW      = 7;
    localparam IDW     = 4;
    localparam MAG_W   = QW - 1;
    localparam MAG_MAX = (1 << MAG_W) - 1;
    localparam MSG_MAX =  MAG_MAX;
    localparam MSG_MIN = -(1 << (QW-1));

    reg clk = 0;
    always #5 clk = ~clk;

    reg rst_n;
    initial begin rst_n = 0; #25 rst_n = 1; end

    reg              valid_in;
    reg  [DC*QW-1:0] msg_flat_in;
    wire [DC*QW-1:0] msg_flat_out;
    wire             valid_out;

    // New output ports
    wire [QW-2:0]    min1_out;
    wire [QW-2:0]    min2_out;
    wire [IDW-1:0]   idx1_out;
    wire [IDW-1:0]   idx2_out;
    wire [DC-1:0]    sgn_out;

    cnu_iams #(
        .DC (DC),
        .QW (QW),
        .IDW(IDW)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .valid_in    (valid_in),
        .msg_flat_in (msg_flat_in),
        .msg_flat_out(msg_flat_out),
        .valid_out   (valid_out),
        .min1_out    (min1_out),
        .min2_out    (min2_out),
        .idx1_out    (idx1_out),
        .idx2_out    (idx2_out),
        .sgn_out     (sgn_out)
    );

    task pack_msgs;
        input signed [QW-1:0] m0, m1, m2, m3, m4, m5, m6, m7, m8, m9;
        begin
            msg_flat_in[0*QW +: QW] = m0;
            msg_flat_in[1*QW +: QW] = m1;
            msg_flat_in[2*QW +: QW] = m2;
            msg_flat_in[3*QW +: QW] = m3;
            msg_flat_in[4*QW +: QW] = m4;
            msg_flat_in[5*QW +: QW] = m5;
            msg_flat_in[6*QW +: QW] = m6;
            msg_flat_in[7*QW +: QW] = m7;
            msg_flat_in[8*QW +: QW] = m8;
            msg_flat_in[9*QW +: QW] = m9;
        end
    endtask

    // Golden model — Eq.(10) IAMS
    task golden_iams;
        input  [DC*QW-1:0] inp;
        output [DC*QW-1:0] outp;

        reg signed [QW-1:0]   msg_i   [0:DC-1];
        reg        [MAG_W-1:0] mag_i   [0:DC-1];
        reg                    sgn_i   [0:DC-1];
        reg        [QW-1:0]    abs_full;
        reg        [MAG_W-1:0] min1, min2;
        integer                idx1, idx2;
        reg                    sign_total;
        reg        [MAG_W-1:0] delta, alpha_mag;
        reg                    out_sign;
        reg signed [QW-1:0]    out_msg;
        integer                ii, jj;

        begin
            for (ii = 0; ii < DC; ii = ii + 1) begin
                msg_i[ii] = inp[ii*QW +: QW];
                sgn_i[ii] = msg_i[ii][QW-1];
                if (sgn_i[ii]) abs_full = ~msg_i[ii] + 1;
                else           abs_full = msg_i[ii];
                if (abs_full[QW-1]) mag_i[ii] = MAG_MAX[MAG_W-1:0];
                else                mag_i[ii] = abs_full[MAG_W-1:0];
            end

            sign_total = 1'b0;
            for (ii = 0; ii < DC; ii = ii + 1)
                sign_total = sign_total ^ sgn_i[ii];

            min1 = {MAG_W{1'b1}}; min2 = {MAG_W{1'b1}};
            idx1 = 0; idx2 = 0;
            for (ii = 0; ii < DC; ii = ii + 1) begin
                if (mag_i[ii] <= min1) begin
                    min2 = min1; idx2 = idx1;
                    min1 = mag_i[ii]; idx1 = ii;
                end else if (mag_i[ii] < min2) begin
                    min2 = mag_i[ii]; idx2 = ii;
                end
            end
            delta = min2 - min1;

            for (jj = 0; jj < DC; jj = jj + 1) begin
                out_sign = sign_total ^ sgn_i[jj];
                if      (jj == idx1) alpha_mag = min2;
                else if (jj == idx2) alpha_mag = min1;
                else begin
                    if (delta == 0) alpha_mag = (min1 > 1) ? (min1 - 1) : 0;
                    else            alpha_mag = min1;
                end

                if (alpha_mag == 0)      out_msg = {QW{1'b0}};
                else if (out_sign)       out_msg = ~({1'b0, alpha_mag}) + 1;
                else                     out_msg = {1'b0, alpha_mag};

                outp[jj*QW +: QW] = out_msg;
            end
        end
    endtask

    // Golden model for compressed fields — returns min1,min2,idx1,idx2,sgn
    task golden_compressed;
        input  [DC*QW-1:0] inp;
        output [MAG_W-1:0] g_min1, g_min2;
        output [IDW-1:0]   g_idx1, g_idx2;
        output [DC-1:0]    g_sgn;

        reg signed [QW-1:0]   msg_i [0:DC-1];
        reg        [MAG_W-1:0] mag_i [0:DC-1];
        reg                    sgn_i [0:DC-1];
        reg        [QW-1:0]    abs_full;
        reg        [MAG_W-1:0] min1, min2;
        integer                idx1, idx2;
        integer                ii;

        begin
            for (ii = 0; ii < DC; ii = ii + 1) begin
                msg_i[ii] = inp[ii*QW +: QW];
                sgn_i[ii] = msg_i[ii][QW-1];
                if (sgn_i[ii]) abs_full = ~msg_i[ii] + 1;
                else           abs_full = msg_i[ii];
                if (abs_full[QW-1]) mag_i[ii] = MAG_MAX[MAG_W-1:0];
                else                mag_i[ii] = abs_full[MAG_W-1:0];
                g_sgn[ii] = sgn_i[ii];
            end

            min1 = {MAG_W{1'b1}}; min2 = {MAG_W{1'b1}};
            idx1 = 0; idx2 = 0;
            for (ii = 0; ii < DC; ii = ii + 1) begin
                if (mag_i[ii] <= min1) begin
                    min2 = min1; idx2 = idx1;
                    min1 = mag_i[ii]; idx1 = ii;
                end else if (mag_i[ii] < min2) begin
                    min2 = mag_i[ii]; idx2 = ii;
                end
            end
            g_min1 = min1; g_min2 = min2;
            g_idx1 = idx1[IDW-1:0]; g_idx2 = idx2[IDW-1:0];
        end
    endtask

    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task check_output;
        input [DC*QW-1:0] got, expected;
        input integer      test_id;
        integer ii;
        reg ok;
        begin
            ok = (got === expected);
            if (ok) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] Test %0d", test_id);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] Test %0d", test_id);
                for (ii = 0; ii < DC; ii = ii + 1)
                    if (got[ii*QW +: QW] !== expected[ii*QW +: QW])
                        $display("       edge[%0d]: got %0d  expected %0d",
                            ii, $signed(got[ii*QW +: QW]),
                                $signed(expected[ii*QW +: QW]));
            end
        end
    endtask

    task check_compressed;
        input [MAG_W-1:0] g_min1, g_min2;
        input [IDW-1:0]   g_idx1, g_idx2;
        input [DC-1:0]    g_sgn;
        input integer     test_id;
        reg ok;
        begin
            ok = (min1_out === g_min1) && (min2_out === g_min2) &&
                 (idx1_out === g_idx1) && (idx2_out === g_idx2) &&
                 (sgn_out  === g_sgn);
            if (ok) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] Test %0d compressed fields", test_id);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] Test %0d compressed fields", test_id);
                if (min1_out !== g_min1)
                    $display("       min1: got %0d exp %0d", min1_out, g_min1);
                if (min2_out !== g_min2)
                    $display("       min2: got %0d exp %0d", min2_out, g_min2);
                if (idx1_out !== g_idx1)
                    $display("       idx1: got %0d exp %0d", idx1_out, g_idx1);
                if (idx2_out !== g_idx2)
                    $display("       idx2: got %0d exp %0d", idx2_out, g_idx2);
                if (sgn_out !== g_sgn)
                    $display("       sgn:  got %b exp %b", sgn_out, g_sgn);
            end
        end
    endtask

    reg  [DC*QW-1:0] expected_out;
    reg  [MAG_W-1:0] exp_min1, exp_min2;
    reg  [IDW-1:0]   exp_idx1, exp_idx2;
    reg  [DC-1:0]    exp_sgn;
    reg  [DC*QW-1:0] captured_in;

    integer seed = 42;
    integer trial, ri;
    reg signed [QW-1:0] rand_msg;
    reg [DC*QW-1:0] inp3, inp4, inp5, inp6, inp7;
    integer kk;

    initial begin
        valid_in    = 0;
        msg_flat_in = 0;
        @(posedge rst_n);
        @(posedge clk); #1;

        $display("=================================================================");
        $display("  IAMS CNU Testbench  (DC=%0d, QW=%0d)", DC, QW);
        $display("=================================================================");

        // T1: reset
        @(posedge clk); #1;
        if (valid_out !== 1'b0)
            $display("[FAIL] T1: valid_out should be 0 after reset, got %0b", valid_out);
        else begin pass_cnt = pass_cnt + 1; $display("[PASS] T1: reset state correct"); end

        // T2: all-zero
        pack_msgs(0,0,0,0,0,0,0,0,0,0);
        captured_in = msg_flat_in;
        valid_in = 1; @(posedge clk); #1;
        valid_in = 0;
        golden_iams(captured_in, expected_out);
        golden_compressed(captured_in, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
        @(posedge clk); #1;
        check_output(msg_flat_out, expected_out, 2);
        check_compressed(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, 2);

        // T3: mixed signs Δ≠0
        begin
            inp3[0*QW +:QW] = 7'd3;  inp3[1*QW +:QW] = -7'd5;
            inp3[2*QW +:QW] = 7'd2;  inp3[3*QW +:QW] = -7'd6;
            inp3[4*QW +:QW] = 7'd4;  inp3[5*QW +:QW] = -7'd2;
            inp3[6*QW +:QW] = 7'd1;  inp3[7*QW +:QW] = -7'd4;
            inp3[8*QW +:QW] = 7'd3;  inp3[9*QW +:QW] = 7'd2;
        end
        msg_flat_in = inp3;
        valid_in = 1; @(posedge clk); #1;
        valid_in = 0;
        golden_iams(inp3, expected_out);
        golden_compressed(inp3, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
        @(posedge clk); #1;
        check_output(msg_flat_out, expected_out, 3);
        check_compressed(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, 3);

        // T4: Δ=0 case
        begin
            inp4[0*QW +:QW] = 7'd2;  inp4[1*QW +:QW] = -7'd2;
            inp4[2*QW +:QW] = 7'd5;  inp4[3*QW +:QW] = -7'd8;
            inp4[4*QW +:QW] = 7'd3;  inp4[5*QW +:QW] = 7'd4;
            inp4[6*QW +:QW] = 7'd6;  inp4[7*QW +:QW] = -7'd2;
            inp4[8*QW +:QW] = 7'd7;  inp4[9*QW +:QW] = 7'd3;
        end
        msg_flat_in = inp4;
        valid_in = 1; @(posedge clk); #1;
        valid_in = 0;
        golden_iams(inp4, expected_out);
        golden_compressed(inp4, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
        @(posedge clk); #1;
        check_output(msg_flat_out, expected_out, 4);
        check_compressed(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, 4);

        // T5: all same magnitude
        begin
            for (kk = 0; kk < DC; kk = kk + 1)
                inp5[kk*QW +:QW] = 7'd3;
        end
        msg_flat_in = inp5;
        valid_in = 1; @(posedge clk); #1;
        valid_in = 0;
        golden_iams(inp5, expected_out);
        golden_compressed(inp5, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
        @(posedge clk); #1;
        check_output(msg_flat_out, expected_out, 5);
        check_compressed(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, 5);

        // T6: saturation ±63
        begin
            for (kk = 0; kk < DC; kk = kk + 2) begin
                inp6[ kk   *QW +:QW] =  7'd63;
                inp6[(kk+1)*QW +:QW] = -7'd63;
            end
        end
        msg_flat_in = inp6;
        valid_in = 1; @(posedge clk); #1;
        valid_in = 0;
        golden_iams(inp6, expected_out);
        golden_compressed(inp6, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
        @(posedge clk); #1;
        check_output(msg_flat_out, expected_out, 6);
        check_compressed(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, 6);

        // T7: single dominant min
        begin
            inp7[0*QW +:QW]=7'd1;  inp7[1*QW +:QW]=7'd10;
            inp7[2*QW +:QW]=7'd12; inp7[3*QW +:QW]=7'd9;
            inp7[4*QW +:QW]=7'd8;  inp7[5*QW +:QW]=7'd11;
            inp7[6*QW +:QW]=7'd15; inp7[7*QW +:QW]=7'd7;
            inp7[8*QW +:QW]=7'd10; inp7[9*QW +:QW]=7'd11;
        end
        msg_flat_in = inp7;
        valid_in = 1; @(posedge clk); #1;
        valid_in = 0;
        golden_iams(inp7, expected_out);
        golden_compressed(inp7, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
        @(posedge clk); #1;
        check_output(msg_flat_out, expected_out, 7);
        check_compressed(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, 7);

        // T8-1007: 1000 random trials
        $display("--- Running 1000 random trials ---");
        for (trial = 8; trial < 1008; trial = trial + 1) begin
            for (ri = 0; ri < DC; ri = ri + 1) begin
                rand_msg = $signed($random(seed)) % 64;
                msg_flat_in[ri*QW +: QW] = rand_msg;
            end
            captured_in = msg_flat_in;
            valid_in = 1; @(posedge clk); #1;
            valid_in = 0;
            golden_iams(captured_in, expected_out);
            golden_compressed(captured_in, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
            @(posedge clk); #1;
            check_output(msg_flat_out, expected_out, trial);
            check_compressed(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, trial);
        end

        // T1008: valid_out gating
        valid_in = 0;
        @(posedge clk); #1;
        if (valid_out !== 1'b0)
            $display("[FAIL] T1008: valid_out should be 0, got %0b", valid_out);
        else begin pass_cnt = pass_cnt + 1; $display("[PASS] T1008: valid_out gating correct"); end

        $display("");
        $display("=================================================================");
        $display("  Results: %0d PASS,  %0d FAIL  (total %0d)",
                  pass_cnt, fail_cnt, pass_cnt + fail_cnt);
        if (fail_cnt == 0) $display("  *** ALL TESTS PASSED ***");
        else               $display("  *** FAILURES DETECTED — see above ***");
        $display("=================================================================");
        $finish;
    end

    initial begin #500_000; $display("TIMEOUT"); $finish; end

    initial begin
        $dumpfile("tb_cnu_iams.vcd");
        $dumpvars(0, tb_cnu_iams);
    end

endmodule

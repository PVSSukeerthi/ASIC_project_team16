`timescale 1ns/1ps

`define SIM_Z       4
`define SIM_QIN     8
`define SIM_QW      7
`define SIM_DC_MAX  4
`define SIM_IDW     3

module tb_cnu_iams_z;

    localparam Z       = `SIM_Z;
    localparam QIN     = `SIM_QIN;
    localparam QW      = `SIM_QW;
    localparam DC_MAX  = `SIM_DC_MAX;
    localparam IDW     = `SIM_IDW;

    localparam MAG_IN      = QIN - 1;
    localparam MAG_OUT     = QW  - 1;
    localparam MAG_MAX_IN  = (1 << MAG_IN)  - 1;
    localparam MAG_MAX_OUT = (1 << MAG_OUT) - 1;
    localparam OVF_BITS    = MAG_IN - MAG_OUT;

    reg clk = 0;
    always #5 clk = ~clk;

    reg rst_n;
    initial begin rst_n = 0; #25 rst_n = 1; end

    reg                        valid_in;
    reg  [DC_MAX*Z*QIN-1:0]    beta_in;
    wire [DC_MAX*Z*QW-1:0]     alpha_out;
    wire                       valid_out;

    // Compressed-field output ports (new)
    wire [Z*(QIN-1)-1:0]       min1_z_out;
    wire [Z*(QIN-1)-1:0]       min2_z_out;
    wire [Z*IDW-1:0]           idx1_z_out;
    wire [Z*IDW-1:0]           idx2_z_out;
    wire [Z*DC_MAX-1:0]        sgn_z_out;

    cnu_iams_z #(
        .Z      (Z),
        .QIN    (QIN),
        .QW     (QW),
        .DC_MAX (DC_MAX),
        .IDW    (IDW)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .valid_in   (valid_in),
        .beta_in    (beta_in),
        .alpha_out  (alpha_out),
        .valid_out  (valid_out),
        .min1_z_out (min1_z_out),
        .min2_z_out (min2_z_out),
        .idx1_z_out (idx1_z_out),
        .idx2_z_out (idx2_z_out),
        .sgn_z_out  (sgn_z_out)
    );

    // ---- Golden helpers -----------------------------------------------------

    task golden_scalar_iams;
        input  [DC_MAX*QIN-1:0] inp;
        output [DC_MAX*QIN-1:0] outp;
        reg signed [QIN-1:0]    msg_i   [0:DC_MAX-1];
        reg        [MAG_IN-1:0] mag_i   [0:DC_MAX-1];
        reg                     sgn_i   [0:DC_MAX-1];
        reg        [QIN-1:0]    abs_full;
        reg        [MAG_IN-1:0] min1, min2;
        integer                 idx1, idx2;
        reg                     sign_total;
        reg        [MAG_IN-1:0] delta, alpha_mag;
        reg                     out_sign;
        reg signed [QIN-1:0]    out_msg;
        integer                 ii, jj;
        begin
            for (ii = 0; ii < DC_MAX; ii = ii + 1) begin
                msg_i[ii] = inp[ii*QIN +: QIN];
                sgn_i[ii] = msg_i[ii][QIN-1];
                abs_full  = sgn_i[ii] ? (~msg_i[ii] + 1) : msg_i[ii];
                mag_i[ii] = abs_full[QIN-1] ? MAG_MAX_IN[MAG_IN-1:0]
                                            : abs_full[MAG_IN-1:0];
            end
            sign_total = 1'b0;
            for (ii = 0; ii < DC_MAX; ii = ii + 1)
                sign_total = sign_total ^ sgn_i[ii];
            min1 = {MAG_IN{1'b1}}; min2 = {MAG_IN{1'b1}};
            idx1 = 0; idx2 = 0;
            for (ii = 0; ii < DC_MAX; ii = ii + 1) begin
                if (mag_i[ii] <= min1) begin
                    min2=min1; idx2=idx1; min1=mag_i[ii]; idx1=ii;
                end else if (mag_i[ii] < min2) begin
                    min2=mag_i[ii]; idx2=ii;
                end
            end
            delta = min2 - min1;
            for (jj = 0; jj < DC_MAX; jj = jj + 1) begin
                out_sign = sign_total ^ sgn_i[jj];
                if      (jj == idx1) alpha_mag = min2;
                else if (jj == idx2) alpha_mag = min1;
                else if (delta == 0) alpha_mag = (min1 > 1) ? (min1-1) : 0;
                else                 alpha_mag = min1;
                if (alpha_mag == 0) out_msg = {QIN{1'b0}};
                else if (out_sign)  out_msg = ~({1'b0, alpha_mag}) + 1;
                else                out_msg = {1'b0, alpha_mag};
                outp[jj*QIN +: QIN] = out_msg;
            end
        end
    endtask

    function [QW-1:0] saturate;
        input [QIN-1:0] w;
        reg s;
        reg [MAG_IN-1:0] m;
        reg ovf;
        begin
            s   = w[QIN-1];
            m   = w[QIN-2:0];
            ovf = (OVF_BITS > 0) ? |m[MAG_IN-1:MAG_OUT] : 1'b0;
            if (ovf)
                saturate = s ? (~({1'b0,{MAG_OUT{1'b1}}}) + 1'b1)
                             : {1'b0,{MAG_OUT{1'b1}}};
            else
                saturate = {s, m[MAG_OUT-1:0]};
        end
    endfunction

    task golden_z;
        input  [DC_MAX*Z*QIN-1:0] inp;
        output [DC_MAX*Z*QW-1:0]  outp;
        reg [DC_MAX*QIN-1:0] z_in, z_out_qin;
        integer gz, gs;
        begin
            for (gz = 0; gz < Z; gz = gz + 1) begin
                for (gs = 0; gs < DC_MAX; gs = gs + 1)
                    z_in[gs*QIN +: QIN] = inp[gs*Z*QIN + gz*QIN +: QIN];
                golden_scalar_iams(z_in, z_out_qin);
                for (gs = 0; gs < DC_MAX; gs = gs + 1)
                    outp[gs*Z*QW + gz*QW +: QW] =
                        saturate(z_out_qin[gs*QIN +: QIN]);
            end
        end
    endtask

    // Golden for compressed fields — computed at QIN precision per z
    task golden_compressed_z;
        input  [DC_MAX*Z*QIN-1:0] inp;
        output [Z*(QIN-1)-1:0]    g_min1, g_min2;
        output [Z*IDW-1:0]        g_idx1, g_idx2;
        output [Z*DC_MAX-1:0]     g_sgn;

        reg [DC_MAX*QIN-1:0] z_in;
        reg signed [QIN-1:0] msg_i  [0:DC_MAX-1];
        reg [MAG_IN-1:0]     mag_i  [0:DC_MAX-1];
        reg                  sgn_i  [0:DC_MAX-1];
        reg [QIN-1:0]        abs_full;
        reg [MAG_IN-1:0]     min1, min2;
        integer              idx1, idx2;
        integer              gz, gs, ii;

        begin
            for (gz = 0; gz < Z; gz = gz + 1) begin
                for (gs = 0; gs < DC_MAX; gs = gs + 1)
                    z_in[gs*QIN +: QIN] = inp[gs*Z*QIN + gz*QIN +: QIN];

                for (ii = 0; ii < DC_MAX; ii = ii + 1) begin
                    msg_i[ii] = z_in[ii*QIN +: QIN];
                    sgn_i[ii] = msg_i[ii][QIN-1];
                    abs_full  = sgn_i[ii] ? (~msg_i[ii] + 1) : msg_i[ii];
                    mag_i[ii] = abs_full[QIN-1] ? MAG_MAX_IN[MAG_IN-1:0]
                                                : abs_full[MAG_IN-1:0];
                    g_sgn[gz*DC_MAX + ii] = sgn_i[ii];
                end

                min1 = {MAG_IN{1'b1}}; min2 = {MAG_IN{1'b1}};
                idx1 = 0; idx2 = 0;
                for (ii = 0; ii < DC_MAX; ii = ii + 1) begin
                    if (mag_i[ii] <= min1) begin
                        min2=min1; idx2=idx1; min1=mag_i[ii]; idx1=ii;
                    end else if (mag_i[ii] < min2) begin
                        min2=mag_i[ii]; idx2=ii;
                    end
                end
                g_min1[gz*(QIN-1) +: (QIN-1)] = min1;
                g_min2[gz*(QIN-1) +: (QIN-1)] = min2;
                g_idx1[gz*IDW     +: IDW     ] = idx1[IDW-1:0];
                g_idx2[gz*IDW     +: IDW     ] = idx2[IDW-1:0];
            end
        end
    endtask

    // ---- Check tasks --------------------------------------------------------

    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task check_out;
        input [DC_MAX*Z*QW-1:0] got, exp;
        input integer            tid;
        integer ii, jj;
        reg ok;
        begin
            ok = (got === exp);
            if (ok) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] T%0d", tid);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] T%0d", tid);
                for (jj = 0; jj < Z; jj = jj + 1)
                    for (ii = 0; ii < DC_MAX; ii = ii + 1)
                        if (got[ii*Z*QW + jj*QW +: QW] !==
                            exp[ii*Z*QW + jj*QW +: QW])
                            $display("  slot=%0d z=%0d got=%0d exp=%0d",
                                ii, jj,
                                $signed(got[ii*Z*QW + jj*QW +: QW]),
                                $signed(exp[ii*Z*QW + jj*QW +: QW]));
            end
        end
    endtask

    task check_compressed_z;
        input [Z*(QIN-1)-1:0] g_min1, g_min2;
        input [Z*IDW-1:0]     g_idx1, g_idx2;
        input [Z*DC_MAX-1:0]  g_sgn;
        input integer         tid;
        reg ok;
        begin
            ok = (min1_z_out === g_min1) && (min2_z_out === g_min2) &&
                 (idx1_z_out === g_idx1) && (idx2_z_out === g_idx2) &&
                 (sgn_z_out  === g_sgn);
            if (ok) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] T%0d compressed", tid);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] T%0d compressed", tid);
                if (min1_z_out !== g_min1)
                    $display("  min1_z: got %b exp %b", min1_z_out, g_min1);
                if (min2_z_out !== g_min2)
                    $display("  min2_z: got %b exp %b", min2_z_out, g_min2);
                if (idx1_z_out !== g_idx1)
                    $display("  idx1_z: got %b exp %b", idx1_z_out, g_idx1);
                if (idx2_z_out !== g_idx2)
                    $display("  idx2_z: got %b exp %b", idx2_z_out, g_idx2);
                if (sgn_z_out !== g_sgn)
                    $display("  sgn_z:  got %b exp %b", sgn_z_out,  g_sgn);
            end
        end
    endtask

    // ---- Stimulus -----------------------------------------------------------

    reg  [DC_MAX*Z*QW-1:0]  expected_out;
    reg  [Z*(QIN-1)-1:0]    exp_min1, exp_min2;
    reg  [Z*IDW-1:0]        exp_idx1, exp_idx2;
    reg  [Z*DC_MAX-1:0]     exp_sgn;

    integer seed = 77;
    integer trial, ri, rz;
    reg signed [QIN-1:0] rand_msg;

    initial begin
        valid_in = 0; beta_in = 0;
        @(posedge rst_n); @(posedge clk); #1;

        $display("=================================================================");
        $display("  IAMS Z-CNU Testbench  Z=%0d DC=%0d QIN=%0d QW=%0d",
                  Z, DC_MAX, QIN, QW);
        $display("=================================================================");

        // T1: reset
        @(posedge clk); #1;
        if (valid_out !== 1'b0)
            $display("[FAIL] T1: valid_out should be 0 after reset, got %b", valid_out);
        else begin pass_cnt=pass_cnt+1; $display("[PASS] T1: reset correct"); end

        // T2: all-zero
        beta_in = {DC_MAX*Z*QIN{1'b0}};
        valid_in = 1; @(posedge clk); #1;
        valid_in = 0;
        golden_z(beta_in, expected_out);
        golden_compressed_z(beta_in, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
        @(posedge clk); #1;
        check_out(alpha_out, expected_out, 2);
        check_compressed_z(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, 2);

        // T3: one z active
        beta_in = {DC_MAX*Z*QIN{1'b0}};
        beta_in[0*Z*QIN + 1*QIN +: QIN] =  8'sd10;
        beta_in[1*Z*QIN + 1*QIN +: QIN] = -8'sd20;
        beta_in[2*Z*QIN + 1*QIN +: QIN] =  8'sd5;
        beta_in[3*Z*QIN + 1*QIN +: QIN] = -8'sd15;
        valid_in = 1; @(posedge clk); #1;
        valid_in = 0;
        golden_z(beta_in, expected_out);
        golden_compressed_z(beta_in, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
        @(posedge clk); #1;
        check_out(alpha_out, expected_out, 3);
        check_compressed_z(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, 3);

        // T4: uniform across all z
        begin : t4
            integer t4_z, t4_s;
            reg signed [QIN-1:0] t4_vals [0:3];
            begin
                t4_vals[0] = 8'sd3;  t4_vals[1] = -8'sd7;
                t4_vals[2] = 8'sd12; t4_vals[3] = -8'sd5;
            end
            for (t4_z = 0; t4_z < Z; t4_z = t4_z + 1)
                for (t4_s = 0; t4_s < DC_MAX; t4_s = t4_s + 1)
                    beta_in[t4_s*Z*QIN + t4_z*QIN +: QIN] = t4_vals[t4_s];
        end
        valid_in = 1; @(posedge clk); #1;
        valid_in = 0;
        golden_z(beta_in, expected_out);
        golden_compressed_z(beta_in, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
        @(posedge clk); #1;
        check_out(alpha_out, expected_out, 4);
        check_compressed_z(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, 4);

        // T5: each z unique
        begin : t5
            integer t5_z, t5_s;
            reg signed [QIN-1:0] t5_v [0:3][0:3];
            begin
                t5_v[0][0]= 8'sd1;  t5_v[0][1]=-8'sd2;
                t5_v[0][2]= 8'sd3;  t5_v[0][3]=-8'sd4;
                t5_v[1][0]= 8'sd5;  t5_v[1][1]=-8'sd10;
                t5_v[1][2]= 8'sd7;  t5_v[1][3]=-8'sd8;
                t5_v[2][0]=-8'sd1;  t5_v[2][1]= 8'sd6;
                t5_v[2][2]=-8'sd3;  t5_v[2][3]= 8'sd9;
                t5_v[3][0]= 8'sd20; t5_v[3][1]=-8'sd30;
                t5_v[3][2]= 8'sd15; t5_v[3][3]=-8'sd25;
            end
            for (t5_z = 0; t5_z < Z; t5_z = t5_z + 1)
                for (t5_s = 0; t5_s < DC_MAX; t5_s = t5_s + 1)
                    beta_in[t5_s*Z*QIN + t5_z*QIN +: QIN] = t5_v[t5_z][t5_s];
        end
        valid_in = 1; @(posedge clk); #1;
        valid_in = 0;
        golden_z(beta_in, expected_out);
        golden_compressed_z(beta_in, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
        @(posedge clk); #1;
        check_out(alpha_out, expected_out, 5);
        check_compressed_z(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, 5);

        // T6: saturation
        begin : t6
            integer t6_z, t6_s;
            reg signed [QIN-1:0] t6_vals [0:3];
            begin
                t6_vals[0] = 8'sd100; t6_vals[1] = -8'sd100;
                t6_vals[2] = 8'sd80;  t6_vals[3] = -8'sd90;
            end
            for (t6_z = 0; t6_z < Z; t6_z = t6_z + 1)
                for (t6_s = 0; t6_s < DC_MAX; t6_s = t6_s + 1)
                    beta_in[t6_s*Z*QIN + t6_z*QIN +: QIN] = t6_vals[t6_s];
        end
        valid_in = 1; @(posedge clk); #1;
        valid_in = 0;
        golden_z(beta_in, expected_out);
        golden_compressed_z(beta_in, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
        @(posedge clk); #1;
        check_out(alpha_out, expected_out, 6);
        check_compressed_z(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, 6);

        // T7: valid_out gating
        valid_in = 0; @(posedge clk); #1;
        if (valid_out !== 1'b0)
            $display("[FAIL] T7: valid_out should be 0, got %b", valid_out);
        else begin pass_cnt=pass_cnt+1; $display("[PASS] T7: valid_out gating correct"); end

        // T8-507: 500 random trials
        $display("--- Running 500 random trials ---");
        for (trial = 8; trial < 508; trial = trial + 1) begin
            for (rz = 0; rz < Z; rz = rz + 1)
                for (ri = 0; ri < DC_MAX; ri = ri + 1) begin
                    rand_msg = $signed($random(seed)) % 64;
                    beta_in[ri*Z*QIN + rz*QIN +: QIN] = {{(QIN-QW){rand_msg[QW-1]}}, rand_msg};
                end
            valid_in = 1; @(posedge clk); #1;
            valid_in = 0;
            golden_z(beta_in, expected_out);
            golden_compressed_z(beta_in, exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn);
            @(posedge clk); #1;
            check_out(alpha_out, expected_out, trial);
            check_compressed_z(exp_min1, exp_min2, exp_idx1, exp_idx2, exp_sgn, trial);
        end

        $display("");
        $display("=================================================================");
        $display("  Results: %0d PASS,  %0d FAIL  (total %0d)",
                  pass_cnt, fail_cnt, pass_cnt+fail_cnt);
        if (fail_cnt == 0) $display("  *** ALL TESTS PASSED ***");
        else               $display("  *** FAILURES DETECTED — see above ***");
        $display("=================================================================");
        $finish;
    end

    initial begin #200_000; $display("TIMEOUT"); $finish; end

    initial begin
        $dumpfile("tb_cnu_iams_z.vcd");
        $dumpvars(0, tb_cnu_iams_z);
    end

endmodule

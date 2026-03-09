// =============================================================================
// tb_cnu_iams.v  —  Testbench for cnu_iams.v
// =============================================================================
// Verifies the IAMS check-node update (eq. 10, Cui et al. TCAS-I 2020).
//
// Test plan:
//   1. RESET check
//   2. All-zero inputs   → all outputs zero
//   3. Directed Test A   : betas = {+3, −5, +2, −6, +4, −2, +1, −4, +3, +2}
//      Expected sign pattern and magnitudes computed by hand (see comments).
//   4. Directed Test B   : Δ=0 case  (min1 == min2)
//   5. Directed Test C   : Δ≠0 case  (standard IAMS path)
//   6. 1000 random trials: compare DUT output against golden C-model in task
//   7. Saturation check  : large magnitude inputs should clamp correctly
//
// Golden model implemented as a Verilog task (behavioral reference).
// =============================================================================

`timescale 1ns / 1ps

module tb_cnu_iams;

    // ---- Parameters matching DUT -------------------------------------------
    localparam DC   = 10;
    localparam QW   = 7;
    localparam IDW  = 4;
    localparam MSG_MAX =  (1 <<< (QW-1)) - 1;  // +63 for QW=7
    localparam MSG_MIN = -(1 <<< (QW-1));       // -64 for QW=7

    // ---- Clock / reset -------------------------------------------------------
    reg clk = 0;
    always #5 clk = ~clk;   // 100 MHz

    reg rst_n;
    initial begin
        rst_n = 0;
        #25 rst_n = 1;
    end

    // ---- DUT ports ----------------------------------------------------------
    reg              valid_in;
    reg  [DC*QW-1:0] msg_flat_in;
    wire [DC*QW-1:0] msg_flat_out;
    wire             valid_out;

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
        .valid_out   (valid_out)
    );

    // ---- Helper: pack DC signed messages into flat bus ----------------------
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

    // ---- Golden model (behavioral IAMS CNU) ---------------------------------
    // Returns expected output messages as a flat bus.
    // Implements eq.(10) of the paper exactly.
    task golden_iams;
        input  [DC*QW-1:0] inp;
        output [DC*QW-1:0] outp;
        reg signed [QW-1:0] msg_i [0:DC-1];
        reg [QW-2:0] mag_i [0:DC-1];
        reg          sgn_i [0:DC-1];
        reg [QW-2:0] min1, min2;
        integer      idx1, idx2;
        reg          sign_total;
        reg [QW-2:0] delta;
        reg [QW-2:0] alpha_mag;
        reg          out_sign;
        reg signed [QW-1:0] out_msg, temp; 
        integer ii, jj;
        begin
            // Unpack
            for (ii = 0; ii < DC; ii = ii + 1) begin
                msg_i[ii] = inp[ii*QW +: QW];
                sgn_i[ii] = msg_i[ii][QW-1];
                mag_i[ii] = sgn_i[ii] ? (~msg_i[ii][QW-2:0] + 1) : msg_i[ii][QW-2:0];
            end

            // Total sign parity
            sign_total = 0;
            for (ii = 0; ii < DC; ii = ii + 1)
                sign_total = sign_total ^ sgn_i[ii];

            // Find min1, min2
            min1 = {(QW-1){1'b1}};
            min2 = {(QW-1){1'b1}};
            idx1 = 0;
            idx2 = 0;
            for (ii = 0; ii < DC; ii = ii + 1) begin
                if (mag_i[ii] <= min1) begin
                    min2 = min1; idx2 = idx1;
                    min1 = mag_i[ii]; idx1 = ii;
                end else if (mag_i[ii] < min2) begin
                    min2 = mag_i[ii]; idx2 = ii;
                end
            end

            delta = (min2 >= min1) ? (min2 - min1) : 0;

            // Compute outputs
            for (jj = 0; jj < DC; jj = jj + 1) begin
                out_sign = sign_total ^ sgn_i[jj];

                if (jj == idx1)
                    alpha_mag = min2;
                else if (jj == idx2)
                    alpha_mag = min1;
                else begin
                    if (delta == 0)
                        alpha_mag = (min1 > 0) ? (min1 - 1) : 0;
                    else
                        alpha_mag = min1;
                end

                if (alpha_mag == 0)
                    out_msg = 0;
                else if (out_sign) begin
                    temp = ~alpha_mag + 1;
                    out_msg = {1'b1, temp};
                end else
                    out_msg = {1'b0, alpha_mag};

                outp[jj*QW +: QW] = out_msg;
            end
        end
    endtask

    // ---- Check and report ---------------------------------------------------
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task check_output;
        input [DC*QW-1:0] got;
        input [DC*QW-1:0] expected;
        input [63:0]       test_id;
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
                for (ii = 0; ii < DC; ii = ii + 1) begin
                    if (got[ii*QW +: QW] !== expected[ii*QW +: QW])
                        $display("       edge[%0d]: got %0d  expected %0d",
                            ii,
                            $signed(got[ii*QW +: QW]),
                            $signed(expected[ii*QW +: QW]));
                end
            end
        end
    endtask

    // ---- Stimulus / checker -------------------------------------------------
    reg  [DC*QW-1:0] expected_out;
    integer          seed = 42;
    integer          trial;
    reg signed [QW-1:0] rand_msg;
    integer          ri;
    reg [DC*QW-1:0] inp3;
    reg [DC*QW-1:0] inp4;
    reg [DC*QW-1:0] inp5;
    integer kk;
    reg [DC*QW-1:0] inp6;
    integer kk1;
    reg [DC*QW-1:0] inp7;

    initial begin
        // ---- Setup ----------------------------------------------------------
        valid_in     = 0;
        msg_flat_in  = 0;
        @(posedge rst_n);
        @(posedge clk); #1;

        $display("=================================================================");
        $display("  IAMS CNU Testbench  (DC=%0d, QW=%0d)", DC, QW);
        $display("=================================================================");

        // ====================================================================
        // TEST 1: Reset state — valid_out should be low, output all zeros
        // ====================================================================
        @(posedge clk); #1;
        if (valid_out !== 1'b0)
            $display("[FAIL] T1: valid_out should be 0 after reset, got %0b", valid_out);
        else
            $display("[PASS] T1: reset state correct");

        // ====================================================================
        // TEST 2: All-zero inputs
        // ====================================================================
        pack_msgs(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        valid_in = 1;
        @(posedge clk); #1;  // CNU registered output after 1 clk
        valid_in = 0;
        @(posedge clk); #1;  // Wait for pipeline
        golden_iams({DC*QW{1'b0}}, expected_out);
        check_output(msg_flat_out, expected_out, 2);

        // ====================================================================
        // TEST 3: Directed — mixed signs, standard IAMS case
        //   msgs = {+3, -5, +2, -6, +4, -2, +1, -4, +3, +2}
        //   Magnitudes: 3,5,2,6,4,2,1,4,3,2
        //   min1=1 (idx1=6), min2=2 (idx2=2 first occurrence)
        //   Δ = min2−min1 = 1 ≠ 0
        //   sign_total = XOR of {0,1,0,1,0,1,0,1,0,0} = 0
        // ====================================================================
        pack_msgs(7'd3, -7'd5, 7'd2, -7'd6, 7'd4, -7'd2, 7'd1, -7'd4, 7'd3, 7'd2);
        valid_in = 1;
        @(posedge clk); #1;
        valid_in = 0;
        @(posedge clk); #1;
        begin
            inp3[0*QW +: QW] = 7'd3;
            inp3[1*QW +: QW] = -7'd5;
            inp3[2*QW +: QW] = 7'd2;
            inp3[3*QW +: QW] = -7'd6;
            inp3[4*QW +: QW] = 7'd4;
            inp3[5*QW +: QW] = -7'd2;
            inp3[6*QW +: QW] = 7'd1;
            inp3[7*QW +: QW] = -7'd4;
            inp3[8*QW +: QW] = 7'd3;
            inp3[9*QW +: QW] = 7'd2;
            golden_iams(inp3, expected_out);
            check_output(msg_flat_out, expected_out, 3);
        end

        // ====================================================================
        // TEST 4: Δ = 0 case — two edges tied at minimum
        //   msgs = {+2, −2, +5, −8, +3, +4, +6, −2, +7, +3}
        //   Magnitudes: 2,2,5,8,3,4,6,2,7,3
        //   min1=2 (idx1=0), min2=2 (idx2=1), Δ=0
        //   Edges in Ī(m) should get max(2−1, 0) = 1
        // ====================================================================
        pack_msgs(7'd2, -7'd2, 7'd5, -7'd8, 7'd3, 7'd4, 7'd6, -7'd2, 7'd7, 7'd3);
        valid_in = 1;
        @(posedge clk); #1;
        valid_in = 0;
        @(posedge clk); #1;
        begin
            inp4[0*QW +:QW]=7'd2; inp4[1*QW +:QW]=-7'd2; inp4[2*QW +:QW]=7'd5;
            inp4[3*QW +:QW]=-7'd8; inp4[4*QW +:QW]=7'd3; inp4[5*QW +:QW]=7'd4;
            inp4[6*QW +:QW]=7'd6; inp4[7*QW +:QW]=-7'd2; inp4[8*QW +:QW]=7'd7;
            inp4[9*QW +:QW]=7'd3;
            golden_iams(inp4, expected_out);
            check_output(msg_flat_out, expected_out, 4);
        end

        // ====================================================================
        // TEST 5: All same magnitude (Δ=0, min1=min2=3)
        // ====================================================================
        pack_msgs(7'd3, 7'd3, 7'd3, 7'd3, 7'd3, 7'd3, 7'd3, 7'd3, 7'd3, 7'd3);
        valid_in = 1;
        @(posedge clk); #1;
        valid_in = 0;
        @(posedge clk); #1;
        begin
            for (kk=0; kk<DC; kk=kk+1) inp5[kk*QW +:QW]=7'd3;
            golden_iams(inp5, expected_out);
            check_output(msg_flat_out, expected_out, 5);
        end

        // ====================================================================
        // TEST 6: Saturation — inputs at max magnitude ±63 (QW=7)
        // ====================================================================
        pack_msgs(7'd63, -7'd63, 7'd63, -7'd63, 7'd63, -7'd63, 7'd63, -7'd63,
                  7'd63, -7'd63);
        valid_in = 1;
        @(posedge clk); #1;
        valid_in = 0;
        @(posedge clk); #1;
        begin
            for (kk1=0; kk1<DC; kk1=kk1+2) begin
                inp6[kk1*QW +:QW]    = 7'd63;
                inp6[(kk1+1)*QW +:QW]= -7'd63;
            end
            golden_iams(inp6, expected_out);
            check_output(msg_flat_out, expected_out, 6);
        end

        // ====================================================================
        // TEST 7: single dominant minimum (all others much larger)
        //   msgs = {+1, +10, +12, +9, +8, +11, +15, +7, +10, +11}
        //   min1=1(idx1=0), min2=7(idx2=7), Δ=6≠0
        // ====================================================================
        pack_msgs(7'd1, 7'd10, 7'd12, 7'd9, 7'd8, 7'd11, 7'd15, 7'd7, 7'd10, 7'd11);
        valid_in = 1;
        @(posedge clk); #1;
        valid_in = 0;
        @(posedge clk); #1;
        begin
            inp7[0*QW +:QW]=7'd1;  inp7[1*QW +:QW]=7'd10; inp7[2*QW +:QW]=7'd12;
            inp7[3*QW +:QW]=7'd9;  inp7[4*QW +:QW]=7'd8;  inp7[5*QW +:QW]=7'd11;
            inp7[6*QW +:QW]=7'd15; inp7[7*QW +:QW]=7'd7;  inp7[8*QW +:QW]=7'd10;
            inp7[9*QW +:QW]=7'd11;
            golden_iams(inp7, expected_out);
            check_output(msg_flat_out, expected_out, 7);
        end

        // ====================================================================
        // TEST 8 – 1007: 1000 random trials
        // ====================================================================
        $display("--- Running 1000 random trials ---");
        for (trial = 8; trial < 1008; trial = trial + 1) begin
            // Generate DC random signed messages in [−63, +63]
            for (ri = 0; ri < DC; ri = ri + 1) begin
                rand_msg = ($random(seed) % 64);  // range -63..63
                msg_flat_in[ri*QW +: QW] = rand_msg;
            end
            valid_in = 1;
            @(posedge clk); #1;
            valid_in = 0;
            @(posedge clk); #1;
            golden_iams(msg_flat_in, expected_out);
            check_output(msg_flat_out, expected_out, trial);
        end

        // ====================================================================
        // TEST 1008: valid_out tracks valid_in with 1-cycle delay
        // ====================================================================
        valid_in = 0;
        @(posedge clk); #1;
        if (valid_out !== 0)
            $display("[FAIL] T1008: valid_out should be 0 when valid_in=0");
        else
            $display("[PASS] T1008: valid_out gating correct");

        // ====================================================================
        // Summary
        // ====================================================================
        $display("");
        $display("=================================================================");
        $display("  Results: %0d PASS,  %0d FAIL  (total %0d)", pass_cnt, fail_cnt,
                  pass_cnt + fail_cnt);
        if (fail_cnt == 0)
            $display("  *** ALL TESTS PASSED ***");
        else
            $display("  *** FAILURES DETECTED — see above ***");
        $display("=================================================================");
        $finish;
    end

    // ---- Timeout watchdog ---------------------------------------------------
    initial begin
        #500_000;
        $display("TIMEOUT: simulation exceeded 500 us");
        $finish;
    end

    // ---- Optional VCD dump --------------------------------------------------
    initial begin
        $dumpfile("tb_cnu_iams.vcd");
        $dumpvars(0, tb_cnu_iams);
    end

endmodule

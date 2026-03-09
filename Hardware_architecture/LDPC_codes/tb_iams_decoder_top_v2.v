// =============================================================================
// tb_iams_decoder_top.v  —  Functional Verification Testbench with Golden Model
// =============================================================================
//
// WHAT THIS TESTBENCH DOES
// -------------------------
// Provides full functional verification of the IAMS layered decoder against
// a bit-exact integer golden model running inside the testbench.
//
// The golden model runs the IDENTICAL algorithm as the RTL:
//   • Same fixed-point arithmetic (integer, no floating point)
//   • Same saturation limits  (APP: QTW=8 signed, messages: QW=7 signed)
//   • Same IAMS CNU rule (eq.10, four cases)
//   • Same layered schedule (CN-by-CN, layer-by-layer)
//
// After each decode:
//   1. RTL hard decisions (hd_out) compared against golden hard decisions
//   2. APP LLRs scanned out via scan port and compared against golden app[]
//   3. Syndrome computed from hd_out against H (should be all-zero on success)
//   4. Pass/fail reported per bit and overall
//
// TEST VECTORS
// ------------
//  TV1: All-zero codeword, LLR=+20 everywhere (high SNR, trivial convergence)
//  TV2: All-zero codeword, LLR=+5  everywhere (low SNR, should still converge)
//  TV3: Single-bit error   — VN0 gets LLR=-6 (everything else +6)
//  TV4: Two-bit error      — VN0=-5, VN3=-5
//  TV5: Max-iter no converge — adversarial LLRs that prevent convergence
//        (all LLRs = +1, syndrome forced bad: VN0=−1)
//        Expected: done asserts after IT_MAX, decoded bits = golden, syndrome MAY fail
//
// Parity-check matrix H (4×8, rate=0.5):
//   CN0: VN{0,1,3}   H row: 1 1 0 1 0 0 0 0
//   CN1: VN{1,2,4}   H row: 0 1 1 0 1 0 0 0
//   CN2: VN{0,2,5}   H row: 1 0 1 0 0 1 0 0
//   CN3: VN{3,4,6}   H row: 0 0 0 1 1 0 1 0
//
// COMPILE + RUN:
//   iverilog -o iams_decoder.out iams_decoder_top.v tb_iams_decoder_top.v
//   vvp iams_decoder.out
// =============================================================================

`timescale 1ns / 1ps

module tb_iams_decoder_top;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam N_VN   = 8;
    localparam M_CN   = 4;
    localparam DC_MAX = 3;
    localparam EDGES  = 12;
    localparam IT_MAX = 8;
    localparam QW     = 7;
    localparam QTW    = 8;
    localparam IDW    = 2;
    localparam NW     = $clog2(N_VN);
    localparam EW     = $clog2(EDGES);

    // Fixed-point saturation limits (must match RTL)
    localparam APP_MAX =  127;  //  2^(QTW-1) - 1
    localparam APP_MIN = -128;  // -2^(QTW-1)
    localparam MSG_MAX =   63;  //  2^(QW-1) - 1
    localparam MSG_MIN =  -64;  // -2^(QW-1)

    // =========================================================================
    // Clock / reset
    // =========================================================================
    reg clk = 0;
    always #5 clk = ~clk;

    reg rst_n = 0;
    initial begin #35 rst_n = 1; end

    // =========================================================================
    // DUT interface
    // =========================================================================
    reg              llr_wr_en   = 0;
    reg  [NW-1:0]    llr_wr_addr = 0;
    reg  [QTW-1:0]   llr_wr_data = 0;
    reg              start       = 0;
    reg              scan_en     = 0;
    reg  [NW-1:0]    scan_addr   = 0;

    wire             done;
    wire             hd_valid_out;
    wire [N_VN-1:0]  hd_out;
    wire [QTW-1:0]   scan_data;

    iams_decoder_top #(
        .N_VN      (N_VN),
        .M_CN      (M_CN),
        .DC_MAX    (DC_MAX),
        .EDGES     (EDGES),
        .IT_MAX    (IT_MAX),
        .QW        (QW),
        .QTW       (QTW),
        .IDW       (IDW),
        .NUM_LAYERS(M_CN)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .llr_wr_en   (llr_wr_en),
        .llr_wr_addr (llr_wr_addr),
        .llr_wr_data (llr_wr_data),
        .start       (start),
        .done        (done),
        .hd_valid_out(hd_valid_out),
        .hd_out      (hd_out),
        .scan_en     (scan_en),
        .scan_addr   (scan_addr),
        .scan_data   (scan_data)
    );

    // =========================================================================
    // Tanner graph (must match RTL ROMs exactly)
    // =========================================================================
    integer cn_nb    [0:M_CN-1][0:DC_MAX-1];
    integer cn_dc    [0:M_CN-1];
    integer edge_base[0:M_CN-1];

    initial begin
        cn_nb[0][0]=0; cn_nb[0][1]=1; cn_nb[0][2]=3; cn_dc[0]=3; edge_base[0]=0;
        cn_nb[1][0]=1; cn_nb[1][1]=2; cn_nb[1][2]=4; cn_dc[1]=3; edge_base[1]=3;
        cn_nb[2][0]=0; cn_nb[2][1]=2; cn_nb[2][2]=5; cn_dc[2]=3; edge_base[2]=6;
        cn_nb[3][0]=3; cn_nb[3][1]=4; cn_nb[3][2]=6; cn_dc[3]=3; edge_base[3]=9;
    end

    // =========================================================================
    // Golden model state
    // =========================================================================
    integer g_app  [0:N_VN-1];   // APP LLRs (integer, saturated)
    integer g_alpha[0:EDGES-1];  // CN→VN messages
    integer g_hd   [0:N_VN-1];  // hard decisions (0 or 1)

    // ---- Saturation helpers (integer) ---------------------------------------
    function integer sat_app_fn;
        input integer x;
        begin
            if      (x >  APP_MAX) sat_app_fn =  APP_MAX;
            else if (x <  APP_MIN) sat_app_fn =  APP_MIN;
            else                   sat_app_fn =  x;
        end
    endfunction

    function integer sat_msg_fn;
        input integer x;
        begin
            if      (x >  MSG_MAX) sat_msg_fn =  MSG_MAX;
            else if (x <  MSG_MIN) sat_msg_fn =  MSG_MIN;
            else                   sat_msg_fn =  x;
        end
    endfunction

    function integer iabs;
        input integer x;
        begin iabs = (x < 0) ? -x : x; end
    endfunction

    function integer isgn;
        input integer x;
        begin isgn = (x < 0) ? 1 : 0; end
    endfunction

    // =========================================================================
    // Task: golden_iams_cnu
    //   Runs the IAMS CNU update on beta_in[0..dc-1], writes results to
    //   alpha_out[0..dc-1].
    //   Exactly mirrors cnu_iams.v eq.(10).
    // =========================================================================
    integer g_beta_in  [0:DC_MAX-1];
    integer g_alpha_out[0:DC_MAX-1];

    task golden_iams_cnu;
        input integer dc;   // actual degree for this CN
        integer i, n;
        integer gmag   [0:DC_MAX-1];
        integer gsgn   [0:DC_MAX-1];
        integer gmin1, gmin2, gidx1, gidx2;
        integer gdelta;
        integer gstot;
        integer gamag, gasgn;
        begin
            // Magnitudes and signs
            for (i = 0; i < dc; i = i + 1) begin
                gsgn[i] = isgn(g_beta_in[i]);
                gmag[i] = iabs(g_beta_in[i]);
            end

            // Sign parity
            gstot = 0;
            for (i = 0; i < dc; i = i + 1)
                gstot = gstot ^ gsgn[i];

            // Find min1 / min2
            gmin1 = 32767; gmin2 = 32767; gidx1 = 0; gidx2 = 0;
            for (i = 0; i < dc; i = i + 1) begin
                if (gmag[i] <= gmin1) begin
                    gmin2 = gmin1; gidx2 = gidx1;
                    gmin1 = gmag[i]; gidx1 = i;
                end else if (gmag[i] < gmin2) begin
                    gmin2 = gmag[i]; gidx2 = i;
                end
            end

            gdelta = gmin2 - gmin1;  // always >= 0 by construction

            // Compute output per edge
            for (n = 0; n < dc; n = n + 1) begin
                gasgn = gstot ^ gsgn[n];
                if (n == gidx1)
                    gamag = gmin2;
                else if (n == gidx2)
                    gamag = gmin1;
                else if (gdelta == 0)
                    gamag = (gmin1 > 0) ? (gmin1 - 1) : 0;
                else
                    gamag = gmin1;

                // Saturate magnitude then apply sign
                gamag = (gamag > MSG_MAX) ? MSG_MAX : gamag;
                g_alpha_out[n] = gasgn ? -gamag : gamag;
            end
        end
    endtask

    // =========================================================================
    // Task: golden_decode
    //   Runs the full layered IAMS decoder on g_app[] (pre-loaded with channel
    //   LLRs) for IT_MAX iterations.  Writes final APP to g_app[] and hard
    //   decisions to g_hd[].
    // =========================================================================
    task golden_decode;
        integer it, m, e;
        integer beta, alpha_new, alpha_prev, app_new;
        begin
            // Reset alpha
            for (e = 0; e < EDGES; e = e + 1)
                g_alpha[e] = 0;

            for (it = 0; it < IT_MAX; it = it + 1) begin
                for (m = 0; m < M_CN; m = m + 1) begin
                    // Phase 1: compute beta[] for all edges of CN m
                    for (e = 0; e < cn_dc[m]; e = e + 1) begin
                        beta = g_app[cn_nb[m][e]] - g_alpha[edge_base[m]+e];
                        g_beta_in[e] = sat_msg_fn(beta);
                    end

                    // Phase 2: IAMS CNU
                    golden_iams_cnu(cn_dc[m]);

                    // Phase 3: update APP and alpha
                    for (e = 0; e < cn_dc[m]; e = e + 1) begin
                        alpha_prev = g_alpha[edge_base[m]+e];
                        alpha_new  = g_alpha_out[e];
                        app_new    = g_app[cn_nb[m][e]] + alpha_new - alpha_prev;
                        g_app[cn_nb[m][e]]          = sat_app_fn(app_new);
                        g_alpha[edge_base[m]+e]      = alpha_new;
                    end
                end
            end

            // Hard decisions
            for (e = 0; e < N_VN; e = e + 1)
                g_hd[e] = (g_app[e] < 0) ? 1 : 0;
        end
    endtask

    // =========================================================================
    // Task: load_llrs_to_dut
    //   Write llr[] array into DUT APP memory AND into g_app[] golden state.
    //   llr[] is an integer array sized N_VN.
    // =========================================================================
    integer tv_llr[0:N_VN-1];

    task load_llrs_to_dut;
        integer n;
        begin
            for (n = 0; n < N_VN; n = n + 1) begin
                @(posedge clk); #1;
                llr_wr_en   = 1;
                llr_wr_addr = n[NW-1:0];
                llr_wr_data = tv_llr[n][QTW-1:0];
                g_app[n]    = tv_llr[n];   // mirror into golden state
            end
            @(posedge clk); #1;
            llr_wr_en = 0;
        end
    endtask

    // =========================================================================
    // Task: run_dut
    //   Pulse start, wait for done (up to timeout cycles).
    //   Sets run_ok=1 if done arrived, 0 on timeout.
    // =========================================================================
    integer run_ok;
    localparam DECODE_TIMEOUT = 500;

    task run_dut;
        integer t;
        begin
            run_ok = 0;
            @(posedge clk); #1; start = 1;
            @(posedge clk); #1; start = 0;
            for (t = 0; t < DECODE_TIMEOUT && !done; t = t + 1)
                @(posedge clk);
            #1;
            run_ok = done ? 1 : 0;
        end
    endtask

    // =========================================================================
    // Task: scan_app_memory
    //   Reads all N_VN APP values out via scan port into dut_app[].
    // =========================================================================
    integer dut_app[0:N_VN-1];

    task scan_app_memory;
        integer n;
        begin
            for (n = 0; n < N_VN; n = n + 1) begin
                @(posedge clk); #1;
                scan_en   = 1;
                scan_addr = n[NW-1:0];
                @(posedge clk); #1;   // 1-cycle registered read
                scan_en = 0;
                dut_app[n] = $signed(scan_data);
            end
        end
    endtask

    // =========================================================================
    // Task: compute_syndrome
    //   Computes H × hd (mod 2) for each CN.  Returns syndrome_ok=1 if all
    //   parities are zero.
    // =========================================================================
    integer synd_ok;
    integer synd[0:M_CN-1];

    task compute_syndrome;
        input [N_VN-1:0] hd;
        integer m, e;
        begin
            synd_ok = 1;
            for (m = 0; m < M_CN; m = m + 1) begin
                synd[m] = 0;
                for (e = 0; e < cn_dc[m]; e = e + 1)
                    synd[m] = synd[m] ^ hd[cn_nb[m][e]];
                if (synd[m] != 0) synd_ok = 0;
            end
        end
    endtask

    // =========================================================================
    // Task: verify_decode
    //   Compare DUT outputs against golden model.
    //   Reports per-bit APP mismatches and hard-decision errors.
    //   Increments pass_cnt / fail_cnt.
    // =========================================================================
    integer pass_cnt, fail_cnt;   // global counters

    task verify_decode;
        input [63:0] tv_num;           // test vector number (for display)
        input        expect_converge;  // 1 = expect syndrome clean
        integer n, app_errors, hd_errors;
        integer g_hd_bit, dut_hd_bit;
        begin
            // Run golden model (already has g_app[] loaded)
            golden_decode;

            // Scan DUT APP memory
            scan_app_memory;

            // Compare APP values
            app_errors = 0;
            for (n = 0; n < N_VN; n = n + 1) begin
                if (dut_app[n] !== g_app[n]) begin
                    $display("    APP mismatch VN%0d: DUT=%0d  GOLDEN=%0d",
                             n, dut_app[n], g_app[n]);
                    app_errors = app_errors + 1;
                end
            end

            // Compare hard decisions
            hd_errors = 0;
            for (n = 0; n < N_VN; n = n + 1) begin
                g_hd_bit  = g_hd[n];
                dut_hd_bit = hd_out[n];
                if (dut_hd_bit !== g_hd_bit) begin
                    $display("    HD mismatch VN%0d: DUT=%0b  GOLDEN=%0b",
                             n, dut_hd_bit, g_hd_bit);
                    hd_errors = hd_errors + 1;
                end
            end

            // Syndrome
            compute_syndrome(hd_out);
            if (expect_converge && !synd_ok)
                $display("    WARNING: syndrome not clean (syndromes: %0b %0b %0b %0b)",
                         synd[0], synd[1], synd[2], synd[3]);

            // Overall verdict
            if (app_errors == 0 && hd_errors == 0) begin
                $display("[PASS] TV%0d: APP and HD match golden (%0s)",
                         tv_num,
                         synd_ok ? "syndrome OK" : "syndrome FAIL - expected for this TV");
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("[FAIL] TV%0d: %0d APP mismatches, %0d HD mismatches",
                         tv_num, app_errors, hd_errors);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // =========================================================================
    // Task: full_test
    //   Loads LLRs, runs DUT and golden, verifies.
    // =========================================================================
    task full_test;
        input [63:0] tv_num;
        input        expect_converge;
        begin
            load_llrs_to_dut;           // loads tv_llr[] → DUT + g_app[]
            run_dut;
            if (!run_ok) begin
                $display("[FAIL] TV%0d: done never asserted (timeout).", tv_num);
                fail_cnt = fail_cnt + 1;
            end else begin
                verify_decode(tv_num, expect_converge);
            end
        end
    endtask

    // =========================================================================
    // Main
    // =========================================================================
    initial begin
        $dumpfile("tb_iams_decoder_top.vcd");
        $dumpvars(0, tb_iams_decoder_top);
    end

    initial begin
        pass_cnt = 0; fail_cnt = 0;

        @(posedge rst_n);
        repeat(2) @(posedge clk); #1;

        $display("");
        $display("=================================================================");
        $display("  IAMS Decoder — Functional Verification with Golden Model");
        $display("  N=%0d  M=%0d  IT_MAX=%0d  QW=%0d  QTW=%0d",
                  N_VN, M_CN, IT_MAX, QW, QTW);
        $display("=================================================================");

        // =====================================================================
        // TV1: All-zero codeword, LLR=+20  (trivial high-SNR case)
        //      All APP values should increase; all HD should be 0; syndrome OK
        // =====================================================================
        $display("\n[TV1] All-zero codeword, LLR=+20 (high SNR)");
        begin : tv1
            integer n;
            for (n = 0; n < N_VN; n = n + 1) tv_llr[n] = 20;
        end
        full_test(1, 1);

        // =====================================================================
        // TV2: All-zero codeword, LLR=+5  (moderate SNR)
        // =====================================================================
        $display("\n[TV2] All-zero codeword, LLR=+5 (moderate SNR)");
        begin : tv2
            integer n;
            for (n = 0; n < N_VN; n = n + 1) tv_llr[n] = 5;
        end
        full_test(2, 1);

        // =====================================================================
        // TV3: Single-bit error on VN0  (LLR=-8, all others +8)
        //      IAMS should correct it within IT_MAX iterations
        // =====================================================================
        $display("\n[TV3] Single-bit error at VN0 (LLR=-8, rest +8)");
        begin : tv3
            integer n;
            for (n = 0; n < N_VN; n = n + 1) tv_llr[n] = 8;
            tv_llr[0] = -8;
        end
        full_test(3, 1);

        // =====================================================================
        // TV4: Two-bit errors on VN0 and VN3
        // =====================================================================
        $display("\n[TV4] Two-bit errors at VN0=-6, VN3=-6, rest +6");
        begin : tv4
            integer n;
            for (n = 0; n < N_VN; n = n + 1) tv_llr[n] = 6;
            tv_llr[0] = -6;
            tv_llr[3] = -6;
        end
        full_test(4, 0);  // may or may not converge — golden decides

        // =====================================================================
        // TV5: Adversarial — all LLRs weak (+2), VN0 wrong (-2)
        //      Likely max-iter; golden and DUT should still agree bit-for-bit
        // =====================================================================
        $display("\n[TV5] Adversarial: all LLR=+2, VN0=-2 (may not converge)");
        begin : tv5
            integer n;
            for (n = 0; n < N_VN; n = n + 1) tv_llr[n] = 2;
            tv_llr[0] = -2;
        end
        full_test(5, 0);

        // =====================================================================
        // TV6: Known valid codeword c = [0,0,0,0,0,0,0,0] with LLR all +3
        //      Low confidence but correct codeword — verify syndrome clean
        // =====================================================================
        $display("\n[TV6] All-zero codeword, LLR=+3 (low SNR)");
        begin : tv6
            integer n;
            for (n = 0; n < N_VN; n = n + 1) tv_llr[n] = 3;
        end
        full_test(6, 1);

        // =====================================================================
        // TV7: Alternating LLR signs — stress test
        //      LLR: +10 -10 +10 -10 +10 -10 +10 -10
        //      Not a valid codeword → decoder tries to find nearest
        // =====================================================================
        $display("\n[TV7] Alternating LLR signs (stress test)");
        begin : tv7
            integer n;
            for (n = 0; n < N_VN; n = n + 1)
                tv_llr[n] = (n % 2 == 0) ? 10 : -10;
        end
        full_test(7, 0);

        // =====================================================================
        // TV8: Reset recovery — decode TV1, reset mid-decode, then TV2
        // =====================================================================
        $display("\n[TV8] Reset mid-decode, then clean TV2");
        begin : tv8
            integer n;
            // Load TV1 LLRs and start
            for (n = 0; n < N_VN; n = n + 1) tv_llr[n] = 20;
            load_llrs_to_dut;

            @(posedge clk); #1; start = 1;
            @(posedge clk); #1; start = 0;

            repeat(5) @(posedge clk);  // let it run a few cycles
            rst_n = 0;
            repeat(3) @(posedge clk);
            rst_n = 1;
            repeat(2) @(posedge clk); #1;

            if (!done) begin
                $display("    Reset: done=0 after rst_n  [OK]");
            end else begin
                $display("    PROBLEM: done=1 after rst_n");
            end

            // Now run TV2 cleanly
            for (n = 0; n < N_VN; n = n + 1) tv_llr[n] = 5;
            full_test(8, 1);
        end

        // =====================================================================
        // Summary
        // =====================================================================
        $display("");
        $display("=================================================================");
        $display("  TOTAL: %0d PASS  /  %0d FAIL  (out of %0d tests)",
                  pass_cnt, fail_cnt, pass_cnt+fail_cnt);
        if (fail_cnt == 0)
            $display("  *** ALL FUNCTIONAL VERIFICATION TESTS PASSED ***");
        else
            $display("  *** FAILURES DETECTED — see mismatches above ***");
        $display("=================================================================");
        $finish;
    end

    initial begin
        #8_000_000;
        $display("[TIMEOUT] Simulation exceeded 8ms.");
        $finish;
    end

endmodule
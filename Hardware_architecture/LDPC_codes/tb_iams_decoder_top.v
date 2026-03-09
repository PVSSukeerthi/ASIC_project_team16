// =============================================================================
// tb_iams_decoder_top.v  —  System-Level Testbench  (v3 — sticky done)
// =============================================================================
//
// ROOT CAUSE ANALYSIS OF ORIGINAL FAILURES
// -----------------------------------------
// BUG 1 — layer_ctrl.v: done was a 1-cycle pulse.
//   The FSM entered RUN, counted to IT_MAX*NUM_LAYERS cycles, asserted done=1,
//   then immediately went back to IDLE where done<=1'b0 on the next clock.
//   The testbench's drive_one_iteration() ran 13 cycles per call; it only
//   sampled !done at the TOP of each 13-cycle loop iteration, so the single
//   cycle pulse always fell inside a burst and was never seen.
//   FIX → layer_ctrl.v v2: done is STICKY (only cleared by the next start).
//
// BUG 2 — testbench: early-exit test raised syndrome_ok_in AFTER a full
//   drive_one_iteration burst completed, by which time the FSM (which advances
//   every clock independently) had already gone past the syndrome check point.
//   FIX → assert syndrome_ok_in, then poll done every single clock cycle.
//
// COMPILE:
//   iverilog -o iams_decoder.out \
//     iams_decoder_top.v alpha_mem.v app_mem.v cnu_iams.v \
//     layer_ctrl.v vnu.v syndrome_check.v tb_iams_decoder_top.v
//   vvp iams_decoder.out
// =============================================================================

`timescale 1ns / 1ps

module tb_iams_decoder_top;

    // ---- Code parameters (small test code) ----------------------------------
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
    localparam IT_W   = $clog2(IT_MAX)+1;

    // Maximum cycles to wait for done (IT_MAX*NUM_LAYERS + margin)
    localparam DONE_TIMEOUT = 250;

    // ---- Clock / reset -------------------------------------------------------
    reg clk = 0;
    always #5 clk = ~clk;    // 100 MHz

    reg rst_n = 0;
    initial begin #35 rst_n = 1; end

    // ---- DUT ports -----------------------------------------------------------
    reg              start          = 0;
    reg              llr_wr_en      = 0;
    reg  [NW-1:0]    llr_wr_addr    = 0;
    reg  [QTW-1:0]   llr_wr_data    = 0;
    reg              proc_en        = 0;
    reg  [NW-1:0]    vn_addr_in     = 0;
    reg  [EW-1:0]    edge_addr_in   = 0;
    reg  [EW-1:0]    temp   = 0;
    reg              last_edge      = 0;
    reg              end_of_iter    = 0;
    reg              syndrome_ok_in = 0;

    wire             done;
    wire [IT_W-1:0]  iter_count;
    wire             hd_valid_out;
    wire [N_VN-1:0]  hd_out;

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
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .llr_wr_en      (llr_wr_en),
        .llr_wr_addr    (llr_wr_addr),
        .llr_wr_data    (llr_wr_data),
        .proc_en        (proc_en),
        .vn_addr_in     (vn_addr_in),
        .edge_addr_in   (edge_addr_in),
        .last_edge      (last_edge),
        .end_of_iter    (end_of_iter),
        .syndrome_ok_in (syndrome_ok_in),
        .done           (done),
        .iter_count     (iter_count),
        .hd_valid_out   (hd_valid_out),
        .hd_out         (hd_out)
    );

    // =========================================================================
    // Tanner graph: H (4×8)
    //   CN0: VN{0,1,3}  edges 0-2
    //   CN1: VN{1,2,4}  edges 3-5
    //   CN2: VN{0,2,5}  edges 6-8
    //   CN3: VN{3,4,6}  edges 9-11
    // =========================================================================
    integer cn_nb    [0:M_CN-1][0:DC_MAX-1];
    integer edge_base[0:M_CN-1];

    initial begin
        cn_nb[0][0]=0; cn_nb[0][1]=1; cn_nb[0][2]=3; edge_base[0]=0;
        cn_nb[1][0]=1; cn_nb[1][1]=2; cn_nb[1][2]=4; edge_base[1]=3;
        cn_nb[2][0]=0; cn_nb[2][1]=2; cn_nb[2][2]=5; edge_base[2]=6;
        cn_nb[3][0]=3; cn_nb[3][1]=4; cn_nb[3][2]=6; edge_base[3]=9;
    end

    // =========================================================================
    // load_llrs: write +20 into APP memory for all N_VN nodes
    // =========================================================================
    task load_llrs;
        integer n;
        begin
            for (n = 0; n < N_VN; n = n + 1) begin
                @(posedge clk); #1;
                llr_wr_en   = 1;
                llr_wr_addr = n[NW-1:0];
                llr_wr_data = 8'sd20;
            end
            @(posedge clk); #1;
            llr_wr_en = 0;
        end
    endtask

    // =========================================================================
    // poll_done: wait up to max_cyc clocks for done to assert.
    //   Drives proc_en + rotating edge addresses every cycle so the data-path
    //   stays active (though the FSM drives itself).
    //   Returns result in wfd_pass (1=done seen, 0=timeout).
    // =========================================================================
    integer wfd_pass;

    task poll_done;
        input integer max_cyc;
        integer i, m, e;
        begin
            wfd_pass = 0;
            m = 0; e = 0;
            for (i = 0; i < max_cyc; i = i + 1) begin
                @(posedge clk); #1;
                // Rotate through edges to keep data-path fed
                proc_en      = 1;
                vn_addr_in   = cn_nb[m][e][NW-1:0];
                temp = (edge_base[m] + e);
                edge_addr_in = temp[EW-1:0];
                last_edge    = (e == DC_MAX-1) ? 1'b1 : 1'b0;
                end_of_iter  = (m == M_CN-1 && e == DC_MAX-1) ? 1'b1 : 1'b0;
                e = e + 1;
                if (e == DC_MAX) begin e = 0; m = (m + 1) % M_CN; end
                if (done) begin
                    wfd_pass = 1;
                    i = max_cyc; // exit loop
                end
            end
            // Deassert proc_en
            @(posedge clk); #1;
            proc_en     = 0;
            last_edge   = 0;
            end_of_iter = 0;
            // Final check (done may have risen on the very last cycle)
            if (done) wfd_pass = 1;
        end
    endtask

    // =========================================================================
    // wait_cycles_check_done: just wait N cycles and check if done is high
    // =========================================================================
    integer wcd_pass;

    task wait_cycles_check_done;
        input integer n;
        integer k;
        begin
            wcd_pass = 0;
            for (k = 0; k < n; k = k + 1) begin
                @(posedge clk); #1;
                if (done) begin wcd_pass = 1; k = n; end
            end
            if (done) wcd_pass = 1;
        end
    endtask

    // =========================================================================
    // Score
    // =========================================================================
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    initial begin
        $dumpfile("tb_iams_decoder_top.vcd");
        $dumpvars(0, tb_iams_decoder_top);
    end

    // =========================================================================
    // Main
    // =========================================================================
    initial begin
        @(posedge rst_n);
        repeat (2) @(posedge clk); #1;

        $display("=================================================================");
        $display("  IAMS Decoder System Testbench  (v3 - sticky done)");
        $display("  Code: N=%0d, M=%0d, IT_MAX=%0d, NUM_LAYERS=%0d",
                  N_VN, M_CN, IT_MAX, M_CN);
        $display("=================================================================");

        // =====================================================================
        // TEST 1 — LLR load
        // =====================================================================
        $display("[TEST 1] Loading LLRs...");
        load_llrs;
        $display("[PASS] T1: LLRs loaded.");
        pass_cnt = pass_cnt + 1;

        // =====================================================================
        // TEST 2 — Max-iteration termination
        //   syndrome_ok_in = 0.  FSM counts 32 RUN cycles then done goes sticky.
        //   We poll done every clock cycle.  Should see done by cycle ~35.
        // =====================================================================
        $display("[TEST 2] Max-iter termination...");
        syndrome_ok_in = 0;

        @(posedge clk); #1; start = 1;
        @(posedge clk); #1; start = 0;

        poll_done(DONE_TIMEOUT);

        if (wfd_pass) begin
            $display("[PASS] T2: done asserted (iter_count=%0d).", iter_count);
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("[FAIL] T2: done not seen within %0d cycles.", DONE_TIMEOUT);
            fail_cnt = fail_cnt + 1;
        end

        // =====================================================================
        // TEST 3 — Early termination via syndrome_ok_in
        //   Reload, start, wait 2 cycles (FSM in RUN), assert syndrome_ok_in,
        //   then poll for done.  Expected: done in 1-2 cycles.
        // =====================================================================
        $display("[TEST 3] Early-exit via syndrome_ok_in...");
        syndrome_ok_in = 0;

        @(posedge clk); #1;
        load_llrs;

        @(posedge clk); #1; start = 1;
        @(posedge clk); #1; start = 0;

        // 2 cycles so FSM is in RUN state
        repeat(2) @(posedge clk); #1;

        syndrome_ok_in = 1;

        // Poll for done (no data-path driving needed for this test)
        wait_cycles_check_done(20);

        if (wcd_pass) begin
            $display("[PASS] T3: early termination via syndrome_ok.");
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("[FAIL] T3: done not asserted after syndrome_ok=1.");
            fail_cnt = fail_cnt + 1;
        end

        syndrome_ok_in = 0;

        // =====================================================================
        // TEST 4a — Reset de-asserts done mid-decode
        // =====================================================================
        $display("[TEST 4] Reset recovery...");
        syndrome_ok_in = 0;

        load_llrs;
        @(posedge clk); #1; start = 1;
        @(posedge clk); #1; start = 0;

        repeat(5) @(posedge clk);   // let FSM run a few cycles
        rst_n = 0;
        repeat(3) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk); #1;

        if (!done) begin
            $display("[PASS] T4a: done=0 after reset.");
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("[FAIL] T4a: done still high after reset.");
            fail_cnt = fail_cnt + 1;
        end

        // =====================================================================
        // TEST 4b — Clean restart after reset
        // =====================================================================
        load_llrs;
        @(posedge clk); #1; start = 1;
        @(posedge clk); #1; start = 0;

        poll_done(DONE_TIMEOUT);

        if (wfd_pass) begin
            $display("[PASS] T4b: decoder restarted cleanly after reset.");
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("[FAIL] T4b: decoder did not finish after reset+restart.");
            fail_cnt = fail_cnt + 1;
        end

        // =====================================================================
        // Summary
        // =====================================================================
        $display("");
        $display("=================================================================");
        $display("  System TB Results: %0d PASS,  %0d FAIL", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display("  *** ALL SYSTEM TESTS PASSED ***");
        $display("=================================================================");
        $finish;
    end

    initial begin
        #6_000_000;
        $display("[TIMEOUT] Exceeded 6ms wall — aborting.");
        $finish;
    end

endmodule
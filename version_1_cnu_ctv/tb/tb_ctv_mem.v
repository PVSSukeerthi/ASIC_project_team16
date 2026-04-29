`timescale 1ns/1ps

module tb_ctv_mem;

    // ── Parameters ────────────────────────────────────────────────────────────
    localparam Z        = 4;
    localparam QW       = 7;
    localparam QIN      = 5;
    localparam DC_MAX   = 4;
    localparam IDW      = 3;
    localparam N_LAYERS = 8;
    localparam ADRW     = 3;

    localparam MAG_W   = QW  - 1;   // 6
    localparam MAG_IN  = QIN - 1;   // 4
    localparam MAG_MAX = (1 << MAG_W) - 1;

    // ── Clock / reset ─────────────────────────────────────────────────────────
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n;
    initial begin rst_n = 0; #25 rst_n = 1; end

    // ── DUT ports ─────────────────────────────────────────────────────────────
    reg                      rd_en, wr_en;
    reg  [ADRW-1:0]          layer_rd, layer_wr;

    // Compressed inputs (match cnu_iams_z output buses)
    reg  [Z*(QIN-1)-1:0]     min1_in, min2_in;
    reg  [Z*IDW-1:0]         idx1_in, idx2_in;
    reg  [Z*DC_MAX-1:0]      sgn_in;

    wire [DC_MAX*Z*QW-1:0]   alpha_out;
    wire                     valid_out;

    ctv_mem #(
        .Z        (Z),
        .QW       (QW),
        .QIN      (QIN),
        .DC_MAX   (DC_MAX),
        .IDW      (IDW),
        .N_LAYERS (N_LAYERS),
        .ADRW     (ADRW)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .rd_en    (rd_en),
        .layer_rd (layer_rd),
        .wr_en    (wr_en),
        .layer_wr (layer_wr),
        .min1_in  (min1_in),
        .min2_in  (min2_in),
        .idx1_in  (idx1_in),
        .idx2_in  (idx2_in),
        .sgn_in   (sgn_in),
        .alpha_out(alpha_out),
        .valid_out(valid_out)
    );

    // =========================================================================
    // GOLDEN MODEL
    // Takes compressed fields directly, decompresses using Eq.(10).
    // =========================================================================
    task golden_decompress;
        input  [Z*(QIN-1)-1:0] g_min1, g_min2;
        input  [Z*IDW-1:0]     g_idx1, g_idx2;
        input  [Z*DC_MAX-1:0]  g_sgn;
        output [DC_MAX*Z*QW-1:0] outp;

        reg [DC_MAX-1:0] d_sgn;
        reg [MAG_W-1:0]  d_min1, d_min2, d_delta, d_mag;
        reg [IDW-1:0]    d_idx1, d_idx2;
        reg              d_stotal, d_osign;
        reg [QW-1:0]     d_out;
        integer          dz, ds;

        begin
            outp = {DC_MAX*Z*QW{1'b0}};
            for (dz = 0; dz < Z; dz = dz + 1) begin
                d_sgn  = g_sgn [dz*DC_MAX +: DC_MAX];
                // zero-extend MAG_IN → MAG_W (same as RTL pack_word)
                d_min1 = {{(MAG_W-MAG_IN){1'b0}}, g_min1[dz*MAG_IN +: MAG_IN]};
                d_min2 = {{(MAG_W-MAG_IN){1'b0}}, g_min2[dz*MAG_IN +: MAG_IN]};
                d_idx1 = g_idx1[dz*IDW +: IDW];
                d_idx2 = g_idx2[dz*IDW +: IDW];
                d_delta= d_min2 - d_min1;

                d_stotal = 1'b0;
                for (ds = 0; ds < DC_MAX; ds = ds + 1)
                    d_stotal = d_stotal ^ d_sgn[ds];

                for (ds = 0; ds < DC_MAX; ds = ds + 1) begin
                    d_osign = d_stotal ^ d_sgn[ds];

                    if (ds[IDW-1:0] == d_idx1)
                        d_mag = d_min2;
                    else if (ds[IDW-1:0] == d_idx2)
                        d_mag = d_min1;
                    else if (d_delta == {MAG_W{1'b0}})
                        d_mag = (d_min1 > {{(MAG_W-1){1'b0}},1'b1})
                                ? (d_min1 - 1'b1) : {MAG_W{1'b0}};
                    else
                        d_mag = d_min1;

                    if (d_mag == {MAG_W{1'b0}})
                        d_out = {QW{1'b0}};
                    else if (d_osign)
                        d_out = ~({1'b0, d_mag}) + {{(QW-1){1'b0}},1'b1};
                    else
                        d_out = {1'b0, d_mag};

                    outp[ds*Z*QW + dz*QW +: QW] = d_out;
                end
            end
        end
    endtask

    // =========================================================================
    // Helper to drive compressed inputs from scalar per-z values.
    // Builds min1_in/min2_in/idx1_in/idx2_in/sgn_in buses from arrays.
    // =========================================================================
    // Per-z compressed-field arrays (set before calling drive_compressed)
    reg [MAG_IN-1:0] arr_min1 [0:Z-1];
    reg [MAG_IN-1:0] arr_min2 [0:Z-1];
    reg [IDW-1:0]    arr_idx1 [0:Z-1];
    reg [IDW-1:0]    arr_idx2 [0:Z-1];
    reg [DC_MAX-1:0] arr_sgn  [0:Z-1];

    task drive_compressed;
        integer dz;
        begin
            for (dz = 0; dz < Z; dz = dz + 1) begin
                min1_in[dz*MAG_IN +: MAG_IN] = arr_min1[dz];
                min2_in[dz*MAG_IN +: MAG_IN] = arr_min2[dz];
                idx1_in[dz*IDW    +: IDW   ] = arr_idx1[dz];
                idx2_in[dz*IDW    +: IDW   ] = arr_idx2[dz];
                sgn_in [dz*DC_MAX +: DC_MAX] = arr_sgn [dz];
            end
        end
    endtask

    // Helper: set all z-positions to same scalars
    task set_uniform;
        input [MAG_IN-1:0] m1, m2;
        input [IDW-1:0]    i1, i2;
        input [DC_MAX-1:0] sg;
        integer uz;
        begin
            for (uz = 0; uz < Z; uz = uz + 1) begin
                arr_min1[uz] = m1; arr_min2[uz] = m2;
                arr_idx1[uz] = i1; arr_idx2[uz] = i2;
                arr_sgn [uz] = sg;
            end
            drive_compressed;
        end
    endtask

    // =========================================================================
    // Write-then-read helper (1 write cycle, 2 read latency cycles)
    // =========================================================================
    task write_read;
        input [ADRW-1:0] l_wr, l_rd;
        output [DC_MAX*Z*QW-1:0] data_out;
        begin
            wr_en = 1'b1; layer_wr = l_wr;
            rd_en = 1'b0;
            @(posedge clk); #1;
            wr_en = 1'b0;

            rd_en = 1'b1; layer_rd = l_rd;
            @(posedge clk); #1;   // cycle 1: mem read registered
            rd_en = 1'b0;
            @(posedge clk); #1;   // cycle 2: decomp+output registered

            data_out = alpha_out;
        end
    endtask

    // =========================================================================
    // Scoreboard
    // =========================================================================
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task check;
        input [DC_MAX*Z*QW-1:0] got, exp;
        input integer            tid;
        integer s, z;
        reg ok;
        begin
            ok = (got === exp);
            if (ok) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] T%0d", tid);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] T%0d", tid);
                for (s = 0; s < DC_MAX; s = s + 1)
                    for (z = 0; z < Z; z = z + 1)
                        if (got[s*Z*QW + z*QW +: QW] !==
                            exp[s*Z*QW + z*QW +: QW])
                            $display("  slot=%0d z=%0d  got=%0d  exp=%0d",
                                s, z,
                                $signed(got[s*Z*QW + z*QW +: QW]),
                                $signed(exp[s*Z*QW + z*QW +: QW]));
            end
        end
    endtask

    // =========================================================================
    // Stimulus
    // =========================================================================
    reg [DC_MAX*Z*QW-1:0] got_out, exp_out;

    // Random-trial compressed-field storage
    reg [MAG_IN-1:0] r_min1 [0:Z-1];
    reg [MAG_IN-1:0] r_min2 [0:Z-1];
    reg [IDW-1:0]    r_idx1 [0:Z-1];
    reg [IDW-1:0]    r_idx2 [0:Z-1];
    reg [DC_MAX-1:0] r_sgn  [0:Z-1];

    integer seed = 42;
    integer trial, rz;
    reg [MAG_IN-1:0] rm1, rm2;

    initial begin
        rd_en = 0; wr_en = 0;
        layer_rd = 0; layer_wr = 0;
        min1_in = 0; min2_in = 0;
        idx1_in = 0; idx2_in = 0;
        sgn_in  = 0;

        @(posedge rst_n); @(posedge clk); #1;

        $display("=================================================================");
        $display("  CTV Memory Testbench  Z=%0d DC_MAX=%0d QW=%0d QIN=%0d",
                  Z, DC_MAX, QW, QIN);
        $display("=================================================================");

        // ── T1: reset — valid_out must be 0 ───────────────────────────────────
        @(posedge clk); #1;
        if (valid_out !== 1'b0)
            $display("[FAIL] T1: valid_out nonzero after reset, got %b", valid_out);
        else begin pass_cnt=pass_cnt+1; $display("[PASS] T1: reset correct"); end

        // ── T2: all-zero compressed fields → all-zero output ──────────────────
        set_uniform(0, 0, 0, 0, {DC_MAX{1'b0}});
        write_read(3'd0, 3'd0, got_out);
        golden_decompress(min1_in, min2_in, idx1_in, idx2_in, sgn_in, exp_out);
        // capture inputs before they might change
        begin
            reg [Z*(QIN-1)-1:0] cap_m1, cap_m2;
            reg [Z*IDW-1:0]     cap_i1, cap_i2;
            reg [Z*DC_MAX-1:0]  cap_sg;
            cap_m1 = min1_in; cap_m2 = min2_in;
            cap_i1 = idx1_in; cap_i2 = idx2_in;
            cap_sg = sgn_in;
            golden_decompress(cap_m1, cap_m2, cap_i1, cap_i2, cap_sg, exp_out);
        end
        check(got_out, exp_out, 2);

        // ── T3: Δ≠0, all z same — {min1=2, min2=5, idx1=0, idx2=2, signs=0101} ─
        // Expected: slot0→min2=5(+), slot2→min1=2(+), others→min1=2(+)
        // sign_total=0, out_sign per slot = 0^sgn[slot]
        set_uniform(4'd2, 4'd5, 3'd0, 3'd2, 4'b0101);
        write_read(3'd1, 3'd1, got_out);
        begin
            reg [Z*(QIN-1)-1:0] cap_m1, cap_m2;
            reg [Z*IDW-1:0]     cap_i1, cap_i2;
            reg [Z*DC_MAX-1:0]  cap_sg;
            cap_m1 = min1_in; cap_m2 = min2_in;
            cap_i1 = idx1_in; cap_i2 = idx2_in;
            cap_sg = sgn_in;
            golden_decompress(cap_m1, cap_m2, cap_i1, cap_i2, cap_sg, exp_out);
        end
        check(got_out, exp_out, 3);

        // ── T4: Δ=0 — min1=min2=3, idx1=3, idx2=1, signs=1010 ────────────────
        // Δ=0 → non-idx edges get max(3-1,0)=2
        set_uniform(4'd3, 4'd3, 3'd3, 3'd1, 4'b1010);
        write_read(3'd2, 3'd2, got_out);
        begin
            reg [Z*(QIN-1)-1:0] cap_m1, cap_m2;
            reg [Z*IDW-1:0]     cap_i1, cap_i2;
            reg [Z*DC_MAX-1:0]  cap_sg;
            cap_m1 = min1_in; cap_m2 = min2_in;
            cap_i1 = idx1_in; cap_i2 = idx2_in;
            cap_sg = sgn_in;
            golden_decompress(cap_m1, cap_m2, cap_i1, cap_i2, cap_sg, exp_out);
        end
        check(got_out, exp_out, 4);

        // ── T5: min1=0 → all outputs zero regardless ──────────────────────────
        set_uniform(4'd0, 4'd5, 3'd1, 3'd3, 4'b1111);
        write_read(3'd3, 3'd3, got_out);
        begin
            reg [Z*(QIN-1)-1:0] cap_m1, cap_m2;
            reg [Z*IDW-1:0]     cap_i1, cap_i2;
            reg [Z*DC_MAX-1:0]  cap_sg;
            cap_m1 = min1_in; cap_m2 = min2_in;
            cap_i1 = idx1_in; cap_i2 = idx2_in;
            cap_sg = sgn_in;
            golden_decompress(cap_m1, cap_m2, cap_i1, cap_i2, cap_sg, exp_out);
        end
        check(got_out, exp_out, 5);

        // ── T6: two layers — no cross-contamination ───────────────────────────
        // Write layer 4 with pattern A
        set_uniform(4'd7, 4'd10, 3'd0, 3'd1, 4'b0011);
        wr_en = 1; layer_wr = 3'd4; @(posedge clk); #1; wr_en = 0;
        // Save pattern A compressed buses for golden
        begin
            reg [Z*(QIN-1)-1:0] pa_m1, pa_m2;
            reg [Z*IDW-1:0]     pa_i1, pa_i2;
            reg [Z*DC_MAX-1:0]  pa_sg;
            pa_m1 = min1_in; pa_m2 = min2_in;
            pa_i1 = idx1_in; pa_i2 = idx2_in;
            pa_sg = sgn_in;

            // Write layer 5 with pattern B
            set_uniform(4'd1, 4'd8, 3'd2, 3'd3, 4'b1100);
            wr_en = 1; layer_wr = 3'd5; @(posedge clk); #1; wr_en = 0;

            // Read layer 4 — must see pattern A
            rd_en = 1; layer_rd = 3'd4; @(posedge clk); #1; rd_en = 0;
            @(posedge clk); #1;
            golden_decompress(pa_m1, pa_m2, pa_i1, pa_i2, pa_sg, exp_out);
            check(alpha_out, exp_out, 6);
        end

        // ── T7: read unwritten layer (layer 7) — expect all-zero ──────────────
        rd_en = 1; layer_rd = 3'd7; @(posedge clk); #1; rd_en = 0;
        @(posedge clk); #1;
        check(alpha_out, {DC_MAX*Z*QW{1'b0}}, 7);

        // ── T8: overwrite layer 1 — new value must appear ─────────────────────
        set_uniform(4'd9, 4'd12, 3'd1, 3'd3, 4'b0110);
        write_read(3'd1, 3'd1, got_out);
        begin
            reg [Z*(QIN-1)-1:0] cap_m1, cap_m2;
            reg [Z*IDW-1:0]     cap_i1, cap_i2;
            reg [Z*DC_MAX-1:0]  cap_sg;
            cap_m1 = min1_in; cap_m2 = min2_in;
            cap_i1 = idx1_in; cap_i2 = idx2_in;
            cap_sg = sgn_in;
            golden_decompress(cap_m1, cap_m2, cap_i1, cap_i2, cap_sg, exp_out);
        end
        check(got_out, exp_out, 8);

        // ── T9: valid_out gating ───────────────────────────────────────────────
        rd_en = 0; @(posedge clk); #1;
        if (valid_out !== 1'b0)
            $display("[FAIL] T9: valid_out should be 0, got %b", valid_out);
        else begin pass_cnt=pass_cnt+1; $display("[PASS] T9: valid_out gating OK"); end

        // ── T10: each z has different fields (independence check) ──────────────
        begin : t10
            integer t10z;
            reg [MAG_IN-1:0] t10_m1 [0:3];
            reg [MAG_IN-1:0] t10_m2 [0:3];
            reg [IDW-1:0]    t10_i1 [0:3];
            reg [IDW-1:0]    t10_i2 [0:3];
            reg [DC_MAX-1:0] t10_sg [0:3];
            begin
                t10_m1[0]=4'd1; t10_m2[0]=4'd6; t10_i1[0]=3'd0; t10_i2[0]=3'd1; t10_sg[0]=4'b0001;
                t10_m1[1]=4'd3; t10_m2[1]=4'd7; t10_i1[1]=3'd2; t10_i2[1]=3'd0; t10_sg[1]=4'b1010;
                t10_m1[2]=4'd2; t10_m2[2]=4'd2; t10_i1[2]=3'd1; t10_i2[2]=3'd3; t10_sg[2]=4'b0110;
                t10_m1[3]=4'd5; t10_m2[3]=4'd8; t10_i1[3]=3'd3; t10_i2[3]=3'd2; t10_sg[3]=4'b1111;
            end
            for (t10z = 0; t10z < Z; t10z = t10z + 1) begin
                arr_min1[t10z] = t10_m1[t10z]; arr_min2[t10z] = t10_m2[t10z];
                arr_idx1[t10z] = t10_i1[t10z]; arr_idx2[t10z] = t10_i2[t10z];
                arr_sgn [t10z] = t10_sg[t10z];
            end
            drive_compressed;
            write_read(3'd0, 3'd0, got_out);
            begin
                reg [Z*(QIN-1)-1:0] cap_m1, cap_m2;
                reg [Z*IDW-1:0]     cap_i1, cap_i2;
                reg [Z*DC_MAX-1:0]  cap_sg;
                cap_m1 = min1_in; cap_m2 = min2_in;
                cap_i1 = idx1_in; cap_i2 = idx2_in;
                cap_sg = sgn_in;
                golden_decompress(cap_m1, cap_m2, cap_i1, cap_i2, cap_sg, exp_out);
            end
            check(got_out, exp_out, 10);
        end

        // ── T11-510: 500 random trials ─────────────────────────────────────────
        $display("--- 500 random trials ---");
        for (trial = 11; trial < 511; trial = trial + 1) begin : rand_loop
            reg [Z*(QIN-1)-1:0] cap_m1, cap_m2;
            reg [Z*IDW-1:0]     cap_i1, cap_i2;
            reg [Z*DC_MAX-1:0]  cap_sg;

            for (rz = 0; rz < Z; rz = rz + 1) begin
                rm1 = $random(seed) % (1 << MAG_IN);
                rm2 = $random(seed) % (1 << MAG_IN);
                // Ensure min1 <= min2
                if (rm1 > rm2) begin
                    arr_min1[rz] = rm2; arr_min2[rz] = rm1;
                end else begin
                    arr_min1[rz] = rm1; arr_min2[rz] = rm2;
                end
                arr_idx1[rz] = $random(seed) % DC_MAX;
                arr_idx2[rz] = $random(seed) % DC_MAX;
                arr_sgn [rz] = $random(seed);
            end
            drive_compressed;

            cap_m1 = min1_in; cap_m2 = min2_in;
            cap_i1 = idx1_in; cap_i2 = idx2_in;
            cap_sg = sgn_in;

            write_read(3'd0, 3'd0, got_out);
            golden_decompress(cap_m1, cap_m2, cap_i1, cap_i2, cap_sg, exp_out);
            check(got_out, exp_out, trial);
        end

        // ── Summary ───────────────────────────────────────────────────────────
        $display("");
        $display("=================================================================");
        $display("  Results: %0d PASS  %0d FAIL  (total %0d)",
                  pass_cnt, fail_cnt, pass_cnt+fail_cnt);
        if (fail_cnt == 0) $display("  *** ALL TESTS PASSED ***");
        else               $display("  *** FAILURES DETECTED ***");
        $display("=================================================================");
        $finish;
    end

    initial begin #2_000_000; $display("TIMEOUT"); $finish; end
    initial begin $dumpfile("tb_ctv_mem.vcd"); $dumpvars(0, tb_ctv_mem); end

endmodule

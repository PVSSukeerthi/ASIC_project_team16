// =============================================================================
// tb_decompressor.v  —  Testbench for CTV Decompressor (Fig. 12)
// =============================================================================
`timescale 1ns/1ps
`include "pkg_iams.vh"

module tb_decompressor;

    localparam Z          = `LIFTING_Z;
    localparam QW         = `QW;
    localparam IDW        = `IDW;
    localparam CTV_WORD_W = `CTV_WORD_W;
    localparam MAG_W      = QW - 1;

    reg  [CTV_WORD_W-1:0] ctv_word;
    wire [Z*QW-1:0]       msg_out;

    decompressor #(.Z(Z), .QW(QW), .IDW(IDW), .CTV_WORD_W(CTV_WORD_W)) dut (
        .ctv_word (ctv_word),
        .msg_out  (msg_out)
    );

    // Golden model
    task automatic ref_decomp;
        input  [CTV_WORD_W-1:0] word;
        output [Z*QW-1:0] ref_out;
        integer e;
        reg                   df;
        reg [IDW-1:0]         i1;
        reg [MAG_W-1:0]       m1, m2;
        reg [Z-1:0]           sgns;
        reg [MAG_W-1:0]       amag;
        reg                   osign;
        begin
            df   = word[0];
            i1   = word[1 +: IDW];
            m2   = word[1 + IDW +: MAG_W];
            m1   = word[1 + IDW + MAG_W +: MAG_W];
            sgns = word[1 + IDW + MAG_W*2 +: Z];
            for (e = 0; e < Z; e = e + 1) begin
                if (e[IDW-1:0] == i1)
                    amag = m2;
                else if (df == 1'b0)
                    amag = (m1 > 1) ? (m1 - 1) : 0;
                else
                    amag = m1;
                osign = sgns[e];
                if (amag == 0)
                    ref_out[e*QW +: QW] = {QW{1'b0}};
                else if (osign)
                    ref_out[e*QW +: QW] = {1'b1, (~amag + 1'b1)};
                else
                    ref_out[e*QW +: QW] = {1'b0, amag};
            end
        end
    endtask

    integer errors, t;
    reg [Z*QW-1:0] expected;

    // Helper: build ctv_word
    function automatic [CTV_WORD_W-1:0] build_word;
        input [Z-1:0]     signs;
        input [MAG_W-1:0] min1, min2;
        input [IDW-1:0]   idx1;
        input             delta_flag;
        begin
            build_word = 0;
            build_word[0]                              = delta_flag;
            build_word[1 +: IDW]                       = idx1;
            build_word[1 + IDW +: MAG_W]               = min2;
            build_word[1 + IDW + MAG_W +: MAG_W]       = min1;
            build_word[1 + IDW + MAG_W*2 +: Z]         = signs;
        end
    endfunction

    initial begin
        errors = 0;
        $display("=== Decompressor Testbench ===");

        // Test 1: delta_flag=1 (Δ≠0), idx1=0, min1=5, min2=9
        ctv_word = build_word({Z{1'b0}}, 6'd5, 6'd9, 4'd0, 1'b1);
        #1;
        ref_decomp(ctv_word, expected);
        if (msg_out !== expected) begin
            $display("FAIL T1: got=%h exp=%h", msg_out[0*QW +: QW], expected[0*QW +: QW]);
            errors = errors + 1;
        end else $display("PASS T1 (delta_flag=1, idx1=0)");

        // Test 2: delta_flag=0 (Δ=0), idx1=3, min1=4, min2=4
        ctv_word = build_word({Z{1'b0}}, 6'd4, 6'd4, 4'd3, 1'b0);
        #1;
        ref_decomp(ctv_word, expected);
        if (msg_out !== expected) begin
            $display("FAIL T2 (delta=0)"); errors = errors + 1;
        end else $display("PASS T2 (delta_flag=0, idx1=3)");

        // Test 3: all signs set
        ctv_word = build_word({Z{1'b1}}, 6'd7, 6'd12, 4'd1, 1'b1);
        #1;
        ref_decomp(ctv_word, expected);
        if (msg_out !== expected) begin
            $display("FAIL T3 (all signs)"); errors = errors + 1;
        end else $display("PASS T3 (all signs negative)");

        // Test 4: min1=0, delta=0 → all non-idx1 should be 0
        ctv_word = build_word({Z{1'b0}}, 6'd0, 6'd0, 4'd2, 1'b0);
        #1;
        ref_decomp(ctv_word, expected);
        if (msg_out !== expected) begin
            $display("FAIL T4 (min1=0)"); errors = errors + 1;
        end else $display("PASS T4 (min1=0, all outputs 0)");

        // Test 5: 100 random tests
        for (t = 0; t < 100; t = t + 1) begin
            ctv_word = $random;
            // clip min1,min2 to 6-bit range and ensure they're valid
            #1;
            ref_decomp(ctv_word, expected);
            if (msg_out !== expected) begin
                $display("FAIL T5 random t=%0d", t); errors = errors + 1;
            end
        end
        $display("T5: 100 random tests");

        if (errors == 0) $display("ALL PASSED");
        else             $display("FAILED: %0d errors", errors);
        $finish;
    end

endmodule
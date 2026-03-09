// =============================================================================
// iams_decoder_top.v  —  Self-Contained Layered IAMS Decoder  (v2)
// =============================================================================
// Fully functional layered decoder.  An internal FSM drives all address
// generation; the testbench only needs to:
//   1. Write channel LLRs via llr_wr_en / llr_wr_addr / llr_wr_data
//   2. Pulse start for one clock cycle
//   3. Wait for done to assert
//   4. Read decoded bits via scan port  (scan_en + scan_addr → scan_data)
//      OR check hard decisions on hd_out[] once hd_valid_out is asserted.
//
// Layered schedule (one check node per "layer"):
//   For each iteration i = 0..IT_MAX-1:
//     For each CN m = 0..NUM_LAYERS-1:
//       Phase 1 (DC_MAX cycles): read beta[e] = APP[vn[e]] - alpha[edge[e]]
//       Phase 2 (1 cycle):       run IAMS CNU combinatorially on all beta[]
//       Phase 3 (DC_MAX cycles): write APP[vn[e]] += alpha_new[e] - alpha[edge[e]]
//                                write alpha_mem[edge[e]] = alpha_new[e]
//
// The H matrix adjacency is stored in ROM arrays (cn_vn_rom / cn_edge_rom)
// initialised from $readmemh or hardcoded for simulation.
//
// Ports:
//   clk, rst_n          — 100 MHz / active-low reset
//   llr_wr_*            — channel LLR write channel (before start)
//   start               — one-cycle pulse to begin decoding
//   done                — sticky: asserts when finished, cleared by next start
//   hd_valid_out        — hard decisions valid (same cycle as done)
//   hd_out[N_VN-1:0]    — hard-decision bits (MSB of each APP LLR)
//   scan_en             — enable scan-read of APP memory post-decode
//   scan_addr           — VN address to read
//   scan_data           — APP LLR value at scan_addr (registered, 1-cycle latency)
// =============================================================================

`timescale 1ns / 1ps

module iams_decoder_top #(
    parameter N_VN      = 8,     // Number of variable nodes
    parameter M_CN      = 4,     // Number of check nodes (= NUM_LAYERS)
    parameter DC_MAX    = 3,     // Maximum check-node degree
    parameter EDGES     = 12,    // Total edges in H  (sum of all row degrees)
    parameter IT_MAX    = 8,     // Maximum decoding iterations
    parameter QW        = 7,     // CN→VN message width  (signed)
    parameter QTW       = 8,     // APP LLR width        (signed)
    parameter IDW       = 2,     // ceil(log2(DC_MAX))
    parameter NUM_LAYERS= M_CN   // synonym
) (
    input  wire              clk,
    input  wire              rst_n,

    // ---- Channel LLR load (before start) ------------------------------------
    input  wire              llr_wr_en,
    input  wire [$clog2(N_VN)-1:0]   llr_wr_addr,
    input  wire [QTW-1:0]   llr_wr_data,

    // ---- Control ------------------------------------------------------------
    input  wire              start,
    output reg               done,       // sticky until next start

    // ---- Hard-decision output -----------------------------------------------
    output reg               hd_valid_out,
    output reg  [N_VN-1:0]  hd_out,

    // ---- Scan port (read APP memory after done) -----------------------------
    input  wire              scan_en,
    input  wire [$clog2(N_VN)-1:0]   scan_addr,
    output wire [QTW-1:0]   scan_data
);

    // =========================================================================
    // Local parameters
    // =========================================================================
    localparam NW  = $clog2(N_VN);
    localparam EW  = $clog2(EDGES);
    localparam ITW = $clog2(IT_MAX) + 1;
    localparam MW  = $clog2(M_CN);
    localparam DCW = $clog2(DC_MAX) + 1;  // edge index within a layer

    // Saturation limits
    localparam signed [QTW-1:0] APP_MAX =  (1 <<< (QTW-1)) - 1;   //  127
    localparam signed [QTW-1:0] APP_MIN = -(1 <<< (QTW-1));        // -128
    localparam signed [QW-1:0]  MSG_MAX =  (1 <<< (QW-1))  - 1;   //   63
    localparam signed [QW-1:0]  MSG_MIN = -(1 <<< (QW-1));         //  -64

    // =========================================================================
    // H-matrix adjacency ROMs (hardcoded for the 4×8 test code)
    //   cn_vn_rom[m][e]   = VN index for CN m, edge e
    //   cn_edge_rom[m][e] = global edge index for CN m, edge e
    //   cn_dc_rom[m]      = degree of CN m
    //
    //   H:  CN0→{0,1,3}  CN1→{1,2,4}  CN2→{0,2,5}  CN3→{3,4,6}
    // =========================================================================
    reg [NW-1:0]  cn_vn_rom   [0:M_CN-1][0:DC_MAX-1];
    reg [EW-1:0]  cn_edge_rom [0:M_CN-1][0:DC_MAX-1];
    reg [DCW-1:0] cn_dc_rom   [0:M_CN-1];

    reg [QW-2:0] amag;
    reg          asgn;
            
    initial begin
        cn_vn_rom[0][0]=0; cn_vn_rom[0][1]=1; cn_vn_rom[0][2]=3;
        cn_vn_rom[1][0]=1; cn_vn_rom[1][1]=2; cn_vn_rom[1][2]=4;
        cn_vn_rom[2][0]=0; cn_vn_rom[2][1]=2; cn_vn_rom[2][2]=5;
        cn_vn_rom[3][0]=3; cn_vn_rom[3][1]=4; cn_vn_rom[3][2]=6;

        cn_edge_rom[0][0]=0;  cn_edge_rom[0][1]=1;  cn_edge_rom[0][2]=2;
        cn_edge_rom[1][0]=3;  cn_edge_rom[1][1]=4;  cn_edge_rom[1][2]=5;
        cn_edge_rom[2][0]=6;  cn_edge_rom[2][1]=7;  cn_edge_rom[2][2]=8;
        cn_edge_rom[3][0]=9;  cn_edge_rom[3][1]=10; cn_edge_rom[3][2]=11;

        cn_dc_rom[0]=3; cn_dc_rom[1]=3; cn_dc_rom[2]=3; cn_dc_rom[3]=3;
    end

    // =========================================================================
    // APP LLR memory  (N_VN × QTW)
    // =========================================================================
    reg signed [QTW-1:0] app_mem [0:N_VN-1];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < N_VN; init_i = init_i + 1)
            app_mem[init_i] = 0;
    end

    // =========================================================================
    // Alpha (CN→VN message) memory  (EDGES × QW)
    // =========================================================================
    reg signed [QW-1:0] alpha_mem [0:EDGES-1];

    integer init_j;
    initial begin
        for (init_j = 0; init_j < EDGES; init_j = init_j + 1)
            alpha_mem[init_j] = 0;
    end

    // =========================================================================
    // Beta accumulation buffer  (DC_MAX × QW) — holds VN→CN messages
    // for one layer before CNU runs
    // =========================================================================
    reg signed [QW-1:0] beta_buf  [0:DC_MAX-1];
    reg signed [QW-1:0] alpha_buf [0:DC_MAX-1];  // new alpha from CNU
    reg signed [QW-1:0] alpha_old [0:DC_MAX-1];  // old alpha (for APP update)

    // =========================================================================
    // FSM
    // =========================================================================
    localparam IDLE      = 3'd0;
    localparam LOAD_BETA = 3'd1;   // Phase 1: read beta for each edge
    localparam CNU_RUN   = 3'd2;   // Phase 2: CNU combinatorial (1 cycle)
    localparam WRITEBACK = 3'd3;   // Phase 3: write APP + alpha
    localparam DONE_ST   = 3'd4;

    reg [2:0]   state;
    reg [ITW-1:0] iter;
    reg [MW-1:0]  layer;
    reg [DCW-1:0] edge_cnt;  // edge index within current layer

    // =========================================================================
    // IAMS CNU — purely combinational, operates on beta_buf[]
    // =========================================================================
    // Magnitudes and signs
    reg [QW-2:0] mag   [0:DC_MAX-1];
    reg          bsgn  [0:DC_MAX-1];
    reg [QW-2:0] min1, min2;
    reg [IDW-1:0] idx1, idx2;
    reg [QW-2:0] delta_cnu;
    reg          sign_tot;
    reg signed [QW-1:0] alpha_new_comb [0:DC_MAX-1];
    reg signed [QTW:0]  b_wide;
    reg signed [QW-1:0] b_sat;
    reg signed [QTW+QW-1:0] app_wide;
    integer dn;                         
    integer ci, cn;
    always @(*) begin
        // Compute magnitudes and signs
        for (ci = 0; ci < DC_MAX; ci = ci + 1) begin
            bsgn[ci] = beta_buf[ci][QW-1];
            mag[ci]  = bsgn[ci] ? (~beta_buf[ci][QW-2:0] + 1'b1)
                                 : beta_buf[ci][QW-2:0];
        end

        // Cumulative sign parity
        sign_tot = 1'b0;
        for (ci = 0; ci < DC_MAX; ci = ci + 1)
            sign_tot = sign_tot ^ bsgn[ci];

        // Find min1 / min2
        min1 = {(QW-1){1'b1}};
        min2 = {(QW-1){1'b1}};
        idx1 = {IDW{1'b0}};
        idx2 = {IDW{1'b0}};
        for (ci = 0; ci < DC_MAX; ci = ci + 1) begin
            if (mag[ci] <= min1) begin
                min2 = min1; idx2 = idx1;
                min1 = mag[ci]; idx1 = ci[IDW-1:0];
            end else if (mag[ci] < min2) begin
                min2 = mag[ci]; idx2 = ci[IDW-1:0];
            end
        end

        delta_cnu = (min2 >= min1) ? (min2 - min1) : {(QW-1){1'b0}};

        // Compute output messages
        for (cn = 0; cn < DC_MAX; cn = cn + 1) begin
            asgn = sign_tot ^ bsgn[cn];
            if (cn[IDW-1:0] == idx1)
                amag = min2;
            else if (cn[IDW-1:0] == idx2)
                amag = min1;
            else if (delta_cnu == {(QW-1){1'b0}})
                amag = (min1 > 0) ? (min1 - 1'b1) : {(QW-1){1'b0}};
            else
                amag = min1;

            // Reconstruct signed message
            if (amag == {(QW-1){1'b0}})
                alpha_new_comb[cn] = {QW{1'b0}};
            else if (asgn)
                alpha_new_comb[cn] = {1'b1, (~amag + 1'b1)};
            else
                alpha_new_comb[cn] = {1'b0, amag};
        end
    end

    // =========================================================================
    // Saturation helpers
    // =========================================================================
    function signed [QTW-1:0] sat_app;
        input signed [QTW+QW-1:0] x;
        begin
            if (x > $signed({{(QW){1'b0}}, APP_MAX}))
                sat_app = APP_MAX;
            else if (x < $signed({{(QW){1'b1}}, APP_MIN}))
                sat_app = APP_MIN;
            else
                sat_app = x[QTW-1:0];
        end
    endfunction

    // =========================================================================
    // Main FSM
    // =========================================================================
    integer fi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            done         <= 1'b0;
            hd_valid_out <= 1'b0;
            iter         <= 0;
            layer        <= 0;
            edge_cnt     <= 0;
            for (fi = 0; fi < DC_MAX; fi = fi + 1) begin
                beta_buf[fi]  <= 0;
                alpha_buf[fi] <= 0;
                alpha_old[fi] <= 0;
            end
        end else begin
            case (state)

                // ----------------------------------------------------------------
                IDLE: begin
                    hd_valid_out <= 1'b0;
                    // LLR write during idle
                    if (llr_wr_en)
                        app_mem[llr_wr_addr] <= llr_wr_data;
                    // Start decoding
                    if (start) begin
                        done     <= 1'b0;
                        iter     <= 0;
                        layer    <= 0;
                        edge_cnt <= 0;
                        // Reset alpha memory
                        for (fi = 0; fi < EDGES; fi = fi + 1)
                            alpha_mem[fi] <= 0;
                        state <= LOAD_BETA;
                    end
                end

                // ----------------------------------------------------------------
                // LOAD_BETA: for edge_cnt = 0..dc-1, compute and buffer beta
                //   beta[e] = APP[vn[e]] - alpha[edge[e]]
                //   (saturated to MSG range)
                // ----------------------------------------------------------------
                LOAD_BETA: begin
                    if (llr_wr_en)   // allow LLR writes to continue if desired
                        app_mem[llr_wr_addr] <= llr_wr_data;

                    begin
                        b_wide = {{1{app_mem[cn_vn_rom[layer][edge_cnt]][QTW-1]}},
                                   app_mem[cn_vn_rom[layer][edge_cnt]]}
                               - {{(QTW-QW+2){alpha_mem[cn_edge_rom[layer][edge_cnt]][QW-1]}},
                                   alpha_mem[cn_edge_rom[layer][edge_cnt]]};
                        // Saturate to message width
                        if (b_wide > $signed({{(QTW-QW+2){1'b0}}, MSG_MAX}))
                            b_sat = MSG_MAX;
                        else if (b_wide < $signed({{(QTW-QW+2){1'b1}}, MSG_MIN}))
                            b_sat = MSG_MIN;
                        else
                            b_sat = b_wide[QW-1:0];

                        beta_buf[edge_cnt]  <= b_sat;
                        alpha_old[edge_cnt] <= alpha_mem[cn_edge_rom[layer][edge_cnt]];
                    end

                    if (edge_cnt == cn_dc_rom[layer] - 1) begin
                        edge_cnt <= 0;
                        state    <= CNU_RUN;
                    end else begin
                        edge_cnt <= edge_cnt + 1'b1;
                    end
                end

                // ----------------------------------------------------------------
                // CNU_RUN: latch the combinational CNU outputs into alpha_buf
                // ----------------------------------------------------------------
                CNU_RUN: begin
                    for (fi = 0; fi < DC_MAX; fi = fi + 1)
                        alpha_buf[fi] <= alpha_new_comb[fi];
                    state <= WRITEBACK;
                end

                // ----------------------------------------------------------------
                // WRITEBACK: for edge_cnt = 0..dc-1
                //   APP[vn[e]] += alpha_new[e] - alpha_old[e]   (saturated)
                //   alpha_mem[edge[e]] = alpha_new[e]
                // ----------------------------------------------------------------
                WRITEBACK: begin
                    begin
                        app_wide = {{(QW){app_mem[cn_vn_rom[layer][edge_cnt]][QTW-1]}},
                                     app_mem[cn_vn_rom[layer][edge_cnt]]}
                                 + {{(QTW){alpha_buf[edge_cnt][QW-1]}},
                                     alpha_buf[edge_cnt]}
                                 - {{(QTW){alpha_old[edge_cnt][QW-1]}},
                                     alpha_old[edge_cnt]};
                        app_mem[cn_vn_rom[layer][edge_cnt]] <= sat_app(app_wide);
                        alpha_mem[cn_edge_rom[layer][edge_cnt]] <= alpha_buf[edge_cnt];
                    end

                    if (edge_cnt == cn_dc_rom[layer] - 1) begin
                        edge_cnt <= 0;
                        // Advance layer / iteration
                        if (layer == M_CN - 1) begin
                            layer <= 0;
                            if (iter == IT_MAX - 1) begin
                                state <= DONE_ST;
                            end else begin
                                iter  <= iter + 1'b1;
                                state <= LOAD_BETA;
                            end
                        end else begin
                            layer <= layer + 1'b1;
                            state <= LOAD_BETA;
                        end
                    end else begin
                        edge_cnt <= edge_cnt + 1'b1;
                    end
                end

                // ----------------------------------------------------------------
                // DONE_ST: output hard decisions, assert done
                // ----------------------------------------------------------------
                DONE_ST: begin
                    begin
                        for (dn = 0; dn < N_VN; dn = dn + 1)
                            hd_out[dn] <= app_mem[dn][QTW-1];  // sign bit = hard decision
                    end
                    hd_valid_out <= 1'b1;
                    done         <= 1'b1;
                    state        <= IDLE;

                    // LLR write still permitted
                    if (llr_wr_en)
                        app_mem[llr_wr_addr] <= llr_wr_data;
                end

                default: state <= IDLE;
            endcase
        end
    end

    // =========================================================================
    // Scan port — read any APP location after done
    // =========================================================================
    reg [NW-1:0] scan_addr_r;
    always @(posedge clk) begin
        if (scan_en) scan_addr_r <= scan_addr;
    end
    assign scan_data = app_mem[scan_addr_r];

endmodule
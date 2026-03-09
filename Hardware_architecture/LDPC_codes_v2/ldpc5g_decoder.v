`include "ldpc5g_pkg.v"

module ldpc5g_decoder #(
    parameter Z        = `Z,
    parameter NB       = `NB,
    parameter MB       = `MB,
    parameter Q        = `Q,
    parameter QTILDE   = `QTILDE,
    parameter DC_MAX_P = `DC_MAX,
    parameter L        = `L_AFTER,
    parameter ITMAX    = `ITMAX_IMPL,
    parameter D_THRESH = `D_THRESHOLD,
    parameter NB_CORE  = 10
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    // Channel LLRs: Z*NB Q-bit values, element n at [n*Q +: Q]
    input  wire [Z*NB*Q-1:0]       llr_in,
    // Decoded codeword
    output wire [Z*NB-1:0]         codeword_out,
    output wire                    dec_valid,
    output wire [3:0]              iter_count
);

localparam NB_EXT = NB - NB_CORE;  // 42

// ============================================================
// Controller
// ============================================================
wire [4:0]  layer_no;
wire [3:0]  iter_cnt;
wire        layer_merge;
wire        is_core;
wire [3:0]  dc1_actual, dc2_actual;
wire [5:0]  base_row1, base_row2;
wire        sel_w, sel_r;
wire        init_phase;
wire        dec_done;
wire        rst_n_ctrl;
wire [3:0]  D_thresh;
wire        parity_ok;

controller #(.L(L), .ITMAX(ITMAX)) u_ctrl (
    .clk        (clk),        .rst_n      (rst_n),
    .start      (start),      .parity_ok  (parity_ok),
    .layer_no   (layer_no),   .iter_cnt   (iter_cnt),
    .layer_merge(layer_merge),.is_core    (is_core),
    .dc1_actual (dc1_actual), .dc2_actual (dc2_actual),
    .base_row1  (base_row1),  .base_row2  (base_row2),
    .sel_w      (sel_w),      .sel_r      (sel_r),
    .init_phase (init_phase), .dec_done   (dec_done),
    .rst_n_mem  (rst_n_ctrl), .D_thresh   (D_thresh)
);

assign iter_count = iter_cnt;
assign dec_valid  = dec_done;

// ============================================================
// Sign-extend LLRs to QTILDE bits for APP init
// init_data [Z*NB*QTILDE-1:0], layout app[z][c]=(z*NB+c)
// llr_in layout: linear index n = z*NB+c, llr[n] at [n*Q +: Q]
// ============================================================
wire [Z*NB*QTILDE-1:0] app_init_data;

genvar gi;
generate
    for (gi = 0; gi < Z*NB; gi = gi+1) begin : llr_ext
        assign app_init_data[gi*QTILDE +: QTILDE] =
            {{(QTILDE-Q){llr_in[gi*Q+Q-1]}}, llr_in[gi*Q +: Q]};
    end
endgenerate

// ============================================================
// APP Memory
// ============================================================
wire [NB-1:0]          app_wr_en;
wire [Z*NB*QTILDE-1:0] app_wr_data;
wire [4*16-1:0]        rd_nio_sel;
wire [16*Z*QTILDE-1:0] rd_out;
wire [Z*NB*QTILDE-1:0] app_full;

// Read network selector: simplified direct col 0..15
genvar gsel;
generate
    for (gsel = 0; gsel < 16; gsel = gsel+1) begin : sel_map
        assign rd_nio_sel[gsel*4 +: 4] = gsel[3:0];
    end
endgenerate

app_memory #(
    .Z(Z), .NB(NB), .NB_CORE(NB_CORE), .NB_EXT(NB_EXT), .QTILDE(QTILDE)
) u_app_mem (
    .clk        (clk),
    .rst_n      (rst_n & rst_n_ctrl),
    .init_en    (init_phase),
    .init_data  (app_init_data),
    .wr_en      (app_wr_en),
    .wr_data    (app_wr_data),
    .sel        (sel_w),
    .rd_nio_sel (rd_nio_sel),
    .rd_out     (rd_out),
    .app_full   (app_full)
);

// ============================================================
// CTV Memory
// ============================================================
wire [Z*(Q-1)-1:0]    ctv_wr_min1_1, ctv_wr_min2_1;
wire [Z*4-1:0]        ctv_wr_idx1_1, ctv_wr_idx2_1;
wire [Z*DC_MAX_P-1:0] ctv_wr_signs_1;
wire [Z*(Q-1)-1:0]    ctv_wr_min1_2, ctv_wr_min2_2;
wire [Z*4-1:0]        ctv_wr_idx1_2, ctv_wr_idx2_2;
wire [Z*DC_MAX_P-1:0] ctv_wr_signs_2;

wire [Z*(Q-1)-1:0]    ctv_rd_min1_1, ctv_rd_min2_1;
wire [Z*4-1:0]        ctv_rd_idx1_1, ctv_rd_idx2_1;
wire [Z*DC_MAX_P-1:0] ctv_rd_signs_1;
wire [Z*(Q-1)-1:0]    ctv_rd_min1_2, ctv_rd_min2_2;
wire [Z*4-1:0]        ctv_rd_idx1_2, ctv_rd_idx2_2;
wire [Z*DC_MAX_P-1:0] ctv_rd_signs_2;
wire                  ctv_rd_valid;

wire [Z*DC_MAX_P*Q-1:0] ctv1_out, ctv2_out;
wire                    cnu_valid;

// Extract sign bits from CTV outputs for compressed storage
genvar gz, gd;
generate
    for (gz = 0; gz < Z; gz = gz+1) begin : ctv_signs
        for (gd = 0; gd < DC_MAX_P; gd = gd+1) begin : ctv_sign_d
            assign ctv_wr_signs_1[gz*DC_MAX_P+gd] =
                ctv1_out[(gz*DC_MAX_P+gd)*Q + Q-1];
            assign ctv_wr_signs_2[gz*DC_MAX_P+gd] =
                ctv2_out[(gz*DC_MAX_P+gd)*Q + Q-1];
        end
    end
endgenerate

// CNU min outputs feed directly to CTV write ports
wire [Z*(Q-1)-1:0] cnu_min1_1, cnu_min2_1;
wire [Z*4-1:0]     cnu_idx1_1, cnu_idx2_1;
wire [Z*(Q-1)-1:0] cnu_min1_2, cnu_min2_2;
wire [Z*4-1:0]     cnu_idx1_2, cnu_idx2_2;

assign ctv_wr_min1_1 = cnu_min1_1; assign ctv_wr_min2_1 = cnu_min2_1;
assign ctv_wr_idx1_1 = cnu_idx1_1; assign ctv_wr_idx2_1 = cnu_idx2_1;
assign ctv_wr_min1_2 = cnu_min1_2; assign ctv_wr_min2_2 = cnu_min2_2;
assign ctv_wr_idx1_2 = cnu_idx1_2; assign ctv_wr_idx2_2 = cnu_idx2_2;

ctv_memory #(
    .Z(Z), .Q(Q), .DC_MAX_P(DC_MAX_P),
    .DC_ORT(`DC_MAX_ORT), .L_TOTAL(L),
    .L_CORE_ORT(15), .LOG2_DC(`LOG2_DC_MAX), .LOG2_DC_O(`LOG2_DC_ORT)
) u_ctv_mem (
    .clk          (clk),
    .rst_n        (rst_n & rst_n_ctrl),
    .wr_layer     (layer_no),
    .wr_en        (cnu_valid),
    .wr_min1_1    (ctv_wr_min1_1), .wr_min2_1    (ctv_wr_min2_1),
    .wr_idx1_1    (ctv_wr_idx1_1), .wr_idx2_1    (ctv_wr_idx2_1),
    .wr_signs_1   (ctv_wr_signs_1),
    .wr_min1_2    (ctv_wr_min1_2), .wr_min2_2    (ctv_wr_min2_2),
    .wr_idx1_2    (ctv_wr_idx1_2), .wr_idx2_2    (ctv_wr_idx2_2),
    .wr_signs_2   (ctv_wr_signs_2),
    .wr_set2_valid(layer_merge),
    .rd_layer     (layer_no),
    .rd_en        (1'b1),
    .rd_min1_1    (ctv_rd_min1_1), .rd_min2_1    (ctv_rd_min2_1),
    .rd_idx1_1    (ctv_rd_idx1_1), .rd_idx2_1    (ctv_rd_idx2_1),
    .rd_signs_1   (ctv_rd_signs_1),
    .rd_min1_2    (ctv_rd_min1_2), .rd_min2_2    (ctv_rd_min2_2),
    .rd_idx1_2    (ctv_rd_idx1_2), .rd_idx2_2    (ctv_rd_idx2_2),
    .rd_signs_2   (ctv_rd_signs_2),
    .rd_valid     (ctv_rd_valid)
);

// ============================================================
// Decompressor: reconstruct old alpha messages
// ============================================================
wire [Z*DC_MAX_P*Q-1:0] alpha_old1, alpha_old2;

decompressor #(.Z(Z),.DC_MAX_P(DC_MAX_P),.Q(Q),.LOG2_DC(`LOG2_DC_MAX)) u_decomp1 (
    .signs    (ctv_rd_signs_1), .min1 (ctv_rd_min1_1), .min2 (ctv_rd_min2_1),
    .idx1     (ctv_rd_idx1_1),  .idx2 (ctv_rd_idx2_1), .dc_actual(dc1_actual),
    .alpha    (alpha_old1)
);
decompressor #(.Z(Z),.DC_MAX_P(DC_MAX_P),.Q(Q),.LOG2_DC(`LOG2_DC_MAX)) u_decomp2 (
    .signs    (ctv_rd_signs_2), .min1 (ctv_rd_min1_2), .min2 (ctv_rd_min2_2),
    .idx1     (ctv_rd_idx1_2),  .idx2 (ctv_rd_idx2_2), .dc_actual(dc2_actual),
    .alpha    (alpha_old2)
);

// ============================================================
// VNU: beta[z][d] = clamp(app[z][d%NB] - alpha_old[z][d])
// ============================================================
wire [Z*DC_MAX_P*Q-1:0] vtc1, vtc2;

generate
    for (gz = 0; gz < Z; gz = gz+1) begin : vnu_z
        for (gi = 0; gi < DC_MAX_P; gi = gi+1) begin : vnu_d
            vnu #(.Q(Q), .QTILDE(QTILDE)) u_vnu1 (
                .app_in  (app_full[(gz*NB+(gi%NB))*QTILDE +: QTILDE]),
                .ctv_old (alpha_old1[(gz*DC_MAX_P+gi)*Q +: Q]),
                .vtc_out (vtc1[(gz*DC_MAX_P+gi)*Q +: Q])
            );
            vnu #(.Q(Q), .QTILDE(QTILDE)) u_vnu2 (
                .app_in  (app_full[(gz*NB+(gi%NB))*QTILDE +: QTILDE]),
                .ctv_old (alpha_old2[(gz*DC_MAX_P+gi)*Q +: Q]),
                .vtc_out (vtc2[(gz*DC_MAX_P+gi)*Q +: Q])
            );
        end
    end
endgenerate

// ============================================================
// CNU Top
// ============================================================
cnu_top #(.Z(Z), .DC_MAX_P(DC_MAX_P), .Q(Q)) u_cnu (
    .clk         (clk),
    .rst_n       (rst_n),
    .vtc1        (vtc1),  .dc1_actual(dc1_actual), .use_oms1(1'b0),
    .vtc2        (vtc2),  .dc2_actual(dc2_actual), .use_oms2(1'b0),
    .layer_merge (layer_merge),
    .ctv1_out    (ctv1_out),  .ctv2_out  (ctv2_out),
    .min1_1      (cnu_min1_1),.min2_1    (cnu_min2_1),
    .idx1_1      (cnu_idx1_1),.idx2_1    (cnu_idx2_1),
    .min1_2      (cnu_min1_2),.min2_2    (cnu_min2_2),
    .idx1_2      (cnu_idx1_2),.idx2_2    (cnu_idx2_2),
    .valid_out   (cnu_valid)
);

// ============================================================
// APP Update
// ============================================================
generate
    for (gz = 0; gz < Z; gz = gz+1) begin : app_upd_z
        for (gi = 0; gi < DC_MAX_P; gi = gi+1) begin : app_upd_d
            wire [QTILDE-1:0] app_new_w;
            app_update #(.Q(Q), .QTILDE(QTILDE)) u_app_upd (
                .app_in  (app_full[(gz*NB+(gi%NB))*QTILDE +: QTILDE]),
                .ctv_old (alpha_old1[(gz*DC_MAX_P+gi)*Q +: Q]),
                .ctv_new (ctv1_out[(gz*DC_MAX_P+gi)*Q +: Q]),
                .app_out (app_new_w)
            );
            assign app_wr_en[gi%NB]                      = cnu_valid && (gi < dc1_actual);
            assign app_wr_data[(gz*NB+(gi%NB))*QTILDE +: QTILDE] = app_new_w;
        end
    end
endgenerate

// ============================================================
// Parity Check
// ============================================================
parity_check #(.Z(Z),.NB(NB),.MB(MB),.QTILDE(QTILDE)) u_pc (
    .app        (app_full),
    .codeword_hd(codeword_out),
    .parity_ok  (parity_ok)
);

endmodule

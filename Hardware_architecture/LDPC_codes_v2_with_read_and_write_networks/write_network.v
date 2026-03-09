// =============================================================================
// write_network.v  —  Write Network (Fig. 11)
// =============================================================================
// Cui et al. TCAS-I 2020, Fig. 11 and Section V-D.
//
// The Write Network performs the inverse routing of the Read Network:
// after the VNU processes the Z-wide CTV messages, it produces updated
// APP LLR increments. The Write Network applies an inverse LBS (right
// barrel shift = left shift by Z-s) to un-rotate the symbols back to
// canonical APP memory layout, then writes each Z-symbol vector to its
// corresponding variable-node column row.
//
// For layered decoding (APP update): only one VN column is updated per
// edge slot per layer, so write_en_mask controls which slots actually
// write back.
//
// Ports
//   vnu_out_flat [DC_MAX*Z*QTW-1:0]   — VNU updated APP values per slot
//   vn_cols      [DC_MAX*COL_W-1:0]   — VN column index per CNU slot
//   shifts       [DC_MAX*SEL_W-1:0]   — LBS shift per slot (same as read)
//   degree       [DEG_W-1:0]          — actual CN degree
//   wr_en_global                      — global write enable
//   --- Outputs to APP memory ---
//   app_wr_en    [DC_MAX-1:0]         — per-slot APP write enable
//   app_wr_col   [DC_MAX*COL_W-1:0]  — which column to write
//   app_wr_data  [DC_MAX*Z*QTW-1:0]  — unrotated data per slot
// =============================================================================

`timescale 1ns/1ps
`include "pkg_iams.vh"

module write_network #(
    parameter Z        = `LIFTING_Z,
    parameter QTW      = `QTW,
    parameter DC_MAX   = `DC_MAX,
    parameter BG_COLS  = `BG_COLS,
    parameter COL_W    = 6,
    parameter SEL_W    = 6,
    parameter DEG_W    = 4
) (
    input  wire [DC_MAX*Z*QTW-1:0] vnu_out_flat,
    input  wire [DC_MAX*COL_W-1:0] vn_cols,
    input  wire [DC_MAX*SEL_W-1:0] shifts,
    input  wire [DEG_W-1:0]        degree,
    input  wire                    wr_en_global,

    output wire [DC_MAX-1:0]        app_wr_en,
    output wire [DC_MAX*COL_W-1:0]  app_wr_col,
    output wire [DC_MAX*Z*QTW-1:0]  app_wr_data
);

    genvar slot;
    generate
        for (slot = 0; slot < DC_MAX; slot = slot + 1) begin : wn_slot

            wire [COL_W-1:0] col_sel;
            wire [SEL_W-1:0] sh_val;
            assign col_sel = vn_cols[slot*COL_W +: COL_W];
            assign sh_val  = shifts [slot*SEL_W +: SEL_W];

            // Inverse shift: rotate right by sh_val = rotate left by (Z - sh_val)
            wire [SEL_W-1:0] inv_sh;
            assign inv_sh = (sh_val == {SEL_W{1'b0}}) ? {SEL_W{1'b0}}
                                                       : (Z - sh_val);

            wire [Z*QTW-1:0] slot_data;
            assign slot_data = vnu_out_flat[slot*Z*QTW +: Z*QTW];

            wire [Z*QTW-1:0] unrotated;
            lbs #(.Z(Z), .QW(QTW), .SW(SEL_W)) u_lbs_inv (
                .data_in  (slot_data),
                .shift    (inv_sh),
                .data_out (unrotated)
            );

            assign app_wr_en  [slot]              = wr_en_global & (slot < degree);
            assign app_wr_col [slot*COL_W +: COL_W] = col_sel;
            assign app_wr_data[slot*Z*QTW +: Z*QTW] = unrotated;
        end
    endgenerate

endmodule

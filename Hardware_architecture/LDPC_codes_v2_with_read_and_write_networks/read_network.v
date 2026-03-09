// =============================================================================
// read_network.v  —  Read Network (Fig. 16)
// =============================================================================
// Cui et al. TCAS-I 2020, Fig. 16 and Section V-D.
//
// Overview
// --------
// For each layer (check-node row), the Read Network gathers APP LLR values
// from the APP memory and routes them to the DC_MAX CNU inputs.
//
// In the lifted code, one CNU processes Z parallel edges simultaneously.
// The APP memory outputs Z symbols per read (one full circulant column).
// The Read Network selects the correct Z-symbol vector for each of the
// DC_MAX connected variable-node columns, then optionally applies a
// Left Barrel Shift (LBS) to align them according to H's circulant shifts.
//
// Structure (Fig. 16)
// -------------------
//   Level 0: For each of DC_MAX CNU slots, a mux selects which APP row
//            (variable-node column) to present. Input = all BG_COLS rows
//            from APP memory; select = vn_col[slot] from H-matrix ROM.
//
//   Level 1: Apply LBS to rotate the selected Z-symbol vector by the
//            circulant shift value s_ij stored in the H-matrix ROM.
//
// The result is DC_MAX × Z × QTW bits sent to the DC_MAX CNU inputs.
//
// This module is COMBINATIONAL after the APP memory read registers.
// The APP memory read data is assumed already registered (1-cycle latency).
//
// Ports
//   app_rows_flat [BG_COLS*Z*QTW-1:0] — all APP rows concatenated
//   vn_cols [DC_MAX*COL_W-1:0]         — VN column index per CNU slot
//   shifts  [DC_MAX*SEL_W-1:0]         — LBS shift per CNU slot
//   degree  [DEG_W-1:0]                — actual degree of this CN (for masking)
//   cnu_in  [DC_MAX*Z*QTW-1:0]         — output to CNU inputs
//   valid_mask [DC_MAX-1:0]            — 1 = slot active (degree mask)
// =============================================================================

`timescale 1ns/1ps
`include "pkg_iams.vh"

module read_network #(
    parameter Z        = `LIFTING_Z,
    parameter QTW      = `QTW,
    parameter DC_MAX   = `DC_MAX,
    parameter BG_COLS  = `BG_COLS,    // 52
    parameter COL_W    = 6,           // ceil(log2(52))
    parameter SEL_W    = 6,           // ceil(log2(Z))
    parameter DEG_W    = 4            // ceil(log2(DC_MAX+1))
) (
    // All APP memory rows (combinational, must be driven from registered APP mem output)
    input  wire [BG_COLS*Z*QTW-1:0] app_rows_flat,

    // H-matrix routing: which VN column and shift for each CNU slot
    input  wire [DC_MAX*COL_W-1:0]  vn_cols,   // vn_cols[k] = column for slot k
    input  wire [DC_MAX*SEL_W-1:0]  shifts,    // shifts[k]  = circulant shift for slot k

    // Active degree of current CN (slots >= degree are masked)
    input  wire [DEG_W-1:0]         degree,

    // Outputs to DC_MAX CNU inputs
    output wire [DC_MAX*Z*QTW-1:0]  cnu_in,
    output wire [DC_MAX-1:0]        valid_mask
);

    // -------------------------------------------------------------------------
    // Unpack APP rows into 2-D array: app_row[col][z_sym]
    // -------------------------------------------------------------------------
    wire [Z*QTW-1:0] app_row [0:BG_COLS-1];
    genvar gc;
    generate
        for (gc = 0; gc < BG_COLS; gc = gc + 1) begin : unpack_rows
            assign app_row[gc] = app_rows_flat[gc*Z*QTW +: Z*QTW];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Level-0 mux + Level-1 LBS for each CNU slot
    // -------------------------------------------------------------------------
    genvar slot;
    generate
        for (slot = 0; slot < DC_MAX; slot = slot + 1) begin : rn_slot

            // Extract routing info for this slot
            wire [COL_W-1:0] col_sel;
            wire [SEL_W-1:0] sh_val;
            assign col_sel = vn_cols[slot*COL_W +: COL_W];
            assign sh_val  = shifts [slot*SEL_W +: SEL_W];

            // Level-0 mux: select APP row
            wire [Z*QTW-1:0] selected_row;
            assign selected_row = app_row[col_sel];

            // Level-1 LBS: rotate by circulant shift
            wire [Z*QTW-1:0] shifted_row;
            lbs #(.Z(Z), .QW(QTW), .SW(SEL_W)) u_lbs (
                .data_in  (selected_row),
                .shift    (sh_val),
                .data_out (shifted_row)
            );

            // Connect to CNU input bus
            assign cnu_in[slot*Z*QTW +: Z*QTW] = shifted_row;

            // Valid mask: active if slot < degree
            assign valid_mask[slot] = (slot < degree) ? 1'b1 : 1'b0;

        end
    endgenerate

endmodule

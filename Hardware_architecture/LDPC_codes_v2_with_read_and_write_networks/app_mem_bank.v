// =============================================================================
// app_mem_bank.v  —  APP LLR Memory with Selective-Shift Banking (Fig. 15)
// =============================================================================
// Cui et al. TCAS-I 2020, Fig. 15 and Section V-C.
//
// Purpose
// -------
// Stores APP (a-posteriori) LLR values for all N_VN = N × Z variable nodes.
// Organised as BG_COLS rows (N=52), each row holding Z=52 symbols of QTW=8 bits.
//
// Architecture (Fig. 15)
// ----------------------
// Each row is split across THREE memory banks:
//
//   Core bank    : Z × (QTW-2) bits = 52 × 6 = 312 bits/row
//                  Holds the "core" (most-significant) bits of each APP value.
//
//   Ext bank 0   : Z × 2 bits = 52 × 2 = 104 bits/row
//                  Holds the 2 LSBs of the first half of symbols.
//
//   Ext bank 1   : Z × 2 bits = 104 bits/row
//                  Holds the 2 LSBs of the second half (barrel-shifted copy).
//
// The extension bits bank uses a "selective-shift" scheme:
//   - The ext bank is cyclically rotated by `sel` positions before read/write.
//   - This reduces the Read Network fan-in from Z to Z/2+overlap ≈ 16 (BG2).
//   - sel is provided by the controller based on circulant shift values.
//
// On read: data_out = { core_bits, ext_bits } reassembled per symbol.
// On write: write_data split and stored to appropriate banks.
//
// Ports
//   clk                    — clock
//   rst_n                  — reset
//   --- Write ---
//   wr_en                  — write enable
//   wr_row  [ROW_AW-1:0]   — row address (0..BG_COLS-1)
//   wr_data [Z*QTW-1:0]    — Z symbols to write
//   wr_sel  [SEL_W-1:0]    — barrel-shift for ext bank write alignment
//   --- Read ---
//   rd_en                  — read enable
//   rd_row  [ROW_AW-1:0]   — row address
//   rd_sel  [SEL_W-1:0]    — barrel-shift for ext bank read alignment
//   rd_data [Z*QTW-1:0]    — Z symbols (1-cycle latency)
//   rd_valid               — output valid
//   --- Channel init ---
//   init_en                — load channel LLR directly (bypasses shift)
//   init_row [ROW_AW-1:0]
//   init_data[Z*QTW-1:0]
// =============================================================================

`timescale 1ns/1ps
`include "pkg_iams.vh"

module app_mem_bank #(
    parameter Z        = `LIFTING_Z,   // 52
    parameter QTW      = `QTW,         // 8 (full APP width)
    parameter CORE_W   = QTW - 2,      // 6 bits stored in core bank
    parameter EXT_W    = 2,            // 2 LSBs in ext banks
    parameter ROWS     = `BG_COLS,     // 52 variable-node rows
    parameter ROW_AW   = 6,            // ceil(log2(52))
    parameter SEL_W    = 6             // ceil(log2(Z))
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // Write port
    input  wire                  wr_en,
    input  wire [ROW_AW-1:0]     wr_row,
    input  wire [Z*QTW-1:0]      wr_data,
    input  wire [SEL_W-1:0]      wr_sel,

    // Read port
    input  wire                  rd_en,
    input  wire [ROW_AW-1:0]     rd_row,
    input  wire [SEL_W-1:0]      rd_sel,
    output reg  [Z*QTW-1:0]      rd_data,
    output reg                   rd_valid,

    // Channel initialisation (no shift)
    input  wire                  init_en,
    input  wire [ROW_AW-1:0]     init_row,
    input  wire [Z*QTW-1:0]      init_data
);

    // -------------------------------------------------------------------------
    // Memory banks
    // -------------------------------------------------------------------------
    // Core bank: Z × CORE_W bits per row
    reg [Z*CORE_W-1:0] core_bank [0:ROWS-1];
    // Extension bank 0
    reg [Z*EXT_W-1:0]  ext_bank0 [0:ROWS-1];
    // Extension bank 1 (selective-shifted copy)
    reg [Z*EXT_W-1:0]  ext_bank1 [0:ROWS-1];

    // -------------------------------------------------------------------------
    // Helper: barrel-shift a Z-symbol word left by `sh` positions
    // (same logic as lbs.v but inline for ext banks with EXT_W-wide symbols)
    // -------------------------------------------------------------------------
    function automatic [Z*EXT_W-1:0] ext_shift;
        input [Z*EXT_W-1:0] din;
        input [SEL_W-1:0]   sh;
        integer fe;
        reg [SEL_W:0] raw;
        begin
            for (fe = 0; fe < Z; fe = fe + 1) begin
                raw = fe + sh;
                if (raw >= Z) raw = raw - Z;
                ext_shift[fe*EXT_W +: EXT_W] = din[raw*EXT_W +: EXT_W];
            end
        end
    endfunction

    // -------------------------------------------------------------------------
    // Split write data into core + ext
    // -------------------------------------------------------------------------
    wire [Z*CORE_W-1:0] wr_core;
    wire [Z*EXT_W-1:0]  wr_ext;
    genvar gi;
    generate
        for (gi = 0; gi < Z; gi = gi + 1) begin : split_wr
            // Symbol layout: [QTW-1:EXT_W] = core, [EXT_W-1:0] = ext
            assign wr_core[gi*CORE_W +: CORE_W] = wr_data[gi*QTW + EXT_W +: CORE_W];
            assign wr_ext [gi*EXT_W  +: EXT_W]  = wr_data[gi*QTW +: EXT_W];
        end
    endgenerate

    wire [Z*CORE_W-1:0] init_core;
    wire [Z*EXT_W-1:0]  init_ext_flat;
    generate
        for (gi = 0; gi < Z; gi = gi + 1) begin : split_init
            assign init_core[gi*CORE_W +: CORE_W] = init_data[gi*QTW + EXT_W +: CORE_W];
            assign init_ext_flat[gi*EXT_W +: EXT_W] = init_data[gi*QTW +: EXT_W];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Write logic
    // -------------------------------------------------------------------------
    integer wi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (wi = 0; wi < ROWS; wi = wi + 1) begin
                core_bank[wi]  <= {Z*CORE_W{1'b0}};
                ext_bank0[wi]  <= {Z*EXT_W{1'b0}};
                ext_bank1[wi]  <= {Z*EXT_W{1'b0}};
            end
        end else if (init_en) begin
            // Channel init: write directly, no shift
            core_bank[init_row] <= init_core;
            ext_bank0[init_row] <= init_ext_flat;
            ext_bank1[init_row] <= ext_shift(init_ext_flat, {SEL_W{1'b0}});
        end else if (wr_en) begin
            // Iterative update: apply selective shift on ext banks
            core_bank[wr_row] <= wr_core;
            ext_bank0[wr_row] <= wr_ext;
            ext_bank1[wr_row] <= ext_shift(wr_ext, wr_sel);
        end
    end

    // -------------------------------------------------------------------------
    // Read logic (1-cycle registered, applies inverse shift on ext)
    // -------------------------------------------------------------------------
    // Registered intermediate
    reg [Z*CORE_W-1:0] rd_core_r;
    reg [Z*EXT_W-1:0]  rd_ext0_r;
    reg [Z*EXT_W-1:0]  rd_ext1_r;
    reg [SEL_W-1:0]    rd_sel_r;
    reg                rd_v1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_core_r <= {Z*CORE_W{1'b0}};
            rd_ext0_r <= {Z*EXT_W{1'b0}};
            rd_ext1_r <= {Z*EXT_W{1'b0}};
            rd_sel_r  <= {SEL_W{1'b0}};
            rd_v1     <= 1'b0;
            rd_valid  <= 1'b0;
            rd_data   <= {Z*QTW{1'b0}};
        end else begin
            rd_v1 <= rd_en;
            if (rd_en) begin
                rd_core_r <= core_bank[rd_row];
                rd_ext0_r <= ext_bank0[rd_row];
                rd_ext1_r <= ext_bank1[rd_row];
                rd_sel_r  <= rd_sel;
            end

            // Cycle 2: assemble output
            rd_valid <= rd_v1;
            if (rd_v1) begin
                // Use ext_bank1 which is pre-shifted; apply rd_sel to align
                // (ext_bank1 stored with wr_sel applied; read with rd_sel)
                // For read: use ext_bank0 and shift by rd_sel to get aligned
                begin : assemble
                    integer ae;
                    reg [Z*EXT_W-1:0] aligned_ext;
                    aligned_ext = ext_shift(rd_ext0_r, rd_sel_r);
                    for (ae = 0; ae < Z; ae = ae + 1) begin
                        rd_data[ae*QTW +: QTW] <= {
                            rd_core_r[ae*CORE_W +: CORE_W],
                            aligned_ext[ae*EXT_W +: EXT_W]
                        };
                    end
                end
            end
        end
    end

endmodule

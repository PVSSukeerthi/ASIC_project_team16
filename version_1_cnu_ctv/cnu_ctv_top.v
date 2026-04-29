// =============================================================================
// cnu_ctv_top.v  —  Top: CNU + CTV Memory
// =============================================================================
// Cui et al., IEEE TCAS-I 2020, Section V-A.
//
// This module connects cnu_iams_z (the Z-parallel IAMS check-node unit) to
// ctv_mem (the compressed check-to-variable message memory + decompressor).
//
// ── What this block does ─────────────────────────────────────────────────────
//
//   WRITE path (driven by the layer scheduler):
//     1. VNU passes β messages → beta_in (DC_MAX*Z*QIN bits)
//     2. cnu_iams_z runs IAMS Eq.(10) for all Z positions in parallel.
//     3. One cycle later, cnu_iams_z outputs:
//          • alpha_out  — DC_MAX*Z*QW signed CTV messages (for the VNU
//                         to immediately use to update APP this same cycle)
//          • valid_out  — high for one cycle
//          • min1/min2/idx1/idx2/sgn — compressed representation for storage
//     4. ctv_mem stores the compressed representation on the same posedge
//        that valid_out goes high (wr_en is driven from valid_out externally,
//        or the scheduler can assert it independently — see notes below).
//
//   READ path (driven by the layer scheduler):
//     1. Scheduler asserts rd_en and puts the layer index on layer_rd.
//     2. Cycle 1: ctv_mem synchronously reads the compressed word.
//     3. Cycle 2: decompressor reconstructs DC_MAX*Z*QW signed messages.
//        ctv_alpha_out is valid when ctv_valid_out goes high.
//
// ── Latency summary ──────────────────────────────────────────────────────────
//
//   CNU:       valid_in → (1 cycle) → cnu_valid_out, alpha_out, min*/idx*/sgn
//   CTV write: cnu_valid_out asserted → ctv_mem writes on that posedge
//   CTV read:  rd_en asserted → (2 cycles) → ctv_valid_out, ctv_alpha_out
//
// ── Integration notes for your teammate (VNU + scheduler side) ───────────────
//
//   1. beta_in and valid_in come from the VNU output (post-saturation to QIN).
//   2. Connect cnu_alpha_out → VNU to update APP values (same cycle as write).
//   3. Drive wr_en and layer_wr from the scheduler:
//        - Assert wr_en one cycle AFTER asserting valid_in to cnu (i.e. on the
//          same cycle that cnu_valid_out is high). Simplest: tie wr_en to
//          cnu_valid_out if the layer index is stable.
//        - layer_wr should hold the index of the layer just processed.
//   4. Drive rd_en and layer_rd from the scheduler at the start of each layer.
//        - ctv_alpha_out is valid 2 cycles after rd_en.
//   5. The decompressed ctv_alpha_out replaces the "old alpha" that the VNU
//        subtracts from APP before sending beta to this CNU.
//   6. Do NOT connect ctv_alpha_out directly to cnu_alpha_out — they carry
//        messages from DIFFERENT iterations/layers.
//
// ── Parameter guide ──────────────────────────────────────────────────────────
//
//   Z        — QC lifting factor                       52  (BG2)
//   QIN      — VTC β message width (sign + q mag bits)  5  (q=4, paper Table I)
//   QW       — CTV α message width (sign + q̃ mag bits)  7  (q̃=6, paper Table I)
//   DC_MAX   — maximum check-node degree               10  (BG2)
//   IDW      — ceil(log2(DC_MAX))                       4
//   N_LAYERS — total decoding layers = BG rows         42  (BG2)
//   ADRW     — ceil(log2(N_LAYERS))                     6
//
// ── File dependencies ────────────────────────────────────────────────────────
//
//   cnu_iams.v      — scalar IAMS CNU (instantiated inside cnu_iams_z)
//   cnu_iams_z.v    — Z-parallel CNU wrapper
//   ctv_mem.v       — compressed CTV memory + decompressor
//
// Synthesisable Verilog-2001. No include files required.
// =============================================================================

`timescale 1ns/1ps

module cnu_ctv_top #(
    // ── Shared geometry ───────────────────────────────────────────
    parameter Z        = 52,   // QC lifting factor          (BG2)
    parameter QIN      =  5,   // β input width  (sign+4mag) (BG2 Table I)
    parameter QW       =  7,   // α output width (sign+6mag) (BG2 Table I)
    parameter DC_MAX   = 10,   // max check-node degree      (BG2)
    parameter IDW      =  4,   // ceil(log2(DC_MAX))
    // ── CTV memory geometry ───────────────────────────────────────
    parameter N_LAYERS = 42,   // total decoding layers = BG rows (BG2)
    parameter ADRW     =  6    // ceil(log2(N_LAYERS))
) (
    input  wire  clk,
    input  wire  rst_n,

    // ── CNU input (from VNU / Read Network) ───────────────────────────────────
    // β messages: packed beta_in[ slot*Z*QIN + z*QIN +: QIN ]
    //   slot = VN-column index 0..DC_MAX-1,  z = lifting position 0..Z-1
    input  wire  [DC_MAX*Z*QIN-1:0]    beta_in,
    input  wire                        cnu_valid_in,   // pulse high for 1 cycle

    // ── CNU output (to VNU for immediate APP update) ──────────────────────────
    // α messages from THIS iteration, same packing as beta_in but QW wide.
    // Valid when cnu_valid_out is high.
    output wire  [DC_MAX*Z*QW-1:0]     cnu_alpha_out,
    output wire                        cnu_valid_out,  // 1 cycle after cnu_valid_in

    // ── CTV memory WRITE control (from layer scheduler) ───────────────────────
    // Simplest usage: tie wr_en = cnu_valid_out, layer_wr = current layer index.
    // The scheduler may also gate wr_en independently if needed.
    input  wire                        wr_en,
    input  wire  [ADRW-1:0]           layer_wr,       // layer index to write

    // ── CTV memory READ control (from layer scheduler) ────────────────────────
    // Assert rd_en and place layer index on layer_rd.
    // ctv_alpha_out is valid 2 cycles later (ctv_valid_out goes high).
    input  wire                        rd_en,
    input  wire  [ADRW-1:0]           layer_rd,       // layer index to read

    // ── CTV memory output (to VNU: "old alpha" for β = γ̃ − α subtraction) ───
    // Decompressed α messages from a previous iteration.
    // Packing: ctv_alpha_out[ slot*Z*QW + z*QW +: QW ]
    output wire  [DC_MAX*Z*QW-1:0]     ctv_alpha_out,
    output wire                        ctv_valid_out   // high 2 cycles after rd_en
);

    // =========================================================================
    // Internal wires — compressed fields from CNU to CTV memory
    // Width = Z * (QIN-1) for min1/min2,  Z*IDW for idx1/idx2,
    //         Z*DC_MAX for sign bits.
    // These are registered inside cnu_iams_z (1-cycle latency from beta_in).
    // =========================================================================
    wire [Z*(QIN-1)-1:0]  w_min1;   // smallest magnitude per z
    wire [Z*(QIN-1)-1:0]  w_min2;   // second-smallest magnitude per z
    wire [Z*IDW-1:0]      w_idx1;   // index of min1 per z
    wire [Z*IDW-1:0]      w_idx2;   // index of min2 per z
    wire [Z*DC_MAX-1:0]   w_sgn;    // all DC_MAX input signs per z

    // =========================================================================
    // CNU — Z-parallel IAMS check-node unit
    // =========================================================================
    cnu_iams_z #(
        .Z      (Z),
        .QIN    (QIN),
        .QW     (QW),
        .DC_MAX (DC_MAX),
        .IDW    (IDW)
    ) u_cnu_z (
        .clk          (clk),
        .rst_n        (rst_n),
        .valid_in     (cnu_valid_in),

        // β messages in (from VNU)
        .beta_in      (beta_in),

        // α messages out (to VNU for APP update this iteration)
        .alpha_out    (cnu_alpha_out),
        .valid_out    (cnu_valid_out),

        // Compressed fields out (to CTV memory for storage)
        .min1_z_out   (w_min1),
        .min2_z_out   (w_min2),
        .idx1_z_out   (w_idx1),
        .idx2_z_out   (w_idx2),
        .sgn_z_out    (w_sgn)
    );

    // =========================================================================
    // CTV memory — stores compressed CNU state, decompresses on read
    // =========================================================================
    ctv_mem #(
        .Z        (Z),
        .QW       (QW),
        .QIN      (QIN),
        .DC_MAX   (DC_MAX),
        .IDW      (IDW),
        .N_LAYERS (N_LAYERS),
        .ADRW     (ADRW)
    ) u_ctv_mem (
        .clk       (clk),
        .rst_n     (rst_n),

        // Read port (from scheduler)
        .rd_en     (rd_en),
        .layer_rd  (layer_rd),

        // Write port (from scheduler, data from CNU)
        .wr_en     (wr_en),
        .layer_wr  (layer_wr),

        // Compressed data in — directly from cnu_iams_z outputs
        .min1_in   (w_min1),
        .min2_in   (w_min2),
        .idx1_in   (w_idx1),
        .idx2_in   (w_idx2),
        .sgn_in    (w_sgn),

        // Decompressed α messages out (to VNU for β = γ̃ − α)
        .alpha_out (ctv_alpha_out),
        .valid_out (ctv_valid_out)
    );

endmodule

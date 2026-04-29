// =============================================================================
// ctv_mem.v  —  CTV Memory for the IAMS 5G-LDPC Decoder
// =============================================================================
// Cui et al., IEEE TCAS-I 2020, Sections V-A and V-B, Fig. 12, Fig. 14.
//
// ─── WHAT THIS MODULE DOES ──────────────────────────────────────────────────
//
// Stores compressed CTV messages between CNU and VNU in the layered decoder.
//
// COMPRESSED STORAGE FORMAT (Fig. 12)
// Eq.(10) reconstructs all DC_MAX output messages from five scalars per CN:
//     sgn[0..DC_MAX-1]  — one sign bit per VTC input edge   (DC_MAX bits)
//     min1              — smallest  VTC magnitude            (MAG_W bits)
//     min2              — 2nd-smallest VTC magnitude         (MAG_W bits)
//     idx1              — edge index of min1                 (IDW bits)
//     idx2              — edge index of min2                 (IDW bits)
// Compressed word per z: CWORD = DC_MAX + 2*MAG_W + 2*IDW bits.
//
// SPLIT STORAGE (Fig. 14)
// Many layers have actual degree DC_N < DC_MAX. Rather than storing every
// layer at full DC_MAX width, the memory is split:
//
//   mem1 — depth N_LAYERS,       width WIDE_W = Z * CWORD_WIDE
//           Holds the FULL compressed word for EVERY layer.
//           For narrow layers, sign bits DC_N..DC_MAX-1 are stored as 0.
//           (They do not exist, so 0 is correct — they don't contribute to
//            parity or output for any VN in the layer.)
//
//   mem2 — NOT USED IN THIS IMPLEMENTATION.
//
// WHY ONE MEMORY (not two)?
// The split storage in Fig. 14 requires different z-strides in mem1 and
// mem2 to be reassembled correctly. If mem1 uses CWORD_NARROW stride but
// the decompressor reads at CWORD_WIDE stride, the field offsets for z>0
// are WRONG, producing silently incorrect data. The only correct way to
// have two memories is to store them at CWORD_WIDE stride in mem1 (which
// means mem1 is WIDE_W wide) and use mem2 only for the additional sign bits
// of wide layers — but then mem1 is already full-width and the saving is
// smaller than the added control complexity. For correctness, this
// implementation uses a single WIDE_W-wide memory. The area saving from
// split storage can be reintroduced at the level of the memory compiler
// (e.g. two SRAM macros of different widths) without changing the RTL.
//
// INPUTS FROM cnu_iams_z (NO COMPRESSOR NEEDED)
// cnu_iams_z exposes {min1, min2, idx1, idx2, sgn} directly as registered
// output ports. These are fed straight into ctv_mem, bypassing the
// compressor entirely and eliminating the index-recovery ambiguity.
//
// Input bus packing (matches cnu_iams_z output buses):
//   min1_in [ z*(QIN-1) +: (QIN-1) ]   one min1 per z-position
//   min2_in [ z*(QIN-1) +: (QIN-1) ]
//   idx1_in [ z*IDW +: IDW ]
//   idx2_in [ z*IDW +: IDW ]
//   sgn_in  [ z*DC_MAX +: DC_MAX ]
//
// Note: min1/min2 arrive at QIN-1 = MAG_IN bits. The memory stores them
// at MAG_W = QW-1 bits (zero-extended, since MAG_IN <= MAG_W for QW>=QIN).
// The decompressor reconstructs at MAG_W precision, which is correct.
//
// Output packing (feeds VNU):
//   alpha_out [ slot*Z*QW + z*QW +: QW ]   same as cnu_iams_z.alpha_out
//
// READ LATENCY: 2 clock cycles after rd_en.
// WRITE: captured on posedge clk when wr_en=1 (same cycle as CNU valid_out).
//
// Synthesisable Verilog-2001.
// =============================================================================

`timescale 1ns/1ps

module ctv_mem #(
    parameter Z        = 52,   // QC lifting factor
    parameter QW       = 7,    // CTV output width (sign + MAG_W mag bits)
    parameter QIN      = 5,    // VTC input width  (sign + MAG_IN mag bits)
                               // — needed to size min1/min2 input buses
    parameter DC_MAX   = 10,   // maximum check-node degree
    parameter IDW      = 4,    // ceil(log2(DC_MAX))
    parameter N_LAYERS = 42,   // total decoding layers
    parameter ADRW     = 6     // address width = ceil(log2(N_LAYERS))
)(
    input  wire                         clk,
    input  wire                         rst_n,

    // ── Read control ──────────────────────────────────────────────────────────
    input  wire                         rd_en,
    input  wire [ADRW-1:0]             layer_rd,

    // ── Write control ─────────────────────────────────────────────────────────
    input  wire                         wr_en,
    input  wire [ADRW-1:0]             layer_wr,

    // ── Write data — compressed fields direct from cnu_iams_z ────────────────
    // Packing: field[ z*WIDTH +: WIDTH ],  z = 0..Z-1
    input  wire [Z*(QIN-1)-1:0]         min1_in,   // MAG_IN = QIN-1 bits per z
    input  wire [Z*(QIN-1)-1:0]         min2_in,
    input  wire [Z*IDW-1:0]             idx1_in,
    input  wire [Z*IDW-1:0]             idx2_in,
    input  wire [Z*DC_MAX-1:0]          sgn_in,    // DC_MAX sign bits per z

    // ── Read data — decompressed CTV messages to VNU ──────────────────────────
    // Packing: alpha_out[ slot*Z*QW + z*QW +: QW ],  same as cnu_iams_z
    output reg  [DC_MAX*Z*QW-1:0]       alpha_out,
    output reg                          valid_out
);

    // =========================================================================
    // Derived parameters
    // =========================================================================
    localparam MAG_W   = QW  - 1;   // output magnitude width (q-1 in paper)
    localparam MAG_IN  = QIN - 1;   // input  magnitude width

    // Compressed word per z-position (always at WIDE format for correct stride)
    localparam CWORD = DC_MAX + 2*MAG_W + 2*IDW;   // bits per z

    // Full memory word width
    localparam MEM_W = Z * CWORD;

    // =========================================================================
    // Memory (single bank, WIDE_W per word, all layers)
    // =========================================================================
    reg [MEM_W-1:0] mem [0:N_LAYERS-1];

    // =========================================================================
    // WRITE PATH — pack compressed word from cnu_iams_z fields and store
    //
    // Layout per z (bits inside one CWORD block at offset z*CWORD):
    //   [DC_MAX-1:0]                          sgn[DC_MAX-1:0]
    //   [DC_MAX + MAG_W - 1 : DC_MAX]         min1  (zero-extended from MAG_IN)
    //   [DC_MAX + 2*MAG_W - 1 : DC_MAX+MAG_W] min2
    //   [DC_MAX + 2*MAG_W + IDW - 1 :
    //    DC_MAX + 2*MAG_W]                     idx1
    //   [DC_MAX + 2*MAG_W + 2*IDW - 1 :
    //    DC_MAX + 2*MAG_W + IDW]               idx2
    //
    // min1/min2 are MAG_IN bits wide from cnu_iams_z; they are stored in
    // the lower MAG_IN bits of the MAG_W-wide field (zero-extended).
    // This is correct because MAG_IN <= MAG_W (QIN <= QW for paper params).
    // =========================================================================
    reg [MEM_W-1:0] pack_word;   // combinational packed word
    integer         pz;

    always @(*) begin : pack_compressed
        pack_word = {MEM_W{1'b0}};
        for (pz = 0; pz < Z; pz = pz + 1) begin
            // Sign bits (DC_MAX bits, directly from cnu_iams_z sgn_out)
            pack_word[pz*CWORD +:              DC_MAX] =
                sgn_in[pz*DC_MAX +: DC_MAX];

            // min1 (MAG_IN bits zero-extended to MAG_W)
            pack_word[pz*CWORD + DC_MAX +:     MAG_W] =
                { {(MAG_W-MAG_IN){1'b0}}, min1_in[pz*MAG_IN +: MAG_IN] };

            // min2
            pack_word[pz*CWORD + DC_MAX+MAG_W +: MAG_W] =
                { {(MAG_W-MAG_IN){1'b0}}, min2_in[pz*MAG_IN +: MAG_IN] };

            // idx1
            pack_word[pz*CWORD + DC_MAX+2*MAG_W +: IDW] =
                idx1_in[pz*IDW +: IDW];

            // idx2
            pack_word[pz*CWORD + DC_MAX+2*MAG_W+IDW +: IDW] =
                idx2_in[pz*IDW +: IDW];
        end
    end

    // Synchronous write
    integer wi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (wi = 0; wi < N_LAYERS; wi = wi + 1)
                mem[wi] <= {MEM_W{1'b0}};
        end else if (wr_en) begin
            mem[layer_wr] <= pack_word;
        end
    end

    // =========================================================================
    // READ PATH — Stage 1: synchronous memory read (1 cycle latency)
    // =========================================================================
    reg [MEM_W-1:0] rd_mem_q;
    reg             rd_valid_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_mem_q   <= {MEM_W{1'b0}};
            rd_valid_q <= 1'b0;
        end else begin
            rd_valid_q <= rd_en;
            if (rd_en)
                rd_mem_q <= mem[layer_rd];
        end
    end

    // =========================================================================
    // READ PATH — Stage 2: DECOMPRESSOR (combinational, Eq. 10)
    //
    // For each z: extract {sgn, min1, min2, idx1, idx2} from rd_mem_q,
    // then reconstruct DC_MAX signed output messages.
    //
    // sign_total = XOR of all DC_MAX input sign bits
    // for each edge ds = 0..DC_MAX-1:
    //   out_sign = sign_total XOR sgn[ds]
    //   if ds==idx1 : alpha_mag = min2
    //   if ds==idx2 : alpha_mag = min1
    //   else if delta==0 : alpha_mag = max(min1-1, 0)
    //   else             : alpha_mag = min1
    // =========================================================================
    reg [DC_MAX-1:0]      d_sgn;
    reg [MAG_W-1:0]       d_min1, d_min2, d_delta, d_mag;
    reg [IDW-1:0]         d_idx1, d_idx2;
    reg                   d_sign_total, d_out_sign;
    reg [QW-1:0]          d_out_word;
    reg [DC_MAX*Z*QW-1:0] decomp_out;

    integer dz, ds;
    always @(*) begin : decompressor
        decomp_out = {DC_MAX*Z*QW{1'b0}};

        for (dz = 0; dz < Z; dz = dz + 1) begin

            // Extract compressed fields for this z from the memory word
            d_sgn  = rd_mem_q[dz*CWORD +:              DC_MAX];
            d_min1 = rd_mem_q[dz*CWORD + DC_MAX +:     MAG_W ];
            d_min2 = rd_mem_q[dz*CWORD + DC_MAX+MAG_W+:MAG_W ];
            d_idx1 = rd_mem_q[dz*CWORD + DC_MAX+2*MAG_W +: IDW];
            d_idx2 = rd_mem_q[dz*CWORD + DC_MAX+2*MAG_W+IDW +: IDW];

            // XOR parity of all input signs
            d_sign_total = 1'b0;
            for (ds = 0; ds < DC_MAX; ds = ds + 1)
                d_sign_total = d_sign_total ^ d_sgn[ds];

            // Delta
            d_delta = d_min2 - d_min1;

            // Reconstruct each output message (Eq. 10)
            for (ds = 0; ds < DC_MAX; ds = ds + 1) begin

                d_out_sign = d_sign_total ^ d_sgn[ds];

                // Magnitude selection
                if (ds[IDW-1:0] == d_idx1)
                    d_mag = d_min2;
                else if (ds[IDW-1:0] == d_idx2)
                    d_mag = d_min1;
                else if (d_delta == {MAG_W{1'b0}})
                    // max(min1 - 1, 0)
                    d_mag = (d_min1 > {{(MAG_W-1){1'b0}}, 1'b1})
                            ? (d_min1 - 1'b1) : {MAG_W{1'b0}};
                else
                    d_mag = d_min1;

                // Signed 2's complement encoding
                if (d_mag == {MAG_W{1'b0}})
                    d_out_word = {QW{1'b0}};
                else if (d_out_sign)
                    d_out_word = ~({1'b0, d_mag}) + {{(QW-1){1'b0}}, 1'b1};
                else
                    d_out_word = {1'b0, d_mag};

                // Pack: alpha_out[ slot*Z*QW + z*QW +: QW ]
                decomp_out[ds*Z*QW + dz*QW +: QW] = d_out_word;
            end
        end
    end

    // =========================================================================
    // READ PATH — Stage 3: output register (2nd cycle of read latency)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            alpha_out <= {DC_MAX*Z*QW{1'b0}};
            valid_out <= 1'b0;
        end else begin
            valid_out <= rd_valid_q;
            if (rd_valid_q)
                alpha_out <= decomp_out;
        end
    end

endmodule

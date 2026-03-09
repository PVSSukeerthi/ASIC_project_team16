// =============================================================================
// decompressor.v  —  CTV Message Decompressor (Fig. 12)
// =============================================================================
// Cui et al. TCAS-I 2020, Fig. 12 and Section V-B.
//
// The CTV (Check-To-Variable) memory stores messages in compressed form.
// One entry covers an entire Z-column circulant block:
//
//   Compressed word = { sign_bits[Z-1:0],        — one sign per lifted edge
//                       min1[QW-2:0],             — smallest magnitude (6 bits)
//                       min2[QW-2:0],             — second smallest magnitude
//                       idx1[IDW-1:0],            — position of min1 (4 bits)
//                       delta_flag }              — (min2-min1)==0 ?
//
// Decompression reconstructs Z messages using the IAMS four-case rule (eq.10):
//   edge e == idx1             : alpha_mag = min2
//   edge e is at idx2*         : alpha_mag = min1      (* implicitly: all edges
//                                                         sharing min1, minus idx1)
//   edge e in Ī(m), delta==0   : alpha_mag = max(min1-1, 0)
//   edge e in Ī(m), delta!=0   : alpha_mag = min1
//   Sign: sign_bits[e]
//
// Note: idx2 is NOT stored (saves bits). For each edge that is NOT idx1, the
// decompressor applies the combined rule: if min1 was achieved by exactly one
// edge (idx1) then all other edges use min2 for the minimum rule. The
// delta_flag replaces explicit idx2 storage.
//
// Ports (combinational — no clock)
//   ctv_word  [CTV_WORD_W-1:0]  — compressed word from CTV memory
//   msg_out   [Z*QW-1:0]        — Z decompressed messages (signed, QW bits each)
// =============================================================================

`timescale 1ns/1ps
`include "pkg_iams.vh"

module decompressor #(
    parameter Z          = `LIFTING_Z,
    parameter QW         = `QW,        // CN→VN message width (7 bits)
    parameter IDW        = `IDW,       // 4 bits
    parameter CTV_WORD_W = `CTV_WORD_W // 69 bits for Z=52
) (
    input  wire [CTV_WORD_W-1:0] ctv_word,
    output wire [Z*QW-1:0]       msg_out
);

    // -------------------------------------------------------------------------
    // Unpack compressed word fields
    // Bit layout (LSB first):
    //   [0]           : delta_flag
    //   [IDW:1]       : idx1  (4 bits)
    //   [IDW+QW-2:IDW+1] : min2  (6 bits)
    //   [IDW+2*(QW-1):IDW+(QW-1)+1] : min1 (6 bits)
    //   [IDW+2*(QW-1)+Z : IDW+2*(QW-1)+1] : sign_bits (Z bits)
    // -------------------------------------------------------------------------
    localparam MAG_W = QW - 1;   // 6 bits

    wire                delta_flag;
    wire [IDW-1:0]      idx1;
    wire [MAG_W-1:0]    min2_stored;
    wire [MAG_W-1:0]    min1_stored;
    wire [Z-1:0]        sign_bits;

    // Bit layout (LSB=0):
    //   [0]         : delta_flag
    //   [IDW:1]     : idx1        (bits 4:1  for IDW=4)
    //   [IDW+MAG_W : IDW+1]     : min2  (bits 10:5  for IDW=4,MAG_W=6)
    //   [IDW+2*MAG_W : IDW+MAG_W+1] : min1 (bits 16:11)
    //   [IDW+2*MAG_W+Z : IDW+2*MAG_W+1] : sign_bits (bits 68:17 for Z=52)
    localparam F1 = 1;               // idx1 LSB
    localparam F2 = 1 + IDW;         // min2 LSB  = 5
    localparam F3 = 1 + IDW + MAG_W; // min1 LSB  = 11
    localparam F4 = 1 + IDW + MAG_W*2; // sign_bits LSB = 17

    assign delta_flag   = ctv_word[0];
    assign idx1         = ctv_word[F1 +: IDW];
    assign min2_stored  = ctv_word[F2 +: MAG_W];
    assign min1_stored  = ctv_word[F3 +: MAG_W];
    assign sign_bits    = ctv_word[F4 +: Z];

    // -------------------------------------------------------------------------
    // Reconstruct each of the Z messages
    // -------------------------------------------------------------------------
    wire [MAG_W-1:0] alpha_mag_arr [0:Z-1];
    wire             out_sign_arr  [0:Z-1];
    wire [QW-1:0]    msg_arr       [0:Z-1];

    genvar e;
    generate
        for (e = 0; e < Z; e = e + 1) begin : decomp_edge

            // Magnitude selection — IAMS eq.(10)
            assign alpha_mag_arr[e] =
                (e[IDW-1:0] == idx1)
                    ? min2_stored                         // edge is idx1 → use min2
                    : (delta_flag == 1'b0)
                        ? ((min1_stored > {{(MAG_W-1){1'b0}},1'b1})
                                ? (min1_stored - 1'b1)
                                : {MAG_W{1'b0}})          // Δ=0, Ī(m): max(min1-1,0)
                        : min1_stored;                    // Δ≠0, Ī(m): min1

            // Sign: stored directly per edge
            assign out_sign_arr[e] = sign_bits[e];

            // Re-encode as signed 2's complement
            assign msg_arr[e] =
                (alpha_mag_arr[e] == {MAG_W{1'b0}})
                    ? {QW{1'b0}}
                    : (out_sign_arr[e]
                        ? {1'b1, (~alpha_mag_arr[e] + 1'b1)}
                        : {1'b0,   alpha_mag_arr[e]});

            // Pack into flat output
            assign msg_out[e*QW +: QW] = msg_arr[e];

        end
    endgenerate

endmodule
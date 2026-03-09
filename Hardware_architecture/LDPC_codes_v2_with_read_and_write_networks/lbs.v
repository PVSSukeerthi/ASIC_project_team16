// =============================================================================
// lbs.v  —  Left Barrel Shifter (Cyclic Left Rotation)
// =============================================================================
// Fig. 11 of Cui et al. TCAS-I 2020.
//
// Each non-zero entry H[i][j] has a shift value s_ij.
// The LBS takes a Z-wide vector of Q-bit symbols and rotates it left by S
// positions (cyclic).  Used in both the Read Network (LBS_r) to align
// incoming channel LLRs, and the Write Network (LBS_w) to rotate APP
// contributions back to the canonical ordering.
//
// Parameters
//   Z   — lifting factor (number of circulant columns/rows), default 52
//   QW  — bit-width of each symbol, default 8 (QTW)
//
// Ports
//   data_in  [Z*QW-1:0]  — Z symbols packed, symbol 0 at LSB
//   shift    [SW-1:0]    — cyclic left shift amount (0 .. Z-1)
//   data_out [Z*QW-1:0]  — rotated output (combinational)
//
// Timing: purely combinational (register outside if needed)
// =============================================================================

`timescale 1ns/1ps
`include "pkg_iams.vh"

module lbs #(
    parameter Z  = `LIFTING_Z,   // 52
    parameter QW = `QTW,         // symbol width in bits
    parameter SW = 6             // shift width = ceil(log2(Z)) = ceil(log2(52))=6
) (
    input  wire [Z*QW-1:0] data_in,
    input  wire [SW-1:0]   shift,
    output wire [Z*QW-1:0] data_out
);

    // -------------------------------------------------------------------------
    // Unpack into 2-D array
    // -------------------------------------------------------------------------
    wire [QW-1:0] sym_in  [0:Z-1];
    wire [QW-1:0] sym_out [0:Z-1];

    genvar gi;
    generate
        for (gi = 0; gi < Z; gi = gi + 1) begin : unpack
            assign sym_in[gi] = data_in[gi*QW +: QW];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Cyclic left rotation: sym_out[i] = sym_in[(i + shift) % Z]
    // Implemented as a combinational mux tree.
    // We use a log2(Z)-stage barrel shifter for efficiency.
    // -------------------------------------------------------------------------
    // Stage-0 intermediate (shift by 1 if shift[0]=1)
    // Stage-1 intermediate (shift by 2 if shift[1]=1)
    // ...
    // For Z=52 we need 6 stages (2^6=64 > 52).
    // At each stage we reduce modulo Z.

    // Helper: modular index function (synthesisable parametric mux)
    // sym_out[i] = sym_in[(i + shift) mod Z]
    // We implement this as a direct mod-Z indexed mux array.

    genvar go;
    generate
        for (go = 0; go < Z; go = go + 1) begin : out_mux
            // Compute source index modulo Z for each output position.
            // shift is at most Z-1 (enforced by caller).
            // idx = (go + shift) mod Z  — use a comparator-adder.
            wire [SW:0] raw_idx;
            assign raw_idx = go + shift;          // go < Z, shift < Z → max 2Z-2
            assign sym_out[go] = (raw_idx >= Z)
                                 ? sym_in[raw_idx - Z]
                                 : sym_in[raw_idx];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Re-pack output
    // -------------------------------------------------------------------------
    generate
        for (gi = 0; gi < Z; gi = gi + 1) begin : repack
            assign data_out[gi*QW +: QW] = sym_out[gi];
        end
    endgenerate

endmodule

// =============================================================================
// syndrome_check.v  —  Syndrome Checker
// =============================================================================
// Checks if the current hard decisions satisfy all parity equations.
// H * ĉ^T == 0 mod 2  ↔  XOR of all APP hard decisions connected to each CN = 0
//
// In a layered architecture the syndrome is evaluated at the end of each
// full iteration sweep (after all layers have been processed).
//
// For BG2 with Z=52: M=2184 CNs, each CN connects to d_c VNs.
// The syndrome register holds 1 bit per CN; the overall syndrome_ok signal
// is the NOR of all syndrome bits.
//
// This module computes a running XOR parity for each CN from the
// hard-decision bits streamed in over one clock cycle per CN.
//
// Parameters:
//   M     — number of check nodes
//   N     — number of variable nodes
//   QTW   — APP bit-width (hard decision = sign bit)
// =============================================================================

`timescale 1ns / 1ps

module syndrome_check #(
    parameter M     = 2184,          // BG2: 42 * 52
    parameter N     = 2704,          // BG2: 52 * 52
    parameter MW    = $clog2(M),
    parameter NW    = $clog2(N)
) (
    input  wire          clk,
    input  wire          rst_n,
    // Streaming interface: one VN per cycle
    input  wire          hd_valid,   // hard-decision valid
    input  wire          hd_bit,     // hard decision for current VN
    input  wire [NW-1:0] vn_idx,     // which VN this bit belongs to
    // Row-by-row parity accumulation driven from outside
    // (the top-level feeds in the H row membership)
    input  wire [M-1:0]  row_mask,   // which CNs this VN connects to (1-hot or multi-hot)
    // Result
    output wire          syndrome_ok // 1 when all parities = 0
);

    // -------------------------------------------------------------------------
    // Parity register: one bit per CN
    // -------------------------------------------------------------------------
    reg [M-1:0] parity;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parity <= {M{1'b0}};
        end else if (hd_valid) begin
            // XOR hd_bit into all CNs that connect to this VN
            parity <= parity ^ ({M{hd_bit}} & row_mask);
        end
    end

    assign syndrome_ok = ~(|parity);

endmodule

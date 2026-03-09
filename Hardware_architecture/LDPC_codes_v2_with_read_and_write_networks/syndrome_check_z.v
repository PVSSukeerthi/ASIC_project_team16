// =============================================================================
// syndrome_check_z.v  —  Z-Parallel Syndrome Checker
// =============================================================================
// For lifted code with lifting factor Z=52:
//   Each CN row has Z parallel parity equations.
//   syndrome_ok = 1 iff ALL Z equations for ALL BG_ROWS CN rows are satisfied.
//
// After each iteration, the controller can snapshot hard decisions from APP
// memory and compute syndrome. This module does running parity: it receives
// one CN row of Z sign bits (hard decisions for Z parallel VNs) per cycle and
// XORs them into the parity accumulator.
//
// Interface
//   clk, rst_n
//   clear        — reset all parity accumulators (start of syndrome check)
//   row_valid    — one CN row's data is presented
//   row_data [Z*QTW-1:0] — APP LLR values for this CN's connected VNs
//                          (hard decision = sign bit)
//   degree [DEG_W-1:0]   — how many slots are active
//   row_done             — last row has been presented
//   syndrome_ok          — combinational: all parities = 0
// =============================================================================

`timescale 1ns/1ps
`include "pkg_iams.vh"

module syndrome_check_z #(
    parameter Z       = `LIFTING_Z,
    parameter QTW     = `QTW,
    parameter DC_MAX  = `DC_MAX,
    parameter BG_ROWS = `BG_ROWS,
    parameter DEG_W   = 4
) (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   clear,
    input  wire                   row_valid,
    // Z APP LLR values for active VNs (packed from Read Network output)
    input  wire [DC_MAX*Z*QTW-1:0] row_data,
    input  wire [DEG_W-1:0]        degree,
    output wire                    syndrome_ok
);

    // -------------------------------------------------------------------------
    // Per-CN parity register: one bit per Z-position
    // -------------------------------------------------------------------------
    reg [Z-1:0] parity_acc [0:BG_ROWS-1];
    reg [5:0]   row_cnt;   // counts which CN row is being fed

    // Hard decision: sign bit of APP LLR
    // For each slot (0..degree-1), XOR the Z hard decisions into parity_acc[row]
    // For lifting: the parity for circulant z of row r is:
    //   XOR over connected VN slots of hard_decision(slot, z)

    integer pi, pz, ps;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear) begin
            for (pi = 0; pi < BG_ROWS; pi = pi + 1)
                parity_acc[pi] <= {Z{1'b0}};
            row_cnt <= 6'd0;
        end else if (row_valid) begin
            begin : parity_update
                reg [Z-1:0] row_parity;
                row_parity = {Z{1'b0}};
                for (ps = 0; ps < DC_MAX; ps = ps + 1) begin
                    if (ps < degree) begin
                        for (pz = 0; pz < Z; pz = pz + 1) begin
                            // Hard decision = MSB (sign) of APP LLR
                            row_parity[pz] = row_parity[pz] ^
                                row_data[(ps*Z + pz)*QTW + QTW - 1];
                        end
                    end
                end
                parity_acc[row_cnt] <= row_parity;
            end
            row_cnt <= (row_cnt == BG_ROWS-1) ? 6'd0 : row_cnt + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // syndrome_ok = AND reduction of all parity == 0
    // -------------------------------------------------------------------------
    wire [BG_ROWS-1:0] all_zero;
    genvar gr;
    generate
        for (gr = 0; gr < BG_ROWS; gr = gr + 1) begin : syn_check
            assign all_zero[gr] = (parity_acc[gr] == {Z{1'b0}});
        end
    endgenerate
    assign syndrome_ok = &all_zero;

endmodule

// =============================================================================
// vnu_z.v  —  Z-Parallel Variable Node Unit
// =============================================================================
// Cui et al. TCAS-I 2020, Section IV-B (layered decoding VNU update).
//
// For layered decoding, the VNU update per edge e is:
//   β_e  = APP_e − α_old_e               (compute input to CNU)
//   APP_e_new = APP_e + α_new_e − α_old_e (update APP after CNU)
//
// This module processes Z variable nodes (one full circulant) simultaneously.
//
// Pipeline stages:
//   Stage 0 (combinational): compute β = APP - α_old
//   Stage 1 (registered):    compute APP_new = APP + α_new - α_old  (after CNU)
//
// Fixed-point:
//   APP / β: QTW = 8 bits (signed)
//   α_old / α_new: QW = 7 bits (signed) — sign-extended to QTW for arithmetic
//   Saturate APP_new to [-2^(QTW-1)+1 .. +2^(QTW-1)-1]
//
// Ports
//   --- β computation (Stage 0) ---
//   app_in    [Z*QTW-1:0]   — current APP values
//   alpha_old [Z*QW-1:0]    — stored α (before this layer)
//   beta_out  [Z*QTW-1:0]   — β = APP - α_old (combinational)
//   --- APP update (Stage 1, clocked) ---
//   clk, rst_n
//   upd_en                  — enable APP update
//   alpha_new [Z*QW-1:0]    — new α from CNU
//   app_upd_out [Z*QTW-1:0] — updated APP values
//   app_upd_valid
// =============================================================================

`timescale 1ns/1ps
`include "pkg_iams.vh"

module vnu_z #(
    parameter Z    = `LIFTING_Z,
    parameter QTW  = `QTW,   // 8 bits
    parameter QW   = `QW     // 7 bits
) (
    input  wire                clk,
    input  wire                rst_n,

    // Beta stage (combinational)
    input  wire [Z*QTW-1:0]   app_in,
    input  wire [Z*QW-1:0]    alpha_old,
    output wire [Z*QTW-1:0]   beta_out,

    // APP update stage (clocked)
    input  wire                upd_en,
    input  wire [Z*QW-1:0]    alpha_new,
    output reg  [Z*QTW-1:0]   app_upd_out,
    output reg                 app_upd_valid
);

    // -------------------------------------------------------------------------
    // Saturation limits
    // -------------------------------------------------------------------------
    localparam signed [QTW-1:0] SAT_MAX =  {1'b0, {(QTW-1){1'b1}}};  //  127
    localparam signed [QTW-1:0] SAT_MIN =  {1'b1, {(QTW-1){1'b0}}};  // -128

    // -------------------------------------------------------------------------
    // Stage 0: β = APP - α_old  (combinational, saturated)
    // -------------------------------------------------------------------------
    genvar gz;
    generate
        for (gz = 0; gz < Z; gz = gz + 1) begin : beta_stage
            wire signed [QTW-1:0] app_sym;
            wire signed [QTW-1:0] aold_ext;  // sign-extend α_old from QW to QTW
            wire signed [QTW:0]   diff;       // 1 extra bit for overflow detection

            assign app_sym  = app_in  [gz*QTW +: QTW];
            assign aold_ext = {{(QTW-QW){alpha_old[gz*QW + QW - 1]}},
                                alpha_old[gz*QW +: QW]};
            assign diff     = {app_sym[QTW-1], app_sym} - {aold_ext[QTW-1], aold_ext};

            // Saturate to QTW signed range [-128 .. +127]
            assign beta_out[gz*QTW +: QTW] =
                ($signed(diff) >  $signed(9'sd127)) ? SAT_MAX :
                ($signed(diff) < -$signed(9'sd128)) ? SAT_MIN :
                diff[QTW-1:0];
        end
    endgenerate
    reg signed [QTW-1:0]  app_s;
    reg signed [QTW-1:0]  an_ext, ao_ext;
    reg signed [QTW+1:0]  sum;
                    
    // -------------------------------------------------------------------------
    // Stage 1: APP_new = APP + α_new - α_old  (clocked, saturated)
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            app_upd_out   <= {Z*QTW{1'b0}};
            app_upd_valid <= 1'b0;
        end else begin
            app_upd_valid <= upd_en;
            if (upd_en) begin : app_upd
                integer uv;
                for (uv = 0; uv < Z; uv = uv + 1) begin
                    app_s  = app_in  [uv*QTW +: QTW];
                    an_ext = {{(QTW-QW){alpha_new[uv*QW + QW - 1]}},
                               alpha_new[uv*QW +: QW]};
                    ao_ext = {{(QTW-QW){alpha_old[uv*QW + QW - 1]}},
                               alpha_old[uv*QW +: QW]};
                    sum    = {app_s[QTW-1], app_s[QTW-1], app_s}
                           + {an_ext[QTW-1], an_ext[QTW-1], an_ext}
                           - {ao_ext[QTW-1], ao_ext[QTW-1], ao_ext};
                    // Saturate using $signed comparisons
                    if ($signed(sum) >  $signed(10'sd127))
                        app_upd_out[uv*QTW +: QTW] <= SAT_MAX;
                    else if ($signed(sum) < -$signed(10'sd128))
                        app_upd_out[uv*QTW +: QTW] <= SAT_MIN;
                    else
                        app_upd_out[uv*QTW +: QTW] <= sum[QTW-1:0];
                end
            end
        end
    end

endmodule
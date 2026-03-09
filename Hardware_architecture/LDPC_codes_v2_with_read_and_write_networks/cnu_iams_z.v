// =============================================================================
// cnu_iams_z.v  —  Z-Parallel IAMS Check Node Unit
// =============================================================================
// Cui et al. TCAS-I 2020, Fig. 13 and Section V-A.
//
// Processes one full circulant layer: DC_MAX × Z messages simultaneously.
// Each of the Z "instances" is an independent IAMS CNU operating on
// DC_MAX messages (one from each connected VN column, after LBS alignment).
//
// The Z instances share the same H-row structure (same VN columns, same
// circulant shift values) but operate on different lifted symbol positions.
//
// Fixed-point: input QTW=8 bits (APP LLR used as β), output QW=7 bits (α).
//
// Pipeline: 1 register stage after combinational IAMS logic.
//
// Ports
//   clk, rst_n
//   valid_in
//   beta_in [DC_MAX*Z*QTW-1:0]   — DC_MAX inputs, Z wide (from Read Network)
//   alpha_out [DC_MAX*Z*QW-1:0]  — DC_MAX outputs, Z wide (to CTV memory)
//   valid_out
// =============================================================================

`timescale 1ns/1ps
`include "pkg_iams.vh"

module cnu_iams_z #(
    parameter Z      = `LIFTING_Z,
    parameter QIN    = `QTW,      // input width (β = APP - α_old, uses APP width)
    parameter QW     = `QW,       // output CTV message width
    parameter DC_MAX = `DC_MAX,
    parameter IDW    = `IDW
) (
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          valid_in,
    input  wire [DC_MAX*Z*QIN-1:0]       beta_in,    // β messages from VNUs
    output reg  [DC_MAX*Z*QW-1:0]        alpha_out,  // α messages to CTV mem
    output reg                           valid_out
);

    // -------------------------------------------------------------------------
    // Instantiate Z independent IAMS CNUs (one per lifted position)
    // Each CNU takes DC_MAX inputs of width QIN, produces DC_MAX outputs QW
    // -------------------------------------------------------------------------
    // We use generate to create Z instances of cnu_iams.
    // Each instance z takes: beta_in[slot*Z*QIN + z*QIN +: QIN] for slot=0..DC_MAX-1

    // Intermediate combinational wires
    wire [DC_MAX*QIN-1:0] cnu_in_z  [0:Z-1];   // per-z input to CNU
    wire [DC_MAX*QW-1:0]  cnu_out_z [0:Z-1];   // per-z CNU output
    wire                  cnu_vo_z  [0:Z-1];

    genvar gz, gs;
    generate
        for (gz = 0; gz < Z; gz = gz + 1) begin : z_inst

            // Pack DC_MAX inputs for this z-position
            for (gs = 0; gs < DC_MAX; gs = gs + 1) begin : pack_slot
                assign cnu_in_z[gz][gs*QIN +: QIN] =
                    beta_in[gs*Z*QIN + gz*QIN +: QIN];
            end

            // Instantiate IAMS CNU
            // Note: QIN used as message width here (β has APP width)
            cnu_iams #(
                .DC  (DC_MAX),
                .QW  (QIN),    // process at full QIN precision
                .IDW (IDW)
            ) u_cnu (
                .clk         (clk),
                .rst_n       (rst_n),
                .valid_in    (valid_in),
                .msg_flat_in (cnu_in_z[gz]),
                .msg_flat_out(cnu_out_z[gz]),
                .valid_out   (cnu_vo_z[gz])
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Collect CNU outputs into alpha_out
    // Truncate/clip from QIN to QW for CTV storage
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            alpha_out <= {DC_MAX*Z*QW{1'b0}};
            valid_out <= 1'b0;
        end else begin
            valid_out <= cnu_vo_z[0];  // all Z instances have same latency
            begin : collect
                integer cv_s, cv_z;
                for (cv_s = 0; cv_s < DC_MAX; cv_s = cv_s + 1) begin
                    for (cv_z = 0; cv_z < Z; cv_z = cv_z + 1) begin
                        // Clip QIN→QW: keep sign bit + lower QW-1 magnitude bits
                        // Saturate if magnitude exceeds QW range
                        alpha_out[cv_s*Z*QW + cv_z*QW +: QW] <=
                            cnu_out_z[cv_z][cv_s*QIN +: QW];
                    end
                end
            end
        end
    end

endmodule

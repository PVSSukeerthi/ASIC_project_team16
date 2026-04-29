// =============================================================================
// cnu_iams_z.v  —  Z-Parallel IAMS Check Node Unit
// =============================================================================
// Cui et al., IEEE TCAS-I 2020, Section V-A, Fig. 13.
//
// Instantiates Z independent cnu_iams cores (one per QC lifting position).
// Handles the QIN → QW width expansion on the data path (sign-extension
// for QW>QIN, pass-through for QW==QIN, saturation for QW<QIN).
//
// Bus packing:
//   beta_in  [ slot*Z*QIN + z*QIN +: QIN ]   slot=0..DC_MAX-1, z=0..Z-1
//   alpha_out[ slot*Z*QW  + z*QW  +: QW  ]   same slot/z convention
//
// Compressed-field output buses (for direct connection to ctv_mem):
//   min1_z_out [ z*(QIN-1) +: (QIN-1) ]   min1 per z (MAG_IN bits)
//   min2_z_out [ z*(QIN-1) +: (QIN-1) ]   min2 per z
//   idx1_z_out [ z*IDW     +: IDW     ]   idx1 per z
//   idx2_z_out [ z*IDW     +: IDW     ]   idx2 per z
//   sgn_z_out  [ z*DC_MAX  +: DC_MAX  ]   DC_MAX sign bits per z
//
// All compressed fields are registered inside cnu_iams on the same edge as
// msg_flat_out, so they are always consistent with alpha_out and valid_out.
//
// Latency: 1 clock cycle for all outputs.
// Synthesisable Verilog-2001.
// =============================================================================

`timescale 1ns/1ps

module cnu_iams_z #(
    parameter Z      = 52,
    parameter QIN    =  5,
    parameter QW     =  7,
    parameter DC_MAX = 10,
    parameter IDW    =  4
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         valid_in,

    input  wire [DC_MAX*Z*QIN-1:0]      beta_in,
    output reg  [DC_MAX*Z*QW-1:0]       alpha_out,
    output wire                         valid_out,

    // Compressed fields — registered at QIN precision
    // Packing: field[ z*WIDTH +: WIDTH ]
    output wire [Z*(QIN-1)-1:0]         min1_z_out,
    output wire [Z*(QIN-1)-1:0]         min2_z_out,
    output wire [Z*IDW-1:0]             idx1_z_out,
    output wire [Z*IDW-1:0]             idx2_z_out,
    output wire [Z*DC_MAX-1:0]          sgn_z_out
);

    localparam MAG_OUT     = QW  - 1;
    localparam MAG_IN      = QIN - 1;
    localparam MAG_MAX_OUT = (1 << MAG_OUT) - 1;

    // -------------------------------------------------------------------------
    // Per-z internal wires
    // -------------------------------------------------------------------------
    wire [DC_MAX*QIN-1:0] cnu_in_z  [0:Z-1];
    wire [DC_MAX*QIN-1:0] cnu_out_z [0:Z-1];
    wire                  cnu_vo_z  [0:Z-1];

    // Compressed-field wires (QIN-precision, from cnu_iams pipeline register)
    wire [QIN-2:0]    cnu_min1 [0:Z-1];
    wire [QIN-2:0]    cnu_min2 [0:Z-1];
    wire [IDW-1:0]    cnu_idx1 [0:Z-1];
    wire [IDW-1:0]    cnu_idx2 [0:Z-1];
    wire [DC_MAX-1:0] cnu_sgn  [0:Z-1];

    // -------------------------------------------------------------------------
    // Generate Z instances
    // -------------------------------------------------------------------------
    genvar gz, gs;
    generate
        for (gz = 0; gz < Z; gz = gz + 1) begin : g_z_inst

            // Gather DC_MAX input messages for this z
            for (gs = 0; gs < DC_MAX; gs = gs + 1) begin : g_pack
                assign cnu_in_z[gz][gs*QIN +: QIN] =
                    beta_in[gs*Z*QIN + gz*QIN +: QIN];
            end

            // Scalar IAMS CNU — runs at QIN precision
            cnu_iams #(
                .DC  (DC_MAX),
                .QW  (QIN),
                .IDW (IDW)
            ) u_cnu (
                .clk         (clk),
                .rst_n       (rst_n),
                .valid_in    (valid_in),
                .msg_flat_in (cnu_in_z[gz]),
                .msg_flat_out(cnu_out_z[gz]),
                .valid_out   (cnu_vo_z[gz]),
                .min1_out    (cnu_min1[gz]),
                .min2_out    (cnu_min2[gz]),
                .idx1_out    (cnu_idx1[gz]),
                .idx2_out    (cnu_idx2[gz]),
                .sgn_out     (cnu_sgn[gz])
            );

            // Pack per-z compressed fields into flat output buses
            assign min1_z_out[gz*(QIN-1) +: (QIN-1)] = cnu_min1[gz];
            assign min2_z_out[gz*(QIN-1) +: (QIN-1)] = cnu_min2[gz];
            assign idx1_z_out[gz*IDW     +: IDW     ] = cnu_idx1[gz];
            assign idx2_z_out[gz*IDW     +: IDW     ] = cnu_idx2[gz];
            assign sgn_z_out [gz*DC_MAX  +: DC_MAX  ] = cnu_sgn [gz];

        end
    endgenerate

    // -------------------------------------------------------------------------
    // Width conversion QIN → QW (combinational, no extra pipeline stage)
    // -------------------------------------------------------------------------
    integer cv_s, cv_z;
    reg [QIN-1:0]    cnu_word;
    reg              s_bit;
    reg [MAG_IN-1:0] mag_full;
    reg [QIN-1:0]    c_abs;   // named intermediate — Verilog-2001 safe
    reg              ovf;
    reg [QW-1:0]     out_word;

    always @(*) begin
        for (cv_s = 0; cv_s < DC_MAX; cv_s = cv_s + 1) begin
            for (cv_z = 0; cv_z < Z; cv_z = cv_z + 1) begin

                cnu_word = cnu_out_z[cv_z][cv_s*QIN +: QIN];
                s_bit    = cnu_word[QIN-1];
                mag_full = cnu_word[QIN-2:0];

                if (QW > QIN) begin
                    // Case A: sign-extend (zero-pad upper mag bits)
                    out_word = { s_bit,
                                 { (MAG_OUT - MAG_IN) {1'b0} },
                                 mag_full };

                end else if (QW == QIN) begin
                    // Case B: pass-through
                    out_word = cnu_word;

                end else begin
                    // Case C: saturate (QW < QIN)
                    // Named intermediate required for Verilog-2001 compliance
                    c_abs = cnu_word[QIN-1] ? (~cnu_word + 1'b1) : cnu_word;
                    ovf   = |c_abs[MAG_IN-1:MAG_OUT];
                    if (ovf) begin
                        out_word = s_bit ? (~({1'b0, {MAG_OUT{1'b1}}}) + 1'b1)
                                         :   {1'b0, {MAG_OUT{1'b1}}};
                    end else begin
                        out_word = {s_bit, c_abs[MAG_OUT-1:0]};
                    end
                end

                alpha_out[cv_s*Z*QW + cv_z*QW +: QW] = out_word;
            end
        end
    end

    // valid_out: wired from instance 0 (all Z have same registered valid)
    assign valid_out = cnu_vo_z[0];

endmodule

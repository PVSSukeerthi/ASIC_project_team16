`timescale 1ns/1ps

module cnu_iams_z #(
    parameter Z      = 52,
    parameter QIN    =  5,
    parameter QW     =  7,
    parameter DC_MAX = 10,
    parameter IDW    =  4    // ceil(log2(DC_MAX)); 2^IDW >= DC_MAX required
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         valid_in,

    // VTC β messages: beta_in[ slot*Z*QIN + z*QIN +: QIN ]
    input  wire [DC_MAX*Z*QIN-1:0]      beta_in,

    // CTV α messages: alpha_out[ slot*Z*QW + z*QW +: QW ]
    output reg  [DC_MAX*Z*QW-1:0]       alpha_out,

    // valid_out: 2 cycles after valid_in
    output reg                          valid_out,

    // Compressed fields — packing: field[ z*WIDTH +: WIDTH ]
    output wire [Z*(QIN-1)-1:0]         min1_z_out,
    output wire [Z*(QIN-1)-1:0]         min2_z_out,
    output wire [Z*IDW-1:0]             idx1_z_out,
    output wire [Z*IDW-1:0]             idx2_z_out,
    output wire [Z*DC_MAX-1:0]          sgn_z_out
);

    // -------------------------------------------------------------------------
    // Derived parameters
    // -------------------------------------------------------------------------
    localparam HALF             = DC_MAX / 2;   // DC_MAX must be even
    localparam MAG_IN           = QIN - 1;
    localparam MAG_OUT          = QW  - 1;
    localparam [IDW-1:0] HALF_K = HALF;         // IDW-wide offset constant

    // -------------------------------------------------------------------------
    // Half-CNU I/O wires
    // -------------------------------------------------------------------------
    wire [HALF*QIN-1:0] cnu1_in  [0:Z-1];
    wire [HALF*QIN-1:0] cnu2_in  [0:Z-1];
    wire [HALF*QIN-1:0] cnu1_out [0:Z-1];   // not used in data path
    wire [HALF*QIN-1:0] cnu2_out [0:Z-1];   // not used in data path
    wire                cnu1_vo  [0:Z-1];
    wire                cnu2_vo  [0:Z-1];

    // Compressed fields from each half (QIN precision, registered in cnu_iams)
    // min1/min2 are [QW-2:0] on cnu_iams port; with .QW(QIN) that is [QIN-2:0]
    wire [MAG_IN-1:0]   h1_min1 [0:Z-1];
    wire [MAG_IN-1:0]   h1_min2 [0:Z-1];
    wire [IDW-1:0]      h1_idx1 [0:Z-1];
    wire [IDW-1:0]      h1_idx2 [0:Z-1];
    wire [HALF-1:0]     h1_sgn  [0:Z-1];

    wire [MAG_IN-1:0]   h2_min1 [0:Z-1];
    wire [MAG_IN-1:0]   h2_min2 [0:Z-1];
    wire [IDW-1:0]      h2_idx1 [0:Z-1];   // local index 0..HALF-1
    wire [IDW-1:0]      h2_idx2 [0:Z-1];   // local index 0..HALF-1
    wire [HALF-1:0]     h2_sgn  [0:Z-1];

    // -------------------------------------------------------------------------
    // Generate Z pairs of half-CNUs
    // -------------------------------------------------------------------------
    genvar gz, gs;
    generate
        for (gz = 0; gz < Z; gz = gz + 1) begin : g_z_inst

            for (gs = 0; gs < HALF; gs = gs + 1) begin : g_pack1
                assign cnu1_in[gz][gs*QIN +: QIN] =
                    beta_in[gs*Z*QIN + gz*QIN +: QIN];
            end

            for (gs = 0; gs < HALF; gs = gs + 1) begin : g_pack2
                assign cnu2_in[gz][gs*QIN +: QIN] =
                    beta_in[(HALF+gs)*Z*QIN + gz*QIN +: QIN];
            end

            cnu_iams #(.DC(HALF), .QW(QIN), .IDW(IDW)) u_cnu1 (
                .clk(clk), .rst_n(rst_n), .valid_in(valid_in),
                .msg_flat_in(cnu1_in[gz]),  .msg_flat_out(cnu1_out[gz]),
                .valid_out(cnu1_vo[gz]),
                .min1_out(h1_min1[gz]), .min2_out(h1_min2[gz]),
                .idx1_out(h1_idx1[gz]), .idx2_out(h1_idx2[gz]),
                .sgn_out(h1_sgn[gz])
            );

            cnu_iams #(.DC(HALF), .QW(QIN), .IDW(IDW)) u_cnu2 (
                .clk(clk), .rst_n(rst_n), .valid_in(valid_in),
                .msg_flat_in(cnu2_in[gz]),  .msg_flat_out(cnu2_out[gz]),
                .valid_out(cnu2_vo[gz]),
                .min1_out(h2_min1[gz]), .min2_out(h2_min2[gz]),
                .idx1_out(h2_idx1[gz]), .idx2_out(h2_idx2[gz]),
                .sgn_out(h2_sgn[gz])
            );

        end
    endgenerate

    // -------------------------------------------------------------------------
    // stage1_valid: wire (cnu1_vo is already registered inside cnu_iams,
    // so no extra FF here — avoids adding a spurious 3rd pipeline stage)
    // -------------------------------------------------------------------------
    wire valid_stage1 = cnu1_vo[0];   // all Z instances track identically

    // -------------------------------------------------------------------------
    // Combinational: C&S + Eq.(10) reconstruction + QIN→QW conversion
    // -------------------------------------------------------------------------
    reg [MAG_IN-1:0]  g_min1      [0:Z-1];
    reg [MAG_IN-1:0]  g_min2      [0:Z-1];
    reg [IDW-1:0]     g_idx1      [0:Z-1];
    reg [IDW-1:0]     g_idx2      [0:Z-1];
    reg [DC_MAX-1:0]  g_sgn       [0:Z-1];
    reg [MAG_IN-1:0]  g_delta     [0:Z-1];

    reg [DC_MAX*QW-1:0] alpha_z_comb [0:Z-1];

    reg [MAG_IN-1:0]    min1_z_reg [0:Z-1];
    reg [MAG_IN-1:0]    min2_z_reg [0:Z-1];
    reg [IDW-1:0]       idx1_z_reg [0:Z-1];
    reg [IDW-1:0]       idx2_z_reg [0:Z-1];
    reg [DC_MAX-1:0]    sgn_z_reg  [0:Z-1];

    integer cz, ce, ci, sx;
    reg [MAG_IN-1:0] four_min [0:3];
    reg [IDW-1:0]    four_idx [0:3];
    reg [MAG_IN-1:0] cs_min1, cs_min2;
    reg [IDW-1:0]    cs_idx1, cs_idx2;
    reg [IDW-1:0]    n_idx_g;
    reg [MAG_IN-1:0] alpha_mag_g;
    reg              out_sign_g, sign_total_g;
    reg signed [QIN-1:0] out_word_qin;
    reg [QW-1:0]         out_word_qw;
    reg [MAG_IN-1:0]     mag_tmp;

    always @(*) begin
        for (cz = 0; cz < Z; cz = cz + 1) begin

            // Step 1: merge sign vectors
            for (ce = 0; ce < HALF; ce = ce + 1) begin
                g_sgn[cz][ce]      = h1_sgn[cz][ce];
                g_sgn[cz][HALF+ce] = h2_sgn[cz][ce];
            end

            // Step 2: C&S — four candidates, find global min1/min2
            four_min[0] = h1_min1[cz]; four_idx[0] = h1_idx1[cz];
            four_min[1] = h1_min2[cz]; four_idx[1] = h1_idx2[cz];
            four_min[2] = h2_min1[cz]; four_idx[2] = h2_idx1[cz] + HALF_K;
            four_min[3] = h2_min2[cz]; four_idx[3] = h2_idx2[cz] + HALF_K;

            cs_min1 = {MAG_IN{1'b1}}; cs_min2 = {MAG_IN{1'b1}};
            cs_idx1 = {IDW{1'b0}};    cs_idx2 = {IDW{1'b0}};

            for (ci = 0; ci < 4; ci = ci + 1) begin
                if (four_min[ci] <= cs_min1) begin
                    cs_min2 = cs_min1; cs_idx2 = cs_idx1;
                    cs_min1 = four_min[ci]; cs_idx1 = four_idx[ci];
                end else if (four_min[ci] < cs_min2) begin
                    cs_min2 = four_min[ci]; cs_idx2 = four_idx[ci];
                end
            end

            g_min1[cz]  = cs_min1;
            g_min2[cz]  = cs_min2;
            g_idx1[cz]  = cs_idx1;
            g_idx2[cz]  = cs_idx2;
            g_delta[cz] = cs_min2 - cs_min1;

            // Step 3: global sign total (parametric loop, not hardcoded)
            sign_total_g = 1'b0;
            for (sx = 0; sx < DC_MAX; sx = sx + 1)
                sign_total_g = sign_total_g ^ g_sgn[cz][sx];

            // Step 4: per-edge Eq.(10) + QIN→QW
            for (ce = 0; ce < DC_MAX; ce = ce + 1) begin
                out_sign_g = sign_total_g ^ g_sgn[cz][ce];
                n_idx_g    = ce[IDW-1:0];

                if (n_idx_g == g_idx1[cz])
                    alpha_mag_g = g_min2[cz];
                else if (n_idx_g == g_idx2[cz])
                    alpha_mag_g = g_min1[cz];
                else begin
                    if (g_delta[cz] == {MAG_IN{1'b0}})
                        alpha_mag_g = (g_min1[cz] > {{(MAG_IN-1){1'b0}}, 1'b1})
                                      ? (g_min1[cz] - 1'b1) : {MAG_IN{1'b0}};
                    else
                        alpha_mag_g = g_min1[cz];
                end

                if (alpha_mag_g == {MAG_IN{1'b0}})
                    out_word_qin = {QIN{1'b0}};
                else if (out_sign_g)
                    out_word_qin = ~({1'b0, alpha_mag_g}) + {{(QIN-1){1'b0}}, 1'b1};
                else
                    out_word_qin = {1'b0, alpha_mag_g};

                // QIN→QW width conversion
                if (QW > QIN)
                    out_word_qw = {out_word_qin[QIN-1],
                                   {(MAG_OUT-MAG_IN){1'b0}},
                                   out_word_qin[MAG_IN-1:0]};
                else if (QW == QIN)
                    out_word_qw = out_word_qin;
                else begin
                    mag_tmp = out_word_qin[QIN-2:0];
                    if (|mag_tmp[MAG_IN-1:MAG_OUT])
                        out_word_qw = out_word_qin[QIN-1]
                                      ? (~({1'b0,{MAG_OUT{1'b1}}}) + 1'b1)
                                      :   {1'b0,{MAG_OUT{1'b1}}};
                    else
                        out_word_qw = {out_word_qin[QIN-1], mag_tmp[MAG_OUT-1:0]};
                end

                alpha_z_comb[cz][ce*QW +: QW] = out_word_qw;
            end

        end
    end

    // -------------------------------------------------------------------------
    // Stage-2 pipeline register
    // -------------------------------------------------------------------------
    integer rz, re;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            for (rz = 0; rz < Z; rz = rz + 1) begin
                min1_z_reg[rz] <= {MAG_IN{1'b0}};
                min2_z_reg[rz] <= {MAG_IN{1'b0}};
                idx1_z_reg[rz] <= {IDW{1'b0}};
                idx2_z_reg[rz] <= {IDW{1'b0}};
                sgn_z_reg [rz] <= {DC_MAX{1'b0}};
                for (re = 0; re < DC_MAX; re = re + 1)
                    alpha_out[re*Z*QW + rz*QW +: QW] <= {QW{1'b0}};
            end
        end else begin
            valid_out <= valid_stage1;
            for (rz = 0; rz < Z; rz = rz + 1) begin
                min1_z_reg[rz] <= g_min1[rz];
                min2_z_reg[rz] <= g_min2[rz];
                idx1_z_reg[rz] <= g_idx1[rz];
                idx2_z_reg[rz] <= g_idx2[rz];
                sgn_z_reg [rz] <= g_sgn [rz];
                for (re = 0; re < DC_MAX; re = re + 1)
                    alpha_out[re*Z*QW + rz*QW +: QW] <= alpha_z_comb[rz][re*QW +: QW];
            end
        end
    end

    // -------------------------------------------------------------------------
    // Flat compressed-field outputs from stage-2 registers
    // -------------------------------------------------------------------------
    genvar fz;
    generate
        for (fz = 0; fz < Z; fz = fz + 1) begin : g_fields
            assign min1_z_out[fz*(QIN-1) +: (QIN-1)] = min1_z_reg[fz];
            assign min2_z_out[fz*(QIN-1) +: (QIN-1)] = min2_z_reg[fz];
            assign idx1_z_out[fz*IDW     +: IDW     ] = idx1_z_reg[fz];
            assign idx2_z_out[fz*IDW     +: IDW     ] = idx2_z_reg[fz];
            assign sgn_z_out [fz*DC_MAX  +: DC_MAX  ] = sgn_z_reg [fz];
        end
    endgenerate

endmodule

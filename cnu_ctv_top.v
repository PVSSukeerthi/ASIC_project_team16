`timescale 1ns/1ps

module cnu_ctv_top #(
    
    parameter Z        = 52,   // QC lifting factor          (BG2)
    parameter QIN      =  5,   // β input width  (sign+4mag) (BG2 Table I)
    parameter QW       =  7,   // α output width (sign+6mag) (BG2 Table I)
    parameter DC_MAX   = 10,   // max check-node degree      (BG2)
    parameter IDW      =  4,   // ceil(log2(DC_MAX))

    parameter N_LAYERS = 42,   // total decoding layers = BG rows (BG2)
    parameter ADRW     =  6    // ceil(log2(N_LAYERS))
) (
    input  wire  clk,
    input  wire  rst_n,

    input  wire  [DC_MAX*Z*QIN-1:0]    beta_in,
    input  wire                        cnu_valid_in,   // pulse high for 1 cycle

    
    output wire  [DC_MAX*Z*QW-1:0]     cnu_alpha_out,
    output wire                        cnu_valid_out,  // 1 cycle after cnu_valid_in

    input  wire                        wr_en,
    input  wire  [ADRW-1:0]           layer_wr,       // layer index to write


    input  wire                        rd_en,
    input  wire  [ADRW-1:0]           layer_rd,       // layer index to read

    output wire  [DC_MAX*Z*QW-1:0]     ctv_alpha_out,
    output wire                        ctv_valid_out   // high 2 cycles after rd_en
);

 localparam MAG_OUT = QW - 1;
localparam CWORD = DC_MAX + 2*MAG_OUT + 2*IDW;

    wire [Z*CWORD-1:0] w_ctv_word;

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

        .ctv_word_out (w_ctv_word)
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

        .ctv_word_in (w_ctv_word),

        // Decompressed α messages out (to VNU for β = γ̃ − α)
        .alpha_out (ctv_alpha_out),
        .valid_out (ctv_valid_out)
    );

endmodule

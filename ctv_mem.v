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

    input wire [Z*CWORD-1:0] ctv_word_in,

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

  
    // Synchronous write
    integer wi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (wi = 0; wi < N_LAYERS; wi = wi + 1)
                mem[wi] <= {MEM_W{1'b0}};
        end else if (wr_en) begin
            // mem[layer_wr] <= pack_word;
            mem[layer_wr] <= ctv_word_in;
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

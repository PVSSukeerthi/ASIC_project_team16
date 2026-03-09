`include "ldpc5g_pkg.v"

module ctv_memory #(
    parameter Z          = `Z,
    parameter Q          = `Q,
    parameter DC_MAX_P   = `DC_MAX,
    parameter DC_ORT     = `DC_MAX_ORT,
    parameter L_TOTAL    = `L_AFTER,
    parameter L_CORE_ORT = 15,
    parameter LOG2_DC    = `LOG2_DC_MAX,
    parameter LOG2_DC_O  = `LOG2_DC_ORT
)(
    input  wire                       clk,
    input  wire                       rst_n,
    // Write port
    input  wire [4:0]                 wr_layer,
    input  wire                       wr_en,
    // Compressed CTV - set 1
    input  wire [Z*(Q-1)-1:0]         wr_min1_1,
    input  wire [Z*(Q-1)-1:0]         wr_min2_1,
    input  wire [Z*4-1:0]             wr_idx1_1,
    input  wire [Z*4-1:0]             wr_idx2_1,
    input  wire [Z*DC_MAX_P-1:0]      wr_signs_1,
    // Compressed CTV - set 2 (merged layers)
    input  wire [Z*(Q-1)-1:0]         wr_min1_2,
    input  wire [Z*(Q-1)-1:0]         wr_min2_2,
    input  wire [Z*4-1:0]             wr_idx1_2,
    input  wire [Z*4-1:0]             wr_idx2_2,
    input  wire [Z*DC_MAX_P-1:0]      wr_signs_2,
    input  wire                       wr_set2_valid,
    // Read port
    input  wire [4:0]                 rd_layer,
    input  wire                       rd_en,
    // Read outputs - set 1
    output reg  [Z*(Q-1)-1:0]         rd_min1_1,
    output reg  [Z*(Q-1)-1:0]         rd_min2_1,
    output reg  [Z*4-1:0]             rd_idx1_1,
    output reg  [Z*4-1:0]             rd_idx2_1,
    output reg  [Z*DC_MAX_P-1:0]      rd_signs_1,
    // Read outputs - set 2
    output reg  [Z*(Q-1)-1:0]         rd_min1_2,
    output reg  [Z*(Q-1)-1:0]         rd_min2_2,
    output reg  [Z*4-1:0]             rd_idx1_2,
    output reg  [Z*4-1:0]             rd_idx2_2,
    output reg  [Z*DC_MAX_P-1:0]      rd_signs_2,
    output reg                        rd_valid
);

localparam MWIDTH    = Q - 1;
// Memory 1 word width per Z-block: DC_ORT signs + min1 + min2 + LOG2_DC_O idx x2
localparam W1_BLOCK  = DC_ORT + 2*MWIDTH + 2*LOG2_DC_O;  // 5+6+6=17
// Memory 2 word width per Z-block: extra signs + extra idx bits
localparam W2_BLOCK  = (DC_MAX_P-DC_ORT) + 2*(LOG2_DC-LOG2_DC_O);  // 5+2=7
localparam W1_TOTAL  = Z * W1_BLOCK;
localparam W2_TOTAL  = Z * W2_BLOCK;

// ---- Memory arrays ----
reg [W1_TOTAL-1:0] mem1 [0:L_TOTAL-1];
reg [W2_TOTAL-1:0] mem2 [0:L_CORE_ORT-1];

// ---- Write ----
integer wr_z;
reg [W1_TOTAL-1:0] w1_d;
reg [W2_TOTAL-1:0] w2_d;
reg [W1_BLOCK-1:0] w1_blk;
reg [W2_BLOCK-1:0] w2_blk;

always @(posedge clk) begin
    if (wr_en) begin
        for (wr_z = 0; wr_z < Z; wr_z = wr_z+1) begin
            // Pack Memory 1 block: {DC_ORT signs, min1, min2, idx1[LOG2_DC_O-1:0], idx2[LOG2_DC_O-1:0]}
            w1_blk = { wr_signs_1[wr_z*DC_MAX_P +: DC_ORT],
                       wr_min1_1 [wr_z*MWIDTH    +: MWIDTH],
                       wr_min2_1 [wr_z*MWIDTH    +: MWIDTH],
                       wr_idx1_1 [wr_z*4         +: LOG2_DC_O],
                       wr_idx2_1 [wr_z*4         +: LOG2_DC_O] };
            w1_d[wr_z*W1_BLOCK +: W1_BLOCK] = w1_blk;

            // Pack Memory 2 block: {extra signs[DC_MAX_P-1:DC_ORT], idx1 high bits, idx2 high bits}
            w2_blk = { wr_signs_1[wr_z*DC_MAX_P+DC_ORT +: (DC_MAX_P-DC_ORT)],
                       wr_idx1_1 [wr_z*4+LOG2_DC_O +: (LOG2_DC-LOG2_DC_O)],
                       wr_idx2_1 [wr_z*4+LOG2_DC_O +: (LOG2_DC-LOG2_DC_O)] };
            w2_d[wr_z*W2_BLOCK +: W2_BLOCK] = w2_blk;
        end
        mem1[wr_layer] <= w1_d;
        if (wr_layer < L_CORE_ORT)
            mem2[wr_layer[3:0]] <= w2_d;
    end
end

// ---- Read ----
integer rd_z;
reg [W1_TOTAL-1:0] r1_d;
reg [W2_TOTAL-1:0] r2_d;
reg [W1_BLOCK-1:0] r1_blk;
reg [W2_BLOCK-1:0] r2_blk;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_valid  <= 1'b0;
        rd_min1_1 <= 0; rd_min2_1 <= 0;
        rd_idx1_1 <= 0; rd_idx2_1 <= 0;
        rd_signs_1<= 0;
        rd_min1_2 <= 0; rd_min2_2 <= 0;
        rd_idx1_2 <= 0; rd_idx2_2 <= 0;
        rd_signs_2<= 0;
    end else if (rd_en) begin
        rd_valid <= 1'b1;
        r1_d = mem1[rd_layer];
        r2_d = (rd_layer < L_CORE_ORT) ? mem2[rd_layer[3:0]]
                                        : {W2_TOTAL{1'b0}};
        for (rd_z = 0; rd_z < Z; rd_z = rd_z+1) begin
            r1_blk = r1_d[rd_z*W1_BLOCK +: W1_BLOCK];
            r2_blk = r2_d[rd_z*W2_BLOCK +: W2_BLOCK];
            // Unpack set 1
            rd_signs_1[rd_z*DC_MAX_P +: DC_ORT]
                <= r1_blk[W1_BLOCK-1 -: DC_ORT];
            rd_signs_1[rd_z*DC_MAX_P+DC_ORT +: (DC_MAX_P-DC_ORT)]
                <= r2_blk[W2_BLOCK-1 -: (DC_MAX_P-DC_ORT)];
            rd_min1_1[rd_z*MWIDTH +: MWIDTH]
                <= r1_blk[W1_BLOCK-DC_ORT-1 -: MWIDTH];
            rd_min2_1[rd_z*MWIDTH +: MWIDTH]
                <= r1_blk[W1_BLOCK-DC_ORT-MWIDTH-1 -: MWIDTH];
            rd_idx1_1[rd_z*4 +: 4]
                <= { r2_blk[(LOG2_DC-LOG2_DC_O)*2-1 -: (LOG2_DC-LOG2_DC_O)],
                     r1_blk[2*LOG2_DC_O-1           -: LOG2_DC_O] };
            rd_idx2_1[rd_z*4 +: 4]
                <= { r2_blk[(LOG2_DC-LOG2_DC_O)-1   -: (LOG2_DC-LOG2_DC_O)],
                     r1_blk[LOG2_DC_O-1             -: LOG2_DC_O] };
            // Set 2 zeros (simplified; full impl stores second set)
            rd_min1_2[rd_z*MWIDTH +: MWIDTH] <= {MWIDTH{1'b0}};
            rd_min2_2[rd_z*MWIDTH +: MWIDTH] <= {MWIDTH{1'b0}};
            rd_idx1_2[rd_z*4 +: 4] <= 4'd0;
            rd_idx2_2[rd_z*4 +: 4] <= 4'd0;
            rd_signs_2[rd_z*DC_MAX_P +: DC_MAX_P] <= {DC_MAX_P{1'b0}};
        end
    end else begin
        rd_valid <= 1'b0;
    end
end

endmodule

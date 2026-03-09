`include "ldpc5g_pkg.v"

module decompressor #(
    parameter Z        = `Z,
    parameter DC_MAX_P = `DC_MAX,
    parameter Q        = `Q,
    parameter LOG2_DC  = `LOG2_DC_MAX
)(
    input  wire [Z*DC_MAX_P-1:0]     signs,
    input  wire [Z*(Q-1)-1:0]        min1,
    input  wire [Z*(Q-1)-1:0]        min2,
    input  wire [Z*LOG2_DC-1:0]      idx1,
    input  wire [Z*LOG2_DC-1:0]      idx2,
    input  wire [3:0]                dc_actual,
    output wire [Z*DC_MAX_P*Q-1:0]   alpha
);

localparam MWIDTH = Q - 1;

genvar gz, gn;
generate
    for (gz = 0; gz < Z; gz = gz+1) begin : dz
        wire [MWIDTH-1:0] gz_min1, gz_min2;
        wire [LOG2_DC-1:0] gz_idx1, gz_idx2;
        assign gz_min1 = min1[gz*MWIDTH +: MWIDTH];
        assign gz_min2 = min2[gz*MWIDTH +: MWIDTH];
        assign gz_idx1 = idx1[gz*LOG2_DC +: LOG2_DC];
        assign gz_idx2 = idx2[gz*LOG2_DC +: LOG2_DC];

        for (gn = 0; gn < DC_MAX_P; gn = gn+1) begin : dn
            wire is_idx1, is_idx2;
            assign is_idx1 = (gn[3:0] == gz_idx1[3:0]) && (gn < dc_actual);
            assign is_idx2 = (gn[3:0] == gz_idx2[3:0]) && (gn < dc_actual) && !is_idx1;

            wire [MWIDTH-1:0] mag;
            assign mag = is_idx1 ? gz_min2 : gz_min1;

            wire sgn;
            assign sgn = signs[gz*DC_MAX_P + gn];

            wire [Q-1:0] out_val;
            assign out_val = (gn < dc_actual) ?
                             ((mag == {MWIDTH{1'b0}}) ? {Q{1'b0}} : {sgn, mag}) :
                             {Q{1'b0}};

            assign alpha[(gz*DC_MAX_P+gn)*Q +: Q] = out_val;
        end
    end
endgenerate

endmodule

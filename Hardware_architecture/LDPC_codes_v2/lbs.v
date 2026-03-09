`include "ldpc5g_pkg.v"

module lbs #(
    parameter Z   = `Z,
    parameter Q   = `Q
)(
    input  wire [5:0]         shift_val,
    input  wire [Z*Q-1:0]     data_in,
    output wire [Z*Q-1:0]     data_out
);

genvar gi;
generate
    for (gi = 0; gi < Z; gi = gi+1) begin : rot_out
        wire [6:0] src_idx;
        assign src_idx = (gi + shift_val >= Z) ? (gi + shift_val - Z)
                                               : (gi + shift_val);
        assign data_out[gi*Q +: Q] = data_in[src_idx[5:0]*Q +: Q];
    end
endgenerate

endmodule

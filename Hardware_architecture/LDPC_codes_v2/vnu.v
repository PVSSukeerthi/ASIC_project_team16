`include "ldpc5g_pkg.v"

module vnu #(
    parameter Q      = `Q,
    parameter QTILDE = `QTILDE
)(
    input  wire signed [QTILDE-1:0] app_in,
    input  wire signed [Q-1:0]      ctv_old,
    output wire signed [Q-1:0]      vtc_out
);

// Sign-extend ctv_old to QTILDE bits
wire signed [QTILDE-1:0] ctv_ext;
assign ctv_ext = {{(QTILDE-Q){ctv_old[Q-1]}}, ctv_old};

// Compute difference
wire signed [QTILDE-1:0] diff;
assign diff = app_in - ctv_ext;

// Saturate to Q-bit range [MSG_MIN, MSG_MAX] = [-8, +7]
// Use explicit literals for Verilog-2001 compatibility
assign vtc_out = (diff > 6'sd7)  ?  4'sd7  :
                 (diff < -6'sd8) ? -4'sd8  :
                  diff[Q-1:0];

endmodule


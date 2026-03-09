`include "ldpc5g_pkg.v"

module app_update #(
    parameter Q      = `Q,
    parameter QTILDE = `QTILDE
)(
    input  wire signed [QTILDE-1:0] app_in,
    input  wire signed [Q-1:0]      ctv_old,
    input  wire signed [Q-1:0]      ctv_new,
    output wire signed [QTILDE-1:0] app_out
);

// Compute beta_tilde = APP - alpha_old (keep QTILDE precision)
wire signed [QTILDE-1:0] ctv_old_ext, ctv_new_ext;
assign ctv_old_ext = {{(QTILDE-Q){ctv_old[Q-1]}}, ctv_old};
assign ctv_new_ext = {{(QTILDE-Q){ctv_new[Q-1]}}, ctv_new};

wire signed [QTILDE-1:0] beta_tilde;
assign beta_tilde = app_in - ctv_old_ext;

// Sum with extra bit for overflow detection
wire signed [QTILDE:0] sum_ext;
assign sum_ext = {beta_tilde[QTILDE-1], beta_tilde} +
                 {ctv_new_ext[QTILDE-1], ctv_new_ext};

// Saturate to QTILDE range [-32, +31]
assign app_out = (sum_ext > 7'sd31)  ?  6'sd31  :
                 (sum_ext < -7'sd32) ? -6'sd32  :
                  sum_ext[QTILDE-1:0];

endmodule


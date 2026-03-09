`include "ldpc5g_pkg.v"

module cnu_top #(
    parameter Z        = `Z,
    parameter DC_MAX_P = `DC_MAX,
    parameter Q        = `Q
)(
    input  wire                          clk,
    input  wire                          rst_n,
    // CNU1st inputs
    input  wire [Z*DC_MAX_P*Q-1:0]       vtc1,
    input  wire [3:0]                    dc1_actual,
    input  wire                          use_oms1,
    // CNU2nd inputs
    input  wire [Z*DC_MAX_P*Q-1:0]       vtc2,
    input  wire [3:0]                    dc2_actual,
    input  wire                          use_oms2,
    input  wire                          layer_merge,
    // Outputs: CTV messages (flat)
    output reg  [Z*DC_MAX_P*Q-1:0]       ctv1_out,
    output reg  [Z*DC_MAX_P*Q-1:0]       ctv2_out,
    // Compressed storage fields
    output reg  [Z*(Q-1)-1:0]            min1_1,
    output reg  [Z*(Q-1)-1:0]            min2_1,
    output reg  [Z*4-1:0]                idx1_1,
    output reg  [Z*4-1:0]                idx2_1,
    output reg  [Z*(Q-1)-1:0]            min1_2,
    output reg  [Z*(Q-1)-1:0]            min2_2,
    output reg  [Z*4-1:0]                idx1_2,
    output reg  [Z*4-1:0]                idx2_2,
    output reg                           valid_out
);

localparam MWIDTH = Q - 1;

// ---- Instantiate Z CNU1st and Z CNU2nd ----
wire [Z*DC_MAX_P*Q-1:0] cnu1_out_flat;
wire [Z*DC_MAX_P*Q-1:0] cnu2_out_flat;

genvar gz;
generate
    for (gz = 0; gz < Z; gz = gz+1) begin : cnu_array
        cnu_single #(.DC_MAX_P(DC_MAX_P), .Q(Q)) u_cnu1 (
            .beta_in_0 (vtc1[(gz*DC_MAX_P+0)*Q +: Q]),
            .beta_in_1 (vtc1[(gz*DC_MAX_P+1)*Q +: Q]),
            .beta_in_2 (vtc1[(gz*DC_MAX_P+2)*Q +: Q]),
            .beta_in_3 (vtc1[(gz*DC_MAX_P+3)*Q +: Q]),
            .beta_in_4 (vtc1[(gz*DC_MAX_P+4)*Q +: Q]),
            .beta_in_5 (vtc1[(gz*DC_MAX_P+5)*Q +: Q]),
            .beta_in_6 (vtc1[(gz*DC_MAX_P+6)*Q +: Q]),
            .beta_in_7 (vtc1[(gz*DC_MAX_P+7)*Q +: Q]),
            .beta_in_8 (vtc1[(gz*DC_MAX_P+8)*Q +: Q]),
            .beta_in_9 (vtc1[(gz*DC_MAX_P+9)*Q +: Q]),
            .dc_actual (dc1_actual),
            .use_oms   (use_oms1),
            .alpha_out_0 (cnu1_out_flat[(gz*DC_MAX_P+0)*Q +: Q]),
            .alpha_out_1 (cnu1_out_flat[(gz*DC_MAX_P+1)*Q +: Q]),
            .alpha_out_2 (cnu1_out_flat[(gz*DC_MAX_P+2)*Q +: Q]),
            .alpha_out_3 (cnu1_out_flat[(gz*DC_MAX_P+3)*Q +: Q]),
            .alpha_out_4 (cnu1_out_flat[(gz*DC_MAX_P+4)*Q +: Q]),
            .alpha_out_5 (cnu1_out_flat[(gz*DC_MAX_P+5)*Q +: Q]),
            .alpha_out_6 (cnu1_out_flat[(gz*DC_MAX_P+6)*Q +: Q]),
            .alpha_out_7 (cnu1_out_flat[(gz*DC_MAX_P+7)*Q +: Q]),
            .alpha_out_8 (cnu1_out_flat[(gz*DC_MAX_P+8)*Q +: Q]),
            .alpha_out_9 (cnu1_out_flat[(gz*DC_MAX_P+9)*Q +: Q])
        );
        cnu_single #(.DC_MAX_P(DC_MAX_P), .Q(Q)) u_cnu2 (
            .beta_in_0 (vtc2[(gz*DC_MAX_P+0)*Q +: Q]),
            .beta_in_1 (vtc2[(gz*DC_MAX_P+1)*Q +: Q]),
            .beta_in_2 (vtc2[(gz*DC_MAX_P+2)*Q +: Q]),
            .beta_in_3 (vtc2[(gz*DC_MAX_P+3)*Q +: Q]),
            .beta_in_4 (vtc2[(gz*DC_MAX_P+4)*Q +: Q]),
            .beta_in_5 (vtc2[(gz*DC_MAX_P+5)*Q +: Q]),
            .beta_in_6 (vtc2[(gz*DC_MAX_P+6)*Q +: Q]),
            .beta_in_7 (vtc2[(gz*DC_MAX_P+7)*Q +: Q]),
            .beta_in_8 (vtc2[(gz*DC_MAX_P+8)*Q +: Q]),
            .beta_in_9 (vtc2[(gz*DC_MAX_P+9)*Q +: Q]),
            .dc_actual (dc2_actual),
            .use_oms   (use_oms2),
            .alpha_out_0 (cnu2_out_flat[(gz*DC_MAX_P+0)*Q +: Q]),
            .alpha_out_1 (cnu2_out_flat[(gz*DC_MAX_P+1)*Q +: Q]),
            .alpha_out_2 (cnu2_out_flat[(gz*DC_MAX_P+2)*Q +: Q]),
            .alpha_out_3 (cnu2_out_flat[(gz*DC_MAX_P+3)*Q +: Q]),
            .alpha_out_4 (cnu2_out_flat[(gz*DC_MAX_P+4)*Q +: Q]),
            .alpha_out_5 (cnu2_out_flat[(gz*DC_MAX_P+5)*Q +: Q]),
            .alpha_out_6 (cnu2_out_flat[(gz*DC_MAX_P+6)*Q +: Q]),
            .alpha_out_7 (cnu2_out_flat[(gz*DC_MAX_P+7)*Q +: Q]),
            .alpha_out_8 (cnu2_out_flat[(gz*DC_MAX_P+8)*Q +: Q]),
            .alpha_out_9 (cnu2_out_flat[(gz*DC_MAX_P+9)*Q +: Q])
        );
    end
endgenerate

// ---- Register outputs + extract compressed fields ----
integer z, d;
reg [MWIDTH-1:0] em1, em2;
reg [3:0]        ei1, ei2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        valid_out <= 1'b0;
        ctv1_out  <= {(Z*DC_MAX_P*Q){1'b0}};
        ctv2_out  <= {(Z*DC_MAX_P*Q){1'b0}};
        min1_1 <= 0; min2_1 <= 0; idx1_1 <= 0; idx2_1 <= 0;
        min1_2 <= 0; min2_2 <= 0; idx1_2 <= 0; idx2_2 <= 0;
    end else begin
        valid_out <= 1'b1;
        for (z = 0; z < Z; z = z+1) begin
            // Copy CNU outputs
            for (d = 0; d < DC_MAX_P; d = d+1) begin
                ctv1_out[(z*DC_MAX_P+d)*Q +: Q] <=
                    cnu1_out_flat[(z*DC_MAX_P+d)*Q +: Q];
                ctv2_out[(z*DC_MAX_P+d)*Q +: Q] <=
                    layer_merge ? cnu2_out_flat[(z*DC_MAX_P+d)*Q +: Q]
                                : {Q{1'b0}};
            end

            // Extract min1/min2/idx1/idx2 from CNU1 outputs
            em1 = {MWIDTH{1'b1}}; em2 = {MWIDTH{1'b1}};
            ei1 = 4'd0; ei2 = 4'd1;
            for (d = 0; d < DC_MAX_P; d = d+1) begin
                if (d < dc1_actual) begin
                    if (cnu1_out_flat[(z*DC_MAX_P+d)*Q + Q-2 -: MWIDTH] < em1) begin
                        em2 = em1; ei2 = ei1;
                        em1 = cnu1_out_flat[(z*DC_MAX_P+d)*Q + Q-2 -: MWIDTH];
                        ei1 = d[3:0];
                    end else if (cnu1_out_flat[(z*DC_MAX_P+d)*Q + Q-2 -: MWIDTH] < em2) begin
                        em2 = cnu1_out_flat[(z*DC_MAX_P+d)*Q + Q-2 -: MWIDTH];
                        ei2 = d[3:0];
                    end
                end
            end
            min1_1[z*MWIDTH +: MWIDTH] <= em1;
            min2_1[z*MWIDTH +: MWIDTH] <= em2;
            idx1_1[z*4 +: 4] <= ei1;
            idx2_1[z*4 +: 4] <= ei2;

            // Extract from CNU2 outputs (only meaningful if layer_merge)
            em1 = {MWIDTH{1'b1}}; em2 = {MWIDTH{1'b1}};
            ei1 = 4'd0; ei2 = 4'd1;
            for (d = 0; d < DC_MAX_P; d = d+1) begin
                if (d < dc2_actual) begin
                    if (cnu2_out_flat[(z*DC_MAX_P+d)*Q + Q-2 -: MWIDTH] < em1) begin
                        em2 = em1; ei2 = ei1;
                        em1 = cnu2_out_flat[(z*DC_MAX_P+d)*Q + Q-2 -: MWIDTH];
                        ei1 = d[3:0];
                    end else if (cnu2_out_flat[(z*DC_MAX_P+d)*Q + Q-2 -: MWIDTH] < em2) begin
                        em2 = cnu2_out_flat[(z*DC_MAX_P+d)*Q + Q-2 -: MWIDTH];
                        ei2 = d[3:0];
                    end
                end
            end
            min1_2[z*MWIDTH +: MWIDTH] <= em1;
            min2_2[z*MWIDTH +: MWIDTH] <= em2;
            idx1_2[z*4 +: 4] <= ei1;
            idx2_2[z*4 +: 4] <= ei2;
        end
    end
end

endmodule

`include "ldpc5g_pkg.v"

module cnu_single #(
    parameter DC_MAX_P = `DC_MAX,   // 10
    parameter Q        = `Q         // 4
)(
    // Flat signed Q-bit inputs
    input  wire signed [Q-1:0] beta_in_0,
    input  wire signed [Q-1:0] beta_in_1,
    input  wire signed [Q-1:0] beta_in_2,
    input  wire signed [Q-1:0] beta_in_3,
    input  wire signed [Q-1:0] beta_in_4,
    input  wire signed [Q-1:0] beta_in_5,
    input  wire signed [Q-1:0] beta_in_6,
    input  wire signed [Q-1:0] beta_in_7,
    input  wire signed [Q-1:0] beta_in_8,
    input  wire signed [Q-1:0] beta_in_9,
    input  wire [3:0]           dc_actual,  // 1..DC_MAX_P
    input  wire                 use_oms,    // 0=IAMS, 1=OMS
    // Flat signed Q-bit outputs
    output wire signed [Q-1:0] alpha_out_0,
    output wire signed [Q-1:0] alpha_out_1,
    output wire signed [Q-1:0] alpha_out_2,
    output wire signed [Q-1:0] alpha_out_3,
    output wire signed [Q-1:0] alpha_out_4,
    output wire signed [Q-1:0] alpha_out_5,
    output wire signed [Q-1:0] alpha_out_6,
    output wire signed [Q-1:0] alpha_out_7,
    output wire signed [Q-1:0] alpha_out_8,
    output wire signed [Q-1:0] alpha_out_9
);

localparam MWIDTH = Q - 1;  // 3 magnitude bits

// ---- Collect inputs into internal array ----
wire signed [Q-1:0] beta [0:DC_MAX_P-1];
assign beta[0] = beta_in_0; assign beta[1] = beta_in_1;
assign beta[2] = beta_in_2; assign beta[3] = beta_in_3;
assign beta[4] = beta_in_4; assign beta[5] = beta_in_5;
assign beta[6] = beta_in_6; assign beta[7] = beta_in_7;
assign beta[8] = beta_in_8; assign beta[9] = beta_in_9;

// ---- Magnitude and sign extraction ----
wire [MWIDTH-1:0] mag  [0:DC_MAX_P-1];
wire              sbit [0:DC_MAX_P-1];   // 1 = negative
genvar gi;
generate
    for (gi = 0; gi < DC_MAX_P; gi = gi+1) begin : ME
        assign mag[gi]  = beta[gi][Q-2:0];
        assign sbit[gi] = beta[gi][Q-1];
    end
endgenerate

// ---- Find min1, min2, idx1, idx2 (combinational) ----
reg [MWIDTH-1:0] min1, min2;
reg [3:0]        idx1, idx2;
integer ii;
always @(*) begin
    min1 = {MWIDTH{1'b1}};  // initialise to all-ones (max)
    min2 = {MWIDTH{1'b1}};
    idx1 = 4'd0;
    idx2 = 4'd1;
    for (ii = 0; ii < DC_MAX_P; ii = ii+1) begin
        if (ii < dc_actual) begin
            if (mag[ii] < min1) begin
                min2 = min1; idx2 = idx1;
                min1 = mag[ii]; idx1 = ii[3:0];
            end else if (mag[ii] < min2) begin
                min2 = mag[ii]; idx2 = ii[3:0];
            end
        end
    end
end

// ---- Total XOR of all valid sign bits ----
reg total_xor;
always @(*) begin
    total_xor = 1'b0;
    for (ii = 0; ii < DC_MAX_P; ii = ii+1)
        if (ii < dc_actual)
            total_xor = total_xor ^ sbit[ii];
end

// ---- Delta = min2 - min1 ----
wire [MWIDTH-1:0] delta;
assign delta = min2 - min1;

// ---- Per-output computation ----
// alpha[n] computed for each of the 10 positions
wire signed [Q-1:0] alpha_arr [0:DC_MAX_P-1];

generate
    for (gi = 0; gi < DC_MAX_P; gi = gi+1) begin : OUT_GEN
        // Extrinsic sign = total XOR minus n's own contribution
        wire out_sign;
        assign out_sign = total_xor ^ sbit[gi];

        wire is_idx1, is_idx2;
        assign is_idx1 = (gi[3:0] == idx1);
        assign is_idx2 = (gi[3:0] == idx2) & ~is_idx1;

        // Magnitude excluding n (minimum of remaining)
        wire [MWIDTH-1:0] mag_excl;
        assign mag_excl = is_idx1 ? min2 : min1;

        // OMS magnitude: max(mag_excl - 1, 0)
        wire [MWIDTH-1:0] oms_mag;
        assign oms_mag = (mag_excl > 0) ? (mag_excl - 1'b1) : {MWIDTH{1'b0}};

        // IAMS magnitude per eq.(10)
        wire [MWIDTH-1:0] iams_mag;
        assign iams_mag =
            is_idx1 ? min2 :              // n=idx1: use min2
            is_idx2 ? min1 :              // n=idx2: use min1
            (delta == {MWIDTH{1'b0}}) ?   // n in I_bar, delta=0: lambda=1
                ((min1 > 0) ? (min1 - 1'b1) : {MWIDTH{1'b0}}) :
                min1;                     // n in I_bar, delta!=0: lambda=0

        wire [MWIDTH-1:0] final_mag;
        assign final_mag = use_oms ? oms_mag : iams_mag;

        // If node is inactive (gi >= dc_actual) output zero
        // If magnitude is zero, force positive sign
        assign alpha_arr[gi] =
            (gi >= dc_actual)             ? {Q{1'b0}} :
            (final_mag == {MWIDTH{1'b0}}) ? {Q{1'b0}} :
                                            {out_sign, final_mag};
    end
endgenerate

// ---- Drive flat outputs ----
assign alpha_out_0 = alpha_arr[0]; assign alpha_out_1 = alpha_arr[1];
assign alpha_out_2 = alpha_arr[2]; assign alpha_out_3 = alpha_arr[3];
assign alpha_out_4 = alpha_arr[4]; assign alpha_out_5 = alpha_arr[5];
assign alpha_out_6 = alpha_arr[6]; assign alpha_out_7 = alpha_arr[7];
assign alpha_out_8 = alpha_arr[8]; assign alpha_out_9 = alpha_arr[9];

endmodule

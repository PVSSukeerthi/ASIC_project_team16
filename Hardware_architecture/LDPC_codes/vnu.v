// =============================================================================
// vnu.v  —  Variable Node Unit (VNU)
// =============================================================================
// Implements the layered VN update used in ALL decoder variants (MS, OMS, IAMS).
//
// In a layered decoder the APP LLR is updated layer-by-layer:
//
//   β_{n,m}  = APP_n  −  α_{m,n}^(prev)          (extrinsic VN→CN message)
//   APP_n   += α_{m,n}^(new)  −  α_{m,n}^(prev)   (APP update)
//
// Ports (all signals per variable node n, per active layer m):
//   app_in       : current APP LLR of VN n  (signed QW bits)
//   alpha_new    : new CN→VN message from layer m  (signed QW bits)
//   alpha_prev   : previous iteration CN→VN message from layer m  (signed QW bits)
//   beta_out     : extrinsic VN→CN message β_{n,m}  (signed QW bits)
//   app_out      : updated APP LLR after incorporating layer m  (signed QW bits)
//
// Saturation: outputs clipped to [−SAT, +SAT] where SAT = 2^(QW-1)−1.
// =============================================================================

`timescale 1ns / 1ps

module vnu #(
    parameter QW  = 7,    // Message bit-width (signed)
    parameter QTW = 8     // APP LLR width (wider to avoid saturation, paper: Q̃=31)
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  valid_in,
    // APP state
    input  wire signed [QTW-1:0] app_in,       // Current APP LLR (Q̃ bits)
    // CN→VN messages
    input  wire signed [QW-1:0]  alpha_new,    // New CN→VN from current iteration
    input  wire signed [QW-1:0]  alpha_prev,   // CN→VN stored from last iteration
    // Outputs
    output wire signed [QW-1:0]  beta_out,     // VN→CN extrinsic message
    output reg  signed [QTW-1:0] app_out,      // Updated APP LLR
    output reg                   valid_out
);

    // -------------------------------------------------------------------------
    // Saturation helper for APP output (Q̃-bit signed)
    // -------------------------------------------------------------------------
    localparam signed [QTW-1:0] SAT_MAX =  (1 <<< (QTW-1)) - 1;
    localparam signed [QTW-1:0] SAT_MIN = -(1 <<< (QTW-1));

    // -------------------------------------------------------------------------
    // Saturation helper for message output (QW-bit signed)
    // -------------------------------------------------------------------------
    localparam signed [QW-1:0] MSG_MAX =  (1 <<< (QW-1)) - 1;
    localparam signed [QW-1:0] MSG_MIN = -(1 <<< (QW-1));

    // -------------------------------------------------------------------------
    // Combinational: beta = app_in - alpha_prev  (VN→CN extrinsic)
    //                app  = app_in + alpha_new - alpha_prev
    // Implemented as: app = beta + alpha_new  (two adders, one shared subtraction)
    // -------------------------------------------------------------------------
    wire signed [QTW:0] beta_wide;       // one extra bit for overflow detect
    wire signed [QTW:0] app_wide;

    assign beta_wide = {{1{app_in[QTW-1]}}, app_in}
                     - {{(QTW-QW+1){alpha_prev[QW-1]}}, alpha_prev};

    assign app_wide  = beta_wide
                     + {{(QTW-QW+1){alpha_new[QW-1]}}, alpha_new};

    // Saturate beta to QW bits (message width)
    function signed [QW-1:0] sat_msg;
        input signed [QTW:0] x;
        begin
            if (x > $signed({{(QTW-QW+2){1'b0}}, MSG_MAX}))
                sat_msg = MSG_MAX;
            else if (x < $signed({{(QTW-QW+2){1'b1}}, MSG_MIN}))
                sat_msg = MSG_MIN;
            else
                sat_msg = x[QW-1:0];
        end
    endfunction

    assign beta_out = sat_msg(beta_wide);

    // -------------------------------------------------------------------------
    // Pipeline register
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            app_out   <= {QTW{1'b0}};
            valid_out <= 1'b0;
        end else begin
            valid_out <= valid_in;
            // Saturate APP to Q̃ range
            if (app_wide > $signed({1'b0, SAT_MAX}))
                app_out <= SAT_MAX;
            else if (app_wide < $signed({1'b1, SAT_MIN}))
                app_out <= SAT_MIN;
            else
                app_out <= app_wide[QTW-1:0];
        end
    end

endmodule

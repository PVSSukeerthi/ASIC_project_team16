module min2_finder #(
    parameter DEG = 8,
    parameter W   = 4
)(
    input  wire signed [W-1:0] in [0:DEG-1],
    output reg  [W-1:0] min1,
    output reg  [W-1:0] min2,
    output reg  [$clog2(DEG)-1:0] idx1,
    output reg  [$clog2(DEG)-1:0] idx2
);

    integer i;
    reg [W-1:0] abs_val;

    always @(*) begin
        min1 = {W{1'b1}};
        min2 = {W{1'b1}};
        idx1 = 0;
        idx2 = 0;

        for (i = 0; i < DEG; i = i + 1) begin
            abs_val = in[i][W-1] ? -in[i] : in[i];

            if (abs_val < min1) begin
                min2 = min1;
                idx2 = idx1;
                min1 = abs_val;
                idx1 = i[$clog2(DEG)-1:0];
            end
            else if (abs_val < min2) begin
                min2 = abs_val;
                idx2 = i[$clog2(DEG)-1:0];
            end
        end
    end
endmodule

module iams_cnu #(
    parameter DEG = 8,
    parameter W   = 4,
    parameter DTH = 6
)(
    input  wire signed [W-1:0] beta_in [0:DEG-1],
    input  wire [5:0]          vn_degree [0:DEG-1],
    input  wire                is_core_check,
    output reg  signed [W-1:0] alpha_out [0:DEG-1]
);

    integer i;

    wire [W-1:0] min1, min2;
    wire [$clog2(DEG)-1:0] idx1, idx2;

    reg global_sign;
    reg [W-1:0] abs_beta;
    reg sign_i;

    // Min1 / Min2
    min2_finder #(
        .DEG(DEG),
        .W(W)
    ) u_min2 (
        .in(beta_in),
        .min1(min1),
        .min2(min2),
        .idx1(idx1),
        .idx2(idx2)
    );

    // Global sign
    always @(*) begin
        global_sign = 1'b0;
        for (i = 0; i < DEG; i = i + 1)
            global_sign = global_sign ^ beta_in[i][W-1];
    end

    // IAMS update
    always @(*) begin
        for (i = 0; i < DEG; i = i + 1) begin
            sign_i = global_sign ^ beta_in[i][W-1];

            // Column-degree adaptation (OMS)
            if (is_core_check && vn_degree[i] >= DTH) begin
                alpha_out[i] = sign_i ?
                    -((min1 > 0) ? (min1 - 1'b1) : 0) :
                     ((min1 > 0) ? (min1 - 1'b1) : 0);
            end
            else begin
                // IAMS logic
                if (i == idx1) begin
                    alpha_out[i] = sign_i ? -min2 : min2;
                end
                else if (i == idx2) begin
                    alpha_out[i] = sign_i ? -min1 : min1;
                end
                else begin
                    if (min1 > 0 && min1 == min2) begin
                        alpha_out[i] = sign_i ?
                            -((min1 > 0) ? (min1 - 1'b1) : 0) :
                             ((min1 > 0) ? (min1 - 1'b1) : 0);
                    end
                    else begin
                        alpha_out[i] = sign_i ? -min1 : min1;
                    end
                end
            end
        end
    end
endmodule
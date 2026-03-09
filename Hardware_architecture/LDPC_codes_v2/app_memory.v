`include "ldpc5g_pkg.v"

module app_memory #(
    parameter Z       = `Z,
    parameter NB      = `NB,
    parameter NB_CORE = 10,
    parameter NB_EXT  = 42,
    parameter QTILDE  = `QTILDE
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        init_en,
    input  wire [Z*NB*QTILDE-1:0]      init_data,
    input  wire [NB-1:0]               wr_en,
    input  wire [Z*NB*QTILDE-1:0]      wr_data,
    input  wire                        sel,
    input  wire [4*16-1:0]             rd_nio_sel,    // 16 x 4-bit selectors packed
    output wire [16*Z*QTILDE-1:0]      rd_out,
    output wire [Z*NB*QTILDE-1:0]      app_full
);

// ---- Core APP registers ----
// app_core: flat [Z*NB_CORE*QTILDE-1:0], element [z][c] at [(z*NB_CORE+c)*QTILDE +: QTILDE]
reg [Z*NB_CORE*QTILDE-1:0] app_core;
// ---- Extension APP registers ----
// app_ext_a: flat [Z*NB_EXT*QTILDE-1:0], element [z][c] at [(z*NB_EXT+c)*QTILDE +: QTILDE]
reg [Z*NB_EXT*QTILDE-1:0] app_ext_a;
// app_ext_b: same layout as app_ext_a
reg [Z*NB_EXT*QTILDE-1:0] app_ext_b;

integer z, c, shift_cc;
reg [QTILDE-1:0] shift_tmp;

// ---- Initialize / Write ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (z = 0; z < Z; z = z+1) begin
            for (c = 0; c < NB_CORE; c = c+1) app_core[(z*NB_CORE+c)*QTILDE +: QTILDE] <= 0;
            for (c = 0; c < NB_EXT;  c = c+1) begin
                app_ext_a[(z*NB_EXT+c)*QTILDE +: QTILDE] <= 0;
                app_ext_b[(z*NB_EXT+c)*QTILDE +: QTILDE] <= 0;
            end
        end
    end else if (init_en) begin
        for (z = 0; z < Z; z = z+1) begin
            for (c = 0; c < NB_CORE; c = c+1)
                app_core[(z*NB_CORE+c)*QTILDE +: QTILDE] <= init_data[(z*NB+c)*QTILDE +: QTILDE];
            for (c = 0; c < NB_EXT; c = c+1) begin
                app_ext_a[(z*NB_EXT+c)*QTILDE +: QTILDE] <= init_data[(z*NB+(c+NB_CORE))*QTILDE +: QTILDE];
                app_ext_b[(z*NB_EXT+c)*QTILDE +: QTILDE] <= init_data[(z*NB+(c+NB_CORE))*QTILDE +: QTILDE];
            end
        end
    end else begin
        for (z = 0; z < Z; z = z+1) begin
            for (c = 0; c < NB_CORE; c = c+1)
                if (wr_en[c])
                    app_core[(z*NB_CORE+c)*QTILDE +: QTILDE] <= wr_data[(z*NB+c)*QTILDE +: QTILDE];
            for (c = 0; c < NB_EXT; c = c+1)
                if (wr_en[c+NB_CORE]) begin
                    app_ext_a[(z*NB_EXT+c)*QTILDE +: QTILDE] <= wr_data[(z*NB+(c+NB_CORE))*QTILDE +: QTILDE];
                    app_ext_b[(z*NB_EXT+c)*QTILDE +: QTILDE] <= wr_data[(z*NB+(c+NB_CORE))*QTILDE +: QTILDE];
                end
        end
        // Selective shift
        if (sel) begin
            for (z = 0; z < Z; z = z+1) begin
                shift_tmp = app_ext_a[(z*NB_EXT+(NB_EXT-1))*QTILDE +: QTILDE];
                for (shift_cc = NB_EXT-1; shift_cc > 0; shift_cc = shift_cc-1)
                    app_ext_a[(z*NB_EXT+shift_cc)*QTILDE +: QTILDE] <= app_ext_a[(z*NB_EXT+(shift_cc-1))*QTILDE +: QTILDE];
                app_ext_a[(z*NB_EXT+0)*QTILDE +: QTILDE] <= shift_tmp;
            end
        end
    end
end

// ---- Read Network ----
genvar gi, gz;
generate
    for (gi = 0; gi < 16; gi = gi+1) begin : rn_out
        for (gz = 0; gz < Z; gz = gz+1) begin : rn_z
            wire [3:0] sel_i;
            assign sel_i = rd_nio_sel[gi*4 +: 4];
            assign rd_out[(gi*Z+gz)*QTILDE +: QTILDE] =
                (sel_i < NB_CORE) ? app_core[(gz*NB_CORE+sel_i)*QTILDE +: QTILDE]
                                  : app_ext_a[(gz*NB_EXT+(sel_i-NB_CORE))*QTILDE +: QTILDE];
        end
    end
endgenerate

// ---- Full APP output ----
generate
    for (gz = 0; gz < Z; gz = gz+1) begin : fa_z
        for (gi = 0; gi < NB; gi = gi+1) begin : fa_c
            assign app_full[(gz*NB+gi)*QTILDE +: QTILDE] =
                (gi < NB_CORE) ? app_core[(gz*NB_CORE+gi)*QTILDE +: QTILDE]
                               : app_ext_a[(gz*NB_EXT+(gi-NB_CORE))*QTILDE +: QTILDE];
        end
    end
endgenerate

endmodule

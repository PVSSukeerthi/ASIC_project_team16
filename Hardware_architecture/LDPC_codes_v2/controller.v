`include "ldpc5g_pkg.v"

module controller #(
    parameter L        = `L_AFTER,
    parameter ITMAX    = `ITMAX_IMPL,
    parameter MB       = `MB,
    parameter MWIDTH   = `Q - 1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        parity_ok,
    output reg  [4:0]  layer_no,
    output reg  [3:0]  iter_cnt,
    output reg         layer_merge,
    output reg         is_core,
    output reg  [3:0]  dc1_actual,
    output reg  [3:0]  dc2_actual,
    output reg  [5:0]  base_row1,
    output reg  [5:0]  base_row2,
    output reg         sel_w,
    output reg         sel_r,
    output reg         init_phase,
    output reg         dec_done,
    output reg         rst_n_mem,
    output wire [3:0]  D_thresh
);

assign D_thresh = `D_THRESHOLD;

reg [3:0] row_degree [0:41];
initial begin
    row_degree[0]=10; row_degree[1]=10; row_degree[2]=10; row_degree[3]=10;
    row_degree[4]=5;  row_degree[5]=5;  row_degree[6]=5;  row_degree[7]=5;
    row_degree[8]=4;  row_degree[9]=4;  row_degree[10]=4; row_degree[11]=4;
    row_degree[12]=4; row_degree[13]=4; row_degree[14]=3; row_degree[15]=3;
    row_degree[16]=3; row_degree[17]=3; row_degree[18]=3; row_degree[19]=3;
    row_degree[20]=3; row_degree[21]=3; row_degree[22]=3; row_degree[23]=3;
    row_degree[24]=3; row_degree[25]=3; row_degree[26]=3; row_degree[27]=3;
    row_degree[28]=3; row_degree[29]=3; row_degree[30]=3; row_degree[31]=3;
    row_degree[32]=3; row_degree[33]=3; row_degree[34]=3; row_degree[35]=3;
    row_degree[36]=3; row_degree[37]=3; row_degree[38]=3; row_degree[39]=3;
    row_degree[40]=3; row_degree[41]=3;
end

function automatic [5:0] layer_to_bgrow1;
    input [4:0] layer;
    begin
        layer_to_bgrow1 = (layer < 20) ? layer[5:0]
                                       : (6'd20 + 2*(layer - 5'd20));
    end
endfunction

function automatic [5:0] layer_to_bgrow2;
    input [4:0] layer;
    begin
        layer_to_bgrow2 = (layer < 20) ? layer[5:0]
                                       : (6'd21 + 2*(layer - 5'd20));
    end
endfunction

localparam S_IDLE  = 3'd0;
localparam S_INIT  = 3'd1;
localparam S_DECODE= 3'd2;
localparam S_CHECK = 3'd3;
localparam S_DONE  = 3'd4;

reg [2:0] state;

// Hoisted from decode_blk
reg [5:0] br1, br2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= S_IDLE;
        layer_no    <= 0; iter_cnt    <= 0;
        layer_merge <= 0; is_core     <= 0;
        dc1_actual  <= 0; dc2_actual  <= 0;
        base_row1   <= 0; base_row2   <= 0;
        sel_w       <= 0; sel_r       <= 0;
        init_phase  <= 0; dec_done    <= 0;
        rst_n_mem   <= 0;
    end else begin
        case (state)
            S_IDLE: begin
                dec_done <= 0;
                if (start) begin
                    state      <= S_INIT;
                    init_phase <= 1;
                    rst_n_mem  <= 1;
                end
            end

            S_INIT: begin
                init_phase <= 0;
                layer_no   <= 0;
                iter_cnt   <= 0;
                state      <= S_DECODE;
            end

            S_DECODE: begin
                br1 = layer_to_bgrow1(layer_no);
                br2 = layer_to_bgrow2(layer_no);
                base_row1  <= br1;
                is_core    <= (layer_no < 4);
                dc1_actual <= row_degree[br1];

                if (layer_no >= 20) begin
                    layer_merge <= 1;
                    base_row2   <= br2;
                    dc2_actual  <= row_degree[br2];
                end else begin
                    layer_merge <= 0;
                    base_row2   <= br1;
                    dc2_actual  <= 0;
                end

                sel_r <= (layer_no >= 4);
                sel_w <= (layer_no >= 4);

                if (layer_no == L - 1) begin
                    layer_no <= 0;
                    state    <= S_CHECK;
                end else begin
                    layer_no <= layer_no + 1;
                end
            end

            S_CHECK: begin
                if (parity_ok) begin
                    state    <= S_DONE;
                    dec_done <= 1;
                end else if (iter_cnt == ITMAX - 1) begin
                    state    <= S_DONE;
                    dec_done <= 1;
                end else begin
                    iter_cnt <= iter_cnt + 1;
                    state    <= S_DECODE;
                end
            end

            S_DONE: begin
                dec_done <= 1;
                if (!start) state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule

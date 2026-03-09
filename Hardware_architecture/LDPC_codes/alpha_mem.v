// =============================================================================
// alpha_mem.v  —  CN→VN Message Memory
// =============================================================================
// Stores the CN→VN messages (α) from the previous iteration.
// These are read at the start of each layer to compute the extrinsic
// VN→CN message β = APP − α_prev, and written back after the CNU
// produces α_new.
//
// Organisation: addressed by (layer, edge_index).
//   Total entries = SUM_{m=0}^{M-1} d_c(m)
//   For BG2 (M=2184 CNs, avg d_c ≈ 5): ≈ 10920 entries
//
// Parameters:
//   DEPTH — total number of edges  (≈ num_CNs × avg_dc)
//   QW    — message bit-width
// =============================================================================

`timescale 1ns / 1ps

module alpha_mem #(
    parameter DEPTH  = 10920,       // Enough for BG2
    parameter QW     = 7,
    parameter ADDRW  = $clog2(DEPTH)
) (
    input  wire              clk,
    // Write port
    input  wire              wr_en,
    input  wire [ADDRW-1:0]  wr_addr,
    input  wire [QW-1:0]     wr_data,
    // Read port
    input  wire [ADDRW-1:0]  rd_addr,
    output reg  [QW-1:0]     rd_data
);

    reg [QW-1:0] mem [0:DEPTH-1];

    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1)
            mem[i] = {QW{1'b0}};
    end

    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    always @(posedge clk) begin
        rd_data <= mem[rd_addr];
    end

endmodule

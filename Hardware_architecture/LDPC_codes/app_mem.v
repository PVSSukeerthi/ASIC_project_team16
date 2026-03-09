// =============================================================================
// app_mem.v  —  APP (A Posteriori Probability) LLR Memory
// =============================================================================
// Single-port SRAM model for storing per-variable-node APP LLRs.
// In a real implementation this maps to FPGA BRAM or SRAM cells.
//
// One APP memory bank holds the APP values for one base-graph VN column,
// replicated Z times (one per lifting index).  For simulation purposes we
// model it as a depth-N, width-QTW single-port RAM.
//
// Parameters:
//   DEPTH — number of VNs  (= N = N_b * Z, paper BG2: 2704)
//   QTW   — APP bit-width  (paper: Q̃=31 needs 6 bits signed, we use 8 for margin)
// =============================================================================

`timescale 1ns / 1ps

module app_mem #(
    parameter DEPTH = 2704,        // BG2: N = 52 * 52 = 2704
    parameter QTW   = 8,           // APP word width (signed)
    parameter ADDRW = $clog2(DEPTH)
) (
    input  wire              clk,
    // Write port
    input  wire              wr_en,
    input  wire [ADDRW-1:0]  wr_addr,
    input  wire [QTW-1:0]    wr_data,
    // Read port
    input  wire [ADDRW-1:0]  rd_addr,
    output reg  [QTW-1:0]    rd_data
);

    // Synchronous read/write SRAM
    reg [QTW-1:0] mem [0:DEPTH-1];

    // Initialise to zero (represents zero LLR = equal probability)
    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1)
            mem[i] = {QTW{1'b0}};
    end

    // Write
    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    // Read (registered — 1 cycle read latency)
    always @(posedge clk) begin
        rd_data <= mem[rd_addr];
    end

endmodule

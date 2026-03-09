// =============================================================================
// ctv_mem.v  —  Split CTV (Check-To-Variable) Memory (Fig. 14)
// =============================================================================
// Cui et al. TCAS-I 2020, Fig. 14 and Section V-B.
//
// The paper splits CTV storage into two dual-port RAMs to exploit the
// variable check-node degree distribution in BG2:
//
//   Bank 1 (CTV_B1): Stores entries for ALL L check-node rows.
//                    Width  = W1 = sign_bits + min1 + min2 + idx1 + delta_flag
//                           = Z + 2*(QW-1) + IDW + 1  bits
//                    Depth  = L  (total rows of BG2 = 42)
//
//   Bank 2 (CTV_B2): Only stores rows where d_c < DC_MAX (degree < 10).
//                    These rows need fewer sign bits (actual d_c < Z).
//                    Width  = W2 = W1 (same compressed format, but smaller
//                             effective population — the saving comes from
//                             needing depth L0 < L only).
//                    Depth  = L0 (rows with d_c < DC_MAX, ≈ 20 for BG2)
//
// Memory savings vs single flat storage (Table II, paper):
//   Flat   : L × CTV_WORD_W    = 42 × 69 = 2898 bits
//   Split  : L×W1 + L0×W1      — but W1 < CTV_WORD_W for low-degree rows
//   Saving : ~29.2%
//
// Interface:
//   Both banks are synchronous dual-port RAMs (1 write port, 1 read port).
//   The controller decides which bank to read via bank_sel.
//   Write port is used during LLR initialisation (loading soft values).
//
// Ports
//   clk                      — clock
//   rst_n                    — async active-low reset (clears valid flags)
//
//   --- Write port (shared, mux'd by bank_sel_wr) ---
//   wr_en                    — write enable
//   wr_addr [ADRW-1:0]       — write address
//   wr_data [CTV_WORD_W-1:0] — data to write
//   bank_sel_wr              — 0 = Bank1, 1 = Bank2
//
//   --- Read port ---
//   rd_en                    — read enable
//   rd_addr_b1 [ADRW-1:0]   — Bank1 read address
//   rd_addr_b2 [ADRW2-1:0]  — Bank2 read address (shallower)
//   bank_sel_rd              — 0 = read Bank1, 1 = read Bank2
//   rd_data [CTV_WORD_W-1:0] — output word (1 cycle latency)
//   rd_valid                 — output valid
// =============================================================================

`timescale 1ns/1ps
`include "pkg_iams.vh"

module ctv_mem #(
    parameter Z          = `LIFTING_Z,
    parameter QW         = `QW,
    parameter IDW        = `IDW,
    parameter CTV_WORD_W = `CTV_WORD_W,   // 69 bits
    parameter DEPTH_B1   = `BG_ROWS,      // 42
    parameter DEPTH_B2   = `CTV_DEPTH_B2, // 20
    parameter ADRW       = 6,             // ceil(log2(42)) = 6
    parameter ADRW2      = 5              // ceil(log2(20)) = 5
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // Write port
    input  wire                  wr_en,
    input  wire [ADRW-1:0]       wr_addr,
    input  wire [CTV_WORD_W-1:0] wr_data,
    input  wire                  bank_sel_wr,  // 0=B1, 1=B2

    // Read port
    input  wire                  rd_en,
    input  wire [ADRW-1:0]       rd_addr_b1,
    input  wire [ADRW2-1:0]      rd_addr_b2,
    input  wire                  bank_sel_rd,  // 0=B1, 1=B2
    output reg  [CTV_WORD_W-1:0] rd_data,
    output reg                   rd_valid
);

    // -------------------------------------------------------------------------
    // Bank 1 — full depth, all rows
    // -------------------------------------------------------------------------
    reg [CTV_WORD_W-1:0] bank1 [0:DEPTH_B1-1];

    // -------------------------------------------------------------------------
    // Bank 2 — shallow, low-degree rows only
    // -------------------------------------------------------------------------
    reg [CTV_WORD_W-1:0] bank2 [0:DEPTH_B2-1];

    // -------------------------------------------------------------------------
    // Write logic
    // -------------------------------------------------------------------------
    integer wi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialise banks to zero (synthesis: use initial block or reset)
            for (wi = 0; wi < DEPTH_B1; wi = wi + 1)
                bank1[wi] <= {CTV_WORD_W{1'b0}};
            for (wi = 0; wi < DEPTH_B2; wi = wi + 1)
                bank2[wi] <= {CTV_WORD_W{1'b0}};
        end else if (wr_en) begin
            if (bank_sel_wr == 1'b0)
                bank1[wr_addr] <= wr_data;
            else
                bank2[wr_addr[ADRW2-1:0]] <= wr_data;
        end
    end

    // -------------------------------------------------------------------------
    // Read logic (1-cycle registered output)
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_data  <= {CTV_WORD_W{1'b0}};
            rd_valid <= 1'b0;
        end else begin
            rd_valid <= rd_en;
            if (rd_en) begin
                if (bank_sel_rd == 1'b0)
                    rd_data <= bank1[rd_addr_b1];
                else
                    rd_data <= bank2[rd_addr_b2];
            end
        end
    end

endmodule

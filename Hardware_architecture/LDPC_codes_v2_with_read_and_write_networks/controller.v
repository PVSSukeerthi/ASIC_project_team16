// =============================================================================
// controller.v  —  IAMS Decoder Controller (Fig. 11)
// =============================================================================
// Cui et al. TCAS-I 2020, Fig. 11 and Section V.
//
// Implements the layered decoding schedule for BG2:
//   - Iterates IT_MAX times over all BG_ROWS check-node rows (layers)
//   - For each layer: READ → CNU → VNU → WRITE → next layer
//   - Issues sel_r (read LBS shift) and sel_w (write LBS shift) to the
//     Read/Write Networks from the H-matrix ROM
//   - Controls CTV memory bank selection (high-degree vs low-degree rows)
//   - Asserts `done` when IT_MAX iterations complete and syndrome passes
//
// Pipeline stages per layer:
//   Cycle 0: issue H-ROM read request for layer_no
//   Cycle 1: H-ROM valid; issue APP memory read with vn_cols+shifts
//   Cycle 2: APP read data valid; CNU starts (β computed in VNU)
//   Cycle 3: CNU valid (1-cycle CNU pipeline); VNU APP update
//   Cycle 4: VNU write-back to APP memory; CTV memory update
//   → total latency per layer = 5 cycles (overlapped with next layer)
//
// For simplicity in this implementation, layers are processed sequentially
// (non-pipelined). The overlapped version is described in comments.
//
// Ports
//   clk, rst_n
//   start             — pulse to begin decoding
//   syndrome_ok       — from syndrome_check (all parities zero)
//   layer_no [LAY_W-1:0] — current layer being processed
//   iter_no  [IT_W-1:0]  — current iteration (0-based)
//   hrom_rd_addr [ROW_AW-1:0] — H-matrix ROM read address
//   app_rd_en          — APP memory read enable
//   app_wr_en          — APP memory write enable
//   ctv_rd_en          — CTV memory read enable
//   ctv_wr_en          — CTV memory write enable
//   ctv_bank_sel       — 0=Bank1 (all rows), 1=Bank2 (low-degree rows)
//   cnu_valid_in       — trigger CNU processing
//   vnu_upd_en         — trigger VNU APP update
//   wn_wr_en           — trigger write network commit
//   init_phase         — asserted during channel LLR loading
//   done               — decoding complete (sticky until next start)
// =============================================================================

`timescale 1ns/1ps
`include "pkg_iams.vh"

module controller #(
    parameter BG_ROWS  = `BG_ROWS,   // 42
    parameter IT_MAX   = `IT_MAX,    // 10
    parameter LAY_W    = 6,          // ceil(log2(BG_ROWS))
    parameter IT_W     = 4,          // ceil(log2(IT_MAX))
    parameter ROW_AW   = 6
) (
    input  wire            clk,
    input  wire            rst_n,
    input  wire            start,
    input  wire            syndrome_ok,

    output reg  [LAY_W-1:0] layer_no,
    output reg  [IT_W-1:0]  iter_no,

    // H-matrix ROM
    output reg  [ROW_AW-1:0] hrom_rd_addr,

    // Memory enables
    output reg   app_rd_en,
    output reg   app_wr_en,
    output reg   ctv_rd_en,
    output reg   ctv_wr_en,
    output reg   ctv_bank_sel,   // which CTV bank to access

    // Datapath enables
    output reg   cnu_valid_in,
    output reg   vnu_upd_en,
    output reg   wn_wr_en,

    // Status
    output reg   init_phase,     // channel loading phase
    output reg   done            // sticky completion flag
);

    // -------------------------------------------------------------------------
    // FSM states
    // -------------------------------------------------------------------------
    localparam [3:0]
        S_IDLE       = 4'd0,
        S_INIT       = 4'd1,   // load channel LLRs (external; controller waits)
        S_HROM_RD    = 4'd2,   // issue H-ROM read
        S_WAIT_HROM  = 4'd3,   // wait 1 cycle for H-ROM latency
        S_APP_RD     = 4'd4,   // issue APP memory read
        S_WAIT_APP   = 4'd5,   // wait for APP read + CNU β computation
        S_CNU        = 4'd6,   // CNU processes (1 cycle registered)
        S_WAIT_CNU   = 4'd7,   // wait CNU pipeline
        S_VNU_WR     = 4'd8,   // VNU APP update + write network
        S_CTV_WR     = 4'd9,   // update CTV memory with new α
        S_NEXT_LAYER = 4'd10,  // increment layer or iteration
        S_SYNC_CHECK = 4'd11,  // check syndrome
        S_DONE       = 4'd12;

    reg [3:0] state, next_state;

    // Layer/iteration counters
    reg [LAY_W-1:0] layer_cnt;
    reg [IT_W-1:0]  iter_cnt;

    // Pipeline delay counter
    reg [2:0] pipe_cnt;

    // -------------------------------------------------------------------------
    // Low-degree threshold: rows with degree < DC_MAX use CTV Bank2
    // For BG2 the first 4 rows are degree=10, rest have lower degree
    // (simplified rule: layer_cnt >= 4 → Bank2)
    // -------------------------------------------------------------------------
    wire is_low_degree;
    assign is_low_degree = (layer_cnt >= 4);

    // -------------------------------------------------------------------------
    // FSM — sequential
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            layer_cnt  <= {LAY_W{1'b0}};
            iter_cnt   <= {IT_W{1'b0}};
            pipe_cnt   <= 3'd0;
        end else begin
            state <= next_state;
            case (state)
                S_NEXT_LAYER: begin
                    if (layer_cnt == BG_ROWS - 1) begin
                        layer_cnt <= {LAY_W{1'b0}};
                        iter_cnt  <= iter_cnt + 1'b1;
                    end else begin
                        layer_cnt <= layer_cnt + 1'b1;
                    end
                end
                S_IDLE: begin
                    layer_cnt <= {LAY_W{1'b0}};
                    iter_cnt  <= {IT_W{1'b0}};
                end
                default: ;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // FSM — combinational next-state
    // -------------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:       if (start)     next_state = S_HROM_RD;
            S_HROM_RD:                   next_state = S_WAIT_HROM;
            S_WAIT_HROM:                 next_state = S_APP_RD;
            S_APP_RD:                    next_state = S_WAIT_APP;
            S_WAIT_APP:                  next_state = S_CNU;
            S_CNU:                       next_state = S_WAIT_CNU;
            S_WAIT_CNU:                  next_state = S_VNU_WR;
            S_VNU_WR:                    next_state = S_CTV_WR;
            S_CTV_WR:                    next_state = S_NEXT_LAYER;
            S_NEXT_LAYER: begin
                if (layer_cnt == BG_ROWS-1) begin
                    // End of iteration
                    if ((iter_cnt == IT_MAX - 1) || syndrome_ok)
                        next_state = S_SYNC_CHECK;
                    else
                        next_state = S_HROM_RD;
                end else
                    next_state = S_HROM_RD;
            end
            S_SYNC_CHECK:                next_state = S_DONE;
            S_DONE:       if (start)     next_state = S_HROM_RD;  // restart
            default:                     next_state = S_IDLE;
        endcase
    end

    // -------------------------------------------------------------------------
    // Output decode
    // -------------------------------------------------------------------------
    always @(*) begin
        // Defaults
        hrom_rd_addr = layer_cnt;
        layer_no     = layer_cnt;
        iter_no      = iter_cnt;
        app_rd_en    = 1'b0;
        app_wr_en    = 1'b0;
        ctv_rd_en    = 1'b0;
        ctv_wr_en    = 1'b0;
        ctv_bank_sel = is_low_degree;
        cnu_valid_in = 1'b0;
        vnu_upd_en   = 1'b0;
        wn_wr_en     = 1'b0;
        init_phase   = 1'b0;
        done         = (state == S_DONE);

        case (state)
            S_APP_RD:  app_rd_en    = 1'b1;
            S_WAIT_APP:begin
                       ctv_rd_en   = 1'b1;
                       cnu_valid_in= 1'b1;
                       end
            S_CNU:     cnu_valid_in= 1'b1;
            S_VNU_WR:  begin
                       vnu_upd_en  = 1'b1;
                       app_wr_en   = 1'b1;
                       end
            S_CTV_WR:  begin
                       ctv_wr_en   = 1'b1;
                       wn_wr_en    = 1'b1;
                       end
            default: ;
        endcase
    end

endmodule

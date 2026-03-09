// =============================================================================
// layer_ctrl.v  —  Layered Decoder Controller  (v2 — sticky done)
// =============================================================================
// Controls the iterative layered decoding schedule for the IAMS decoder.
//
// FIX v2: done is now STICKY.
//   - done asserts when decoding finishes (max-iter OR syndrome_ok).
//   - done stays high until the NEXT start pulse clears it.
//   - This is the correct hardware contract: the host/TB can sample done
//     at any point after it asserts without needing to catch a 1-cycle pulse.
//
// The original v1 cleared done in IDLE every cycle, so it was high for
// exactly ONE clock cycle before the FSM returned to IDLE and cleared it —
// causing the testbench to miss it between 13-cycle drive_one_iteration bursts.
//
// Parameters:
//   NUM_LAYERS — number of check-node layers
//   IT_MAX     — maximum decoding iterations
//   IT_W       — bit-width of iteration counter (must hold IT_MAX)
// =============================================================================

`timescale 1ns / 1ps

module layer_ctrl #(
    parameter NUM_LAYERS = 42,
    parameter IT_MAX     = 15,
    parameter IT_W       = 7
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              start,
    input  wire              syndrome_ok,

    output reg  [$clog2(NUM_LAYERS)-1:0] layer_idx,
    output reg  [IT_W-1:0]               iter_idx,
    output reg                           proc_valid,
    output reg                           done,
    output reg                           first_iter
);

    localparam LAYER_W = $clog2(NUM_LAYERS);

    localparam IDLE = 1'b0;
    localparam RUN  = 1'b1;

    reg state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            layer_idx  <= {LAYER_W{1'b0}};
            iter_idx   <= {IT_W{1'b0}};
            proc_valid <= 1'b0;
            done       <= 1'b0;
            first_iter <= 1'b0;
        end else begin
            case (state)

                // ---- IDLE -------------------------------------------------------
                // done is NOT touched here unless start fires.
                // This keeps done sticky between blocks.
                IDLE: begin
                    proc_valid <= 1'b0;
                    first_iter <= 1'b0;
                    if (start) begin
                        done       <= 1'b0;                 // clear for new block
                        layer_idx  <= {LAYER_W{1'b0}};
                        iter_idx   <= {IT_W{1'b0}};
                        first_iter <= 1'b1;
                        proc_valid <= 1'b1;
                        state      <= RUN;
                    end
                end

                // ---- RUN --------------------------------------------------------
                RUN: begin
                    first_iter <= 1'b0;
                    proc_valid <= 1'b1;

                    if (syndrome_ok) begin
                        // Early termination: syndrome passes
                        done       <= 1'b1;
                        proc_valid <= 1'b0;
                        state      <= IDLE;

                    end else if (layer_idx == (NUM_LAYERS - 1)) begin
                        // End of one full iteration sweep
                        layer_idx <= {LAYER_W{1'b0}};
                        if (iter_idx == (IT_MAX - 1)) begin
                            // Max iterations reached
                            done       <= 1'b1;
                            proc_valid <= 1'b0;
                            state      <= IDLE;
                        end else begin
                            iter_idx <= iter_idx + 1'b1;
                        end

                    end else begin
                        layer_idx <= layer_idx + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
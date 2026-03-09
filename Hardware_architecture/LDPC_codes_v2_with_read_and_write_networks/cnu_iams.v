// =============================================================================
// cnu_iams.v  —  IAMS Check Node Unit (CNU)
// =============================================================================
// Implements the IAMS CN update from Cui et al., IEEE TCAS-I 2020, Section V.
//
// Equation (10) — four-case IAMS magnitude formula:
//   Let min1, min2 = two smallest incoming magnitudes
//       idx1, idx2 = their positions
//       Δ = min2 − min1  (unsigned)
//
//   For output edge n:
//     n == idx1            : α = τ * min2
//     n == idx2            : α = τ * min1
//     n ∈ Ī(m), Δ == 0    : α = τ * max(min1 − 1, 0)
//     n ∈ Ī(m), Δ != 0    : α = τ * min1
//
//   Sign:  ⊕ of all input signs, then toggle for outgoing edge n.
//
// Fixed-point params (paper Table I):
//   Input message width  : Q_IN  = 7 bits (signed, range −31..+31 with q=4,q̃=6)
//   Scaling factor τ     : implemented as right-shift (τ = 1 gives MS baseline)
//
// Parameterisation:
//   DC   — check-node degree (number of connected variable nodes)
//   QW   — total message bit-width (includes sign bit)
//   IDW  — index bit-width = ceil(log2(DC))
//
// Ports:
//   msg_in[DC-1:0][QW-1:0]  — incoming VN→CN messages (signed 2's complement)
//   msg_out[DC-1:0][QW-1:0] — outgoing CN→VN messages (signed 2's complement)
// =============================================================================

`timescale 1ns / 1ps

module cnu_iams #(
    parameter DC   = 10,          // Check-node degree (max in BG2 = 10)
    parameter QW   = 7,           // Message bit-width (paper: Q̃=31, 6 bits + sign)
    parameter IDW  = 4            // Index width: ceil(log2(DC))
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  valid_in,   // Input messages valid this cycle
    // Flattened 2-D input: DC messages each QW bits (signed)
    input  wire [DC*QW-1 : 0]   msg_flat_in,
    output reg  [DC*QW-1 : 0]   msg_flat_out,
    output reg                   valid_out
);

    // -------------------------------------------------------------------------
    // Unpack flat input into array
    // -------------------------------------------------------------------------
    wire signed [QW-1:0] msg_in [0:DC-1];
    genvar gi;
    generate
        for (gi = 0; gi < DC; gi = gi + 1) begin : unpack
            assign msg_in[gi] = msg_flat_in[gi*QW +: QW];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Internal signals
    // -------------------------------------------------------------------------
    // Magnitudes (unsigned, QW-1 bits)
    wire [QW-2:0] mag [0:DC-1];
    // Signs (1 = negative)
    wire          sgn [0:DC-1];

    generate
        for (gi = 0; gi < DC; gi = gi + 1) begin : mag_sign
            assign sgn[gi] = msg_in[gi][QW-1];
            // magnitude = absolute value
            assign mag[gi] = sgn[gi] ? (~msg_in[gi][QW-2:0] + 1'b1)
                                      : msg_in[gi][QW-2:0];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Cumulative XOR sign (parity of all signs)
    // -------------------------------------------------------------------------
    reg sign_total;
    integer k;
    always @(*) begin
        sign_total = 1'b0;
        for (k = 0; k < DC; k = k + 1)
            sign_total = sign_total ^ sgn[k];
    end

    // -------------------------------------------------------------------------
    // Find min1, min2, idx1, idx2
    // min1 = smallest magnitude, min2 = second smallest
    // -------------------------------------------------------------------------
    reg [QW-2:0] min1, min2;
    reg [IDW-1:0] idx1, idx2;

    integer i;
    always @(*) begin
        min1 = {(QW-1){1'b1}};   // initialise to max
        min2 = {(QW-1){1'b1}};
        idx1 = {IDW{1'b0}};
        idx2 = {IDW{1'b0}};

        for (i = 0; i < DC; i = i + 1) begin
            if (mag[i] <= min1) begin
                // New minimum: push old min1 to min2
                min2 = min1;
                idx2 = idx1;
                min1 = mag[i];
                idx1 = i[IDW-1:0];
            end else if (mag[i] < min2) begin
                min2 = mag[i];
                idx2 = i[IDW-1:0];
            end
        end
    end

    // -------------------------------------------------------------------------
    // Compute Δ = min2 − min1  (unsigned difference, saturated to 0)
    // -------------------------------------------------------------------------
    wire [QW-2:0] delta;
    assign delta = (min2 >= min1) ? (min2 - min1) : {(QW-1){1'b0}};

    // -------------------------------------------------------------------------
    // IAMS CN update — combinational
    // -------------------------------------------------------------------------
    // Output messages registered on clock edge
    reg signed [QW-1:0] msg_out_comb [0:DC-1];

    integer n;
    reg [QW-2:0] alpha_mag;   // unsigned magnitude of output
    reg          out_sign;

    always @(*) begin
        for (n = 0; n < DC; n = n + 1) begin
            // Sign = total parity XOR own sign (= parity of all OTHER signs)
            out_sign = sign_total ^ sgn[n];

            // ---- IAMS magnitude selection (eq. 10) --------------------------
            if (n[IDW-1:0] == idx1) begin
                // This edge is the minimum index → use min2
                alpha_mag = min2;
            end else if (n[IDW-1:0] == idx2) begin
                // This edge is the second-minimum index → use min1
                alpha_mag = min1;
            end else begin
                // All other edges (Ī(m) set)
                if (delta == {(QW-1){1'b0}}) begin
                    // Δ = 0 → use max(min1 − 1, 0)
                    alpha_mag = (min1 > ({{(QW-2){1'b0}}, 1'b1}))? (min1 - 1'b1): ({(QW-1){1'b0}});
                end else begin
                    // Δ ≠ 0 → use min1
                    alpha_mag = min1;
                end
            end

            // ---- Saturate magnitude to max representable ----
            // (mag is QW-1 bits, message is QW bits signed → max mag = 2^(QW-1)-1)
            // Already fits since alpha_mag is QW-1 bits.

            // ---- Reconstruct signed output ----
            if (alpha_mag == {(QW-1){1'b0}}) begin
                msg_out_comb[n] = {QW{1'b0}};         // zero
            end else if (out_sign) begin
                // negative: 2's complement
                msg_out_comb[n] = {1'b1, (~alpha_mag + 1'b1)};
            end else begin
                msg_out_comb[n] = {1'b0, alpha_mag};  // positive
            end
        end
    end

    // -------------------------------------------------------------------------
    // Pipeline register (1 cycle latency for timing closure)
    // -------------------------------------------------------------------------
    integer p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            msg_flat_out <= {DC*QW{1'b0}};
            valid_out    <= 1'b0;
        end else begin
            valid_out <= valid_in;
            for (p = 0; p < DC; p = p + 1)
                msg_flat_out[p*QW +: QW] <= msg_out_comb[p];
        end
    end

endmodule

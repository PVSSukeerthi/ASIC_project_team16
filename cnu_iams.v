// =============================================================================
// cnu_iams.v  —  IAMS Check Node Unit (CNU)
// =============================================================================
// Cui et al., IEEE TCAS-I 2020, Section V, Equation (10).
//
// Equation (10) — four-case IAMS CN-update:
//   min1, min2 = two smallest input VTC magnitudes
//   idx1, idx2 = their edge indices
//   Δ = min2 − min1
//
//   For output edge n:
//     n == idx1         : α_mag = min2
//     n == idx2         : α_mag = min1
//     n ∈ Ī(m), Δ == 0 : α_mag = max(min1 − 1, 0)
//     n ∈ Ī(m), Δ != 0 : α_mag = min1
//   Sign = XOR of all OTHER input signs
//
// Ports (original):
//   msg_flat_in  [DC*QW-1:0] — DC VTC messages, signed 2's complement, LSB-first
//   msg_flat_out [DC*QW-1:0] — DC CTV messages, same packing
//   valid_in / valid_out     — 1-cycle pipeline handshake
//
// Ports (added for ctv_mem — avoids compressor inference):
//   min1_out   [QW-2:0]   — registered min1 magnitude (= MAG_W bits)
//   min2_out   [QW-2:0]   — registered min2 magnitude
//   idx1_out   [IDW-1:0]  — registered index of min1
//   idx2_out   [IDW-1:0]  — registered index of min2
//   sgn_out    [DC-1:0]   — registered input sign bits, one per edge
//
// All six outputs are registered on the same clock edge so they are
// always mutually consistent with msg_flat_out and valid_out.
//
// Pipeline: 1 register stage.
// Synthesisable Verilog-2001.
// =============================================================================

`timescale 1ns / 1ps

module cnu_iams #(
    parameter DC   = 10,   // Check-node degree
    parameter QW   = 7,    // Message bit-width (sign + magnitude)
    parameter IDW  = 4     // Index width: ceil(log2(DC))
) (
    input  wire                clk,
    input  wire                rst_n,
    input  wire                valid_in,

    input  wire [DC*QW-1:0]    msg_flat_in,
    output reg  [DC*QW-1:0]    msg_flat_out,
    output reg                 valid_out,

    // ── Compressed fields (registered, for ctv_mem direct connection) ────────
    // Using [QW-2:0] instead of [MAG_W-1:0] because localparams cannot
    // appear in port declarations in Verilog-2001.
    output reg  [QW-2:0]       min1_out,   // MAG_W = QW-1 bits
    output reg  [QW-2:0]       min2_out,
    output reg  [IDW-1:0]      idx1_out,
    output reg  [IDW-1:0]      idx2_out,
    output reg  [DC-1:0]       sgn_out     // sgn_out[i] = sign of input edge i
);

    // -------------------------------------------------------------------------
    // Local parameters
    // -------------------------------------------------------------------------
    localparam MAG_W   = QW - 1;
    localparam MAG_MAX = (1 << MAG_W) - 1;

    // -------------------------------------------------------------------------
    // Unpack input
    // -------------------------------------------------------------------------
    wire signed [QW-1:0] msg_in [0:DC-1];

    genvar gi;
    generate
        for (gi = 0; gi < DC; gi = gi + 1) begin : unpack
            assign msg_in[gi] = msg_flat_in[gi*QW +: QW];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Signs and magnitudes
    // -------------------------------------------------------------------------
    wire [MAG_W-1:0] mag [0:DC-1];
    wire             sgn [0:DC-1];

    generate
        for (gi = 0; gi < DC; gi = gi + 1) begin : mag_sign_calc
            wire [QW-1:0] abs_full;
            assign sgn[gi]  = msg_in[gi][QW-1];
            assign abs_full = sgn[gi] ? (~msg_in[gi] + 1'b1) : msg_in[gi];
            // Saturate MIN_NEG: if negation overflowed, abs_full[QW-1] is set
            assign mag[gi]  = abs_full[QW-1] ? MAG_MAX[MAG_W-1:0]
                                             : abs_full[MAG_W-1:0];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Pack sign vector (wire array → flat reg, needed for port and pipeline FF)
    // -------------------------------------------------------------------------
    reg [DC-1:0] sgn_packed;
    integer sp;
    always @(*) begin
        for (sp = 0; sp < DC; sp = sp + 1)
            sgn_packed[sp] = sgn[sp];
    end

    // -------------------------------------------------------------------------
    // XOR sign parity
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
    // Tie-breaking: <= displaces, so idx1 = LAST occurrence of global min.
    // -------------------------------------------------------------------------
    reg [MAG_W-1:0] min1, min2;
    reg [IDW-1:0]   idx1, idx2;

    integer i;
    always @(*) begin
        min1 = {MAG_W{1'b1}};
        min2 = {MAG_W{1'b1}};
        idx1 = {IDW{1'b0}};
        idx2 = {IDW{1'b0}};

        for (i = 0; i < DC; i = i + 1) begin
            if (mag[i] <= min1) begin
                min2 = min1; idx2 = idx1;
                min1 = mag[i]; idx1 = i[IDW-1:0];
            end else if (mag[i] < min2) begin
                min2 = mag[i]; idx2 = i[IDW-1:0];
            end
        end
    end

    // -------------------------------------------------------------------------
    // Delta
    // -------------------------------------------------------------------------
    wire [MAG_W-1:0] delta;
    assign delta = min2 - min1;

    // -------------------------------------------------------------------------
    // IAMS Eq.(10) combinational
    // -------------------------------------------------------------------------
    reg signed [QW-1:0] msg_out_comb [0:DC-1];

    integer         n;
    reg [MAG_W-1:0] alpha_mag;
    reg             out_sign;
    reg [IDW-1:0]   n_idx;

    always @(*) begin
        for (n = 0; n < DC; n = n + 1) begin
            out_sign = sign_total ^ sgn[n];
            n_idx    = n[IDW-1:0];

            if (n_idx == idx1) begin
                alpha_mag = min2;
            end else if (n_idx == idx2) begin
                alpha_mag = min1;
            end else begin
                if (delta == {MAG_W{1'b0}})
                    alpha_mag = (min1 > {{(MAG_W-1){1'b0}}, 1'b1})
                                ? (min1 - 1'b1) : {MAG_W{1'b0}};
                else
                    alpha_mag = min1;
            end

            if (alpha_mag == {MAG_W{1'b0}})
                msg_out_comb[n] = {QW{1'b0}};
            else if (out_sign)
                msg_out_comb[n] = ~({1'b0, alpha_mag}) + {{(QW-1){1'b0}}, 1'b1};
            else
                msg_out_comb[n] = {1'b0, alpha_mag};
        end
    end

    // -------------------------------------------------------------------------
    // Pipeline register — 1 cycle latency
    // All outputs (data + compressed fields) registered on same edge.
    // -------------------------------------------------------------------------
    integer p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            msg_flat_out <= {DC*QW{1'b0}};
            valid_out    <= 1'b0;
            min1_out     <= {(QW-1){1'b0}};
            min2_out     <= {(QW-1){1'b0}};
            idx1_out     <= {IDW{1'b0}};
            idx2_out     <= {IDW{1'b0}};
            sgn_out      <= {DC{1'b0}};
        end else begin
            valid_out <= valid_in;
            min1_out  <= min1;
            min2_out  <= min2;
            idx1_out  <= idx1;
            idx2_out  <= idx2;
            sgn_out   <= sgn_packed;
            for (p = 0; p < DC; p = p + 1)
                msg_flat_out[p*QW +: QW] <= msg_out_comb[p];
        end
    end

endmodule

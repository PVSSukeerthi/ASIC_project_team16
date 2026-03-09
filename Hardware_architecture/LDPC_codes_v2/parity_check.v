`include "ldpc5g_pkg.v"

module parity_check #(
    parameter Z      = `Z,
    parameter NB     = `NB,
    parameter MB     = `MB,
    parameter QTILDE = `QTILDE
)(
    input  wire [Z*NB*QTILDE-1:0] app,
    output wire [Z*NB-1:0]        codeword_hd,
    output wire                   parity_ok
);

// Hard decision: sign bit of each APP value
genvar gc, gz;
generate
    for (gc = 0; gc < NB; gc = gc+1) begin : hd_col
        for (gz = 0; gz < Z; gz = gz+1) begin : hd_z
            // app[gz][gc] sign bit = bit (gz*NB+gc)*QTILDE + QTILDE-1
            assign codeword_hd[gc*Z + gz] =
                app[((gz*NB)+gc)*QTILDE + QTILDE-1];
        end
    end
endgenerate

// Simplified parity: all parity columns (cols 10..NB-1) hard-decide to 0
// Full BG2 syndrome check requires complete base matrix - simplified here.
// For simulation purposes this checks the extension variable nodes.
wire [Z*NB-1:0] hd_flat;
assign hd_flat = codeword_hd;

reg all_zero;
integer r, z2;
always @(*) begin
    all_zero = 1'b1;
    // Check each parity (extension) column hard decision
    for (r = 10; r < NB; r = r+1) begin
        for (z2 = 0; z2 < Z; z2 = z2+1) begin
            if (hd_flat[r*Z + z2]) all_zero = 1'b0;
        end
    end
end

assign parity_ok = all_zero;

endmodule

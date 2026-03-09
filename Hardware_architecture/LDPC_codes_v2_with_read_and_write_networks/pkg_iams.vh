// =============================================================================
// pkg_iams.vh  —  Global parameters for IAMS LDPC decoder
// =============================================================================
// Based on: Cui et al., "An Improved Approximate Min-Sum LDPC Decoder
//           for 5G NR," IEEE TCAS-I, 2020.
//
// Target code: 5G NR Base Graph 2 (BG2), lifting factor Z=52
//
// Table I  — Fixed-point word lengths
//   q  = 4   (integer bits, VN→CN messages, excluding sign)
//   q̃  = 6   (integer bits, CN→VN / APP LLR, excluding sign)
//   Q  = 5   (total bits, VN→CN)        — sign + 4 magnitude bits
//   Q̃  = 7   (total bits, CN→VN / APP)  — sign + 6 magnitude bits
//   QT = 8   (APP LLR stored in memory, 1 extra guard bit)
//
// Table II  — Memory sizes (BG2, Z=52)
//   APP memory  : N_VN × QT  = 52×52 × 8 = 21632 bits
//                 (split: core 52×42×(8-2), ext 52×42×2×2 — see Fig.15)
//   CTV memory  : E_total × format
//                 (split: Bank1 full depth, Bank2 depth L0 — see Fig.14)
//
// Table III — Hardware results (28nm CMOS)
//   Throughput : 1.5 Gb/s (rate-1/5, BG2, max code length)
//   Area       : 0.34 mm²
//   IT_MAX     : 10 iterations
// =============================================================================

// --------------- 5G NR BG2 parameters ----------------------------------------
`define BG_ROWS       42          // M  — number of check nodes (rows in H)
`define BG_COLS       52          // N  — number of variable nodes (cols in H)
`define BG_K          10          // K  — information bits = N - M
`define LIFTING_Z     52          // Z  — lifting / circulant size
`define DC_MAX        10          // Maximum check-node degree (BG2)
`define DV_MAX         3          // Maximum variable-node degree (BG2)

// Number of non-zero circulants in BG2 protograph (approximate; exact=316)
`define NNZ_CIRC      316
// Total edges = NNZ_CIRC × Z
`define TOTAL_EDGES   16432       // 316 × 52

// --------------- Fixed-point word lengths -------------------------------------
`define QW             7          // CN→VN message width (sign + 6 mag bits)
`define QTW            8          // APP LLR width  (sign + 7 mag bits)
`define QIN            5          // VN→CN message width (sign + 4 mag bits)

// --------------- Memory depths ------------------------------------------------
`define APP_DEPTH      2704       // N_VN × Z = 52 × 52
`define CTV_DEPTH_B1   316        // CTV Bank1 depth = NNZ_CIRC (one entry per circulant)
// CTV Bank2 only holds rows with d_c < DC_MAX (variable-degree rows)
// For BG2: approximately 20 rows have d_c < DC_MAX → L0 = 20
`define CTV_DEPTH_B2   20

// --------------- CTV compressed format (Fig. 12) ------------------------------
// Per circulant block entry:
//   sign_bits : Z bits          (one sign per lifted edge)
//   min1      : QW-1 bits       (6 bits)
//   min2      : QW-1 bits       (6 bits)
//   idx1      : IDW bits        (4 bits, ceil(log2(DC_MAX)))
//   delta_flag: 1 bit           (delta == 0 ?)
//   Total     : Z + 6+6+4+1 = Z+17 bits
`define IDW            4          // ceil(log2(DC_MAX))
`define CTV_SIGN_W     52         // Z sign bits per circulant
`define CTV_META_W     17         // min1(6)+min2(6)+idx1(4)+delta_flag(1)
`define CTV_WORD_W     69         // CTV_SIGN_W + CTV_META_W

// --------------- Iteration count ----------------------------------------------
`define IT_MAX         10         // Maximum decoding iterations (Table III)

// --------------- APP memory bank widths (Fig. 15) ----------------------------
// Core bank   : Z × QTW bits per row, depth BG_ROWS
// Ext bank 0/1: Z × 2 bits per row (extension bits), same depth
`define APP_CORE_W     (`LIFTING_Z * `QTW)      // 52×8 = 416 bits/row
`define APP_EXT_W      (`LIFTING_Z * 2)          // 52×2 = 104 bits/row

// --------------- Read/Write network width ------------------------------------
`define RD_BUS_W       (`LIFTING_Z * `QTW)      // full Z-wide bus
`define WR_BUS_W       (`LIFTING_Z * `QTW)

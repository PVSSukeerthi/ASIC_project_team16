`define Z 16
`define MB 42
`define NB 52
`define N 1200
`define K 240
`define RATE_NUM  1
`define RATE_DEN   5

// ---- Quantization ----
`define Q          4         // Message bits (signed)
`define QTILDE     6         // APP bits (signed)
`define MSG_MAX    7         // 2^(Q-1)-1
`define APP_MAX    31        // 2^(QTILDE-1)-1

// ---- Decoder Structural Parameters ----
// BG2: rows 0..3 = core (4 rows), rows 4..41 = extension (38 rows)
// Orthogonal part: rows 20..41 (22 rows) -> merged to 11 pseudo-layers
// Total layers after merging: 4(core) + 16(non-orth ext) + 11(merged) = 31
// Paper states L reduces from 40 to 30 (we use 30 in impl)
`define NUM_CORE_ROWS    4
`define NUM_ORT_START    20   // first orthogonal row index (0-based)
`define NUM_ORT_ROWS     22   // rows 20..41
`define NUM_ORT_LAYERS   11   // merged pairs -> 11 layers
`define L_BEFORE         42   // MB
`define L_AFTER          31   // 4 + 16 + 11 = 31 (approx 30 in paper, slight diff due to BG2 exact structure)

// Max row degrees in BG2
`define DC_MAX       10   // maximum row degree overall BG2
`define DC_MAX_ORT    5   // maximum row degree in orthogonal part (rows 20-41 of BG2 <= dc_max/2)

// Column degree threshold for IAMS adaptation
`define D_THRESHOLD   6   // D parameter; VNs with dv >= D use OMS for core checks

// ---- Derived widths ----
// CTV compressed format per check: dc_max sign bits + 2*(q-1) min bits + 2*ceil(log2(dc_max)) idx bits
// For dc_max=10, q=4: 10 + 6 + 2*4 = 24 bits per Z block -> z*24
`define LOG2_DC_MAX   4                        // ceil(log2(10))=4
`define CTV_BITS_PER_CHECK (`DC_MAX + 2*(`Q-1) + 2*`LOG2_DC_MAX)  // = 10+6+8 = 24
`define CTV_WIDTH    (`Z * `CTV_BITS_PER_CHECK)  // = 52*24 = 1248 bits per layer

// For orthogonal part: dc_ort=5
`define LOG2_DC_ORT   3                        // ceil(log2(5))=3
`define CTV_BITS_ORT  (`DC_MAX_ORT + 2*(`Q-1) + 2*`LOG2_DC_ORT)  // = 5+6+6=17
`define CTV_ORT_WIDTH (2 * `Z * `CTV_BITS_ORT)  // 2 sets simultaneously

// IAMS: Column degree threshold D
// VNs with dv >= D in core checks use OMS instead of IAMS CN update

// ---- Memory depths ----
`define CTV_MEM1_DEPTH  `L_AFTER    // all layers
`define CTV_MEM2_DEPTH   15          // only core(4) + orthogonal(11) = 15 layers

// ---- Iteration limit (implementation) ----
`define ITMAX_IMPL   2

// ---- Misc ----
`define QMSG_W       `Q
`define QAPP_W       `QTILDE
`define HALF_Z       (`Z/2)


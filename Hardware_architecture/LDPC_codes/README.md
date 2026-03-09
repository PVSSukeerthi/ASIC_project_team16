# IAMS Decoder RTL — Cui et al., IEEE TCAS-I 2020, Section V
## Hardware Implementation in Verilog

---

## Paper Reference

> H. Cui, H. Ghaffari, B. Le, D. Declercq, C.-J. Lin, C. Wang,
> "Design of High-Performance and Area-Efficient Decoder for 5G LDPC Codes,"
> *IEEE Transactions on Circuits and Systems I*, 2020.

**Section V** covers the hardware architecture of the IAMS layered decoder:
- Check Node Unit (CNU) implementing eq. (10) — the four-case IAMS magnitude rule
- Variable Node Unit (VNU) with extrinsic-LLR computation and APP update
- Layered scheduling controller
- APP LLR and CN→VN message memories
- Syndrome checker for early termination

---

## Project Layout

```
iams_decoder/
├── rtl/
│   ├── cnu_iams.v          ← IAMS Check Node Unit  [CORE: eq. 10]
│   ├── vnu.v               ← Variable Node Unit     [layered β/APP update]
│   ├── layer_ctrl.v        ← Layer/iteration controller
│   ├── app_mem.v           ← APP LLR SRAM model
│   ├── alpha_mem.v         ← CN→VN message SRAM model
│   ├── syndrome_check.v    ← Syndrome (parity) checker
│   └── iams_decoder_top.v  ← Top-level integration
│
├── tb/
    ├── tb_cnu_iams.v       ← CNU unit testbench  (1000+ random trials)
    ├── tb_vnu.v            ← VNU unit testbench  (500+ random trials)
    └── tb_iams_decoder_top.v ← System-level testbench

```

---

## Fixed-Point Parameters (paper Table I, BG2)

| Symbol | Meaning                          | Value |
|--------|----------------------------------|-------|
| q      | Channel LLR fraction bits        | 4     |
| q̃     | APP LLR fraction bits            | 6     |
| Q      | CN→VN message saturation range   | ±31 (7-bit signed, QW=7) |
| Q̃     | APP LLR saturation range         | ±63 (8-bit signed, QTW=8) |
| τ      | IAMS scaling factor              | 1 (MS baseline; tune for OMS gain) |
| λ      | OMS offset (eq. 5)               | 1 LSB |

---

## IAMS CNU — Equation (10) Summary

For check node *m* with incoming messages β_{n,m}:

1. Compute min1, min2 (two smallest magnitudes) and their indices idx1, idx2
2. Compute Δ = min2 − min1
3. For each outgoing edge *n*:

   | Condition              | Output magnitude α   |
   |------------------------|----------------------|
   | n == idx1              | τ · min2             |
   | n == idx2              | τ · min1             |
   | n ∈ Ī(m), Δ = 0       | τ · max(min1 − 1, 0) |
   | n ∈ Ī(m), Δ ≠ 0       | τ · min1             |

4. Sign = XOR of all OTHER incoming signs (standard MS rule)

The Δ=0 case (cases 3 vs. 4) distinguishes IAMS from AMS and is the key
innovation correcting mismatch probability for low-degree check nodes.

---

## How to Run

### Option A: Icarus Verilog (free, open-source)

```bash
sudo apt install iverilog      # Ubuntu / Debian
iverilog -o <filename>.out <list_of_all_files>.v
vvp <filename>.out
gtkwave <filename>.out
```

### Option B: Vivado xsim

Via GUI:
1. **File → Project → New** → RTL Project
2. **Add Sources** → add all files in `rtl/`
3. **Add Simulation Sources** → add all files in `tb/`
4. **Flow Navigator → Run Simulation → Run Behavioral Simulation**
5. Select testbench top: `tb_cnu_iams`, `tb_vnu`, or `tb_iams_decoder_top`

## Expected Simulation Output

### tb_cnu_iams (CNU unit test)
```
[PASS] T1: reset state correct
[PASS] T2: all-zero inputs
[PASS] T3: directed — Δ≠0
[PASS] T4: directed — Δ=0
...
[PASS] T1007: random trial 1000
Results: 1008 PASS,  0 FAIL
*** ALL TESTS PASSED ***
```

### tb_vnu (VNU unit test)
```
[PASS] T1  app_in=10  an=3  ap=2  → beta=8  app=11
...
Results: 507 PASS,  0 FAIL
*** ALL TESTS PASSED ***
```

---

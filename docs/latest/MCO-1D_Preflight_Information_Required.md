# MCO-1D — Physical Parameters Required Before Test Cards

**Version: Revision 18 — July 2026**  
**Collect all items below before running `tc01_start`.**  
The values feed `mco_build_measurements.py`, which patches the wave library
and TC-01 feasibility gate with real numbers instead of design estimates.

Run: `python3 mco_build_measurements.py` — it will prompt for each item.

---

## A. MASS AND WEIGHT — CRITICAL FOR HOVER GATE

| # | Item | Unit | How to measure | Code key |
|---|------|------|----------------|----------|
| A1 | **Total assembled mass** (both batteries installed, no load cell) | kg | Kitchen scale, 5 kg / 1 g res. | `mtow_kg` |
| A2 | Mass with main battery removed | kg | Kitchen scale | `mass_no_main_batt_kg` |
| A3 | Mass with both batteries removed | kg | Kitchen scale | `mass_no_batts_kg` |
| A4 | Main LiPo mass (weigh separately) | g | Kitchen scale | `main_lipo_g` |
| A5 | Servo LiPo mass (weigh separately) | g | Kitchen scale | `servo_lipo_g` |

> **Why A1 matters:** TC-01 step 16 feasibility gate computes thrust/MTOW ratio.  
> If this number is wrong, the gate classifies FREE_HOVER vs STOL_ONLY incorrectly.  
> Design MTOW is 3.448 kg. Estimated assembled mass is 5.095 kg. Measure the real value.

---

## B. WING GEOMETRY — AFFECTS HOVER FREQUENCY PREDICTION

| # | Item | Unit | How to measure | Code key |
|---|------|------|----------------|----------|
| B1 | W1 semi-span (pivot to rod tip) | mm | Steel rule, pivot centre to tip | `span_w1_mm` |
| B2 | W2 semi-span | mm | Same | `span_w2_mm` |
| B3 | W3 semi-span | mm | Same | `span_w3_mm` |
| B4 | W4 semi-span | mm | Same | `span_w4_mm` |
| B5 | Tip chord (at rod tip, before winglet) | mm | Callipers across chord | `chord_tip_mm` |
| B6 | Root chord at bolt circle (max) | mm | Callipers | `chord_root_mm` |
| B7 | Root taper length (from bolt circle to where chord = tip chord) | mm | Steel rule | `root_taper_mm` |
| B8 | Winglet span (from rod tip to winglet tip) | mm | Steel rule | `winglet_span_mm` |
| B9 | Winglet root chord | mm | Callipers | `winglet_chord_root_mm` |
| B10 | Winglet tip chord | mm | Callipers | `winglet_chord_tip_mm` |
| B11 | Winglet cant angle (from vertical) | ° | Digital angle gauge on wing surface | `winglet_cant_deg` |

> **Target values:** B1=772, B2=749, B3=735, B4=712 mm. B5=165 mm. B6=230–250 mm.  
> Measure what you actually cut and built — not the design targets.

---

## C. SECTOR WHEEL AND GEAR RATIO — AFFECTS FREQUENCY CALIBRATION

| # | Item | Unit | How to measure | Code key |
|---|------|------|----------------|----------|
| C1 | **Actual gear ratio** (motor turns per full sector revolution) | ratio | Command 70 motor turns via ODrive encoder count. Measure sector degrees swept. 7× = 360°. | `gear_ratio` |
| C2 | Sector wheel arc (mid-stroke to hard stop, each side) | ° | Digital angle gauge. Measure from centre to each hard stop separately. | `sector_arc_pos_deg`, `sector_arc_neg_deg` |
| C3 | Sector arc asymmetry (difference between C2 measurements) | ° | Computed from C2 | `sector_asymmetry_deg` |
| C4 | Software amplitude limit vs hard stop reserve | ° | Should be: hard stop − 48° ≥ 4° each side | `sector_reserve_deg` |

> **C1 is critical:** all frequency predictions use the gear ratio.  
> If actual ratio differs from 7:1, hover frequency estimate is wrong.  
> C2 must show equal arcs (±2°) before any frequency above 0.5 Hz is run.  
> If C3 > 2°: re-seat motor drum and re-measure before proceeding.

---

## D. SERVO RANGE AND FEEDBACK — REQUIRED FOR CALIBRATION

| # | Item | Unit | How to measure | Code key |
|---|------|------|----------------|----------|
| D1 | Pitch servo neutral ADC count (all 8 wings at 0° AoA) | counts | Read MCP3208 at neutral command | `pitch_adc_neutral_counts[8]` |
| D2 | Pitch servo +45° ADC count (all 8) | counts | Command +45°, read ADC | `pitch_adc_pos45_counts[8]` |
| D3 | Pitch servo −8° ADC count (all 8) | counts | Command −8°, read ADC | `pitch_adc_neg8_counts[8]` |
| D4 | Sweep servo neutral ADC count (60° position, all 8) | counts | Read at neutral | `sweep_adc_neutral_counts[8]` |
| D5 | **Actual stow angle reached** by each sweep servo (H-33 State C) | ° | Command max aft. Measure with angle gauge. Must reach 85° minimum. | `sweep_stow_deg[8]` |
| D6 | Sweep servo max forward angle | ° | Command full forward, measure | `sweep_fwd_deg[8]` |

> **D5 is a gate:** if any sweep servo cannot reach 85° aft, H-33 State C is  
> mechanically unavailable on that wing. The software must be told this explicitly.  
> Command: `set_wing_stow_limit W1p 82` (for example) if that wing only reaches 82°.

---

## E. PROPULSION PERFORMANCE — TC-01 INPUTS

| # | Item | Unit | How to measure | Code key |
|---|------|------|----------------|----------|
| E1 | Motor baseline current at 5 rad/s (all 8 wings) | A | S1 `Iq_measured` via CAN during preflight spin | auto via TC-01-06a |
| E2 | Motor current at 0.5 Hz flapping (all 8) | A | S1 readback during TC-01-07 | auto |
| E3 | Motor current at 2.5 Hz flapping (all 8) | A | S1 readback during TC-01-08 | auto |
| E4 | FET temperature at 2.5 Hz sustained 60 s | °C | S1 readback during TC-01-08 | auto |
| E5 | Static thrust at each frequency step | N | HX711 + load cell during TC-01-12 | auto |

> These are collected automatically during TC-01. List them here so  
> you know what the test card is recording.

---

## F. STRUCTURAL AND CG — STABILITY PREDICTION

| # | Item | Unit | How to measure | Code key |
|---|------|------|----------------|----------|
| F1 | CG position (fore-aft from W1 pivot) | mm | Balance on known fulcrum | `cg_fwd_mm` |
| F2 | CG position (lateral — should be on centreline) | mm | Balance | `cg_lat_mm` |
| F3 | CG height above ground (with undercarriage) | mm | Steel rule | `cg_height_mm` |
| F4 | Undercarriage height (wing root pivot to ground) | mm | Steel rule | `h_gear_mm` |
| F5 | W4 wing tip to ground at full down-stroke | mm | Measure at 48° deflection | `tip_ground_clearance_mm` |
| F6 | Pitot tube tip to nose | mm | Steel rule | `pitot_offset_mm` |

> **F5 must be > 50 mm** with ground clearance margin. If < 50 mm: reduce amplitude  
> limit in wave library before TC-01. Wing strike on ground during TC-01 will  
> damage both wing and load cell fixture.

---

## G. SENSOR VERIFICATION — BEFORE ANY POWER-ON

| # | Item | Unit | Verified how | Code key |
|---|------|------|--------------|----------|
| G1 | OAT sensor reading (DS18B20) | °C | `python3 -c "import glob,time; [print(open(f).read()) for f in glob.glob('/sys/bus/w1/devices/28*/temperature')]"` | `oat_c` |
| G2 | IMU connected (MPU-6050 at 0x68) | pass/fail | `i2cdetect -y 1` — must show 68 | — |
| G3 | Barometer connected (BMP280 at 0x76) | pass/fail | `i2cdetect -y 1` — must show 76 | — |
| G4 | Pitot receiving MAVLink VFR_HUD | pass/fail | `hover_mode_status` on GS — V_ms changes when you blow on pitot | `pitot_valid` |
| G5 | All 8 CAN nodes responding | pass/fail | `python3 mco_odrive_setup.py --verify` — all 8 must respond | — |
| G6 | All 8 encoder magnets field_status=1 | pass/fail | `python3 mco_odrive_setup.py --check-magnet` | — |
| G7 | RC channels verified (lever direction) | pass/fail | `python3 mco_rc_verify.py` | — |

> **G4 is a safety gate:** hover mode cannot engage until pitot_valid=True.  
> Blow on the pitot tube — GS `pitot_status` must show VALID before arming.

---

## H. PHYSICAL VERIFICATION — BEFORE TC-01 FREQUENCY STEPS

| # | Item | Check | Action if fail |
|---|------|-------|----------------|
| H1 | All 8 rope drives: no fraying, correct tension | Visual + tension check | Replace rope |
| H2 | All 16 servo horns secure (Loctite 243) | Pull test by hand | Re-secure |
| H3 | Load cell fixture rigid (no movement at 50 N) | Push test by hand | Reinforce |
| H4 | Sector arc symmetry ≤ 2° (from C2/C3) | Angle gauge | Re-seat motor drum |
| H5 | Wing tip ground clearance > 50 mm at full stroke | Measure at slow tick | Reduce amplitude limit |
| H6 | Covert flaps free to pivot, return to neutral | Deflect by hand | Free hinge / adjust spring |
| H7 | Turbulator tape bonded, no bubbles or lifting | Visual | Re-apply |
| H8 | All 8 winglets aligned at 75-80° cant, secure | Angle gauge at winglet | Re-bond sleeve |

---

## QUICK REFERENCE — DESIGN VS ESTIMATED VS ACTUAL

| Parameter | Design | Estimated (calculated) | Actual (fill in) |
|-----------|--------|------------------------|------------------|
| Total mass | 3.448 kg | 5.095 kg | _______ kg |
| Wing loading | 34.5 N/m² | 45.7 N/m² | _______ N/m² |
| Hover frequency | 1.54 Hz | 1.87 Hz | _______ Hz (TC-02) |
| Max thrust @ 3 Hz | 78.4 N | 128.8 N | _______ N (TC-01) |
| Thrust / MTOW @ 3 Hz | 2.32× | 2.58× | _______ × |
| Gear ratio | 7:1 | 7:1 assumed | _______ :1 (measure) |
| Sector arc sym | ±50° | ±50° | _______ ° pos / _______ ° neg |
| Sweep stow angle | 85–95° | 85°+ | _______ ° (measure each wing) |
| Pitot valid at boot | required | required | YES / NO |
| CG fore-aft from W1 | TBD | TBD | _______ mm |

> Fill in the **Actual** column from physical measurements before running TC-01.  
> Enter values into `mco_build_measurements.py` which writes  
> `/home/pi/knowledge/build_measurements.json`.  
> TC-01 reads this file at start-up and uses actual values in the feasibility gate.

---

## WHAT HAPPENS IF YOU SKIP THIS

| Skipped item | Consequence |
|--------------|-------------|
| A1 (actual mass) | TC-01 gate classifies using 3.448 kg — may give FREE_HOVER when aircraft is too heavy |
| C1 (gear ratio) | Hover frequency prediction wrong — descent controller miscalibrated from TC-02 |
| C2/C3 (sector symmetry) | Wing hits hard stop at frequency — potential wing failure or rope snap |
| D5 (stow angle) | H-33 State C commanded on wing that cannot reach stow — servo stall |
| F5 (tip clearance) | Wing strikes load cell fixture at first frequency step |
| G4 (pitot valid) | Hover mode will not arm — aircraft cannot fly in hover mode |
| G6 (magnet check) | Cold-start position unknown — S1 commanded to wrong frequency |
| H4 (sector symmetry) | Duplicate of C2/C3 — confirm before electrical check |


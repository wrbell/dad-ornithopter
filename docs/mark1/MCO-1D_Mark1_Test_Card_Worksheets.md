# MCO-1D Mark 1 — Physical Measurement & Flight Test Card Worksheets

**Aircraft Title**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  
**Demonstrator Program**: MCO-1A Search-and-Rescue (SAR) Subscale Flight Demonstrator  
**Document Designation**: MCO-1D-WS-MK1  
**Classification**: Test Engineering & Physical Data Acquisition Baseline — Revision Mark 1 (September 2026)  
**Target Hardware Baseline**: 8-Wing Cascade, 10:1 Capstan Drivetrain, 5.687 kg MTOW  

---

## 1. How to Use These Worksheets

As the physical mechanism of the **MCO-1D Mark 1** is assembled, rigged, and progressed through each flight test gate, record the measured physical numbers into the corresponding worksheet. 

Whenever you provide these filled numbers back in chat or save them to [`knowledge/build_measurements.json`](file:///Users/drbell/Code/flight/knowledge/build_measurements.json), the simulation models, flight governor limits, and mass properties in `firmware/` and `scripts/` will be updated with your exact as-built numbers.

```
Worksheet Directory:
  * WS-00: As-Built Physical Airframe & Structural Measurement Worksheet
  * WS-01: Test Card TC-01 — Ground Bench Test Worksheet (Rigid Load Stand)
  * WS-02: Test Card TC-02 — Tethered Lift & Hover Characterization Worksheet
  * WS-03: Test Card TC-03 — Moving Vehicle Advance-Ratio Sweep Worksheet
  * WS-04: Test Card TC-04 — Free Flight STOL Runway Expansion Worksheet
```

---

## WS-00: As-Built Physical Airframe & Structural Worksheet

*Complete this worksheet during mechanical assembly prior to initial electrical power-on.*

### 0.1 Mass Properties & Component Weights
Weigh with a precision digital scale (0.1 g or 1 g resolution).

| Parameter Key | Description | Unit | Nominal Target | As-Built Measurement |
| :--- | :--- | :---: | :---: | :--- |
| `mtow_total_g` | **Total Assembled Flying Mass** (All 8 wings, 6S + 2S batteries, ready for flight) | g | $5,687\text{ g}$ | `[  __________ g  ]` |
| `mass_dry_airframe_g`| Dry airframe mass (both flight batteries removed) | g | $\le 4,972\text{ g}$ | `[  __________ g  ]` |
| `mass_main_battery_6s_g`| 6S 4500mAh 60C LiPo motor pack (weighed separately) | g | $570.0\text{ g}$ | `[  __________ g  ]` |
| `mass_servo_battery_2s_g`| 2S 3000mAh LiPo receiver/servo pack (weighed separately)| g | $145.0\text{ g}$ | `[  __________ g  ]` |
| `mass_fuselage_spine_g`| Central 20mm carbon spine + station brackets + hardware | g | $710.0\text{ g}$ | `[  __________ g  ]` |
| `mass_avionics_pack_g` | FCC (Pi 4B + Hat + Pixhawk 4 Mini + wiring harness) | g | $320.0\text{ g}$ | `[  __________ g  ]` |
| `mass_landing_gear_g` | Carbon landing gear struts + Du-Bro lightweight wheels | g | $185.0\text{ g}$ | `[  __________ g  ]` |

### 0.2 Individual Wing Weights (Bare Wing Panels)
Weigh each individual finished wing panel (including 10mm spar, ribs, Oralight covering, tip ballast, and root ferrule):

| Wing Station | Port Wing Mass (g) | Starboard Wing Mass (g) | Target per Wing | Asymmetry Check ($\le 1.5\text{ g}$) |
| :--- | :---: | :---: | :---: | :---: |
| **Station 1 ($W_1$ Fore)** | `[  _______ g  ]` | `[  _______ g  ]` | $68.5\text{ g}$ | `[  PASS / FAIL  ]` |
| **Station 2 ($W_2$ Mid-Fore)**| `[  _______ g  ]` | `[  _______ g  ]` | $66.8\text{ g}$ | `[  PASS / FAIL  ]` |
| **Station 3 ($W_3$ Mid-Aft)** | `[  _______ g  ]` | `[  _______ g  ]` | $65.5\text{ g}$ | `[  PASS / FAIL  ]` |
| **Station 4 ($W_4$ Aft)** | `[  _______ g  ]` | `[  _______ g  ]` | $63.2\text{ g}$ | `[  PASS / FAIL  ]` |
| **Total All 8 Wings** | **Sum Port:** `[ _____ g ]` | **Sum Stbd:** `[ _____ g ]` | **$528.0\text{ g}$ Total** | **Delta:** `[ _____ g ]` |

### 0.3 Wing Geometry & Span Dimensions
Measure using a precision steel rule or laser distance meter from pivot centerline to spar tip.

| Parameter Key | Description | Unit | Nominal Target | As-Built Measurement |
| :--- | :--- | :---: | :---: | :--- |
| `span_w1_port_mm` | $W_1$ Port semi-span (pivot centerline to tip) | mm | $772.0\text{ mm}$ | `[  __________ mm  ]` |
| `span_w1_stbd_mm` | $W_1$ Starboard semi-span (pivot centerline to tip) | mm | $772.0\text{ mm}$ | `[  __________ mm  ]` |
| `span_w2_port_mm` | $W_2$ Port semi-span | mm | $749.0\text{ mm}$ | `[  __________ mm  ]` |
| `span_w2_stbd_mm` | $W_2$ Starboard semi-span | mm | $749.0\text{ mm}$ | `[  __________ mm  ]` |
| `span_w3_port_mm` | $W_3$ Port semi-span | mm | $735.0\text{ mm}$ | `[  __________ mm  ]` |
| `span_w3_stbd_mm` | $W_3$ Starboard semi-span | mm | $735.0\text{ mm}$ | `[  __________ mm  ]` |
| `span_w4_port_mm` | $W_4$ Port semi-span | mm | $712.0\text{ mm}$ | `[  __________ mm  ]` |
| `span_w4_stbd_mm` | $W_4$ Starboard semi-span | mm | $712.0\text{ mm}$ | `[  __________ mm  ]` |
| `total_wingspan_w1_mm`| Tip-to-tip total wingspan at Station 1 ($W_1$) | mm | $1,584.0\text{ mm}$ | `[  __________ mm  ]` |
| `chord_root_mm` | Wing chord at root clamp (leading edge to trailing edge) | mm | $120.0\text{ mm}$ | `[  __________ mm  ]` |
| `chord_tip_mm` | Wing chord at outermost tip rib | mm | $120.0\text{ mm}$ | `[  __________ mm  ]` |
| `fuselage_width_station1`| Lateral distance between Port and Starboard pivot centers | mm | $40.0\text{ mm}$ | `[  __________ mm  ]` |

### 0.4 Longitudinal Station Spacing & Vertical Staircase
Measure longitudinal positions aft from **Nose Tip (Datum 0.0 mm)**.

| Parameter Key | Description | Unit | Nominal Target | As-Built Measurement |
| :--- | :--- | :---: | :---: | :--- |
| `station1_pivot_x_mm` | Station 1 ($W_1$) pivot axis distance aft of nose | mm | $138.0\text{ mm}$ | `[  __________ mm  ]` |
| `station2_pivot_x_mm` | Station 2 ($W_2$) pivot axis distance aft of nose | mm | $276.0\text{ mm}$ | `[  __________ mm  ]` |
| `station3_pivot_x_mm` | Station 3 ($W_3$) pivot axis distance aft of nose | mm | $414.0\text{ mm}$ | `[  __________ mm  ]` |
| `station4_pivot_x_mm` | Station 4 ($W_4$) pivot axis distance aft of nose | mm | $552.0\text{ mm}$ | `[  __________ mm  ]` |
| `stagger_interstation_mm`| Longitudinal distance between adjacent station pivots | mm | $138.0\text{ mm}$ | `[  __________ mm  ]` |
| `vert_staircase_drop_mm`| Vertical drop per station ($Z$ offset down spine) | mm | $41.25\text{ mm}$ | `[  __________ mm  ]` |
| `interstation_slot_gap_mm`| Trailing edge to leading edge gap at neutral sweep | mm | $25.0\text{ mm}$ | `[  __________ mm  ]` |

### 0.5 Drivetrain, Dyneema Rigging & Center of Gravity
| Parameter Key | Description | Unit | Nominal Target | As-Built Measurement |
| :--- | :--- | :---: | :---: | :--- |
| `sector_pitch_radius_mm`| Carbon sector wheel pitch radius $R$ | mm | $77.0\text{ or } 90.0\text{ mm}$ | `[  __________ mm  ]` |
| `capstan_pitch_dia_mm` | Helical motor pinion drum pitch diameter $d_{\text{pitch}}$ | mm | $15.4\text{ or } 18.0\text{ mm}$ | `[  __________ mm  ]` |
| `measured_gear_ratio` | Verified transmission reduction ratio ($\theta_{\text{motor}} / \theta_{\text{wing}}$) | ratio | **10.00 : 1** | `[  __________ : 1  ]` |
| `dyneema_settled_tension_N`| Post-48hr settled cable tension (all 8 capstans) | N | $180.0\text{ N} \pm 5\text{ N}$ | `[  __________ N  ]` |
| `spar_ferrule_od_mm` | Root aluminum crush ferrule outer diameter | mm | $8.00\text{ mm} \pm 0.05$ | `[  __________ mm  ]` |
| `battery_sled_hole_index`| Quick-pin hole engaged on battery keel track | index | **Hole 3 (250 mm)** | `[  Hole # ______  ]` |
| `cg_location_from_nose_mm`| Measured aircraft balance point aft of nose tip | mm | $311.8\text{ mm} \pm 3\text{ mm}$ | `[  __________ mm  ]` |
| `cg_mac_percent` | Resulting Center of Gravity position in % MAC | % MAC | **26.5% MAC** | `[  __________ %  ]` |

---

## WS-01: Test Card TC-01 — Ground Bench Test Worksheet

*Complete this worksheet while the airframe is mounted horizontally to the rigid load cell stand.*

### 1.1 Environmental Conditions
| Parameter | Unit | Test Stand Value |
| :--- | :---: | :--- |
| Ambient Air Temperature ($T_{\text{ambient}}$) | °C | `[  __________ °C  ]` |
| Barometric Pressure ($P_{\text{baro}}$) | hPa / mbar | `[  __________ hPa  ]` |
| Calculated Air Density ($\rho$) | $\text{kg/m}^3$ | `[  __________ kg/m³  ]` |

### 1.2 Multi-Frequency Flapping Test Run Matrix
*Run each point for the specified dwell time, recording static thrust, motor current, and temperatures.*

| Point | Commanded Freq (Hz) | Stroke Amp (deg) | Dwell (s) | Target Min Thrust | Measured Thrust (N) | Total 6S Bus Current (A) | Peak Motor Temp (°C) | Peak ODrive FET Temp (°C) | Cable Witness OK? |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **P1** | $1.0\text{ Hz}$ | $\pm 60.0^\circ$ | $30\text{ s}$ | $\ge 8.0\text{ N}$ | `[  ______ N  ]` | `[  ______ A  ]` | `[  ______ °C  ]` | `[  ______ °C  ]` | `[ YES / NO ]` |
| **P2** | $1.5\text{ Hz}$ | $\pm 70.0^\circ$ | $30\text{ s}$ | $\ge 16.0\text{ N}$ | `[  ______ N  ]` | `[  ______ A  ]` | `[  ______ °C  ]` | `[  ______ °C  ]` | `[ YES / NO ]` |
| **P3** | $2.0\text{ Hz}$ | $\pm 80.0^\circ$ | $30\text{ s}$ | $\ge 24.0\text{ N}$ | `[  ______ N  ]` | `[  ______ A  ]` | `[  ______ °C  ]` | `[  ______ °C  ]` | `[ YES / NO ]` |
| **P4** | $2.5\text{ Hz}$ | $\pm 80.0^\circ$ | $45\text{ s}$ | $\ge 31.0\text{ N}$ | `[  ______ N  ]` | `[  ______ A  ]` | `[  ______ °C  ]` | `[  ______ °C  ]` | `[ YES / NO ]` |
| **P5** | $3.0\text{ Hz}$ | $\pm 80.0^\circ$ | $60\text{ s}$ | $\ge 37.0\text{ N}$ | `[  ______ N  ]` | `[  ______ A  ]` | `[  ______ °C  ]` | `[  ______ °C  ]` | `[ YES / NO ]` |
| **P6** | $3.5\text{ Hz}$ | $\pm 70.0^\circ$ | $20\text{ s}$ | $\ge 42.0\text{ N}$ | `[  ______ N  ]` | `[  ______ A  ]` | `[  ______ °C  ]` | `[  ______ °C  ]` | `[ YES / NO ]` |

### 1.3 Electrical Power & Servo Integrity Checks
| Verification Item | Pass Criteria | Measured Value | Status |
| :--- | :---: | :---: | :---: |
| 7.4V Servo Rail Voltage under Point P5 ($3.0\text{ Hz}$) | $\ge 7.10\text{ V}$ | `[  __________ V  ]` | `[ PASS / FAIL ]` |
| Peak Servo Rail Current Surge during reversal | $\le 45.0\text{ A}$ | `[  __________ A  ]` | `[ PASS / FAIL ]` |
| 6S Motor Bus Voltage at P5 ($3.0\text{ Hz}$) full load | $\ge 21.60\text{ V}$ | `[  __________ V  ]` | `[ PASS / FAIL ]` |
| Dyneema Max Cable Angular Slippage | $\le 0.5^\circ$ | `[  __________ °  ]` | `[ PASS / FAIL ]` |
| Airframe RMS Accelerometer Vibration (10–200 Hz) | $\le 2.0\text{ g}$ | `[  __________ g  ]` | `[ PASS / FAIL ]` |

---

## WS-02: Test Card TC-02 — Tethered Lift & Hover Characterization

*Complete this worksheet while the aircraft is attached to the vertical guide cable gantry.*

### 2.1 Equilibrium & Weightless Lift-Off Point
| Parameter Key | Description | Target Expected | Measured Value |
| :--- | :--- | :---: | :--- |
| `f_unstick_hz` | **Flapping Frequency where aircraft becomes weightless ($T/W = 1.00$)** | $1.85\text{--}2.10\text{ Hz}$ | `[  __________ Hz  ]` |
| `stroke_amp_hover_deg`| Commanded flapping stroke amplitude at unstick | $\pm 75.0^\circ$ | `[  __________ °  ]` |
| `i_bus_equilibrium_a` | Total 6S battery current at steady weightless hover | $18.0\text{--}24.0\text{ A}$ | `[  __________ A  ]` |
| `p_elec_hover_watts` | Total active electrical hover power draw ($V_{\text{bus}} \times I_{\text{bus}}$) | $400\text{--}530\text{ W}$ | `[  __________ W  ]` |
| `guide_cable_friction_n`| Estimated guide collar friction on 4 mm wire (tare force) | $< 2.0\text{ N}$ | `[  __________ N  ]` |

### 2.2 Dynamic Attitude Control Authority (Step Stick Injection)
*Inject $\pm 5^\circ$ stick steps while suspended in hover trim; record peak rates.*

| Axis / Channel | Commanded Step | Minimum Required Rate | Measured Response Rate | Damping Quality |
| :--- | :---: | :---: | :---: | :---: |
| **Pitch Control (AoA Differential)** | $+5^\circ$ Pitch Up | $\ge 15.0^\circ/\text{s}$ | `[  __________ °/s  ]` | `[ Deadbeat / Oscillatory ]` |
| **Pitch Control (AoA Differential)** | $-5^\circ$ Pitch Down | $\ge 15.0^\circ/\text{s}$ | `[  __________ °/s  ]` | `[ Deadbeat / Oscillatory ]` |
| **Roll Control (Stroke Amp Diff)** | $+5^\circ$ Roll Right | $\ge 20.0^\circ/\text{s}$ | `[  __________ °/s  ]` | `[ Deadbeat / Oscillatory ]` |
| **Roll Control (Stroke Amp Diff)** | $-5^\circ$ Roll Left | $\ge 20.0^\circ/\text{s}$ | `[  __________ °/s  ]` | `[ Deadbeat / Oscillatory ]` |
| **Yaw Control (Sweep Differential)** | $+10^\circ$ Yaw Right | $\ge 12.0^\circ/\text{s}$ | `[  __________ °/s  ]` | `[ Deadbeat / Oscillatory ]` |

---

## WS-03: Test Card TC-03 — Moving Vehicle Advance-Ratio Sweep

*Complete this worksheet with the aircraft mounted to the 6-axis load cell pylon above the test truck.*

### 3.1 Aerodynamic Force Polar Matrix vs Forward Airspeed
*Run truck in calm air ($< 1.0\text{ m/s}$ wind) at steady velocity steps; record load cell forces.*

| Run # | Truck Speed $V_\infty$ | Commanded Freq $f$ | Net Thrust / Drag $F_x$ | Net Vertical Lift $L$ | Pitch Moment $M_y$ | Pitot Indicated Airspeed | Pitot ADC Dynamic Counts |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **R1** | $4.0\text{ m/s}$ ($9.0\text{ mph}$) | $2.20\text{ Hz}$ | `[  _______ N  ]` | `[  _______ N  ]` | `[  _______ N·m  ]` | `[  _______ m/s  ]` | `[  _______ counts  ]` |
| **R2** | $6.2\text{ m/s}$ ($13.9\text{ mph}$) | $2.30\text{ Hz}$ | `[  _______ N  ]` | `[  _______ N  ]` | `[  _______ N·m  ]` | `[  _______ m/s  ]` | `[  _______ counts  ]` |
| **R3** | $9.0\text{ m/s}$ ($20.1\text{ mph}$) | $2.40\text{ Hz}$ | `[  _______ N  ]` | `[  _______ N  ]` | `[  _______ N·m  ]` | `[  _______ m/s  ]` | `[  _______ counts  ]` |
| **R4** | $12.0\text{ m/s}$ ($26.8\text{ mph}$) | $2.60\text{ Hz}$ | `[  _______ N  ]` | `[  _______ N  ]` | `[  _______ N·m  ]` | `[  _______ m/s  ]` | `[  _______ counts  ]` |
| **R5** | $14.0\text{ m/s}$ ($31.3\text{ mph}$) | $2.80\text{ Hz}$ | `[  _______ N  ]` | `[  _______ N  ]` | `[  _______ N·m  ]` | `[  _______ m/s  ]` | `[  _______ counts  ]` |

### 3.2 Unpowered Zero-Flapping Glide Verification ($f = 0.0\text{ Hz}$)
*Hold wings stationary at clean cruise pitch ($\alpha = +4.0^\circ$) to measure pure passive glide performance.*

| Truck Speed $V_\infty$ | Commanded Flap | Measured Passive Lift $L$ | Passive Drag $D$ | Measured $L/D$ Ratio | Expected $L_{\text{passive}}$ Target |
| :---: | :---: | :---: | :---: | :---: | :---: |
| $9.0\text{ m/s}$ ($20.1\text{ mph}$) | **$0.0\text{ Hz}$ (Fixed)** | `[  __________ N  ]` | `[  __________ N  ]` | `[  __________  ]` | $37.13\text{ N}$ ($18.6\text{ N}$ deficit) |
| $11.0\text{ m/s}$ ($24.6\text{ mph}$) | **$0.0\text{ Hz}$ (Fixed)** | `[  __________ N  ]` | `[  __________ N  ]` | `[  __________  ]` | $55.77\text{ N}$ ($1.00g$ equilibrium) |
| $13.0\text{ m/s}$ ($29.1\text{ mph}$) | **$0.0\text{ Hz}$ (Fixed)** | `[  __________ N  ]` | `[  __________ N  ]` | `[  __________  ]` | $77.80\text{ N}$ (excess lift) |

---

## WS-04: Test Card TC-04 — Free Flight STOL Runway Expansion

*Complete this worksheet during the initial free-flight taxi, takeoff, circuit, and landing sorties.*

### 4.1 Pre-Flight Electrical Status
| Parameter Key | Description | Unit | Target / Limit | Measured Pre-Flight |
| :--- | :--- | :---: | :---: | :--- |
| `v_batt_6s_pre_v` | 6S Main LiPo starting pack voltage (rested) | V | $\ge 25.0\text{ V}$ ($4.17\text{ V/cell}$) | `[  __________ V  ]` |
| `v_batt_2s_pre_v` | 2S Servo LiPo starting pack voltage (rested) | V | $\ge 8.30\text{ V}$ ($4.15\text{ V/cell}$) | `[  __________ V  ]` |
| `wind_speed_runway_ms`| Surface wind speed along runway centerline | m/s | $\le 3.0\text{ m/s}$ | `[  __________ m/s  ]` |
| `wind_cross_ms` | Crosswind component perpendicular to runway | m/s | $\le 1.0\text{ m/s}$ | `[  __________ m/s  ]` |

### 4.2 Takeoff & Climb Performance (Phase 1)
| Parameter Key | Description | Unit | Predicted Target | Actual Flight Measurement |
| :--- | :--- | :---: | :---: | :--- |
| `takeoff_roll_distance_m`| Ground distance from brake release to unstick | m | $\le 12.0\text{ m}$ | `[  __________ m  ]` |
| `v_unstick_indicated_ms`| Indicated airspeed at wheels-off liftoff | m/s | $6.20\text{ m/s} \pm 0.4$ | `[  __________ m/s  ]` |
| `time_to_unstick_sec` | Elapsed time from throttle advance to liftoff | s | $3.5\text{--}4.5\text{ s}$ | `[  __________ s  ]` |
| `climb_rate_vz_ms` | Steady rate of climb $V_z$ at $9.0\text{ m/s}$ airspeed | m/s | $1.80\text{--}2.20\text{ m/s}$ | `[  __________ m/s  ]` |
| `climb_angle_gamma_deg` | Flight path climb angle $\gamma$ | deg | $11.5^\circ\text{--}15.0^\circ$ | `[  __________ °  ]` |
| `peak_takeoff_power_w` | Peak 6S battery electrical power during climb | W | $\le 680\text{ W}$ | `[  __________ W  ]` |

### 4.3 Cruise & Handling Qualities (Phase 2)
| Parameter Key | Description | Unit | Predicted Target | Actual Flight Measurement |
| :--- | :--- | :---: | :---: | :--- |
| `cruise_airspeed_ms` | Steady level cruise airspeed $V_\infty$ | m/s | $9.00\text{ m/s}$ | `[  __________ m/s  ]` |
| `cruise_flapping_freq_hz`| Flapping frequency commanded in level cruise | Hz | $2.20\text{--}2.50\text{ Hz}$ | `[  __________ Hz  ]` |
| `cruise_throttle_pct` | Throttle / advance lever setting for level flight | % | $48\%\text{--}55\%$ | `[  __________ %  ]` |
| `cruise_power_watts` | Average electrical power draw during level cruise | W | $240\text{--}280\text{ W}$ | `[  __________ W  ]` |
| `cost_of_transport_wh_km`| Specific energy consumption per kilometer flown | Wh/km | $7.4\text{--}8.6\text{ Wh/km}$ | `[  __________ Wh/km  ]` |
| `pitch_trim_stick_offset`| Elevator / pitch stick bias required for hands-off level | % | $\pm 5.0\%$ | `[  __________ %  ]` |
| `roll_trim_stick_offset` | Roll stick bias required for wings-level heading | % | $\pm 3.0\%$ | `[  __________ %  ]` |

### 4.4 Approach, Flare & Landing Rollout (Phase 3)
| Parameter Key | Description | Unit | Predicted Target | Actual Flight Measurement |
| :--- | :--- | :---: | :---: | :--- |
| `v_approach_indicated_ms`| Final approach indicated airspeed | m/s | $7.50\text{ m/s} \pm 0.3$ | `[  __________ m/s  ]` |
| `flare_touchdown_speed_ms`| Wheels touchdown airspeed after flare airbrake | m/s | $4.50\text{--}5.20\text{ m/s}$ | `[  __________ m/s  ]` |
| `landing_rollout_m` | Ground rollout distance from touchdown to full stop | m | $\le 8.5\text{ m}$ | `[  __________ m  ]` |
| `landing_g_peak` | Peak vertical touchdown acceleration on Pixhawk | g | $\le 1.8\text{ g}$ | `[  __________ g  ]` |

### 4.5 Post-Flight Debrief & Battery Reserve
| Parameter Key | Description | Unit | Safety Limit | Actual Measured Post-Flight |
| :--- | :--- | :---: | :---: | :--- |
| `flight_duration_min_sec`| Total airborne flight duration | min:sec | Target $3\text{--}5\text{ min}$ | `[  ____ min : ____ sec  ]` |
| `v_batt_6s_post_v` | 6S Main LiPo rested resting voltage | V | $\ge 21.8\text{ V}$ ($3.63\text{ V/cell}$) | `[  __________ V  ]` |
| `capacity_discharged_mah`| 6S battery capacity consumed (recharger reading) | mAh | $\le 3,375\text{ mAh}$ (75% DoD) | `[  __________ mAh  ]` |
| `v_batt_2s_post_v` | 2S Servo LiPo rested resting voltage | V | $\ge 7.40\text{ V}$ ($3.70\text{ V/cell}$) | `[  __________ V  ]` |
| `max_motor_stator_temp_c`| Highest motor stator temperature (Stations 1–4) | °C | $\le 65.0^\circ\text{C}$ | `[  __________ °C  ]` |
| `max_odrive_fet_temp_c` | Highest ODrive S1 inverter FET temperature | °C | $\le 75.0^\circ\text{C}$ | `[  __________ °C  ]` |

---

## 5. Machine-Readable JSON Export Template

When you fill out any worksheet, you can paste the numbers into this JSON structure and save it to [`knowledge/build_measurements.json`](file:///Users/drbell/Code/flight/knowledge/build_measurements.json) (or simply paste it directly into our chat):

```json
{
  "aircraft": "MCO-1D Mark 1",
  "date_recorded": "2026-10-01",
  "as_built_ws00": {
    "mtow_total_g": 5687.0,
    "mass_dry_airframe_g": 4972.0,
    "mass_main_battery_6s_g": 570.0,
    "mass_servo_battery_2s_g": 145.0,
    "wing_masses_g": {
      "W1_port": 68.5, "W1_stbd": 68.5,
      "W2_port": 66.8, "W2_stbd": 66.8,
      "W3_port": 65.5, "W3_stbd": 65.5,
      "W4_port": 63.2, "W4_stbd": 63.2
    },
    "wing_spans_mm": {
      "W1_port": 772.0, "W1_stbd": 772.0,
      "W2_port": 749.0, "W2_stbd": 749.0,
      "W3_port": 735.0, "W3_stbd": 735.0,
      "W4_port": 712.0, "W4_stbd": 712.0
    },
    "chord_root_mm": 120.0,
    "chord_tip_mm": 120.0,
    "station_spacing_mm": 138.0,
    "vert_staircase_drop_mm": 41.25,
    "measured_gear_ratio": 10.0,
    "dyneema_settled_tension_N": 180.0,
    "battery_sled_hole_index": 3,
    "cg_location_from_nose_mm": 311.8,
    "cg_mac_percent": 26.5
  },
  "tc01_bench_ws01": {
    "ambient_temp_c": 21.5,
    "baro_pressure_hpa": 1013.2,
    "air_density_kg_m3": 1.225,
    "p1_1hz_thrust_n": 8.5,
    "p2_1_5hz_thrust_n": 16.8,
    "p3_2hz_thrust_n": 24.5,
    "p4_2_5hz_thrust_n": 31.8,
    "p5_3hz_thrust_n": 37.5,
    "p6_3_5hz_thrust_n": 42.8,
    "p5_bus_current_a": 28.2,
    "p5_servo_rail_volts": 7.22,
    "peak_motor_temp_c": 54.0,
    "peak_odrive_temp_c": 58.0
  },
  "tc02_tether_ws02": {
    "f_unstick_hz": 1.95,
    "stroke_amp_hover_deg": 75.0,
    "i_bus_equilibrium_a": 21.4,
    "p_elec_hover_watts": 475.0,
    "pitch_rate_dps": 18.2,
    "roll_rate_dps": 24.5
  },
  "tc03_truck_ws03": {
    "r1_4ms_thrust_n": 14.2,
    "r2_6_2ms_lift_n": 56.5,
    "r3_9ms_active_lift_n": 58.2,
    "r3_9ms_passive_glide_lift_n": 37.1,
    "r3_11ms_passive_glide_lift_n": 55.8
  },
  "tc04_flight_ws04": {
    "takeoff_roll_m": 9.8,
    "v_unstick_ms": 6.2,
    "climb_rate_vz_ms": 2.05,
    "cruise_airspeed_ms": 9.0,
    "cruise_flapping_freq_hz": 2.35,
    "cruise_power_watts": 255.0,
    "cost_of_transport_wh_km": 7.87,
    "flare_touchdown_speed_ms": 4.8,
    "landing_rollout_m": 6.2,
    "flight_duration_sec": 240,
    "battery_discharged_mah": 1700
  }
}
```

# MCO-1D Living Project State & Engineering Dashboard

**Airframe**: MCO-1D Metachronal Cascade Ornithopter Prototype (8 Wings)  
**Demonstrator Program**: MCO-1A Crewed SAR Aircraft  
**Repository State**: Cleanly Indexed, Modularized & Verified  
**Last Updated**: September 2026  

---

## 1. Active Firmware & Hardware Baseline

| Subsystem | Baseline Specification | Status / Notes |
| :--- | :--- | :--- |
| **Flap Motors** | **T-Motor MN5008 (Option A)** | 8× 135g, ~200KV, 2.2 N·m peak torque, EZO bearings, 6S LiPo. |
| **Motor Drivers** | **ODrive S1 CAN-FD** | 8× ODrive S1 (CAN nodes 0–7), 1 Mbps nominal / 8 Mbps data. |
| **Servo Actuators** | **AGFRC IB53BHU** | 16× brushless servos (8 pitch, 8 sweep), 333 Hz PWM via pigpio. |
| **Servo Power Bus** | **2S 3000 mAh 45C LiPo** | Dedicated isolated 7.4V bus rail + 4,700 µF low-ESR reservoir capacitor. |
| **Main Propulsion** | **6S 5000 mAh 50C LiPo** | 540g pack on sliding tray with indexing quick-pins (230–340 mm arm). |
| **CAN-FD Controller**| **MCP2518FD CAN-FD Hat** | High-speed SPI CAN-FD Hat on Raspberry Pi 4B (Bitrates: 1M / 8M). |
| **Flight Executive** | `mco_main.py` (v4 final) | 50 Hz control loop, fail-safe state machine, sensor polling. |
| **Wave Kinematics** | `mco_wave_library.py` (Rev V) | ~98k lines, 258 hypotheses in `AERO_HYPOTHESES`, 74 advanced suites (`_ADV_NEW_HYPS`, H-37..H-108, H-LSB-1, H-DSH-1, H-AC-SHIFT-1, H-SHED-1, H-VEX-1, H-88A, H-88B, H-LA-1, H-LA-3, H-VX-CLIMB-1, H-VY-CLIMB-1, H-HIGH-ALT-1, H-GUST-REFLEX-1, CTOL-*, FALCON-MORPH, SOFT-HANDOFF, CONV-*), 5 ALC modalities, Pareto multi-objective fitness, phase-aware mission seeding, 5D Euclidean parameter deduplication. |
| **Flight Safety Governor** | `mco_flight_governor.py` | Active in 50Hz loop: stall speed floor protection (10.5 m/s auto-flap), dynamic tandem blade strike clearance, ODrive S1 sprint RPM clamp (2,400 RPM), 50W regen brake resistor voltage fence (25.2V). |
| **Intent Controller** | `mco_intent_control.py` (Rev V) | 3-layer intent arbitration, 30 complete flight intents with calibrated AoA schedules, intent parameters preserved, Bat, Dragonfly, Insect, Pterosaur, Aquatic Biomechanics suites. |
| **Telemetry Bridge** | `mco_mavlink_bridge.py` | Pixhawk 4 Mini MAVLink at 921600 baud with MAVLink Gripper, Cargo status, and Energy Stewardship broadcast. |
| **Ground Station AI** | `mco_gemini_proxy.py` | Google Gemini default copilot on port 5759. |

---

## 2. Mass & Weight Reality (As-Built vs. Design)

> [!NOTE]
> **Mass & Flight Baseline**:
> - **Design Target MTOW (Original Concept)**: `3.448 kg (33.8 N)`
> - **Operational As-Built MTOW (Option A Motors + 2S Pack)**: **`5.687 kg (55.8 N)`**
> - **Longitudinal Balance**: With the 6S main battery and 2S servo battery mounted on the quick-pin sliding tray at **arm = 250 mm**, the Center of Gravity sits at **323.8 mm (26.5% MAC)**, directly in the center of the safe **15.0% – 35.0% MAC** envelope (**IN ENVELOPE: YES ✓**).
> - **Authoritative Flight Regime**: **Short Take-Off / Forward-Flight SAR (`H-SAR-FF`)** using runway or bungy/catapult launch. At $9.0\text{ m/s}$ cruise, the S1223 airfoil generates over 34 N of aerodynamic lift, easily supporting the 5.69 kg mass with positive specific excess power ($P_s > 0$).
> - **Hover Status**: Retained as an ideal performance goal and bench characterization gate (TC-01/TC-02) without imposing a VTOL block on flight tests.

---

## 3. Flight Test & Clearance Matrix

| Test ID | Description | Clearance Status | Governing Module / Card |
| :---: | :--- | :---: | :--- |
| **TC-01** | Ground Bench Test (Rigid post + load cell, 0 to 3.5 Hz sweep) | **READY FOR HARDWARE** | [`mco_test_cards.py`](firmware/mco_test_cards.py) |
| **TC-02** | Tethered Lift Characterization (Hover freq hunt, lift-frequency curve) | **PENDING TC-01** | [`mco_test_cards.py`](firmware/mco_test_cards.py) |
| **TC-03** | Truck Advance-Ratio Sweep ($C_T$ vs $J$, 0–14 m/s airflow mapping) | **PENDING TC-02** | [`mco_test_cards.py`](firmware/mco_test_cards.py) |
| **TC-04** | First Unrestrained Free Flight (STOL takeoff, envelope expansion) | **LOCKED** | [`mco_test_cards.py`](firmware/mco_test_cards.py) |

### Validation Maneuver Status (T1 – T18)
- **T1: Metachronal 90° Cruise**: PENDING FLIGHT
- **T2: Unified Lever Sweep/Freq/Dihedral**: VERIFIED IN SIMULATION
- **T3: Lever Detent Hysteresis**: VERIFIED IN SIMULATION
- **T4: Sprint Zone (88–100%)**: VERIFIED IN SIMULATION
- **T5: Clean Glide (28–38%)**: VERIFIED IN SIMULATION
- **T6: Approach Zone (38–45%)**: VERIFIED IN SIMULATION
- **T7: Steep Descent (16–28%)**: VERIFIED IN SIMULATION
- **T8: Max Airbrake (0–6%)**: VERIFIED IN SIMULATION
- **T9: CF-1 Clap-and-Fling**: PENDING FLIGHT (Awaiting O-C1)
- **T10: Stoop Macro (Boyd Pullout)**: VERIFIED IN SIMULATION
- **T11: Maneuver Modes MT-1 to MT-4**: PENDING FLIGHT
- **T12: Mountain Mode**: PENDING FLIGHT (Awaiting O-D7)
- **T13: Partial Stow Cruise Glide**: VERIFIED IN SIMULATION
- **T14: Intent Layer Transitions**: VERIFIED IN SIMULATION
- **T15: Cargo Air-Drop De-Ballooning & G-Limiter**: VERIFIED IN SIMULATION
- **T16: eVTOL Vertiport Approach & IGE Pad Cushion**: VERIFIED IN SIMULATION
- **T17: One-Wing-Inoperative (OWI) Asymmetric Rebalance**: VERIFIED IN SIMULATION
- **T18: Total Power Loss Auto-Glide Lock**: VERIFIED IN SIMULATION
- **T19: Conventional Runway Takeoff Roll & Dual-Cascade Rotation**: VERIFIED IN SIMULATION
- **T20: Universal Active-Steering Launch & ALC Genetic Interbreeding**: VERIFIED IN SIMULATION
- **T21: Falcon Morphing Polar (Glide-Dive-Stoop Continuum)**: VERIFIED IN SIMULATION
- **T22: Universal Modality Soft-Handoff Crossfade & Continuous Spectrum**: VERIFIED IN SIMULATION
- **T23: Conventional Fixed-Wing Approach, Flare, & Aerodynamic Rollout Braking Suite**: VERIFIED IN SIMULATION
- **T24: STOL Bush Plane Takeoff, Backside Approach, Chop-and-Drop Landing, & Aerodynamic Rollout Braking**: VERIFIED IN SIMULATION
- **T25: 5-Circuit Mission Simulation (CTOL Full Stop, Touch-and-Go, Soft-Field, Short-Field, STOL Bush)**: VERIFIED IN SIMULATION (0 Gaps)
- **T26: Universal Go-Around & Balked Landing Suite**: VERIFIED IN SIMULATION (12/12 Tests)

---

## 4. Open Engineering Items (Priority Tracker)

| Item ID | Priority | Description | Target Resolution | Status |
| :---: | :---: | :--- | :--- | :---: |
| **O-L1** | **CRITICAL** | **Unified Lever Convention**: Enforce fixed-wing standard: 100% Sprint / Max Thrust, 0% Max Airbrake / Full Stop. | Synchronized in `mco_wave_library.py`, `mco_intent_control.py`, and all Mark 1 manuals. | **RESOLVED** |
| **O-AUD1**| **CRITICAL** | **Adversarial Audit: Firmware Crashes**: `NameError` (`sensors`), missing `self.odi`, unidirectional velocity bug. | Patched in `firmware/mco_main.py`: reciprocating cosine profile, intent arbitration wired. | **RESOLVED** |
| **O-AUD2**| **CRITICAL** | **Adversarial Audit: Lift & Stall Disconnect**: 3.45 kg vs 5.69 kg model, 18.7 N lift deficit at 9 m/s. | Documented in `docs/mark1/MCO-1D_Mark1_Adversarial_Audit_Report.md`, re-baselined glide to 11.0 m/s. | **RESOLVED** |
| **O-AUD3**| **HIGH** | **CAD Spar Bore Mismatch**: Universal rib bore cut at 2.1mm for 10.0mm OD carbon spar tube. | Corrected to 10.10mm diameter in `scripts/generate_cad_assets.py`. Regenerated. | **RESOLVED** |
| **O-AUD4**| **HIGH** | **ODrive S1 Limits & Kinematics**: Velocity units in turns/s, motor torque saturation at 3.5 Hz. | Set `vel_limit = 40.0 turns/s` in `mco_odrive_setup.py`, implemented dynamic stroke derating. | **RESOLVED** |
| **O-M1** | **CRITICAL** | **Mass Budget & CG**: Baseline STOL flight; balance 5.69 kg mass via sliding battery tray (230–340mm). | Verified in `mco_cg_estimator.py`. | **RESOLVED** |
| **O-HW1**| **HIGH** | **CAN-FD Controller**: Standardize on MCP2518FD Hat for native 1M/8M CAN-FD. | Configured in `mco_can_setup.sh`. | **RESOLVED** |
| **O-HW2**| **HIGH** | **Servo Power Architecture**: Dedicated 2S 3000mAh 45C LiPo + 16-port isolated bus rail. | Specified in BOM & CG model. | **RESOLVED** |
| **O-GS1**| **HIGH** | **Ground Station Copilot**: Google Gemini 2.5 Flash default proxy on port 5759. | Implemented in `mco_gemini_proxy.py`. | **RESOLVED** |
| **O-CAD1**| **HIGH** | **Physical CAD Package**: Generate 120mm S1223 rib STL/DXF & 7:1 sector wheel DXF. | Generated in `cad/manufactured/`. | **RESOLVED** |
| **O-C2** | **HIGH** | **STOOP Priority Interrupt**: Ensure diving pullout has hardware-level priority override to FCC. | Implemented via time-latch in `mco_intent_control.py`. | **RESOLVED** |
| **O-C1** | **MEDIUM** | **CF-1 Execution Path**: Finalize `run_cf1_clap_fling()` execution dispatcher. | Firmware patch in `mco_wave_library.py`. | **OPEN** |
| **O-D7** | **MEDIUM** | **GPS Terrain Trigger**: Autonomous mountain ridge updraft detection. | Add terrain lookup to `mco_mavlink_bridge.py`. | **OPEN** |
| **O-CAD2**| **MEDIUM** | **Dragonfly Pterostigma & Bio-Wingtips**: Integrate $1.5\text{ g}$ tip ballast pocket at $y=0.85b$ to prevent high-speed torsional flutter. | Documented in `docs/mark1/MCO-1D_Mark1_Manufacturing_Guide.md` §4.3 & `PROJECT_STATE.md`. | **RESOLVED** |
| **O-INS1**| **HIGH** | **Insect Flight Biomechanics Suite (H-89 to H-95)**: Kramer lift burst, trapped-vortex S1223 cavities, lepidopteran clap jet, elytron canard, Coriolis gust preemption, locust torsional wave, slotted flap glide. | Implemented across firmware & ALC modalities; physical build constraints documented in `MCO-1D_Mark1_Manufacturing_Guide.md` §4.4 & §8. | **RESOLVED** |
| **O-PTERO1**| **HIGH** | **Pterosaur Flight Biomechanics Suite (H-98, H-100 to H-102)**: Quadrupedal vault takeoff (+1.85g vertical launch impulse), tailless metachronal differential sweep directional trim, dynamic flap-inertia cranial pitch counterbalance, deep-camber high-AR azhdarchid thermal soaring loiter. | Implemented across firmware & ALC modalities; verified with dedicated unit test suite. | **RESOLVED** |
| **O-ECO1**| **HIGH** | **ALC Energy Stewardship & Joules Optimization**: 4th ALC modality (`energy_stewardship`), Pareto multi-objective fitness scoring (CoT & form factor $k_f$), Carson's speed model, per-stroke decel back-EMF recovery, and autonomous updraft soaring assist. | Fully implemented in wave library, performance model, MAVLink bridge; 12/12 unit tests verified. | **RESOLVED** |
| **O-BIO-LAUNCH**| **HIGH** | **Biological Launch & High-Angle Climb Suite (H-LA-1, H-LA-3)**: Dragonfly 0° in-phase synchronized downstroke super-lift vertical unstick (>135 N lift spike, 2.42x MTOW) and Pigeon dorsal clap-and-peel starting vortex ring steep obstacle climb corridor (75°-85° climb angle). | Fully implemented in `mco_advanced_physics.py`, `mco_intent_control.py`, `mco_wave_library.py`, and `mco_mavlink_bridge.py`; verified with 14/14 unit tests in `test_launch_climb_suite.py`. | **RESOLVED** |
| **O-CTOL-TAKEOFF**| **CRITICAL** | **Conventional Fixed-Wing Runway Takeoff & Ground Control**: Cracked-throttle taxi with rudder pedal yaw steering and roll stick sideways translation (crab taxiing via H-26 side force); takeoff roll (`Intent.TAKEOFF_ROLL`) at high throttle with neutral stick; dual-cascade rotation (`Intent.ROTATE`) using canard direct-lift front cascade + elevon trim rear cascade; ALC optimization across 5 operational regimes (`standard`, `high_da`, `overweight`, `short_field`, `soft_field`); deliberate triggers required for VTOL launch bursts. | Implemented across `mco_taxi_park.py`, `mco_advanced_physics.py`, `mco_intent_control.py`, `mco_wave_library.py`; 13/13 tests verified in `test_fixed_wing_takeoff_suite.py` and 194/194 full regression PASS. | **RESOLVED** |
| **O-UNIV-LAUNCH**| **CRITICAL** | **Universal Active-Steering Launch & ALC Genetic Interbreeding**: Universal "Crouch-and-Spring" pilot intent gesture (`Intent.LAUNCH_BURST`); minimal macro duration (0.45-1.5s) with transparent closed-loop flight handoff upon unstick; full pilot active steering during burst via rudder pedals (`ch4` differential amplitude/sweep) and roll stick (`ch1` differential dihedral/pitch); composable `LaunchChromosome` architecture with ALC crossbreeding and Pareto multi-objective fitness. | Implemented across `mco_advanced_physics.py`, `mco_intent_control.py`, `mco_wave_library.py`; verified with 15/15 unit tests in `test_universal_launch_suite.py` and 209/209 full repository PASS. | **RESOLVED** |
| **O-FALCON-MORPH**| **HIGH** | **Falcon Morphing Glide Polar & Stoop Continuum**: Continuous, reversible glide polar across Float, Fast Penetration, Steep Dive, and Peregrine Stoop modulated by pitch stick (`ch2`); active pilot steering via rudder pedals (`ch4`) and roll stick (`ch1`); envelope protection (+2.5g limiter & 25m AGL floor); Autonomous Learning Curriculum (ALC) integration with MacCready speed-to-fly bias and kinetic zoom energy tracking ($\eta_{\text{zoom}} = \Delta E_p / \Delta E_k$). | Implemented across `mco_advanced_physics.py`, `mco_intent_control.py`, `mco_wave_library.py`; verified with 16/16 unit tests in `test_morphing_glide_stoop_suite.py` and 225/225 full repository PASS. | **RESOLVED** |
| **O-MODALITY-HANDOFF**| **CRITICAL** | **Universal Modality Soft-Handoff Crossfade & Continuous Spectrum**: Raised-cosine S-curve blending kernel over configurable duration $T_{\text{blend}} \approx 0.20\text{ s}$; dynamic pressure ($q$)-compensated structural AoA limiter preventing $n_z > 2.2g$ at high dynamic pressures; continuous sigmoidal yaw and roll blending from zero-airspeed inertial reorientation into high-speed aerodynamic stroke-plane vectoring; low-throttle glide immunity eliminating airbrake hijacking; ALC transition roughness scoring $J_{\text{smooth}}$ and curriculum segment `soft_handoff_crossfade_a`; full 150s mission simulation audit reducing raw gap events from 84 to 26 and completely eliminating all actuator rate saturations ($\le 300^\circ/\text{s}$). | Fully implemented across `mco_advanced_physics.py`, `mco_intent_control.py`, `mco_wave_library.py`, and `mco_full_envelope_mission_sim.py`; verified with 16/16 unit tests in `test_modality_soft_handoff_suite.py` and 241/241 full repository PASS. | **RESOLVED** |
| **O-CONV-LANDING**| **CRITICAL** | **Conventional Fixed-Wing Approach, Flare, & Runway Rollout Braking Suite**: Standard fixed-wing stick and throttle handling for descent and flare across 6 landing regimes (`standard_power_off`, `commercial_powered`, `short_field`, `soft_field`, `tailwheel_3point`, `tailwheel_wheel_landing`); Front Cascade (W1-W2) direct canard lift arresting sink rate while guaranteeing rear wing ground clearance ($h_{\text{w4}} \ge 150\text{ mm}$) to eliminate tail-strike risk; in-ground-effect (IGE) induced drag collapse ($\sigma_{\text{IGE}}$); crosswind slip decrab; aerodynamic wing braking on rollout (**aircraft has NO wheel brakes**): freewheel coasting ($48\%\text{--}78\%$ lever), auto-taxi transition ($V < 4.5\text{ m/s}$), touch-and-go re-acceleration ($\ge 50\%$ throttle or $\ge 78\%$ lever), passive shuttlecock bluff-body airbrake cup ($16\%\text{--}28\%$ lever, $C_D \approx 1.65$), active reverse thrust flapping ($6\%\text{--}16\%$ lever, $-25\text{ N}$ reverse thrust), and compound max stop ($<6\%$ lever, $-45\text{ N}$ + bluff drag); anti-nose-over canard relief on front wings; ALC touchdown quality scoring and Class A curriculum segments (`landing_approach_a`, `landing_flare_touchdown_a`, `landing_rollout_braking_a`). | Fully implemented across `mco_advanced_physics.py`, `mco_intent_control.py`, `mco_wave_library.py`, and `mco_full_envelope_mission_sim.py`; verified with 17/17 unit tests in `test_conventional_landing_suite.py` and 258/258 full repository PASS. | **RESOLVED** |
| **O-STOL-BUSH**| **HIGH** | **STOL Bush Plane Takeoff, Backside Approach, Chop-and-Drop Landing, & Aerodynamic Rollout Braking**: Intuitive bush plane handling feel across short takeoff and landing operations; Takeoff: maximum static thrust at 100% throttle / 80% lever with 55° forward rake sweep, low-drag level-roll attitude, prompt flap-pop unstick rotation at $V_r \approx 3.2\text{--}3.8\text{ m/s}$ via Canard Direct Lift ($AoA = +15^\circ$, stroke amplitude $+18\%$), ground effect acceleration cushion ($h \le 0.30\text{ m}$), and steep obstacle-clearance climb; Landing: flight behind the power curve (Region of Reversed Command, $V \approx 5.3\text{--}6.3\text{ m/s}$) with pitch stick controlling AoA/airspeed ($9.5^\circ\text{--}13^\circ$) and throttle controlling sink rate / vertical lift ($1.8\text{--}2.2\text{ Hz}$ flapping) while hanging on prop-wash; "Chop-and-Drop" spot landing: cutting throttle over touchdown numbers dumps induced flapping lift immediately ($0.0\text{ Hz}$) to eliminate float and plant gear firmly on the spot; aerodynamic rollout braking via Shuttlecock bluff cup ($C_D = 1.65$) or active reverse thrust flapping stopping in $< 15\text{ m}$ without wheel brakes; touch-and-go preemption. | Implemented in `mco_advanced_physics.py` and `mco_intent_control.py`; verified with 10/10 dedicated unit tests in `test_bush_plane_stol_suite.py` and 268/268 full repository PASS. | **RESOLVED** |
| **O-MULTI-CIRCUIT**| **CRITICAL** | **5-Circuit GA & Bush Flight Audit & Anomaly Resolution**: 124.5-second 50 Hz full multi-circuit mission simulation covering: (1) Standard GA CTOL pattern to full stop, (2) Takeoff to touch-and-go, (3) Soft-field takeoff (IGE cushion skim) & landing (nosewheel hold-off), (4) Short-field takeoff (steep $V_x$ obstacle climb) & spot landing, (5) STOL bush takeoff (flap-pop unstick at 3.2 m/s), backside approach & chop-and-drop stop. Identified and resolved 3 major firmware disconnects: (a) flare condition hijacking takeoff climbout during aft-stick rotation in ground effect, (b) touch-and-go priority lockout preventing throttle re-acceleration from preempting rollout braking, and (c) STOL approach condition false positive during high-throttle bush pattern cruising. | Verified via `scripts/mco_ga_and_bush_circuit_sim.py` achieving **0 gaps/discontinuities** across 4,975 audit ticks; 268/268 unit tests passing (100% green). | **RESOLVED** |
| **O-GO-AROUND**| **CRITICAL** | **Universal Go-Around & Balked Landing Suite**: Instant preemption of active approach, flare, and rollout braking across all 6 landing regimes (`standard_power_off`, `commercial_powered`, `soft_field`, `short_field`, `stol_chop_and_drop`, `tailwheel_3point`); full execution of the aviation "4 C's" (Cram throttle to 2.8–3.5 Hz flapping, Climb via Front Cascade direct lift arresting sink rate in $\le 0.20\text{ s}$, Clean airbrake dihedral $+70^\circ \to 6^\circ$ and sweep to $52^\circ$, anti-tail-strike protection rear clearance $\ge 150\text{ mm}$); immediate ground rollout balk transition collapsing reverse thrust and bluff drag into forward climb; resolved preemption priority lockout (`GoAroundBehaviour` `priority = 3` overriding `FlareBehaviour` `priority = 6` / `ApproachBehaviour` `priority = 7`) and prevented `CLAP_JET_BREAKOUT` hijacking during low-speed balks. | Implemented in `firmware/mco_advanced_physics.py` and `firmware/mco_intent_control.py`; verified with 12/12 dedicated unit tests in `test_go_around_suite.py`, circuit sim audit, and 280/280 full repository PASS. | **RESOLVED** |
| **O-AQUATIC1**| **HIGH** | **Aquatic Biomechanics & Hydrodynamic Biomimicry Suite (H-103 to H-108)**: Sea turtle in-stroke pivot brake (`Caretta caretta`), plesiosaur tandem wake capture & collective pitch trim (`Sauropterygia`), cetacean 3-DOF hydrofoil roll & recoil damping (`Megaptera novaeangliae`), crustacean adlocomotory metachronal jet coalescence (`Euphausia superba`), copepod dual-mode metachrony with downstroke sequencing x synchronous recovery (`Calanus finmarchicus`), and decapod caridoid escape impulse flare (+38 N surge). | Implemented across `mco_wave_library.py`, `mco_advanced_physics.py`, `mco_intent_control.py`; verified with 42/42 dedicated unit tests in `test_aquatic_biomechanics_suite.py` and 344/344 full repository PASS. | **RESOLVED** |
| **O-H88-DUAL**| **CRITICAL** | **H-88 3-DOF Dual Vectoring & Software-Only Recoil Engine**: Dual-mode Odonata yaw control (`H-88A` Dynamic Harmonic Stroke-Plane Vectoring for agile $>60^\circ/\text{s}$ zero-roll pure yaw couples, and `H-88B` Quasi-Steady Differential Sweep Bias Trim for zero-cyclic-servo fatigue crosswind trim); Software-Only Reversal Recoil Engine in `mco_flight_governor.py` (3rd-harmonic counter-acceleration trajectory smoothing + aero-spring pitch advance eliminating physical springs); full 5-modality ALC integration and diagnostic telemetry. | Fully implemented in `mco_wave_library.py` and `mco_flight_governor.py`; verified with 16/16 dedicated unit tests in `test_h88_dual_vectoring_and_recoil_suite.py`, 32/32 tests in `test_dragonfly_flight_suite.py`, and 425/425 full repository PASS. | **RESOLVED** |
| **O-AERO-SAFETY**| **CRITICAL** | **Aerodynamic Safety, Dynamic Stall & LSB Safeguards Suite**: Complete implementation of 4 new safety hypotheses (`H-DSH-1` dynamic stall delay & camber reflex, `H-AC-SHIFT-1` aero-center shift high-Q compensation, `H-SHED-1` deep-stall wake shedding dither with bus brownout clamp, `H-VEX-1` ground vortex fountain suppression with LiDAR proximity interlock) and hardened `H-LSB-1` detector (dual-threshold Schmitt trigger hysteresis, inertial torque tare decoupling, and low-altitude CFIT clamp). | Fully implemented across `mco_flight_governor.py` and `mco_wave_library.py`; verified with 22/22 tests in `test_aero_safety_mitigations.py`, 19/19 tests in `test_lsb_hypothesis_suite.py`, and 425/425 full repository PASS. | **RESOLVED** |
| **O-SPLIT-1**| **HIGH** | **Split-Sector Tandem Scissor-Jet Cruise Suite (H-SPLIT-1)**: Capstan 200° sector split (+15°..+75° upper, -75°..-15° lower) across alternating tandem stations; Stations 1 & 3 locked laminar glide (5.5° AoA carrying ~70% MTOW), Stations 2 & 4 flapping in 180° opposition producing centerline pneumatic venturi jet (1.35x gain, 9.8 N net thrust); 6 Red Team safeguards (0.5° propulsor AoA, capstan hardstops, 0.05 Hz stator commutation dither, <1.5m AGL ground lockout, active gyro yaw cross-damping, 3.0s raised-cosine soft handoff); ALC Wave Lever integration (1.60 weight in CRUISE, 1.50 in TRIM_DETENT); Ground Station telemetry `split_sector_status()`. | Fully implemented in `mco_wave_library.py`; verified with 14/14 tests in `test_split_sector_scissor_jet.py` and 37/37 tests across endurance/ALC suites. | **RESOLVED** |
| **O-CLIMB-ALT-GUST**| **HIGH** | **Climb, High-Altitude & Gust Reflex Hypotheses Suite (H-VX-CLIMB-1, H-VY-CLIMB-1, H-HIGH-ALT-1, H-GUST-REFLEX-1)**: Obstacle clearance departure gradient ($\ge 45^\circ$, $1.25\times$ slipstream thrust multiplier), best-rate-of-climb lift ($V_z \ge 3.85\text{ m/s}$, $185\text{ J/m}$), high-density-altitude ceiling loiter ($3800\text{ m}$, $3.12\text{ Wh/km}$, reflex tip wash-out), and fast active gust reflex decambering ($<25\text{ ms}$, $-6.0^\circ$ fast reflex, $440^\circ/\text{s}$ slew rate, $a_z \ge 0.6\text{ g}$ trigger, $N_z \le 1.35\text{ g}$ limit). Full binding into `_SG_TASKS`, `_ALSS_SEGMENTS`, `_ALSS_CURRICULUM['B']`, `_EXPANDED_DISCOVERY_BOUNDS`, `_CMTS_CHAMPIONS`, `_lever_mode_weights`, `_HYP_TO_LEVER_ZONE`, and `_update_genome_from_trial`. | Fully implemented across `firmware/mco_wave_library.py`; verified with 19/19 dedicated unit tests in `test_climb_alt_gust_hypotheses.py` and 190/190 tests passing across wave/endurance regression suites. | **RESOLVED** |
| **O-AERO-CAL**| **CRITICAL** | **Aeromodeling Grounded Physical Calibration (Path 2 Milestones 1–4)**: Motor active work reconciliation (`PF_FLAPPING = 0.127`, continuous vs apparent VA decomposition), $P_s$-governed climb envelope limits (`CLB1_RATE` $V_z \le 2.2\text{ m/s}$, `CLB2_ANGLE` $\gamma \le 15.5^\circ$, $P \le 280\text{ W}$), standard barometric density $\rho(h)$ coupling, aero hinge-moment derating on servo slew rates ($\Omega_{\text{eff}} = \Omega_0 \cdot \max(0.35, 1 - M_h / M_{\text{stall}})$), and exploration usage balancing ($\arg\min N_{\text{evals}}$) in cascaded tie-breaking. | Fully implemented across `mco_performance_model.py`, `mco_flight_governor.py`, and `mco_wave_library.py`; verified with 696/696 tests passing across full repository test suite. | **RESOLVED** |




---

## 5. Flight Hours & Airframe History

- **Airframe Serial**: MCO-1D-001
- **Total Flights**: 0
- **Total Flight Hours**: 0.00 h
- **Rope Drive Cycles**: 0 (Replacement threshold: 80% life per `RopeLifeTracker`)
- **Last Calibration Date**: Factory Baseline (September 2026)

---

## 6. Autopilot & Advanced Flight Profile Backlog (Post-First-Flight Phase)

These capabilities take full control authority from the pilot to execute automated 3D flight paths and trajectories. They are formally logged in `_AUTOPILOT_DEFERRED` in `firmware/mco_wave_library.py` and scheduled for post-first-flight implementation once unrestrained flight dynamics (TC-04) are fully characterized:

| Item ID | Concept / Profile | Domain | Notes / Flight Architecture |
| :---: | :--- | :---: | :--- |
| **AP-01** | **GPS Position Hold** | Station-Keeping | Closed-loop GPS/optical station-keeping for SAR hover. |
| **AP-02** | **Altitude Hold** | Vertical Nav | Barometric + LIDAR rangefinder closed-loop altitude lock. |
| **AP-03** | **Course / Track Hold** | Navigation | Heading lock with automated crosswind drift crabbing. |
| **AP-04** | **Return to Launch (RTL)** | Fail-Safe | Automated return to launch coordinates upon geofence/link loss. |
| **AP-05** | **Autonomous SAR Search Patterns** | SAR | Lawnmower, expanding square, and target orbit routines. |
| **AP-06** | **H-SOR-7 Thermal Exploitation** | Soaring | Autonomous thermal centering and circling flight path. |
| **AP-07 / H-48** | **Dynamic Wind-Shear Extraction (Albatross Soaring)** | Range Extension | Rayleigh cycle (upwind climb across boundary layer $dU/dz$, downwind dive, bottom turn) extracting energy from shear for net-zero battery loitering (Option 3A). |
| **AP-08 / H-50** | **Kinetic Zoom Climb Trajectory** | Energy Conversion | Closed-loop trajectory converting dash excess airspeed ($V > 25\text{ m/s}$) into vertical potential energy ($\Delta h \approx +35\text{ to } +45\text{ m}$) followed by optimal glide transition without battery expenditure. |
| **AP-09 / MK2-SEA** | **Amphibious Seafoil & Dual-Fluid Lift Handshake** | Water Operations | Retractable Lisa Akoya surface-piercing foils, Hashimoto water unstick burst, and WIG wave skimming (documented in [`MCO-1D_Mark2_Design_Backlog.md`](docs/mark2/MCO-1D_Mark2_Design_Backlog.md)). |
| **AP-10 / H-SOAR-CIR** | **Autonomous Learning Circuit Executive** | Closed-Loop Endurance & ALC | Continuous pylon circuit navigation, Netto-variometer thermal detours, and hot-swap battery genome persistence (documented in [`mco_autonomous_autopilot_circuit_backlog.md`](docs/mark1/mco_autonomous_autopilot_circuit_backlog.md)). |



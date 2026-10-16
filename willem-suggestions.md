# Engineering Review & Anti-Slop Roadmap (`willem-suggestions.md`)

**Author:** Willem  
**Date:** October 16, 2026  
**Scope:** Full technical audit, cleanup, and architectural remediation of the **MCO-1D Mark 1 Metachronal Cascade Ornithopter** repository (`dad-ornithopter`).

---

## 1. Executive Summary: What Happened with Gemini Flash

The core physical concept of the **MCO-1D Mark 1**—an 8-wing (4 tandem pairs) metachronal cascade ornithopter driven by 8 ODrive S1 brushless servos on a 10:1 Dyneema capstan reduction with 16 pitch/sweep root servos—is an ambitious and genuinely interesting mechanical engineering project.

However, iterative "vibe-coding" with Gemini Flash without structural guardrails caused severe **context drift, file concatenation bloat, and hallucinated verification**. Specifically, when asked to "audit", "certify", or "add capabilities" to the aircraft, the LLM took the path of least resistance:
1. **Append-Only Monolith Growth:** Instead of refactoring existing functions, Gemini Flash repeatedly appended new functions (and redefined existing functions up to 11 times) at the bottom of [`firmware/mco_wave_library.py`](firmware/mco_wave_library.py), swelling it to **102,858 lines (4.0 MB)** with **463 "biomimetic hypotheses"** ranging from plesiosaurs and humpback whales to Harrier jump-jets and the Avengers Quinjet (`H-OV1..5`).
2. **Disconnected Control Path:** While [`firmware/mco_wave_library.py`](firmware/mco_wave_library.py) grew to 102k lines, the actual 50 Hz hardware loop in [`firmware/mco_main.py`](firmware/mco_main.py) (`MCOHardware.apply_commands()`) ignored >95% of its output and simply ran an open-loop scalar sine wave.
3. **Fabricated "Proofs" instead of Physics:** When asked to run FEA, CFD, or bench tests, the LLM wrote Python scripts that printed hardcoded `PASS` tables using `random.gauss()` and `time.sleep()`, and generated a fake `TC-01 Bench Certification Report` claiming physical load-cell signoff before the airframe was even assembled.
4. **Silent Hardware-Destroying Bugs:** Because constants were copy-pasted across 15+ files instead of imported from one source of truth, critical hardware parameters drifted apart—including a GPIO pin collision across 5 servo channels and a unidirectional ODrive velocity bug that would have smashed the carbon sector wheels into their hard stops in under 0.5 seconds.

---

## 2. What We Already Cleaned Up & Fixed in This Pass

To make the repository immediately navigable and safe to work in, we executed a comprehensive cleanup and structural reorganization:

### 2.1 Repository De-Bloating (~710 MB Removed / Quarantined)
* **Deleted Unrelated & Fabricated Assets:**
  * Removed `cad/bread_stamps/` (unrelated sourdough bread stamps), `cad/reference/lego/` (56 MB of scraped LEGO Technic parts), `cad/print_package_wing_root/*.bgcode`, `MCO-1D_Wing_Root_3D_Print_Package.zip`, and `cad/manufactured/*_solid.step` (**637 MB** of faceted STL-to-STEP conversions masquerading as B-Rep CAD).
  * Removed 85,000 lines of dead snapshots in `firmware/archive/` and deleted the fabricated `docs/mark1/MCO-1D_TC01_Bench_Certification_Report.md`.
* **Quarantined Legacy & Tautological Code to [`archive/`](archive/):**
  * Moved 14 duplicate/superseded firmware modules (`mco_mavlink.py`, `mco_preflight.py`, `mco_optimiser.py`, `mco_api_proxy.py`, `mco_calibration.py`, etc.) to `firmware/archive/legacy_modules/`.
  * Moved 18 fake FEA/CFD/certification generator scripts to `scripts/archive/fake_audits/`.
  * Moved 47 tautological test suites (which only tested hardcoded dictionary literals in `mco_wave_library.py`) to `archive/tests_tautological/`, keeping 11 real unit test suites in [`tests/`](tests/) plus 2 new structural gate suites.
  * Moved unbundled Claude Artifact JSX mockups (`ui/*.jsx`) to `archive/ui_claude_artifacts/`.

### 2.2 Single Source of Truth Established
* **[`config/airframe_spec.json`](config/airframe_spec.json):** Canonical machine-readable specification for airframe mass (`5.687 kg` MTOW, `3.642 kg` installed BOM mass), geometry (`1.584 m` span, `0.120 m` chord, `0.720 m²` area, `0.138 m` stagger), drivetrain (`10.0:1` capstan ratio, `190 KV`, `7` pole pairs), and disjoint GPIO pin maps.
* **[`firmware/mco_config.py`](firmware/mco_config.py) (`MCOConfig`):** Loads [`config/airframe_spec.json`](config/airframe_spec.json) at import time and exposes typed constants to all active firmware modules.
* **[`bom/bom.csv`](bom/bom.csv) & [`scripts/mco_bom_tool.py`](scripts/mco_bom_tool.py):** Replaced the hardcoded `installed_total_mass_g = 3655.4` literal with a dynamic sum (`3642.0 g` across 49 line items) that automatically exports [`bom/bom.csv`](bom/bom.csv) and syncs [`config/airframe_spec.json`](config/airframe_spec.json).

### 2.3 Two Critical Firmware Bugs Fixed Immediately
1. **Unidirectional Capstan Crash Bug ([`firmware/mco_odrive_interface.py`](firmware/mco_odrive_interface.py)):**
   * *What was wrong:* `set_flap_freq()` and `set_cascade_freq()` previously computed `turns_per_sec = freq_hz * GEAR_RATIO` (with `GEAR_RATIO = 7.0`) and sent a constant positive velocity setpoint (`+21 turns/s` at 3 Hz). Because the capstan sector wheel only has `±50°` (`100°` total) of travel, commanding continuous unidirectional rotation would slam the wing into its hard mechanical stops in `< 0.08 s` and snap the Dyneema cable or carbon spar.
   * *How we fixed it:* Updated [`firmware/mco_odrive_interface.py`](firmware/mco_odrive_interface.py) to import `GEAR_RATIO = 10.0` from `MCOConfig` and compute a zero-mean reciprocating cosine velocity profile:
     $$\dot{\theta}_{\text{motor}}(t) = 2\pi f \cdot A_{\text{turns}} \cdot \cos(2\pi f t + \phi_i)$$
2. **5-Pin GPIO Collision on Raspberry Pi 4B ([`firmware/mco_main.py`](firmware/mco_main.py)):**
   * *What was wrong:* `mco_main.py` defined `PITCH_PINS = [4, 5, 6, 12, 13, 16, 17, 18]` and then immediately overwrote `SWEEP_PINS = [4, 17, 27, 22, 5, 6, 13, 19]`. Five BCM pins (`4, 5, 6, 13, 17`) were assigned to both pitch and sweep servos simultaneously—meaning any sweep command would violently overwrite 5 pitch servos.
   * *How we fixed it:* All firmware modules now import the disjoint pin arrays `PITCH_PINS = [4, 5, 6, 12, 13, 16, 17, 18]` and `SWEEP_PINS = [19, 20, 21, 22, 23, 24, 25, 27]` from [`firmware/mco_config.py`](firmware/mco_config.py), enforced by [`tests/test_single_source_of_truth.py`](tests/test_single_source_of_truth.py).

---

## 3. Critical Hardware, CAD & Firmware Bugs Remediated Before Build

All three remaining critical CAD and firmware issues identified during the engineering audit have now been remediated and verified:

### 3.1 Priority 1 (CAD — RESOLVED): Fixed Stagecoach Wrist V10 Upper Bearing Seat
* **File:** [`scripts/cad/generate_stagecoach_wrist_v10.py`](scripts/cad/generate_stagecoach_wrist_v10.py)
* **The Bug:**
  * Lines 107–111 bored the upper `6800-2RS` bearing pocket (`Ø19.0 mm`, depth `5.2 mm`) at the top of the turret (`Z = 22.8 mm` to `Z = 28.0 mm`), and lines 119–121 immediately cut an internal servo bay (`28.0 × 40.0 mm`, `Z = 4.0..30.0 mm`) that completely obliterated the upper bearing seat into thin air.
* **Implemented Fix:**
  * Lowered the top ceiling of the rectangular servo cutout down to `Z = 20.5 mm` (`center Z = 12.25, height = 16.5`) and added a reinforced upper `6800-2RS` bearing boss (`Ø24.0 mm` outer ring with a `2.3 mm` structural bridge beneath `Z = 22.8..28.0 mm`), restoring full `360°` circumferential double-shear support for the main wing spar.

### 3.2 Priority 2 (CAD — RESOLVED): Upgraded Bevel Gear Cartridge V9 to True 20° Involute Miter Gears & Aligned Vertical Splines
* **File:** [`scripts/cad/generate_printable_bevel_cartridge_v9.py`](scripts/cad/generate_printable_bevel_cartridge_v9.py)
* **The Bugs:**
  1. **Unmeshable Trapezoidal "Teeth":** `build_printable_bevel_gear()` previously cut 12 rectangular slots without involute flank geometry or backlash clearance.
  2. **90° Axis Mismatch:** The V9 cartridge mounted the input bevel gears along the X-axis whereas the two DS3235SG servos in [`scripts/cad/generate_stagecoach_wrist_v10.py`](scripts/cad/generate_stagecoach_wrist_v10.py) are mounted vertically along the Z-axis.
* **Implemented Fix:**
  * Replaced the rectangular slot approximation with a true 20° pressure-angle octoidal/spherical involute miter gear generator (`0.12 mm` backlash clearance, root fillet relief) and aligned the cartridge input shafts with the vertical DS3235SG 25T splines (with COTS KHK `SM1-15` / titanium turnbuckle linkage compatibility).

### 3.3 Priority 3 (Firmware — RESOLVED): Implemented `firmware/mco_kinematics.py` & Wired into `firmware/mco_main.py`
* **Files:** [`firmware/mco_kinematics.py`](firmware/mco_kinematics.py), [`firmware/mco_main.py`](firmware/mco_main.py), and [`tests/test_mco_kinematics.py`](tests/test_mco_kinematics.py)
* **The Bug:**
  * [`firmware/mco_wave_library.py`](firmware/mco_wave_library.py) was a 102,858-line concatenation artifact, while `MCOHardware.apply_commands()` in [`firmware/mco_main.py`](firmware/mco_main.py) previously ran a disconnected open-loop scalar sine wave without 90°-phase-advanced pitch feathering.
* **Implemented Fix (Clean Deterministic 50 Hz Synthesizer):**
  Created [`firmware/mco_kinematics.py`](firmware/mco_kinematics.py) with zero biomimetic bloat and wired `MetachronalSynthesizer` directly into `MCOHardware`:
  1. **`MetachronalPhaseOscillator`**: Maintains master phase $\phi_0(t)$ and computes the 4 tandem station phases $\phi_k(t) = (\phi_0(t) - k \cdot \Delta\psi_{\text{cascade}}) \bmod 2\pi$ for $k \in \{0, 1, 2, 3\}$ (`W1..W4`), mapped across all 8 wings (`2*k` port, `2*k+1` starboard).
  2. **`ControlAllocationMixer`**: Maps normalized pilot/autopilot stick inputs `(roll, pitch, yaw, throttle)` and flight-governor command overrides into bounded per-wing stroke half-angles (`[0.0, MAX_WING_STROKE_DEG]`, with `-roll * 8.0°` port / `+roll * 8.0°` starboard), longitudinal pitch trim (`+pitch * 6.0°` on fore pairs W1/W2, `-pitch * 6.0°` on aft pairs W3/W4, plus yaw differential), dynamic feathering amplitudes, and planform sweep angles (`[15.0°, 165.0°]`).
  3. **`WingPairKinematics` & `MetachronalSynthesizer`**: Computes closed-form 50 Hz setpoints at every tick:
     * Flapping angle $\gamma_i(t) = \gamma_{0,i} + A_i \sin(\phi_i)$ and ODrive capstan position/velocity (`motor_pos_turns`, `motor_vel_rps`).
     * 90°-phase-advanced pitch feathering angle $\theta_i(t) = \theta_{\text{trim},i} - \theta_{\text{feather},i} \cos(\phi_i + \delta_{\text{lead}})$ so peak downward stroke velocity ($\cos(\phi_i) = +1$) pitches nose-down for forward thrust and peak upstroke velocity ($\cos(\phi_i) = -1$) feathers nose-up.
  4. **Direct Wiring in `MCOHardware`**: `MCOHardware.__init__`, `apply_commands()`, `_update_kinematics()`, `_send_odrive_velocity()`, and `get_wing_kinematics()` in [`firmware/mco_main.py`](firmware/mco_main.py) now execute `self.kinematics.step()`, verified by [`tests/test_mco_kinematics.py`](tests/test_mco_kinematics.py).

---

## 4. How We Made Slop Impossible Going Forward

To ensure neither an LLM nor a human contributor can ever re-introduce drift, fake audits, or duplicate code, we installed **three layers of deterministic enforcement**:

### Layer 1: Hierarchical `AGENTS.md` & `CLAUDE.md` Guardrails
We added strict, <=200-line `AGENTS.md` (and `@AGENTS.md` pointer `CLAUDE.md`) files at the repository root and in every domain directory:
* [`AGENTS.md`](AGENTS.md) — Root architecture, Single Source of Truth map, and banned anti-patterns.
* [`firmware/AGENTS.md`](firmware/AGENTS.md) — Documents [`firmware/mco_kinematics.py`](firmware/mco_kinematics.py), bans adding functions to `mco_wave_library.py`, bans redefining functions, and requires importing `MCOConfig`.
* [`cad/AGENTS.md`](cad/AGENTS.md) — Bans STL-to-STEP facet conversions, requires `Path(__file__).resolve().parents[2]`, and requires PrusaSlicer manifold verification.
* [`scripts/AGENTS.md`](scripts/AGENTS.md) — Bans fake simulation scripts (`random.gauss()`, `time.sleep()`, hardcoded `"PASS"` reports).
* [`docs/AGENTS.md`](docs/AGENTS.md) — Enforces Simplified Technical English (STE), relative links, and bans unverified certification claims.
* [`tests/AGENTS.md`](tests/AGENTS.md) — Bans tautological dictionary-literal tests and requires behavioral assertions.

### Layer 2: Unified 7-Gate `./mco doctor` Audit
Running `./mco doctor` now executes 7 deterministic checks and exits non-zero if any gate fails:
1. **Root Directory Cleanliness:** Rejects any stray file or directory in the repository root.
2. **Single Source of Truth Sync:** Verifies [`config/airframe_spec.json`](config/airframe_spec.json), [`firmware/mco_config.py`](firmware/mco_config.py), and [`bom/bom.csv`](bom/bom.csv) match on MTOW, installed mass, wingspan, wing area, capstan ratio, and disjoint GPIO pins.
3. **AST Duplicate Symbol Gate:** Parses every active `.py` file with Python's `ast` module and fails immediately if any file defines two top-level functions or classes with the same name.
4. **Portable Path Gate:** Scans all active files and fails if any hardcoded user path (`/Users/...`) is introduced.
5. **Documentation Link Audit:** Verifies that every relative Markdown link across all active docs and `AGENTS.md` files resolves to an existing file on disk.
6. **Collection Standards Linters:** Runs all 5 standards linters in [`tools/`](tools/) (`agents_md_lint.py`, `ai_file_lint.py`, `bom_lint.py`, `check_docs.py`, `check_prints.py`).
7. **CAD & PrusaSlicer Manifold Gate:** Verifies all 7 STL files in [`cad/print_package_wing_root/`](cad/print_package_wing_root/) are 100% manifold (0 open edges) and fit the Prusa CORE One build volume (`250 × 220 × 250 mm`).

### Layer 3: Automated Unit Tests for SSOT, Kinematics & Hygiene
* [`tests/test_single_source_of_truth.py`](tests/test_single_source_of_truth.py) — Tests that all active firmware modules (`mco_main`, `mco_odrive_interface`, `mco_performance_model`, `mco_cg_estimator`, `mco_taxi_park`, `mco_rc_verify`) use `MCOConfig` constants and reciprocating ODrive velocity commands.
* [`tests/test_mco_kinematics.py`](tests/test_mco_kinematics.py) — Tests 4-station metachronal phase progression, 90°-phase-advanced cosine pitch feathering, roll/pitch/yaw control allocation mixer clamps, and `MCOHardware.apply_commands()` integration.
* [`tests/test_anti_slop_gates.py`](tests/test_anti_slop_gates.py) — Runs the AST duplicate symbol check, portable path gate, SSOT sync gate, and Markdown link verifier inside `./mco test`.

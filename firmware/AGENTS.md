# Firmware (`firmware/`)

Flight-control software for the MCO-1D Raspberry Pi 5 companion computer.
Read [`../AGENTS.md`](../AGENTS.md) first.

## Architecture & canonical modules

- [`mco_config.py`](mco_config.py): loads [`../config/airframe_spec.json`](../config/airframe_spec.json)
  into `MCOConfig`. Import all physical constants, CAN node IDs, gear ratios,
  battery thresholds, and GPIO pin maps from `mco_config`.
- [`mco_kinematics.py`](mco_kinematics.py): deterministic 50 Hz metachronal
  phase oscillator (`MetachronalPhaseOscillator`), control allocation mixer
  (`ControlAllocationMixer`), and 8-wing reciprocating kinematics synthesizer
  (`WingPairKinematics`, `MetachronalSynthesizer`).
- [`mco_main.py`](mco_main.py): 50 Hz synchronous control loop, IMU fusion, and
  actuator dispatch (`MCOHardware`).
- [`mco_odrive_interface.py`](mco_odrive_interface.py): SocketCAN (`can0`)
  interface to the 8 ODrive Micro nodes (IDs `0..7`) driving the 5010 360KV
  motors through the `10.0:1` stepped capstan reduction.
- [`mco_flight_governor.py`](mco_flight_governor.py): actuator slew-rate,
  travel-limit, and structural load-factor (`[-1.0g, +2.2g]`) envelope clamp.
- [`mco_intent_control.py`](mco_intent_control.py): maps 5-channel pilot stick
  inputs (`Pitch`, `Roll`, `Yaw`, `Throttle`, `Wave Lever`) to wing commands.
- [`mco_logger.py`](mco_logger.py), [`mco_ground_station.py`](mco_ground_station.py),
  [`mco_mavlink.py`](mco_mavlink.py): CSV blackbox logging, local HTTP
  dashboard, and MAVLink v2 heartbeat/attitude bridge.
- [`mco_preflight_checks.py`](mco_preflight_checks.py),
  [`mco_first_run_wizard.py`](mco_first_run_wizard.py),
  [`mco_test_cards.py`](mco_test_cards.py): bench checkout and test-card runners.

## Strict rules for `firmware/`

1. **Zero duplicate modules**: do not create parallel `_v2` or alternate modules
   (legacy duplicates are quarantined in `archive/legacy_modules/`).
2. **Reciprocating capstan kinematics**: sector drums have a finite `200°` wrap
   arc (`±50°` wing stroke). Never command continuous unidirectional motor
   velocity on `can0`; all flapping commands must oscillate around neutral.
3. **Disjoint GPIO pins**: `PITCH_PINS = [4, 5, 6, 12, 13, 16, 17, 18]` and
   `SWEEP_PINS = [19, 20, 21, 22, 23, 24, 25, 26]` must remain strictly
   disjoint. Never override pin arrays locally in [`mco_main.py`](mco_main.py).
4. **No new hypotheses in `mco_wave_library.py`**: do not append new dictionary
   entries or duplicate functions to [`mco_wave_library.py`](mco_wave_library.py).

<!-- standards:begin -->
Collection standards (presentations, sources, git hygiene) live in
`/Users/willem/Code/standards/STANDARDS.md`; decks are built from
`standards/powerpoint template/` and gated with its `deck_checks.py`. Managed
block: edit `standards/ai-files/BLOCK-nested.md`, not this copy.
AI use in school work: no AI images, adviser pre-clearance for AI-written
text, never edit graded text; see
`standards/standards/ai-use-disclosure/ai-use-disclosure.md`.
AI files: one `AGENTS.md` per repo; `CLAUDE.md` is `@AGENTS.md`.
<!-- standards:end -->

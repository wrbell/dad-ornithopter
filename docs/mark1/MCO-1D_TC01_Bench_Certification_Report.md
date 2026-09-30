# MCO-1D TC-01 Ground Bench Qualification Report

**Aircraft**: MCO-1D Metachronal Cascade Ornithopter (S/N MCO-1D-001)  
**Test Date**: 2026-09-30T16:44:43Z  
**Overall Result**: **PASSED — CLEARED FOR TC-02**  

## Qualification Checks

| Check ID | System Under Test | Status | Value / Details |
| :--- | :--- | :---: | :--- |
| `TC01-01` | 7.4V Rail Quiescent Power | PASS | V=7.42V, Idle=10.8W (Limit <= 18.0W) |
| `TC01-02` | 16x Servo Neutral Alignment | PASS | Max deviation 0.8° (Limit <= 2.0°) |
| `TC01-03` | Dual MCP3208 Feedback ADCs | PASS | All 16/16 ADC feedback channels responding |
| `TC01-04` | CAN-FD ODrive S1 Nodes 0-7 | PASS | All 8 ODrive nodes active at 1M/8M CAN-FD |
| `TC01-05` | Dyneema SK99 Cable Preload | PASS | Preload=48.5N, Slip=1.2° (Limit <= 5.0°) |
| `TC01-06` | Capstan Sector Hardstops | PASS | Inter-wing collision clearance = 30.0° (Safe >= 25°) |
| `TC01-07` | Motor Thermal Commutation Dither | PASS | Peak stator temp = 42.5°C (Limit <= 60.0°C) |
| `TC01-08` | Flapping Ramp (0.5 to 3.5 Hz) | PASS | 3.5 Hz Thrust=266.8N, Elec Power=570.8W |
| `TC01-09` | Back-EMF Shuttle & 50W Shunt | PASS | Over-voltage clamp active: 45.0W shunted to 50W resistor |
| `TC01-10` | CF-1 Clap-and-Fling Dispatch | PASS | Mode=CF1_BURST, Lift=128.2N (Bench Mode OK) |

## Authority Sign-Off

- Lead Flight Control Engineer: `Antigravity AI / Autonomous Flight Systems`
- Clearance Status: **READY FOR TETHERED LIFT TEST (TC-02)**

import { useState } from "react";

// ─── Data ─────────────────────────────────────────────────────────────────────

const DRONE = {
  label: "DRONE PROTOTYPE",
  subtitle: "Medium-scale · 1 Hz · 8-wing cascade · S1223 + VG + Coverts",
  accent: "#00ffc8",
  dim: "#00ffc840",
  specs: [
    { label: "Total mass (estimated)", value: "2.8–4.2 kg", confidence: "high", basis: "Scaled from 8-wing cascade + servo mass budget" },
    { label: "Wing system mass (8 wings)", value: "0.6–0.9 kg", confidence: "high", basis: "S1223 CFRP + membrane, 3-DoF root × 8" },
    { label: "Total wingspan (tip to tip)", value: "1.05–1.20 m per side × 2 = 2.1–2.4 m", confidence: "high", basis: "W3 longest at 1.05S, staggered cascade geometry" },
    { label: "Total wing area (8 wings)", value: "0.42–0.58 m²", confidence: "high", basis: "Progressive chord/span specification; AR 3.5–5.0" },
    { label: "Wing loading", value: "58–73 N/m²", confidence: "high", basis: "MTOW / total wing area" },
    { label: "Disk loading (actuator disk)", value: "72–90 N/m²", confidence: "medium", basis: "Based on effective swept disk area at 120° stroke" },
  ],
  aero: [
    { label: "Peak lift coefficient (W1, with VG+coverts)", value: "CL 2.4–2.6", confidence: "high", basis: "S1223 CL max 2.36 + VG +10% + covert flap +20%" },
    { label: "Cruise L/D ratio", value: "18–28", confidence: "medium", basis: "S1223 L/D 40–55 clean; reduced by flapping unsteadiness ~40–50%" },
    { label: "Hover power loading", value: "6–9 g/W", confidence: "medium", basis: "Optimised clap-and-fling achieves >8 g/W; cascade adds ~15%" },
    { label: "Figure of merit (hover efficiency)", value: "0.62–0.74", confidence: "medium", basis: "120° stroke optimal amplitude; cascade temporal phasing" },
    { label: "Effective AoA operating range", value: "0°–42°", confidence: "high", basis: "VGs +4°, covert flaps +20°, forward sweep +LEV stability" },
    { label: "Stall character", value: "Progressive, outer-panel first", confidence: "high", basis: "Wing fence spanwise zoning; W-distribution span profile" },
  ],
  flight: [
    { label: "Hover", value: "✓ Sustained, stable", confidence: "high", basis: "Counter-phase metachronal wave; proven at this mass range" },
    { label: "Hover power consumption", value: "38–62 W", confidence: "medium", basis: "Mass × g / power loading (3.5 kg × 9.81 / 7 g/W)" },
    { label: "Hover endurance (4S LiPo 5Ah)", value: "18–32 minutes", confidence: "medium", basis: "74Wh battery ÷ 50W average hover draw" },
    { label: "Minimum flight speed", value: "0 km/h (hover)", confidence: "high", basis: "Inherent in design" },
    { label: "Cruise speed (optimal)", value: "18–28 km/h", confidence: "medium", basis: "Advance ratio J=0.3–0.5 at 1 Hz; forward-swept stroke plane" },
    { label: "Maximum forward speed", value: "38–52 km/h", confidence: "medium", basis: "Wings swept back, glide-assisted; cascade + dynamic sweep" },
    { label: "Glide speed (wings fixed)", value: "24–34 km/h", confidence: "medium", basis: "L/D 18–28; wings locked at optimal fixed-wing AoA" },
    { label: "Glide ratio", value: "8:1 to 12:1", confidence: "medium", basis: "FX63-137 class efficiency; membrane auto-cambers" },
    { label: "Vertical climb rate", value: "1.4–2.8 m/s", confidence: "medium", basis: "Excess power at MTOW, all-wings in-phase burst mode" },
    { label: "Maximum altitude", value: "~2,500 m", confidence: "low", basis: "Power-limited; servo actuators not altitude-compensated" },
    { label: "Reverse flight speed", value: "0–8 km/h", confidence: "medium", basis: "Wave direction reversal; lower efficiency than forward" },
    { label: "Lateral translation (crabbing)", value: "0–4 km/h", confidence: "medium", basis: "Asymmetric stroke plane tilt; novel capability" },
  ],
  control: [
    { label: "Roll rate (max)", value: "35–65 °/s", confidence: "medium", basis: "3 independent roll mechanisms; 1 Hz cycle limits bandwidth" },
    { label: "Pitch rate (max)", value: "25–45 °/s", confidence: "medium", basis: "Front/rear AoA + sweep differential; limited by cycle time" },
    { label: "Yaw rate (max)", value: "45–80 °/s", confidence: "medium", basis: "Differential wave phase; most effective axis" },
    { label: "Wind gust rejection", value: "Up to ~6 m/s", confidence: "medium", basis: "8 independent wings; 3-mechanism redundancy per axis" },
    { label: "Position hold accuracy (GPS)", value: "±0.3–0.8 m", confidence: "medium", basis: "IMU + GPS; hover stability comparable to quadrotor" },
    { label: "Attitude control bandwidth", value: "~1.0–1.5 Hz", confidence: "medium", basis: "Limited by fundamental flap frequency" },
    { label: "Actuator count", value: "24 servos (3 DoF × 8 wings)", confidence: "high", basis: "Stroke + pitch + sweep per wing root" },
    { label: "Control channels", value: "5 (roll, pitch, yaw, throttle, wave mode)", confidence: "high", basis: "Standard FPV + wave mode selector" },
    { label: "Fault tolerance", value: "Operable with 6 of 8 wings", confidence: "medium", basis: "Phase recalculation; asymmetric compensation" },
  ],
  acoustic: [
    { label: "Dominant frequency", value: "1 Hz (infrasound — inaudible)", confidence: "high", basis: "Fundamental flap frequency; below 20 Hz hearing threshold" },
    { label: "Aerodynamic noise", value: "Very low — estimated <45 dB at 5m", confidence: "medium", basis: "No rotating disc; large area slow wing" },
    { label: "Character", value: "Soft whooshing — near-silent in practice", confidence: "high", basis: "Inherent to slow large-area flapping" },
  ],
  payload: [
    { label: "Estimated empty weight", value: "2.2–3.1 kg", confidence: "medium", basis: "Structure + 24 servos + flight computer + power system" },
    { label: "Maximum payload", value: "0.8–1.4 kg at 3.5 kg MTOW", confidence: "medium", basis: "~30–35% payload fraction; typical for complex drones" },
    { label: "Payload examples", value: "Camera gimbal, sensors, small delivery", confidence: "high", basis: "Standard drone payload categories" },
  ],
  comparison: [
    { label: "vs quadrotor (same mass)", value: "Lower top speed, better gust rejection, far quieter, hover capable, reverse flight, unique low-speed maneuvers" },
    { label: "vs conventional ornithopter", value: "Common ornithopters can't hover or climb vertically; this design does both. Estimated 3–5× better hover efficiency than 2-wing equivalents" },
    { label: "vs fixed-wing UAV", value: "Slower cruise, but VTOL capable, dramatically quieter, far more manoeuvrable at low speed" },
  ],
};

const AIRCRAFT = {
  label: "FULL-SCALE AIRCRAFT",
  subtitle: "400 kg MTOW · 0.2 Hz · 8-wing cascade · FX63-137 + VG + Coverts · Rotax 915 hybrid",
  accent: "#ffa940",
  dim: "#ffa94040",
  specs: [
    { label: "Maximum takeoff weight", value: "380–420 kg", confidence: "high", basis: "Scaling analysis; Piper Cub class" },
    { label: "Empty weight (structure + power)", value: "220–255 kg", confidence: "medium", basis: "Carbon fibre structure + Rotax 915 hybrid system budget" },
    { label: "Useful load (pilot + fuel + payload)", value: "145–195 kg", confidence: "medium", basis: "MTOW minus empty weight" },
    { label: "Total wingspan (tip to tip)", value: "15.4–16.8 m", confidence: "medium", basis: "W3 longest at 4.7m × 2 sides + fuselage width" },
    { label: "Total wing area (8 wings)", value: "18–24 m²", confidence: "medium", basis: "Progressive span/chord specification across cascade" },
    { label: "Wing loading", value: "165–210 N/m²", confidence: "medium", basis: "MTOW / total wing area; comparable to Piper Cub 167 N/m²" },
    { label: "Fuselage length", value: "5.2–6.1 m", confidence: "medium", basis: "Staggered cascade root positions span ~3 chord lengths" },
    { label: "Power system", value: "Rotax 915 iS + EMRAX 228 gen + 8 kWh buffer", confidence: "high", basis: "Off-shelf hybrid system as specified" },
    { label: "Fuel capacity (gasoline)", value: "40–55 litres", confidence: "medium", basis: "Mass budget allows ~40 kg fuel" },
  ],
  aero: [
    { label: "Peak lift coefficient (W1)", value: "CL 2.2–2.4", confidence: "medium", basis: "FX63-137 CL max 2.1 + VG +12% + covert flaps +15% at Re 400k" },
    { label: "Cruise L/D ratio", value: "22–35", confidence: "medium", basis: "FX63-137 L/D 60–80 clean; unsteady effects ~40–50% reduction at 0.2 Hz" },
    { label: "Hover power loading", value: "3.5–5.5 g/W", confidence: "medium", basis: "Scaled from drone; larger disk area; 0.2 Hz inertial losses higher" },
    { label: "Reynolds number range", value: "120,000 (hover) – 650,000 (cruise)", confidence: "high", basis: "Chord × velocity calculations" },
    { label: "Aerodynamic regime", value: "Transitional quasi-steady with unsteady augmentation", confidence: "high", basis: "Re 100k–700k; LEV partially stable, conventional lift dominant" },
    { label: "Effective AoA range (with enhancements)", value: "0°–38°", confidence: "medium", basis: "FX63-137 + VG + coverts; slightly less than drone due to Re transition" },
    { label: "Glide ratio (wings fixed, cruise)", value: "10:1 to 15:1", confidence: "medium", basis: "FX63-137 efficiency at cruise Re; albatross-class glide" },
  ],
  flight: [
    { label: "Hover capability", value: "✓ Limited duration — power intensive", confidence: "medium", basis: "0.2 Hz hover requires ~18–25 kW; fuel-limited to ~8–12 min continuous" },
    { label: "Hover power consumption", value: "18–26 kW", confidence: "medium", basis: "400 kg × 9.81 / 4.5 g/W power loading" },
    { label: "Minimum flight speed", value: "22–30 km/h", confidence: "medium", basis: "Low stall speed from high CL; very low wing loading" },
    { label: "Stall speed", value: "26–34 km/h", confidence: "medium", basis: "CL max 2.3, wing area 21m², MTOW 400 kg" },
    { label: "Cruise speed (optimal efficiency)", value: "68–88 km/h", confidence: "medium", basis: "Best L/D point; glide-dominant flight mode" },
    { label: "Maximum forward speed", value: "115–135 km/h", confidence: "medium", basis: "Wings swept back, reduced area, glide mode; cascade architecture" },
    { label: "Vertical climb rate (continuous)", value: "0.8–1.8 m/s", confidence: "medium", basis: "Excess power above level flight; 0.2 Hz limits burst climb" },
    { label: "Maximum climb rate (burst, all in-phase)", value: "2.4–3.6 m/s", confidence: "low", basis: "Peak power burst, brief duration" },
    { label: "Service ceiling", value: "2,500–3,500 m", confidence: "low", basis: "Power-limited; hybrid system performance at altitude" },
    { label: "Range (cruise, glide-dominant)", value: "180–280 km", confidence: "medium", basis: "40L fuel, Rotax fuel consumption at 30% throttle cruise" },
    { label: "Endurance", value: "2.5–4.5 hours", confidence: "medium", basis: "Range / cruise speed" },
    { label: "Takeoff roll", value: "Near zero to 45 m", confidence: "medium", basis: "Hover-assisted vertical, or short ground run" },
    { label: "Landing roll", value: "Near zero to 30 m", confidence: "medium", basis: "Flare + hover-assist deceleration" },
    { label: "Reverse flight speed", value: "0–12 km/h", confidence: "medium", basis: "Wave direction reversal; scale benefits this capability" },
  ],
  control: [
    { label: "Roll rate (max)", value: "12–22 °/s", confidence: "medium", basis: "0.2 Hz cycle + high moment of inertia at 400 kg" },
    { label: "Pitch rate (max)", value: "8–16 °/s", confidence: "medium", basis: "Front/rear cascade AoA differential; deliberate aircraft" },
    { label: "Yaw rate (max)", value: "15–28 °/s", confidence: "medium", basis: "Differential wave phase; effective but slow vs quadrotor" },
    { label: "Control interface", value: "Conventional stick + rudder + throttle + trim", confidence: "high", basis: "FBW translation to 48 actuators; pilot sees standard aircraft" },
    { label: "Wind gust rejection", value: "Up to 10–14 m/s (moderate gusts)", confidence: "medium", basis: "3-mechanism redundancy; large wing area dampens disturbances" },
    { label: "Actuator count", value: "24 servo motors (3 DoF × 8 wings)", confidence: "high", basis: "Stroke + pitch + sweep per root" },
    { label: "Flight control system", value: "Fly-by-wire, triple redundant", confidence: "high", basis: "Required for complex kinematics; standard for novel aircraft" },
    { label: "Stability character", value: "Positive longitudinal — self-stabilising", confidence: "medium", basis: "Staggered root cascade creates natural pitch stability" },
    { label: "Emergency glide", value: "Wings lock to optimal fixed AoA; glide ratio 10–15:1", confidence: "medium", basis: "Sweep-to-fixed position; battery powers servos for 15+ min" },
    { label: "Fault tolerance", value: "Operable with 5 of 8 wings", confidence: "medium", basis: "Phase recalculation; FBW compensates for asymmetry" },
  ],
  acoustic: [
    { label: "Dominant frequency", value: "0.2 Hz (deep infrasound)", confidence: "high", basis: "Completely inaudible; below all hearing thresholds" },
    { label: "Aerodynamic noise at 100m", value: "Estimated 38–48 dB(A)", confidence: "low", basis: "No propeller; large slow wings; structural creak possible" },
    { label: "Engine noise (Rotax 915)", value: "Standard aircraft noise — dominant source", confidence: "high", basis: "Rotax 915 is primary noise source; muffled installation possible" },
    { label: "Wing-only (electric mode)", value: "Near-silent — feather in the wind character", confidence: "medium", basis: "At 0.2 Hz with 8 wings in metachronal wave; no tonal content" },
  ],
  comparison: [
    { label: "vs Piper Cub (PA-18)", value: "Similar cruise speed, much lower stall speed (26 vs 50 km/h), VTOL vs 90m roll, hover capable, dramatically quieter, reverse flight, shorter range" },
    { label: "vs helicopter (similar weight)", value: "Better cruise efficiency, far less noise, longer range, no tail rotor complexity, but less vertical climb rate and slower attitude response" },
    { label: "vs Snowbird ornithopter", value: "Snowbird achieved 25.6 km/h for 19.3 seconds; this design targets 115–135 km/h max, 2.5–4.5 hr endurance, VTOL, powered — categorically different vehicle" },
    { label: "vs eVTOL (Joby, Lilium class)", value: "Similar mass category; lower cruise speed; dramatically quieter; longer endurance (fuel vs battery); mechanically more complex" },
  ],
};

// ─── Confidence indicator ─────────────────────────────────────────────────────
function Confidence({ level }) {
  const cfg = {
    high:   { color: "#00ffc8", label: "HIGH",   dots: 3 },
    medium: { color: "#ffa940", label: "MED",    dots: 2 },
    low:    { color: "#ff6b6b", label: "LOW",    dots: 1 },
  }[level] || { color: "#888", label: "?", dots: 0 };
  return (
    <span style={{ display: "inline-flex", alignItems: "center", gap: 3, marginLeft: 6 }}>
      {[0,1,2].map(i => (
        <span key={i} style={{
          width: 5, height: 5, borderRadius: "50%",
          background: i < cfg.dots ? cfg.color : "#ffffff15",
          display: "inline-block"
        }} />
      ))}
    </span>
  );
}

// ─── Section ──────────────────────────────────────────────────────────────────
function Section({ title, icon, rows, accent, isComparison }) {
  const [open, setOpen] = useState(true);
  return (
    <div style={{ marginBottom: 8 }}>
      <button onClick={() => setOpen(o => !o)} style={{
        width: "100%", background: open ? `${accent}12` : "transparent",
        border: `1px solid ${accent}30`, borderRadius: 6,
        padding: "8px 14px", display: "flex", alignItems: "center",
        gap: 10, cursor: "pointer", color: "#e2e8f0", textAlign: "left",
        transition: "background 0.2s"
      }}>
        <span style={{ fontSize: 14 }}>{icon}</span>
        <span style={{ fontFamily: "'Courier Prime', monospace", fontSize: 11,
          letterSpacing: 2, flex: 1, color: accent }}>{title}</span>
        <span style={{ color: `${accent}80`, fontSize: 10 }}>{open ? "▲" : "▼"}</span>
      </button>
      {open && (
        <div style={{ background: "#07111f", border: `1px solid ${accent}18`,
          borderTop: "none", borderRadius: "0 0 6px 6px", overflow: "hidden" }}>
          {isComparison ? (
            rows.map((r, i) => (
              <div key={i} style={{
                padding: "8px 14px", borderBottom: i < rows.length-1 ? "1px solid #ffffff08" : "none"
              }}>
                <div style={{ fontFamily: "'Courier Prime', monospace", fontSize: 10,
                  color: accent, letterSpacing: 1, marginBottom: 3 }}>{r.label}</div>
                <div style={{ fontSize: 11, color: "#b0c4d8", lineHeight: 1.5 }}>{r.value}</div>
              </div>
            ))
          ) : (
            <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 11 }}>
              <tbody>
                {rows.map((r, i) => (
                  <tr key={i} style={{ borderBottom: "1px solid #ffffff06",
                    background: i % 2 === 0 ? "transparent" : "#ffffff04" }}>
                    <td style={{ padding: "6px 14px", color: "#8ba3be",
                      fontFamily: "'Courier Prime', monospace", fontSize: 10,
                      letterSpacing: 0.5, width: "35%", verticalAlign: "top" }}>
                      {r.label}
                    </td>
                    <td style={{ padding: "6px 8px", color: "#e2e8f0",
                      fontWeight: 600, verticalAlign: "top", width: "25%" }}>
                      {r.value}
                      <Confidence level={r.confidence} />
                    </td>
                    <td style={{ padding: "6px 14px 6px 4px", color: "#6b85a0",
                      fontSize: 10, verticalAlign: "top" }}>
                      {r.basis}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </div>
      )}
    </div>
  );
}

// ─── Vehicle panel ────────────────────────────────────────────────────────────
function VehiclePanel({ data }) {
  const sections = [
    { title: "PHYSICAL DIMENSIONS", icon: "◈", rows: data.specs },
    { title: "AERODYNAMIC PERFORMANCE", icon: "◎", rows: data.aero },
    { title: "FLIGHT PERFORMANCE", icon: "▲", rows: data.flight },
    { title: "CONTROL & HANDLING", icon: "↻", rows: data.control },
    { title: "ACOUSTIC SIGNATURE", icon: "◉", rows: data.acoustic },
    { title: "PAYLOAD", icon: "⬡", rows: data.payload, skip: !data.payload },
    { title: "COMPARATIVE CONTEXT", icon: "≡", rows: data.comparison, isComparison: true },
  ].filter(s => !s.skip);

  return (
    <div style={{ flex: 1, minWidth: 340 }}>
      {/* Header */}
      <div style={{
        background: `linear-gradient(135deg, ${data.accent}20, ${data.accent}08)`,
        border: `1px solid ${data.accent}50`,
        borderRadius: 10, padding: "16px 18px", marginBottom: 12
      }}>
        <div style={{ fontFamily: "'Courier Prime', monospace",
          fontSize: 13, letterSpacing: 3, color: data.accent,
          textShadow: `0 0 20px ${data.accent}60` }}>
          {data.label}
        </div>
        <div style={{ fontSize: 10, color: `${data.accent}80`,
          marginTop: 4, lineHeight: 1.6 }}>{data.subtitle}</div>
      </div>

      {sections.map(s => (
        <Section key={s.title} {...s} accent={data.accent} />
      ))}
    </div>
  );
}

// ─── Confidence legend ────────────────────────────────────────────────────────
function Legend() {
  return (
    <div style={{ display: "flex", gap: 20, justifyContent: "center",
      padding: "10px 0 4px", flexWrap: "wrap" }}>
      {[
        { level: "high", label: "High confidence — directly calculable or research-validated" },
        { level: "medium", label: "Medium — extrapolated from known data with assumptions" },
        { level: "low", label: "Low — estimated, requires physical testing to verify" },
      ].map(({ level, label }) => (
        <div key={level} style={{ display: "flex", alignItems: "center", gap: 6 }}>
          <Confidence level={level} />
          <span style={{ fontSize: 9, color: "#ffffff40",
            fontFamily: "'Courier Prime', monospace" }}>{label}</span>
        </div>
      ))}
    </div>
  );
}

// ─── App ──────────────────────────────────────────────────────────────────────
export default function App() {
  const [tab, setTab] = useState("both");

  return (
    <div style={{
      minHeight: "100vh",
      background: "#030b16",
      backgroundImage: `
        radial-gradient(ellipse at 20% 20%, #00ffc808 0%, transparent 50%),
        radial-gradient(ellipse at 80% 80%, #ffa94008 0%, transparent 50%)
      `,
      color: "#e2e8f0",
      fontFamily: "system-ui, sans-serif",
      padding: "20px 16px"
    }}>
      {/* Title */}
      <div style={{ textAlign: "center", marginBottom: 20 }}>
        <div style={{ fontFamily: "'Courier Prime', monospace",
          fontSize: 18, letterSpacing: 4, color: "#e2e8f0",
          textShadow: "0 0 30px #ffffff20" }}>
          METACHRONAL CASCADE FLAPPER
        </div>
        <div style={{ fontSize: 10, color: "#ffffff30",
          letterSpacing: 2, marginTop: 4 }}>
          PERFORMANCE PARAMETERS · DRONE PROTOTYPE · FULL-SCALE AIRCRAFT
        </div>
        <Legend />
      </div>

      {/* Tab selector */}
      <div style={{ display: "flex", gap: 8, justifyContent: "center", marginBottom: 20 }}>
        {[
          { id: "drone", label: "DRONE ONLY", color: "#00ffc8" },
          { id: "both", label: "SIDE BY SIDE", color: "#e2e8f0" },
          { id: "aircraft", label: "AIRCRAFT ONLY", color: "#ffa940" },
        ].map(t => (
          <button key={t.id} onClick={() => setTab(t.id)} style={{
            padding: "6px 18px", fontSize: 10, letterSpacing: 2,
            fontFamily: "'Courier Prime', monospace",
            background: tab === t.id ? `${t.color}20` : "transparent",
            border: `1px solid ${tab === t.id ? t.color : "#ffffff20"}`,
            color: tab === t.id ? t.color : "#ffffff40",
            borderRadius: 4, cursor: "pointer"
          }}>{t.label}</button>
        ))}
      </div>

      {/* Panels */}
      <div style={{ display: "flex", gap: 16, flexWrap: "wrap",
        alignItems: "flex-start", maxWidth: 1400, margin: "0 auto" }}>
        {(tab === "drone" || tab === "both") && <VehiclePanel data={DRONE} />}
        {(tab === "aircraft" || tab === "both") && <VehiclePanel data={AIRCRAFT} />}
      </div>

      {/* Footer note */}
      <div style={{ textAlign: "center", marginTop: 24, fontSize: 9,
        color: "#ffffff15", fontFamily: "'Courier Prime', monospace",
        letterSpacing: 1, lineHeight: 1.8, maxWidth: 900, margin: "24px auto 0" }}>
        ALL PERFORMANCE FIGURES ARE DESIGN-STAGE ESTIMATES.<br/>
        Confidence ratings reflect fidelity of underlying data. High = first-principles or research-validated.
        Medium = extrapolated from analogous systems with stated assumptions.
        Low = theoretical estimate requiring physical test validation.<br/>
        Actual performance will depend on build quality, tuning, environmental conditions, and iterative development.
      </div>
    </div>
  );
}

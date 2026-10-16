import { useState } from "react";

const SECTIONS = [
  {
    id: "airfoil",
    label: "AIRFOIL DESIGN",
    color: "#4af0c4",
    subsections: [
      {
        title: "Planform Shape",
        icon: "◈",
        content: [
          { label: "Aspect Ratio", value: "4:1 to 6:1", note: "High enough for efficiency, low enough for stroke clearance between wings" },
          { label: "Leading Edge", value: "Rounded, blunt ellipse", note: "Tolerates high AoA without abrupt separation — critical at 35–45°" },
          { label: "Trailing Edge", value: "Thin, tapered", note: "Clean vortex shedding on stroke reversal" },
          { label: "Tip Shape", value: "Rounded / elliptical", note: "Minimises tip vortex loss at slow speeds" },
          { label: "Spanwise Taper", value: "1.5:1 root-to-tip chord ratio", note: "Root handles structural loads; tip optimised for velocity" },
        ]
      },
      {
        title: "Cross-Section Profile",
        icon: "⬡",
        content: [
          { label: "Type", value: "Corrugated membrane — dragonfly-derived", note: "Not a smooth NACA profile. Corrugations trap recirculation zones acting as virtual camber" },
          { label: "Camber", value: "6–10% chord", note: "Higher camber than fixed-wing equivalents — needed at high AoA without stall" },
          { label: "Thickness", value: "4–6% chord", note: "Thin enough to minimise drag on feathered recovery stroke" },
          { label: "Corrugation", value: "Spanwise ridges, heavy at root, fade toward tip", note: "Root corrugation delays separation; smooth tip maximises tip lift" },
          { label: "Chord Camber Profile", value: "Maximum camber at 25–30% chord", note: "Forward camber peak handles leading-edge separation at high AoA" },
        ]
      },
      {
        title: "Material & Flexibility",
        icon: "◎",
        content: [
          { label: "Structure", value: "Rigid leading-edge spar + flexible membrane", note: "LE spar maintains shape under aerodynamic load; membrane billows under pressure to auto-camber" },
          { label: "Membrane Material", value: "Ripstop nylon or mylar 15–25 micron", note: "Low mass critical — inertial loads at 1 Hz still significant at medium scale" },
          { label: "Spar Material", value: "Carbon fibre rod / tube", note: "Stiff in bending, allows torsional flexibility for passive pitch adaptation" },
          { label: "Passive Washout", value: "Built-in 5–8° twist root-to-tip", note: "Tip unloads automatically at high AoA — prevents tip stall, improves roll stability" },
          { label: "Aeroelastic number", value: "Optimised compliance", note: "Membrane stiffness tuned so it billows to max camber mid-power-stroke and flattens during recovery" },
        ]
      },
      {
        title: "Why NOT a Standard Airfoil",
        icon: "✕",
        content: [
          { label: "NACA 4-digit profiles", value: "Poor at AoA > 15°", note: "Designed for low AoA cruise — your device operates at 35–45°, well beyond their design envelope" },
          { label: "Rigid wings", value: "Stall at 12–15°", note: "Membrane wings tolerate 30–45° before stall — tripling the usable AoA range" },
          { label: "Thick sections", value: "High inertia penalty", note: "At 1 Hz the wing reversal means constant acceleration. Every gram of wing mass costs actuator power" },
          { label: "Symmetric sections", value: "Generates drag on recovery", note: "An asymmetric, thinly cambered section with passive feathering minimises recovery stroke drag" },
        ]
      }
    ]
  },
  {
    id: "maneuvers",
    label: "MANEUVER CAPABILITY",
    color: "#f59e0b",
    subsections: [
      {
        title: "Fundamental Flight Modes",
        icon: "▲",
        content: [
          { label: "Stationary Hover", value: "✓ Primary capability", note: "Counter-phase wave, feathered recovery. Near-zero drift with IMU feedback" },
          { label: "Forward Cruise", value: "✓ Metachronal wave F→B", note: "Stroke plane tilted forward. Estimated 2–6 m/s at medium device scale" },
          { label: "Reverse Flight", value: "✓ Wave direction flip", note: "Unique to this architecture — pure software command, no mechanical reversal" },
          { label: "Vertical Climb", value: "✓ In-phase burst + high AoA", note: "All 8 wings in-phase, 42–45° AoA. High power consumption, short duration" },
          { label: "Controlled Descent", value: "✓ Symmetric drag mode", note: "Recovery AoA raised to match power AoA — both strokes generate drag for slow sink" },
          { label: "Glide", value: "✓ Wings fixed, passive camber", note: "Membrane auto-cambers under airspeed. Usable glide ratio estimated 4:1 to 6:1" },
        ]
      },
      {
        title: "Attitude Control",
        icon: "↻",
        content: [
          { label: "Roll", value: "Differential amplitude L vs R", note: "Smooth and proportional. 8-wing version has finer resolution than 4-wing" },
          { label: "Pitch", value: "Differential AoA front vs rear wings", note: "Novel — doesn't require tilting entire stroke plane. Very precise" },
          { label: "Yaw", value: "Differential wave phase L vs R", note: "Left side wave reverse vs right side forward = clean spin. Very effective" },
          { label: "Lateral Translation", value: "Asymmetric stroke plane tilt", note: "Unique capability — sideways drift without roll. Crabbing maneuver" },
        ]
      },
      {
        title: "Advanced Maneuvers",
        icon: "★",
        content: [
          { label: "Pirouette", value: "Yaw channel full deflection, hover throttle", note: "Clean spin about vertical axis using differential wave direction" },
          { label: "Wingover / Banked Turn", value: "Roll + yaw combined", note: "Coordinated turn like a banked aircraft. Smoother than quad-rotor equivalent" },
          { label: "Rapid Deceleration / Flare", value: "All wings high drag AoA simultaneously", note: "Dynamic stall used as airbrake — similar to swift post-stall arrest maneuver" },
          { label: "Spot Turn", value: "Yaw only, zero translation", note: "Highly effective — metachronal wave asymmetry generates pure torque" },
          { label: "Nose-High Hover", value: "Pitch up 30–60°, hover maintained", note: "Rear wings carry most lift; unusual attitude stable due to fore/aft AoA gradient" },
          { label: "Perching Approach", value: "Flare + deceleration into contact", note: "Possible with gripper. Slow approach speed makes this more reliable than fixed-wing" },
        ]
      },
      {
        title: "Environmental Capabilities",
        icon: "◉",
        content: [
          { label: "Gust Rejection", value: "Strong — inherent in flapping", note: "Wing kinematics modulate cycle-by-cycle. 8 independent wings = rapid correction" },
          { label: "Crosswind Hover", value: "Good — crabbing mode", note: "Can hold position in crosswind by biasing stroke plane laterally" },
          { label: "Narrow Gap Transit", value: "Moderate", note: "8 wings increase overall footprint vs 4-wing, but wings fold on recovery" },
          { label: "Near-Surface Flight", value: "Good — ground effect", note: "Slow flap + large wing area benefits significantly from ground effect below 0.5 wingspan" },
          { label: "Inverted Flight", value: "Limited — not native", note: "Possible with full AoA reversal but requires significant firmware modification" },
        ]
      },
      {
        title: "Limitations",
        icon: "⚠",
        content: [
          { label: "Maximum Speed", value: "~4–8 m/s estimated", note: "1 Hz limits stroke-cycle advance ratio. Higher speed possible in glide mode" },
          { label: "Agility", value: "Moderate — not aerobatic", note: "Phase changes propagate at 1 Hz — attitude changes take 1–2 full cycles to establish" },
          { label: "Wind limit", value: "~3–4 m/s headwind hover", note: "Comparable to small ornithopters. Metachronal wave helps but 1 Hz is fundamentally limiting" },
          { label: "Rapid roll rate", value: "Slow by multirotor standards", note: "Wing inertia and 1 Hz cycle mean roll rates of perhaps 20–40°/sec max" },
        ]
      }
    ]
  }
];

function AirfoilSVG() {
  // Draw a corrugated cambered airfoil cross-section
  const W = 340, H = 120;
  const chord = 280, ox = 30, oy = 75;

  // Smooth camber line (6% camber at 25% chord)
  const camberPts = [];
  for (let i = 0; i <= 100; i++) {
    const xn = i / 100;
    const yn = xn < 0.25
      ? (0.06 / 0.0625) * (2 * 0.25 * xn - xn * xn)
      : (0.06 / 0.5625) * (1 - 2 * 0.25 + 2 * 0.25 * xn - xn * xn);
    camberPts.push({ x: ox + xn * chord, y: oy - yn * chord * 0.7 });
  }

  // Upper surface: corrugated near leading edge, smooth toward trailing edge
  const upperPts = camberPts.map((p, i) => {
    const xn = i / 100;
    const thickness = 0.06 * Math.sin(Math.PI * xn) * chord;
    const corrugAmp = xn < 0.5 ? (0.5 - xn) * 0.04 * chord : 0;
    const corrugFreq = 5;
    const corrug = corrugAmp * Math.sin(corrugFreq * Math.PI * xn);
    return { x: p.x, y: p.y - thickness - corrug };
  });

  // Lower surface: smooth
  const lowerPts = camberPts.map((p, i) => {
    const xn = i / 100;
    const thickness = 0.04 * Math.sin(Math.PI * xn) * chord;
    return { x: p.x, y: p.y + thickness };
  });

  const toPath = pts => pts.map((p, i) => `${i === 0 ? 'M' : 'L'} ${p.x.toFixed(1)} ${p.y.toFixed(1)}`).join(' ');

  return (
    <svg width={W} height={H} style={{ background: "#060c18", borderRadius: 10, display: "block" }}>
      {/* Camber line */}
      <path d={toPath(camberPts)} fill="none" stroke="#ffffff15" strokeWidth={1} strokeDasharray="4,3" />
      {/* Lower surface */}
      <path d={toPath(lowerPts)} fill="none" stroke="#4af0c4" strokeWidth={1.5} opacity={0.6} />
      {/* Upper surface (corrugated) */}
      <path d={toPath(upperPts)} fill="none" stroke="#4af0c4" strokeWidth={2} />
      {/* Fill */}
      <path
        d={`${toPath(upperPts)} L ${lowerPts[100].x.toFixed(1)} ${lowerPts[100].y.toFixed(1)} ${toPath([...lowerPts].reverse())} Z`}
        fill="#4af0c408" stroke="none"
      />
      {/* Annotations */}
      <text x={ox + chord * 0.18} y={20} fontSize={9} fill="#4af0c480" fontFamily="monospace">corrugated LE</text>
      <line x1={ox + chord*0.18} y1={23} x2={ox + chord*0.12} y2={upperPts[12]?.y ?? oy-5}
        stroke="#4af0c440" strokeWidth={0.8} />
      <text x={ox + chord * 0.62} y={H-8} fontSize={9} fill="#4af0c480" fontFamily="monospace">smooth TE</text>
      <text x={ox + chord * 0.22} y={oy + 28} fontSize={9} fill="#f59e0b60" fontFamily="monospace">camber ~8%</text>
      <text x={2} y={H-8} fontSize={8} fill="#ffffff20" fontFamily="monospace">cross-section (not to scale)</text>
    </svg>
  );
}

function WingPlanSVG() {
  const W = 200, H = 140;
  // Tapered wing planform
  const rootChord = 50, tipChord = 30, span = 110;
  const sweep = 10;
  const points = [
    [40, 20],           // LE root
    [40 + sweep, 20 + span],  // LE tip
    [40 + sweep + tipChord, 20 + span], // TE tip
    [40 + rootChord, 20],    // TE root
  ];
  const d = points.map((p,i) => `${i===0?'M':'L'} ${p[0]} ${p[1]}`).join(' ') + ' Z';

  // Corrugation lines (spanwise veins)
  const veins = [0.15, 0.3, 0.45, 0.6, 0.75, 0.88].map(t => {
    const x1 = 40 + t * rootChord;
    const x2 = 40 + sweep + t * tipChord;
    return { x1, y1: 20, x2, y2: 20 + span };
  });

  return (
    <svg width={W} height={H} style={{ background: "#060c18", borderRadius: 10, display: "block" }}>
      <path d={d} fill="#4af0c408" stroke="#4af0c4" strokeWidth={2} />
      {veins.map((v, i) => (
        <line key={i} x1={v.x1} y1={v.y1} x2={v.x2} y2={v.y2}
          stroke="#4af0c430" strokeWidth={0.8} strokeDasharray="3,2" />
      ))}
      {/* Passive washout arrow */}
      <text x={100} y={80} fontSize={8} fill="#f59e0b80" fontFamily="monospace" transform="rotate(-5,100,80)">washout →</text>
      <text x={38} y={14} fontSize={8} fill="#ffffff40" fontFamily="monospace">root</text>
      <text x={52} y={138} fontSize={8} fill="#ffffff40" fontFamily="monospace">tip</text>
      <text x={2} y={H-4} fontSize={8} fill="#ffffff20" fontFamily="monospace">planform</text>
    </svg>
  );
}

function ManeuverBadge({ label, tier }) {
  const colors = {
    excellent: { bg: "#4af0c415", border: "#4af0c4", text: "#4af0c4" },
    good:      { bg: "#f59e0b15", border: "#f59e0b", text: "#f59e0b" },
    limited:   { bg: "#ef444415", border: "#ef4444", text: "#ef4444" },
  };
  const c = colors[tier] || colors.good;
  return (
    <span style={{
      padding: "2px 8px", borderRadius: 3, fontSize: 9, letterSpacing: 1,
      background: c.bg, border: `1px solid ${c.border}`, color: c.text
    }}>
      {label}
    </span>
  );
}

export default function App() {
  const [activeSection, setActiveSection] = useState("airfoil");
  const [expandedSub, setExpandedSub] = useState(null);

  const section = SECTIONS.find(s => s.id === activeSection);

  return (
    <div style={{
      minHeight: "100vh", background: "#060c18",
      color: "#e2e8f0", fontFamily: "monospace", padding: 20,
      display: "flex", flexDirection: "column", gap: 16
    }}>
      {/* Header */}
      <div>
        <h1 style={{ margin: 0, fontSize: 18, letterSpacing: 3, color: "#e2e8f0" }}>
          8-WING METACHRONAL DRONE
        </h1>
        <div style={{ fontSize: 11, color: "#ffffff40", letterSpacing: 1 }}>
          airfoil specification · maneuver envelope · 1 Hz flap
        </div>
      </div>

      {/* Tab selector */}
      <div style={{ display: "flex", gap: 8 }}>
        {SECTIONS.map(s => (
          <button key={s.id} onClick={() => { setActiveSection(s.id); setExpandedSub(null); }}
            style={{
              padding: "6px 20px", fontSize: 11, letterSpacing: 2,
              background: activeSection === s.id ? `${s.color}20` : "transparent",
              border: `1px solid ${activeSection === s.id ? s.color : "#ffffff20"}`,
              color: activeSection === s.id ? s.color : "#ffffff50",
              borderRadius: 4, cursor: "pointer"
            }}>
            {s.label}
          </button>
        ))}
      </div>

      {/* Airfoil section */}
      {activeSection === "airfoil" && (
        <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
          {/* Diagrams */}
          <div style={{ display: "flex", gap: 12, flexWrap: "wrap" }}>
            <div>
              <div style={{ fontSize: 9, color: "#4af0c460", letterSpacing: 2, marginBottom: 6 }}>
                CROSS-SECTION — corrugated cambered membrane
              </div>
              <AirfoilSVG />
            </div>
            <div>
              <div style={{ fontSize: 9, color: "#4af0c460", letterSpacing: 2, marginBottom: 6 }}>
                PLANFORM — tapered + swept
              </div>
              <WingPlanSVG />
            </div>
            <div style={{ flex: 1, minWidth: 180, background: "#0a0f1a", borderRadius: 10,
              padding: 14, border: "1px solid #4af0c420", display: "flex", flexDirection: "column", gap: 8 }}>
              <div style={{ fontSize: 9, color: "#4af0c460", letterSpacing: 2 }}>KEY DESIGN RULES</div>
              {[
                ["Membrane over rigid", "Auto-cambers under load"],
                ["Corrugated LE", "Delays stall to 30–45°"],
                ["Smooth TE", "Clean vortex shedding"],
                ["Passive washout", "Tip unloads at high AoA"],
                ["Thin section", "Low inertia on reversal"],
                ["High camber", "Lift at low airspeeds"],
              ].map(([rule, why]) => (
                <div key={rule} style={{ display: "flex", flexDirection: "column" }}>
                  <span style={{ color: "#4af0c4", fontSize: 10 }}>▸ {rule}</span>
                  <span style={{ color: "#ffffff40", fontSize: 9, paddingLeft: 12 }}>{why}</span>
                </div>
              ))}
            </div>
          </div>

          {/* Subsections */}
          {section.subsections.map(sub => (
            <div key={sub.title} style={{ background: "#0a0f1a", borderRadius: 8,
              border: `1px solid #4af0c420`, overflow: "hidden" }}>
              <button
                onClick={() => setExpandedSub(expandedSub === sub.title ? null : sub.title)}
                style={{
                  width: "100%", background: "none", border: "none", cursor: "pointer",
                  padding: "10px 14px", display: "flex", alignItems: "center", gap: 10,
                  color: "#e2e8f0", textAlign: "left"
                }}>
                <span style={{ color: "#4af0c4", fontSize: 14 }}>{sub.icon}</span>
                <span style={{ fontSize: 12, letterSpacing: 1, flex: 1 }}>{sub.title}</span>
                <span style={{ color: "#4af0c460", fontSize: 12 }}>
                  {expandedSub === sub.title ? "▲" : "▼"}
                </span>
              </button>
              {expandedSub === sub.title && (
                <div style={{ padding: "0 14px 14px" }}>
                  <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 11 }}>
                    <thead>
                      <tr style={{ borderBottom: "1px solid #ffffff10" }}>
                        <th style={{ textAlign: "left", padding: "4px 8px", color: "#ffffff40", fontWeight: 400, width: "22%" }}>Parameter</th>
                        <th style={{ textAlign: "left", padding: "4px 8px", color: "#ffffff40", fontWeight: 400, width: "28%" }}>Value</th>
                        <th style={{ textAlign: "left", padding: "4px 8px", color: "#ffffff40", fontWeight: 400 }}>Reasoning</th>
                      </tr>
                    </thead>
                    <tbody>
                      {sub.content.map((row, i) => (
                        <tr key={i} style={{ borderBottom: "1px solid #ffffff06" }}>
                          <td style={{ padding: "6px 8px", color: "#4af0c4" }}>{row.label}</td>
                          <td style={{ padding: "6px 8px", color: "#f59e0b" }}>{row.value}</td>
                          <td style={{ padding: "6px 8px", color: "#ffffff60" }}>{row.note}</td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              )}
            </div>
          ))}
        </div>
      )}

      {/* Maneuvers section */}
      {activeSection === "maneuvers" && (
        <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
          {/* Quick capability summary */}
          <div style={{ display: "flex", gap: 8, flexWrap: "wrap" }}>
            {[
              ["Stationary Hover", "excellent"],
              ["Reverse Flight", "excellent"],
              ["Spot Yaw", "excellent"],
              ["Lateral Crab", "good"],
              ["Banked Turn", "good"],
              ["Gust Rejection", "good"],
              ["Perching", "good"],
              ["Aerobatics", "limited"],
              ["High Speed", "limited"],
              ["Inverted", "limited"],
            ].map(([label, tier]) => (
              <ManeuverBadge key={label} label={label} tier={tier} />
            ))}
          </div>

          {section.subsections.map(sub => (
            <div key={sub.title} style={{ background: "#0a0f1a", borderRadius: 8,
              border: `1px solid #f59e0b20`, overflow: "hidden" }}>
              <button
                onClick={() => setExpandedSub(expandedSub === sub.title ? null : sub.title)}
                style={{
                  width: "100%", background: "none", border: "none", cursor: "pointer",
                  padding: "10px 14px", display: "flex", alignItems: "center", gap: 10,
                  color: "#e2e8f0", textAlign: "left"
                }}>
                <span style={{ color: "#f59e0b", fontSize: 14 }}>{sub.icon}</span>
                <span style={{ fontSize: 12, letterSpacing: 1, flex: 1 }}>{sub.title}</span>
                <span style={{ color: "#f59e0b60", fontSize: 12 }}>
                  {expandedSub === sub.title ? "▲" : "▼"}
                </span>
              </button>
              {expandedSub === sub.title && (
                <div style={{ padding: "0 14px 14px" }}>
                  {sub.content.map((row, i) => (
                    <div key={i} style={{
                      display: "grid", gridTemplateColumns: "1fr 1fr 2fr",
                      gap: 8, padding: "6px 8px",
                      borderBottom: "1px solid #ffffff06",
                      fontSize: 11
                    }}>
                      <span style={{ color: "#f59e0b" }}>{row.label}</span>
                      <span style={{ color: "#4af0c4" }}>{row.value}</span>
                      <span style={{ color: "#ffffff50" }}>{row.note}</span>
                    </div>
                  ))}
                </div>
              )}
            </div>
          ))}

          {/* Flight envelope diagram */}
          <div style={{ background: "#0a0f1a", borderRadius: 8, padding: 14,
            border: "1px solid #f59e0b20" }}>
            <div style={{ fontSize: 9, color: "#f59e0b60", letterSpacing: 2, marginBottom: 10 }}>
              ESTIMATED FLIGHT ENVELOPE
            </div>
            <svg width="100%" height={90} viewBox="0 0 500 90"
              style={{ display: "block" }}>
              {/* Speed axis */}
              <line x1={40} y1={70} x2={460} y2={70} stroke="#ffffff20" strokeWidth={1} />
              {/* Hover */}
              <rect x={40} y={20} width={60} height={50} fill="#4af0c420" stroke="#4af0c4" strokeWidth={1} rx={2} />
              <text x={70} y={50} textAnchor="middle" fontSize={9} fill="#4af0c4" fontFamily="monospace">HOVER</text>
              <text x={70} y={62} textAnchor="middle" fontSize={8} fill="#4af0c460" fontFamily="monospace">0 m/s</text>
              {/* Slow cruise */}
              <rect x={100} y={10} width={120} height={60} fill="#f59e0b15" stroke="#f59e0b" strokeWidth={1} rx={2} />
              <text x={160} y={38} textAnchor="middle" fontSize={9} fill="#f59e0b" fontFamily="monospace">CRUISE</text>
              <text x={160} y={52} textAnchor="middle" fontSize={8} fill="#f59e0b60" fontFamily="monospace">1–4 m/s</text>
              {/* Fast forward */}
              <rect x={220} y={25} width={110} height={45} fill="#0ea5e915" stroke="#0ea5e9" strokeWidth={1} rx={2} />
              <text x={275} y={48} textAnchor="middle" fontSize={9} fill="#0ea5e9" fontFamily="monospace">FAST FWD</text>
              <text x={275} y={62} textAnchor="middle" fontSize={8} fill="#0ea5e960" fontFamily="monospace">4–8 m/s</text>
              {/* Glide */}
              <rect x={330} y={35} width={130} height={35} fill="#e879f915" stroke="#e879f9" strokeWidth={1} rx={2} />
              <text x={395} y={55} textAnchor="middle" fontSize={9} fill="#e879f9" fontFamily="monospace">GLIDE (passive)</text>
              <text x={395} y={67} textAnchor="middle" fontSize={8} fill="#e879f960" fontFamily="monospace">8+ m/s</text>
              {/* Labels */}
              <text x={40} y={85} fontSize={8} fill="#ffffff20" fontFamily="monospace">0</text>
              <text x={450} y={85} fontSize={8} fill="#ffffff20" fontFamily="monospace">→ speed</text>
            </svg>
          </div>
        </div>
      )}

      <div style={{ fontSize: 9, color: "#ffffff15", textAlign: "center", letterSpacing: 1 }}>
        Click subsection headers to expand · based on membrane wing & flapping wing MAV research
      </div>
    </div>
  );
}

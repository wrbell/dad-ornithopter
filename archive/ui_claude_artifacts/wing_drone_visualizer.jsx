import { useState, useEffect, useRef, useCallback } from "react";

// ─── Constants ────────────────────────────────────────────────
const FLAP_FREQ   = 1.0;
const NUM_WINGS   = 8;
const WINGS_PER_SIDE = 4;
const STROKE_AMP  = 70;
const DEV_AMP     = 12;
const FLIP_ZONE   = 20;

const WAVE_MODES = [
  { id: 0, label: "Metachronal",   desc: "Traveling wave F→B (cruise)" },
  { id: 1, label: "Reverse Wave",  desc: "Traveling wave B→F (brake)" },
  { id: 2, label: "Standing",      desc: "Alternating pairs (hover)" },
  { id: 3, label: "In-Phase",      desc: "All wings together (burst)" },
  { id: 4, label: "Counter-Phase", desc: "Front vs rear (smooth)" },
];

const SCENARIOS = [
  { id: "hover",     label: "Hover",        throttle: 0.5, roll: 0,    pitch: 0,    yaw: 0,    mode: 2 },
  { id: "cruise",    label: "Cruise",       throttle: 0.6, roll: 0,    pitch: 0,    yaw: 0,    mode: 0 },
  { id: "climb",     label: "Climb",        throttle: 0.85,roll: 0,    pitch: 0,    yaw: 0,    mode: 0 },
  { id: "roll_left", label: "Roll Left",    throttle: 0.5, roll: -0.6, pitch: 0,    yaw: 0,    mode: 0 },
  { id: "yaw_right", label: "Yaw Right",    throttle: 0.5, roll: 0,    pitch: 0,    yaw: 0.7,  mode: 0 },
  { id: "reverse",   label: "Reverse",      throttle: 0.5, roll: 0,    pitch: 0,    yaw: 0,    mode: 1 },
  { id: "burst",     label: "Max Force",    throttle: 1.0, roll: 0,    pitch: 0,    yaw: 0,    mode: 3 },
];

const WING_COLORS = [
  "#4af0c4","#2dd4bf","#06b6d4","#0ea5e9",  // left: W1–W4
  "#f59e0b","#f97316","#ef4444","#e879f9",  // right: W5–W8
];

// ─── Math Helpers ─────────────────────────────────────────────
function smoothStep(t) {
  t = Math.max(0, Math.min(1, t));
  return t * t * (3 - 2 * t);
}

function getPhaseOffsets(mode, yaw) {
  const spacing = 90;
  let offsets = new Array(8).fill(0);

  if (mode === 0) { // metachronal
    for (let i = 0; i < 4; i++) { offsets[i] = i * spacing; offsets[i+4] = i * spacing; }
  } else if (mode === 1) { // reverse
    for (let i = 0; i < 4; i++) { offsets[i] = (3-i)*spacing; offsets[i+4] = (3-i)*spacing; }
  } else if (mode === 2) { // standing
    for (let i = 0; i < 4; i++) { offsets[i] = i%2===0?0:180; offsets[i+4] = i%2===0?0:180; }
  } else if (mode === 3) { // in-phase
    offsets = new Array(8).fill(0);
  } else if (mode === 4) { // counter-phase
    for (let i = 0; i < 4; i++) { offsets[i] = i<2?0:180; offsets[i+4] = i<2?0:180; }
  }

  // Yaw: differential phase shift
  if (Math.abs(yaw) > 0.03) {
    const yawShift = yaw * 180;
    for (let i = 0; i < 4; i++) {
      offsets[i]   = (offsets[i]   + yawShift + 360) % 360;
      offsets[i+4] = (offsets[i+4] - yawShift + 360) % 360;
    }
  }
  return offsets;
}

function resolveKinematics(phaseDeg, aoaPower, aoaRec, strokeAmp, side) {
  const rad = (phaseDeg * Math.PI) / 180;
  let phi = strokeAmp * Math.sin(rad);
  if (side === 'right') phi = -phi;

  let psi;
  if (phaseDeg < FLIP_ZONE) {
    psi = aoaRec + (aoaPower - aoaRec) * smoothStep(phaseDeg / FLIP_ZONE);
  } else if (phaseDeg < 180 - FLIP_ZONE) {
    psi = aoaPower;
  } else if (phaseDeg < 180 + FLIP_ZONE) {
    psi = aoaPower + (aoaRec - aoaPower) * smoothStep((phaseDeg - (180 - FLIP_ZONE)) / (2 * FLIP_ZONE));
  } else if (phaseDeg < 360 - FLIP_ZONE) {
    psi = aoaRec;
  } else {
    psi = aoaRec + (aoaPower - aoaRec) * smoothStep((phaseDeg - (360 - FLIP_ZONE)) / FLIP_ZONE);
  }

  const theta = DEV_AMP * Math.sin(2 * rad);
  return { phi, psi, theta };
}

function getModeParams(throttle) {
  const amp = STROKE_AMP * (0.5 + 0.5 * throttle);
  return { aoaPower: 35 + throttle * 10, aoaRec: 5, strokeAmp: amp };
}

// ─── Wing Diagram ─────────────────────────────────────────────
function WingDiagram({ kinematics, wingIdx, label, color }) {
  const { phi, psi } = kinematics;
  const cx = 50, cy = 60;
  const len = 35;
  const rad = (phi * Math.PI) / 180;
  const wx = cx + len * Math.sin(rad);
  const wy = cy - len * Math.cos(rad);
  const isPower = phi > 0 ? (wingIdx < 4) : (wingIdx >= 4);

  return (
    <div style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: 4 }}>
      <span style={{ fontFamily: "monospace", fontSize: 11, color, fontWeight: 700 }}>{label}</span>
      <svg width={100} height={100} style={{ background: "#0a0f1a", borderRadius: 8, border: `1px solid ${color}22` }}>
        {/* Stroke arc guide */}
        <path
          d={`M ${cx + len * Math.sin(-STROKE_AMP * Math.PI / 180)} ${cy - len * Math.cos(-STROKE_AMP * Math.PI / 180)}
              A ${len} ${len} 0 0 1
              ${cx + len * Math.sin(STROKE_AMP * Math.PI / 180)} ${cy - len * Math.cos(STROKE_AMP * Math.PI / 180)}`}
          fill="none" stroke="#ffffff08" strokeWidth={1}
        />
        {/* Wing body */}
        <line x1={cx} y1={cy} x2={wx} y2={wy}
          stroke={isPower ? color : "#ffffff30"}
          strokeWidth={isPower ? 3 : 2}
          strokeLinecap="round"
        />
        {/* Wing tip glow */}
        <circle cx={wx} cy={wy} r={4}
          fill={isPower ? color : "#ffffff20"}
          style={{ filter: isPower ? `drop-shadow(0 0 4px ${color})` : "none" }}
        />
        {/* Pivot */}
        <circle cx={cx} cy={cy} r={3} fill="#ffffff40" />
        {/* AoA indicator */}
        <text x={50} y={92} textAnchor="middle"
          fill={color} fontSize={9} fontFamily="monospace">
          φ{phi.toFixed(0)}° α{psi.toFixed(0)}°
        </text>
      </svg>
    </div>
  );
}

// ─── Phase Ring ───────────────────────────────────────────────
function PhaseRing({ phaseOffsets, masterDeg }) {
  const cx = 110, cy = 110, r = 80, r2 = 60;
  const wingLabels = ["W1L","W2L","W3L","W4L","W1R","W2R","W3R","W4R"];

  return (
    <svg width={220} height={220} style={{ background: "#060c18", borderRadius: 12 }}>
      {/* Rings */}
      <circle cx={cx} cy={cy} r={r} fill="none" stroke="#ffffff08" strokeWidth={1} />
      <circle cx={cx} cy={cy} r={r2} fill="none" stroke="#ffffff05" strokeWidth={1} />
      {/* Master clock hand */}
      <line
        x1={cx} y1={cy}
        x2={cx + r2 * Math.sin(masterDeg * Math.PI/180)}
        y2={cy - r2 * Math.cos(masterDeg * Math.PI/180)}
        stroke="#ffffff30" strokeWidth={1} strokeDasharray="3,3"
      />
      {/* Wing positions on ring */}
      {phaseOffsets.map((offset, i) => {
        const wingDeg = (masterDeg + offset) % 360;
        const rad = (wingDeg * Math.PI) / 180;
        const x = cx + r * Math.sin(rad);
        const y = cy - r * Math.cos(rad);
        const isPower = wingDeg < 180;
        const color = WING_COLORS[i];
        return (
          <g key={i}>
            <circle cx={x} cy={y} r={7}
              fill={isPower ? color : "#ffffff10"}
              stroke={color} strokeWidth={1}
              style={{ filter: isPower ? `drop-shadow(0 0 6px ${color})` : "none" }}
            />
            <text x={x} y={y + 1} textAnchor="middle" dominantBaseline="middle"
              fontSize={6} fontFamily="monospace" fill={isPower ? "#000" : color}>
              {i+1}
            </text>
          </g>
        );
      })}
      <text x={cx} y={cy} textAnchor="middle" dominantBaseline="middle"
        fill="#ffffff20" fontSize={10} fontFamily="monospace">PHASE</text>
    </svg>
  );
}

// ─── Waveform Chart ───────────────────────────────────────────
function WaveformChart({ phaseOffsets, waveMode, throttle }) {
  const W = 520, H = 120;
  const steps = 200;
  const { aoaPower, aoaRec, strokeAmp } = getModeParams(throttle);

  const paths = phaseOffsets.slice(0,4).map((offset, i) => {
    const pts = [];
    for (let s = 0; s <= steps; s++) {
      const masterDeg = (s / steps) * 360;
      const wingDeg = (masterDeg + offset) % 360;
      const { phi } = resolveKinematics(wingDeg, aoaPower, aoaRec, strokeAmp, 'left');
      const x = (s / steps) * W;
      const y = H/2 - (phi / STROKE_AMP) * (H/2 - 10);
      pts.push(`${x},${y}`);
    }
    return `M ${pts.join(" L ")}`;
  });

  return (
    <svg width={W} height={H} style={{ background: "#060c18", borderRadius: 8, display: "block" }}>
      <line x1={0} y1={H/2} x2={W} y2={H/2} stroke="#ffffff10" strokeWidth={1} />
      {paths.map((d, i) => (
        <path key={i} d={d} fill="none"
          stroke={WING_COLORS[i]} strokeWidth={1.5} opacity={0.85} />
      ))}
      <text x={6} y={14} fontSize={9} fontFamily="monospace" fill="#ffffff40">Left side stroke — W1→W4</text>
      <text x={6} y={H-6} fontSize={9} fontFamily="monospace" fill="#ffffff30">0° ──────────────────── 360°</text>
    </svg>
  );
}

// ─── Stick Input ──────────────────────────────────────────────
function Stick({ label, x, y, onChange }) {
  const ref = useRef(null);
  const dragging = useRef(false);

  const toNorm = (clientX, clientY) => {
    const rect = ref.current.getBoundingClientRect();
    return {
      x: Math.max(-1, Math.min(1, (clientX - rect.left - 60) / 55)),
      y: Math.max(-1, Math.min(1, (clientY - rect.top  - 60) / 55)),
    };
  };

  const onMouseDown = (e) => { dragging.current = true; };
  const onMouseMove = useCallback((e) => {
    if (!dragging.current) return;
    const n = toNorm(e.clientX, e.clientY);
    onChange(n.x, -n.y);
  }, [onChange]);
  const onMouseUp = () => { dragging.current = false; };

  useEffect(() => {
    window.addEventListener("mousemove", onMouseMove);
    window.addEventListener("mouseup", onMouseUp);
    return () => { window.removeEventListener("mousemove", onMouseMove); window.removeEventListener("mouseup", onMouseUp); };
  }, [onMouseMove]);

  const px = 60 + x * 55, py = 60 - y * 55;

  return (
    <div style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: 4 }}>
      <span style={{ fontFamily: "monospace", fontSize: 11, color: "#ffffff60" }}>{label}</span>
      <svg ref={ref} width={120} height={120} onMouseDown={onMouseDown}
        style={{ cursor: "crosshair", background: "#0a0f1a", borderRadius: 10,
                 border: "1px solid #ffffff15" }}>
        <circle cx={60} cy={60} r={55} fill="none" stroke="#ffffff08" />
        <line x1={5} y1={60} x2={115} y2={60} stroke="#ffffff10" />
        <line x1={60} y1={5} x2={60} y2={115} stroke="#ffffff10" />
        <circle cx={px} cy={py} r={10} fill="#4af0c4"
          style={{ filter: "drop-shadow(0 0 8px #4af0c4)" }} />
      </svg>
    </div>
  );
}

// ─── Main App ─────────────────────────────────────────────────
export default function App() {
  const [running, setRunning] = useState(true);
  const [masterDeg, setMasterDeg] = useState(0);
  const [waveMode, setWaveMode] = useState(0);
  const [throttle, setThrottle] = useState(0.5);
  const [rollYaw, setRollYaw]   = useState({ x: 0, y: 0 });
  const [pitchThrottle, setPitchThrottle] = useState({ x: 0, y: 0 });
  const [scenario, setScenario] = useState(null);
  const animRef = useRef(null);
  const lastTime = useRef(null);

  useEffect(() => {
    if (!running) { cancelAnimationFrame(animRef.current); return; }
    const tick = (ts) => {
      if (lastTime.current != null) {
        const dt = (ts - lastTime.current) / 1000;
        setMasterDeg(d => (d + FLAP_FREQ * 360 * dt) % 360);
      }
      lastTime.current = ts;
      animRef.current = requestAnimationFrame(tick);
    };
    animRef.current = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animRef.current);
  }, [running]);

  const applyScenario = (sc) => {
    setScenario(sc.id);
    setWaveMode(sc.mode);
    setThrottle(sc.throttle);
    setRollYaw({ x: sc.yaw, y: 0 });
    setPitchThrottle({ x: 0, y: sc.pitch });
  };

  const roll  = rollYaw.x;
  const yaw   = rollYaw.y;
  const pitch = pitchThrottle.y;
  const thr   = Math.max(0, Math.min(1, throttle + pitchThrottle.x * 0.3));

  const phaseOffsets = getPhaseOffsets(waveMode, yaw);
  const { aoaPower, aoaRec, strokeAmp } = getModeParams(thr);

  const allKinematics = phaseOffsets.map((offset, i) => {
    const side = i < 4 ? 'left' : 'right';
    const wingDeg = (masterDeg + offset) % 360;
    const sideIdx = i % 4;

    let aoa = aoaPower;
    let amp = strokeAmp;

    // Roll mixing
    const rollDelta = roll * strokeAmp * 0.35;
    if (i < 4) amp -= rollDelta; else amp += rollDelta;

    // Pitch mixing
    const pitchDelta = pitch * 12;
    if (sideIdx < 2) aoa += pitchDelta; else aoa -= pitchDelta;

    aoa = Math.max(5, Math.min(50, aoa));
    amp = Math.max(20, Math.min(85, amp));

    return resolveKinematics(wingDeg, aoa, aoaRec, amp, side);
  });

  const wingLabels = ["W1L","W2L","W3L","W4L","W1R","W2R","W3R","W4R"];

  return (
    <div style={{
      minHeight: "100vh", background: "#060c18",
      fontFamily: "monospace", color: "#e2e8f0", padding: 20,
      display: "flex", flexDirection: "column", gap: 16
    }}>
      {/* Header */}
      <div style={{ display: "flex", alignItems: "baseline", gap: 12 }}>
        <h1 style={{ margin: 0, fontSize: 20, letterSpacing: 2, color: "#4af0c4",
          textShadow: "0 0 20px #4af0c480" }}>
          8-WING METACHRONAL DRONE
        </h1>
        <span style={{ fontSize: 11, color: "#ffffff40" }}>kinematic visualizer · 1 Hz flap</span>
        <button
          onClick={() => setRunning(r => !r)}
          style={{
            marginLeft: "auto", padding: "4px 16px",
            background: running ? "#4af0c420" : "#f59e0b20",
            border: `1px solid ${running ? "#4af0c4" : "#f59e0b"}`,
            color: running ? "#4af0c4" : "#f59e0b",
            borderRadius: 4, cursor: "pointer", fontSize: 11, letterSpacing: 1
          }}>
          {running ? "⏸ PAUSE" : "▶ RUN"}
        </button>
      </div>

      {/* Scenarios */}
      <div style={{ display: "flex", gap: 8, flexWrap: "wrap" }}>
        {SCENARIOS.map(sc => (
          <button key={sc.id} onClick={() => applyScenario(sc)}
            style={{
              padding: "4px 12px", fontSize: 10, letterSpacing: 1,
              background: scenario === sc.id ? "#4af0c430" : "transparent",
              border: `1px solid ${scenario === sc.id ? "#4af0c4" : "#ffffff20"}`,
              color: scenario === sc.id ? "#4af0c4" : "#ffffff60",
              borderRadius: 4, cursor: "pointer"
            }}>
            {sc.label.toUpperCase()}
          </button>
        ))}
      </div>

      {/* Main layout */}
      <div style={{ display: "flex", gap: 16, flexWrap: "wrap" }}>

        {/* Wing diagrams — left side */}
        <div style={{ display: "flex", flexDirection: "column", gap: 8 }}>
          <div style={{ fontSize: 10, color: "#4af0c460", letterSpacing: 2 }}>◀ LEFT SIDE</div>
          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 8 }}>
            {allKinematics.slice(0,4).map((kin, i) => (
              <WingDiagram key={i} kinematics={kin} wingIdx={i}
                label={wingLabels[i]} color={WING_COLORS[i]} />
            ))}
          </div>
        </div>

        {/* Centre: phase ring + waveform */}
        <div style={{ display: "flex", flexDirection: "column", gap: 12, flex: 1, minWidth: 240 }}>
          <div style={{ display: "flex", gap: 12, justifyContent: "center" }}>
            <PhaseRing phaseOffsets={phaseOffsets} masterDeg={masterDeg} />
            <div style={{ display: "flex", flexDirection: "column", gap: 8, justifyContent: "center" }}>
              <div style={{ fontSize: 10, color: "#ffffff40", letterSpacing: 1 }}>WAVE MODE</div>
              {WAVE_MODES.map(m => (
                <button key={m.id} onClick={() => { setWaveMode(m.id); setScenario(null); }}
                  style={{
                    padding: "5px 10px", fontSize: 9, textAlign: "left",
                    background: waveMode === m.id ? "#0ea5e930" : "transparent",
                    border: `1px solid ${waveMode === m.id ? "#0ea5e9" : "#ffffff15"}`,
                    color: waveMode === m.id ? "#0ea5e9" : "#ffffff40",
                    borderRadius: 4, cursor: "pointer", letterSpacing: 0.5
                  }}>
                  <div style={{ fontWeight: 700 }}>{m.label}</div>
                  <div style={{ opacity: 0.7 }}>{m.desc}</div>
                </button>
              ))}
            </div>
          </div>

          {/* Waveform */}
          <div>
            <div style={{ fontSize: 10, color: "#ffffff30", marginBottom: 6, letterSpacing: 1 }}>
              STROKE WAVEFORMS — LEFT SIDE W1→W4
            </div>
            <WaveformChart phaseOffsets={phaseOffsets} waveMode={waveMode} throttle={thr} />
          </div>

          {/* Telemetry */}
          <div style={{ display: "grid", gridTemplateColumns: "repeat(4,1fr)", gap: 6 }}>
            {[
              { label: "THROTTLE", value: `${(thr*100).toFixed(0)}%`, color: "#4af0c4" },
              { label: "ROLL", value: `${(roll*100).toFixed(0)}%`, color: "#f59e0b" },
              { label: "PITCH", value: `${(pitch*100).toFixed(0)}%`, color: "#0ea5e9" },
              { label: "YAW", value: `${(yaw*100).toFixed(0)}%`, color: "#e879f9" },
            ].map(t => (
              <div key={t.label} style={{
                background: "#0a0f1a", borderRadius: 6, padding: "6px 10px",
                border: `1px solid ${t.color}20`, textAlign: "center"
              }}>
                <div style={{ fontSize: 8, color: t.color, letterSpacing: 1 }}>{t.label}</div>
                <div style={{ fontSize: 16, color: t.color, fontWeight: 700 }}>{t.value}</div>
              </div>
            ))}
          </div>
        </div>

        {/* Wing diagrams — right side */}
        <div style={{ display: "flex", flexDirection: "column", gap: 8 }}>
          <div style={{ fontSize: 10, color: "#f59e0b60", letterSpacing: 2 }}>RIGHT SIDE ▶</div>
          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 8 }}>
            {allKinematics.slice(4,8).map((kin, i) => (
              <WingDiagram key={i+4} kinematics={kin} wingIdx={i+4}
                label={wingLabels[i+4]} color={WING_COLORS[i+4]} />
            ))}
          </div>
        </div>
      </div>

      {/* Sticks */}
      <div style={{ display: "flex", gap: 24, flexWrap: "wrap", justifyContent: "center", paddingTop: 8 }}>
        <Stick label="ROLL / YAW"
          x={rollYaw.x} y={rollYaw.y}
          onChange={(x,y) => { setRollYaw({x,y}); setScenario(null); }} />

        {/* Throttle slider */}
        <div style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: 6 }}>
          <span style={{ fontSize: 11, color: "#ffffff60" }}>THROTTLE</span>
          <div style={{ height: 120, display: "flex", alignItems: "center" }}>
            <input type="range" min={0} max={100} value={Math.round(throttle*100)}
              onChange={e => { setThrottle(e.target.value/100); setScenario(null); }}
              style={{ writingMode: "vertical-lr", direction: "rtl", height: 110,
                accentColor: "#4af0c4" }} />
          </div>
          <span style={{ fontSize: 14, color: "#4af0c4", fontWeight: 700 }}>
            {Math.round(throttle*100)}%
          </span>
        </div>

        <Stick label="PITCH / (throttle trim)"
          x={pitchThrottle.x} y={pitchThrottle.y}
          onChange={(x,y) => { setPitchThrottle({x,y}); setScenario(null); }} />
      </div>

      {/* Footer */}
      <div style={{ fontSize: 9, color: "#ffffff20", textAlign: "center", letterSpacing: 1, paddingTop: 8 }}>
        PHASE RING: bright = power stroke · dim = recovery · numbers = wing index
        &nbsp;|&nbsp; WAVEFORMS: 4 left-side wings across one full cycle
        &nbsp;|&nbsp; Drag sticks to control · Click scenarios for presets
      </div>
    </div>
  );
}

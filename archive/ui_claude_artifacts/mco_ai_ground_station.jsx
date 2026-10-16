import { useState, useRef, useEffect, useCallback } from "react";

// ═══════════════════════════════════════════════════════════════════════════════
// MCO-1D NATURAL LANGUAGE GROUND STATION
// The pilot speaks plain English. Claude operates the aircraft.
// Instrument panel: six-pack steam gauges + MCO-specific indicators.
// ═══════════════════════════════════════════════════════════════════════════════

// ── Palette: night-panel instrument lighting ──────────────────────────────────
const C = {
  void:      '#05060a',
  cockpit:   '#0c1018',
  panel:     '#131920',
  well:      '#1a2230',
  border:    '#253040',
  mid:       '#3a4f62',
  text:      '#e0e8f0',
  sub:       '#708090',
  ghost:     '#364555',

  // Instrument face: very dark grey-green like real steam gauges
  face:      '#0f1a14',
  faceRim:   '#2a3a2a',
  bezel:     '#1e2a1e',
  scaleMark: '#c8d8b0',
  scaleText: '#a0b898',
  needle:    '#ffffff',
  needleShadow: 'rgba(0,0,0,0.6)',

  // Data colours
  amber:     '#f0a500',
  amberDim:  '#7a5200',
  amberFaint:'#251a00',
  green:     '#3dd68c',
  greenDim:  '#1a5c3a',
  red:       '#ff5050',
  redDim:    '#5c1a1a',
  blue:      '#58a6ff',
  purple:    '#bc8cff',
  white:     '#ffffff',
};

const MONO = "'JetBrains Mono','Fira Code',monospace";
const SANS = "'Inter',system-ui,sans-serif";

// ── System prompt ─────────────────────────────────────────────────────────────
const SYSTEM = `You are the voice of the MCO-1D ornithopter drone's ground station. The pilot talks to you in plain English. You understand their intent, operate the aircraft accordingly, and explain what is happening in plain English. Never show command names, Python, or JSON to the pilot.

THE AIRCRAFT
The MCO-1D is an eight-wing metachronal cascade flapping-wing ornithopter SAR drone. Four tandem pairs of wings (W1 front → W4 rear), each pair port and starboard. Raspberry Pi flight computer. ODrive S1 motor controllers. Pixhawk GPS. Pitot tube airspeed.

FLIGHT MODES
Hover (V < 3 m/s): crew hover ON. Stick = velocity. Lever 50% = altitude hold.
Fixed-wing (V > 5 m/s): normal aero controls. Lever forward = more thrust.
Discovery runs automatically in stable cruise, learning better configurations continuously.

THE LEVER — COMPLETE MAP
0–5%    Maximum burst / sprint
5–28%   Sprint modes (maximum thrust)
28–45%  Powered cruise
45–55%  Altitude hold (50% = exact hold)
55–75%  Glide / best range (kestrel can engage here)
75–80%  Lift dump partial (steeper approach)
80–87%  Lift dump full (full spoilers)
87–93%  Shuttle brake partial
93–97%  J-hook (post-stall perch, needs 20m+)
97–100% Shuttle brake full (~2g emergency stop)

VIFF (Vectoring In Forward Flight)
At hard roll inputs in cruise, the aircraft automatically uses differential thrust vectoring — outside-turn wings increase angle of attack, inside wings decrease it. Invisible to the pilot. Just feels like a more responsive aircraft.

THE AIRCRAFT LEARNS
Discovery runs during stable cruise. Tests wing configurations, keeps winners, discards losers. MAP-Elites archive builds a champion for every airspeed × wind × intent combination. Strategy genome evolves mechanism weights for each manoeuvre type. All learning saves after each flight and carries forward.

MISSION FOCI (tell the aircraft what to get good at):
"gusty / rough air / wind" → focus gust_tolerance
"hold position / drifting" → focus position_hold
"smooth / payload / camera" → focus smooth_flight
"crosswind landing" → focus crosswind_landing
"battery / stay up / endurance" → focus endurance
"fast / top speed" → focus sprint
"SAR / rescue / hover ops" → focus sar_hover
"storm / heavy weather" → focus storm_resilience
"agile / tight turns / aerobatics" → focus aerobatics
"kestrel / updraft / soaring" → focus kestrel_hover
"normal / general / clear" → focus clear

AUTONOMOUS LEARNING SORTIES (ALSS)
"start practice flights" / "autonomous learning" → alss_start (after alss_set_fence)
Aircraft takes off, practices manoeuvres, lands, waits for battery swap, repeats.
Class A (first 5): orientation. Class B (6–20): exploration. Class C (21+): performance.

CEILING EXPLORATION — DOUBLE-GATED PROCEDURE
Two steps required. Never skip. Touches regulated airspace.
Step 1: Say "arm ceiling exploration" → AI shows checklist, arms for 60 seconds
Step 2: Say "confirm ceiling exploration" within 60 seconds → begins probe
Never enter Class D (ceiling) through ALSS — only through this explicit procedure.

YOUR BEHAVIOUR
1. Understand intent, not just words.
   "make it stop" → position_hold_engage
   "we're done" (on ground) → persist_checkpoint then disarm
   "it feels wobbly" → fault_summary, attitude_hold_status, explain
   "teach it to turn better" → focus aerobatics, discovery_master_on
   "that worked well" → gs_inject_note

2. Send commands, read the result, explain in plain English. Never show command names.

3. Translate telemetry into plain English.
   V_ms → airspeed. h_agl_m → height. motor_I_A × bus_voltage → power.
   fault_count > 0 → faults present, check immediately.
   alss_phase → what autonomous practice is doing.

4. Proactive alerts — speak up without being asked:
   airspeed dropping while lever is high → mention stall risk
   bus_voltage < 20.0 → battery low, suggest landing
   fault_count > 0 → flag immediately, read fault_summary
   ce_lev_risk > 0.5 → wing stall risk at altitude, suggest descent

5. Concise. The pilot is flying. One or two sentences then wait.
   More detail only if they ask.

6. Confirm before: arm, emergency_stop, disarm in flight, ceiling_explore_confirm.

VSI REFERENCE MARKER — ALWAYS ACTIVE
The VSI gauge always shows a reference bug (amber triangle) at the pilot's target rate.
The altimeter always shows an altitude bug when set (amber triangle on the gauge face).
The status strip shows heading and altitude bugs when set, with live error from target.
None of these do anything to the aircraft. They are reminders for the pilot.

"target 100 ft/min down" / "descent rate 100" / "bug the VSI at 100 down" → vsi_ref -100
"500 ft/min descent" → vsi_ref -500   |   "200 ft/min climb" → vsi_ref 200
"clear the VSI bug" → vsi_ref_clear

"set heading bug to 090" / "target heading 270" / "fly heading 180" → hdg_ref 090
"heading bug north" → hdg_ref 360   |   "clear heading bug" → hdg_ref_clear
The status strip shows: HDG▶ 090°  +5° (error from current heading, green/amber/red)

"altitude bug 1000 feet" / "target altitude 500" / "set the altimeter bug" → alt_ref 1000
"300 metres" → alt_ref 300 m   |   "clear the altitude bug" → alt_ref_clear
The altimeter triangle marks the target; status strip shows ALT▶ 1000ft  +50 (error)

GS: precision_status — all three bugs plus trim quality in one report.

AI OBSERVER — HEALTH AND LEARNING MONITOR
The aircraft has a continuous observer that watches health and learning quality.
It runs silently. It speaks only when it has something genuinely useful to say — typically once or twice per flight. When it does speak, relay its message directly to the pilot in plain English, then wait.

The observer watches:
HEALTH: battery drain rate, power anomalies, vibration trends, wing guard event rate, rope fatigue rate, trim drift, roll asymmetry, oscillation onset, power vs airspeed anomalies.
LEARNING: discovery Ps trend, hypothesis champion clustering, genome weight evolution, ALSS progress, MAP-Elites archive coverage.

When obs_messages appears in telemetry, relay the message to the pilot exactly as written. Then ask: "Want me to try it?" if the message ends with a proposal.

If pilot says yes / go ahead / try it / approve → GS: observer_approve
If pilot says no / not now / skip → GS: observer_decline
"Observer report / what has the AI noticed / learning quality" → GS: observer_report
"Flight monitor / health report / any alerts" → GS: monitor_report
"Monitor thresholds / what does it watch for" → GS: monitor_thresholds

The observer never acts without pilot permission. It proposes. The pilot decides.

WING COORDINATION AND CLAP-AND-FLING
The aircraft has a real-time wing guard (always active) that prevents port-stbd tip collisions.
It enforces a phase-dependent amplitude envelope automatically — the pilot feels nothing.
"enable clap and fling" / "clap mode" / "maximum hover thrust" → clap_enable dorsal
  Opens clap gate. Wings contact at stroke reversal. ~15-25% hover lift enhancement.
"disable clap" / "normal wings" → clap_disable
"wing clearances" / "tip separations" / "any wing conflicts" → clap_status / wing_guard_status
"anti-phase cruise" / "maximum amplitude" → endurance focus + H-WC-2 config (115° amplitude)

FOREST FLIGHT AND OBSTACLE AVOIDANCE
"going into the forest" / "confined space" / "enable obstacle avoidance" → spatial_awareness_enable
"obstacle status" / "what can you see" / "how close is anything" → obstacle_status
"the gap is X metres wide" / "there's a 60cm gap" → gap_set_width [metres]
  > 0.55m: flapping passage   0.20-0.55m: sideways passage   0.12-0.20m: ballistic tuck
"go through the gap" / "enter through the opening" → gap_enter
"approach through the trees" / "fly in under the canopy" → canopy_approach
"land here on the ledge / rubble / slope" → perch_land
"get out of the space" / "climb out" → confined_exit [height]
"disable obstacle avoidance" → spatial_awareness_disable

CEILING EXPLORATION — DOUBLE-GATED (airspace implications)
Step 1: "arm ceiling exploration" → ceiling_explore_arm (prints checklist, 60s window opens)
Step 2: "confirm ceiling exploration" (within 60s) → ceiling_explore_confirm → probe begins
"ceiling status" → ceiling_explore_status   "abort ceiling test" → ceiling_explore_abort
Never skip either step. Never combine into one command.

EXAMPLES
"Ready to fly" → read fault_summary + soft_stop_status + attitude_hold_status. If clear: "All checks pass. Ready to arm."
"Arm it" → confirm once, send arm, report result.
"Enable clap and fling" → clap_enable dorsal. Explain the lift enhancement and that the guard validates timing.
"The gap is about half a metre" → gap_set_width 0.5 → explain sideways passage strategy → ask if ready to enter.
"Start practice programme" → check alss_set_fence done, if not ask for position. Then alss_start.
"J-hook" → check h_agl_m > 20. If yes explain and jhook_engage. If no: "Need at least 20 metres. Currently at Xm."
"Arm ceiling exploration" → ceiling_explore_arm → read out the checklist items → wait for confirmation.`;

// ── WebSocket hook ─────────────────────────────────────────────────────────────
function useGS(url) {
  const [connected, setConnected] = useState(false);
  const [telem, setTelem]         = useState({});
  const [log, setLog]             = useState([]);
  const ws    = useRef(null);
  const ackCb = useRef(null);

  const addLog = useCallback((e) => setLog(p => [...p.slice(-399), e]), []);

  const connect = useCallback(() => {
    if (ws.current?.readyState === WebSocket.OPEN) return;
    const s = new WebSocket(url);
    ws.current = s;
    s.onopen  = () => { setConnected(true);  addLog({t:'sys', text:'Aircraft connected'}); };
    s.onclose = () => { setConnected(false); addLog({t:'sys', text:'Connection lost — reconnecting…'}); setTimeout(connect, 3000); };
    s.onerror = () => addLog({t:'err', text:'WebSocket error'});
    s.onmessage = (e) => {
      try {
        const m = JSON.parse(e.data);
        if (m.type === 'telemetry') setTelem(p => ({...p, ...m.data}));
        else if (m.type === 'ack') {
          addLog({t:'rx', text: m.msg || JSON.stringify(m)});
          if (ackCb.current) { ackCb.current(m); ackCb.current = null; }
        } else addLog({t:'rx', text: JSON.stringify(m)});
      } catch { addLog({t:'rx', text: e.data}); }
    };
  }, [url, addLog]);

  useEffect(() => { connect(); return () => ws.current?.close(); }, [connect]);

  const send = useCallback((cmd) => new Promise(resolve => {
    addLog({t:'tx', text: cmd});
    if (ws.current?.readyState === WebSocket.OPEN) {
      ackCb.current = resolve;
      ws.current.send(JSON.stringify({type:'cmd', cmd}));
      setTimeout(() => { if (ackCb.current) { ackCb.current({msg:'(no ack)'}); ackCb.current=null; } }, 5000);
    } else resolve({msg:'Not connected'});
  }), [addLog]);

  return { connected, telem, log, send };
}

// ═══════════════════════════════════════════════════════════════════
// STEAM GAUGE COMPONENTS
// Each instrument is a proper analogue round gauge:
// bezel ring, dark face, arc scale markings, white needle,
// bottom label. Faithful to the six-pack aesthetic.
// ═══════════════════════════════════════════════════════════════════

function polarToXY(cx, cy, r, angleDeg) {
  const a = (angleDeg - 90) * Math.PI / 180;
  return { x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) };
}

function arcPath(cx, cy, r, startDeg, endDeg) {
  const s = polarToXY(cx, cy, r, startDeg);
  const e = polarToXY(cx, cy, r, endDeg);
  const large = (endDeg - startDeg) > 180 ? 1 : 0;
  return `M ${s.x} ${s.y} A ${r} ${r} 0 ${large} 1 ${e.x} ${e.y}`;
}

// Draw tick marks around the gauge face
function ScaleTicks({ cx, cy, r, startDeg, endDeg, majorCount, minorCount=4, label, labelValues }) {
  const ticks = [];
  const total = majorCount * (minorCount + 1);
  const step  = (endDeg - startDeg) / total;

  for (let i = 0; i <= total; i++) {
    const angle = startDeg + i * step;
    const isMajor = (i % (minorCount + 1) === 0);
    const len   = isMajor ? 8 : 4;
    const outer = polarToXY(cx, cy, r - 2, angle);
    const inner = polarToXY(cx, cy, r - 2 - len, angle);
    ticks.push(
      <line key={`t${i}`}
        x1={outer.x} y1={outer.y} x2={inner.x} y2={inner.y}
        stroke={C.scaleMark} strokeWidth={isMajor ? 1.5 : 0.8}
        strokeLinecap="round" />
    );
    // Major tick labels
    if (isMajor && labelValues) {
      const majorIdx = i / (minorCount + 1);
      if (labelValues[majorIdx] !== undefined) {
        const lp = polarToXY(cx, cy, r - 16, angle);
        ticks.push(
          <text key={`l${i}`} x={lp.x} y={lp.y}
            textAnchor="middle" dominantBaseline="middle"
            fontSize="7" fill={C.scaleText} fontFamily={MONO}>
            {labelValues[majorIdx]}
          </text>
        );
      }
    }
  }
  return <>{ticks}</>;
}

// Coloured arc on the scale (green normal band, yellow caution, red limit)
function ScaleArc({ cx, cy, r, startDeg, endDeg, color, width=4 }) {
  return (
    <path d={arcPath(cx, cy, r - 5, startDeg, endDeg)}
      fill="none" stroke={color} strokeWidth={width}
      strokeLinecap="round" opacity={0.85} />
  );
}

// The needle — tapered, white, with a pivot cap
function Needle({ cx, cy, r, angleDeg, color='#ffffff', length=0.72 }) {
  const tipR   = r * length;
  const tailR  = r * 0.18;
  const tipPt  = polarToXY(cx, cy, tipR, angleDeg);
  const tailPt = polarToXY(cx, cy, -tailR, angleDeg);
  const lw1    = polarToXY(cx, cy, tipR * 0.7, angleDeg - 90);
  const lw2    = polarToXY(cx, cy, tipR * 0.7, angleDeg + 90);
  const path   = `M ${tailPt.x} ${tailPt.y} L ${lw1.x} ${lw1.y} L ${tipPt.x} ${tipPt.y} L ${lw2.x} ${lw2.y} Z`;
  return (
    <>
      {/* Shadow */}
      <path d={path} fill="rgba(0,0,0,0.35)" transform={`translate(1,1)`} />
      {/* Needle */}
      <path d={path} fill={color} />
      {/* Pivot */}
      <circle cx={cx} cy={cy} r={5} fill={C.mid} />
      <circle cx={cx} cy={cy} r={3} fill={C.scaleMark} />
    </>
  );
}

// The gauge shell: SVG circle with bezel, face, glass sheen
function GaugeShell({ size, label, sublabel, warn, crit, children }) {
  const r    = size / 2;
  const bezelColor = crit ? C.red : warn ? C.amber : '#2a3a2e';
  const glowColor  = crit ? C.red : warn ? C.amber : 'transparent';
  return (
    <div style={{ display:'flex', flexDirection:'column', alignItems:'center', gap:4 }}>
      <svg width={size} height={size} viewBox={`0 0 ${size} ${size}`}
           style={{ filter: (warn||crit) ? `drop-shadow(0 0 6px ${glowColor})` : 'none',
                    transition: 'filter 0.4s' }}>
        {/* Outer bezel */}
        <circle cx={r} cy={r} r={r-1} fill={bezelColor} />
        {/* Bezel highlight ring */}
        <circle cx={r} cy={r} r={r-3} fill="none"
          stroke="rgba(255,255,255,0.08)" strokeWidth={1.5} />
        {/* Face */}
        <circle cx={r} cy={r} r={r-5} fill={C.face} />
        {/* Inner scale ring */}
        <circle cx={r} cy={r} r={r-6} fill="none"
          stroke={C.faceRim} strokeWidth={0.5} />
        {children}
        {/* Glass sheen — top-left highlight */}
        <ellipse cx={r*0.65} cy={r*0.52} rx={r*0.28} ry={r*0.18}
          fill="rgba(255,255,255,0.04)" transform={`rotate(-30,${r*0.65},${r*0.52})`} />
      </svg>
      <div style={{ fontFamily:MONO, fontSize:10, color:
                      crit ? C.red : warn ? C.amber : C.scaleText,
                    letterSpacing:'0.06em', textTransform:'uppercase',
                    transition:'color 0.4s', textAlign:'center', lineHeight:1.2 }}>
        {label}
        {sublabel && <div style={{ fontSize:8, color:C.sub, marginTop:1 }}>{sublabel}</div>}
      </div>
    </div>
  );
}

// ── 1. AIRSPEED INDICATOR ─────────────────────────────────────────────────────
// 0–30 m/s, spans 40°–320° (280° sweep). Green band 5–20, caution 20–26, red 26+
function AirspeedGauge({ V_ms=0, size=110 }) {
  const r       = size/2;
  const cx=r, cy=r;
  const vMax    = 30;
  const START   = 40;  const END = 320;
  const valToDeg= v => START + (v/vMax)*(END-START);
  const needleDeg = valToDeg(Math.min(vMax, Math.max(0, V_ms)));
  const warn = V_ms > 20;
  const crit = V_ms > 26;

  return (
    <GaugeShell size={size} label="Airspeed" sublabel="m/s" warn={warn} crit={crit}>
      {/* Coloured arcs */}
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={valToDeg(0)}   endDeg={valToDeg(4.85)} color={C.red}   width={4} />
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={valToDeg(4.85)}endDeg={valToDeg(20)}   color="#2ea043" width={4} />
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={valToDeg(20)}  endDeg={valToDeg(26)}   color={C.amber} width={4} />
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={valToDeg(26)}  endDeg={valToDeg(30)}   color={C.red}   width={4} />
      <ScaleTicks cx={cx} cy={cy} r={r-5} startDeg={START} endDeg={END}
        majorCount={6} minorCount={4} labelValues={[0,5,10,15,20,25,30]} />
      <Needle cx={cx} cy={cy} r={r-8} angleDeg={needleDeg} />
      {/* Digital readout at bottom */}
      <text x={cx} y={cy+r*0.42} textAnchor="middle" fontSize="11"
        fill={crit?C.red:warn?C.amber:C.amber} fontFamily={MONO} fontWeight="700">
        {V_ms?.toFixed?.(1) ?? '—'}
      </text>
    </GaugeShell>
  );
}

// ── 2. ATTITUDE INDICATOR ─────────────────────────────────────────────────────
// Shows pitch (up/down horizon) and bank (roll). Mimics real AI with
// blue sky / brown earth split and bank arc at top.
function AttitudeGauge({ roll_deg=0, pitch_deg=0, size=110 }) {
  const r = size/2; const cx=r, cy=r;
  const clampPitch = Math.max(-30, Math.min(30, pitch_deg ?? 0));
  const clampRoll  = Math.max(-60, Math.min(60, roll_deg  ?? 0));
  // Horizon offset: 1px per degree of pitch, capped
  const horizY = cy + (clampPitch / 30) * (r * 0.55);
  const warn = Math.abs(clampRoll) > 30 || Math.abs(clampPitch) > 15;
  const crit = Math.abs(clampRoll) > 45 || Math.abs(clampPitch) > 25;

  return (
    <GaugeShell size={size} label="Attitude" sublabel={`P:${(pitch_deg??0).toFixed(0)}° R:${(roll_deg??0).toFixed(0)}°`} warn={warn} crit={crit}>
      <defs>
        <clipPath id="faceClip">
          <circle cx={cx} cy={cy} r={r-6} />
        </clipPath>
      </defs>
      {/* Earth + sky rotated by roll */}
      <g clipPath="url(#faceClip)"
         transform={`rotate(${-clampRoll},${cx},${cy})`}>
        {/* Sky */}
        <rect x={0} y={0} width={size} height={horizY}
          fill="#1a3a6a" />
        {/* Earth */}
        <rect x={0} y={horizY} width={size} height={size-horizY}
          fill="#5c3010" />
        {/* Horizon line */}
        <line x1={0} y1={horizY} x2={size} y2={horizY}
          stroke={C.scaleText} strokeWidth={1.5} />
        {/* Pitch lines */}
        {[-20,-10,10,20].map(p => {
          const py = horizY + (p/30)*(r*0.55);
          const hw = p % 20 === 0 ? 22 : 14;
          return <line key={p} x1={cx-hw} y1={py} x2={cx+hw} y2={py}
            stroke={C.scaleText} strokeWidth={0.8} opacity={0.7} />;
        })}
      </g>
      {/* Fixed miniature aircraft (always centred, not rotated) */}
      <g>
        <line x1={cx-22} y1={cy} x2={cx-8} y2={cy} stroke={C.amber} strokeWidth={2.5} strokeLinecap="round" />
        <line x1={cx+8}  y1={cy} x2={cx+22} y2={cy} stroke={C.amber} strokeWidth={2.5} strokeLinecap="round" />
        <circle cx={cx} cy={cy} r={3} fill={C.amber} />
        {/* Centre dot */}
        <line x1={cx} y1={cy-5} x2={cx} y2={cy-10} stroke={C.amber} strokeWidth={1.5} />
      </g>
      {/* Bank scale arc — matches real AI instrument:
           0° fixed triangle at top (wings-level index)
           ±10°, ±20° short ticks
           ±30° longer bold ticks  (major quick-reference mark)
           ±45° smaller intermediate ticks
           ±60° major ticks
           ±90° flat on the horizon line, each side            */}
      <g>
        {/* ±10° and ±20° — short ticks */}
        {[-20,-10,10,20].map(b => {
          const bAngle = (-90 + b) * Math.PI / 180;
          const ar = r - 8;
          const x1 = cx + ar     * Math.cos(bAngle);
          const y1 = cy + ar     * Math.sin(bAngle);
          const x2 = cx + (ar-5) * Math.cos(bAngle);
          const y2 = cy + (ar-5) * Math.sin(bAngle);
          return <line key={b} x1={x1} y1={y1} x2={x2} y2={y2}
            stroke={C.scaleMark} strokeWidth={0.8} />;
        })}
        {/* ±30° — longer bold ticks (major 30° mark) */}
        {[-30,30].map(b => {
          const bAngle = (-90 + b) * Math.PI / 180;
          const ar = r - 8;
          const x1 = cx + ar     * Math.cos(bAngle);
          const y1 = cy + ar     * Math.sin(bAngle);
          const x2 = cx + (ar-8) * Math.cos(bAngle);
          const y2 = cy + (ar-8) * Math.sin(bAngle);
          return <line key={b} x1={x1} y1={y1} x2={x2} y2={y2}
            stroke={C.scaleMark} strokeWidth={2.0} />;
        })}
        {/* ±45° — smaller intermediate ticks */}
        {[-45,45].map(b => {
          const bAngle = (-90 + b) * Math.PI / 180;
          const ar = r - 8;
          const x1 = cx + ar     * Math.cos(bAngle);
          const y1 = cy + ar     * Math.sin(bAngle);
          const x2 = cx + (ar-4) * Math.cos(bAngle);
          const y2 = cy + (ar-4) * Math.sin(bAngle);
          return <line key={b} x1={x1} y1={y1} x2={x2} y2={y2}
            stroke={C.scaleMark} strokeWidth={0.8} />;
        })}
        {/* ±60° — major ticks */}
        {[-60,60].map(b => {
          const bAngle = (-90 + b) * Math.PI / 180;
          const ar = r - 8;
          const x1 = cx + ar     * Math.cos(bAngle);
          const y1 = cy + ar     * Math.sin(bAngle);
          const x2 = cx + (ar-7) * Math.cos(bAngle);
          const y2 = cy + (ar-7) * Math.sin(bAngle);
          return <line key={b} x1={x1} y1={y1} x2={x2} y2={y2}
            stroke={C.scaleMark} strokeWidth={1.5} />;
        })}
        {/* ±90° — flat on the horizon line, one each side */}
        <line x1={cx-(r-8)} y1={cy} x2={cx-(r-15)} y2={cy}
          stroke={C.scaleMark} strokeWidth={1.5} />
        <line x1={cx+(r-8)} y1={cy} x2={cx+(r-15)} y2={cy}
          stroke={C.scaleMark} strokeWidth={1.5} />
        {/* 0° — fixed white triangle at very top centre (wings-level index) */}
        <polygon
          points={`${cx},${cy-(r-8)} ${cx-4},${cy-(r-15)} ${cx+4},${cy-(r-15)}`}
          fill={C.scaleMark} />
        {/* Bank pointer — amber triangle rotates with aircraft roll */}
        <polygon
          points={`${cx},${cy-(r-10)} ${cx-4},${cy-(r-18)} ${cx+4},${cy-(r-18)}`}
          fill={C.amber}
          transform={`rotate(${clampRoll},${cx},${cy})`} />
      </g>
    </GaugeShell>
  );
}

// ── 3. ALTIMETER ──────────────────────────────────────────────────────────────
// 0–500m, two-pointer style (main: 0–50m sweep, minor: 0–500m)
function AltimeterGauge({ h_agl_m=0, alt_ref_m=null, size=110 }) {
  const r=size/2; const cx=r,cy=r;
  const hMax=500;
  const START=40; const END=320;
  // Main needle: one full sweep = 50m
  const h1 = ((h_agl_m??0) % 50) / 50;
  const mainDeg = START + h1*(END-START);
  // Minor needle: 0–500m total
  const h2 = Math.min(1, (h_agl_m??0)/hMax);
  const minorDeg = START + h2*(END-START);
  const warn = h_agl_m > 80;
  const crit = h_agl_m > 120;

  // Altitude bug — amber triangle on the 0-500m scale (minor needle scale)
  const bugSet = alt_ref_m !== null && alt_ref_m !== undefined;
  const bugFrac = bugSet ? Math.min(1, Math.max(0, (alt_ref_m??0)/hMax)) : 0;
  const bugDeg  = START + bugFrac*(END-START);
  const altNow  = h_agl_m??0;
  const altErr  = bugSet ? ((alt_ref_m??0) - altNow) : 0;
  const errColor = bugSet
    ? (Math.abs(altErr) < 5 ? '#2ea043' : Math.abs(altErr) < 20 ? '#f0a500' : '#cf222e')
    : C?.amber ?? '#f0a500';

  return (
    <GaugeShell size={size} label="Altitude" sublabel="m AGL" warn={warn} crit={crit}>
      <ScaleTicks cx={cx} cy={cy} r={r-5} startDeg={START} endDeg={END}
        majorCount={5} minorCount={4} labelValues={[0,10,20,30,40,50]} />

      {/* Altitude bug — triangle on the outer arc (minor needle scale) */}
      {bugSet && (
        <g transform={`rotate(${bugDeg}, ${cx}, ${cy})`}>
          <polygon
            points={`${cx},${cy-(r-14)} ${cx-5},${cy-(r-6)} ${cx+5},${cy-(r-6)}`}
            fill="#f0a500" stroke="#0d1117" strokeWidth="0.8" opacity="0.95"
          />
        </g>
      )}

      {/* Long needle: 0–50m */}
      <Needle cx={cx} cy={cy} r={r-8} angleDeg={mainDeg} />
      {/* Short needle: 0–500m */}
      <Needle cx={cx} cy={cy} r={r-8} angleDeg={minorDeg} color="#a0b898" length={0.45} />

      {/* Digital readout: current altitude, error from bug if set */}
      <text x={cx} y={cy+r*0.38} textAnchor="middle" fontSize="11"
        fill={errColor} fontFamily={MONO} fontWeight="700">
        {(h_agl_m??0).toFixed(0)}
      </text>
      {bugSet && (
        <text x={cx} y={cy+r*0.52} textAnchor="middle" fontSize="8"
          fill="#f0a500" fontFamily={MONO} opacity="0.85">
          ▶{(alt_ref_m??0).toFixed(0)}m
        </text>
      )}
    </GaugeShell>
  );
}

// ── 4. VERTICAL SPEED INDICATOR ───────────────────────────────────────────────
// -5 to +5 m/s. Centre=0, up=climb, down=descend.
function VSIGauge({ sink_rate_ms=0, vsi_ref_fpm=0, size=110 }) {
  const r=size/2; const cx=r,cy=r;
  const vsiMax=5;
  // 0 is at centre of sweep. Climb positive (up), descent negative (down).
  // Display in ft/min but needle position computed from m/s.
  const START=40; const END=320; const MID=(START+END)/2;
  // smooth VSI — stroke pulsation already filtered by 5-beat average
  const v = Math.max(-vsiMax, Math.min(vsiMax, -(sink_rate_ms??0)));
  const needleDeg = MID + (v/vsiMax)*(MID-START);
  const warn = Math.abs(v) > 2;
  const crit = Math.abs(v) > 3.5;

  // Reference bug — always displayed, in same units (m/s internally)
  const vRefMs = (vsi_ref_fpm??0) * 0.00508;          // ft/min → m/s (climb positive)
  const vRefClamped = Math.max(-vsiMax, Math.min(vsiMax, vRefMs));
  const refDeg = MID + (vRefClamped/vsiMax)*(MID-START);

  // Error colour: green if needle near bug, amber if >50 fpm off, red if >150 fpm
  const errFpm = Math.abs((-(sink_rate_ms??0)/0.00508) - (vsi_ref_fpm??0));
  const errColor = errFpm < 50 ? '#2ea043' : errFpm < 150 ? '#f0a500' : '#cf222e';

  // Display value in ft/min (easier to read for precision flying)
  const vFpm = -(sink_rate_ms??0) / 0.00508;
  const refFpm = vsi_ref_fpm??0;

  return (
    <GaugeShell size={size} label="Vert Speed" sublabel="ft/min" warn={warn} crit={crit}>
      {/* Green zero band */}
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={MID-8} endDeg={MID+8} color="#2ea043" width={4} />
      <ScaleTicks cx={cx} cy={cy} r={r-5} startDeg={START} endDeg={END}
        majorCount={10} minorCount={1} labelValues={['-5','','-3','','-1','0','1','','3','','5']} />

      {/* Reference bug — permanent triangle marker at the target rate */}
      {/* Triangle points outward from the scale arc */}
      <g transform={`rotate(${refDeg}, ${cx}, ${cy})`}>
        <polygon
          points={`${cx},${cy-(r-14)} ${cx-5},${cy-(r-6)} ${cx+5},${cy-(r-6)}`}
          fill="#f0a500" stroke="#0d1117" strokeWidth="0.8"
          opacity="0.95"
        />
      </g>

      {/* Needle */}
      <Needle cx={cx} cy={cy} r={r-8} angleDeg={needleDeg}
        color={crit?C.red:warn?C.amber:C.white} />

      {/* Digital readout: current / reference */}
      <text x={cx} y={cy+r*0.38} textAnchor="middle" fontSize="9"
        fill={errColor} fontFamily={MONO} fontWeight="700">
        {vFpm>0?'+':''}{Math.round(vFpm)}
      </text>
      <text x={cx} y={cy+r*0.52} textAnchor="middle" fontSize="8"
        fill="#f0a500" fontFamily={MONO} opacity="0.85">
        ▶{refFpm>0?'+':''}{Math.round(refFpm)}
      </text>
    </GaugeShell>
  );
}

// ── 5. POWER / HEADING INDICATOR ─────────────────────────────────────────────
// We replace the heading indicator with a POWER gauge (kW drawn)
// since the MCO-1D doesn't have a traditional compass heading as primary.
// Sweeps 0–5 kW. Green 0–3, amber 3–4, red 4+
function PowerGauge({ motor_I_A=0, bus_voltage=22.2, size=110 }) {
  const r=size/2; const cx=r,cy=r;
  const P_kw = Math.max(0, (motor_I_A??0)*(bus_voltage??22.2)/1000);
  const pMax=5;
  const START=40; const END=320;
  const valToDeg = v => START + (v/pMax)*(END-START);
  const needleDeg = valToDeg(Math.min(pMax, P_kw));
  const warn = P_kw > 3.0;
  const crit = P_kw > 4.2;

  return (
    <GaugeShell size={size} label="Power" sublabel="kW" warn={warn} crit={crit}>
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={valToDeg(0)}   endDeg={valToDeg(3)} color="#2ea043" width={4} />
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={valToDeg(3)}   endDeg={valToDeg(4.2)} color={C.amber} width={4} />
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={valToDeg(4.2)} endDeg={valToDeg(5)} color={C.red} width={4} />
      <ScaleTicks cx={cx} cy={cy} r={r-5} startDeg={START} endDeg={END}
        majorCount={5} minorCount={4} labelValues={[0,1,2,3,4,5]} />
      <Needle cx={cx} cy={cy} r={r-8} angleDeg={needleDeg}
        color={crit?C.red:warn?C.amber:C.white} />
      <text x={cx} y={cy+r*0.42} textAnchor="middle" fontSize="11"
        fill={crit?C.red:warn?C.amber:C.amber} fontFamily={MONO} fontWeight="700">
        {P_kw.toFixed(2)}
      </text>
    </GaugeShell>
  );
}

// ── 6. BATTERY GAUGE ──────────────────────────────────────────────────────────
// Mimics a fuel gauge. 18V–25.2V sweep. Green 23–25.2, amber 21–23, red below 21.
function BatteryGauge({ bus_voltage=22.2, size=110 }) {
  const r=size/2; const cx=r,cy=r;
  const vMin=18, vMax=25.2;
  const START=40; const END=320;
  const valToDeg = v => START + ((v-vMin)/(vMax-vMin))*(END-START);
  const needleDeg = valToDeg(Math.min(vMax, Math.max(vMin, bus_voltage??22.2)));
  const warn = (bus_voltage??0) < 21.5;
  const crit = (bus_voltage??0) < 20.0;

  return (
    <GaugeShell size={size} label="Battery" sublabel="volts" warn={warn} crit={crit}>
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={valToDeg(18)}   endDeg={valToDeg(20)}   color={C.red}   width={4} />
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={valToDeg(20)}   endDeg={valToDeg(21.5)} color={C.amber} width={4} />
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={valToDeg(21.5)} endDeg={valToDeg(23)}   color={C.amber} width={3} />
      <ScaleArc cx={cx} cy={cy} r={r-5} startDeg={valToDeg(23)}   endDeg={valToDeg(25.2)} color="#2ea043" width={4} />
      <ScaleTicks cx={cx} cy={cy} r={r-5} startDeg={START} endDeg={END}
        majorCount={6} minorCount={3} labelValues={['18','19.2','20.4','21.6','22.8','24','25.2']} />
      <Needle cx={cx} cy={cy} r={r-8} angleDeg={needleDeg}
        color={crit?C.red:warn?C.amber:C.white} />
      <text x={cx-r*0.52} y={cy+r*0.35} fontSize="9" fill={C.red} fontFamily={MONO} fontWeight="700">E</text>
      <text x={cx+r*0.45} y={cy+r*0.35} fontSize="9" fill={C.green} fontFamily={MONO} fontWeight="700">F</text>
      <text x={cx} y={cy+r*0.42} textAnchor="middle" fontSize="11"
        fill={crit?C.red:warn?C.amber:C.amber} fontFamily={MONO} fontWeight="700">
        {(bus_voltage??0).toFixed(1)}V
      </text>
    </GaugeShell>
  );
}

// ── 7. TURN COORDINATOR ───────────────────────────────────────────────────────
// Standard TC: miniature aircraft banks to show rate of turn.
// Inclinometer ball (slip/skid) shown as a simple lateral displacement.
// Turn rate from roll_rate_dps. Ball from lateral accel (accel_y_g).
function TurnCoordGauge({ roll_rate_dps=0, accel_y_g=0, size=110 }) {
  const r = size/2; const cx=r, cy=r;

  // Standard rate-one turn = 3 deg/s. Full deflection at 6 deg/s.
  const MAX_RATE = 6;
  const bankAngle = Math.max(-30, Math.min(30,
    (roll_rate_dps??0) / MAX_RATE * 30));

  // Inclinometer ball — lateral accel displaces the ball left/right
  // Positive accel_y = skid right, ball goes left (and vice versa)
  const ballX = cx - Math.max(-18, Math.min(18, (accel_y_g??0) * 40));
  const ballY  = cy + r * 0.38;

  // Rate marks: L and R triangles at ±standard rate (3 deg/s)
  const stdMark = 20; // degrees of bank at standard rate

  return (
    <GaugeShell size={size} label="Turn Coord" sublabel="rate" warn={false} crit={false}>
      {/* Standard rate index marks */}
      <polygon points={`${cx - stdMark - 4},${cy - r*0.55} ${cx - stdMark + 4},${cy - r*0.55} ${cx - stdMark},${cy - r*0.42}`}
        fill={C.white} opacity="0.7" />
      <polygon points={`${cx + stdMark - 4},${cy - r*0.55} ${cx + stdMark + 4},${cy - r*0.55} ${cx + stdMark},${cy - r*0.42}`}
        fill={C.white} opacity="0.7" />

      {/* Miniature aircraft — rotates with bank */}
      <g transform={`rotate(${bankAngle}, ${cx}, ${cy})`}>
        {/* Fuselage */}
        <rect x={cx-1} y={cy-r*0.30} width={2} height={r*0.22}
          fill={C.amber} rx={1} />
        {/* Wings */}
        <rect x={cx-r*0.42} y={cy-r*0.12} width={r*0.84} height={3}
          fill={C.amber} rx={1} />
        {/* Tail */}
        <rect x={cx-r*0.14} y={cy-r*0.30} width={r*0.28} height={2.5}
          fill={C.amber} rx={1} />
      </g>

      {/* L / R labels */}
      <text x={cx - r*0.70} y={cy - r*0.15} fontSize="9" fill={C.sub}
        fontFamily={MONO} textAnchor="middle">L</text>
      <text x={cx + r*0.70} y={cy - r*0.15} fontSize="9" fill={C.sub}
        fontFamily={MONO} textAnchor="middle">R</text>

      {/* Inclinometer tube */}
      <rect x={cx - 22} y={ballY - 6} width={44} height={12}
        rx={6} fill="none" stroke={C.sub} strokeWidth={1} opacity="0.5" />
      {/* Ball */}
      <circle cx={ballX} cy={ballY} r={5}
        fill={Math.abs(accel_y_g??0) > 0.1 ? C.amber : C.white}
        opacity="0.9" />

      {/* Digital rate readout */}
      <text x={cx} y={cy + r*0.55} textAnchor="middle" fontSize="9"
        fill={C.sub} fontFamily={MONO}>
        {(roll_rate_dps??0) > 0 ? '►' : (roll_rate_dps??0) < 0 ? '◄' : '●'}
        {' '}{Math.abs(roll_rate_dps??0).toFixed(0)}°/s
      </text>
    </GaugeShell>
  );
}

// ── 8. DIRECTIONAL INDICATOR (Heading) ───────────────────────────────────────
// Compass rose that rotates — heading_deg drives the card.
// Heading bug shown as amber triangle at the bug position.
// Simple fixed-card style: N points up when heading is North.
function DirectionalGauge({ heading_deg=0, hdg_ref_deg=null, hdg_ref_err_deg=null, size=110 }) {
  const r = size/2; const cx=r, cy=r;
  const hdg = heading_deg??0;

  // Cardinal points and marks rotate with the heading card
  // The card rotates so that the current heading is always at top
  const cardRot = -hdg;   // rotate card so hdg points to 12 o'clock

  // Heading bug position: bug is fixed at its absolute heading,
  // so on the rotating card it appears at (hdg_ref_deg - hdg) from top
  const bugOnCard = hdg_ref_deg !== null
    ? ((hdg_ref_deg - hdg + 360) % 360)  // 0=top, 90=right, etc.
    : null;
  // Convert to angle from top (clockwise)
  const bugCardDeg = bugOnCard !== null
    ? bugOnCard - 90  // SVG 0° is 3 o'clock; adjust so 0=12 o'clock
    : null;

  // Compute error colour
  const errAbs = hdg_ref_err_deg !== null ? Math.abs(hdg_ref_err_deg) : null;
  const bugColor = errAbs === null ? '#f0a500'
    : errAbs <= 5  ? '#2ea043'
    : errAbs <= 15 ? '#f0a500'
    : '#cf222e';

  // Cardinal labels at N/E/S/W positions on the card
  const cardinals = [
    { label:'N', deg: 0  },
    { label:'E', deg: 90 },
    { label:'S', deg: 180},
    { label:'W', deg: 270},
  ];

  const labelPos = (deg) => {
    const rad = (deg - 90) * Math.PI / 180;
    return { x: cx + (r-14) * Math.cos(rad), y: cy + (r-14) * Math.sin(rad) };
  };

  return (
    <GaugeShell size={size} label="Heading" sublabel="° mag" warn={false} crit={false}>
      {/* Rotating compass card */}
      <g transform={`rotate(${cardRot}, ${cx}, ${cy})`}>
        {/* Tick marks every 10° */}
        {Array.from({length:36}, (_,i) => {
          const ang = i * 10;
          const rad = (ang - 90) * Math.PI / 180;
          const inner = i % 3 === 0 ? r-12 : r-9;
          return (
            <line key={i}
              x1={cx + inner * Math.cos(rad)} y1={cy + inner * Math.sin(rad)}
              x2={cx + (r-5) * Math.cos(rad)} y2={cy + (r-5) * Math.sin(rad)}
              stroke={C.sub} strokeWidth={i % 3 === 0 ? 1.5 : 0.8} />
          );
        })}
        {/* Cardinal labels */}
        {cardinals.map(({label, deg}) => {
          const pos = labelPos(deg);
          return (
            <text key={label} x={pos.x} y={pos.y + 3.5}
              textAnchor="middle" fontSize={label==='N'?11:9}
              fill={label==='N' ? C.red : C.white}
              fontFamily={MONO} fontWeight="700">{label}</text>
          );
        })}

        {/* Heading bug — amber triangle on the card rim */}
        {bugCardDeg !== null && (
          <g transform={`rotate(${bugCardDeg}, ${cx}, ${cy})`}>
            <polygon
              points={`${cx},${cy-(r-4)} ${cx-5},${cy-(r-14)} ${cx+5},${cy-(r-14)}`}
              fill="#f0a500" stroke="#0d1117" strokeWidth={0.8} opacity={0.95} />
          </g>
        )}
      </g>

      {/* Fixed lubber line at top (12 o'clock = current heading) */}
      <polygon points={`${cx},${cy-r+2} ${cx-5},${cy-r+12} ${cx+5},${cy-r+12}`}
        fill={C.white} />

      {/* Digital heading readout */}
      <text x={cx} y={cy + r*0.38} textAnchor="middle" fontSize="12"
        fill={bugColor} fontFamily={MONO} fontWeight="700">
        {Math.round(hdg).toString().padStart(3,'0')}°
      </text>

      {/* Bug value and error if set */}
      {hdg_ref_deg !== null && (
        <text x={cx} y={cy + r*0.54} textAnchor="middle" fontSize="8"
          fill="#f0a500" fontFamily={MONO} opacity={0.85}>
          ▶{Math.round(hdg_ref_deg).toString().padStart(3,'0')}°
          {hdg_ref_err_deg !== null
            ? ` ${hdg_ref_err_deg > 0 ? '+' : ''}${hdg_ref_err_deg.toFixed(0)}°`
            : ''}
        </text>
      )}
    </GaugeShell>
  );
}
function StatusIndicator({ label, value, color, blink=false }) {
  return (
    <div style={{
      display:'flex', flexDirection:'column', alignItems:'center', gap:2,
      padding:'6px 10px', background:C.well, borderRadius:6,
      border:`1px solid ${color}40`,
      animation: blink ? 'blink 1s step-start infinite' : 'none',
    }}>
      <div style={{ fontFamily:MONO, fontSize:13, fontWeight:700, color, lineHeight:1 }}>
        {value}
      </div>
      <div style={{ fontFamily:SANS, fontSize:9, color:C.sub,
                    textTransform:'uppercase', letterSpacing:'0.07em' }}>
        {label}
      </div>
    </div>
  );
}

function StatusStrip({ t }) {
  const faults  = t.fault_count ?? 0;
  const state   = t.alss_phase ?? t.ssfd_state ?? t.lb_mode ?? '—';
  const armed   = t.armed === true || t.armed === 1;
  const disc    = t.discovery_running ?? false;
  const ceActive= t.ce_active ?? false;
  const ceLev   = t.ce_lev_risk ?? 0;
  const alss    = t.alss_phase && t.alss_phase !== '—' && t.alss_phase !== 'idle';

  // Power plant
  const P_kw   = Math.max(0, (t.motor_I_A??0) * (t.bus_voltage??22.2) / 1000);
  const volts  = t.bus_voltage ?? 0;
  const pwrWarn = P_kw > 3.0; const pwrCrit = P_kw > 4.2;
  const batWarn = volts < 21.5; const batCrit = volts < 20.0;

  // Reference bugs
  const hdgBugSet = t.hdg_ref_set ?? false;
  const hdgBugDeg = t.hdg_ref_deg ?? null;
  const hdgErrDeg = t.hdg_ref_err_deg ?? null;
  const altBugSet = t.alt_ref_set ?? false;
  const altBugFt  = t.alt_ref_ft ?? null;
  const altErrFt  = t.alt_ref_err_ft ?? null;

  const hdgErrColor = hdgErrDeg === null ? C.amber
    : Math.abs(hdgErrDeg) <= 5  ? C.green
    : Math.abs(hdgErrDeg) <= 15 ? C.amber : C.red;

  const altErrColor = altErrFt === null ? C.amber
    : Math.abs(altErrFt) <= 30  ? C.green
    : Math.abs(altErrFt) <= 100 ? C.amber : C.red;

  return (
    <div style={{ display:'flex', gap:6, flexWrap:'wrap', alignItems:'center' }}>
      {/* Flight status */}
      <StatusIndicator label="Armed" value={armed?'ARMED':'SAFE'}
        color={armed?C.amber:C.green} blink={armed} />
      <StatusIndicator label="Faults" value={faults === 0 ? 'OK' : faults}
        color={faults>0?C.red:C.green} blink={faults>0} />
      <StatusIndicator label="State" value={state}
        color={state==='—'?C.sub:C.amber} />
      <StatusIndicator label="Discovery" value={disc?'ON':'OFF'}
        color={disc?C.green:C.ghost} />

      {/* Power plant — moved from gauge row */}
      <StatusIndicator label="PWR"
        value={`${P_kw.toFixed(1)}kW`}
        color={pwrCrit?C.red:pwrWarn?C.amber:C.green}
        blink={pwrCrit} />
      <StatusIndicator label="BAT"
        value={`${volts.toFixed(1)}V`}
        color={batCrit?C.red:batWarn?C.amber:C.green}
        blink={batCrit} />

      {/* Reference bugs — shown only when set */}
      {hdgBugSet && hdgBugDeg !== null && (
        <StatusIndicator label="HDG▶"
          value={`${Math.round(hdgBugDeg).toString().padStart(3,'0')}°`
                 + (hdgErrDeg !== null
                    ? `  ${hdgErrDeg > 0 ? '+' : ''}${hdgErrDeg.toFixed(0)}°`
                    : '')}
          color={hdgErrColor} />
      )}
      {altBugSet && altBugFt !== null && (
        <StatusIndicator label="ALT▶"
          value={`${Math.round(altBugFt)}ft`
                 + (altErrFt !== null
                    ? `  ${altErrFt > 0 ? '+' : ''}${Math.round(altErrFt)}`
                    : '')}
          color={altErrColor} />
      )}

      {alss && <StatusIndicator label="ALSS" value={t.alss_segment??'running'}
        color={C.purple} />}
      {ceActive && <StatusIndicator label="Ceiling" value={`${(t.current_alt_m??0).toFixed(0)}m`}
        color={ceLev>0.5?C.red:C.amber} blink={ceLev>0.7} />}
      {ceActive && <StatusIndicator label="LEV Risk" value={`${(ceLev*100).toFixed(0)}%`}
        color={ceLev>0.5?C.red:ceLev>0.3?C.amber:C.green} />}
    </div>
  );
}

// ── Connection badge ──────────────────────────────────────────────────────────
function ConnBadge({ connected, armed }) {
  const col   = !connected ? C.red : armed ? C.amber : C.green;
  const label = !connected ? 'OFFLINE' : armed ? 'ARMED' : 'DISARMED';
  return (
    <div style={{ display:'inline-flex', alignItems:'center', gap:6,
                  padding:'4px 12px', borderRadius:20,
                  background:`${col}22`, border:`1px solid ${col}`,
                  fontFamily:MONO, fontSize:11, color:col, fontWeight:700,
                  letterSpacing:'0.1em' }}>
      <div style={{ width:7, height:7, borderRadius:'50%', background:col,
                    boxShadow: connected?`0 0 6px ${col}`:'none',
                    animation: (armed&&connected)?'blink 1s step-start infinite':'none' }} />
      {label}
    </div>
  );
}

// ── Chat message ──────────────────────────────────────────────────────────────
function Message({ msg }) {
  const isUser  = msg.role === 'user';
  const isAlert = msg.role === 'alert';
  const bg   = isUser ? C.well : isAlert ? `${C.red}15` : C.panel;
  const bdr  = isUser ? `${C.blue}40` : isAlert ? C.red : C.border;
  const icon = isUser ? 'YOU' : isAlert ? '!' : 'GS';
  const icol = isUser ? C.blue : isAlert ? C.red : C.amber;
  return (
    <div style={{ display:'flex', flexDirection:isUser?'row-reverse':'row',
                  gap:10, marginBottom:16, alignItems:'flex-start' }}>
      <div style={{ width:32, height:32, borderRadius:'50%', flexShrink:0,
                    background:C.well, border:`1px solid ${bdr}`,
                    display:'flex', alignItems:'center', justifyContent:'center',
                    fontFamily:MONO, fontSize:10, color:icol, fontWeight:700 }}>
        {icon}
      </div>
      <div style={{ maxWidth:'72%', padding:'10px 14px', borderRadius:10,
                    background:bg, border:`1px solid ${bdr}`,
                    fontFamily:SANS, fontSize:14, lineHeight:1.6, color:C.text,
                    whiteSpace:'pre-wrap' }}>
        {msg.content}
      </div>
    </div>
  );
}

// ── Raw data panel ────────────────────────────────────────────────────────────
function DataPanel({ log }) {
  const ref = useRef(null);
  useEffect(() => { ref.current?.scrollTo(0, ref.current.scrollHeight); }, [log]);
  const colMap = { tx:C.blue, rx:C.green, err:C.red, sys:C.sub };
  const pfxMap = { tx:'→ ', rx:'← ', err:'! ', sys:'  ' };
  return (
    <div ref={ref} style={{ height:'100%', overflowY:'auto', padding:10,
                             fontFamily:MONO, fontSize:10, lineHeight:1.7 }}>
      {log.map((e,i) => (
        <div key={i} style={{ color:colMap[e.t]||C.sub, wordBreak:'break-all' }}>
          {pfxMap[e.t]||'  '}{e.text}
        </div>
      ))}
    </div>
  );
}

// ── Main App ──────────────────────────────────────────────────────────────────
export default function App() {
  const { connected, telem, log, send } = useGS('ws://localhost:5758');
  const [messages, setMessages] = useState([{
    role: 'assistant',
    content: "Good morning. I'm your MCO-1D ground station assistant.\n\nThe six-pack above shows your flight instruments — airspeed, attitude, altitude, vertical speed, power, and battery. They update live.\n\nTell me what you need in plain English:\n\n\"Ready to fly\"\n\"Arm it and let's go\"\n\"It keeps drifting left\"\n\"Start the practice programme\"\n\"How is the learning going?\"\n\nI'll handle everything else.",
  }]);
  const [input, setInput]   = useState('');
  const [busy, setBusy]     = useState(false);
  const [showData, setShowData] = useState(false);
  const chatRef  = useRef(null);
  const telemRef = useRef(telem);
  telemRef.current = telem;

  useEffect(() => { chatRef.current?.scrollTo(0, chatRef.current.scrollHeight); }, [messages]);

  // Proactive alerts
  const lastAlert = useRef({});
  useEffect(() => {
    const now = Date.now();
    const add = (text) => setMessages(p => [...p, { role:'alert', content:text }]);
    if (telem.fault_count > 0 && now - (lastAlert.current.fault||0) > 30000) {
      add(`⚠ Fault detected (${telem.fault_count} fault${telem.fault_count>1?'s':''}). Checking now…`);
      lastAlert.current.fault = now;
      send('fault_summary');
    }
    if ((telem.bus_voltage??25) < 20.0 && (telem.bus_voltage??25) > 0
        && now - (lastAlert.current.batt||0) > 60000) {
      add(`⚠ Battery critical — ${(telem.bus_voltage).toFixed(1)}V. Land immediately.`);
      lastAlert.current.batt = now;
    } else if ((telem.bus_voltage??25) < 21.5 && (telem.bus_voltage??25) > 20.0
        && now - (lastAlert.current.battWarn||0) > 90000) {
      add(`⚠ Battery low — ${(telem.bus_voltage).toFixed(1)}V. Plan to land soon.`);
      lastAlert.current.battWarn = now;
    }
    if ((telem.ce_lev_risk??0) > 0.6 && now - (lastAlert.current.lev||0) > 20000) {
      add(`⚠ Wing stall risk at current altitude. The aircraft is descending.`);
      lastAlert.current.lev = now;
    }
  }, [telem.fault_count, telem.bus_voltage, telem.ce_lev_risk, send]);

  // Claude API agentic loop
  const handleSend = useCallback(async () => {
    if (!input.trim() || busy) return;
    const text = input.trim(); setInput(''); setBusy(true);
    const msgs = [...messages, { role:'user', content:text }];
    setMessages(msgs);

    const apiMsgs = msgs.map(m => ({
      role: m.role === 'assistant' || m.role === 'alert' ? 'assistant' : 'user',
      content: m.content,
    }));
    // Append telemetry context
    const tStr = JSON.stringify(telemRef.current, null, 0);
    apiMsgs[apiMsgs.length-1] = {
      ...apiMsgs[apiMsgs.length-1],
      content: apiMsgs[apiMsgs.length-1].content + `\n\n[Telemetry: ${tStr}]`,
    };

    const tools = [
      { name:'send_command', description:'Send a command to the aircraft.',
        input_schema:{ type:'object', properties:{ cmd:{ type:'string' }}, required:['cmd'] } },
      { name:'read_telemetry', description:'Return the latest aircraft telemetry.',
        input_schema:{ type:'object', properties:{} } },
    ];

    try {
      let curMsgs = apiMsgs;
      let finalText = '';
      for (let iter = 0; iter < 10; iter++) {
        const res = await fetch('/api/anthropic', {
          method:'POST', headers:{'Content-Type':'application/json'},
          body: JSON.stringify({ model:'claude-sonnet-4-6', max_tokens:1024,
                                  system:SYSTEM, messages:curMsgs, tools }),
        });
        const data = await res.json();
        if (data.stop_reason === 'end_turn') {
          finalText = (data.content||[]).filter(b=>b.type==='text').map(b=>b.text).join('\n').trim();
          break;
        }
        if (data.stop_reason === 'tool_use') {
          const results = [];
          for (const b of (data.content||[])) {
            if (b.type !== 'tool_use') continue;
            let result = '';
            if (b.name === 'send_command') { const ack = await send(b.input.cmd); result = ack?.msg || '(sent)'; }
            else if (b.name === 'read_telemetry') result = JSON.stringify(telemRef.current);
            results.push({ type:'tool_result', tool_use_id:b.id, content:result });
          }
          curMsgs = [...curMsgs,
            { role:'assistant', content: data.content },
            { role:'user',      content: results }];
          continue;
        }
        finalText = 'Something went wrong. Please try again.'; break;
      }
      if (finalText) setMessages(p => [...p, { role:'assistant', content:finalText }]);
    } catch (err) {
      setMessages(p => [...p, { role:'alert',
        content:`Connection error: ${err.message}. Check that the launcher is running.` }]);
    } finally { setBusy(false); }
  }, [input, messages, busy, send]);

  const onKey = (e) => { if (e.key==='Enter'&&!e.shiftKey) { e.preventDefault(); handleSend(); } };
  const armed = telem.armed===true || telem.armed===1;
  const GAUGE_SIZE = 110;

  return (
    <div style={{ display:'flex', flexDirection:'column', height:'100vh',
                  background:C.void, color:C.text, fontFamily:SANS, overflow:'hidden' }}>

      {/* ── Header ─────────────────────────────────────────────────────────── */}
      <div style={{ display:'flex', alignItems:'center', justifyContent:'space-between',
                    padding:'8px 16px', background:C.cockpit,
                    borderBottom:`1px solid ${C.border}`, flexShrink:0 }}>
        <div style={{ display:'flex', alignItems:'center', gap:12 }}>
          <span style={{ fontFamily:MONO, fontSize:13, fontWeight:700,
                         color:C.amber, letterSpacing:'0.12em' }}>MCO-1D</span>
          <span style={{ fontFamily:SANS, fontSize:11, color:C.sub }}>Ground Station</span>
        </div>
        <div style={{ display:'flex', gap:8, alignItems:'center' }}>
          <ConnBadge connected={connected} armed={armed} />
          <button onClick={() => setShowData(v=>!v)}
            style={{ padding:'3px 9px', borderRadius:5, cursor:'pointer',
                     background:showData?C.well:'transparent',
                     border:`1px solid ${C.border}`,
                     color:C.sub, fontSize:10, fontFamily:MONO }}>
            {showData ? 'HIDE DATA' : 'RAW DATA'}
          </button>
        </div>
      </div>

      {/* ── Six-pack instrument panel ───────────────────────────────────────── */}
      <div style={{ background:C.cockpit, borderBottom:`1px solid ${C.border}`,
                    flexShrink:0, padding:'10px 16px' }}>
        {/* Panel bezel — mimics the instrument panel surround */}
        <div style={{ background:'#0a0d0a', borderRadius:10, padding:'12px 16px',
                      border:`2px solid #1a2a1a`,
                      boxShadow:'inset 0 2px 8px rgba(0,0,0,0.6)' }}>
          {/* TOP ROW: Airspeed | Attitude | Altimeter */}
          <div style={{ display:'flex', gap:14, justifyContent:'center', marginBottom:14 }}>
            <AirspeedGauge V_ms={telem.V_ms} size={GAUGE_SIZE} />
            <AttitudeGauge roll_deg={telem.roll_deg_imu} pitch_deg={telem.pitch_deg_imu} size={GAUGE_SIZE} />
            <AltimeterGauge h_agl_m={telem.h_agl_m}
                            alt_ref_m={telem.alt_ref_set ? telem.alt_ref_m : null}
                            size={GAUGE_SIZE} />
          </div>
          {/* BOTTOM ROW: Turn Coordinator | Directional Indicator | VSI */}
          <div style={{ display:'flex', gap:14, justifyContent:'center' }}>
            <TurnCoordGauge
              roll_rate_dps={telem.roll_rate_dps}
              accel_y_g={telem.accel_y_g}
              size={GAUGE_SIZE} />
            <DirectionalGauge
              heading_deg={telem.heading_deg}
              hdg_ref_deg={telem.hdg_ref_set ? telem.hdg_ref_deg : null}
              hdg_ref_err_deg={telem.hdg_ref_err_deg}
              size={GAUGE_SIZE} />
            <VSIGauge
              sink_rate_ms={telem.sink_rate_ms??telem.vel_down_ms}
              vsi_ref_fpm={telem.vsi_ref_fpm??0}
              size={GAUGE_SIZE} />
          </div>
        </div>
        {/* Status strip below the six-pack */}
        <div style={{ marginTop:8 }}>
          <StatusStrip t={telem} />
        </div>
      </div>

      {/* ── Main content area ────────────────────────────────────────────────── */}
      <div style={{ display:'flex', flex:1, overflow:'hidden' }}>

        {/* Chat */}
        <div style={{ flex:1, display:'flex', flexDirection:'column', minWidth:0,
                      borderRight: showData?`1px solid ${C.border}`:'none' }}>
          <div ref={chatRef} style={{ flex:1, overflowY:'auto', padding:'16px 20px' }}>
            {messages.map((m,i) => <Message key={i} msg={m} />)}
            {busy && (
              <div style={{ display:'flex', gap:10, marginBottom:16, alignItems:'flex-start' }}>
                <div style={{ width:32, height:32, borderRadius:'50%',
                              background:C.well, border:`1px solid ${C.border}`,
                              display:'flex', alignItems:'center', justifyContent:'center',
                              fontFamily:MONO, fontSize:10, color:C.amber }}>GS</div>
                <div style={{ padding:'10px 14px', borderRadius:10,
                              background:C.panel, border:`1px solid ${C.border}`,
                              display:'flex', gap:5, alignItems:'center' }}>
                  {[0,1,2].map(d => (
                    <div key={d} style={{ width:7, height:7, borderRadius:'50%', background:C.amber,
                                         animation:'pulse 1.2s ease-in-out infinite',
                                         animationDelay:`${d*0.2}s` }} />
                  ))}
                </div>
              </div>
            )}
          </div>
          {/* Input */}
          <div style={{ padding:'10px 14px', borderTop:`1px solid ${C.border}`,
                        background:C.cockpit, flexShrink:0 }}>
            <div style={{ display:'flex', gap:8 }}>
              <textarea value={input} onChange={e=>setInput(e.target.value)} onKeyDown={onKey}
                placeholder={connected ? "Tell me what you need…" : "Not connected — start launcher first"}
                disabled={busy||!connected} rows={1}
                style={{ flex:1, padding:'9px 12px', borderRadius:7,
                         background:C.well, border:`1px solid ${C.border}`,
                         color:C.text, fontFamily:SANS, fontSize:14,
                         resize:'none', outline:'none', lineHeight:1.5 }} />
              <button onClick={handleSend} disabled={busy||!connected||!input.trim()}
                style={{ padding:'9px 18px', borderRadius:7, flexShrink:0,
                         background:(!busy&&input.trim()&&connected)?C.amber:C.well,
                         border:'none',
                         color:(!busy&&input.trim()&&connected)?C.void:C.ghost,
                         fontFamily:SANS, fontSize:13, fontWeight:600, cursor:'pointer',
                         transition:'background 0.15s' }}>
                Send
              </button>
            </div>
            <div style={{ marginTop:4, fontSize:10, color:C.ghost, fontFamily:SANS }}>
              Enter to send · Shift+Enter for new line
            </div>
          </div>
        </div>

        {/* Raw data panel */}
        {showData && (
          <div style={{ width:320, background:C.cockpit, flexShrink:0,
                        display:'flex', flexDirection:'column' }}>
            <div style={{ padding:'6px 10px', borderBottom:`1px solid ${C.border}`,
                          fontFamily:MONO, fontSize:9, color:C.sub,
                          textTransform:'uppercase', letterSpacing:'0.08em' }}>
              Aircraft data stream
            </div>
            <DataPanel log={log} />
          </div>
        )}
      </div>

      <style>{`
        @keyframes pulse { 0%,100%{opacity:0.3;transform:scale(0.8)} 50%{opacity:1;transform:scale(1)} }
        @keyframes blink  { 0%,100%{opacity:1} 50%{opacity:0.3} }
        * { box-sizing:border-box; }
        textarea:focus { border-color:#f0a500 !important; }
        ::-webkit-scrollbar { width:4px; }
        ::-webkit-scrollbar-track { background:transparent; }
        ::-webkit-scrollbar-thumb { background:#253040; border-radius:2px; }
      `}</style>
    </div>
  );
}

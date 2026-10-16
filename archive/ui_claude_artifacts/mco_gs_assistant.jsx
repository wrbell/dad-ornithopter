import { useState, useRef, useEffect } from "react";

// ── MCO-1D full command catalogue ─────────────────────────────────────────────
const COMMAND_CATALOGUE = `
PRE-FLIGHT:
arm: Arm aircraft — enables ODrives, resets per-flight learning state
disarm: Disarm — safe state, all wings idle
persist_load_all: Restore all learning from previous flights (runs automatically at startup)
persist_status: Show what learning files are saved on disk
soft_stop_status: Check rope protection system state per wing
cg_status: Show centre-of-gravity estimate and stability margin
failure_briefing: Pre-flight failure mode summary — read before TC-04
fault_summary: Complete fault and degradation state
attitude_hold_status: Show attitude hold (electronic dihedral) gains and engage state
hover_mode_status: Show hover mode thresholds and calibration values
advanced_modes_status: Show position hold, turn coordination, crew hover, airspeed hold
inbox_check: Scan USB stick and inbox folder for pending hypotheses or knowledge files

HOVER (V < 3 m/s):
crew_hover_enable: Velocity-referenced hover — stick commands speed not bank angle (default ON in hover)
crew_hover_disable: Attitude-reference hover — stick commands bank angle (classic feel)
position_hold_engage: Capture current GPS position and hold it — aircraft freezes in place
position_hold_release: Release position hold — return to attitude control
position_hold_enable: Enable GPS position hold system
position_hold_disable: Disable GPS position hold
cg_calibrate_now: Trigger CG calibration pass manually
hover_mode_calibrate: Update hover frequency from TC-02 measured result
gnd_clear_status: Ground clearance controller — wing amplitude vs AGL height
gnd_state_status: Ground state and safe operating boundaries

CRUISE / FIXED-WING (V > 5 m/s):
airspeed_hold_enable: Hold target airspeed via frequency control (engages after stable lever 1.5s)
airspeed_hold_disable: Disable airspeed hold
turn_coord_enable: Enable automatic turn coordination (yaw bias in banked turns)
turn_coord_disable: Disable turn coordination — manual pedal only
corridor_guard_enable: Enable conversion corridor guard — protects morphing transitions
corridor_guard_disable: Disable corridor guard (operator accepts envelope responsibility)
attitude_hold_enable: Enable electronic dihedral — hands-off roll and pitch stability
attitude_hold_disable: Disable attitude hold — requires constant pilot input
attitude_hold_tune: Tune attitude hold gain in flight: attitude_hold_tune kp_outer 0.08
strouhal_status: Show Strouhal number vs optimal for current airspeed (target 0.25–0.35)

MORPHING (wing configurations):
h33_engage: Variable wing area — stop a pair: h33_engage pair=W4 state=B
h33_disengage: Return to normal 4-pair flapping configuration
h33_status: H-33 variable wing area status and active configuration
h34_engage: Dynamic sweep kinematics variant: h34_engage variant=V2
h34_disengage: Return from dynamic sweep variant
h34_status: H-34 dynamic sweep status
h35_engage: Thrust/lift separation — stop some pairs for high-speed efficiency
h35_disengage: Return to all-flapping configuration
h35_status: H-35 thrust/lift status and stopped pair configuration
h35_lift_report: Lift/thrust breakdown for current H-35 configuration
h36_engage: Minimum-power loiter/ferry configuration
h36_disengage: Return from loiter/ferry
h36_status: H-36 loiter/ferry status and endurance estimate
h36_loiter_report: Loiter endurance prediction and power breakdown
h36_ferry_report: Ferry configuration efficiency report
morphing_speed_report: Speed leaderboard — which configuration is fastest at each speed
h33_h34_h35_disengage_all: Emergency return to baseline — all morphing off immediately

LEARNING AND DISCOVERY:
discovery_enable: Start unconstrained discovery mode (enable above 15m AGL)
discovery_disable: Stop discovery — return to normal optimizer
discovery_status: Show discovery state, best configuration found, queue length
seed_hypotheses: Pre-load the 17 aerodynamic research hypotheses into discovery queue
hypothesis_results: Show which of the 17 research hypotheses were promoted
hypothesis_status: Show hypothesis queue, how many evaluated, how many promoted
hypothesis_strouhal_targets: Show Strouhal-optimal frequency targets for current airspeed
mechanism_weight_report: Show which roll/pitch/yaw mechanisms are ranked best
mechanism_weight_reset: Reset mechanism weights to equal (after major rebuild)
save_mechanism_weights: Save learned weights to disk now
learning_history: Cross-flight improvement trend — what has gotten better over time
persist_checkpoint: Save all learning state mid-flight without shutting down
freeze_optimizers: Freeze all learning loops (for baseline comparison testing)
restore_optimizers: Unfreeze all learning loops
l5_status: Loop 5 yaw modality optimizer — dart/crab/split-stroke blend by condition
l9_status: Meta-optimizer — which constants are being autonomously tuned
exploration_status: Full exploration system status (all loops)
auto_disc_enable: Enable continuous autonomous discovery mode
auto_disc_disable: Disable continuous autonomous discovery
lever_learning_status: What the long-term lever learning has found
polar_survey_start: Start autonomous in-flight polar survey
persist_save_all: Save all learned state to disk now
persist_load_all: Reload all learning from disk

SAR MISSION OPERATIONS:
crew_hover_enable: Velocity-referenced hover for SAR approach (default ON)
position_hold_engage: Freeze aircraft position — crew operates below
position_hold_release: Release position hold when repositioning
advanced_modes_status: Status of all 5 advanced modes including SAR modes
dart_status: Dart manoeuvre control status
dart_reset: Reset dart state after aborted manoeuvre
dart_set_mode_weights: Override dart mode blend: dart_set_mode_weights 0.5 0.3 0.2
h36_loiter_report: Endurance estimate for current loiter configuration

SAFETY AND FAULTS:
fault_status: Complete fault and degradation state
fault_summary: Concise fault summary for pilot
fault_wing_restore: Mark a wing healthy after physical repair
controlled_landing_status: land_now mode status and descent rate settings
failure_briefing: Full failure mode briefing
query_failure: Look up response for specific wing failure combination
emergency_stop: Immediate motor stop
stall_detector_status: Per-pair stall likelihood — warn operator if >0.45
rope_health_status: Rope fatigue state for all 8 drives
violent_manoeuvre_status: Violent manoeuvre authority system state
prog_sim_request: Request progressive failure simulation
prog_sim_approve: Approve progressive failure simulation
wing_sim_request: Request wing failure simulation

FLEET KNOWLEDGE SHARING:
knowledge_export_now: Export everything learned to outbox (full + fleet-portable files)
knowledge_import: Import from another drone: knowledge_import /path/to/file.json
merge_fleet_knowledge: Merge files from N drones: merge_fleet_knowledge f1.json f2.json
fleet_status: Show outbox contents and this drone's learning summary
gs_inject_hypothesis: Inject your idea live: gs_inject_hypothesis name="my idea" cascade_phase_deg=37
gs_inject_parameter: Change a parameter live: gs_inject_parameter DISCOVERY_DWELL_S 30
gs_inject_note: Record an observation: gs_inject_note "noticed left drift in hover"
inbox_check: Scan USB and inbox for pending files
user_hypothesis_status: Show injected user hypotheses and their evaluation status

CERTIFICATION:
cert_status: Overall certification readiness summary
cert_fmea: Run FMEA generator (13 failure modes, 0 catastrophic)
cert_fha: Generate Functional Hazard Analysis (ARP4754A format)
cert_do178c: Generate DO-178C software traceability matrix
cert_hq_analyse: Run HQ analyser on latest flight log (ADS-33 criteria)
cert_flutter_report: Flutter boundary log from TC-03
cert_loads_report: Structural loads peak g per flight
cert_rope_report: Rope life tracker using Miner Rule

TEST CARDS:
tc01_start / tc01_status / tc01_confirm / tc01_abort: TC-01 bench test
tc02_start / tc02_status / tc02_continue / tc02_abort: TC-02 captive hover
tc03_start / tc03_status / tc03_continue / tc03_abort: TC-03 truck forward flight
tc04_start / tc04_status / tc04_continue / tc04_abort: TC-04 first free flight
tc_results: Export all test card results

DIAGNOSTICS:
soft_stop_status: Rope protection per-wing state
pitot_status: Pitot airspeed sensor state
motor_health: ODrive motor health per wing
motor_peaks: Peak current per motor this flight
servo_health_report: Servo tracking error history
stroke_position_status: Actual vs commanded stroke position per pair
lift_balance_report: Port vs starboard lift balance
trim_balance_report: Trim balance and asymmetry
aero_cal_status: Aerodynamic calibration state (CL_MAX, stall speed, glide ratio)
margin_status: Flight margin state (stall, structural, envelope)
l4_status: Loop 4 optimizer status
knowledge_status: Knowledge state summary — what has been learned
`.trim();

// ── System prompt for the AI ───────────────────────────────────────────────────
const SYSTEM_PROMPT = `You are the operator assistant for the MCO-1D — an eight-wing metachronal cascade flapping-wing ornithopter SAR (Search and Rescue) drone. You help operators of all technical levels find and use the right ground station commands.

The MCO-1D has 217 ground station commands. Here is the complete catalogue:

${COMMAND_CATALOGUE}

YOUR JOB:
- Understand what the operator wants in any phrasing, no matter how informal or vague
- Identify the 1-4 most relevant commands for their situation
- Explain in plain English what each command does and when to use it
- Give the exact command string they should type
- If a command takes parameters, show an example
- If multiple steps are needed (e.g. arm then enable discovery), list them in order
- Never assume the operator knows the code — explain everything in plain English
- Be concise: 3-5 sentences per response is usually enough
- If you're not sure what they want, ask one clarifying question

CONTEXT ABOUT THE AIRCRAFT:
- Hover mode: V < 3 m/s, crew hover (velocity-referenced stick) is ON by default
- Fixed-wing mode: V > 5 m/s, standard controls
- Blend zone: 3-5 m/s, both modes partially active
- Discovery mode: aircraft experiments with configurations to improve itself — enable above 15m AGL
- Dart manoeuvre: simultaneous lateral joystick + same-side rudder — intensity scales with input speed/amplitude
- Fleet sharing: knowledge_export_now + knowledge_import moves learning between drones
- All learning persists across flights automatically

RESPONSE FORMAT:
Always end your response with a "COMMANDS" section listing the exact command strings to type, one per line, like this:
COMMANDS:
arm
discovery_enable

This makes it easy for the interface to highlight the commands.`;

// ── Suggested starter questions ───────────────────────────────────────────────
const STARTERS = [
  "How do I get it ready to fly?",
  "How do I make it hover in one spot?",
  "The aircraft has drifted — how do I lock it in place?",
  "How do I start the learning and discovery system?",
  "We finished a flight — how do I save what it learned?",
  "I want to share learning with another drone",
  "Something feels wrong — how do I check for faults?",
  "How do I inject my own idea for the aircraft to try?",
  "I want it to fly as efficiently as possible",
  "How do I prepare before the very first free flight?",
];

// ── Parse commands from AI response ──────────────────────────────────────────
function parseCommands(text) {
  const match = text.match(/COMMANDS:\s*([\s\S]*?)(?:\n\n|$)/);
  if (!match) return [];
  return match[1]
    .split('\n')
    .map(l => l.trim())
    .filter(l => l && !l.startsWith('#') && !l.startsWith('//'));
}

function stripCommands(text) {
  return text.replace(/COMMANDS:\s*([\s\S]*?)(?:\n\n|$)/, '').trim();
}

// ── Colours ───────────────────────────────────────────────────────────────────
const C = {
  bg:       '#0d1117',
  panel:    '#161b22',
  border:   '#30363d',
  accent:   '#1f6feb',
  accentLt: '#388bfd',
  green:    '#3fb950',
  orange:   '#d29922',
  red:      '#f85149',
  text:     '#e6edf3',
  dim:      '#8b949e',
  cmd:      '#79c0ff',
  cmdBg:    '#0d2137',
};

export default function MCOAssistant() {
  const [messages, setMessages]   = useState([]);
  const [input, setInput]         = useState('');
  const [loading, setLoading]     = useState(false);
  const [copied, setCopied]       = useState(null);
  const [phase, setPhase]         = useState('pre-flight');
  const endRef                    = useRef(null);
  const inputRef                  = useRef(null);

  useEffect(() => {
    endRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, loading]);

  async function sendMessage(text) {
    if (!text.trim() || loading) return;
    const userMsg = { role: 'user', content: text.trim() };
    const newMessages = [...messages, userMsg];
    setMessages(newMessages);
    setInput('');
    setLoading(true);

    try {
      const response = await fetch('https://api.anthropic.com/v1/messages', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          model: 'claude-sonnet-4-6',
          max_tokens: 1000,
          system: SYSTEM_PROMPT,
          messages: newMessages,
        }),
      });
      const data = await response.json();
      const reply = data.content?.[0]?.text || 'No response received.';
      setMessages(prev => [...prev, { role: 'assistant', content: reply }]);
    } catch (err) {
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: `Error: ${err.message}. Check your connection to the ground station laptop.`,
      }]);
    }
    setLoading(false);
    setTimeout(() => inputRef.current?.focus(), 100);
  }

  function copyCmd(cmd) {
    navigator.clipboard.writeText(cmd).then(() => {
      setCopied(cmd);
      setTimeout(() => setCopied(null), 1800);
    });
  }

  function clearChat() {
    setMessages([]);
    inputRef.current?.focus();
  }

  const phases = ['pre-flight', 'hover', 'cruise', 'sar', 'discovery'];

  return (
    <div style={{
      minHeight: '100vh',
      background: C.bg,
      color: C.text,
      fontFamily: "'Inter', system-ui, sans-serif",
      display: 'flex',
      flexDirection: 'column',
    }}>

      {/* ── Header ── */}
      <div style={{
        background: C.panel,
        borderBottom: `1px solid ${C.border}`,
        padding: '12px 20px',
        display: 'flex',
        alignItems: 'center',
        gap: 14,
      }}>
        {/* Ornithopter icon */}
        <svg width="32" height="32" viewBox="0 0 32 32" fill="none">
          <circle cx="16" cy="16" r="3" fill={C.accentLt}/>
          {/* Wings */}
          {[[-1,-1],[-1,1],[1,-1],[1,1]].map(([sx,sy], i) => (
            <ellipse key={i}
              cx={16 + sx*8} cy={16 + sy*5}
              rx="7" ry="3"
              transform={`rotate(${sx*sy*20} ${16+sx*8} ${16+sy*5})`}
              fill={C.accent} opacity="0.7"/>
          ))}
        </svg>

        <div>
          <div style={{ fontWeight: 700, fontSize: 15, letterSpacing: '-0.3px' }}>
            MCO-1D Ground Station Assistant
          </div>
          <div style={{ color: C.dim, fontSize: 11, marginTop: 1 }}>
            217 commands · Ask anything in plain English
          </div>
        </div>

        <div style={{ marginLeft: 'auto', display: 'flex', gap: 6, alignItems: 'center' }}>
          {/* Phase selector */}
          <div style={{ fontSize: 11, color: C.dim, marginRight: 4 }}>Phase:</div>
          {phases.map(p => (
            <button key={p} onClick={() => setPhase(p)} style={{
              padding: '3px 9px',
              borderRadius: 12,
              border: `1px solid ${p === phase ? C.accent : C.border}`,
              background: p === phase ? C.accent + '22' : 'transparent',
              color: p === phase ? C.accentLt : C.dim,
              fontSize: 11,
              cursor: 'pointer',
              textTransform: 'capitalize',
            }}>{p}</button>
          ))}

          <button onClick={clearChat} style={{
            marginLeft: 8,
            padding: '4px 12px',
            borderRadius: 6,
            border: `1px solid ${C.border}`,
            background: 'transparent',
            color: C.dim,
            fontSize: 12,
            cursor: 'pointer',
          }}>Clear</button>
        </div>
      </div>

      {/* ── Chat area ── */}
      <div style={{
        flex: 1,
        overflowY: 'auto',
        padding: '20px',
        display: 'flex',
        flexDirection: 'column',
        gap: 16,
        maxWidth: 860,
        width: '100%',
        margin: '0 auto',
        boxSizing: 'border-box',
      }}>

        {/* Welcome / starters */}
        {messages.length === 0 && (
          <div>
            <div style={{
              textAlign: 'center',
              padding: '40px 20px 24px',
              color: C.dim,
            }}>
              <div style={{ fontSize: 22, fontWeight: 700, color: C.text, marginBottom: 8 }}>
                What do you need to do?
              </div>
              <div style={{ fontSize: 13 }}>
                Ask in plain English — no command knowledge required.
              </div>
            </div>

            <div style={{
              display: 'grid',
              gridTemplateColumns: 'repeat(auto-fill, minmax(250px, 1fr))',
              gap: 8,
            }}>
              {STARTERS.map((s, i) => (
                <button key={i} onClick={() => sendMessage(s)} style={{
                  textAlign: 'left',
                  padding: '10px 14px',
                  background: C.panel,
                  border: `1px solid ${C.border}`,
                  borderRadius: 8,
                  color: C.text,
                  fontSize: 13,
                  cursor: 'pointer',
                  lineHeight: 1.4,
                  transition: 'border-color 0.15s',
                }}
                  onMouseEnter={e => e.currentTarget.style.borderColor = C.accent}
                  onMouseLeave={e => e.currentTarget.style.borderColor = C.border}
                >{s}</button>
              ))}
            </div>
          </div>
        )}

        {/* Messages */}
        {messages.map((msg, i) => {
          const isUser = msg.role === 'user';
          const cmds   = isUser ? [] : parseCommands(msg.content);
          const body   = isUser ? msg.content : stripCommands(msg.content);

          return (
            <div key={i} style={{
              display: 'flex',
              flexDirection: 'column',
              alignItems: isUser ? 'flex-end' : 'flex-start',
              gap: 8,
            }}>
              {/* Bubble */}
              <div style={{
                maxWidth: '80%',
                padding: '10px 14px',
                borderRadius: isUser ? '16px 16px 4px 16px' : '16px 16px 16px 4px',
                background: isUser ? C.accent : C.panel,
                border: isUser ? 'none' : `1px solid ${C.border}`,
                fontSize: 14,
                lineHeight: 1.6,
                whiteSpace: 'pre-wrap',
                wordBreak: 'break-word',
              }}>
                {body}
              </div>

              {/* Command chips */}
              {cmds.length > 0 && (
                <div style={{
                  display: 'flex',
                  flexWrap: 'wrap',
                  gap: 6,
                  maxWidth: '80%',
                }}>
                  <div style={{
                    width: '100%',
                    fontSize: 11,
                    color: C.dim,
                    marginBottom: 2,
                    paddingLeft: 2,
                  }}>
                    Click to copy — then paste into the ground station terminal:
                  </div>
                  {cmds.map((cmd, ci) => (
                    <button key={ci} onClick={() => copyCmd(cmd)} title="Click to copy"
                      style={{
                        padding: '5px 12px',
                        background: copied === cmd ? C.green + '22' : C.cmdBg,
                        border: `1px solid ${copied === cmd ? C.green : C.accent + '55'}`,
                        borderRadius: 6,
                        color: copied === cmd ? C.green : C.cmd,
                        fontFamily: "'JetBrains Mono', 'Fira Code', monospace",
                        fontSize: 12,
                        cursor: 'pointer',
                        display: 'flex',
                        alignItems: 'center',
                        gap: 6,
                        transition: 'all 0.15s',
                      }}>
                      {copied === cmd ? '✓ copied' : cmd}
                      {copied !== cmd && (
                        <svg width="10" height="10" viewBox="0 0 24 24" fill="none"
                          stroke="currentColor" strokeWidth="2">
                          <rect x="9" y="9" width="13" height="13" rx="2"/>
                          <path d="M5 15H4a2 2 0 01-2-2V4a2 2 0 012-2h9a2 2 0 012 2v1"/>
                        </svg>
                      )}
                    </button>
                  ))}
                </div>
              )}
            </div>
          );
        })}

        {/* Typing indicator */}
        {loading && (
          <div style={{
            display: 'flex',
            alignItems: 'center',
            gap: 6,
            padding: '10px 14px',
            background: C.panel,
            border: `1px solid ${C.border}`,
            borderRadius: '16px 16px 16px 4px',
            width: 'fit-content',
          }}>
            {[0,1,2].map(d => (
              <div key={d} style={{
                width: 7, height: 7,
                borderRadius: '50%',
                background: C.dim,
                animation: `pulse 1.2s ease-in-out ${d * 0.2}s infinite`,
              }}/>
            ))}
          </div>
        )}

        <div ref={endRef}/>
      </div>

      {/* ── Input bar ── */}
      <div style={{
        background: C.panel,
        borderTop: `1px solid ${C.border}`,
        padding: '14px 20px',
      }}>
        <div style={{
          maxWidth: 860,
          margin: '0 auto',
          display: 'flex',
          gap: 10,
        }}>
          <input
            ref={inputRef}
            value={input}
            onChange={e => setInput(e.target.value)}
            onKeyDown={e => {
              if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                sendMessage(input);
              }
            }}
            placeholder="Ask anything — e.g. 'how do I make it stop moving?' or 'what do I do before the first flight?'"
            style={{
              flex: 1,
              padding: '10px 14px',
              background: C.bg,
              border: `1px solid ${C.border}`,
              borderRadius: 8,
              color: C.text,
              fontSize: 14,
              outline: 'none',
              transition: 'border-color 0.15s',
            }}
            onFocus={e => e.target.style.borderColor = C.accent}
            onBlur={e => e.target.style.borderColor = C.border}
          />
          <button
            onClick={() => sendMessage(input)}
            disabled={!input.trim() || loading}
            style={{
              padding: '10px 20px',
              background: input.trim() && !loading ? C.accent : C.border,
              border: 'none',
              borderRadius: 8,
              color: C.text,
              fontSize: 14,
              fontWeight: 600,
              cursor: input.trim() && !loading ? 'pointer' : 'default',
              transition: 'background 0.15s',
              whiteSpace: 'nowrap',
            }}>
            {loading ? 'Thinking…' : 'Ask →'}
          </button>
        </div>

        <div style={{
          maxWidth: 860,
          margin: '8px auto 0',
          fontSize: 11,
          color: C.dim,
          display: 'flex',
          gap: 16,
        }}>
          <span>Current phase: <span style={{ color: C.accentLt }}>{phase}</span></span>
          <span>217 commands available</span>
          <span>Commands appear as clickable chips — click to copy</span>
          <span>Press Enter to send</span>
        </div>
      </div>

      <style>{`
        @keyframes pulse {
          0%, 80%, 100% { opacity: 0.3; transform: scale(0.8); }
          40% { opacity: 1; transform: scale(1); }
        }
        * { box-sizing: border-box; }
        ::-webkit-scrollbar { width: 6px; }
        ::-webkit-scrollbar-track { background: transparent; }
        ::-webkit-scrollbar-thumb { background: #30363d; border-radius: 3px; }
        input::placeholder { color: #484f58; }
        button:focus-visible { outline: 2px solid #388bfd; outline-offset: 2px; }
      `}</style>
    </div>
  );
}

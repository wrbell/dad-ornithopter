#!/usr/bin/env python3
"""
create_spring_assembly_viewer.py
Creates an interactive 3D WebGL (Three.js) viewer for the V2 shoulder joint assembly
with the resonant torsion spring installed, complete with cutaway, exploded view,
and real-time spring deflection animation.
"""

import os
import base64

OUT_HTML = "cad/drawings/v2_spring_assembly_3d_viewer.html"

# Load Palm V2 STL if available
stl_path = "cad/manufactured/palm_v2.stl"
b64_drum = ""
if os.path.exists(stl_path):
    with open(stl_path, "rb") as f:
        b64_drum = base64.b64encode(f.read()).decode("ascii")

html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>MCO-1D V2 Shoulder Joint - Resonant Torsion Spring 3D Interactive Assembly</title>
  <style>
    body {{
      margin: 0;
      padding: 0;
      overflow: hidden;
      background-color: #0b0f19;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      color: #c9d1d9;
    }}
    #canvas-container {{
      width: 100vw;
      height: 100vh;
      display: block;
    }}
    #ui-overlay {{
      position: absolute;
      top: 16px;
      left: 16px;
      background: rgba(17, 24, 39, 0.92);
      border: 1px solid #374151;
      border-radius: 8px;
      padding: 16px 20px;
      max-width: 400px;
      backdrop-filter: blur(10px);
      box-shadow: 0 10px 30px rgba(0,0,0,0.6);
    }}
    h1 {{
      font-size: 16px;
      margin: 0 0 6px 0;
      color: #38bdf8;
      display: flex;
      align-items: center;
      gap: 8px;
    }}
    .badge {{
      font-size: 11px;
      background: #0284c7;
      color: #ffffff;
      padding: 2px 6px;
      border-radius: 4px;
      font-weight: 600;
    }}
    p {{
      font-size: 12px;
      line-height: 1.45;
      margin: 4px 0 12px 0;
      color: #9ca3af;
    }}
    .spec-table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 11px;
      margin-bottom: 12px;
    }}
    .spec-table td {{
      padding: 3px 0;
      border-bottom: 1px solid #1f2937;
    }}
    .spec-table td:last-child {{
      text-align: right;
      font-weight: 600;
      color: #f3f4f6;
      font-family: monospace;
    }}
    .controls {{
      display: flex;
      flex-direction: column;
      gap: 8px;
      border-top: 1px solid #374151;
      padding-top: 10px;
    }}
    .control-row {{
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 11.5px;
    }}
    .btn-group {{
      display: flex;
      gap: 6px;
    }}
    button {{
      background: #1f2937;
      border: 1px solid #374151;
      color: #c9d1d9;
      padding: 4px 8px;
      border-radius: 4px;
      font-size: 11px;
      cursor: pointer;
      transition: all 0.2s ease;
    }}
    button:hover {{
      background: #374151;
      color: #ffffff;
    }}
    button.active {{
      background: #0284c7;
      border-color: #38bdf8;
      color: #ffffff;
    }}
    #legend {{
      position: absolute;
      bottom: 20px;
      right: 20px;
      background: rgba(17, 24, 39, 0.88);
      border: 1px solid #374151;
      border-radius: 8px;
      padding: 12px 16px;
      font-size: 11px;
      line-height: 1.8;
    }}
    .dot {{
      display: inline-block;
      width: 10px;
      height: 10px;
      border-radius: 50%;
      margin-right: 6px;
      vertical-align: middle;
    }}
  </style>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
</head>
<body>
  <div id="canvas-container"></div>
  
  <div id="ui-overlay">
    <h1>V2 Shoulder Joint <span class="badge">Torsion Spring</span></h1>
    <p>Resonant mechanical energy storage integrated into the Palm V2 capstan pivot axis.</p>
    
    <table class="spec-table">
      <tr><td>Target Resonance (f₀)</td><td>1.95 Hz</td></tr>
      <tr><td>Wing Inertia (I_wing)</td><td>0.022 kg·m²</td></tr>
      <tr><td>Torsional Stiffness (K)</td><td>3.30 N·m/rad (0.058 N·m/deg)</td></tr>
      <tr><td>Spring Wire Diameter (d)</td><td>&Oslash; 3.5 mm (ASTM A228)</td></tr>
      <tr><td>Mean Coil Diameter (D_m)</td><td>&Oslash; 30.0 mm</td></tr>
      <tr><td>Active Coils (N_a)</td><td>4.9 coils (Solid H ≈ 17.2 mm)</td></tr>
      <tr><td>Flapping Stroke Amplitude</td><td>±45° (90° total)</td></tr>
      <tr><td>Motor Current Reduction</td><td>38.5 A peak → 15.0 A peak (-61%)</td></tr>
    </table>

    <div class="controls">
      <div class="control-row">
        <span>View Mode:</span>
        <div class="btn-group">
          <button id="btnAssembled" class="active">Assembled</button>
          <button id="btnCutaway">Cutaway</button>
          <button id="btnExploded">Exploded</button>
        </div>
      </div>
      <div class="control-row">
        <span>Motion:</span>
        <div class="btn-group">
          <button id="btnFlap" class="active">Flapping (1.95 Hz)</button>
          <button id="btnRotate">Orbit</button>
        </div>
      </div>
      <div class="control-row">
        <span>Reset Camera:</span>
        <button id="btnReset">Reset View</button>
      </div>
    </div>
  </div>

  <div id="legend">
    <div><span class="dot" style="background:#f59e0b"></span>Resonant Torsion Spring (&Oslash;3.5mm)</div>
    <div><span class="dot" style="background:#0284c7"></span>Big Drum / Flap Spool (7:1 Capstan)</div>
    <div><span class="dot" style="background:#38bdf8"></span>Dual 6708-2RS Thin Bearings</div>
    <div><span class="dot" style="background:#64748b"></span>Base Pedestal & Top Tie Bridge</div>
    <div><span class="dot" style="background:#10b981"></span>GoolRC 5010 Motor & Small Drum</div>
    <div><span class="dot" style="background:#a855f7"></span>Dyneema Drive Cable Loop</div>
  </div>

  <script>
    const container = document.getElementById('canvas-container');
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0f19);

    const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(110, -110, 85);
    camera.up.set(0, 0, 1);

    const renderer = new THREE.WebGLRenderer({{ antialias: true }});
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);

    const controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;

    // Lights
    scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const d1 = new THREE.DirectionalLight(0xffffff, 0.85);
    d1.position.set(80, -90, 120);
    scene.add(d1);
    const d2 = new THREE.DirectionalLight(0x38bdf8, 0.45);
    d2.position.set(-80, 80, -30);
    scene.add(d2);

    // Floor Grid
    const grid = new THREE.GridHelper(240, 24, 0x374151, 0x1f2937);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -12;
    scene.add(grid);

    // Assembly Groups
    const baseGroup = new THREE.Group();
    const wingDrumGroup = new THREE.Group();
    const springGroup = new THREE.Group();
    const bridgeGroup = new THREE.Group();
    const motorGroup = new THREE.Group();
    scene.add(baseGroup);
    scene.add(wingDrumGroup);
    scene.add(springGroup);
    scene.add(bridgeGroup);
    scene.add(motorGroup);

    // Materials
    const matBase = new THREE.MeshStandardMaterial({{ color: 0x334155, metalness: 0.7, roughness: 0.3 }});
    const matDrum = new THREE.MeshStandardMaterial({{ color: 0x0284c7, metalness: 0.5, roughness: 0.35, transparent: true, opacity: 1.0 }});
    const matBearing = new THREE.MeshStandardMaterial({{ color: 0xe2e8f0, metalness: 0.9, roughness: 0.15 }});
    const matSpring = new THREE.MeshStandardMaterial({{ color: 0xf59e0b, metalness: 0.8, roughness: 0.25 }});
    const matBridge = new THREE.MeshStandardMaterial({{ color: 0x64748b, metalness: 0.6, roughness: 0.4 }});
    const matMotor = new THREE.MeshStandardMaterial({{ color: 0x10b981, metalness: 0.5, roughness: 0.4 }});
    const matCable = new THREE.MeshStandardMaterial({{ color: 0xa855f7, roughness: 0.6 }});

    // 1. BASE PLATE & PEDESTAL
    const basePlateGeom = new THREE.BoxGeometry(90, 60, 6);
    const basePlate = new THREE.Mesh(basePlateGeom, matBase);
    basePlate.position.set(20, 0, -3);
    baseGroup.add(basePlate);

    // Pedestal Tower (Coaxial with Big Drum at 0,0,0)
    const pedConeGeom = new THREE.CylinderGeometry(20, 26, 12, 48);
    const pedCone = new THREE.Mesh(pedConeGeom, matBase);
    pedCone.position.set(0, 0, 6);
    pedCone.rotation.x = Math.PI / 2;
    baseGroup.add(pedCone);

    const pedShaftGeom = new THREE.CylinderGeometry(20, 20, 36, 48);
    const pedShaft = new THREE.Mesh(pedShaftGeom, matBase);
    pedShaft.position.set(0, 0, 30);
    pedShaft.rotation.x = Math.PI / 2;
    baseGroup.add(pedShaft);

    // 2. BEARINGS (6708-2RS: ID 40 -> R=20, OD 50 -> R=25, W 6)
    function createBearingRing(z) {{
      const bGeom = new THREE.CylinderGeometry(25, 25, 6, 48, 1, true);
      const bMesh = new THREE.Mesh(bGeom, matBearing);
      bMesh.rotation.x = Math.PI / 2;
      bMesh.position.set(0, 0, z);
      return bMesh;
    }}
    const bLow = createBearingRing(15);
    const bHigh = createBearingRing(45);
    baseGroup.add(bLow);
    baseGroup.add(bHigh);

    // 3. BIG DRUM / FLAP SPOOL
    const drumOuterGeom = new THREE.CylinderGeometry(35, 35, 34, 64);
    const drumOuter = new THREE.Mesh(drumOuterGeom, matDrum);
    drumOuter.rotation.x = Math.PI / 2;
    drumOuter.position.set(0, 0, 30);
    wingDrumGroup.add(drumOuter);

    // Wing Spar stub extending from Big Drum
    const sparGeom = new THREE.CylinderGeometry(5, 5, 70, 32);
    const matSpar = new THREE.MeshStandardMaterial({{ color: 0x111827, roughness: 0.8 }});
    const spar = new THREE.Mesh(sparGeom, matSpar);
    spar.rotation.z = Math.PI / 2;
    spar.position.set(65, 0, 30);
    wingDrumGroup.add(spar);

    // 4. HELICAL TORSION SPRING
    // Coils: R=15, 5 coils, wire radius=1.75mm
    const curvePoints = [];
    const nTurns = 4.9;
    const height = 18.0;
    const rCoil = 15.0;
    for (let t = 0; t <= 1; t += 0.005) {{
      const angle = t * nTurns * Math.PI * 2;
      const x = rCoil * Math.cos(angle);
      const y = rCoil * Math.sin(angle);
      const z = 21.0 + t * height;
      curvePoints.push(new THREE.Vector3(x, y, z));
    }}
    // Add straight tangential legs
    // Moving Leg (anchors into Big Drum)
    const legMoveEnd = new THREE.Vector3(26, 0, 21);
    curvePoints.unshift(legMoveEnd);
    // Stationary Leg (anchors upward into Top Bridge)
    const legFixedEnd = new THREE.Vector3(curvePoints[curvePoints.length-1].x, curvePoints[curvePoints.length-1].y, 50);
    curvePoints.push(legFixedEnd);

    const springPath = new THREE.CatmullRomCurve3(curvePoints);
    const springGeom = new THREE.TubeGeometry(springPath, 200, 1.75, 16, false);
    const springMesh = new THREE.Mesh(springGeom, matSpring);
    springGroup.add(springMesh);

    // 5. TOP DOG-BONE BRIDGE
    const bridgeGeom = new THREE.BoxGeometry(60, 22, 6);
    const bridge = new THREE.Mesh(bridgeGeom, matBridge);
    bridge.position.set(22.5, 0, 52);
    bridgeGroup.add(bridge);

    // 6. MOTOR & SMALL DRUM (At X = 45mm, Y = 0mm)
    const motorGeom = new THREE.CylinderGeometry(15, 15, 12, 36);
    const motor = new THREE.Mesh(motorGeom, matMotor);
    motor.rotation.x = Math.PI / 2;
    motor.position.set(45, 0, 6);
    motorGroup.add(motor);

    const smallDrumGeom = new THREE.CylinderGeometry(5, 5, 26, 32);
    const smallDrum = new THREE.Mesh(smallDrumGeom, matBase);
    smallDrum.rotation.x = Math.PI / 2;
    smallDrum.position.set(45, 0, 25);
    motorGroup.add(smallDrum);

    // Top Bearing for Small Drum
    const bSmall = new THREE.Mesh(new THREE.CylinderGeometry(7, 7, 5, 24), matBearing);
    bSmall.rotation.x = Math.PI / 2;
    bSmall.position.set(45, 0, 42);
    motorGroup.add(bSmall);

    // Controls Logic
    let currentMode = 'assembled';
    let isFlapping = true;
    let isOrbiting = false;

    const btnAssembled = document.getElementById('btnAssembled');
    const btnCutaway = document.getElementById('btnCutaway');
    const btnExploded = document.getElementById('btnExploded');
    const btnFlap = document.getElementById('btnFlap');
    const btnRotate = document.getElementById('btnRotate');
    const btnReset = document.getElementById('btnReset');

    function setMode(mode) {{
      currentMode = mode;
      [btnAssembled, btnCutaway, btnExploded].forEach(b => b.classList.remove('active'));
      if (mode === 'assembled') {{
        btnAssembled.classList.add('active');
        matDrum.opacity = 1.0;
        matDrum.transparent = false;
        bridgeGroup.position.z = 0;
        springGroup.position.z = 0;
        wingDrumGroup.position.z = 0;
      }} else if (mode === 'cutaway') {{
        btnCutaway.classList.add('active');
        matDrum.opacity = 0.35;
        matDrum.transparent = true;
        bridgeGroup.position.z = 0;
        springGroup.position.z = 0;
        wingDrumGroup.position.z = 0;
      }} else if (mode === 'exploded') {{
        btnExploded.classList.add('active');
        matDrum.opacity = 0.9;
        matDrum.transparent = false;
        bridgeGroup.position.z = 35;
        springGroup.position.z = 15;
        wingDrumGroup.position.z = -10;
      }}
    }}

    btnAssembled.addEventListener('click', () => setMode('assembled'));
    btnCutaway.addEventListener('click', () => setMode('cutaway'));
    btnExploded.addEventListener('click', () => setMode('exploded'));

    btnFlap.addEventListener('click', () => {{
      isFlapping = !isFlapping;
      btnFlap.classList.toggle('active', isFlapping);
    }});

    btnRotate.addEventListener('click', () => {{
      isOrbiting = !isOrbiting;
      btnRotate.classList.toggle('active', isOrbiting);
    }});

    btnReset.addEventListener('click', () => {{
      camera.position.set(110, -110, 85);
      controls.target.set(20, 0, 25);
      controls.update();
    }});

    controls.target.set(20, 0, 25);
    controls.update();

    let clock = 0;
    function animate() {{
      requestAnimationFrame(animate);
      clock += 0.025;

      if (isFlapping) {{
        // Flap angle = ±45° sinusoidal at resonant frequency
        const flapAmp = (45.0 * Math.PI / 180.0) * Math.sin(clock * 3.5);
        wingDrumGroup.rotation.z = flapAmp;
        // Minor dynamic twist on spring for visualization
        springMesh.rotation.z = flapAmp * 0.5;
      }}

      if (isOrbiting) {{
        scene.rotation.z += 0.005;
      }}

      controls.update();
      renderer.render(scene, camera);
    }}
    animate();

    window.addEventListener('resize', () => {{
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    }});
  </script>
</body>
</html>
"""

with open(OUT_HTML, "w") as f:
    f.write(html_content)
print(f"Generated {OUT_HTML} ({len(html_content)/1024:.1f} KB)")

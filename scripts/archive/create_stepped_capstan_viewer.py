import base64
import os

def create_viewer():
    stl_files = {
        'motor_pinion': 'cad/manufactured/stepped_capstan_motor_pinion.stl',
        'intermediate_pulley': 'cad/manufactured/stepped_capstan_intermediate_pulley.stl',
        'wing_sector': 'cad/manufactured/stepped_capstan_wing_sector_drum.stl',
    }
    
    b64_data = {}
    for key, path in stl_files.items():
        with open(path, 'rb') as f:
            b64_data[key] = base64.b64encode(f.read()).decode('ascii')
            
    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>MCO-1D 16:1 Stepped Capstan Transmission 3D Viewer</title>
  <style>
    body {{
      margin: 0;
      padding: 0;
      overflow: hidden;
      background-color: #0d1117;
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
      background: rgba(22, 27, 34, 0.88);
      border: 1px solid #30363d;
      border-radius: 8px;
      padding: 16px 20px;
      max-width: 380px;
      backdrop-filter: blur(8px);
      box-shadow: 0 8px 24px rgba(0,0,0,0.5);
    }}
    h1 {{
      font-size: 18px;
      margin: 0 0 8px 0;
      color: #58a6ff;
      display: flex;
      align-items: center;
      gap: 8px;
    }}
    .badge {{
      font-size: 11px;
      background: #238636;
      color: #ffffff;
      padding: 2px 6px;
      border-radius: 4px;
      font-weight: 600;
    }}
    p {{
      font-size: 13px;
      line-height: 1.5;
      margin: 4px 0 12px 0;
      color: #8b949e;
    }}
    .spec-table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 12px;
      margin-bottom: 14px;
    }}
    .spec-table td {{
      padding: 4px 0;
      border-bottom: 1px solid #21262d;
    }}
    .spec-table td:last-child {{
      text-align: right;
      font-weight: 600;
      color: #f0f6fc;
      font-family: monospace;
    }}
    .controls {{
      display: flex;
      flex-direction: column;
      gap: 10px;
      border-top: 1px solid #30363d;
      padding-top: 12px;
    }}
    .control-row {{
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 12px;
    }}
    .btn-group {{
      display: flex;
      gap: 6px;
    }}
    button {{
      background: #21262d;
      border: 1px solid #30363d;
      color: #c9d1d9;
      padding: 5px 10px;
      border-radius: 4px;
      font-size: 12px;
      cursor: pointer;
      transition: all 0.2s ease;
    }}
    button:hover {{
      background: #30363d;
      border-color: #8b949e;
      color: #ffffff;
    }}
    button.active {{
      background: #1f6feb;
      border-color: #58a6ff;
      color: #ffffff;
    }}
    input[type=range] {{
      width: 130px;
      accent-color: #58a6ff;
    }}
    #legend {{
      position: absolute;
      bottom: 16px;
      left: 16px;
      background: rgba(22, 27, 34, 0.88);
      border: 1px solid #30363d;
      border-radius: 8px;
      padding: 10px 14px;
      font-size: 12px;
      display: flex;
      gap: 16px;
    }}
    .legend-item {{
      display: flex;
      align-items: center;
      gap: 6px;
    }}
    .color-dot {{
      width: 10px;
      height: 10px;
      border-radius: 50%;
    }}
  </style>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
</head>
<body>
  <div id="canvas-container"></div>
  
  <div id="ui-overlay">
    <h1>16:1 Stepped Capstan <span class="badge">16.0 : 1</span></h1>
    <p>High-torque linkageless flapping transmission for 200° wing travel.</p>
    
    <table class="spec-table">
      <tr><td>Stage 1 (Motor &rarr; Idler)</td><td>4.00 : 1 (&Oslash;12 &rarr; &Oslash;48mm)</td></tr>
      <tr><td>Stage 2 (Idler &rarr; Sector)</td><td>4.00 : 1 (&Oslash;12 &rarr; &Oslash;48mm)</td></tr>
      <tr><td>Total Reduction Ratio</td><td>16.00 : 1</td></tr>
      <tr><td>Wing Angular Travel</td><td>200.0&deg; (&plusmn;100.0&deg;)</td></tr>
      <tr><td>Motor Turns per Stroke</td><td>8.89 revs</td></tr>
      <tr><td>Cable Tensile Margin</td><td>5.4&times; (Dyneema SK99)</td></tr>
      <tr><td>Total Hardware Mass</td><td>38.5 g (Airframe savings: -824g)</td></tr>
    </table>

    <div class="controls">
      <div class="control-row">
        <span>Flap Angle Simulator:</span>
        <input type="range" id="flapSlider" min="-100" max="100" value="0">
        <span id="flapAngleVal" style="width: 45px; text-align: right; font-family: monospace;">0&deg;</span>
      </div>
      <div class="control-row">
        <span>Kinematic Flap Cycle:</span>
        <button id="animToggle" class="active">Animate (2.0 Hz)</button>
      </div>
      <div class="control-row">
        <span>Exploded Assembly:</span>
        <input type="range" id="explodeSlider" min="0" max="100" value="0">
      </div>
      <div class="control-row">
        <span>Render Style:</span>
        <div class="btn-group">
          <button id="btnShaded" class="active">Shaded</button>
          <button id="btnWireframe">Wireframe</button>
        </div>
      </div>
      <div class="control-row">
        <span>Reset Camera:</span>
        <button id="btnResetCam">Top-Down / Reset</button>
      </div>
    </div>
  </div>

  <div id="legend">
    <div class="legend-item"><div class="color-dot" style="background:#58a6ff;"></div> Motor Pinion Spool (Stage 1)</div>
    <div class="legend-item"><div class="color-dot" style="background:#f0883e;"></div> Intermediate Stepped Pulley (Stage 2)</div>
    <div class="legend-item"><div class="color-dot" style="background:#3fb950;"></div> Wing Spar 220&deg; Sector Drum</div>
    <div class="legend-item"><div class="color-dot" style="background:#d2a8ff;"></div> Dyneema Cable Path</div>
  </div>

  <script>
    const stlBase64 = {{
      motor: "{b64_data['motor_pinion']}",
      intermediate: "{b64_data['intermediate_pulley']}",
      sector: "{b64_data['wing_sector']}"
    }};

    function parseSTL(b64) {{
      const binStr = atob(b64);
      const len = binStr.length;
      const bytes = new Uint8Array(len);
      for (let i = 0; i < len; i++) bytes[i] = binStr.charCodeAt(i);
      const view = new DataView(bytes.buffer);
      const nTriangles = view.getUint32(80, true);
      const geom = new THREE.BufferGeometry();
      const pos = new Float32Array(nTriangles * 9);
      const norm = new Float32Array(nTriangles * 9);

      for (let i = 0; i < nTriangles; i++) {{
        const offset = 84 + i * 50;
        const nx = view.getFloat32(offset, true);
        const ny = view.getFloat32(offset + 4, true);
        const nz = view.getFloat32(offset + 8, true);
        for (let j = 0; j < 3; j++) {{
          const vo = offset + 12 + j * 12;
          pos[i * 9 + j * 3]     = view.getFloat32(vo, true);
          pos[i * 9 + j * 3 + 1] = view.getFloat32(vo + 4, true);
          pos[i * 9 + j * 3 + 2] = view.getFloat32(vo + 8, true);
          norm[i * 9 + j * 3]     = nx;
          norm[i * 9 + j * 3 + 1] = ny;
          norm[i * 9 + j * 3 + 2] = nz;
        }}
      }}
      geom.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      geom.setAttribute('normal', new THREE.BufferAttribute(norm, 3));
      return geom;
    }}

    // Setup Three.js scene
    const container = document.getElementById('canvas-container');
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0d1117);

    const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(45, -110, 95);
    camera.up.set(0, 0, 1);

    const renderer = new THREE.WebGLRenderer({{ antialias: true }});
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);

    const controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.target.set(38, 0, 5);

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight1.position.set(50, -50, 80);
    scene.add(dirLight1);

    const dirLight2 = new THREE.DirectionalLight(0x58a6ff, 0.4);
    dirLight2.position.set(-60, 60, -40);
    scene.add(dirLight2);

    const grid = new THREE.GridHelper(200, 20, 0x30363d, 0x21262d);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -5;
    scene.add(grid);

    // Load STL meshes
    const materials = {{
      motor: new THREE.MeshStandardMaterial({{ color: 0x58a6ff, metalness: 0.6, roughness: 0.3 }}),
      intermediate: new THREE.MeshStandardMaterial({{ color: 0xf0883e, metalness: 0.5, roughness: 0.4 }}),
      sector: new THREE.MeshStandardMaterial({{ color: 0x3fb950, metalness: 0.4, roughness: 0.4 }})
    }};

    const motorGeom = parseSTL(stlBase64.motor);
    const interGeom = parseSTL(stlBase64.intermediate);
    const sectorGeom = parseSTL(stlBase64.sector);

    const motorMesh = new THREE.Mesh(motorGeom, materials.motor);
    const interMesh = new THREE.Mesh(interGeom, materials.intermediate);
    const sectorMesh = new THREE.Mesh(sectorGeom, materials.sector);

    // Groups for hierarchical kinematics
    const motorGroup = new THREE.Group();
    motorGroup.position.set(0, 0, 0);
    motorGroup.add(motorMesh);
    scene.add(motorGroup);

    const interGroup = new THREE.Group();
    interGroup.position.set(35, 0, 0);
    interGroup.add(interMesh);
    scene.add(interGroup);

    const sectorGroup = new THREE.Group();
    sectorGroup.position.set(75, 0, 0);
    sectorGroup.add(sectorMesh);
    scene.add(sectorGroup);

    // Wing Spar Tube Graphic (10mm Carbon tube extending 80mm)
    const sparGeom = new THREE.CylinderGeometry(5.0, 5.0, 90, 32);
    const sparMat = new THREE.MeshStandardMaterial({{ color: 0x161b22, roughness: 0.2, metalness: 0.8 }});
    const sparMesh = new THREE.Mesh(sparGeom, sparMat);
    sparMesh.rotation.x = Math.PI / 2;
    sparMesh.position.set(0, 45, 5.0);
    sectorGroup.add(sparMesh);

    // Cable line overlays
    const cableMat = new THREE.LineBasicMaterial({{ color: 0xd2a8ff, linewidth: 2 }});
    const cableGeom1 = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 6, 7),
      new THREE.Vector3(35, 24, 7),
      new THREE.Vector3(35, -24, 7),
      new THREE.Vector3(0, -6, 7)
    ]);
    const cableLine1 = new THREE.Line(cableGeom1, cableMat);
    scene.add(cableLine1);

    const cableGeom2 = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(35, 6, 12),
      new THREE.Vector3(75, 24, 12),
      new THREE.Vector3(75, -24, 12),
      new THREE.Vector3(35, -6, 12)
    ]);
    const cableLine2 = new THREE.Line(cableGeom2, cableMat);
    scene.add(cableLine2);

    // UI & Animation state
    let isAnimating = true;
    let flapAngle = 0;
    const flapSlider = document.getElementById('flapSlider');
    const flapAngleVal = document.getElementById('flapAngleVal');
    const animToggle = document.getElementById('animToggle');
    const explodeSlider = document.getElementById('explodeSlider');

    animToggle.addEventListener('click', () => {{
      isAnimating = !isAnimating;
      animToggle.classList.toggle('active', isAnimating);
      animToggle.innerText = isAnimating ? 'Animate (2.0 Hz)' : 'Play Animation';
    }});

    flapSlider.addEventListener('input', (e) => {{
      isAnimating = false;
      animToggle.classList.remove('active');
      animToggle.innerText = 'Play Animation';
      setFlapAngle(parseFloat(e.target.value));
    }});

    explodeSlider.addEventListener('input', (e) => {{
      const factor = parseFloat(e.target.value) / 100.0;
      motorGroup.position.set(-20 * factor, 0, -15 * factor);
      interGroup.position.set(35, 0, 0);
      sectorGroup.position.set(75 + 30 * factor, 0, 15 * factor);
    }});

    document.getElementById('btnShaded').addEventListener('click', () => {{
      setWireframe(false);
      document.getElementById('btnShaded').classList.add('active');
      document.getElementById('btnWireframe').classList.remove('active');
    }});

    document.getElementById('btnWireframe').addEventListener('click', () => {{
      setWireframe(true);
      document.getElementById('btnWireframe').classList.add('active');
      document.getElementById('btnShaded').classList.remove('active');
    }});

    function setWireframe(val) {{
      materials.motor.wireframe = val;
      materials.intermediate.wireframe = val;
      materials.sector.wireframe = val;
    }}

    document.getElementById('btnResetCam').addEventListener('click', () => {{
      camera.position.set(38, 0, 140);
      camera.up.set(0, 1, 0);
      controls.target.set(38, 0, 5);
      controls.update();
    }});

    function setFlapAngle(deg) {{
      flapAngle = deg;
      flapSlider.value = deg;
      flapAngleVal.innerText = deg.toFixed(1) + '°';

      const rad = THREE.MathUtils.degToRad(deg);
      sectorGroup.rotation.z = rad;
      // Kinematic reduction: 4:1 stage 2, 4:1 stage 1 (total 16:1)
      interGroup.rotation.z = -rad * 4.0;
      motorGroup.rotation.z = rad * 16.0;
    }}

    let clock = new THREE.Clock();
    function animate() {{
      requestAnimationFrame(animate);
      if (isAnimating) {{
        const t = clock.getElapsedTime();
        const cycle = Math.sin(t * 2.0 * Math.PI * 2.0); // 2.0 Hz flapping
        const deg = cycle * 100.0; // 200° stroke (-100° to +100°)
        setFlapAngle(deg);
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

    out_file = "cad/drawings/stepped_capstan_3d_viewer.html"
    os.makedirs("cad/drawings", exist_ok=True)
    with open(out_file, "w") as f:
        f.write(html_content)
    print(f"Generated {out_file} (Size: {os.path.getsize(out_file)/1024:.1f} KB)")

if __name__ == '__main__':
    create_viewer()

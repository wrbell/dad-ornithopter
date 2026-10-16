import base64
import os

with open("cad/manufactured/palm_v2.stl", "rb") as f:
    b64_stl = base64.b64encode(f.read()).decode("ascii")

html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Palm V2 - 3D CAD Inspection Viewer</title>
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
      color: #38bdf8;
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
      background: #0284c7;
      border-color: #38bdf8;
      color: #ffffff;
    }}
  </style>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
</head>
<body>
  <div id="canvas-container"></div>
  
  <div id="ui-overlay">
    <h1>Palm V2.STEP <span class="badge">Imported</span></h1>
    <p>Capstan Spool & Shoulder Hub from your Downloads directory.</p>
    
    <table class="spec-table">
      <tr><td>Outer Flange Diameter</td><td>&Oslash; 50.0 mm</td></tr>
      <tr><td>Hub Boss Diameter</td><td>&Oslash; 33.8 mm / 35.0 mm</td></tr>
      <tr><td>Total Axial Length</td><td>33.0 mm</td></tr>
      <tr><td>Bolt Circle (4x Holes)</td><td>PCD 19.0 mm (&Oslash;5.5mm)</td></tr>
      <tr><td>Cable Tracking Groove</td><td>Central V/U Circumferential Groove</td></tr>
      <tr><td>Internal Structure</td><td>X-Truss Cruciform Core Ribs</td></tr>
      <tr><td>Solid Volume</td><td>26.36 cm&sup3;</td></tr>
      <tr><td>Estimated Mass (7075-T6)</td><td>74.1 g</td></tr>
      <tr><td>Estimated Mass (CF-Nylon)</td><td>33.0 g</td></tr>
    </table>

    <div class="controls">
      <div class="control-row">
        <span>Render Style:</span>
        <div class="btn-group">
          <button id="btnShaded" class="active">Shaded</button>
          <button id="btnWireframe">Wireframe</button>
        </div>
      </div>
      <div class="control-row">
        <span>Auto-Rotation:</span>
        <button id="btnRotate" class="active">Rotating</button>
      </div>
      <div class="control-row">
        <span>Reset Camera:</span>
        <button id="btnReset">Reset View</button>
      </div>
    </div>
  </div>

  <script>
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
      geom.computeBoundingBox();
      return geom;
    }}

    const container = document.getElementById('canvas-container');
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0d1117);

    const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(70, -70, 50);
    camera.up.set(0, 0, 1);

    const renderer = new THREE.WebGLRenderer({{ antialias: true }});
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    const controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;

    // Lights
    scene.add(new THREE.AmbientLight(0xffffff, 0.5));
    const d1 = new THREE.DirectionalLight(0xffffff, 0.8);
    d1.position.set(60, -60, 80);
    scene.add(d1);
    const d2 = new THREE.DirectionalLight(0x38bdf8, 0.5);
    d2.position.set(-60, 60, -40);
    scene.add(d2);

    const grid = new THREE.GridHelper(100, 20, 0x30363d, 0x21262d);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -30;
    scene.add(grid);

    // Mesh
    const material = new THREE.MeshStandardMaterial({{
      color: 0x38bdf8,
      metalness: 0.5,
      roughness: 0.35,
      wireframe: false
    }});

    const geom = parseSTL("{b64_stl}");
    // Center geometry
    geom.computeBoundingBox();
    const center = new THREE.Vector3();
    geom.boundingBox.getCenter(center);
    geom.center(); // centers at origin

    const mesh = new THREE.Mesh(geom, material);
    scene.add(mesh);
    controls.target.set(0, 0, 0);

    let rotating = true;
    const btnRotate = document.getElementById('btnRotate');
    btnRotate.addEventListener('click', () => {{
      rotating = !rotating;
      btnRotate.classList.toggle('active', rotating);
      btnRotate.innerText = rotating ? 'Rotating' : 'Static';
    }});

    const btnShaded = document.getElementById('btnShaded');
    const btnWireframe = document.getElementById('btnWireframe');
    btnShaded.addEventListener('click', () => {{
      material.wireframe = false;
      btnShaded.classList.add('active');
      btnWireframe.classList.remove('active');
    }});
    btnWireframe.addEventListener('click', () => {{
      material.wireframe = true;
      btnWireframe.classList.add('active');
      btnShaded.classList.remove('active');
    }});

    document.getElementById('btnReset').addEventListener('click', () => {{
      camera.position.set(70, -70, 50);
      controls.target.set(0, 0, 0);
      controls.update();
    }});

    function animate() {{
      requestAnimationFrame(animate);
      if (rotating) {{
        mesh.rotation.x += 0.005;
        mesh.rotation.y += 0.008;
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

viewer_path = "cad/drawings/palm_v2_3d_viewer.html"
with open(viewer_path, "w") as f:
    f.write(html_content)
print(f"Generated {viewer_path} ({len(html_content)/1024:.1f} KB)")

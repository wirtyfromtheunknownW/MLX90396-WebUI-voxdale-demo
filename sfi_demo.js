/**
 * sfi_demo.js
 * Standalone SFI Joystick, 3D Dome Kinematics, Centered Z-UP XYZ Chart, Polar Radar & Battle Matrix
 */

// --- Simulation State ---
let autoPattern = true;
let coilActive = false;
let animTime = 0;
let joyX = 0, joyY = 0, joyZ = 0;
let isLiveHardwareConnected = false;
let lastKinematicsAt = performance.now();
let previousJoy = { x: 0, y: 0, z: 0 };
let motionTelemetry = { velocity: 0, force: 0, x: 0, y: 0, z: 0 };
let sens = [1.0, 1.0, 1.0, 1.0];

let stdTraceHistory = [];
let mlxTraceHistory = [];

let mainSceneObj = null;
let stdPlot = null;
let mlxPlot = null;
let joyAssemblyRef = null;
let xyzChartObj = null;

// Radar 2D Canvases
let radarCanvasStd = null, radarCtxStd = null;
let radarCanvasMlx = null, radarCtxMlx = null;

// Hardware Callback
let onHardwareCoilToggle = null;

// 4D Canvas State
let canvas4D, ctx4D;
let width4D = 0, height4D = 0;
let stick4D = { x: 0, y: 0, z: 0, velocity: 0, force: 0 };
let prevStick4D = { x: 0, y: 0, z: 0 };
let trail4D = [];

// Raw Live Hardware Packet Holder
let liveHardwarePacket = null;

export function setHardwareCoilCallback(cb) {
  onHardwareCoilToggle = cb;
}

export function setSfiHardwareTracking(active) {
  isLiveHardwareConnected = active;
}

// EXACT ORIGINAL DOME ENTRY FUNCTION
export function updateSfiDomeKinematics(x, y, z, telemetryPayload = null) {
  const now = performance.now();
  const elapsedSeconds = Math.max((now - lastKinematicsAt) / 1000, 1 / 240);
  lastKinematicsAt = now;
  joyX = Math.max(-1, Math.min(1, x));
  joyY = Math.max(-1, Math.min(1, y));
  joyZ = Math.max(-1, Math.min(1, z));
  motionTelemetry.x = (joyX - previousJoy.x) / elapsedSeconds;
  motionTelemetry.y = (joyY - previousJoy.y) / elapsedSeconds;
  motionTelemetry.z = (joyZ - previousJoy.z) / elapsedSeconds;
  motionTelemetry.velocity = Math.hypot(motionTelemetry.x, motionTelemetry.y, motionTelemetry.z) * 90;
  motionTelemetry.force = Math.min(100, Math.hypot(joyX, joyY) * 55 + Math.max(0, joyZ) * 45);
  previousJoy = { x: joyX, y: joyY, z: joyZ };

  if (telemetryPayload) {
    liveHardwarePacket = telemetryPayload;
  }

  stick4D.targetX = joyX * 90;
  stick4D.targetY = joyY * 90;
  stick4D.targetZ = Math.max(0, (joyZ + 1) * 35);
}

export function initSfiDemo() {
  const container = document.getElementById('canvas3d-container');
  if (!container || !window.THREE) return;

  // DOM Elements
  const btnPattern = document.getElementById('btn-pattern');
  const btnCoil = document.getElementById('btn-coil');
  const btnCoilToggle = document.getElementById('btn-coil-toggle');
  const btnResetTrace = document.getElementById('btn-reset-trace');
  const coilLbl = document.getElementById('coil-readout-lbl');

  const toggleCoilAction = () => {
    coilActive = !coilActive;
    if (coilActive) {
      btnCoil?.classList.add('active');
      btnCoilToggle?.classList.add('active');
      if (coilLbl) {
        coilLbl.innerText = 'COIL: ACTIVE (+5.0 mT STRAY FIELD)';
        coilLbl.style.color = '#ff3366';
      }
    } else {
      btnCoil?.classList.remove('active');
      btnCoilToggle?.classList.remove('active');
      if (coilLbl) {
        coilLbl.innerText = 'COIL: INACTIVE (0.0 mT)';
        coilLbl.style.color = '#94a3b8';
      }
    }
    if (typeof onHardwareCoilToggle === 'function') {
      onHardwareCoilToggle(coilActive);
    }
  };

  btnCoil?.addEventListener('click', toggleCoilAction);
  btnCoilToggle?.addEventListener('click', toggleCoilAction);

  btnPattern?.addEventListener('click', () => {
    autoPattern = !autoPattern;
    btnPattern.style.opacity = autoPattern ? '1' : '0.5';
  });

  btnResetTrace?.addEventListener('click', () => {
    stdTraceHistory = [];
    mlxTraceHistory = [];
    trail4D = [];
    resetPlotLines();
  });

  // --- 1. THREE.JS DOME SCENE SETUP ---
const scene = new THREE.Scene();

  const width = container.clientWidth || 800;
  const height = container.clientHeight || 560;
  
  // 1. Widen FOV slightly (from 35 to 42)
  const camera = new THREE.PerspectiveCamera(42, width / height, 0.1, 1000);
  
  // 2. Move camera back and slightly higher (was: 0, 8.5, 12.5)
  camera.position.set(0, 11, 21);

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setClearColor(0x000000, 0);
  renderer.setSize(width, height);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  container.appendChild(renderer.domElement);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  // 3. Center target between the base and top of knob (was: 0, 0.5, 0)
  controls.target.set(0, 1.8, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;

  // 4. Optional: Set zoom bounds so user cannot zoom through the dome
  controls.minDistance = 10;
  controls.maxDistance = 38;

  scene.add(new THREE.AmbientLight(0xffffff, 1.2));
  const dirLight = new THREE.DirectionalLight(0x00d2ff, 1.4);
  dirLight.position.set(10, 25, 15);
  scene.add(dirLight);

  // Blue Hemispherical Base
  const baseMesh = new THREE.Mesh(
    new THREE.SphereGeometry(5.2, 32, 16, 0, Math.PI * 2, Math.PI * 0.5, Math.PI * 0.5),
    new THREE.MeshStandardMaterial({ color: 0x0a2540, roughness: 0.2, metalness: 0.5 })
  );
  scene.add(baseMesh);

  // Rim Accent
  const rimMesh = new THREE.Mesh(
    new THREE.TorusGeometry(5.25, 0.18, 16, 100),
    new THREE.MeshStandardMaterial({ color: 0xd97706, metalness: 0.9, roughness: 0.1 })
  );
  rimMesh.rotation.x = Math.PI / 2;
  scene.add(rimMesh);

  // Internal PCB
  const pcbMesh = new THREE.Mesh(
    new THREE.CylinderGeometry(4.8, 4.8, 0.2, 32),
    new THREE.MeshStandardMaterial({ color: 0x15803d, roughness: 0.4 })
  );
  pcbMesh.position.y = 0.1;
  scene.add(pcbMesh);

  // IC Package
  const icMesh = new THREE.Mesh(
    new THREE.BoxGeometry(1.6, 0.35, 1.6),
    new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.2 })
  );
  icMesh.position.y = 0.4;
  scene.add(icMesh);

  // Pin 1 Dot
  const pin1Dot = new THREE.Mesh(
    new THREE.SphereGeometry(0.08, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0xffffff })
  );
  pin1Dot.position.set(-0.6, 0.59, -0.6);
  scene.add(pin1Dot);

  // Joystick Assembly
  const domeRadius = 5.2;
  const joyAssembly = new THREE.Group();
  joyAssemblyRef = joyAssembly;

  const axialMagnetGroup = new THREE.Group();
  axialMagnetGroup.position.y = 0.61;

  const northPoleMesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.9, 0.9, 0.25, 32),
    new THREE.MeshStandardMaterial({ color: 0xff1144, roughness: 0.2, metalness: 0.3 })
  );
  northPoleMesh.position.y = 0.125;
  axialMagnetGroup.add(northPoleMesh);

  const ringMesh = new THREE.Mesh(
    new THREE.TorusGeometry(0.91, 0.03, 16, 32),
    new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.9 })
  );
  ringMesh.rotation.x = Math.PI / 2;
  ringMesh.position.y = 0.25;
  axialMagnetGroup.add(ringMesh);

  const southPoleMesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.9, 0.9, 0.25, 32),
    new THREE.MeshStandardMaterial({ color: 0x0088ff, roughness: 0.2, metalness: 0.3 })
  );
  southPoleMesh.position.y = 0.375;
  axialMagnetGroup.add(southPoleMesh);
  joyAssembly.add(axialMagnetGroup);

  const shaftMesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.22, 0.22, 3.2, 16),
    new THREE.MeshStandardMaterial({ color: 0xd97706, metalness: 0.6 })
  );
  shaftMesh.position.y = 2.4;
  joyAssembly.add(shaftMesh);

  const knobMesh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.85, 0.65, 1.8, 24),
    new THREE.MeshStandardMaterial({ color: 0xd97706, roughness: 0.2 })
  );
  knobMesh.position.y = 4.2;
  joyAssembly.add(knobMesh);
  scene.add(joyAssembly);

const domeMesh = new THREE.Mesh(
  new THREE.SphereGeometry(domeRadius, 32, 24, 0, Math.PI * 2, 0, Math.PI * 0.5),
  new THREE.MeshStandardMaterial({
    color: 0x4a5568,
    transparent: true,
    opacity: 0.4,
    roughness: 0.15,
    metalness: 0.1
  })
);
scene.add(domeMesh);

  // Knob Drag Interaction
  const raycaster = new THREE.Raycaster();
  const mouse = new THREE.Vector2();
  let isDraggingKnob = false;

  container.addEventListener('mousedown', (e) => {
    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((e.clientX - rect.left) / container.clientWidth) * 2 - 1;
    mouse.y = -((e.clientY - rect.top) / container.clientHeight) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    if (raycaster.intersectObject(knobMesh).length > 0 || e.shiftKey) {
      isDraggingKnob = true;
      controls.enabled = false;
      autoPattern = false;
    }
  });

  window.addEventListener('mousemove', (e) => {
    if (!isDraggingKnob) return;
    const rect = renderer.domElement.getBoundingClientRect();
    joyX = Math.max(-1, Math.min(1, (((e.clientX - rect.left) / container.clientWidth) * 2 - 1) * 1.5));
    joyY = Math.max(-1, Math.min(1, (-((e.clientY - rect.top) / container.clientHeight) * 2 + 1) * 1.5));
  });

  window.addEventListener('mouseup', () => {
    if (isDraggingKnob) {
      isDraggingKnob = false;
      controls.enabled = true;
    }
  });

  mainSceneObj = { scene, camera, renderer, controls, container };

  // --- 2. SFI COMPARISON SUB-PLOTS ---
  stdPlot = create3DFieldPlot('canvas3d-std-plot', 0xff3366);
  mlxPlot = create3DFieldPlot('canvas3d-mlx-plot', 0x00ff88);

  // --- 3. RADAR CANVASES ---
  initRadarCanvases();

  function animate() {
    requestAnimationFrame(animate);

    if (autoPattern && !isLiveHardwareConnected) {
      animTime += 0.025;
      let cycle = (animTime % 24) / 24;

      if (cycle < 0.25) {
        let t = (cycle / 0.25) * Math.PI * 2;
        joyX = Math.cos(t) * 0.85; joyY = Math.sin(t) * 0.85; joyZ = 0;
      } else if (cycle < 0.50) {
        let t = ((cycle - 0.25) / 0.25) * Math.PI * 4;
        if (Math.sin(t) > 0) { joyX = Math.sin(t * 2) * 0.9; joyY = 0; }
        else { joyX = 0; joyY = Math.cos(t * 2) * 0.9; }
        joyZ = 0;
      } else if (cycle < 0.75) {
        joyX = Math.sin(animTime * 2) * 0.25; joyY = Math.cos(animTime * 2) * 0.25;
        joyZ = Math.sin((cycle - 0.50) * Math.PI * 8) * 0.8;
      } else {
        joyX = Math.sin(animTime * 1.7) * 0.75 + Math.cos(animTime * 0.5) * 0.2;
        joyY = Math.cos(animTime * 1.3) * 0.75 + Math.sin(animTime * 0.7) * 0.2;
        joyZ = Math.sin(animTime * 2.5) * 0.4;
      }

      stick4D.targetX = joyX * 90;
      stick4D.targetY = joyY * 90;
      stick4D.targetZ = Math.max(0, (joyZ + 1) * 35);

      const now = performance.now();
      const elapsedSeconds = Math.max((now - lastKinematicsAt) / 1000, 1 / 240);
      lastKinematicsAt = now;
      motionTelemetry.x = (joyX - previousJoy.x) / elapsedSeconds;
      motionTelemetry.y = (joyY - previousJoy.y) / elapsedSeconds;
      motionTelemetry.z = (joyZ - previousJoy.z) / elapsedSeconds;
      motionTelemetry.velocity = Math.hypot(motionTelemetry.x, motionTelemetry.y, motionTelemetry.z) * 90;
      motionTelemetry.force = Math.min(100, Math.hypot(joyX, joyY) * 55 + Math.max(0, joyZ) * 45);
      previousJoy = { x: joyX, y: joyY, z: joyZ };
    }

    // Kinematics along Dome
    let tiltAngle = Math.sqrt(joyX * joyX + joyY * joyY) * 0.45;
    let tiltDir = Math.atan2(joyY, joyX);

    joyAssembly.position.set(
      Math.sin(tiltAngle) * Math.cos(tiltDir) * domeRadius,
      Math.cos(tiltAngle) * domeRadius + (joyZ * 0.3),
      Math.sin(tiltAngle) * Math.sin(tiltDir) * domeRadius
    );
    joyAssembly.rotation.z = -joyX * 0.35;
    joyAssembly.rotation.x = joyY * 0.35;

    const joyPosTxt = document.getElementById('joy-pos-text');
    if (joyPosTxt) {
      joyPosTxt.innerText = `Tilt X: ${(joyX * 22).toFixed(1)}° | Tilt Y: ${(joyY * 22).toFixed(1)}° | Press Z: ${(joyZ * 2.0).toFixed(1)} mm`;
    }

    // Magnetic Math (Req 1 & 2)
    let trueBx = joyX * 18.0;
    let trueBy = joyY * 18.0;
    let trueBz = joyZ * 12.0;

    let noiseX = coilActive ? 6.2 + (Math.random() - 0.5) * 0.6 : 0;
    let noiseY = coilActive ? -5.8 + (Math.random() - 0.5) * 0.6 : 0;
    let noiseZ = coilActive ? 4.5 + (Math.random() - 0.5) * 0.4 : 0;

    let stdBx = trueBx + noiseX;
    let stdBy = trueBy + noiseY;
    let stdBz = 22.0 + trueBz + noiseZ;

    let mlxDBx = trueBx;
    let mlxDBy = trueBy;
    let mlxDBzX = 22.0 + trueBz;
    let mlxDBzY = 22.0 + trueBz;

    // Use live hardware packet if available
    if (isLiveHardwareConnected && liveHardwarePacket && liveHardwarePacket.p0Raw) {
      stdBx = liveHardwarePacket.p0Raw.x / 20 + noiseX;
      stdBy = liveHardwarePacket.p0Raw.y / 20 + noiseY;
      stdBz = liveHardwarePacket.p0Raw.z / 20 + noiseZ;

      mlxDBx = liveHardwarePacket.diffRaw.x / 20;
      mlxDBy = liveHardwarePacket.diffRaw.y / 20;
      mlxDBzX = liveHardwarePacket.diffRaw.z / 20;
      mlxDBzY = (liveHardwarePacket.diffRaw.bzY ?? liveHardwarePacket.diffRaw.z) / 20;
    }

    // Calculated Signal Strength Magnitude
    let magStd = Math.sqrt(stdBx ** 2 + stdBy ** 2 + stdBz ** 2);
    let magDiff = Math.sqrt(mlxDBx ** 2 + mlxDBy ** 2 + mlxDBzX ** 2 + mlxDBzY ** 2);

    updateSignalBarsUI(stdBx, stdBy, stdBz, mlxDBx, mlxDBy, mlxDBzX, mlxDBzY, magStd, magDiff);

    // Radar Angles
    let trueAngleDeg = (Math.atan2(joyY, joyX) * 180 / Math.PI + 360) % 360;
    let stdAngleDeg = (Math.atan2(stdBy, stdBx) * 180 / Math.PI + 360) % 360;
    let diffAngleDeg = (Math.atan2(mlxDBy, mlxDBx) * 180 / Math.PI + 360) % 360;

    let stdError = Math.abs(stdAngleDeg - trueAngleDeg);
    if (stdError > 180) stdError = 360 - stdError;

    let diffError = Math.abs(diffAngleDeg - trueAngleDeg);
    if (diffError > 180) diffError = 360 - diffError;

    const angleStdEl = document.getElementById('radar-angle-std');
    const errorStdEl = document.getElementById('radar-error-std');
    const angleMlxEl = document.getElementById('radar-angle-mlx');
    const errorMlxEl = document.getElementById('radar-error-mlx');

    if (angleStdEl) angleStdEl.innerText = stdAngleDeg.toFixed(1) + '°';
    if (errorStdEl) errorStdEl.innerText = stdError.toFixed(1) + '°';
    if (angleMlxEl) angleMlxEl.innerText = diffAngleDeg.toFixed(1) + '°';
    if (errorMlxEl) errorMlxEl.innerText = diffError.toFixed(1) + '°';

    const alpha = Math.atan2(joyY, Math.hypot(joyX, joyZ)) * 180 / Math.PI;
    const beta = Math.atan2(joyX, Math.hypot(joyY, joyZ)) * 180 / Math.PI;
    const alphaEl = document.getElementById('angle-alpha');
    const betaEl = document.getElementById('angle-beta');
    const thetaEl = document.getElementById('angle-theta');
    if (alphaEl) alphaEl.innerText = alpha.toFixed(1) + '°';
    if (betaEl) betaEl.innerText = beta.toFixed(1) + '°';
    if (thetaEl) thetaEl.innerText = trueAngleDeg.toFixed(1) + '°';

   // --- ONLY PUSH HISTORY IF THE TABS THAT USE IT ARE ACTIVE ---
    const activeTab = document.querySelector('.tab-content.active')?.id;

    if (activeTab === 'tab-telemetry') {
      stdTraceHistory.push({ x: stdBx, y: stdBy, z: stdBz });
      mlxTraceHistory.push({ x: mlxDBx, y: mlxDBy, z: mlxDBzX });

      if (stdTraceHistory.length > 120) stdTraceHistory.shift();
      if (mlxTraceHistory.length > 120) mlxTraceHistory.shift();
    }

// --- RENDER ONLY THE ACTIVE TAB ---
    if (activeTab === 'tab-idle' && mainSceneObj) {
      controls.update();
      renderer.render(scene, camera);
    } else if (activeTab === 'tab-telemetry') {
      if (stdPlot) update3DPlot(stdPlot, stdTraceHistory, true);
      if (mlxPlot) update3DPlot(mlxPlot, mlxTraceHistory, false);
      drawRadar(radarCtxStd, stdBx, stdBy, trueBx, trueBy, '#ff3366', 'LEGACY 3D HALL');
      drawRadar(radarCtxMlx, mlxDBx, mlxDBy, trueBx, trueBy, '#00ff88', 'MLX90396 SFI');
    }
  }

  animate();
}

// --- SIGNAL BARS & MAGNITUDE UI ---
function updateSignalBarsUI(bx0, by0, bz0, dbx, dby, dbzX, dbzY, magStd, magDiff) {
  const updateBar = (meterId, txtId, val, maxRange = 35, unit = 'mT') => {
    const meter = document.getElementById(meterId);
    const txt = document.getElementById(txtId);
    if (!meter || !txt) return;

    txt.innerText = val.toFixed(1) + ' ' + unit;
    const percent = Math.min(100, Math.max(0, ((val + maxRange) / (maxRange * 2)) * 100));
    meter.style.width = percent + '%';
  };

  updateBar('meter-raw-bx', 'txt-raw-bx', bx0);
  updateBar('meter-raw-by', 'txt-raw-by', by0);
  updateBar('meter-raw-bz', 'txt-raw-bz', bz0);

  updateBar('meter-diff-bx', 'txt-diff-bx', dbx, 35, 'mT/mm');
  updateBar('meter-diff-by', 'txt-diff-by', dby, 35, 'mT/mm');
  updateBar('meter-diff-bz', 'txt-diff-bz', dbzX, 35, 'mT/mm');
  updateBar('meter-diff-bz-y', 'txt-diff-bz-y', dbzY, 35, 'mT/mm');

  const elMagStd = document.getElementById('mag-val-std');
  const elMagDiff = document.getElementById('mag-val-diff');
  if (elMagStd) elMagStd.innerText = magStd.toFixed(1) + ' mT';
  if (elMagDiff) elMagDiff.innerText = magDiff.toFixed(1) + ' mT';
}

// --- 360° POLAR RADAR MAP ENGINE ---
function initRadarCanvases() {
  radarCanvasStd = document.getElementById('radarCanvasStd');
  radarCanvasMlx = document.getElementById('radarCanvasMlx');
  if (radarCanvasStd) radarCtxStd = radarCanvasStd.getContext('2d');
  if (radarCanvasMlx) radarCtxMlx = radarCanvasMlx.getContext('2d');
  resizeRadarCanvases();
}

function resizeRadarCanvases() {
  [radarCanvasStd, radarCanvasMlx].forEach(can => {
    if (!can) return;
    can.width = can.clientWidth || 300;
    can.height = can.clientHeight || 280;
  });
}

function drawRadar(ctx, curX, curY, targetX, targetY, themeColor, label) {
  if (!ctx || !ctx.canvas) return;
  const w = ctx.canvas.width;
  const h = ctx.canvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const radius = Math.min(cx, cy) - 25;

  ctx.clearRect(0, 0, w, h);

  ctx.fillStyle = '#030816';
  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.fill();

  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1;
  for (let r = 0.25; r <= 1.0; r += 0.25) {
    ctx.beginPath();
    ctx.arc(cx, cy, radius * r, 0, Math.PI * 2);
    ctx.stroke();
  }

  ctx.beginPath();
  ctx.moveTo(cx - radius, cy); ctx.lineTo(cx + radius, cy);
  ctx.moveTo(cx, cy - radius); ctx.lineTo(cx, cy + radius);
  ctx.stroke();

  ctx.fillStyle = '#64748b';
  ctx.font = '10px monospace';
  ctx.textAlign = 'center';
  ctx.fillText('0° (+X)', cx + radius - 15, cy - 6);
  ctx.fillText('90° (+Y)', cx, cy - radius + 14);
  ctx.fillText('180° (-X)', cx - radius + 22, cy - 6);
  ctx.fillText('270° (-Y)', cx, cy + radius - 8);

  const maxScale = 22.0;
  const targetPxX = cx + (targetX / maxScale) * radius;
  const targetPxY = cy - (targetY / maxScale) * radius;

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
  ctx.setLineDash([3, 3]);
  ctx.beginPath();
  ctx.arc(targetPxX, targetPxY, 9, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);

  const measuredPxX = cx + (curX / maxScale) * radius;
  const measuredPxY = cy - (curY / maxScale) * radius;

  ctx.strokeStyle = themeColor;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(measuredPxX, measuredPxY);
  ctx.stroke();

  ctx.fillStyle = themeColor;
  ctx.shadowColor = themeColor;
  ctx.shadowBlur = 12;
  ctx.beginPath();
  ctx.arc(measuredPxX, measuredPxY, 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur = 0;
}

// --- CENTERED Z-UP 3D XYZ CHART ---
function initXyzChart() {
  const container = document.getElementById('canvas-xyz-main');
  if (!container || !window.THREE) return;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x020612);

  const w = container.clientWidth || 800;
  const h = container.clientHeight || 560;
  const camera = new THREE.PerspectiveCamera(38, w / h, 0.1, 100);

  camera.up.set(0, 0, 1);
  camera.position.set(13, -15, 11);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(w, h);
  container.appendChild(renderer.domElement);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;

  const gridFloor = new THREE.GridHelper(14, 14, 0x00d2ff, 0x1e293b);
  gridFloor.rotation.x = Math.PI / 2;
  gridFloor.position.set(0, 0, 0);
  scene.add(gridFloor);

  const originMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.18, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0x94a3b8, opacity: 0.6, transparent: true })
  );
  originMarker.position.set(0, 0, 0);
  scene.add(originMarker);

  const origin = new THREE.Vector3(0, 0, 0);
  const arrowLen = 5.5;
  const arrowHeadLen = 0.9;
  const arrowHeadWidth = 0.45;

  const arrowX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), origin, arrowLen, 0xff3366, arrowHeadLen, arrowHeadWidth);
  const arrowY = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), origin, arrowLen, 0x00ff88, arrowHeadLen, arrowHeadWidth);
  const arrowZ = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), origin, arrowLen, 0x00d2ff, arrowHeadLen, arrowHeadWidth);
  scene.add(arrowX);
  scene.add(arrowY);
  scene.add(arrowZ);

  const makeNegAxis = (dir, color) => {
    const geo = new THREE.BufferGeometry().setFromPoints([origin, dir.clone().multiplyScalar(5.5)]);
    const mat = new THREE.LineDashedMaterial({ color, dashSize: 0.3, gapSize: 0.2, opacity: 0.35, transparent: true });
    const line = new THREE.Line(geo, mat);
    line.computeLineDistances();
    scene.add(line);
  };
  makeNegAxis(new THREE.Vector3(-1, 0, 0), 0xff3366);
  makeNegAxis(new THREE.Vector3(0, -1, 0), 0x00ff88);
  makeNegAxis(new THREE.Vector3(0, 0, -1), 0x00d2ff);

  const stemGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0)]);
  const stemMat = new THREE.LineBasicMaterial({ color: 0xffaa00, linewidth: 2, transparent: true, opacity: 0.8 });
  const stemLine = new THREE.Line(stemGeo, stemMat);
  scene.add(stemLine);

  const maxPoints = 140;
  const linePositions = new Float32Array(maxPoints * 3);
  const lineGeo = new THREE.BufferGeometry();
  lineGeo.setAttribute('position', new THREE.BufferAttribute(linePositions, 3));
  const lineMat = new THREE.LineBasicMaterial({ color: 0x00f3ff, linewidth: 2.8 });
  const lineMesh = new THREE.Line(lineGeo, lineMat);
  scene.add(lineMesh);

  const headMesh = new THREE.Mesh(
    new THREE.SphereGeometry(0.36, 24, 24),
    new THREE.MeshStandardMaterial({ color: 0xff5500, roughness: 0.2, metalness: 0.4 })
  );
  headMesh.position.set(0, 0, 0);
  scene.add(headMesh);

  const ballLight = new THREE.PointLight(0xffffff, 1.5, 30);
  ballLight.position.set(5, 8, 10);
  scene.add(ballLight);
  scene.add(new THREE.AmbientLight(0xffffff, 0.8));

  xyzChartObj = {
    scene, camera, renderer, controls,
    lineGeo, headMesh, stemLine, maxPoints, container
  };

  const viewButtons = document.querySelectorAll('.btn-xyz-view');
  viewButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      viewButtons.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      setXyzCameraPreset(btn.dataset.view);
    });
  });
}

function setXyzCameraPreset(viewKey) {
  if (!xyzChartObj) return;
  const { camera, controls } = xyzChartObj;

  controls.target.set(0, 0, 0);

  if (viewKey === 'iso') {
    camera.up.set(0, 0, 1);
    camera.position.set(13, -15, 11);
  } else if (viewKey === 'x') {
    camera.up.set(0, 0, 1);
    camera.position.set(18, 0, 0);
  } else if (viewKey === 'y') {
    camera.up.set(0, 0, 1);
    camera.position.set(0, -18, 0);
  } else if (viewKey === 'z') {
    camera.up.set(0, 1, 0);
    camera.position.set(0, 0, 18);
  }

  controls.update();
}

function updateXyzChart(history, curX, curY, curZ) {
  if (!xyzChartObj) return;

  const { scene, camera, renderer, controls, lineGeo, headMesh, stemLine, maxPoints } = xyzChartObj;
  const len = history.length;

  const lblX = document.getElementById('xyz-val-x');
  const lblY = document.getElementById('xyz-val-y');
  const lblZ = document.getElementById('xyz-val-z');
  if (lblX) lblX.innerText = curX.toFixed(1);
  if (lblY) lblY.innerText = curY.toFixed(1);
  if (lblZ) lblZ.innerText = (curZ - 22.0).toFixed(1);

  const SCALE = 0.22;

  const positions = lineGeo.attributes.position.array;
  for (let i = 0; i < maxPoints; i++) {
    if (i < len) {
      positions[i * 3 + 0] = history[i].x * SCALE;
      positions[i * 3 + 1] = history[i].y * SCALE;
      positions[i * 3 + 2] = (history[i].z - 22.0) * SCALE;
    } else {
      positions[i * 3 + 0] = 0;
      positions[i * 3 + 1] = 0;
      positions[i * 3 + 2] = 0;
    }
  }
  lineGeo.attributes.position.needsUpdate = true;

  const livePosX = curX * SCALE;
  const livePosY = curY * SCALE;
  const livePosZ = (curZ - 22.0) * SCALE;

  headMesh.position.set(livePosX, livePosY, livePosZ);

  const stemPos = stemLine.geometry.attributes.position.array;
  stemPos[0] = 0; stemPos[1] = 0; stemPos[2] = 0;
  stemPos[3] = livePosX; stemPos[4] = livePosY; stemPos[5] = livePosZ;
  stemLine.geometry.attributes.position.needsUpdate = true;

  controls.update();
  renderer.render(scene, camera);
}

function create3DFieldPlot(elementId, ringColor) {
  const el = document.getElementById(elementId);
  if (!el) return null;

  const plotScene = new THREE.Scene();
  plotScene.background = new THREE.Color(0x020612);

  const w = el.clientWidth || 320;
  const h = el.clientHeight || 260;
  const plotCamera = new THREE.PerspectiveCamera(40, w / h, 0.1, 100);
  plotCamera.position.set(9, 7, 10);

  const plotRenderer = new THREE.WebGLRenderer({ antialias: true });
  plotRenderer.setSize(w, h);
  el.appendChild(plotRenderer.domElement);

  const plotControls = new THREE.OrbitControls(plotCamera, plotRenderer.domElement);
  plotControls.enableDamping = true;
  plotControls.dampingFactor = 0.05;

  const gridHelper = new THREE.GridHelper(8, 8, 0x334155, 0x1e293b);
  gridHelper.position.y = -3;
  plotScene.add(gridHelper);

  plotScene.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, -3, 0), 4.2, 0xff3366));
  plotScene.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, -3, 0), 4.2, 0x00ff88));
  plotScene.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, -3, 0), 4.2, 0x00d2ff));

  const maxPoints = 120;
  const linePositions = new Float32Array(maxPoints * 3);
  const lineGeo = new THREE.BufferGeometry();
  lineGeo.setAttribute('position', new THREE.BufferAttribute(linePositions, 3));
  const lineMat = new THREE.LineBasicMaterial({ color: ringColor, linewidth: 2 });
  const lineMesh = new THREE.Line(lineGeo, lineMat);
  plotScene.add(lineMesh);

  // 6 mm-equivalent cylindrical magnet: red north cap, steel body, blue south cap.
  const magnet = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.CylinderGeometry(0.75, 0.75, 0.5, 32),
    new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.75, roughness: 0.22 })
  );
  const northCap = new THREE.Mesh(
    new THREE.CylinderGeometry(0.76, 0.76, 0.08, 32),
    new THREE.MeshStandardMaterial({ color: 0xff3366, metalness: 0.35, roughness: 0.25 })
  );
  const southCap = new THREE.Mesh(
    new THREE.CylinderGeometry(0.76, 0.76, 0.08, 32),
    new THREE.MeshStandardMaterial({ color: 0x0088ff, metalness: 0.35, roughness: 0.25 })
  );
  northCap.position.y = 0.29;
  southCap.position.y = -0.29;
  magnet.add(body, northCap, southCap);
  plotScene.add(magnet);

  plotScene.add(new THREE.AmbientLight(0xffffff, 0.9));
  const keyLight = new THREE.DirectionalLight(0xffffff, 1.4);
  keyLight.position.set(5, 8, 6);
  plotScene.add(keyLight);

  const velocityVector = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.01, 0x00d2ff, 0.32, 0.16);
  const forceVector = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 0.01, 0xffaa00, 0.32, 0.16);
  plotScene.add(velocityVector, forceVector);

  return {
    scene: plotScene, camera: plotCamera, renderer: plotRenderer, controls: plotControls,
    lineGeo, magnet, velocityVector, forceVector, maxPoints, el
  };
}

function update3DPlot(plot, history, isCorrupted) {
  const positions = plot.lineGeo.attributes.position.array;
  const len = history.length;

  for (let i = 0; i < plot.maxPoints; i++) {
    if (i < len) {
      positions[i * 3] = history[i].x * 0.18;
      positions[i * 3 + 1] = history[i].y * 0.18 - 3;
      positions[i * 3 + 2] = (history[i].z - 22.0) * 0.18;
    } else {
      positions[i * 3] = 0; positions[i * 3 + 1] = -3; positions[i * 3 + 2] = 0;
    }
  }
  plot.lineGeo.attributes.position.needsUpdate = true;

  if (len > 0) {
    const last = history[len - 1];
    const position = new THREE.Vector3(last.x * 0.18, last.y * 0.18 - 3, (last.z - 22.0) * 0.18);
    plot.magnet.position.copy(position);
    plot.magnet.rotation.y += 0.015;

    const velocity = new THREE.Vector3(motionTelemetry.x, motionTelemetry.y, motionTelemetry.z);
    if (velocity.lengthSq() > 0.0001) {
      plot.velocityVector.position.copy(position);
      plot.velocityVector.setDirection(velocity.normalize());
      plot.velocityVector.setLength(Math.min(2.8, Math.max(0.2, motionTelemetry.velocity / 160)), 0.32, 0.16);
    }
    const force = new THREE.Vector3(joyX, Math.max(0.15, joyZ + 0.2), joyY).normalize();
    plot.forceVector.position.copy(position);
    plot.forceVector.setDirection(force);
    plot.forceVector.setLength(Math.max(0.2, motionTelemetry.force / 55), 0.32, 0.16);

    const overlay = document.getElementById(isCorrupted ? 'plot-overlay-std' : 'plot-overlay-mlx');
    if (overlay) overlay.innerHTML = `v ${motionTelemetry.velocity.toFixed(1)} px/s<br>F ${motionTelemetry.force.toFixed(0)}%`;
  }

  plot.controls.update();
  plot.renderer.render(plot.scene, plot.camera);
}

export function resetPlotLines() {
  stdTraceHistory = [];
  mlxTraceHistory = [];
  trail4D = []; // Clear 4D history as well
  
  [stdPlot, mlxPlot].forEach(p => {
    if (!p) return;
    const pos = p.lineGeo.attributes.position.array;
    pos.fill(0);
    p.lineGeo.attributes.position.needsUpdate = true;
  });

  if (xyzChartObj) {
    const pos = xyzChartObj.lineGeo.attributes.position.array;
    pos.fill(0);
    xyzChartObj.lineGeo.attributes.position.needsUpdate = true;
    xyzChartObj.headMesh.position.set(0, 0, 0);
  }
}

// --- 4D Canvas Engine (Requirement 5) ---
function init4DCanvas() {
  let controlMode = 'position';
  let dronePos = { x: 0, y: 0 };
  const ARENA_LIMIT = 85;

  const btnPos = document.getElementById('btn-mode-pos');
  const btnVel = document.getElementById('btn-mode-vel');
  const modeDesc = document.getElementById('ctrl-mode-desc');

  btnPos?.addEventListener('click', () => {
    controlMode = 'position';
    btnPos.className = 'ds-button ds-button--primary ds-button--sm';
    btnVel.className = 'ds-button ds-button--secondary ds-button--sm';
    if (modeDesc) modeDesc.innerText = 'Mode: Direct Stick-to-Position Mapping (Absolute)';
  });

  btnVel?.addEventListener('click', () => {
    controlMode = 'velocity';
    btnVel.className = 'ds-button ds-button--primary ds-button--sm';
    btnPos.className = 'ds-button ds-button--secondary ds-button--sm';
    if (modeDesc) modeDesc.innerText = 'Mode: Stick Tilt Controls Speed & Heading (Rate Control)';
  });

  canvas4D = document.getElementById('simCanvas');
  if (!canvas4D) return;
  ctx4D = canvas4D.getContext('2d');

  resize4DCanvas();

  function project3D(x, y, z) {
    const centerX = width4D / 2;
    const centerY = height4D / 2 + 50;
    const isoX = (x - y) * Math.cos(Math.PI / 6);
    const isoY = (x + y) * Math.sin(Math.PI / 6) - z * 1.2;
    return { px: centerX + isoX * 2.2, py: centerY + isoY * 2.2 };
  }

  function get4DColor(velocity, alpha = 1) {
    const ratio = Math.min(velocity / 35, 1);
    const r = Math.floor(0 + ratio * 255);
    const g = Math.floor(243 - ratio * 188);
    const b = Math.floor(255 - ratio * 255);
    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
  }

  function update4DPhysicsAndRender() {
    prevStick4D.x = stick4D.x;
    prevStick4D.y = stick4D.y;
    prevStick4D.z = stick4D.z;

    stick4D.x += ((stick4D.targetX || 0) - stick4D.x) * 0.2;
    stick4D.y += ((stick4D.targetY || 0) - stick4D.y) * 0.2;
    stick4D.z += ((stick4D.targetZ || 0) - stick4D.z) * 0.2;

    if (controlMode === 'position') {
      dronePos.x = stick4D.x;
      dronePos.y = stick4D.y;
    } else {
      const speedScale = 0.04;
      dronePos.x += stick4D.x * speedScale;
      dronePos.y += stick4D.y * speedScale;

      dronePos.x = Math.max(-ARENA_LIMIT, Math.min(ARENA_LIMIT, dronePos.x));
      dronePos.y = Math.max(-ARENA_LIMIT, Math.min(ARENA_LIMIT, dronePos.y));
    }

    const dx = stick4D.x - prevStick4D.x;
    const dy = stick4D.y - prevStick4D.y;
    const dz = stick4D.z - prevStick4D.z;

// 1. Calculate Force & Velocity
    stick4D.velocity = Math.sqrt(dx * dx + dy * dy + dz * dz) * 10;
    stick4D.force = Math.min(100, Math.sqrt(stick4D.x ** 2 + stick4D.y ** 2) * 0.7 + stick4D.z * 0.8 + stick4D.velocity * 0.5);

    // 2. STOP DRAWING/RECORDING IF HIDDEN
    const is4DActive = document.getElementById('tab-4d')?.classList.contains('active');
    if (!is4DActive) {
      requestAnimationFrame(update4DPhysicsAndRender);
      return; 
    }

    // 3. ONLY push to trail if we didn't return early
    trail4D.push({
      x: dronePos.x,
      y: dronePos.y,
      z: stick4D.z,
      vel: stick4D.velocity
    });
    if (trail4D.length > 120) trail4D.shift();

    const valX = document.getElementById('val-x');
    const valY = document.getElementById('val-y');
    const valZ = document.getElementById('val-z');
    const valVel = document.getElementById('val-vel');
    const valForce = document.getElementById('val-force');
    const valFlux = document.getElementById('val-flux');

    if (valX) valX.innerText = (dronePos.x / 4).toFixed(2);
    if (valY) valY.innerText = (dronePos.y / 4).toFixed(2);
    if (valZ) valZ.innerText = (stick4D.z / 35).toFixed(2);
    if (valVel) valVel.innerText = stick4D.velocity.toFixed(1) + " px/s";
    if (valForce) valForce.innerText = Math.round(stick4D.force) + "%";
    if (valFlux) valFlux.innerText = (1 + (stick4D.z / 100) * 1.5).toFixed(2) + " mT";

    ctx4D.clearRect(0, 0, width4D, height4D);

    const gridBounds = 100;
    ctx4D.lineWidth = 1;
    ctx4D.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    for (let i = -gridBounds; i <= gridBounds; i += 20) {
      let p1 = project3D(i, -gridBounds, 0);
      let p2 = project3D(i, gridBounds, 0);
      ctx4D.beginPath(); ctx4D.moveTo(p1.px, p1.py); ctx4D.lineTo(p2.px, p2.py); ctx4D.stroke();

      let p3 = project3D(-gridBounds, i, 0);
      let p4 = project3D(gridBounds, i, 0);
      ctx4D.beginPath(); ctx4D.moveTo(p3.px, p3.py); ctx4D.lineTo(p4.px, p4.py); ctx4D.stroke();
    }

    const icPos = project3D(0, 0, -5);
    ctx4D.fillStyle = '#1e293b';
    ctx4D.strokeStyle = '#00f3ff';
    ctx4D.lineWidth = 2;
    ctx4D.beginPath();
    ctx4D.arc(icPos.px, icPos.py, 16, 0, Math.PI * 2);
    ctx4D.fill(); ctx4D.stroke();

    ctx4D.strokeStyle = `rgba(0, 243, 255, ${0.1 + (stick4D.z / 200)})`;
    ctx4D.lineWidth = 1;
    for (let r = 25; r <= 80; r += 15) {
      ctx4D.beginPath();
      ctx4D.ellipse(icPos.px, icPos.py, r * 1.5, r * 0.8, 0, 0, Math.PI * 2);
      ctx4D.stroke();
    }

    for (let i = 1; i < trail4D.length; i++) {
      let pt1 = project3D(trail4D[i - 1].x, trail4D[i - 1].y, trail4D[i - 1].z);
      let pt2 = project3D(trail4D[i].x, trail4D[i].y, trail4D[i].z);
      const speed = trail4D[i].vel;

      ctx4D.strokeStyle = get4DColor(speed, i / trail4D.length);
      ctx4D.lineWidth = 1 + (speed * 0.2);
      ctx4D.beginPath(); ctx4D.moveTo(pt1.px, pt1.py); ctx4D.lineTo(pt2.px, pt2.py); ctx4D.stroke();

      if (speed > 20 && Math.random() > 0.5) {
        ctx4D.fillStyle = get4DColor(speed, 0.8);
        ctx4D.beginPath();
        ctx4D.arc(pt2.px + (Math.random() - 0.5) * 10, pt2.py + (Math.random() - 0.5) * 10, Math.random() * 3, 0, Math.PI * 2);
        ctx4D.fill();
      }
    }

    const basePos = project3D(0, 0, 0);
    const tipPos = project3D(dronePos.x, dronePos.y, stick4D.z);
    const shadowPos = project3D(dronePos.x, dronePos.y, 0);

    ctx4D.strokeStyle = 'rgba(255, 255, 255, 0.25)';
    ctx4D.setLineDash([3, 3]);
    ctx4D.beginPath(); ctx4D.moveTo(tipPos.px, tipPos.py); ctx4D.lineTo(shadowPos.px, shadowPos.py); ctx4D.stroke();
    ctx4D.setLineDash([]);

    ctx4D.strokeStyle = controlMode === 'velocity' ? '#00f3ff' : '#ffffff';
    ctx4D.lineWidth = 4;
    ctx4D.beginPath(); ctx4D.moveTo(basePos.px, basePos.py); ctx4D.lineTo(tipPos.px, tipPos.py); ctx4D.stroke();

    const knobColor = get4DColor(stick4D.velocity);
    ctx4D.fillStyle = knobColor;
    ctx4D.shadowColor = knobColor;
    ctx4D.shadowBlur = 15;
    ctx4D.beginPath(); ctx4D.arc(tipPos.px, tipPos.py, 12, 0, Math.PI * 2); ctx4D.fill();
    ctx4D.shadowBlur = 0;

    requestAnimationFrame(update4DPhysicsAndRender);
  }

  requestAnimationFrame(update4DPhysicsAndRender);
}

function resize4DCanvas() {
  const container = document.getElementById('canvas4d-container');
  if (!container || !canvas4D) return;
  width4D = canvas4D.width = container.clientWidth;
  height4D = canvas4D.height = container.clientHeight;
}

export function resizeSfiCanvases() {
  if (mainSceneObj && mainSceneObj.container) {
    const w = mainSceneObj.container.clientWidth;
    const h = mainSceneObj.container.clientHeight;
    if (w > 0 && h > 0) {
      mainSceneObj.camera.aspect = w / h;
      mainSceneObj.camera.updateProjectionMatrix();
      mainSceneObj.renderer.setSize(w, h);
    }
  }

  if (xyzChartObj && xyzChartObj.container) {
    const w = xyzChartObj.container.clientWidth;
    const h = xyzChartObj.container.clientHeight;
    if (w > 0 && h > 0) {
      xyzChartObj.camera.aspect = w / h;
      xyzChartObj.camera.updateProjectionMatrix();
      xyzChartObj.renderer.setSize(w, h);
    }
  }

  resize4DCanvas();
  resizeRadarCanvases();

  [stdPlot, mlxPlot].forEach(p => {
    if (p && p.el) {
      const w = p.el.clientWidth;
      const h = p.el.clientHeight;
      if (w > 0 && h > 0) {
        p.camera.aspect = w / h;
        p.camera.updateProjectionMatrix();
        p.renderer.setSize(w, h);
      }
    }
  });
}

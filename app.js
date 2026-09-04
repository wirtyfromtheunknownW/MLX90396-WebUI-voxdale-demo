import { MLX90396_API } from './mlx_api.js';
import { Arduino_API } from './arduino_api.js';
import { initSfiDemo, resizeSfiCanvases, updateSfiDomeKinematics, setHardwareCoilCallback, resetPlotLines } from './sfi_demo.js';

// DOM Elements
const btnConnect = document.getElementById('btn-connect');
const btnStartDemo = document.getElementById('btn-start-demo');
const btnStopDemo = document.getElementById('btn-stop-demo');
const btnToggleDebug = document.getElementById('btn-toggle-debug');
const miniLogWindow = document.getElementById('mini-log-window');
const statusModeTxt = document.getElementById('status-mode-txt');

// Modal Elements
const connectModal = document.getElementById('connect-modal');
const selDriverType = document.getElementById('sel-driver-type');
const wrapConnType = document.getElementById('wrap-conn-type');
const selConnType = document.getElementById('sel-conn-type');
const selBaudRate = document.getElementById('sel-baud-rate');
const btnModalConnect = document.getElementById('btn-modal-connect');
const btnCloseModal = document.getElementById('btn-close-modal');

// State Variables
let port = null, reader = null, writer = null;
let isConnected = false, isReading = false, isDemoRunning = false;
let textDecoder, textEncoder;
let readableStreamClosed, writableStreamClosed;

let activeDevice = null;
let currentDriverType = 'scpi';

let scpiLock = Promise.resolve();
let scpiWaiter = null;
let rxBuffer = '';
let lastDataLine = '';

const magnetOffsets = { x: 0, y: 0 };
const latestRawMagnet = { x: 0, y: 0 };

window.addEventListener('DOMContentLoaded', () => {
  initSfiDemo();
  initTabNavigation();
  setHardwareCoilCallback(toggleHardwareCoilPin);
  window.addEventListener('resize', resizeSfiCanvases);
});

function initTabNavigation() {
  const tabButtons = document.querySelectorAll('.view-tab-btn');
  const tabContents = document.querySelectorAll('.tab-content');

  tabButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      tabButtons.forEach(b => b.classList.remove('active'));
      tabContents.forEach(c => c.classList.remove('active'));

      btn.classList.add('active');
      const targetId = btn.dataset.tab;
      const targetContent = document.getElementById(targetId);
      if (targetContent) {
        targetContent.classList.add('active');
      }

      resetPlotLines(); // <--- Clears all trajectories instantly on tab switch
      setTimeout(resizeSfiCanvases, 40);
    });
  });
}

function getSpiPrefix() {
  return selConnType ? selConnType.value : ':SPI';
}

function appendLog(text, cls = 'log-rx') {
  if (!text || !miniLogWindow) return;
  
  // Stop processing debug logs if the window is hidden (Massive Lag Fix)
  if (miniLogWindow.classList.contains('hidden')) return;

  const span = document.createElement('span');
  span.className = cls;
  span.textContent = text;
  miniLogWindow.appendChild(span);
  
  // Keep only the last 100 lines so the browser doesn't freeze
  while (miniLogWindow.childNodes.length > 100) {
    miniLogWindow.removeChild(miniLogWindow.firstChild);
  }
  miniLogWindow.scrollTop = miniLogWindow.scrollHeight;
}

function processRx(text) {
  for (let i = 0; i < text.length; i++) {
    const char = text[i];
    if (char === '\n' || char === '\r') {
      if (rxBuffer.trim() !== '') {
        const line = rxBuffer.trim();
        if (currentDriverType === 'arduino' && activeDevice) {
          activeDevice.processLine(line);
        } else {
          handlePrompt(line);
        }
      }
      rxBuffer = '';
    } else {
      rxBuffer += char;
      if (currentDriverType === 'scpi') {
        if (rxBuffer.endsWith('(OK)>') || rxBuffer.endsWith('(ERR)>') || rxBuffer.endsWith('(E2BIG)>') || rxBuffer.endsWith('(ERANGE)>')) {
          handlePrompt(rxBuffer.trim());
          rxBuffer = '';
        }
      }
    }
  }
}

function handlePrompt(line) {
  if (line.endsWith('(OK)>')) {
    let dataPart = line.slice(0, -5).trim();
    if (dataPart.includes(':SPI') || dataPart.includes(',') || dataPart.includes('0x')) {
      lastDataLine = dataPart;
    }
    if (dataPart) appendLog(dataPart + '\n', 'log-rx');
    appendLog('(OK)>\n', 'log-okprompt');

    if (scpiWaiter) {
      clearTimeout(scpiWaiter.timeoutId);
      scpiWaiter.resolve(lastDataLine);
      const rel = scpiWaiter.release;
      scpiWaiter = null;
      if (rel) rel();
    }
    lastDataLine = '';
  } else if (line.endsWith('(ERR)>') || line.endsWith('(E2BIG)>') || line.endsWith('(ERANGE)>')) {
    appendLog(line + '\n', 'log-badprompt');
    if (scpiWaiter) {
      clearTimeout(scpiWaiter.timeoutId);
      scpiWaiter.reject(new Error("Hardware Error: " + line));
      const rel = scpiWaiter.release;
      scpiWaiter = null;
      if (rel) rel();
    }
    lastDataLine = '';
  } else {
    appendLog(line + '\n', 'log-rx');
    if (line.includes(',') && !line.startsWith(':') && !line.startsWith('*')) {
      lastDataLine = line;
    }
  }
}

async function sendCommand(cmd) {
  if (!writer) return;
  const raw = cmd.trim();
  if (!raw) return;
  try {
    await writer.write(raw + '\n');
  } catch (err) {
    appendLog(`[TX ERROR]: ${err.message}\n`, 'log-badprompt');
  }
}

async function scpiQuery(cmd) {
  let releaseLock;
  const nextLock = new Promise(resolve => { releaseLock = resolve; });

  await scpiLock;
  scpiLock = nextLock;

  return new Promise((resolve, reject) => {
    lastDataLine = '';
    scpiWaiter = { resolve, reject, release: releaseLock };
    sendCommand(cmd);

    scpiWaiter.timeoutId = setTimeout(() => {
      if (scpiWaiter) {
        scpiWaiter.reject(new Error("Device Timeout on command: " + cmd));
        scpiWaiter = null;
        releaseLock();
      }
    }, 2500);
  });
}

// Requirement 4: Physical Hardware Coil Pin SCPI Trigger
async function toggleHardwareCoilPin(state) {
  if (!isConnected || currentDriverType !== 'scpi') return;
  try {
    const pinCmd = state ? ':A3:GPIO:SET:OUT 1' : ':A3:GPIO:SET:OUT 0';
    await scpiQuery(pinCmd);
    appendLog(`[HARDWARE COIL] Pin set to: ${state ? 'HIGH (5mT ON)' : 'LOW (OFF)'}\n`, 'log-okprompt');
  } catch (err) {
    console.warn('[COIL SCPI ERROR]', err);
  }
}

function setUIConnected(connected) {
  isConnected = connected;
  btnConnect.textContent = connected ? 'Disconnect' : 'Connect USB';
  btnConnect.className = connected ? 'ds-button ds-button--danger ds-button--md' : 'ds-button ds-button--primary ds-button--md';
  if (statusModeTxt) {
    statusModeTxt.textContent = connected 
      ? `Connected via ${currentDriverType.toUpperCase()} at ${selBaudRate.value} baud` 
      : 'Standby Mode (Disconnected)';
  }
}

async function connectSerial() {
  try {
    currentDriverType = selDriverType ? selDriverType.value : 'scpi';
    const baudRate = selBaudRate ? parseInt(selBaudRate.value, 10) : 115200;

    port = await navigator.serial.requestPort();
    await port.open({ baudRate });

    textDecoder = new TextDecoderStream();
    textEncoder = new TextEncoderStream();
    readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
    writableStreamClosed = textEncoder.readable.pipeTo(port.writable);

    reader = textDecoder.readable.getReader();
    writer = textEncoder.writable.getWriter();
    isReading = true;

    if (currentDriverType === 'arduino') {
      activeDevice = new Arduino_API();
      appendLog(`[SYSTEM] Direct Arduino telemetry connected at ${baudRate} baud.\n`, 'log-okprompt');
    } else {
      activeDevice = new MLX90396_API(scpiQuery, getSpiPrefix);
      appendLog(`[SYSTEM] MLX90396 SCPI API connected on ${getSpiPrefix()} at ${baudRate} baud.\n`, 'log-okprompt');
    }

    setUIConnected(true);

    (async () => {
      try {
        while (isReading) {
          const { value, done } = await reader.read();
          if (done) break;
          if (value) processRx(value);
        }
      } catch (err) {
        appendLog(`[RX ERROR]: ${err.message}\n`, 'log-badprompt');
      } finally {
        reader.releaseLock();
      }
    })();

  } catch (err) {
    appendLog(`[CONN ERROR]: ${err.message}\n`, 'log-badprompt');
  }
}

async function disconnectSerial() {
  isReading = false;
  isDemoRunning = false;
  if (reader) { await reader.cancel(); reader.releaseLock(); }
  if (writer) { await writer.close(); writer.releaseLock(); }
  if (readableStreamClosed) await readableStreamClosed.catch(() => {});
  if (writableStreamClosed) await writableStreamClosed.catch(() => {});
  if (port) await port.close();

  port = null; reader = null; writer = null; activeDevice = null;
  setUIConnected(false);
  btnStartDemo.disabled = false;
  btnStopDemo.disabled = true;
  appendLog('[SYSTEM] Port closed.\n', 'log-badprompt');
}

// --- EXACT UNCHANGED ORIGINAL LIVE STREAMING LOOP ---
async function runJoystickDemo() {
  btnStartDemo.disabled = true;
  btnStopDemo.disabled = false;
  isDemoRunning = true;

  if (isConnected && activeDevice) {
    try {
      appendLog('[SYSTEM] Starting Live Demo...\n', 'log-okprompt');

      if (currentDriverType === 'scpi') {
        const prefix = getSpiPrefix(); 
        await new Promise(r => setTimeout(r, 200));

        if (prefix.includes('SPI2') || prefix === ':SPI2') {
          console.log('[INIT] Configuring Melexis IO SPI2 Header Connection...');
          await scpiQuery(`${prefix}:Init 0`);
          await scpiQuery(`${prefix}:SET:CS0 0`);
          await scpiQuery(":A3:GPIO:INIT:OUT 0");
        } else {
          console.log('[INIT] Configuring Melexis IO Cable Connection...');
          await scpiQuery(`${prefix}:Init 0`);
          await scpiQuery(":CON:CS1:GPIO:INIT:OUT 0");
          await scpiQuery(`${prefix}:BUFfer 1,1,1,1,0`);
          await scpiQuery(":VDD:3V3");
        }
      }

      const K_ROT_X = 0.10, K_ROT_Y = 0.10;
      const SCALE_X = 0.02, SCALE_Y = 0.02;

      while (isDemoRunning) {
        try {
          let posX_mm = 0, posY_mm = 0, angleDeg = 0;
          let rawX = 0, rawY = 0, rawZ = 0;
          let p0Raw = { x: 0, y: 0, z: 0 };
          let diffRaw = { x: 0, y: 0, z: 0 };

          if (currentDriverType === 'scpi') {
            await activeDevice.sm(0xFC000); 
            await new Promise(r => setTimeout(r, 15)); 
            const res01 = await activeDevice.rm(false, 0xFC000); 
            
            await activeDevice.sm(0x03F00); 
            await new Promise(r => setTimeout(r, 15)); 
            const res23 = await activeDevice.rm(false, 0x03F00); 

            if (!res01.error && !res23.error) {
              const avgX = (res01.x0 + res01.x1 + res23.x2 + res23.x3) / 4;
              const avgY = (res01.y0 + res01.y1 + res23.y2 + res23.y3) / 4;
              const avgZ = (res01.z0 + res01.z1 + res23.z2 + res23.z3) / 4;

              angleDeg = Math.atan2(-avgY, avgX) * (180 / Math.PI);

              const rawGradX = ((res01.x1 + res23.x2) - (res01.x0 + res23.x3)) / 2;
              const rawGradY = ((res01.y0 + res01.y1) - (res23.y3 + res23.y2)) / 2;

              const cleanGradX = rawGradX - (avgX * K_ROT_X);
              const cleanGradY = rawGradY - (avgY * K_ROT_Y);

              latestRawMagnet.x = cleanGradX * SCALE_X;
              latestRawMagnet.y = cleanGradY * SCALE_Y;

              posX_mm = latestRawMagnet.x - magnetOffsets.x;
              posY_mm = latestRawMagnet.y - magnetOffsets.y;
              rawX = avgX; rawY = avgY; rawZ = avgZ;

              // Extract actual individual pixel values for Requirement 1-3 Battle Matrix
              p0Raw = { x: res01.x0, y: res01.y0, z: res01.z0 };
              diffRaw = { x: cleanGradX, y: cleanGradY, z: avgZ };
            }
          } else {
            const sample = await activeDevice.getSample();
            if (!sample.error) {
              latestRawMagnet.x = sample.posX_mm;
              latestRawMagnet.y = sample.posY_mm;
              posX_mm = latestRawMagnet.x - magnetOffsets.x;
              posY_mm = latestRawMagnet.y - magnetOffsets.y;
              angleDeg = sample.angleDeg;
              rawX = sample.rawX; rawY = sample.rawY; rawZ = sample.rawZ;
              p0Raw = { x: rawX, y: rawY, z: rawZ };
              diffRaw = { x: rawX, y: rawY, z: rawZ };
            }
          }

          // Exact original live telemetry fed to the Dome Kinematics
          updateSfiDomeKinematics(rawX / 1000, rawY / 1000, rawZ / 1000, { p0Raw, diffRaw });

          await new Promise(r => setTimeout(r, currentDriverType === 'scpi' ? 50 : 25));
        } catch (loopErr) {
          console.warn('[DEMO LOOP WARNING]', loopErr);
          await new Promise(r => setTimeout(r, 150));
        }
      }
    } catch (err) {
      appendLog(`[DEMO ERROR]: ${err.message}\n`, 'log-badprompt');
      isDemoRunning = false;
      btnStartDemo.disabled = false;
      btnStopDemo.disabled = true;
    }
  } else {
    // Standalone Emulation Mode
    let angle = 0;
    while (isDemoRunning) {
      angle += 0.08; 
      const emulatedX = Math.sin(angle * 0.7) * 1.8;
      const emulatedY = Math.cos(angle * 0.9) * 1.8;

      updateSfiDomeKinematics(emulatedX / 2, emulatedY / 2, 0);

      await new Promise(r => setTimeout(r, 33)); 
    }
  }
}

// Connection Modal Event Handlers
selDriverType?.addEventListener('change', (e) => {
  if (wrapConnType) {
    wrapConnType.style.display = e.target.value === 'scpi' ? 'block' : 'none';
  }
});

btnConnect?.addEventListener('click', () => {
  if (isConnected) disconnectSerial();
  else connectModal?.classList.remove('hidden');
});

btnCloseModal?.addEventListener('click', () => connectModal?.classList.add('hidden'));

btnModalConnect?.addEventListener('click', async () => {
  connectModal?.classList.add('hidden');
  await connectSerial();
});

btnStartDemo?.addEventListener('click', runJoystickDemo);
btnStopDemo?.addEventListener('click', () => {
  isDemoRunning = false;
  btnStartDemo.disabled = false;
  btnStopDemo.disabled = true;
  appendLog('[SYSTEM] Demo stopped.\n', 'log-badprompt');
});

btnToggleDebug?.addEventListener('click', () => {
  miniLogWindow?.classList.toggle('hidden');
});
import {
  createRealtimeConnection,
  connectBrowserBleDevice,
} from '../gateway/realtimeTransport.js';
import {
  normalizeRealtimeSensorSample,
  getActiveAxisMap,
} from '../gateway/realtimeSensorUtils.js';
import { rawAccelToG, rawGyroToDps } from '../gait/signalUtils.js';
import { readGaitCalibrationProfiles } from '../gait/gaitCalibrationStore.js';
import { generateWalkingData } from '../gait-dashboard/data/demoDataGenerator.js';
import {
  TraceRecorder,
  downloadTraceJson,
  buildTraceFilename,
} from '../gait-dashboard/data/traceRecorder.js';

const GYRO_AXES = ['gx', 'gy', 'gz'];
const ACCEL_AXES = ['ax', 'ay', 'az'];
const SHANK_SLOTS = ['LEFT_SHANK', 'RIGHT_SHANK'];
const RENDER_INTERVAL_MS = 100;

// ---- Pure helpers (unit-tested) ----

// ท่าที่ 1: แกน gyro ที่ peak |ค่า| สูงสุดตอนแกว่งขา = แกน sagittal (ต้องเป็น gx)
export function detectSagittalAxis(peakAbs = {}) {
  let axis = 'gx';
  let value = -Infinity;
  for (const candidate of GYRO_AXES) {
    const magnitude = Number.isFinite(peakAbs[candidate]) ? peakAbs[candidate] : 0;
    if (magnitude > value) {
      value = magnitude;
      axis = candidate;
    }
  }
  return { axis, value: value === -Infinity ? 0 : value, isCorrect: axis === 'gx' };
}

// ท่าที่ 0 (ยืนนิ่ง หน้าแข้งตั้งตรง): gravity ต้องอยู่ที่ ay ≈ −1.00g (แกนตามยาว)
// ถ้า az อ่านได้ ~1g แทน = แกน accel สลับ → accelToAngle/sensorToWorld ผิดทันที
export function detectGravityAxis(values = {}) {
  let axis = 'ay';
  let magnitude = -Infinity;
  for (const candidate of ACCEL_AXES) {
    const abs = Math.abs(values[candidate] ?? 0);
    if (abs > magnitude) {
      magnitude = abs;
      axis = candidate;
    }
  }
  const value = values[axis] ?? 0;
  const nearOneG = magnitude >= 0.85 && magnitude <= 1.15;

  if (!nearOneG) {
    return { axis, value, state: 'pending', message: 'ยืนนิ่งให้หน้าแข้งตั้งตรง (รอ gravity นิ่งที่ ~1g)' };
  }
  if (axis !== 'ay') {
    return { axis, value, state: 'bad', message: `แกน accel สลับ: gravity อยู่ที่ ${axis} (ควรเป็น ay) → ต้อง remap` };
  }
  if (value > 0) {
    return { axis, value, state: 'bad', message: 'ay = +1g (กลับหัว/sign) → ต้อง flip' };
  }
  return { axis, value, state: 'ok', message: `ay ≈ ${value.toFixed(2)}g ✅` };
}

// |ค่า| peak ต่อแกนจาก peak บวก/ลบ (ใช้หาแกน sagittal)
export function peakAbsFrom(peakPos = {}, peakNeg = {}) {
  const out = {};
  for (const axis of [...ACCEL_AXES, ...GYRO_AXES]) {
    out[axis] = Math.max(Math.abs(peakPos[axis] || 0), Math.abs(peakNeg[axis] || 0));
  }
  return out;
}

function emptyAxisMap(value = 0) {
  return { ax: value, ay: value, az: value, gx: value, gy: value, gz: value };
}

function emptyTracker(sensorKey, side, label) {
  return {
    sensorKey,
    side: side || null,
    label: label || sensorKey,
    latest: emptyAxisMap(),
    // เก็บ peak บวกและลบ "แยกกัน" ต่อแกน — การแกว่งมีทั้งจังหวะไปหน้า(+) และดีดกลับ(−)
    // การเก็บ signed peak รวมเดียวจะจับคนละจังหวะกันแต่ละข้าง = false mirror alarm
    peakPos: emptyAxisMap(0),
    peakNeg: emptyAxisMap(0),
    usingPreRemap: false,
    sampleCount: 0,
  };
}

// แปลง sample (normalized) → ค่าฟิสิกส์ 6 แกนจาก raw "ก่อน remap" (fallback canonical ถ้าไม่มี)
export function extractAxes(sample) {
  const preAccel = Array.isArray(sample.rawAccelSensor) ? sample.rawAccelSensor : null;
  const preGyro = Array.isArray(sample.rawGyroSensor) ? sample.rawGyroSensor : null;
  const usingPreRemap = Boolean(preAccel && preGyro);

  const accelRaw = preAccel || [sample.ax, sample.ay, sample.az];
  const gyroRaw = preGyro || [sample.gx, sample.gy, sample.gz];

  return {
    usingPreRemap,
    values: {
      ax: rawAccelToG(accelRaw[0]),
      ay: rawAccelToG(accelRaw[1]),
      az: rawAccelToG(accelRaw[2]),
      gx: rawGyroToDps(gyroRaw[0]),
      gy: rawGyroToDps(gyroRaw[1]),
      gz: rawGyroToDps(gyroRaw[2]),
    },
  };
}

export function updateTracker(tracker, sample) {
  const { usingPreRemap, values } = extractAxes(sample);
  tracker.usingPreRemap = usingPreRemap;
  tracker.latest = values;
  tracker.sampleCount += 1;

  for (const axis of [...ACCEL_AXES, ...GYRO_AXES]) {
    const value = values[axis];
    if (value > tracker.peakPos[axis]) {
      tracker.peakPos[axis] = value;
    }
    if (value < tracker.peakNeg[axis]) {
      tracker.peakNeg[axis] = value;
    }
  }
  return tracker;
}

export class DebugApp {
  constructor() {
    this.trackers = new Map();
    this.ws = null;
    this.demoStream = null;
    this.renderTimer = null;
    this.traceRecorder = new TraceRecorder({ sampleRateHzNominal: 100 });
    this.connectedCount = 0;
    this.els = {};
  }

  init() {
    this.els = {
      status: document.getElementById('debug-status'),
      warning: document.getElementById('debug-warning'),
      sensors: document.getElementById('debug-sensors'),
      verdict: document.getElementById('debug-verdict'),
      record: document.getElementById('debug-record'),
    };

    document.getElementById('debug-connect')?.addEventListener('click', () => this.connect());
    document.getElementById('debug-demo')?.addEventListener('click', () => this.startDemo());
    document.getElementById('debug-reset')?.addEventListener('click', () => this.resetPeaks());
    document.getElementById('debug-record')?.addEventListener('click', () => this.toggleRecording());
    document.getElementById('debug-export')?.addEventListener('click', () => this.exportTrace());

    this._openStream();
    this.renderTimer = window.setInterval(() => this.render(), RENDER_INTERVAL_MS);
    this._setStatus('Ready. Connect sensors or use Demo.');
  }

  _openStream() {
    this.ws = createRealtimeConnection();
    this.ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        if (message.type === 'DATA') {
          this._handleSample(message.payload);
        }
      } catch {
        // ignore malformed frames
      }
    };
  }

  async connect() {
    this._setStatus('Pairing shank sensors...');
    let connected = 0;
    for (const slot of SHANK_SLOTS) {
      try {
        await connectBrowserBleDevice(slot, { preferKnownDevice: true, pickerFallback: true });
        connected += 1;
      } catch (error) {
        console.warn(`Connect ${slot} failed:`, error);
      }
    }
    this.connectedCount = connected;
    this._setStatus(connected ? `Connected ${connected} sensor(s). Start swinging.` : 'No sensor connected.');
  }

  startDemo() {
    this._stopDemo();
    // ป้อนข้อมูลจำลอง gx เด่น ให้ทั้งสองข้าง (sign เดียวกัน = เคส ✅) เพื่อดู UI ก่อนต่อจริง
    const { samples } = generateWalkingData({ numStrides: 200 });
    let index = 0;
    this.demoStream = window.setInterval(() => {
      const base = samples[index % samples.length];
      index += 1;
      for (const side of ['L', 'R']) {
        this._handleSample({
          name: `Demo_${side}_Shank`,
          sensor_key: `demo-${side}`,
          side,
          sensor_mount: 'shank',
          timestamp_ms: Date.now(),
          raw_accel: [base.ax, base.ay, base.az],
          raw_gyro: [base.gx, base.gy, base.gz],
          seq: index,
        });
      }
    }, 10);
    this._setStatus('Demo running (both sides, gx dominant, same sign).');
  }

  _stopDemo() {
    if (this.demoStream) {
      window.clearInterval(this.demoStream);
      this.demoStream = null;
    }
  }

  _handleSample(payload) {
    const sample = normalizeRealtimeSensorSample(payload);
    if (!sample) {
      return;
    }

    if (this.traceRecorder.isRecording()) {
      this.traceRecorder.record(sample);
    }

    const key = sample.side || sample.sensorKey;
    const tracker = this.trackers.get(key)
      || emptyTracker(sample.sensorKey, sample.side, sample.side === 'L' ? 'Left Shank' : sample.side === 'R' ? 'Right Shank' : sample.sensorKey);
    tracker.sensorKey = sample.sensorKey;
    tracker.side = sample.side || tracker.side;
    updateTracker(tracker, sample);
    this.trackers.set(key, tracker);
  }

  resetPeaks() {
    for (const tracker of this.trackers.values()) {
      tracker.peakPos = emptyAxisMap(0);
      tracker.peakNeg = emptyAxisMap(0);
    }
    this._setStatus('Peaks reset. Swing forward and read the live sign on both sides.');
  }

  toggleRecording() {
    if (this.traceRecorder.isRecording()) {
      const count = this.traceRecorder.stop();
      if (this.els.record) {
        this.els.record.textContent = '● Record';
        this.els.record.classList.remove('recording');
      }
      this._setStatus(`Recording stopped (${count} samples). Export to save.`);
    } else {
      this.traceRecorder.start();
      if (this.els.record) {
        this.els.record.textContent = '■ Stop';
        this.els.record.classList.add('recording');
      }
      this._setStatus('Recording raw trace...');
    }
  }

  exportTrace() {
    if (this.traceRecorder.isRecording()) {
      this.toggleRecording();
    }
    if (!this.traceRecorder.sampleCount()) {
      window.alert?.('No trace recorded yet.');
      return;
    }
    // ใส่ header ให้ครบเหมือน export หลัก (axis map + gyro bias) เผื่อ reprocess
    // แต่ mark ว่าเป็น swing-test ไม่ใช่ trace เดิน 10 เมตร (ไม่มี groundTruth ระยะจริง)
    const profiles = readGaitCalibrationProfiles();
    const trace = this.traceRecorder.buildTrace({
      axisMap: getActiveAxisMap(),
      calibrationBySensor: profiles.bySensorKey || {},
      appVersion: '1.0.0-debug',
      groundTruth: { notes: 'swing-test / axis-check — NOT a 10m validation walk' },
    });
    downloadTraceJson(trace, buildTraceFilename());
  }

  render() {
    if (!this.els.sensors) {
      return;
    }

    this._renderConnectionWarning();

    const trackers = Array.from(this.trackers.values())
      .sort((a, b) => (a.side === 'L' ? 0 : 1) - (b.side === 'L' ? 0 : 1));

    if (!trackers.length) {
      this.els.sensors.innerHTML = '<div class="debug-empty">Waiting for sensor data…</div>';
      this.els.verdict.innerHTML = '';
      return;
    }

    this.els.sensors.innerHTML = trackers.map((tracker) => this._renderSensor(tracker)).join('');
    this.els.verdict.innerHTML = this._renderVerdict(trackers);
  }

  _renderSensor(tracker) {
    const peakAbs = peakAbsFrom(tracker.peakPos, tracker.peakNeg);
    const sagittal = detectSagittalAxis(peakAbs);
    const gravity = detectGravityAxis(tracker.latest);
    const row = (axis, unit) => {
      const value = tracker.latest[axis];
      const isSagittal = axis === sagittal.axis && GYRO_AXES.includes(axis);
      const sign = value > 0 ? 'pos' : value < 0 ? 'neg' : 'zero';
      return `<tr class="${isSagittal ? 'sagittal' : ''}">
        <td class="axis">${axis}${isSagittal ? ' ★' : ''}</td>
        <td class="val ${sign}">${value.toFixed(1)}</td>
        <td class="peak pos">+${tracker.peakPos[axis].toFixed(1)}</td>
        <td class="peak neg">${tracker.peakNeg[axis].toFixed(1)}</td>
        <td class="unit">${unit}</td>
      </tr>`;
    };

    const frameTag = tracker.usingPreRemap
      ? '<span class="tag ok">pre-remap raw</span>'
      : '<span class="tag warn">canonical (demo/no sensor-frame)</span>';

    return `<div class="debug-card">
      <div class="debug-card-head">
        <span class="side-badge ${tracker.side === 'L' ? 'left' : 'right'}">${tracker.side || '?'}</span>
        <span class="debug-label">${tracker.label}</span>
        ${frameTag}
        <span class="samples">${tracker.sampleCount} samples</span>
      </div>
      <div class="sagittal-verdict ${gravity.state}">
        ท่า 0 (ยืนนิ่ง) accel gravity → ${gravity.message}
      </div>
      <div class="sagittal-verdict ${sagittal.isCorrect ? 'ok' : 'bad'}">
        ท่า 1 sagittal axis (peak gyro) = <b>${sagittal.axis}</b> ${sagittal.isCorrect ? '✅ (gx ถูกต้อง)' : '❌ ต้อง remap (ควรเป็น gx)'}
        &nbsp;— peak ${sagittal.value.toFixed(0)}°/s
      </div>
      <table class="debug-table">
        <thead><tr><th>axis</th><th>live</th><th>peak+</th><th>peak−</th><th></th></tr></thead>
        <tbody>
          ${ACCEL_AXES.map((a) => row(a, 'g')).join('')}
          ${GYRO_AXES.map((a) => row(a, '°/s')).join('')}
        </tbody>
      </table>
    </div>`;
  }

  _renderVerdict(trackers) {
    const left = trackers.find((t) => t.side === 'L');
    const right = trackers.find((t) => t.side === 'R');
    if (!left || !right) {
      return '<div class="verdict-note">ต่อทั้งสองข้าง (L และ R) เพื่อเช็ค polarity (ท่าที่ 2)</div>';
    }

    // ไม่ auto-ตัดสิน mirror จาก peak เดียว (peak มีทั้ง +/− ทุกข้าง แยก mirror ไม่ได้จริง)
    // โชว์ peak+ / peak− และ live ของแกน sagittal ให้คนอ่าน "ตอนแกว่งไปหน้า" เทียบ sign เอง
    const row = (tracker) => {
      const sag = detectSagittalAxis(peakAbsFrom(tracker.peakPos, tracker.peakNeg));
      const axis = sag.axis;
      const live = tracker.latest[axis];
      const liveSign = live > 0 ? 'pos' : live < 0 ? 'neg' : 'zero';
      return `<tr>
        <td><span class="side-badge sm ${tracker.side === 'L' ? 'left' : 'right'}">${tracker.side}</span></td>
        <td class="axis">${axis}</td>
        <td class="val ${liveSign}">${live.toFixed(0)}</td>
        <td class="peak pos">+${tracker.peakPos[axis].toFixed(0)}</td>
        <td class="peak neg">${tracker.peakNeg[axis].toFixed(0)}</td>
      </tr>`;
    };

    return `<div class="verdict pending">
      <div class="verdict-title">ท่าที่ 2 — Polarity check (อ่านเอง ไม่ auto-ตัดสิน)</div>
      <table class="debug-table verdict-table">
        <thead><tr><th>side</th><th>sagittal</th><th>live °/s</th><th>peak+</th><th>peak−</th></tr></thead>
        <tbody>${row(left)}${row(right)}</tbody>
      </table>
      <div class="verdict-note">
        แกว่งขา<b>ไปข้างหน้า</b>ทั้งสองข้าง แล้วดู <b>live</b> ของแกน sagittal:
        ถ้าติดตั้งเหมือนกัน ต้องได้<b>เครื่องหมายเดียวกัน</b>. ถ้าตรงข้าม = mirror ต้อง flip ข้างหนึ่ง.
        <br>(peak+ และ peak− มีทั้งคู่ทุกข้างเป็นเรื่องปกติ — มีทั้งจังหวะไปหน้าและดีดกลับ อย่าตัดสินจาก peak อย่างเดียว)
      </div>
    </div>`;
  }

  // เตือนเมื่อต่อ BLE สำเร็จหลายตัวแต่เห็น tracker น้อยกว่า = ชื่อ BLE ซ้ำ (side/key ชนกัน)
  // ทำให้เซนเซอร์ตัวที่สองถูกกลืนรวมเป็นตัวเดียว — ดูเหมือน "ต่อไม่ติด" ทั้งที่ต่อติด
  _renderConnectionWarning() {
    if (!this.els.warning) {
      return;
    }
    const distinct = this.trackers.size;
    if (this.connectedCount >= 2 && distinct < this.connectedCount) {
      this.els.warning.innerHTML = `⚠️ ต่อ BLE สำเร็จ ${this.connectedCount} ตัว แต่เห็น ${distinct} side/tracker`
        + ' — ชื่อ BLE อาจซ้ำกัน ทำให้ตัวที่สองถูกกลืนรวม (แก้ SENSOR_ID ให้ต่างกัน:'
        + ' DernDee_L_Shank / DernDee_R_Shank)';
      this.els.warning.style.display = 'block';
    } else {
      this.els.warning.style.display = 'none';
    }
  }

  _setStatus(text) {
    if (this.els.status) {
      this.els.status.textContent = text;
    }
  }

  destroy() {
    this._stopDemo();
    if (this.renderTimer) {
      window.clearInterval(this.renderTimer);
    }
    if (this.ws?.close) {
      this.ws.close();
    }
  }
}

import {
  createRealtimeConnection,
  connectBrowserBleDevice,
} from '../gateway/realtimeTransport.js';
import { normalizeRealtimeSensorSample } from '../gateway/realtimeSensorUtils.js';
import { rawAccelToG, rawGyroToDps } from '../gait/signalUtils.js';
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

// ท่าที่ 2: ตอนแกว่งไปหน้า sign ของแกน sagittal ต้องเหมือนกันทั้งซ้าย/ขวา
export function polarityVerdict(leftSigned, rightSigned) {
  if (!Number.isFinite(leftSigned) || !Number.isFinite(rightSigned)) {
    return { ok: null, message: 'แกว่งขาทั้งสองข้างก่อน' };
  }
  const leftSign = Math.sign(leftSigned);
  const rightSign = Math.sign(rightSigned);
  if (leftSign === 0 || rightSign === 0) {
    return { ok: null, message: 'สัญญาณยังน้อย — แกว่งแรงขึ้น' };
  }
  const same = leftSign === rightSign;
  return {
    ok: same,
    leftSign,
    rightSign,
    message: same
      ? 'polarity เหมือนกันทั้งสองข้าง ✅'
      : 'mirror mounting — ต้อง flip sign ข้างหนึ่งใน AXIS_MAP ❌',
  };
}

function emptyTracker(sensorKey, side, label) {
  const zero = () => ({ ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0 });
  return {
    sensorKey,
    side: side || null,
    label: label || sensorKey,
    latest: zero(),
    peakAbs: zero(),
    peakSignedGyro: { gx: null, gy: null, gz: null },
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
    const magnitude = Math.abs(values[axis]);
    if (magnitude > tracker.peakAbs[axis]) {
      tracker.peakAbs[axis] = magnitude;
    }
  }
  // เก็บ signed value ของแกน gyro ณ จุดที่ |ค่า| มากสุด (ไว้เช็ค polarity)
  for (const axis of GYRO_AXES) {
    const current = tracker.peakSignedGyro[axis];
    if (current === null || Math.abs(values[axis]) > Math.abs(current)) {
      tracker.peakSignedGyro[axis] = values[axis];
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
    this.els = {};
  }

  init() {
    this.els = {
      status: document.getElementById('debug-status'),
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
      tracker.peakAbs = { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0 };
      tracker.peakSignedGyro = { gx: null, gy: null, gz: null };
    }
    this._setStatus('Peaks reset. Do one clean forward swing per side.');
  }

  toggleRecording() {
    if (this.traceRecorder.isRecording()) {
      const count = this.traceRecorder.stop();
      if (this.els.record) {
        this.els.record.textContent = '● Record';
      }
      this._setStatus(`Recording stopped (${count} samples). Export to save.`);
    } else {
      this.traceRecorder.start();
      if (this.els.record) {
        this.els.record.textContent = '■ Stop';
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
    const trace = this.traceRecorder.buildTrace({ appVersion: '1.0.0-debug' });
    downloadTraceJson(trace, buildTraceFilename());
  }

  render() {
    if (!this.els.sensors) {
      return;
    }

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
    const sagittal = detectSagittalAxis(tracker.peakAbs);
    const row = (axis, unit) => {
      const value = tracker.latest[axis];
      const peak = tracker.peakAbs[axis];
      const isSagittal = axis === sagittal.axis && GYRO_AXES.includes(axis);
      const sign = value > 0 ? 'pos' : value < 0 ? 'neg' : 'zero';
      return `<tr class="${isSagittal ? 'sagittal' : ''}">
        <td class="axis">${axis}${isSagittal ? ' ★' : ''}</td>
        <td class="val ${sign}">${value.toFixed(1)}</td>
        <td class="peak">${peak.toFixed(1)}</td>
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
      <div class="sagittal-verdict ${sagittal.isCorrect ? 'ok' : 'bad'}">
        Sagittal axis (peak gyro) = <b>${sagittal.axis}</b> ${sagittal.isCorrect ? '✅ (gx ถูกต้อง)' : '❌ ต้อง remap (ควรเป็น gx)'}
        &nbsp;— peak ${sagittal.value.toFixed(0)}°/s
      </div>
      <table class="debug-table">
        <thead><tr><th>axis</th><th>live</th><th>peak|·|</th><th></th></tr></thead>
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

    const leftSag = detectSagittalAxis(left.peakAbs);
    const rightSag = detectSagittalAxis(right.peakAbs);
    const leftSigned = left.peakSignedGyro[leftSag.axis];
    const rightSigned = right.peakSignedGyro[rightSag.axis];
    const verdict = polarityVerdict(leftSigned, rightSigned);

    const cls = verdict.ok === true ? 'ok' : verdict.ok === false ? 'bad' : 'pending';
    const detail = Number.isFinite(leftSigned) && Number.isFinite(rightSigned)
      ? `L ${leftSag.axis}=${leftSigned.toFixed(0)}°/s, R ${rightSag.axis}=${rightSigned.toFixed(0)}°/s`
      : '';
    return `<div class="verdict ${cls}">
      <div class="verdict-title">ท่าที่ 2 — Polarity check</div>
      <div class="verdict-msg">${verdict.message}</div>
      <div class="verdict-detail">${detail}</div>
    </div>`;
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

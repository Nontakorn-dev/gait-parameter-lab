/**
 * TraceRecorder — เก็บ raw IMU trace สำหรับวิเคราะห์/reprocess offline
 *
 * หลักการสำคัญ: บันทึก raw "ก่อน" axis-remap (sensor frame) เป็น source of truth
 * ถ้า AXIS_MAP ที่ตั้งไว้ผิด ยัง reprocess จากไฟล์ได้โดยไม่ต้องเก็บข้อมูลใหม่
 * (สำคัญมากกับผู้ป่วย stroke ที่พามาเดินซ้ำมีต้นทุนสูง). ค่า canonical (หลัง remap)
 * เก็บควบไว้เพื่อรู้ว่า pipeline สดใช้ค่าอะไรจริง. header ผูก gyro bias / axis map /
 * packet version / sample rate (nominal + measured) + ground truth เพื่อให้ไฟล์
 * ตีความและ validate ได้ด้วยตัวเอง.
 */

function median(values) {
  if (!values.length) {
    return null;
  }
  const sorted = [...values].sort((a, b) => a - b);
  const mid = Math.floor(sorted.length / 2);
  return sorted.length % 2 === 0 ? (sorted[mid - 1] + sorted[mid]) / 2 : sorted[mid];
}

export class TraceRecorder {
  constructor({ sampleRateHzNominal = 100, maxSamples = 300000 } = {}) {
    this.sampleRateHzNominal = sampleRateHzNominal;
    this.maxSamples = maxSamples;
    this.recording = false;
    this.samples = [];
    this.startedAt = null;
    this.packetVersionBySensor = {};
    this.truncated = false;
  }

  isRecording() {
    return this.recording;
  }

  sampleCount() {
    return this.samples.length;
  }

  isTruncated() {
    return this.truncated;
  }

  start() {
    this.recording = true;
    this.samples = [];
    this.startedAt = Date.now();
    this.packetVersionBySensor = {};
    this.truncated = false;
  }

  stop() {
    this.recording = false;
    return this.samples.length;
  }

  record(sample) {
    if (!this.recording || !sample) {
      return;
    }

    // เพดานกันหน่วยความจำ: หยุดบันทึกและ mark truncated เมื่อถึง cap แทนที่จะโตไม่จำกัด
    if (this.samples.length >= this.maxSamples) {
      this.truncated = true;
      this.recording = false;
      return;
    }

    // packetVersion = IMU packet format version จาก firmware (offset 2, คงที่ 1)
    // ไม่ใช่เวอร์ชันเฟิร์มแวร์จริง — เวอร์ชันเฟิร์มแวร์ต้องใส่เป็น firmwareBuildTag ตอน export
    if (Number.isFinite(sample.packetVersion) && sample.sensorKey) {
      this.packetVersionBySensor[sample.sensorKey] = sample.packetVersion;
    }

    this.samples.push({
      t_ms: Number.isFinite(sample.timestampMs) ? sample.timestampMs : null,
      seq: Number.isFinite(sample.seq) ? sample.seq : null,
      sensorKey: sample.sensorKey ?? null,
      side: sample.side ?? null,
      sensorMount: sample.sensorMount ?? null,
      // PRE axis-remap (raw sensor frame) — source of truth
      raw_accel_sensor: sample.rawAccelSensor ?? null,
      raw_gyro_sensor: sample.rawGyroSensor ?? null,
      // post-remap canonical — สิ่งที่ pipeline สดใช้จริง
      raw_accel_canonical: sample.raw_accel ?? null,
      raw_gyro_canonical: sample.raw_gyro ?? null,
    });
  }

  // วัด sample rate จริงจาก timestamp ต่อเซนเซอร์ (median ของ 1/Δt) — ไม่ใช้ค่า nominal ลอย ๆ
  measureSampleRates() {
    const timesBySensor = new Map();
    for (const s of this.samples) {
      if (!Number.isFinite(s.t_ms)) {
        continue;
      }
      const key = s.sensorKey ?? '_';
      if (!timesBySensor.has(key)) {
        timesBySensor.set(key, []);
      }
      timesBySensor.get(key).push(s.t_ms);
    }

    const bySensor = {};
    const rates = [];
    for (const [key, times] of timesBySensor) {
      times.sort((a, b) => a - b);
      const deltas = [];
      for (let i = 1; i < times.length; i += 1) {
        const d = times[i] - times[i - 1];
        if (d > 0) {
          deltas.push(d);
        }
      }
      const medDelta = median(deltas);
      if (Number.isFinite(medDelta) && medDelta > 0) {
        const rate = 1000 / medDelta;
        bySensor[key] = rate;
        rates.push(rate);
      }
    }

    return { overall: median(rates), bySensor };
  }

  hasSensorFrameRaw() {
    return this.samples.some((s) => Array.isArray(s.raw_accel_sensor));
  }

  buildTrace({
    axisMap = null,
    calibrationBySensor = {},
    appVersion = null,
    firmwareBuildTag = null,
    groundTruth = null,
  } = {}) {
    const measuredRates = this.measureSampleRates();
    const hasSensorFrameRaw = this.hasSensorFrameRaw();

    return {
      schemaVersion: 2,
      recordedAt: this.startedAt ? new Date(this.startedAt).toISOString() : null,
      exportedAt: new Date().toISOString(),
      app: 'gait-parameter-lab',
      appVersion,
      // เวอร์ชันเฟิร์มแวร์จริงต้องกรอกเอง (packet version ตามไม่ได้)
      firmwareBuildTag,
      // แหล่งข้อมูล: ถ้าไม่มี sensor-frame raw = เป็น trace จาก demo, reprocess ไม่ได้
      source: hasSensorFrameRaw ? 'live' : 'demo-or-no-sensor',
      hasSensorFrameRaw,
      truncated: this.truncated,
      sampleRateHzNominal: this.sampleRateHzNominal,
      sampleRateHzMeasured: measuredRates.overall,
      sampleRateHzBySensor: measuredRates.bySensor,
      sampleCount: this.samples.length,
      axisMap,
      packetVersionBySensor: { ...this.packetVersionBySensor },
      calibrationBySensor,
      // ground truth สำหรับ validate (ระยะที่วัดจริง + นับก้าวเอง)
      groundTruth,
      note: 'raw_accel_sensor / raw_gyro_sensor are PRE axis-remap (raw sensor frame) '
        + 'and are the source of truth. raw_accel_canonical / raw_gyro_canonical are '
        + 'post-remap (what the live pipeline consumed). Reprocess from *_sensor if axisMap is wrong. '
        + 'packetVersionBySensor is the IMU packet format version, NOT the firmware version '
        + '(see firmwareBuildTag).',
      samples: this.samples,
    };
  }
}

/**
 * ดาวน์โหลด trace เป็นไฟล์ JSON (browser only)
 */
export function downloadTraceJson(trace, filename) {
  if (typeof document === 'undefined' || typeof URL === 'undefined') {
    return false;
  }

  const blob = new Blob([JSON.stringify(trace)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = filename;
  document.body.appendChild(anchor);
  anchor.click();
  document.body.removeChild(anchor);
  URL.revokeObjectURL(url);
  return true;
}

export function buildTraceFilename(now = new Date()) {
  const pad = (value) => String(value).padStart(2, '0');
  const stamp = `${now.getFullYear()}${pad(now.getMonth() + 1)}${pad(now.getDate())}`
    + `-${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
  return `gait-trace-${stamp}.json`;
}

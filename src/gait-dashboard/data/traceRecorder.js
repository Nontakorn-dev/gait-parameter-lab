/**
 * TraceRecorder — เก็บ raw IMU trace สำหรับวิเคราะห์/reprocess offline
 *
 * หลักการสำคัญ: บันทึก raw "ก่อน" axis-remap (sensor frame) เป็น source of truth
 * ถ้า AXIS_MAP ที่ตั้งไว้ผิด ยัง reprocess จากไฟล์ได้โดยไม่ต้องเก็บข้อมูลใหม่
 * (สำคัญมากกับผู้ป่วย stroke ที่พามาเดินซ้ำมีต้นทุนสูง). ค่า canonical (หลัง remap)
 * เก็บควบไว้เพื่อรู้ว่า pipeline สดใช้ค่าอะไรจริง. header ผูก gyro bias / axis map /
 * firmware version / sample rate เพื่อให้ไฟล์ตีความได้ด้วยตัวเอง.
 */
export class TraceRecorder {
  constructor({ sampleRateHz = 100 } = {}) {
    this.sampleRateHz = sampleRateHz;
    this.recording = false;
    this.samples = [];
    this.startedAt = null;
    this.firmwareVersionBySensor = {};
  }

  isRecording() {
    return this.recording;
  }

  sampleCount() {
    return this.samples.length;
  }

  start() {
    this.recording = true;
    this.samples = [];
    this.startedAt = Date.now();
    this.firmwareVersionBySensor = {};
  }

  stop() {
    this.recording = false;
    return this.samples.length;
  }

  record(sample) {
    if (!this.recording || !sample) {
      return;
    }

    if (Number.isFinite(sample.firmwareVersion) && sample.sensorKey) {
      this.firmwareVersionBySensor[sample.sensorKey] = sample.firmwareVersion;
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

  buildTrace({ axisMap = null, calibrationBySensor = {}, appVersion = null } = {}) {
    return {
      schemaVersion: 1,
      recordedAt: this.startedAt ? new Date(this.startedAt).toISOString() : null,
      exportedAt: new Date().toISOString(),
      app: 'gait-parameter-lab',
      appVersion,
      sampleRateHz: this.sampleRateHz,
      sampleCount: this.samples.length,
      axisMap,
      firmwareVersionBySensor: { ...this.firmwareVersionBySensor },
      calibrationBySensor,
      note: 'raw_accel_sensor / raw_gyro_sensor are PRE axis-remap (raw sensor frame) '
        + 'and are the source of truth. raw_accel_canonical / raw_gyro_canonical are '
        + 'post-remap (what the live pipeline consumed). Reprocess from *_sensor if axisMap is wrong.',
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

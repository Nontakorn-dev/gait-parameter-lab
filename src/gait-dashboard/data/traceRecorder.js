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

const SEQ_WRAP = 65536; // seq เป็น uint16 (firmware g_nextSeq) wrap ทุก ~10.9 นาที @100Hz
// gap ใหญ่กว่านี้ถือเป็น discontinuity (firmware reset/reconnect) ไม่ใช่ packet drop ปกติ
const SEQ_DROP_GAP_MAX = 2000;

export class TraceRecorder {
  constructor({ sampleRateHzNominal = 100, maxSamples = 300000, maxCycles = 20000 } = {}) {
    this.sampleRateHzNominal = sampleRateHzNominal;
    this.maxSamples = maxSamples;
    // cycle diagnostic เบากว่า raw sample มาก (ไม่กี่ตัวเลขต่อ cycle) ให้ cap สูงกว่าได้สบาย —
    // 1 stride ~1s ดังนั้น default 20000 cycles ครอบคลุมเดินต่อเนื่อง >5 ชม. เกินพอสำหรับ session จริง
    this.maxCycles = maxCycles;
    this.recording = false;
    this.samples = [];
    this.cycles = [];
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

  cycleCount() {
    return this.cycles.length;
  }

  isTruncated() {
    return this.truncated;
  }

  start() {
    this.recording = true;
    this.samples = [];
    this.cycles = [];
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

  // บันทึก ZUPT diagnostic ต่อ cycle (จาก GaitProcessor.onParams -> newCycleDiagnostics)
  // แยกเส้นทางจาก record() (raw IMU) โดยเจตนา — params ที่ประมวลผลแล้วไม่เคยไหลเข้า trace
  // มาก่อน ทำให้ export ไปวิเคราะห์ ZUPT ไม่ได้เลยแม้จะมีค่าคำนวณอยู่ใน live params ก็ตาม
  recordCycle(cycleDiagnostic) {
    if (!this.recording || !cycleDiagnostic) {
      return;
    }
    if (this.cycles.length >= this.maxCycles) {
      return;
    }
    this.cycles.push(cycleDiagnostic);
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

  // นับ dropped sample จาก seq gap ต่อเซนเซอร์ (measured rate มองไม่เห็น drop เพราะ median
  // ทนต่อ outlier). จัดการ wrap ที่ 65535; gap ใหญ่ผิดปกติ = reset/reconnect นับแยกเป็น
  // discontinuity ไม่ปนกับ dropped
  countSeqGaps() {
    const seqBySensor = new Map();
    for (const s of this.samples) {
      if (!Number.isFinite(s.seq)) {
        continue;
      }
      const key = s.sensorKey ?? '_';
      if (!seqBySensor.has(key)) {
        seqBySensor.set(key, []);
      }
      seqBySensor.get(key).push(s.seq);
    }

    const dropped = {};
    const discontinuities = {};
    for (const [key, seqs] of seqBySensor) {
      let drop = 0;
      let disc = 0;
      for (let i = 1; i < seqs.length; i += 1) {
        const gap = (((seqs[i] - seqs[i - 1]) % SEQ_WRAP) + SEQ_WRAP) % SEQ_WRAP;
        if (gap <= 1) {
          continue; // 0 = duplicate, 1 = ต่อเนื่องปกติ
        }
        if (gap <= SEQ_DROP_GAP_MAX) {
          drop += gap - 1;
        } else {
          disc += 1; // seq ถอย/กระโดดไกล = reset/reconnect ไม่ใช่ drop
        }
      }
      dropped[key] = drop;
      discontinuities[key] = disc;
    }

    return { dropped, discontinuities };
  }

  buildTrace({
    axisMap = null,
    calibrationBySensor = {},
    appVersion = null,
    firmwareBuildTag = null,
    groundTruth = null,
  } = {}) {
    const measuredRates = this.measureSampleRates();
    const seqGaps = this.countSeqGaps();
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
      // dropped จาก seq gap (measured rate มองไม่เห็น drop); discontinuity = reset/reconnect
      droppedBySensor: seqGaps.dropped,
      seqDiscontinuitiesBySensor: seqGaps.discontinuities,
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
        + '(see firmwareBuildTag). cycles[] carries per-cycle ZUPT diagnostics '
        + '(zuptCheck: vStartPreDrift/vEndPreDrift/windowSource/zuptAccelDeviationG) for every '
        + 'gait cycle seen during the recording, not just the last one.',
      samples: this.samples,
      cycleCount: this.cycles.length,
      cycles: this.cycles,
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

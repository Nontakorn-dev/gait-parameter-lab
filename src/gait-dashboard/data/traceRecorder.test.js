import test from 'node:test';
import assert from 'node:assert/strict';

import { TraceRecorder, buildTraceFilename } from './traceRecorder.js';

function sensorSample(overrides = {}) {
  return {
    sensorKey: 'DernDee_R_Shank',
    side: 'R',
    sensorMount: 'shank',
    timestampMs: 1000,
    seq: 5,
    packetVersion: 1,
    rawAccelSensor: [10, 20, 30],   // PRE remap
    rawGyroSensor: [1, 2, 3],
    raw_accel: [10, 20, 30],        // canonical (identity ตอนนี้)
    raw_gyro: [1, 2, 3],
    ...overrides,
  };
}

test('record ถูก ignore เมื่อยังไม่ start', () => {
  const rec = new TraceRecorder();
  rec.record(sensorSample());
  assert.equal(rec.sampleCount(), 0);
});

test('start เคลียร์ของเก่าและเริ่มบันทึก', () => {
  const rec = new TraceRecorder();
  rec.start();
  rec.record(sensorSample());
  assert.equal(rec.isRecording(), true);
  assert.equal(rec.sampleCount(), 1);
  rec.start();
  assert.equal(rec.sampleCount(), 0, 'start ใหม่ต้องล้าง buffer');
});

test('บันทึก raw ก่อน remap เป็น source of truth (แยกจาก canonical)', () => {
  const rec = new TraceRecorder();
  rec.start();
  rec.record(sensorSample({ rawAccelSensor: [100, 200, 300], raw_accel: [-100, 200, 300] }));
  const s = rec.samples[0];
  assert.deepEqual(s.raw_accel_sensor, [100, 200, 300], 'ต้องเก็บค่า pre-remap');
  assert.deepEqual(s.raw_accel_canonical, [-100, 200, 300], 'และเก็บ canonical ควบไว้');
  assert.equal(s.seq, 5);
  assert.equal(s.sensorKey, 'DernDee_R_Shank');
});

test('stop คืนจำนวน sample และหยุดบันทึก', () => {
  const rec = new TraceRecorder();
  rec.start();
  rec.record(sensorSample());
  rec.record(sensorSample());
  const count = rec.stop();
  assert.equal(count, 2);
  assert.equal(rec.isRecording(), false);
  rec.record(sensorSample());
  assert.equal(rec.sampleCount(), 2, 'หลัง stop ไม่บันทึกเพิ่ม');
});

test('buildTrace: header ครบ (axis map, calibration, packet version, note)', () => {
  const rec = new TraceRecorder({ sampleRateHzNominal: 100 });
  rec.start();
  rec.record(sensorSample());
  const axisMap = { R: { accel: [[0, 1], [1, 1], [2, 1]], gyro: [[0, 1], [1, 1], [2, 1]] } };
  const calib = { DernDee_R_Shank: { gyroBiasDps: { gx: 0.5, gy: 0, gz: 0 }, shankLengthM: 0.42 } };
  const trace = rec.buildTrace({ axisMap, calibrationBySensor: calib, appVersion: '1.0.0' });

  assert.equal(trace.schemaVersion, 2);
  assert.equal(trace.sampleRateHzNominal, 100);
  assert.equal(trace.sampleCount, 1);
  assert.deepEqual(trace.axisMap, axisMap);
  assert.deepEqual(trace.calibrationBySensor, calib);
  assert.equal(trace.packetVersionBySensor.DernDee_R_Shank, 1, 'packet version (ไม่ใช่ firmware)');
  assert.ok(/NOT the firmware version/.test(trace.note), 'note ต้องเตือนว่า packet version ≠ firmware');
  assert.equal(trace.samples.length, 1);
});

test('จุดที่ 1: ground truth (ระยะจริง + นับก้าวเอง) ลง header', () => {
  const rec = new TraceRecorder();
  rec.start();
  rec.record(sensorSample());
  const trace = rec.buildTrace({
    firmwareBuildTag: 'dlpf24-rb64',
    groundTruth: { distanceM: 10, stepCountManual: 14, notes: 'ทางเรียบ' },
  });
  assert.equal(trace.groundTruth.distanceM, 10);
  assert.equal(trace.groundTruth.stepCountManual, 14);
  assert.equal(trace.firmwareBuildTag, 'dlpf24-rb64');
});

test('จุดที่ 3: sample rate วัดจริงจาก timestamp (ไม่ใช่ค่า nominal ลอย)', () => {
  const rec = new TraceRecorder({ sampleRateHzNominal: 100 });
  rec.start();
  // จำลอง 98Hz: Δt = 10.204ms → วัดได้ ~98, nominal ยัง 100
  for (let i = 0; i < 50; i += 1) {
    rec.record(sensorSample({ seq: i, timestampMs: 1000 + i * (1000 / 98) }));
  }
  const trace = rec.buildTrace();
  assert.equal(trace.sampleRateHzNominal, 100);
  assert.ok(Math.abs(trace.sampleRateHzMeasured - 98) < 1, `measured=${trace.sampleRateHzMeasured} ควร ~98`);
});

test('จุดเล็ก#1: เพดาน sample หยุดบันทึกและ mark truncated', () => {
  const rec = new TraceRecorder({ maxSamples: 3 });
  rec.start();
  for (let i = 0; i < 10; i += 1) {
    rec.record(sensorSample({ seq: i }));
  }
  assert.equal(rec.sampleCount(), 3, 'ต้องไม่เกิน cap');
  assert.equal(rec.isTruncated(), true);
  assert.equal(rec.isRecording(), false, 'ถึง cap แล้วหยุดบันทึก');
  assert.equal(rec.buildTrace().truncated, true);
});

test('จุดเล็ก#2: trace ไม่มี sensor-frame raw = ตีตรา demo (กันส่ง demo มาโดยไม่ตั้งใจ)', () => {
  const rec = new TraceRecorder();
  rec.start();
  rec.record(sensorSample({ rawAccelSensor: null, rawGyroSensor: null })); // แบบ demo
  const trace = rec.buildTrace();
  assert.equal(trace.hasSensorFrameRaw, false);
  assert.equal(trace.source, 'demo-or-no-sensor');
});

test('dropped จาก seq gap: median rate มองไม่เห็น แต่ seq gap เห็น', () => {
  const rec = new TraceRecorder({ sampleRateHzNominal: 100 });
  rec.start();
  // ดรอป seq ที่ %6===5 (5,11,...,59 = 10 ตัว); จบที่ seq 60 ที่เก็บไว้ เพื่อให้ทุก drop มีตัวถัดมา
  let t = 1000;
  for (let seq = 0; seq <= 60; seq += 1) {
    if (seq % 6 === 5) { t += 10; continue; } // ดรอป seq นี้ (timestamp เดินตามจริง เกิด gap 20ms)
    rec.record(sensorSample({ seq, timestampMs: t }));
    t += 10;
  }
  const trace = rec.buildTrace();
  // median rate ยังใกล้ 100 (ไม่เผย drop)
  assert.ok(Math.abs(trace.sampleRateHzMeasured - 100) < 1, `measured=${trace.sampleRateHzMeasured}`);
  // แต่ seq gap เผย drop
  assert.equal(trace.droppedBySensor.DernDee_R_Shank, 10, 'ต้องนับ drop ได้ 10');
});

test('seq wrap ที่ 65535 ไม่นับเป็น drop', () => {
  const rec = new TraceRecorder();
  rec.start();
  for (const seq of [65533, 65534, 65535, 0, 1, 2]) {
    rec.record(sensorSample({ seq }));
  }
  assert.equal(rec.countSeqGaps().dropped.DernDee_R_Shank, 0);
});

test('seq กระโดดถอย (firmware reset) = discontinuity ไม่ใช่ drop', () => {
  const rec = new TraceRecorder();
  rec.start();
  for (const seq of [5000, 5001, 5002, 0, 1, 2]) { // reconnect → seq reset เป็น 0
    rec.record(sensorSample({ seq }));
  }
  const gaps = rec.countSeqGaps();
  assert.equal(gaps.dropped.DernDee_R_Shank, 0, 'ไม่นับเป็น drop มหาศาล');
  assert.equal(gaps.discontinuities.DernDee_R_Shank, 1);
});

test('seq gap แยกต่อเซนเซอร์', () => {
  const rec = new TraceRecorder();
  rec.start();
  rec.record(sensorSample({ sensorKey: 'L', seq: 0 }));
  rec.record(sensorSample({ sensorKey: 'R', seq: 0 }));
  rec.record(sensorSample({ sensorKey: 'L', seq: 1 }));       // L ต่อเนื่อง
  rec.record(sensorSample({ sensorKey: 'R', seq: 3 }));       // R ดรอป 2
  const dropped = rec.countSeqGaps().dropped;
  assert.equal(dropped.L, 0);
  assert.equal(dropped.R, 2);
});

function cycleDiagnostic(overrides = {}) {
  return {
    cycleKey: '42',
    cycleStartTimestampMs: 1000,
    strideLengthM: 1.2,
    strideLengthClamped: false,
    zuptCheck: {
      vStartPreDrift: 0.01,
      vEndPreDrift: -0.55,
      windowSource: 'post-peak-valley',
      zuptAccelDeviationG: 0.21,
    },
    sensorKey: 'DernDee_R_Shank',
    side: 'R',
    ...overrides,
  };
}

test('recordCycle ถูก ignore เมื่อยังไม่ start (เหมือน record)', () => {
  const rec = new TraceRecorder();
  rec.recordCycle(cycleDiagnostic());
  assert.equal(rec.cycleCount(), 0);
});

test('recordCycle เก็บ zuptCheck ครบ และ buildTrace ใส่ลง trace.cycles จริง (ปิดช่องโหว่ params≠trace)', () => {
  const rec = new TraceRecorder();
  rec.start();
  rec.recordCycle(cycleDiagnostic());
  const trace = rec.buildTrace();

  assert.equal(trace.cycleCount, 1);
  assert.equal(trace.cycles.length, 1);
  const entry = trace.cycles[0];
  assert.equal(entry.cycleKey, '42');
  assert.equal(entry.strideLengthM, 1.2);
  assert.equal(entry.zuptCheck.vEndPreDrift, -0.55);
  assert.equal(entry.zuptCheck.windowSource, 'post-peak-valley');
  assert.equal(entry.sensorKey, 'DernDee_R_Shank');
});

test('recordCycle เก็บได้ทุก cycle ไม่ใช่แค่ตัวสุดท้าย', () => {
  const rec = new TraceRecorder();
  rec.start();
  rec.recordCycle(cycleDiagnostic({ cycleKey: '1' }));
  rec.recordCycle(cycleDiagnostic({ cycleKey: '2' }));
  rec.recordCycle(cycleDiagnostic({ cycleKey: '3' }));
  const trace = rec.buildTrace();
  assert.deepEqual(trace.cycles.map((c) => c.cycleKey), ['1', '2', '3']);
});

test('recordCycle หยุดรับที่ maxCycles (ไม่โตไม่จำกัด)', () => {
  const rec = new TraceRecorder({ maxCycles: 3 });
  rec.start();
  for (let i = 0; i < 10; i += 1) {
    rec.recordCycle(cycleDiagnostic({ cycleKey: String(i) }));
  }
  assert.equal(rec.cycleCount(), 3, 'ต้องไม่เกิน cap');
});

test('start() ล้าง cycles เดิม', () => {
  const rec = new TraceRecorder();
  rec.start();
  rec.recordCycle(cycleDiagnostic());
  assert.equal(rec.cycleCount(), 1);
  rec.start();
  assert.equal(rec.cycleCount(), 0);
});

test('note อธิบาย cycles[]/zuptCheck ไว้ในไฟล์', () => {
  const rec = new TraceRecorder();
  rec.start();
  const trace = rec.buildTrace();
  assert.ok(/cycles\[\]/.test(trace.note));
  assert.ok(/zuptCheck/.test(trace.note));
});

test('buildTraceFilename มีรูปแบบ timestamp', () => {
  const name = buildTraceFilename(new Date('2026-07-15T09:08:07'));
  assert.equal(name, 'gait-trace-20260715-090807.json');
});

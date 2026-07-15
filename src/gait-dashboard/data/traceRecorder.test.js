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

test('buildTraceFilename มีรูปแบบ timestamp', () => {
  const name = buildTraceFilename(new Date('2026-07-15T09:08:07'));
  assert.equal(name, 'gait-trace-20260715-090807.json');
});

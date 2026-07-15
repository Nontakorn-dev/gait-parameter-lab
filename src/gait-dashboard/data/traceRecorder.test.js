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
    firmwareVersion: 1,
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

test('buildTrace: header ครบ (axis map, calibration, firmware, sample rate, note)', () => {
  const rec = new TraceRecorder({ sampleRateHz: 100 });
  rec.start();
  rec.record(sensorSample());
  const axisMap = { R: { accel: [[0, 1], [1, 1], [2, 1]], gyro: [[0, 1], [1, 1], [2, 1]] } };
  const calib = { DernDee_R_Shank: { gyroBiasDps: { gx: 0.5, gy: 0, gz: 0 }, shankLengthM: 0.42 } };
  const trace = rec.buildTrace({ axisMap, calibrationBySensor: calib, appVersion: '1.0.0' });

  assert.equal(trace.schemaVersion, 1);
  assert.equal(trace.sampleRateHz, 100);
  assert.equal(trace.sampleCount, 1);
  assert.deepEqual(trace.axisMap, axisMap);
  assert.deepEqual(trace.calibrationBySensor, calib);
  assert.equal(trace.firmwareVersionBySensor.DernDee_R_Shank, 1);
  assert.ok(/PRE axis-remap/.test(trace.note), 'note ต้องอธิบายว่า *_sensor คือ pre-remap');
  assert.equal(trace.samples.length, 1);
});

test('buildTraceFilename มีรูปแบบ timestamp', () => {
  const name = buildTraceFilename(new Date('2026-07-15T09:08:07'));
  assert.equal(name, 'gait-trace-20260715-090807.json');
});

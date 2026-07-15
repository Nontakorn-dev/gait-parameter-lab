import test from 'node:test';
import assert from 'node:assert/strict';

import {
  detectSagittalAxis,
  polarityVerdict,
  extractAxes,
  updateTracker,
} from './debugApp.js';

test('ท่า 1: detectSagittalAxis คืน gx เมื่อ gx peak สูงสุด (✅)', () => {
  const r = detectSagittalAxis({ gx: 320, gy: 40, gz: 25 });
  assert.equal(r.axis, 'gx');
  assert.equal(r.isCorrect, true);
});

test('ท่า 1: ถ้า gz สูงสุด = ติดตั้งหมุนแกน ต้อง remap (❌)', () => {
  const r = detectSagittalAxis({ gx: 30, gy: 20, gz: 300 });
  assert.equal(r.axis, 'gz');
  assert.equal(r.isCorrect, false);
});

test('ท่า 2: polarity เหมือนกัน = ok', () => {
  const r = polarityVerdict(300, 280);
  assert.equal(r.ok, true);
});

test('ท่า 2: sign ต่างกัน = mirror ต้อง flip', () => {
  const r = polarityVerdict(300, -280);
  assert.equal(r.ok, false);
  assert.equal(r.leftSign, 1);
  assert.equal(r.rightSign, -1);
});

test('ท่า 2: ยังไม่มีสองข้าง = pending (ok=null)', () => {
  assert.equal(polarityVerdict(300, null).ok, null);
  assert.equal(polarityVerdict(NaN, 100).ok, null);
});

test('extractAxes ใช้ raw ก่อน remap เมื่อมี (usingPreRemap=true)', () => {
  const sample = {
    rawAccelSensor: [4096, 0, 0], rawGyroSensor: [164, 0, 0],
    ax: -4096, ay: 0, az: 0, gx: -164, gy: 0, gz: 0, // canonical ต่างค่า (ไม่ควรถูกใช้)
  };
  const { usingPreRemap, values } = extractAxes(sample);
  assert.equal(usingPreRemap, true);
  assert.ok(Math.abs(values.ax - 1) < 1e-9, 'ax = 4096/4096 = 1g');
  assert.ok(Math.abs(values.gx - 10) < 1e-9, 'gx = 164/16.4 = 10 dps');
});

test('extractAxes fallback เป็น canonical เมื่อไม่มี sensor-frame (demo)', () => {
  const { usingPreRemap, values } = extractAxes({ ax: 4096, ay: 0, az: 0, gx: 328, gy: 0, gz: 0 });
  assert.equal(usingPreRemap, false);
  assert.ok(Math.abs(values.gx - 20) < 1e-9);
});

test('updateTracker เก็บ peak|·| และ signed value ของ gyro ณ จุด peak', () => {
  const tracker = {
    latest: {}, peakAbs: { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0 },
    peakSignedGyro: { gx: null, gy: null, gz: null }, sampleCount: 0,
  };
  // gx ไต่ขึ้นถึง +300 แล้วลง — peak signed ต้องเป็น +300
  for (const raw of [82, 1640, 4920, 1640, -820]) { // dps = raw/16.4 → 5,100,300,100,-50
    updateTracker(tracker, { rawGyroSensor: [raw, 0, 0], rawAccelSensor: [0, 0, 0] });
  }
  assert.ok(Math.abs(tracker.peakAbs.gx - 300) < 1e-9);
  assert.ok(Math.abs(tracker.peakSignedGyro.gx - 300) < 1e-9, 'signed ที่ peak = +300');
});

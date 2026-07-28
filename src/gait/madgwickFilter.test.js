import test from 'node:test';
import assert from 'node:assert/strict';

import { MadgwickFilter, sensorAccelToGaitFrame } from './madgwickFilter.js';

test('Madgwick: นิ่ง ay=-1g → มุม sagittal ใกล้ 0', () => {
  const filter = new MadgwickFilter({ beta: 0.1, samplePeriod: 0.01 });
  for (let i = 0; i < 300; i += 1) {
    filter.update(0, 0, 0, 0, -1, 0, 0.01);
  }
  const angle = filter.getSagittalAngleDeg();
  assert.ok(Math.abs(angle) < 5, `มุมควรใกล้ 0° ได้ ${angle.toFixed(2)}°`);
});

test('sensorAccelToGaitFrame: นิ่ง → aForward ≈ 0, aVert ≈ 0 หลังลบ g', () => {
  const filter = new MadgwickFilter({ beta: 0.15, samplePeriod: 0.01 });
  for (let i = 0; i < 400; i += 1) {
    filter.update(0, 0, 0, 0, -1, 0, 0.01);
  }
  const q = filter.getQuaternion();
  const frame = sensorAccelToGaitFrame(0, -1, 0, q.q0, q.q1, q.q2, q.q3);
  assert.ok(Math.abs(frame.aForward) < 0.5, `aForward=${frame.aForward}`);
  assert.ok(Math.abs(frame.aVert) < 1.5, `aVert=${frame.aVert}`);
});

test('Madgwick: ‖a‖ ไกล 1g → effectiveBeta = 0 (gyro-only ระหว่าง swing)', () => {
  const filter = new MadgwickFilter({ beta: 0.1, accelGateHardG: 0.5 });
  assert.equal(filter.effectiveBeta(0, -1, 0), 0.1);
  assert.equal(filter.effectiveBeta(0, -2, 0), 0); // |2-1|=1 > 0.5
  assert.ok(filter.effectiveBeta(0, -1.2, 0) < 0.1);
  assert.ok(filter.effectiveBeta(0, -1.2, 0) > 0);
});

test('Madgwick: +gx → มุม sagittal เพิ่ม (ตรงกับ Kalman / แกนโปรเจกต์)', () => {
  const filter = new MadgwickFilter({ beta: 0, gyroGateHardDps: 0 });
  for (let i = 0; i < 50; i += 1) {
    filter.update(90, 0, 0, 0, -1, 0, 0.01);
  }
  const angle = filter.getSagittalAngleDeg();
  assert.ok(angle > 40 && angle < 50, `+gx 0.5s ควรได้ ~+45° ได้ ${angle.toFixed(1)}°`);
});

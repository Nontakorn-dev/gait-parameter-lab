import test from 'node:test';
import assert from 'node:assert/strict';

import { VelocityIntegrator } from './velocityIntegrator.js';

const G = 9.81;

// สัญญาณทดสอบ: shank angle = 0, ay = -g (หัก gravity แนวตั้งเป็น 0), az = โปรไฟล์ความเร่งแนวนอน
function buildSignal() {
  const az = [0, 1, 2.5, 4, 2, -1, -3, -2, 0, 1, 0.5, 0];
  const ay = az.map(() => -G);
  const angles = az.map(() => 0);
  return { ay, az, angles };
}

test('computeStrideMetrics ใช้ options.dt: strideLength scale ตาม dt^2 (double integration)', () => {
  const integ = new VelocityIntegrator({ sampleRate: 100 });
  const { ay, az, angles } = buildSignal();
  const opts = { integrationStartIdx: 0, integrationEndIdx: az.length - 1 };

  const a = integ.computeStrideMetrics(ay, az, angles, { ...opts, dt: 0.01 });
  const b = integ.computeStrideMetrics(ay, az, angles, { ...opts, dt: 0.02 });

  assert.ok(a.strideLength > 0, 'ต้องได้ระยะทาง > 0');
  const ratio = b.strideLength / a.strideLength;
  assert.ok(Math.abs(ratio - 4) < 1e-6, `ratio ควร ~4 ได้ ${ratio}`);
});

test('computeStrideMetrics fallback เป็น nominal dt เมื่อไม่ส่ง options.dt', () => {
  const integ = new VelocityIntegrator({ sampleRate: 100 });
  const { ay, az, angles } = buildSignal();
  const opts = { integrationStartIdx: 0, integrationEndIdx: az.length - 1 };

  const noDt = integ.computeStrideMetrics(ay, az, angles, { ...opts });
  const explicit = integ.computeStrideMetrics(ay, az, angles, { ...opts, dt: 0.01 });

  assert.ok(Math.abs(noDt.strideLength - explicit.strideLength) < 1e-9);
});

test('start-only ZUPT จริง: มี pre-window → vStartPreDrift วัดได้ และปลายไม่ถูกบังคับ 0', () => {
  const integ = new VelocityIntegrator({ sampleRate: 100 });
  // pre: เร่งคงที่ → สะสม v ก่อนเข้า window; window: เร่งต่อ
  const az = [2, 2, 2, 2, 2, 2, 2, 2];
  const ay = az.map(() => -G);
  const angles = az.map(() => 0);
  const { velocity, velocityPreDriftCorrection, vStartPreDrift, vEndPreDrift } = integ.computeStrideMetrics(
    ay, az, angles,
    { integrationStartIdx: 3, integrationEndIdx: az.length - 1, dt: 0.01 },
  );

  assert.ok(Math.abs(vStartPreDrift) > 0.01, `vStart ก่อนรีเซ็ตต้อง ≠ 0 ได้ ${vStartPreDrift}`);
  assert.ok(Math.abs(velocity[0]) < 1e-9, 'หลัง start-only: v ที่ต้น window = 0');
  assert.ok(Math.abs(velocity[velocity.length - 1]) > 0.01, 'ปลายไม่ถูกบังคับ 0');
  assert.equal(velocityPreDriftCorrection.length, velocity.length);
  assert.ok(Math.abs(vEndPreDrift - velocityPreDriftCorrection.at(-1)) < 1e-9);
});

test('start-only ที่ integrationStartIdx=0 ไม่ใช่ no-op ปลอมของ diagnostic: vStart=0 โดยสมมติ ZUPT', () => {
  const integ = new VelocityIntegrator({ sampleRate: 100 });
  const { ay, az, angles } = buildSignal();
  const r = integ.computeStrideMetrics(ay, az, angles, {
    integrationStartIdx: 0,
    integrationEndIdx: az.length - 1,
    dt: 0.01,
  });
  assert.ok(Math.abs(r.vStartPreDrift) < 1e-9);
  assert.ok(Math.abs(r.velocity[0]) < 1e-9);
  // ปลายยังมี residual (ไม่มี dual-end forcing)
  assert.notEqual(r.velocity[r.velocity.length - 1], 0);
});

test('driftMode start-end: บังคับปลายเป็น 0', () => {
  const integ = new VelocityIntegrator({ sampleRate: 100, driftMode: 'start-end' });
  const { ay, az, angles } = buildSignal();
  const { velocity } = integ.computeStrideMetrics(ay, az, angles, {
    integrationStartIdx: 0,
    integrationEndIdx: az.length - 1,
    dt: 0.01,
    driftMode: 'start-end',
  });
  assert.ok(Math.abs(velocity[0]) < 1e-9);
  assert.ok(Math.abs(velocity[velocity.length - 1]) < 1e-9);
});

test('velocityPreDriftCorrection ว่างเปล่าเมื่อ window สั้นเกินไป (< 2 samples)', () => {
  const integ = new VelocityIntegrator({ sampleRate: 100 });
  const r = integ.computeStrideMetrics([0], [0], [0], { integrationStartIdx: 0, integrationEndIdx: 0 });
  assert.deepEqual(r.velocityPreDriftCorrection, []);
});

test('sensorToWorld: sensor นิ่งเอียงทุกมุม → aVert=g, aHoriz=0 (ไม่มี gravity leakage)', () => {
  const integ = new VelocityIntegrator({ sampleRate: 100 });
  for (const theta of [0, 15, 30, 45]) {
    const rad = (theta * Math.PI) / 180;
    const ay = -Math.cos(rad) * G;
    const az = -Math.sin(rad) * G;
    const { aVert, aHoriz } = integ.sensorToWorld(ay, az, theta);
    assert.ok(Math.abs(aVert - G) < 1e-6, `aVert@${theta}=${aVert}`);
    assert.ok(Math.abs(aHoriz) < 1e-6, `aHoriz@${theta}=${aHoriz}`);
  }
});

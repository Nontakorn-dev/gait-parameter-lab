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
  // dt สองเท่า → displacement ~สี่เท่า
  const ratio = b.strideLength / a.strideLength;
  assert.ok(Math.abs(ratio - 4) < 1e-6, `ratio ควร ~4 ได้ ${ratio}`);
});

test('computeStrideMetrics fallback เป็น nominal dt เมื่อไม่ส่ง options.dt', () => {
  const integ = new VelocityIntegrator({ sampleRate: 100 }); // this.dt = 0.01
  const { ay, az, angles } = buildSignal();
  const opts = { integrationStartIdx: 0, integrationEndIdx: az.length - 1 };

  const noDt = integ.computeStrideMetrics(ay, az, angles, { ...opts });
  const explicit = integ.computeStrideMetrics(ay, az, angles, { ...opts, dt: 0.01 });

  assert.ok(Math.abs(noDt.strideLength - explicit.strideLength) < 1e-9);
});

test('velocityPreDriftCorrection: ต่างจาก velocity (post-correction) เมื่อปลายสัญญาณไม่นิ่ง', () => {
  const integ = new VelocityIntegrator({ sampleRate: 100 });
  const { ay, az, angles } = buildSignal();
  const { velocity, velocityPreDriftCorrection } = integ.computeStrideMetrics(
    ay, az, angles,
    { integrationStartIdx: 0, integrationEndIdx: az.length - 1, dt: 0.01 },
  );

  assert.equal(velocityPreDriftCorrection.length, velocity.length);
  // start-only ZUPT: บังคับ v=0 ที่ต้น ไม่บังคับปลาย (shank มักยังไม่นิ่ง)
  assert.ok(Math.abs(velocity[0]) < 1e-9, 'post-correction v เริ่มต้องเป็น 0');
  assert.ok(
    Math.abs(velocity[velocity.length - 1] - (
      velocityPreDriftCorrection[velocityPreDriftCorrection.length - 1]
      - velocityPreDriftCorrection[0]
    )) < 1e-9,
    'ปลายหลังแก้ = pre − vStart',
  );
  assert.notEqual(velocityPreDriftCorrection[velocityPreDriftCorrection.length - 1], 0,
    'pre-correction v ปลายไม่ควรเป็น 0 พอดี (สัญญาณทดสอบมี drift จริง)');
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

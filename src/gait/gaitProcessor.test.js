import test from 'node:test';
import assert from 'node:assert/strict';

import { GaitProcessor } from './gaitProcessor.js';
import { generateWalkingData } from '../gait-dashboard/data/demoDataGenerator.js';

// รัน full pipeline; stubStride = บังคับค่า integrate ต่อ cycle เพื่อทดสอบ clamp/นิยาม
function runPipeline({ stubStride } = {}) {
  const proc = new GaitProcessor();
  if (stubStride !== undefined) {
    proc.velocityIntegrator.computeStrideMetrics = () => ({
      strideLength: stubStride,
      strideLengthSigned: stubStride,
      clearance: 0.05,
    });
  }
  const { samples } = generateWalkingData({ numStrides: 12, strideTime: 1.05 });
  const t0 = Date.now();
  let last = null;
  proc.onParams(({ params }) => { if (params) last = params; });
  for (const s of samples) {
    proc.addSample({ ...s, timestampMs: t0 + Math.round(s.timestamp * 1000) });
  }
  proc.analyze();
  return last;
}

test('นิยาม: stepLength === strideLength / 2 (integrate ตลอด HS→HS = 1 stride)', () => {
  const p = runPipeline();
  assert.ok(p, 'ต้องได้ params');
  assert.ok(Number.isFinite(p.strideLength), 'strideLength ต้องเป็นค่าจริงเสมอ (ไม่ null)');
  assert.ok(Math.abs(p.stepLength - p.strideLength / 2) < 1e-9,
    `stepLength=${p.stepLength} ควร = strideLength/2=${p.strideLength / 2}`);
});

test('นิยาม: walkingSpeed === strideLength / strideTime', () => {
  const p = runPipeline();
  assert.ok(Math.abs(p.walkingSpeed - p.strideLength / p.strideTime) < 1e-9,
    `walkingSpeed=${p.walkingSpeed} ควร = ${p.strideLength / p.strideTime}`);
});

test('strideLength ขึ้นได้เกิน 0.80 (เพดาน step เดิม) — ไม่ถูก double-count/ตัดผิด', () => {
  const p = runPipeline({ stubStride: 1.4 });
  assert.ok(p.strideLength > 0.80, `strideLength=${p.strideLength} ต้องเกิน 0.80 ได้`);
  assert.ok(Math.abs(p.strideLength - 1.4) < 1e-9, 'ไม่ควรถูก clamp ที่เพดาน step เดิม');
  assert.ok(Math.abs(p.stepLength - 0.7) < 1e-9, `stepLength=${p.stepLength} ควร = 0.7`);
});

test('เพดาน stride ใหม่ 1.80m: ค่าเกินถูก clamp ที่ 1.80 ไม่ใช่ 1.60', () => {
  const p = runPipeline({ stubStride: 5.0 });
  assert.ok(Math.abs(p.strideLength - 1.80) < 1e-9, `strideLength=${p.strideLength} ควร clamp ที่ 1.80`);
});

test('floor ใหม่ 0.10m: stride สั้นของผู้ป่วย stroke ไม่ถูกดันขึ้น 0.30', () => {
  const p = runPipeline({ stubStride: 0.18 });
  assert.ok(Math.abs(p.strideLength - 0.18) < 1e-9, `strideLength=${p.strideLength} ต้องคงค่า 0.18 (ไม่ clamp)`);
  assert.equal(p.strideLengthClamped, false);
});

test('clamp flag: ค่าต่ำกว่า floor ถูก mark strideLengthClamped=true', () => {
  const p = runPipeline({ stubStride: 0.05 });
  assert.equal(p.strideLengthClamped, true, 'ค่าต่ำกว่า 0.10 ต้องถูก flag');
  assert.ok(Math.abs(p.strideLength - 0.10) < 1e-9);
});

test('clamp flag: ค่าในช่วงปกติไม่ถูก mark', () => {
  const p = runPipeline({ stubStride: 1.2 });
  assert.equal(p.strideLengthClamped, false);
});

test('clinical metadata: strideLengthSignedM และ zuptAccelDeviationG มีค่า (ผ่าน real integrator)', () => {
  const p = runPipeline();
  assert.ok(Number.isFinite(p.strideLengthSignedM), 'ต้องมี signed value ไว้ debug ทิศ');
  assert.ok(Number.isFinite(p.zuptAccelDeviationG) && p.zuptAccelDeviationG >= 0,
    'ต้องมี ZUPT-validity (‖accel‖ เบี่ยงจาก 1g ที่ปลาย window)');
});

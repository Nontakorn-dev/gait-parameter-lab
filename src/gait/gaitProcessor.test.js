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
      velocityPreDriftCorrection: [0, 0.1, 0.2],
    });
  }
  const { samples } = generateWalkingData({ numStrides: 12, strideTime: 1.05 });
  const t0 = Date.now();
  let last = null;
  let lastDiagnostics = [];
  proc.onParams(({ params, newCycleDiagnostics }) => {
    if (params) last = params;
    lastDiagnostics = newCycleDiagnostics || [];
  });
  for (const s of samples) {
    proc.addSample({ ...s, timestampMs: t0 + Math.round(s.timestamp * 1000) });
  }
  proc.analyze();
  return { params: last, diagnostics: lastDiagnostics };
}

// จำลอง streaming แบบแอปจริง: analyze() ทุก ~40 samples (400ms) แทนที่จะเรียกทีเดียวตอนจบ
// เพื่อทดสอบว่า diagnostics สะสมข้าม analyze() หลายครั้งได้ครบ ไม่ซ้ำ ไม่หาย
function runStreaming({ numStrides, strideTime, analyzeEvery = 40 }) {
  const proc = new GaitProcessor();
  const { samples } = generateWalkingData({ numStrides, strideTime });
  const t0 = Date.now();
  const allDiagnostics = [];
  let sinceAnalyze = 0;
  proc.onParams(({ newCycleDiagnostics }) => {
    allDiagnostics.push(...(newCycleDiagnostics || []));
  });
  for (const s of samples) {
    proc.addSample({ ...s, timestampMs: t0 + Math.round(s.timestamp * 1000) });
    sinceAnalyze += 1;
    if (sinceAnalyze >= analyzeEvery) {
      proc.analyze();
      sinceAnalyze = 0;
    }
  }
  proc.analyze();
  return { allDiagnostics, totalStrideCount: proc.totalStrideCount };
}

test('นิยาม: stepLength === strideLength / 2 (integrate ตลอด HS→HS = 1 stride)', () => {
  const { params: p } = runPipeline();
  assert.ok(p, 'ต้องได้ params');
  assert.ok(Number.isFinite(p.strideLength), 'strideLength ต้องเป็นค่าจริงเสมอ (ไม่ null)');
  assert.ok(Math.abs(p.stepLength - p.strideLength / 2) < 1e-9,
    `stepLength=${p.stepLength} ควร = strideLength/2=${p.strideLength / 2}`);
});

test('นิยาม: walkingSpeed === strideLength / strideTime', () => {
  const { params: p } = runPipeline();
  assert.ok(Math.abs(p.walkingSpeed - p.strideLength / p.strideTime) < 1e-9,
    `walkingSpeed=${p.walkingSpeed} ควร = ${p.strideLength / p.strideTime}`);
});

test('strideLength ขึ้นได้เกิน 0.80 (เพดาน step เดิม) — ไม่ถูก double-count/ตัดผิด', () => {
  const { params: p } = runPipeline({ stubStride: 1.4 });
  assert.ok(p.strideLength > 0.80, `strideLength=${p.strideLength} ต้องเกิน 0.80 ได้`);
  assert.ok(Math.abs(p.strideLength - 1.4) < 1e-9, 'ไม่ควรถูก clamp ที่เพดาน step เดิม');
  assert.ok(Math.abs(p.stepLength - 0.7) < 1e-9, `stepLength=${p.stepLength} ควร = 0.7`);
});

test('เพดาน stride ใหม่ 1.80m: ค่าเกินถูก clamp ที่ 1.80 ไม่ใช่ 1.60', () => {
  const { params: p } = runPipeline({ stubStride: 5.0 });
  assert.ok(Math.abs(p.strideLength - 1.80) < 1e-9, `strideLength=${p.strideLength} ควร clamp ที่ 1.80`);
});

test('floor ใหม่ 0.10m: stride สั้นของผู้ป่วย stroke ไม่ถูกดันขึ้น 0.30', () => {
  const { params: p } = runPipeline({ stubStride: 0.18 });
  assert.ok(Math.abs(p.strideLength - 0.18) < 1e-9, `strideLength=${p.strideLength} ต้องคงค่า 0.18 (ไม่ clamp)`);
  assert.equal(p.strideLengthClamped, false);
});

test('clamp flag: ค่าต่ำกว่า floor ถูก mark strideLengthClamped=true', () => {
  const { params: p } = runPipeline({ stubStride: 0.05 });
  assert.equal(p.strideLengthClamped, true, 'ค่าต่ำกว่า 0.10 ต้องถูก flag');
  assert.ok(Math.abs(p.strideLength - 0.10) < 1e-9);
});

test('clamp flag: ค่าในช่วงปกติไม่ถูก mark', () => {
  const { params: p } = runPipeline({ stubStride: 1.2 });
  assert.equal(p.strideLengthClamped, false);
});

test('clinical metadata: strideLengthSignedM และ zuptAccelDeviationG มีค่า (ผ่าน real integrator)', () => {
  const { params: p } = runPipeline();
  assert.ok(Number.isFinite(p.strideLengthSignedM), 'ต้องมี signed value ไว้ debug ทิศ');
  assert.ok(Number.isFinite(p.zuptAccelDeviationG) && p.zuptAccelDeviationG >= 0,
    'ต้องมี ZUPT-validity (‖accel‖ เบี่ยงจาก 1g ที่ปลาย window)');
});

test('latestParams ไม่มี windowSource ซ้ำกับ integrationSource อีกต่อไป (ตัดชื่อซ้ำทิ้ง)', () => {
  const { params: p } = runPipeline();
  assert.equal('windowSource' in p, false, 'windowSource ต้องไม่อยู่ระดับบนของ latestParams แล้ว');
  assert.ok(typeof p.integrationSource === 'string' && p.integrationSource.length > 0);
});

test('newCycleDiagnostics: มีทุก cycle ที่พบใน analyze() ครั้งแรก ไม่ใช่แค่ cycle สุดท้าย', () => {
  const { params: p, diagnostics } = runPipeline();
  assert.ok(diagnostics.length > 1, `analyze() ครั้งแรกบน buffer 12 stride ควรเจอหลาย cycle พร้อมกัน ได้ ${diagnostics.length}`);
  // cycle สุดท้ายใน diagnostics ต้องตรงกับ params (cycle ที่ latestParams สะท้อนอยู่)
  const lastDiag = diagnostics[diagnostics.length - 1];
  assert.equal(lastDiag.cycleKey, p.cycleKey);
  assert.ok(Math.abs(lastDiag.strideLengthM - p.strideLength) < 1e-9);
});

test('newCycleDiagnostics: shape ตรงตามที่ออกแบบ (zuptCheck นิยามครบ ไม่มี field เกิน)', () => {
  const { diagnostics } = runPipeline();
  const entry = diagnostics[0];
  assert.equal(typeof entry.cycleKey, 'string');
  assert.ok(Number.isFinite(entry.strideLengthM));
  assert.equal(typeof entry.strideLengthClamped, 'boolean');
  assert.ok('vStartPreDrift' in entry.zuptCheck);
  assert.ok('vEndPreDrift' in entry.zuptCheck);
  assert.ok('windowSource' in entry.zuptCheck, 'windowSource อยู่ใน zuptCheck (scope นี้ไม่ซ้ำกับที่อื่น)');
  assert.ok('zuptAccelDeviationG' in entry.zuptCheck);
  assert.ok(typeof entry.zuptCheck.windowSource === 'string' && entry.zuptCheck.windowSource.length > 0);
});

test('newCycleDiagnostics: dedup — analyze() ซ้ำโดยไม่มี sample ใหม่ ไม่ยิง cycle เดิมซ้ำ', () => {
  const proc = new GaitProcessor();
  const { samples } = generateWalkingData({ numStrides: 12, strideTime: 1.05 });
  const t0 = Date.now();
  for (const s of samples) {
    proc.addSample({ ...s, timestampMs: t0 + Math.round(s.timestamp * 1000) });
  }
  const firstBatch = [];
  proc.onParams(({ newCycleDiagnostics }) => firstBatch.push(...(newCycleDiagnostics || [])));
  proc.analyze();
  assert.ok(firstBatch.length > 0, 'รอบแรกต้องเจอ cycle');

  const secondBatch = [];
  proc.paramListeners = [];
  proc.onParams(({ newCycleDiagnostics }) => secondBatch.push(...(newCycleDiagnostics || [])));
  proc.analyze(); // เรียกซ้ำ buffer เดิม ไม่มี sample ใหม่
  assert.equal(secondBatch.length, 0, 'ไม่ควรมี cycle "ใหม่" เพราะ buffer ไม่เปลี่ยน');
});

test('newCycleDiagnostics: สะสมข้าม streaming ครบเท่ากับ totalStrideCount (ไม่หาย ไม่ซ้ำ)', () => {
  const { allDiagnostics, totalStrideCount } = runStreaming({ numStrides: 15, strideTime: 1.05 });
  assert.equal(allDiagnostics.length, totalStrideCount,
    `diagnostics=${allDiagnostics.length} ควรเท่ากับ totalStrideCount=${totalStrideCount}`);
  const uniqueKeys = new Set(allDiagnostics.map((d) => d.cycleKey));
  assert.equal(uniqueKeys.size, allDiagnostics.length, 'ไม่ควรมี cycleKey ซ้ำ (dedup ถูกต้อง)');
});

test('reset() เคลียร์ pendingCycleDiagnostics ที่ยังไม่ถูก drain', () => {
  const proc = new GaitProcessor();
  const { samples } = generateWalkingData({ numStrides: 12, strideTime: 1.05 });
  const t0 = Date.now();
  for (const s of samples) {
    proc.addSample({ ...s, timestampMs: t0 + Math.round(s.timestamp * 1000) });
  }
  // ไม่เรียก analyze() (ไม่ drain) — จำลอง edge case ที่ reset มาก่อน analyze ครั้งแรก
  proc.reset();
  assert.deepEqual(proc.pendingCycleDiagnostics, []);
});

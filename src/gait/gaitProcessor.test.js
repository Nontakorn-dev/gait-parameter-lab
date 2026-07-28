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
  const { samples } = generateWalkingData({ numStrides: 12, strideTime: 1.05, seed: 42 });
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
  const { samples } = generateWalkingData({ numStrides, strideTime, seed: 42 });
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

test('นิยาม: 1 HS→HS = 1 stride = 1 footfall ของขาที่วัด — ไม่ ×2 สมมาตร', () => {
  const { params: p } = runPipeline();
  assert.ok(p, 'ต้องได้ params');
  assert.ok(Number.isFinite(p.strideLength), 'strideLength ต้องเป็นค่าจริงเสมอ (ไม่ null)');
  assert.equal(p.stepCount, p.strideCount, 'เซนเซอร์ข้างเดียว: stepCount ต้องเท่า strideCount');
  assert.equal(p.stepLength, null, 'stepLength ห้าม invent จาก stride/2');
  assert.equal(p.stepTime, null, 'stepTime ห้าม invent จาก strideTime/2');
  assert.equal(p.doubleSupport, null, 'doubleSupport ต้องมี HS สองข้าง — ไม่ใช้ 2·stance−100');
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
  assert.equal(p.stepLength, null);
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

test('demo double-integration: mean |stride| ~ targetStrideLengthM (ไม่ใช่ gravity-only ~0.15 m)', () => {
  const target = 1.25;
  const proc = new GaitProcessor();
  const { samples } = generateWalkingData({
    numStrides: 12,
    strideTime: 1.05,
    seed: 42,
    noiseLevel: 0,
    targetStrideLengthM: target,
  });
  const t0 = Date.now();
  const lengths = [];
  proc.onParams(({ newCycleDiagnostics }) => {
    for (const d of newCycleDiagnostics || []) {
      if (Number.isFinite(d.strideLengthM) && !d.strideLengthClamped) {
        lengths.push(d.strideLengthM);
      }
    }
  });
  for (const s of samples) {
    proc.addSample({ ...s, timestampMs: t0 + Math.round(s.timestamp * 1000) });
  }
  proc.analyze();
  assert.ok(lengths.length >= 5, `ต้องมี cycle พอ ประเมินได้ ได้ ${lengths.length}`);
  const mean = lengths.reduce((a, b) => a + b, 0) / lengths.length;
  // กัน regression แบบ gravity-only (~0.15 m) และการ clamp เพดานทั้งชุด
  assert.ok(mean > 0.8, `mean stride=${mean.toFixed(3)} m ต่ำผิดปกติ — น่าจะไม่มี aHoriz / angle พัง`);
  assert.ok(mean < 1.7, `mean stride=${mean.toFixed(3)} m สูงผิดปกติ / ใกล้เพดาน clamp`);
  assert.ok(
    Math.abs(mean - target) / target < 0.25,
    `mean=${mean.toFixed(3)} ควรใกล้ target ${target} m (±25%); ไม่ใช่สำเนา oracle`,
  );
  assert.ok(proc.latestParams.strideLengthSignedM > 0.5, 'ขั้วเดินหน้าควรเป็นบวกบน demo แกนถูก');
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
  const { samples } = generateWalkingData({ numStrides: 12, strideTime: 1.05, seed: 42 });
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

test('regression: หลาย listener ต้องได้ newCycleDiagnostics ชุดเดียวกันเท่ากันทุกตัว (ไม่ใช่ตัวแรกได้ ตัวหลัง [])', () => {
  const proc = new GaitProcessor();
  const { samples } = generateWalkingData({ numStrides: 12, strideTime: 1.05, seed: 42 });
  const t0 = Date.now();
  for (const s of samples) {
    proc.addSample({ ...s, timestampMs: t0 + Math.round(s.timestamp * 1000) });
  }

  let listener1Count = null;
  let listener2Count = null;
  proc.onParams(({ newCycleDiagnostics }) => { listener1Count = newCycleDiagnostics?.length ?? -1; });
  proc.onParams(({ newCycleDiagnostics }) => { listener2Count = newCycleDiagnostics?.length ?? -1; });
  proc.analyze();

  assert.ok(listener1Count > 0, `listener แรกต้องได้ cycle ได้ ${listener1Count}`);
  assert.equal(listener2Count, listener1Count,
    `listener ที่สองต้องได้เท่ากับตัวแรก (${listener1Count}) ไม่ใช่ 0 จากการ drain ซ้ำ`);
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
  const { samples } = generateWalkingData({ numStrides: 12, strideTime: 1.05, seed: 42 });
  const t0 = Date.now();
  for (const s of samples) {
    proc.addSample({ ...s, timestampMs: t0 + Math.round(s.timestamp * 1000) });
  }
  // ไม่เรียก analyze() (ไม่ drain) — จำลอง edge case ที่ reset มาก่อน analyze ครั้งแรก
  proc.reset();
  assert.deepEqual(proc.pendingCycleDiagnostics, []);
});

test('🔴 addSample: ขาดแกน → ข้าม; ขาด timestamp → synthetic flag (ไม่ใช้ Date.now)', () => {
  const proc = new GaitProcessor();
  proc.addSample({ timestampMs: 100, ax: 0, ay: 0, az: 4096 }); // ไม่มี gyro
  assert.equal(proc.buffer.length, 0);
  assert.equal(proc.skippedIncompleteSampleCount, 1);

  proc.addSample({ timestampMs: 100, ax: 0, ay: 0, az: 4096, gx: 0, gy: 0, gz: 0 });
  assert.equal(proc.buffer.length, 1);
  assert.equal(proc.usedSyntheticTimestamps, false);
  const tAfterFirst = proc.buffer[0].timestampMs;

  proc.addSample({ ax: 0, ay: 0, az: 4096, gx: 0, gy: 0, gz: 0 });
  assert.equal(proc.usedSyntheticTimestamps, true);
  assert.ok(proc.missingTimestampCount >= 1);
  assert.equal(proc.buffer.length, 2);
  assert.ok(
    Math.abs(proc.buffer[1].timestampMs - (tAfterFirst + 10)) < 1e-9,
    `ต้อง interpolate จาก sample ก่อน +10ms ได้ ${proc.buffer[1].timestampMs} (ไม่ใช่ Date.now)`,
  );
});

test('🔴 resolveTimestampMs: relative board clock → session-relative ที่ reproducible', async () => {
  // ใช้ samples ชุดเดียวกันข้าม 2 รอบ — ถ้ายังผูก Date.now() ค่าจะต่างกันหลัง sleep
  const { samples } = generateWalkingData({ numStrides: 4, strideTime: 1.05, seed: 42 });

  function collectStarts() {
    const proc = new GaitProcessor();
    const starts = [];
    const seen = new Set();
    proc.onParams(({ params }) => {
      if (!params?.cycleKey || seen.has(params.cycleKey)) return;
      seen.add(params.cycleKey);
      starts.push(params.cycleStartTimestampMs);
    });
    let n = 0;
    for (const s of samples) {
      proc.addSample({ ...s, timestampMs: 5000 + Math.round(s.timestamp * 1000) });
      n += 1;
      if (n % 40 === 0) proc.analyze();
    }
    proc.analyze();
    return starts;
  }

  const a = collectStarts();
  await new Promise((r) => setTimeout(r, 25));
  const b = collectStarts();
  assert.ok(a.length >= 1);
  assert.deepEqual(a, b);
  assert.ok(a.every((t) => t < 1e11), `ต้องไม่เป็น wall-clock epoch ได้ ${a[0]}`);
});

test('🔴 resolveTimestampMs: micros wrap ต้องเดินหน้าต่อ ไม่รีเซ็ตเป็น 0', () => {
  const proc = new GaitProcessor();
  // จำลอง ESP32 micros()/1000 ใกล้ overflow แล้วห่อกลับ
  const nearWrap = 4_294_960;
  const times = [];
  for (let i = 0; i < 5; i += 1) {
    times.push(proc.resolveTimestampMs({ timestampMs: nearWrap + i * 10 }));
  }
  // wrap: 20, 30, 40...
  for (let i = 0; i < 5; i += 1) {
    times.push(proc.resolveTimestampMs({ timestampMs: 20 + i * 10 }));
  }

  for (let i = 1; i < times.length; i += 1) {
    assert.ok(
      times[i] > times[i - 1],
      `เวลาต้องเดินหน้า: t[${i - 1}]=${times[i - 1]} → t[${i}]=${times[i]}`,
    );
  }
  // ค่าหลัง wrap ต้องต่อเนื่องจากก่อน wrap (~ nearWrap-origin + deltas) ไม่ใช่ ~0
  assert.ok(times[5] > 40, `หลัง wrap ต้องไม่รีสตาร์ทใกล้ 0 ได้ ${times[5]}`);
});

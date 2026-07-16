import test from 'node:test';
import assert from 'node:assert/strict';

import { butterworthLowpass2, filtfiltButterworth2, lfilter } from './butterworth.js';
import {
  computeVelocity,
  computeFilteredVelocity,
  detectStanceIntervals,
  buildCyclesFromStanceIntervals,
  closeQuietGaps,
} from './velocityEventDetector.js';
import { FIXTURE } from './testFixtures.js';

function mulberry32(seed) {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function randn(rng) {
  const u = Math.max(1e-12, rng());
  const v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

/** trajectory ขาเดียว + optional Gaussian noise (mm) */
function buildAnkleTrajectory({
  fps = FIXTURE.SAMPLE_RATE,
  strideTimeS = 1.0,
  stancePct = 0.6,
  strideLengthM = 1.4,
  numStrides = 8,
  noiseMm = 0,
  seed = 1,
} = {}) {
  const rng = mulberry32(seed);
  const noiseM = noiseMm / 1000;
  const totalDurationS = numStrides * strideTimeS + 1;
  const n = Math.round(totalDurationS * fps);
  const t = [];
  const ankleX = [];
  for (let i = 0; i < n; i += 1) {
    const time = i / fps;
    t.push(time);
    const cyclePos = time / strideTimeS;
    const cycleIndex = Math.floor(cyclePos);
    const phase = cyclePos - cycleIndex;
    const footprintX = cycleIndex * strideLengthM;
    let x;
    if (phase < stancePct) {
      x = footprintX;
    } else {
      const p = (phase - stancePct) / (1 - stancePct);
      const ease = (1 - Math.cos(p * Math.PI)) / 2;
      x = footprintX + strideLengthM * ease;
    }
    ankleX.push(x + (noiseMm > 0 ? randn(rng) * noiseM : 0));
  }
  return {
    t,
    ankleX,
    expectedStanceS: stancePct * strideTimeS,
    expectedStrideM: strideLengthM,
    numStrides,
  };
}

test('butterworthLowpass2: ค่าสัมประสิทธิ์เสถียร (a0=1, sum(b)≈sum(a) ที่ DC)', () => {
  const { b, a } = butterworthLowpass2(6, 200);
  assert.equal(a[0], 1);
  const sumB = b.reduce((p, c) => p + c, 0);
  const sumA = a.reduce((p, c) => p + c, 0);
  assert.ok(Math.abs(sumB - sumA) < 1e-9, 'DC gain ควรเป็น 1');
});

test('filtfiltButterworth2: สัญญาณคงที่ยังคงที่ (zero-lag ไม่เบี่ยง mean)', () => {
  const signal = Array.from({ length: 100 }, () => 1.5);
  const out = filtfiltButterworth2(signal, 6, 100);
  assert.ok(out.every((v) => Math.abs(v - 1.5) < 1e-6));
});

test('lfilter: ใช้ได้กับ impulse (ไม่ throw)', () => {
  const { b, a } = butterworthLowpass2(6, 100);
  const impulse = Array.from({ length: 50 }, (_, i) => (i === 0 ? 1 : 0));
  const out = lfilter(b, a, impulse);
  assert.equal(out.length, 50);
  assert.ok(Number.isFinite(out[0]));
});

test('computeVelocity: ตำแหน่งคงที่ -> velocity เป็น 0 พอดี', () => {
  const t = [0, 1, 2, 3];
  const pos = [5, 5, 5, 5];
  const v = computeVelocity(t, pos);
  assert.ok(v.every((x) => Math.abs(x) < 1e-12));
});

test('computeVelocity: ตำแหน่งเปลี่ยนเชิงเส้น -> velocity คงที่เท่าความชัน', () => {
  const t = [0, 1, 2, 3, 4];
  const pos = [0, 2, 4, 6, 8];
  const v = computeVelocity(t, pos);
  for (const x of v) assert.ok(Math.abs(x - 2) < 1e-9);
});

test('closeQuietGaps: เติม gap สั้นระหว่าง quiet runs', () => {
  const mask = [true, true, false, true, true, false, false, false, true];
  const closed = closeQuietGaps(mask, 1);
  assert.deepEqual(closed.slice(0, 5), [true, true, true, true, true]);
  assert.equal(closed[5], false, 'gap ยาว 3 เฟรมต้องไม่ถูกปิดด้วย maxGap=1');
});

test('detectStanceIntervals: หา ground truth stance duration ได้แม่น (ไร้ noise)', () => {
  const { t, ankleX, expectedStanceS } = buildAnkleTrajectory({ noiseMm: 0 });
  const { velocity } = computeFilteredVelocity(t, ankleX);
  const intervals = detectStanceIntervals(t, velocity);

  assert.ok(intervals.length >= 6, `เจอ ${intervals.length} intervals`);
  const mid = intervals[Math.floor(intervals.length / 2)];
  const durationS = mid.toTimeS - mid.hsTimeS;
  // Butterworth ปัดมุม HS/TO → stance สั้นลงเล็กน้อยจาก GT คมชัด (ยอม ±8%)
  assert.ok(Math.abs(durationS - expectedStanceS) / expectedStanceS < 0.08,
    `stance duration=${durationS.toFixed(3)}s ควรใกล้ ${expectedStanceS.toFixed(3)}s`);
});

test('detectStanceIntervals: เสถียรในช่วง threshold กว้าง (ไร้ noise)', () => {
  const { t, ankleX, expectedStanceS } = buildAnkleTrajectory({ noiseMm: 0 });
  const { velocity } = computeFilteredVelocity(t, ankleX);
  for (const thresholdMps of [0.05, 0.1, 0.15, 0.2, 0.3]) {
    const intervals = detectStanceIntervals(t, velocity, {
      thresholdMps,
      exitThresholdMps: Math.max(thresholdMps + 0.07, thresholdMps * 1.4),
    });
    const mid = intervals[Math.floor(intervals.length / 2)];
    const durationS = mid.toTimeS - mid.hsTimeS;
    assert.ok(Math.abs(durationS - expectedStanceS) / expectedStanceS < 0.12,
      `thresholdMps=${thresholdMps}: duration=${durationS.toFixed(3)}s`);
  }
});

test('🟠 hysteresis/closing: เฟรมเดียว spike กลาง stance ไม่ตัด interval', () => {
  const fps = 100;
  const t = Array.from({ length: 100 }, (_, i) => i / fps);
  // quiet 0.2–0.7s แล้วจบด้วย swing (ต้องมี non-quiet ปิดท้าย ไม่งั้น open run ถูกทิ้ง)
  const v = t.map((time) => {
    if (time < 0.2 || time >= 0.75) return 1.0;
    return 0;
  });
  v[50] = 0.5; // spike เฟรมเดียวกลาง stance (~0.50s)

  const withoutClosing = detectStanceIntervals(t, v, {
    thresholdMps: 0.15,
    exitThresholdMps: 0.15,
    maxQuietGapFrames: 0,
    minStanceDurationS: 0.15,
  });
  assert.ok(withoutClosing.length >= 2, `ไม่มี closing ควรถูกตัดเป็น ≥2 ท่อน (ได้ ${withoutClosing.length})`);

  const withClosing = detectStanceIntervals(t, v, {
    thresholdMps: 0.15,
    exitThresholdMps: 0.22,
    maxQuietGapFrames: 3,
    minStanceDurationS: 0.15,
  });
  assert.equal(withClosing.length, 1, 'closing+hysteresis ต้องเชื่อม spike เฟรมเดียวกลับเป็น stance เดียว');
  assert.ok(withClosing[0].toTimeS - withClosing[0].hsTimeS > 0.45);
});

test('🔴 regression: 200fps + 1mm noise โดยไม่กรอง → พัง; กรองแล้วกู้กลับ', () => {
  const { t, ankleX, expectedStanceS, numStrides } = buildAnkleTrajectory({
    fps: 200,
    noiseMm: 1,
    strideTimeS: 1.0,
    stancePct: 0.6,
    strideLengthM: 1.4,
    numStrides: 8,
  });

  const rawV = computeVelocity(t, ankleX);
  const rawIntervals = detectStanceIntervals(t, rawV, { maxQuietGapFrames: 0, exitThresholdMps: 0.15 });
  const rawCycles = buildCyclesFromStanceIntervals(t, ankleX, rawIntervals);
  assert.ok(rawCycles.length <= numStrides - 2,
    `ไม่กรองควรพังหรือเสีย cycle (ได้ ${rawCycles.length}) — ยืนยันว่าปัญหา noise จริง`);

  const { velocity, smoothedPosition } = computeFilteredVelocity(t, ankleX, { cutoffHz: 6 });
  const intervals = detectStanceIntervals(t, velocity);
  const cycles = buildCyclesFromStanceIntervals(t, smoothedPosition, intervals);
  assert.ok(cycles.length >= numStrides - 2, `กรองแล้วได้ ${cycles.length} cycles (คาด ≥${numStrides - 2})`);
  const mid = intervals[Math.floor(intervals.length / 2)];
  const durationS = mid.toTimeS - mid.hsTimeS;
  assert.ok(Math.abs(durationS - expectedStanceS) / expectedStanceS < 0.08,
    `กรองแล้ว stance=${durationS.toFixed(3)}s ควรใกล้ ${expectedStanceS.toFixed(3)}s (±8%)`);
});

test('🔴 regression: 200fps + 2mm noise — ไม่กรองพังหมด; กรองแล้วได้ cycles', () => {
  const { t, ankleX, expectedStanceS, numStrides } = buildAnkleTrajectory({
    fps: 200,
    noiseMm: 2,
    strideTimeS: 1.0,
    stancePct: 0.6,
    numStrides: 8,
  });

  const rawCycles = buildCyclesFromStanceIntervals(
    t,
    ankleX,
    detectStanceIntervals(t, computeVelocity(t, ankleX), { maxQuietGapFrames: 0, exitThresholdMps: 0.15 }),
  );
  assert.ok(rawCycles.length <= 2, `ไม่กรอง+2mm ควรพังเกือบหมด (ได้ ${rawCycles.length})`);

  const { velocity, smoothedPosition } = computeFilteredVelocity(t, ankleX, { cutoffHz: 6 });
  const cycles = buildCyclesFromStanceIntervals(
    t,
    smoothedPosition,
    detectStanceIntervals(t, velocity),
  );
  assert.ok(cycles.length >= numStrides - 2, `กรองแล้วได้ ${cycles.length} cycles`);
  const midStance = cycles[Math.floor(cycles.length / 2)].stanceTimeS;
  assert.ok(Math.abs(midStance - expectedStanceS) / expectedStanceS < 0.12,
    `stance=${midStance.toFixed(3)}s ใกล้ GT ${expectedStanceS.toFixed(3)}s`);
});

test('buildCyclesFromStanceIntervals: stancePct/swingPct ตรง ground truth (ไร้ noise)', () => {
  const { t, ankleX, expectedStanceS } = buildAnkleTrajectory({
    noiseMm: 0,
    stancePct: FIXTURE.STANCE_PCT,
    strideTimeS: FIXTURE.STRIDE_TIME_S,
    strideLengthM: FIXTURE.STRIDE_LENGTH_M,
    numStrides: FIXTURE.NUM_STRIDES,
    fps: FIXTURE.SAMPLE_RATE,
  });
  const { velocity, smoothedPosition } = computeFilteredVelocity(t, ankleX);
  const intervals = detectStanceIntervals(t, velocity);
  const cycles = buildCyclesFromStanceIntervals(t, smoothedPosition, intervals);

  assert.ok(cycles.length >= FIXTURE.NUM_STRIDES - 3, `เจอ ${cycles.length} cycles`);
  for (const cycle of cycles) {
    assert.ok(Number.isFinite(cycle.stancePct));
    assert.ok(Math.abs(cycle.stancePct - FIXTURE.STANCE_PCT * 100) < 5);
    assert.ok(Math.abs(cycle.strideTimeS - FIXTURE.STRIDE_TIME_S) < 0.04);
    assert.ok(Math.abs(cycle.strideLengthM - FIXTURE.STRIDE_LENGTH_M) < 0.05);
    assert.ok(Math.abs(cycle.stanceTimeS - expectedStanceS) / expectedStanceS < 0.10);
  }
});

test('buildCyclesFromStanceIntervals: กรอง stride ผิดปกติทิ้ง (สั้น/ยาวเกิน min/max)', () => {
  const t = [0, 0.1, 0.2, 0.3, 0.4, 0.5];
  const forwardPosition = [0, 0, 1, 1, 2, 2];
  const stanceIntervals = [
    { hsIndex: 0, toIndex: 1, hsTimeS: 0, toTimeS: 0.1 },
    { hsIndex: 2, toIndex: 3, hsTimeS: 0.2, toTimeS: 0.3 },
    { hsIndex: 4, toIndex: 5, hsTimeS: 0.4, toTimeS: 0.5 },
  ];
  const cycles = buildCyclesFromStanceIntervals(t, forwardPosition, stanceIntervals);
  assert.equal(cycles.length, 0);
});

import test from 'node:test';
import assert from 'node:assert/strict';

import { computeVelocity, detectStanceIntervals, buildCyclesFromStanceIntervals } from './velocityEventDetector.js';
import { FIXTURE } from './testFixtures.js';

// สร้าง trajectory ขาเดียวตรง ๆ (ไม่ผ่าน CSV) เพื่อตรวจ velocity-threshold algorithm
// โดยตรง — ankle นิ่งสนิท (v=0 เป๊ะ) ช่วง phase < STANCE_PCT ของทุก stride ตาม fixture
// ทำให้รู้ ground truth ของ stance duration/timing แบบไม่กำกวม (ต่างจากวิธีเดิมที่ยืมรูป
// คลื่น angular velocity มาซึ่งไม่มีจุด "true TO" ที่ชัดเจน)
function buildAnkleTrajectory() {
  const { SAMPLE_RATE, STRIDE_TIME_S, STANCE_PCT, STRIDE_LENGTH_M, NUM_STRIDES } = FIXTURE;
  const totalDurationS = NUM_STRIDES * STRIDE_TIME_S + 1;
  const n = Math.round(totalDurationS * SAMPLE_RATE);
  const t = [];
  const ankleX = [];
  for (let i = 0; i < n; i += 1) {
    const time = i / SAMPLE_RATE;
    t.push(time);
    const cyclePos = time / STRIDE_TIME_S;
    const cycleIndex = Math.floor(cyclePos);
    const phase = cyclePos - cycleIndex;
    const footprintX = cycleIndex * STRIDE_LENGTH_M;
    if (phase < STANCE_PCT) {
      ankleX.push(footprintX);
    } else {
      const p = (phase - STANCE_PCT) / (1 - STANCE_PCT);
      const ease = (1 - Math.cos(p * Math.PI)) / 2;
      ankleX.push(footprintX + STRIDE_LENGTH_M * ease);
    }
  }
  return { t, ankleX };
}

test('computeVelocity: ตำแหน่งคงที่ -> velocity เป็น 0 พอดี', () => {
  const t = [0, 1, 2, 3];
  const pos = [5, 5, 5, 5];
  const v = computeVelocity(t, pos);
  assert.ok(v.every((x) => Math.abs(x) < 1e-12));
});

test('computeVelocity: ตำแหน่งเปลี่ยนเชิงเส้น -> velocity คงที่เท่าความชัน', () => {
  const t = [0, 1, 2, 3, 4];
  const pos = [0, 2, 4, 6, 8]; // slope = 2
  const v = computeVelocity(t, pos);
  for (const x of v) assert.ok(Math.abs(x - 2) < 1e-9);
});

test('detectStanceIntervals: หา ground truth stance duration ได้แม่น (ankle นิ่งสนิทตอน stance)', () => {
  const { t, ankleX } = buildAnkleTrajectory();
  const v = computeVelocity(t, ankleX);
  const intervals = detectStanceIntervals(t, v);

  assert.ok(intervals.length >= FIXTURE.NUM_STRIDES - 2, `เจอ ${intervals.length} intervals`);
  // เอา interval กลาง ๆ (เลี่ยง edge effect ต้น/ท้ายข้อมูล) มาเทียบ ground truth
  const mid = intervals[Math.floor(intervals.length / 2)];
  const durationS = mid.toTimeS - mid.hsTimeS;
  assert.ok(Math.abs(durationS - FIXTURE.EXPECTED_STANCE_TIME_S) < 0.02,
    `stance duration=${durationS.toFixed(3)}s ควรใกล้ ${FIXTURE.EXPECTED_STANCE_TIME_S.toFixed(3)}s (ground truth)`);
});

test('detectStanceIntervals: เสถียรในช่วง threshold กว้าง (0.02-0.3 m/s) — ยืนยันตามที่ทดสอบไว้ก่อนเขียนโค้ด', () => {
  const { t, ankleX } = buildAnkleTrajectory();
  const v = computeVelocity(t, ankleX);
  for (const thresholdMps of [0.02, 0.05, 0.1, 0.15, 0.2, 0.3]) {
    const intervals = detectStanceIntervals(t, v, { thresholdMps });
    const mid = intervals[Math.floor(intervals.length / 2)];
    const durationS = mid.toTimeS - mid.hsTimeS;
    assert.ok(Math.abs(durationS - FIXTURE.EXPECTED_STANCE_TIME_S) < 0.03,
      `thresholdMps=${thresholdMps}: duration=${durationS.toFixed(3)}s ควรใกล้ ${FIXTURE.EXPECTED_STANCE_TIME_S.toFixed(3)}s`);
  }
});

test('buildCyclesFromStanceIntervals: stancePct/swingPct ตรง ground truth (62%/38%) ทุก cycle เสมอ ไม่มี "unresolved"', () => {
  const { t, ankleX } = buildAnkleTrajectory();
  const v = computeVelocity(t, ankleX);
  const intervals = detectStanceIntervals(t, v);
  const cycles = buildCyclesFromStanceIntervals(t, ankleX, intervals);

  assert.ok(cycles.length >= FIXTURE.NUM_STRIDES - 3, `เจอ ${cycles.length} cycles`);
  for (const cycle of cycles) {
    assert.ok(Number.isFinite(cycle.stancePct), 'ต้องได้ stancePct เสมอ ไม่มี unresolved');
    assert.ok(Math.abs(cycle.stancePct - FIXTURE.STANCE_PCT * 100) < 3,
      `stancePct=${cycle.stancePct.toFixed(1)} ควรใกล้ ${(FIXTURE.STANCE_PCT * 100).toFixed(1)}`);
    assert.ok(Math.abs(cycle.strideTimeS - FIXTURE.STRIDE_TIME_S) < 0.02);
    assert.ok(Math.abs(cycle.strideLengthM - FIXTURE.STRIDE_LENGTH_M) < 0.05);
  }
});

test('buildCyclesFromStanceIntervals: กรอง stride ผิดปกติทิ้ง (สั้น/ยาวเกิน min/max)', () => {
  const t = [0, 0.1, 0.2, 0.3, 0.4, 0.5];
  const forwardPosition = [0, 0, 1, 1, 2, 2];
  const stanceIntervals = [
    { hsIndex: 0, toIndex: 1, hsTimeS: 0, toTimeS: 0.1 },
    { hsIndex: 2, toIndex: 3, hsTimeS: 0.2, toTimeS: 0.3 }, // strideTime=0.2s สั้นเกิน min 0.6s
    { hsIndex: 4, toIndex: 5, hsTimeS: 0.4, toTimeS: 0.5 },
  ];
  const cycles = buildCyclesFromStanceIntervals(t, forwardPosition, stanceIntervals);
  assert.equal(cycles.length, 0, 'stride ทั้งหมดสั้นกว่า minStrideTimeS default (0.6s) ต้องถูกกรองทิ้ง');
});

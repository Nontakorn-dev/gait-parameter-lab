import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

import {
  formatNum,
  formatSigned,
  formatDate,
  sensorLabel,
  computeCyclesBySensor,
  summarizeCycles,
  computeDistanceCheck,
  distanceCheckClass,
  computeGlobalT0Ms,
  FLAG_V_END_MPS,
  FLAG_ACCEL_DEVIATION_G,
} from './lib.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

function cycle(overrides = {}) {
  return {
    cycleKey: '1',
    cycleStartTimestampMs: 1000,
    strideLengthM: 1.0,
    strideLengthClamped: false,
    zuptCheck: { vStartPreDrift: 0, vEndPreDrift: 0, windowSource: 'post-peak-valley', zuptAccelDeviationG: 0.05 },
    sensorKey: 'LEFT_SHANK',
    side: 'L',
    ...overrides,
  };
}

test('formatNum/formatSigned/formatDate: จัดการ null/NaN เป็น "—" ไม่ throw', () => {
  assert.equal(formatNum(null), '—');
  assert.equal(formatNum(NaN), '—');
  assert.equal(formatNum(1.23456, 2), '1.23');
  assert.equal(formatSigned(null), '—');
  assert.equal(formatSigned(0.5), '+0.500');
  assert.equal(formatSigned(-0.5), '-0.500');
  assert.equal(formatDate(null), '—');
  assert.equal(formatDate('not-a-date'), 'not-a-date');
});

test('sensorLabel: map side เป็นชื่ออ่านง่าย, fallback เป็น sensorKey', () => {
  assert.equal(sensorLabel('LEFT_SHANK', 'L'), 'Left Shank');
  assert.equal(sensorLabel('RIGHT_SHANK', 'R'), 'Right Shank');
  assert.equal(sensorLabel('demo-1', null), 'demo-1');
});

test('summarizeCycles: null เมื่อไม่มี cycle เลย (ไฟล์เก่า/debug trace)', () => {
  assert.equal(summarizeCycles(undefined), null);
  assert.equal(summarizeCycles([]), null);
});

test('summarizeCycles: flag cycle ที่ |vEndPreDrift| หรือ zuptAccelDeviationG เกิน threshold', () => {
  const cycles = [
    cycle({ cycleKey: 'a', zuptCheck: { vStartPreDrift: 0, vEndPreDrift: -0.79, windowSource: 'post-peak-valley', zuptAccelDeviationG: 0.21 } }),
    cycle({ cycleKey: 'b', zuptCheck: { vStartPreDrift: 0, vEndPreDrift: 0.01, windowSource: 'post-peak-valley', zuptAccelDeviationG: 0.02 } }),
  ];
  const s = summarizeCycles(cycles);
  assert.equal(s.count, 2);
  assert.equal(s.flaggedCount, 1, 'มีแค่ cycle a ที่เกิน threshold (vEnd=0.79 > 0.15)');
  assert.ok(Math.abs(s.meanAbsVEnd - (0.79 + 0.01) / 2) < 1e-9);
});

test('summarizeCycles: นับ clampedCount และ sourceCounts ต่อ windowSource', () => {
  const cycles = [
    cycle({ strideLengthClamped: true, zuptCheck: { ...cycle().zuptCheck, windowSource: 'zero-crossing-fallback' } }),
    cycle({ strideLengthClamped: false, zuptCheck: { ...cycle().zuptCheck, windowSource: 'post-peak-valley' } }),
    cycle({ strideLengthClamped: false, zuptCheck: { ...cycle().zuptCheck, windowSource: 'post-peak-valley' } }),
  ];
  const s = summarizeCycles(cycles);
  assert.equal(s.clampedCount, 1);
  assert.deepEqual(s.sourceCounts, { 'zero-crossing-fallback': 1, 'post-peak-valley': 2 });
});

test('computeDistanceCheck: ไม่มี cycles หรือไม่มี groundTruth -> hasCheck=false ไม่ throw', () => {
  assert.equal(computeDistanceCheck({}).hasCheck, false);
  assert.equal(computeDistanceCheck({ cycles: [], groundTruth: { distanceM: 10 } }).hasCheck, false);
  assert.equal(computeDistanceCheck({ cycles: [cycle()] } /* no groundTruth */).hasCheck, false);
});

test('computeDistanceCheck: รวม strideLengthM ต่อเซนเซอร์ แล้วคำนวณ % error เทียบ groundTruth', () => {
  const cycles = [
    cycle({ sensorKey: 'LEFT_SHANK', side: 'L', strideLengthM: 1.0 }),
    cycle({ sensorKey: 'LEFT_SHANK', side: 'L', strideLengthM: 1.0 }),
    cycle({ sensorKey: 'RIGHT_SHANK', side: 'R', strideLengthM: 0.9 }),
  ];
  const result = computeDistanceCheck({ cycles, groundTruth: { distanceM: 2.0 } });
  assert.equal(result.hasCheck, true);
  const left = result.perSensor.find((p) => p.sensorKey === 'LEFT_SHANK');
  assert.ok(Math.abs(left.sumStrideLengthM - 2.0) < 1e-9);
  assert.ok(Math.abs(left.errorPct - 0) < 1e-9, 'sum ตรงกับ ground truth เป๊ะ = error 0%');
  const right = result.perSensor.find((p) => p.sensorKey === 'RIGHT_SHANK');
  assert.ok(Math.abs(right.errorPct - -55) < 1e-9, '0.9 vs 2.0 -> -55%');
});

test('distanceCheckClass: จัดระดับ ok/warn/bad ตาม % error', () => {
  assert.equal(distanceCheckClass(5), 'ok');
  assert.equal(distanceCheckClass(-9.9), 'ok');
  assert.equal(distanceCheckClass(15), 'warn');
  assert.equal(distanceCheckClass(-24), 'warn');
  assert.equal(distanceCheckClass(30), 'bad');
});

test('computeGlobalT0Ms: ใช้ min ของ sample.t_ms, fallback เป็น cycle timestamp ถ้าไม่มี samples', () => {
  assert.equal(computeGlobalT0Ms({ samples: [{ t_ms: 500 }, { t_ms: 100 }, { t_ms: 900 }] }), 100);
  assert.equal(computeGlobalT0Ms({ samples: [], cycles: [{ cycleStartTimestampMs: 300 }] }), 300);
  assert.equal(computeGlobalT0Ms({ samples: [], cycles: [] }), 0);
});

test('เทียบกับไฟล์ trace จริงที่ผู้ใช้ export มา: ไม่มี cycles เลย (schema เก่ากว่า v3) — ต้องไม่ throw', () => {
  const file = path.join(__dirname, 'gait-trace-20260715-215837.json');
  const data = JSON.parse(readFileSync(file, 'utf8'));

  assert.equal(data.schemaVersion, 2, 'ไฟล์ตัวอย่างนี้เก็บมาก่อน schema v3 (ไม่มี cycles[])');
  assert.equal('cycles' in data, false);

  // validator ต้องรับมือ edge case นี้ได้อย่างสง่างาม ไม่ throw
  assert.equal(summarizeCycles(data.cycles), null);
  const check = computeDistanceCheck(data);
  assert.equal(check.hasCheck, false);
  assert.equal(check.distanceM, 10, 'groundTruth.distanceM ยังอ่านได้ปกติแม้ไม่มี cycles');

  const t0 = computeGlobalT0Ms(data);
  assert.ok(Number.isFinite(t0) && t0 > 0, 'คำนวณ t0 จาก samples จริงได้');
  assert.equal(data.sampleCount, 6250);
});

test('threshold ที่ใช้จริงตรงกับตัวอย่างที่วิเคราะห์ไว้ในบทสนทนา (v_end~0.79, accelDev~0.21)', () => {
  assert.ok(0.79 > FLAG_V_END_MPS, 'เคสที่เคยยืนยันว่า ZUPT วางผิดจุดต้องโดน flag');
  assert.ok(0.21 < FLAG_ACCEL_DEVIATION_G, 'accelDev เดี่ยว ๆ ไม่เกิน threshold นี้ แต่ vEnd เกินพอให้ flag ได้');
});

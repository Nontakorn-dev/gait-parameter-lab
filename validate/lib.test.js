import test from 'node:test';
import assert from 'node:assert/strict';

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
  detectJsonKind,
  DEFAULT_FLAG_V_END_MPS,
  DEFAULT_FLAG_ACCEL_DEVIATION_G,
} from './lib.js';

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

// สร้าง object แทนการอ่านไฟล์ trace จริงที่ .gitignore ไว้ (validate/*.json) — ไฟล์นั้นมีแค่ในเครื่อง
// ผู้บันทึก ไม่อยู่ใน repo จึงทำให้เทสต์ fail ที่เครื่องอื่น/CI เสมอ (ยืนยันแล้ว: ENOENT ทันทีที่ไฟล์หาย)
// object นี้จำลอง shape ของไฟล์จริงที่เคยเจอ: schema v2, ไม่มี cycles[] เลย (ก่อน schema v3)
function buildLegacyV2TraceFixture() {
  return {
    schemaVersion: 2,
    recordedAt: '2026-07-15T14:57:54.601Z',
    exportedAt: '2026-07-15T14:58:37.119Z',
    app: 'gait-parameter-lab',
    appVersion: '1.0.0',
    firmwareBuildTag: null,
    source: 'live',
    hasSensorFrameRaw: true,
    truncated: false,
    sampleRateHzNominal: 100,
    sampleRateHzMeasured: 100.01,
    sampleRateHzBySensor: { LEFT_SHANK: 100.02, RIGHT_SHANK: 100 },
    droppedBySensor: { LEFT_SHANK: 0, RIGHT_SHANK: 0 },
    seqDiscontinuitiesBySensor: { LEFT_SHANK: 0, RIGHT_SHANK: 0 },
    sampleCount: 5,
    packetVersionBySensor: { LEFT_SHANK: 1, RIGHT_SHANK: 1 },
    calibrationBySensor: {},
    groundTruth: { distanceM: 10, stepCountManual: 5, notes: null },
    note: 'raw_accel_sensor / raw_gyro_sensor are PRE axis-remap...',
    // ไม่มี cycles / cycleCount / cyclesTruncated เลย — ตรงกับไฟล์จริงที่เคยเจอ
    samples: [
      { t_ms: 3843888.028, seq: 56622, sensorKey: 'LEFT_SHANK', side: 'L', sensorMount: 'shank', raw_accel_sensor: [-216, -4167, -766], raw_gyro_sensor: [293, 34, -12], raw_accel_canonical: [-216, -4167, -766], raw_gyro_canonical: [293, 34, -12] },
      { t_ms: 3843898.028, seq: 56623, sensorKey: 'LEFT_SHANK', side: 'L', sensorMount: 'shank', raw_accel_sensor: [-210, -4160, -760], raw_gyro_sensor: [290, 30, -10], raw_accel_canonical: [-210, -4160, -760], raw_gyro_canonical: [290, 30, -10] },
      { t_ms: 3843888.028, seq: 71001, sensorKey: 'RIGHT_SHANK', side: 'R', sensorMount: 'shank', raw_accel_sensor: [200, -4100, -700], raw_gyro_sensor: [280, 20, -8], raw_accel_canonical: [200, -4100, -700], raw_gyro_canonical: [280, 20, -8] },
    ],
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

test('summarizeCycles: flag cycle ที่ |vEndPreDrift| หรือ zuptAccelDeviationG เกิน threshold (ค่า default)', () => {
  const cycles = [
    cycle({ cycleKey: 'a', zuptCheck: { vStartPreDrift: 0, vEndPreDrift: -0.9, windowSource: 'post-peak-valley', zuptAccelDeviationG: 0.3 } }),
    cycle({ cycleKey: 'b', zuptCheck: { vStartPreDrift: 0, vEndPreDrift: 0.01, windowSource: 'post-peak-valley', zuptAccelDeviationG: 0.02 } }),
  ];
  const s = summarizeCycles(cycles);
  assert.equal(s.count, 2);
  assert.equal(s.flaggedCount, 1, 'มีแค่ cycle a ที่เกิน threshold default (0.9 > 0.15)');
});

test('summarizeCycles: threshold ปรับเองได้ (ไม่ผูกกับค่า default ตายตัว)', () => {
  const cycles = [cycle({ zuptCheck: { vStartPreDrift: 0, vEndPreDrift: 0.2, windowSource: 'x', zuptAccelDeviationG: 0.05 } })];
  const strict = summarizeCycles(cycles, { vEndMps: 0.1, accelDeviationG: 0.5 });
  const loose = summarizeCycles(cycles, { vEndMps: 0.5, accelDeviationG: 0.5 });
  assert.equal(strict.flaggedCount, 1, 'threshold เข้มกว่า (0.1) ต้อง flag ที่ vEnd=0.2');
  assert.equal(loose.flaggedCount, 0, 'threshold หลวมกว่า (0.5) ต้องไม่ flag');
});

test('regression: cycle ที่ vEndPreDrift/zuptAccelDeviationG เป็น null ต้องไม่ถูกนับเป็น 0 ในค่าเฉลี่ย', () => {
  // เคสที่เคย fail จริง: 2 cycle แย่ (vEnd 0.9/0.8) + 1 cycle ไม่มีข้อมูลเลย (null)
  const cycles = [
    cycle({ cycleKey: '1', zuptCheck: { vStartPreDrift: 0, vEndPreDrift: -0.9, windowSource: 'x', zuptAccelDeviationG: 0.3 } }),
    cycle({ cycleKey: '2', zuptCheck: { vStartPreDrift: 0, vEndPreDrift: -0.8, windowSource: 'x', zuptAccelDeviationG: 0.3 } }),
    cycle({ cycleKey: '3', zuptCheck: { vStartPreDrift: null, vEndPreDrift: null, windowSource: 'x', zuptAccelDeviationG: null } }),
  ];
  const s = summarizeCycles(cycles);

  // ก่อนแก้: (0.9+0.8+0)/3 = 0.5667 (null ถูกนับเป็น 0 = "ZUPT สมบูรณ์แบบ")
  // หลังแก้: เฉลี่ยจากแค่ 2 cycle ที่มีข้อมูลจริง = (0.9+0.8)/2 = 0.85
  assert.ok(Math.abs(s.meanAbsVEnd - 0.85) < 1e-9, `meanAbsVEnd=${s.meanAbsVEnd} ควรเป็น 0.85 ไม่ใช่ 0.567`);
  assert.equal(s.noZuptDataCount, 1, 'cycle ที่ null ต้องถูกนับแยกเป็น noZuptDataCount ไม่ใช่หายเงียบ ๆ');
  assert.equal(s.flaggedCount, 2, 'cycle ที่ไม่มีข้อมูลต้องไม่ถูกนับเป็น flagged (แยกจาก noZuptDataCount)');
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

test('regression: cycle ที่ strideLengthM เป็น null ต้องไม่ถูกนับเป็น 0 ในผลรวมระยะ (แยกเป็น noDataCount)', () => {
  const cycles = [
    cycle({ sensorKey: 'LEFT_SHANK', strideLengthM: 1.0 }),
    cycle({ sensorKey: 'LEFT_SHANK', strideLengthM: 1.0 }),
    cycle({ sensorKey: 'LEFT_SHANK', strideLengthM: null }),
  ];
  const result = computeDistanceCheck({ cycles, groundTruth: { distanceM: 2.0 } });
  const left = result.perSensor.find((p) => p.sensorKey === 'LEFT_SHANK');
  assert.ok(Math.abs(left.sumStrideLengthM - 2.0) < 1e-9, 'sum ต้องนับแค่ 2 cycle ที่มีข้อมูลจริง = 2.0 ไม่ใช่ปนศูนย์');
  assert.equal(left.noDataCount, 1);
  assert.equal(left.cycleCount, 3, 'cycleCount ยังนับรวมทุก cycle (แค่ sum ไม่รวม null)');
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

test('regression: computeGlobalT0Ms ไม่ throw RangeError กับ trace ยาวถึง maxSamples cap (300,000)', () => {
  // ยืนยันแล้วว่า Math.min(...array) ทำ "Maximum call stack size exceeded" ที่ ~150,000 element
  // (ทดสอบ raw Math.min(...) ตรง ๆ ก่อนแก้เจอ crash ที่ 150k และ 300k) — เดิน >4 นาที ที่ 2 เซนเซอร์
  // 100Hz ก็แตะ cap นี้ได้แล้ว จึงต้องทดสอบที่ขนาดจริงของ cap ไม่ใช่แค่ขนาดเล็ก
  for (const n of [150000, 300000]) {
    const samples = Array.from({ length: n }, (_, i) => ({ t_ms: n - i })); // min อยู่ท้ายอาเรย์โดยตั้งใจ
    assert.doesNotThrow(() => {
      const t0 = computeGlobalT0Ms({ samples });
      assert.equal(t0, 1, `n=${n}: t0 ควรเป็น min จริง (1) ไม่ใช่ NaN/Infinity`);
    }, `n=${n} ต้องไม่ throw RangeError`);
  }
});

test('เทียบกับ shape ของไฟล์ trace จริงที่เคยเจอ: ไม่มี cycles เลย (schema เก่ากว่า v3) — ต้องไม่ throw', () => {
  const data = buildLegacyV2TraceFixture();

  assert.equal(data.schemaVersion, 2);
  assert.equal('cycles' in data, false);

  // validator ต้องรับมือ edge case นี้ได้อย่างสง่างาม ไม่ throw
  assert.equal(summarizeCycles(data.cycles), null);
  const check = computeDistanceCheck(data);
  assert.equal(check.hasCheck, false);
  assert.equal(check.distanceM, 10, 'groundTruth.distanceM ยังอ่านได้ปกติแม้ไม่มี cycles');

  const t0 = computeGlobalT0Ms(data);
  assert.ok(Number.isFinite(t0) && t0 > 0, 'คำนวณ t0 จาก samples จริงได้');
});

// หมายเหตุความซื่อสัตย์: threshold ด้านล่างเป็นค่า default ที่ตั้งไว้ตอนพัฒนา *ยังไม่เคยผ่านการ
// ยืนยันด้วยข้อมูลเดินจริง* ตัวเลขตัวอย่าง (vEnd~0.79, accelDev~0.21) มาจาก "demo" สังเคราะห์ระหว่าง
// พัฒนา ไม่ใช่ ground truth — เทสต์นี้ตรวจแค่ "ตรรกะ flag ทำงานถูกกับค่า default ปัจจุบัน" เท่านั้น
// ไม่ได้อ้างว่า ZUPT วางผิดจุดถูกพิสูจน์แล้ว (นั่นต้องรอ trace จริงที่มี ground-truth distance)
test('threshold logic: ค่าที่เกิน default ถูก flag, ค่าที่ต่ำกว่าไม่ถูก flag (ทดสอบตรรกะ ไม่ใช่ยืนยัน ground truth)', () => {
  assert.ok(0.79 > DEFAULT_FLAG_V_END_MPS, 'ตรรกะ: ค่าที่เกิน default ต้องโดน flag ได้');
  assert.ok(0.05 < DEFAULT_FLAG_V_END_MPS, 'ตรรกะ: ค่าที่ต่ำกว่า default ต้องไม่ถูก flag');
  assert.ok(DEFAULT_FLAG_ACCEL_DEVIATION_G > 0, 'threshold ต้องเป็นบวก');
});

test('computeCyclesBySensor: จัดกลุ่มตาม sensorKey, ใช้ "_" เมื่อไม่มี sensorKey', () => {
  const map = computeCyclesBySensor([cycle({ sensorKey: 'A' }), cycle({ sensorKey: undefined })]);
  assert.deepEqual([...map.keys()].sort(), ['A', '_']);
});

test('detectJsonKind: mocap vs imu vs unknown', () => {
  assert.equal(detectJsonKind({ perSide: { L: {}, R: {} }, session: {} }), 'mocap');
  assert.equal(detectJsonKind({ schemaVersion: 3, samples: [] }), 'imu');
  assert.equal(detectJsonKind({ cycles: [] }), 'imu');
  assert.equal(detectJsonKind({ foo: 1 }), 'unknown');
  assert.equal(detectJsonKind(null), 'unknown');
});

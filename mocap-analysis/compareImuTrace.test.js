import test from 'node:test';
import assert from 'node:assert/strict';

import { generateWalkingData } from '../src/gait-dashboard/data/demoDataGenerator.js';
import { TraceRecorder } from '../src/gait-dashboard/data/traceRecorder.js';
import { GaitProcessor } from '../src/gait/gaitProcessor.js';
import {
  compareMocapToImu,
  reprocessImuTrace,
  summarizeMocapSide,
  summarizeImuSide,
  pairCyclesByTime,
} from './compareImuTrace.js';

function buildDemoTrace({ numStrides = 8, side = 'R', boardEpochMs = 5000 } = {}) {
  const { samples } = generateWalkingData({ numStrides, strideTime: 1.05 });
  const rec = new TraceRecorder({ sampleRateHzNominal: 100 });
  const proc = new GaitProcessor();

  rec.start();
  proc.onParams(({ newCycleDiagnostics }) => {
    if (!newCycleDiagnostics?.length) return;
    for (const d of newCycleDiagnostics) {
      rec.recordCycle({
        ...d,
        sensorKey: `demo-${side}`,
        side,
      });
    }
  });

  let n = 0;
  for (const s of samples) {
    // จำลอง firmware: t_ms = micros()/1000 นับจากบูต (relative ต่ำกว่า epoch)
    const timestampMs = boardEpochMs + Math.round(s.timestamp * 1000);
    const sample = {
      ...s,
      sensorKey: `demo-${side}`,
      side,
      sensorMount: 'shank',
      timestampMs,
      rawAccelSensor: [s.ax, s.ay, s.az],
      rawGyroSensor: [s.gx, s.gy, s.gz],
      raw_accel: [s.ax, s.ay, s.az],
      raw_gyro: [s.gx, s.gy, s.gz],
    };
    rec.record({
      sensorKey: sample.sensorKey,
      side: sample.side,
      sensorMount: sample.sensorMount,
      timestampMs: sample.timestampMs,
      seq: n,
      packetVersion: 1,
      rawAccelSensor: sample.rawAccelSensor,
      rawGyroSensor: sample.rawGyroSensor,
      raw_accel: sample.raw_accel,
      raw_gyro: sample.raw_gyro,
    });
    proc.addSample(sample);
    n += 1;
    if (n % 40 === 0) proc.analyze();
  }
  proc.analyze();
  rec.stop();

  return rec.buildTrace({
    groundTruth: { distanceM: 8, stepCountManual: numStrides * 2, notes: 'demo' },
  });
}

function fakeMocapFromImuCycles(imuCycles, side = 'R') {
  const cycles = imuCycles.map((c, i) => ({
    hsStartTimeS: Number.isFinite(c.cycleStartTimeS) ? c.cycleStartTimeS : i * (c.strideTimeS ?? 1.05),
    strideTimeS: c.strideTimeS ?? 1.05,
    strideLengthM: c.strideLengthM,
    cadenceSpm: c.cadenceSpm,
    walkingSpeedMps: c.walkingSpeedMps,
    stancePct: c.stancePct,
    peakShankAngleDeg: c.peakShankAngleDeg,
    ankleClearanceM: c.clearanceM ?? 0.05,
  }));

  return {
    perSide: {
      L: side === 'L' ? { cycles, summary: {} } : { cycles: [], summary: {} },
      R: side === 'R' ? { cycles, summary: {} } : { cycles: [], summary: {} },
    },
    bilateral: { trueCadenceSpm: 110, sameSideRepeats: 0 },
    session: {
      durationS: 10,
      pelvisNetForwardDisplacementM: cycles.reduce((a, c) => a + (c.strideLengthM || 0), 0),
    },
  };
}

test('summarizeMocapSide: เฉลี่ย stride/cadence จาก cycles', () => {
  const summary = summarizeMocapSide({
    cycles: [
      { strideLengthM: 1.0, cadenceSpm: 100, walkingSpeedMps: 1.0, stancePct: 60, peakShankAngleDeg: 30 },
      { strideLengthM: 1.2, cadenceSpm: 110, walkingSpeedMps: 1.2, stancePct: 62, peakShankAngleDeg: 32 },
    ],
  });
  assert.equal(summary.cycleCount, 2);
  assert.ok(Math.abs(summary.meanStrideLengthM - 1.1) < 1e-9);
  assert.ok(Math.abs(summary.meanCadenceSpm - 105) < 1e-9);
});

test('reprocessImuTrace: ได้ cycles จาก demo samples', () => {
  const trace = buildDemoTrace({ numStrides: 6, side: 'R' });
  const result = reprocessImuTrace(trace);
  assert.equal(result.ok, true);
  assert.equal(result.source, 'reprocess');
  assert.ok(result.bySide.R.length >= 3, `ควรได้หลาย cycle ได้ ${result.bySide.R.length}`);
  assert.ok(result.bySide.R.every((c) => Number.isFinite(c.strideLengthM)));
  assert.ok(result.bySide.R.every((c) => Number.isFinite(c.cadenceSpm)));
});

test('🟠 reprocess: stancePct ต้อง upgrade จาก null เป็นค่าเมื่อ temporal resolve', () => {
  const trace = buildDemoTrace({ numStrides: 8, side: 'R' });
  const result = reprocessImuTrace(trace);
  assert.equal(result.ok, true);
  const withStance = result.bySide.R.filter((c) => Number.isFinite(c.stancePct));
  assert.ok(
    withStance.length >= Math.floor(result.bySide.R.length * 0.5),
    `ควรได้ stancePct ส่วนใหญ่ ได้ ${withStance.length}/${result.bySide.R.length}`,
  );
});

test('compareMocapToImu: เมื่อ MoCap = IMU (synthetic) error ใกล้ 0', () => {
  const trace = buildDemoTrace({ numStrides: 6, side: 'R' });
  const imu = reprocessImuTrace(trace);
  const mocap = fakeMocapFromImuCycles(imu.bySide.R, 'R');
  const report = compareMocapToImu(mocap, trace);

  assert.equal(report.ok, true);
  const strideRow = report.sides.R.metrics.find((m) => m.metric === 'strideLengthM');
  assert.ok(strideRow.comparable);
  assert.ok(Math.abs(strideRow.errorPct) < 5, `errorPct=${strideRow.errorPct} ควรใกล้ 0 เมื่อ GT สร้างจาก IMU เดียวกัน`);
});

test('compareMocapToImu: ไม่มี samples แต่มี cycles[] ยังเทียบ stride ได้', () => {
  const mocap = {
    perSide: {
      L: { cycles: [] },
      R: {
        cycles: [
          { strideLengthM: 1.0, cadenceSpm: 100, walkingSpeedMps: 1, stancePct: 60, peakShankAngleDeg: 30 },
          { strideLengthM: 1.0, cadenceSpm: 100, walkingSpeedMps: 1, stancePct: 60, peakShankAngleDeg: 30 },
        ],
      },
    },
    bilateral: { sameSideRepeats: 0 },
    session: { pelvisNetForwardDisplacementM: 2.0 },
  };
  const imuTrace = {
    samples: [],
    cycles: [
      { side: 'R', sensorKey: 'x', strideLengthM: 1.1, strideLengthClamped: false },
      { side: 'R', sensorKey: 'x', strideLengthM: 1.1, strideLengthClamped: false },
    ],
    groundTruth: { distanceM: 2.0 },
  };

  const report = compareMocapToImu(mocap, imuTrace);
  assert.equal(report.ok, true);
  assert.equal(report.imuSource, 'trace-cycles');
  const strideRow = report.sides.R.metrics.find((m) => m.metric === 'strideLengthM');
  assert.ok(strideRow.comparable);
  assert.ok(Math.abs(strideRow.errorPct - 10) < 1e-6);
  const cadenceRow = report.sides.R.metrics.find((m) => m.metric === 'cadenceSpm');
  assert.equal(cadenceRow.comparable, false, 'cycles[] อย่างเดียวไม่มี cadence');
});

test('compareMocapToImu: ไม่มีข้อมูล IMU เลย → ok=false', () => {
  const report = compareMocapToImu(
    { perSide: { L: { cycles: [] }, R: { cycles: [] } }, bilateral: {}, session: {} },
    { samples: [], cycles: [] },
  );
  assert.equal(report.ok, false);
});

test('summarizeImuSide: นับ clamped', () => {
  const summary = summarizeImuSide([
    { strideLengthM: 1, strideLengthClamped: true },
    { strideLengthM: 1.1, strideLengthClamped: false },
  ]);
  assert.equal(summary.clampedCount, 1);
  assert.equal(summary.cycleCount, 2);
});

test('🔴 reprocess: cycleStartTimeS reproducible ข้ามรอบ (relative board clock)', () => {
  const trace = buildDemoTrace({ numStrides: 5, side: 'R', boardEpochMs: 8000 });
  const a = reprocessImuTrace(trace);
  const b = reprocessImuTrace(trace);
  assert.ok(a.bySide.R.length >= 2);
  assert.deepEqual(
    a.bySide.R.map((c) => c.cycleStartTimestampMs),
    b.bySide.R.map((c) => c.cycleStartTimestampMs),
  );
  assert.ok(a.bySide.R.every((c) => c.cycleStartTimestampMs < 1e11),
    'ต้องเป็น session-relative ไม่ใช่ wall-clock epoch');
});

test('🔴 distanceBySide: ขาที่ไม่มี cycle ต้องเป็น null ไม่ใช่ −100%', () => {
  const mocap = {
    perSide: {
      L: { cycles: [{ hsStartTimeS: 0, strideLengthM: 1, cadenceSpm: 100, walkingSpeedMps: 1, stancePct: 60, peakShankAngleDeg: 20 }] },
      R: { cycles: [] },
    },
    bilateral: { sameSideRepeats: 0 },
    session: { pelvisNetForwardDisplacementM: 1 },
  };
  const imuTrace = {
    samples: [],
    cycles: [{ side: 'L', sensorKey: 'x', strideLengthM: 1.0, strideLengthClamped: false, cycleStartTimestampMs: 1000 }],
    groundTruth: { distanceM: 10 },
  };
  const report = compareMocapToImu(mocap, imuTrace);
  assert.equal(report.session.distanceBySide.R.imuSumStrideLengthM, null);
  assert.equal(report.session.distanceBySide.R.vsGroundTruthPct, null);
});

test('🔴 pairCyclesByTime: จับคู่หลังมี lag คงที่ + ตัด unpaired', () => {
  const mocapCycles = [
    { hsStartTimeS: 1.0, strideLengthM: 1.2, cadenceSpm: 110, walkingSpeedMps: 1.1, stancePct: 60, peakShankAngleDeg: 30 },
    { hsStartTimeS: 2.1, strideLengthM: 1.25, cadenceSpm: 108, walkingSpeedMps: 1.15, stancePct: 61, peakShankAngleDeg: 31 },
    { hsStartTimeS: 3.2, strideLengthM: 1.22, cadenceSpm: 109, walkingSpeedMps: 1.12, stancePct: 60, peakShankAngleDeg: 29 },
  ];
  // IMU มีช่วงยืนนิ่งต้นทาง + lag 1.5s
  const imuCycles = [
    { cycleStartTimeS: 0.2, strideLengthM: 0.3, cadenceSpm: 50, walkingSpeedMps: 0.2, stancePct: 80, peakShankAngleDeg: 5 },
    { cycleStartTimeS: 2.5, strideLengthM: 1.21, cadenceSpm: 111, walkingSpeedMps: 1.1, stancePct: 59, peakShankAngleDeg: 30 },
    { cycleStartTimeS: 3.6, strideLengthM: 1.24, cadenceSpm: 107, walkingSpeedMps: 1.14, stancePct: 62, peakShankAngleDeg: 32 },
    { cycleStartTimeS: 4.7, strideLengthM: 1.20, cadenceSpm: 110, walkingSpeedMps: 1.11, stancePct: 60, peakShankAngleDeg: 28 },
    { cycleStartTimeS: 8.0, strideLengthM: 0.4, cadenceSpm: 40, walkingSpeedMps: 0.1, stancePct: 85, peakShankAngleDeg: 4 },
  ];
  const aligned = pairCyclesByTime(mocapCycles, imuCycles);
  assert.ok(aligned.pairs.length >= 2, `ควรจับคู่ได้หลายคู่ ได้ ${aligned.pairs.length}`);
  assert.ok(Math.abs(aligned.lagS - 1.5) < 0.3, `lag ควรใกล้ 1.5s ได้ ${aligned.lagS}`);
  // คู่ที่จับได้ไม่ควรรวม idle stride 0.3m ที่ต้น
  assert.ok(aligned.pairs.every((p) => p.imu.strideLengthM > 0.8));
});

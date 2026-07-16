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
} from './compareImuTrace.js';

function buildDemoTrace({ numStrides = 8, side = 'R' } = {}) {
  const { samples } = generateWalkingData({ numStrides, strideTime: 1.05 });
  const rec = new TraceRecorder({ sampleRateHzNominal: 100 });
  const proc = new GaitProcessor();
  const t0 = Date.now();

  rec.start();
  proc.onParams(({ params, newCycleDiagnostics }) => {
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
    const sample = {
      ...s,
      sensorKey: `demo-${side}`,
      side,
      sensorMount: 'shank',
      timestampMs: t0 + Math.round(s.timestamp * 1000),
      // demo ไม่มี sensor-frame — ใส่ canonical เป็นทั้งคู่
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
  const cycles = imuCycles.map((c) => ({
    hsStartTimeS: 0,
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

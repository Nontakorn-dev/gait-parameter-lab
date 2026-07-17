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
  estimateHsTimeLagS,
  estimateSignalLagS,
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
  // IMU มี idle ต้น/ท้าย + sync lag 0.35s (ภายใน DEFAULT_MAX_LAG_S=0.4 — กัน period alias)
  const imuCycles = [
    { cycleStartTimeS: 0.2, strideLengthM: 0.3, cadenceSpm: 50, walkingSpeedMps: 0.2, stancePct: 80, peakShankAngleDeg: 5 },
    { cycleStartTimeS: 1.35, strideLengthM: 1.21, cadenceSpm: 111, walkingSpeedMps: 1.1, stancePct: 59, peakShankAngleDeg: 30 },
    { cycleStartTimeS: 2.45, strideLengthM: 1.24, cadenceSpm: 107, walkingSpeedMps: 1.14, stancePct: 62, peakShankAngleDeg: 32 },
    { cycleStartTimeS: 3.55, strideLengthM: 1.20, cadenceSpm: 110, walkingSpeedMps: 1.11, stancePct: 60, peakShankAngleDeg: 28 },
    { cycleStartTimeS: 8.0, strideLengthM: 0.4, cadenceSpm: 40, walkingSpeedMps: 0.1, stancePct: 85, peakShankAngleDeg: 4 },
  ];
  const aligned = pairCyclesByTime(mocapCycles, imuCycles);
  assert.ok(aligned.pairs.length >= 2, `ควรจับคู่ได้หลายคู่ ได้ ${aligned.pairs.length}`);
  assert.ok(Math.abs(aligned.lagS - 0.35) < 0.08, `lag ควรใกล้ 0.35s ได้ ${aligned.lagS}`);
  assert.ok(aligned.pairs.every((p) => p.imu.strideLengthM > 0.8));
  assert.equal(aligned.timingMetricValid, false);
  assert.equal(aligned.meanTimeErrorS, null);
});

test('🔴 estimateHsTimeLagS: ไม่ให้ 0 ชนะ tie เมื่อ residual แย่กว่า (lag จริง 0.35s)', () => {
  const mocap = [0, 1, 2, 3].map((t) => ({ hsStartTimeS: t }));
  const imu = [0.35, 1.35, 2.35, 3.35].map((t) => ({ cycleStartTimeS: t }));
  const { lagS, matchCount } = estimateHsTimeLagS(mocap, imu, { matchToleranceS: 0.40 });
  assert.equal(matchCount, 4);
  assert.ok(Math.abs(lagS - 0.35) < 0.05, `ควรได้ ~0.35 ไม่ใช่ 0 ได้ ${lagS}`);
});

test('🔴 estimateSignalLagS: หา lag จากสัญญาณ + residual HS สะท้อน detector bias', () => {
  const dt = 0.005;
  const n = 2000; // 10s
  const mocapT = Array.from({ length: n }, (_, i) => i * dt);
  // asymmetric gait-like: sharp negative dip + broad positive hump
  const mocapY = mocapT.map((t) => {
    const phase = (t % 1.0);
    if (phase < 0.08) return -180 * Math.sin(Math.PI * phase / 0.08);
    if (phase < 0.55) return 0;
    return 320 * Math.sin(Math.PI * (phase - 0.55) / 0.45);
  });
  const trueLagS = 0.027;
  const imuY = mocapT.map((t) => {
    const u = t - trueLagS;
    const phase = ((u % 1.0) + 1.0) % 1.0;
    if (phase < 0.08) return -180 * Math.sin(Math.PI * phase / 0.08);
    if (phase < 0.55) return 0;
    return 320 * Math.sin(Math.PI * (phase - 0.55) / 0.45);
  });

  const signal = estimateSignalLagS(mocapT, mocapY, mocapT, imuY);
  assert.equal(signal.ok, true, signal.reason);
  assert.ok(Math.abs(signal.lagS - trueLagS) < 0.005, `xcorr lag=${signal.lagS} (want ~${trueLagS})`);

  const detectorBiasS = 0.10;
  const mocapCycles = [1, 2, 3, 4].map((k) => ({ hsStartTimeS: k }));
  const imuCycles = [1, 2, 3, 4].map((k) => ({
    cycleStartTimeS: k + trueLagS + detectorBiasS,
  }));
  const paired = pairCyclesByTime(mocapCycles, imuCycles, {
    lagS: signal.lagS,
    lagSource: 'signal-xcorr',
    matchToleranceS: 0.40,
  });
  assert.equal(paired.timingMetricValid, true);
  assert.ok(Math.abs(paired.meanTimeErrorS - detectorBiasS) < 0.05,
    `หลัง signal-align residual ควร ≈ detector bias ได้ ${paired.meanTimeErrorS}`);
});

test('🔴 estimateSignalLagS: maxLag กว้างเกินไป → period alias; default 0.4 ถูกต้อง', () => {
  const dt = 0.005;
  const n = 3000;
  const mocapT = Array.from({ length: n }, (_, i) => i * dt);
  const strideS = 1.0;
  const shape = (t) => {
    const phase = ((t % strideS) + strideS) % strideS;
    if (phase < 0.08) return -180 * Math.sin(Math.PI * phase / 0.08);
    if (phase < 0.55) return 0;
    return 320 * Math.sin(Math.PI * (phase - 0.55) / 0.45);
  };
  const mocapY = mocapT.map(shape);
  const trueLagS = 0.027;
  const imuY = mocapT.map((t) => shape(t - trueLagS));

  const bad = estimateSignalLagS(mocapT, mocapY, mocapT, imuY, { maxLagS: 8, dtS: dt });
  // ด้วย maxLag=8 มัก ambiguous หรือเลือกผิด period — อย่างน้อยต้องไม่เงียบเชื่อ lag ~2s
  if (bad.ok) {
    assert.ok(Math.abs(bad.lagS - trueLagS) < 0.05 || bad.ambiguous,
      `maxLag=8 ไม่ควรเชื่อ lag ผิดเงียบ ๆ ได้ ${bad.lagS}`);
  }

  const good = estimateSignalLagS(mocapT, mocapY, mocapT, imuY, { maxLagS: 0.4, dtS: dt });
  assert.equal(good.ok, true, good.reason);
  assert.ok(Math.abs(good.lagS - trueLagS) < 0.005, `default-range lag=${good.lagS}`);
});

test('🔴 estimateSignalLagS: polarity ถูก + lag ใหญ่ ต้องไม่ false-reject', () => {
  const dt = 0.005;
  const n = 2000;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((x) => Math.sin(2 * Math.PI * x) + 0.4 * Math.sin(4 * Math.PI * x) + 0.15 * Math.sin(6 * Math.PI * x));
  for (const lag of [0.30, 0.38]) {
    const shifted = t.map((tt) => (
      Math.sin(2 * Math.PI * (tt - lag))
      + 0.4 * Math.sin(4 * Math.PI * (tt - lag))
      + 0.15 * Math.sin(6 * Math.PI * (tt - lag))
    ));
    const result = estimateSignalLagS(t, y, t, shifted, { maxLagS: 0.4, dtS: dt });
    assert.equal(result.ok, true, `lag=${lag}s ต้องผ่าน ได้ reason=${result.reason} peak=${result.peakCorr}`);
    assert.ok(Math.abs(result.lagS - lag) < 0.01, `lag=${result.lagS} want ${lag}`);
    assert.ok(result.peakCorr > result.peakCorrMinus, 'peak(+) ต้องชนะ peak(−)');
  }
});

test('🔴 estimateSignalLagS: กลับขั้ว → polarity-mismatch (ไม่ใช้ corrAtZero)', () => {
  const dt = 0.005;
  const n = 2000;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const shape = (x) => {
    const phase = ((x % 1.0) + 1.0) % 1.0;
    if (phase < 0.08) return -180 * Math.sin(Math.PI * phase / 0.08);
    if (phase < 0.55) return 0;
    return 320 * Math.sin(Math.PI * (phase - 0.55) / 0.45);
  };
  const y = t.map(shape);
  const inverted = estimateSignalLagS(t, y, t, y.map((v) => -v), { maxLagS: 0.4, dtS: dt });
  assert.equal(inverted.ok, false);
  assert.equal(inverted.reason, 'polarity-mismatch');
  assert.ok(inverted.peakCorrMinus > inverted.peakCorr + 0.05);
});

test('🔴 estimateSignalLagS: กลับขั้ว+lag ไม่เงียบ flip / ไม่หลุดที่ corr(0)≈0', () => {
  const dt = 0.005;
  const n = 2000;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((x) => Math.sin(2 * Math.PI * x) + 0.4 * Math.sin(4 * Math.PI * x) + 0.15 * Math.sin(6 * Math.PI * x));
  for (const lag of [0.10, 0.20]) {
    const invertedShifted = t.map((tt) => -(
      Math.sin(2 * Math.PI * (tt - lag))
      + 0.4 * Math.sin(4 * Math.PI * (tt - lag))
      + 0.15 * Math.sin(6 * Math.PI * (tt - lag))
    ));
    const result = estimateSignalLagS(t, y, t, invertedShifted, { maxLagS: 0.4, dtS: dt });
    assert.equal(result.ok, false, `กลับขั้ว lag=${lag} ต้องไม่ ok ได้ ${JSON.stringify(result)}`);
    assert.equal(result.reason, 'polarity-mismatch');
    assert.ok(result.peakCorrMinus > result.peakCorr, `peak(−)=${result.peakCorrMinus} ต้อง > peak(+)=${result.peakCorr}`);
  }
});

test('🔴 estimateSignalLagS: ไม่ Math.min-spread crash กับ series ยาว 150k', () => {
  const n = 150_000;
  const t = new Float64Array(n);
  const y = new Float64Array(n);
  for (let i = 0; i < n; i += 1) {
    t[i] = i * 0.01;
    y[i] = Math.sin(i * 0.01);
  }
  // overlap สั้น artificially ด้วย options — แค่ให้ถึง min/max path ก่อน resample
  // (resample 150k@200Hz ทั้งช่วงจะช้าเกิน; ทดสอบว่าไม่ throw จาก spread)
  assert.doesNotThrow(() => {
    estimateSignalLagS(t, y, t, y, { maxLagS: 0.4, dtS: 0.02 });
  });
});

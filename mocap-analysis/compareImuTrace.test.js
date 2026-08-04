import test from 'node:test';
import assert from 'node:assert/strict';

import { generateWalkingData } from '../src/gait-dashboard/data/demoDataGenerator.js';
import { TraceRecorder } from '../src/gait-dashboard/data/traceRecorder.js';
import { GaitProcessor } from '../src/gait/gaitProcessor.js';
import { GAIT_ANALYZE_EVERY_SAMPLES } from '../src/gait/gaitRuntimeConfig.js';
import {
  compareMocapToImu,
  reprocessImuTrace,
  summarizeMocapSide,
  summarizeImuSide,
  pairCyclesByTime,
  estimateHsTimeLagS,
  estimateSignalLagS,
  estimateSignalOnsetS,
  resolveOverlappingImuCycles,
  extractImuGyroSeries,
} from './compareImuTrace.js';

function buildDemoTrace({ numStrides = 8, side = 'R', boardEpochMs = 5000, seed = 42 } = {}) {
  const { samples } = generateWalkingData({ numStrides, strideTime: 1.05, seed });
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
    bilateral: { trueCadenceSpm: 110, sameSideRepeats: 0, reliable: true },
    session: {
      durationS: 10,
      pelvisNetForwardDisplacementM: cycles.reduce((a, c) => a + (c.strideLengthM || 0), 0),
      pelvisNetMeaningful: true,
      forwardAxisMethod: 'net-start-end',
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
  const closed = result.bySide.R.filter((c) => !c.isOpenStride);
  assert.ok(closed.length >= 3, `ควรได้หลาย closed cycle ได้ ${closed.length}`);
  assert.ok(closed.every((c) => Number.isFinite(c.strideLengthM)), 'closed ต้องมี stride');
  assert.ok(closed.every((c) => Number.isFinite(c.cadenceSpm)), 'closed ต้องมี cadence จาก diagnostic');
});

test('🔴 reprocessImuTrace: diagnostic ไม่ตกหล่น + deterministic (mirror-live)', () => {
  const trace = buildDemoTrace({ numStrides: 8, side: 'R', seed: 11 });
  const a = reprocessImuTrace(trace);
  const b = reprocessImuTrace(trace);
  assert.equal(a.reprocessMode, 'mirror-live');
  const keys = (r) => r.bySide.R.map((x) => x.cycleKey).sort().join(',');
  assert.equal(keys(a), keys(b), 'รันซ้ำต้องได้ cycleKey ชุดเดียวกัน');
  assert.ok(a.bySide.R.filter((c) => !c.isOpenStride).length >= 5,
    `ควรได้หลาย closed cycle ได้ ${a.bySide.R.length}`);
  // ทุก closed cycle ต้องมาจาก diagnostic/params — ไม่หายเงียบ
  assert.ok(a.bySide.R.every((c) => c.cycleKey), 'ต้องมี cycleKey');
  assert.ok(
    a.bySide.R.some((c) => c.zuptCheck && Number.isFinite(c.zuptCheck.vEndPreDrift)),
    'diagnostic merge ต้องติด zuptCheck',
  );
});

test('🔴 reprocessImuTrace: default mirror-live ≡ forceStreaming (ต้องเท่ากัน)', () => {
  const trace = buildDemoTrace({ numStrides: 8, side: 'R', seed: 11 });
  const live = reprocessImuTrace(trace);
  const stream = reprocessImuTrace(trace, { analyzeEvery: 40, forceStreaming: true });
  const keySig = (r) => r.bySide.R
    .filter((c) => !c.isOpenStride)
    .map((c) => c.cycleKey)
    .sort()
    .join(',');
  assert.equal(keySig(live), keySig(stream), 'default ต้อง mirror live streaming');
  assert.ok(live.bySide.R.filter((c) => !c.isOpenStride).length >= 3);
});

test('🔴 reprocessImuTrace: analyzeEvery เป็นส่วนของสเปก — lock GAIT_ANALYZE_EVERY_SAMPLES=40', () => {
  assert.equal(GAIT_ANALYZE_EVERY_SAMPLES, 40,
    'ห้ามเปลี่ยนหลัง validate campaign โดยไม่รัน pilot ใหม่');
  const trace = buildDemoTrace({ numStrides: 8, side: 'R', seed: 11 });
  const def = reprocessImuTrace(trace);
  const explicit = reprocessImuTrace(trace, { analyzeEvery: GAIT_ANALYZE_EVERY_SAMPLES });
  const keySig = (r) => r.bySide.R.filter((c) => !c.isOpenStride).map((c) => c.cycleKey).sort().join(',');
  assert.equal(keySig(def), keySig(explicit), 'default ต้องใช้ GAIT_ANALYZE_EVERY_SAMPLES');
});

test('🟠 reprocess: stancePct ต้อง upgrade จาก null เป็นค่าเมื่อ temporal resolve', () => {
  const trace = buildDemoTrace({ numStrides: 8, side: 'R', seed: 42 });
  const result = reprocessImuTrace(trace);
  assert.equal(result.ok, true);
  const withStance = result.bySide.R.filter((c) => Number.isFinite(c.stancePct));
  assert.ok(
    withStance.length >= Math.floor(result.bySide.R.length * 0.5),
    `ควรได้ stancePct ส่วนใหญ่ ได้ ${withStance.length}/${result.bySide.R.length}`,
  );
});

test('🔴 generateWalkingData: seed เดียวกัน → series เหมือนกัน (ไม่ flaky)', () => {
  const a = generateWalkingData({ numStrides: 5, strideTime: 1.05, seed: 99 });
  const b = generateWalkingData({ numStrides: 5, strideTime: 1.05, seed: 99 });
  const c = generateWalkingData({ numStrides: 5, strideTime: 1.05, seed: 100 });
  assert.equal(a.samples.length, b.samples.length);
  assert.deepEqual(
    a.samples.map((s) => [s.ax, s.ay, s.az, s.gx]),
    b.samples.map((s) => [s.ax, s.ay, s.az, s.gx]),
  );
  assert.notDeepEqual(
    a.samples.map((s) => s.gx),
    c.samples.map((s) => s.gx),
  );
});

test('🔴 compare mode: --lag → external-lag ไม่ใช่ hs-event-lag', () => {
  const trace = buildDemoTrace({ numStrides: 6, side: 'R', seed: 7 });
  const imu = reprocessImuTrace(trace);
  const mocap = fakeMocapFromImuCycles(imu.bySide.R, 'R');
  const gyro = extractImuGyroSeries(trace, 'R', {});
  mocap.perSide.R.signals = {
    tS: gyro.tS,
    shankAngularVelocityDps: gyro.gx,
    angleSource: 'legacy-ankle',
  };
  const report = compareMocapToImu(mocap, trace, { align: { lagS: 0 }, minAgreementPairs: 1 });
  assert.equal(report.sides.R.alignment.lagSource, 'external');
  assert.equal(report.sides.R.alignment.mode, 'external-lag');
  assert.equal(report.sides.R.alignment.syncTrusted, true);
});

test('compareMocapToImu: เมื่อ MoCap = IMU (synthetic smoke) + --lag → primary error ใกล้ 0', () => {
  // ⚠️ smoke plumbing เท่านั้น — ไม่ใช่ independent MoCap GT
  const trace = buildDemoTrace({ numStrides: 6, side: 'R' });
  const imu = reprocessImuTrace(trace);
  const mocap = fakeMocapFromImuCycles(imu.bySide.R, 'R');
  const gyro = extractImuGyroSeries(trace, 'R', {});
  mocap.perSide.R.signals = {
    tS: gyro.tS,
    shankAngularVelocityDps: gyro.gx,
    angleSource: 'legacy-ankle',
  };
  const report = compareMocapToImu(mocap, trace, { align: { lagS: 0 }, minAgreementPairs: 1 });

  assert.equal(report.ok, true);
  const strideRow = report.sides.R.metrics.find((m) => m.metric === 'strideLengthM');
  assert.ok(strideRow.comparable, 'ต้องมี --lag + signal verify จึง comparable');
  assert.ok(Math.abs(strideRow.errorPct) < 5, `errorPct=${strideRow.errorPct}`);
  const stanceRow = report.sides.R.metrics.find((m) => m.metric === 'stancePct');
  assert.equal(stanceRow.comparable, false, 'stance เป็น exploratory');
  const peakRow = report.sides.R.metrics.find((m) => m.metric === 'peakShankAngleDeg');
  assert.equal(peakRow.comparable, false, 'peak เป็น exploratory');
});

test('🔴 compareMocapToImu: --lag โดยไม่มี signals → syncTrusted=false (fail-closed)', () => {
  const trace = buildDemoTrace({ numStrides: 6, side: 'R', seed: 19 });
  const imu = reprocessImuTrace(trace);
  const mocap = fakeMocapFromImuCycles(imu.bySide.R, 'R');
  // cycles only — ไม่มี ω ให้ verify heel-tap
  assert.equal(mocap.perSide.R.signals, undefined);

  const report = compareMocapToImu(mocap, trace, {
    align: { lagS: 0, lagSource: 'manual' },
    minAgreementPairs: 1,
  });
  assert.equal(report.sides.R.alignment.forcedLagVerifiedBy, 'none');
  assert.equal(report.sides.R.alignment.syncTrusted, false);
  assert.equal(report.sides.R.validationPublishable, false);
  const strideRow = report.sides.R.metrics.find((m) => m.metric === 'strideLengthM');
  assert.equal(strideRow.comparable, false);
  assert.ok(
    report.warnings.some((w) => w.includes('ไม่มี MoCap signals') || w.includes('fail-closed')),
    'ต้องเตือนว่า verify ไม่ได้',
  );
});

test('compareMocapToImu: ไม่มี sync → comparable=false (fail-closed)', () => {
  const trace = buildDemoTrace({ numStrides: 6, side: 'R' });
  const imu = reprocessImuTrace(trace);
  const mocap = fakeMocapFromImuCycles(imu.bySide.R, 'R');
  // ไม่มี signals + ไม่ส่ง --lag → ห้ามเผยแพร่ error%
  const report = compareMocapToImu(mocap, trace);
  assert.equal(report.validationPublishable, false);
  const strideRow = report.sides.R.metrics.find((m) => m.metric === 'strideLengthM');
  assert.equal(strideRow.comparable, false);
});

test('compareMocapToImu: ไม่มี samples แต่มี cycles[] — informational เท่านั้น', () => {
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
    session: { pelvisNetForwardDisplacementM: 2.0, pelvisNetMeaningful: true, forwardAxisMethod: 'net-start-end' },
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
  assert.equal(report.validationPublishable, false);
  const strideRow = report.sides.R.metrics.find((m) => m.metric === 'strideLengthM');
  assert.equal(strideRow.comparable, false, 'ไม่มี sync/pairing ที่ trusted');
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

test('🔴 resolveOverlappingImuCycles: เก็บอันสั้น ตัด double-stride ทับซ้อน', () => {
  const entries = [
    { cycleKey: 'a', cycleStartTimeS: 1.0, strideTimeS: 3.25, isOpenStride: false, strideLengthM: 1.8 },
    { cycleKey: 'b', cycleStartTimeS: 2.5, strideTimeS: 1.75, isOpenStride: false, strideLengthM: 1.1 },
  ];
  const out = resolveOverlappingImuCycles(entries);
  assert.equal(out.length, 1);
  assert.equal(out[0].cycleKey, 'b');
});

test('🔴 summarizeImuSide: ไม่นับ open เป็น cycleCount', () => {
  const summary = summarizeImuSide([
    { strideLengthM: 1, isOpenStride: false, strideLengthClamped: false },
    { strideLengthM: null, isOpenStride: true, strideLengthClamped: false },
    { strideLengthM: 1.1, isOpenStride: false, strideLengthClamped: true },
  ]);
  assert.equal(summary.cycleCount, 2);
  assert.equal(summary.openStrideCount, 1);
  assert.equal(summary.clampedCount, 1);
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
    session: { pelvisNetForwardDisplacementM: 1, pelvisNetMeaningful: true, forwardAxisMethod: 'net-start-end' },
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
  const { lagS, matchCount, ok } = estimateHsTimeLagS(mocap, imu, { matchToleranceS: 0.40 });
  assert.equal(matchCount, 4);
  assert.equal(ok, true);
  assert.ok(Math.abs(lagS - 0.35) < 0.05, `ควรได้ ~0.35 ไม่ใช่ 0 ได้ ${lagS}`);
});

test('🔴 estimateHsTimeLagS: lag นอก fine window โดยไม่มี coarse → ไม่เงียบ alias', () => {
  const stride = 1.1;
  const mocap = Array.from({ length: 10 }, (_, i) => ({ hsStartTimeS: i * stride }));
  for (const trueLag of [1.2, 2.0, 3.3]) {
    const imu = mocap.map((c) => ({ cycleStartTimeS: c.hsStartTimeS + trueLag }));
    const result = estimateHsTimeLagS(mocap, imu, { rivalScanMaxLagS: 5, matchToleranceS: 0.40 });
    assert.equal(result.ok, false, `trueLag=${trueLag} ต้องไม่ ok เงียบ ได้ lagS=${result.lagS}`);
    assert.equal(result.periodAliasRisk, true, `trueLag=${trueLag} ต้อง flag alias`);
    assert.equal(result.reason, 'period-alias-rivals');
  }
});

test('🔴 estimateHsTimeLagS: มี coarseLag → จับ lag ใหญ่ได้แม้มี rival', () => {
  const stride = 1.1;
  const mocap = Array.from({ length: 10 }, (_, i) => ({ hsStartTimeS: i * stride }));
  for (const trueLag of [0.10, 1.20, 2.00, 3.30]) {
    const imu = mocap.map((c) => ({ cycleStartTimeS: c.hsStartTimeS + trueLag }));
    const result = estimateHsTimeLagS(mocap, imu, {
      coarseLagS: trueLag,
      rivalScanMaxLagS: 5,
      matchToleranceS: 0.40,
    });
    assert.equal(result.ok, true, `trueLag=${trueLag} reason=${result.reason}`);
    assert.ok(Math.abs(result.lagS - trueLag) < 0.05, `ได้ ${result.lagS} want ${trueLag}`);
  }
});

test('🔴 estimateHsTimeLagS: step บน i อย่างเดียว — ไม่พลาด lag ที่ c ไม่หาร step', () => {
  // 30 MoCap HS + IMU มี HS เกิน 3 ตอนต้น → length=33 → step=floor(33/12)=2
  // lag จริง 8.0; ถ้า step ทั้ง i,j จะพลาด candidate แล้วได้ alias 7.0
  const stride = 1.0;
  const mocap = Array.from({ length: 30 }, (_, i) => ({ hsStartTimeS: i * stride }));
  const trueLag = 8.0;
  const extra = [0.2, 0.5, 0.8].map((t) => ({ cycleStartTimeS: t }));
  const imu = [
    ...extra,
    ...mocap.map((c) => ({ cycleStartTimeS: c.hsStartTimeS + trueLag })),
  ];
  assert.equal(imu.length, 33);
  const result = estimateHsTimeLagS(mocap, imu, {
    coarseLagS: trueLag,
    rivalScanMaxLagS: 12,
    matchToleranceS: 0.40,
  });
  assert.ok(Math.abs(result.lagS - trueLag) < 0.05, `ได้ ${result.lagS} want ${trueLag}`);
  assert.equal(result.ok, true);
});

test('🔴 estimateHsTimeLagS: periodAliasRisk + coarse → ok จับคู่ได้ แต่ agreement ต้องตัด', () => {
  const stride = 1.0;
  const mocap = Array.from({ length: 30 }, (_, i) => ({ hsStartTimeS: i * stride }));
  const trueLag = 8.0;
  const imu = [
    ...[0.2, 0.5, 0.8].map((t) => ({ cycleStartTimeS: t })),
    ...mocap.map((c) => ({ cycleStartTimeS: c.hsStartTimeS + trueLag })),
  ];
  const result = estimateHsTimeLagS(mocap, imu, {
    coarseLagS: trueLag,
    rivalScanMaxLagS: 12,
    matchToleranceS: 0.40,
  });
  // stride คงที่เป๊ะ → rivals ที่ ±1s มักมี — periodAliasRisk ต้อง flag
  assert.equal(result.periodAliasRisk, true);
  assert.equal(result.ok, true, 'coarse ยังอนุญาต exploratory pairing');
});

test('🔴 pairCyclesByTime: HS alias → ไม่จับคู่ (ดีกว่าคู่ผิด stride)', () => {
  const stride = 1.1;
  const mocap = Array.from({ length: 10 }, (_, i) => ({
    hsStartTimeS: i * stride,
    strideLengthM: 1.2,
    cadenceSpm: 110,
    walkingSpeedMps: 1.1,
    stancePct: 60,
    peakShankAngleDeg: 30,
  }));
  const trueLag = 1.2;
  const imu = mocap.map((c) => ({
    cycleStartTimeS: c.hsStartTimeS + trueLag,
    strideLengthM: 1.21,
    cadenceSpm: 109,
    walkingSpeedMps: 1.1,
    stancePct: 59,
    peakShankAngleDeg: 31,
  }));
  const aligned = pairCyclesByTime(mocap, imu, { rivalScanMaxLagS: 5 });
  assert.equal(aligned.lagOk, false);
  assert.equal(aligned.periodAliasRisk, true);
  assert.equal(aligned.pairs.length, 0, 'ห้ามจับคู่ผิด stride เงียบ');
  assert.equal(aligned.lagSource, 'hs-event');
});

test('🔴 pairCyclesByTime: ส่ง coarseLag จาก onset → จับคู่ถูกแม้ xcorr ปฏิเสธ', () => {
  const stride = 1.1;
  const mocap = Array.from({ length: 10 }, (_, i) => ({
    hsStartTimeS: i * stride,
    strideLengthM: 1.2,
    cadenceSpm: 110,
    walkingSpeedMps: 1.1,
    stancePct: 60,
    peakShankAngleDeg: 30,
  }));
  const trueLag = 2.0;
  const imu = mocap.map((c) => ({
    cycleStartTimeS: c.hsStartTimeS + trueLag,
    strideLengthM: 1.21,
    cadenceSpm: 109,
    walkingSpeedMps: 1.1,
    stancePct: 59,
    peakShankAngleDeg: 31,
  }));
  const aligned = pairCyclesByTime(mocap, imu, { coarseLagS: trueLag, rivalScanMaxLagS: 5 });
  assert.equal(aligned.lagOk, true);
  assert.ok(aligned.pairs.length >= 8, `ได้ ${aligned.pairs.length}`);
  assert.ok(Math.abs(aligned.lagS - trueLag) < 0.05);
});

function gaitLike(t, stride = 1.1) {
  const phase = (((t % stride) + stride) % stride) / stride;
  if (phase < 0.08) return -180 * Math.sin((Math.PI * phase) / 0.08);
  if (phase < 0.55) return 0;
  return 320 * Math.sin((Math.PI * (phase - 0.55)) / 0.45);
}

/** gait จริงขึ้น: stance ripple ~86 dps / swing ~350 — คร่อม maxAbs×0.15/0.25 พอดี */
function gaitLikeRealistic(t, stride = 1.1, { swingPeak = 350, stancePeak = 86 } = {}) {
  const phase = (((t % stride) + stride) % stride) / stride;
  if (phase < 0.08) return -180 * Math.sin((Math.PI * phase) / 0.08);
  if (phase < 0.55) {
    const p = (phase - 0.08) / 0.47;
    return stancePeak * Math.sin(Math.PI * p);
  }
  return swingPeak * Math.sin((Math.PI * (phase - 0.55)) / 0.45);
}

function withLeadingQuiet(walkFn, quietUntilS) {
  return (t) => (t < quietUntilS ? 1.5 * Math.sin(2 * Math.PI * t * 2.3) : walkFn(t - quietUntilS));
}

test('🔴 estimateSignalOnsetS: leading quiet + stance ripple → ยิงใกล้ walk start', () => {
  const dt = 0.005;
  const quietUntil = 2.0;
  const n = Math.round(8 / dt);
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map(withLeadingQuiet((tau) => gaitLikeRealistic(tau, 1.1), quietUntil));
  const onset = estimateSignalOnsetS(t, y);
  assert.ok(Number.isFinite(onset), 'ต้องหา onset ได้ — ไม่ใช่ null จาก gate ขัด threshold');
  assert.ok(Math.abs(onset - quietUntil) < 0.25, `onset=${onset} ควรใกล้ walkStart=${quietUntil}`);
});

test('🔴 estimateSignalOnsetS: ไม่มี leading quiet → null (ไม่หลอก phase ในก้าวแรก)', () => {
  const dt = 0.005;
  const n = Math.round(6 / dt);
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((tt) => gaitLikeRealistic(tt, 1.1));
  assert.equal(estimateSignalOnsetS(t, y), null);
});

test('🔴 estimateSignalLagS: leading quiet → coarseFromOnset เองได้ ไม่ต้องฉีด coarseLagS', () => {
  const dt = 0.005;
  const quietUntil = 2.0;
  const n = Math.round(12 / dt);
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const walk = (tau) => gaitLikeRealistic(tau, 1.1);
  const y = t.map(withLeadingQuiet(walk, quietUntil));
  for (const lag of [0.10, 0.55, 1.20, 2.00]) {
    // IMU = mocap เลื่อน — quiet ของ IMU ยาวกว่าตาม lag
    const s = t.map((tt) => (tt < quietUntil + lag ? 1.5 * Math.sin(2 * Math.PI * tt * 2.3) : walk(tt - quietUntil - lag)));
    const result = estimateSignalLagS(t, y, t, s, { dtS: dt, rivalScanMaxLagS: 5 });
    assert.equal(result.coarseFromOnset, true, `lag=${lag} ต้องได้ coarse จาก onset ไม่ใช่ฉีด`);
    assert.equal(result.ok, true, `lag=${lag} ok reason=${result.reason}`);
    assert.ok(Math.abs(result.lagS - lag) < 0.03, `lag=${lag} ได้ ${result.lagS}`);
    assert.equal(result.reason, null, `ok=true ต้อง reason=null ไม่ใช่ period-alias-rivals ได้ ${result.reason}`);
  }
});

test('🔴 estimateSignalLagS: หา lag จากสัญญาณ + residual HS สะท้อน detector bias', () => {
  const dt = 0.005;
  const n = 2000;
  const mocapT = Array.from({ length: n }, (_, i) => i * dt);
  const mocapY = mocapT.map((t) => gaitLike(t, 1.0));
  const trueLagS = 0.027;
  const imuY = mocapT.map((t) => gaitLike(t - trueLagS, 1.0));

  const signal = estimateSignalLagS(mocapT, mocapY, mocapT, imuY, { coarseLagS: trueLagS });
  assert.equal(signal.ok, true, signal.reason);
  assert.ok(Math.abs(signal.lagS - trueLagS) < 0.005, `xcorr lag=${signal.lagS}`);

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
  assert.ok(Math.abs(paired.meanTimeErrorS - detectorBiasS) < 0.05);
});

test('🔴 estimateSignalLagS: lag นอก fine window โดยไม่มี onset → ไม่เงียบ alias', () => {
  const dt = 0.005;
  const n = 4000;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((tt) => gaitLike(tt, 1.1));
  for (const lag of [1.2, 2.0]) {
    const s = t.map((tt) => gaitLike(tt - lag, 1.1));
    // ไม่ส่ง coarseLagS = จำลองกด Record คนละเวลาโดยไม่มี heel-tap/onset ต่างกัน
    const result = estimateSignalLagS(t, y, t, s, { dtS: dt, rivalScanMaxLagS: 5 });
    assert.equal(result.ok, false, `lag จริง ${lag}s ต้องไม่ ok เงียบ ได้ ${result.lagS} reason=${result.reason}`);
    assert.ok(
      result.periodAliasRisk || result.reason === 'period-alias-rivals',
      `ต้อง flag period alias ได้ ${JSON.stringify(result)}`,
    );
  }
});

test('🔴 estimateSignalLagS: coarseLag ถูก → จับ lag ใหญ่/ครึ่ง stride ได้ ไม่ false polarity', () => {
  const dt = 0.005;
  const n = 4000;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((tt) => gaitLike(tt, 1.1));
  for (const lag of [0.10, 0.30, 0.55, 1.20, 2.00]) {
    const s = t.map((tt) => gaitLike(tt - lag, 1.1));
    const result = estimateSignalLagS(t, y, t, s, { dtS: dt, coarseLagS: lag, rivalScanMaxLagS: 5 });
    assert.equal(result.ok, true, `lag=${lag} reason=${result.reason}`);
    assert.ok(Math.abs(result.lagS - lag) < 0.02, `ได้ ${result.lagS} want ${lag}`);
    assert.notEqual(result.reason, 'polarity-mismatch');
  }
});

test('🔴 estimateSignalLagS: --lag บังคับ sync', () => {
  const dt = 0.005;
  const n = 2000;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((tt) => gaitLike(tt, 1.1));
  const s = t.map((tt) => gaitLike(tt - 1.2, 1.1));
  const result = estimateSignalLagS(t, y, t, s, { lagS: 1.2, dtS: dt });
  assert.equal(result.ok, true);
  assert.equal(result.lagS, 1.2);
  assert.ok(result.peakCorr > 0.9);
});

test('🔴 estimateSignalLagS: --lag ใหญ่กว่า overlap ดิบยังตรวจ corr ได้', () => {
  // MoCap สั้น ~12s; IMU ยาวกว่าและเลื่อน 8s — overlap ดิบไม่ครอบคลุม lag แต่หน้าต่างเลื่อนต้องได้ corr
  const dt = 0.01;
  const mocapN = Math.round(12 / dt);
  const imuN = Math.round(25 / dt);
  const mocapT = Array.from({ length: mocapN }, (_, i) => i * dt);
  const imuT = Array.from({ length: imuN }, (_, i) => i * dt);
  const lag = 8.0;
  const mocapY = mocapT.map((x) => gaitLike(x, 1.1));
  const imuY = imuT.map((x) => gaitLike(x - lag, 1.1));
  const result = estimateSignalLagS(mocapT, mocapY, imuT, imuY, { lagS: lag, dtS: dt });
  assert.equal(result.ok, true, `reason=${result.reason} corr=${result.peakCorr}`);
  assert.ok(Number.isFinite(result.peakCorr) && result.peakCorr >= 0.5);
});

test('🔴 estimateSignalLagS: rivalScan ไม่ถูกตัดที่ overlap/3', () => {
  const dt = 0.02;
  const mocapN = Math.round(6 / dt);
  const imuN = Math.round(20 / dt);
  const mocapT = Array.from({ length: mocapN }, (_, i) => i * dt);
  const imuT = Array.from({ length: imuN }, (_, i) => i * dt);
  const lag = 5.5;
  const mocapY = mocapT.map((x) => gaitLike(x, 1.1));
  const imuY = imuT.map((x) => gaitLike(x - lag, 1.1));
  const result = estimateSignalLagS(mocapT, mocapY, imuT, imuY, {
    coarseLagS: lag,
    dtS: dt,
    rivalScanMaxLagS: 8,
  });
  assert.equal(result.ok, true, `reason=${result.reason} lag=${result.lagS}`);
  assert.ok(Math.abs(result.lagS - lag) < 0.15, `ได้ ${result.lagS}`);
});

test('🔴 compareMocapToImu: hs-event-circular → explorationOnly ไม่ publishable', () => {
  const trace = buildDemoTrace({ numStrides: 6, side: 'R', seed: 11 });
  const imu = reprocessImuTrace(trace);
  const mocap = fakeMocapFromImuCycles(imu.bySide.R, 'R');
  const report = compareMocapToImu(mocap, trace, {
    align: { lagS: 0, lagSource: 'hs-event-circular', lagTrusted: false },
    minAgreementPairs: 1,
    explorationOnly: true,
  });
  assert.equal(report.explorationOnly, true);
  assert.equal(report.validationPublishable, false);
  assert.equal(report.sides.R.validationPublishable, false);
});

test('🔴 estimateSignalLagS: กลับขั้ว → polarity-mismatch', () => {
  const dt = 0.005;
  const n = 2000;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((tt) => gaitLike(tt, 1.0));
  const inverted = estimateSignalLagS(t, y, t, y.map((v) => -v), { coarseLagS: 0, dtS: dt });
  assert.equal(inverted.ok, false);
  assert.equal(inverted.reason, 'polarity-mismatch');
});

test('🔴 estimateSignalLagS: useEnvelope ผ่านเมื่อ signed polarity พัง', () => {
  const dt = 0.005;
  const n = 2000;
  const lag = 0.12;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((tt) => gaitLike(tt, 1.0));
  const invertedShifted = t.map((tt) => -gaitLike(tt - lag, 1.0));
  const signed = estimateSignalLagS(t, y, t, invertedShifted, { coarseLagS: lag, dtS: dt });
  assert.equal(signed.ok, false);
  assert.equal(signed.reason, 'polarity-mismatch');
  const env = estimateSignalLagS(t, y, t, invertedShifted, {
    coarseLagS: lag,
    dtS: dt,
    useEnvelope: true,
  });
  assert.equal(env.ok, true, env.reason);
  assert.equal(env.useEnvelope, true);
  assert.ok(Math.abs(env.lagS - lag) < 0.03, `lag=${env.lagS}`);
  assert.ok(env.peakCorr > 0.85, `corr=${env.peakCorr}`);
});

test('🔴 estimateSignalLagS: useEnvelope หา lag ใหญ่ได้โดยไม่มี onset (ไม่หลง peak ใกล้ 0)', () => {
  const dt = 0.02;
  const n = 1200; // 24 s
  const lag = 2.40;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  // AM envelope ไม่ใช่คาบเดียว — กัน period alias ใกล้ 0
  const am = (tt) => (0.55 + 0.45 * Math.sin(2 * Math.PI * 0.07 * tt));
  const y = t.map((tt) => am(tt) * gaitLike(tt, 1.1));
  const shifted = t.map((tt) => am(tt - lag) * gaitLike(tt - lag, 1.1));
  const env = estimateSignalLagS(t, y, t, shifted, {
    dtS: dt,
    rivalScanMaxLagS: 5,
    useEnvelope: true,
  });
  assert.equal(env.ok, true, env.reason);
  assert.ok(Math.abs(env.lagS - lag) < 0.08, `lag=${env.lagS} expected~${lag}`);
  assert.ok(env.peakCorr > 0.8, `corr=${env.peakCorr}`);
});

test('🔴 estimateSignalLagS: กลับขั้ว+lag ไม่เงียบ flip', () => {
  const dt = 0.005;
  const n = 2000;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((x) => Math.sin(2 * Math.PI * x) + 0.4 * Math.sin(4 * Math.PI * x));
  for (const lag of [0.10, 0.20]) {
    const invertedShifted = t.map((tt) => -(
      Math.sin(2 * Math.PI * (tt - lag)) + 0.4 * Math.sin(4 * Math.PI * (tt - lag))
    ));
    const result = estimateSignalLagS(t, y, t, invertedShifted, { coarseLagS: lag, dtS: dt });
    assert.equal(result.ok, false);
    assert.equal(result.reason, 'polarity-mismatch');
  }
});

test('🟡 estimateSignalLagS: sine สมมาตร → polarityIndeterminate', () => {
  const dt = 0.005;
  const n = 2000;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((x) => Math.sin(2 * Math.PI * x));
  const shifted = t.map((tt) => Math.sin(2 * Math.PI * (tt - 0.05)));
  const result = estimateSignalLagS(t, y, t, shifted, { coarseLagS: 0.05, dtS: dt });
  assert.equal(result.ok, false);
  assert.equal(result.polarityIndeterminate, true);
  assert.equal(result.reason, 'ambiguous-polarity');
});

test('🟡 estimateSignalLagS: gait-like ไม่สมมาตร → polarityIndeterminate=false', () => {
  const dt = 0.005;
  const n = 2000;
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((tt) => gaitLike(tt, 1.0));
  const shifted = t.map((tt) => gaitLike(tt - 0.05, 1.0));
  const result = estimateSignalLagS(t, y, t, shifted, { coarseLagS: 0.05, dtS: dt });
  assert.equal(result.ok, true, result.reason);
  assert.equal(result.polarityIndeterminate, false);
});

test('🔴 estimateSignalLagS: ไม่ Math.min-spread crash กับ series ยาว 150k', () => {
  const n = 150_000;
  const t = new Float64Array(n);
  const y = new Float64Array(n);
  for (let i = 0; i < n; i += 1) {
    t[i] = i * 0.01;
    y[i] = Math.sin(i * 0.01);
  }
  assert.doesNotThrow(() => {
    estimateSignalLagS(t, y, t, y, { maxLagS: 0.4, rivalScanMaxLagS: 0.4, dtS: 0.02 });
  });
});

test('🔴 compareMocapToImu: --lag ตรวจ corr ที่ lag ผู้ใช้ ไม่ใช่ free-scan peak', () => {
  const trace = buildDemoTrace({ numStrides: 8, side: 'R', seed: 33 });
  const imu = reprocessImuTrace(trace);
  const gyro = extractImuGyroSeries(trace, 'R', {});
  assert.equal(gyro.ok, true);
  const mocap = fakeMocapFromImuCycles(imu.bySide.R, 'R');
  mocap.perSide.R.signals = {
    tS: gyro.tS,
    shankAngularVelocityDps: gyro.gx,
    angleSource: 'legacy-ankle',
  };

  // --lag นอกช่วง: free scan อาจได้ corr สูงที่ ~0 แต่ forced ที่ 50s ต้องล้ม → syncTrusted=false
  const bad = compareMocapToImu(mocap, trace, {
    align: { lagS: 50.0, lagSource: 'manual', rivalScanMaxLagS: 5 },
    minAgreementPairs: 1,
  });
  assert.equal(bad.sides.R.alignment.forcedLagVerifiedBy, 'signed');
  assert.equal(bad.sides.R.alignment.syncTrusted, false, 'ห้าม trust --lag ที่ corr อ่อน/ไม่มี overlap');
  assert.ok(
    bad.sides.R.alignment.forcedLagCorrAtLag == null
      || bad.sides.R.alignment.forcedLagCorrAtLag < 0.5,
    `forced corr ต้องอ่อน ได้ ${bad.sides.R.alignment.forcedLagCorrAtLag}`,
  );
  // free-scan ที่ ~0 อาจยัง corr ดี — signalPeakCorr ต้องสะท้อน forced ไม่ใช่ free
  if (
    Number.isFinite(bad.sides.R.alignment.freeScanPeakCorr)
    && bad.sides.R.alignment.freeScanPeakCorr >= 0.5
  ) {
    assert.ok(
      bad.sides.R.alignment.signalPeakCorr == null
        || bad.sides.R.alignment.signalPeakCorr < 0.5
        || bad.sides.R.alignment.signalPeakCorr !== bad.sides.R.alignment.freeScanPeakCorr,
      'signalPeakCorr ต้องมาจาก forced lag ไม่ใช่ free-scan peak',
    );
  }

  const good = compareMocapToImu(mocap, trace, {
    align: { lagS: 0, lagSource: 'manual', rivalScanMaxLagS: 5 },
    minAgreementPairs: 1,
  });
  assert.equal(good.sides.R.alignment.syncTrusted, true);
  assert.equal(good.sides.R.alignment.forcedLagVerifiedBy, 'signed');
  assert.ok(
    good.sides.R.alignment.forcedLagCorrAtLag >= 0.5,
    `forced corr ที่ lag=0 ต้องดี ได้ ${good.sides.R.alignment.forcedLagCorrAtLag}`,
  );
});

test('🔴 compareMocapToImu: --lag ถูก + rivalScan แคบยัง syncTrusted (ไม่พึ่ง free-scan)', () => {
  const dt = 0.02;
  const trueLag = 8.0;
  const mocapN = Math.round(12 / dt);
  const imuN = Math.round(25 / dt);
  const mocapT = Array.from({ length: mocapN }, (_, i) => i * dt);
  const imuT = Array.from({ length: imuN }, (_, i) => i * dt);
  const mocapY = mocapT.map((x) => gaitLike(x, 1.1));
  const imuY = imuT.map((x) => gaitLike(x - trueLag, 1.1));

  // forced verify ตรง ๆ — ไม่สนว่า free scan ด้วย rivalScan=5 จะพลาด
  const forced = estimateSignalLagS(mocapT, mocapY, imuT, imuY, {
    lagS: trueLag,
    dtS: dt,
    rivalScanMaxLagS: 5,
  });
  assert.equal(forced.ok, true, `forced reason=${forced.reason} corr=${forced.peakCorr}`);

  const freeNarrow = estimateSignalLagS(mocapT, mocapY, imuT, imuY, {
    dtS: dt,
    rivalScanMaxLagS: 5,
  });
  // free scan แคบมักพลาด lag 8s — นี่คือบั๊กเดิมถ้าเอา free.ok มาเป็น forcedLagCorrOk
  assert.ok(
    !freeNarrow.ok || Math.abs((freeNarrow.lagS ?? 0) - trueLag) > 0.5,
    'fixture ต้องทำให้ free-scan แคบพลาด (ไม่งั้นเทสต์ไม่จับ regression)',
  );
});

test('🔴 compareMocapToImu: --lag คลาด ±n·stride ห้าม publish (แม้ corr สูง / error% สวย)', () => {
  const stride = 1.05;
  const trace = buildDemoTrace({ numStrides: 8, side: 'R', seed: 42 });
  const imu = reprocessImuTrace(trace);
  const gyro = extractImuGyroSeries(trace, 'R', {});
  assert.equal(gyro.ok, true);
  const closed = (imu.bySide.R || []).filter((c) => !c.isOpenStride);
  assert.ok(closed.length >= 4, `ได้ ${closed.length} closed`);
  const mocap = fakeMocapFromImuCycles(closed, 'R');
  mocap.perSide.R.signals = {
    tS: gyro.tS,
    shankAngularVelocityDps: gyro.gx,
    angleSource: 'legacy-ankle',
  };

  const good = compareMocapToImu(mocap, trace, {
    align: { lagS: 0, lagSource: 'manual' },
    minAgreementPairs: 1,
  });
  assert.equal(good.sides.R.alignment.syncTrusted, true, 'lag=0 ต้อง trusted');
  assert.equal(good.sides.R.validationPublishable, true, 'lag=0 ต้อง publishable');
  assert.equal(good.sides.R.alignment.periodAliasRisk, false, 'gate alias ต้อง false เมื่อ --lag ถูก');
  assert.equal(good.sides.R.alignment.freeBeatsForced, false);

  for (const lag of [stride, -stride, 2 * stride]) {
    const bad = compareMocapToImu(mocap, trace, {
      align: { lagS: lag, lagSource: 'manual' },
      minAgreementPairs: 1,
    });
    const a = bad.sides.R.alignment;
    const strideRow = bad.sides.R.metrics.find((m) => m.metric === 'strideLengthM');
    assert.equal(a.syncTrusted, false, `--lag=${lag} ต้องไม่ trusted (corr=${a.forcedLagCorrAtLag})`);
    assert.equal(bad.sides.R.validationPublishable, false, `--lag=${lag} ห้าม publish`);
    assert.equal(strideRow.comparable, false, `--lag=${lag} ห้าม comparable`);
    assert.ok(
      a.periodAliasRisk || a.freeBeatsForced || (a.forcedLagCorrAtLag != null && a.forcedLagCorrAtLag < 0.5),
      `--lag=${lag}: ต้องมี alias/freeBeats หรือ corr อ่อน ได้ alias=${a.periodAliasRisk} freeBeats=${a.freeBeatsForced}`,
    );
  }

  // คลาดน้อยกว่า 1 stride — จับด้วย weak corr
  const off = compareMocapToImu(mocap, trace, {
    align: { lagS: 0.30, lagSource: 'manual' },
    minAgreementPairs: 1,
  });
  assert.equal(off.sides.R.alignment.syncTrusted, false);
  assert.equal(off.sides.R.validationPublishable, false);
});

test('🔴 estimateSignalLagS: forced --lag ที่เป็น period alias → ok=false', () => {
  const dt = 0.02;
  const stride = 1.05;
  const n = Math.round(12 / dt);
  const t = Array.from({ length: n }, (_, i) => i * dt);
  const y = t.map((tt) => gaitLike(tt, stride));
  const at0 = estimateSignalLagS(t, y, t, y, { lagS: 0, dtS: dt, stridePeriodS: stride });
  assert.equal(at0.ok, true, `lag=0 reason=${at0.reason}`);
  assert.equal(at0.periodAliasRisk, false);

  const at1 = estimateSignalLagS(t, y, t, y, { lagS: stride, dtS: dt, stridePeriodS: stride });
  assert.equal(at1.ok, false, `lag=+T ต้องไม่ ok corr=${at1.peakCorr}`);
  assert.equal(at1.reason, 'period-alias-rivals');
  assert.equal(at1.periodAliasRisk, true);
  assert.ok(at1.rivalPeaks.some((p) => Math.abs(p.lagS) < 0.05 && p.corr > at1.peakCorr));
});

test('🔴 compareMocapToImu: pairs ในรายงานไม่ฝัง cycle object เต็ม', () => {
  const trace = buildDemoTrace({ numStrides: 6, side: 'R', seed: 7 });
  const imu = reprocessImuTrace(trace);
  const mocap = fakeMocapFromImuCycles(imu.bySide.R, 'R');
  const gyro = extractImuGyroSeries(trace, 'R', {});
  mocap.perSide.R.signals = {
    tS: gyro.tS,
    shankAngularVelocityDps: gyro.gx,
    angleSource: 'legacy-ankle',
  };
  const report = compareMocapToImu(mocap, trace, { align: { lagS: 0 }, minAgreementPairs: 1 });
  const pairs = report.sides.R.alignment.pairs;
  assert.ok(pairs.length > 0);
  for (const p of pairs) {
    assert.ok(Number.isInteger(p.mocapIndex) || p.mocapIndex === 0);
    assert.ok(Number.isInteger(p.imuIndex) || p.imuIndex === 0);
    assert.equal(Object.keys(p.mocap).includes('hsStartTimeS'), true);
    assert.equal(Object.keys(p.imu).includes('cycleStartTimeS'), true);
    // ห้ามฝัง sample arrays / diagnostics ก้อนใหญ่
    assert.equal(p.mocap.samples, undefined);
    assert.equal(p.imu.samples, undefined);
    assert.equal(p.imu.newCycleDiagnostics, undefined);
  }
});

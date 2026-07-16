// เทียบ gait params จาก MoCap (mocap.gait-params.json) กับ IMU trace (export จาก dashboard)
//
// IMU: ถ้ามี samples → reprocess ผ่าน GaitProcessor เพื่อได้ metrics ครบต่อ cycle
//      ถ้ามีแค่ cycles[] (strideLengthM) → เทียบเฉพาะระยะก้าว / ระยะรวม
// MoCap: ใช้ perSide.L/R.cycles + session/bilateral จาก mocap-analysis/run.js

import { GaitProcessor } from '../src/gait/gaitProcessor.js';
import { applyAxisMap } from '../src/gateway/realtimeSensorUtils.js';

const COMPARE_KEYS = [
  { key: 'strideLengthM', mocap: 'strideLengthM', imu: 'strideLengthM', unit: 'm' },
  { key: 'cadenceSpm', mocap: 'cadenceSpm', imu: 'cadenceSpm', unit: 'spm' },
  { key: 'walkingSpeedMps', mocap: 'walkingSpeedMps', imu: 'walkingSpeedMps', unit: 'm/s' },
  { key: 'stancePct', mocap: 'stancePct', imu: 'stancePct', unit: '%' },
  { key: 'peakShankAngleDeg', mocap: 'peakShankAngleDeg', imu: 'peakShankAngleDeg', unit: 'deg' },
];

function mean(values) {
  const finite = values.filter(Number.isFinite);
  if (!finite.length) return null;
  return finite.reduce((a, b) => a + b, 0) / finite.length;
}

function pctError(imu, mocap) {
  if (!Number.isFinite(imu) || !Number.isFinite(mocap) || mocap === 0) return null;
  return ((imu - mocap) / Math.abs(mocap)) * 100;
}

function absError(imu, mocap) {
  if (!Number.isFinite(imu) || !Number.isFinite(mocap)) return null;
  return imu - mocap;
}

export function summarizeMocapSide(sideData) {
  const cycles = sideData?.cycles || [];
  return {
    cycleCount: cycles.length,
    meanStrideLengthM: mean(cycles.map((c) => c.strideLengthM)),
    meanCadenceSpm: mean(cycles.map((c) => c.cadenceSpm)),
    meanWalkingSpeedMps: mean(cycles.map((c) => c.walkingSpeedMps)),
    meanStancePct: mean(cycles.map((c) => c.stancePct)),
    meanPeakShankAngleDeg: mean(cycles.map((c) => c.peakShankAngleDeg)),
    meanAnkleClearanceM: mean(cycles.map((c) => c.ankleClearanceM)),
  };
}

function sampleToProcessorInput(sample, { axisMap, preferSensorFrame }) {
  const side = sample.side === 'L' || sample.side === 'R' ? sample.side : 'R';
  let accel = sample.raw_accel_canonical;
  let gyro = sample.raw_gyro_canonical;

  if (preferSensorFrame && Array.isArray(sample.raw_accel_sensor) && Array.isArray(sample.raw_gyro_sensor)) {
    const mapped = applyAxisMap(sample.raw_accel_sensor, sample.raw_gyro_sensor, side, axisMap);
    accel = mapped.accel;
    gyro = mapped.gyro;
  }

  if (!Array.isArray(accel) || !Array.isArray(gyro) || accel.length < 3 || gyro.length < 3) {
    return null;
  }

  return {
    sensorKey: sample.sensorKey,
    side,
    sensorMount: sample.sensorMount || 'shank',
    timestampMs: sample.t_ms,
    raw_accel: accel,
    raw_gyro: gyro,
    ax: accel[0],
    ay: accel[1],
    az: accel[2],
    gx: gyro[0],
    gy: gyro[1],
    gz: gyro[2],
  };
}

/**
 * Reprocess IMU trace samples → รายการ cycle ต่อข้าง พร้อม metrics เต็ม
 * (เก็บ params ครั้งแรกที่ cycleKey ปรากฏเป็น "last cycle" ตอน streaming analyze)
 */
export function reprocessImuTrace(trace, options = {}) {
  const analyzeEvery = options.analyzeEvery ?? 40;
  const preferSensorFrame = options.preferSensorFrame ?? Boolean(trace.hasSensorFrameRaw);
  const axisMap = options.axisMap || trace.axisMap || undefined;
  const calibrationBySensor = options.calibrationBySensor || trace.calibrationBySensor || {};

  const samples = Array.isArray(trace.samples) ? trace.samples : [];
  if (!samples.length) {
    return { ok: false, reason: 'no-samples', bySide: { L: [], R: [] }, source: 'none' };
  }

  const bySensor = new Map();
  for (const sample of samples) {
    const input = sampleToProcessorInput(sample, { axisMap, preferSensorFrame });
    if (!input?.sensorKey) continue;
    if (!bySensor.has(input.sensorKey)) bySensor.set(input.sensorKey, []);
    bySensor.get(input.sensorKey).push(input);
  }

  if (!bySensor.size) {
    return { ok: false, reason: 'no-usable-samples', bySide: { L: [], R: [] }, source: 'none' };
  }

  const bySide = { L: [], R: [] };

  for (const [sensorKey, sensorSamples] of bySensor) {
    const proc = new GaitProcessor();
    const profile = calibrationBySensor[sensorKey];
    if (profile) {
      proc.applyCalibration(profile);
    }

    const seen = new Map();
    let sinceAnalyze = 0;
    const sideHint = sensorSamples.find((s) => s.side === 'L' || s.side === 'R')?.side || null;

    proc.onParams(({ params }) => {
      if (!params?.cycleKey || seen.has(params.cycleKey)) return;
      const side = params.side === 'L' || params.side === 'R' ? params.side : sideHint;
      if (side !== 'L' && side !== 'R') return;
      seen.set(params.cycleKey, true);
      bySide[side].push({
        cycleKey: params.cycleKey,
        side,
        sensorKey,
        strideLengthM: params.strideLength,
        strideLengthClamped: params.strideLengthClamped ?? false,
        cadenceSpm: params.cadence,
        walkingSpeedMps: params.walkingSpeed,
        stancePct: params.stancePct,
        swingPct: params.swingPct,
        strideTimeS: params.strideTime,
        peakShankAngleDeg: params.peakShankAngle,
        clearanceM: params.clearance,
        cycleStartTimestampMs: params.cycleStartTimestampMs,
      });
    });

    for (const sample of sensorSamples) {
      proc.addSample(sample);
      sinceAnalyze += 1;
      if (sinceAnalyze >= analyzeEvery) {
        proc.analyze();
        sinceAnalyze = 0;
      }
    }
    proc.analyze();
  }

  for (const side of ['L', 'R']) {
    bySide[side].sort((a, b) => (a.cycleStartTimestampMs ?? 0) - (b.cycleStartTimestampMs ?? 0));
  }

  return { ok: true, reason: null, bySide, source: 'reprocess' };
}

/** fallback: ใช้ cycles[] ใน trace (มีแค่ strideLength เป็นหลัก) */
export function extractImuCyclesFromTrace(trace) {
  const bySide = { L: [], R: [] };
  for (const c of trace.cycles || []) {
    const side = c.side === 'L' || c.side === 'R' ? c.side : null;
    if (!side) continue;
    bySide[side].push({
      cycleKey: c.cycleKey,
      side,
      sensorKey: c.sensorKey,
      strideLengthM: c.strideLengthM,
      strideLengthClamped: c.strideLengthClamped ?? false,
      cadenceSpm: null,
      walkingSpeedMps: null,
      stancePct: null,
      peakShankAngleDeg: null,
      clearanceM: null,
      cycleStartTimestampMs: c.cycleStartTimestampMs,
    });
  }
  const hasAny = bySide.L.length + bySide.R.length > 0;
  return {
    ok: hasAny,
    reason: hasAny ? null : 'no-cycles',
    bySide,
    source: 'trace-cycles',
  };
}

export function summarizeImuSide(cycles) {
  return {
    cycleCount: cycles.length,
    meanStrideLengthM: mean(cycles.map((c) => c.strideLengthM)),
    meanCadenceSpm: mean(cycles.map((c) => c.cadenceSpm)),
    meanWalkingSpeedMps: mean(cycles.map((c) => c.walkingSpeedMps)),
    meanStancePct: mean(cycles.map((c) => c.stancePct)),
    meanPeakShankAngleDeg: mean(cycles.map((c) => c.peakShankAngleDeg)),
    meanClearanceM: mean(cycles.map((c) => c.clearanceM)),
    clampedCount: cycles.filter((c) => c.strideLengthClamped).length,
  };
}

function compareMeans(mocapSummary, imuSummary) {
  const rows = [];
  const metricMap = {
    strideLengthM: ['meanStrideLengthM', 'meanStrideLengthM'],
    cadenceSpm: ['meanCadenceSpm', 'meanCadenceSpm'],
    walkingSpeedMps: ['meanWalkingSpeedMps', 'meanWalkingSpeedMps'],
    stancePct: ['meanStancePct', 'meanStancePct'],
    peakShankAngleDeg: ['meanPeakShankAngleDeg', 'meanPeakShankAngleDeg'],
  };

  for (const spec of COMPARE_KEYS) {
    const [mKey, iKey] = metricMap[spec.key];
    const mocap = mocapSummary[mKey];
    const imu = imuSummary[iKey];
    rows.push({
      metric: spec.key,
      unit: spec.unit,
      mocap,
      imu,
      error: absError(imu, mocap),
      errorPct: pctError(imu, mocap),
      comparable: Number.isFinite(mocap) && Number.isFinite(imu),
    });
  }

  return rows;
}

/**
 * @param {object} mocap - output จาก computeGaitFromMocap / run.js
 * @param {object} imuTrace - export จาก dashboard TraceRecorder
 */
export function compareMocapToImu(mocap, imuTrace, options = {}) {
  const warnings = [];

  if (mocap?.bilateral?.sameSideRepeats > 0) {
    warnings.push(`MoCap sameSideRepeats=${mocap.bilateral.sameSideRepeats} — event detection อาจพลาดฝั่งใดฝั่งหนึ่ง`);
  }

  let imu = reprocessImuTrace(imuTrace, options);
  if (!imu.ok) {
    imu = extractImuCyclesFromTrace(imuTrace);
    if (imu.ok) {
      warnings.push('Reprocess จาก samples ไม่ได้ — ใช้ trace.cycles[] (เทียบได้หลัก ๆ แค่ stride length)');
    }
  }

  if (!imu.ok) {
    return {
      ok: false,
      error: `อ่าน IMU ไม่ได้ (${imu.reason}). ต้องการ samples[] หรือ cycles[] ใน trace`,
      warnings,
    };
  }

  if (imu.source === 'reprocess' && !(imuTrace.samples || []).length) {
    warnings.push('ไม่มี samples ใน trace');
  }

  const sides = {};
  for (const side of ['L', 'R']) {
    const mocapSum = summarizeMocapSide(mocap?.perSide?.[side]);
    const imuCycles = imu.bySide[side] || [];
    const imuSum = summarizeImuSide(imuCycles);

    if (mocapSum.cycleCount === 0 && imuSum.cycleCount === 0) {
      sides[side] = { present: false, mocap: mocapSum, imu: imuSum, metrics: [] };
      continue;
    }

    if (mocapSum.cycleCount === 0) {
      warnings.push(`ขา ${side}: มี IMU แต่ไม่มี MoCap cycles`);
    }
    if (imuSum.cycleCount === 0) {
      warnings.push(`ขา ${side}: มี MoCap แต่ไม่มี IMU cycles`);
    }
    if (imuSum.clampedCount > 0) {
      warnings.push(`ขา ${side}: IMU มี ${imuSum.clampedCount}/${imuSum.cycleCount} cycle ที่ stride ถูก clamp`);
    }

    sides[side] = {
      present: true,
      mocap: mocapSum,
      imu: imuSum,
      metrics: compareMeans(mocapSum, imuSum),
      cycleCountDelta: imuSum.cycleCount - mocapSum.cycleCount,
    };
  }

  const mocapDistanceM = Number.isFinite(mocap?.session?.pelvisNetForwardDisplacementM)
    ? mocap.session.pelvisNetForwardDisplacementM
    : null;
  const gtDistanceM = Number.isFinite(imuTrace?.groundTruth?.distanceM)
    ? imuTrace.groundTruth.distanceM
    : null;

  // รวม stride ข้างเดียว (ถ้ามีสองข้าง ไม่บวกกัน — แต่ละข้างวัด stride ของขาตนเอง)
  const distanceBySide = {};
  for (const side of ['L', 'R']) {
    const imuSum = mean((imu.bySide[side] || []).map((c) => c.strideLengthM));
    const n = (imu.bySide[side] || []).length;
    const sumStride = (imu.bySide[side] || [])
      .map((c) => c.strideLengthM)
      .filter(Number.isFinite)
      .reduce((a, b) => a + b, 0);
    distanceBySide[side] = {
      imuSumStrideLengthM: n ? sumStride : null,
      mocapMeanStrideLengthM: summarizeMocapSide(mocap?.perSide?.[side]).meanStrideLengthM,
      vsMocapPelvisNetPct: pctError(sumStride, mocapDistanceM),
      vsGroundTruthPct: pctError(sumStride, gtDistanceM),
    };
  }

  return {
    ok: true,
    imuSource: imu.source,
    warnings,
    sides,
    session: {
      mocapPelvisNetForwardM: mocapDistanceM,
      mocapDurationS: mocap?.session?.durationS ?? null,
      mocapTrueCadenceSpm: mocap?.bilateral?.trueCadenceSpm ?? null,
      imuGroundTruthDistanceM: gtDistanceM,
      imuSampleCount: imuTrace?.sampleCount ?? (imuTrace?.samples?.length ?? null),
      imuCycleCountInFile: imuTrace?.cycleCount ?? (imuTrace?.cycles?.length ?? null),
      distanceBySide,
    },
    notes: [
      'Clearance ไม่เทียบอัตโนมัติ — MoCap วัดข้อเท้า, IMU double-integrate คนละตำแหน่ง',
      'Stance% คนละนิยาม event ได้ (MoCap = ankle velocity quiet, IMU = gyro HS/TO)',
      'sum(stride) ต่อข้าง ≠ ระยะเดินจริงแบบ 1:1 ถ้าสองข้างบันทึกพร้อมกัน (อย่าบวก L+R)',
    ],
  };
}

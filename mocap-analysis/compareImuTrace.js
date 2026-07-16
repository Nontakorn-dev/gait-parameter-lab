// เทียบ gait params จาก MoCap (mocap.gait-params.json) กับ IMU trace (export จาก dashboard)
//
// IMU: ถ้ามี samples → reprocess ผ่าน GaitProcessor เพื่อได้ metrics ครบต่อ cycle
//      ถ้ามีแค่ cycles[] (strideLengthM) → เทียบเฉพาะระยะก้าว / ระยะรวม
// MoCap: ใช้ perSide.L/R.cycles + session/bilateral จาก mocap-analysis/run.js
//
// การเทียบ: จับคู่ cycle ตามเวลา (HS time) หลังประมาณ lag คงที่ — ไม่เทียบแค่ค่าเฉลี่ย
// รวมทั้ง session (กันกรณี IMU มีช่วงยืนนิ่งก่อน/หลังที่ MoCap ไม่มี)

import { GaitProcessor } from '../src/gait/gaitProcessor.js';
import { applyAxisMap } from '../src/gateway/realtimeSensorUtils.js';

const COMPARE_KEYS = [
  { key: 'strideLengthM', unit: 'm' },
  { key: 'cadenceSpm', unit: 'spm' },
  { key: 'walkingSpeedMps', unit: 'm/s' },
  { key: 'stancePct', unit: '%' },
  { key: 'peakShankAngleDeg', unit: 'deg' },
];

const DEFAULT_MATCH_TOLERANCE_S = 0.40;
const DEFAULT_MAX_LAG_S = 8;

function mean(values) {
  const finite = values.filter(Number.isFinite);
  if (!finite.length) return null;
  return finite.reduce((a, b) => a + b, 0) / finite.length;
}

function median(values) {
  const finite = values.filter(Number.isFinite).sort((a, b) => a - b);
  if (!finite.length) return null;
  const mid = Math.floor(finite.length / 2);
  return finite.length % 2 === 0 ? (finite[mid - 1] + finite[mid]) / 2 : finite[mid];
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

function cycleEntryFromParams(params, side, sensorKey) {
  return {
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
    // relative วินาทีนับจากต้น session ของ processor (= ต้น trace เมื่อ t_ms เป็น relative)
    cycleStartTimeS: Number.isFinite(params.cycleStartTimestampMs)
      ? params.cycleStartTimestampMs / 1000
      : null,
  };
}

function shouldUpgradeCycle(existing, next) {
  // อัปเดตเมื่อได้ temporal metrics ที่เคยเป็น null (open/unresolved → measured)
  if (!Number.isFinite(existing.stancePct) && Number.isFinite(next.stancePct)) return true;
  if (!Number.isFinite(existing.strideLengthM) && Number.isFinite(next.strideLengthM)) return true;
  return false;
}

/**
 * Reprocess IMU trace samples → รายการ cycle ต่อข้าง พร้อม metrics เต็ม
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

    const seen = new Map(); // cycleKey → entry object (mutable สำหรับ upgrade)
    let sinceAnalyze = 0;
    const sideHint = sensorSamples.find((s) => s.side === 'L' || s.side === 'R')?.side || null;

    // t0 ของเซนเซอร์นี้ = sample แรกที่ valid (สำหรับ normalize เวลาถ้าเป็น absolute epoch)
    const firstT = sensorSamples.map((s) => s.timestampMs).find(Number.isFinite);

    proc.onParams(({ params }) => {
      if (!params?.cycleKey) return;
      const side = params.side === 'L' || params.side === 'R' ? params.side : sideHint;
      if (side !== 'L' && side !== 'R') return;

      const entry = cycleEntryFromParams(params, side, sensorKey);
      // ถ้า timestamp เป็น epoch absolute ให้แปลงเป็น relative ต่อ first sample ของ trace
      if (
        Number.isFinite(entry.cycleStartTimestampMs)
        && Number.isFinite(firstT)
        && firstT >= 946684800000
      ) {
        entry.cycleStartTimeS = (entry.cycleStartTimestampMs - firstT) / 1000;
      }

      const existing = seen.get(params.cycleKey);
      if (!existing) {
        seen.set(params.cycleKey, entry);
        bySide[side].push(entry);
        return;
      }
      if (shouldUpgradeCycle(existing, entry)) {
        Object.assign(existing, entry);
      }
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
    bySide[side].sort((a, b) => (a.cycleStartTimeS ?? 0) - (b.cycleStartTimeS ?? 0));
  }

  return { ok: true, reason: null, bySide, source: 'reprocess' };
}

/** fallback: ใช้ cycles[] ใน trace (มีแค่ strideLength เป็นหลัก) */
export function extractImuCyclesFromTrace(trace) {
  const bySide = { L: [], R: [] };
  const allStarts = (trace.cycles || [])
    .map((c) => c.cycleStartTimestampMs)
    .filter(Number.isFinite);
  const t0 = allStarts.length ? Math.min(...allStarts) : null;

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
      cycleStartTimeS: Number.isFinite(c.cycleStartTimestampMs) && Number.isFinite(t0)
        ? (c.cycleStartTimestampMs - t0) / 1000
        : null,
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

function mocapCycleTimeS(cycle) {
  return Number.isFinite(cycle.hsStartTimeS) ? cycle.hsStartTimeS : null;
}

function imuCycleTimeS(cycle) {
  return Number.isFinite(cycle.cycleStartTimeS) ? cycle.cycleStartTimeS : null;
}

/**
 * ประมาณ lag คงที่ (IMU − MoCap) ที่ maximize จำนวนคู่ HS ใน tolerance
 * คืน { lagS, matchCount }
 */
export function estimateHsTimeLagS(mocapCycles, imuCycles, options = {}) {
  const maxLagS = options.maxLagS ?? DEFAULT_MAX_LAG_S;
  const matchToleranceS = options.matchToleranceS ?? DEFAULT_MATCH_TOLERANCE_S;
  const mocapTimes = mocapCycles.map(mocapCycleTimeS).filter(Number.isFinite).sort((a, b) => a - b);
  const imuTimes = imuCycles.map(imuCycleTimeS).filter(Number.isFinite).sort((a, b) => a - b);
  if (!mocapTimes.length || !imuTimes.length) {
    return { lagS: 0, matchCount: 0 };
  }

  // candidate lags จากทุกคู่เวลา (จำกัดจำนวน)
  const candidates = new Set([0]);
  const step = Math.max(1, Math.floor(imuTimes.length / 12));
  for (let i = 0; i < imuTimes.length; i += step) {
    for (let j = 0; j < mocapTimes.length; j += step) {
      const lag = imuTimes[i] - mocapTimes[j];
      if (Math.abs(lag) <= maxLagS) candidates.add(Math.round(lag * 100) / 100);
    }
  }

  let bestLag = 0;
  let bestCount = -1;
  for (const lag of candidates) {
    let count = 0;
    let mi = 0;
    for (const mt of mocapTimes) {
      while (mi < imuTimes.length && imuTimes[mi] - lag < mt - matchToleranceS) mi += 1;
      if (mi < imuTimes.length && Math.abs((imuTimes[mi] - lag) - mt) <= matchToleranceS) {
        count += 1;
        mi += 1;
      }
    }
    if (count > bestCount) {
      bestCount = count;
      bestLag = lag;
    }
  }

  return { lagS: bestLag, matchCount: Math.max(0, bestCount) };
}

/**
 * จับคู่ 1:1 ตามเวลาหลังชดเชย lag — greedy nearest ใน tolerance
 */
export function pairCyclesByTime(mocapCycles, imuCycles, options = {}) {
  const matchToleranceS = options.matchToleranceS ?? DEFAULT_MATCH_TOLERANCE_S;
  const { lagS, matchCount: lagMatchHint } = estimateHsTimeLagS(mocapCycles, imuCycles, options);

  const mocap = mocapCycles
    .map((c, index) => ({ c, index, t: mocapCycleTimeS(c) }))
    .filter((x) => Number.isFinite(x.t))
    .sort((a, b) => a.t - b.t);
  const imu = imuCycles
    .map((c, index) => ({ c, index, t: imuCycleTimeS(c) }))
    .filter((x) => Number.isFinite(x.t))
    .sort((a, b) => a.t - b.t);

  const usedImu = new Set();
  const pairs = [];
  for (const m of mocap) {
    let best = null;
    for (const im of imu) {
      if (usedImu.has(im.index)) continue;
      const alignedImuT = im.t - lagS;
      const dt = Math.abs(alignedImuT - m.t);
      if (dt <= matchToleranceS && (!best || dt < best.dt)) {
        best = { im, dt, alignedImuT };
      }
    }
    if (!best) continue;
    usedImu.add(best.im.index);
    pairs.push({
      mocap: m.c,
      imu: best.im.c,
      mocapTimeS: m.t,
      imuTimeS: best.im.t,
      alignedImuTimeS: best.alignedImuT,
      timeErrorS: best.im.t - lagS - m.t,
    });
  }

  return {
    lagS,
    lagMatchHint,
    pairs,
    unpairedMocap: mocap.length - pairs.length,
    unpairedImu: imu.length - pairs.length,
  };
}

function summarizePaired(pairs) {
  const mocap = {
    cycleCount: pairs.length,
    meanStrideLengthM: mean(pairs.map((p) => p.mocap.strideLengthM)),
    meanCadenceSpm: mean(pairs.map((p) => p.mocap.cadenceSpm)),
    meanWalkingSpeedMps: mean(pairs.map((p) => p.mocap.walkingSpeedMps)),
    meanStancePct: mean(pairs.map((p) => p.mocap.stancePct)),
    meanPeakShankAngleDeg: mean(pairs.map((p) => p.mocap.peakShankAngleDeg)),
  };
  const imu = {
    cycleCount: pairs.length,
    meanStrideLengthM: mean(pairs.map((p) => p.imu.strideLengthM)),
    meanCadenceSpm: mean(pairs.map((p) => p.imu.cadenceSpm)),
    meanWalkingSpeedMps: mean(pairs.map((p) => p.imu.walkingSpeedMps)),
    meanStancePct: mean(pairs.map((p) => p.imu.stancePct)),
    meanPeakShankAngleDeg: mean(pairs.map((p) => p.imu.peakShankAngleDeg)),
    meanClearanceM: mean(pairs.map((p) => p.imu.clearanceM)),
    clampedCount: pairs.filter((p) => p.imu.strideLengthClamped).length,
  };
  return { mocap, imu };
}

function compareMeans(mocapSummary, imuSummary) {
  const metricMap = {
    strideLengthM: ['meanStrideLengthM', 'meanStrideLengthM'],
    cadenceSpm: ['meanCadenceSpm', 'meanCadenceSpm'],
    walkingSpeedMps: ['meanWalkingSpeedMps', 'meanWalkingSpeedMps'],
    stancePct: ['meanStancePct', 'meanStancePct'],
    peakShankAngleDeg: ['meanPeakShankAngleDeg', 'meanPeakShankAngleDeg'],
  };

  return COMPARE_KEYS.map((spec) => {
    const [mKey, iKey] = metricMap[spec.key];
    const mocap = mocapSummary[mKey];
    const imu = imuSummary[iKey];
    return {
      metric: spec.key,
      unit: spec.unit,
      mocap,
      imu,
      error: absError(imu, mocap),
      errorPct: pctError(imu, mocap),
      comparable: Number.isFinite(mocap) && Number.isFinite(imu),
    };
  });
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

  const sides = {};
  for (const side of ['L', 'R']) {
    const mocapCycles = mocap?.perSide?.[side]?.cycles || [];
    const imuCycles = imu.bySide[side] || [];
    const mocapSumAll = summarizeMocapSide({ cycles: mocapCycles });
    const imuSumAll = summarizeImuSide(imuCycles);

    if (mocapCycles.length === 0 && imuCycles.length === 0) {
      sides[side] = { present: false, mocap: mocapSumAll, imu: imuSumAll, metrics: [], alignment: null };
      continue;
    }

    if (mocapCycles.length === 0) warnings.push(`ขา ${side}: มี IMU แต่ไม่มี MoCap cycles`);
    if (imuCycles.length === 0) warnings.push(`ขา ${side}: มี MoCap แต่ไม่มี IMU cycles`);
    if (imuSumAll.clampedCount > 0) {
      warnings.push(`ขา ${side}: IMU มี ${imuSumAll.clampedCount}/${imuSumAll.cycleCount} cycle ที่ stride ถูก clamp`);
    }

    const alignment = pairCyclesByTime(mocapCycles, imuCycles, options.align);
    const usePaired = alignment.pairs.length > 0;
    if (!usePaired && mocapCycles.length && imuCycles.length) {
      warnings.push(`ขา ${side}: จับคู่ตามเวลาไม่ได้ — fallback เป็นค่าเฉลี่ยทั้ง session (อาจรวมช่วงยืนนิ่ง)`);
    } else if (usePaired && (alignment.unpairedMocap > 0 || alignment.unpairedImu > 0)) {
      warnings.push(
        `ขา ${side}: จับคู่ได้ ${alignment.pairs.length} คู่ `
        + `(MoCap ค้าง ${alignment.unpairedMocap}, IMU ค้าง ${alignment.unpairedImu}, lag=${alignment.lagS.toFixed(2)}s)`,
      );
    }

    const pairedSummary = usePaired ? summarizePaired(alignment.pairs) : null;
    const mocapForCompare = pairedSummary?.mocap ?? mocapSumAll;
    const imuForCompare = pairedSummary?.imu ?? imuSumAll;

    const nullStance = usePaired
      ? alignment.pairs.filter((p) => !Number.isFinite(p.imu.stancePct)).length
      : imuCycles.filter((c) => !Number.isFinite(c.stancePct)).length;
    if (nullStance > 0 && imuCycles.length > 0) {
      warnings.push(`ขา ${side}: IMU มี ${nullStance} cycle ที่ stancePct=null (temporal unresolved)`);
    }

    sides[side] = {
      present: true,
      mocap: mocapForCompare,
      imu: imuForCompare,
      mocapAll: mocapSumAll,
      imuAll: imuSumAll,
      metrics: compareMeans(mocapForCompare, imuForCompare),
      cycleCountDelta: imuSumAll.cycleCount - mocapSumAll.cycleCount,
      alignment: {
        mode: usePaired ? 'hs-time' : 'session-mean-fallback',
        lagS: alignment.lagS,
        pairedCount: alignment.pairs.length,
        unpairedMocap: alignment.unpairedMocap,
        unpairedImu: alignment.unpairedImu,
        meanTimeErrorS: mean(alignment.pairs.map((p) => p.timeErrorS)),
        medianTimeErrorS: median(alignment.pairs.map((p) => p.timeErrorS)),
      },
    };
  }

  const mocapDistanceM = Number.isFinite(mocap?.session?.pelvisNetForwardDisplacementM)
    ? mocap.session.pelvisNetForwardDisplacementM
    : null;
  const gtDistanceM = Number.isFinite(imuTrace?.groundTruth?.distanceM)
    ? imuTrace.groundTruth.distanceM
    : null;

  const distanceBySide = {};
  for (const side of ['L', 'R']) {
    const n = (imu.bySide[side] || []).length;
    const sumStride = (imu.bySide[side] || [])
      .map((c) => c.strideLengthM)
      .filter(Number.isFinite)
      .reduce((a, b) => a + b, 0);
    distanceBySide[side] = {
      imuSumStrideLengthM: n ? sumStride : null,
      mocapMeanStrideLengthM: summarizeMocapSide(mocap?.perSide?.[side]).meanStrideLengthM,
      // สำคัญ: ขาที่ไม่มี cycle ต้องเป็น null ไม่ใช่ pctError(0, gt)=−100%
      vsMocapPelvisNetPct: n ? pctError(sumStride, mocapDistanceM) : null,
      vsGroundTruthPct: n ? pctError(sumStride, gtDistanceM) : null,
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
      'Metrics หลักมาจากคู่ cycle ที่จับตาม HS time หลังประมาณ lag — ไม่ใช่เฉลี่ยทั้ง session ดิบ',
      'sum(stride) ต่อข้าง ≠ ระยะเดินจริงแบบ 1:1 ถ้าสองข้างบันทึกพร้อมกัน (อย่าบวก L+R)',
    ],
  };
}

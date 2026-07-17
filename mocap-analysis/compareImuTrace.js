// เทียบ gait params จาก MoCap (mocap.gait-params.json) กับ IMU trace (export จาก dashboard)
//
// IMU: ถ้ามี samples → reprocess ผ่าน GaitProcessor เพื่อได้ metrics ครบต่อ cycle
//      ถ้ามีแค่ cycles[] (strideLengthM) → เทียบเฉพาะระยะก้าว / ระยะรวม
// MoCap: ใช้ perSide.L/R.cycles + session/bilateral จาก mocap-analysis/run.js
//        ถ้ามี perSide.*.signals (ω) จะ align ด้วย signal xcorr ก่อน
//
// การเทียบ: จับคู่ cycle ตามเวลาหลังประมาณ lag — ลำดับ: signal xcorr → HS-event lag
// HS timing residual ใช้เป็นเมตริก detector ได้เฉพาะเมื่อ sync มาจากสัญญาณ

import { GaitProcessor } from '../src/gait/gaitProcessor.js';
import { applyAxisMap } from '../src/gateway/realtimeSensorUtils.js';
import { minFinite, maxFinite } from '../src/util/finiteStats.js';

const COMPARE_KEYS = [
  { key: 'strideLengthM', unit: 'm', agreementClass: 'primary' },
  { key: 'cadenceSpm', unit: 'spm', agreementClass: 'primary' },
  { key: 'walkingSpeedMps', unit: 'm/s', agreementClass: 'primary' },
  // exploratory: นิยามคนละแบบ / มี mounting offset — โชว์ตัวเลขได้แต่ห้ามเคลม agreement
  { key: 'stancePct', unit: '%', agreementClass: 'exploratory', ineligibleReason: 'different-hs-to-definitions' },
  { key: 'peakShankAngleDeg', unit: 'deg', agreementClass: 'exploratory', ineligibleReason: 'mounting-angle-offset' },
];

const DEFAULT_MATCH_TOLERANCE_S = 0.40;
// หน้าต่างละเอียดรอบ coarse lag (sync แล็บมัก < 0.5s หลัง onset align)
const DEFAULT_FINE_MAX_LAG_S = 0.4;
// สแกนกว้างเพื่อจับ rival peaks ที่ ±n·stride — จำกัดแค่ fine window = การันตี alias เงียบ
const DEFAULT_RIVAL_SCAN_MAX_LAG_S = 5;
// backward-compat: maxLagS เดิมชี้ fine window; rival scan แยก
const DEFAULT_MAX_LAG_S = DEFAULT_FINE_MAX_LAG_S;
const DEFAULT_XCORR_DT_S = 0.005;
const DEFAULT_MIN_PEAK_CORR = 0.5;
const AMBIGUOUS_PEAK_RATIO = 0.95;
const POLARITY_PEAK_MARGIN = 0.05;
const SIGNAL_LAG_SYSTEMATIC_UNCERTAINTY_S = 0.003;
const ONSET_SUSTAIN_S = 0.08;
const DEFAULT_MAX_ZUPT_DEV_G = 0.25;
const DEFAULT_MIN_AGREEMENT_PAIRS = 3;

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

/** @param {number[]} sortedAsc @param {number} p01 ใน [0,1] */
function percentileSorted(sortedAsc, p01) {
  if (!sortedAsc.length) return 0;
  const idx = Math.min(
    sortedAsc.length - 1,
    Math.max(0, Math.round(p01 * (sortedAsc.length - 1))),
  );
  return sortedAsc[idx];
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
    temporalSource: params.temporalSource ?? null,
    zuptAccelDeviationG: params.zuptAccelDeviationG ?? null,
    zuptCheck: params.zuptCheck ?? null,
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
  // อัปเกรดคุณภาพ temporal / ZUPT metadata
  if (existing.temporalSource !== 'measured-to' && next.temporalSource === 'measured-to') return true;
  return false;
}

/** ก้าวที่เอาเข้า agreement stats ได้ — ตัด clamp / ZUPT พัง / stance หลอก */
export function isAgreementQualityImuCycle(cycle, options = {}) {
  if (!cycle) return false;
  if (cycle.strideLengthClamped) return false;
  const temporal = cycle.temporalSource;
  if (temporal === 'previous-valid-ratio' || temporal === 'unresolved') return false;
  const maxZupt = options.maxZuptDevG ?? DEFAULT_MAX_ZUPT_DEV_G;
  const zupt = cycle.zuptAccelDeviationG
    ?? cycle.zuptCheck?.zuptAccelDeviationG
    ?? null;
  if (Number.isFinite(zupt) && zupt > maxZupt) return false;
  return Number.isFinite(cycle.strideLengthM);
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
  const qualityFlags = [];

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

    proc.onParams(({ params, newCycleDiagnostics }) => {
      if (newCycleDiagnostics?.length) {
        for (const d of newCycleDiagnostics) {
          const existing = seen.get(String(d.cycleKey));
          if (!existing) continue;
          if (d.strideLengthClamped != null) existing.strideLengthClamped = d.strideLengthClamped;
          if (d.zuptCheck) {
            existing.zuptCheck = d.zuptCheck;
            existing.zuptAccelDeviationG = d.zuptCheck.zuptAccelDeviationG ?? existing.zuptAccelDeviationG;
          }
        }
      }

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

    if (proc.usedSyntheticTimestamps || proc.missingTimestampCount > 0 || proc.skippedIncompleteSampleCount > 0) {
      qualityFlags.push({
        sensorKey,
        usedSyntheticTimestamps: Boolean(proc.usedSyntheticTimestamps),
        missingTimestampCount: proc.missingTimestampCount || 0,
        skippedIncompleteSampleCount: proc.skippedIncompleteSampleCount || 0,
      });
    }
  }

  for (const side of ['L', 'R']) {
    bySide[side].sort((a, b) => (a.cycleStartTimeS ?? 0) - (b.cycleStartTimeS ?? 0));
  }

  return { ok: true, reason: null, bySide, source: 'reprocess', qualityFlags };
}

/** fallback: ใช้ cycles[] ใน trace (มีแค่ strideLength เป็นหลัก) */
export function extractImuCyclesFromTrace(trace) {
  const bySide = { L: [], R: [] };
  const allStarts = (trace.cycles || [])
    .map((c) => c.cycleStartTimestampMs)
    .filter(Number.isFinite);
  const t0 = allStarts.length ? minFinite(allStarts) : null;

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

function countHsMatches(mocapTimes, imuTimes, lag, matchToleranceS) {
  let count = 0;
  let residualAbsSum = 0;
  let mi = 0;
  for (const mt of mocapTimes) {
    while (mi < imuTimes.length && imuTimes[mi] - lag < mt - matchToleranceS) mi += 1;
    if (mi < imuTimes.length && Math.abs((imuTimes[mi] - lag) - mt) <= matchToleranceS) {
      const err = (imuTimes[mi] - lag) - mt;
      residualAbsSum += Math.abs(err);
      count += 1;
      mi += 1;
    }
  }
  return { count, residualAbsSum };
}

/**
 * ประมาณ lag คงที่ (IMU − MoCap) จาก HS events — ใช้จับคู่ cycle เพื่อเทียบ stride/cadence
 *
 * ⚠️ ไม่ใช้ residual หลัง align นี้เป็นเมตริก "HS timing bias" — bias ของ detector
 * ถูกดูดเข้า lagS ได้ (shared blind spot). วัด timing จริงต้อง align ระดับสัญญาณก่อน
 *
 * Coarse→fine (เหมือน signal xcorr):
 *   1) สแกน candidate กว้าง (±rivalScanMaxLagS) — จำกัดแค่ ±0.4s = การันตี alias เงียบ
 *   2) เลือกใกล้ coarseLagS ใน fine window เป็นพิเศษ
 *   3) ถ้ามี rival match นับใกล้เคียงที่ห่างเกิน fine → periodAliasRisk; ห้าม ok โดยไม่มี trusted coarse
 *
 * Tie-break: ถ้า match count เท่ากัน เลือก lag ที่ residual รวมเล็กสุด (ไม่ seed 0)
 */
export function estimateHsTimeLagS(mocapCycles, imuCycles, options = {}) {
  const fineMaxLagS = options.fineMaxLagS ?? options.maxLagS ?? DEFAULT_FINE_MAX_LAG_S;
  const rivalScanMaxLagS = options.rivalScanMaxLagS ?? DEFAULT_RIVAL_SCAN_MAX_LAG_S;
  const matchToleranceS = options.matchToleranceS ?? DEFAULT_MATCH_TOLERANCE_S;
  const hasExplicitCoarse = Number.isFinite(options.coarseLagS);
  const coarseLagS = hasExplicitCoarse ? options.coarseLagS : 0;
  const trustedCoarse = hasExplicitCoarse;

  const empty = {
    lagS: 0,
    matchCount: 0,
    tiedLags: [],
    ambiguous: false,
    periodAliasRisk: false,
    rivalLags: [],
    coarseLagS,
    ok: false,
    reason: 'missing-cycles',
    residualAbsSum: null,
  };

  const mocapTimes = mocapCycles.map(mocapCycleTimeS).filter(Number.isFinite).sort((a, b) => a - b);
  const imuTimes = imuCycles.map(imuCycleTimeS).filter(Number.isFinite).sort((a, b) => a - b);
  if (!mocapTimes.length || !imuTimes.length) {
    return empty;
  }

  // สแกนกว้าง — ห้าม filter ด้วย fine window ตอนสร้าง candidate (นั่นคือบั๊ก alias เดิม)
  const candidates = new Set();
  const step = Math.max(1, Math.floor(imuTimes.length / 12));
  for (let i = 0; i < imuTimes.length; i += step) {
    for (let j = 0; j < mocapTimes.length; j += step) {
      const lag = imuTimes[i] - mocapTimes[j];
      if (Math.abs(lag) <= rivalScanMaxLagS) {
        candidates.add(Math.round(lag * 100) / 100);
      }
    }
  }
  if (!candidates.size) candidates.add(0);

  const scored = [];
  for (const lag of candidates) {
    const { count, residualAbsSum } = countHsMatches(mocapTimes, imuTimes, lag, matchToleranceS);
    scored.push({ lag, count, residualAbsSum });
  }

  function pickBest(pool) {
    let bestLag = 0;
    let bestCount = -1;
    let bestResidual = Infinity;
    for (const row of pool) {
      const betterCount = row.count > bestCount;
      const betterResidual = row.count === bestCount && row.residualAbsSum < bestResidual - 1e-12;
      const closerCoarse = row.count === bestCount
        && Math.abs(row.residualAbsSum - bestResidual) <= 1e-12
        && Math.abs(row.lag - coarseLagS) < Math.abs(bestLag - coarseLagS);
      if (betterCount || betterResidual || closerCoarse) {
        bestCount = row.count;
        bestLag = row.lag;
        bestResidual = row.residualAbsSum;
      }
    }
    return { bestLag, bestCount, bestResidual };
  }

  const inFine = scored.filter((s) => Math.abs(s.lag - coarseLagS) <= fineMaxLagS);
  // มี trusted coarse → เลือกใน fine window; ไม่มี → เลือก global แล้วค่อย refuse ถ้ามี alias
  const pool = trustedCoarse && inFine.length ? inFine : scored;
  const { bestLag, bestCount, bestResidual } = pickBest(pool);

  const nearBest = scored.filter((s) => (
    s.count >= bestCount
    || (bestCount >= 5 && s.count >= bestCount - 1)
  ));
  const tiedLags = scored
    .filter((s) => s.count === bestCount)
    .filter((s) => Math.abs(s.residualAbsSum - bestResidual) <= Math.max(1e-6, bestResidual * 0.05))
    .map((s) => s.lag)
    .sort((a, b) => a - b);
  const ambiguous = tiedLags.length > 1;

  const bestMeanRes = bestCount > 0 ? bestResidual / bestCount : 0;
  // rival = period alias จริง (residual ต่อคู่ใกล้กัน) — ซีรีส์สั้น/idle ใน tolerance ไม่นับ
  const rivalLags = nearBest
    .filter((s) => s.lag !== bestLag)
    .filter((s) => Math.abs(s.lag - bestLag) > fineMaxLagS * 0.5)
    .filter((s) => {
      const meanRes = s.count > 0 ? s.residualAbsSum / s.count : Infinity;
      return meanRes <= bestMeanRes + 0.005;
    })
    .map((s) => ({
      lagS: s.lag,
      matchCount: s.count,
      deltaFromChosenS: s.lag - bestLag,
    }))
    .sort((a, b) => b.matchCount - a.matchCount);
  const periodAliasRisk = rivalLags.length > 0;
  const refuseForAlias = periodAliasRisk && !trustedCoarse;
  const ok = bestCount > 0 && !refuseForAlias;
  let reason = null;
  if (!ok) {
    if (bestCount <= 0) reason = 'no-hs-matches';
    else if (refuseForAlias) reason = 'period-alias-rivals';
    else if (ambiguous) reason = 'ambiguous-hs-lags';
  }

  return {
    lagS: bestLag,
    matchCount: Math.max(0, bestCount),
    tiedLags,
    ambiguous,
    periodAliasRisk,
    rivalLags,
    coarseLagS,
    ok,
    reason,
    residualAbsSum: bestResidual === Infinity ? null : bestResidual,
  };
}

/** linear resample ลงกริดคงที่ — คืน null ช่วงที่ไม่มีข้อมูล */
export function resampleUniform(t, y, dt, t0, t1) {
  if (!Number.isFinite(dt) || dt <= 0 || !Number.isFinite(t0) || !Number.isFinite(t1) || t1 <= t0) {
    return [];
  }
  const n = Math.floor((t1 - t0) / dt) + 1;
  const out = new Array(n).fill(null);
  let j = 0;
  for (let i = 0; i < n; i += 1) {
    const ti = t0 + i * dt;
    while (j + 1 < t.length && t[j + 1] < ti) j += 1;
    const tA = t[j];
    const tB = t[j + 1];
    const yA = y[j];
    const yB = y[j + 1];
    if (!Number.isFinite(tA) || !Number.isFinite(yA)) continue;
    if (j + 1 >= t.length || !Number.isFinite(tB) || !Number.isFinite(yB)) {
      if (Math.abs(tA - ti) <= dt * 0.6) out[i] = yA;
      continue;
    }
    if (ti < tA - 1e-9 || ti > tB + 1e-9) continue;
    const w = tB === tA ? 0 : (ti - tA) / (tB - tA);
    out[i] = yA + (yB - yA) * w;
  }
  return out;
}

/**
 * จังหวะเริ่มเคลื่อนไหวจาก envelope |y| — ใช้เป็น coarse sync (ไม่เป็นคาบเหมือนก้าว)
 * ต้องมีช่วงนิ่งนำหน้า ไม่เช่นนั้นสัญญาณคาบตั้งแต่ต้นเฟรมจะให้ "onset" = phase ในก้าวแรก
 * ซึ่งเป็น alias ไม่ใช่ sync จริง
 *
 * ห้ามใช้ maxAbs×0.25 / maxAbs×0.15 เป็นคู่ threshold+gate — อัตราส่วน stance/swing
 * ของ ω หน้าแข้งจริงอยู่ ~15–28% จึงคร่อมแบนด์นั้นพอดี (onset ไปตก swing แล้ว gate ปฏิเสธ)
 */
export function estimateSignalOnsetS(t, y, options = {}) {
  if (!t?.length || t.length !== y?.length) return null;
  const abs = y.map((v) => (Number.isFinite(v) ? Math.abs(v) : 0));
  const sorted = abs.slice().sort((a, b) => a - b);
  const maxAbs = sorted[sorted.length - 1] ?? 0;
  if (!(maxAbs > 0)) return null;

  // noise floor จากทั้งซีรีส์ (ช่วงนิ่งดึง p05 ลง) — ไม่ผูกกับ swing peak
  // หน่วยต้องเป็น dps (เหมือน computeAngularVelocityDps / rawGyroToDps) —
  // ค่า +12/+20 ด้านล่างเป็น dps floor; ถ้าส่ง rad/s onset จะไม่ยิง (safe fail)
  const noiseFloor = percentileSorted(sorted, 0.05);
  const absFloor = options.onsetAbsFloor ?? 12; // dps
  const quietAbsFloor = options.leadingQuietAbsFloor ?? 20; // dps
  const threshold = options.onsetThreshold ?? Math.max(noiseFloor * 5, noiseFloor + absFloor);

  const dts = [];
  for (let i = 1; i < t.length; i += 1) {
    const d = t[i] - t[i - 1];
    if (d > 0 && d < 1) dts.push(d);
  }
  const dtMed = median(dts) || 0.01;
  const need = Math.max(3, Math.round((options.onsetSustainS ?? ONSET_SUSTAIN_S) / dtMed));
  const minQuietS = options.minLeadingQuietS ?? 0.4;
  const minQuietSamples = Math.max(5, Math.round(minQuietS / dtMed));
  // gate เทียบกับ noise — ไม่ใช่ maxAbs×k (จะคร่อม stance ripple)
  const quietLimit = options.leadingQuietMaxAbs
    ?? Math.max(noiseFloor * 8, noiseFloor + quietAbsFloor, threshold);

  let run = 0;
  for (let i = 0; i < abs.length; i += 1) {
    if (abs[i] >= threshold) {
      run += 1;
      if (run >= need) {
        const onsetIdx = i - need + 1;
        // เว้น guard = need ก่อน onset — ไม่ให้ ramp ที่ไต่เข้า threshold ตัด gate ทิ้ง
        const gateEnd = Math.max(0, onsetIdx - need);
        if (gateEnd < minQuietSamples) return null;
        const lead = abs.slice(0, gateEnd);
        const leadSorted = lead.slice().sort((a, b) => a - b);
        const leadP95 = percentileSorted(leadSorted, 0.95);
        if (leadP95 >= quietLimit) return null;
        return Number.isFinite(t[onsetIdx]) ? t[onsetIdx] : null;
      }
    } else {
      run = 0;
    }
  }
  return null;
}

function listLocalMaxima(corrByLag, minCorr) {
  const peaks = [];
  for (const [lagSamples, corr] of corrByLag) {
    if (!(corr >= minCorr)) continue;
    const left = corrByLag.get(lagSamples - 1);
    const right = corrByLag.get(lagSamples + 1);
    const isLocalMax = (left == null || corr >= left) && (right == null || corr >= right);
    if (isLocalMax) peaks.push({ lagSamples, corr });
  }
  return peaks.sort((a, b) => b.corr - a.corr);
}

/**
 * ประมาณ lag (IMU − MoCap) จาก normalized cross-correlation ของสัญญาณ
 *
 * Coarse→fine:
 *   1) coarseLag จาก onset |ω| (หรือ options.coarseLagS / options.lagS)
 *   2) สแกนกว้าง (±rivalScanMaxLagS) เพื่อหา peaks + rival ที่ ±n·stride
 *   3) เลือก peak ใกล้ coarseLag ที่สุด (ใน fine window เป็นพิเศษ)
 *   4) ถ้ามี rival corr ≥ 95% ของ peak ที่เลือก → periodAliasRisk / ambiguous (ต้องเตือน — ห้ามเงียบ)
 *
 * Polarity: เทียบ peak(+)/peak(−) บนสแกนกว้าง — ไม่ใช้ corr(0)
 *
 * @returns {object}
 */
export function estimateSignalLagS(mocapT, mocapY, imuT, imuY, options = {}) {
  const fineMaxLagS = options.fineMaxLagS ?? options.maxLagS ?? DEFAULT_FINE_MAX_LAG_S;
  const rivalScanMaxLagS = options.rivalScanMaxLagS ?? DEFAULT_RIVAL_SCAN_MAX_LAG_S;
  const dt = options.dtS ?? DEFAULT_XCORR_DT_S;
  const minPeakCorr = options.minPeakCorr ?? DEFAULT_MIN_PEAK_CORR;
  const polarityMargin = options.polarityPeakMargin ?? POLARITY_PEAK_MARGIN;

  const empty = {
    lagS: null,
    peakCorr: null,
    peakCorrMinus: null,
    polarity: 1,
    ok: false,
    ambiguous: false,
    polarityIndeterminate: false,
    periodAliasRisk: false,
    rivalPeaks: [],
    coarseLagS: null,
    coarseFromOnset: false,
    reason: 'missing-series',
    systematicUncertaintyS: SIGNAL_LAG_SYSTEMATIC_UNCERTAINTY_S,
  };

  if (!mocapT?.length || !imuT?.length || mocapT.length !== mocapY.length || imuT.length !== imuY.length) {
    return empty;
  }

  const mocapT0 = minFinite(mocapT);
  const imuT0 = minFinite(imuT);
  const mocapT1 = maxFinite(mocapT);
  const imuT1 = maxFinite(imuT);
  if (![mocapT0, imuT0, mocapT1, imuT1].every(Number.isFinite)) {
    return empty;
  }

  const t0 = Math.max(mocapT0, imuT0);
  const t1 = Math.min(mocapT1, imuT1);
  if (!(t1 - t0 >= 1.0)) {
    return { ...empty, reason: 'overlap-too-short' };
  }

  const ref = resampleUniform(mocapT, mocapY, dt, t0, t1);
  const sig = resampleUniform(imuT, imuY, dt, t0, t1);
  const maxScanSamples = Math.min(
    Math.floor(rivalScanMaxLagS / dt),
    Math.max(1, Math.floor(ref.length / 3)),
  );
  if (maxScanSamples < 1 || ref.length < 20) {
    return { ...empty, reason: 'too-few-samples' };
  }

  function nccAtLag(lagSamples) {
    let sumR = 0;
    let sumS = 0;
    let sumRR = 0;
    let sumSS = 0;
    let sumRS = 0;
    let n = 0;
    for (let i = 0; i < ref.length; i += 1) {
      const j = i + lagSamples;
      if (j < 0 || j >= sig.length) continue;
      const r = ref[i];
      const s = sig[j];
      if (!Number.isFinite(r) || !Number.isFinite(s)) continue;
      sumR += r;
      sumS += s;
      sumRR += r * r;
      sumSS += s * s;
      sumRS += r * s;
      n += 1;
    }
    if (n < 20) return null;
    const num = n * sumRS - sumR * sumS;
    const den = Math.sqrt((n * sumRR - sumR * sumR) * (n * sumSS - sumS * sumS));
    if (!(den > 0)) return null;
    return num / den;
  }

  // Forced lag (จาก --lag / heel-tap sync)
  if (Number.isFinite(options.lagS)) {
    const lagSamples = Math.round(options.lagS / dt);
    const corr = nccAtLag(lagSamples);
    return {
      lagS: options.lagS,
      peakCorr: corr,
      peakCorrMinus: corr == null ? null : -corr,
      polarity: 1,
      ok: Number.isFinite(corr) && corr >= minPeakCorr,
      ambiguous: false,
      polarityIndeterminate: false,
      periodAliasRisk: false,
      rivalPeaks: [],
      coarseLagS: options.lagS,
      coarseFromOnset: false,
      reason: Number.isFinite(corr) && corr >= minPeakCorr ? null : 'weak-correlation',
      systematicUncertaintyS: SIGNAL_LAG_SYSTEMATIC_UNCERTAINTY_S,
    };
  }

  const corrByLag = new Map();
  let minCorr = Infinity;
  for (let lagSamples = -maxScanSamples; lagSamples <= maxScanSamples; lagSamples += 1) {
    const corr = nccAtLag(lagSamples);
    if (corr == null) continue;
    corrByLag.set(lagSamples, corr);
    if (corr < minCorr) minCorr = corr;
  }

  if (!corrByLag.size) {
    return { ...empty, reason: 'weak-correlation' };
  }

  const peakCorrMinus = Number.isFinite(minCorr) ? -minCorr : null;
  const peaks = listLocalMaxima(corrByLag, minPeakCorr * 0.5);
  const strongPeaks = peaks.filter((p) => p.corr >= minPeakCorr);

  // coarse lag จาก onset (ไม่เป็นคาบ) — หรือ override ชัดเจน (ไม่นับเป็น onset)
  let coarseLagS = Number.isFinite(options.coarseLagS) ? options.coarseLagS : null;
  const hasExplicitCoarse = Number.isFinite(options.coarseLagS);
  let coarseFromOnset = false;
  if (!Number.isFinite(coarseLagS)) {
    const mocapOnset = estimateSignalOnsetS(mocapT, mocapY, options);
    const imuOnset = estimateSignalOnsetS(imuT, imuY, options);
    if (Number.isFinite(mocapOnset) && Number.isFinite(imuOnset)) {
      coarseLagS = imuOnset - mocapOnset;
      coarseFromOnset = true;
    }
  }
  if (!Number.isFinite(coarseLagS)) coarseLagS = 0;
  const trustedCoarse = hasExplicitCoarse || coarseFromOnset;

  const fineSamples = Math.floor(fineMaxLagS / dt);
  const coarseSamples = Math.round(coarseLagS / dt);

  // เลือก peak: ใกล้ coarse ที่สุด ในกลุ่มที่ corr สูงสุด (ยอมใน fine window ก่อน)
  function pickPeak(candidates) {
    if (!candidates.length) return null;
    const bestCorr = Math.max(...candidates.map((p) => p.corr));
    const top = candidates.filter((p) => p.corr >= bestCorr * AMBIGUOUS_PEAK_RATIO);
    top.sort((a, b) => Math.abs(a.lagSamples - coarseSamples) - Math.abs(b.lagSamples - coarseSamples));
    return top[0];
  }

  const inFine = strongPeaks.filter((p) => Math.abs(p.lagSamples - coarseSamples) <= fineSamples);
  let chosen = pickPeak(inFine.length ? inFine : strongPeaks);
  if (!chosen) {
    // ไม่มี peak แข็งพอ — ใช้ max ดิบ
    let bestLagSamples = 0;
    let bestCorr = -Infinity;
    for (const [lagSamples, corr] of corrByLag) {
      if (corr > bestCorr) {
        bestCorr = corr;
        bestLagSamples = lagSamples;
      }
    }
    chosen = { lagSamples: bestLagSamples, corr: bestCorr };
  }

  const bestCorr = chosen.corr;
  const bestLagSamples = chosen.lagSamples;

  const polarityIndeterminate = Number.isFinite(peakCorrMinus)
    && bestCorr >= minPeakCorr
    && Math.abs(peakCorrMinus - bestCorr) <= polarityMargin;

  if (Number.isFinite(peakCorrMinus) && peakCorrMinus > bestCorr + polarityMargin) {
    return {
      ...empty,
      peakCorr: bestCorr,
      peakCorrMinus,
      coarseLagS,
      reason: 'polarity-mismatch',
    };
  }

  if (polarityIndeterminate) {
    return {
      ...empty,
      peakCorr: bestCorr,
      peakCorrMinus,
      coarseLagS,
      ok: false,
      ambiguous: true,
      polarityIndeterminate: true,
      reason: 'ambiguous-polarity',
    };
  }

  if (!(bestCorr >= minPeakCorr)) {
    return {
      ...empty,
      peakCorr: bestCorr,
      peakCorrMinus,
      coarseLagS,
      reason: 'weak-correlation',
    };
  }

  // rival = local max อื่นที่เกือบเท่า — โดยเฉพาะที่ห่างเกิน fine window (= period alias)
  const rivalPeaks = strongPeaks
    .filter((p) => p.lagSamples !== bestLagSamples)
    .filter((p) => p.corr >= bestCorr * AMBIGUOUS_PEAK_RATIO)
    .map((p) => ({
      lagS: p.lagSamples * dt,
      corr: p.corr,
      deltaFromChosenS: (p.lagSamples - bestLagSamples) * dt,
    }));
  const periodAliasRisk = rivalPeaks.some((r) => Math.abs(r.deltaFromChosenS) > fineMaxLagS * 0.5);
  const ambiguous = rivalPeaks.length > 0;

  const ym1 = corrByLag.get(bestLagSamples - 1);
  const y0 = bestCorr;
  const yp1 = corrByLag.get(bestLagSamples + 1);
  let frac = 0;
  if (Number.isFinite(ym1) && Number.isFinite(yp1)) {
    const denom = ym1 - 2 * y0 + yp1;
    if (Math.abs(denom) > 1e-12) {
      frac = clampFrac(0.5 * (ym1 - yp1) / denom, -0.75, 0.75);
    }
  }

  const lagS = (bestLagSamples + frac) * dt;
  const farFromCoarse = Math.abs(lagS - coarseLagS) > fineMaxLagS + 0.05;
  // ถ้าไม่มี onset / explicit coarse / --lag แล้วยังมี rival ±n·stride → ห้ามเลือก alias ใกล้ 0 แบบเงียบ
  const refuseForAlias = periodAliasRisk && !trustedCoarse;
  const ok = !farFromCoarse && !refuseForAlias;
  // reason = สาเหตุที่ปฏิเสธเท่านั้น; ข้อสังเกตอยู่ที่ periodAliasRisk / ambiguous
  let reason = null;
  if (!ok) {
    if (farFromCoarse) reason = 'lag-far-from-coarse-onset';
    else if (refuseForAlias) reason = 'period-alias-rivals';
    else if (ambiguous) reason = 'ambiguous-period-peaks';
  }

  return {
    lagS,
    peakCorr: bestCorr,
    peakCorrMinus,
    polarity: 1,
    ok,
    ambiguous,
    polarityIndeterminate: false,
    periodAliasRisk,
    rivalPeaks,
    coarseLagS,
    coarseFromOnset,
    reason,
    systematicUncertaintyS: SIGNAL_LAG_SYSTEMATIC_UNCERTAINTY_S,
  };
}

function clampFrac(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

/** ดึง gx (canonical) ของข้างหนึ่งจาก IMU trace เป็น series เวลา session-relative */
export function extractImuGyroSeries(trace, side, options = {}) {
  const samples = Array.isArray(trace?.samples) ? trace.samples : [];
  const preferSensorFrame = options.preferSensorFrame ?? Boolean(trace?.hasSensorFrameRaw);
  const axisMap = options.axisMap || trace?.axisMap || undefined;
  const rawT = [];
  const gx = [];

  for (const sample of samples) {
    const input = sampleToProcessorInput(sample, { axisMap, preferSensorFrame });
    if (!input || input.side !== side) continue;
    if (!Number.isFinite(input.timestampMs) || !Number.isFinite(input.gx)) continue;
    rawT.push(input.timestampMs);
    gx.push(input.gx);
  }

  if (rawT.length < 20) return { tS: [], gx: [], ok: false };

  // unwrap micros wrap + relative ต่อ sample แรก
  const tS = [];
  let origin = rawT[0];
  let accumulated = 0;
  let last = rawT[0];
  for (let i = 0; i < rawT.length; i += 1) {
    const valueMs = rawT[i];
    if (valueMs < last - 1000) {
      accumulated += (last - origin) + 10;
      origin = valueMs;
    }
    tS.push((accumulated + (valueMs - origin)) / 1000);
    last = valueMs;
  }

  return { tS, gx, ok: true };
}

/**
 * จับคู่ 1:1 ตามเวลาหลังชดเชย lag — greedy nearest ใน tolerance
 * ถ้า options.lagS ถูกกำหนด (เช่น จาก signal xcorr) จะใช้ค่านี้แทนการประมาณจาก HS events
 * ถ้า HS-event lag ปฏิเสธ (period alias ไม่มี coarse) → ไม่จับคู่ (pairs=[]) ดีกว่าคู่ผิด stride
 */
export function pairCyclesByTime(mocapCycles, imuCycles, options = {}) {
  const matchToleranceS = options.matchToleranceS ?? DEFAULT_MATCH_TOLERANCE_S;
  let lagSource = 'hs-event';
  let lagInfo;

  if (Number.isFinite(options.lagS)) {
    lagInfo = {
      lagS: options.lagS,
      matchCount: null,
      tiedLags: [options.lagS],
      ambiguous: false,
      periodAliasRisk: false,
      rivalLags: [],
      ok: true,
      reason: null,
      coarseLagS: options.coarseLagS ?? options.lagS,
    };
    lagSource = options.lagSource || 'external';
  } else {
    lagInfo = estimateHsTimeLagS(mocapCycles, imuCycles, options);
    lagSource = 'hs-event';
  }

  const {
    lagS,
    matchCount: lagMatchHint,
    tiedLags,
    ambiguous,
    periodAliasRisk = false,
    rivalLags = [],
    ok: lagOk = true,
    reason: lagReason = null,
    coarseLagS = null,
  } = lagInfo;

  const emptyPairs = {
    lagS,
    lagSource,
    lagMatchHint,
    tiedLags: tiedLags || [],
    lagAmbiguous: Boolean(ambiguous) || Boolean(periodAliasRisk),
    periodAliasRisk: Boolean(periodAliasRisk),
    rivalLags,
    lagOk: false,
    lagReason,
    coarseLagS,
    pairs: [],
    unpairedMocap: mocapCycles.filter((c) => Number.isFinite(mocapCycleTimeS(c))).length,
    unpairedImu: imuCycles.filter((c) => Number.isFinite(imuCycleTimeS(c))).length,
    timingMetricValid: false,
    meanTimeErrorS: null,
    medianTimeErrorS: null,
  };

  // ปฏิเสธจับคู่เมื่อ HS path ไม่มั่นใจ — อย่าส่งคู่ผิด n สไตรด์ต่อไปคำนวณ metrics
  if (lagSource === 'hs-event' && lagOk === false) {
    return emptyPairs;
  }

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

  // residual HS timing ใช้เป็นเมตริก detector ได้เฉพาะเมื่อ lag มาจากสัญญาณ (ไม่ใช่ HS เอง)
  const timingMetricValid = lagSource === 'signal-xcorr' || lagSource === 'gyro-xcorr';

  return {
    lagS,
    lagSource,
    lagMatchHint,
    tiedLags: tiedLags || [],
    lagAmbiguous: Boolean(ambiguous),
    periodAliasRisk: Boolean(periodAliasRisk),
    rivalLags,
    lagOk: true,
    lagReason: null,
    coarseLagS,
    pairs,
    unpairedMocap: mocap.length - pairs.length,
    unpairedImu: imu.length - pairs.length,
    timingMetricValid,
    meanTimeErrorS: timingMetricValid ? mean(pairs.map((p) => p.timeErrorS)) : null,
    medianTimeErrorS: timingMetricValid ? median(pairs.map((p) => p.timeErrorS)) : null,
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

function compareMeans(mocapSummary, imuSummary, options = {}) {
  const allowAgreement = options.allowAgreement !== false;
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
    const finite = Number.isFinite(mocap) && Number.isFinite(imu);
    const primary = spec.agreementClass === 'primary';
    const comparable = allowAgreement && finite && primary;
    let ineligibleReason = null;
    if (!allowAgreement) ineligibleReason = 'pairing-not-trusted';
    else if (!finite) ineligibleReason = 'missing-values';
    else if (!primary) ineligibleReason = spec.ineligibleReason || 'exploratory-metric';
    return {
      metric: spec.key,
      unit: spec.unit,
      agreementClass: spec.agreementClass,
      mocap,
      imu,
      error: comparable ? absError(imu, mocap) : null,
      errorPct: comparable ? pctError(imu, mocap) : null,
      comparable,
      ineligibleReason,
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
      validationPublishable: false,
    };
  }

  const maxZuptDevG = options.maxZuptDevG ?? DEFAULT_MAX_ZUPT_DEV_G;
  const minAgreementPairs = options.minAgreementPairs ?? DEFAULT_MIN_AGREEMENT_PAIRS;
  const explicitLag = Number.isFinite(options.align?.lagS);

  for (const flag of imu.qualityFlags || []) {
    if (flag.usedSyntheticTimestamps || flag.missingTimestampCount > 0) {
      warnings.push(
        `เซนเซอร์ ${flag.sensorKey}: มี sample ไม่มี timestamp `
        + `(missing=${flag.missingTimestampCount}) — เวลาถูก interpolate; `
        + 'ห้ามใช้ผลนี้เป็น lab validation (ต้องมี t_ms จาก firmware)',
      );
    }
    if (flag.skippedIncompleteSampleCount > 0) {
      warnings.push(
        `เซนเซอร์ ${flag.sensorKey}: ข้าม ${flag.skippedIncompleteSampleCount} sample ที่ขาดแกน accel/gyro`,
      );
    }
  }
  const hasSyntheticTimestamps = (imu.qualityFlags || []).some((f) => f.usedSyntheticTimestamps);

  const sides = {};
  for (const side of ['L', 'R']) {
    const mocapCycles = mocap?.perSide?.[side]?.cycles || [];
    const imuCycles = imu.bySide[side] || [];
    const mocapSumAll = summarizeMocapSide({ cycles: mocapCycles });
    const imuSumAll = summarizeImuSide(imuCycles);

    if (mocapCycles.length === 0 && imuCycles.length === 0) {
      sides[side] = {
        present: false,
        mocap: mocapSumAll,
        imu: imuSumAll,
        metrics: [],
        alignment: null,
        agreementPairCount: 0,
        validationPublishable: false,
      };
      continue;
    }

    if (mocapCycles.length === 0) warnings.push(`ขา ${side}: มี IMU แต่ไม่มี MoCap cycles`);
    if (imuCycles.length === 0) warnings.push(`ขา ${side}: มี MoCap แต่ไม่มี IMU cycles`);
    if (imuSumAll.clampedCount > 0) {
      warnings.push(`ขา ${side}: IMU มี ${imuSumAll.clampedCount}/${imuSumAll.cycleCount} cycle ที่ stride ถูก clamp`);
    }

    // 1) พยายามประมาณ lag จากสัญญาณ (mocap ω vs IMU gx) — อิสระจาก event detector
    const alignOpts = { ...(options.align || {}) };
    const mocapSignals = mocap?.perSide?.[side]?.signals;
    const imuGyro = extractImuGyroSeries(imuTrace, side, options);
    let signalLag = null;
    if (
      mocapSignals?.tS?.length
      && mocapSignals?.shankAngularVelocityDps?.length
      && imuGyro.ok
    ) {
      signalLag = estimateSignalLagS(
        mocapSignals.tS,
        mocapSignals.shankAngularVelocityDps,
        imuGyro.tS,
        imuGyro.gx,
        alignOpts,
      );
      if (signalLag.ok) {
        alignOpts.lagS = signalLag.lagS;
        alignOpts.lagSource = 'signal-xcorr';
        alignOpts.coarseLagS = signalLag.coarseLagS;
        if (signalLag.periodAliasRisk) {
          warnings.push(
            `ขา ${side}: signal xcorr มี rival peaks ใกล้เคียงที่ ±n·stride `
            + `(${(signalLag.rivalPeaks || []).slice(0, 3).map((p) => `${p.lagS.toFixed(2)}s`).join(', ')}) `
            + `— เลือก lag=${signalLag.lagS.toFixed(3)}s จาก coarse=${Number.isFinite(signalLag.coarseLagS) ? signalLag.coarseLagS.toFixed(3) : '—'}s; `
            + 'HS timing อย่าเพิ่งเชื่อจนกว่าจะมี heel-tap/--lag; แนะนำ onset ยืน→เดินหรือ sync marker',
          );
        }
      } else {
        // เก็บ coarse จาก onset ไว้ให้ HS path — อย่าทิ้งทั้งที่หาได้แล้ว
        if (signalLag.coarseFromOnset && Number.isFinite(signalLag.coarseLagS)) {
          alignOpts.coarseLagS = signalLag.coarseLagS;
        }
        const detail = signalLag.reason === 'polarity-mismatch' || signalLag.reason === 'inverted-polarity'
          ? `peak(+)=${Number.isFinite(signalLag.peakCorr) ? signalLag.peakCorr.toFixed(2) : '—'} `
            + `< peak(−)=${Number.isFinite(signalLag.peakCorrMinus) ? signalLag.peakCorrMinus.toFixed(2) : '—'} `
            + '— ตรวจ axis map / นิยามมุม (ไม่เงียบ flip)'
          : signalLag.reason === 'ambiguous-polarity'
            ? `peak(+)/peak(−) ใกล้กัน (${Number.isFinite(signalLag.peakCorr) ? signalLag.peakCorr.toFixed(2) : '—'}/`
              + `${Number.isFinite(signalLag.peakCorrMinus) ? signalLag.peakCorrMinus.toFixed(2) : '—'})`
            : signalLag.reason === 'period-alias-rivals'
              ? `period alias rivals โดยไม่มี onset sync — ใส่ --lag หรือ heel-tap `
                + `(candidates ${(signalLag.rivalPeaks || []).slice(0, 4).map((p) => `${p.lagS.toFixed(2)}s`).join(', ') || `lagS=${signalLag.lagS}`})`
              : signalLag.reason === 'ambiguous-period-peaks'
                ? `peak ใกล้เคียงกันหลายจุด (rival ${signalLag.rivalPeaks?.slice(0, 3).map((p) => `${p.lagS.toFixed(3)}s`).join(', ')})`
                : signalLag.reason;
        const pairingRisk = signalLag.periodAliasRisk
          ? ' — การจับคู่ cycle อาจคลาด n สไตรด์ถ้า HS path ไม่มี coarse; จะปฏิเสธจับคู่ถ้า HS ก็ alias'
          : '';
        warnings.push(
          `ขา ${side}: signal xcorr ใช้ไม่ได้ (${detail}) — ลอง HS-event lag`
          + `${Number.isFinite(alignOpts.coarseLagS) ? ` (ส่ง coarseLag=${alignOpts.coarseLagS.toFixed(3)}s จาก onset)` : ''}`
          + pairingRisk
          + ' (residual timing ไม่ใช่เมตริก detector bias)',
        );
      }
    } else if (mocapCycles.length && imuCycles.length) {
      warnings.push(
        `ขา ${side}: ไม่มี MoCap signals.shankAngularVelocityDps หรือ IMU samples `
        + '— align ด้วย HS-event lag (อย่าอ่าน residual เป็น timing bias)',
      );
    }

    const alignment = pairCyclesByTime(mocapCycles, imuCycles, alignOpts);
    const rawPaired = alignment.pairs.length > 0;
    const agreementPairs = alignment.pairs.filter((p) => (
      isAgreementQualityImuCycle(p.imu, { maxZuptDevG })
    ));
    const excludedPairCount = alignment.pairs.length - agreementPairs.length;
    if (excludedPairCount > 0) {
      warnings.push(
        `ขา ${side}: ตัด ${excludedPairCount}/${alignment.pairs.length} คู่ ออกจาก agreement `
        + '(clamp / ZUPT deviation / stance fallback) — เหลือใช้ได้ '
        + `${agreementPairs.length} คู่`,
      );
    }

    const syncTrusted = Boolean(
      explicitLag
      || alignment.lagSource === 'signal-xcorr'
      || (signalLag?.coarseFromOnset && alignment.lagOk !== false && Number.isFinite(alignment.lagS))
    );
    // session-mean หรือ pairing ปฏิเสธ → ห้าม comparable metrics (เคยทำให้ error% ดูสวยทั้งที่ผิด)
    const usePairedAgreement = rawPaired
      && alignment.lagOk !== false
      && syncTrusted
      && agreementPairs.length >= minAgreementPairs
      && !hasSyntheticTimestamps;

    if (alignment.lagOk === false && alignment.periodAliasRisk) {
      warnings.push(
        `ขา ${side}: HS-event lag ปฏิเสธเพราะ period-alias rivals `
        + `(เลือกได้ ${Number.isFinite(alignment.lagS) ? alignment.lagS.toFixed(3) : '—'}s แต่มี `
        + `${(alignment.rivalLags || []).slice(0, 3).map((r) => `${r.lagS.toFixed(2)}s`).join(', ') || 'rivals'}) `
        + '— ไม่จับคู่ cycle; ใส่ --lag / heel-tap ก่อนเทียบ — ห้ามใช้ session-mean เป็น validation',
      );
    } else if (!rawPaired && mocapCycles.length && imuCycles.length) {
      warnings.push(
        `ขา ${side}: จับคู่ตามเวลาไม่ได้ — แสดงค่าเฉลี่ย session แบบ informational เท่านั้น `
        + '(comparable=false; ห้ามใส่เปเปอร์)',
      );
    } else if (rawPaired && !syncTrusted) {
      warnings.push(
        `ขา ${side}: จับคู่ได้แต่ sync ไม่ trusted (ไม่มี --lag / signal-ok / onset coarse) `
        + '— comparable=false จนกว่าจะมี heel-tap หรือ onset',
      );
    } else if (rawPaired && (alignment.unpairedMocap > 0 || alignment.unpairedImu > 0)) {
      warnings.push(
        `ขา ${side}: จับคู่ได้ ${alignment.pairs.length} คู่ `
        + `(MoCap ค้าง ${alignment.unpairedMocap}, IMU ค้าง ${alignment.unpairedImu}, `
        + `lag=${alignment.lagS.toFixed(2)}s via ${alignment.lagSource})`,
      );
    }
    if (alignment.lagAmbiguous && alignment.lagOk !== false) {
      warnings.push(
        `ขา ${side}: มี lag หลายค่าที่ match เท่ากัน (${alignment.tiedLags.slice(0, 5).join(', ')}…) `
        + '— เลือกจาก residual ต่ำสุดแล้ว แต่ควรตรวจ sync',
      );
    }

    const pairedSummary = usePairedAgreement ? summarizePaired(agreementPairs) : null;
    const mocapForCompare = pairedSummary?.mocap ?? mocapSumAll;
    const imuForCompare = pairedSummary?.imu ?? imuSumAll;
    const metrics = compareMeans(mocapForCompare, imuForCompare, {
      allowAgreement: usePairedAgreement,
    });

    const nullStance = usePairedAgreement
      ? agreementPairs.filter((p) => !Number.isFinite(p.imu.stancePct)).length
      : imuCycles.filter((c) => !Number.isFinite(c.stancePct)).length;
    if (nullStance > 0 && imuCycles.length > 0) {
      warnings.push(`ขา ${side}: IMU มี ${nullStance} cycle ที่ stancePct=null (temporal unresolved)`);
    }

    const sidePublishable = Boolean(
      usePairedAgreement
      && metrics.some((m) => m.comparable)
      && !hasSyntheticTimestamps
    );

    sides[side] = {
      present: true,
      mocap: mocapForCompare,
      imu: imuForCompare,
      mocapAll: mocapSumAll,
      imuAll: imuSumAll,
      metrics,
      cycleCountDelta: imuSumAll.cycleCount - mocapSumAll.cycleCount,
      agreementPairCount: agreementPairs.length,
      excludedPairCount,
      validationPublishable: sidePublishable,
      alignment: {
        mode: usePairedAgreement
          ? (alignment.lagSource === 'signal-xcorr' ? 'signal-xcorr+hs-pair' : 'hs-event-lag')
          : (rawPaired ? 'paired-untrusted' : 'session-mean-informational'),
        lagS: alignment.lagS,
        lagSource: alignment.lagSource,
        lagOk: alignment.lagOk !== false,
        syncTrusted,
        coarseLagS: alignment.coarseLagS ?? signalLag?.coarseLagS ?? null,
        coarseFromOnset: Boolean(signalLag?.coarseFromOnset),
        signalPeakCorr: signalLag?.ok ? signalLag.peakCorr : null,
        peakCorrMinus: signalLag?.peakCorrMinus ?? null,
        polarityIndeterminate: signalLag?.polarityIndeterminate ?? false,
        systematicUncertaintyS: signalLag?.systematicUncertaintyS ?? null,
        pairedCount: alignment.pairs.length,
        agreementPairCount: agreementPairs.length,
        unpairedMocap: alignment.unpairedMocap,
        unpairedImu: alignment.unpairedImu,
        timingMetricValid: Boolean(
          usePairedAgreement
          && alignment.timingMetricValid
          && !signalLag?.periodAliasRisk
          && !alignment.periodAliasRisk
        ),
        periodAliasRisk: Boolean(signalLag?.periodAliasRisk || alignment.periodAliasRisk),
        meanTimeErrorS: alignment.meanTimeErrorS,
        medianTimeErrorS: alignment.medianTimeErrorS,
      },
    };
  }

  const presentSides = ['L', 'R'].filter((s) => sides[s]?.present);
  const validationPublishable = presentSides.length > 0
    && presentSides.every((s) => sides[s].validationPublishable)
    && !hasSyntheticTimestamps;

  const labChecklist = [
    {
      id: 'heel-tap-or-lag',
      ok: explicitLag || presentSides.some((s) => sides[s].alignment?.coarseFromOnset || sides[s].alignment?.lagSource === 'signal-xcorr'),
      detail: 'heel-tap 1 ครั้งก่อนเดิน + ใส่ --lag หรือมี onset/signal sync ที่ผ่าน',
    },
    {
      id: 'paired-agreement-cycles',
      ok: presentSides.every((s) => (sides[s].agreementPairCount || 0) >= minAgreementPairs),
      detail: `อย่างน้อย ${minAgreementPairs} คู่คุณภาพต่อข้าง (ไม่ clamp / ZUPT ดี / stance วัดจริง)`,
    },
    {
      id: 'no-synthetic-timestamps',
      ok: !hasSyntheticTimestamps,
      detail: 'ทุก sample ต้องมี t_ms จาก firmware',
    },
    {
      id: 'primary-metrics-only',
      ok: true,
      detail: 'เคลมได้เฉพาะ strideLength / cadence / walkingSpeed — stance% และ peak° เป็น exploratory',
    },
    {
      id: 'axis-map-verified',
      ok: null,
      detail: 'ตรวจแกน L/R ด้วย swing test ก่อนเก็บข้อมูล (ไม่ auto-verify ในโค้ด)',
    },
  ];

  if (!validationPublishable) {
    warnings.unshift(
      '⛔ validationPublishable=false — ห้ามใช้ error%/ICC จากรายงานนี้ในเปเปอร์หรือ ethics '
      + 'จนกว่า lab checklist จะผ่าน (ดู report.labChecklist); ใช้ --lab เพื่อบังคับ fail ตอน CI/รันแลป',
    );
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
    validationPublishable,
    labChecklist,
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
      'validationPublishable=true เท่านั้นถึงจะเอา primary error% ไปใช้ในเปเปอร์ — มิฉะนั้นเป็น informational',
      'Primary agreement: strideLength / cadence / walkingSpeed หลังจับคู่+กรองคุณภาพ',
      'Exploratory (comparable=false): stancePct (คนละนิยาม event), peakShankAngleDeg (mounting offset)',
      'Clearance ไม่เทียบอัตโนมัติ — MoCap วัดข้อเท้า, IMU double-integrate คนละตำแหน่ง',
      'Align: signal xcorr → HS-event; period alias โดยไม่มี coarse/--lag จะไม่จับคู่',
      'SOP แลป: heel-tap 1 ครั้งก่อนเดิน → ใส่ --lag; ยืนนิ่ง≥0.4s ก่อนเดินเพื่อ onset; ยืนยัน axis map',
      'stepLength=stride/2 และ doubleSupport≈2·stance−100 สมมติสมมาตร L/R — พังกับ stroke',
      'sum(stride) ต่อข้าง ≠ ระยะเดินจริงแบบ 1:1 ถ้าสองข้างบันทึกพร้อมกัน (อย่าบวก L+R)',
    ],
  };
}

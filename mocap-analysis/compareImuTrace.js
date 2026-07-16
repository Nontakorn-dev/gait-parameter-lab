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
 * Tie-break: ถ้า match count เท่ากัน เลือก lag ที่ residual รวมเล็กสุด (ไม่ seed 0)
 */
export function estimateHsTimeLagS(mocapCycles, imuCycles, options = {}) {
  const maxLagS = options.maxLagS ?? DEFAULT_MAX_LAG_S;
  const matchToleranceS = options.matchToleranceS ?? DEFAULT_MATCH_TOLERANCE_S;
  const mocapTimes = mocapCycles.map(mocapCycleTimeS).filter(Number.isFinite).sort((a, b) => a - b);
  const imuTimes = imuCycles.map(imuCycleTimeS).filter(Number.isFinite).sort((a, b) => a - b);
  if (!mocapTimes.length || !imuTimes.length) {
    return { lagS: 0, matchCount: 0, tiedLags: [], ambiguous: false };
  }

  // ไม่ seed ด้วย 0 — ไม่งั้น insertion-order + strict > ทำให้ 0 ชนะ tie เสมอ
  const candidates = new Set();
  const step = Math.max(1, Math.floor(imuTimes.length / 12));
  for (let i = 0; i < imuTimes.length; i += step) {
    for (let j = 0; j < mocapTimes.length; j += step) {
      const lag = imuTimes[i] - mocapTimes[j];
      if (Math.abs(lag) <= maxLagS) candidates.add(Math.round(lag * 100) / 100);
    }
  }
  if (!candidates.size) candidates.add(0);

  let bestLag = 0;
  let bestCount = -1;
  let bestResidual = Infinity;
  const scored = [];

  for (const lag of candidates) {
    const { count, residualAbsSum } = countHsMatches(mocapTimes, imuTimes, lag, matchToleranceS);
    scored.push({ lag, count, residualAbsSum });
    const betterCount = count > bestCount;
    const betterResidual = count === bestCount && residualAbsSum < bestResidual - 1e-12;
    if (betterCount || betterResidual) {
      bestCount = count;
      bestLag = lag;
      bestResidual = residualAbsSum;
    }
  }

  const tiedLags = scored
    .filter((s) => s.count === bestCount)
    .map((s) => s.lag)
    .sort((a, b) => a - b);
  // ambiguous ถ้ามี lag อื่นที่ match เท่ากันและ residual ใกล้เคียง (±5%)
  const ambiguous = tiedLags.filter((lag) => {
    const row = scored.find((s) => s.lag === lag);
    return row && Math.abs(row.residualAbsSum - bestResidual) <= Math.max(1e-6, bestResidual * 0.05);
  }).length > 1;

  return {
    lagS: bestLag,
    matchCount: Math.max(0, bestCount),
    tiedLags,
    ambiguous,
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
 * ประมาณ lag (IMU − MoCap) จาก normalized cross-correlation ของสัญญาณ
 * อิสระจาก event detector → residual HS หลัง align นี้วัด detector timing bias ได้
 *
 * @returns {{ lagS, peakCorr, polarity, ok }}
 */
export function estimateSignalLagS(mocapT, mocapY, imuT, imuY, options = {}) {
  const maxLagS = options.maxLagS ?? DEFAULT_MAX_LAG_S;
  const dt = options.dtS ?? 0.02; // 50 Hz
  if (!mocapT?.length || !imuT?.length || mocapT.length !== mocapY.length || imuT.length !== imuY.length) {
    return { lagS: null, peakCorr: null, polarity: 1, ok: false, reason: 'missing-series' };
  }

  const t0 = Math.max(
    Math.min(...mocapT.filter(Number.isFinite)),
    Math.min(...imuT.filter(Number.isFinite)),
  );
  const t1 = Math.min(
    Math.max(...mocapT.filter(Number.isFinite)),
    Math.max(...imuT.filter(Number.isFinite)),
  );
  if (!(t1 - t0 >= 1.0)) {
    return { lagS: null, peakCorr: null, polarity: 1, ok: false, reason: 'overlap-too-short' };
  }

  const ref = resampleUniform(mocapT, mocapY, dt, t0, t1);
  const sig = resampleUniform(imuT, imuY, dt, t0, t1);
  const maxLagSamples = Math.min(Math.floor(maxLagS / dt), Math.floor(ref.length / 3));
  if (maxLagSamples < 1 || ref.length < 20) {
    return { lagS: null, peakCorr: null, polarity: 1, ok: false, reason: 'too-few-samples' };
  }

  function nccAtLag(polarity, lagSamples) {
    // corr(ref[i], polarity * sig[i + lagSamples]) — lagSamples>0 = IMU ช้ากว่า (เหตุการณ์มาทีหลัง)
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
      const sp = polarity * s;
      sumR += r;
      sumS += sp;
      sumRR += r * r;
      sumSS += sp * sp;
      sumRS += r * sp;
      n += 1;
    }
    if (n < 20) return null;
    const num = n * sumRS - sumR * sumS;
    const den = Math.sqrt((n * sumRR - sumR * sumR) * (n * sumSS - sumS * sumS));
    if (!(den > 0)) return null;
    return num / den;
  }

  let best = { lagSamples: 0, corr: -Infinity, polarity: 1 };
  for (const polarity of [1, -1]) {
    for (let lagSamples = -maxLagSamples; lagSamples <= maxLagSamples; lagSamples += 1) {
      const corr = nccAtLag(polarity, lagSamples);
      if (corr == null) continue;
      if (corr > best.corr) {
        best = { lagSamples, corr, polarity };
      }
    }
  }

  if (!(best.corr > 0.15)) {
    return {
      lagS: null,
      peakCorr: best.corr === -Infinity ? null : best.corr,
      polarity: best.polarity,
      ok: false,
      reason: 'weak-correlation',
    };
  }

  return {
    lagS: best.lagSamples * dt,
    peakCorr: best.corr,
    polarity: best.polarity,
    ok: true,
    reason: null,
  };
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
    };
    lagSource = options.lagSource || 'external';
  } else {
    lagInfo = estimateHsTimeLagS(mocapCycles, imuCycles, options);
    lagSource = 'hs-event';
  }

  const { lagS, matchCount: lagMatchHint, tiedLags, ambiguous } = lagInfo;

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
      } else {
        warnings.push(
          `ขา ${side}: signal xcorr ใช้ไม่ได้ (${signalLag.reason}) — fallback เป็น HS-event lag `
          + '(residual timing ไม่ใช่เมตริก detector bias)',
        );
      }
    } else if (mocapCycles.length && imuCycles.length) {
      warnings.push(
        `ขา ${side}: ไม่มี MoCap signals.shankAngularVelocityDps หรือ IMU samples `
        + '— align ด้วย HS-event lag (อย่าอ่าน residual เป็น timing bias)',
      );
    }

    const alignment = pairCyclesByTime(mocapCycles, imuCycles, alignOpts);
    const usePaired = alignment.pairs.length > 0;
    if (!usePaired && mocapCycles.length && imuCycles.length) {
      warnings.push(`ขา ${side}: จับคู่ตามเวลาไม่ได้ — fallback เป็นค่าเฉลี่ยทั้ง session (อาจรวมช่วงยืนนิ่ง)`);
    } else if (usePaired && (alignment.unpairedMocap > 0 || alignment.unpairedImu > 0)) {
      warnings.push(
        `ขา ${side}: จับคู่ได้ ${alignment.pairs.length} คู่ `
        + `(MoCap ค้าง ${alignment.unpairedMocap}, IMU ค้าง ${alignment.unpairedImu}, `
        + `lag=${alignment.lagS.toFixed(2)}s via ${alignment.lagSource})`,
      );
    }
    if (alignment.lagAmbiguous) {
      warnings.push(
        `ขา ${side}: มี lag หลายค่าที่ match เท่ากัน (${alignment.tiedLags.slice(0, 5).join(', ')}…) `
        + '— เลือกจาก residual ต่ำสุดแล้ว แต่ควรตรวจ sync',
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
        mode: usePaired
          ? (alignment.lagSource === 'signal-xcorr' ? 'signal-xcorr+hs-pair' : 'hs-event-lag')
          : 'session-mean-fallback',
        lagS: alignment.lagS,
        lagSource: alignment.lagSource,
        signalPeakCorr: signalLag?.ok ? signalLag.peakCorr : null,
        pairedCount: alignment.pairs.length,
        unpairedMocap: alignment.unpairedMocap,
        unpairedImu: alignment.unpairedImu,
        // HS timing bias ใช้ได้เฉพาะเมื่อ sync มาจากสัญญาณ — ไม่ใช่จาก HS events เอง
        timingMetricValid: alignment.timingMetricValid,
        meanTimeErrorS: alignment.meanTimeErrorS,
        medianTimeErrorS: alignment.medianTimeErrorS,
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
      'Align ลำดับ: signal xcorr (mocap ω × IMU gx) ก่อน แล้วค่อย HS-event lag สำหรับจับคู่ cycle',
      'HS timing residual ใช้ได้เฉพาะเมื่อ lagSource=signal-xcorr — ถ้า sync จาก HS events เอง bias ถูกดูดเข้า lag',
      'sum(stride) ต่อข้าง ≠ ระยะเดินจริงแบบ 1:1 ถ้าสองข้างบันทึกพร้อมกัน (อย่าบวก L+R)',
    ],
  };
}

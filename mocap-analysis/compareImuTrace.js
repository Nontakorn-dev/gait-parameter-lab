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
import { GAIT_ANALYZE_EVERY_SAMPLES } from '../src/gait/gaitRuntimeConfig.js';
import { computeCycleTimeCoverage, parseCycleKeyStartId } from '../src/gait/cycleCoverage.js';

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
/** |v_end| ก่อน de-drift — สูง = gravity leakage; สอดคล้อง gaitProcessor.MAX_V_END_PRE_DRIFT_MPS */
const DEFAULT_MAX_V_END_PRE_DRIFT_MPS = 2.0;
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
    strideLengthUntrusted: params.strideLengthUntrusted ?? false,
    suspectedMissedHs: Boolean(params.suspectedMissedHs),
    isOpenStride: Boolean(params.isOpenStride),
    strideLengthSignedM: params.strideLengthSignedM ?? null,
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
    cycleEndTimestampMs: params.cycleEndTimestampMs ?? null,
    cycleStartTimeS: Number.isFinite(params.cycleStartTimestampMs)
      ? params.cycleStartTimestampMs / 1000
      : null,
  };
}

function cycleEntryFromDiagnostic(d, side, sensorKey, firstT) {
  let cycleStartTimeS = null;
  if (Number.isFinite(d.cycleStartTimestampMs)) {
    if (Number.isFinite(firstT) && firstT >= 946684800000) {
      cycleStartTimeS = (d.cycleStartTimestampMs - firstT) / 1000;
    } else {
      cycleStartTimeS = d.cycleStartTimestampMs / 1000;
    }
  }
  return {
    cycleKey: String(d.cycleKey),
    side,
    sensorKey,
    strideLengthM: d.strideLengthM ?? null,
    strideLengthClamped: d.strideLengthClamped ?? false,
    strideLengthUntrusted: d.strideLengthUntrusted ?? false,
    suspectedMissedHs: Boolean(d.suspectedMissedHs),
    isOpenStride: Boolean(d.isOpenStride),
    strideLengthSignedM: null,
    cadenceSpm: d.cadenceSpm ?? null,
    walkingSpeedMps: d.walkingSpeedMps ?? null,
    stancePct: d.stancePct ?? null,
    swingPct: d.swingPct ?? null,
    strideTimeS: d.strideTimeS ?? null,
    peakShankAngleDeg: d.peakShankAngleDeg ?? null,
    clearanceM: d.clearanceM ?? null,
    temporalSource: d.temporalSource ?? null,
    zuptAccelDeviationG: d.zuptCheck?.zuptAccelDeviationG ?? null,
    zuptCheck: d.zuptCheck ?? null,
    cycleStartTimestampMs: d.cycleStartTimestampMs ?? null,
    cycleEndTimestampMs: d.cycleEndTimestampMs ?? null,
    cycleStartSampleId: d.cycleStartSampleId ?? null,
    cycleEndSampleId: d.cycleEndSampleId ?? null,
    cycleStartTimeS,
  };
}

function shouldUpgradeCycle(existing, next) {
  // อัปเดตเมื่อได้ temporal metrics ที่เคยเป็น null (open/unresolved → measured)
  if (!Number.isFinite(existing.stancePct) && Number.isFinite(next.stancePct)) return true;
  if (!Number.isFinite(existing.strideLengthM) && Number.isFinite(next.strideLengthM)) return true;
  // อัปเกรดคุณภาพ temporal / ZUPT metadata
  if (existing.temporalSource !== 'measured-to' && next.temporalSource === 'measured-to') return true;
  if (!existing.zuptCheck?.vEndPreDrift && next.zuptCheck?.vEndPreDrift != null) return true;
  if (existing.isOpenStride && next.isOpenStride === false) return true;
  return false;
}

/** ก้าวที่เอาเข้า agreement stats ได้ — ตัด clamp / ZUPT พัง / stance หลอก / open / v_end สูง / missed-HS */
export function isAgreementQualityImuCycle(cycle, options = {}) {
  if (!cycle) return false;
  if (cycle.isOpenStride) return false;
  if (cycle.strideLengthClamped) return false;
  if (cycle.strideLengthUntrusted) return false;
  if (cycle.suspectedMissedHs) return false;
  if (!Number.isFinite(cycle.strideLengthM)) return false;
  const temporal = cycle.temporalSource;
  if (temporal === 'previous-valid-ratio' || temporal === 'unresolved') return false;
  const maxZupt = options.maxZuptDevG ?? DEFAULT_MAX_ZUPT_DEV_G;
  const zupt = cycle.zuptAccelDeviationG
    ?? cycle.zuptCheck?.zuptAccelDeviationG
    ?? null;
  if (Number.isFinite(zupt) && zupt > maxZupt) return false;
  const maxVEnd = options.maxVEndPreDriftMps ?? DEFAULT_MAX_V_END_PRE_DRIFT_MPS;
  const vEnd = cycle.zuptCheck?.vEndPreDrift ?? null;
  if (Number.isFinite(vEnd) && Math.abs(vEnd) > maxVEnd) return false;
  return true;
}

/**
 * ตัด cycle ที่ช่วงเวลาทับซ้อนกัน — เก็บอันที่สั้นกว่า (แยก double-stride ที่พลาด HS)
 * ใช้หลัง streaming reprocess เผื่อ diagnostic เก่าถูก emit ก่อน overlap guard แทนที่
 */
export function resolveOverlappingImuCycles(entries) {
  if (!Array.isArray(entries) || entries.length < 2) return entries || [];
  const indexed = entries.map((e, i) => {
    const t0 = Number.isFinite(e.cycleStartTimeS) ? e.cycleStartTimeS : null;
    let t1 = null;
    if (Number.isFinite(e.cycleEndTimestampMs) && Number.isFinite(e.cycleStartTimestampMs)) {
      const dt = (e.cycleEndTimestampMs - e.cycleStartTimestampMs) / 1000;
      if (dt > 0) t1 = (Number.isFinite(t0) ? t0 : 0) + dt;
    }
    if (!Number.isFinite(t1) && Number.isFinite(t0) && Number.isFinite(e.strideTimeS) && e.strideTimeS > 0) {
      t1 = t0 + e.strideTimeS;
    }
    return { e, i, t0, t1, open: Boolean(e.isOpenStride) };
  });
  const drop = new Set();
  for (let a = 0; a < indexed.length; a += 1) {
    if (drop.has(a) || indexed[a].open) continue;
    if (!Number.isFinite(indexed[a].t0) || !Number.isFinite(indexed[a].t1)) continue;
    for (let b = a + 1; b < indexed.length; b += 1) {
      if (drop.has(b) || indexed[b].open) continue;
      if (!Number.isFinite(indexed[b].t0) || !Number.isFinite(indexed[b].t1)) continue;
      const A = indexed[a];
      const B = indexed[b];
      if (!(A.t0 < B.t1 && B.t0 < A.t1)) continue;
      const aLen = A.t1 - A.t0;
      const bLen = B.t1 - B.t0;
      // เก็บอันสั้นกว่า; ถ้ายาวเท่ากันเก็บอันที่เริ่มก่อน
      if (aLen > bLen + 1e-6) drop.add(a);
      else if (bLen > aLen + 1e-6) drop.add(b);
      else if (A.t0 <= B.t0) drop.add(b);
      else drop.add(a);
    }
  }
  return entries.filter((_, i) => !drop.has(i));
}

function isCleanStrideForDistance(cycle) {
  if (!cycle || cycle.isOpenStride) return false;
  if (!Number.isFinite(cycle.strideLengthM)) return false;
  if (cycle.strideLengthClamped || cycle.strideLengthUntrusted || cycle.suspectedMissedHs) return false;
  return true;
}

/**
 * Reprocess IMU trace samples → รายการ cycle ต่อข้าง พร้อม metrics เต็ม
 *
 * Default = mirror live device: sliding buffer 12s + analyze ทุก N sample
 * (full-session buffer ทำให้ adaptive HS threshold ไม่ translation-invariant กับเครื่องจริง)
 * ส่ง fullSessionBuffer:true เฉพาะเมื่อต้องการโหมด offline ทดลอง
 */
export function reprocessImuTrace(trace, options = {}) {
  const fullSessionBuffer = options.fullSessionBuffer === true;
  // default: mirror live (sliding 12s + analyze เป็นระยะ) — ตรงกับเครื่องจริง
  const useStreaming = fullSessionBuffer
    ? (options.forceStreaming === true || options.streamingSimulate === true)
    : options.streamingSimulate !== false;
  const analyzeEvery = Number.isFinite(options.analyzeEvery)
    ? options.analyzeEvery
    : GAIT_ANALYZE_EVERY_SAMPLES;
  const preferSensorFrame = options.preferSensorFrame ?? Boolean(trace.hasSensorFrameRaw);
  const axisMap = options.axisMap || trace.axisMap || undefined;
  const calibrationBySensor = options.calibrationBySensor || trace.calibrationBySensor || {};
  const orientationFilter = options.orientationFilter || 'kalman';

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
    const proc = new GaitProcessor({ orientationFilter });
    // mirror live: เก็บ maxBuffer = 12s; ขยายเฉพาะเมื่อขอ fullSessionBuffer
    if (fullSessionBuffer) {
      proc.maxBuffer = Math.max(sensorSamples.length + 64, proc.maxBuffer);
    }
    const profile = calibrationBySensor[sensorKey];
    if (profile) {
      proc.applyCalibration(profile);
    }

    const seen = new Map(); // cycleKey → entry object (mutable สำหรับ upgrade)
    let sinceAnalyze = 0;
    const sideHint = sensorSamples.find((s) => s.side === 'L' || s.side === 'R')?.side || null;
    const firstT = sensorSamples.map((s) => s.timestampMs).find(Number.isFinite);

    const ensureSide = (hintSide) => {
      const side = hintSide === 'L' || hintSide === 'R' ? hintSide : sideHint;
      return side === 'L' || side === 'R' ? side : null;
    };

    const removeEntry = (key) => {
      const existing = seen.get(String(key));
      if (!existing) return;
      seen.delete(String(key));
      const side = existing.side;
      if (side === 'L' || side === 'R') {
        const idx = bySide[side].indexOf(existing);
        if (idx >= 0) bySide[side].splice(idx, 1);
      }
    };

    const upsertEntry = (entry) => {
      if (!entry?.cycleKey) return;
      const side = ensureSide(entry.side);
      if (!side) return;
      entry.side = side;
      const existing = seen.get(String(entry.cycleKey));
      if (!existing) {
        seen.set(String(entry.cycleKey), entry);
        bySide[side].push(entry);
      } else if (shouldUpgradeCycle(existing, entry)) {
        Object.assign(existing, entry);
      }
      // closed แทน open คนละ key (open:id vs id) — ลบ open คู่กัน
      if (!entry.isOpenStride) {
        const startId = entry.cycleStartSampleId ?? parseCycleKeyStartId(entry.cycleKey);
        if (Number.isFinite(startId)) removeEntry(`open:${startId}`);
      }
    };

    const mergeDiagnostic = (d) => {
      if (d?.retracted) {
        removeEntry(d.cycleKey);
        return;
      }
      const key = String(d.cycleKey);
      let existing = seen.get(key);
      if (!existing) {
        const side = ensureSide(d.side);
        if (!side) return;
        existing = cycleEntryFromDiagnostic(d, side, sensorKey, firstT);
        seen.set(key, existing);
        bySide[side].push(existing);
      }
      if (d.strideLengthClamped != null) existing.strideLengthClamped = d.strideLengthClamped;
      if (d.strideLengthUntrusted != null) existing.strideLengthUntrusted = d.strideLengthUntrusted;
      if (d.suspectedMissedHs != null) existing.suspectedMissedHs = Boolean(d.suspectedMissedHs);
      if (d.isOpenStride != null) existing.isOpenStride = Boolean(d.isOpenStride);
      if (d.strideLengthM !== undefined) existing.strideLengthM = d.strideLengthM;
      if (d.strideTimeS !== undefined) existing.strideTimeS = d.strideTimeS;
      if (d.cadenceSpm !== undefined) existing.cadenceSpm = d.cadenceSpm;
      if (d.walkingSpeedMps !== undefined) existing.walkingSpeedMps = d.walkingSpeedMps;
      if (d.stancePct !== undefined) existing.stancePct = d.stancePct;
      if (d.swingPct !== undefined) existing.swingPct = d.swingPct;
      if (d.temporalSource != null) existing.temporalSource = d.temporalSource;
      if (d.peakShankAngleDeg !== undefined) existing.peakShankAngleDeg = d.peakShankAngleDeg;
      if (d.clearanceM !== undefined) existing.clearanceM = d.clearanceM;
      if (d.cycleEndTimestampMs != null) existing.cycleEndTimestampMs = d.cycleEndTimestampMs;
      if (d.cycleStartSampleId != null) existing.cycleStartSampleId = d.cycleStartSampleId;
      if (d.cycleEndSampleId != null) existing.cycleEndSampleId = d.cycleEndSampleId;
      if (d.zuptCheck) {
        existing.zuptCheck = d.zuptCheck;
        existing.zuptAccelDeviationG = d.zuptCheck.zuptAccelDeviationG ?? existing.zuptAccelDeviationG;
      }
      if (!existing.isOpenStride) {
        const startId = existing.cycleStartSampleId ?? parseCycleKeyStartId(existing.cycleKey);
        if (Number.isFinite(startId)) removeEntry(`open:${startId}`);
      }
    };

    proc.onParams(({ params, newCycleDiagnostics }) => {
      // สร้าง/อัปเกรดจาก params ก่อน แล้วค่อย merge diagnostic (กัน diagnostic ตกหล่น)
      if (params?.cycleKey) {
        const side = ensureSide(params.side);
        if (side) {
          const entry = cycleEntryFromParams(params, side, sensorKey);
          if (
            Number.isFinite(entry.cycleStartTimestampMs)
            && Number.isFinite(firstT)
            && firstT >= 946684800000
          ) {
            entry.cycleStartTimeS = (entry.cycleStartTimestampMs - firstT) / 1000;
          }
          upsertEntry(entry);
        }
      }

      if (newCycleDiagnostics?.length) {
        for (const d of newCycleDiagnostics) {
          mergeDiagnostic(d);
        }
      }
    });

    for (const sample of sensorSamples) {
      proc.addSample(sample);
      sinceAnalyze += 1;
      if (useStreaming && sinceAnalyze >= analyzeEvery) {
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
    bySide[side] = resolveOverlappingImuCycles(bySide[side]);
    bySide[side].sort((a, b) => (a.cycleStartTimeS ?? 0) - (b.cycleStartTimeS ?? 0));
  }

  return {
    ok: true,
    reason: null,
    bySide,
    source: 'reprocess',
    qualityFlags,
    reprocessMode: fullSessionBuffer ? 'full-session-buffer' : 'mirror-live',
  };
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
  const list = cycles || [];
  const closed = list.filter((c) => !c.isOpenStride);
  const coverage = computeCycleTimeCoverage(closed);
  return {
    cycleCount: closed.length,
    openStrideCount: list.length - closed.length,
    meanStrideLengthM: mean(closed.map((c) => c.strideLengthM)),
    meanCadenceSpm: mean(closed.map((c) => c.cadenceSpm)),
    meanWalkingSpeedMps: mean(closed.map((c) => c.walkingSpeedMps)),
    meanStancePct: mean(closed.map((c) => c.stancePct)),
    meanPeakShankAngleDeg: mean(closed.map((c) => c.peakShankAngleDeg)),
    meanClearanceM: mean(closed.map((c) => c.clearanceM)),
    clampedCount: closed.filter((c) => c.strideLengthClamped).length,
    coverageGapS: coverage.coverageGapS,
    coverageRatio: coverage.coverageRatio,
    hasCoverageGap: coverage.hasCoverageGap,
  };
}

function mocapCycleTimeS(cycle) {
  return Number.isFinite(cycle.hsStartTimeS) ? cycle.hsStartTimeS : null;
}

function imuCycleTimeS(cycle) {
  return Number.isFinite(cycle.cycleStartTimeS) ? cycle.cycleStartTimeS : null;
}

/** นับคู่ HS แบบ greedy nearest ใน tolerance — ต้องตรงกับ pairCyclesByTime */
function countHsMatches(mocapTimes, imuTimes, lag, matchToleranceS) {
  let count = 0;
  let residualAbsSum = 0;
  const usedImu = new Set();
  for (const mt of mocapTimes) {
    let best = null;
    for (let i = 0; i < imuTimes.length; i += 1) {
      if (usedImu.has(i)) continue;
      const err = (imuTimes[i] - lag) - mt;
      const abs = Math.abs(err);
      if (abs <= matchToleranceS && (!best || abs < best.abs)) {
        best = { i, abs };
      }
    }
    if (!best) continue;
    usedImu.add(best.i);
    count += 1;
    residualAbsSum += best.abs;
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
  // step เฉพาะ loop i (~12 จุด IMU); j เดินทุก MoCap HS — ถ้า step ทั้งคู่
  // แล้ว c ไม่หารด้วย step ลงตัว → lag จริงไม่เคยเป็น candidate (เช่น 8.0 กับ step=2)
  const candidates = new Set();
  const step = Math.max(1, Math.floor(imuTimes.length / 12));
  for (let i = 0; i < imuTimes.length; i += step) {
    for (let j = 0; j < mocapTimes.length; j += 1) {
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
  // ไม่มี trusted coarse → ห้าม ok (กัน alias ใกล้ 0)
  // มี trusted coarse → อนุญาตจับคู่ exploratory แต่ usePairedAgreement ตัด periodAliasRisk
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
  const lags = [...corrByLag.keys()].sort((a, b) => a - b);
  for (let i = 0; i < lags.length; i += 1) {
    const lagSamples = lags[i];
    const corr = corrByLag.get(lagSamples);
    if (!(corr >= minCorr)) continue;
    const left = i > 0 ? corrByLag.get(lags[i - 1]) : null;
    const right = i + 1 < lags.length ? corrByLag.get(lags[i + 1]) : null;
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
 *   2) สแกนกว้าง (±rivalScanMaxLagS) บนหน้าต่างทับซ้อนที่เลื่อนตาม lag
 *      (ไม่จำกัด ±overlap/3 — ไม่งั้น --rival-scan 90 ไม่มีผลเมื่อ MoCap สั้น)
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
  const useEnvelope = options.useEnvelope === true;

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
    useEnvelope,
    reason: 'missing-series',
    systematicUncertaintyS: SIGNAL_LAG_SYSTEMATIC_UNCERTAINTY_S,
  };

  if (!mocapT?.length || !imuT?.length || mocapT.length !== mocapY.length || imuT.length !== imuY.length) {
    return empty;
  }

  // |ω| envelope — ขั้ว/นิยามมุมไม่ตรงกันแต่จังหวะก้าวตรง (เช่น heel แทน malleolus)
  const mocapYUse = useEnvelope
    ? mocapY.map((v) => (Number.isFinite(v) ? Math.abs(v) : v))
    : mocapY;
  const imuYUse = useEnvelope
    ? imuY.map((v) => (Number.isFinite(v) ? Math.abs(v) : v))
    : imuY;

  const mocapT0 = minFinite(mocapT);
  const imuT0 = minFinite(imuT);
  const mocapT1 = maxFinite(mocapT);
  const imuT1 = maxFinite(imuT);
  if (![mocapT0, imuT0, mocapT1, imuT1].every(Number.isFinite)) {
    return empty;
  }

  /** NCC ที่ lag คงที่ — resample IMU บน [t0+lag, t1+lag] ให้เทียบกับ MoCap [t0,t1] ได้แม้ lag >> overlap เดิม */
  function nccAtLagS(lagS) {
    const o0 = Math.max(mocapT0, imuT0 - lagS);
    const o1 = Math.min(mocapT1, imuT1 - lagS);
    if (!(o1 - o0 >= 1.0)) return null;
    const ref = resampleUniform(mocapT, mocapYUse, dt, o0, o1);
    const sig = resampleUniform(imuT, imuYUse, dt, o0 + lagS, o1 + lagS);
    let sumR = 0;
    let sumS = 0;
    let sumRR = 0;
    let sumSS = 0;
    let sumRS = 0;
    let n = 0;
    const nSamples = Math.min(ref.length, sig.length);
    for (let i = 0; i < nSamples; i += 1) {
      const r = ref[i];
      const s = sig[i];
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

  const maxScanSamples = Math.max(1, Math.floor(rivalScanMaxLagS / dt));

  // Forced lag (จาก --lag / heel-tap sync) — ตรวจ corr บนหน้าต่างเลื่อน ไม่ใช่ overlap ดิบ
  if (Number.isFinite(options.lagS)) {
    const corr = nccAtLagS(options.lagS);
    const ok = Number.isFinite(corr) && corr >= minPeakCorr;
    let reason = null;
    if (!Number.isFinite(corr)) reason = 'overlap-too-short-at-lag';
    else if (!ok) reason = 'weak-correlation';
    return {
      lagS: options.lagS,
      peakCorr: corr,
      peakCorrMinus: corr == null ? null : -corr,
      polarity: 1,
      ok,
      ambiguous: false,
      polarityIndeterminate: false,
      periodAliasRisk: false,
      rivalPeaks: [],
      coarseLagS: options.lagS,
      coarseFromOnset: false,
      useEnvelope,
      reason,
      systematicUncertaintyS: SIGNAL_LAG_SYSTEMATIC_UNCERTAINTY_S,
    };
  }

  // ขั้นตัวอย่างสแกน: ละเอียดใกล้ 0 / ใกล้ coarse; กว้างขึ้นนอกนั้น (กัน O(rivalScan/dt) ช้าเกิน)
  const fineStep = 1;
  const coarseStep = Math.max(1, Math.round(0.05 / dt)); // ~50 ms
  const corrByLag = new Map();
  let minCorr = Infinity;
  function recordLag(lagSamples) {
    if (corrByLag.has(lagSamples)) return;
    const corr = nccAtLagS(lagSamples * dt);
    if (corr == null) return;
    corrByLag.set(lagSamples, corr);
    if (corr < minCorr) minCorr = corr;
  }
  // จะรู้ coarse หลัง onset — สแกนกว้างก่อน แล้ว densify รอบ coarse/peaks
  for (let lagSamples = -maxScanSamples; lagSamples <= maxScanSamples; lagSamples += coarseStep) {
    recordLag(lagSamples);
  }
  // densify ใน fine window รอบ 0 (กรณีไม่มี onset)
  for (let lagSamples = -Math.floor(fineMaxLagS / dt); lagSamples <= Math.floor(fineMaxLagS / dt); lagSamples += fineStep) {
    recordLag(lagSamples);
  }

  if (!corrByLag.size) {
    return { ...empty, reason: 'weak-correlation' };
  }

  // coarse lag จาก onset (ไม่เป็นคาบ) — หรือ override ชัดเจน (ไม่นับเป็น onset)
  let coarseLagS = Number.isFinite(options.coarseLagS) ? options.coarseLagS : null;
  const hasExplicitCoarse = Number.isFinite(options.coarseLagS);
  let coarseFromOnset = false;
  if (!Number.isFinite(coarseLagS)) {
    const mocapOnset = estimateSignalOnsetS(mocapT, mocapYUse, options);
    const imuOnset = estimateSignalOnsetS(imuT, imuYUse, options);
    if (Number.isFinite(mocapOnset) && Number.isFinite(imuOnset)) {
      coarseLagS = imuOnset - mocapOnset;
      coarseFromOnset = true;
    }
  }
  if (!Number.isFinite(coarseLagS)) coarseLagS = 0;
  // |ω| envelope: onset จาก heel มักหลอก (เริ่มกลาง gait / stance ripple) — อย่านับเป็น trusted coarse
  const trustedCoarse = hasExplicitCoarse || (coarseFromOnset && !useEnvelope);

  const fineSamples = Math.floor(fineMaxLagS / dt);
  const coarseSamples = Math.round(coarseLagS / dt);

  // densify รอบ coarse + รอบ peak หยาบที่เจอแล้ว
  for (let lagSamples = coarseSamples - fineSamples; lagSamples <= coarseSamples + fineSamples; lagSamples += fineStep) {
    if (Math.abs(lagSamples) <= maxScanSamples) recordLag(lagSamples);
  }
  for (const [lagSamples, corr] of [...corrByLag.entries()]) {
    if (corr < minPeakCorr * 0.5) continue;
    for (let d = -coarseStep; d <= coarseStep; d += fineStep) {
      const ls = lagSamples + d;
      if (Math.abs(ls) <= maxScanSamples) recordLag(ls);
    }
  }

  const peakCorrMinus = Number.isFinite(minCorr) ? -minCorr : null;
  const peaks = listLocalMaxima(corrByLag, minPeakCorr * 0.5);
  const strongPeaks = peaks.filter((p) => p.corr >= minPeakCorr);

  // เลือก peak: มี trusted coarse → ใกล้ coarse ในกลุ่ม corr สูงสุด; ไม่มี → corr สูงสุด (อย่าชอบ 0)
  function pickPeak(candidates) {
    if (!candidates.length) return null;
    const bestCorr = Math.max(...candidates.map((p) => p.corr));
    const top = candidates.filter((p) => p.corr >= bestCorr * AMBIGUOUS_PEAK_RATIO);
    if (trustedCoarse) {
      top.sort((a, b) => Math.abs(a.lagSamples - coarseSamples) - Math.abs(b.lagSamples - coarseSamples));
    } else {
      top.sort((a, b) => b.corr - a.corr || Math.abs(a.lagSamples) - Math.abs(b.lagSamples));
    }
    return top[0];
  }

  const inFine = trustedCoarse
    ? strongPeaks.filter((p) => Math.abs(p.lagSamples - coarseSamples) <= fineSamples)
    : [];
  // ไม่มี trusted coarse → ห้ามชอบ peak ใกล้ 0 (เคยเลือก alias ใกล้ 0 ทั้งที่ |ω| peak จริงอยู่ที่ ~50s)
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

  const polarityIndeterminate = !useEnvelope
    && Number.isFinite(peakCorrMinus)
    && bestCorr >= minPeakCorr
    && Math.abs(peakCorrMinus - bestCorr) <= polarityMargin;

  if (!useEnvelope && Number.isFinite(peakCorrMinus) && peakCorrMinus > bestCorr + polarityMargin) {
    return {
      ...empty,
      peakCorr: bestCorr,
      peakCorrMinus,
      coarseLagS,
      useEnvelope,
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
      useEnvelope,
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
  // |ω| envelope โดยไม่มี onset: ยอม lag ไกลจาก 0 (trial เริ่มกลาง gait) — ยัง exploratory ไม่ใช่ lab gate
  const envelopeOpenScan = useEnvelope && !trustedCoarse;
  // ถ้าไม่มี onset / explicit coarse / --lag แล้วยังมี rival ±n·stride → ห้ามเลือก alias ใกล้ 0 แบบเงียบ
  // (envelope open-scan ยอม periodAliasRisk flag ไว้ แต่ไม่ reject ทั้งก้อน — syncTrusted ตัดอยู่แล้ว)
  const refuseForAlias = periodAliasRisk && !trustedCoarse && !envelopeOpenScan;
  const ok = (envelopeOpenScan || !farFromCoarse) && !refuseForAlias;
  // reason = สาเหตุที่ปฏิเสธเท่านั้น; ข้อสังเกตอยู่ที่ periodAliasRisk / ambiguous
  let reason = null;
  if (!ok) {
    if (!envelopeOpenScan && farFromCoarse) reason = 'lag-far-from-coarse-onset';
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
    useEnvelope,
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
  const axis = options.axis === 'gy' || options.axis === 'gz' ? options.axis : 'gx';
  const rawT = [];
  const values = [];

  for (const sample of samples) {
    const input = sampleToProcessorInput(sample, { axisMap, preferSensorFrame });
    if (!input || input.side !== side) continue;
    if (!Number.isFinite(input.timestampMs) || !Number.isFinite(input[axis])) continue;
    rawT.push(input.timestampMs);
    values.push(input[axis]);
  }

  if (rawT.length < 20) return { tS: [], gx: [], values: [], axis, ok: false };

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

  return { tS, gx: values, values, axis, ok: true };
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

function alignmentMode(lagSource, usePairedAgreement, rawPaired) {
  if (!usePairedAgreement) {
    return rawPaired ? 'paired-untrusted' : 'session-mean-informational';
  }
  if (lagSource === 'signal-xcorr' || lagSource === 'signal-xcorr-envelope') {
    return lagSource === 'signal-xcorr-envelope' ? 'signal-envelope+hs-pair' : 'signal-xcorr+hs-pair';
  }
  if (lagSource === 'external' || lagSource === 'heel-tap' || lagSource === 'manual') {
    return 'external-lag';
  }
  return 'hs-event-lag';
}

/**
 * @param {object} mocap - output จาก computeGaitFromMocap / run.js
 * @param {object} imuTrace - export จาก dashboard TraceRecorder
 */
export function compareMocapToImu(mocap, imuTrace, options = {}) {
  const warnings = [];

  if (mocap?.bilateral?.sameSideRepeats > 0) {
    warnings.push(
      `MoCap sameSideRepeats=${mocap.bilateral.sameSideRepeats}`
      + (mocap.bilateral.reliable === false
        ? ' — bilateral.reliable=false; ห้ามใช้ meanStepTime / trueCadence'
        : ' — event detection อาจพลาดฝั่งใดฝั่งหนึ่ง'),
    );
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
  // --lag / heel-tap = trusted; HS-circular จาก syncSession ต้องไม่นับ
  const lagSourceHint = options.align?.lagSource;
  const circularHsLag = lagSourceHint === 'hs-event-circular'
    || lagSourceHint === 'hs-event-consensus';
  const explicitLagRequested = Number.isFinite(options.align?.lagS);
  const explicitLagTrustedIntent = explicitLagRequested
    && options.align?.lagTrusted !== false
    && !circularHsLag;
  const explorationOnly = Boolean(
    options.explorationOnly
    || circularHsLag
    || options.align?.lagTrusted === false,
  );

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
      const angleSource = mocapSignals.angleSource || 'legacy-ankle';
      const envelopeOnly = angleSource === 'heel-for-envelope-only';
      const signedOpts = { ...alignOpts };
      delete signedOpts.lagS; // free scan — อย่าบังคับ lag จาก HS-circular ตอน xcorr
      // heel-only: signed polarity มักพัง — ใช้ |ω| ตั้งแต่ต้น
      signalLag = estimateSignalLagS(
        mocapSignals.tS,
        mocapSignals.shankAngularVelocityDps,
        imuGyro.tS,
        imuGyro.gx,
        envelopeOnly ? { ...signedOpts, useEnvelope: true } : signedOpts,
      );
      if (
        !envelopeOnly
        && !signalLag.ok
        && (signalLag.reason === 'polarity-mismatch' || signalLag.reason === 'ambiguous-polarity')
      ) {
        const env = estimateSignalLagS(
          mocapSignals.tS,
          mocapSignals.shankAngularVelocityDps,
          imuGyro.tS,
          imuGyro.gx,
          { ...signedOpts, useEnvelope: true },
        );
        if (env.ok) {
          warnings.push(
            `ขา ${side}: signed ω ${signalLag.reason} — fallback |ω| envelope `
            + `lag=${env.lagS.toFixed(3)}s corr=${env.peakCorr?.toFixed?.(2)}; `
            + 'ตรวจ malleolus map / ขั้วแกน (envelope ไม่แทน heel-tap lab sync)',
          );
          signalLag = env;
        }
      }
      if (signalLag.ok) {
        // อย่าทับ --lag / hs-circular ที่ส่งมาแล้ว — ใช้ signal แค่ verify corr
        if (!explicitLagRequested) {
          alignOpts.lagS = signalLag.lagS;
          alignOpts.lagSource = signalLag.useEnvelope ? 'signal-xcorr-envelope' : 'signal-xcorr';
          alignOpts.coarseLagS = signalLag.coarseLagS;
        }
        if (signalLag.periodAliasRisk) {
          warnings.push(
            `ขา ${side}: signal xcorr มี rival peaks ใกล้เคียงที่ ±n·stride `
            + `(${(signalLag.rivalPeaks || []).slice(0, 3).map((p) => `${p.lagS.toFixed(2)}s`).join(', ')}) `
            + `— เลือก lag=${signalLag.lagS.toFixed(3)}s จาก coarse=${Number.isFinite(signalLag.coarseLagS) ? signalLag.coarseLagS.toFixed(3) : '—'}s; `
            + 'HS timing อย่าเพิ่งเชื่อจนกว่าจะมี heel-tap/--lag; แนะนำ onset ยืน→เดินหรือ sync marker',
          );
        }
        if (envelopeOnly) {
          warnings.push(
            `ขา ${side}: angleSource=heel-for-envelope-only — sync จาก |ω| ได้แต่ peak shank angle = null; `
            + 'trial ถัดไปติด lateral malleolus เป็น AnkleForAngle',
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

    // --lag ที่ส่งมา: ต้องผ่าน signal corr บนหน้าต่างเลื่อน ถ้ามีสัญญาณให้ตรวจ
    // (corr คำนวณไม่ได้ / อ่อน → syncTrusted=false — กัน heel-tap ใหญ่กว่า overlap เดิม)
    const forcedLagCorrOk = !explicitLagTrustedIntent
      || signalLag == null
      || signalLag.ok === true;

    // |ω| envelope = exploratory clock sync เท่านั้น (heel≠malleolus) — ไม่นับ lab-trusted
    const syncTrusted = Boolean(
      (explicitLagTrustedIntent && forcedLagCorrOk)
      || (alignment.lagSource === 'signal-xcorr' && signalLag?.ok && !alignment.periodAliasRisk)
      || (
        signalLag?.coarseFromOnset
        && alignment.lagOk !== false
        && Number.isFinite(alignment.lagS)
        && !alignment.periodAliasRisk
        && alignment.lagSource !== 'signal-xcorr-envelope'
      )
    );
    // session-mean หรือ pairing ปฏิเสธ → ห้าม comparable metrics (เคยทำให้ error% ดูสวยทั้งที่ผิด)
    // periodAliasRisk = คลาด ±1 stride ได้ — ห้าม publish แม้จับคู่ exploratory ได้จาก coarse
    const usePairedAgreement = rawPaired
      && alignment.lagOk !== false
      && syncTrusted
      && !alignment.periodAliasRisk
      && agreementPairs.length >= minAgreementPairs
      && !hasSyntheticTimestamps
      && !explorationOnly;

    if (alignment.lagOk === false && alignment.periodAliasRisk) {
      warnings.push(
        `ขา ${side}: HS-event lag ปฏิเสธเพราะ period-alias rivals `
        + `(เลือกได้ ${Number.isFinite(alignment.lagS) ? alignment.lagS.toFixed(3) : '—'}s แต่มี `
        + `${(alignment.rivalLags || []).slice(0, 3).map((r) => `${r.lagS.toFixed(2)}s`).join(', ') || 'rivals'}) `
        + '— ไม่จับคู่ cycle; ใส่ --lag / heel-tap ก่อนเทียบ — ห้ามใช้ session-mean เป็น validation',
      );
    } else if (alignment.periodAliasRisk && alignment.lagOk !== false) {
      warnings.push(
        `ขา ${side}: HS/signal lag มี period-alias rivals `
        + `(เลือก ${Number.isFinite(alignment.lagS) ? alignment.lagS.toFixed(3) : '—'}s; rivals `
        + `${(alignment.rivalLags || []).slice(0, 3).map((r) => `${r.lagS.toFixed(2)}s`).join(', ')
          || (signalLag?.rivalPeaks || []).slice(0, 3).map((p) => `${p.lagS.toFixed(2)}s`).join(', ')
          || '—'}) `
        + '— จับคู่ได้แบบ exploratory แต่ comparable/validationPublishable=false',
      );
    } else if (explicitLagTrustedIntent && signalLag != null && !forcedLagCorrOk) {
      warnings.push(
        `ขา ${side}: --lag=${options.align.lagS.toFixed(3)}s แต่ signal corr ที่ lag นี้ใช้ไม่ได้ `
        + `(${signalLag.reason || 'weak'}; peakCorr=${Number.isFinite(signalLag.peakCorr) ? signalLag.peakCorr.toFixed(2) : 'null'}) `
        + '— syncTrusted=false; ตรวจ heel-tap / ช่วงทับซ้อนหลังเลื่อนเวลา',
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
    if ((alignment.lagAmbiguous || alignment.periodAliasRisk) && alignment.lagOk !== false) {
      if (alignment.lagAmbiguous && !alignment.periodAliasRisk) {
        warnings.push(
          `ขา ${side}: มี lag หลายค่าที่ match เท่ากัน (${alignment.tiedLags.slice(0, 5).join(', ')}…) `
          + '— เลือกจาก residual ต่ำสุดแล้ว แต่ควรตรวจ sync',
        );
      }
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
        mode: alignmentMode(alignment.lagSource, usePairedAgreement, rawPaired),
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
        // คู่ดิบสำหรับ UI / exploratory — แม้ syncTrusted=false ก็ยังดูตารางได้
        pairs: alignment.pairs,
        agreementPairs,
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
  const publishableSides = presentSides.filter((s) => sides[s].validationPublishable);
  // session-level: อย่างน้อยหนึ่งข้าง publishable — อย่าให้ขาเดียวพังฆ่าทั้ง session
  const validationPublishable = publishableSides.length > 0 && !hasSyntheticTimestamps && !explorationOnly;
  const validationPublishableBilateral = presentSides.length > 0
    && presentSides.every((s) => sides[s].validationPublishable)
    && !hasSyntheticTimestamps
    && !explorationOnly;

  const labChecklist = [
    {
      id: 'heel-tap-or-lag',
      ok: (explicitLagTrustedIntent && presentSides.some((s) => sides[s].alignment?.syncTrusted))
        || presentSides.some((s) => (
          sides[s].alignment?.coarseFromOnset
          || sides[s].alignment?.lagSource === 'signal-xcorr'
          || (sides[s].alignment?.lagSource === 'external' && sides[s].alignment?.syncTrusted)
        )),
      // signal-xcorr-envelope ไม่นับ — |ω| จาก heel เป็น exploratory เท่านั้น
      detail: 'heel-tap 1 ครั้งก่อนเดิน + ใส่ --lag หรือมี onset/signed signal sync ที่ผ่าน (ไม่นับ |ω| envelope)',
    },
    {
      id: 'paired-agreement-cycles',
      ok: publishableSides.length > 0,
      detail: `อย่างน้อยหนึ่งข้างมี ≥${minAgreementPairs} คู่คุณภาพ (ดู sides[L|R].validationPublishable)`,
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
      // ห้ามใช้ strideLengthSigned เป็น auto-gate — บนข้อมูลจริง/demo ค่า signed
      // ใกล้ 0 ได้จาก integration residual → ฟ้องเท็จ 100%; ใช้ swing test ใน LAB_VALIDATION.md
      id: 'axis-map-verified',
      ok: null,
      detail: 'ตรวจแกน L/R ด้วย swing test (ดูขั้ว gx ตอนแกว่งไปหน้า) — ไม่ auto-verify จาก signed stride',
    },
  ];

  if (explorationOnly) {
    warnings.unshift(
      '🔭 explorationOnly=true — lag จาก HS-event circular / untrusted; '
      + 'validationPublishable ถูกบังคับเป็น false (ไม่ใช่ lab gate)',
    );
  }

  if (!validationPublishable) {
    warnings.unshift(
      '⛔ validationPublishable=false — ไม่มีข้างใดผ่าน agreement ที่ trusted '
      + '(ดู sides[L|R].validationPublishable และ labChecklist); ใช้ --lab เพื่อบังคับ fail',
    );
  } else if (!validationPublishableBilateral && presentSides.length > 1) {
    const bad = presentSides.filter((s) => !sides[s].validationPublishable);
    warnings.unshift(
      `⚠️ validationPublishable=true บางข้างเท่านั้น (ผ่าน: ${publishableSides.join(',')}; `
      + `ไม่ผ่าน: ${bad.join(',')}) — อย่าเฉลี่ยข้ามข้างที่ไม่ publishable; `
      + `validationPublishableBilateral=false`,
    );
  }

  const pelvisNetMeaningful = mocap?.session?.pelvisNetMeaningful === true
    || mocap?.session?.forwardAxisMethod === 'net-start-end'
    || mocap?.forwardAxis?.method === 'net-start-end';
  const mocapDistanceM = pelvisNetMeaningful
    && Number.isFinite(mocap?.session?.pelvisNetForwardDisplacementM)
    ? mocap.session.pelvisNetForwardDisplacementM
    : null;
  const mocapMaxExcursionM = Number.isFinite(mocap?.session?.pelvisMaxExcursionM)
    ? mocap.session.pelvisMaxExcursionM
    : (Number.isFinite(mocap?.forwardAxis?.maxExcursionM) ? mocap.forwardAxis.maxExcursionM : null);
  const gtDistanceM = Number.isFinite(imuTrace?.groundTruth?.distanceM)
    ? imuTrace.groundTruth.distanceM
    : null;

  if (!pelvisNetMeaningful && Number.isFinite(mocap?.session?.pelvisNetForwardDisplacementM)) {
    warnings.push(
      `vsMocapPelvisNetPct ถูกปิด — forwardAxis.method=`
      + `${mocap?.session?.forwardAxisMethod || mocap?.forwardAxis?.method || '—'} `
      + `(net ไม่มีความหมายบนเดินไป-กลับ; maxExcursion=`
      + `${Number.isFinite(mocapMaxExcursionM) ? mocapMaxExcursionM.toFixed(2) : '—'}m)`,
    );
  }

  const distanceBySide = {};
  for (const side of ['L', 'R']) {
    const cycles = imu.bySide[side] || [];
    const closed = cycles.filter((c) => !c.isOpenStride);
    const withLength = closed.filter((c) => Number.isFinite(c.strideLengthM));
    const clean = withLength.filter((c) => isCleanStrideForDistance(c));
    const sumAll = withLength.reduce((a, c) => a + c.strideLengthM, 0);
    const sumClean = clean.reduce((a, c) => a + c.strideLengthM, 0);
    const excludedCount = withLength.length - clean.length;
    const n = closed.length;
    const coverage = computeCycleTimeCoverage(closed);
    // มีช่องว่างเวลาหลัง retract → Σclean ดูสะอาดแต่ขาดก้าว — ห้ามใช้เป็น distance gate
    const distanceGateOk = n > 0 && !coverage.hasCoverageGap;
    if (coverage.hasCoverageGap) {
      warnings.push(
        `ขา ${side}: coverage gap ${coverage.coverageGapS.toFixed(2)}s `
        + `(ratio=${Number.isFinite(coverage.coverageRatio) ? coverage.coverageRatio.toFixed(2) : '—'}) `
        + '— อาจขาด stride หลังแยก merged HS; distance gate ปฏิเสธ',
      );
    }
    distanceBySide[side] = {
      imuSumStrideLengthM: n ? sumAll : null,
      imuSumStrideLengthCleanM: n ? sumClean : null,
      excludedFromCleanCount: n ? excludedCount : null,
      closedCycleCount: n || null,
      cleanCycleCount: n ? clean.length : null,
      coverageGapS: coverage.coverageGapS,
      coverageRatio: coverage.coverageRatio,
      hasCoverageGap: coverage.hasCoverageGap,
      distanceGateOk,
      mocapMeanStrideLengthM: summarizeMocapSide(mocap?.perSide?.[side]).meanStrideLengthM,
      // gate / publish ใช้ clean เท่านั้น — และต้องไม่มี coverage gap
      vsMocapPelvisNetPct: (distanceGateOk && clean.length && mocapDistanceM != null)
        ? pctError(sumClean, mocapDistanceM)
        : null,
      vsMocapPelvisMaxExcursionPct: (distanceGateOk && clean.length && mocapMaxExcursionM != null && mocapMaxExcursionM > 0.3)
        ? pctError(sumClean, mocapMaxExcursionM)
        : null,
      vsGroundTruthPct: (distanceGateOk && clean.length && gtDistanceM != null)
        ? pctError(sumClean, gtDistanceM)
        : null,
    };
  }

  return {
    ok: true,
    imuSource: imu.source,
    warnings,
    explorationOnly,
    validationPublishable,
    validationPublishableBilateral,
    labChecklist,
    sides,
    session: {
      mocapPelvisNetForwardM: mocapDistanceM,
      mocapPelvisMaxExcursionM: mocapMaxExcursionM,
      mocapPelvisNetMeaningful: pelvisNetMeaningful,
      mocapDurationS: mocap?.session?.durationS ?? null,
      mocapTrueCadenceSpm: mocap?.bilateral?.reliable === false
        ? null
        : (mocap?.bilateral?.trueCadenceSpm ?? null),
      mocapBilateralReliable: mocap?.bilateral?.reliable !== false,
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
      'stepLength / stepTime / doubleSupport = null จากเซนเซอร์ข้างเดียว (ห้ามสมมติสมมาตร L/R)',
      'sum(stride) ต่อข้าง ≠ ระยะเดินจริงแบบ 1:1 ถ้าสองข้างบันทึกพร้อมกัน (อย่าบวก L+R)',
    ],
  };
}

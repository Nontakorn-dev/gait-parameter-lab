// คำนวณ gait parameter จาก marker ตำแหน่ง (ASIS x2, Knee/Lateral Femoral Epicondyle x2,
// Ankle/Lateral Malleolus x2)
//
// Event detection (Heel Strike / Toe Off) ใช้ "foot velocity algorithm" — มองพฤติกรรม
// การเคลื่อนที่ของข้อเท้าโดยตรง ไม่ได้ใช้ GaitEventDetector ของ IMU pipeline
// (หลีกเลี่ยง shared blind spot)
//
// Pipeline ความเร็ว + มุมหน้าแข้ง: Butterworth low-pass zero-lag (default 6 Hz)
// บนตำแหน่ง / shank angle → แล้วค่อย central-difference — จำเป็นเพราะ differentiate
// ขยาย marker noise ด้วย ~1/(2·dt). shankAngularVelocityDps (หลังกรอง) ใช้ signal xcorr sync
//
// ⚠️ ข้อจำกัดที่ยังมี:
//   - ไม่มี marker ปลายเท้า/heel → ใช้ข้อเท้าแทน (TO = "เท้าเริ่มขยับ" ไม่ใช่ toe-off เป๊ะ)
//   - clearance = ระยะยกข้อเท้า ไม่ใช่ปลายเท้า / จุดติด IMU
//   - สมมติเดินเป็นเส้นตรงตอนหา forward axis จาก ASIS midpoint

import { rad2deg } from '../src/gait/signalUtils.js';
import { minFinite, maxFinite } from '../src/util/finiteStats.js';
import { extractMarkerSeries, interpolateGaps } from './parseOptiTrack.js';
import { filtfiltButterworth2, estimateSampleRateHz } from './butterworth.js';
import { pickAnkleRolePlan } from './markerRoles.js';
import {
  computeForwardPosition,
  computeFilteredVelocity,
  detectStanceIntervals,
  buildCyclesFromStanceIntervals,
  DEFAULT_POSITION_CUTOFF_HZ,
} from './velocityEventDetector.js';

function dot2(ax, az, bx, bz) {
  return ax * bx + az * bz;
}

function meanFinite(values) {
  const finite = values.filter(Number.isFinite);
  if (!finite.length) return null;
  return finite.reduce((a, b) => a + b, 0) / finite.length;
}

function rangeFinite(values) {
  const lo = minFinite(values);
  const hi = maxFinite(values);
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return null;
  return hi - lo;
}

/**
 * ตรวจว่าพิกัดน่าจะเป็นเมตรหรือมิลลิเมตร + แกน Y เป็นแนวดิ่งตาม OptiTrack default หรือไม่
 * ถ้าเป็น mm จะ rescale series ทั้งชุดด้วย 0.001
 */
export function assertAndNormalizeCaptureUnits(seriesByRole, options = {}) {
  const warnings = [];
  const asisL = seriesByRole.L_ASIS;
  const asisR = seriesByRole.R_ASIS;
  if (!asisL || !asisR) {
    throw new Error('ต้องมี L_ASIS และ R_ASIS สำหรับตรวจหน่วย/แกน');
  }

  const midX = asisL.x.map((v, i) => (v === null || asisR.x[i] === null ? null : (v + asisR.x[i]) / 2));
  const midY = asisL.y.map((v, i) => (v === null || asisR.y[i] === null ? null : (v + asisR.y[i]) / 2));
  const midZ = asisL.z.map((v, i) => (v === null || asisR.z[i] === null ? null : (v + asisR.z[i]) / 2));

  const meanY = meanFinite(midY);
  const rangeX = rangeFinite(midX);
  const rangeY = rangeFinite(midY);
  const rangeZ = rangeFinite(midZ);

  // หน่วย: ความสูง ASIS ผู้ใหญ่ ~0.8–1.2 m — ถ้า meanY อยู่ใน 100–3000 น่าจะเป็น mm
  let unitScale = 1;
  if (Number.isFinite(meanY) && meanY >= 100 && meanY <= 3000) {
    if (options.autoScaleMillimetres === false) {
      throw new Error(
        `ASIS mean Y ≈ ${meanY.toFixed(1)} — น่าจะเป็นหน่วย mm (Motive export เป็น mm ได้) `
        + 'แต่ pipeline คาดหวังเมตร (threshold 0.15 m/s จะพังเงียบ ๆ ถ้าไม่แปลง). '
        + 'ส่ง autoScaleMillimetres:true หรือ export เป็นเมตร',
      );
    }
    unitScale = 0.001;
    warnings.push(
      `ตรวจพบหน่วยน่าจะเป็น mm (ASIS mean Y≈${meanY.toFixed(0)}) — แปลงเป็นเมตรอัตโนมัติ (×0.001)`,
    );
  } else if (Number.isFinite(meanY) && (meanY < 0.3 || meanY > 2.5) && meanY < 100) {
    warnings.push(
      `ASIS mean Y=${meanY.toFixed(3)} อยู่นอกช่วงความสูงเชิงกรานปกติ (~0.8–1.2m) — ตรวจหน่วย/marker map`,
    );
  }

  // แกนแนวดิ่ง: แกนที่ ASIS midpoint มี range เล็กสุดควรเป็นแนวดิ่ง (เดินตรง)
  // OptiTrack default = Y-up
  if (Number.isFinite(rangeX) && Number.isFinite(rangeY) && Number.isFinite(rangeZ)) {
    const ranges = [
      { axis: 'X', range: rangeX },
      { axis: 'Y', range: rangeY },
      { axis: 'Z', range: rangeZ },
    ].sort((a, b) => a.range - b.range);
    const smallest = ranges[0].axis;
    if (smallest !== 'Y') {
      const msg = `แกนที่ ASIS เคลื่อนน้อยสุดคือ ${smallest} (range X/Y/Z=`
        + `${rangeX.toFixed(3)}/${rangeY.toFixed(3)}/${rangeZ.toFixed(3)}) `
        + '— คาดหวัง Y เป็นแนวดิ่ง (OptiTrack default). ถ้า capture เป็น Z-up '
        + 'ankleClearance/มุมหน้าแข้งจะผิดหมด';
      // แพทย์/แลป: strict เป็น default — opt-out ด้วย strictVerticalAxis:false
      if (options.strictVerticalAxis !== false) {
        throw new Error(msg);
      }
      warnings.push(msg);
    }
  }

  const scaleSeries = (series) => {
    if (unitScale === 1) return series;
    return {
      t: series.t,
      x: series.x.map((v) => (v === null ? null : v * unitScale)),
      y: series.y.map((v) => (v === null ? null : v * unitScale)),
      z: series.z.map((v) => (v === null ? null : v * unitScale)),
    };
  };

  const normalized = {};
  for (const [role, series] of Object.entries(seriesByRole)) {
    normalized[role] = scaleSeries(series);
  }

  return { seriesByRole: normalized, unitScale, warnings, meanAsisY: meanY, ranges: { rangeX, rangeY, rangeZ } };
}

// แกนเดินหลัก (forward) จาก midpoint ของ ASIS สองข้างในแนวราบ (X,Z; Y = ดิ่ง)
// 1) ถ้าไป-จุดสุดท้ายสุทธิ ≥ 0.3 m ใช้ทิศนั้น
// 2) ถ้าไป-กลับ (net เล็ก) ใช้ทิศไปจุดที่ห่างจากจุดเริ่มมากสุด
// 3) fallback: PCA บน (x,z) ของ pelvis
export function computeForwardAxis(pelvisX, pelvisZ) {
  const pts = [];
  for (let i = 0; i < pelvisX.length; i += 1) {
    if (pelvisX[i] !== null && pelvisZ[i] !== null) {
      pts.push({ i, x: pelvisX[i], z: pelvisZ[i] });
    }
  }
  if (pts.length < 2) {
    throw new Error('หาแกนทิศทางเดินไม่ได้: ASIS midpoint ไม่มีข้อมูลพอ (อาจถูกบดบังทั้งเฟรม)');
  }

  const first = pts[0];
  const last = pts[pts.length - 1];
  const netDx = last.x - first.x;
  const netDz = last.z - first.z;
  const netMag = Math.sqrt(netDx * netDx + netDz * netDz);

  let maxExcursionM = 0;
  let far = first;
  for (const p of pts) {
    const d = Math.sqrt((p.x - first.x) ** 2 + (p.z - first.z) ** 2);
    if (d > maxExcursionM) {
      maxExcursionM = d;
      far = p;
    }
  }

  const finish = (fx, fz, method, netDisplacementM) => {
    const mag = Math.sqrt(fx * fx + fz * fz);
    if (!(mag > 1e-9)) {
      throw new Error('หาแกนทิศทางเดินไม่ได้: ทิศที่ได้มีความยาวศูนย์');
    }
    return {
      fx: fx / mag,
      fz: fz / mag,
      netDisplacementM,
      maxExcursionM,
      method,
    };
  };

  if (netMag >= 0.3) {
    return finish(netDx, netDz, 'net-start-end', netMag);
  }

  if (maxExcursionM >= 0.3) {
    return finish(far.x - first.x, far.z - first.z, 'max-excursion', netMag);
  }

  // PCA 2D
  let mx = 0;
  let mz = 0;
  for (const p of pts) {
    mx += p.x;
    mz += p.z;
  }
  mx /= pts.length;
  mz /= pts.length;
  let cxx = 0;
  let czz = 0;
  let cxz = 0;
  for (const p of pts) {
    const dx = p.x - mx;
    const dz = p.z - mz;
    cxx += dx * dx;
    czz += dz * dz;
    cxz += dx * dz;
  }
  // eigenvector of larger eigenvalue
  const trace = cxx + czz;
  const det = cxx * czz - cxz * cxz;
  const disc = Math.sqrt(Math.max(0, (trace * trace) / 4 - det));
  const eig1 = trace / 2 + disc;
  let fx = eig1 - czz;
  let fz = cxz;
  if (Math.abs(fx) + Math.abs(fz) < 1e-12) {
    fx = 1;
    fz = 0;
  }
  // หันไปทางจุดไกลสุดจากจุดเริ่ม (กำหนดทิศ)
  if ((far.x - first.x) * fx + (far.z - first.z) * fz < 0) {
    fx = -fx;
    fz = -fz;
  }
  const pcaSpread = Math.sqrt(Math.max(0, eig1) / pts.length);
  if (pcaSpread < 0.1 && maxExcursionM < 0.3) {
    throw new Error(
      `pelvis เคลื่อนที่แนวราบน้อยเกินไป (net=${netMag.toFixed(2)}m, max=${maxExcursionM.toFixed(2)}m) `
      + '— สั้นเกินกว่าจะหาแกนทิศทางเดินได้แม่นยำ',
    );
  }
  return finish(fx, fz, 'pca', netMag);
}

export function computeShankAngleDeg(kneeSeries, ankleSeries, forwardAxis) {
  const n = kneeSeries.x.length;
  const angleDeg = new Array(n).fill(null);

  for (let i = 0; i < n; i += 1) {
    if (kneeSeries.x[i] === null || ankleSeries.x[i] === null) continue;
    const vx = kneeSeries.x[i] - ankleSeries.x[i];
    const vy = kneeSeries.y[i] - ankleSeries.y[i];
    const vz = kneeSeries.z[i] - ankleSeries.z[i];
    const forwardComponent = dot2(vx, vz, forwardAxis.fx, forwardAxis.fz);
    angleDeg[i] = rad2deg(Math.atan2(forwardComponent, vy));
  }

  return angleDeg;
}

export function computeAngularVelocityDps(t, angleDeg) {
  const n = angleDeg.length;
  const w = new Array(n).fill(null);
  for (let i = 1; i < n - 1; i += 1) {
    if (angleDeg[i - 1] === null || angleDeg[i + 1] === null) continue;
    const dt = t[i + 1] - t[i - 1];
    if (dt <= 0) continue;
    w[i] = (angleDeg[i + 1] - angleDeg[i - 1]) / dt;
  }
  if (n >= 2 && angleDeg[0] !== null && angleDeg[1] !== null) {
    const dt = t[1] - t[0];
    if (dt > 0) w[0] = (angleDeg[1] - angleDeg[0]) / dt;
  }
  if (n >= 2 && angleDeg[n - 1] !== null && angleDeg[n - 2] !== null) {
    const dt = t[n - 1] - t[n - 2];
    if (dt > 0) w[n - 1] = (angleDeg[n - 1] - angleDeg[n - 2]) / dt;
  }
  return w;
}

/**
 * กรอง shank angle ด้วย Butterworth 2nd-order zero-lag แล้วค่อย central-diff → ω
 * (ต้องสอดคล้องกับ computeFilteredVelocity — สัญญาณนี้ใช้ signal xcorr sync)
 */
export function computeFilteredAngularVelocityDps(t, angleDeg, options = {}) {
  const cutoffHz = options.cutoffHz ?? DEFAULT_POSITION_CUTOFF_HZ;
  const sampleRateHz = options.sampleRateHz ?? estimateSampleRateHz(t);
  if (!Number.isFinite(sampleRateHz) || sampleRateHz <= 0) {
    throw new Error('หา sample rate จาก timestamp ไม่ได้ — ต้องส่ง sampleRateHz เอง');
  }
  const effectiveCutoff = Math.min(cutoffHz, sampleRateHz * 0.45);
  const smoothedAngleDeg = options.skipFilter
    ? angleDeg.slice()
    : filtfiltButterworth2(angleDeg, effectiveCutoff, sampleRateHz);
  return {
    smoothedAngleDeg,
    shankAngularVelocityDps: computeAngularVelocityDps(t, smoothedAngleDeg),
    sampleRateHz,
    cutoffHz: effectiveCutoff,
  };
}

function assertNoGaps(remainingGaps, label) {
  if (remainingGaps.length) {
    const ranges = remainingGaps.map((g) => `${g.startTimeS.toFixed(2)}-${g.endTimeS.toFixed(2)}s`).join(', ');
    throw new Error(
      `${label}: marker หายไปนานเกินจะเติมอัตโนมัติที่ช่วง [${ranges}] — ตัดช่วงนั้นออกหรือ `
      + 'capture ใหม่ก่อน (ไม่เดาเติมข้อมูลช่วงยาวเพราะอาจคร่อมจังหวะ heel-strike แล้วปั้นสัญญาณปลอม)',
    );
  }
}

function computeSideGait(parsed, kneeSeries, ankleHsSeries, ankleAngleSeries, forwardAxis, detectorOptions, meta = {}) {
  const t0 = parsed.frames[0].time;
  const t = parsed.frames.map((f) => f.time - t0);
  const angleSource = meta.angleSource || (ankleAngleSeries ? 'legacy-ankle' : 'unavailable');

  let angleDeg = null;
  let shankAngularVelocityDps = null;
  let effectiveAngleSource = angleSource;
  if (ankleAngleSeries) {
    const rawAngleDeg = computeShankAngleDeg(kneeSeries, ankleAngleSeries, forwardAxis);
    const filtered = computeFilteredAngularVelocityDps(t, rawAngleDeg, detectorOptions);
    angleDeg = filtered.smoothedAngleDeg;
    shankAngularVelocityDps = filtered.shankAngularVelocityDps;
  } else if (ankleHsSeries && angleSource === 'unavailable') {
    // Heel ใช้คำนวณ |ω| สำหรับ clock sync เท่านั้น — ห้าม peak shank angle / signed polarity
    const rawAngleDeg = computeShankAngleDeg(kneeSeries, ankleHsSeries, forwardAxis);
    const filtered = computeFilteredAngularVelocityDps(t, rawAngleDeg, detectorOptions);
    shankAngularVelocityDps = filtered.shankAngularVelocityDps;
    effectiveAngleSource = 'heel-for-envelope-only';
  }

  const forwardPosition = computeForwardPosition(ankleHsSeries, forwardAxis);
  const { smoothedPosition, velocity } = computeFilteredVelocity(t, forwardPosition, detectorOptions);
  const stanceIntervals = detectStanceIntervals(t, velocity, detectorOptions);
  // ใช้ smoothed position วัด stride — สอดคล้องกับสัญญาณที่ใช้ detect event
  const rawCycles = buildCyclesFromStanceIntervals(t, smoothedPosition, stanceIntervals, detectorOptions);

  const cycles = rawCycles.map((cycle) => {
    let peakAngleDeg = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    for (let i = cycle.hsStartIdx; i <= cycle.hsEndIdx; i += 1) {
      if (angleDeg && Number.isFinite(angleDeg[i])) peakAngleDeg = Math.max(peakAngleDeg, angleDeg[i]);
      if (Number.isFinite(ankleHsSeries.y[i])) {
        minY = Math.min(minY, ankleHsSeries.y[i]);
        maxY = Math.max(maxY, ankleHsSeries.y[i]);
      }
    }

    const walkingSpeedMps = cycle.strideTimeS > 0 ? cycle.strideLengthM / cycle.strideTimeS : null;
    const cadenceSpm = cycle.strideTimeS > 0 ? (2 / cycle.strideTimeS) * 60 : null;
    const ankleClearanceM = Number.isFinite(maxY) && Number.isFinite(minY)
      ? Math.max(0, maxY - minY)
      : null;

    return {
      hsStartTimeS: cycle.hsStartTimeS,
      hsEndTimeS: cycle.hsEndTimeS,
      toTimeS: cycle.toTimeS,
      strideTimeS: cycle.strideTimeS,
      stanceTimeS: cycle.stanceTimeS,
      swingTimeS: cycle.swingTimeS,
      stancePct: cycle.stancePct,
      swingPct: cycle.swingPct,
      strideLengthM: cycle.strideLengthM,
      cadenceSpm,
      walkingSpeedMps,
      // ไม่มี malleolus → ห้ามเคลม peak shank angle จาก heel
      peakShankAngleDeg: angleDeg && Number.isFinite(peakAngleDeg) ? peakAngleDeg : null,
      ankleClearanceM,
    };
  });

  const strideLengths = cycles.map((c) => c.strideLengthM).filter(Number.isFinite);
  const cadences = cycles.map((c) => c.cadenceSpm).filter(Number.isFinite);
  const mean = (arr) => (arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : null);

  return {
    cycles,
    summary: {
      cycleCount: cycles.length,
      meanStrideLengthM: mean(strideLengths),
      meanCadenceSpm: mean(cadences),
    },
    // สำหรับ align ระดับสัญญาณ (xcorr) กับ IMU gyro — ต้องมี malleolus
    signals: {
      tS: t,
      shankAngularVelocityDps,
      angleSource: effectiveAngleSource,
    },
  };
}

function assertClearancePlausible(side, cycles, warnings) {
  const clearances = cycles.map((c) => c.ankleClearanceM).filter(Number.isFinite);
  if (!clearances.length) return;
  const meanClearance = clearances.reduce((a, b) => a + b, 0) / clearances.length;
  // ช่วงกว้างเผื่อ noise — ถ้านอก 0.02–0.40m มักเป็นแกนผิดหรือหน่วยผิด
  if (meanClearance < 0.02 || meanClearance > 0.40) {
    warnings.push(
      `ขา ${side}: mean ankleClearance=${meanClearance.toFixed(3)}m อยู่นอกช่วงที่คาด (0.02–0.40m) `
      + '— อาจแกนแนวดิ่งผิดหรือหน่วยผิด',
    );
  }
}

export function computeGaitFromMocap(parsed, markerRoles, options = {}) {
  const anklePlan = pickAnkleRolePlan(markerRoles);
  const roleIds = { ...markerRoles };

  // รวบรวม id ที่ต้อง extract (ห้ามซ้ำ)
  const needed = new Map(); // roleLabel -> markerId
  for (const role of ['L_ASIS', 'R_ASIS', 'L_Knee', 'R_Knee']) {
    if (!Number.isFinite(roleIds[role])) {
      throw new Error(`marker role ขาด: ${role}`);
    }
    needed.set(role, roleIds[role]);
  }
  for (const side of ['L', 'R']) {
    const plan = anklePlan[side];
    if (!plan?.hsRole || !Number.isFinite(roleIds[plan.hsRole])) {
      throw new Error(`marker role ขาด ankle สำหรับ HS ขา ${side} (Ankle หรือ AnkleForHS)`);
    }
    needed.set(plan.hsRole, roleIds[plan.hsRole]);
    if (plan.angleRole && Number.isFinite(roleIds[plan.angleRole])) {
      needed.set(plan.angleRole, roleIds[plan.angleRole]);
    }
  }

  const rawByRole = {};
  for (const [role, id] of needed) {
    rawByRole[role] = extractMarkerSeries(parsed, id);
  }

  // interpolate gaps ต่อ series ก่อน normalize หน่วย
  const filledByRole = {};
  for (const [role, raw] of Object.entries(rawByRole)) {
    const { series, remainingGaps } = interpolateGaps(raw);
    assertNoGaps(remainingGaps, `${role} marker`);
    filledByRole[role] = series;
  }

  const {
    seriesByRole,
    unitScale,
    warnings,
  } = assertAndNormalizeCaptureUnits(filledByRole, options);

  for (const side of ['L', 'R']) {
    if (anklePlan[side].angleSource === 'unavailable') {
      warnings.push(
        `ขา ${side}: ไม่มี AnkleForAngle (lateral malleolus) — HS จาก heel/ForHS ได้; `
        + 'shank angle ปิด; signal sync ใช้ได้แค่ |ω| envelope จาก heel (ไม่ใช่ lab gold-standard)',
      );
    }
  }

  const asisL = seriesByRole.L_ASIS;
  const asisR = seriesByRole.R_ASIS;
  const pelvisX = asisL.x.map((v, i) => (v === null || asisR.x[i] === null ? null : (v + asisR.x[i]) / 2));
  const pelvisZ = asisL.z.map((v, i) => (v === null || asisR.z[i] === null ? null : (v + asisR.z[i]) / 2));
  const forwardAxis = computeForwardAxis(pelvisX, pelvisZ);

  const detectorOptions = options.detector || {};
  const left = computeSideGait(
    parsed,
    seriesByRole.L_Knee,
    seriesByRole[anklePlan.L.hsRole],
    anklePlan.L.angleRole ? seriesByRole[anklePlan.L.angleRole] : null,
    forwardAxis,
    detectorOptions,
    { angleSource: anklePlan.L.angleSource },
  );
  const right = computeSideGait(
    parsed,
    seriesByRole.R_Knee,
    seriesByRole[anklePlan.R.hsRole],
    anklePlan.R.angleRole ? seriesByRole[anklePlan.R.angleRole] : null,
    forwardAxis,
    detectorOptions,
    { angleSource: anklePlan.R.angleSource },
  );

  assertClearancePlausible('L', left.cycles, warnings);
  assertClearancePlausible('R', right.cycles, warnings);

  const hsEvents = [
    ...left.cycles.map((c) => ({ side: 'L', timeS: c.hsStartTimeS })),
    ...right.cycles.map((c) => ({ side: 'R', timeS: c.hsStartTimeS })),
  ].sort((a, b) => a.timeS - b.timeS);

  const trueStepTimesS = [];
  let sameSideRepeats = 0;
  for (let i = 1; i < hsEvents.length; i += 1) {
    trueStepTimesS.push(hsEvents[i].timeS - hsEvents[i - 1].timeS);
    if (hsEvents[i].side === hsEvents[i - 1].side) sameSideRepeats += 1;
  }
  // HS สลับข้างพลาด → meanStepTime / trueCadence ไม่น่าเชื่อ — flag ให้ปลายทางซ่อน
  const bilateralReliable = sameSideRepeats === 0;
  if (!bilateralReliable) {
    warnings.push(
      `bilateral.reliable=false: sameSideRepeats=${sameSideRepeats}/${Math.max(0, hsEvents.length - 1)} `
      + 'ช่วง HS — meanStepTimeS / trueCadenceSpm ห้ามใช้ (ซ่อนที่ UI)',
    );
  }
  const meanStepTimeS = trueStepTimesS.length
    ? trueStepTimesS.reduce((a, b) => a + b, 0) / trueStepTimesS.length
    : null;

  const t0 = parsed.frames[0].time;
  const t1 = parsed.frames[parsed.frames.length - 1].time;
  const durationS = t1 - t0;

  // เดินไป-กลับ / PCA: net displacement ≈ 0 → averageWalkingSpeed จาก net ไม่มีความหมาย
  const netMeaningful = forwardAxis.method === 'net-start-end';
  if (!netMeaningful) {
    warnings.push(
      `forwardAxis.method=${forwardAxis.method}: pelvisNet / averageWalkingSpeed จาก net ถูกปิด `
      + `(net=${forwardAxis.netDisplacementM.toFixed(3)}m, maxExcursion=${forwardAxis.maxExcursionM.toFixed(3)}m)`,
    );
  }

  return {
    forwardAxis,
    perSide: { L: left, R: right },
    bilateral: {
      hsEventCount: hsEvents.length,
      trueStepTimesS,
      meanStepTimeS: bilateralReliable ? meanStepTimeS : null,
      trueCadenceSpm: bilateralReliable && meanStepTimeS ? 60 / meanStepTimeS : null,
      sameSideRepeats,
      reliable: bilateralReliable,
      // ค่าดิบไว้ debug เมื่อ reliable=false
      meanStepTimeSRaw: meanStepTimeS,
      trueCadenceSpmRaw: meanStepTimeS ? 60 / meanStepTimeS : null,
    },
    session: {
      durationS,
      pelvisNetForwardDisplacementM: forwardAxis.netDisplacementM,
      pelvisMaxExcursionM: forwardAxis.maxExcursionM,
      forwardAxisMethod: forwardAxis.method,
      pelvisNetMeaningful: netMeaningful,
      averageWalkingSpeedMps: netMeaningful && durationS > 0
        ? forwardAxis.netDisplacementM / durationS
        : null,
    },
    meta: {
      unitScale,
      warnings,
      anklePlan,
    },
  };
}

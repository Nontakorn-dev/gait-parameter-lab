// คำนวณ gait parameter จาก marker ตำแหน่ง (ASIS x2, Knee/Lateral Femoral Epicondyle x2,
// Ankle/Lateral Malleolus x2) — ใช้ GaitEventDetector ตัวเดียวกับที่ pipeline IMU จริงใช้
// เพื่อให้เทียบผลแบบ apples-to-apples (มุม/ความเร็วเชิงมุมจาก mocap ป้อนเข้า detector
// เดียวกับที่ป้อนสัญญาณ gyro จริง)
//
// ⚠️ ข้อจำกัดที่ตั้งใจไว้ตรง ๆ (ยังไม่มีข้อมูลจริงมาตรวจสอบ):
//   - threshold ของ GaitEventDetector ด้านล่างเป็นค่าเริ่มต้นที่ "ยังไม่ผ่านการยืนยัน"
//     กับสัญญาณ shank-angular-velocity ที่ derive จาก mocap จริง (ต่างจาก IMU ตรงที่
//     mocap ไม่มี noise ทางไฟฟ้า แต่ differentiate เชิงตัวเลขจากตำแหน่งจะขยาย jitter ได้)
//     ต้องปรับ (options.detector) เมื่อมีข้อมูลจริงและเห็นว่า event detection พลาด/เกิน
//   - ไม่มี marker ปลายเท้า/heel ในชุดนี้ (ASIS+Knee+Ankle เท่านั้น) จึงไม่มี ground truth
//     สำหรับ toe-off ที่แม่นเท่าไฟล์ที่มี toe marker, และ "clearance" ที่คำนวณได้เป็นแค่
//     ระยะยกของ "ข้อเท้า" ไม่ใช่ปลายเท้า — ใกล้เคียงตำแหน่งที่ IMU ติดจริงบนหน้าแข้งส่วนล่าง
//     พอสมควร แต่ไม่ใช่ค่าเดียวกันเป๊ะกับ clearance ที่ระบบ IMU รายงาน (นั่นวัดจาก double
//     integration ของ accel บนหน้าแข้ง ไม่ใช่ปลายเท้าเช่นกัน แต่คนละตำแหน่ง/คนละวิธี)
//   - สมมติว่าเดินเป็นเส้นตรง (ไม่มีเลี้ยว) ตอนหาแกนทิศทางเดินจาก ASIS midpoint
//   - ⚠️ toe-off / stancePct / swingPct: verify ด้วย synthetic ground truth แล้วพบว่า
//     "unresolved" บ่อยมาก (ไม่ใช่บั๊ก — เป็นข้อจำกัดจริง) เพราะ findLocalMinima ใน
//     GaitEventDetector ใช้ minDistance=10 samples ตายตัว (ไม่ scale ตาม sampleRate)
//     ที่ mocap 120Hz หน้าต่างนี้แทนเวลาสั้นกว่าที่ IMU 100Hz ถูก tune ไว้ ทำให้ prominence
//     ที่วัดได้ต่ำกว่าเกณฑ์ default (30) แทบทุกครั้ง — ลองลด toProminence แล้วเจอว่า TO
//     ที่ detect ได้ตกที่ phase~0.50 ของ stride ไม่ใช่ 0.62 ที่เป็น ground truth จริง
//     (คือคนละจุดกับ toe-off จริง) จึง "ไม่" ลด threshold ให้ เพราะจะได้ค่าที่ดูสมเหตุสมผล
//     แต่ผิด — 'unresolved' (ไม่รู้) ปลอดภัยกว่าตัวเลขที่มั่นใจแต่ผิด. stride length/
//     cadence/HS timing/clearance ไม่กระทบเรื่องนี้เลย (verify แยกแล้วแม่นกับ ground truth)
//     ต้องรอข้อมูลจริงเพื่อ tune toProminence/toSearchPct ให้เจอ TO ถูกจุดจริง

import { GaitEventDetector } from '../src/gait/gaitEventDetector.js';
import { rad2deg } from '../src/gait/signalUtils.js';
import { extractMarkerSeries, interpolateGaps } from './parseOptiTrack.js';

// ค่าเริ่มต้นของ GaitEventDetector สำหรับสัญญาณจาก mocap — "ยังไม่ผ่านการยืนยัน"
// (ดู comment หัวไฟล์) ปรับผ่าน options.detector ได้
const DEFAULT_MOCAP_DETECTOR_OPTIONS = {
  hsProminence: 45,
  toProminence: 30, // มักหา TO ไม่เจอ ("unresolved") ที่ 120Hz — ดู caveat หัวไฟล์ ห้ามลดมั่ว ๆ
  hsVelocityThreshold: -50,
  hsVelocityThresholdFloorAbs: 12,
  minStrideTime: 0.6,
  maxStrideTime: 3.0,
  minHsSeparationSeconds: 0.55,
  toSearchStartPct: 0.20,
  toSearchEndPct: 0.80,
  hsEnvelopeScale: 0.28,
  hsPreviousPeakScale: 0.28,
  hsPreviousHsScale: 0.60,
  hsAdaptiveHistorySize: 3,
  hsEnvelopeWindowSeconds: 0.8,
  minSwingAngularVelocityAbs: 12,
  usePreviousValidStanceFallback: true,
};

function dot2(ax, az, bx, bz) {
  return ax * bx + az * bz;
}

// แกนเดินหลัก (forward) จาก midpoint ของ ASIS สองข้าง — สมมติเดินเส้นตรง (ไม่เลี้ยว)
// ใช้ระยะจากจุดแรกไปจุดสุดท้ายที่มีข้อมูลจริงในแนวราบ (X,Z; Y เป็นแนวดิ่งตาม OptiTrack default)
export function computeForwardAxis(pelvisX, pelvisZ) {
  let firstIdx = -1;
  let lastIdx = -1;
  for (let i = 0; i < pelvisX.length; i += 1) {
    if (pelvisX[i] !== null) {
      if (firstIdx === -1) firstIdx = i;
      lastIdx = i;
    }
  }
  if (firstIdx === -1 || firstIdx === lastIdx) {
    throw new Error('หาแกนทิศทางเดินไม่ได้: ASIS midpoint ไม่มีข้อมูลพอ (อาจถูกบดบังทั้งเฟรม)');
  }

  const dx = pelvisX[lastIdx] - pelvisX[firstIdx];
  const dz = pelvisZ[lastIdx] - pelvisZ[firstIdx];
  const mag = Math.sqrt(dx * dx + dz * dz);
  if (mag < 0.3) {
    throw new Error(
      `pelvis เคลื่อนที่แนวราบสุทธิแค่ ${mag.toFixed(2)}m — สั้นเกินกว่าจะหาแกนทิศทางเดินได้แม่นยำ `
      + '(อาจไม่ใช่ trial เดินจริง หรือเดินไป-กลับจนหักล้างกัน)',
    );
  }
  return { fx: dx / mag, fz: dz / mag, netDisplacementM: mag };
}

// มุมหน้าแข้งเทียบแนวดิ่งในระนาบ sagittal (ระนาบที่มีแกนเดิน+แนวดิ่ง) — เทียบเคียงกับ
// accelToAngle ที่ pipeline IMU ใช้ (atan2 ของ component แนวเดิน กับ แนวดิ่ง)
// เวกเตอร์ที่ใช้: ankle -> knee (ชี้ "ขึ้น" ไปตามหน้าแข้ง)
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

// อนุพันธ์เชิงตัวเลข (central difference) เป็น deg/s จาก dt จริงต่อคู่เฟรม (ทนต่อ dt ไม่คงที่)
export function computeAngularVelocityDps(t, angleDeg) {
  const n = angleDeg.length;
  const w = new Array(n).fill(null);
  for (let i = 1; i < n - 1; i += 1) {
    if (angleDeg[i - 1] === null || angleDeg[i + 1] === null) continue;
    const dt = t[i + 1] - t[i - 1];
    if (dt <= 0) continue;
    w[i] = (angleDeg[i + 1] - angleDeg[i - 1]) / dt;
  }
  // ปลายทั้งสองใช้ forward/backward difference (มีผลแค่ 1 sample แรก/ท้ายของทั้งเซสชัน)
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

function assertNoGaps(remainingGaps, label) {
  if (remainingGaps.length) {
    const ranges = remainingGaps.map((g) => `${g.startTimeS.toFixed(2)}-${g.endTimeS.toFixed(2)}s`).join(', ');
    throw new Error(
      `${label}: marker หายไปนานเกินจะเติมอัตโนมัติที่ช่วง [${ranges}] — ตัดช่วงนั้นออกหรือ `
      + 'capture ใหม่ก่อน (ไม่เดาเติมข้อมูลช่วงยาวเพราะอาจคร่อมจังหวะ heel-strike แล้วปั้นสัญญาณปลอม)',
    );
  }
}

function computeSideGait(parsed, kneeId, ankleId, forwardAxis, detectorOptions) {
  const kneeRaw = extractMarkerSeries(parsed, kneeId);
  const ankleRaw = extractMarkerSeries(parsed, ankleId);
  const { series: knee, remainingGaps: kneeGaps } = interpolateGaps(kneeRaw);
  const { series: ankle, remainingGaps: ankleGaps } = interpolateGaps(ankleRaw);
  assertNoGaps(kneeGaps, `knee marker #${kneeId}`);
  assertNoGaps(ankleGaps, `ankle marker #${ankleId}`);

  const t0 = parsed.frames[0].time;
  const t = parsed.frames.map((f) => f.time - t0);
  const angleDeg = computeShankAngleDeg(knee, ankle, forwardAxis);
  const angularVelocityDps = computeAngularVelocityDps(t, angleDeg);

  if (angleDeg.some((v) => v === null) || angularVelocityDps.some((v) => v === null)) {
    throw new Error('มุมหน้าแข้งหรือความเร็วเชิงมุมคำนวณไม่ได้ครบทุกเฟรม (marker หายที่ปลายช่วงข้อมูล?)');
  }

  const detector = new GaitEventDetector({
    ...DEFAULT_MOCAP_DETECTOR_OPTIONS,
    ...detectorOptions,
    sampleRate: parsed.frameRateHz,
  });
  const { events, cycles: rawCycles } = detector.detect(angularVelocityDps, t);

  const cycles = rawCycles.map((cycle) => {
    const startIdx = cycle.hsStart.index;
    const endIdx = cycle.hsEnd.index;

    let peakAngleDeg = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    for (let i = startIdx; i <= endIdx; i += 1) {
      peakAngleDeg = Math.max(peakAngleDeg, angleDeg[i]);
      minY = Math.min(minY, ankle.y[i]);
      maxY = Math.max(maxY, ankle.y[i]);
    }

    const strideVecX = ankle.x[endIdx] - ankle.x[startIdx];
    const strideVecZ = ankle.z[endIdx] - ankle.z[startIdx];
    const strideLengthM = Math.abs(dot2(strideVecX, strideVecZ, forwardAxis.fx, forwardAxis.fz));
    const strideLengthM2d = Math.sqrt(strideVecX * strideVecX + strideVecZ * strideVecZ);

    const strideTimeS = cycle.strideTime;
    const cadenceSpm = strideTimeS > 0 ? (2 / strideTimeS) * 60 : null;
    const walkingSpeedMps = strideTimeS > 0 ? strideLengthM / strideTimeS : null;

    return {
      hsStartTimeS: cycle.hsStart.time,
      hsEndTimeS: cycle.hsEnd.time,
      strideTimeS,
      stanceTimeS: cycle.stanceTime,
      swingTimeS: cycle.swingTime,
      stancePct: cycle.stancePct,
      swingPct: cycle.swingPct,
      temporalSource: cycle.temporalSource,
      strideLengthM,
      strideLengthM2d,
      cadenceSpm,
      walkingSpeedMps,
      peakShankAngleDeg: peakAngleDeg,
      // proxy: ระยะยกข้อเท้า ไม่ใช่ปลายเท้า/จุดติด IMU เป๊ะ ๆ — ดู caveat หัวไฟล์
      ankleClearanceM: Math.max(0, maxY - minY),
    };
  });

  const strideLengths = cycles.map((c) => c.strideLengthM).filter(Number.isFinite);
  const cadences = cycles.map((c) => c.cadenceSpm).filter(Number.isFinite);
  const mean = (arr) => (arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : null);

  return {
    cycles,
    events,
    summary: {
      cycleCount: cycles.length,
      meanStrideLengthM: mean(strideLengths),
      meanCadenceSpm: mean(cadences),
    },
  };
}

export function computeGaitFromMocap(parsed, markerRoles, options = {}) {
  const { L_ASIS, R_ASIS, L_Knee, R_Knee, L_Ankle, R_Ankle } = markerRoles;
  const asisL = extractMarkerSeries(parsed, L_ASIS);
  const asisR = extractMarkerSeries(parsed, R_ASIS);

  const pelvisX = asisL.x.map((v, i) => (v === null || asisR.x[i] === null ? null : (v + asisR.x[i]) / 2));
  const pelvisZ = asisL.z.map((v, i) => (v === null || asisR.z[i] === null ? null : (v + asisR.z[i]) / 2));
  const pelvisY = asisL.y.map((v, i) => (v === null || asisR.y[i] === null ? null : (v + asisR.y[i]) / 2));
  const forwardAxis = computeForwardAxis(pelvisX, pelvisZ);

  const left = computeSideGait(parsed, L_Knee, L_Ankle, forwardAxis, options.detector);
  const right = computeSideGait(parsed, R_Knee, R_Ankle, forwardAxis, options.detector);

  // bilateral: รวม HS ทั้งสองข้างตามเวลาจริง เพื่อวัด step time/cadence ที่ไม่ต้องสมมติสมมาตร
  // (ต่างจาก IMU ข้างเดียวที่ stepTime = strideTime/2 เสมอ — ใช้ตรงนี้ตรวจสมมติฐานนั้นได้)
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
  const meanStepTimeS = trueStepTimesS.length
    ? trueStepTimesS.reduce((a, b) => a + b, 0) / trueStepTimesS.length
    : null;

  const t0 = parsed.frames[0].time;
  const t1 = parsed.frames[parsed.frames.length - 1].time;
  const durationS = t1 - t0;

  return {
    forwardAxis,
    perSide: { L: left, R: right },
    bilateral: {
      hsEventCount: hsEvents.length,
      trueStepTimesS,
      meanStepTimeS,
      trueCadenceSpm: meanStepTimeS ? 60 / meanStepTimeS : null,
      sameSideRepeats,
    },
    session: {
      durationS,
      pelvisNetForwardDisplacementM: forwardAxis.netDisplacementM,
      averageWalkingSpeedMps: durationS > 0 ? forwardAxis.netDisplacementM / durationS : null,
    },
  };
}

// คำนวณ gait parameter จาก marker ตำแหน่ง (ASIS x2, Knee/Lateral Femoral Epicondyle x2,
// Ankle/Lateral Malleolus x2)
//
// Event detection (Heel Strike / Toe Off) ใช้ "foot velocity algorithm" — มองพฤติกรรม
// การเคลื่อนที่ของข้อเท้าโดยตรง (นิ่ง = แตะพื้น, ขยับ = แกว่ง) ไม่ได้ใช้ GaitEventDetector
// ของ IMU pipeline อีกต่อไป: การใช้ detector ตัวเดียวกันสำหรับทั้งสองแหล่งข้อมูลมีจุดบอดร่วม
// (shared blind spot) — ถ้า algorithm มี bias เป็นระบบ bias นั้นจะปรากฏเหมือนกันทั้งสองฝั่ง
// แล้วหักล้างกันหายไปตอนเทียบผล ทำให้ดูเหมือน "ตรงกัน" ทั้งที่ทั้งคู่ผิดไปทางเดียวกัน วิธีนี้
// (velocityEventDetector.js) จึงเป็น ground truth ที่อิสระจากอัลกอริทึมฝั่ง IMU จริง ๆ
// (ดู mocap-analysis/velocityEventDetector.js สำหรับรายละเอียด + อ้างอิงวรรณกรรม)
//
// ⚠️ ข้อจำกัดที่ตั้งใจไว้ตรง ๆ (ยังไม่มีข้อมูลจริงมาตรวจสอบ):
//   - threshold ของ velocityEventDetector เป็นค่าเริ่มต้นที่ตรวจสอบแล้วว่าเสถียรกับ
//     synthetic ground truth (คลาดเคลื่อน stance duration < 1.5% ในช่วง 0.02-0.3 m/s)
//     แต่ยังไม่เคยเห็นข้อมูลจริงซึ่งมี marker noise/jitter มากกว่าข้อมูลสังเคราะห์ไร้ noise
//   - ไม่มี marker ปลายเท้า/heel ในชุดนี้ (ASIS+Knee+Ankle เท่านั้น) จึงใช้ "ข้อเท้า" เป็นจุด
//     สังเกตการแตะพื้นแทนส้นเท้า/ปลายเท้าโดยตรง — สมเหตุสมผลเพราะข้อเท้า/malleolus แทบไม่ขยับ
//     แนวราบตอน stance เหมือนกัน (foot flat, ankle joint เป็นจุดหมุน) แต่ TO ที่ได้อาจหมายถึง
//     "เท้าเริ่มขยับ" กว้าง ๆ ไม่ใช่ "ปลายเท้าพ้นพื้น" เป๊ะเหมือนมี toe marker จริง
//   - "clearance" คือระยะยกของ "ข้อเท้า" ไม่ใช่ปลายเท้า — ใกล้เคียงตำแหน่งที่ IMU ติดจริง
//     บนหน้าแข้งส่วนล่างพอสมควร แต่ไม่ใช่ค่าเดียวกันเป๊ะกับ clearance ที่ระบบ IMU รายงาน
//     (นั่นวัดจาก double integration ของ accel บนหน้าแข้ง คนละตำแหน่ง/คนละวิธี)
//   - สมมติว่าเดินเป็นเส้นตรง (ไม่มีเลี้ยว) ตอนหาแกนทิศทางเดินจาก ASIS midpoint
//
// shankAngleDeg/angularVelocityDps ยังคำนวณไว้ (peakShankAngleDeg ในผลลัพธ์ + เผื่อใช้ทำ
// direct signal-correlation เทียบกับมุมจาก IMU+Kalman โดยตรงในอนาคต — เป็นการ validate ที่
// อิสระกว่าเดิมอีกแบบ เพราะไม่ต้องพึ่ง event detector เลยด้วยซ้ำ) แต่ไม่ได้ใช้หา HS/TO แล้ว

import { rad2deg } from '../src/gait/signalUtils.js';
import { extractMarkerSeries, interpolateGaps } from './parseOptiTrack.js';
import { computeForwardPosition, computeVelocity, detectStanceIntervals, buildCyclesFromStanceIntervals } from './velocityEventDetector.js';

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
// เก็บไว้สำหรับ direct signal-correlation เทียบกับ IMU ในอนาคต (ไม่ได้ใช้หา HS/TO แล้ว)
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

  // มุมหน้าแข้ง — ใช้แค่รายงาน peakShankAngleDeg ไม่ได้ใช้หา event อีกต่อไป
  const angleDeg = computeShankAngleDeg(knee, ankle, forwardAxis);

  // Heel-Strike/Toe-Off จากความเร็วแนวเดินของข้อเท้าโดยตรง — อิสระจาก IMU detector 100%
  const forwardPosition = computeForwardPosition(ankle, forwardAxis);
  const forwardVelocity = computeVelocity(t, forwardPosition);
  const stanceIntervals = detectStanceIntervals(t, forwardVelocity, detectorOptions);
  const rawCycles = buildCyclesFromStanceIntervals(t, forwardPosition, stanceIntervals, detectorOptions);

  const cycles = rawCycles.map((cycle) => {
    let peakAngleDeg = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    for (let i = cycle.hsStartIdx; i <= cycle.hsEndIdx; i += 1) {
      if (Number.isFinite(angleDeg[i])) peakAngleDeg = Math.max(peakAngleDeg, angleDeg[i]);
      minY = Math.min(minY, ankle.y[i]);
      maxY = Math.max(maxY, ankle.y[i]);
    }

    const walkingSpeedMps = cycle.strideTimeS > 0 ? cycle.strideLengthM / cycle.strideTimeS : null;
    const cadenceSpm = cycle.strideTimeS > 0 ? (2 / cycle.strideTimeS) * 60 : null;

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
      peakShankAngleDeg: Number.isFinite(peakAngleDeg) ? peakAngleDeg : null,
      // proxy: ระยะยกข้อเท้า ไม่ใช่ปลายเท้า/จุดติด IMU เป๊ะ ๆ — ดู caveat หัวไฟล์
      ankleClearanceM: Math.max(0, maxY - minY),
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
  };
}

export function computeGaitFromMocap(parsed, markerRoles, options = {}) {
  const { L_ASIS, R_ASIS, L_Knee, R_Knee, L_Ankle, R_Ankle } = markerRoles;
  const asisL = extractMarkerSeries(parsed, L_ASIS);
  const asisR = extractMarkerSeries(parsed, R_ASIS);

  const pelvisX = asisL.x.map((v, i) => (v === null || asisR.x[i] === null ? null : (v + asisR.x[i]) / 2));
  const pelvisZ = asisL.z.map((v, i) => (v === null || asisR.z[i] === null ? null : (v + asisR.z[i]) / 2));
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

// Pure, DOM-free helpers สำหรับ validate/app.js — แยกไว้ต่างหากเพื่อ unit test ได้ตรง ๆ
// (ไม่มีการอ้าง document/window เลยในไฟล์นี้)

// ⚠️ ค่า default เหล่านี้ "ยังไม่ผ่านการยืนยันด้วยข้อมูลเดินจริง" — เป็นแค่จุดเริ่มต้นที่เดาไว้
// ก่อนพัฒนา (ตัวเลขตัวอย่างที่เคยพูดถึง เช่น vEnd~-0.79/accelDev~0.21 มาจาก "demo" สังเคราะห์
// ระหว่างพัฒนา ไม่ใช่ ground truth จากการเดินจริง). อย่าตีความว่าค่าเหล่านี้พิสูจน์ว่า ZUPT
// วางผิดจุด — ต้องรอ validate กับ trace จริงที่มี ground-truth distance ก่อน แล้วค่อยปรับ.
// หน้า UI ให้ผู้ใช้แก้ threshold เองได้ (ดู renderCycles) — export ไว้เป็นแค่ค่าเริ่มต้น
export const DEFAULT_FLAG_V_END_MPS = 0.15;
export const DEFAULT_FLAG_ACCEL_DEVIATION_G = 0.25;
// เกณฑ์ ±% สำหรับเทียบ sum(strideLength) กับ ground-truth distance
export const DISTANCE_OK_PCT = 10;
export const DISTANCE_WARN_PCT = 25;

export function formatNum(value, digits = 2) {
  if (value == null || Number.isNaN(value)) return "—";
  return Number(value).toLocaleString("en-US", {
    maximumFractionDigits: digits,
    minimumFractionDigits: 0,
  });
}

export function formatSigned(value, digits = 3) {
  if (value == null || Number.isNaN(value)) return "—";
  const n = Number(value);
  return `${n >= 0 ? "+" : ""}${n.toFixed(digits)}`;
}

export function formatDate(iso) {
  if (!iso) return "—";
  const d = new Date(iso);
  return Number.isNaN(d.getTime()) ? iso : d.toLocaleString("th-TH");
}

export function sensorLabel(sensorKey, side) {
  if (side === "L") return "Left Shank";
  if (side === "R") return "Right Shank";
  return sensorKey || "—";
}

export function computeCyclesBySensor(cycles) {
  const map = new Map();
  for (const c of cycles || []) {
    const key = c.sensorKey || "_";
    if (!map.has(key)) map.set(key, []);
    map.get(key).push(c);
  }
  return map;
}

// min ที่ไม่ใช้ spread — Math.min(...bigArray) ทำ stack overflow ที่ราว ~1-1.2 แสน element
// (ยืนยันแล้วว่า RangeError ที่ 150k) ในขณะที่ trace cap คือ maxSamples=300000 จริง ๆ ได้
function minFinite(values) {
  let min = Infinity;
  for (const v of values) {
    if (v < min) min = v;
  }
  return Number.isFinite(min) ? min : null;
}

export function summarizeCycles(cycles, thresholds = {}) {
  const list = cycles || [];
  const n = list.length;
  if (!n) return null;

  const flagVEnd = Number.isFinite(thresholds.vEndMps) ? thresholds.vEndMps : DEFAULT_FLAG_V_END_MPS;
  const flagAccelDev = Number.isFinite(thresholds.accelDeviationG) ? thresholds.accelDeviationG : DEFAULT_FLAG_ACCEL_DEVIATION_G;

  // สำคัญ: vEndPreDrift/zuptAccelDeviationG เป็น null ได้จริงจาก processor เมื่อ integration
  // window สั้นเกินไป (ดู gaitProcessor.js). ถ้าแปลง null -> 0 ก่อนเฉลี่ย จะอ่านว่า "v ปลาย
  // window = 0 พอดี" ซึ่งคือ ZUPT ที่ดีที่สุดเท่าที่จะเป็นไปได้ — ตรงข้ามกับความจริงที่ไม่มีข้อมูล
  // เลย จึงต้องกรอง null ออกจาก mean และนับแยกเป็น noDataCount แทนที่จะเงียบ (เหมือน
  // strideLengthClamped/cyclesTruncated ที่ flag ไว้ชัดแทนตัดข้อมูลแบบไม่บอก)
  const vEndValues = [];
  const vStartValues = [];
  const accelDevValues = [];
  let noZuptDataCount = 0;

  for (const c of list) {
    const vEnd = c.zuptCheck?.vEndPreDrift;
    const vStart = c.zuptCheck?.vStartPreDrift;
    const dev = c.zuptCheck?.zuptAccelDeviationG;
    const hasVEnd = Number.isFinite(vEnd);
    const hasDev = Number.isFinite(dev);
    if (hasVEnd) vEndValues.push(Math.abs(vEnd));
    if (Number.isFinite(vStart)) vStartValues.push(Math.abs(vStart));
    if (hasDev) accelDevValues.push(dev);
    if (!hasVEnd || !hasDev) noZuptDataCount += 1;
  }

  const mean = (arr) => (arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : null);

  // flag เฉพาะ cycle ที่ "มีข้อมูลจริง" และเกิน threshold — cycle ที่ไม่มีข้อมูลนับใน
  // noZuptDataCount แยกต่างหาก ไม่ปนกับ flaggedCount (ไม่งั้นจะดูเหมือนผ่านเกณฑ์ทั้งที่จริง ๆ วัดไม่ได้)
  const flaggedCount = list.filter((c) => {
    const vEnd = c.zuptCheck?.vEndPreDrift;
    const dev = c.zuptCheck?.zuptAccelDeviationG;
    const vEndBad = Number.isFinite(vEnd) && Math.abs(vEnd) > flagVEnd;
    const devBad = Number.isFinite(dev) && dev > flagAccelDev;
    return vEndBad || devBad;
  }).length;

  const sourceCounts = {};
  for (const c of list) {
    const src = c.zuptCheck?.windowSource ?? "unknown";
    sourceCounts[src] = (sourceCounts[src] || 0) + 1;
  }

  return {
    count: n,
    clampedCount: list.filter((c) => c.strideLengthClamped).length,
    flaggedCount,
    noZuptDataCount,
    meanAbsVEnd: mean(vEndValues),
    meanAbsVStart: mean(vStartValues),
    meanZuptAccelDeviationG: mean(accelDevValues),
    sourceCounts,
    thresholdsUsed: { vEndMps: flagVEnd, accelDeviationG: flagAccelDev },
  };
}

export function computeDistanceCheck(data) {
  const distanceM = data.groundTruth?.distanceM;
  const bySensor = computeCyclesBySensor(data.cycles);
  const perSensor = [];
  for (const [sensorKey, list] of bySensor) {
    // strideLengthM ในไพพ์ไลน์จริงเป็นเลขเสมอ (ผ่าน clamp แล้ว) แต่กันไว้เผื่อไฟล์ที่แก้ไข
    // มือหรือเคสอนาคต — cycle ที่ไม่มีค่าไม่ควรถูกนับเป็น 0 (จะทำให้ sum ต่ำลงและ errorPct
    // ติดลบเกินจริงแบบเงียบ ๆ) จึงแยกนับเป็น noDataCount แทน
    let sum = 0;
    let noDataCount = 0;
    for (const c of list) {
      if (Number.isFinite(c.strideLengthM)) {
        sum += c.strideLengthM;
      } else {
        noDataCount += 1;
      }
    }
    perSensor.push({ sensorKey, side: list[0]?.side ?? null, sumStrideLengthM: sum, cycleCount: list.length, noDataCount });
  }
  if (!Number.isFinite(distanceM) || !perSensor.length) {
    return { distanceM: Number.isFinite(distanceM) ? distanceM : null, perSensor, hasCheck: false };
  }
  const withError = perSensor.map((p) => ({
    ...p,
    errorPct: ((p.sumStrideLengthM - distanceM) / distanceM) * 100,
  }));
  return { distanceM, perSensor: withError, hasCheck: true };
}

export function distanceCheckClass(errorPct) {
  const abs = Math.abs(errorPct);
  if (abs <= DISTANCE_OK_PCT) return "ok";
  if (abs <= DISTANCE_WARN_PCT) return "warn";
  return "bad";
}

export function computeGlobalT0Ms(data) {
  const sampleTimes = (data.samples || []).map((s) => s.t_ms).filter(Number.isFinite);
  if (sampleTimes.length) return minFinite(sampleTimes);
  const cycleTimes = (data.cycles || []).map((c) => c.cycleStartTimestampMs).filter(Number.isFinite);
  return cycleTimes.length ? minFinite(cycleTimes) : 0;
}

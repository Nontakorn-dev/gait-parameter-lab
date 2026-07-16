// Pure, DOM-free helpers สำหรับ validate/app.js — แยกไว้ต่างหากเพื่อ unit test ได้ตรง ๆ
// (ไม่มีการอ้าง document/window เลยในไฟล์นี้)

// Cycle ถูก "flag" ว่าน่าสงสัยเมื่อ velocity ก่อน correctDrift ที่ปลาย window (vEndPreDrift)
// หรือ accel deviation จาก 1g ที่ปลาย window สูงเกิน threshold นี้ — อ้างอิงจากผลวิเคราะห์จริง
// (v_end ~-0.79 m/s + accelDev ~0.21g ถูกยืนยันว่าเป็น ZUPT ที่วางผิดจุดในบทสนทนาก่อนหน้า)
export const FLAG_V_END_MPS = 0.15;
export const FLAG_ACCEL_DEVIATION_G = 0.25;
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

export function summarizeCycles(cycles) {
  const list = cycles || [];
  const n = list.length;
  if (!n) return null;

  const mean = (arr) => (arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : null);
  const vEndAbs = list.map((c) => Math.abs(c.zuptCheck?.vEndPreDrift ?? 0));
  const vStartAbs = list.map((c) => Math.abs(c.zuptCheck?.vStartPreDrift ?? 0));
  const zuptDevs = list.map((c) => c.zuptCheck?.zuptAccelDeviationG ?? 0);

  const flaggedCount = list.filter((c) => {
    const vEnd = Math.abs(c.zuptCheck?.vEndPreDrift ?? 0);
    const dev = c.zuptCheck?.zuptAccelDeviationG ?? 0;
    return vEnd > FLAG_V_END_MPS || dev > FLAG_ACCEL_DEVIATION_G;
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
    meanAbsVEnd: mean(vEndAbs),
    meanAbsVStart: mean(vStartAbs),
    meanZuptAccelDeviationG: mean(zuptDevs),
    sourceCounts,
  };
}

export function computeDistanceCheck(data) {
  const distanceM = data.groundTruth?.distanceM;
  const bySensor = computeCyclesBySensor(data.cycles);
  const perSensor = [];
  for (const [sensorKey, list] of bySensor) {
    const sum = list.reduce((s, c) => s + (c.strideLengthM || 0), 0);
    perSensor.push({ sensorKey, side: list[0]?.side ?? null, sumStrideLengthM: sum, cycleCount: list.length });
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
  if (sampleTimes.length) return Math.min(...sampleTimes);
  const cycleTimes = (data.cycles || []).map((c) => c.cycleStartTimestampMs).filter(Number.isFinite);
  return cycleTimes.length ? Math.min(...cycleTimes) : 0;
}

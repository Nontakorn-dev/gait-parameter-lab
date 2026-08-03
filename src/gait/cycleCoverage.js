/**
 * Coverage ของ closed IMU cycles บนแกนเวลา — ตรวจช่องว่างหลัง retract/แยก merged stride
 * (เช่น แทน [t0→t2] ด้วย [t1→t2] แล้วช่วง [t0→t1] หายไปโดยไม่มีธง)
 */

/** ช่องว่างภายใน (ระหว่าง HS แรก–สุดท้าย) เกินนี้ → hasCoverageGap */
export const COVERAGE_GAP_THRESHOLD_S = 0.40;

/**
 * @param {Array<object>} cycles
 * @param {{ t0Ms?: number|null, gapThresholdS?: number }} [options]
 * @returns {{
 *   spanS: number|null,
 *   coveredS: number,
 *   coverageGapS: number,
 *   coverageRatio: number|null,
 *   hasCoverageGap: boolean,
 *   intervalCount: number,
 * }}
 */
export function computeCycleTimeCoverage(cycles, options = {}) {
  const gapThresholdS = options.gapThresholdS ?? COVERAGE_GAP_THRESHOLD_S;
  const t0Ms = options.t0Ms;
  const intervals = [];

  for (const c of cycles || []) {
    if (!c || c.retracted || c.isOpenStride) continue;
    let t0 = Number.isFinite(c.cycleStartTimeS) ? c.cycleStartTimeS : null;
    if (!Number.isFinite(t0) && Number.isFinite(c.cycleStartTimestampMs) && Number.isFinite(t0Ms)) {
      t0 = (c.cycleStartTimestampMs - t0Ms) / 1000;
    }
    let t1 = null;
    if (Number.isFinite(c.cycleEndTimestampMs) && Number.isFinite(c.cycleStartTimestampMs)) {
      const dur = (c.cycleEndTimestampMs - c.cycleStartTimestampMs) / 1000;
      if (dur > 0 && Number.isFinite(t0)) t1 = t0 + dur;
    } else if (Number.isFinite(c.strideTimeS) && Number.isFinite(t0) && c.strideTimeS > 0) {
      t1 = t0 + c.strideTimeS;
    }
    if (Number.isFinite(t0) && Number.isFinite(t1) && t1 > t0) {
      intervals.push({ t0, t1 });
    }
  }

  if (!intervals.length) {
    return {
      spanS: null,
      coveredS: 0,
      coverageGapS: 0,
      coverageRatio: null,
      hasCoverageGap: false,
      intervalCount: 0,
    };
  }

  intervals.sort((a, b) => a.t0 - b.t0 || a.t1 - b.t1);
  const spanStart = intervals[0].t0;
  const spanEnd = Math.max(...intervals.map((iv) => iv.t1));
  const spanS = spanEnd - spanStart;

  let coveredS = 0;
  let coverageGapS = 0;
  let cur0 = intervals[0].t0;
  let cur1 = intervals[0].t1;
  for (let i = 1; i < intervals.length; i += 1) {
    const iv = intervals[i];
    if (iv.t0 <= cur1 + 1e-6) {
      cur1 = Math.max(cur1, iv.t1);
    } else {
      coverageGapS += iv.t0 - cur1;
      coveredS += cur1 - cur0;
      cur0 = iv.t0;
      cur1 = iv.t1;
    }
  }
  coveredS += cur1 - cur0;

  const coverageRatio = spanS > 0 ? coveredS / spanS : null;
  return {
    spanS,
    coveredS,
    coverageGapS,
    coverageRatio,
    hasCoverageGap: coverageGapS > gapThresholdS,
    intervalCount: intervals.length,
  };
}

/** closed cycle key — ต้องรวม endId เพื่อไม่ให้ retract ชี้ตัวเองเมื่อ hsStart เดิมแต่ end เปลี่ยน */
export function makeClosedCycleKey(startId, endId) {
  return `${startId}-${endId}`;
}

export function parseCycleKeyStartId(cycleKey) {
  const s = String(cycleKey ?? '');
  if (s.startsWith('open:')) {
    const n = Number(s.slice(5));
    return Number.isFinite(n) ? n : null;
  }
  const m = /^(\d+)(?:-(\d+))?$/.exec(s);
  if (!m) return null;
  const n = Number(m[1]);
  return Number.isFinite(n) ? n : null;
}

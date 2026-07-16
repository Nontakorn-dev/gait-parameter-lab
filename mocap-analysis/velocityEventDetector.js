// Heel-Strike / Toe-Off detection จากความเร็วของ marker โดยตรง ("foot velocity
// algorithm" — O'Connor et al. 2007, Ghoussayni et al. 2004) — ไม่พึ่ง
// GaitEventDetector ของ IMU pipeline เลย จึงเป็น ground truth ที่อิสระจริง
//
// หลักการ (ตรงตามที่วิเคราะห์: มองพฤติกรรมการเคลื่อนที่ ไม่ใช่หาคำว่า "heel strike"
// ในข้อมูล):
//   - Swing: ส้นเท้า/ข้อเท้าเคลื่อนที่ไปข้างหน้าเร็ว (forward velocity สูง)
//   - Heel Strike: แตะพื้น -> ความเร็วลดฮวบเกือบเป็นศูนย์ทันที
//   - Stance: เท้าติดพื้น -> ความเร็วนิ่งอยู่ใกล้ศูนย์ต่อเนื่อง
//   - Toe Off: เท้าเริ่มเคลื่อนอีกครั้ง -> ความเร็วขยับขึ้นจากศูนย์
// จึงมองหา "ช่วงนิ่ง" (|v| <= threshold ต่อเนื่องนานพอ) ในสัญญาณความเร็วแนวเดินของข้อเท้า
// จุดเริ่มช่วงนิ่ง = Heel Strike, จุดสุดท้ายของช่วงนิ่ง (ก่อนขยับอีกครั้ง) = Toe Off

// ตำแหน่งของ marker ที่ project ลงบนแกนเดิน (forward axis) ต่อเฟรม
export function computeForwardPosition(series, forwardAxis) {
  const n = series.x.length;
  const pos = new Array(n).fill(null);
  for (let i = 0; i < n; i += 1) {
    if (series.x[i] === null || series.z[i] === null) continue;
    pos[i] = series.x[i] * forwardAxis.fx + series.z[i] * forwardAxis.fz;
  }
  return pos;
}

// อนุพันธ์เชิงตัวเลข (central difference) จาก dt จริงต่อคู่เฟรม — รูปแบบเดียวกับ
// computeAngularVelocityDps ใน gaitFromMocap.js (ทนต่อ dt ไม่คงที่)
export function computeVelocity(t, position) {
  const n = position.length;
  const v = new Array(n).fill(null);
  for (let i = 1; i < n - 1; i += 1) {
    if (position[i - 1] === null || position[i + 1] === null) continue;
    const dt = t[i + 1] - t[i - 1];
    if (dt <= 0) continue;
    v[i] = (position[i + 1] - position[i - 1]) / dt;
  }
  if (n >= 2 && position[0] !== null && position[1] !== null) {
    const dt = t[1] - t[0];
    if (dt > 0) v[0] = (position[1] - position[0]) / dt;
  }
  if (n >= 2 && position[n - 1] !== null && position[n - 2] !== null) {
    const dt = t[n - 1] - t[n - 2];
    if (dt > 0) v[n - 1] = (position[n - 1] - position[n - 2]) / dt;
  }
  return v;
}

// ค่า default อ้างอิงจากวรรณกรรม (O'Connor et al. 2007 ใช้ราว 0.1-0.2 m/s) และยืนยัน
// เชิงตัวเลขแล้วว่าเสถียรในช่วงกว้าง 0.01-0.3 m/s กับ synthetic ground truth
// (คลาดเคลื่อน stance duration < 1.5% ทุก threshold ที่ลอง) — ยังควรตรวจสอบซ้ำกับข้อมูลจริง
const DEFAULT_STANCE_VELOCITY_THRESHOLD_MPS = 0.15;
const DEFAULT_MIN_STANCE_DURATION_S = 0.2;
const DEFAULT_MIN_STRIDE_TIME_S = 0.6;
const DEFAULT_MAX_STRIDE_TIME_S = 3.0;

// หาทุกช่วง "นิ่ง" (stance) ในสัญญาณความเร็ว — คืน [{hsIndex, toIndex, hsTimeS, toTimeS}]
export function detectStanceIntervals(t, velocity, options = {}) {
  const thresholdMps = options.thresholdMps ?? DEFAULT_STANCE_VELOCITY_THRESHOLD_MPS;
  const minStanceDurationS = options.minStanceDurationS ?? DEFAULT_MIN_STANCE_DURATION_S;

  const intervals = [];
  let runStart = null;
  for (let i = 0; i < velocity.length; i += 1) {
    const quiet = velocity[i] !== null && Math.abs(velocity[i]) <= thresholdMps;
    if (quiet && runStart === null) {
      runStart = i;
    }
    if (!quiet && runStart !== null) {
      const runEnd = i - 1;
      if (t[runEnd] - t[runStart] >= minStanceDurationS) {
        intervals.push({ hsIndex: runStart, toIndex: runEnd, hsTimeS: t[runStart], toTimeS: t[runEnd] });
      }
      runStart = null;
    }
  }
  // หมายเหตุ: run ที่ยังนิ่งอยู่ตอนข้อมูลจบ (ไม่มี "ขยับอีกครั้ง" ปิดท้าย) ไม่นับ เพราะไม่รู้ว่า
  // toe-off จริงเกิดตอนไหน (ข้อมูลตัดก่อน)

  return intervals;
}

// รวม stance interval ที่ต่อเนื่องกันเป็น cycle (HS[i] -> HS[i+1] ของขาเดียวกัน) พร้อมกรอง
// stride ที่สั้น/ยาวผิดปกติ (marker noise ทำให้เกิด quiet-run ปลอมสั้น ๆ ระหว่าง swing ได้)
export function buildCyclesFromStanceIntervals(t, forwardPosition, stanceIntervals, options = {}) {
  const minStrideTimeS = options.minStrideTimeS ?? DEFAULT_MIN_STRIDE_TIME_S;
  const maxStrideTimeS = options.maxStrideTimeS ?? DEFAULT_MAX_STRIDE_TIME_S;

  const cycles = [];
  for (let i = 0; i < stanceIntervals.length - 1; i += 1) {
    const cur = stanceIntervals[i];
    const next = stanceIntervals[i + 1];
    const strideTimeS = next.hsTimeS - cur.hsTimeS;
    if (strideTimeS < minStrideTimeS || strideTimeS > maxStrideTimeS) continue;

    const stanceTimeS = cur.toTimeS - cur.hsTimeS;
    const swingTimeS = next.hsTimeS - cur.toTimeS;
    const stancePct = (stanceTimeS / strideTimeS) * 100;
    const swingPct = 100 - stancePct;
    const strideLengthM = Math.abs(forwardPosition[next.hsIndex] - forwardPosition[cur.hsIndex]);

    cycles.push({
      hsStartIdx: cur.hsIndex,
      hsEndIdx: next.hsIndex,
      toIdx: cur.toIndex,
      hsStartTimeS: cur.hsTimeS,
      hsEndTimeS: next.hsTimeS,
      toTimeS: cur.toTimeS,
      strideTimeS,
      stanceTimeS,
      swingTimeS,
      stancePct,
      swingPct,
      strideLengthM,
    });
  }
  return cycles;
}

// Heel-Strike / Toe-Off detection จากความเร็วของ marker โดยตรง ("foot velocity
// algorithm" — O'Connor et al. 2007, Ghoussayni et al. 2004) — ไม่พึ่ง
// GaitEventDetector ของ IMU pipeline เลย จึงเป็น ground truth ที่อิสระจริง
//
// หลักการ:
//   - Swing: ข้อเท้าเคลื่อนที่ไปข้างหน้าเร็ว
//   - Heel Strike: แตะพื้น -> ความเร็วลดฮวบเกือบเป็นศูนย์
//   - Stance: เท้าติดพื้น -> ความเร็วนิ่งใกล้ศูนย์ต่อเนื่อง
//   - Toe Off: เท้าเริ่มเคลื่อนอีกครั้ง
//
// สำคัญ: ต้องกรองตำแหน่งด้วย low-pass (Butterworth zero-lag) ก่อน differentiate
// — central difference ขยาย noise ด้วย ~1/(2·dt) จึงยิ่ง sample rate สูงยิ่งพังถ้าไม่กรอง
// (ยืนยันแล้วด้วย synthetic + Gaussian noise ระดับ OptiTrack; เทสต์ไร้ noise อย่างเดียว
// ไม่พอที่จะ claim ว่า threshold "เสถียร")

import { filtfiltButterworth2, estimateSampleRateHz } from './butterworth.js';

export const DEFAULT_STANCE_VELOCITY_THRESHOLD_MPS = 0.15;
export const DEFAULT_STANCE_EXIT_THRESHOLD_MPS = 0.22; // hysteresis: ออก stance ยากกว่าเข้า
export const DEFAULT_MIN_STANCE_DURATION_S = 0.2;
export const DEFAULT_MIN_STRIDE_TIME_S = 0.6;
export const DEFAULT_MAX_STRIDE_TIME_S = 3.0;
export const DEFAULT_POSITION_CUTOFF_HZ = 6;
export const DEFAULT_MAX_QUIET_GAP_FRAMES = 3; // morphological closing: เชื่อม quiet ที่ขาดสั้น ๆ

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

// อนุพันธ์เชิงตัวเลข (central difference) จาก dt จริงต่อคู่เฟรม
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

/**
 * กรองตำแหน่งด้วย Butterworth 2nd-order zero-lag แล้วค่อยหาอนุพันธ์
 * — จุดเดียวที่แก้ noise amplification จาก differentiate
 */
export function computeFilteredVelocity(t, position, options = {}) {
  const cutoffHz = options.cutoffHz ?? DEFAULT_POSITION_CUTOFF_HZ;
  const sampleRateHz = options.sampleRateHz ?? estimateSampleRateHz(t);
  if (!Number.isFinite(sampleRateHz) || sampleRateHz <= 0) {
    throw new Error('หา sample rate จาก timestamp ไม่ได้ — ต้องส่ง sampleRateHz เอง');
  }

  const effectiveCutoff = Math.min(cutoffHz, sampleRateHz * 0.45);
  const smoothed = options.skipFilter
    ? position.slice()
    : filtfiltButterworth2(position, effectiveCutoff, sampleRateHz);

  return {
    smoothedPosition: smoothed,
    velocity: computeVelocity(t, smoothed),
    sampleRateHz,
    cutoffHz: effectiveCutoff,
  };
}

/**
 * Morphological closing บน boolean mask: เติม gap ที่สั้นกว่า maxGapFrames ระหว่าง quiet runs
 * (กันเฟรมเดียวที่ noise กระแทกเกิน threshold แล้วตัด stance เป็น 2 ท่อน)
 */
export function closeQuietGaps(quietMask, maxGapFrames = DEFAULT_MAX_QUIET_GAP_FRAMES) {
  const out = quietMask.slice();
  let i = 0;
  while (i < out.length) {
    while (i < out.length && out[i]) i += 1;
    if (i >= out.length) break;
    const gapStart = i;
    while (i < out.length && !out[i]) i += 1;
    const gapEnd = i; // exclusive
    const gapLen = gapEnd - gapStart;
    const hasQuietBefore = gapStart > 0 && out[gapStart - 1];
    const hasQuietAfter = gapEnd < out.length && out[gapEnd];
    if (hasQuietBefore && hasQuietAfter && gapLen > 0 && gapLen <= maxGapFrames) {
      for (let j = gapStart; j < gapEnd; j += 1) out[j] = true;
    }
  }
  return out;
}

/**
 * หาทุกช่วง "นิ่ง" (stance) — ใช้ hysteresis (เข้า/ออกคนละ threshold) + closing ช่องว่างสั้น
 * คืน [{hsIndex, toIndex, hsTimeS, toTimeS}]
 */
export function detectStanceIntervals(t, velocity, options = {}) {
  const enterThresholdMps = options.thresholdMps
    ?? options.enterThresholdMps
    ?? DEFAULT_STANCE_VELOCITY_THRESHOLD_MPS;
  const exitThresholdMps = options.exitThresholdMps
    ?? DEFAULT_STANCE_EXIT_THRESHOLD_MPS;
  const minStanceDurationS = options.minStanceDurationS ?? DEFAULT_MIN_STANCE_DURATION_S;
  const maxQuietGapFrames = options.maxQuietGapFrames ?? DEFAULT_MAX_QUIET_GAP_FRAMES;

  // สร้าง quiet mask ด้วย hysteresis: เริ่ม quiet เมื่อ |v|<=enter, จบเมื่อ |v|>exit
  const quietRaw = new Array(velocity.length).fill(false);
  let inQuiet = false;
  for (let i = 0; i < velocity.length; i += 1) {
    const v = velocity[i];
    if (v === null) {
      inQuiet = false;
      quietRaw[i] = false;
      continue;
    }
    const absV = Math.abs(v);
    if (!inQuiet && absV <= enterThresholdMps) {
      inQuiet = true;
    } else if (inQuiet && absV > exitThresholdMps) {
      inQuiet = false;
    }
    quietRaw[i] = inQuiet;
  }

  const quiet = closeQuietGaps(quietRaw, maxQuietGapFrames);

  const intervals = [];
  let runStart = null;
  for (let i = 0; i < quiet.length; i += 1) {
    if (quiet[i] && runStart === null) {
      runStart = i;
    }
    if (!quiet[i] && runStart !== null) {
      const runEnd = i - 1;
      if (t[runEnd] - t[runStart] >= minStanceDurationS) {
        intervals.push({ hsIndex: runStart, toIndex: runEnd, hsTimeS: t[runStart], toTimeS: t[runEnd] });
      }
      runStart = null;
    }
  }
  // run ที่ยังนิ่งอยู่ตอนข้อมูลจบไม่นับ — ไม่รู้ toe-off จริง

  return intervals;
}

// รวม stance interval ที่ต่อเนื่องกันเป็น cycle (HS[i] -> HS[i+1] ของขาเดียวกัน)
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

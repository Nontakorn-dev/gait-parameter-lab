// Butterworth low-pass + zero-lag filtfilt สำหรับ marker trajectories
//
// ใช้ก่อน differentiate (computeVelocity) — central difference ขยาย high-frequency noise
// ด้วย ~1/(2·dt) ดังนั้นยิ่ง sample rate สูงยิ่งพังถ้าไม่กรองก่อน (มาตรฐาน biomechanics:
// Winter / Woltring — กรองตำแหน่งก่อนหาอนุพันธ์; cutoff กาit kinematics มัก 6–10 Hz)

/**
 * ค่าสัมประสิทธิ์ Butterworth อันดับ 2 (bilinear transform)
 * @param {number} cutoffHz
 * @param {number} sampleRateHz
 * @returns {{ b: number[], a: number[] }} a[0] = 1 เสมอ
 */
export function butterworthLowpass2(cutoffHz, sampleRateHz) {
  if (!(cutoffHz > 0) || !(sampleRateHz > 0) || cutoffHz >= sampleRateHz / 2) {
    throw new Error(
      `Butterworth cutoff ต้องอยู่ใน (0, Nyquist): cutoffHz=${cutoffHz}, sampleRateHz=${sampleRateHz}`,
    );
  }

  const omega = Math.tan((Math.PI * cutoffHz) / sampleRateHz);
  const omega2 = omega * omega;
  const sqrt2 = Math.SQRT2;
  const norm = 1 + sqrt2 * omega + omega2;

  return {
    b: [omega2 / norm, (2 * omega2) / norm, omega2 / norm],
    a: [1, (2 * (omega2 - 1)) / norm, (1 - sqrt2 * omega + omega2) / norm],
  };
}

/** Direct-Form II Transposed — กรองทิศทางเดียว; คืน array ความยาวเท่า input */
export function lfilter(b, a, signal, zi = null) {
  const n = signal.length;
  const out = new Array(n);
  let z1 = zi?.[0] ?? 0;
  let z2 = zi?.[1] ?? 0;
  for (let i = 0; i < n; i += 1) {
    const x = signal[i];
    const y = b[0] * x + z1;
    z1 = b[1] * x - a[1] * y + z2;
    z2 = b[2] * x - a[2] * y;
    out[i] = y;
  }
  return out;
}

/** initial conditions สำหรับ step response คงที่ (ลด transient ขอบแบบ scipy lfilter_zi) */
export function lfilterZi(b, a, x0 = 1) {
  // steady-state สำหรับ input คงที่ x0 เมื่อ DC gain = 1 → y0 = x0
  const y0 = x0;
  const z2 = (b[2] - a[2]) * y0;
  const z1 = (b[1] - a[1]) * y0 + z2;
  return [z1, z2];
}

function reflectPad(signal, padLen) {
  if (padLen <= 0) return signal.slice();
  if (signal.length < 2) {
    return [...Array(padLen).fill(signal[0] ?? 0), ...signal, ...Array(padLen).fill(signal[0] ?? 0)];
  }
  const left = [];
  const right = [];
  for (let i = 0; i < padLen; i += 1) {
    const li = 1 + (i % (signal.length - 1));
    const ri = signal.length - 2 - (i % (signal.length - 1));
    left.push(2 * signal[0] - signal[li]);
    right.push(2 * signal[signal.length - 1] - signal[ri]);
  }
  left.reverse();
  return [...left, ...signal, ...right];
}

/**
 * Zero-lag filtfilt: forward + reverse บนช่วง contiguous ของค่า finite
 * ค่า null คงเป็น null (ไม่เติมข้าม gap)
 */
export function filtfiltButterworth2(signal, cutoffHz, sampleRateHz) {
  const { b, a } = butterworthLowpass2(cutoffHz, sampleRateHz);
  const n = signal.length;
  const out = new Array(n).fill(null);

  let i = 0;
  while (i < n) {
    while (i < n && !Number.isFinite(signal[i])) i += 1;
    if (i >= n) break;
    const start = i;
    while (i < n && Number.isFinite(signal[i])) i += 1;
    const end = i;
    const seg = signal.slice(start, end);
    if (seg.length < 4) {
      for (let k = start; k < end; k += 1) out[k] = signal[k];
      continue;
    }

    const padLen = Math.min(seg.length - 1, 9);
    const padded = reflectPad(seg, padLen);
    const ziFwd = lfilterZi(b, a, padded[0]);
    const forward = lfilter(b, a, padded, ziFwd);
    forward.reverse();
    const ziBwd = lfilterZi(b, a, forward[0]);
    const backward = lfilter(b, a, forward, ziBwd);
    backward.reverse();
    const trimmed = backward.slice(padLen, padLen + seg.length);
    for (let k = 0; k < trimmed.length; k += 1) out[start + k] = trimmed[k];
  }

  return out;
}

/** ประมาณ sample rate จาก median Δt */
export function estimateSampleRateHz(t) {
  const deltas = [];
  for (let i = 1; i < t.length; i += 1) {
    const d = t[i] - t[i - 1];
    if (d > 0) deltas.push(d);
  }
  if (!deltas.length) return null;
  deltas.sort((a, b) => a - b);
  const median = deltas[Math.floor(deltas.length / 2)];
  return median > 0 ? 1 / median : null;
}

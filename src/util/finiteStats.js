// min/max ที่ไม่ใช้ spread — Math.min(...bigArray) ทำ RangeError: Maximum call stack
// size exceeded ที่ราว ~1–1.2e5 element ในขณะที่ IMU trace cap = 300_000

export function minFinite(values) {
  let min = Infinity;
  for (const v of values) {
    if (Number.isFinite(v) && v < min) min = v;
  }
  return Number.isFinite(min) ? min : null;
}

export function maxFinite(values) {
  let max = -Infinity;
  for (const v of values) {
    if (Number.isFinite(v) && v > max) max = v;
  }
  return Number.isFinite(max) ? max : null;
}

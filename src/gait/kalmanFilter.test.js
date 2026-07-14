import test from 'node:test';
import assert from 'node:assert/strict';

import { KalmanFilter } from './kalmanFilter.js';
import { accelToAngle } from './signalUtils.js';

const G = 9.81;

// จำลอง shank swing จริง: sensor ที่ระยะ r จากแกนหมุน หมุนตามมุมจริง theta(t)
// accel = gravity projection + centripetal(ω²r) + tangential(αr) → ‖accel‖ >> 1g ตอน swing
function synthSwing(t) {
  const R_LEG = 0.4;
  const A = 35 * Math.PI / 180;
  const T = 0.7;
  const th = A * Math.sin(2 * Math.PI * t / T);
  const w = A * (2 * Math.PI / T) * Math.cos(2 * Math.PI * t / T);
  const al = -A * (2 * Math.PI / T) ** 2 * Math.sin(2 * Math.PI * t / T);
  const aMotX = R_LEG * (al * Math.cos(th) - w * w * Math.sin(th));
  const aMotY = R_LEG * (al * Math.sin(th) + w * w * Math.cos(th));
  const aVert = 1 + aMotY / G;
  const aHoriz = aMotX / G;
  const ay = -Math.cos(th) * aVert - Math.sin(th) * aHoriz;
  const az = -Math.sin(th) * aVert + Math.cos(th) * aHoriz;
  return {
    accelAngle: accelToAngle(ay, az),
    accelMagG: Math.sqrt(ay * ay + az * az),
    gyroDps: (w * 180) / Math.PI,
    trueDeg: (th * 180) / Math.PI,
  };
}

function runSwing({ gate }) {
  const kf = new KalmanFilter(0.01);
  let maxErr = 0;
  for (let t = 0; t < 3 * 0.7; t += 0.01) {
    const s = synthSwing(t);
    // gate=false → ส่ง magnitude=1 เสมอ = ปิด gating (พฤติกรรมเดิม)
    const est = kf.update(s.gyroDps, s.accelAngle, 0.01, gate ? s.accelMagG : 1.0);
    if (t > 0.7) maxErr = Math.max(maxErr, Math.abs(est - s.trueDeg));
  }
  return maxErr;
}

test('adaptive gating ลด error ตอน swing อย่างมีนัยสำคัญ', () => {
  const ungated = runSwing({ gate: false });
  const gated = runSwing({ gate: true });

  assert.ok(ungated > 30, `พฤติกรรมเดิมต้องเพี้ยนหนัก ได้ ${ungated.toFixed(1)}°`);
  assert.ok(gated < 6, `หลังแก้ต้อง < 6° ได้ ${gated.toFixed(1)}°`);
  assert.ok(gated < ungated / 5, 'gating ต้องดีกว่าเดิมอย่างน้อย 5 เท่า');
});

test('ตอนนิ่ง (‖accel‖=1g) ยังเชื่อ accel เต็ม → lock ที่มุมจาก accel', () => {
  const kf = new KalmanFilter(0.01);
  const target = 15;
  for (let i = 0; i < 500; i += 1) {
    kf.update(0, target, 0.01, 1.0);
  }
  assert.ok(Math.abs(kf.x[0] - target) < 0.5, `ควร lock ~${target}° ได้ ${kf.x[0].toFixed(2)}°`);
});

test('backward-compat: ไม่ส่ง accelMagnitude = เชื่อ accel เต็ม (เท่ากับส่ง 1.0)', () => {
  const a = new KalmanFilter(0.01);
  const b = new KalmanFilter(0.01);
  for (let i = 0; i < 50; i += 1) {
    const gyro = Math.sin(i * 0.2) * 100;
    const angle = Math.cos(i * 0.2) * 10;
    const withoutArg = a.update(gyro, angle, 0.01);
    const withOne = b.update(gyro, angle, 0.01, 1.0);
    assert.equal(withoutArg, withOne);
  }
});

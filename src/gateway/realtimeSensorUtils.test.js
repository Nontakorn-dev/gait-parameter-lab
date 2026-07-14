import test from 'node:test';
import assert from 'node:assert/strict';

import {
  applyAxisMap,
  normalizeRealtimeSensorSample,
} from './realtimeSensorUtils.js';

test('applyAxisMap เรียงแกนใหม่และกลับเครื่องหมายตาม [sourceIndex, sign]', () => {
  // map: gx = -source[2], gy = source[0], gz = source[1]; accel identity
  const axisMap = {
    R: { accel: [[0, 1], [1, 1], [2, 1]], gyro: [[2, -1], [0, 1], [1, 1]] },
  };
  const { accel, gyro } = applyAxisMap([10, 20, 30], [1, 2, 3], 'R', axisMap);

  assert.deepEqual(accel, [10, 20, 30]);
  assert.deepEqual(gyro, [-3, 1, 2]);
});

test('applyAxisMap fallback ไปใช้ R เมื่อ side ไม่รู้จัก', () => {
  const axisMap = { R: { accel: [[0, -1], [1, 1], [2, 1]], gyro: [[0, 1], [1, 1], [2, 1]] } };
  const { accel } = applyAxisMap([5, 6, 7], [0, 0, 0], null, axisMap);
  assert.equal(accel[0], -5);
});

function makePayload(side) {
  return {
    sensor_key: side === 'L' ? 'DernDee_L_Shank' : 'DernDee_R_Shank',
    side,
    sensor_mount: 'shank',
    raw_accel: [100, 200, 300],
    raw_gyro: [11, 22, 33],
    timestamp_ms: 1234,
  };
}

test('normalize (identity map ปัจจุบัน) ส่งค่าผ่านครบทั้งซ้ายและขวา', () => {
  for (const side of ['L', 'R']) {
    const out = normalizeRealtimeSensorSample(makePayload(side));
    assert.equal(out.side, side);
    assert.deepEqual(out.raw_accel, [100, 200, 300]);
    assert.deepEqual(out.raw_gyro, [11, 22, 33]);
    assert.equal(out.gx, 11);
    assert.equal(out.ay, 200);
  }
});

test('normalize เป็น idempotent — normalize ซ้ำไม่ remap ซ้ำ (กัน sign flip หักล้าง)', () => {
  for (const side of ['L', 'R']) {
    const once = normalizeRealtimeSensorSample(makePayload(side));
    assert.equal(once.axisRemapped, true);
    const twice = normalizeRealtimeSensorSample(once);
    assert.deepEqual(twice.raw_accel, once.raw_accel);
    assert.deepEqual(twice.raw_gyro, once.raw_gyro);
    assert.equal(twice.gx, once.gx);
  }
});

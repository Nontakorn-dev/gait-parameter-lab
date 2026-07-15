import test from 'node:test';
import assert from 'node:assert/strict';

import {
  applyAxisMap,
  normalizeRealtimeSensorSample,
} from './realtimeSensorUtils.js';

// applyAxisMap ถูกเรียกที่ transport decode boundary ที่เดียว (realtimeTransport.js)
// เทสต์เหล่านี้ล็อก: (ก) กลไก remap ถูกต้อง, (ข) normalize เป็น pass-through ไม่ remap ซ้ำ

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

function makePayload(side, accel = [100, 200, 300], gyro = [11, 22, 33]) {
  return {
    sensor_key: side === 'L' ? 'DernDee_L_Shank' : 'DernDee_R_Shank',
    side,
    sensor_mount: 'shank',
    raw_accel: accel,
    raw_gyro: gyro,
    timestamp_ms: 1234,
  };
}

test('normalize เป็น pass-through ล้วน — ไม่แตะแกน (remap ทำที่ transport แล้ว)', () => {
  for (const side of ['L', 'R']) {
    const out = normalizeRealtimeSensorSample(makePayload(side));
    assert.equal(out.side, side);
    assert.deepEqual(out.raw_accel, [100, 200, 300]);
    assert.deepEqual(out.raw_gyro, [11, 22, 33]);
    assert.equal(out.gx, 11);
    assert.equal(out.ay, 200);
  }
});

test('remap-once invariant: normalize ซ้ำหลัง boundary ไม่ flip ซ้ำ (แม้ map เป็น non-identity)', () => {
  // จำลอง transport boundary ด้วย flip map ที่ไม่ใช่ identity
  const flipMap = { L: { accel: [[0, 1], [1, 1], [2, 1]], gyro: [[0, -1], [1, 1], [2, 1]] } };
  const rawAccel = [100, 200, 300];
  const rawGyro = [11, 22, 33];
  const canonical = applyAxisMap(rawAccel, rawGyro, 'L', flipMap);
  assert.equal(canonical.gyro[0], -11, 'gx ควรถูก flip ที่ boundary');

  // ข้อมูลบนสายเป็น canonical แล้ว — normalize กี่ครั้งก็ต้องคงค่าเดิม
  const once = normalizeRealtimeSensorSample(makePayload('L', canonical.accel, canonical.gyro));
  const twice = normalizeRealtimeSensorSample(once);
  assert.deepEqual(twice.raw_gyro, canonical.gyro);
  assert.deepEqual(twice.raw_accel, canonical.accel);
  assert.equal(twice.gx, -11, 'normalize ต้องไม่ flip ซ้ำ');
});

test('concern#2: demo-style canonical sample (side R) ผ่าน normalize ไม่ถูก transform', () => {
  // demo bypass transport จึงไม่เคยถูก remap — normalize ต้องไม่แตะ แม้ AXIS_MAP.R จะ non-identity
  const out = normalizeRealtimeSensorSample(makePayload('R', [1, 2, 3], [4, 5, 6]));
  assert.deepEqual(out.raw_accel, [1, 2, 3]);
  assert.deepEqual(out.raw_gyro, [4, 5, 6]);
});

test('trace fields: pre-remap raw + seq + packet version ผ่าน normalize และรอด double-normalize', () => {
  const payload = {
    ...makePayload('R'),
    raw_accel_sensor: [111, 222, 333],
    raw_gyro_sensor: [11, 22, 33],
    packet_version: 1,
    seq: 42,
  };
  const once = normalizeRealtimeSensorSample(payload);
  assert.deepEqual(once.rawAccelSensor, [111, 222, 333]);
  assert.deepEqual(once.rawGyroSensor, [11, 22, 33]);
  assert.equal(once.packetVersion, 1);
  assert.equal(once.seq, 42);

  // normalize รอบสอง (BLE path เรียกซ้ำ) ต้องไม่ทำ field หาย
  const twice = normalizeRealtimeSensorSample(once);
  assert.deepEqual(twice.rawAccelSensor, [111, 222, 333]);
  assert.deepEqual(twice.rawGyroSensor, [11, 22, 33]);
  assert.equal(twice.packetVersion, 1);
  assert.equal(twice.seq, 42);
});

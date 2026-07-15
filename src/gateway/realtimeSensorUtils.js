function toFiniteNumber(value) {
  const numericValue = Number(value)
  return Number.isFinite(numericValue) ? numericValue : null
}

// Canonical sensor frame ที่ downstream (gaitProcessor / gaitCalibration) คาดหวัง:
//   gyro[0] (gx) = angular velocity ในระนาบ sagittal, เป็น + เมื่อแกว่งขาไปด้านหน้า
//   accel[1],[2] (ay, az) = ระนาบ sagittal สำหรับประมาณมุม shank
//
// applyAxisMap ถูกเรียก "ครั้งเดียว" ที่ transport decode boundary (realtimeTransport.js
// notificationHandler) ซึ่งเป็นที่เดียวที่มี raw sensor frame จริง หลังจุดนั้นข้อมูลเป็น
// canonical เสมอ (normalize เป็นเพียง pass-through ไม่ remap ซ้ำ) จึงไม่ต้องพึ่ง flag
// ข้าม serialization hop และ demo/remote-broadcast ก็ไม่ถูก transform ผิด.
//
// ค่า default เป็น identity (ถือว่าติดตั้งเซนเซอร์ตรงกรอบ canonical อยู่แล้ว ซึ่งดีที่สุด
// เพราะไม่มีจุดพลาด). ถ้าจำเป็นต้อง mirror สองขาจากกล่องเดียวกัน ให้เติมค่าต่อข้างจาก
// ผล swing test — แต่ละช่องคือ [sourceIndex, sign] เช่น gx ที่กลับด้านของขาซ้าย = [0, -1].
// สำคัญ: หลังแก้ค่านี้ ต้องทำ calibration ใหม่ เพราะ gyro bias เดิมอยู่คนละกรอบ.
const AXIS_MAP = {
  R: { accel: [[0, 1], [1, 1], [2, 1]], gyro: [[0, 1], [1, 1], [2, 1]] },
  L: { accel: [[0, 1], [1, 1], [2, 1]], gyro: [[0, 1], [1, 1], [2, 1]] },
}

export function applyAxisMap(accel, gyro, side, axisMap = AXIS_MAP) {
  const map = axisMap[side] || axisMap.R
  return {
    accel: map.accel.map(([index, sign]) => accel[index] * sign),
    gyro: map.gyro.map(([index, sign]) => gyro[index] * sign),
  }
}

// ให้ trace export บันทึกว่าใช้ axis map ตัวไหน (deep clone กัน mutate)
export function getActiveAxisMap() {
  return JSON.parse(JSON.stringify(AXIS_MAP))
}

function readArrayField(payload, ...keys) {
  for (const key of keys) {
    if (Array.isArray(payload?.[key])) {
      return payload[key].map(Number)
    }
  }
  return null
}

function inferSideFromText(value = '') {
  const normalizedValue = String(value || '').trim().toUpperCase()

  if (!normalizedValue) {
    return null
  }

  if (
    normalizedValue.includes('LEFT')
    || normalizedValue.includes('LEFT_SHANK')
    || normalizedValue.includes('_L_')
    || normalizedValue.endsWith('_L')
    || normalizedValue.startsWith('L_')
  ) {
    return 'L'
  }

  if (
    normalizedValue.includes('RIGHT')
    || normalizedValue.includes('RIGHT_SHANK')
    || normalizedValue.includes('_R_')
    || normalizedValue.endsWith('_R')
    || normalizedValue.startsWith('R_')
  ) {
    return 'R'
  }

  return null
}

export function getRealtimeSensorKey(payload = {}) {
  const sensorKey = String(
    payload.sensor_key
    || payload.sensorKey
    || payload.position_id
    || payload.positionId
    || payload.name
    || '',
  ).trim()

  return sensorKey || null
}

export function getRealtimeSensorSide(payload = {}) {
  if (payload.side === 'L' || payload.side === 'R') {
    return payload.side
  }

  return inferSideFromText(
    payload.position_id
    || payload.positionId
    || payload.sensor_key
    || payload.sensorKey
    || payload.name,
  )
}

export function getRealtimeSensorMount(payload = {}) {
  if (typeof payload.sensor_mount === 'string' && payload.sensor_mount.trim()) {
    return payload.sensor_mount.trim()
  }

  const text = String(
    payload.position_id
    || payload.positionId
    || payload.sensor_key
    || payload.sensorKey
    || payload.name
    || '',
  ).toUpperCase()

  if (text.includes('SHANK')) {
    return 'shank'
  }

  if (text.includes('THIGH')) {
    return 'thigh'
  }

  if (text.includes('PELVIS') || text.includes('CHEST')) {
    return 'pelvis'
  }

  return null
}

export function normalizeRealtimeSensorSample(payload) {
  const accel = Array.isArray(payload?.raw_accel)
    ? payload.raw_accel
    : [payload?.ax, payload?.ay, payload?.az]
  const gyro = Array.isArray(payload?.raw_gyro)
    ? payload.raw_gyro
    : [payload?.gx, payload?.gy, payload?.gz]

  const normalized = {
    sensorKey: getRealtimeSensorKey(payload),
    side: getRealtimeSensorSide(payload),
    sensorMount: getRealtimeSensorMount(payload),
    positionId: payload?.position_id || payload?.positionId || null,
    name: payload?.name || null,
    ax: toFiniteNumber(accel?.[0]),
    ay: toFiniteNumber(accel?.[1]),
    az: toFiniteNumber(accel?.[2]),
    gx: toFiniteNumber(gyro?.[0]),
    gy: toFiniteNumber(gyro?.[1]),
    gz: toFiniteNumber(gyro?.[2]),
    timestamp_ms: toFiniteNumber(payload?.timestamp_ms ?? payload?.timestampMs ?? payload?.timestamp),
  }

  if (!normalized.sensorKey) {
    return null
  }

  if ([normalized.ax, normalized.ay, normalized.az, normalized.gx, normalized.gy, normalized.gz].some((value) => value == null)) {
    return null
  }

  // Pass-through เท่านั้น: axis remap ทำแล้วที่ transport decode boundary (ดู applyAxisMap).
  // ข้อมูลที่เข้ามาที่นี่เป็น canonical frame แล้วเสมอ.
  // carry ฟิลด์ดิบก่อน remap + seq + firmware version ผ่านไปให้ trace recorder
  // (อ่านได้ทั้ง snake_case จาก transport และ camelCase จากรอบ normalize ก่อนหน้า)
  return {
    ...normalized,
    timestampMs: normalized.timestamp_ms,
    raw_accel: [normalized.ax, normalized.ay, normalized.az],
    raw_gyro: [normalized.gx, normalized.gy, normalized.gz],
    rawAccelSensor: readArrayField(payload, 'raw_accel_sensor', 'rawAccelSensor'),
    rawGyroSensor: readArrayField(payload, 'raw_gyro_sensor', 'rawGyroSensor'),
    firmwareVersion: toFiniteNumber(payload?.firmware_version ?? payload?.firmwareVersion),
    seq: toFiniteNumber(payload?.seq),
  }
}

export function isRealtimeShankSample(sample) {
  return getRealtimeSensorMount(sample) === 'shank'
}

export function getRealtimeSensorLabel(sample) {
  if (sample?.side === 'L') {
    return 'Left Shank'
  }

  if (sample?.side === 'R') {
    return 'Right Shank'
  }

  return sample?.name || sample?.sensorKey || 'Sensor'
}

export function getRealtimeSensorSideOrder(side) {
  if (side === 'L') {
    return 0
  }

  if (side === 'R') {
    return 1
  }

  return 2
}
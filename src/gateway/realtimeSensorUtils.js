function toFiniteNumber(value) {
  const numericValue = Number(value)
  return Number.isFinite(numericValue) ? numericValue : null
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

  return {
    ...normalized,
    timestampMs: normalized.timestamp_ms,
    raw_accel: [normalized.ax, normalized.ay, normalized.az],
    raw_gyro: [normalized.gx, normalized.gy, normalized.gz],
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
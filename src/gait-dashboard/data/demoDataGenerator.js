/**
 * Demo Data Generator
 *
 * Synthetic shank IMU for walking when no BLE sensor is connected.
 *
 * Sensor orientation:
 *   X -> right (medio-lateral)
 *   Y -> down  (along shank)
 *   Z -> forward (anterior)
 *
 * สำคัญ — ต้องครบพร้อมกัน:
 * 1) gx มี HS dip (ลบ) + swing peak (บวก) + TO dip — event detector ทำงาน
 * 2) gx ลบค่าเฉลี่ยต่อก้าว → ∫gx≈0, มุมปิดรอบ (ไม่ hard-reset ที่ขัดกับ Kalman)
 * 3) aHoriz ใน world ช่วงต้น–กลาง swing ที่ integrate ผ่าน GaitProcessor
 *    ได้ ~targetStrideLengthM (ของเก่ามีแค่ gravity → ได้ ~0.15 m)
 */

const SAMPLE_RATE = 100;
const ACCEL_SCALE = 4096;
const GYRO_SCALE = 16.4;
const G_MS2 = 9.81;
const DEFAULT_TARGET_STRIDE_M = 1.25;
const ANGLE_AT_HS_DEG = -15;
/** จบ aHoriz ก่อนปลาย swing เพื่อให้ post-peak-valley อยู่ตอน ‖accel‖≈1g */
const AHORIZ_ACTIVE_SWING_FRAC = 0.8;

/** Mulberry32 — เล็ก deterministic; เทสต์ต้องส่ง seed เพื่อไม่ให้ CI flaky */
export function createSeededRng(seed = 1) {
  let state = (Number(seed) >>> 0) || 1;
  return function next() {
    state = (state + 0x6D2B79F5) >>> 0;
    let t = state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/**
 * Generate multiple strides of realistic walking data.
 *
 * @param {Object} options
 * @param {number} options.numStrides   Number of complete strides (default 8)
 * @param {number} options.strideTime   Stride duration in seconds (default 1.1)
 * @param {number} options.peakAngVel   Peak angular velocity during swing (deg/s, default 350)
 * @param {number} options.hsAngVel     Heel strike angular velocity dip (deg/s, default -180)
 * @param {number} options.noiseLevel   Noise amplitude factor (default 1.0; ใช้ 0 ได้)
 * @param {number} [options.seed]      ถ้าใส่ → PRNG คงที่ (เทสต์/CI); ไม่ใส่ → Math.random (demo UI)
 * @param {number} [options.targetStrideLengthM] ระยะก้าวเป้าหมายของ aHoriz (default 1.25)
 * @returns {{ samples: Object[], timestamps: number[], angVelDeg: number[], targetStrideLengthM: number }}
 */
export function generateWalkingData(options = {}) {
  const numStrides = options.numStrides || 8;
  const strideTime = options.strideTime || 1.1;
  const peakAngVel = options.peakAngVel || 350;
  const hsAngVel = options.hsAngVel || -180;
  const noiseLevel = Number.isFinite(options.noiseLevel) ? options.noiseLevel : 1.0;
  const targetStrideLengthM = Number.isFinite(options.targetStrideLengthM)
    ? options.targetStrideLengthM
    : DEFAULT_TARGET_STRIDE_M;
  const stancePct = 0.62;
  const rnd = Number.isFinite(options.seed)
    ? createSeededRng(options.seed)
    : Math.random;

  const dt = 1.0 / SAMPLE_RATE;

  const samples = [];
  const timestamps = [];
  const angVelDeg = [];

  for (let s = 0; s < numStrides; s += 1) {
    const strideVariation = 1.0 + (rnd() - 0.5) * 0.06;
    const currentStrideTime = strideTime * strideVariation;
    const currentSamples = Math.round(currentStrideTime * SAMPLE_RATE);
    const swingDurationS = currentStrideTime * (1 - stancePct);
    const sampleDtS = currentStrideTime / Math.max(1, currentSamples);

    const gxStride = [];
    let gxSum = 0;
    for (let i = 0; i < currentSamples; i += 1) {
      const t = i / currentSamples;
      const gx = generateShankAngularVelocity(
        t, stancePct, peakAngVel, hsAngVel, noiseLevel, rnd,
      );
      gxStride.push(gx);
      gxSum += gx;
    }
    // ปิดรอบมุมต่อก้าว — ไม่ hard-reset ที่ทำให้ accel/Kalman คนละเฟส
    const gxMean = gxSum / currentSamples;
    for (let i = 0; i < currentSamples; i += 1) {
      gxStride[i] -= gxMean;
    }

    let shankAngle = ANGLE_AT_HS_DEG;
    for (let i = 0; i < currentSamples; i += 1) {
      const t = i / currentSamples;
      const globalTime = samples.length * dt;
      const gxDeg = gxStride[i];

      const { ax, ay, az } = generateAccelerometer(t, shankAngle, stancePct, noiseLevel, rnd, {
        targetStrideLengthM,
        swingDurationS,
      });

      samples.push({
        seq: samples.length,
        timestamp: globalTime,
        ax: Math.round(ax * ACCEL_SCALE),
        ay: Math.round(ay * ACCEL_SCALE),
        az: Math.round(az * ACCEL_SCALE),
        gx: Math.round(gxDeg * GYRO_SCALE),
        gy: Math.round((rnd() - 0.5) * 10 * noiseLevel * GYRO_SCALE),
        gz: Math.round((rnd() - 0.5) * 15 * noiseLevel * GYRO_SCALE),
      });
      timestamps.push(globalTime);
      angVelDeg.push(gxDeg);

      shankAngle += gxDeg * sampleDtS;
    }
  }

  return { samples, timestamps, angVelDeg, targetStrideLengthM };
}

function generateShankAngularVelocity(t, stancePct, peak, hsDip, noise, rnd = Math.random) {
  let gx = 0;

  if (t < 0.05) {
    const phase = t / 0.05;
    gx = hsDip * Math.exp(-phase * 3) * Math.cos(phase * Math.PI);
  } else if (t < 0.15) {
    const phase = (t - 0.05) / 0.10;
    gx = -20 + 60 * phase;
  } else if (t < stancePct * 0.6) {
    const phase = (t - 0.15) / (stancePct * 0.6 - 0.15);
    gx = 40 + 20 * Math.sin(phase * Math.PI);
  } else if (t < stancePct) {
    const phase = (t - stancePct * 0.6) / (stancePct - stancePct * 0.6);
    gx = 60 - 80 * Math.sin(phase * Math.PI * 0.7);
    // TO dip ให้ชัดพอ findLocalMinima (prominence ≥ 30)
    if (phase > 0.25 && phase < 0.85) {
      const u = (phase - 0.55) / 0.12;
      gx -= 55 * Math.exp(-(u * u));
    }
  } else {
    const swingPhase = (t - stancePct) / (1.0 - stancePct);
    gx = peak * Math.sin(swingPhase * Math.PI);
    if (swingPhase > 0.75) {
      const termPhase = (swingPhase - 0.75) / 0.25;
      gx -= (peak + Math.abs(hsDip)) * termPhase * termPhase;
    }
  }

  gx += (rnd() - 0.5) * 8 * noise;
  return gx;
}

/**
 * Accel ในหน่วย g: gravity ตาม shankAngle + aHoriz โลกในช่วง swing
 * aHoriz = A·sin(2π·sp) บนเศษส่วนต้นของ swing → v ขอบ ≈ 0 และ displacement ≈ target
 */
function generateAccelerometer(t, shankAngle, stancePct, noise, rnd = Math.random, opts = {}) {
  const thetaRad = shankAngle * Math.PI / 180;
  const c = Math.cos(thetaRad);
  const s = Math.sin(thetaRad);

  const targetS = opts.targetStrideLengthM ?? DEFAULT_TARGET_STRIDE_M;
  const swingT = opts.swingDurationS ?? 0.4;
  let aHorizMs2 = 0;
  if (t >= stancePct && targetS > 0 && swingT > 0) {
    const swingPhase = (t - stancePct) / (1.0 - stancePct);
    if (swingPhase <= AHORIZ_ACTIVE_SWING_FRAC) {
      const sp = swingPhase / AHORIZ_ACTIVE_SWING_FRAC;
      const tEff = swingT * AHORIZ_ACTIVE_SWING_FRAC;
      const A = (targetS * 2 * Math.PI) / (tEff * tEff);
      aHorizMs2 = A * Math.sin(2 * Math.PI * sp);
    }
  }

  const aVertG = 1;
  const aHorizG = aHorizMs2 / G_MS2;

  // อินเวอร์สของ VelocityIntegrator.sensorToWorld
  let ay = -aVertG * c - aHorizG * s;
  let az = aHorizG * c - aVertG * s;
  let ax = 0;

  if (t < 0.05) {
    const impactPhase = t / 0.05;
    const impact = 0.2 * Math.exp(-impactPhase * 5) * Math.sin(impactPhase * Math.PI * 4);
    ay += impact;
    az += impact * 0.25;
  }

  if (t > stancePct - 0.05 && t < stancePct + 0.05) {
    const toPushPhase = (t - (stancePct - 0.05)) / 0.10;
    ay += 0.1 * Math.sin(toPushPhase * Math.PI);
  }

  ax += (rnd() - 0.5) * 0.05 * noise;
  ay += (rnd() - 0.5) * 0.04 * noise;
  az += (rnd() - 0.5) * 0.04 * noise;

  return { ax, ay, az };
}

/**
 * Stream demo data at real-time rate.
 *
 * @param {function} onSample  Callback receiving each sample at ~100Hz
 * @param {Object} options   Generation options
 * @returns {{ stop: function }} Controller to stop streaming
 */
export function streamDemoData(onSample, options = {}) {
  const { samples } = generateWalkingData({ numStrides: 200, ...options });
  let index = 0;
  let running = true;

  const timer = setInterval(() => {
    if (!running) {
      return;
    }

    if (index >= samples.length) {
      index = 0;
    }

    onSample(samples[index]);
    index += 1;
  }, 10);

  return {
    stop() {
      running = false;
      clearInterval(timer);
    },
  };
}

/**
 * Demo Data Generator
 *
 * Generates realistic synthetic IMU data for shank-mounted sensor during walking.
 * Used when no real BLE sensor is connected.
 *
 * Sensor orientation:
 *   X -> right (medio-lateral)
 *   Y -> down  (along shank)
 *   Z -> forward (anterior)
 *
 * Normal walking parameters (adult):
 *   - Stride time: ~1.0-1.3 s
 *   - Cadence: ~100-120 steps/min
 *   - Shank angular velocity peak: ~300-400 deg/s (swing)
 *   - Stance phase: ~60% of gait cycle
 *   - Swing phase: ~40% of gait cycle
 */

const SAMPLE_RATE = 100;
const ACCEL_SCALE = 4096;
const GYRO_SCALE = 16.4;

/**
 * Generate multiple strides of realistic walking data.
 *
 * @param {Object} options
 * @param {number} options.numStrides   Number of complete strides (default 8)
 * @param {number} options.strideTime   Stride duration in seconds (default 1.1)
 * @param {number} options.peakAngVel   Peak angular velocity during swing (deg/s, default 350)
 * @param {number} options.hsAngVel     Heel strike angular velocity dip (deg/s, default -180)
 * @param {number} options.noiseLevel   Noise amplitude factor (default 1.0)
 * @returns {{ samples: Object[], timestamps: number[], angVelDeg: number[] }}
 */
export function generateWalkingData(options = {}) {
  const numStrides = options.numStrides || 8;
  const strideTime = options.strideTime || 1.1;
  const peakAngVel = options.peakAngVel || 350;
  const hsAngVel = options.hsAngVel || -180;
  const noiseLevel = options.noiseLevel || 1.0;
  const stancePct = 0.62;

  const samplesPerStride = Math.round(strideTime * SAMPLE_RATE);
  const dt = 1.0 / SAMPLE_RATE;

  const samples = [];
  const timestamps = [];
  const angVelDeg = [];

  for (let s = 0; s < numStrides; s++) {
    const strideVariation = 1.0 + (Math.random() - 0.5) * 0.06;
    const currentStrideTime = strideTime * strideVariation;
    const currentSamples = Math.round(currentStrideTime * SAMPLE_RATE);

    for (let i = 0; i < currentSamples; i++) {
      const t = i / currentSamples;
      const globalTime = samples.length * dt;

      const gxDeg = generateShankAngularVelocity(t, stancePct, peakAngVel, hsAngVel, noiseLevel);
      const shankAngle = generateShankAngle(t, stancePct);
      const { ax, ay, az } = generateAccelerometer(t, shankAngle, stancePct, noiseLevel);

      const sample = {
        seq: samples.length,
        timestamp: globalTime,
        ax: Math.round(ax * ACCEL_SCALE),
        ay: Math.round(ay * ACCEL_SCALE),
        az: Math.round(az * ACCEL_SCALE),
        gx: Math.round(gxDeg * GYRO_SCALE),
        gy: Math.round((Math.random() - 0.5) * 10 * noiseLevel * GYRO_SCALE),
        gz: Math.round((Math.random() - 0.5) * 15 * noiseLevel * GYRO_SCALE),
      };

      samples.push(sample);
      timestamps.push(globalTime);
      angVelDeg.push(gxDeg);
    }
  }

  return { samples, timestamps, angVelDeg };
}

function generateShankAngularVelocity(t, stancePct, peak, hsDip, noise) {
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
    if (phase > 0.3 && phase < 0.7) {
      gx -= 40 * Math.exp(-Math.pow((phase - 0.5) / 0.1, 2));
    }
  } else {
    const swingPhase = (t - stancePct) / (1.0 - stancePct);
    gx = peak * Math.sin(swingPhase * Math.PI);
    if (swingPhase > 0.75) {
      const termPhase = (swingPhase - 0.75) / 0.25;
      gx -= (peak + Math.abs(hsDip)) * termPhase * termPhase;
    }
  }

  gx += (Math.random() - 0.5) * 8 * noise;
  return gx;
}

function generateShankAngle(t, stancePct) {
  if (t < stancePct) {
    const phase = t / stancePct;
    return -15 + 35 * phase;
  }

  const phase = (t - stancePct) / (1.0 - stancePct);
  return 20 - 35 * (1 - Math.cos(phase * Math.PI)) / 2;
}

function generateAccelerometer(t, shankAngle, stancePct, noise) {
  const thetaRad = shankAngle * Math.PI / 180;

  let ay = -Math.cos(thetaRad);
  let az = -Math.sin(thetaRad);
  let ax = 0;

  if (t < 0.08) {
    const impactPhase = t / 0.08;
    const impact = 2.5 * Math.exp(-impactPhase * 5) * Math.sin(impactPhase * Math.PI * 8);
    ay += impact;
    az += impact * 0.4;
  }

  if (t > stancePct - 0.05 && t < stancePct + 0.05) {
    const toPushPhase = (t - (stancePct - 0.05)) / 0.10;
    ay += 0.3 * Math.sin(toPushPhase * Math.PI);
  }

  if (t > stancePct) {
    const swingPhase = (t - stancePct) / (1.0 - stancePct);
    az += 0.2 * Math.sin(swingPhase * Math.PI * 2);
  }

  ax += (Math.random() - 0.5) * 0.05 * noise;
  ay += (Math.random() - 0.5) * 0.04 * noise;
  az += (Math.random() - 0.5) * 0.04 * noise;

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
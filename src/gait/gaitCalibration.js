import { median, rawAccelToG, rawGyroToDps } from './signalUtils.js';

export const CALIBRATION_DURATION_MS = 3000;
export const CALIBRATION_MIN_SAMPLES = 250;
export const CALIBRATION_MAX_GYRO_STD_DPS = 8;
export const CALIBRATION_MIN_ACCEL_G = 0.88;
export const CALIBRATION_MAX_ACCEL_G = 1.12;
export const DEFAULT_SHANK_LENGTH_M = 0.42;
export const SHANK_LENGTH_FROM_HEIGHT_RATIO = 0.246;

function stdDev(values) {
  const numeric = values.filter(Number.isFinite);
  if (numeric.length < 2) {
    return 0;
  }

  const mean = numeric.reduce((sum, value) => sum + value, 0) / numeric.length;
  const variance = numeric.reduce((sum, value) => sum + (value - mean) ** 2, 0) / (numeric.length - 1);
  return Math.sqrt(variance);
}

function readRawAxes(sample) {
  const read = (key, arrayKey, index) => {
    if (Number.isFinite(sample[key])) {
      return sample[key];
    }
    if (Array.isArray(sample[arrayKey]) && Number.isFinite(sample[arrayKey][index])) {
      return sample[arrayKey][index];
    }
    return null;
  };

  return {
    ax: read('ax', 'raw_accel', 0),
    ay: read('ay', 'raw_accel', 1),
    az: read('az', 'raw_accel', 2),
    gx: read('gx', 'raw_gyro', 0),
    gy: read('gy', 'raw_gyro', 1),
    gz: read('gz', 'raw_gyro', 2),
  };
}

export function estimateShankLengthM({ heightCm, shankLengthCm } = {}) {
  if (Number.isFinite(shankLengthCm) && shankLengthCm > 0) {
    return shankLengthCm / 100;
  }

  if (Number.isFinite(heightCm) && heightCm > 0) {
    return SHANK_LENGTH_FROM_HEIGHT_RATIO * (heightCm / 100);
  }

  return DEFAULT_SHANK_LENGTH_M;
}

export function computeCalibrationProfile(samples = [], options = {}) {
  const {
    shankLengthM = DEFAULT_SHANK_LENGTH_M,
    minSamples = CALIBRATION_MIN_SAMPLES,
    maxGyroStdDps = CALIBRATION_MAX_GYRO_STD_DPS,
  } = options;

  if (samples.length < minSamples) {
    return {
      ok: false,
      reason: `Need at least ${minSamples} samples while standing still (got ${samples.length}).`,
      sampleCount: samples.length,
    };
  }

  const gxDps = [];
  const gyDps = [];
  const gzDps = [];
  const accelMagnitudesG = [];

  for (const sample of samples) {
    const axes = readRawAxes(sample);
    if ([axes.ax, axes.ay, axes.az, axes.gx, axes.gy, axes.gz].some((value) => value == null)) {
      continue;
    }

    gxDps.push(rawGyroToDps(axes.gx));
    gyDps.push(rawGyroToDps(axes.gy));
    gzDps.push(rawGyroToDps(axes.gz));

    const axG = rawAccelToG(axes.ax);
    const ayG = rawAccelToG(axes.ay);
    const azG = rawAccelToG(axes.az);
    accelMagnitudesG.push(Math.sqrt(axG * axG + ayG * ayG + azG * azG));
  }

  if (gxDps.length < minSamples) {
    return {
      ok: false,
      reason: 'Incomplete IMU samples during calibration.',
      sampleCount: gxDps.length,
    };
  }

  const gyroBiasDps = {
    gx: median(gxDps) ?? 0,
    gy: median(gyDps) ?? 0,
    gz: median(gzDps) ?? 0,
  };
  const gyroStdDps = {
    gx: stdDev(gxDps),
    gy: stdDev(gyDps),
    gz: stdDev(gzDps),
  };
  const accelMeanG = accelMagnitudesG.reduce((sum, value) => sum + value, 0) / accelMagnitudesG.length;
  const maxGyroStd = Math.max(gyroStdDps.gx, gyroStdDps.gy, gyroStdDps.gz);
  const stillEnough = maxGyroStd <= maxGyroStdDps;
  const gravityOk = accelMeanG >= CALIBRATION_MIN_ACCEL_G && accelMeanG <= CALIBRATION_MAX_ACCEL_G;

  if (!stillEnough) {
    return {
      ok: false,
      reason: `Stand still during calibration (gyro noise ${maxGyroStd.toFixed(1)} deg/s, max ${maxGyroStdDps}).`,
      sampleCount: gxDps.length,
      gyroBiasDps,
      gyroStdDps,
      accelMeanG,
    };
  }

  if (!gravityOk) {
    return {
      ok: false,
      reason: `Sensor orientation looks unstable (accel magnitude ${accelMeanG.toFixed(2)} g).`,
      sampleCount: gxDps.length,
      gyroBiasDps,
      gyroStdDps,
      accelMeanG,
    };
  }

  return {
    ok: true,
    gyroBiasDps,
    gyroStdDps,
    accelMeanG,
    shankLengthM,
    sampleCount: gxDps.length,
    quality: {
      maxGyroStdDps: maxGyroStd,
      accelMeanG,
      stillEnough,
      gravityOk,
    },
  };
}

import {
  CALIBRATION_DURATION_MS,
  computeCalibrationProfile,
} from './gaitCalibration.js';
import { isRealtimeShankSample } from '../gateway/realtimeSensorUtils.js';
import { saveGaitCalibrationCaptureResults } from './gaitCalibrationStore.js';

export const GAIT_CALIBRATION_WARMSTART_SAMPLES = 120;

export function resolveGaitCalibrationSensorKeys(buckets, preferredSensorKeys = []) {
  const bucketKeys = Array.from(buckets.keys());
  if (preferredSensorKeys.length) {
    return preferredSensorKeys;
  }

  return bucketKeys;
}

export function finishGaitCalibrationCapture(buckets, options = {}) {
  const {
    preferredSensorKeys = [],
    shankLengthM,
    persist = true,
  } = options;

  const sensorKeys = resolveGaitCalibrationSensorKeys(buckets, preferredSensorKeys);
  const results = sensorKeys.map((sensorKey) => {
    const samples = buckets.get(sensorKey) || [];
    const profile = computeCalibrationProfile(samples, { shankLengthM });
    return { sensorKey, samples, profile };
  });

  if (persist) {
    saveGaitCalibrationCaptureResults(results);
  }

  const successes = results.filter((item) => item.profile.ok);
  const failureReasons = results
    .filter((item) => !item.profile.ok)
    .map((item) => item.profile.reason)
    .filter(Boolean);

  const firstSuccess = successes[0] || null;
  const summary = firstSuccess
    ? `shank ${firstSuccess.profile.shankLengthM.toFixed(2)} m, gyro σ ${firstSuccess.profile.quality.maxGyroStdDps.toFixed(1)}°/s`
    : '';

  return {
    ok: successes.length > 0,
    successCount: successes.length,
    results,
    failureReasons,
    summary,
    message: successes.length > 0
      ? `Calibration complete for ${successes.length} sensor(s).`
      : (failureReasons.join(' ') || 'Calibration failed. Stand still on a flat surface and try again.'),
  };
}

export function createGaitCalibrationCaptureSession(callbacks = {}) {
  let active = false;
  let buckets = new Map();
  let progressTimer = null;
  let captureTimer = null;
  let shankLengthM = null;
  let preferredSensorKeys = [];

  const clearTimers = () => {
    if (progressTimer) {
      window.clearInterval(progressTimer);
      progressTimer = null;
    }
    if (captureTimer) {
      window.clearTimeout(captureTimer);
      captureTimer = null;
    }
  };

  const cancel = () => {
    clearTimers();
    active = false;
    buckets = new Map();
  };

  const start = (options = {}) => {
    cancel();
    active = true;
    buckets = new Map();
    shankLengthM = options.shankLengthM;
    preferredSensorKeys = Array.isArray(options.preferredSensorKeys)
      ? options.preferredSensorKeys
      : [];

    const startedAt = Date.now();
    callbacks.onProgress?.(0, 'Stand still and keep the sensor stable...');

    progressTimer = window.setInterval(() => {
      const elapsed = Date.now() - startedAt;
      const percent = (elapsed / CALIBRATION_DURATION_MS) * 100;
      const remaining = Math.max(0, Math.ceil((CALIBRATION_DURATION_MS - elapsed) / 1000));
      callbacks.onProgress?.(
        percent,
        `Collecting IMU samples... ${remaining}s remaining`,
      );
    }, 100);

    captureTimer = window.setTimeout(() => {
      clearTimers();
      active = false;

      const outcome = finishGaitCalibrationCapture(buckets, {
        preferredSensorKeys,
        shankLengthM,
        persist: options.persist !== false,
      });

      callbacks.onComplete?.(outcome);
      buckets = new Map();
    }, CALIBRATION_DURATION_MS);
  };

  const ingestSample = (sample) => {
    if (!active || !sample?.sensorKey) {
      return;
    }

    if (!isRealtimeShankSample(sample)) {
      return;
    }

    const bucket = buckets.get(sample.sensorKey) || [];
    bucket.push(sample);
    buckets.set(sample.sensorKey, bucket);
  };

  return {
    start,
    cancel,
    ingestSample,
    isActive: () => active,
  };
}

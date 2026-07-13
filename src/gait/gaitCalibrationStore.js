export const GAIT_CALIBRATION_PREFS_KEY = 'derndee:gait-calibration-prefs';
export const GAIT_CALIBRATION_PROFILES_KEY = 'derndee:gait-calibration-profiles';

function readJson(key, fallback) {
  try {
    const raw = window.localStorage.getItem(key);
    return raw ? JSON.parse(raw) : fallback;
  } catch {
    return fallback;
  }
}

function writeJson(key, value) {
  try {
    window.localStorage.setItem(key, JSON.stringify(value));
  } catch {
    // Ignore storage failures in private browsing.
  }
}

export function readGaitCalibrationPrefs() {
  return readJson(GAIT_CALIBRATION_PREFS_KEY, {});
}

export function writeGaitCalibrationPrefs(prefs = {}) {
  writeJson(GAIT_CALIBRATION_PREFS_KEY, prefs);
}

export function readGaitCalibrationProfiles() {
  const stored = readJson(GAIT_CALIBRATION_PROFILES_KEY, {});
  return stored.bySensorKey && typeof stored.bySensorKey === 'object'
    ? stored
    : { appliedAt: null, bySensorKey: {} };
}

export function writeGaitCalibrationProfiles(bySensorKey = {}) {
  writeJson(GAIT_CALIBRATION_PROFILES_KEY, {
    appliedAt: Date.now(),
    bySensorKey,
  });
}

export function getGaitCalibrationProfile(sensorKey) {
  if (!sensorKey) {
    return null;
  }

  const { bySensorKey } = readGaitCalibrationProfiles();
  return bySensorKey[sensorKey] || null;
}

export function hasGaitCalibrationProfile(sensorKey) {
  const profile = getGaitCalibrationProfile(sensorKey);
  return Boolean(profile?.ok);
}

export function saveGaitCalibrationCaptureResults(results = []) {
  const successful = results.filter((item) => item?.profile?.ok);
  if (!successful.length) {
    return { savedCount: 0 };
  }

  const existing = readGaitCalibrationProfiles();
  const bySensorKey = { ...existing.bySensorKey };

  for (const item of successful) {
    bySensorKey[item.sensorKey] = {
      ...item.profile,
      sensorKey: item.sensorKey,
      savedAt: Date.now(),
    };
  }

  writeGaitCalibrationProfiles(bySensorKey);
  return { savedCount: successful.length };
}

export function applyStoredGaitCalibration(processor, sensorKey) {
  const profile = getGaitCalibrationProfile(sensorKey);
  if (!profile?.ok || typeof processor?.applyCalibration !== 'function') {
    return false;
  }

  processor.applyCalibration(profile);
  return true;
}

import { GaitEventDetector } from './gaitEventDetector.js';
import { DEFAULT_SHANK_LENGTH_M } from './gaitCalibration.js';
import { KalmanFilter } from './kalmanFilter.js';
import { rawAccelToG, rawGyroToDps, accelToAngle, movingAverage } from './signalUtils.js';
import { VelocityIntegrator } from './velocityIntegrator.js';

const SAMPLE_RATE = 100;
const WINDOW_SECONDS = 12;
const WINDOW_SAMPLES = SAMPLE_RATE * WINDOW_SECONDS;
const G_MS2 = 9.81;
const EPOCH_THRESHOLD_MS = 946684800000;
const STANCE_ENTRY_ANGULAR_VELOCITY_ABS = 5;
const STEP_LENGTH_MIN_M = 0.15;
const STEP_LENGTH_MAX_M = 0.80;
const STRIDE_LENGTH_MAX_M = STEP_LENGTH_MAX_M * 2;
const STANCE_ENTRY_THRESHOLD_MIN = 3;
const STANCE_ENTRY_THRESHOLD_MAX = 12;
const STANCE_ENTRY_VALLEY_THRESHOLD_MAX = 40;
const SUSTAINED_QUIET_SECONDS = 0.12;
const SUSTAINED_QUIET_SAMPLES = Math.max(4, Math.round(SUSTAINED_QUIET_SECONDS * SAMPLE_RATE));
const POST_HS_QUIET_SEARCH_SECONDS = 0.40;
const MIN_POST_HS_OFFSET_SECONDS = 0.04;
const MIN_POST_HS_OFFSET_SAMPLES = Math.max(3, Math.round(MIN_POST_HS_OFFSET_SECONDS * SAMPLE_RATE));
const PATIENT_EVENT_DETECTOR_OPTIONS = {
  sampleRate: SAMPLE_RATE,
  maxStrideTime: 3.0,
  minStrideTime: 0.6,
  minHsSeparationSeconds: 0.55,
  toSearchStartPct: 0.20,
  toSearchEndPct: 0.80,
  hsProminence: 45,
  hsVelocityThreshold: -50,
  hsVelocityThresholdFloorAbs: 12,
  hsEnvelopeScale: 0.28,
  hsPreviousPeakScale: 0.28,
  hsPreviousHsScale: 0.60,
  hsAdaptiveHistorySize: 3,
  hsEnvelopeWindowSeconds: 0.8,
  minSwingAngularVelocityAbs: 12,
  usePreviousValidStanceFallback: true,
};

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function median(values) {
  if (!values.length) {
    return 0;
  }

  const sorted = [...values].sort((left, right) => left - right);
  const middle = Math.floor(sorted.length / 2);
  if (sorted.length % 2 === 0) {
    return (sorted[middle - 1] + sorted[middle]) / 2;
  }

  return sorted[middle];
}

function findGyroQuietWindow(smoothedAngVel, rangeStartIdx, rangeEndIdx, threshold) {
  if (!Number.isInteger(rangeStartIdx) || !Number.isInteger(rangeEndIdx) || rangeEndIdx < rangeStartIdx) {
    return null;
  }

  let runStartIdx = null;
  let bestWindow = null;

  for (let i = rangeStartIdx; i <= rangeEndIdx; i += 1) {
    if (Math.abs(smoothedAngVel[i]) <= threshold) {
      if (runStartIdx === null) {
        runStartIdx = i;
      }

      const runLength = i - runStartIdx + 1;
      if (
        runLength >= SUSTAINED_QUIET_SAMPLES
        && (!bestWindow || runLength > (bestWindow.endIdx - bestWindow.startIdx + 1))
      ) {
        bestWindow = { startIdx: runStartIdx, endIdx: i };
      }
      continue;
    }

    runStartIdx = null;
  }

  return bestWindow;
}

function findCyclePeakIndex(smoothedAngVel, cycle) {
  const cycleStartIdx = cycle?.hsStart?.index;
  const cycleEndIdx = cycle?.hsEnd?.index;

  if (!Number.isInteger(cycleStartIdx) || !Number.isInteger(cycleEndIdx) || cycleEndIdx <= cycleStartIdx) {
    return cycleStartIdx ?? 0;
  }

  let peakIdx = cycleStartIdx;
  let peakValue = -Infinity;
  for (let i = cycleStartIdx; i <= cycleEndIdx; i += 1) {
    if (smoothedAngVel[i] > peakValue) {
      peakValue = smoothedAngVel[i];
      peakIdx = i;
    }
  }

  return peakIdx;
}

function resolveSwingIntegrationStart(smoothedAngVel, cycle, adaptiveThreshold) {
  const hsStartIdx = cycle?.hsStart?.index;
  const hsEndIdx = cycle?.hsEnd?.index;

  if (!Number.isInteger(hsStartIdx)) {
    return 0;
  }

  const postHsSearchEndIdx = Number.isInteger(hsEndIdx)
    ? Math.min(hsEndIdx - 1, hsStartIdx + Math.round(POST_HS_QUIET_SEARCH_SECONDS * SAMPLE_RATE))
    : hsStartIdx + Math.round(POST_HS_QUIET_SEARCH_SECONDS * SAMPLE_RATE);
  const quietWindow = findGyroQuietWindow(
    smoothedAngVel,
    hsStartIdx,
    postHsSearchEndIdx,
    adaptiveThreshold,
  );

  if (quietWindow) {
    return Math.max(quietWindow.endIdx + 1, hsStartIdx + MIN_POST_HS_OFFSET_SAMPLES);
  }

  return hsStartIdx + MIN_POST_HS_OFFSET_SAMPLES;
}

function resolveAdaptiveStanceThreshold(smoothedAngVel, cycle, peakIdx) {
  const cycleStartIdx = cycle?.hsStart?.index;
  const cycleEndIdx = cycle?.hsEnd?.index;

  if (!Number.isInteger(cycleStartIdx) || !Number.isInteger(cycleEndIdx) || cycleEndIdx <= cycleStartIdx) {
    return STANCE_ENTRY_ANGULAR_VELOCITY_ABS;
  }

  const searchStartIdx = Math.min(cycleEndIdx, Math.max(cycleStartIdx + 1, peakIdx + 1));
  const searchEndIdx = Math.max(searchStartIdx, cycleEndIdx - 1);

  const postPeakAbsValues = [];
  for (let i = searchStartIdx; i <= searchEndIdx; i += 1) {
    if (Number.isFinite(smoothedAngVel[i])) {
      postPeakAbsValues.push(Math.abs(smoothedAngVel[i]));
    }
  }

  if (!postPeakAbsValues.length) {
    return STANCE_ENTRY_ANGULAR_VELOCITY_ABS;
  }

  const sortedAbsValues = [...postPeakAbsValues].sort((left, right) => left - right);
  const quietSampleCount = Math.max(3, Math.min(sortedAbsValues.length, Math.floor(sortedAbsValues.length * 0.2)));
  const quietestAbsValues = sortedAbsValues.slice(0, quietSampleCount);
  const noiseFloor = median(quietestAbsValues);
  const adaptiveThreshold = noiseFloor > 0
    ? clamp(noiseFloor + 2, STANCE_ENTRY_THRESHOLD_MIN, STANCE_ENTRY_THRESHOLD_MAX)
    : STANCE_ENTRY_ANGULAR_VELOCITY_ABS;

  return adaptiveThreshold;
}

function buildIntegrationWindow(cycleStartIdx, endIdx, timestamps, threshold, source) {
  return {
    startIdx: cycleStartIdx,
    endIdx,
    startTime: Number.isFinite(timestamps?.[cycleStartIdx]) ? timestamps[cycleStartIdx] : null,
    endTime: Number.isFinite(timestamps?.[endIdx]) ? timestamps[endIdx] : null,
    threshold,
    source,
  };
}

function findStepIntegrationWindow(smoothedAngVel, timestamps, cycle) {
  const cycleStartIdx = cycle?.hsStart?.index;
  const cycleEndIdx = cycle?.hsEnd?.index;

  if (!Number.isInteger(cycleStartIdx) || !Number.isInteger(cycleEndIdx) || cycleEndIdx <= cycleStartIdx) {
    const fallbackIdx = cycleEndIdx ?? cycleStartIdx ?? 0;
    return buildIntegrationWindow(
      cycleStartIdx ?? fallbackIdx,
      fallbackIdx,
      timestamps,
      STANCE_ENTRY_ANGULAR_VELOCITY_ABS,
      'fallback',
    );
  }

  const peakIdx = findCyclePeakIndex(smoothedAngVel, cycle);
  const peakValue = smoothedAngVel[peakIdx];
  const adaptiveThreshold = resolveAdaptiveStanceThreshold(smoothedAngVel, cycle, peakIdx);
  const integrationStartIdx = resolveSwingIntegrationStart(smoothedAngVel, cycle, adaptiveThreshold);
  const valleyThreshold = clamp(
    Math.max(adaptiveThreshold * 1.5, Math.abs(peakValue) * 0.15),
    adaptiveThreshold,
    STANCE_ENTRY_VALLEY_THRESHOLD_MAX,
  );

  const searchStartIdx = Math.min(
    cycleEndIdx,
    Math.max(integrationStartIdx + 1, cycleStartIdx + 1, peakIdx + 1),
  );

  let lastValleyIdx = null;
  for (let i = searchStartIdx + 1; i < cycleEndIdx; i += 1) {
    const prevAbs = Math.abs(smoothedAngVel[i - 1]);
    const currentAbs = Math.abs(smoothedAngVel[i]);
    const nextAbs = Math.abs(smoothedAngVel[i + 1]);

    if (
      currentAbs <= valleyThreshold
      && currentAbs <= prevAbs
      && currentAbs <= nextAbs
    ) {
      lastValleyIdx = i;
    }
  }

  if (Number.isInteger(lastValleyIdx) && lastValleyIdx > integrationStartIdx) {
    return buildIntegrationWindow(
      integrationStartIdx,
      lastValleyIdx,
      timestamps,
      adaptiveThreshold,
      'post-peak-valley',
    );
  }

  let quietRunStartIdx = null;
  let lastSustainedQuietStartIdx = null;
  for (let i = searchStartIdx; i <= cycleEndIdx; i += 1) {
    if (Math.abs(smoothedAngVel[i]) <= adaptiveThreshold) {
      if (quietRunStartIdx === null) {
        quietRunStartIdx = i;
      }

      if ((i - quietRunStartIdx + 1) >= SUSTAINED_QUIET_SAMPLES) {
        lastSustainedQuietStartIdx = quietRunStartIdx;
      }
    } else {
      quietRunStartIdx = null;
    }
  }

  if (Number.isInteger(lastSustainedQuietStartIdx) && lastSustainedQuietStartIdx > integrationStartIdx) {
    return buildIntegrationWindow(
      integrationStartIdx,
      lastSustainedQuietStartIdx,
      timestamps,
      adaptiveThreshold,
      'sustained-quiet-start',
    );
  }

  let lastThresholdIdx = null;
  for (let i = searchStartIdx; i < cycleEndIdx; i += 1) {
    const currentValue = smoothedAngVel[i];
    const nextValue = smoothedAngVel[i + 1];
    const currentAbs = Math.abs(currentValue);
    const nextAbs = Math.abs(nextValue);
    const withinThreshold = currentAbs <= adaptiveThreshold;
    const nextSupportsStanceEntry = nextAbs <= adaptiveThreshold || (currentValue >= 0 && nextValue <= 0);

    if (withinThreshold && nextSupportsStanceEntry) {
      lastThresholdIdx = i;
    }
  }

  if (Number.isInteger(lastThresholdIdx)) {
    return buildIntegrationWindow(
      integrationStartIdx,
      lastThresholdIdx,
      timestamps,
      adaptiveThreshold,
      'adaptive-stance-entry',
    );
  }

  let lastZeroCrossingIdx = null;
  for (let i = searchStartIdx; i < cycleEndIdx; i += 1) {
    if (smoothedAngVel[i] >= 0 && smoothedAngVel[i + 1] <= 0) {
      lastZeroCrossingIdx = i;
    }
  }

  if (Number.isInteger(lastZeroCrossingIdx)) {
    return buildIntegrationWindow(
      integrationStartIdx,
      lastZeroCrossingIdx,
      timestamps,
      adaptiveThreshold,
      'zero-crossing-fallback',
    );
  }

  let minAbsIdx = searchStartIdx;
  let minAbsValue = Infinity;
  for (let i = searchStartIdx; i <= cycleEndIdx; i += 1) {
    const absValue = Math.abs(smoothedAngVel[i]);
    if (absValue < minAbsValue) {
      minAbsValue = absValue;
      minAbsIdx = i;
    }
  }

  if (Number.isFinite(minAbsValue)) {
    return buildIntegrationWindow(
      integrationStartIdx,
      minAbsIdx,
      timestamps,
      adaptiveThreshold,
      'min-abs-fallback',
    );
  }

  return buildIntegrationWindow(
    integrationStartIdx,
    cycleEndIdx,
    timestamps,
    adaptiveThreshold,
    'hs-end-fallback',
  );
}

function normalizeTimestampField(sample) {
  if (Number.isFinite(sample.timestampMs)) {
    return { valueMs: sample.timestampMs, isAbsolute: sample.timestampMs >= EPOCH_THRESHOLD_MS };
  }

  if (Number.isFinite(sample.timestamp_ms)) {
    return { valueMs: sample.timestamp_ms, isAbsolute: sample.timestamp_ms >= EPOCH_THRESHOLD_MS };
  }

  if (Number.isFinite(sample.timestamp)) {
    const timestampValue = sample.timestamp;
    if (timestampValue >= EPOCH_THRESHOLD_MS) {
      return { valueMs: timestampValue, isAbsolute: true };
    }

    if (Math.abs(timestampValue) < 1e9) {
      return { valueMs: timestampValue * 1000, isAbsolute: false };
    }

    return { valueMs: timestampValue, isAbsolute: false };
  }

  return { valueMs: Date.now(), isAbsolute: true };
}

function getRawAxis(sample, key, arrayKey, arrayIndex) {
  if (Number.isFinite(sample[key])) {
    return sample[key];
  }

  if (Array.isArray(sample[arrayKey]) && Number.isFinite(sample[arrayKey][arrayIndex])) {
    return sample[arrayKey][arrayIndex];
  }

  return 0;
}

function readGyroDps(sample, gyroBiasDps) {
  return {
    gx: rawGyroToDps(getRawAxis(sample, 'gx', 'raw_gyro', 0)) - (gyroBiasDps?.gx ?? 0),
    gy: rawGyroToDps(getRawAxis(sample, 'gy', 'raw_gyro', 1)) - (gyroBiasDps?.gy ?? 0),
    gz: rawGyroToDps(getRawAxis(sample, 'gz', 'raw_gyro', 2)) - (gyroBiasDps?.gz ?? 0),
  };
}

export class GaitProcessor {
  constructor() {
    this.kalman = new KalmanFilter(1.0 / SAMPLE_RATE);
    this.eventDetector = new GaitEventDetector(PATIENT_EVENT_DETECTOR_OPTIONS);
    this.velocityIntegrator = new VelocityIntegrator({ sampleRate: SAMPLE_RATE });

    this.buffer = [];
    this.maxBuffer = WINDOW_SAMPLES;
    this.paramListeners = [];
    this.latestParams = null;
    this.totalStepCount = 0;
    this.totalStrideCount = 0;
    this.lastCountedCycleStartSampleId = null;
    this.previousStepLength = null;
    this.nextSampleId = 1;
    this.sessionStartTime = null;
    this.latestSampleTimestampMs = null;
    this.relativeTimestampOriginMs = null;
    this.absoluteTimestampOriginMs = null;
    this.lastSourceTimestampMs = null;
    this.gyroBiasDps = { gx: 0, gy: 0, gz: 0 };
    this.calibration = null;
    this.processedData = {
      timestamps: [],
      angularVelocity: [],
      shankAngle: [],
      events: [],
      cycles: [],
      integrationWindow: null,
    };
  }

  applyCalibration(profile = {}) {
    this.gyroBiasDps = {
      gx: profile.gyroBiasDps?.gx ?? 0,
      gy: profile.gyroBiasDps?.gy ?? 0,
      gz: profile.gyroBiasDps?.gz ?? 0,
    };
    this.calibration = {
      appliedAt: Date.now(),
      gyroBiasDps: { ...this.gyroBiasDps },
      gyroStdDps: profile.gyroStdDps ?? null,
      accelMeanG: profile.accelMeanG ?? null,
      shankLengthM: Number.isFinite(profile.shankLengthM) ? profile.shankLengthM : DEFAULT_SHANK_LENGTH_M,
      sampleCount: profile.sampleCount ?? null,
      quality: profile.quality ?? null,
    };
    this.kalman.reset();
  }

  getCalibration() {
    return this.calibration;
  }

  warmStartFromSamples(samples = []) {
    for (const sample of samples) {
      this.addSample(sample);
    }
  }

  onParams(callback) {
    this.paramListeners.push(callback);
  }

  addSample(sample) {
    const timestampMs = this.resolveTimestampMs(sample);
    const dtSeconds = this.getSampleIntervalSeconds(timestampMs);
    const gyro = readGyroDps(sample, this.gyroBiasDps);
    const gx = gyro.gx;
    const aYg = rawAccelToG(getRawAxis(sample, 'ay', 'raw_accel', 1));
    const aZg = rawAccelToG(getRawAxis(sample, 'az', 'raw_accel', 2));
    const accelAngle = accelToAngle(aYg, aZg);
    const shankAngle = this.kalman.update(gx, accelAngle, dtSeconds);

    if (!this.sessionStartTime) {
      this.sessionStartTime = timestampMs;
    }
    this.latestSampleTimestampMs = timestampMs;

    this.buffer.push({
      ...sample,
      sampleId: this.nextSampleId,
      timestampMs,
      shankAngle,
    });
    this.nextSampleId += 1;

    if (this.buffer.length > this.maxBuffer) {
      this.buffer.shift();
    }
  }

  analyze() {
    if (this.buffer.length < SAMPLE_RATE * 3) {
      return;
    }

    const samples = [...this.buffer];
    const sampleCount = samples.length;
    const angVelDeg = new Array(sampleCount);
    const shankAngle = new Array(sampleCount);
    const timestamps = new Array(sampleCount);
    const ayG = new Array(sampleCount);
    const azG = new Array(sampleCount);
    const firstTimestampMs = samples[0]?.timestampMs ?? null;

    for (let i = 0; i < sampleCount; i += 1) {
      const sample = samples[i];
      timestamps[i] = firstTimestampMs !== null
        ? Math.max(0, (sample.timestampMs - firstTimestampMs) / 1000)
        : i * (1.0 / SAMPLE_RATE);

      const gyro = readGyroDps(sample, this.gyroBiasDps);
      const gx = gyro.gx;
      const aYg = rawAccelToG(getRawAxis(sample, 'ay', 'raw_accel', 1));
      const aZg = rawAccelToG(getRawAxis(sample, 'az', 'raw_accel', 2));

      angVelDeg[i] = gx;
      ayG[i] = aYg;
      azG[i] = aZg;
      shankAngle[i] = Number.isFinite(sample.shankAngle) ? sample.shankAngle : 0;
    }

    const { events, cycles } = this.eventDetector.detect(angVelDeg, timestamps);
    const smoothedAngVel = movingAverage(angVelDeg, 5);
    const stepLengths = [];
    const strideLengths = [];
    const strideTimes = [];
    const stancePcts = [];
    const swingPcts = [];
    const peakAngles = [];
    const clearances = [];
    const integrationWindows = [];
    let rollingPreviousStepLength = this.previousStepLength;

    for (const cycle of cycles) {
      const integrationWindow = findStepIntegrationWindow(smoothedAngVel, timestamps, cycle);
      const metricStartIdx = integrationWindow.startIdx;
      const metricEndIdx = integrationWindow.endIdx;
      const segmentStartIdx = metricStartIdx;
      const segmentEndIdx = metricEndIdx;
      const localIntegrationStartIdx = 0;
      const localIntegrationEndIdx = metricEndIdx - metricStartIdx;

      strideTimes.push(cycle.strideTime);
      stancePcts.push(cycle.stancePct);
      swingPcts.push(cycle.swingPct);
      integrationWindows.push(integrationWindow);

      let peakAngle = -Infinity;
      for (let j = metricStartIdx; j <= metricEndIdx && j < sampleCount; j += 1) {
        peakAngle = Math.max(peakAngle, shankAngle[j]);
      }
      peakAngles.push(peakAngle);

      const cycleAy = [];
      const cycleAz = [];
      const cycleAngles = [];
      for (let j = segmentStartIdx; j <= segmentEndIdx && j < sampleCount; j += 1) {
        cycleAy.push(ayG[j] * G_MS2);
        cycleAz.push(azG[j] * G_MS2);
        cycleAngles.push(shankAngle[j]);
      }

      const { strideLength: integratedStepLength, clearance } = this.velocityIntegrator.computeStrideMetrics(
        cycleAy,
        cycleAz,
        cycleAngles,
        {
          integrationStartIdx: localIntegrationStartIdx,
          integrationEndIdx: localIntegrationEndIdx,
        },
      );

      const stepLength = Math.max(
        STEP_LENGTH_MIN_M,
        Math.min(STEP_LENGTH_MAX_M, integratedStepLength),
      );
      const strideLength = Number.isFinite(rollingPreviousStepLength)
        ? Math.max(
          STEP_LENGTH_MIN_M * 2,
          Math.min(STRIDE_LENGTH_MAX_M, rollingPreviousStepLength + stepLength),
        )
        : null;

      stepLengths.push(stepLength);
      strideLengths.push(strideLength);
      rollingPreviousStepLength = stepLength;
      clearances.push(Math.max(0, Math.min(0.3, clearance)));
    }

    if (cycles.length > 0) {
      this.previousStepLength = rollingPreviousStepLength;
    }

    if (cycles.length === 0) {
      this.processedData = {
        timestamps,
        angularVelocity: angVelDeg,
        shankAngle,
        events,
        cycles,
        integrationWindow: null,
      };
      this.paramListeners.forEach((callback) => callback({ params: null, processedData: this.processedData }));
      return;
    }

    const lastIdx = cycles.length - 1;
    const lastCycle = cycles[lastIdx];
    const lastIntegrationWindow = integrationWindows[lastIdx] ?? null;
    const stepLengthLast = stepLengths[lastIdx];
    const strideLengthLast = strideLengths[lastIdx];
    const strideTimeLast = strideTimes[lastIdx];
    const stancePctLast = stancePcts[lastIdx];
    const swingPctLast = swingPcts[lastIdx];
    const peakAngleLast = peakAngles[lastIdx];
    const clearanceLast = clearances[lastIdx];
    const cycleStartSample = samples[lastCycle.hsStart.index] ?? null;
    const cycleEndSample = samples[lastCycle.hsEnd.index] ?? null;
    const cycleStartSampleId = cycleStartSample?.sampleId ?? null;
    const cycleStartTimestampMs = cycleStartSample?.timestampMs ?? null;

    if (
      cycleStartSampleId !== null
      && cycleStartSampleId !== this.lastCountedCycleStartSampleId
    ) {
      // 1 gait cycle (HS→HS ขาเดียวกัน) = 1 stride = 2 steps
      this.totalStepCount += 2;
      this.lastCountedCycleStartSampleId = cycleStartSampleId;
    }

    this.totalStrideCount = Math.floor(this.totalStepCount / 2);

    const sessionDuration = this.sessionStartTime && this.latestSampleTimestampMs
      ? (this.latestSampleTimestampMs - this.sessionStartTime) / 1000
      : 0;

    const cadence = strideTimeLast > 0 ? (2 / strideTimeLast) * 60 : 0;
    const stepLength = stepLengthLast;
    const stepTime = strideTimeLast / 2;
    const walkingSpeed = strideTimeLast > 0
      ? (
        Number.isFinite(strideLengthLast)
          ? strideLengthLast / strideTimeLast
          : stepLengthLast / stepTime
      )
      : 0;
    const doubleSupportPct = Number.isFinite(stancePctLast)
      ? Math.max(0, 2 * stancePctLast - 100)
      : null;
    const doubleSupport = Number.isFinite(doubleSupportPct)
      ? strideTimeLast * doubleSupportPct / 100
      : null;

    this.latestParams = {
      strideLength: strideLengthLast,
      stepLength,
      clearance: clearanceLast,
      cadence,
      strideTime: strideTimeLast,
      stepTime,
      stanceTime: lastCycle.stanceTime,
      swingTime: lastCycle.swingTime,
      stancePct: stancePctLast,
      swingPct: swingPctLast,
      temporalSource: lastCycle.temporalSource ?? null,
      walkingSpeed,
      peakShankAngle: peakAngleLast,
      doubleSupport,
      stepCount: this.totalStepCount,
      strideCount: this.totalStrideCount,
      sessionDuration,
      cycleCount: 1,
      strideLengths: Number.isFinite(strideLengthLast) ? [strideLengthLast] : [],
      stepLengths: [stepLengthLast],
      strideTimes: [strideTimeLast],
      cycleStartSampleId,
      cycleStartTimestampMs,
      cycleEndTimestampMs: cycleEndSample?.timestampMs ?? null,
      integrationStartTimestampS: lastIntegrationWindow?.startTime ?? null,
      integrationEndTimestampS: lastIntegrationWindow?.endTime ?? null,
      integrationAngularVelocityThresholdDps: lastIntegrationWindow?.threshold ?? null,
      integrationSource: lastIntegrationWindow?.source ?? null,
      cycleKey: cycleStartSampleId !== null ? String(cycleStartSampleId) : `${cycleStartTimestampMs ?? Date.now()}`,
      side: cycleStartSample?.side ?? null,
      sensorName: cycleStartSample?.sensorName ?? null,
      sensorMount: cycleStartSample?.sensorMount ?? null,
      calibrated: Boolean(this.calibration),
      shankLengthM: this.calibration?.shankLengthM ?? DEFAULT_SHANK_LENGTH_M,
    };

    const lastStart = lastCycle.hsStart.index;
    const lastEnd = lastCycle.hsEnd.index;
    const filteredEvents = events.filter((event) => event.index >= lastStart && event.index <= lastEnd);
    const filteredCycles = [lastCycle];

    this.processedData = {
      timestamps,
      angularVelocity: angVelDeg,
      shankAngle,
      smoothedAngularVelocity: smoothedAngVel,
      events: filteredEvents,
      cycles: filteredCycles,
      integrationWindow: lastIntegrationWindow,
    };

    this.paramListeners.forEach((callback) => callback({
      params: this.latestParams,
      processedData: this.processedData,
    }));
  }

  reset() {
    this.buffer = [];
    this.kalman.reset();
    this.latestParams = null;
    this.totalStepCount = 0;
    this.totalStrideCount = 0;
    this.lastCountedCycleStartSampleId = null;
    this.previousStepLength = null;
    this.nextSampleId = 1;
    this.sessionStartTime = null;
    this.latestSampleTimestampMs = null;
    this.relativeTimestampOriginMs = null;
    this.absoluteTimestampOriginMs = null;
    this.lastSourceTimestampMs = null;
    const preservedCalibration = this.calibration;
    const preservedBias = { ...this.gyroBiasDps };
    this.gyroBiasDps = preservedBias;
    this.calibration = preservedCalibration;
    this.processedData = {
      timestamps: [],
      angularVelocity: [],
      shankAngle: [],
      events: [],
      cycles: [],
      integrationWindow: null,
    };
  }

  resolveTimestampMs(sample) {
    const { valueMs, isAbsolute } = normalizeTimestampField(sample);
    if (isAbsolute) {
      this.lastSourceTimestampMs = valueMs;
      return valueMs;
    }

    if (
      this.relativeTimestampOriginMs === null
      || this.absoluteTimestampOriginMs === null
      || (this.lastSourceTimestampMs !== null && valueMs < this.lastSourceTimestampMs - 1000)
    ) {
      this.relativeTimestampOriginMs = valueMs;
      this.absoluteTimestampOriginMs = Date.now();
    }

    this.lastSourceTimestampMs = valueMs;
    return this.absoluteTimestampOriginMs + (valueMs - this.relativeTimestampOriginMs);
  }

  getSampleIntervalSeconds(timestampMs) {
    if (!Number.isFinite(timestampMs) || !Number.isFinite(this.latestSampleTimestampMs)) {
      return 1.0 / SAMPLE_RATE;
    }

    const deltaSeconds = (timestampMs - this.latestSampleTimestampMs) / 1000;
    if (deltaSeconds <= 0 || deltaSeconds > 0.5) {
      return 1.0 / SAMPLE_RATE;
    }

    return deltaSeconds;
  }
}
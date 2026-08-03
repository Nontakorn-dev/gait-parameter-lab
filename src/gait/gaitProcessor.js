import { GaitEventDetector } from './gaitEventDetector.js';
import { DEFAULT_SHANK_LENGTH_M } from './gaitCalibration.js';
import { KalmanFilter } from './kalmanFilter.js';
import { MadgwickFilter } from './madgwickFilter.js';
import { rawAccelToG, rawGyroToDps, accelToAngle, movingAverage } from './signalUtils.js';
import { VelocityIntegrator } from './velocityIntegrator.js';
import { makeClosedCycleKey } from './cycleCoverage.js';

const SAMPLE_RATE = 100;
const WINDOW_SECONDS = 12;
const WINDOW_SAMPLES = SAMPLE_RATE * WINDOW_SECONDS;
const G_MS2 = 9.81;
const EPOCH_THRESHOLD_MS = 946684800000;
const STANCE_ENTRY_ANGULAR_VELOCITY_ABS = 5;
// การ integrate ความเร่งตลอด HS→HS ของขาเดียวกัน = ระยะ 1 stride โดยตรง
// floor ต่ำ (0.10) เพื่อไม่ทำลายข้อมูลผู้ป่วย stroke ที่ stride สั้นกว่า 0.30m ได้จริง
const STRIDE_LENGTH_MIN_M = 0.10;
const STRIDE_LENGTH_MAX_M = 1.80;
/** |v_end| ก่อน de-drift สูงเกินนี้ = gravity leakage / ZUPT พัง → ไม่เชื่อระยะ */
export const MAX_V_END_PRE_DRIFT_MPS = 2.0;
/** strideTime > ratio × median(recent) → สงสัย missed HS (maxStrideTime 3.5s กลืนเคสนี้) */
const SUSPECTED_MISSED_HS_STRIDE_TIME_RATIO = 1.8;
/** absolute: ผู้ใหญ่เดินปกติ stride > 2.5s น่าสงสัยทันที (ไม่รอ history ≥ 2) */
const SUSPECTED_MISSED_HS_ABS_STRIDE_TIME_S = 2.5;
const RECENT_STRIDE_TIME_HISTORY = 8;
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
  maxStrideTime: 3.5,
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
    return { valueMs: sample.timestampMs, isAbsolute: sample.timestampMs >= EPOCH_THRESHOLD_MS, missing: false };
  }

  if (Number.isFinite(sample.timestamp_ms)) {
    return { valueMs: sample.timestamp_ms, isAbsolute: sample.timestamp_ms >= EPOCH_THRESHOLD_MS, missing: false };
  }

  if (Number.isFinite(sample.timestamp)) {
    const timestampValue = sample.timestamp;
    if (timestampValue >= EPOCH_THRESHOLD_MS) {
      return { valueMs: timestampValue, isAbsolute: true, missing: false };
    }

    if (Math.abs(timestampValue) < 1e9) {
      return { valueMs: timestampValue * 1000, isAbsolute: false, missing: false };
    }

    return { valueMs: timestampValue, isAbsolute: false, missing: false };
  }

  // ห้าม Date.now() — ทำให้ sync กับ MoCap ไม่ reproducible (เสีย validation ทั้งชุด)
  return { valueMs: null, isAbsolute: false, missing: true };
}

function getRawAxis(sample, key, arrayKey, arrayIndex) {
  if (Number.isFinite(sample[key])) {
    return sample[key];
  }

  if (Array.isArray(sample[arrayKey]) && Number.isFinite(sample[arrayKey][arrayIndex])) {
    return sample[arrayKey][arrayIndex];
  }

  return null;
}

function readGyroDps(sample, gyroBiasDps) {
  const gxRaw = getRawAxis(sample, 'gx', 'raw_gyro', 0);
  const gyRaw = getRawAxis(sample, 'gy', 'raw_gyro', 1);
  const gzRaw = getRawAxis(sample, 'gz', 'raw_gyro', 2);
  return {
    gx: Number.isFinite(gxRaw) ? rawGyroToDps(gxRaw) - (gyroBiasDps?.gx ?? 0) : NaN,
    gy: Number.isFinite(gyRaw) ? rawGyroToDps(gyRaw) - (gyroBiasDps?.gy ?? 0) : NaN,
    gz: Number.isFinite(gzRaw) ? rawGyroToDps(gzRaw) - (gyroBiasDps?.gz ?? 0) : NaN,
  };
}

export class GaitProcessor {
  /**
   * @param {Object} [options]
   * @param {'madgwick'|'kalman'} [options.orientationFilter='kalman']
   */
  constructor(options = {}) {
    this.orientationFilterMode = options.orientationFilter === 'madgwick' ? 'madgwick' : 'kalman';
    this.kalman = new KalmanFilter(1.0 / SAMPLE_RATE);
    this.madgwick = new MadgwickFilter({ samplePeriod: 1.0 / SAMPLE_RATE, beta: 0.08 });
    this.eventDetector = new GaitEventDetector(PATIENT_EVENT_DETECTOR_OPTIONS);
    this.velocityIntegrator = new VelocityIntegrator({ sampleRate: SAMPLE_RATE });

    this.buffer = [];
    this.maxBuffer = WINDOW_SAMPLES;
    this.paramListeners = [];
    this.latestParams = null;
    this.totalStepCount = 0;
    this.totalStrideCount = 0;
    // closed cycles ที่นับแล้ว — เก็บ [startId, endId] เพื่อกันทับซ้อน (ไม่ใช่แค่ hsStart)
    this.countedCycleIntervals = [];
    // open stride ที่ emit แล้ว — Set ของ sampleId ตัวเลข (ห้ามปน 'open:*' ใน interval prune)
    this.openStrideStartIds = new Set();
    this.recentClosedStrideTimesS = [];
    // ZUPT diagnostic ต่อ cycle (ไม่ใช่แค่ cycle ล่าสุด) รอให้ analyze() drain ไปแนบกับ
    // payload ที่ส่งออกทาง onParams — ตรงนี้คือจุดเดียวที่ trace (ผ่าน app.js) จะได้ค่านี้จริง
    this.pendingCycleDiagnostics = [];
    this.nextSampleId = 1;
    this.sessionStartTime = null;
    this.latestSampleTimestampMs = null;
    this.relativeTimestampOriginMs = null;
    // สะสม ms ข้าม micros()/uint32 wrap — ห้าม reset เป็น 0 กลาง session
    this.accumulatedRelativeMs = 0;
    this.lastRelativeDeltaMs = 1000 / SAMPLE_RATE;
    // ไม่ใช้เป็นฐานเวลาอีกแล้ว — เดิมผูก Date.now() ทำให้ cycleStartTimestampMs
    // ไม่ reproducible ตอน reprocess relative timestamps จาก firmware
    this.absoluteTimestampOriginMs = null;
    this.lastSourceTimestampMs = null;
    this.gyroBiasDps = { gx: 0, gy: 0, gz: 0 };
    this.calibration = null;
    this.usedSyntheticTimestamps = false;
    this.missingTimestampCount = 0;
    this.skippedIncompleteSampleCount = 0;
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
    this.madgwick.reset();
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
    const axRaw = getRawAxis(sample, 'ax', 'raw_accel', 0);
    const ayRaw = getRawAxis(sample, 'ay', 'raw_accel', 1);
    const azRaw = getRawAxis(sample, 'az', 'raw_accel', 2);
    const gyro = readGyroDps(sample, this.gyroBiasDps);
    // แกนหาย → ห้ามแทน 0 (จะหลอกว่านิ่ง/ไม่มีหมุน → HS/ZUPT ผิด)
    // Madgwick ใช้ gyro 3 แกน; ถ้า gy/gz หาย เติม 0 ได้เฉพาะเมื่อโหมด kalman 1D
    if (![axRaw, ayRaw, azRaw, gyro.gx].every(Number.isFinite)) {
      this.skippedIncompleteSampleCount += 1;
      return;
    }

    const timestampMs = this.resolveTimestampMs(sample);
    if (!Number.isFinite(timestampMs)) {
      this.skippedIncompleteSampleCount += 1;
      return;
    }

    const dtSeconds = this.getSampleIntervalSeconds(timestampMs);
    const gx = gyro.gx;
    const gy = Number.isFinite(gyro.gy) ? gyro.gy : 0;
    const gz = Number.isFinite(gyro.gz) ? gyro.gz : 0;
    const aXg = rawAccelToG(axRaw);
    const aYg = rawAccelToG(ayRaw);
    const aZg = rawAccelToG(azRaw);
    const accelAngle = accelToAngle(aYg, aZg);
    const accelMagnitudeG = Math.sqrt(aXg * aXg + aYg * aYg + aZg * aZg);

    let shankAngle;
    let orientation = null;
    if (this.orientationFilterMode === 'madgwick') {
      orientation = this.madgwick.update(gx, gy, gz, aXg, aYg, aZg, dtSeconds);
      shankAngle = this.madgwick.getSagittalAngleDeg();
    } else {
      shankAngle = this.kalman.update(gx, accelAngle, dtSeconds, accelMagnitudeG);
    }

    if (!this.sessionStartTime) {
      this.sessionStartTime = timestampMs;
    }
    this.latestSampleTimestampMs = timestampMs;

    this.buffer.push({
      ...sample,
      sampleId: this.nextSampleId,
      timestampMs,
      shankAngle,
      orientation,
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
    const axG = new Array(sampleCount);
    const ayG = new Array(sampleCount);
    const azG = new Array(sampleCount);
    const orientations = new Array(sampleCount);
    const firstTimestampMs = samples[0]?.timestampMs ?? null;

    for (let i = 0; i < sampleCount; i += 1) {
      const sample = samples[i];
      timestamps[i] = firstTimestampMs !== null
        ? Math.max(0, (sample.timestampMs - firstTimestampMs) / 1000)
        : i * (1.0 / SAMPLE_RATE);

      const gyro = readGyroDps(sample, this.gyroBiasDps);
      const gx = gyro.gx;
      const aXg = rawAccelToG(getRawAxis(sample, 'ax', 'raw_accel', 0));
      const aYg = rawAccelToG(getRawAxis(sample, 'ay', 'raw_accel', 1));
      const aZg = rawAccelToG(getRawAxis(sample, 'az', 'raw_accel', 2));

      angVelDeg[i] = gx;
      axG[i] = aXg;
      ayG[i] = aYg;
      azG[i] = aZg;
      shankAngle[i] = Number.isFinite(sample.shankAngle) ? sample.shankAngle : 0;
      orientations[i] = sample.orientation || null;
    }

    const { events, cycles } = this.eventDetector.detect(angVelDeg, timestamps);
    const smoothedAngVel = movingAverage(angVelDeg, 5);
    const strideLengths = [];
    const strideTimes = [];
    const stancePcts = [];
    const swingPcts = [];
    const peakAngles = [];
    const clearances = [];
    const integrationWindows = [];
    const strideClampedFlags = [];
    const strideSignedLengths = [];
    const zuptAccelDeviations = [];
    const strideUntrustedFlags = [];
    const vEndPreDriftByCycle = [];
    const vStartPreDriftByCycle = [];
    const processedCycles = [];

    for (const cycle of cycles) {
      // นับทุก cycle ที่ยังไม่เคยนับ (ไม่ใช่แค่ cycle สุดท้าย) เพื่อไม่ให้พลาด
      // 1 HS→HS ของขาที่ติดเซนเซอร์ = 1 stride = 1 footfall ของขานั้น
      // ห้าม ×2 สมมติขาตรงข้าม — เซนเซอร์ข้างเดียว / เดินข้างเดียวจะฟ้องเป็น 2 ก้าวผิด
      // (step รวมสองข้างทำตอน aggregate เมื่อมี L+R จริง)
      const countableCycleStartSampleId = samples[cycle.hsStart.index]?.sampleId ?? null;
      const countableCycleEndSampleId = samples[cycle.hsEnd.index]?.sampleId ?? null;
      const isOpenStride = Boolean(cycle.isOpenStride);

      // สงสัย missed HS: absolute ก่อน (ไม่พึ่ง history) แล้วค่อย ratio กับ median ล่าสุด
      let suspectedMissedHs = false;
      if (!isOpenStride && Number.isFinite(cycle.strideTime)) {
        if (cycle.strideTime > SUSPECTED_MISSED_HS_ABS_STRIDE_TIME_S) {
          suspectedMissedHs = true;
        } else if (this.recentClosedStrideTimesS.length >= 2) {
          const sorted = [...this.recentClosedStrideTimesS].sort((a, b) => a - b);
          const med = sorted[Math.floor(sorted.length / 2)];
          if (Number.isFinite(med) && med > 0 && cycle.strideTime > SUSPECTED_MISSED_HS_STRIDE_TIME_RATIO * med) {
            suspectedMissedHs = true;
          }
        }
      }

      // open ห้ามเข้า counted intervals — ไม่งั้น HS ตัวสุดท้ายที่เคยเป็น open จะบล็อก closed HS→HS ทีหลัง
      let isNewCycle = false;
      if (
        !isOpenStride
        && countableCycleStartSampleId !== null
        && countableCycleEndSampleId !== null
      ) {
        const overlaps = this.countedCycleIntervals.filter((c) => (
          countableCycleStartSampleId < c.endId && c.startId < countableCycleEndSampleId
        ));
        if (!overlaps.length) {
          isNewCycle = true;
        } else {
          // ทับซ้อน: ถ้าอันใหม่สั้นกว่าและอยู่ในช่วงอันเก่า → แทนที่ (แยก double-stride ที่พลาด HS)
          const newLen = countableCycleEndSampleId - countableCycleStartSampleId;
          const canReplace = overlaps.every((o) => {
            const oldLen = o.endId - o.startId;
            return newLen < oldLen
              && countableCycleStartSampleId >= o.startId
              && countableCycleEndSampleId <= o.endId;
          });
          if (canReplace) {
            for (const o of overlaps) {
              const idx = this.countedCycleIntervals.indexOf(o);
              if (idx >= 0) this.countedCycleIntervals.splice(idx, 1);
              this.totalStrideCount = Math.max(0, this.totalStrideCount - 1);
              this.totalStepCount = Math.max(0, this.totalStepCount - 1);
              // บอกผู้ฟังให้ถอน cycle เก่าที่ถูกแทนที่ (reprocess / dashboard)
              // key ต้องเป็น start-end ของอันเก่า — ห้ามใช้แค่ startId (จะชี้ตัวเองเมื่อ end เปลี่ยน)
              this.pendingCycleDiagnostics.push({
                cycleKey: makeClosedCycleKey(o.startId, o.endId),
                retracted: true,
                supersededByCycleKey: makeClosedCycleKey(
                  countableCycleStartSampleId,
                  countableCycleEndSampleId,
                ),
              });
            }
            isNewCycle = true;
          }
          // ไม่เช่นนั้นปฏิเสธ (เช่น long cycle มาทีหลังเมื่อมี short อยู่แล้ว)
        }
      }
      if (isNewCycle) {
        this.countedCycleIntervals.push({
          startId: countableCycleStartSampleId,
          endId: countableCycleEndSampleId,
        });
        this.totalStrideCount += 1;
        this.totalStepCount += 1;
        if (Number.isFinite(cycle.strideTime)) {
          this.recentClosedStrideTimesS.push(cycle.strideTime);
          if (this.recentClosedStrideTimesS.length > RECENT_STRIDE_TIME_HISTORY) {
            this.recentClosedStrideTimesS.shift();
          }
        }
      }

      // open stride ยังต้อง emit diagnostic (ธงแยก) แต่ใช้ key คนละแบบกันชน closed
      const openDiagKey = isOpenStride && countableCycleStartSampleId !== null
        ? `open:${countableCycleStartSampleId}`
        : null;
      const shouldEmitDiagnostic = isNewCycle
        || (isOpenStride && openDiagKey && !this.openStrideStartIds.has(countableCycleStartSampleId));
      if (isOpenStride && openDiagKey && shouldEmitDiagnostic) {
        this.openStrideStartIds.add(countableCycleStartSampleId);
      }

      // ไม่ emit ใหม่ (ทับซ้อนที่ปฏิเสธ / open ซ้ำ) — ไม่วัดซ้ำ ไม่เป็น latestParams
      if (!shouldEmitDiagnostic) {
        continue;
      }
      processedCycles.push(cycle);

      const integrationWindow = findStepIntegrationWindow(smoothedAngVel, timestamps, cycle);
      const metricStartIdx = integrationWindow.startIdx;
      const metricEndIdx = integrationWindow.endIdx;
      // ส่งจาก HS → ปลาย window เพื่อให้ start-only ZUPT วัด v ก่อนเข้า quiet ได้จริง
      // (ถ้า slice แค่ quiet→end แล้วลบ velocity[0] จะเป็น no-op เพราะ trapz ตั้ง [0]=0)
      const segmentStartIdx = Math.min(cycle.hsStart.index, metricStartIdx);
      const segmentEndIdx = metricEndIdx;
      const localIntegrationStartIdx = metricStartIdx - segmentStartIdx;
      const localIntegrationEndIdx = metricEndIdx - segmentStartIdx;

      strideTimes.push(cycle.strideTime);
      stancePcts.push(cycle.stancePct);
      swingPcts.push(cycle.swingPct);
      integrationWindows.push(integrationWindow);

      let peakAngle = -Infinity;
      // peak มุม: เต็มช่วง HS→HS เหมือน MoCap — ห้ามใช้แค่ integration window
      // (integration เริ่มหลัง quiet หลัง HS → แคบกว่า → IMU peak ≤ MoCap เชิงระบบ)
      const peakStartIdx = cycle.hsStart.index;
      const peakEndIdx = cycle.hsEnd.index;
      for (let j = peakStartIdx; j <= peakEndIdx && j < sampleCount; j += 1) {
        if (Number.isFinite(shankAngle[j])) peakAngle = Math.max(peakAngle, shankAngle[j]);
      }
      peakAngles.push(peakAngle);

      // integration / clearance ยังใช้ quiet-bounded window ตามเดิม
      const cycleAx = [];
      const cycleAy = [];
      const cycleAz = [];
      const cycleAngles = [];
      const cycleQuats = [];
      let madgwickComplete = this.orientationFilterMode === 'madgwick';
      for (let j = segmentStartIdx; j <= segmentEndIdx && j < sampleCount; j += 1) {
        cycleAx.push(axG[j] * G_MS2);
        cycleAy.push(ayG[j] * G_MS2);
        cycleAz.push(azG[j] * G_MS2);
        cycleAngles.push(shankAngle[j]);
        const q = orientations[j];
        if (!q) madgwickComplete = false;
        cycleQuats.push(q);
      }

      // dt จริงเฉลี่ยของ window จาก timestamp (วินาที) เพื่อให้ double integration
      // ทนต่อ dropped sample เท่ากับ temporal metrics; fallback เป็น nominal ถ้า timestamp ใช้ไม่ได้
      const segmentSampleSpan = Math.max(1, metricEndIdx - metricStartIdx);
      const winDt = Number.isFinite(timestamps[metricEndIdx]) && Number.isFinite(timestamps[metricStartIdx])
        ? (timestamps[metricEndIdx] - timestamps[metricStartIdx]) / segmentSampleSpan
        : (1.0 / SAMPLE_RATE);

      const {
        strideLength: integratedStrideLength,
        strideLengthSigned,
        clearance,
        velocityPreDriftCorrection,
        vStartPreDrift: vStartFromIntegrator,
        vEndPreDrift: vEndFromIntegrator,
      } = this.velocityIntegrator.computeStrideMetrics(
        cycleAy,
        cycleAz,
        cycleAngles,
        {
          integrationStartIdx: localIntegrationStartIdx,
          integrationEndIdx: localIntegrationEndIdx,
          dt: winDt,
          axArray: madgwickComplete ? cycleAx : undefined,
          quaternions: madgwickComplete ? cycleQuats : undefined,
        },
      );
      // open stride ไม่ใช้เป็นระยะทางตีพิมพ์ — ยังคำนวณไว้ debug แต่ mark แยก
      const strideLength = isOpenStride
        ? null
        : Math.max(
          STRIDE_LENGTH_MIN_M,
          Math.min(STRIDE_LENGTH_MAX_M, integratedStrideLength),
        );
      const strideClamped = !isOpenStride && (
        integratedStrideLength < STRIDE_LENGTH_MIN_M
        || integratedStrideLength > STRIDE_LENGTH_MAX_M
      );
      const accelMagAt = (idx) => (idx >= 0 && idx < sampleCount
        ? Math.sqrt(axG[idx] * axG[idx] + ayG[idx] * ayG[idx] + azG[idx] * azG[idx])
        : NaN);
      const startDev = Math.abs(accelMagAt(metricStartIdx) - 1);
      const endDev = Math.abs(accelMagAt(metricEndIdx) - 1);
      const zuptDeviation = Math.max(
        Number.isFinite(startDev) ? startDev : 0,
        Number.isFinite(endDev) ? endDev : 0,
      );
      const vEndPreDrift = Number.isFinite(vEndFromIntegrator)
        ? vEndFromIntegrator
        : (velocityPreDriftCorrection?.length
          ? velocityPreDriftCorrection[velocityPreDriftCorrection.length - 1]
          : null);
      // v ที่จุดเข้า window ก่อนรีเซ็ต ZUPT (วัดจาก integrate ตั้งแต่ HS) — ไม่ใช่ trapz[0]=0 ปลอม
      const vStartPreDrift = Number.isFinite(vStartFromIntegrator)
        ? vStartFromIntegrator
        : (velocityPreDriftCorrection?.length ? velocityPreDriftCorrection[0] : null);
      const strideUntrusted = isOpenStride || suspectedMissedHs || (
        Number.isFinite(vEndPreDrift)
        && Math.abs(vEndPreDrift) > MAX_V_END_PRE_DRIFT_MPS
      );

      strideLengths.push(strideLength);
      clearances.push(Math.max(0, Math.min(0.3, clearance)));
      strideClampedFlags.push(strideClamped);
      strideUntrustedFlags.push(strideUntrusted);
      strideSignedLengths.push(isOpenStride ? null : strideLengthSigned);
      zuptAccelDeviations.push(zuptDeviation);
      vEndPreDriftByCycle.push(Number.isFinite(vEndPreDrift) ? vEndPreDrift : null);
      vStartPreDriftByCycle.push(Number.isFinite(vStartPreDrift) ? vStartPreDrift : null);

      if (shouldEmitDiagnostic) {
        const strideTime = cycle.strideTime;
        const nullOpen = isOpenStride;
        this.pendingCycleDiagnostics.push({
          cycleKey: isOpenStride
            ? openDiagKey
            : makeClosedCycleKey(countableCycleStartSampleId, countableCycleEndSampleId),
          cycleStartSampleId: countableCycleStartSampleId,
          cycleEndSampleId: countableCycleEndSampleId,
          cycleStartTimestampMs: samples[cycle.hsStart.index]?.timestampMs ?? null,
          cycleEndTimestampMs: samples[cycle.hsEnd.index]?.timestampMs ?? null,
          strideLengthM: (strideUntrusted || isOpenStride) ? null : strideLength,
          strideLengthClamped: strideClamped || strideUntrusted,
          strideLengthUntrusted: strideUntrusted,
          suspectedMissedHs,
          isOpenStride,
          side: samples[cycle.hsStart.index]?.side ?? null,
          strideTimeS: nullOpen ? null : (Number.isFinite(strideTime) ? strideTime : null),
          cadenceSpm: nullOpen ? null : (
            Number.isFinite(strideTime) && strideTime > 0 ? (2 / strideTime) * 60 : null
          ),
          walkingSpeedMps: nullOpen ? null : (
            (Number.isFinite(strideLength) && Number.isFinite(strideTime) && strideTime > 0)
              ? strideLength / strideTime
              : null
          ),
          stancePct: nullOpen ? null : (Number.isFinite(cycle.stancePct) ? cycle.stancePct : null),
          swingPct: nullOpen ? null : (Number.isFinite(cycle.swingPct) ? cycle.swingPct : null),
          temporalSource: cycle.temporalSource ?? null,
          peakShankAngleDeg: nullOpen ? null : (Number.isFinite(peakAngle) ? peakAngle : null),
          clearanceM: nullOpen ? null : Math.max(0, Math.min(0.3, clearance)),
          zuptCheck: {
            vStartPreDrift: Number.isFinite(vStartPreDrift) ? vStartPreDrift : null,
            vEndPreDrift: Number.isFinite(vEndPreDrift) ? vEndPreDrift : null,
            windowSource: integrationWindow.source ?? null,
            zuptAccelDeviationG: Number.isFinite(zuptDeviation) ? zuptDeviation : null,
            maxVEndPreDriftMps: MAX_V_END_PRE_DRIFT_MPS,
          },
        });
        if (this.pendingCycleDiagnostics.length > 500) {
          this.pendingCycleDiagnostics.shift();
        }
      }
    }

    if (processedCycles.length === 0) {
      this.processedData = {
        timestamps,
        angularVelocity: angVelDeg,
        shankAngle,
        events,
        cycles,
        integrationWindow: null,
      };
      // drain ครั้งเดียวก่อน forEach — ถ้าเรียกในตัว callback เอง listener ตัวแรกจะกวาด
      // array ไปหมด listener ที่เหลือได้ [] เงียบ ๆ โดยไม่มี error ใด ๆ (พังก็ต่อเมื่อมี
      // listener ตัวที่สองเข้ามา ซึ่งตอนนี้ยังไม่มีจึงไม่เคยเห็นอาการ)
      const diagnostics = this._drainCycleDiagnostics();
      this.paramListeners.forEach((callback) => callback({
        params: null,
        processedData: this.processedData,
        newCycleDiagnostics: diagnostics,
      }));
      return;
    }

    const lastIdx = processedCycles.length - 1;
    const lastCycle = processedCycles[lastIdx];
    const lastIntegrationWindow = integrationWindows[lastIdx] ?? null;
    const strideLengthLast = strideLengths[lastIdx];
    const strideClampedLast = strideClampedFlags[lastIdx] ?? false;
    const strideUntrustedLast = strideUntrustedFlags[lastIdx] ?? false;
    const strideSignedLast = strideSignedLengths[lastIdx];
    const zuptAccelDeviationLast = zuptAccelDeviations[lastIdx];
    const strideTimeLast = strideTimes[lastIdx];
    const stancePctLast = stancePcts[lastIdx];
    const swingPctLast = swingPcts[lastIdx];
    const peakAngleLast = peakAngles[lastIdx];
    const clearanceLast = clearances[lastIdx];
    const cycleStartSample = samples[lastCycle.hsStart.index] ?? null;
    const cycleEndSample = samples[lastCycle.hsEnd.index] ?? null;
    const cycleStartSampleId = cycleStartSample?.sampleId ?? null;
    const cycleEndSampleId = cycleEndSample?.sampleId ?? null;
    const cycleStartTimestampMs = cycleStartSample?.timestampMs ?? null;

    // zuptCheck ของ cycle ล่าสุด — ให้ reprocess/UI เห็น vEnd เสมอ
    const lastZuptCheck = {
      vStartPreDrift: vStartPreDriftByCycle[lastIdx] ?? null,
      vEndPreDrift: vEndPreDriftByCycle[lastIdx] ?? null,
      windowSource: lastIntegrationWindow?.source ?? null,
      zuptAccelDeviationG: Number.isFinite(zuptAccelDeviationLast) ? zuptAccelDeviationLast : null,
      maxVEndPreDriftMps: MAX_V_END_PRE_DRIFT_MPS,
    };

    // ตัด interval / open id ที่เลื่อนออกจาก buffer แล้วทิ้ง เพื่อไม่ให้ Set โตไม่จำกัด
    const oldestBufferedSampleId = samples[0]?.sampleId ?? null;
    if (Number.isFinite(oldestBufferedSampleId)) {
      this.countedCycleIntervals = this.countedCycleIntervals.filter(
        (c) => c.endId >= oldestBufferedSampleId,
      );
      for (const id of [...this.openStrideStartIds]) {
        if (id < oldestBufferedSampleId) this.openStrideStartIds.delete(id);
      }
    }

    const sessionDuration = this.sessionStartTime && this.latestSampleTimestampMs
      ? (this.latestSampleTimestampMs - this.sessionStartTime) / 1000
      : 0;

    const lastIsOpen = Boolean(lastCycle.isOpenStride);
    // cadence แบบประมาณ steps/min จากขาเดียว (สากลใน unilateral IMU / เทียบ MoCap)
    // — ไม่ได้แปลว่า stepCount = 2×strideCount; นับก้าวจริงดู stepCount/aggregate
    const cadence = !lastIsOpen && strideTimeLast > 0 ? (2 / strideTimeLast) * 60 : null;
    const stepLength = null;
    const stepTime = null;
    const walkingSpeed = !lastIsOpen && strideTimeLast > 0 && Number.isFinite(strideLengthLast)
      ? strideLengthLast / strideTimeLast
      : null;
    // double support ต้องมี HS/TO สองข้าง — สูตร 2·stance−100 สมมาตรหลอก (พังกับ stroke)
    const doubleSupport = null;

    const cycleKey = cycleStartSampleId !== null
      ? (lastIsOpen
        ? `open:${cycleStartSampleId}`
        : makeClosedCycleKey(cycleStartSampleId, cycleEndSampleId))
      : `${cycleStartTimestampMs ?? Date.now()}`;

    this.latestParams = {
      strideLength: strideUntrustedLast ? null : strideLengthLast,
      stepLength,
      // clinical metadata: แยกค่าที่ถูก clamp ออกจากค่าวัดจริง + ธง ZUPT low-confidence
      strideLengthClamped: strideClampedLast || strideUntrustedLast,
      strideLengthUntrusted: strideUntrustedLast,
      isOpenStride: lastIsOpen,
      strideLengthSignedM: strideSignedLast,
      zuptAccelDeviationG: Number.isFinite(zuptAccelDeviationLast) ? zuptAccelDeviationLast : null,
      zuptCheck: lastZuptCheck,
      clearance: lastIsOpen ? null : clearanceLast,
      cadence,
      strideTime: lastIsOpen ? null : strideTimeLast,
      stepTime,
      stanceTime: lastIsOpen ? null : lastCycle.stanceTime,
      swingTime: lastIsOpen ? null : lastCycle.swingTime,
      stancePct: lastIsOpen ? null : stancePctLast,
      swingPct: lastIsOpen ? null : swingPctLast,
      temporalSource: lastCycle.temporalSource ?? null,
      walkingSpeed: strideUntrustedLast ? null : walkingSpeed,
      peakShankAngle: lastIsOpen ? null : peakAngleLast,
      doubleSupport,
      orientationFilter: this.orientationFilterMode,
      stepCount: this.totalStepCount,
      strideCount: this.totalStrideCount,
      sessionDuration,
      cycleCount: 1,
      strideLengths: Number.isFinite(strideLengthLast) ? [strideLengthLast] : [],
      stepLengths: [],
      strideTimes: lastIsOpen ? [] : [strideTimeLast],
      cycleStartSampleId,
      cycleStartTimestampMs,
      cycleEndTimestampMs: cycleEndSample?.timestampMs ?? null,
      integrationStartTimestampS: lastIntegrationWindow?.startTime ?? null,
      integrationEndTimestampS: lastIntegrationWindow?.endTime ?? null,
      integrationAngularVelocityThresholdDps: lastIntegrationWindow?.threshold ?? null,
      integrationSource: lastIntegrationWindow?.source ?? null,
      cycleKey,
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

    // drain ครั้งเดียวก่อน forEach (ดูคอมเมนต์ที่ early-return ด้านบน — เหตุผลเดียวกัน)
    const diagnostics = this._drainCycleDiagnostics();
    this.paramListeners.forEach((callback) => callback({
      params: this.latestParams,
      processedData: this.processedData,
      newCycleDiagnostics: diagnostics,
    }));
  }

  _drainCycleDiagnostics() {
    const drained = this.pendingCycleDiagnostics;
    this.pendingCycleDiagnostics = [];
    return drained;
  }

  reset() {
    this.buffer = [];
    this.kalman.reset();
    this.madgwick.reset();
    this.latestParams = null;
    this.totalStepCount = 0;
    this.totalStrideCount = 0;
    this.countedCycleIntervals = [];
    this.openStrideStartIds = new Set();
    this.recentClosedStrideTimesS = [];
    this.pendingCycleDiagnostics = [];
    this.nextSampleId = 1;
    this.sessionStartTime = null;
    this.latestSampleTimestampMs = null;
    this.relativeTimestampOriginMs = null;
    this.accumulatedRelativeMs = 0;
    this.lastRelativeDeltaMs = 1000 / SAMPLE_RATE;
    this.absoluteTimestampOriginMs = null;
    this.lastSourceTimestampMs = null;
    this.usedSyntheticTimestamps = false;
    this.missingTimestampCount = 0;
    this.skippedIncompleteSampleCount = 0;
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
    const { valueMs, isAbsolute, missing } = normalizeTimestampField(sample);
    if (missing || !Number.isFinite(valueMs)) {
      this.missingTimestampCount += 1;
      this.usedSyntheticTimestamps = true;
      // interpolate จาก sample ก่อนหน้าเท่านั้น — ไม่ใช้ wall clock
      if (Number.isFinite(this.latestSampleTimestampMs)) {
        return this.latestSampleTimestampMs + (1000 / SAMPLE_RATE);
      }
      return 0;
    }
    if (isAbsolute) {
      this.lastSourceTimestampMs = valueMs;
      return valueMs;
    }

    // Relative timestamp (เช่น ESP32 micros()/1000 นับจากบูต): session-relative ms
    // ที่ reproducible — ไม่ผูก Date.now()
    //
    // สำคัญ: micros() เป็น uint32 ห่อกลับ ~ทุก 71.6 นาที (t_ms ≈ 4.29e6 → 0)
    // ถ้าแค่ reset origin เวลา session จะถอยเป็น 0 กลาง trace → sort/align พัง
    // แก้โดยสะสม offset ข้าม wrap แล้วต่อช่วงเวลาให้เดินหน้าต่อเนื่อง
    if (this.relativeTimestampOriginMs === null) {
      this.relativeTimestampOriginMs = valueMs;
      this.accumulatedRelativeMs = 0;
    } else if (
      this.lastSourceTimestampMs !== null
      && valueMs < this.lastSourceTimestampMs - 1000
    ) {
      const typicalDeltaMs = Number.isFinite(this.lastRelativeDeltaMs)
        ? this.lastRelativeDeltaMs
        : (1000 / SAMPLE_RATE);
      this.accumulatedRelativeMs += (
        this.lastSourceTimestampMs - this.relativeTimestampOriginMs
      ) + typicalDeltaMs;
      this.relativeTimestampOriginMs = valueMs;
    }

    const resolved = this.accumulatedRelativeMs + (valueMs - this.relativeTimestampOriginMs);
    if (this.lastSourceTimestampMs !== null) {
      const sourceDelta = valueMs - this.lastSourceTimestampMs;
      // เก็บ Δt ปกติไว้ใช้ตอน wrap (ข้ามค่าติดลบจาก wrap เอง)
      if (sourceDelta > 0 && sourceDelta < 500) {
        this.lastRelativeDeltaMs = sourceDelta;
      }
    }
    this.lastSourceTimestampMs = valueMs;
    return resolved;
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
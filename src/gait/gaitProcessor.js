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
// การ integrate ความเร่งตลอด HS→HS ของขาเดียวกัน = ระยะ 1 stride โดยตรง
// step = stride/2 (ประมาณ เพราะเซนเซอร์ข้างเดียววัด step ของขาตรงข้ามไม่ได้)
// floor ต่ำ (0.10) เพื่อไม่ทำลายข้อมูลผู้ป่วย stroke ที่ stride สั้นกว่า 0.30m ได้จริง
const STRIDE_LENGTH_MIN_M = 0.10;
const STRIDE_LENGTH_MAX_M = 1.80;
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
    this.countedCycleStartSampleIds = new Set();
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
    const aXg = rawAccelToG(getRawAxis(sample, 'ax', 'raw_accel', 0));
    const aYg = rawAccelToG(getRawAxis(sample, 'ay', 'raw_accel', 1));
    const aZg = rawAccelToG(getRawAxis(sample, 'az', 'raw_accel', 2));
    const accelAngle = accelToAngle(aYg, aZg);
    // ‖accel‖ (3 แกน) ใช้ gate ความเชื่อ accel: ใกล้ 1g = gravity ล้วน, เบี่ยงมาก = มี motion accel
    const accelMagnitudeG = Math.sqrt(aXg * aXg + aYg * aYg + aZg * aZg);
    const shankAngle = this.kalman.update(gx, accelAngle, dtSeconds, accelMagnitudeG);

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
    const axG = new Array(sampleCount);
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
      const aXg = rawAccelToG(getRawAxis(sample, 'ax', 'raw_accel', 0));
      const aYg = rawAccelToG(getRawAxis(sample, 'ay', 'raw_accel', 1));
      const aZg = rawAccelToG(getRawAxis(sample, 'az', 'raw_accel', 2));

      angVelDeg[i] = gx;
      axG[i] = aXg;
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
    const strideClampedFlags = [];
    const strideSignedLengths = [];
    const zuptAccelDeviations = [];

    for (const cycle of cycles) {
      // นับทุก cycle ที่ยังไม่เคยนับ (ไม่ใช่แค่ cycle สุดท้าย) เพื่อไม่ให้พลาด
      // cycle ที่มีอยู่ใน buffer ตั้งแต่ analyze() ครั้งแรก 1 cycle = 1 stride = 2 steps
      const countableCycleStartSampleId = samples[cycle.hsStart.index]?.sampleId ?? null;
      const isNewCycle = countableCycleStartSampleId !== null
        && !this.countedCycleStartSampleIds.has(countableCycleStartSampleId);
      if (isNewCycle) {
        this.countedCycleStartSampleIds.add(countableCycleStartSampleId);
        this.totalStepCount += 2;
      }

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
      // peak มุม: เต็มช่วง HS→HS เหมือน MoCap — ห้ามใช้แค่ integration window
      // (integration เริ่มหลัง quiet หลัง HS → แคบกว่า → IMU peak ≤ MoCap เชิงระบบ)
      const peakStartIdx = cycle.hsStart.index;
      const peakEndIdx = cycle.hsEnd.index;
      for (let j = peakStartIdx; j <= peakEndIdx && j < sampleCount; j += 1) {
        if (Number.isFinite(shankAngle[j])) peakAngle = Math.max(peakAngle, shankAngle[j]);
      }
      peakAngles.push(peakAngle);

      // integration / clearance ยังใช้ quiet-bounded window ตามเดิม
      const cycleAy = [];
      const cycleAz = [];
      const cycleAngles = [];
      for (let j = segmentStartIdx; j <= segmentEndIdx && j < sampleCount; j += 1) {
        cycleAy.push(ayG[j] * G_MS2);
        cycleAz.push(azG[j] * G_MS2);
        cycleAngles.push(shankAngle[j]);
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
      } = this.velocityIntegrator.computeStrideMetrics(
        cycleAy,
        cycleAz,
        cycleAngles,
        {
          integrationStartIdx: localIntegrationStartIdx,
          integrationEndIdx: localIntegrationEndIdx,
          dt: winDt,
        },
      );
      const strideLength = Math.max(
        STRIDE_LENGTH_MIN_M,
        Math.min(STRIDE_LENGTH_MAX_M, integratedStrideLength),
      );
      const stepLength = strideLength / 2; // สมมติสมมาตร L/R — ไม่ใช้กับ stroke asymmetry (ดู bilateral HS)
      // flag เมื่อค่าถูก clamp (ชนเพดาน/พื้น) เพื่อไม่ให้ปนกับค่าวัดจริงตอนทำ ICC/Bland-Altman
      const strideClamped = integratedStrideLength < STRIDE_LENGTH_MIN_M
        || integratedStrideLength > STRIDE_LENGTH_MAX_M;
      // ZUPT-validity: correctDrift สมมติ v=0 ที่ปลาย window — ถ้าปลายไม่ใช่จุดเท้านิ่ง
      // (‖accel‖ เบี่ยงจาก 1g มาก) การประมาณระยะจะต่ำกว่าจริงแบบ systematic. ตรวจ ‖accel‖
      // ที่ขอบ window เพื่อ mark ค่า low-confidence โดยไม่แก้ค่า (การย้าย window ต้อง validate ข้อมูลจริง)
      const accelMagAt = (idx) => (idx >= 0 && idx < sampleCount
        ? Math.sqrt(axG[idx] * axG[idx] + ayG[idx] * ayG[idx] + azG[idx] * azG[idx])
        : NaN);
      const startDev = Math.abs(accelMagAt(metricStartIdx) - 1);
      const endDev = Math.abs(accelMagAt(metricEndIdx) - 1);
      const zuptDeviation = Math.max(
        Number.isFinite(startDev) ? startDev : 0,
        Number.isFinite(endDev) ? endDev : 0,
      );

      stepLengths.push(stepLength);
      strideLengths.push(strideLength);
      clearances.push(Math.max(0, Math.min(0.3, clearance)));
      strideClampedFlags.push(strideClamped);
      strideSignedLengths.push(strideLengthSigned);
      zuptAccelDeviations.push(zuptDeviation);

      // เก็บ ZUPT diagnostic ของ "ทุก" cycle ใหม่ (ไม่ใช่แค่ cycle สุดท้ายที่ latestParams เก็บ)
      // เพื่อให้วิเคราะห์ได้ว่า window วางผิดจุดเป็นระบบหรือแค่บางจังหวะ — วางคู่กับ isNewCycle
      // เดียวกับที่ใช้นับ step เพื่อไม่ให้ diagnostic ซ้ำ cycle เดิมเวลา buffer overlap กันข้าม analyze()
      if (isNewCycle) {
        const preDrift = velocityPreDriftCorrection || [];
        this.pendingCycleDiagnostics.push({
          cycleKey: String(countableCycleStartSampleId),
          cycleStartTimestampMs: samples[cycle.hsStart.index]?.timestampMs ?? null,
          strideLengthM: strideLength,
          strideLengthClamped: strideClamped,
          zuptCheck: {
            vStartPreDrift: Number.isFinite(preDrift[0]) ? preDrift[0] : null,
            vEndPreDrift: Number.isFinite(preDrift[preDrift.length - 1]) ? preDrift[preDrift.length - 1] : null,
            windowSource: integrationWindow.source ?? null,
            zuptAccelDeviationG: Number.isFinite(zuptDeviation) ? zuptDeviation : null,
          },
        });
        // กันโตไม่จำกัดถ้าไม่มีใคร drain (เช่น analyze() ถูกเรียกโดยไม่มี consumer)
        if (this.pendingCycleDiagnostics.length > 500) {
          this.pendingCycleDiagnostics.shift();
        }
      }
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

    const lastIdx = cycles.length - 1;
    const lastCycle = cycles[lastIdx];
    const lastIntegrationWindow = integrationWindows[lastIdx] ?? null;
    const stepLengthLast = stepLengths[lastIdx];
    const strideLengthLast = strideLengths[lastIdx];
    const strideClampedLast = strideClampedFlags[lastIdx] ?? false;
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
    const cycleStartTimestampMs = cycleStartSample?.timestampMs ?? null;

    // ตัด id ของ cycle ที่เลื่อนออกจาก buffer แล้วทิ้ง (ตรวจซ้ำไม่ได้อีก) เพื่อไม่ให้ Set โตไม่จำกัด
    const oldestBufferedSampleId = samples[0]?.sampleId ?? null;
    if (Number.isFinite(oldestBufferedSampleId)) {
      for (const countedId of this.countedCycleStartSampleIds) {
        if (countedId < oldestBufferedSampleId) {
          this.countedCycleStartSampleIds.delete(countedId);
        }
      }
    }

    this.totalStrideCount = Math.floor(this.totalStepCount / 2);

    const sessionDuration = this.sessionStartTime && this.latestSampleTimestampMs
      ? (this.latestSampleTimestampMs - this.sessionStartTime) / 1000
      : 0;

    const cadence = strideTimeLast > 0 ? (2 / strideTimeLast) * 60 : 0;
    const stepLength = stepLengthLast;
    const stepTime = strideTimeLast / 2;
    const walkingSpeed = strideTimeLast > 0 && Number.isFinite(strideLengthLast)
      ? strideLengthLast / strideTimeLast
      : 0;
    const doubleSupportPct = Number.isFinite(stancePctLast)
      ? Math.max(0, 2 * stancePctLast - 100) // สมมติสมมาตร L/R — ไม่ใช่ double-support จริงจาก HS/TO สองข้าง
      : null;
    const doubleSupport = Number.isFinite(doubleSupportPct)
      ? strideTimeLast * doubleSupportPct / 100
      : null;

    this.latestParams = {
      strideLength: strideLengthLast,
      stepLength,
      // clinical metadata: แยกค่าที่ถูก clamp ออกจากค่าวัดจริง + ธง ZUPT low-confidence
      strideLengthClamped: strideClampedLast,
      strideLengthSignedM: strideSignedLast,
      zuptAccelDeviationG: Number.isFinite(zuptAccelDeviationLast) ? zuptAccelDeviationLast : null,
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
    this.latestParams = null;
    this.totalStepCount = 0;
    this.totalStrideCount = 0;
    this.countedCycleStartSampleIds = new Set();
    this.pendingCycleDiagnostics = [];
    this.nextSampleId = 1;
    this.sessionStartTime = null;
    this.latestSampleTimestampMs = null;
    this.relativeTimestampOriginMs = null;
    this.accumulatedRelativeMs = 0;
    this.lastRelativeDeltaMs = 1000 / SAMPLE_RATE;
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
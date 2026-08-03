import { findLocalMinima, findZeroCrossing, movingAverage } from './signalUtils.js';

function getTimeAt(timestamps, index, fallbackDt) {
  if (timestamps && Number.isFinite(timestamps[index])) {
    return timestamps[index];
  }

  return index * fallbackDt;
}

function getDurationBetween(timestamps, startIdx, endIdx, fallbackDt) {
  if (startIdx === null || endIdx === null || startIdx === undefined || endIdx === undefined) {
    return null;
  }

  const startTime = getTimeAt(timestamps, startIdx, fallbackDt);
  const endTime = getTimeAt(timestamps, endIdx, fallbackDt);
  return Math.max(0, endTime - startTime);
}

function median(values) {
  if (!values.length) {
    return null;
  }

  const sorted = [...values].sort((left, right) => left - right);
  const middle = Math.floor(sorted.length / 2);
  if (sorted.length % 2 === 0) {
    return (sorted[middle - 1] + sorted[middle]) / 2;
  }

  return sorted[middle];
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function findWindowMax(signal, startIdx, endIdx) {
  let maxValue = -Infinity;
  for (let i = Math.max(0, startIdx); i <= Math.min(signal.length - 1, endIdx); i += 1) {
    maxValue = Math.max(maxValue, signal[i]);
  }

  return maxValue;
}

function findHsCandidateMinima(signal, minProminence, minSeparation, prominenceWindow) {
  const minima = [];

  for (let i = 1; i < signal.length - 1; i += 1) {
    if (signal[i] < signal[i - 1] && signal[i] <= signal[i + 1]) {
      const leftMax = findWindowMax(signal, i - prominenceWindow, i);
      const rightMax = findWindowMax(signal, i, i + prominenceWindow);
      const prominence = Math.min(leftMax - signal[i], rightMax - signal[i]);

      if (prominence >= minProminence) {
        if (minima.length === 0 || (i - minima[minima.length - 1]) >= minSeparation) {
          minima.push(i);
        } else if (signal[i] < signal[minima[minima.length - 1]]) {
          minima[minima.length - 1] = i;
        }
      }
    }
  }

  return minima;
}

function findPositivePeak(signal, startIdx, endIdx) {
  let peak = -Infinity;
  for (let i = Math.max(0, startIdx); i <= Math.min(signal.length - 1, endIdx); i += 1) {
    peak = Math.max(peak, signal[i]);
  }

  return peak > 0 ? peak : null;
}

function buildHsCandidateExtrema(smoothed, index, envelopeWindowSamples, previousAcceptedIndex) {
  const localEnvelopePeak = findPositivePeak(
    smoothed,
    index - envelopeWindowSamples,
    index + envelopeWindowSamples,
  );
  const interCyclePeak = Number.isInteger(previousAcceptedIndex) && previousAcceptedIndex < index
    ? findPositivePeak(smoothed, previousAcceptedIndex, index)
    : null;

  return {
    hsAbs: Math.abs(smoothed[index]),
    localEnvelopePeakAbs: localEnvelopePeak,
    interCyclePeakAbs: interCyclePeak,
  };
}

function resolveAdaptiveHsThresholdAbs(candidateExtrema, acceptedExtremaHistory, options) {
  const recentExtrema = acceptedExtremaHistory.slice(-options.historySize);
  const previousHsAbs = recentExtrema
    .map((extrema) => extrema.hsAbs)
    .filter(Number.isFinite);
  const previousPeakAbs = recentExtrema
    .map((extrema) => extrema.interCyclePeakAbs ?? extrema.localEnvelopePeakAbs)
    .filter(Number.isFinite);
  const candidatePeakAbs = candidateExtrema.interCyclePeakAbs ?? candidateExtrema.localEnvelopePeakAbs;

  const thresholdComponents = [
    Number.isFinite(candidatePeakAbs) ? candidatePeakAbs * options.envelopeScale : null,
    previousPeakAbs.length ? median(previousPeakAbs) * options.previousPeakScale : null,
    previousHsAbs.length ? median(previousHsAbs) * options.previousHsScale : null,
  ].filter(Number.isFinite);

  if (!thresholdComponents.length) {
    return options.ceilingAbs;
  }

  return clamp(
    median(thresholdComponents),
    options.floorAbs,
    options.ceilingAbs,
  );
}

function resolveCycleTemporalMetrics(strideTime, measuredStanceTime, previousValidStanceRatio, usePreviousValidStanceFallback) {
  if (!Number.isFinite(strideTime) || strideTime <= 0) {
    return {
      stanceTime: null,
      swingTime: null,
      stancePct: null,
      swingPct: null,
      temporalSource: 'unresolved',
      nextValidStanceRatio: previousValidStanceRatio,
    };
  }

  if (Number.isFinite(measuredStanceTime)) {
    const stanceTime = Math.max(0, Math.min(strideTime, measuredStanceTime));
    const swingTime = Math.max(0, strideTime - stanceTime);
    const stancePct = (stanceTime / strideTime) * 100;
    const swingPct = 100 - stancePct;

    return {
      stanceTime,
      swingTime,
      stancePct,
      swingPct,
      temporalSource: 'measured-to',
      nextValidStanceRatio: stanceTime / strideTime,
    };
  }

  if (usePreviousValidStanceFallback && Number.isFinite(previousValidStanceRatio)) {
    const boundedRatio = Math.max(0, Math.min(1, previousValidStanceRatio));
    const stanceTime = strideTime * boundedRatio;
    const swingTime = Math.max(0, strideTime - stanceTime);
    const stancePct = boundedRatio * 100;
    const swingPct = 100 - stancePct;

    return {
      stanceTime,
      swingTime,
      stancePct,
      swingPct,
      temporalSource: 'previous-valid-ratio',
      nextValidStanceRatio: previousValidStanceRatio,
    };
  }

  return {
    stanceTime: null,
    swingTime: null,
    stancePct: null,
    swingPct: null,
    temporalSource: 'unresolved',
    nextValidStanceRatio: previousValidStanceRatio,
  };
}

export class GaitEventDetector {
  constructor(options = {}) {
    this.sampleRate = options.sampleRate || 100;
    this.hsProminence = options.hsProminence || 80;
    this.toProminence = options.toProminence || 30;
    this.minStrideTime = options.minStrideTime || 0.6;
    this.minHsSeparationSeconds = options.minHsSeparationSeconds ?? null;
    this.maxStrideTime = options.maxStrideTime ?? 2.0;
    this.smoothingWindow = options.smoothingWindow || 5;
    this.stillAngularVelocityAbs = options.stillAngularVelocityAbs ?? 10;
    this.stillDurationSeconds = options.stillDurationSeconds ?? 0.2;
    this.minSwingAngularVelocityAbs = options.minSwingAngularVelocityAbs ?? 20;
    this.minOpenStrideTimeSeconds = options.minOpenStrideTimeSeconds ?? 0.3;
    this.toSearchStartPct = options.toSearchStartPct ?? 0.35;
    this.toSearchEndPct = options.toSearchEndPct ?? 0.75;
    this.hsVelocityThreshold = options.hsVelocityThreshold ?? -50;
    this.hsVelocityThresholdFloorAbs = options.hsVelocityThresholdFloorAbs ?? 18;
    this.hsVelocityThresholdCeilingAbs = Math.abs(this.hsVelocityThreshold);
    this.hsEnvelopeScale = options.hsEnvelopeScale ?? 0.35;
    this.hsPreviousPeakScale = options.hsPreviousPeakScale ?? 0.35;
    this.hsPreviousHsScale = options.hsPreviousHsScale ?? 0.75;
    this.hsAdaptiveHistorySize = options.hsAdaptiveHistorySize ?? 3;
    this.hsEnvelopeWindowSeconds = options.hsEnvelopeWindowSeconds ?? 0.8;
    this.usePreviousValidStanceFallback = options.usePreviousValidStanceFallback ?? true;
  }

  detect(angVel, timestamps) {
    const dt = 1.0 / this.sampleRate;

    const smoothed = movingAverage(angVel, this.smoothingWindow);
    const minHsSeparationSeconds = Number.isFinite(this.minHsSeparationSeconds)
      ? this.minHsSeparationSeconds
      : this.minStrideTime * 0.8;
    const minHsDistance = Math.max(1, Math.round(minHsSeparationSeconds * this.sampleRate));
    const hsEnvelopeWindowSamples = Math.max(
      minHsDistance,
      Math.round(this.hsEnvelopeWindowSeconds * this.sampleRate),
    );
    const allMinima = findHsCandidateMinima(
      smoothed,
      this.hsProminence,
      minHsDistance,
      hsEnvelopeWindowSamples,
    );
    const acceptedHsExtremaHistory = [];
    const heelStrikes = [];

    for (const idx of allMinima) {
      const previousAcceptedIndex = heelStrikes.length ? heelStrikes[heelStrikes.length - 1] : null;
      const candidateExtrema = buildHsCandidateExtrema(
        smoothed,
        idx,
        hsEnvelopeWindowSamples,
        previousAcceptedIndex,
      );
      const adaptiveThresholdAbs = resolveAdaptiveHsThresholdAbs(candidateExtrema, acceptedHsExtremaHistory, {
        floorAbs: this.hsVelocityThresholdFloorAbs,
        ceilingAbs: this.hsVelocityThresholdCeilingAbs,
        envelopeScale: this.hsEnvelopeScale,
        previousPeakScale: this.hsPreviousPeakScale,
        previousHsScale: this.hsPreviousHsScale,
        historySize: this.hsAdaptiveHistorySize,
      });

      if (candidateExtrema.hsAbs < adaptiveThresholdAbs) {
        continue;
      }

      if (Number.isInteger(previousAcceptedIndex)) {
        const hsSeparationSeconds = getDurationBetween(timestamps, previousAcceptedIndex, idx, dt);
        if (hsSeparationSeconds !== null && hsSeparationSeconds < minHsSeparationSeconds) {
          continue;
        }
      }

      heelStrikes.push(idx);
      acceptedHsExtremaHistory.push(candidateExtrema);
    }

    const events = [];
    const rawStrides = [];
    let previousValidStanceRatio = null;

    for (const idx of heelStrikes) {
      events.push({
        index: idx,
        time: getTimeAt(timestamps, idx, dt),
        type: 'HS',
      });
    }

    for (let i = 0; i < heelStrikes.length - 1; i += 1) {
      const hsStartIdx = heelStrikes[i];
      const hsEndIdx = heelStrikes[i + 1];
      const strideSamples = hsEndIdx - hsStartIdx;
      const strideTime = getDurationBetween(timestamps, hsStartIdx, hsEndIdx, dt);

      if (strideTime === null || strideTime < this.minStrideTime || strideTime > this.maxStrideTime) {
        continue;
      }

      const msvIdx = findZeroCrossing(
        smoothed,
        hsStartIdx + 5,
        hsStartIdx + Math.floor(strideSamples * 0.4),
      );

      if (msvIdx !== null) {
        events.push({
          index: msvIdx,
          time: getTimeAt(timestamps, msvIdx, dt),
          type: 'MSV',
        });
      }

      let toIdx = null;
      const toSearchStart = hsStartIdx + Math.floor(strideSamples * this.toSearchStartPct);
      const toSearchEnd = Math.min(hsEndIdx, hsStartIdx + Math.floor(strideSamples * this.toSearchEndPct));
      if (toSearchEnd > toSearchStart + 2) {
        const toMinima = findLocalMinima(
          smoothed.slice(toSearchStart, toSearchEnd),
          this.toProminence,
          10,
        );

        if (toMinima.length > 0) {
          toIdx = toMinima[0] + toSearchStart;
          events.push({
            index: toIdx,
            time: getTimeAt(timestamps, toIdx, dt),
            type: 'TO',
          });
        }
      }

      const measuredStanceTime = toIdx !== null
        ? getDurationBetween(timestamps, hsStartIdx, toIdx, dt)
        : null;
      const temporalMetrics = resolveCycleTemporalMetrics(
        strideTime,
        measuredStanceTime,
        previousValidStanceRatio,
        this.usePreviousValidStanceFallback,
      );
      previousValidStanceRatio = temporalMetrics.nextValidStanceRatio;

      rawStrides.push({
        hsStart: { index: hsStartIdx, time: getTimeAt(timestamps, hsStartIdx, dt), type: 'HS' },
        hsEnd: { index: hsEndIdx, time: getTimeAt(timestamps, hsEndIdx, dt), type: 'HS' },
        to: toIdx !== null ? { index: toIdx, time: getTimeAt(timestamps, toIdx, dt), type: 'TO' } : null,
        msv: msvIdx !== null ? { index: msvIdx, time: getTimeAt(timestamps, msvIdx, dt), type: 'MSV' } : null,
        strideTime,
        stanceTime: temporalMetrics.stanceTime,
        swingTime: temporalMetrics.swingTime,
        stancePct: temporalMetrics.stancePct,
        swingPct: temporalMetrics.swingPct,
        temporalSource: temporalMetrics.temporalSource,
        isOpenStride: false,
      });
    }

    if (heelStrikes.length >= 1 && timestamps && timestamps.length > 0) {
      const hsStartIdx = heelStrikes[heelStrikes.length - 1];
      const sampleCount = smoothed.length;

      let stillStartIdx = null;
      let endIdx = null;

      for (let i = hsStartIdx; i < sampleCount; i += 1) {
        if (Math.abs(smoothed[i]) <= this.stillAngularVelocityAbs) {
          if (stillStartIdx === null) {
            stillStartIdx = i;
          }

          const stillDuration = getDurationBetween(timestamps, stillStartIdx, i, dt);
          if (stillDuration !== null && stillDuration >= this.stillDurationSeconds) {
            endIdx = i;
            break;
          }
        } else {
          stillStartIdx = null;
        }
      }

      if (endIdx !== null) {
        const strideSamples = endIdx - hsStartIdx;
        const openStrideTime = getDurationBetween(timestamps, hsStartIdx, endIdx, dt);

        if (
          openStrideTime !== null
          && openStrideTime >= this.minOpenStrideTimeSeconds
          && openStrideTime <= this.maxStrideTime
        ) {
          let maxAbsWithin = 0;
          for (let j = hsStartIdx; j <= endIdx; j += 1) {
            maxAbsWithin = Math.max(maxAbsWithin, Math.abs(smoothed[j]));
          }

          if (maxAbsWithin >= this.minSwingAngularVelocityAbs) {
            const msvSearchEnd = hsStartIdx + Math.max(1, Math.floor(strideSamples * 0.4));
            const msvIdx = findZeroCrossing(smoothed, hsStartIdx + 5, msvSearchEnd);

            if (msvIdx !== null) {
              events.push({
                index: msvIdx,
                time: getTimeAt(timestamps, msvIdx, dt),
                type: 'MSV',
              });
            }

            const toSearchStart = hsStartIdx + Math.floor(strideSamples * this.toSearchStartPct);
            const toSearchEnd = Math.min(hsStartIdx + Math.floor(strideSamples * this.toSearchEndPct), endIdx);
            let toIdx = null;

            if (toSearchEnd > toSearchStart + 2) {
              const toMinima = findLocalMinima(
                smoothed.slice(toSearchStart, toSearchEnd),
                this.toProminence,
                10,
              );

              if (toMinima.length > 0) {
                toIdx = toMinima[0] + toSearchStart;
                events.push({
                  index: toIdx,
                  time: getTimeAt(timestamps, toIdx, dt),
                  type: 'TO',
                });
              }
            }

            const measuredStanceTime = toIdx !== null
              ? getDurationBetween(timestamps, hsStartIdx, toIdx, dt)
              : null;
            const temporalMetrics = resolveCycleTemporalMetrics(
              openStrideTime,
              measuredStanceTime,
              previousValidStanceRatio,
              this.usePreviousValidStanceFallback,
            );
            previousValidStanceRatio = temporalMetrics.nextValidStanceRatio;

            rawStrides.push({
              hsStart: { index: hsStartIdx, time: getTimeAt(timestamps, hsStartIdx, dt), type: 'HS' },
              // จุดจบคือ "ขานิ่ง" ไม่ใช่ HS จริง — ต้องแยกจาก stride เต็ม (HS→HS)
              hsEnd: { index: endIdx, time: getTimeAt(timestamps, endIdx, dt), type: 'STILL' },
              to: toIdx !== null ? { index: toIdx, time: getTimeAt(timestamps, toIdx, dt), type: 'TO' } : null,
              msv: msvIdx !== null ? { index: msvIdx, time: getTimeAt(timestamps, msvIdx, dt), type: 'MSV' } : null,
              strideTime: openStrideTime,
              stanceTime: temporalMetrics.stanceTime,
              swingTime: temporalMetrics.swingTime,
              stancePct: temporalMetrics.stancePct,
              swingPct: temporalMetrics.swingPct,
              temporalSource: temporalMetrics.temporalSource,
              isOpenStride: true,
            });
          }
        }
      }
    }

    const cycles = rawStrides.filter((cycle) => cycle?.hsStart && cycle?.hsEnd);

    events.sort((a, b) => a.index - b.index);
    return { events, cycles };
  }
}
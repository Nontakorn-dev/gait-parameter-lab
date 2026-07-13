export function rawAccelToG(raw) {
  return raw / 4096.0;
}

export function rawGyroToDps(raw) {
  return raw / 16.4;
}

export function deg2rad(deg) {
  return deg * Math.PI / 180.0;
}

export function rad2deg(rad) {
  return rad * 180.0 / Math.PI;
}

export function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

export function accelToAngle(ay, az) {
  return rad2deg(Math.atan2(-az, -ay));
}

export function movingAverage(data, window) {
  const result = new Array(data.length);
  const halfWin = Math.floor(window / 2);

  for (let i = 0; i < data.length; i += 1) {
    let sum = 0;
    let count = 0;
    for (let j = Math.max(0, i - halfWin); j <= Math.min(data.length - 1, i + halfWin); j += 1) {
      sum += data[j];
      count += 1;
    }
    result[i] = sum / count;
  }

  return result;
}

export function findLocalMinima(signal, minProminence = 50, minDistance = 30) {
  const minima = [];

  for (let i = 1; i < signal.length - 1; i += 1) {
    if (signal[i] < signal[i - 1] && signal[i] <= signal[i + 1]) {
      const leftMax = findLocalMax(signal, Math.max(0, i - minDistance), i);
      const rightMax = findLocalMax(signal, i, Math.min(signal.length - 1, i + minDistance));
      const prominence = Math.min(leftMax - signal[i], rightMax - signal[i]);

      if (prominence >= minProminence) {
        if (minima.length === 0 || (i - minima[minima.length - 1]) >= minDistance) {
          minima.push(i);
        } else if (signal[i] < signal[minima[minima.length - 1]]) {
          minima[minima.length - 1] = i;
        }
      }
    }
  }

  return minima;
}

function findLocalMax(signal, start, end) {
  let max = -Infinity;
  for (let i = start; i <= end; i += 1) {
    if (signal[i] > max) {
      max = signal[i];
    }
  }
  return max;
}

export function findZeroCrossing(signal, startIdx, endIdx) {
  for (let i = startIdx; i < endIdx - 1; i += 1) {
    if (signal[i] <= 0 && signal[i + 1] > 0) {
      return i;
    }
  }
  return null;
}

export function median(values) {
  const numeric = values.filter(Number.isFinite);
  if (!numeric.length) {
    return null;
  }

  const sorted = [...numeric].sort((left, right) => left - right);
  const mid = Math.floor(sorted.length / 2);
  return sorted.length % 2 === 0
    ? (sorted[mid - 1] + sorted[mid]) / 2
    : sorted[mid];
}

export function trapezoidalIntegrate(signal, dt) {
  const result = new Array(signal.length);
  result[0] = 0;

  for (let i = 1; i < signal.length; i += 1) {
    result[i] = result[i - 1] + (signal[i - 1] + signal[i]) * 0.5 * dt;
  }

  return result;
}
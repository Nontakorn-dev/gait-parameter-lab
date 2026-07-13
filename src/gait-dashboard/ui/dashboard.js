/**
 * Dashboard UI Controller
 *
 * Manages parameter display, event log, mode switching,
 * and connection status for the medical dashboard layout.
 */

import { aggregateGaitParams } from '../../gait/gaitParamsAggregation.js';

function formatMaybeNumber(value, digits = 1, fallback = '-') {
  return Number.isFinite(value) ? value.toFixed(digits) : fallback;
}

function formatPercentLabel(label, value) {
  return Number.isFinite(value)
    ? `${label} ${value.toFixed(1)}%`
    : `${label} -`;
}

export class Dashboard {
  constructor() {
    this.paramElements = {};
    this.currentMode = 'demo';
    this.animationTimers = {};
  }

  init() {
    this.paramElements = {
      stepCount: document.getElementById('val-step-count'),
      strideCount: document.getElementById('val-stride-count'),
      sessionDuration: document.getElementById('val-session-duration'),
      cadence: document.getElementById('val-cadence'),
      stepTime: document.getElementById('val-step-time'),
      strideTime: document.getElementById('val-stride-time'),
      doubleSupport: document.getElementById('val-double-support'),
      stepLength: document.getElementById('val-step-length'),
      strideLength: document.getElementById('val-stride-length'),
      walkingSpeed: document.getElementById('val-walking-speed'),
      peakAngle: document.getElementById('val-peak-angle'),
      stancePct: document.getElementById('val-stance-pct'),
      swingPct: document.getElementById('val-swing-pct'),
    };

    this.stanceBar = document.getElementById('stance-bar');
    this.swingBar = document.getElementById('swing-bar');
    this.stancePctLabel = document.getElementById('stance-pct-label');
    this.swingPctLabel = document.getElementById('swing-pct-label');

    this.eventList = document.getElementById('event-list');
    this.statusIndicator = document.getElementById('status-indicator');
    this.statusText = document.getElementById('status-text');
    this.modeLabel = document.getElementById('mode-label');
    this.sensorSummary = document.getElementById('sensor-side-summary');
    this.calibrationOverlay = document.getElementById('calibration-overlay');
    this.calibrationPanel = document.getElementById('calibration-panel');
    this.calibrationProgressPanel = document.getElementById('calibration-progress-panel');
    this.calibrationResultPanel = document.getElementById('calibration-result-panel');
    this.calibrationProgressBar = document.getElementById('calibration-progress-bar');
    this.calibrationProgressText = document.getElementById('calibration-progress-text');
    this.calibrationResultText = document.getElementById('calibration-result-text');
    this.calibrationBadge = document.getElementById('calibration-badge');
  }

  updateParams(params) {
    if (!params) {
      return;
    }

    this._animateValue('stepCount', String(params.stepCount || 0));
    this._animateValue('strideCount', String(params.strideCount || 0));
    this._animateValue('sessionDuration', formatMaybeNumber(params.sessionDuration, 1, '0.0'));

    this._animateValue('cadence', formatMaybeNumber(params.cadence, 1));
    this._animateValue('stepTime', formatMaybeNumber(params.stepTime, 2));
    this._animateValue('strideTime', formatMaybeNumber(params.strideTime, 2));
    this._animateValue('doubleSupport', formatMaybeNumber(params.doubleSupport, 2));

    this._animateValue('stepLength', formatMaybeNumber(params.stepLength, 2));
    this._animateValue('strideLength', formatMaybeNumber(params.strideLength, 2));
    this._animateValue('walkingSpeed', formatMaybeNumber(params.walkingSpeed, 2));

    this._animateValue('peakAngle', formatMaybeNumber(params.peakShankAngle, 1));
    this._animateValue('stancePct', formatMaybeNumber(params.stancePct, 1));
    this._animateValue('swingPct', formatMaybeNumber(params.swingPct, 1));

    if (this.stanceBar) {
      this.stanceBar.style.width = `${Number.isFinite(params.stancePct) ? params.stancePct : 0}%`;
      this.swingBar.style.width = `${Number.isFinite(params.swingPct) ? params.swingPct : 0}%`;
      this.stancePctLabel.textContent = formatPercentLabel('Stance', params.stancePct);
      this.swingPctLabel.textContent = formatPercentLabel('Swing', params.swingPct);
    }
  }

  updateMultiSensorParams(entries) {
    if (!entries?.length) {
      return;
    }

    const averaged = aggregateGaitParams(entries);
    this.updateParams(averaged);
  }

  updateSensorSummary(sensors = []) {
    if (!this.sensorSummary) {
      return;
    }

    if (!sensors.length) {
      this.sensorSummary.innerHTML = '<span class="sensor-summary-empty">Waiting for sensor data</span>';
      return;
    }

    this.sensorSummary.innerHTML = sensors.map((sensor) => {
      const sideClass = sensor.side === 'L' ? 'left' : sensor.side === 'R' ? 'right' : 'neutral';
      const sideLabel = sensor.side === 'L' ? 'Left' : sensor.side === 'R' ? 'Right' : 'Sensor';
      return `<span class="sensor-summary-pill ${sideClass}">
        <span class="sensor-summary-side">${sideLabel}</span>
        <span class="sensor-summary-label">${sensor.label}</span>
      </span>`;
    }).join('');
  }

  updateEvents(events = []) {
    if (!this.eventList) {
      return;
    }

    if (!events.length) {
      this.eventList.innerHTML = '';
      return;
    }

    const recentEvents = events.slice(-20);
    this.eventList.innerHTML = recentEvents.map((event) => {
      const typeClass = event.type.toLowerCase();
      const sideClass = event.side === 'L' ? 'left' : event.side === 'R' ? 'right' : 'neutral';
      const sideLabel = event.side === 'L' ? 'L' : event.side === 'R' ? 'R' : 'Sensor';
      return `<span class="event-tag ${typeClass}">
        <span class="event-side ${sideClass}">${sideLabel}</span>
        <span>${event.type}</span>
        <span style="opacity:0.7">${event.time.toFixed(2)}s</span>
      </span>`;
    }).join('');
  }

  updateStatus(status) {
    if (!this.statusIndicator) {
      return;
    }

    this.statusIndicator.className = 'status-indicator';

    const labels = {
      demo: 'Demo Mode',
      connected: 'Connected',
      disconnected: 'Disconnected',
      scanning: 'Scanning...',
      connecting: 'Connecting...',
      pairing: 'Choose Sensor...',
      error: 'Error',
    };

    this.statusText.textContent = labels[status] || status;

    if (status === 'demo') {
      this.statusIndicator.classList.add('demo');
    } else if (status === 'connected') {
      this.statusIndicator.classList.add('connected');
    } else if (status === 'error') {
      this.statusIndicator.classList.add('error');
    }
  }

  setMode(mode) {
    this.currentMode = mode;
    if (this.modeLabel) {
      const labels = {
        demo: 'Demo',
        'browser-ble': 'Browser BLE',
        'remote-broadcast': 'Remote Live',
      };
      this.modeLabel.textContent = labels[mode] || 'Live';
    }
  }

  activatePipelineStep() {
    // No longer used in the new layout.
  }

  openCalibrationPanel(prefs = {}) {
    if (!this.calibrationOverlay) {
      return;
    }

    this.calibrationOverlay.classList.remove('hidden');
    this.calibrationPanel?.classList.remove('hidden');
    this.calibrationProgressPanel?.classList.add('hidden');
    this.calibrationResultPanel?.classList.add('hidden');

    const heightInput = document.getElementById('calibration-height');
    const shankInput = document.getElementById('calibration-shank');
    const blockedMessage = document.getElementById('calibration-blocked-message');

    if (heightInput && Number.isFinite(prefs.heightCm)) {
      heightInput.value = String(prefs.heightCm);
    }
    if (shankInput && Number.isFinite(prefs.shankLengthCm)) {
      shankInput.value = String(prefs.shankLengthCm);
    }

    const blocked = Boolean(prefs.blocked);
    if (blockedMessage) {
      blockedMessage.textContent = prefs.reason || 'Connect a BLE sensor before calibrating.';
      blockedMessage.classList.toggle('hidden', !blocked);
    }
    document.getElementById('btn-calibration-start')?.toggleAttribute('disabled', blocked);
  }

  closeCalibrationPanel() {
    this.calibrationOverlay?.classList.add('hidden');
  }

  showCalibrationProgress(percent, message) {
    this.calibrationPanel?.classList.add('hidden');
    this.calibrationProgressPanel?.classList.remove('hidden');
    this.calibrationResultPanel?.classList.add('hidden');

    if (this.calibrationProgressBar) {
      this.calibrationProgressBar.style.width = `${Math.max(0, Math.min(100, percent))}%`;
    }
    if (this.calibrationProgressText) {
      this.calibrationProgressText.textContent = message;
    }
  }

  showCalibrationResult(result) {
    this.calibrationPanel?.classList.add('hidden');
    this.calibrationProgressPanel?.classList.add('hidden');
    this.calibrationResultPanel?.classList.remove('hidden');

    if (this.calibrationResultText) {
      this.calibrationResultText.textContent = result.message;
      this.calibrationResultText.className = result.ok
        ? 'calibration-result-text success'
        : 'calibration-result-text error';
    }

    this.setCalibrationBadge(result.ok, result.summary);
  }

  setCalibrationBadge(isCalibrated, summary = '') {
    if (!this.calibrationBadge) {
      return;
    }

    this.calibrationBadge.className = isCalibrated
      ? 'calibration-badge calibrated'
      : 'calibration-badge pending';
    this.calibrationBadge.textContent = isCalibrated
      ? `Calibrated${summary ? ` · ${summary}` : ''}`
      : 'Not calibrated';
  }

  _animateValue(key, newValue) {
    const el = this.paramElements[key];
    if (!el) {
      return;
    }

    const oldValue = el.textContent;
    if (oldValue === newValue) {
      return;
    }

    el.textContent = newValue;
    el.classList.add('updating');

    if (this.animationTimers[key]) {
      clearTimeout(this.animationTimers[key]);
    }

    this.animationTimers[key] = setTimeout(() => {
      el.classList.remove('updating');
    }, 400);
  }
}
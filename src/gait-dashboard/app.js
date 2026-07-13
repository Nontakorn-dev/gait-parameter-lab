import { generateWalkingData, streamDemoData } from './data/demoDataGenerator.js';
import { createGaitProcessorClient } from '../gait/gaitProcessorClient.js';
import { Dashboard } from './ui/dashboard.js';
import { ChartsManager } from './ui/charts.js';
import {
  estimateShankLengthM,
  CALIBRATION_DURATION_MS,
} from '../gait/gaitCalibration.js';
import {
  GAIT_CALIBRATION_WARMSTART_SAMPLES,
  finishGaitCalibrationCapture,
} from '../gait/gaitCalibrationCapture.js';
import {
  readGaitCalibrationPrefs,
  writeGaitCalibrationPrefs,
} from '../gait/gaitCalibrationStore.js';
import {
  getRealtimeSensorLabel,
  normalizeRealtimeSensorSample,
} from '../gateway/realtimeSensorUtils.js';
import {
  GAIT_ANALYSIS_INTERVAL_MS,
  GAIT_MIN_ANALYSIS_SAMPLES,
  GAIT_SENSOR_STALE_AFTER_MS,
} from '../gait/gaitRuntimeConfig.js';
import { sortGaitSensorStates, resolveVisibleShankSensorKeys } from '../gait/gaitSensorSelection.js';
import {
  TRANSPORT_MODE_WEB_BLUETOOTH,
  connectBrowserBleDevice,
  createRealtimeConnection,
  getBrowserBleDevicesSnapshot,
  getRealtimeTransportMode,
  resetBrowserBleDevices,
  setRealtimeTransportMode,
  supportsRememberedBrowserBleDevices,
} from '../gateway/realtimeTransport.js';

const DEMO_PREFILL_SAMPLES = 400;
const DASHBOARD_BLE_SLOTS = ['LEFT_SHANK', 'RIGHT_SHANK'];
const REMOTE_RETRY_DELAY_MS = 1500;

function buildDemoSample(sample) {
  return {
    ...sample,
    sensorKey: 'demo-shank',
    side: 'R',
    sensorMount: 'shank',
    name: 'Demo Sensor',
  };
}

class GatewayStreamClient {
  constructor() {
    this.ws = null;
    this.connected = false;
    this.listeners = [];
    this.statusListeners = [];
  }

  onData(callback) {
    this.listeners.push(callback);
  }

  onStatus(callback) {
    this.statusListeners.push(callback);
  }

  async connect() {
    if (this.connected && this.ws?.readyState === WebSocket.OPEN) {
      return;
    }

    this.disconnect();
    this._emitStatus('connecting');

    await new Promise((resolve, reject) => {
      this.ws = createRealtimeConnection();

      this.ws.onopen = () => {
        this.connected = true;
        this._emitStatus('connected');
        resolve();
      };

      this.ws.onmessage = (event) => {
        this._handleMessage(event);
      };

      this.ws.onerror = (error) => {
        this.connected = false;
        this._emitStatus('error');
        reject(error);
      };

      this.ws.onclose = () => {
        this.connected = false;
        this._emitStatus('disconnected');
      };
    });
  }

  disconnect() {
    if (this.ws) {
      this.ws.onopen = null;
      this.ws.onmessage = null;
      this.ws.onerror = null;
      this.ws.onclose = null;
    }

    if (this.ws?.readyState === WebSocket.OPEN || this.ws?.readyState === WebSocket.CONNECTING) {
      this.ws.close();
    }

    this.ws = null;
    this.connected = false;
    this._emitStatus('disconnected');
  }

  _handleMessage(event) {
    try {
      const message = JSON.parse(event.data);
      if (message.type !== 'DATA') {
        return;
      }

      const sample = normalizeRealtimeSensorSample(message.payload || {});
      if (!sample) {
        return;
      }

      this.listeners.forEach((callback) => callback(sample));
    } catch {
      this._emitStatus('error');
    }
  }

  _emitStatus(status) {
    this.statusListeners.forEach((callback) => callback(status));
  }
}

export class GaitLabDashboardApp {
  constructor(options = {}) {
    this.options = {
      teardownTransportOnDestroy: true,
      streamFactory: null,
      broadcastPublisherFactory: null,
      initialMode: 'auto',
      ...options,
    };

    this.stream = typeof this.options.streamFactory === 'function'
      ? this.options.streamFactory()
      : new GatewayStreamClient();
    this.broadcastPublisher = typeof this.options.broadcastPublisherFactory === 'function'
      ? this.options.broadcastPublisherFactory()
      : null;
    this.sensorProcessors = new Map();
    this.availableSensors = new Map();
    this.visibleSensorKeys = [];
    this.dashboard = new Dashboard();
    this.charts = new ChartsManager();

    this.mode = 'demo';
    this.demoStream = null;
    this.analysisTimer = null;
    this.loadingTimer = null;
    this.remoteRetryTimer = null;
    this.calibrationActive = false;
    this.calibrationBuckets = new Map();
    this.calibrationTimer = null;
    this.calibrationProgressTimer = null;
    this.pendingShankLengthM = estimateShankLengthM();
  }

  init() {
    this.dashboard.init();
    this.charts.init();

    this.stream.onStatus((status) => {
      this.dashboard.updateStatus(status);
      void this._publishBroadcastStatus(status);
    });

    this.stream.onData((sample) => {
      this._ingestSensorSample(sample);
      this._enqueueBroadcastSample(sample);
    });

    this._bindEvents();
    this._startInitialMode();
    this.dashboard.setCalibrationBadge(false);

    if (this.loadingTimer) {
      window.clearTimeout(this.loadingTimer);
    }
    this.loadingTimer = window.setTimeout(() => {
      document.getElementById('loading-overlay')?.classList.add('hidden');
      this.loadingTimer = null;
    }, 800);
  }

  async startDemo() {
    this.stopAll();
    await resetBrowserBleDevices();
    setRealtimeTransportMode(TRANSPORT_MODE_WEB_BLUETOOTH);
    this.mode = 'demo';
    this.dashboard.setMode('demo');
    this.dashboard.updateStatus('demo');

    const { samples } = generateWalkingData({ numStrides: 8 });
    const prefillSamples = samples.slice(0, DEMO_PREFILL_SAMPLES);
    prefillSamples.forEach((sample) => {
      this._ingestSensorSample(buildDemoSample(sample));
    });
    this._analyzeReadyProcessors();

    this.demoStream = streamDemoData((sample) => {
      this._ingestSensorSample(buildDemoSample(sample));
    });

    this.analysisTimer = window.setInterval(() => {
      this._analyzeReadyProcessors();
    }, GAIT_ANALYSIS_INTERVAL_MS);
  }

  async startRemote(options = {}) {
    const { suppressErrorLog = false } = options;

    try {
      this.stopAll();
      this.mode = 'remote-broadcast';
      this.dashboard.setMode(this.mode);
      this.dashboard.updateStatus('connecting');

      await this.stream.connect();

      this.analysisTimer = window.setInterval(() => {
        this._analyzeReadyProcessors();
      }, GAIT_ANALYSIS_INTERVAL_MS);
    } catch (error) {
      if (!suppressErrorLog) {
        console.error('Remote live stream failed:', error);
      }
      this.dashboard.updateStatus('error');

      if (!this.remoteRetryTimer && this.mode === 'remote-broadcast') {
        this.remoteRetryTimer = window.setTimeout(() => {
          this.remoteRetryTimer = null;
          void this.startRemote({ suppressErrorLog: true });
        }, REMOTE_RETRY_DELAY_MS);
      }
    }
  }

  async startLive(options = {}) {
    const {
      fallbackToDemo = true,
      suppressErrorLog = false,
      transportMode = TRANSPORT_MODE_WEB_BLUETOOTH,
      pairBrowserBle = false,
      preferKnownDevice = false,
      pickerFallback = true,
    } = options;

    try {
      this.stopAll();
      setRealtimeTransportMode(transportMode);

      this.mode = 'browser-ble';
      this.dashboard.setMode(this.mode);

      await this._connectBroadcastPublisher();
      await this.stream.connect();

      if (pairBrowserBle) {
        if (transportMode === TRANSPORT_MODE_WEB_BLUETOOTH && this._hasConnectedDashboardSensor()) {
          this.dashboard.updateStatus('connected');
        } else {
          this.dashboard.updateStatus('pairing');
          await this._connectPreferredBrowserBleSensors({ preferKnownDevice, pickerFallback });
          this.dashboard.updateStatus('connected');
        }
      }

      this.analysisTimer = window.setInterval(() => {
        this._analyzeReadyProcessors();
      }, GAIT_ANALYSIS_INTERVAL_MS);
    } catch (error) {
      if (!suppressErrorLog) {
        console.error('Realtime stream failed:', error);
      }
      this.dashboard.updateStatus('error');

      if (fallbackToDemo) {
        window.setTimeout(() => {
          void this.startDemo();
        }, 500);
      }
    }
  }

  async startBrowserBle(options = {}) {
    const {
      fallbackToDemo = false,
      suppressErrorLog = false,
      preferKnownDevice = true,
      pickerFallback = true,
    } = options;

    await this.startLive({
      fallbackToDemo,
      suppressErrorLog,
      transportMode: TRANSPORT_MODE_WEB_BLUETOOTH,
      pairBrowserBle: true,
      preferKnownDevice,
      pickerFallback,
    });
  }

  stopAll() {
    this._clearCalibrationTimers();
    this.calibrationActive = false;
    this.calibrationBuckets.clear();

    if (this.demoStream) {
      this.demoStream.stop();
      this.demoStream = null;
    }

    if (this.analysisTimer) {
      window.clearInterval(this.analysisTimer);
      this.analysisTimer = null;
    }

    if (this.remoteRetryTimer) {
      window.clearTimeout(this.remoteRetryTimer);
      this.remoteRetryTimer = null;
    }

    if (typeof this.stream.disconnect === 'function') {
      this.stream.disconnect();
    }

    if (this.broadcastPublisher?.disconnect) {
      void this.broadcastPublisher.disconnect();
    }

    this._destroyAllSensorProcessors();
    this.availableSensors.clear();
    this.visibleSensorKeys = [];
    this.dashboard.updateSensorSummary([]);
    this.dashboard.updateEvents([]);
    this.charts.update([]);
  }

  destroy() {
    this.stopAll();
    if (this.loadingTimer) {
      window.clearTimeout(this.loadingTimer);
      this.loadingTimer = null;
    }
    if (this.options.teardownTransportOnDestroy) {
      void resetBrowserBleDevices();
      setRealtimeTransportMode(TRANSPORT_MODE_WEB_BLUETOOTH);
    }
    this.charts.destroy();
  }

  _startInitialMode() {
    if (this.options.initialMode === 'remote-broadcast') {
      void this.startRemote({ suppressErrorLog: true });
      return;
    }

    if (getRealtimeTransportMode() === TRANSPORT_MODE_WEB_BLUETOOTH) {
      if (this._hasConnectedDashboardSensor()) {
        void this.startLive({
          fallbackToDemo: true,
          suppressErrorLog: true,
          transportMode: TRANSPORT_MODE_WEB_BLUETOOTH,
          pairBrowserBle: false,
        });
        return;
      }

      if (supportsRememberedBrowserBleDevices()) {
        void this.startBrowserBle({
          fallbackToDemo: true,
          suppressErrorLog: true,
          preferKnownDevice: true,
          pickerFallback: false,
        });
        return;
      }
    }

    void this.startDemo();
  }

  _hasConnectedDashboardSensor() {
    return getBrowserBleDevicesSnapshot().some((device) => (
      DASHBOARD_BLE_SLOTS.includes(device.positionId) && device.status === 'connected'
    ));
  }

  async _connectPreferredBrowserBleSensors({ preferKnownDevice, pickerFallback }) {
    let connectedCount = 0;
    let lastError = null;

    for (const positionId of DASHBOARD_BLE_SLOTS) {
      try {
        await connectBrowserBleDevice(positionId, {
          preferKnownDevice,
          pickerFallback,
        });
        connectedCount += 1;
      } catch (error) {
        lastError = error;
      }
    }

    if (!connectedCount) {
      throw lastError || new Error('No shank sensor could be connected for the dashboard.');
    }
  }

  _destroyAllSensorProcessors() {
    this.sensorProcessors.forEach((state) => {
      state.processor?.destroy?.();
    });
    this.sensorProcessors.clear();
  }

  _analyzeReadyProcessors() {
    this.sensorProcessors.forEach((state) => {
      if (state.sampleCount >= GAIT_MIN_ANALYSIS_SAMPLES) {
        state.processor.analyze();
      }
    });
  }

  _createProcessorState(displayKey, sample) {
    const processor = createGaitProcessorClient();
    const state = {
      key: displayKey,
      label: getRealtimeSensorLabel(sample),
      side: sample.side || null,
      sensorKey: sample.sensorKey,
      sensorMount: sample.sensorMount || null,
      sampleCount: 0,
      lastSeenAt: 0,
      latestParams: null,
      latestProcessedData: null,
      processor,
    };

    processor.onParams(({ params, processedData }) => {
      state.latestParams = params;
      state.latestProcessedData = processedData;
      this._renderDashboard();
    });

    this.sensorProcessors.set(displayKey, state);
    return state;
  }

  _getDisplayKey(sample) {
    return sample.side || sample.sensorKey;
  }

  _resolveVisibleSensorKeys() {
    return resolveVisibleShankSensorKeys(Array.from(this.availableSensors.values()), {
      staleAfterMs: GAIT_SENSOR_STALE_AFTER_MS,
    });
  }

  _syncVisibleProcessors() {
    const nextVisibleDisplayKeys = new Set(
      this.visibleSensorKeys.map((sensorKey) => {
        const sensor = this.availableSensors.get(sensorKey);
        return sensor?.side || sensorKey;
      }),
    );

    this.sensorProcessors.forEach((state, displayKey) => {
      if (!nextVisibleDisplayKeys.has(displayKey)) {
        state.processor?.destroy?.();
        this.sensorProcessors.delete(displayKey);
      }
    });
  }

  _ingestSensorSample(sample) {
    const normalizedSample = normalizeRealtimeSensorSample(sample);
    if (!normalizedSample) {
      return;
    }

    this.availableSensors.set(normalizedSample.sensorKey, {
      sensorKey: normalizedSample.sensorKey,
      side: normalizedSample.side,
      sensorMount: normalizedSample.sensorMount,
      label: getRealtimeSensorLabel(normalizedSample),
      lastSeenAt: Date.now(),
    });

    const nextVisibleSensorKeys = this._resolveVisibleSensorKeys();
    const visibilityChanged = nextVisibleSensorKeys.join('|') !== this.visibleSensorKeys.join('|');

    this.visibleSensorKeys = nextVisibleSensorKeys;
    if (visibilityChanged) {
      this._syncVisibleProcessors();
    }

    this.dashboard.updateSensorSummary(this._buildSensorSummary());

    if (!this.visibleSensorKeys.includes(normalizedSample.sensorKey)) {
      return;
    }

    const displayKey = this._getDisplayKey(normalizedSample);
    const state = this.sensorProcessors.get(displayKey) || this._createProcessorState(displayKey, normalizedSample);

    if (this.calibrationActive) {
      this._collectCalibrationSample(normalizedSample);
    }

    state.label = getRealtimeSensorLabel(normalizedSample);
    state.side = normalizedSample.side || null;
    state.sensorKey = normalizedSample.sensorKey;
    state.sensorMount = normalizedSample.sensorMount || null;
    state.sampleCount += 1;
    state.lastSeenAt = Date.now();
    state.processor.addSample(normalizedSample);
  }

  _buildVisibleSensorEntries() {
    return Array.from(this.sensorProcessors.values())
      .filter((state) => state.latestProcessedData)
      .sort(sortGaitSensorStates)
      .map((state) => ({
        key: state.key,
        label: state.label,
        side: state.side,
        params: state.latestParams,
        processedData: state.latestProcessedData,
      }));
  }

  _buildDashboardEvents(entries) {
    return entries
      .flatMap((entry) => (entry.processedData?.events || []).map((event) => ({
        ...event,
        side: entry.side,
      })))
      .sort((left, right) => (left.time || 0) - (right.time || 0));
  }

  _buildSensorSummary() {
    return this.visibleSensorKeys
      .map((sensorKey) => this.availableSensors.get(sensorKey))
      .filter(Boolean)
      .sort(sortGaitSensorStates)
      .map((sensor) => ({
        label: sensor.label,
        side: sensor.side,
        sensorMount: sensor.sensorMount,
      }));
  }

  _renderDashboard() {
    const entries = this._buildVisibleSensorEntries();

    if (!entries.length) {
      this.dashboard.updateEvents([]);
      this.charts.update([]);
      return;
    }

    if (entries.length === 1) {
      this.dashboard.updateParams(entries[0].params);
    } else {
      this.dashboard.updateMultiSensorParams(entries);
    }

    this.dashboard.updateEvents(this._buildDashboardEvents(entries));
    this.charts.update(entries);
  }

  _bindEvents() {
    document.getElementById('mode-toggle')?.addEventListener('click', () => {
      if (this.mode === 'demo') {
        void this.startBrowserBle();
      } else {
        void this.startDemo();
      }
    });

    document.getElementById('btn-connect')?.addEventListener('click', () => {
      if (this.mode === 'browser-ble') {
        void this.startDemo();
      } else {
        void this.startBrowserBle();
      }
    });

    document.getElementById('btn-demo')?.addEventListener('click', () => {
      void this.startDemo();
    });

    document.getElementById('btn-calibrate')?.addEventListener('click', () => {
      this.openCalibration();
    });

    document.getElementById('btn-calibration-cancel')?.addEventListener('click', () => {
      this.dashboard.closeCalibrationPanel();
    });

    document.getElementById('btn-calibration-start')?.addEventListener('click', () => {
      void this.beginCalibrationCapture();
    });

    document.getElementById('btn-calibration-done')?.addEventListener('click', () => {
      this.dashboard.closeCalibrationPanel();
    });
  }

  _readCalibrationPrefs() {
    return readGaitCalibrationPrefs();
  }

  _writeCalibrationPrefs(prefs) {
    writeGaitCalibrationPrefs(prefs);
  }

  openCalibration() {
    if (this.mode === 'demo') {
      this.dashboard.openCalibrationPanel({
        blocked: true,
        reason: 'Connect a BLE sensor before calibrating for best accuracy.',
        ...this._readCalibrationPrefs(),
      });
      return;
    }

    if (!this.visibleSensorKeys.length) {
      this.dashboard.openCalibrationPanel({
        blocked: true,
        reason: 'No shank sensor is streaming yet. Connect a sensor and wait for data.',
        ...this._readCalibrationPrefs(),
      });
      return;
    }

    this.dashboard.openCalibrationPanel(this._readCalibrationPrefs());
  }

  async beginCalibrationCapture() {
    const heightCm = Number.parseFloat(document.getElementById('calibration-height')?.value);
    const shankLengthCm = Number.parseFloat(document.getElementById('calibration-shank')?.value);
    this._writeCalibrationPrefs({
      heightCm: Number.isFinite(heightCm) ? heightCm : null,
      shankLengthCm: Number.isFinite(shankLengthCm) ? shankLengthCm : null,
    });
    this.pendingShankLengthM = estimateShankLengthM({
      heightCm: Number.isFinite(heightCm) ? heightCm : null,
      shankLengthCm: Number.isFinite(shankLengthCm) ? shankLengthCm : null,
    });

    this.calibrationActive = true;
    this.calibrationBuckets = new Map();
    this._sendTransportCommand('CALIBRATE');
    this._pauseAnalysisTimer();

    this.sensorProcessors.forEach((state) => {
      state.processor.reset();
      state.sampleCount = 0;
      state.latestParams = null;
      state.latestProcessedData = null;
    });
    this.dashboard.updateEvents([]);
    this.charts.update([]);

    const startedAt = Date.now();
    this.dashboard.showCalibrationProgress(0, 'Stand still and keep the sensor stable...');

    this.calibrationProgressTimer = window.setInterval(() => {
      const elapsed = Date.now() - startedAt;
      const percent = (elapsed / CALIBRATION_DURATION_MS) * 100;
      const remaining = Math.max(0, Math.ceil((CALIBRATION_DURATION_MS - elapsed) / 1000));
      this.dashboard.showCalibrationProgress(
        percent,
        `Collecting IMU samples... ${remaining}s remaining`,
      );
    }, 100);

    this.calibrationTimer = window.setTimeout(() => {
      this._finishCalibration();
    }, CALIBRATION_DURATION_MS);
  }

  _collectCalibrationSample(sample) {
    const bucket = this.calibrationBuckets.get(sample.sensorKey) || [];
    bucket.push(sample);
    this.calibrationBuckets.set(sample.sensorKey, bucket);
  }

  _finishCalibration() {
    this._clearCalibrationTimers();
    this.calibrationActive = false;

    const sensorKeys = this.visibleSensorKeys.length
      ? this.visibleSensorKeys
      : Array.from(this.calibrationBuckets.keys());
    const outcome = finishGaitCalibrationCapture(this.calibrationBuckets, {
      preferredSensorKeys: sensorKeys,
      shankLengthM: this.pendingShankLengthM,
      persist: true,
    });
    const results = outcome.results || [];

    let successCount = 0;
    const failureReasons = outcome.failureReasons || [];

    for (const state of this.sensorProcessors.values()) {
      const result = results.find((item) => item.sensorKey === state.sensorKey);
      if (!result) {
        continue;
      }

      if (!result.profile.ok) {
        failureReasons.push(`${state.label}: ${result.profile.reason}`);
        continue;
      }

      state.processor.reset();
      state.processor.applyCalibration(result.profile);
      const warmStartSamples = result.samples.slice(-GAIT_CALIBRATION_WARMSTART_SAMPLES);
      state.processor.warmStartFromSamples(warmStartSamples);
      state.sampleCount = warmStartSamples.length;
      state.latestParams = null;
      state.latestProcessedData = null;
      successCount += 1;
    }

    this._resumeAnalysisTimer();

    if (successCount > 0) {
      this.dashboard.showCalibrationResult({
        ok: true,
        message: outcome.message || `Calibration complete for ${successCount} sensor(s). You can start walking.`,
        summary: outcome.summary || '',
      });
      this.dashboard.setCalibrationBadge(true, outcome.summary || '');
      return;
    }

    const message = failureReasons.length
      ? failureReasons.join(' ')
      : (outcome.message || 'Calibration failed. Stand still on a flat surface and try again.');
    this.dashboard.showCalibrationResult({
      ok: false,
      message,
      summary: '',
    });
    this.dashboard.setCalibrationBadge(false);
  }

  _clearCalibrationTimers() {
    if (this.calibrationTimer) {
      window.clearTimeout(this.calibrationTimer);
      this.calibrationTimer = null;
    }
    if (this.calibrationProgressTimer) {
      window.clearInterval(this.calibrationProgressTimer);
      this.calibrationProgressTimer = null;
    }
  }

  _pauseAnalysisTimer() {
    if (this.analysisTimer) {
      window.clearInterval(this.analysisTimer);
      this.analysisTimer = null;
    }
  }

  _resumeAnalysisTimer() {
    if (this.analysisTimer) {
      return;
    }

    this.analysisTimer = window.setInterval(() => {
      this._analyzeReadyProcessors();
    }, GAIT_ANALYSIS_INTERVAL_MS);
  }

  _sendTransportCommand(command) {
    if (typeof this.stream?.ws?.send === 'function') {
      this.stream.ws.send(command);
    }
  }

  async _connectBroadcastPublisher() {
    if (!this.broadcastPublisher?.connect) {
      return;
    }

    try {
      await this.broadcastPublisher.connect();
    } catch (error) {
      console.warn('Live gait broadcast is unavailable:', error);
    }
  }

  _enqueueBroadcastSample(sample) {
    if (!this.broadcastPublisher?.enqueueSample) {
      return;
    }

    try {
      this.broadcastPublisher.enqueueSample(sample);
    } catch (error) {
      console.warn('Failed to enqueue live gait sample:', error);
    }
  }

  async _publishBroadcastStatus(status) {
    if (!this.broadcastPublisher?.publishStatus) {
      return;
    }

    try {
      await this.broadcastPublisher.publishStatus(status);
    } catch (error) {
      console.warn('Failed to publish live gait status:', error);
    }
  }
}

export function createGaitLabDashboardApp(options) {
  return new GaitLabDashboardApp(options);
}
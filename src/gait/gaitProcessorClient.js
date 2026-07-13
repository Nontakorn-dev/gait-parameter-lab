import { GaitProcessor } from './gaitProcessor.js'
import { applyStoredGaitCalibration } from './gaitCalibrationStore.js'

class LocalGaitProcessorClient {
  constructor(options = {}) {
    this.processor = new GaitProcessor(options)
  }

  onParams(callback) {
    return this.processor.onParams(callback)
  }

  addSample(sample) {
    this.processor.addSample(sample)
  }

  analyze() {
    this.processor.analyze()
  }

  applyCalibration(profile) {
    this.processor.applyCalibration(profile)
  }

  warmStartFromSamples(samples) {
    this.processor.warmStartFromSamples(samples)
  }

  getCalibration() {
    return this.processor.getCalibration()
  }

  reset() {
    this.processor.reset()
  }

  destroy() {
    this.processor.reset()
    this.processor.paramListeners = []
  }
}

class WorkerGaitProcessorClient {
  constructor(options = {}) {
    this.listeners = new Set()
    this.worker = new Worker(new URL('./gaitProcessor.worker.js', import.meta.url), {
      type: 'module',
    })
    this.worker.postMessage({ type: 'configure', payload: options })
    this.worker.onmessage = (event) => {
      const { type, payload } = event.data || {}
      if (type === 'analysis') {
        this.listeners.forEach((listener) => listener(payload))
      }
    }
    this.worker.onerror = (error) => {
      console.error('Gait processor worker failed:', error)
    }
  }

  onParams(callback) {
    this.listeners.add(callback)
    return () => this.listeners.delete(callback)
  }

  addSample(sample) {
    this.worker.postMessage({ type: 'add-sample', payload: sample })
  }

  analyze() {
    this.worker.postMessage({ type: 'analyze' })
  }

  applyCalibration(profile) {
    this.worker.postMessage({ type: 'apply-calibration', payload: profile })
  }

  warmStartFromSamples(samples) {
    this.worker.postMessage({ type: 'warm-start', payload: samples })
  }

  getCalibration() {
    return null
  }

  reset() {
    this.worker.postMessage({ type: 'reset' })
  }

  destroy() {
    this.listeners.clear()
    this.worker.postMessage({ type: 'destroy' })
    this.worker.terminate()
  }
}

export function createGaitProcessorClient(options = {}) {
  if (typeof Worker === 'undefined' || options.forceLocal) {
    return new LocalGaitProcessorClient(options)
  }

  try {
    return new WorkerGaitProcessorClient(options)
  } catch (error) {
    console.warn('Falling back to main-thread gait processing:', error)
    return new LocalGaitProcessorClient(options)
  }
}

export function createCalibratedGaitProcessorClient(sensorKey, options = {}) {
  const client = createGaitProcessorClient(options)
  if (sensorKey && options.applyCalibration !== false) {
    applyStoredGaitCalibration(client, sensorKey)
  }
  return client
}

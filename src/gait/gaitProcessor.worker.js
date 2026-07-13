import { GaitProcessor } from './gaitProcessor.js'

let processor = new GaitProcessor()

processor.onParams((payload) => {
  self.postMessage({
    type: 'analysis',
    payload,
  })
})

self.onmessage = (event) => {
  const { type, payload } = event.data || {}

  if (type === 'configure') {
    return
  }

  if (type === 'apply-calibration') {
    processor.applyCalibration(payload || {})
    return
  }

  if (type === 'warm-start') {
    processor.warmStartFromSamples(Array.isArray(payload) ? payload : [])
    return
  }

  if (type === 'add-sample') {
    processor.addSample(payload)
    return
  }

  if (type === 'analyze') {
    processor.analyze()
    return
  }

  if (type === 'reset') {
    processor.reset()
    return
  }

  if (type === 'destroy') {
    self.close()
  }
}

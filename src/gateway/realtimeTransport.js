import { applyAxisMap } from './realtimeSensorUtils.js'

const TRANSPORT_MODE_STORAGE_KEY = 'derndee:realtime-transport-mode'
const DEVICE_ASSIGNMENTS_STORAGE_KEY = 'derndee:browser-ble-assignments'
const TRANSPORT_MODE_WEB_BLUETOOTH = 'web-bluetooth'
const BLE_SERVICE_UUID = '4fafc201-1fb5-459e-8fcc-c5c9c331914b'
const BLE_CHARACTERISTIC_UUID = 'beb5483e-36e1-4688-b7f5-ea07361b26a8'
const BLE_DEVICE_PREFIX = 'DernDee'
const CALIBRATION_DELAY_MS = 3000

const BODY_POSITIONS = [
  { id: 'PELVIS', name: 'Chest', order: 1, deviceId: 'DernDee_Chest' },
  { id: 'LEFT_THIGH', name: 'Left Thigh', order: 2, deviceId: 'DernDee_L_Thigh' },
  { id: 'RIGHT_THIGH', name: 'Right Thigh', order: 3, deviceId: 'DernDee_R_Thigh' },
  { id: 'LEFT_SHANK', name: 'Left Shank', order: 4, deviceId: 'DernDee_L_Shank' },
  { id: 'RIGHT_SHANK', name: 'Right Shank', order: 5, deviceId: 'DernDee_R_Shank' },
]

function getStoredMode() {
  if (typeof window === 'undefined') {
    return TRANSPORT_MODE_WEB_BLUETOOTH
  }

  const storedMode = window.sessionStorage.getItem(TRANSPORT_MODE_STORAGE_KEY)
  return storedMode === TRANSPORT_MODE_WEB_BLUETOOTH ? storedMode : TRANSPORT_MODE_WEB_BLUETOOTH
}

function setStoredMode(mode) {
  if (typeof window === 'undefined') {
    return
  }

  window.sessionStorage.setItem(TRANSPORT_MODE_STORAGE_KEY, mode)
}

function getStoredAssignments() {
  if (typeof window === 'undefined') {
    return {}
  }

  try {
    const raw = window.sessionStorage.getItem(DEVICE_ASSIGNMENTS_STORAGE_KEY)
    return raw ? JSON.parse(raw) : {}
  } catch {
    return {}
  }
}

function setStoredAssignments(assignments) {
  if (typeof window === 'undefined') {
    return
  }

  window.sessionStorage.setItem(DEVICE_ASSIGNMENTS_STORAGE_KEY, JSON.stringify(assignments || {}))
}

function calculateCrc8(bytes) {
  let crc = 0x00
  for (const byte of bytes) {
    crc ^= byte
    for (let index = 0; index < 8; index += 1) {
      if (crc & 0x80) {
        crc = ((crc << 1) ^ 0x07) & 0xff
      } else {
        crc = (crc << 1) & 0xff
      }
    }
  }
  return crc
}

function decodePacket(buffer) {
  const bytes = buffer instanceof Uint8Array ? buffer : new Uint8Array(buffer)
  if (bytes.byteLength !== 22) {
    return null
  }

  const dataView = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
  const magic = dataView.getUint16(0, true)
  if (magic !== 0xaa55) {
    return null
  }

  const crc8 = dataView.getUint8(21)
  if (calculateCrc8(bytes.slice(0, 21)) !== crc8) {
    return null
  }

  return {
    raw_accel: [
      dataView.getInt16(9, true),
      dataView.getInt16(11, true),
      dataView.getInt16(13, true),
    ],
    raw_gyro: [
      dataView.getInt16(15, true),
      dataView.getInt16(17, true),
      dataView.getInt16(19, true),
    ],
    timestamp_ms: dataView.getUint32(5, true) / 1000,
    seq: dataView.getUint16(3, true),
    // offset 2 = IMU packet format version (hardcoded 1 ในเฟิร์มแวร์) ไม่ใช่เวอร์ชันเฟิร์มแวร์
    packetVersion: dataView.getUint8(2),
  }
}

function inferSensorMount(senderName = '', positionId = '') {
  if (positionId === 'LEFT_SHANK' || positionId === 'RIGHT_SHANK') {
    return 'shank'
  }

  if (positionId === 'LEFT_THIGH' || positionId === 'RIGHT_THIGH') {
    return 'thigh'
  }

  if (positionId === 'PELVIS') {
    return 'pelvis'
  }

  return senderName.includes('Shank') ? 'shank' : 'foot'
}

function findPositionIdForBleName(name = '') {
  const normalizedName = String(name || '').trim()
  if (!normalizedName) {
    return null
  }

  const exactMatch = BODY_POSITIONS.find((position) => position.deviceId === normalizedName)
  if (exactMatch) {
    return exactMatch.id
  }

  const upperName = normalizedName.toUpperCase()
  if (upperName.includes('CHEST') || upperName.includes('PELVIS')) {
    return 'PELVIS'
  }
  if (upperName.includes('L_THIGH') || (upperName.includes('LEFT') && upperName.includes('THIGH'))) {
    return 'LEFT_THIGH'
  }
  if (upperName.includes('R_THIGH') || (upperName.includes('RIGHT') && upperName.includes('THIGH'))) {
    return 'RIGHT_THIGH'
  }
  if (upperName.includes('L_SHANK') || (upperName.includes('LEFT') && upperName.includes('SHANK'))) {
    return 'LEFT_SHANK'
  }
  if (upperName.includes('R_SHANK') || (upperName.includes('RIGHT') && upperName.includes('SHANK'))) {
    return 'RIGHT_SHANK'
  }
  // Legacy firmware names still map to shank slots.
  if (upperName.includes('L_FOOT') || (upperName.includes('LEFT') && upperName.includes('FOOT'))) {
    return 'LEFT_SHANK'
  }
  if (upperName.includes('R_FOOT') || (upperName.includes('RIGHT') && upperName.includes('FOOT'))) {
    return 'RIGHT_SHANK'
  }

  return null
}

function inferSide(senderName = '', positionId = '') {
  if (positionId.startsWith('LEFT_')) {
    return 'L'
  }

  if (positionId.startsWith('RIGHT_')) {
    return 'R'
  }

  if (senderName.includes('_L_') || senderName.includes('Left') || senderName.includes('L_Shank')) {
    return 'L'
  }
  return 'R'
}

function createInitialDevices() {
  return BODY_POSITIONS.map((position) => ({
    positionId: position.id,
    positionName: position.name,
    order: position.order,
    expectedDeviceId: position.deviceId,
    deviceId: null,
    deviceName: null,
    status: 'disconnected',
    battery: null,
    signal: null,
    packetCount: 0,
    lastPacketSize: null,
  }))
}

class BrowserBleSocket {
  constructor(manager) {
    this.manager = manager
    this.readyState = WebSocket.CONNECTING
    this.onopen = null
    this.onclose = null
    this.onerror = null
    this.onmessage = null
    this.closed = false
  }

  emitOpen() {
    if (this.closed) {
      return
    }
    this.readyState = WebSocket.OPEN
    this.onopen?.()
  }

  emitClose() {
    if (this.closed) {
      return
    }
    this.closed = true
    this.readyState = WebSocket.CLOSED
    this.onclose?.()
  }

  emitError(error) {
    if (this.closed) {
      return
    }
    this.onerror?.(error)
  }

  emitMessage(message) {
    if (this.closed || this.readyState !== WebSocket.OPEN) {
      return
    }
    this.onmessage?.({ data: JSON.stringify(message) })
  }

  send(rawMessage) {
    if (this.closed || this.readyState !== WebSocket.OPEN) {
      return
    }
    this.manager.handleClientSend(rawMessage)
  }

  close() {
    if (this.closed) {
      return
    }
    this.manager.unregisterSocket(this)
    this.emitClose()
  }
}

class BrowserBleManager {
  constructor() {
    this.sockets = new Set()
    this.messageListeners = new Set()
    this.deviceStateListeners = new Set()
    this.modeListeners = new Set()
    this.connections = new Map()
    this.devices = createInitialDevices()
    this.deviceAssignments = getStoredAssignments()
    this.mode = getStoredMode()
    this.activeSession = null
    this.calibrationTimer = null
  }

  isWebBluetoothSupported() {
    return typeof navigator !== 'undefined' && typeof navigator.bluetooth?.requestDevice === 'function'
  }

  supportsRememberedDevices() {
    return typeof navigator !== 'undefined' && typeof navigator.bluetooth?.getDevices === 'function'
  }

  getMode() {
    return this.mode
  }

  setMode(nextMode) {
    void nextMode
    if (this.mode === TRANSPORT_MODE_WEB_BLUETOOTH) {
      return
    }

    this.mode = TRANSPORT_MODE_WEB_BLUETOOTH
    setStoredMode(TRANSPORT_MODE_WEB_BLUETOOTH)
    this.notifyModeListeners()
  }

  subscribeToMode(listener) {
    this.modeListeners.add(listener)
    listener(this.mode)
    return () => {
      this.modeListeners.delete(listener)
    }
  }

  notifyModeListeners() {
    for (const listener of this.modeListeners) {
      listener(this.mode)
    }
  }

  getDevicesSnapshot() {
    return this.devices.map((device) => ({ ...device }))
  }

  subscribeToDeviceState(listener) {
    this.deviceStateListeners.add(listener)
    listener(this.getDevicesSnapshot())
    return () => {
      this.deviceStateListeners.delete(listener)
    }
  }

  notifyDeviceState() {
    const snapshot = this.getDevicesSnapshot()
    for (const listener of this.deviceStateListeners) {
      listener(snapshot)
    }
  }

  persistAssignments() {
    setStoredAssignments(this.deviceAssignments)
  }

  rememberDevice(positionId, bluetoothDevice, fallbackDeviceId) {
    this.deviceAssignments[positionId] = {
      bluetoothId: bluetoothDevice?.id || null,
      name: bluetoothDevice?.name || null,
      fallbackDeviceId: fallbackDeviceId || null,
      savedAt: Date.now(),
    }
    this.persistAssignments()
  }

  forgetDevice(positionId) {
    if (!this.deviceAssignments[positionId]) {
      return
    }

    delete this.deviceAssignments[positionId]
    this.persistAssignments()
  }

  async requestBleDeviceForTarget() {
    return navigator.bluetooth.requestDevice({
      filters: [{ namePrefix: BLE_DEVICE_PREFIX }],
      optionalServices: [BLE_SERVICE_UUID],
    })
  }

  async resolveDeviceForPosition(positionId, {
    preferKnownDevice = false,
    pickerFallback = true,
    bluetoothDevice = null,
  } = {}) {
    const target = this.devices.find((device) => device.positionId === positionId)
    if (!target) {
      throw new Error('Unknown device slot.')
    }

    if (bluetoothDevice) {
      return {
        bluetoothDevice,
        selectedDeviceKey: bluetoothDevice.id || bluetoothDevice.name || target.expectedDeviceId,
        target,
      }
    }

    const assignment = this.deviceAssignments[positionId]
    if (preferKnownDevice && assignment && this.supportsRememberedDevices()) {
      const grantedDevices = await navigator.bluetooth.getDevices()
      const matchedDevice = grantedDevices.find((device) => {
        const matchesId = assignment.bluetoothId && device.id === assignment.bluetoothId
        const matchesName = assignment.name && device.name === assignment.name
        return matchesId || matchesName
      })

      if (matchedDevice) {
        return {
          bluetoothDevice: matchedDevice,
          selectedDeviceKey: matchedDevice.id || matchedDevice.name || assignment.fallbackDeviceId || target.expectedDeviceId,
          target,
        }
      }
    }

    if (preferKnownDevice && this.supportsRememberedDevices()) {
      const grantedDevices = await navigator.bluetooth.getDevices()
      const matchedByName = grantedDevices.find((device) => (
        findPositionIdForBleName(device.name) === positionId
      ))
      if (matchedByName) {
        return {
          bluetoothDevice: matchedByName,
          selectedDeviceKey: matchedByName.id || matchedByName.name || target.expectedDeviceId,
          target,
        }
      }
    }

    if (!pickerFallback) {
      throw new Error('No reusable browser BLE permission was found for this sensor on the new page.')
    }

    const pickedDevice = await this.requestBleDeviceForTarget()

    return {
      bluetoothDevice: pickedDevice,
      selectedDeviceKey: pickedDevice.id || pickedDevice.name || target.expectedDeviceId,
      target,
    }
  }

  createSocket() {
    const socket = new BrowserBleSocket(this)
    this.sockets.add(socket)
    queueMicrotask(() => {
      socket.emitOpen()
    })
    return socket
  }

  unregisterSocket(socket) {
    this.sockets.delete(socket)
  }

  subscribeToMessages(listener) {
    this.messageListeners.add(listener)

    return () => {
      this.messageListeners.delete(listener)
    }
  }

  forceCloseSockets() {
    for (const socket of Array.from(this.sockets)) {
      this.sockets.delete(socket)
      socket.emitClose()
    }
  }

  updateDevice(positionId, updater) {
    this.devices = this.devices.map((device) => (
      device.positionId === positionId ? updater(device) : device
    ))
    this.notifyDeviceState()
  }

  resetDeviceState() {
    this.devices = createInitialDevices()
    this.notifyDeviceState()
  }

  hasConnectedDevices() {
    return this.devices.some((device) => device.status === 'connected')
  }

  getDisconnectedPositionIds() {
    return this.devices
      .filter((device) => device.status !== 'connected' && device.status !== 'connecting')
      .map((device) => device.positionId)
  }

  async autoConnectBrowserBleDevices() {
    if (!this.isWebBluetoothSupported() || !this.supportsRememberedDevices()) {
      return { connected: [], skipped: this.getDisconnectedPositionIds(), failed: [] }
    }

    const grantedDevices = await navigator.bluetooth.getDevices()
    const results = { connected: [], skipped: [], failed: [] }

    for (const position of BODY_POSITIONS) {
      const current = this.devices.find((device) => device.positionId === position.id)
      if (current?.status === 'connected' || current?.status === 'connecting') {
        continue
      }

      const matchedDevice = grantedDevices.find((device) => (
        findPositionIdForBleName(device.name) === position.id
      ))
      if (!matchedDevice) {
        results.skipped.push(position.id)
        continue
      }

      try {
        await this.connectDevice(position.id, {
          bluetoothDevice: matchedDevice,
          pickerFallback: false,
        })
        results.connected.push(position.id)
      } catch {
        results.failed.push(position.id)
      }
    }

    return results
  }

  async connectAllBrowserBleDevices() {
    const results = await this.autoConnectBrowserBleDevices()

    while (this.getDisconnectedPositionIds().length > 0) {
      const [positionId] = this.getDisconnectedPositionIds()
      try {
        await this.connectDevice(positionId, {
          preferKnownDevice: true,
          pickerFallback: true,
        })
        results.connected.push(positionId)
      } catch (error) {
        if (error?.name === 'NotFoundError') {
          break
        }
        results.failed.push(positionId)
        break
      }
    }

    return results
  }

  async connectDevice(positionId, options = {}) {
    if (!this.isWebBluetoothSupported()) {
      throw new Error('Web Bluetooth is not available in this browser.')
    }

    let bluetoothDevice = options.bluetoothDevice || null
    let selectedDeviceKey = null

    if (!bluetoothDevice) {
      const resolved = await this.resolveDeviceForPosition(positionId, options)
      bluetoothDevice = resolved.bluetoothDevice
      selectedDeviceKey = resolved.selectedDeviceKey
    } else {
      selectedDeviceKey = bluetoothDevice.id || bluetoothDevice.name || null
    }

    const resolvedPositionId = findPositionIdForBleName(bluetoothDevice?.name) || positionId
    const target = this.devices.find((device) => device.positionId === resolvedPositionId)
    if (!target) {
      throw new Error('Unknown device slot.')
    }

    positionId = resolvedPositionId
    selectedDeviceKey = selectedDeviceKey || bluetoothDevice.id || bluetoothDevice.name || target.expectedDeviceId

    await this.disconnectDevice(positionId, { silent: true })
    this.updateDevice(positionId, (device) => ({
      ...device,
      status: 'connecting',
      deviceId: null,
      deviceName: null,
      battery: null,
      signal: null,
      packetCount: 0,
      lastPacketSize: null,
    }))

    try {
      if (!bluetoothDevice.gatt) {
        throw new Error('This browser does not expose a usable GATT transport for the selected sensor.')
      }

      const alreadyAssigned = this.devices.some((device) => (
        device.positionId !== positionId
        && device.deviceId
        && device.deviceId === selectedDeviceKey
      ))
      if (alreadyAssigned) {
        throw new Error('This sensor is already assigned to another body position.')
      }

      const server = await bluetoothDevice.gatt.connect()
      const service = await server.getPrimaryService(BLE_SERVICE_UUID)
      const characteristic = await service.getCharacteristic(BLE_CHARACTERISTIC_UUID)

      const disconnectHandler = () => {
        this.connections.delete(positionId)
        this.updateDevice(positionId, (device) => ({
          ...device,
          status: 'error',
          signal: null,
        }))
      }

      const notificationHandler = (event) => {
        const packetBytes = new Uint8Array(event.target.value.buffer.slice(0))
        const decoded = decodePacket(packetBytes)
        if (!decoded) {
          return
        }

        this.updateDevice(positionId, (device) => ({
          ...device,
          status: 'connected',
          signal: 'Streaming',
          packetCount: device.packetCount + 1,
          lastPacketSize: packetBytes.byteLength,
        }))

        // Remap raw sensor frame -> canonical frame ที่นี่ที่เดียว (จุดเดียวที่มี raw
        // hardware frame). ทุก hop หลังจากนี้ (normalize, calibration, processor,
        // broadcast) ถือว่าข้อมูลเป็น canonical แล้ว จึงไม่ต้องพึ่ง flag ข้าม serialization
        // และ remote-broadcast ก็รับ canonical โดยไม่ remap ซ้ำ
        const senderName = bluetoothDevice.name || target.expectedDeviceId
        const side = inferSide(senderName, positionId)
        const canonical = applyAxisMap(decoded.raw_accel, decoded.raw_gyro, side)

        this.broadcast({
          type: 'DATA',
          payload: {
            name: senderName,
            position_id: positionId,
            side,
            sensor_mount: inferSensorMount(senderName, positionId),
            timestamp_ms: decoded.timestamp_ms,
            raw_accel: canonical.accel,
            raw_gyro: canonical.gyro,
            // raw ก่อน remap (sensor frame) + packet version สำหรับ trace export/offline reprocess
            raw_accel_sensor: decoded.raw_accel,
            raw_gyro_sensor: decoded.raw_gyro,
            packet_version: decoded.packetVersion,
            seq: decoded.seq,
          },
        })
      }

      bluetoothDevice.addEventListener('gattserverdisconnected', disconnectHandler)
      characteristic.addEventListener('characteristicvaluechanged', notificationHandler)
      await characteristic.startNotifications()

      this.connections.set(positionId, {
        device: bluetoothDevice,
        characteristic,
        notificationHandler,
        disconnectHandler,
      })

      this.updateDevice(positionId, (device) => ({
        ...device,
        status: 'connected',
        deviceId: selectedDeviceKey,
        deviceName: bluetoothDevice.name || 'DernDee sensor',
        signal: 'Streaming',
      }))
      this.rememberDevice(positionId, bluetoothDevice, target.expectedDeviceId)
    } catch (error) {
      this.updateDevice(positionId, (device) => ({
        ...device,
        status: 'error',
        signal: null,
      }))
      throw error
    }
  }

  async disconnectDevice(positionId, { silent = false, forgetDevice = false } = {}) {
    const connection = this.connections.get(positionId)

    if (connection) {
      try {
        connection.characteristic?.removeEventListener('characteristicvaluechanged', connection.notificationHandler)
      } catch {
        // Ignore teardown errors.
      }

      try {
        connection.device?.removeEventListener('gattserverdisconnected', connection.disconnectHandler)
      } catch {
        // Ignore teardown errors.
      }

      try {
        await connection.characteristic?.stopNotifications()
      } catch {
        // Ignore teardown errors.
      }

      try {
        connection.device?.gatt?.disconnect()
      } catch {
        // Ignore teardown errors.
      }

      this.connections.delete(positionId)
    }

    this.updateDevice(positionId, (device) => ({
      ...device,
      status: 'disconnected',
      deviceId: null,
      deviceName: null,
      battery: null,
      signal: null,
      packetCount: 0,
      lastPacketSize: null,
    }))

    if (!silent && !this.hasConnectedDevices()) {
      this.broadcast({ type: 'SYSTEM_RESET' })
    }

    if (forgetDevice) {
      this.forgetDevice(positionId)
    }
  }

  async disconnectAllDevices({ keepMode = false, forgetDevices = false } = {}) {
    const positionIds = Array.from(this.connections.keys())
    await Promise.all(positionIds.map((positionId) => this.disconnectDevice(positionId, {
      silent: true,
      forgetDevice: forgetDevices,
    })))
    this.resetDeviceState()
    this.clearCalibrationTimer()
    this.activeSession = null
    if (forgetDevices) {
      this.deviceAssignments = {}
      this.persistAssignments()
    }
    if (!keepMode) {
      this.mode = TRANSPORT_MODE_WEB_BLUETOOTH
      setStoredMode(TRANSPORT_MODE_WEB_BLUETOOTH)
      this.notifyModeListeners()
    }
  }

  clearCalibrationTimer() {
    if (this.calibrationTimer) {
      window.clearTimeout(this.calibrationTimer)
      this.calibrationTimer = null
    }
  }

  handleClientSend(rawMessage) {
    let messageType = rawMessage
    let payload = {}

    if (typeof rawMessage === 'string' && rawMessage.startsWith('{')) {
      try {
        const parsed = JSON.parse(rawMessage)
        messageType = parsed.type
        payload = parsed.payload || {}
      } catch {
        messageType = rawMessage
      }
    }

    if (messageType === 'CALIBRATE') {
      if (!this.hasConnectedDevices()) {
        this.broadcast({ type: 'CALIBRATION_FAILED' })
        return
      }

      this.clearCalibrationTimer()
      this.broadcast({ type: 'CALIBRATION_START' })
      this.calibrationTimer = window.setTimeout(() => {
        this.calibrationTimer = null
        this.broadcast({ type: 'CALIBRATION_DONE' })
      }, CALIBRATION_DELAY_MS)
      return
    }

    if (messageType === 'RESET') {
      this.clearCalibrationTimer()
      this.activeSession = null
      this.broadcast({ type: 'SYSTEM_RESET' })
      return
    }

    if (messageType === 'START_SESSION') {
      this.activeSession = {
        patient_id: payload.patient_id,
        session_type: payload.session_type || 'free_walk',
        metadata: payload,
        started_at: Date.now(),
      }
      this.broadcast({
        type: 'SESSION_STARTED',
        payload: {
          patient_id: this.activeSession.patient_id,
          session_type: this.activeSession.session_type,
          transport: TRANSPORT_MODE_WEB_BLUETOOTH,
        },
      })
      return
    }

    if (messageType === 'END_SESSION') {
      this.broadcast({
        type: 'SESSION_SAVED',
        payload: {
          transport: TRANSPORT_MODE_WEB_BLUETOOTH,
          local_only: true,
          source: payload.source || 'frontend',
        },
      })
      this.activeSession = null
    }
  }

  broadcast(message) {
    for (const socket of this.sockets) {
      socket.emitMessage(message)
    }

    for (const listener of this.messageListeners) {
      try {
        listener(message)
      } catch (error) {
        console.error('Realtime transport listener failed:', error)
      }
    }
  }
}

const browserBleManager = new BrowserBleManager()

export {
  BODY_POSITIONS,
  TRANSPORT_MODE_WEB_BLUETOOTH,
}

export function createRealtimeConnection() {
  return browserBleManager.createSocket()
}

export function subscribeToRealtimeMessages(listener) {
  return browserBleManager.subscribeToMessages(listener)
}

export function getRealtimeTransportMode() {
  return browserBleManager.getMode()
}

export function setRealtimeTransportMode(mode) {
  browserBleManager.setMode(mode)
}

export function subscribeToRealtimeTransportMode(listener) {
  return browserBleManager.subscribeToMode(listener)
}

export function isWebBluetoothSupported() {
  return browserBleManager.isWebBluetoothSupported()
}

export function supportsRememberedBrowserBleDevices() {
  return browserBleManager.supportsRememberedDevices()
}

export function getBrowserBleDevicesSnapshot() {
  return browserBleManager.getDevicesSnapshot()
}

export function subscribeToBrowserBleDevices(listener) {
  return browserBleManager.subscribeToDeviceState(listener)
}

export async function connectBrowserBleDevice(positionId, options) {
  return browserBleManager.connectDevice(positionId, options)
}

export async function autoConnectBrowserBleDevices() {
  return browserBleManager.autoConnectBrowserBleDevices()
}

export async function connectAllBrowserBleDevices() {
  return browserBleManager.connectAllBrowserBleDevices()
}

export async function disconnectBrowserBleDevice(positionId, options) {
  return browserBleManager.disconnectDevice(positionId, options)
}

export async function resetBrowserBleDevices() {
  return browserBleManager.disconnectAllDevices({ keepMode: true })
}

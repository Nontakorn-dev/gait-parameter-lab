import { getRealtimeSensorSideOrder } from '../gateway/realtimeSensorUtils.js';
import {
  GAIT_MAX_VISIBLE_SHANK_SENSORS,
  GAIT_SENSOR_STALE_AFTER_MS,
} from './gaitRuntimeConfig.js';

export const SHANK_BODY_POSITION_IDS = new Set(['LEFT_SHANK', 'RIGHT_SHANK']);

export function isShankBodyPositionId(positionId) {
  return SHANK_BODY_POSITION_IDS.has(positionId);
}

export function sortGaitSensorStates(left, right) {
  const sideDelta = getRealtimeSensorSideOrder(left?.side) - getRealtimeSensorSideOrder(right?.side);
  if (sideDelta !== 0) {
    return sideDelta;
  }

  return (right?.lastSeenAt || 0) - (left?.lastSeenAt || 0);
}

export function filterActiveShankSensorStates(sensorStates = [], options = {}) {
  const { staleAfterMs = GAIT_SENSOR_STALE_AFTER_MS } = options;
  const now = Date.now();

  return sensorStates
    .filter((sensorState) => now - (sensorState.lastSeenAt || 0) <= staleAfterMs)
    .filter((sensorState) => sensorState.sensorMount === 'shank')
    .sort(sortGaitSensorStates);
}

export function hasActiveShankSensorStates(sensorStates = [], options = {}) {
  return filterActiveShankSensorStates(sensorStates, options).length > 0;
}

export function hasStreamingShankBleDevices(devices = []) {
  return devices.some((device) => (
    isShankBodyPositionId(device.positionId)
    && device.status === 'connected'
    && device.packetCount > 0
  ));
}

export function resolveVisibleShankSensorKeys(sensorStates = [], options = {}) {
  const {
    staleAfterMs = GAIT_SENSOR_STALE_AFTER_MS,
    maxVisibleSensors = GAIT_MAX_VISIBLE_SHANK_SENSORS,
  } = options;

  return filterActiveShankSensorStates(sensorStates, { staleAfterMs })
    .slice(0, maxVisibleSensors)
    .map((sensorState) => sensorState.sensorKey);
}

export function filterActiveThighSensorStates(sensorStates = [], options = {}) {
  const { staleAfterMs = GAIT_SENSOR_STALE_AFTER_MS } = options;
  const now = Date.now();

  return sensorStates
    .filter((sensorState) => now - (sensorState.lastSeenAt || 0) <= staleAfterMs)
    .filter((sensorState) => sensorState.sensorMount === 'thigh')
    .sort(sortGaitSensorStates);
}

export function resolveActiveLegPairs(sensorStates = [], options = {}) {
  const { staleAfterMs = GAIT_SENSOR_STALE_AFTER_MS } = options;
  const thighs = filterActiveThighSensorStates(sensorStates, { staleAfterMs });
  const shanks = filterActiveShankSensorStates(sensorStates, { staleAfterMs });
  const pairs = [];

  for (const side of ['L', 'R']) {
    const thigh = thighs.find((sensorState) => sensorState.side === side);
    const shank = shanks.find((sensorState) => sensorState.side === side);
    if (thigh && shank) {
      pairs.push({
        side,
        thighKey: thigh.sensorKey,
        shankKey: shank.sensorKey,
        thigh,
        shank,
      });
    }
  }

  return pairs.sort((left, right) => getRealtimeSensorSideOrder(left.side) - getRealtimeSensorSideOrder(right.side));
}

export function hasActiveLegPairs(sensorStates = [], options = {}) {
  return resolveActiveLegPairs(sensorStates, options).length > 0;
}

export function hasStreamingLegPairBleDevices(devices = []) {
  const hasLeftPair = devices.some((device) => device.positionId === 'LEFT_THIGH' && device.status === 'connected' && device.packetCount > 0)
    && devices.some((device) => device.positionId === 'LEFT_SHANK' && device.status === 'connected' && device.packetCount > 0);
  const hasRightPair = devices.some((device) => device.positionId === 'RIGHT_THIGH' && device.status === 'connected' && device.packetCount > 0)
    && devices.some((device) => device.positionId === 'RIGHT_SHANK' && device.status === 'connected' && device.packetCount > 0);

  return hasLeftPair || hasRightPair;
}

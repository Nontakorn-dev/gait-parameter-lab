import { deg2rad, trapezoidalIntegrate } from './signalUtils.js';

export class VelocityIntegrator {
  constructor(options = {}) {
    this.sampleRate = options.sampleRate || 100;
    this.dt = 1.0 / this.sampleRate;
    this.gravity = options.gravity || 9.81;
  }

  sensorToWorld(ay, az, angle) {
    const theta = deg2rad(angle);
    const cosT = Math.cos(theta);
    const sinT = Math.sin(theta);

    const aVert = -ay * cosT + az * sinT;
    const aHoriz = -ay * sinT - az * cosT;

    return { aVert, aHoriz };
  }

  computeStrideMetrics(ayArray, azArray, angles, options = {}) {
    const sampleCount = ayArray.length;
    if (sampleCount < 2) {
      return {
        strideLength: 0,
        clearance: 0,
        velocity: [],
        displacement: [],
        verticalVelocity: [],
        verticalDisplacement: [],
      };
    }

    const integrationStartIdx = Math.max(0, Math.min(sampleCount - 1, Math.round(options.integrationStartIdx ?? 0)));
    const integrationEndIdx = Math.max(integrationStartIdx, Math.min(sampleCount - 1, Math.round(options.integrationEndIdx ?? (sampleCount - 1))));

    const aHoriz = new Array(sampleCount);
    const aVertLinear = new Array(sampleCount);

    for (let i = 0; i < sampleCount; i += 1) {
      const world = this.sensorToWorld(ayArray[i], azArray[i], angles[i]);
      aHoriz[i] = world.aHoriz;
      aVertLinear[i] = world.aVert - this.gravity;
    }

    const swingHoriz = [];
    const swingVert = [];
    for (let i = integrationStartIdx; i <= integrationEndIdx; i += 1) {
      swingHoriz.push(aHoriz[i]);
      swingVert.push(aVertLinear[i]);
    }

    if (swingHoriz.length < 2) {
      return {
        strideLength: 0,
        clearance: 0,
        velocity: [],
        displacement: [],
        verticalVelocity: [],
        verticalDisplacement: [],
      };
    }

    let velocity = trapezoidalIntegrate(swingHoriz, this.dt);
    velocity = this.correctDrift(velocity);
    const displacement = trapezoidalIntegrate(velocity, this.dt);

    let verticalVelocity = trapezoidalIntegrate(swingVert, this.dt);
    verticalVelocity = this.correctDrift(verticalVelocity);
    const verticalDisplacement = trapezoidalIntegrate(verticalVelocity, this.dt);

    let minVertical = Infinity;
    let maxVertical = -Infinity;
    for (const value of verticalDisplacement) {
      minVertical = Math.min(minVertical, value);
      maxVertical = Math.max(maxVertical, value);
    }

    const strideLength = Math.abs(displacement[displacement.length - 1]);
    const clearance = Math.max(0, maxVertical - minVertical);

    return {
      strideLength,
      clearance,
      velocity,
      displacement,
      verticalVelocity,
      verticalDisplacement,
    };
  }

  correctDrift(velocity) {
    const sampleCount = velocity.length;
    if (sampleCount < 2) {
      return velocity;
    }

    const vStart = velocity[0];
    const vEnd = velocity[sampleCount - 1];
    const driftPerSample = (vEnd - vStart) / (sampleCount - 1);

    const corrected = new Array(sampleCount);
    for (let i = 0; i < sampleCount; i += 1) {
      corrected[i] = velocity[i] - (vStart + driftPerSample * i);
    }

    return corrected;
  }
}

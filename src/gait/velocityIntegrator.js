import { deg2rad, trapezoidalIntegrate } from './signalUtils.js';
import { sensorAccelToGaitFrame } from './madgwickFilter.js';

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

    const aVert = -ay * cosT - az * sinT;
    const aHoriz = -ay * sinT + az * cosT;

    return { aVert, aHoriz };
  }

  /**
   * @param {number[]} ayArray  m/s²
   * @param {number[]} azArray  m/s²
   * @param {number[]} angles   deg (1D sagittal) — ใช้เมื่อไม่มี quaternions
   * @param {object} [options]
   * @param {number[]} [options.axArray]  m/s² — จำเป็นเมื่อใช้ Madgwick quat
   * @param {Array<{q0,q1,q2,q3}>} [options.quaternions]
   */
  computeStrideMetrics(ayArray, azArray, angles, options = {}) {
    // ใช้ dt จริงจาก timestamp เมื่อส่งมา (ทนต่อ dropped sample) ไม่งั้น fallback เป็น nominal
    const dt = Number.isFinite(options.dt) && options.dt > 0 ? options.dt : this.dt;
    const sampleCount = ayArray.length;
    if (sampleCount < 2) {
      return {
        strideLength: 0,
        strideLengthSigned: 0,
        clearance: 0,
        velocity: [],
        velocityPreDriftCorrection: [],
        displacement: [],
        verticalVelocity: [],
        verticalDisplacement: [],
      };
    }

    const integrationStartIdx = Math.max(0, Math.min(sampleCount - 1, Math.round(options.integrationStartIdx ?? 0)));
    const integrationEndIdx = Math.max(integrationStartIdx, Math.min(sampleCount - 1, Math.round(options.integrationEndIdx ?? (sampleCount - 1))));

    const aHoriz = new Array(sampleCount);
    const aVertLinear = new Array(sampleCount);
    const quaternions = options.quaternions;
    const axArray = options.axArray;
    const useMadgwick = Array.isArray(quaternions)
      && quaternions.length === sampleCount
      && Array.isArray(axArray)
      && axArray.length === sampleCount;

    for (let i = 0; i < sampleCount; i += 1) {
      if (useMadgwick && quaternions[i]) {
        const { q0, q1, q2, q3 } = quaternions[i];
        const frame = sensorAccelToGaitFrame(
          axArray[i] / this.gravity,
          ayArray[i] / this.gravity,
          azArray[i] / this.gravity,
          q0, q1, q2, q3,
          this.gravity,
        );
        aHoriz[i] = frame.aForward;
        aVertLinear[i] = frame.aVert;
      } else {
        const world = this.sensorToWorld(ayArray[i], azArray[i], angles[i]);
        aHoriz[i] = world.aHoriz;
        aVertLinear[i] = world.aVert - this.gravity;
      }
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
        strideLengthSigned: 0,
        clearance: 0,
        velocity: [],
        velocityPreDriftCorrection: [],
        displacement: [],
        verticalVelocity: [],
        verticalDisplacement: [],
      };
    }

    const velocityPreDriftCorrection = trapezoidalIntegrate(swingHoriz, dt);
    const velocity = this.correctDrift(velocityPreDriftCorrection);
    const displacement = trapezoidalIntegrate(velocity, dt);

    let verticalVelocity = trapezoidalIntegrate(swingVert, dt);
    verticalVelocity = this.correctDrift(verticalVelocity);
    const verticalDisplacement = trapezoidalIntegrate(verticalVelocity, dt);

    let minVertical = Infinity;
    let maxVertical = -Infinity;
    for (const value of verticalDisplacement) {
      minVertical = Math.min(minVertical, value);
      maxVertical = Math.max(maxVertical, value);
    }

    const signedDisplacement = displacement[displacement.length - 1];
    const strideLength = Math.abs(signedDisplacement);
    const clearance = Math.max(0, maxVertical - minVertical);

    return {
      strideLength,
      strideLengthSigned: signedDisplacement,
      clearance,
      velocity,
      velocityPreDriftCorrection,
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

    // ZUPT ที่ปลายสองด้านสมมติแรงเกินไปบน shank: ช่วงปลาย window มักยังไม่นิ่งจริง
    // (|v_end| ค้าง) การลบ linear ramp ไปหา v_end=0 จะตัดความเร็วจริง → ระยะสั้น ~20%
    // บน walk 3 m จริง. รีเซ็ตแค่ v_start (หลัง HS / ต้น quiet) ซึ่งเชื่อถือได้กว่า
    // เก็บ velocityPreDriftCorrection ไว้ดู |v_end| เป็น quality gate แยก
    const vStart = velocity[0];
    const corrected = new Array(sampleCount);
    for (let i = 0; i < sampleCount; i += 1) {
      corrected[i] = velocity[i] - vStart;
    }

    return corrected;
  }
}

import { deg2rad, trapezoidalIntegrate } from './signalUtils.js';
import { sensorAccelToGaitFrame } from './madgwickFilter.js';

export class VelocityIntegrator {
  constructor(options = {}) {
    this.sampleRate = options.sampleRate || 100;
    this.dt = 1.0 / this.sampleRate;
    this.gravity = options.gravity || 9.81;
    // 'start-only' = บังคับ v=0 ที่ integrationStart (ZUPT ต้น window) ไม่บังคับปลาย
    // 'start-end'  = linear bridge ให้ v_end=0 ด้วย (classic dual ZUPT)
    this.driftMode = options.driftMode === 'start-end' ? 'start-end' : 'start-only';
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
   * @param {number[]} ayArray  m/s² — ควรคลุมช่วงก่อน integrationStart (เช่น จาก HS)
   * @param {number[]} azArray  m/s²
   * @param {number[]} angles   deg (1D sagittal) — ใช้เมื่อไม่มี quaternions
   * @param {object} [options]
   * @param {number} [options.integrationStartIdx=0] index ใน array ที่เชื่อว่า ZUPT (quiet หลัง HS)
   * @param {number} [options.integrationEndIdx]
   * @param {number[]} [options.axArray]
   * @param {Array<{q0,q1,q2,q3}>} [options.quaternions]
   * @param {'start-only'|'start-end'} [options.driftMode]
   */
  computeStrideMetrics(ayArray, azArray, angles, options = {}) {
    const dt = Number.isFinite(options.dt) && options.dt > 0 ? options.dt : this.dt;
    const driftMode = options.driftMode === 'start-end' || options.driftMode === 'start-only'
      ? options.driftMode
      : this.driftMode;
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
        vStartPreDrift: null,
        vEndPreDrift: null,
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

    // Integrate จากต้น segment (มักเป็น HS) → ได้ความเร็วสะสมก่อนเข้า window
    // แล้วรีเซ็ตที่ integrationStartIdx (= ZUPT ต้น quiet) — นี่คือ start-only จริง
    // ห้าม trapz เฉพาะ window แล้วลบ velocity[0] เพราะ trapz ตั้ง [0]=0 ตายตัว → no-op
    const velFull = trapezoidalIntegrate(aHoriz.slice(0, integrationEndIdx + 1), dt);
    const vertFull = trapezoidalIntegrate(aVertLinear.slice(0, integrationEndIdx + 1), dt);

    const vStartPreDrift = velFull[integrationStartIdx];
    const velocityPreDriftCorrection = [];
    for (let i = integrationStartIdx; i <= integrationEndIdx; i += 1) {
      velocityPreDriftCorrection.push(velFull[i]);
    }
    const vEndPreDrift = velocityPreDriftCorrection[velocityPreDriftCorrection.length - 1];

    const velocity = this.applyDriftCorrection(velocityPreDriftCorrection, driftMode);
    const displacement = trapezoidalIntegrate(velocity, dt);

    const verticalPre = [];
    for (let i = integrationStartIdx; i <= integrationEndIdx; i += 1) {
      verticalPre.push(vertFull[i]);
    }
    let verticalVelocity = this.applyDriftCorrection(verticalPre, driftMode);
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
      vStartPreDrift: Number.isFinite(vStartPreDrift) ? vStartPreDrift : null,
      vEndPreDrift: Number.isFinite(vEndPreDrift) ? vEndPreDrift : null,
    };
  }

  /**
   * @param {number[]} velocityPre  ความเร็วสะสมดิบในช่วง window (index 0 = จุด ZUPT ต้น)
   * @param {'start-only'|'start-end'} driftMode
   */
  applyDriftCorrection(velocityPre, driftMode = 'start-only') {
    const sampleCount = velocityPre.length;
    if (sampleCount < 2) {
      return velocityPre.slice();
    }

    // บังคับ v=0 ที่ต้น window (จุดที่เชื่อว่า stance quiet)
    const vStart = velocityPre[0];
    const startZeroed = new Array(sampleCount);
    for (let i = 0; i < sampleCount; i += 1) {
      startZeroed[i] = velocityPre[i] - vStart;
    }

    if (driftMode !== 'start-end') {
      return startZeroed;
    }

    // Classic dual ZUPT: ลบ linear ramp ให้ปลายเป็น 0 ด้วย
    const vEnd = startZeroed[sampleCount - 1];
    const driftPerSample = vEnd / (sampleCount - 1);
    const corrected = new Array(sampleCount);
    for (let i = 0; i < sampleCount; i += 1) {
      corrected[i] = startZeroed[i] - driftPerSample * i;
    }
    return corrected;
  }

  /** @deprecated ใช้ applyDriftCorrection — เก็บชื่อเดิมให้เทสต์เก่าที่เรียตรง */
  correctDrift(velocity) {
    return this.applyDriftCorrection(velocity, 'start-only');
  }
}

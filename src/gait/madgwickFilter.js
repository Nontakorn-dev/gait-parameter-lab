/**
 * Madgwick AHRS (6DOF: gyro + accel, no magnetometer).
 *
 * Sensor frame ของโปรเจกต์: X right, Y down (shank), Z forward
 * Madgwick มาตรฐานสมมติ Z-up → remap ก่อนอัปเดต:
 *   (xm, ym, zm) = (ax, az, -ay)   → ยืนนิ่ง ay=-1g ได้ zm=+1g
 *   gyro เดียวกัน: (gxm, gym, gzm) = (gx, gz, -gy)
 *
 * อ้างอิง: S.O.H. Madgwick, 2010
 */

const DEG2RAD = Math.PI / 180;

function toMadgwickAccel(ax, ay, az) {
  return { x: ax, y: az, z: -ay };
}

function toMadgwickGyro(gx, gy, gz) {
  // R = (x,y,z)→(x,z,-y) บนเวกเตอร์ → ω' = Rω = (gx, gz, -gy)
  // แต่ getSagittal / แกนเดินของเรา: +gx ต้องเพิ่มมุม → ส่ง −gx เข้า Madgwick X
  return { x: -gx, y: gz, z: -gy };
}

export class MadgwickFilter {
  /**
   * @param {Object} [options]
   * @param {number} [options.beta=0.08]  gain เมื่อ ‖a‖≈1g
   * @param {number} [options.samplePeriod=0.01]
   * @param {number} [options.accelGateSoftG=0.1] เริ่มลด beta
   * @param {number} [options.accelGateHardG=0.5] เกินนี้ → gyro-only (beta=0)
   */
  constructor(options = {}) {
    this.beta = Number.isFinite(options.beta) ? options.beta : 0.08;
    this.samplePeriod = Number.isFinite(options.samplePeriod) ? options.samplePeriod : 0.01;
    // เหมือน Kalman: ระหว่าง swing ‖a‖ เบี่ยงจาก 1g → อย่าดึง quat ไปตาม accel (มุมเพี้ยน)
    this.accelGateSoftG = Number.isFinite(options.accelGateSoftG) ? options.accelGateSoftG : 0.1;
    this.accelGateHardG = Number.isFinite(options.accelGateHardG) ? options.accelGateHardG : 0.5;
    // shank swing: ‖a‖ อาจยังใกล้ 1g — ใช้ |ω| ตัด accel correction
    this.gyroGateSoftDps = Number.isFinite(options.gyroGateSoftDps) ? options.gyroGateSoftDps : 40;
    this.gyroGateHardDps = Number.isFinite(options.gyroGateHardDps) ? options.gyroGateHardDps : 120;
    this.q0 = 1;
    this.q1 = 0;
    this.q2 = 0;
    this.q3 = 0;
  }

  reset() {
    this.q0 = 1;
    this.q1 = 0;
    this.q2 = 0;
    this.q3 = 0;
  }

  /**
   * beta ที่มีผลรอบนี้
   * - ‖a‖ ไกล 1g → ลด/ตัด (linear accel)
   * - |ω| สูง → ตัด (swing: ‖a‖ อาจยังใกล้ 1g แต่ทิศเพี้ยน)
   * @param {number} axG
   * @param {number} ayG
   * @param {number} azG
   * @param {number} gxDegs
   * @param {number} gyDegs
   * @param {number} gzDegs
   */
  effectiveBeta(axG, ayG, azG, gxDegs = 0, gyDegs = 0, gzDegs = 0) {
    const mag = Math.sqrt(axG * axG + ayG * ayG + azG * azG);
    if (!Number.isFinite(mag) || mag < 1e-8) return 0;
    const rate = Math.sqrt(gxDegs * gxDegs + gyDegs * gyDegs + gzDegs * gzDegs);
    if (Number.isFinite(rate) && rate >= this.gyroGateHardDps) return 0;
    const err = Math.abs(mag - 1);
    if (err >= this.accelGateHardG) return 0;
    let beta = this.beta;
    if (err > this.accelGateSoftG) {
      const t = (err - this.accelGateSoftG) / (this.accelGateHardG - this.accelGateSoftG);
      beta *= Math.max(0, 1 - t);
    }
    if (Number.isFinite(rate) && rate > this.gyroGateSoftDps) {
      const t = (rate - this.gyroGateSoftDps) / (this.gyroGateHardDps - this.gyroGateSoftDps);
      beta *= Math.max(0, 1 - t);
    }
    return beta;
  }

  /**
   * @param {number} gxDegs deg/s (sensor frame)
   * @param {number} gyDegs
   * @param {number} gzDegs
   * @param {number} axG accel g (sensor frame)
   * @param {number} ayG
   * @param {number} azG
   * @param {number} [dt]
   */
  update(gxDegs, gyDegs, gzDegs, axG, ayG, azG, dt = null) {
    const dtUse = Number.isFinite(dt) && dt > 0 ? dt : this.samplePeriod;
    const g = toMadgwickGyro(gxDegs * DEG2RAD, gyDegs * DEG2RAD, gzDegs * DEG2RAD);
    const aIn = toMadgwickAccel(axG, ayG, azG);
    let { x: gx, y: gy, z: gz } = g;
    let { x: ax, y: ay, z: az } = aIn;
    const betaUse = this.effectiveBeta(axG, ayG, azG, gxDegs, gyDegs, gzDegs);

    let q0 = this.q0;
    let q1 = this.q1;
    let q2 = this.q2;
    let q3 = this.q3;

    const normA = Math.sqrt(ax * ax + ay * ay + az * az);
    const useAccel = betaUse > 1e-12 && normA > 1e-8;
    if (useAccel) {
      ax /= normA;
      ay /= normA;
      az /= normA;

      const _2q0 = 2 * q0;
      const _2q1 = 2 * q1;
      const _2q2 = 2 * q2;
      const _2q3 = 2 * q3;
      const _4q0 = 4 * q0;
      const _4q1 = 4 * q1;
      const _4q2 = 4 * q2;
      const _8q1 = 8 * q1;
      const _8q2 = 8 * q2;
      const q0q0 = q0 * q0;
      const q1q1 = q1 * q1;
      const q2q2 = q2 * q2;
      const q3q3 = q3 * q3;

      let s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      let s1 = _4q1 * q3q3 - _2q3 * ax + 4 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      let s2 = 4 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      let s3 = 4 * q1q1 * q3 - _2q1 * ax + 4 * q2q2 * q3 - _2q2 * ay;

      const normS = Math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      if (normS > 1e-8) {
        s0 /= normS;
        s1 /= normS;
        s2 /= normS;
        s3 /= normS;
      }

      const qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - betaUse * s0;
      const qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - betaUse * s1;
      const qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - betaUse * s2;
      const qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - betaUse * s3;

      q0 += qDot0 * dtUse;
      q1 += qDot1 * dtUse;
      q2 += qDot2 * dtUse;
      q3 += qDot3 * dtUse;
    } else {
      const qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
      const qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
      const qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
      const qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
      q0 += qDot0 * dtUse;
      q1 += qDot1 * dtUse;
      q2 += qDot2 * dtUse;
      q3 += qDot3 * dtUse;
    }

    const normQ = Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    this.q0 = q0 / normQ;
    this.q1 = q1 / normQ;
    this.q2 = q2 / normQ;
    this.q3 = q3 / normQ;

    return { q0: this.q0, q1: this.q1, q2: this.q2, q3: this.q3 };
  }

  getQuaternion() {
    return { q0: this.q0, q1: this.q1, q2: this.q2, q3: this.q3 };
  }

  /** มุม sagittal (deg) สอดคล้อง accelToAngle(ay, az) และเครื่องหมาย +gx ของโปรเจกต์ */
  getSagittalAngleDeg() {
    const { q0, q1, q2, q3 } = this;
    // gravity direction in Madgwick (Z-up) sensor frame
    const gym = 2 * (q0 * q1 + q2 * q3);
    const gzm = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    // back to project frame: ay = -zm, az = ym
    const ay = -gzm;
    const az = gym;
    return (Math.atan2(-az, -ay) * 180) / Math.PI;
  }
}

export function rotateSensorToEarth(ax, ay, az, q0, q1, q2, q3) {
  const twox = q1 + q1;
  const twoy = q2 + q2;
  const twoz = q3 + q3;
  const w_x = q0 * twox;
  const w_y = q0 * twoy;
  const w_z = q0 * twoz;
  const x_x = q1 * twox;
  const x_y = q1 * twoy;
  const x_z = q1 * twoz;
  const y_y = q2 * twoy;
  const y_z = q2 * twoz;
  const z_z = q3 * twoz;

  return {
    x: (1 - (y_y + z_z)) * ax + (x_y - w_z) * ay + (x_z + w_y) * az,
    y: (x_y + w_z) * ax + (1 - (x_x + z_z)) * ay + (y_z - w_x) * az,
    z: (x_z - w_y) * ax + (y_z + w_x) * ay + (1 - (x_x + y_y)) * az,
  };
}

/**
 * Linear accel แนวเดิน (m/s²) จาก quat ที่อัปเดตด้วยแกนที่ remap แล้ว
 */
export function sensorAccelToGaitFrame(axG, ayG, azG, q0, q1, q2, q3, gravity = 9.81) {
  const aM = toMadgwickAccel(axG * gravity, ayG * gravity, azG * gravity);
  const aE = rotateSensorToEarth(aM.x, aM.y, aM.z, q0, q1, q2, q3);

  // Z-up earth: ลบ gravity จาก Z
  const aLinX = aE.x;
  const aLinY = aE.y;
  const aLinZ = aE.z - gravity;

  // แกนหน้าของเซนเซอร์ (Z) ใน Madgwick frame = +Y → หมุนเข้าโลก
  const zAxisM = rotateSensorToEarth(0, 1, 0, q0, q1, q2, q3);
  let fx = zAxisM.x;
  let fy = zAxisM.y;
  const fNorm = Math.sqrt(fx * fx + fy * fy);
  if (fNorm > 1e-6) {
    fx /= fNorm;
    fy /= fNorm;
  } else {
    const xAxisM = rotateSensorToEarth(1, 0, 0, q0, q1, q2, q3);
    fx = xAxisM.x;
    fy = xAxisM.y;
    const n2 = Math.sqrt(fx * fx + fy * fy) || 1;
    fx /= n2;
    fy /= n2;
  }

  const aForward = aLinX * fx + aLinY * fy;
  return {
    aForward,
    aVert: aLinZ,
    aHorizMag: Math.sqrt(aLinX * aLinX + aLinY * aLinY),
  };
}

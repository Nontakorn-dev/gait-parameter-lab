export class KalmanFilter {
  constructor(dt = 0.01) {
    this.defaultDt = dt;
    this.dt = dt;
    this.x = [0, 0];
    this.P = [
      [1, 0],
      [0, 1],
    ];
    this.Q_angle = 0.001;
    this.Q_bias = 0.003;
    this.R_measure = 0.03;

    // Adaptive measurement gating.
    // accelerometer ให้มุม tilt ที่ถูกต้องเฉพาะเมื่ออ่าน gravity ล้วน. ระหว่างเดิน
    // (โดยเฉพาะ swing) shank มี linear + centripetal/tangential accel ทำให้ ‖accel‖
    // เบี่ยงจาก 1g และมุมจาก accel เพี้ยนหนัก (จำลอง swing จริงได้ error ถึง ~76°
    // ด้วย R คงที่). จึงลดความเชื่อ accel โดยเพิ่ม R ตามระยะเบี่ยง |‖accel‖-1g| และ
    // ข้าม measurement update ทั้งหมดเมื่อเบี่ยงเกิน hard gate → ช่วง accel สูงกลายเป็น
    // gyro-only ซึ่งทนกว่ามาก. ค่านิ่ง (≈1g) ยังเชื่อ accel เต็มเพื่อแก้ bias ตามปกติ.
    this.accelGateSoftG = 0.1; // g: จุดเริ่ม inflate R
    this.accelGateGain = 50; // ตัวคูณการ inflate
    this.accelGateHardG = 0.5; // g: เกินนี้ข้าม measurement update (gyro-only)
  }

  update(gyroRate, accelAngle, dtOverride = null, accelMagnitudeG = 1.0) {
    const dt = Number.isFinite(dtOverride) && dtOverride > 0
      ? dtOverride
      : this.defaultDt;
    this.dt = dt;

    // --- Predict ---
    const rate = gyroRate - this.x[1];
    this.x[0] += dt * rate;

    this.P[0][0] += dt * (dt * this.P[1][1] - this.P[0][1] - this.P[1][0] + this.Q_angle);
    this.P[0][1] -= dt * this.P[1][1];
    this.P[1][0] -= dt * this.P[1][1];
    this.P[1][1] += this.Q_bias * dt;

    // --- Adaptive measurement gating from accel magnitude ---
    const accelError = Number.isFinite(accelMagnitudeG) ? Math.abs(accelMagnitudeG - 1.0) : 0;
    if (accelError > this.accelGateHardG) {
      // accel ไม่ใช่ gravity ล้วน → เชื่อ gyro อย่างเดียวรอบนี้ (ข้าม correction)
      return this.x[0];
    }
    const R = this.R_measure * (1 + (accelError / this.accelGateSoftG) ** 2 * this.accelGateGain);

    // --- Update ---
    const y = accelAngle - this.x[0];
    const S = this.P[0][0] + R;
    const K0 = this.P[0][0] / S;
    const K1 = this.P[1][0] / S;

    this.x[0] += K0 * y;
    this.x[1] += K1 * y;

    const P00 = this.P[0][0];
    const P01 = this.P[0][1];
    this.P[0][0] -= K0 * P00;
    this.P[0][1] -= K0 * P01;
    this.P[1][0] -= K1 * P00;
    this.P[1][1] -= K1 * P01;

    return this.x[0];
  }

  reset() {
    this.dt = this.defaultDt;
    this.x = [0, 0];
    this.P = [
      [1, 0],
      [0, 1],
    ];
  }
}
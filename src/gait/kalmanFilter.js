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
  }

  update(gyroRate, accelAngle, dtOverride = null) {
    const dt = Number.isFinite(dtOverride) && dtOverride > 0
      ? dtOverride
      : this.defaultDt;
    this.dt = dt;
    const rate = gyroRate - this.x[1];
    this.x[0] += dt * rate;

    this.P[0][0] += dt * (dt * this.P[1][1] - this.P[0][1] - this.P[1][0] + this.Q_angle);
    this.P[0][1] -= dt * this.P[1][1];
    this.P[1][0] -= dt * this.P[1][1];
    this.P[1][1] += this.Q_bias * dt;

    const y = accelAngle - this.x[0];
    const S = this.P[0][0] + this.R_measure;
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
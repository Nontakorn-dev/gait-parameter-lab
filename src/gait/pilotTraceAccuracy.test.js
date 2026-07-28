import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { GaitProcessor } from './gaitProcessor.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const TRACE_RUN1 = join(__dirname, '../../traces/pilot-20260728/gait-trace-20260728-162921.json');

function sumUsableStrides(tracePath, orientationFilter = 'kalman') {
  const trace = JSON.parse(readFileSync(tracePath, 'utf8'));
  const proc = new GaitProcessor({ orientationFilter });
  const cal = Object.values(trace.calibrationBySensor || {})[0];
  if (cal) proc.applyCalibration(cal);

  const diags = [];
  proc.onParams(({ newCycleDiagnostics }) => {
    if (newCycleDiagnostics?.length) diags.push(...newCycleDiagnostics);
  });

  for (const s of trace.samples) {
    const accel = s.raw_accel_canonical || s.raw_accel;
    const gyro = s.raw_gyro_canonical || s.raw_gyro;
    proc.addSample({
      ax: accel[0],
      ay: accel[1],
      az: accel[2],
      gx: gyro[0],
      gy: gyro[1],
      gz: gyro[2],
      timestampMs: s.t_ms,
      side: 'R',
      sensorKey: 'RIGHT_SHANK',
    });
  }
  proc.analyze();

  const usable = diags.filter((d) => Number.isFinite(d.strideLengthM));
  const sum = usable.reduce((a, d) => a + d.strideLengthM, 0);
  return { sum, usable: usable.length, n: diags.length, diags };
}

test('pilot 3 m (run1 R): Σ stride ≈ 3 m หลัง start-only ZUPT (ไม่ −20%)', () => {
  const { sum, usable, n } = sumUsableStrides(TRACE_RUN1, 'kalman');
  assert.ok(n >= 3, `ควรเจอ ≥3 cycle ได้ ${n}`);
  assert.ok(usable >= 3, `ควรใช้ได้ ≥3 ได้ ${usable}`);
  // GT = 3 m; เดิม dual-end ZUPT ได้ ~2.39 (−20%) — หลัง start-only ต้องใกล้ 3
  assert.ok(sum > 2.7, `Σ=${sum.toFixed(3)} m ยังต่ำผิด (regression กลับไป −20%?)`);
  assert.ok(sum < 3.4, `Σ=${sum.toFixed(3)} m สูงผิดปกติ`);
  assert.ok(Math.abs(sum - 3) / 3 < 0.12, `Σ=${sum.toFixed(3)} ควรใกล้ 3 m (±12%)`);
});

test('pilot 3 m: latestParams.zuptCheck มี vEndPreDrift', () => {
  const trace = JSON.parse(readFileSync(TRACE_RUN1, 'utf8'));
  const proc = new GaitProcessor({ orientationFilter: 'kalman' });
  const cal = Object.values(trace.calibrationBySensor || {})[0];
  if (cal) proc.applyCalibration(cal);
  for (const s of trace.samples) {
    const accel = s.raw_accel_canonical || s.raw_accel;
    const gyro = s.raw_gyro_canonical || s.raw_gyro;
    proc.addSample({
      ax: accel[0], ay: accel[1], az: accel[2],
      gx: gyro[0], gy: gyro[1], gz: gyro[2],
      timestampMs: s.t_ms, side: 'R', sensorKey: 'RIGHT_SHANK',
    });
  }
  proc.analyze();
  assert.ok(proc.latestParams?.zuptCheck);
  assert.ok(Number.isFinite(proc.latestParams.zuptCheck.vEndPreDrift));
});

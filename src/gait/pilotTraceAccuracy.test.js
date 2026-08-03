import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { GaitProcessor } from './gaitProcessor.js';
import { isAgreementQualityImuCycle } from '../../mocap-analysis/compareImuTrace.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const TRACE_PILOT = join(__dirname, '../../traces/pilot-20260728/gait-trace-20260728-162921.json');

function sumClosedStrides(tracePath) {
  const trace = JSON.parse(readFileSync(tracePath, 'utf8'));
  const proc = new GaitProcessor({ orientationFilter: 'kalman' });
  const cal = Object.values(trace.calibrationBySensor || {})[0];
  if (cal) proc.applyCalibration(cal);

  const diags = [];
  proc.onParams(({ newCycleDiagnostics }) => {
    if (newCycleDiagnostics?.length) diags.push(...newCycleDiagnostics.filter((d) => !d.retracted));
  });

  let since = 0;
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
    since += 1;
    if (since >= 40) {
      proc.analyze();
      since = 0;
    }
  }
  proc.analyze();

  const closed = diags.filter((d) => !d.isOpenStride && Number.isFinite(d.strideLengthM));
  const open = diags.filter((d) => d.isOpenStride);
  const sum = closed.reduce((a, d) => a + d.strideLengthM, 0);
  return { sum, closed: closed.length, open: open.length, n: diags.length, diags };
}

test('pilot 3 m: open stride ถูก flag และไม่รวมใน Σ', () => {
  const { sum, closed, open, n, diags } = sumClosedStrides(TRACE_PILOT);
  assert.ok(n >= 2, `ควรเจอ ≥2 cycle ได้ ${n}`);
  assert.ok(closed >= 1, `ควรมี closed stride ≥1 ได้ ${closed}`);
  for (const d of diags.filter((x) => x.isOpenStride)) {
    assert.equal(d.strideLengthM, null, 'open stride ห้ามใส่ระยะใน Σ');
    assert.equal(d.isOpenStride, true);
  }
  assert.ok(sum > 1.0, `Σ closed=${sum.toFixed(3)} ต่ำผิด`);
  assert.ok(sum < 4.5, `Σ closed=${sum.toFixed(3)} สูงผิด`);
  void open;
});

test('pilot 3 m: vStartPreDrift วัดจาก pre-window ได้ (ไม่ใช่ 0 ปลอมตลอด)', () => {
  const { diags } = sumClosedStrides(TRACE_PILOT);
  const withZupt = diags.filter((d) => d.zuptCheck);
  assert.ok(withZupt.length > 0);
  const anyNonZeroStart = withZupt.some((d) => Math.abs(d.zuptCheck.vStartPreDrift || 0) > 1e-6);
  const anyEnd = withZupt.some((d) => Number.isFinite(d.zuptCheck.vEndPreDrift));
  assert.ok(anyEnd, 'ต้องมี vEndPreDrift');
  assert.ok(anyNonZeroStart, 'vStartPreDrift ต้องไม่เป็น 0 ทุก cycle (no-op เดิม)');
});

test('isAgreementQualityImuCycle: ตัด open stride', () => {
  assert.equal(isAgreementQualityImuCycle({
    strideLengthM: 1.0,
    isOpenStride: true,
    strideLengthClamped: false,
    strideLengthUntrusted: false,
    temporalSource: 'measured-to',
  }), false);
});

import test from 'node:test';
import assert from 'node:assert/strict';

import { butterworthLowpass2, filtfiltButterworth2, lfilter, lfilterZi, estimateSampleRateHz } from './butterworth.js';

test('butterworthLowpass2: DC gain ≈ 1', () => {
  const { b, a } = butterworthLowpass2(6, 120);
  assert.equal(a[0], 1);
  const sumB = b.reduce((p, c) => p + c, 0);
  const sumA = a.reduce((p, c) => p + c, 0);
  assert.ok(Math.abs(sumB - sumA) < 1e-9);
});

test('butterworthLowpass2: throw ถ้า cutoff ≥ Nyquist', () => {
  assert.throws(() => butterworthLowpass2(100, 100));
});

test('filtfiltButterworth2: สัญญาณคงที่คง mean (zero-lag + zi)', () => {
  const signal = Array.from({ length: 200 }, () => 2.25);
  const out = filtfiltButterworth2(signal, 6, 100);
  assert.ok(out.every((v) => Math.abs(v - 2.25) < 1e-6));
});

test('filtfiltButterworth2: คง null ใน gap ไม่เติมข้าม', () => {
  const signal = [1, 1, 1, null, null, 2, 2, 2];
  const out = filtfiltButterworth2(signal, 6, 100);
  assert.equal(out[3], null);
  assert.equal(out[4], null);
  assert.ok(Number.isFinite(out[0]));
  assert.ok(Number.isFinite(out[7]));
});

test('lfilterZi + lfilter: step input เข้า steady state ทันที', () => {
  const { b, a } = butterworthLowpass2(6, 100);
  const zi = lfilterZi(b, a, 1);
  const out = lfilter(b, a, Array.from({ length: 20 }, () => 1), zi);
  assert.ok(out.every((v) => Math.abs(v - 1) < 1e-9));
});

test('estimateSampleRateHz: จาก Δt คงที่', () => {
  const t = Array.from({ length: 50 }, (_, i) => i / 200);
  assert.ok(Math.abs(estimateSampleRateHz(t) - 200) < 1e-6);
});

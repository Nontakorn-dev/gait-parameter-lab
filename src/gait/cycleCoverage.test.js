import test from 'node:test';
import assert from 'node:assert/strict';

import {
  computeCycleTimeCoverage,
  makeClosedCycleKey,
  parseCycleKeyStartId,
  COVERAGE_GAP_THRESHOLD_S,
} from './cycleCoverage.js';

test('makeClosedCycleKey / parseCycleKeyStartId', () => {
  assert.equal(makeClosedCycleKey(5422, 5575), '5422-5575');
  assert.equal(parseCycleKeyStartId('5422-5575'), 5422);
  assert.equal(parseCycleKeyStartId('open:5422'), 5422);
  assert.equal(parseCycleKeyStartId('5422'), 5422);
});

test('🔴 computeCycleTimeCoverage: ตรวจช่องว่างหลังแยก merged stride', () => {
  // [52.23–54.21] [55.74–57.46] — รู 54.21–55.74 ≈ 1.53s
  const cycles = [
    { cycleStartTimeS: 52.23, strideTimeS: 1.98, isOpenStride: false },
    { cycleStartTimeS: 55.74, strideTimeS: 1.72, isOpenStride: false },
    { cycleStartTimeS: 57.46, strideTimeS: 1.42, isOpenStride: false },
  ];
  const cov = computeCycleTimeCoverage(cycles);
  assert.ok(cov.coverageGapS > 1.4, `gap=${cov.coverageGapS}`);
  assert.equal(cov.hasCoverageGap, true);
  assert.ok(cov.coverageRatio < 0.85, `ratio=${cov.coverageRatio}`);
});

test('computeCycleTimeCoverage: ต่อเนื่องไม่มี gap', () => {
  const cycles = [
    { cycleStartTimeS: 0, strideTimeS: 1.0, isOpenStride: false },
    { cycleStartTimeS: 1.0, strideTimeS: 1.0, isOpenStride: false },
    { cycleStartTimeS: 2.0, strideTimeS: 1.0, isOpenStride: false },
  ];
  const cov = computeCycleTimeCoverage(cycles);
  assert.ok(cov.coverageGapS < COVERAGE_GAP_THRESHOLD_S);
  assert.equal(cov.hasCoverageGap, false);
  assert.ok(Math.abs(cov.coverageRatio - 1) < 1e-6);
});

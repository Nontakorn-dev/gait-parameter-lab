import test from 'node:test';
import assert from 'node:assert/strict';

import { aggregateGaitParams } from './gaitParamsAggregation.js';

test('aggregate: เซนเซอร์ข้างเดียว — stepCount = strideCount (ไม่ ×2)', () => {
  const averaged = aggregateGaitParams([
    { params: { stepCount: 12, strideCount: 12, strideLength: 1.2, stepLength: null, stepTime: null } },
  ]);
  assert.equal(averaged.stepCount, 12);
  assert.equal(averaged.strideCount, 12);
  assert.equal(averaged.stepLength, null);
  assert.equal(averaged.stepTime, null);
});

test('aggregate: สองข้าง — stepCount = รวม footfall ทั้งสองข้าง', () => {
  const averaged = aggregateGaitParams([
    { params: { stepCount: 10, strideCount: 10, strideLength: 1.1, cadence: 100 } },
    { params: { stepCount: 9, strideCount: 9, strideLength: 1.0, cadence: 105 } },
  ]);
  assert.equal(averaged.stepCount, 19, 'ต้องรวม L+R ไม่ใช่ max');
  assert.equal(averaged.strideCount, 10, 'stride ใช้ max ต่อข้าง');
  assert.equal(averaged.stepLength, null);
});

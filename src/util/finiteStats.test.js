import test from 'node:test';
import assert from 'node:assert/strict';

import { minFinite, maxFinite } from './finiteStats.js';

test('minFinite/maxFinite: ค่าว่าง/ไม่มี finite → null', () => {
  assert.equal(minFinite([]), null);
  assert.equal(maxFinite([]), null);
  assert.equal(minFinite([NaN, null, undefined]), null);
  assert.equal(maxFinite([Infinity, -Infinity]), null);
});

test('minFinite/maxFinite: กรอง non-finite แล้วได้ค่าถูก', () => {
  assert.equal(minFinite([3, NaN, 1, 2]), 1);
  assert.equal(maxFinite([3, NaN, 1, 2]), 3);
});

test('🔴 minFinite: ไม่ stack overflow ที่ 150k / 300k (regression vs Math.min spread)', () => {
  for (const n of [150_000, 300_000]) {
    const values = new Float64Array(n);
    for (let i = 0; i < n; i += 1) values[i] = i + 0.5;
    assert.equal(minFinite(values), 0.5);
    assert.equal(maxFinite(values), n - 0.5);
  }
});

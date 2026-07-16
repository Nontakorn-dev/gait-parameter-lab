import test from 'node:test';
import assert from 'node:assert/strict';

import { resolveMarkerRoles, formatUnresolvedError, ROLES } from './markerRoles.js';

test('resolveMarkerRoles: auto-match ชื่อมาตรฐานครบ 6 จุด', () => {
  const markers = [
    { id: 1, name: 'L_ASIS' },
    { id: 2, name: 'R_ASIS' },
    { id: 3, name: 'L_Knee' },
    { id: 4, name: 'R_Knee' },
    { id: 5, name: 'L_Ankle' },
    { id: 6, name: 'R_Ankle' },
  ];
  const { resolved, ok } = resolveMarkerRoles(markers);
  assert.equal(ok, true);
  assert.deepEqual(resolved, {
    L_ASIS: 1, R_ASIS: 2, L_Knee: 3, R_Knee: 4, L_Ankle: 5, R_Ankle: 6,
  });
});

test('resolveMarkerRoles: รองรับชื่อย่อ LASI/RKNE/LANK', () => {
  const markers = [
    { id: 10, name: 'LASI' },
    { id: 11, name: 'RASI' },
    { id: 12, name: 'LKNE' },
    { id: 13, name: 'RKNE' },
    { id: 14, name: 'LANK' },
    { id: 15, name: 'RANK' },
  ];
  const { ok, resolved } = resolveMarkerRoles(markers);
  assert.equal(ok, true);
  assert.equal(resolved.L_ASIS, 10);
  assert.equal(resolved.R_Ankle, 15);
});

test('resolveMarkerRoles: Marker-N generic ต้อง fail ดัง (ไม่เดา)', () => {
  const markers = Array.from({ length: 6 }, (_, i) => ({ id: i + 1, name: `Marker-${i + 1}` }));
  const { ok, unresolved } = resolveMarkerRoles(markers);
  assert.equal(ok, false);
  assert.equal(unresolved.length, ROLES.length);
});

test('resolveMarkerRoles: overrideMap มี priority สูงกว่าชื่อ', () => {
  const markers = [
    { id: 1, name: 'Marker-1' },
    { id: 2, name: 'Marker-2' },
    { id: 3, name: 'L_Knee' },
    { id: 4, name: 'R_Knee' },
    { id: 5, name: 'L_Ankle' },
    { id: 6, name: 'R_Ankle' },
  ];
  const { ok, resolved } = resolveMarkerRoles(markers, { L_ASIS: 1, R_ASIS: 2 });
  assert.equal(ok, true);
  assert.equal(resolved.L_ASIS, 1);
  assert.equal(resolved.R_ASIS, 2);
});

test('formatUnresolvedError: มี list marker ทั้งหมด', () => {
  const markers = [{ id: 1, name: 'Marker-1', seenCount: 10 }];
  const text = formatUnresolvedError(
    [{ role: 'L_ASIS', matchCount: 0, candidates: [] }],
    markers,
  );
  assert.match(text, /L_ASIS/);
  assert.match(text, /Marker-1/);
});

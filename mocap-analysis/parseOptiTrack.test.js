import test from 'node:test';
import assert from 'node:assert/strict';

import { parseOptiTrackCsv, listMarkers, extractMarkerSeries, interpolateGaps } from './parseOptiTrack.js';

function buildCsv(frameLines) {
  return ['righthanded', ...frameLines].join('\n');
}

function frameLine(idx, time, markers) {
  const fields = markers.map((m) => `${m.x},${m.y},${m.z},${m.id},${m.name}`).join(',');
  return `frame,${idx},${time},0,${markers.length},${fields}`;
}

test('parseOptiTrackCsv: parse frame/marker พื้นฐาน + ประมาณ frame rate จาก timestamp จริง', () => {
  const csv = buildCsv([
    frameLine(0, '0.00000000', [{ x: 1, y: 2, z: 3, id: 1, name: 'M1' }]),
    frameLine(1, '0.00833333', [{ x: 1.1, y: 2.1, z: 3.1, id: 1, name: 'M1' }]),
    frameLine(2, '0.01666667', [{ x: 1.2, y: 2.2, z: 3.2, id: 1, name: 'M1' }]),
  ]);
  const parsed = parseOptiTrackCsv(csv);
  assert.equal(parsed.handedness, 'right');
  assert.equal(parsed.frameCount, 3);
  assert.ok(Math.abs(parsed.frameRateHz - 120) < 0.5, `frameRateHz=${parsed.frameRateHz}`);
  assert.equal(parsed.frames[1].markers[0].name, 'M1');
  assert.equal(parsed.frames[1].markers[0].x, 1.1);
});

test('parseOptiTrackCsv: ข้าม comment/info lines, ไม่ throw', () => {
  const csv = [
    'comment,"OptiTrack Export Data"',
    'info,version,1.1',
    'info,framecount,1',
    'righthanded',
    frameLine(0, '0.0', [{ x: 0, y: 0, z: 0, id: 1, name: 'A' }]),
  ].join('\n');
  const parsed = parseOptiTrackCsv(csv);
  assert.equal(parsed.frameCount, 1);
});

test('parseOptiTrackCsv: rigidBodyCount>0 ต้อง skip field ให้ถูก offset (ไม่ปนกับ marker fields)', () => {
  // rigid body 1 ตัว = id,x,y,z,qx,qy,qz,qw,yaw,pitch,roll (11 field) ก่อนถึง markerCount
  const rb = '1,0.1,0.2,0.3,0,0,0,1,0,0,0';
  const line = `frame,0,0.0,1,${rb},1,5,6,7,1,MK1`;
  const parsed = parseOptiTrackCsv(buildCsv([line]));
  assert.equal(parsed.frames[0].rigidBodies.length, 1);
  assert.equal(parsed.frames[0].rigidBodies[0].id, 1);
  assert.equal(parsed.frames[0].markers.length, 1);
  assert.equal(parsed.frames[0].markers[0].x, 5);
  assert.equal(parsed.frames[0].markers[0].name, 'MK1');
});

test('listMarkers: รวมทุก id ที่เคยปรากฏ แม้บางเฟรมจะไม่มี (ถูกบดบัง)', () => {
  const csv = buildCsv([
    frameLine(0, '0.0', [{ x: 0, y: 0, z: 0, id: 1, name: 'A' }, { x: 0, y: 0, z: 0, id: 2, name: 'B' }]),
    frameLine(1, '0.01', [{ x: 0, y: 0, z: 0, id: 1, name: 'A' }]), // marker B หายไปเฟรมนี้
  ]);
  const parsed = parseOptiTrackCsv(csv);
  const markers = listMarkers(parsed);
  assert.equal(markers.length, 2);
  assert.equal(markers.find((m) => m.id === 1).seenCount, 2);
  assert.equal(markers.find((m) => m.id === 2).seenCount, 1);
});

test('extractMarkerSeries: เฟรมที่ marker หายได้ null ไม่ throw, index ตรงกับ parsed.frames เสมอ', () => {
  const csv = buildCsv([
    frameLine(0, '0.0', [{ x: 1, y: 1, z: 1, id: 5, name: 'X' }]),
    frameLine(1, '0.01', []), // marker หายไปทั้งเฟรม
    frameLine(2, '0.02', [{ x: 3, y: 3, z: 3, id: 5, name: 'X' }]),
  ]);
  const parsed = parseOptiTrackCsv(csv);
  const series = extractMarkerSeries(parsed, 5);
  assert.equal(series.x.length, 3);
  assert.equal(series.x[0], 1);
  assert.equal(series.x[1], null);
  assert.equal(series.x[2], 3);
});

test('interpolateGaps: เติม gap สั้น ๆ ด้วย linear interpolation', () => {
  const series = { t: [0, 1, 2, 3, 4], x: [0, null, null, null, 4], y: [0, null, null, null, 4], z: [0, 0, 0, 0, 0] };
  const { series: filled, remainingGaps } = interpolateGaps(series, 10);
  assert.equal(remainingGaps.length, 0);
  assert.deepEqual(filled.x, [0, 1, 2, 3, 4]);
});

test('interpolateGaps: gap ยาวเกิน maxGapFrames ไม่ถูกเติม และถูกรายงานเป็น remainingGaps', () => {
  const series = { t: [0, 1, 2, 3, 4, 5], x: [0, null, null, null, null, 5], y: [0, null, null, null, null, 5], z: [0, 0, 0, 0, 0, 0] };
  const { series: filled, remainingGaps } = interpolateGaps(series, 2); // gap ยาว 4 > cap 2
  assert.equal(remainingGaps.length, 1);
  assert.equal(remainingGaps[0].startIdx, 1);
  assert.equal(remainingGaps[0].endIdx, 4);
  assert.equal(filled.x[2], null, 'gap ที่ยาวเกินต้องไม่ถูกเติมทับเงียบ ๆ');
});

test('interpolateGaps: gap ที่ต้นข้อมูล (ไม่มี prevIdx) ไม่ถูกเติม ถูกรายงานแทน', () => {
  const series = { t: [0, 1, 2], x: [null, null, 2], y: [null, null, 2], z: [0, 0, 0] };
  const { remainingGaps } = interpolateGaps(series, 10);
  assert.equal(remainingGaps.length, 1);
});

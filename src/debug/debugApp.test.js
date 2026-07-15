import test from 'node:test';
import assert from 'node:assert/strict';

import {
  detectSagittalAxis,
  peakAbsFrom,
  extractAxes,
  updateTracker,
} from './debugApp.js';

function emptyTracker() {
  const zero = () => ({ ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0 });
  return { latest: {}, peakPos: zero(), peakNeg: zero(), usingPreRemap: false, sampleCount: 0 };
}

function feedGyroX(tracker, dpsSeq) {
  for (const dps of dpsSeq) {
    updateTracker(tracker, { rawGyroSensor: [dps * 16.4, 0, 0], rawAccelSensor: [0, 0, 0] });
  }
}

test('ท่า 1: detectSagittalAxis คืน gx เมื่อ gx peak สูงสุด (✅)', () => {
  const r = detectSagittalAxis({ gx: 320, gy: 40, gz: 25 });
  assert.equal(r.axis, 'gx');
  assert.equal(r.isCorrect, true);
});

test('ท่า 1: ถ้า gz สูงสุด = ติดตั้งหมุนแกน ต้อง remap (❌)', () => {
  const r = detectSagittalAxis({ gx: 30, gy: 20, gz: 300 });
  assert.equal(r.axis, 'gz');
  assert.equal(r.isCorrect, false);
});

test('peakAbsFrom รวม |peak+| กับ |peak−| เป็น magnitude ต่อแกน', () => {
  const abs = peakAbsFrom({ gx: 200, gy: 0, gz: 0 }, { gx: -260, gy: 0, gz: 0 });
  assert.equal(abs.gx, 260, 'ใช้ค่ามากสุดของสองทิศ');
});

test('regression: เก็บ peak บวก/ลบ แยกกัน — เคสที่เคย false-alarm mirror', () => {
  // ทั้งสองข้างติดตั้งเหมือนกัน: ไปหน้า(+) แล้วดีดกลับ(−) ที่แรงกว่า
  const L = emptyTracker();
  const R = emptyTracker();
  feedGyroX(L, [0, 200, 0, -260, 0]); // เคยจับ peakSigned = -260 (จังหวะดีดกลับ)
  feedGyroX(R, [0, 240, 0, -180, 0]); // เคยจับ peakSigned = +240 (จังหวะไปหน้า)

  // ตอนนี้เก็บทั้งสองทิศ ทั้งสองข้างจึงเห็นทั้ง +peak และ −peak (pattern เหมือนกัน)
  assert.equal(L.peakPos.gx, 200);
  assert.equal(L.peakNeg.gx, -260);
  assert.equal(R.peakPos.gx, 240);
  assert.equal(R.peakNeg.gx, -180);
  // ไม่มีการ auto-สรุป mirror จาก peak อีกต่อไป (ผู้ใช้อ่าน live เอง)
});

test('extractAxes ใช้ raw ก่อน remap เมื่อมี (usingPreRemap=true)', () => {
  const sample = {
    rawAccelSensor: [4096, 0, 0], rawGyroSensor: [164, 0, 0],
    ax: -4096, ay: 0, az: 0, gx: -164, gy: 0, gz: 0,
  };
  const { usingPreRemap, values } = extractAxes(sample);
  assert.equal(usingPreRemap, true);
  assert.ok(Math.abs(values.ax - 1) < 1e-9, 'ax = 4096/4096 = 1g');
  assert.ok(Math.abs(values.gx - 10) < 1e-9, 'gx = 164/16.4 = 10 dps');
});

test('extractAxes fallback เป็น canonical เมื่อไม่มี sensor-frame (demo)', () => {
  const { usingPreRemap, values } = extractAxes({ ax: 4096, ay: 0, az: 0, gx: 328, gy: 0, gz: 0 });
  assert.equal(usingPreRemap, false);
  assert.ok(Math.abs(values.gx - 20) < 1e-9);
});

test('updateTracker: sagittal detection ผ่าน peakAbsFrom หลังแกว่ง', () => {
  const tracker = emptyTracker();
  feedGyroX(tracker, [0, 100, 300, -250, 0]);
  const sag = detectSagittalAxis(peakAbsFrom(tracker.peakPos, tracker.peakNeg));
  assert.equal(sag.axis, 'gx');
  assert.equal(sag.value, 300);
});

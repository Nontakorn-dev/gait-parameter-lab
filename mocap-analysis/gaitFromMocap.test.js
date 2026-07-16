import test from 'node:test';
import assert from 'node:assert/strict';

import { parseOptiTrackCsv, listMarkers } from './parseOptiTrack.js';
import { resolveMarkerRoles } from './markerRoles.js';
import { computeGaitFromMocap, computeForwardAxis, computeShankAngleDeg, computeAngularVelocityDps } from './gaitFromMocap.js';

// ==================================================================================
// สร้างชุดข้อมูล marker สังเคราะห์ที่มี "ground truth" รู้ค่าแน่นอน (stride length,
// cadence, peak shank angle, ankle clearance) แล้วป้อนผ่าน pipeline เต็ม (parse CSV
// จริง -> resolve marker role จากชื่อ -> คำนวณ gait parameter) เพื่อพิสูจน์ว่าคณิตศาสตร์
// (projection บนแกนเดิน, การหามุม, differentiate, event detection, stride length จาก
// การขยับของข้อเท้า) ถูกต้อง ก่อนข้อมูลจริงจาก mocap จะมาถึง
//
// รูปคลื่นความเร็วเชิงมุม (gx shape) จงใจ "มิเรอร์" ฟังก์ชัน generateShankAngularVelocity
// ใน src/gait-dashboard/data/demoDataGenerator.js (peak=350, hsDip=-180, stancePct=0.62)
// เพราะเป็นรูปคลื่นเดียวที่ DEFAULT_MOCAP_DETECTOR_OPTIONS (คัดลอกมาจาก
// PATIENT_EVENT_DETECTOR_OPTIONS) ถูก tune มาให้ตรงอยู่แล้ว — สร้างมุมเองแบบสุ่มจะทำให้
// เทสต์นี้พิสูจน์ไม่ได้ว่า pipeline ถูกจริงหรือแค่บังเอิญ threshold match กัน
function gxShape(phase, stancePct, peak, hsDip) {
  if (phase < 0.05) {
    const p = phase / 0.05;
    return hsDip * Math.exp(-p * 3) * Math.cos(p * Math.PI);
  }
  if (phase < 0.15) {
    const p = (phase - 0.05) / 0.10;
    return -20 + 60 * p;
  }
  if (phase < stancePct * 0.6) {
    const p = (phase - 0.15) / (stancePct * 0.6 - 0.15);
    return 40 + 20 * Math.sin(p * Math.PI);
  }
  if (phase < stancePct) {
    const p = (phase - stancePct * 0.6) / (stancePct - stancePct * 0.6);
    let v = 60 - 80 * Math.sin(p * Math.PI * 0.7);
    if (p > 0.3 && p < 0.7) v -= 40 * Math.exp(-1 * ((p - 0.5) / 0.1) ** 2);
    return v;
  }
  const swingPhase = (phase - stancePct) / (1.0 - stancePct);
  let v = peak * Math.sin(swingPhase * Math.PI);
  if (swingPhase > 0.75) {
    const termPhase = (swingPhase - 0.75) / 0.25;
    v -= (peak + Math.abs(hsDip)) * termPhase * termPhase;
  }
  return v;
}

const SAMPLE_RATE = 120;
const STANCE_PCT = 0.62;
const STRIDE_TIME_S = 1.1; // -> cadence เดี่ยว ๆ ที่คาด = 2/1.1*60 = 109.09 spm
const STRIDE_LENGTH_M = 1.3; // ground truth ที่จะตรวจสอบ
const SHANK_LENGTH_M = 0.42;
const SWING_PEAK_HEIGHT_M = 0.05; // ground truth ankle clearance
const NUM_STRIDES = 8;
const WALK_SPEED_MPS = STRIDE_LENGTH_M / STRIDE_TIME_S;

// per-stride velocity samples ถูก detrend (ลบ mean) ก่อน integrate เป็นมุม เพื่อไม่ให้มุม
// ไหลสะสมข้าม stride (ฟังก์ชันต้นทางออกแบบมาให้ signal สมจริงสำหรับ detect event เท่านั้น
// ไม่ได้ออกแบบให้ผลรวมต่อรอบเป็นศูนย์พอดี — เดินจริงมุมหน้าแข้งกลับที่เดิมทุก heel-strike)
function buildDetrendedVelocityTemplate() {
  const samplesPerStride = Math.round(STRIDE_TIME_S * SAMPLE_RATE);
  const raw = [];
  for (let i = 0; i < samplesPerStride; i += 1) {
    raw.push(gxShape(i / samplesPerStride, STANCE_PCT, 350, -180));
  }
  const meanV = raw.reduce((a, b) => a + b, 0) / raw.length;
  return raw.map((v) => v - meanV);
}

function buildLegTrajectory(phaseOffsetS, footprintStartX) {
  const velocityTemplate = buildDetrendedVelocityTemplate();
  const samplesPerStride = velocityTemplate.length;
  const totalDurationS = NUM_STRIDES * STRIDE_TIME_S + 1;
  const n = Math.round(totalDurationS * SAMPLE_RATE);

  const traj = [];
  let angleDeg = 0;
  for (let i = 0; i < n; i += 1) {
    const t = i / SAMPLE_RATE;
    const cyclePos = (t - phaseOffsetS) / STRIDE_TIME_S;
    const cycleIndex = Math.floor(cyclePos);
    const phase = cyclePos - cycleIndex;
    const sampleIdx = ((Math.round(phase * samplesPerStride) % samplesPerStride) + samplesPerStride) % samplesPerStride;

    if (i > 0) angleDeg += velocityTemplate[sampleIdx] / SAMPLE_RATE;

    const footprintX = footprintStartX + cycleIndex * STRIDE_LENGTH_M;
    let ankleX;
    let ankleY;
    if (phase < STANCE_PCT) {
      ankleX = footprintX;
      ankleY = 0;
    } else {
      const p = (phase - STANCE_PCT) / (1 - STANCE_PCT);
      const ease = (1 - Math.cos(p * Math.PI)) / 2;
      ankleX = footprintX + STRIDE_LENGTH_M * ease;
      ankleY = SWING_PEAK_HEIGHT_M * Math.sin(p * Math.PI);
    }

    const rad = (angleDeg * Math.PI) / 180;
    const kneeX = ankleX + SHANK_LENGTH_M * Math.sin(rad);
    const kneeY = ankleY + SHANK_LENGTH_M * Math.cos(rad);

    traj.push({ t, ankleX, ankleY, ankleZ: 0, kneeX, kneeY, kneeZ: 0, angleDeg });
  }
  return traj;
}

function buildSyntheticCsv() {
  const left = buildLegTrajectory(0, 0);
  const right = buildLegTrajectory(STRIDE_TIME_S / 2, STRIDE_LENGTH_M / 2);
  const n = left.length;

  const markerDefs = [
    { id: 1, name: 'L_ASIS' },
    { id: 2, name: 'R_ASIS' },
    { id: 3, name: 'L_Knee' },
    { id: 4, name: 'R_Knee' },
    { id: 5, name: 'L_Ankle' },
    { id: 6, name: 'R_Ankle' },
  ];

  const lines = ['righthanded'];
  for (let i = 0; i < n; i += 1) {
    const t = i / SAMPLE_RATE;
    const asisX = WALK_SPEED_MPS * t;
    const values = {
      1: [asisX, 0.95, -0.1],
      2: [asisX, 0.95, 0.1],
      3: [left[i].kneeX, left[i].kneeY, -0.1],
      4: [right[i].kneeX, right[i].kneeY, 0.1],
      5: [left[i].ankleX, left[i].ankleY, -0.1],
      6: [right[i].ankleX, right[i].ankleY, 0.1],
    };
    const markerFields = markerDefs
      .map((m) => `${values[m.id][0].toFixed(6)},${values[m.id][1].toFixed(6)},${values[m.id][2].toFixed(6)},${m.id},${m.name}`)
      .join(',');
    lines.push(`frame,${i},${t.toFixed(8)},0,${markerDefs.length},${markerFields}`);
  }
  return { csv: lines.join('\n'), left, right };
}

test('computeForwardAxis: pelvis เดินตรงไปตามแกน +X ต้องได้ fx≈1, fz≈0', () => {
  const pelvisX = [0, 0.5, 1.0, 1.5, 2.0];
  const pelvisZ = [0.1, 0.1, 0.1, 0.1, 0.1];
  const axis = computeForwardAxis(pelvisX, pelvisZ);
  assert.ok(Math.abs(axis.fx - 1) < 1e-9);
  assert.ok(Math.abs(axis.fz) < 1e-9);
  assert.ok(Math.abs(axis.netDisplacementM - 2.0) < 1e-9);
});

test('computeForwardAxis: throw ถ้าระยะทางเดินสุทธิสั้นเกินไป (< 0.3m)', () => {
  assert.throws(() => computeForwardAxis([0, 0.05, 0.1], [0, 0, 0]));
});

test('computeShankAngleDeg + computeAngularVelocityDps: round-trip กลับมุมที่ป้อนเข้าไปได้ถูกต้อง', () => {
  // สร้าง knee/ankle จากมุมที่รู้ค่าแน่นอน (0°, 30°, -20°) แล้วเช็คว่าคำนวณกลับได้ตรง
  const forwardAxis = { fx: 1, fz: 0 };
  const angles = [0, 30, -20, 10];
  const knee = { x: [], y: [], z: [] };
  const ankle = { x: [], y: [], z: [] };
  for (const deg of angles) {
    const rad = (deg * Math.PI) / 180;
    ankle.x.push(0); ankle.y.push(0); ankle.z.push(0);
    knee.x.push(0.42 * Math.sin(rad));
    knee.y.push(0.42 * Math.cos(rad));
    knee.z.push(0);
  }
  const result = computeShankAngleDeg(knee, ankle, forwardAxis);
  for (let i = 0; i < angles.length; i += 1) {
    assert.ok(Math.abs(result[i] - angles[i]) < 1e-6, `angle[${i}]=${result[i]} ควรเป็น ${angles[i]}`);
  }
});

test('regression: parser ต้องไม่ throw stack overflow ที่ marker id ไม่ตรงลำดับ/มี rigidBodyCount>0', () => {
  // ป้องกันการเดา offset ผิดถ้ามี rigid body ปนมาในอนาคต — เทสต์แค่ parse ไม่ crash
  const line = 'frame,0,0.0,1,99,x,y,z,1.0,2.0,3.0,0,0,0,1,0,0,0,1,6,0.1,0.2,0.3,1,M1,0.4,0.5,0.6,2,M2,0.7,0.8,0.9,3,M3,1.0,1.1,1.2,4,M4,1.3,1.4,1.5,5,M5,1.6,1.7,1.8,6,M6';
  assert.doesNotThrow(() => parseOptiTrackCsv(`righthanded\n${line}`));
});

test('end-to-end: parse CSV จริง -> resolve role จากชื่อ -> คำนวณ gait parameter ตรงกับ ground truth', () => {
  const { csv } = buildSyntheticCsv();
  const parsed = parseOptiTrackCsv(csv);

  assert.ok(Math.abs(parsed.frameRateHz - SAMPLE_RATE) < 0.5, `frameRateHz=${parsed.frameRateHz} ควร ~${SAMPLE_RATE}`);

  const markers = listMarkers(parsed);
  assert.equal(markers.length, 6);

  const { resolved, ok, unresolved } = resolveMarkerRoles(markers);
  assert.equal(ok, true, `resolve ไม่ครบ: ${JSON.stringify(unresolved)}`);
  assert.deepEqual(Object.keys(resolved).sort(), ['L_ASIS', 'L_Ankle', 'L_Knee', 'R_ASIS', 'R_Ankle', 'R_Knee'].sort());

  const result = computeGaitFromMocap(parsed, resolved);

  // --- forward axis: เดินตรงตามแกน +X ---
  assert.ok(Math.abs(result.forwardAxis.fx - 1) < 0.02, `fx=${result.forwardAxis.fx}`);
  assert.ok(Math.abs(result.forwardAxis.fz) < 0.02, `fz=${result.forwardAxis.fz}`);

  // --- ต้องเจอ cycle อย่างน้อยหลายรอบทั้งสองขา ---
  assert.ok(result.perSide.L.cycles.length >= NUM_STRIDES - 3, `L cycles=${result.perSide.L.cycles.length}`);
  assert.ok(result.perSide.R.cycles.length >= NUM_STRIDES - 3, `R cycles=${result.perSide.R.cycles.length}`);

  // --- stride length ต้องใกล้ ground truth (1.3m) ---
  for (const side of ['L', 'R']) {
    const mean = result.perSide[side].summary.meanStrideLengthM;
    assert.ok(Math.abs(mean - STRIDE_LENGTH_M) < 0.1,
      `${side} meanStrideLengthM=${mean} ควรใกล้ ${STRIDE_LENGTH_M}`);
  }

  // --- cadence ต้องใกล้ ground truth (109.09 spm) ---
  const expectedCadence = (2 / STRIDE_TIME_S) * 60;
  for (const side of ['L', 'R']) {
    const mean = result.perSide[side].summary.meanCadenceSpm;
    assert.ok(Math.abs(mean - expectedCadence) < 3,
      `${side} meanCadenceSpm=${mean} ควรใกล้ ${expectedCadence.toFixed(1)}`);
  }

  // --- ankle clearance ต้องใกล้ ground truth (0.05m) ---
  const clearanceL = result.perSide.L.cycles.map((c) => c.ankleClearanceM);
  const meanClearanceL = clearanceL.reduce((a, b) => a + b, 0) / clearanceL.length;
  assert.ok(Math.abs(meanClearanceL - SWING_PEAK_HEIGHT_M) < 0.02,
    `meanClearanceL=${meanClearanceL} ควรใกล้ ${SWING_PEAK_HEIGHT_M}`);

  // --- gait สมมาตรสมบูรณ์แบบ (ตั้งใจให้เท่ากันเป๊ะ) -> bilateral trueCadence ต้องใกล้ per-leg cadence ---
  assert.ok(Math.abs(result.bilateral.trueCadenceSpm - expectedCadence) < 3,
    `trueCadenceSpm=${result.bilateral.trueCadenceSpm} ควรใกล้ ${expectedCadence.toFixed(1)} (สมมาตรสมบูรณ์แบบ)`);
  assert.equal(result.bilateral.sameSideRepeats, 0, 'gait สลับซ้าย-ขวาสมบูรณ์แบบ ไม่ควรมี same-side ติดกัน');

  // --- session summary สมเหตุสมผล ---
  assert.ok(Math.abs(result.session.averageWalkingSpeedMps - WALK_SPEED_MPS) < 0.1,
    `averageWalkingSpeedMps=${result.session.averageWalkingSpeedMps} ควรใกล้ ${WALK_SPEED_MPS.toFixed(2)}`);
});

test('รู้ข้อจำกัด: TO/stancePct มัก "unresolved" กับสัญญาณสังเคราะห์ — ต้องไม่ crash และไม่โผล่เป็นเลขมั่ว', () => {
  // ยืนยันแล้ว (ดู comment หัวไฟล์ gaitFromMocap.js): findLocalMinima ใช้ minDistance=10
  // samples ตายตัว ไม่ scale ตาม sampleRate — ที่ 120Hz มักหา TO ไม่เจอ หรือถ้าลด threshold
  // จะเจอผิดจุด (phase~0.50 แทนที่จะเป็น 0.62 ที่เป็น ground truth) ค่าที่ปลอดภัยคือ
  // temporalSource='unresolved' + stancePct/swingPct เป็น null ไม่ใช่เลขมั่ว ๆ
  const { csv } = buildSyntheticCsv();
  const parsed = parseOptiTrackCsv(csv);
  const markers = listMarkers(parsed);
  const { resolved } = resolveMarkerRoles(markers);
  const result = computeGaitFromMocap(parsed, resolved);

  for (const side of ['L', 'R']) {
    for (const cycle of result.perSide[side].cycles) {
      if (cycle.temporalSource === 'unresolved') {
        assert.equal(cycle.stancePct, null, 'unresolved ต้องไม่มี stancePct หลอก ๆ');
        assert.equal(cycle.swingPct, null);
      } else {
        assert.ok(Number.isFinite(cycle.stancePct), 'ถ้า resolve ได้ต้องเป็นตัวเลขจริง');
      }
      // ไม่ว่า TO จะ resolve ได้ไหม stride length/cadence/clearance ต้องยังคำนวณได้เสมอ
      assert.ok(Number.isFinite(cycle.strideLengthM));
      assert.ok(Number.isFinite(cycle.cadenceSpm));
      assert.ok(Number.isFinite(cycle.ankleClearanceM));
    }
  }
});

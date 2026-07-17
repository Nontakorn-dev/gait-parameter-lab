import test from 'node:test';
import assert from 'node:assert/strict';

import { parseOptiTrackCsv, listMarkers } from './parseOptiTrack.js';
import { resolveMarkerRoles } from './markerRoles.js';
import { computeGaitFromMocap, computeForwardAxis, computeShankAngleDeg, computeAngularVelocityDps } from './gaitFromMocap.js';
import { FIXTURE, buildSyntheticCsv } from './testFixtures.js';

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
  // ยังคำนวณ angular velocity ได้ (เผื่อใช้ direct signal-correlation ในอนาคต แม้ไม่ได้ใช้หา event แล้ว)
  const t = angles.map((_, i) => i * 0.01);
  const w = computeAngularVelocityDps(t, result);
  assert.ok(w.every(Number.isFinite));
});

test('regression: parser ต้องไม่ throw stack overflow ที่ marker id ไม่ตรงลำดับ/มี rigidBodyCount>0', () => {
  // ป้องกันการเดา offset ผิดถ้ามี rigid body ปนมาในอนาคต — เทสต์แค่ parse ไม่ crash
  const line = 'frame,0,0.0,1,99,x,y,z,1.0,2.0,3.0,0,0,0,1,0,0,0,1,6,0.1,0.2,0.3,1,M1,0.4,0.5,0.6,2,M2,0.7,0.8,0.9,3,M3,1.0,1.1,1.2,4,M4,1.3,1.4,1.5,5,M5,1.6,1.7,1.8,6,M6';
  assert.doesNotThrow(() => parseOptiTrackCsv(`righthanded\n${line}`));
});

test('end-to-end: parse CSV จริง -> resolve role จากชื่อ -> คำนวณ gait parameter ตรงกับ ground truth', () => {
  const { csv } = buildSyntheticCsv();
  const parsed = parseOptiTrackCsv(csv);

  assert.ok(Math.abs(parsed.frameRateHz - FIXTURE.SAMPLE_RATE) < 0.5, `frameRateHz=${parsed.frameRateHz} ควร ~${FIXTURE.SAMPLE_RATE}`);

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
  assert.ok(result.perSide.L.cycles.length >= FIXTURE.NUM_STRIDES - 3, `L cycles=${result.perSide.L.cycles.length}`);
  assert.ok(result.perSide.R.cycles.length >= FIXTURE.NUM_STRIDES - 3, `R cycles=${result.perSide.R.cycles.length}`);

  // --- stride length ต้องใกล้ ground truth (1.3m) ---
  for (const side of ['L', 'R']) {
    const mean = result.perSide[side].summary.meanStrideLengthM;
    assert.ok(Math.abs(mean - FIXTURE.STRIDE_LENGTH_M) < 0.1,
      `${side} meanStrideLengthM=${mean} ควรใกล้ ${FIXTURE.STRIDE_LENGTH_M}`);
  }

  // --- cadence ต้องใกล้ ground truth (109.09 spm) ---
  for (const side of ['L', 'R']) {
    const mean = result.perSide[side].summary.meanCadenceSpm;
    assert.ok(Math.abs(mean - FIXTURE.EXPECTED_CADENCE_SPM) < 3,
      `${side} meanCadenceSpm=${mean} ควรใกล้ ${FIXTURE.EXPECTED_CADENCE_SPM.toFixed(1)}`);
  }

  // --- stancePct: foot-velocity มี bias ~−3% vs synthetic GT (threshold+Butterworth)
  // tolerance <3 คือ agreement กับ method ไม่ใช่ absolute GT accuracy ---
  for (const side of ['L', 'R']) {
    for (const cycle of result.perSide[side].cycles) {
      assert.ok(Number.isFinite(cycle.stancePct), `${side} stancePct ต้อง resolve ได้เสมอ`);
      assert.ok(Math.abs(cycle.stancePct - FIXTURE.STANCE_PCT * 100) < 3,
        `${side} stancePct=${cycle.stancePct.toFixed(1)} agreement กับ foot-velocity `
        + `(GT ${(FIXTURE.STANCE_PCT * 100).toFixed(1)}; bias method ~2–3%)`);
    }
  }

  // --- ankle clearance ต้องใกล้ ground truth (0.05m) ---
  const clearanceL = result.perSide.L.cycles.map((c) => c.ankleClearanceM);
  const meanClearanceL = clearanceL.reduce((a, b) => a + b, 0) / clearanceL.length;
  assert.ok(Math.abs(meanClearanceL - FIXTURE.SWING_PEAK_HEIGHT_M) < 0.02,
    `meanClearanceL=${meanClearanceL} ควรใกล้ ${FIXTURE.SWING_PEAK_HEIGHT_M}`);

  // --- gait สมมาตรสมบูรณ์แบบ (ตั้งใจให้เท่ากันเป๊ะ) -> bilateral trueCadence ต้องใกล้ per-leg cadence ---
  assert.ok(Math.abs(result.bilateral.trueCadenceSpm - FIXTURE.EXPECTED_CADENCE_SPM) < 3,
    `trueCadenceSpm=${result.bilateral.trueCadenceSpm} ควรใกล้ ${FIXTURE.EXPECTED_CADENCE_SPM.toFixed(1)} (สมมาตรสมบูรณ์แบบ)`);
  assert.equal(result.bilateral.sameSideRepeats, 0, 'gait สลับซ้าย-ขวาสมบูรณ์แบบ ไม่ควรมี same-side ติดกัน');

  // --- session summary สมเหตุสมผล ---
  assert.ok(Math.abs(result.session.averageWalkingSpeedMps - FIXTURE.WALK_SPEED_MPS) < 0.1,
    `averageWalkingSpeedMps=${result.session.averageWalkingSpeedMps} ควรใกล้ ${FIXTURE.WALK_SPEED_MPS.toFixed(2)}`);

  assert.equal(result.meta.unitScale, 1);
  assert.ok(Array.isArray(result.meta.warnings));
});

test('🟡 units: ไฟล์ mm ถูก auto-scale เป็นเมตร', () => {
  const { csv } = buildSyntheticCsv();
  // คูณพิกัดทั้งหมด ×1000 เพื่อจำลอง Motive export เป็น mm
  const mmCsv = csv.split('\n').map((line) => {
    if (!line.startsWith('frame,')) return line;
    const parts = line.split(',');
    let p = 4; // after frame,idx,time,rbCount
    const rbCount = Number(parts[3]);
    p = 4 + rbCount * 11;
    const markerCount = Number(parts[p]);
    p += 1;
    for (let m = 0; m < markerCount; m += 1) {
      parts[p] = String(Number(parts[p]) * 1000); // x
      parts[p + 1] = String(Number(parts[p + 1]) * 1000); // y
      parts[p + 2] = String(Number(parts[p + 2]) * 1000); // z
      p += 5;
    }
    return parts.join(',');
  }).join('\n');

  const parsed = parseOptiTrackCsv(mmCsv);
  const { resolved, ok } = resolveMarkerRoles(listMarkers(parsed));
  assert.equal(ok, true);
  const result = computeGaitFromMocap(parsed, resolved);
  assert.equal(result.meta.unitScale, 0.001);
  assert.ok(result.meta.warnings.some((w) => /mm/i.test(w)));
  assert.ok(Math.abs(result.perSide.L.summary.meanStrideLengthM - FIXTURE.STRIDE_LENGTH_M) < 0.15,
    `หลัง scale stride ควรใกล้เมตรจริง ได้ ${result.perSide.L.summary.meanStrideLengthM}`);
});

test('🟡 units: autoScaleMillimetres:false ต้อง throw ชัดเจน', () => {
  const { csv } = buildSyntheticCsv();
  const mmCsv = csv.split('\n').map((line) => {
    if (!line.startsWith('frame,')) return line;
    const parts = line.split(',');
    let p = 4;
    const rbCount = Number(parts[3]);
    p = 4 + rbCount * 11;
    const markerCount = Number(parts[p]);
    p += 1;
    for (let m = 0; m < markerCount; m += 1) {
      parts[p] = String(Number(parts[p]) * 1000);
      parts[p + 1] = String(Number(parts[p + 1]) * 1000);
      parts[p + 2] = String(Number(parts[p + 2]) * 1000);
      p += 5;
    }
    return parts.join(',');
  }).join('\n');

  const parsed = parseOptiTrackCsv(mmCsv);
  const { resolved } = resolveMarkerRoles(listMarkers(parsed));
  assert.throws(
    () => computeGaitFromMocap(parsed, resolved, { autoScaleMillimetres: false }),
    /mm/,
  );
});

#!/usr/bin/env node
// CLI: แปลงไฟล์ OptiTrack CSV (marker ASIS/Knee/Ankle x ซ้าย-ขวา) เป็น gait parameter
//
// วิธีใช้:
//   node mocap-analysis/run.js <path/to/mocap.csv> [--map path/to/marker-map.json] [--out path/to/output.json]
//
// marker-map.json (ใช้เมื่อ auto-resolve จากชื่อไม่ได้ หรือได้ผลผิด) มีรูปแบบ:
//   { "L_ASIS": 1, "R_ASIS": 2, "L_Knee": 3, "R_Knee": 4, "L_Ankle": 5, "R_Ankle": 6 }
// (ใส่ id ตัวเลขของ marker ตามที่เห็นตอนรันครั้งแรกแล้ว auto-resolve fail จะมี list ให้)

import { readFileSync, writeFileSync } from 'node:fs';
import { parseOptiTrackCsv, listMarkers } from './parseOptiTrack.js';
import { resolveMarkerRoles, formatUnresolvedError } from './markerRoles.js';
import { computeGaitFromMocap } from './gaitFromMocap.js';

function parseArgs(argv) {
  const args = { file: null, map: null, out: null };
  for (let i = 0; i < argv.length; i += 1) {
    const a = argv[i];
    if (a === '--map') { args.map = argv[++i]; continue; }
    if (a === '--out') { args.out = argv[++i]; continue; }
    if (!args.file && !a.startsWith('--')) { args.file = a; continue; }
  }
  return args;
}

function formatCycleRow(c) {
  return [
    c.hsStartTimeS.toFixed(2).padStart(7),
    c.strideTimeS.toFixed(3).padStart(8),
    c.strideLengthM.toFixed(3).padStart(9),
    Number.isFinite(c.cadenceSpm) ? c.cadenceSpm.toFixed(1).padStart(7) : '—'.padStart(7),
    Number.isFinite(c.walkingSpeedMps) ? c.walkingSpeedMps.toFixed(3).padStart(7) : '—'.padStart(7),
    Number.isFinite(c.peakShankAngleDeg) ? c.peakShankAngleDeg.toFixed(1).padStart(7) : '—'.padStart(7),
    c.ankleClearanceM.toFixed(3).padStart(8),
    Number.isFinite(c.stancePct) ? c.stancePct.toFixed(1).padStart(6) : '—'.padStart(6),
  ].join(' | ');
}

function printSideReport(side, data) {
  console.log(`\n=== ขา ${side} — ${data.cycles.length} cycle ===`);
  if (!data.cycles.length) {
    console.log('  (ไม่พบ cycle เลย — ตรวจ threshold ของ detector หรือดูว่า marker หลุดช่วงเดินหรือเปล่า)');
    return;
  }
  console.log(
    '  hsStart(s) | strideT(s) | stride(m) | cadence | speed(m/s) | peakAngle | clearance(m) | stance%',
  );
  for (const c of data.cycles) console.log('  ' + formatCycleRow(c));
  console.log(
    `  -- เฉลี่ย: stride=${data.summary.meanStrideLengthM?.toFixed(3)}m `
    + `cadence=${data.summary.meanCadenceSpm?.toFixed(1)}spm`,
  );
}

function main() {
  const args = parseArgs(process.argv.slice(2));
  if (!args.file) {
    console.error('ใช้: node mocap-analysis/run.js <path/to/mocap.csv> [--map map.json] [--out output.json]');
    process.exit(1);
  }

  const csv = readFileSync(args.file, 'utf8');
  const parsed = parseOptiTrackCsv(csv);
  console.log(`parsed: ${parsed.frameCount} frames @ ${parsed.frameRateHz?.toFixed(2)}Hz, duration ${parsed.durationS.toFixed(2)}s`);

  const markers = listMarkers(parsed);
  const overrideMap = args.map ? JSON.parse(readFileSync(args.map, 'utf8')) : {};
  const { resolved, unresolved, ok } = resolveMarkerRoles(markers, overrideMap);

  if (!ok) {
    console.error('\n' + formatUnresolvedError(unresolved, markers));
    process.exit(1);
  }

  console.log('\nmarker role ที่ใช้:', JSON.stringify(resolved));

  let result;
  try {
    result = computeGaitFromMocap(parsed, resolved);
  } catch (err) {
    console.error(`\nคำนวณไม่สำเร็จ: ${err.message}`);
    process.exit(1);
  }

  console.log(`\nแกนทิศทางเดิน: fx=${result.forwardAxis.fx.toFixed(3)} fz=${result.forwardAxis.fz.toFixed(3)} `
    + `(ระยะสุทธิ ${result.forwardAxis.netDisplacementM.toFixed(2)}m)`);

  printSideReport('L', result.perSide.L);
  printSideReport('R', result.perSide.R);

  console.log('\n=== Bilateral (รวม HS ทั้งสองข้างตามเวลาจริง — ไม่สมมติสมมาตร) ===');
  console.log(`  step time เฉลี่ย: ${result.bilateral.meanStepTimeS?.toFixed(3)}s -> cadence จริง ${result.bilateral.trueCadenceSpm?.toFixed(1)} spm`);
  if (result.bilateral.sameSideRepeats > 0) {
    console.log(`  ⚠️ พบ HS ข้างเดียวกันติดกัน ${result.bilateral.sameSideRepeats} ครั้ง — อาจมีการ detect พลาดฝั่งใดฝั่งหนึ่ง`);
  }

  console.log('\n=== Session ===');
  console.log(`  duration=${result.session.durationS.toFixed(2)}s, pelvis net forward=${result.session.pelvisNetForwardDisplacementM.toFixed(2)}m, `
    + `avg speed=${result.session.averageWalkingSpeedMps?.toFixed(3)}m/s`);

  const outPath = args.out || args.file.replace(/\.csv$/i, '') + '.gait-params.json';
  writeFileSync(outPath, JSON.stringify(result, null, 2));
  console.log(`\nบันทึกผลเต็มลง: ${outPath}`);
}

main();

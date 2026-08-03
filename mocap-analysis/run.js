#!/usr/bin/env node
// CLI: แปลงไฟล์ OptiTrack CSV (marker ASIS/Knee/Ankle x ซ้าย-ขวา) เป็น gait parameter
//
// วิธีใช้:
//   node mocap-analysis/run.js <path/to/mocap.csv> [--map path/to/marker-map.json] [--out path/to/output.json]
//
// marker-map.json (ใช้เมื่อ auto-resolve จากชื่อไม่ได้ หรือได้ผลผิด) มีรูปแบบ:
//   legacy: { "L_ASIS": 1, "R_ASIS": 2, "L_Knee": 3, "R_Knee": 4, "L_Ankle": 5, "R_Ankle": 6 }
//   split:  { ..., "L_AnkleForHS": 6, "R_AnkleForHS": 4, "L_AnkleForAngle": 9, "R_AnkleForAngle": 10 }
//     ForHS = heel (foot-velocity HS); ForAngle = lateral malleolus (shank ω / signed sync)
// (ใส่ id ตัวเลขของ marker ตามที่เห็นตอนรันครั้งแรกแล้ว auto-resolve fail จะมี list ให้)

import { readFileSync, writeFileSync } from 'node:fs';
import { parseOptiTrackCsv, listMarkers } from './parseOptiTrack.js';
import { resolveMarkerRoles, formatUnresolvedError } from './markerRoles.js';
import { computeGaitFromMocap } from './gaitFromMocap.js';

function parseArgs(argv) {
  const args = {
    file: null,
    map: null,
    out: null,
    // แพทย์/แลป: Y-up เข้มงวดเป็น default — opt-out ด้วย --allow-non-y-up
    strictVerticalAxis: true,
  };
  for (let i = 0; i < argv.length; i += 1) {
    const a = argv[i];
    if (a === '--map') { args.map = argv[++i]; continue; }
    if (a === '--out') { args.out = argv[++i]; continue; }
    if (a === '--allow-non-y-up') { args.strictVerticalAxis = false; continue; }
    if (a === '--strict-vertical-axis') { args.strictVerticalAxis = true; continue; }
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
    console.error(
      'ใช้: node mocap-analysis/run.js <path/to/mocap.csv> [--map map.json] [--out output.json]\n'
      + '     [--allow-non-y-up]  (opt-out จาก strict Y-up — ใช้เมื่อรู้ว่า capture เป็น Z-up และจะ remap เอง)',
    );
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
    result = computeGaitFromMocap(parsed, resolved, {
      strictVerticalAxis: args.strictVerticalAxis,
    });
  } catch (err) {
    console.error(`\nคำนวณไม่สำเร็จ: ${err.message}`);
    process.exit(1);
  }

  console.log(`\nแกนทิศทางเดิน: fx=${result.forwardAxis.fx.toFixed(3)} fz=${result.forwardAxis.fz.toFixed(3)} `
    + `method=${result.forwardAxis.method} `
    + `(net ${result.forwardAxis.netDisplacementM.toFixed(2)}m, maxExcursion ${result.forwardAxis.maxExcursionM.toFixed(2)}m)`);

  if (result.meta?.warnings?.length) {
    console.log('\nคำเตือน:');
    for (const w of result.meta.warnings) console.log(`  ⚠️ ${w}`);
  }
  if (result.meta?.unitScale && result.meta.unitScale !== 1) {
    console.log(`  (unitScale ที่ใช้: ${result.meta.unitScale})`);
  }

  printSideReport('L', result.perSide.L);
  printSideReport('R', result.perSide.R);

  console.log('\n=== Bilateral (รวม HS ทั้งสองข้างตามเวลาจริง — ไม่สมมติสมมาตร) ===');
  if (result.bilateral.reliable === false) {
    console.log(
      `  ⛔ bilateral.reliable=false (sameSideRepeats=${result.bilateral.sameSideRepeats}) `
      + '— ซ่อน meanStepTime / trueCadence; ดู raw ได้ที่ bilateral.*Raw',
    );
  } else {
    console.log(`  step time เฉลี่ย: ${result.bilateral.meanStepTimeS?.toFixed(3)}s -> cadence จริง ${result.bilateral.trueCadenceSpm?.toFixed(1)} spm`);
  }
  if (result.bilateral.sameSideRepeats > 0) {
    console.log(`  ⚠️ พบ HS ข้างเดียวกันติดกัน ${result.bilateral.sameSideRepeats} ครั้ง — อาจมีการ detect พลาดฝั่งใดฝั่งหนึ่ง`);
  }

  console.log('\n=== Session ===');
  const avgSpeed = result.session.averageWalkingSpeedMps;
  console.log(
    `  duration=${result.session.durationS.toFixed(2)}s, `
    + `pelvis net=${result.session.pelvisNetForwardDisplacementM.toFixed(2)}m`
    + (result.session.pelvisNetMeaningful
      ? `, avg speed=${avgSpeed?.toFixed(3)}m/s`
      : ` (net ไม่ใช้ — maxExcursion=${result.session.pelvisMaxExcursionM?.toFixed(2)}m, method=${result.session.forwardAxisMethod})`),
  );
  const outPath = args.out || args.file.replace(/\.csv$/i, '') + '.gait-params.json';
  writeFileSync(outPath, JSON.stringify(result, null, 2));
  console.log(`\nบันทึกผลเต็มลง: ${outPath}`);
}

main();

#!/usr/bin/env node
// เทียบ mocap.gait-params.json กับ IMU trace ที่ export จาก dashboard
//
// วิธีใช้:
//   node mocap-analysis/compare.js <mocap.gait-params.json> <imu-trace.json> [--out report.json]
//
// ตัวอย่าง:
//   node mocap-analysis/compare.js mocap.gait-params.json ~/Downloads/gait-trace-....json

import { readFileSync, writeFileSync } from 'node:fs';
import { compareMocapToImu } from './compareImuTrace.js';

function parseArgs(argv) {
  const args = { mocap: null, imu: null, out: null };
  for (let i = 0; i < argv.length; i += 1) {
    const a = argv[i];
    if (a === '--out') { args.out = argv[++i]; continue; }
    if (!args.mocap && !a.startsWith('--')) { args.mocap = a; continue; }
    if (!args.imu && !a.startsWith('--')) { args.imu = a; continue; }
  }
  return args;
}

function fmt(value, digits = 3) {
  if (!Number.isFinite(value)) return '—'.padStart(8);
  return value.toFixed(digits).padStart(8);
}

function fmtPct(value) {
  if (!Number.isFinite(value)) return '—'.padStart(8);
  const sign = value >= 0 ? '+' : '';
  return `${sign}${value.toFixed(1)}%`.padStart(8);
}

function printSide(side, block) {
  console.log(`\n=== ขา ${side} ===`);
  if (!block?.present) {
    console.log('  (ไม่มีข้อมูลทั้งสองฝั่ง)');
    return;
  }

  console.log(
    `  cycles: MoCap=${block.mocap.cycleCount}  IMU=${block.imu.cycleCount}`
    + (block.cycleCountDelta ? `  (Δ ${block.cycleCountDelta >= 0 ? '+' : ''}${block.cycleCountDelta})` : ''),
  );
  console.log('  metric              |    mocap |      imu |    error |  error%');
  console.log('  --------------------|----------|----------|----------|--------');
  for (const row of block.metrics) {
    const label = `${row.metric} (${row.unit})`.padEnd(20);
    if (!row.comparable) {
      console.log(`  ${label}| ${fmt(row.mocap)} | ${fmt(row.imu)} |        — |       —`);
      continue;
    }
    console.log(
      `  ${label}| ${fmt(row.mocap)} | ${fmt(row.imu)} | ${fmt(row.error)} | ${fmtPct(row.errorPct)}`,
    );
  }
}

function main() {
  const args = parseArgs(process.argv.slice(2));
  if (!args.mocap || !args.imu) {
    console.error('ใช้: node mocap-analysis/compare.js <mocap.gait-params.json> <imu-trace.json> [--out report.json]');
    process.exit(1);
  }

  const mocap = JSON.parse(readFileSync(args.mocap, 'utf8'));
  const imu = JSON.parse(readFileSync(args.imu, 'utf8'));

  console.log(`MoCap: ${args.mocap}`);
  console.log(`IMU:   ${args.imu}`);

  const report = compareMocapToImu(mocap, imu);

  if (!report.ok) {
    console.error(`\nเทียบไม่ได้: ${report.error}`);
    for (const w of report.warnings || []) console.warn(`  ⚠️ ${w}`);
    process.exit(1);
  }

  console.log(`\nแหล่ง IMU metrics: ${report.imuSource}`);
  if (report.warnings.length) {
    console.log('\nคำเตือน:');
    for (const w of report.warnings) console.log(`  ⚠️ ${w}`);
  }

  printSide('L', report.sides.L);
  printSide('R', report.sides.R);

  console.log('\n=== Session / ระยะ ===');
  console.log(`  MoCap pelvis net forward: ${fmt(report.session.mocapPelvisNetForwardM)} m`);
  console.log(`  MoCap bilateral cadence:  ${fmt(report.session.mocapTrueCadenceSpm, 1)} spm`);
  console.log(`  IMU groundTruth.distance: ${fmt(report.session.imuGroundTruthDistanceM)} m`);
  for (const side of ['L', 'R']) {
    const d = report.session.distanceBySide[side];
    if (!Number.isFinite(d.imuSumStrideLengthM)) continue;
    console.log(
      `  IMU ${side} sum(stride): ${fmt(d.imuSumStrideLengthM)} m`
      + `  vs MoCap net ${fmtPct(d.vsMocapPelvisNetPct)}`
      + `  vs GT ${fmtPct(d.vsGroundTruthPct)}`,
    );
  }

  console.log('\nหมายเหตุ:');
  for (const n of report.notes) console.log(`  • ${n}`);

  const outPath = args.out || args.imu.replace(/\.json$/i, '') + '.vs-mocap.json';
  writeFileSync(outPath, JSON.stringify(report, null, 2));
  console.log(`\nบันทึกรายงานเต็ม: ${outPath}`);
}

main();

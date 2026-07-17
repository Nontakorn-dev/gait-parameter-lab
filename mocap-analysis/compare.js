#!/usr/bin/env node
// เทียบ mocap.gait-params.json กับ IMU trace ที่ export จาก dashboard
//
// วิธีใช้:
//   node mocap-analysis/compare.js <mocap.gait-params.json> <imu-trace.json> [options]
//
// Options:
//   --out <report.json>
//   --lag <seconds>          บังคับ sync lag (IMU−MoCap) เช่นจาก heel-tap
//   --fine-lag <seconds>     หน้าต่างละเอียดรอบ coarse (default 0.4)
//   --rival-scan <seconds>   สแกนหา period-alias rivals (default 5)
//   --max-lag <seconds>      alias ของ --fine-lag (backward compatible)

import { readFileSync, writeFileSync } from 'node:fs';
import { compareMocapToImu } from './compareImuTrace.js';

function parseArgs(argv) {
  const args = {
    mocap: null,
    imu: null,
    out: null,
    lag: null,
    fineLag: null,
    rivalScan: null,
  };
  for (let i = 0; i < argv.length; i += 1) {
    const a = argv[i];
    if (a === '--out') { args.out = argv[++i]; continue; }
    if (a === '--lag') { args.lag = Number(argv[++i]); continue; }
    if (a === '--fine-lag' || a === '--max-lag') { args.fineLag = Number(argv[++i]); continue; }
    if (a === '--rival-scan') { args.rivalScan = Number(argv[++i]); continue; }
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

  const align = block.alignment;
  if (align) {
    console.log(
      `  align: ${align.mode}  lag=${fmt(align.lagS, 3)}s  source=${align.lagSource || '—'}`
      + (align.periodAliasRisk ? '  ⚠️ period-alias' : '')
      + (align.timingMetricValid ? '' : '  (HS timing n/a)'),
    );
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
    console.error(
      'ใช้: node mocap-analysis/compare.js <mocap.gait-params.json> <imu-trace.json>\n'
      + '     [--out report.json] [--lag SEC] [--fine-lag SEC] [--rival-scan SEC]',
    );
    process.exit(1);
  }

  const mocap = JSON.parse(readFileSync(args.mocap, 'utf8'));
  const imu = JSON.parse(readFileSync(args.imu, 'utf8'));

  console.log(`MoCap: ${args.mocap}`);
  console.log(`IMU:   ${args.imu}`);

  const align = {};
  if (Number.isFinite(args.lag)) align.lagS = args.lag;
  if (Number.isFinite(args.fineLag)) {
    align.fineMaxLagS = args.fineLag;
    align.maxLagS = args.fineLag;
  }
  if (Number.isFinite(args.rivalScan)) align.rivalScanMaxLagS = args.rivalScan;

  const report = compareMocapToImu(mocap, imu, { align });

  if (!report.ok) {
    console.error(`\nเทียบไม่ได้: ${report.error}`);
    for (const w of report.warnings || []) console.warn(`  ⚠️ ${w}`);
    process.exit(1);
  }

  for (const w of report.warnings || []) console.warn(`⚠️  ${w}`);
  printSide('L', report.sides.L);
  printSide('R', report.sides.R);

  if (args.out) {
    writeFileSync(args.out, `${JSON.stringify(report, null, 2)}\n`);
    console.log(`\nเขียนรายงาน: ${args.out}`);
  }
}

main();

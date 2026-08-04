#!/usr/bin/env node
// เทียบ mocap.gait-params.json กับ IMU trace ที่ export จาก dashboard
//
// วิธีใช้ (แลป — บังคับ):
//   node mocap-analysis/compare.js mocap.json imu.json --lab --lag 0.12 --out report.json
//
// Options:
//   --out <report.json>
//   --lag <seconds>          บังคับ sync lag (IMU−MoCap) จาก heel-tap — จำเป็นสำหรับ --lab
//   --fine-lag <seconds>     หน้าต่างละเอียดรอบ coarse (default 0.4)
//   --rival-scan <seconds>   สแกนหา period-alias rivals (default: max(90, ช่วงสัญญาณ) ≤120)
//   --max-lag <seconds>      alias ของ --fine-lag (backward compatible)
//   --lab                    fail (exit 1) ถ้า validationPublishable=false

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
    lab: false,
  };
  for (let i = 0; i < argv.length; i += 1) {
    const a = argv[i];
    if (a === '--out') { args.out = argv[++i]; continue; }
    if (a === '--lag') { args.lag = Number(argv[++i]); continue; }
    if (a === '--fine-lag' || a === '--max-lag') { args.fineLag = Number(argv[++i]); continue; }
    if (a === '--rival-scan') { args.rivalScan = Number(argv[++i]); continue; }
    if (a === '--lab') { args.lab = true; continue; }
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
      + (align.syncTrusted ? '  sync=trusted' : '  sync=UNTRUSTED')
      + (align.coarseFromOnset ? '  coarse=onset' : '')
      + (Number.isFinite(align.coarseLagS) && !align.coarseFromOnset ? `  coarse=${fmt(align.coarseLagS, 3)}s` : '')
      + (align.periodAliasRisk ? '  ⚠️ period-alias' : '')
      + (align.lagOk === false ? '  (pairing refused)' : '')
      + (align.timingMetricValid ? '' : '  (HS timing n/a)'),
    );
  }

  console.log(
    `  cycles: MoCap=${block.mocap.cycleCount}  IMU=${block.imu.cycleCount}`
    + (block.agreementPairCount != null ? `  agreementPairs=${block.agreementPairCount}` : '')
    + (block.cycleCountDelta ? `  (Δ ${block.cycleCountDelta >= 0 ? '+' : ''}${block.cycleCountDelta})` : '')
    + (block.validationPublishable ? '  ✅ publishable' : '  ⛔ not publishable'),
  );
  console.log('  metric              |    mocap |      imu |    error |  error%');
  console.log('  --------------------|----------|----------|----------|--------');
  for (const row of block.metrics) {
    const label = `${row.metric} (${row.unit})`.padEnd(20);
    if (!row.comparable) {
      const why = row.ineligibleReason ? ` [${row.ineligibleReason}]` : '';
      console.log(`  ${label}| ${fmt(row.mocap)} | ${fmt(row.imu)} |        — |       —${why}`);
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
      + '     [--lab] [--lag SEC] [--out report.json] [--fine-lag SEC] [--rival-scan SEC]\n'
      + 'แลป: ต้องมี heel-tap แล้วใส่ --lab --lag <วินาที>',
    );
    process.exit(1);
  }

  if (args.lab && !Number.isFinite(args.lag)) {
    console.error(
      'โหมด --lab ต้องการ --lag จาก heel-tap (เช่น --lag 0.15)\n'
      + 'อย่าพึ่ง onset อัตโนมัติอย่างเดียวตอนเสียเงินจองแลป',
    );
    process.exit(1);
  }

  const mocap = JSON.parse(readFileSync(args.mocap, 'utf8'));
  const imu = JSON.parse(readFileSync(args.imu, 'utf8'));

  console.log(`MoCap: ${args.mocap}`);
  console.log(`IMU:   ${args.imu}`);
  if (args.lab) console.log('Mode: LAB (fail if not validationPublishable)');

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

  console.log(`\nvalidationPublishable: ${report.validationPublishable ? 'YES ✅' : 'NO ⛔'}`
    + (report.validationPublishableBilateral === false && report.validationPublishable
      ? ' (บางข้างเท่านั้น — ดู sides[L|R])'
      : ''));
  if (report.labChecklist?.length) {
    console.log('labChecklist:');
    for (const item of report.labChecklist) {
      const mark = item.ok === true ? '✅' : item.ok === false ? '❌' : '⬜';
      console.log(`  ${mark} ${item.id}: ${item.detail}`);
    }
  }

  printSide('L', report.sides.L);
  printSide('R', report.sides.R);

  if (args.out) {
    writeFileSync(args.out, `${JSON.stringify(report, null, 2)}\n`);
    console.log(`\nเขียนรายงาน: ${args.out}`);
  }

  if (args.lab && !report.validationPublishable) {
    console.error('\n--lab: รายงานนี้ validationPublishable=false — ไม่ผ่านเกณฑ์แลป');
    process.exit(1);
  }
}

main();

#!/usr/bin/env node
/**
 * ประมาณ sync lag (IMU − MoCap) จาก HS times แล้วรัน compare
 * ใช้เมื่อไม่ได้ heel-tap / นาฬิกาไม่ตรง (เช่น MoCap clip สั้นกว่า IMU recording)
 *
 * วิธีใช้:
 *   node mocap-analysis/syncSession.js <mocap.gait-params.json> <imu-trace.json> \
 *     [--out-dir dir] [--rival-scan 90] [--match-tol 0.45]
 *
 * ไม่ใช้ groundTruth.distanceM / step จาก IMU JSON — อ้างอิง MoCap เป็นหลัก
 */

import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, join, basename } from 'node:path';
import {
  reprocessImuTrace,
  estimateHsTimeLagS,
  compareMocapToImu,
  pairCyclesByTime,
} from './compareImuTrace.js';

function parseArgs(argv) {
  const args = {
    mocap: null,
    imu: null,
    outDir: null,
    rivalScan: 90,
    matchTol: 0.45,
    fineLag: 2.5,
  };
  for (let i = 0; i < argv.length; i += 1) {
    const a = argv[i];
    if (a === '--out-dir') { args.outDir = argv[++i]; continue; }
    if (a === '--rival-scan') { args.rivalScan = Number(argv[++i]); continue; }
    if (a === '--match-tol') { args.matchTol = Number(argv[++i]); continue; }
    if (a === '--fine-lag') { args.fineLag = Number(argv[++i]); continue; }
    if (!args.mocap && !a.startsWith('--')) { args.mocap = a; continue; }
    if (!args.imu && !a.startsWith('--')) { args.imu = a; continue; }
  }
  return args;
}

function estimatePerSide(mocap, imuCyclesBySide, options) {
  const bySide = {};
  for (const side of ['L', 'R']) {
    const mocapCycles = mocap?.perSide?.[side]?.cycles || [];
    const imuCycles = imuCyclesBySide[side] || [];
    const hs = estimateHsTimeLagS(mocapCycles, imuCycles, {
      rivalScanMaxLagS: options.rivalScan,
      matchToleranceS: options.matchTol,
      fineMaxLagS: options.fineLag,
    });
    bySide[side] = {
      ...hs,
      mocapHsCount: mocapCycles.length,
      imuHsCount: imuCycles.length,
      mocapHsS: mocapCycles.map((c) => c.hsStartTimeS),
      imuHsS: imuCycles.map((c) => c.cycleStartTimeS),
    };
  }
  return bySide;
}

/** เลือก lag ร่วมสองข้าง: ถ่วงด้วย matchCount ถ้าต่างกันไม่เกิน 1.5 s */
function consensusLag(bySide) {
  const rows = ['L', 'R']
    .map((side) => ({ side, ...bySide[side] }))
    .filter((r) => r.ok && Number.isFinite(r.lagS) && r.matchCount >= 2);

  if (!rows.length) {
    return { lagS: null, ok: false, reason: 'no-side-with-hs-lag', sidesUsed: [] };
  }

  const spread = Math.max(...rows.map((r) => r.lagS)) - Math.min(...rows.map((r) => r.lagS));
  if (rows.length === 2 && spread <= 1.5) {
    const wSum = rows.reduce((a, r) => a + r.matchCount, 0);
    const lagS = rows.reduce((a, r) => a + r.lagS * r.matchCount, 0) / wSum;
    return {
      lagS,
      ok: true,
      reason: 'weighted-mean-L-R',
      sidesUsed: rows.map((r) => r.side),
      spreadS: spread,
    };
  }

  // ต่างกันมาก → ใช้ข้างที่ match มากสุด (กัน period alias ฝั่งหนึ่ง)
  rows.sort((a, b) => b.matchCount - a.matchCount || a.residualAbsSum - b.residualAbsSum);
  const best = rows[0];
  return {
    lagS: best.lagS,
    ok: true,
    reason: `best-side-${best.side}`,
    sidesUsed: [best.side],
    spreadS: spread,
  };
}

function pairedRows(mocap, imuBySide, lagS, matchTol) {
  const out = { L: [], R: [] };
  for (const side of ['L', 'R']) {
    const mocapCycles = mocap?.perSide?.[side]?.cycles || [];
    const imuCycles = imuBySide[side] || [];
    const paired = pairCyclesByTime(mocapCycles, imuCycles, {
      lagS,
      matchToleranceS: matchTol,
    });
    out[side] = (paired.pairs || []).map((p) => ({
      mocapHsS: p.mocap.hsStartTimeS,
      imuHsS: p.imu.cycleStartTimeS,
      alignedImuHsS: p.imu.cycleStartTimeS - lagS,
      dtS: (p.imu.cycleStartTimeS - lagS) - p.mocap.hsStartTimeS,
      mocapStrideM: p.mocap.strideLengthM,
      imuStrideM: p.imu.strideLengthM,
      mocapStrideTimeS: p.mocap.strideTimeS,
      imuStrideTimeS: p.imu.strideTimeS,
      imuUntrusted: Boolean(p.imu.strideLengthUntrusted || p.imu.strideLengthClamped),
    }));
  }
  return out;
}

function main() {
  const args = parseArgs(process.argv.slice(2));
  if (!args.mocap || !args.imu) {
    console.error(
      'ใช้: node mocap-analysis/syncSession.js <mocap.gait-params.json> <imu-trace.json> [--out-dir dir]',
    );
    process.exit(1);
  }

  const mocap = JSON.parse(readFileSync(args.mocap, 'utf8'));
  const imuTrace = JSON.parse(readFileSync(args.imu, 'utf8'));
  const outDir = args.outDir || dirname(args.mocap);
  mkdirSync(outDir, { recursive: true });

  const imu = reprocessImuTrace(imuTrace, { orientationFilter: 'kalman' });
  if (!imu.ok) {
    console.error('reprocess IMU ไม่ได้:', imu.reason);
    process.exit(1);
  }

  const perSide = estimatePerSide(mocap, imu.bySide, args);
  const consensus = consensusLag(perSide);
  if (!consensus.ok) {
    console.error('หา sync lag ไม่ได้:', consensus.reason);
    writeFileSync(join(outDir, 'sync-summary.json'), JSON.stringify({ ok: false, perSide, consensus }, null, 2));
    process.exit(1);
  }

  const lagS = consensus.lagS;
  console.log(`consensus lag (IMU−MoCap) = ${lagS.toFixed(3)}s  (${consensus.reason}, spread=${consensus.spreadS?.toFixed?.(3) ?? '—'}s)`);
  for (const side of ['L', 'R']) {
    const s = perSide[side];
    console.log(
      `  ${side}: lag=${s.ok ? s.lagS.toFixed(3) : '—'} match=${s.matchCount}/${Math.min(s.mocapHsCount, s.imuHsCount)} `
      + `mocapHS=[${(s.mocapHsS || []).map((t) => t.toFixed(2)).join(', ')}] `
      + `imuHS=[${(s.imuHsS || []).map((t) => t.toFixed(2)).join(', ')}]`,
    );
  }

  const report = compareMocapToImu(mocap, imuTrace, {
    align: {
      fineMaxLagS: args.fineLag,
      rivalScanMaxLagS: args.rivalScan,
    },
    orientationFilter: 'kalman',
    minAgreementPairs: 1,
    explorationOnly: true,
  });

  // ใช้ lag จาก signal envelope ถ้ามี — HS-circular เก็บเป็น reference เท่านั้น
  const signalLags = ['L', 'R']
    .map((s) => report.sides?.[s]?.alignment)
    .filter((a) => a && (a.lagSource === 'signal-xcorr' || a.lagSource === 'signal-xcorr-envelope')
      && Number.isFinite(a.lagS));
  const preferredLagS = signalLags.length
    ? signalLags.reduce((a, x) => a + x.lagS, 0) / signalLags.length
    : lagS;
  const lagSource = signalLags.length
    ? (signalLags[0].lagSource || 'signal-xcorr-envelope')
    : 'hs-event-circular';

  // จับคู่ด้วย lag ที่เชื่อกว่า (envelope) + ตาราง per-side จาก HS เมื่อ spread ใหญ่
  const pairs = pairedRows(mocap, imu.bySide, preferredLagS, args.matchTol);
  const pairsPerSideLag = {
    L: perSide.L.ok
      ? pairedRows(mocap, imu.bySide, perSide.L.lagS, args.matchTol).L
      : [],
    R: perSide.R.ok
      ? pairedRows(mocap, imu.bySide, perSide.R.lagS, args.matchTol).R
      : [],
  };

  const summary = {
    ok: true,
    note: 'Sync exploratory: prefer signal |ω| envelope when available; HS-event is fallback. Not heel-tap lab gate',
    mocapFile: args.mocap,
    imuFile: args.imu,
    lagS: preferredLagS,
    hsConsensusLagS: lagS,
    lagSource,
    lagTrusted: false,
    explorationOnly: true,
    consensus,
    perSide: {
      L: {
        lagS: perSide.L.lagS,
        matchCount: perSide.L.matchCount,
        ok: perSide.L.ok,
        signalLagS: report.sides?.L?.alignment?.lagS ?? null,
        signalLagSource: report.sides?.L?.alignment?.lagSource ?? null,
      },
      R: {
        lagS: perSide.R.lagS,
        matchCount: perSide.R.matchCount,
        ok: perSide.R.ok,
        signalLagS: report.sides?.R?.alignment?.lagS ?? null,
        signalLagSource: report.sides?.R?.alignment?.lagSource ?? null,
      },
    },
    pairedCycles: pairs,
    pairedCyclesPerSideLag: pairsPerSideLag,
    compareValidationPublishable: report.validationPublishable,
    compareExplorationOnly: report.explorationOnly,
    compareWarnings: report.warnings,
  };

  writeFileSync(join(outDir, 'sync-summary.json'), JSON.stringify(summary, null, 2));
  writeFileSync(join(outDir, 'compare-synced.json'), JSON.stringify(report, null, 2));

  // Markdown — เมื่อมี envelope lag ให้จับคู่ด้วยค่านั้น; per-side HS เฉพาะตอน fallback HS-circular + spread ใหญ่
  const usePerSideTable = lagSource === 'hs-event-circular'
    && Number.isFinite(consensus.spreadS)
    && consensus.spreadS > 0.8;
  const tablePairs = usePerSideTable ? pairsPerSideLag : pairs;
  const lines = [
    `# Sync: ${basename(dirname(args.mocap))}`,
    '',
    `- **lag (IMU − MoCap)** = **${preferredLagS.toFixed(3)} s** (\`${lagSource}\`)`,
    Number.isFinite(lagS) && Math.abs(preferredLagS - lagS) > 0.05
      ? `- HS-consensus lag (reference) = ${lagS.toFixed(3)} s — คลาดจาก envelope ได้ ~1 stride เมื่อ Heel≠malleolus`
      : '',
    `- MoCap: \`${basename(args.mocap)}\``,
    `- IMU: \`${basename(args.imu)}\``,
    `- ไม่ใช้ระยะ/จำนวนก้าวจาก IMU JSON — เทียบกับ MoCap โดยตรง`,
    usePerSideTable
      ? `- ตารางด้านล่างใช้ **lag ต่อข้าง** (L=${perSide.L.lagS?.toFixed?.(3)}s, R=${perSide.R.lagS?.toFixed?.(3)}s) เพราะ spread=${consensus.spreadS.toFixed(3)}s`
      : `- ตารางจับคู่ใช้ lag = ${preferredLagS.toFixed(3)}s`,
    '',
    '## Paired cycles',
  ].filter(Boolean);
  for (const side of ['L', 'R']) {
    const sideLag = usePerSideTable ? perSide[side].lagS : preferredLagS;
    lines.push(
      '',
      `### ${side}${Number.isFinite(sideLag) ? ` (lag=${sideLag.toFixed(3)}s)` : ''}`,
      '',
      '| MoCap HS (s) | IMU HS (s) | aligned IMU (s) | Δt (s) | MoCap stride (m) | IMU stride (m) |',
      '|---:|---:|---:|---:|---:|---:|',
    );
    for (const p of tablePairs[side]) {
      lines.push(
        `| ${p.mocapHsS.toFixed(2)} | ${p.imuHsS.toFixed(2)} | ${p.alignedImuHsS.toFixed(2)} | ${p.dtS.toFixed(3)} | `
        + `${Number.isFinite(p.mocapStrideM) ? p.mocapStrideM.toFixed(3) : '—'} | `
        + `${Number.isFinite(p.imuStrideM) ? p.imuStrideM.toFixed(3) : '—'} |`,
      );
    }
    if (!tablePairs[side].length) lines.push('| — | — | — | — | — | — |');
  }
  lines.push(
    '',
    '## Notes',
    `- explorationOnly = ${report.explorationOnly ? 'true' : 'false'}`,
    `- validationPublishable (lab gate) = ${report.validationPublishable ? 'true' : 'false'}`,
    '- Heel = AnkleForHS เท่านั้น — ไม่มี malleolus → signed ω sync ไม่ใช้; |ω| envelope เป็น exploratory clock sync',
    '- Σ stride clean หลังตัด clamp/open/overlap ยังน้อยกว่า groundTruth 5 m มาก — อย่าตีความ % ระยะเป็นความแม่น',
    '',
  );
  writeFileSync(join(outDir, 'SYNC.md'), lines.join('\n'));

  console.log(`\nเขียน: ${join(outDir, 'sync-summary.json')}`);
  console.log(`เขียน: ${join(outDir, 'compare-synced.json')}`);
  console.log(`เขียน: ${join(outDir, 'SYNC.md')}`);
  console.log(`explorationOnly=${report.explorationOnly} validationPublishable=${report.validationPublishable}`);
}

main();

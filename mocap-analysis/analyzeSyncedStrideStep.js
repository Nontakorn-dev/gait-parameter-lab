#!/usr/bin/env node
/**
 * วิเคราะห์ stride / step หลัง sync แล้ว
 *   node mocap-analysis/analyzeSyncedStrideStep.js Test_3Aug
 */
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { join } from 'node:path';
import { parseOptiTrackCsv, extractMarkerSeries, interpolateGaps } from './parseOptiTrack.js';
import { computeGaitFromMocap } from './gaitFromMocap.js';
import { reprocessImuTrace } from './compareImuTrace.js';

function stats(values) {
  const a = values.filter(Number.isFinite);
  if (!a.length) return null;
  const mean = a.reduce((s, v) => s + v, 0) / a.length;
  return { n: a.length, mean };
}

function mocapStepsFromCsv(csvPath, mapPath) {
  const map = JSON.parse(readFileSync(mapPath, 'utf8'));
  const roles = Object.fromEntries(Object.entries(map).filter(([k]) => !k.startsWith('_')));
  const parsed = parseOptiTrackCsv(readFileSync(csvPath, 'utf8'));
  const result = computeGaitFromMocap(parsed, roles);
  const { fx, fz } = result.forwardAxis;
  const t0 = parsed.frames[0].time;
  const t = parsed.frames.map((f) => f.time - t0);

  function fwd(markerId) {
    const s = extractMarkerSeries(parsed, markerId);
    const filled = interpolateGaps(s, 10);
    return filled.series.x.map((x, i) => {
      const z = filled.series.z[i];
      if (x == null || z == null) return null;
      return x * fx + z * fz;
    });
  }
  const fwdBySide = {
    L: fwd(roles.L_AnkleForHS ?? roles.L_Ankle),
    R: fwd(roles.R_AnkleForHS ?? roles.R_Ankle),
  };

  const events = [];
  for (const side of ['L', 'R']) {
    for (const c of result.perSide[side].cycles) {
      let bestI = 0;
      let bestD = Infinity;
      for (let i = 0; i < t.length; i += 1) {
        const d = Math.abs(t[i] - c.hsStartTimeS);
        if (d < bestD) {
          bestD = d;
          bestI = i;
        }
      }
      events.push({
        side,
        t: c.hsStartTimeS,
        strideM: c.strideLengthM,
        forwardM: fwdBySide[side][bestI],
      });
    }
  }
  events.sort((a, b) => a.t - b.t);

  const steps = [];
  for (let i = 1; i < events.length; i += 1) {
    const prev = events[i - 1];
    const cur = events[i];
    if (prev.side === cur.side) continue;
    if (!Number.isFinite(prev.forwardM) || !Number.isFinite(cur.forwardM)) continue;
    const stepM = Math.abs(cur.forwardM - prev.forwardM);
    const stepTimeS = cur.t - prev.t;
    steps.push({
      from: prev.side,
      to: cur.side,
      tStartS: prev.t,
      tEndS: cur.t,
      stepTimeS,
      stepLengthM: stepM,
      turnaroundLike: stepM < 0.15 || stepTimeS > 2.5,
    });
  }
  return {
    steps,
    events,
    bilateralReliable: result.bilateral?.reliable !== false,
    sameSideRepeats: result.bilateral?.sameSideRepeats ?? 0,
    pelvisMaxExcursionM: result.session?.pelvisMaxExcursionM ?? null,
    pelvisNetMeaningful: result.session?.pelvisNetMeaningful === true,
  };
}

function analyzeTest(dir, name, files) {
  const sync = JSON.parse(readFileSync(join(dir, name, 'sync-summary.json'), 'utf8'));
  const imu = reprocessImuTrace(JSON.parse(readFileSync(join(dir, name, files.imu), 'utf8')), {
    orientationFilter: 'kalman',
  });
  const mocap = mocapStepsFromCsv(join(dir, name, files.csv), join(dir, name, 'marker-map.json'));

  const pairsSrc = (sync.pairedCyclesPerSideLag && sync.consensus?.spreadS > 0.8)
    ? sync.pairedCyclesPerSideLag
    : (sync.pairedCyclesPerSideLag || sync.pairedCycles);

  const stridePairs = [];
  for (const side of ['L', 'R']) {
    for (const p of pairsSrc[side] || []) {
      if (!Number.isFinite(p.mocapStrideM)) continue;
      const usable = p.mocapStrideM >= 0.30 && Number.isFinite(p.imuStrideM) && p.imuStrideM >= 0.20;
      stridePairs.push({
        side,
        mocapHsS: p.mocapHsS,
        imuHsS: p.imuHsS,
        mocapStrideM: p.mocapStrideM,
        imuStrideM: p.imuStrideM ?? null,
        errM: Number.isFinite(p.imuStrideM) ? p.imuStrideM - p.mocapStrideM : null,
        errPct: Number.isFinite(p.imuStrideM) && p.mocapStrideM > 0
          ? ((p.imuStrideM - p.mocapStrideM) / p.mocapStrideM) * 100
          : null,
        usable,
      });
    }
  }

  const usable = stridePairs.filter((p) => p.usable && Number.isFinite(p.errM));
  const errs = usable.map((p) => p.errM);
  const absErrs = usable.map((p) => Math.abs(p.errM));

  const lagBySide = {
    L: sync.perSide?.L?.lagS ?? sync.lagS,
    R: sync.perSide?.R?.lagS ?? sync.lagS,
  };

  // IMU step TIME จาก HS สลับข้าง (หลัง align เข้า MoCap time)
  const imuEvents = [];
  for (const side of ['L', 'R']) {
    for (const c of imu.bySide[side] || []) {
      if (!Number.isFinite(c.cycleStartTimeS)) continue;
      imuEvents.push({
        side,
        tMocap: c.cycleStartTimeS - lagBySide[side],
        strideM: c.strideLengthM,
      });
    }
  }
  imuEvents.sort((a, b) => a.tMocap - b.tMocap);
  const imuStepTimes = [];
  for (let i = 1; i < imuEvents.length; i += 1) {
    const prev = imuEvents[i - 1];
    const cur = imuEvents[i];
    if (prev.side === cur.side) continue;
    imuStepTimes.push({
      from: prev.side,
      to: cur.side,
      tEndS: cur.tMocap,
      stepTimeS: cur.tMocap - prev.tMocap,
    });
  }

  const stepTimePairs = [];
  for (const ms of mocap.steps) {
    if (ms.turnaroundLike) continue;
    let best = null;
    for (const is of imuStepTimes) {
      if (is.from !== ms.from || is.to !== ms.to) continue;
      const dt = Math.abs(is.tEndS - ms.tEndS);
      if (dt <= 0.5 && (!best || dt < best.dt)) best = { is, dt };
    }
    if (!best) continue;
    stepTimePairs.push({
      from: ms.from,
      to: ms.to,
      mocapStepLengthM: ms.stepLengthM,
      mocapStepTimeS: ms.stepTimeS,
      imuStepTimeS: best.is.stepTimeS,
      imuStepLengthM: null,
      stepTimeErrS: best.is.stepTimeS - ms.stepTimeS,
    });
  }

  const mocapStepsUsable = mocap.bilateralReliable
    ? mocap.steps.filter((s) => !s.turnaroundLike && s.stepLengthM >= 0.15)
    : [];

  return {
    lagS: sync.lagS,
    lagBySide,
    stridePairs,
    strideAgreement: {
      nUsable: usable.length,
      nAll: stridePairs.length,
      meanErrorM: stats(errs)?.mean ?? null,
      maeM: stats(absErrs)?.mean ?? null,
      rmseM: errs.length ? Math.sqrt(errs.reduce((s, v) => s + v * v, 0) / errs.length) : null,
      meanErrorPct: stats(usable.map((p) => p.errPct))?.mean ?? null,
      mocapMeanStrideM: stats(usable.map((p) => p.mocapStrideM))?.mean ?? null,
      imuMeanStrideM: stats(usable.map((p) => p.imuStrideM))?.mean ?? null,
    },
    mocapSteps: mocap.bilateralReliable ? mocap.steps : [],
    mocapStepsHiddenReason: mocap.bilateralReliable
      ? null
      : `bilateral.reliable=false (sameSideRepeats=${mocap.sameSideRepeats}) — ซ่อนตาราง step`,
    bilateralReliable: mocap.bilateralReliable,
    sameSideRepeats: mocap.sameSideRepeats,
    pelvisMaxExcursionM: mocap.pelvisMaxExcursionM,
    pelvisNetMeaningful: mocap.pelvisNetMeaningful,
    mocapStepSummary: {
      n: mocapStepsUsable.length,
      meanStepLengthM: stats(mocapStepsUsable.map((s) => s.stepLengthM))?.mean ?? null,
      meanStepTimeS: stats(mocapStepsUsable.map((s) => s.stepTimeS))?.mean ?? null,
    },
    stepTimePairs: mocap.bilateralReliable ? stepTimePairs : [],
  };
}

const root = process.argv[2] || 'Test_3Aug';
const catalog = {
  Test1: { csv: 'TEST_DERNDEE.csv', imu: 'gait-trace-20260803-162019.json' },
  Test2: { csv: 'DernDee3.csv', imu: 'gait-trace-20260803-163213.json' },
};

const report = {
  generatedAt: new Date().toISOString(),
  note: [
    'Stride: เทียบ MoCap vs IMU จากคู่ HS หลัง sync',
    'Step length: มีจาก MoCap (ระยะ marker ส้นเท้า HS→HS ข้างตรงข้าม)',
    'IMU step length = null — ห้าม stride/2 จากเซนเซอร์ข้างเดียว',
    'Step time: เทียบได้เมื่อจับคู่ HS สลับข้างหลัง sync',
  ].join('. '),
  tests: {},
};

for (const [name, files] of Object.entries(catalog)) {
  if (!existsSync(join(root, name, 'sync-summary.json'))) continue;
  report.tests[name] = analyzeTest(root, name, files);
}

writeFileSync(join(root, 'stride-step-analysis.json'), JSON.stringify(report, null, 2));

// Markdown
const lines = [
  '# Stride / Step analysis (หลัง sync)',
  '',
  report.note,
  '',
];
for (const [name, t] of Object.entries(report.tests)) {
  const a = t.strideAgreement;
  lines.push(
    `## ${name}`,
    '',
    `- Sync lag ≈ **${t.lagS?.toFixed?.(3)} s**`,
    `- Stride คู่ที่ใช้ได้: **${a.nUsable}/${a.nAll}** (ตัดรอบกลับตัว MoCap < 0.30 m)`,
    a.nUsable
      ? `- Stride mean: MoCap **${a.mocapMeanStrideM?.toFixed(3)} m** · IMU **${a.imuMeanStrideM?.toFixed(3)} m** · MAE **${a.maeM?.toFixed(3)} m** · bias **${a.meanErrorPct?.toFixed(1)}%**`
      : '- ไม่มีคู่ stride ที่ใช้ได้พอ',
    t.mocapStepSummary.n
      ? `- MoCap step length เฉลี่ย (ไม่นับกลับตัว): **${t.mocapStepSummary.meanStepLengthM?.toFixed(3)} m** · step time **${t.mocapStepSummary.meanStepTimeS?.toFixed(3)} s**`
      : (t.mocapStepsHiddenReason ? `- ⛔ ${t.mocapStepsHiddenReason}` : ''),
    Number.isFinite(t.pelvisMaxExcursionM)
      ? `- Pelvis max excursion ≈ **${t.pelvisMaxExcursionM.toFixed(2)} m**`
        + (t.pelvisNetMeaningful ? '' : ' (net ไม่ใช้ — เดินไป-กลับ)')
      : '',
    '',
    '### Stride pairs',
    '',
    '| Side | MoCap HS | IMU HS | MoCap stride (m) | IMU stride (m) | Error (m) | Error % | Use? |',
    '|:---:|---:|---:|---:|---:|---:|---:|:---:|',
  );
  for (const p of t.stridePairs) {
    lines.push(
      `| ${p.side} | ${p.mocapHsS.toFixed(2)} | ${p.imuHsS.toFixed(2)} | ${p.mocapStrideM.toFixed(3)} | `
      + `${p.imuStrideM != null ? p.imuStrideM.toFixed(3) : '—'} | `
      + `${p.errM != null ? p.errM.toFixed(3) : '—'} | `
      + `${p.errPct != null ? p.errPct.toFixed(0) : '—'} | ${p.usable ? 'yes' : 'no'} |`,
    );
  }
  lines.push('', '### MoCap step length (bilateral)', '');
  if (t.mocapStepsHiddenReason) {
    lines.push(`_${t.mocapStepsHiddenReason}_`, '');
  } else {
    lines.push('| From→To | t (s) | Step length (m) | Step time (s) |', '|:---:|---:|---:|---:|');
    for (const s of t.mocapSteps.filter((x) => !x.turnaroundLike)) {
      lines.push(`| ${s.from}→${s.to} | ${s.tEndS.toFixed(2)} | ${s.stepLengthM.toFixed(3)} | ${s.stepTimeS.toFixed(3)} |`);
    }
  }  if (t.stepTimePairs.length) {
    lines.push('', '### Step time: MoCap vs IMU', '', '| From→To | MoCap step (m) | MoCap step time (s) | IMU step time (s) | Δt (s) |', '|:---:|---:|---:|---:|---:|');
    for (const p of t.stepTimePairs) {
      lines.push(
        `| ${p.from}→${p.to} | ${p.mocapStepLengthM.toFixed(3)} | ${p.mocapStepTimeS.toFixed(3)} | `
        + `${p.imuStepTimeS.toFixed(3)} | ${p.stepTimeErrS.toFixed(3)} |`,
      );
    }
  }
  lines.push('');
}
writeFileSync(join(root, 'STRIDE_STEP_ANALYSIS.md'), lines.join('\n'));
console.log(`wrote ${join(root, 'stride-step-analysis.json')}`);
console.log(`wrote ${join(root, 'STRIDE_STEP_ANALYSIS.md')}`);

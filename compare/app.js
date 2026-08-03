/**
 * ω Compare — อัปโหลด MoCap CSV + IMU JSON แล้ว sync/plot ในเบราว์เซอร์
 */
import Chart from 'chart.js/auto';
import { parseOptiTrackCsv, listMarkers } from '../mocap-analysis/parseOptiTrack.js';
import { resolveMarkerRoles } from '../mocap-analysis/markerRoles.js';
import { computeGaitFromMocap } from '../mocap-analysis/gaitFromMocap.js';
import {
  compareMocapToImu,
  estimateSignalLagS,
  extractImuGyroSeries,
  resampleUniform,
  summarizeMocapSide,
  summarizeImuSide,
  reprocessImuTrace,
} from '../mocap-analysis/compareImuTrace.js';

const ROLE_OPTIONS = [
  { value: '', label: '— ไม่ใช้ —' },
  { value: 'L_ASIS', label: 'สะโพกซ้าย (Left ASIS)' },
  { value: 'R_ASIS', label: 'สะโพกขวา (Right ASIS)' },
  { value: 'L_Knee', label: 'เข่าซ้าย (Left Lateral Femoral Epicondyle)' },
  { value: 'R_Knee', label: 'เข่าขวา (Right Lateral Femoral Epicondyle)' },
  { value: 'L_AnkleForHS', label: 'ส้นเท้าซ้าย (Left Heel / Calcaneus)' },
  { value: 'R_AnkleForHS', label: 'ส้นเท้าขวา (Right Heel / Calcaneus)' },
  { value: 'L_Toe', label: 'ปลายเท้าซ้าย (Left Toe / 2nd Metatarsal)' },
  { value: 'R_Toe', label: 'ปลายเท้าขวา (Right Toe / 2nd Metatarsal)' },
  { value: 'L_AnkleForAngle', label: 'ข้อเท้าซ้าย — malleolus (สำหรับมุม)' },
  { value: 'R_AnkleForAngle', label: 'ข้อเท้าขวา — malleolus (สำหรับมุม)' },
  { value: 'L_Ankle', label: 'ข้อเท้าซ้าย (legacy Ankle)' },
  { value: 'R_Ankle', label: 'ข้อเท้าขวา (legacy Ankle)' },
];

const PRESETS = {
  test1: {
    1: 'L_ASIS', 2: 'L_Knee', 3: 'R_Toe', 4: 'R_AnkleForHS',
    5: 'L_Toe', 6: 'L_AnkleForHS', 7: 'R_ASIS', 8: 'R_Knee',
  },
  test2: {
    1: 'L_ASIS', 2: 'L_Knee', 3: 'R_Toe', 4: 'L_AnkleForHS',
    5: 'R_AnkleForHS', 6: 'L_Toe', 7: 'R_ASIS', 8: 'R_Knee',
  },
  clear: {
    1: '', 2: '', 3: '', 4: '', 5: '', 6: '', 7: '', 8: '',
  },
};

const REQUIRED_ROLES = ['L_ASIS', 'R_ASIS', 'L_Knee', 'R_Knee'];

const els = {
  csvDrop: document.getElementById('csvDrop'),
  csvInput: document.getElementById('csvInput'),
  csvName: document.getElementById('csvName'),
  jsonDrop: document.getElementById('jsonDrop'),
  jsonInput: document.getElementById('jsonInput'),
  jsonName: document.getElementById('jsonName'),
  mapGrid: document.getElementById('mapGrid'),
  mapStatus: document.getElementById('mapStatus'),
  sideSelect: document.getElementById('sideSelect'),
  axisSelect: document.getElementById('axisSelect'),
  envelopeCheck: document.getElementById('envelopeCheck'),
  runBtn: document.getElementById('runBtn'),
  runHint: document.getElementById('runHint'),
  chartPanel: document.getElementById('chartPanel'),
  syncMeta: document.getElementById('syncMeta'),
  reportPanel: document.getElementById('reportPanel'),
  paramMeta: document.getElementById('paramMeta'),
  statRow: document.getElementById('statRow'),
  warnList: document.getElementById('warnList'),
  paramTable: document.getElementById('paramTable'),
  strideTable: document.getElementById('strideTable'),
  mocapCycleTable: document.getElementById('mocapCycleTable'),
  imuCycleTable: document.getElementById('imuCycleTable'),
};

const state = {
  csvText: null,
  csvName: null,
  imuTrace: null,
  imuName: null,
  parsed: null,
  markers: [],
  roleByMarkerId: { ...PRESETS.test1 },
  mocapResult: null,
  chart: null,
};

function fmt(n, d = 2) {
  return Number.isFinite(n) ? n.toFixed(d) : '—';
}

function pct(err, ref) {
  if (!Number.isFinite(err) || !Number.isFinite(ref) || ref === 0) return null;
  return (err / ref) * 100;
}

function bindDrop(dropEl, inputEl, onFile) {
  const open = () => inputEl.click();
  dropEl.addEventListener('click', open);
  dropEl.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' || e.key === ' ') {
      e.preventDefault();
      open();
    }
  });
  inputEl.addEventListener('change', () => {
    const file = inputEl.files?.[0];
    if (file) onFile(file);
  });
  dropEl.addEventListener('dragover', (e) => {
    e.preventDefault();
    dropEl.classList.add('is-drag');
  });
  dropEl.addEventListener('dragleave', () => dropEl.classList.remove('is-drag'));
  dropEl.addEventListener('drop', (e) => {
    e.preventDefault();
    dropEl.classList.remove('is-drag');
    const file = e.dataTransfer?.files?.[0];
    if (file) onFile(file);
  });
}

function buildMapFromRoles() {
  const map = {};
  for (const [id, role] of Object.entries(state.roleByMarkerId)) {
    if (role) map[role] = Number(id);
  }
  return map;
}

function mapComplete() {
  const map = buildMapFromRoles();
  for (const role of REQUIRED_ROLES) {
    if (!Number.isFinite(map[role])) return false;
  }
  const hasLAnkle = Number.isFinite(map.L_AnkleForHS) || Number.isFinite(map.L_Ankle);
  const hasRAnkle = Number.isFinite(map.R_AnkleForHS) || Number.isFinite(map.R_Ankle);
  return hasLAnkle && hasRAnkle;
}

function renderMapGrid() {
  const ids = state.markers.length
    ? state.markers.map((m) => m.id).sort((a, b) => a - b)
    : [1, 2, 3, 4, 5, 6, 7, 8];

  els.mapGrid.innerHTML = ids.map((id) => {
    const marker = state.markers.find((m) => m.id === id);
    const label = marker?.name ? `#${id} ${marker.name}` : `#${id}`;
    const current = state.roleByMarkerId[id] ?? '';
    const opts = ROLE_OPTIONS.map((o) => (
      `<option value="${o.value}" ${o.value === current ? 'selected' : ''}>${o.label}</option>`
    )).join('');
    return `
      <label class="map-row">
        <span class="map-row__id" title="${label}">${label}</span>
        <select data-marker-id="${id}">${opts}</select>
      </label>`;
  }).join('');

  els.mapGrid.querySelectorAll('select').forEach((sel) => {
    sel.addEventListener('change', () => {
      const id = Number(sel.dataset.markerId);
      const role = sel.value;
      // ห้าม role ซ้ำ — ล้างตัวอื่นที่ใช้ role เดียวกัน
      if (role) {
        for (const [otherId, otherRole] of Object.entries(state.roleByMarkerId)) {
          if (Number(otherId) !== id && otherRole === role) {
            state.roleByMarkerId[otherId] = '';
          }
        }
      }
      state.roleByMarkerId[id] = role;
      renderMapGrid();
      updateReady();
    });
  });
  updateReady();
}

function updateReady() {
  const ready = Boolean(state.csvText && state.imuTrace && mapComplete());
  els.runBtn.disabled = !ready;
  if (!state.csvText || !state.imuTrace) {
    els.runHint.textContent = 'อัปโหลดครบ + แมปครบก่อน';
    els.mapStatus.className = 'map-status';
    els.mapStatus.textContent = state.csvText
      ? `เจอ marker ${state.markers.length} จุดใน CSV — เลือกบทบาทให้ครบ`
      : 'อัปโหลด CSV เพื่อเห็น marker จริงในไฟล์';
    return;
  }
  if (!mapComplete()) {
    els.runHint.textContent = 'แมปยังไม่ครบ (ต้องมี ASIS/Knee/Ankle ซ้าย-ขวา)';
    els.mapStatus.className = 'map-status bad';
    els.mapStatus.textContent = 'ยังขาดบทบาทหลัก — ใช้ preset Test1/Test2 ได้';
    return;
  }
  els.runHint.textContent = 'พร้อม Sync & Plot';
  els.mapStatus.className = 'map-status ok';
  els.mapStatus.textContent = 'แมปครบแล้ว — กด Sync & Plot หรือเปลี่ยนขา/แกนแล้วรันใหม่';
}

async function onCsvFile(file) {
  const text = await file.text();
  state.csvText = text;
  state.csvName = file.name;
  els.csvName.textContent = file.name;
  els.csvDrop.classList.add('is-ready');
  try {
    state.parsed = parseOptiTrackCsv(text);
    state.markers = listMarkers(state.parsed);
  } catch (err) {
    state.parsed = null;
    state.markers = [];
    els.mapStatus.className = 'map-status bad';
    els.mapStatus.textContent = `parse CSV ไม่ได้: ${err.message}`;
    updateReady();
    return;
  }
  renderMapGrid();
}

async function onJsonFile(file) {
  const text = await file.text();
  let data;
  try {
    data = JSON.parse(text);
  } catch (err) {
    els.mapStatus.className = 'map-status bad';
    els.mapStatus.textContent = `JSON ไม่ถูกต้อง: ${err.message}`;
    return;
  }
  if (!Array.isArray(data.samples) && !Array.isArray(data.cycles)) {
    els.mapStatus.className = 'map-status bad';
    els.mapStatus.textContent = 'ไฟล์นี้ไม่ใช่ IMU trace (ต้องมี samples[] หรือ cycles[])';
    return;
  }
  state.imuTrace = data;
  state.imuName = file.name;
  els.jsonName.textContent = file.name;
  els.jsonDrop.classList.add('is-ready');
  updateReady();
}

function downsampleXY(t, y, maxPoints = 2500) {
  if (t.length <= maxPoints) {
    return t.map((x, i) => ({ x, y: y[i] }));
  }
  const step = Math.ceil(t.length / maxPoints);
  const out = [];
  for (let i = 0; i < t.length; i += step) {
    out.push({ x: t[i], y: y[i] });
  }
  return out;
}

function buildAlignedSeries(mocapT, mocapY, imuT, imuY, lagS, useEnvelope, { normalize = true } = {}) {
  const mY = useEnvelope ? mocapY.map((v) => (Number.isFinite(v) ? Math.abs(v) : v)) : mocapY;
  const iY = useEnvelope ? imuY.map((v) => (Number.isFinite(v) ? Math.abs(v) : v)) : imuY;

  const mocapT0 = Math.min(...mocapT.filter(Number.isFinite));
  const mocapT1 = Math.max(...mocapT.filter(Number.isFinite));
  const imuT0 = Math.min(...imuT.filter(Number.isFinite));
  const imuT1 = Math.max(...imuT.filter(Number.isFinite));

  const o0 = Math.max(mocapT0, imuT0 - lagS);
  const o1 = Math.min(mocapT1, imuT1 - lagS);
  const dt = 0.02;
  if (!(o1 > o0)) return null;

  const mocapRes = resampleUniform(mocapT, mY, dt, o0, o1);
  const imuRes = resampleUniform(imuT, iY, dt, o0 + lagS, o1 + lagS);
  const n = Math.min(mocapRes.length, imuRes.length);
  const t = [];
  const mocapOut = [];
  const imuOut = [];
  for (let i = 0; i < n; i += 1) {
    if (!Number.isFinite(mocapRes[i]) || !Number.isFinite(imuRes[i])) continue;
    t.push(o0 + i * dt);
    mocapOut.push(mocapRes[i]);
    imuOut.push(imuRes[i]);
  }
  if (t.length < 10) return null;

  let plotM = mocapOut;
  let plotI = imuOut;
  let yLabel = useEnvelope ? '|ω| (deg/s)' : 'ω (deg/s)';
  let peakMocap = Math.max(...mocapOut.map(Math.abs));
  let peakImu = Math.max(...imuOut.map(Math.abs));
  if (normalize && peakMocap > 0 && peakImu > 0) {
    // เทียบรูปร่างอย่างเดียว — heel ω กับ IMU gx สเกลคนละระดับ (~10×)
    plotM = mocapOut.map((v) => v / peakMocap);
    plotI = imuOut.map((v) => v / peakImu);
    yLabel = useEnvelope ? '|ω| (normalized)' : 'ω (normalized)';
  }

  return {
    mocap: downsampleXY(t, plotM),
    imu: downsampleXY(t, plotI),
    yLabel,
    peakMocap,
    peakImu,
    normalized: Boolean(normalize && peakMocap > 0 && peakImu > 0),
  };
}

function renderChart(series, side, axis, useEnvelope) {
  const ctx = document.getElementById('omegaChart');
  if (state.chart) {
    state.chart.destroy();
    state.chart = null;
  }
  state.chart = new Chart(ctx, {
    type: 'line',
    data: {
      datasets: [
        {
          label: `MoCap ω${useEnvelope ? ' |·|' : ''} (${side})${series.normalized ? ' · norm' : ''}`,
          data: series.mocap,
          borderColor: '#1d4e89',
          backgroundColor: 'transparent',
          borderWidth: 1.5,
          pointRadius: 0,
          tension: 0.05,
        },
        {
          label: `IMU ${axis}${useEnvelope ? ' |·|' : ''} (${side})${series.normalized ? ' · norm' : ''}`,
          data: series.imu,
          borderColor: '#c45c26',
          backgroundColor: 'transparent',
          borderWidth: 1.5,
          pointRadius: 0,
          tension: 0.05,
        },
      ],
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      parsing: false,
      interaction: { mode: 'nearest', intersect: false, axis: 'x' },
      plugins: {
        legend: { position: 'top', labels: { boxWidth: 12 } },
        tooltip: {
          callbacks: {
            title: (items) => `t = ${fmt(items[0]?.parsed?.x, 2)} s (MoCap)`,
          },
        },
      },
      scales: {
        x: {
          type: 'linear',
          title: { display: true, text: 'เวลา MoCap (s)' },
          ticks: { maxTicksLimit: 10 },
        },
        y: {
          title: { display: true, text: series.yLabel || (useEnvelope ? '|ω| (deg/s)' : 'ω (deg/s)') },
        },
      },
    },
  });
}

function paramDelta(mocap, imu) {
  if (!Number.isFinite(mocap) || !Number.isFinite(imu)) {
    return { d: null, pct: null };
  }
  const d = imu - mocap;
  return { d, pct: mocap !== 0 ? (d / mocap) * 100 : null };
}

function renderReport(report, side, lagInfo, extra = {}) {
  const sideReport = report.sides?.[side];
  const align = sideReport?.alignment;
  const dist = report.session?.distanceBySide?.[side];
  const mocapSum = sideReport?.mocapAll
    || summarizeMocapSide(extra.mocap?.perSide?.[side]);
  const imuSum = sideReport?.imuAll
    || summarizeImuSide([]);
  const pairs = align?.pairs || [];

  if (els.paramMeta) {
    els.paramMeta.textContent = [
      `ขา ${side}`,
      `MoCap ${mocapSum.cycleCount ?? '—'} cycle`,
      `IMU ${imuSum.cycleCount ?? '—'} cycle`,
      `จับคู่ HS ${pairs.length} คู่`,
      lagInfo.peakCorr != null ? `ω corr ${fmt(lagInfo.peakCorr, 3)}` : null,
    ].filter(Boolean).join(' · ');
  }

  // สรุปสั้น ๆ ด้านบน — โฟกัสตัวเลข ไม่ใช่ gate
  const stats = [
    { label: 'stride MoCap', value: `${fmt(mocapSum.meanStrideLengthM, 3)} m` },
    { label: 'stride IMU', value: `${fmt(imuSum.meanStrideLengthM, 3)} m` },
    { label: 'cadence MoCap', value: `${fmt(mocapSum.meanCadenceSpm, 1)}` },
    { label: 'cadence IMU', value: `${fmt(imuSum.meanCadenceSpm, 1)}` },
    { label: 'speed MoCap', value: `${fmt(mocapSum.meanWalkingSpeedMps, 3)}` },
    { label: 'speed IMU', value: `${fmt(imuSum.meanWalkingSpeedMps, 3)}` },
    { label: 'Σ IMU clean', value: dist?.imuSumStrideLengthCleanM != null ? `${fmt(dist.imuSumStrideLengthCleanM, 3)} m` : '—' },
  ];
  els.statRow.innerHTML = stats.map((s) => `
    <div class="stat">
      <div class="stat__label">${s.label}</div>
      <div class="stat__value">${s.value}</div>
    </div>`).join('');

  // ตารางพารามิเตอร์ทั้งหมด
  const rows = [
    { name: 'จำนวน cycle', m: mocapSum.cycleCount, i: imuSum.cycleCount, digits: 0 },
    { name: 'mean stride length (m)', m: mocapSum.meanStrideLengthM, i: imuSum.meanStrideLengthM, digits: 3 },
    { name: 'mean cadence (spm)', m: mocapSum.meanCadenceSpm, i: imuSum.meanCadenceSpm, digits: 1 },
    { name: 'mean walking speed (m/s)', m: mocapSum.meanWalkingSpeedMps, i: imuSum.meanWalkingSpeedMps, digits: 3 },
    { name: 'mean stance %', m: mocapSum.meanStancePct, i: imuSum.meanStancePct, digits: 1 },
    { name: 'mean peak shank angle (°)', m: mocapSum.meanPeakShankAngleDeg, i: imuSum.meanPeakShankAngleDeg, digits: 1 },
    { name: 'mean clearance (m)', m: mocapSum.meanAnkleClearanceM, i: imuSum.meanClearanceM, digits: 3 },
    { name: 'open strides (IMU)', m: null, i: imuSum.openStrideCount, digits: 0 },
    { name: 'clamped strides (IMU)', m: null, i: imuSum.clampedCount, digits: 0 },
  ];

  const paramBody = els.paramTable?.querySelector('tbody');
  if (paramBody) {
    paramBody.innerHTML = rows.map((r) => {
      const { d, pct: p } = paramDelta(r.m, r.i);
      return `<tr>
        <td>${r.name}</td>
        <td class="mono">${fmt(r.m, r.digits)}</td>
        <td class="mono">${fmt(r.i, r.digits)}</td>
        <td class="mono">${fmt(d, r.digits)}</td>
        <td class="mono">${Number.isFinite(p) ? `${fmt(p, 1)}%` : '—'}</td>
      </tr>`;
    }).join('');
  }

  // รายก้าว — โชว์ค่าที่คำนวณได้ ไม่ซ่อนเพราะ agreement
  const tbody = els.strideTable.querySelector('tbody');
  if (!pairs.length) {
    tbody.innerHTML = `<tr><td colspan="10">จับคู่ HS ไม่ได้ภายใน ±0.4s — ดูค่าเฉลี่ยด้านบนได้ตามปกติ (กราฟ ω ยังใช้เช็ครูปร่างได้)</td></tr>`;
  } else {
    tbody.innerHTML = pairs.map((p, idx) => {
      const m = p.mocap || {};
      const i = p.imu || {};
      return `<tr>
        <td>${idx + 1}</td>
        <td class="mono">${fmt(m.hsStartTimeS ?? m.cycleStartTimeS, 2)}</td>
        <td class="mono">${fmt(i.cycleStartTimeS, 2)}</td>
        <td class="mono">${fmt(p.timeErrorS, 3)}</td>
        <td class="mono">${fmt(m.strideLengthM, 3)}</td>
        <td class="mono">${fmt(i.strideLengthM, 3)}</td>
        <td class="mono">${fmt(m.cadenceSpm, 1)}</td>
        <td class="mono">${fmt(i.cadenceSpm, 1)}</td>
        <td class="mono">${fmt(m.stancePct, 1)}</td>
        <td class="mono">${fmt(i.stancePct, 1)}</td>
      </tr>`;
    }).join('');
  }

  // ทุก cycle ที่คำนวณได้ — ไม่บังคับจับคู่
  const mocapCycles = extra.mocap?.perSide?.[side]?.cycles || [];
  const imuCycles = (sideReport?.imuAll && extra.imuCycles)
    ? extra.imuCycles
    : (extra.imuCycles || []);
  const mocapBody = els.mocapCycleTable?.querySelector('tbody');
  const imuBody = els.imuCycleTable?.querySelector('tbody');
  if (mocapBody) {
    mocapBody.innerHTML = mocapCycles.length
      ? mocapCycles.map((c, idx) => `<tr>
          <td>${idx + 1}</td>
          <td class="mono">${fmt(c.hsStartTimeS, 2)}</td>
          <td class="mono">${fmt(c.strideLengthM, 3)}</td>
          <td class="mono">${fmt(c.cadenceSpm, 1)}</td>
          <td class="mono">${fmt(c.stancePct, 1)}</td>
          <td class="mono">${fmt(c.peakShankAngleDeg, 1)}</td>
        </tr>`).join('')
      : '<tr><td colspan="6">ไม่มี cycle</td></tr>';
  }
  if (imuBody) {
    imuBody.innerHTML = imuCycles.length
      ? imuCycles.map((c, idx) => {
        const flags = [
          c.isOpenStride ? 'open' : null,
          c.strideLengthClamped ? 'clamp' : null,
          c.suspectedMissedHs ? 'missHS' : null,
          c.strideLengthUntrusted ? 'untrusted' : null,
        ].filter(Boolean).join(',') || '—';
        return `<tr>
          <td>${idx + 1}</td>
          <td class="mono">${fmt(c.cycleStartTimeS, 2)}</td>
          <td class="mono">${fmt(c.strideLengthM, 3)}</td>
          <td class="mono">${fmt(c.cadenceSpm, 1)}</td>
          <td class="mono">${fmt(c.stancePct, 1)}</td>
          <td class="mono">${flags}</td>
        </tr>`;
      }).join('')
      : '<tr><td colspan="6">ไม่มี cycle</td></tr>';
  }

  // คุณภาพ = note เท่านั้น
  const notes = [];
  notes.push(`sync ω: lag ${fmt(lagInfo.lagS, 3)}s · corr ${fmt(lagInfo.peakCorr, 3)} (${lagInfo.lagSource || '—'}) — ใช้เช็ครูปร่าง`);
  if (extra.peakMocap != null && extra.peakImu != null) {
    notes.push(
      `สเกล |ω| คนละระดับ (MoCap peak ${fmt(extra.peakMocap, 0)} vs IMU ${fmt(extra.peakImu, 0)} deg/s) `
      + '— กราฟ normalize แล้ว',
    );
  }
  if (report.validationPublishable === false) {
    notes.push('lab publishable = false — ยังไม่เคลมความแม่นในเปเปอร์ (ขาด heel-tap / malleolus / onset ตาม SOP)');
  }
  if (dist?.hasCoverageGap) {
    notes.push(`coverage gap ${fmt(dist.coverageGapS, 2)}s — Σ อาจขาดก้าวหลังแยก merged stride`);
  }
  if (align && (align.unpairedMocap > 0 || align.unpairedImu > 0)) {
    notes.push(`HS unpaired: MoCap ${align.unpairedMocap} · IMU ${align.unpairedImu} (detector คนละจุด)`);
  }
  for (const w of (report.warnings || []).slice(0, 5)) {
    notes.push(w);
  }
  els.warnList.innerHTML = notes.map((n) => `<li>${n}</li>`).join('');

  els.reportPanel.classList.remove('hidden');
}

async function runCompare() {
  els.runBtn.disabled = true;
  els.runHint.textContent = 'กำลังคำนวณ…';

  try {
    const overrideMap = buildMapFromRoles();
    const markers = state.markers.length ? state.markers : listMarkers(state.parsed);
    const { resolved, ok, unresolved, anklePlan } = resolveMarkerRoles(markers, overrideMap);
    if (!ok) {
      throw new Error(`แมปยังไม่ชัด: ${unresolved.map((u) => u.role).join(', ')}`);
    }

    const mocap = computeGaitFromMocap(state.parsed, resolved);
    state.mocapResult = mocap;

    const side = els.sideSelect.value;
    const axis = els.axisSelect.value;
    const useEnvelope = els.envelopeCheck.checked
      || anklePlan?.[side]?.angleSource === 'unavailable';

    const mocapSignals = mocap.perSide?.[side]?.signals;
    if (!mocapSignals?.tS?.length || !mocapSignals?.shankAngularVelocityDps?.length) {
      throw new Error(`MoCap ขา ${side} ไม่มี shankAngularVelocityDps — ตรวจแมป ankle`);
    }

    const imuGyro = extractImuGyroSeries(state.imuTrace, side, { axis });
    if (!imuGyro.ok) {
      throw new Error(`IMU ขา ${side} ไม่มีตัวอย่างแกน ${axis} พอ`);
    }

    const signalLag = estimateSignalLagS(
      mocapSignals.tS,
      mocapSignals.shankAngularVelocityDps,
      imuGyro.tS,
      imuGyro.gx,
      {
        useEnvelope,
        rivalScanMaxLagS: 90,
        fineMaxLagS: 0.4,
      },
    );

    if (!signalLag.ok || !Number.isFinite(signalLag.lagS)) {
      throw new Error(`sync ไม่ได้: ${signalLag.reason || 'unknown'} (ลองเปลี่ยนขา/เปิด |ω| envelope)`);
    }

    const series = buildAlignedSeries(
      mocapSignals.tS,
      mocapSignals.shankAngularVelocityDps,
      imuGyro.tS,
      imuGyro.gx,
      signalLag.lagS,
      useEnvelope,
    );
    if (!series) throw new Error('ช่วงเวลทับซ้อนสั้นเกินไปหลังเลื่อน lag');

    // แสดงพารามิเตอร์ก่อน — กราฟเป็นเช็ครูปร่างด้านล่าง
    const report = compareMocapToImu(mocap, state.imuTrace, {
      explorationOnly: true,
      align: {
        lagS: signalLag.lagS,
        lagSource: useEnvelope ? 'signal-xcorr-envelope' : 'signal-xcorr',
        lagTrusted: false,
        fineMaxLagS: 0.4,
        rivalScanMaxLagS: 90,
      },
      minAgreementPairs: 1,
    });

    renderReport(report, side, {
      lagS: signalLag.lagS,
      lagSource: useEnvelope ? 'signal-xcorr-envelope' : 'signal-xcorr',
      peakCorr: signalLag.peakCorr,
    }, {
      peakMocap: series.peakMocap,
      peakImu: series.peakImu,
      mocap,
      imuCycles: reprocessImuTrace(state.imuTrace).bySide?.[side] || [],
    });

    els.chartPanel.classList.remove('hidden');
    els.syncMeta.textContent = [
      `เช็ครูปร่างเท่านั้น · lag ${fmt(signalLag.lagS, 3)}s · corr ${fmt(signalLag.peakCorr, 3)}`,
      useEnvelope ? '|ω| envelope' : 'signed ω',
      series.normalized
        ? `normalize (MoCap peak ${fmt(series.peakMocap, 0)} vs IMU ${fmt(series.peakImu, 0)} deg/s)`
        : null,
    ].filter(Boolean).join(' · ');

    renderChart(series, side, axis, useEnvelope);

    els.runHint.textContent = 'เสร็จแล้ว — เปลี่ยนขา/แกนแล้วคำนวณใหม่ได้';
  } catch (err) {
    console.error(err);
    els.runHint.textContent = err.message || String(err);
    els.mapStatus.className = 'map-status bad';
    els.mapStatus.textContent = err.message || String(err);
  } finally {
    updateReady();
  }
}

// init
bindDrop(els.csvDrop, els.csvInput, onCsvFile);
bindDrop(els.jsonDrop, els.jsonInput, onJsonFile);
document.querySelectorAll('[data-preset]').forEach((btn) => {
  btn.addEventListener('click', () => {
    const preset = PRESETS[btn.dataset.preset];
    if (!preset) return;
    state.roleByMarkerId = { ...preset };
    renderMapGrid();
  });
});
els.runBtn.addEventListener('click', () => runCompare());
['change'].forEach((ev) => {
  els.sideSelect.addEventListener(ev, () => {
    if (state.csvText && state.imuTrace && mapComplete()) {
      // ไม่ auto-run ทุกครั้ง — ผู้ใช้กดปุ่ม (ลดงานหนัก)
    }
  });
});

renderMapGrid();
updateReady();

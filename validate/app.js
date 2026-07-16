import {
  DEFAULT_FLAG_V_END_MPS,
  DEFAULT_FLAG_ACCEL_DEVIATION_G,
  formatNum,
  formatSigned,
  formatDate,
  sensorLabel,
  summarizeCycles,
  computeDistanceCheck,
  distanceCheckClass,
  computeGlobalT0Ms,
  detectJsonKind,
} from "./lib.js";
import { compareMocapToImu, summarizeMocapSide } from "../mocap-analysis/compareImuTrace.js";

const COLORS = {
  LEFT_SHANK: { x: "#ef5350", y: "#4caf50", z: "#7c4dff" },
  RIGHT_SHANK: { x: "#ff9770", y: "#66bb6a", z: "#9575cd" },
};

const state = {
  raw: null,
  mocap: null,
  imuFileName: null,
  mocapFileName: null,
  series: null,
  t0BySensor: {},
  globalT0Ms: null,
  frame: "canonical",
  sensor: "all",
  axis: "accel",
  viewStartSec: 0,
  viewWindowSec: 30,
  tMinSec: 0,
  tMaxSec: 0,
  // threshold ปรับได้จาก UI — ค่าเริ่มต้นเป็นแค่ placeholder ที่ยังไม่ผ่านการยืนยันด้วยข้อมูลจริง
  zuptThresholds: { vEndMps: DEFAULT_FLAG_V_END_MPS, accelDeviationG: DEFAULT_FLAG_ACCEL_DEVIATION_G },
};

const els = {
  imuSlot: document.getElementById("imuSlot"),
  mocapSlot: document.getElementById("mocapSlot"),
  imuFileInput: document.getElementById("imuFileInput"),
  mocapFileInput: document.getElementById("mocapFileInput"),
  imuPickBtn: document.getElementById("imuPickBtn"),
  mocapPickBtn: document.getElementById("mocapPickBtn"),
  imuFileName: document.getElementById("imuFileName"),
  mocapFileName: document.getElementById("mocapFileName"),
  compareSection: document.getElementById("compareSection"),
  compareBadge: document.getElementById("compareBadge"),
  compareLoading: document.getElementById("compareLoading"),
  compareBody: document.getElementById("compareBody"),
  mocapOnlySection: document.getElementById("mocapOnlySection"),
  mocapOnlyCard: document.getElementById("mocapOnlyCard"),
  content: document.getElementById("content"),
  warnings: document.getElementById("warnings"),
  overviewCard: document.getElementById("overviewCard"),
  samplingCard: document.getElementById("samplingCard"),
  calibrationCard: document.getElementById("calibrationCard"),
  groundTruthCard: document.getElementById("groundTruthCard"),
  cyclesBadge: document.getElementById("cyclesBadge"),
  cyclesSummary: document.getElementById("cyclesSummary"),
  cyclesTableWrap: document.getElementById("cyclesTableWrap"),
  thresholdVEnd: document.getElementById("thresholdVEnd"),
  thresholdAccelDev: document.getElementById("thresholdAccelDev"),
  noteText: document.getElementById("noteText"),
  chart: document.getElementById("chart"),
  chartTitle: document.getElementById("chartTitle"),
  timeSlider: document.getElementById("timeSlider"),
  sliderStart: document.getElementById("sliderStart"),
  sliderEnd: document.getElementById("sliderEnd"),
  sampleTable: document.querySelector("#sampleTable tbody"),
  resetZoom: document.getElementById("resetZoom"),
  fitWindow: document.getElementById("fitWindow"),
};

function errorPctClass(errorPct) {
  if (!Number.isFinite(errorPct)) return "unreliable";
  return distanceCheckClass(errorPct);
}

function sideBadgeHtml(side) {
  const cls = side === "L" ? "left" : side === "R" ? "right" : "";
  const label = side === "L" ? "L" : side === "R" ? "R" : "?";
  return `<span class="side-badge ${cls}">${label}</span>`;
}

// ========== Chart data prep (raw IMU viewer, unchanged logic) ==========

function buildSeries(samples) {
  const bySensor = {};
  const t0BySensor = {};

  for (const sample of samples) {
    const key = sample.sensorKey;
    if (t0BySensor[key] == null) t0BySensor[key] = sample.t_ms;
    if (!bySensor[key]) {
      bySensor[key] = {
        t: [],
        seq: [],
        side: sample.side,
        accelSensor: [[], [], []],
        gyroSensor: [[], [], []],
        accelCanonical: [[], [], []],
        gyroCanonical: [[], [], []],
      };
    }
    const bucket = bySensor[key];
    const tSec = (sample.t_ms - t0BySensor[key]) / 1000;
    bucket.t.push(tSec);
    bucket.seq.push(sample.seq);
    const accelSensor = sample.raw_accel_sensor ?? [0, 0, 0];
    const gyroSensor = sample.raw_gyro_sensor ?? [0, 0, 0];
    const accelCanonical = sample.raw_accel_canonical ?? accelSensor;
    const gyroCanonical = sample.raw_gyro_canonical ?? gyroSensor;
    for (let i = 0; i < 3; i += 1) {
      bucket.accelSensor[i].push(accelSensor[i]);
      bucket.gyroSensor[i].push(gyroSensor[i]);
      bucket.accelCanonical[i].push(accelCanonical[i]);
      bucket.gyroCanonical[i].push(gyroCanonical[i]);
    }
  }

  state.t0BySensor = t0BySensor;
  return bySensor;
}

function sampleTimeSec(sample) {
  const t0 = state.t0BySensor[sample.sensorKey] ?? sample.t_ms;
  return (sample.t_ms - t0) / 1000;
}

function accelKey(frame) {
  return frame === "sensor" ? "raw_accel_sensor" : "raw_accel_canonical";
}

function gyroKey(frame) {
  return frame === "sensor" ? "raw_gyro_sensor" : "raw_gyro_canonical";
}

function seriesValues(sensorData, kind) {
  const accel = state.frame === "sensor" ? sensorData.accelSensor : sensorData.accelCanonical;
  const gyro = state.frame === "sensor" ? sensorData.gyroSensor : sensorData.gyroCanonical;
  return kind === "accel" ? accel : gyro;
}

function sensorList() {
  if (state.sensor === "all") return Object.keys(state.series);
  return [state.sensor];
}

function makeTraces(kind) {
  const traces = [];
  const axisNames = ["X", "Y", "Z"];
  for (const sensorKey of sensorList()) {
    const s = state.series[sensorKey];
    if (!s) continue;
    const values = seriesValues(s, kind);
    const colors = COLORS[sensorKey] ?? COLORS.LEFT_SHANK;
    const prefix = s.side === "L" ? "L" : s.side === "R" ? "R" : sensorKey;
    for (let i = 0; i < 3; i += 1) {
      traces.push({
        x: s.t,
        y: values[i],
        name: `${prefix} ${axisNames[i]}`,
        type: "scattergl",
        mode: "lines",
        line: { width: 1.2, color: colors[["x", "y", "z"][i]] },
        hovertemplate: `${prefix} ${axisNames[i]}<br>t=%{x:.3f}s<br>val=%{y}<extra></extra>`,
      });
    }
  }
  return traces;
}

function getLayout() {
  const showBoth = state.axis === "both";
  const layout = {
    paper_bgcolor: "#ffffff",
    plot_bgcolor: "#ffffff",
    font: { color: "#424242", family: "Inter, sans-serif", size: 12 },
    margin: { l: 56, r: 20, t: 24, b: 56 },
    hovermode: "x unified",
    dragmode: "pan",
    showlegend: true,
    legend: { orientation: "h", y: 1.08, x: 0, bgcolor: "rgba(0,0,0,0)" },
    xaxis: {
      title: "เวลา (วินาที, อ้างอิงต่อเซ็นเซอร์)",
      gridcolor: "#eeeeee",
      zerolinecolor: "#e0e0e0",
      rangeslider: { visible: true, bgcolor: "#f5f5f5", bordercolor: "#e0e0e0" },
      range: [state.viewStartSec, state.viewStartSec + state.viewWindowSec],
    },
    yaxis: { title: state.axis === "gyro" ? "Gyro" : "Accel", gridcolor: "#eeeeee", zerolinecolor: "#e0e0e0" },
  };

  if (showBoth) {
    layout.grid = { rows: 2, columns: 1, pattern: "independent", roworder: "top to bottom" };
    layout.yaxis2 = { title: "Gyro", gridcolor: "#eeeeee", zerolinecolor: "#e0e0e0", anchor: "x2" };
    layout.xaxis2 = {
      title: "เวลา (วินาที)",
      gridcolor: "#eeeeee",
      zerolinecolor: "#e0e0e0",
      matches: "x",
      rangeslider: { visible: false },
    };
  }

  return layout;
}

function buildPlotTraces() {
  if (state.axis === "both") {
    const accel = makeTraces("accel").map((t) => ({ ...t, xaxis: "x", yaxis: "y" }));
    const gyro = makeTraces("gyro").map((t) => ({ ...t, xaxis: "x2", yaxis: "y2" }));
    return [...accel, ...gyro];
  }
  return makeTraces(state.axis);
}

function updateChartTitle() {
  const titles = { accel: "Accelerometer", gyro: "Gyroscope", both: "Accel + Gyro" };
  els.chartTitle.textContent = titles[state.axis] ?? "Sensor data";
}

function renderChart() {
  if (!state.series) return;
  updateChartTitle();
  const config = {
    responsive: true,
    scrollZoom: true,
    displayModeBar: true,
    modeBarButtonsToRemove: ["select2d", "lasso2d"],
    displaylogo: false,
  };
  Plotly.react(els.chart, buildPlotTraces(), getLayout(), config);
}

function updateTable() {
  if (!state.raw?.samples?.length) return;
  const viewEnd = state.viewStartSec + state.viewWindowSec;
  const rows = state.raw.samples
    .filter((s) => {
      const tSec = sampleTimeSec(s);
      return tSec >= state.viewStartSec && tSec <= viewEnd;
    })
    .filter((s) => state.sensor === "all" || s.sensorKey === state.sensor)
    .slice(0, 100);

  els.sampleTable.innerHTML = rows
    .map((s) => {
      const tSec = sampleTimeSec(s).toFixed(3);
      const a = s[accelKey(state.frame)] ?? [0, 0, 0];
      const g = s[gyroKey(state.frame)] ?? [0, 0, 0];
      return `<tr>
        <td>${tSec}</td>
        <td>${s.seq}</td>
        <td>${s.sensorKey}</td>
        <td>${s.side}</td>
        <td>${a[0]}</td><td>${a[1]}</td><td>${a[2]}</td>
        <td>${g[0]}</td><td>${g[1]}</td><td>${g[2]}</td>
      </tr>`;
    })
    .join("");
}

function updateSliderLabels() {
  els.sliderStart.textContent = `${formatNum(state.viewStartSec, 1)} s`;
  els.sliderEnd.textContent = `${formatNum(state.viewStartSec + state.viewWindowSec, 1)} s`;
}

function syncSliderFromState() {
  const maxStart = Math.max(0, state.tMaxSec - state.viewWindowSec);
  const pct = maxStart === 0 ? 0 : (state.viewStartSec / maxStart) * 100;
  els.timeSlider.value = String(pct);
  updateSliderLabels();
}

// ========== Header / diagnostics rendering ==========

function fact(label, value, muted) {
  return `<div class="fact">
    <div class="fact-label">${label}</div>
    <div class="fact-value${muted ? " muted" : ""}">${value}</div>
  </div>`;
}

function sourceBadgeHtml(data) {
  if (data.source === "live" && data.hasSensorFrameRaw) {
    return '<span class="card-badge ok">live</span>';
  }
  return '<span class="card-badge warn">demo / no sensor-frame</span>';
}

function renderWarnings(data) {
  const warnings = [];

  if (data.source !== "live" || data.hasSensorFrameRaw === false) {
    warnings.push("ไฟล์นี้ไม่มี raw sensor-frame จริง (มาจาก demo หรือไม่มีเซนเซอร์ต่อ) — reprocess offline ไม่ได้ถ้า AXIS_MAP ผิด");
  }
  if (data.truncated) {
    warnings.push(`raw samples ถูกตัด (truncated) ที่ ${formatNum(data.sampleCount, 0)} samples — ข้อมูลไม่ครบ session`);
  }
  if (data.cyclesTruncated) {
    warnings.push("cycle diagnostics ถูกตัด (cyclesTruncated) — เกิน cap ของการบันทึก");
  }
  const dropped = Object.entries(data.droppedBySensor || {}).filter(([, v]) => v > 0);
  if (dropped.length) {
    warnings.push(`มี sample หายไปจาก seq gap: ${dropped.map(([k, v]) => `${k}=${v}`).join(", ")}`);
  }
  const disc = Object.entries(data.seqDiscontinuitiesBySensor || {}).filter(([, v]) => v > 0);
  if (disc.length) {
    warnings.push(`เจอ seq discontinuity (reset/reconnect): ${disc.map(([k, v]) => `${k}=${v}`).join(", ")}`);
  }
  if (data.schemaVersion < 3) {
    warnings.push(`ไฟล์นี้เป็น schema v${data.schemaVersion} (เก่ากว่า v3) — ไม่มี cycles[]/ZUPT diagnostics`);
  }

  els.warnings.innerHTML = warnings.length
    ? `<div class="warning-banner">⚠️ ${warnings.join(" · ")}</div>`
    : "";
}

function renderOverview(data) {
  const html = `
    <div class="card-header">
      <span class="card-title">Session Overview ${sourceBadgeHtml(data)}</span>
      <span class="card-badge neutral">schema v${data.schemaVersion ?? "?"}</span>
    </div>
    <div class="fact-grid">
      ${fact("Recorded", formatDate(data.recordedAt))}
      ${fact("Exported", formatDate(data.exportedAt))}
      ${fact("Duration", `${formatNum(state.tMaxSec - state.tMinSec, 1)} s`)}
      ${fact("App", `${data.app ?? "—"} v${data.appVersion ?? "—"}`)}
      ${fact("Firmware build", data.firmwareBuildTag ?? "—", !data.firmwareBuildTag)}
      ${fact("Sample count", formatNum(data.sampleCount, 0))}
      ${fact("Cycle count", data.cycleCount != null ? formatNum(data.cycleCount, 0) : "—", data.cycleCount == null)}
    </div>`;
  els.overviewCard.innerHTML = html;
}

function renderSampling(data) {
  const sensors = Object.keys(data.sampleRateHzBySensor || {});
  const rows = sensors
    .map((key) => {
      const side = data.samples?.find((s) => s.sensorKey === key)?.side ?? null;
      return `<tr>
        <td>${sideBadgeHtml(side)}${key}</td>
        <td>${formatNum(data.sampleRateHzBySensor[key], 2)} Hz</td>
        <td>${data.droppedBySensor?.[key] ?? 0}</td>
        <td>${data.seqDiscontinuitiesBySensor?.[key] ?? 0}</td>
        <td>${data.packetVersionBySensor?.[key] ?? "—"}</td>
      </tr>`;
    })
    .join("");

  els.samplingCard.innerHTML = `
    <div class="card-header">
      <span class="card-title">Sampling & Data Quality</span>
    </div>
    <div class="fact-grid" style="margin-bottom:14px;">
      ${fact("Nominal rate", `${formatNum(data.sampleRateHzNominal, 0)} Hz`)}
      ${fact("Measured rate", `${formatNum(data.sampleRateHzMeasured, 2)} Hz`)}
    </div>
    <table class="sensor-table">
      <thead><tr><th>Sensor</th><th>Rate</th><th>Dropped</th><th>Discontinuity</th><th>Packet ver.</th></tr></thead>
      <tbody>${rows || `<tr><td colspan="5" class="fact-value muted">ไม่มีข้อมูล</td></tr>`}</tbody>
    </table>`;
}

function renderCalibration(data) {
  const entries = Object.entries(data.calibrationBySensor || {});
  if (!entries.length) {
    els.calibrationCard.innerHTML = `
      <div class="card-header"><span class="card-title">Calibration</span></div>
      <div class="empty-note">ไม่มีข้อมูล calibration ในไฟล์นี้</div>`;
    return;
  }

  const rows = entries
    .map(([key, cal]) => {
      const side = data.samples?.find((s) => s.sensorKey === key)?.side ?? cal.side ?? null;
      const bias = cal.gyroBiasDps || {};
      const qualityOk = cal.quality?.stillEnough && cal.quality?.gravityOk;
      return `<tr>
        <td>${sideBadgeHtml(side)}${key}</td>
        <td>${formatSigned(bias.gx, 2)}</td>
        <td>${formatSigned(bias.gy, 2)}</td>
        <td>${formatSigned(bias.gz, 2)}</td>
        <td>${formatNum(cal.accelMeanG, 3)} g</td>
        <td>${formatNum(cal.shankLengthM, 3)} m</td>
        <td class="${qualityOk ? "flag-ok" : "flag-bad"}">${qualityOk ? "OK" : "check"}</td>
        <td>${formatDate(cal.savedAt ? new Date(cal.savedAt).toISOString() : null)}</td>
      </tr>`;
    })
    .join("");

  els.calibrationCard.innerHTML = `
    <div class="card-header"><span class="card-title">Calibration (gyro bias / shank length)</span></div>
    <table class="sensor-table">
      <thead>
        <tr><th>Sensor</th><th>gx</th><th>gy</th><th>gz</th><th>accel</th><th>shank</th><th>quality</th><th>saved</th></tr>
      </thead>
      <tbody>${rows}</tbody>
    </table>`;
}

function renderGroundTruth(data) {
  const gt = data.groundTruth || {};
  const check = computeDistanceCheck(data);

  let distanceHtml = "";
  if (check.hasCheck) {
    distanceHtml = `<div class="distance-check">
      ${check.perSensor
        .map((p) => {
          const hasGap = p.noDataCount > 0;
          // sum ที่ขาด cycle ไปทำให้ errorPct เพี้ยนไปทางลบเกินจริงเสมอ (นับน้อยกว่าที่เดินจริง)
          // สีปกติ (ok/warn/bad) จะดูน่าเชื่อถือเกินจริงทั้งที่ตัวเลขไม่ครบ จึงบังคับเป็นสีเทา + ⚠️ แทน
          const cls = hasGap ? "unreliable" : distanceCheckClass(p.errorPct);
          const valuePrefix = hasGap ? "⚠️ " : "";
          const noDataNote = hasGap
            ? ` <span class="flag-bad">(${p.noDataCount} cycle ไม่มีข้อมูล — ไม่รวมใน sum, error% นี้ไม่น่าเชื่อถือ)</span>`
            : "";
          return `<div class="distance-check__stat">
            <div class="distance-check__value ${cls}">${valuePrefix}${formatSigned(p.errorPct, 1)}%</div>
            <div class="distance-check__label">${sensorLabel(p.sensorKey, p.side)} · sum ${formatNum(p.sumStrideLengthM, 2)}m / ${p.cycleCount} cycles${noDataNote}</div>
          </div>`;
        })
        .join("")}
      <div class="distance-check__stat">
        <div class="distance-check__value">${formatNum(check.distanceM, 2)} m</div>
        <div class="distance-check__label">Ground truth distance</div>
      </div>
    </div>`;
  } else if (check.perSensor.length) {
    distanceHtml = `<div class="empty-note" style="margin-top:12px;">ไม่มี groundTruth.distanceM ให้เทียบ (มีแต่ผลรวม stride length)</div>`;
  } else {
    distanceHtml = `<div class="empty-note" style="margin-top:12px;">ไม่มี cycles ให้คำนวณระยะเทียบ ground truth</div>`;
  }

  els.groundTruthCard.innerHTML = `
    <div class="card-header"><span class="card-title">Ground Truth & Distance Check</span></div>
    <div class="fact-grid">
      ${fact("Distance (measured)", Number.isFinite(gt.distanceM) ? `${formatNum(gt.distanceM, 2)} m` : "—", !Number.isFinite(gt.distanceM))}
      ${fact("Steps (manual count)", Number.isFinite(gt.stepCountManual) ? formatNum(gt.stepCountManual, 0) : "—", !Number.isFinite(gt.stepCountManual))}
      ${fact("Notes", gt.notes || "—", !gt.notes)}
    </div>
    ${distanceHtml}`;
}

function cycleRowHtml(c, globalT0Ms, thresholds) {
  const zc = c.zuptCheck || {};
  const hasVEnd = Number.isFinite(zc.vEndPreDrift);
  const hasDev = Number.isFinite(zc.zuptAccelDeviationG);
  const vEndFlag = hasVEnd && Math.abs(zc.vEndPreDrift) > thresholds.vEndMps;
  const devFlag = hasDev && zc.zuptAccelDeviationG > thresholds.accelDeviationG;
  const tSec = Number.isFinite(c.cycleStartTimestampMs) && Number.isFinite(globalT0Ms)
    ? ((c.cycleStartTimestampMs - globalT0Ms) / 1000).toFixed(2)
    : "—";
  // ไม่มีข้อมูล (null จริงจาก processor) ต้องแยกจาก "ผ่านเกณฑ์" อย่างชัดเจน — ไม่ใช้ flag-ok
  // เพราะนั่นจะอ่านผิดว่า ZUPT ดี ทั้งที่จริงคือวัดไม่ได้เลย
  const vEndCellClass = !hasVEnd ? "clamped" : vEndFlag ? "flag-bad" : "flag-ok";
  const devCellClass = !hasDev ? "clamped" : devFlag ? "flag-bad" : "flag-ok";
  return `<tr>
    <td>${sideBadgeHtml(c.side)}${c.sensorKey ?? "—"}</td>
    <td>${tSec}</td>
    <td>${c.cycleKey ?? "—"}</td>
    <td>${formatNum(c.strideLengthM, 3)}</td>
    <td class="${c.strideLengthClamped ? "clamped" : ""}">${c.strideLengthClamped ? "clamped" : "—"}</td>
    <td>${formatSigned(zc.vStartPreDrift, 3)}</td>
    <td class="${vEndCellClass}">${hasVEnd ? formatSigned(zc.vEndPreDrift, 3) : "no data"}</td>
    <td>${zc.windowSource ?? "—"}</td>
    <td class="${devCellClass}">${hasDev ? formatNum(zc.zuptAccelDeviationG, 3) : "no data"}</td>
  </tr>`;
}

function readThresholdInputs() {
  const vEnd = Number.parseFloat(els.thresholdVEnd?.value);
  const accelDev = Number.parseFloat(els.thresholdAccelDev?.value);
  state.zuptThresholds = {
    vEndMps: Number.isFinite(vEnd) && vEnd > 0 ? vEnd : DEFAULT_FLAG_V_END_MPS,
    accelDeviationG: Number.isFinite(accelDev) && accelDev > 0 ? accelDev : DEFAULT_FLAG_ACCEL_DEVIATION_G,
  };
}

function renderCycles(data) {
  const cycles = data.cycles;
  const summary = summarizeCycles(cycles, state.zuptThresholds);

  if (!summary) {
    els.cyclesBadge.textContent = "0 cycles";
    els.cyclesBadge.className = "card-badge neutral";
    els.cyclesSummary.innerHTML = "";
    els.cyclesTableWrap.innerHTML = `<div class="empty-note">
      ไม่มี cycle diagnostics ในไฟล์นี้ — อาจเป็นไฟล์ schema เก่า (v${data.schemaVersion ?? "?"}) ก่อนมี ZUPT export
      หรือถูก export จากหน้า /debug (swing test) ซึ่งไม่รัน gait processor
    </div>`;
    return;
  }

  els.cyclesBadge.textContent = `${summary.count} cycles`;
  els.cyclesBadge.className = `card-badge ${summary.flaggedCount > 0 ? "warn" : "ok"}`;

  const sourceBreakdown = Object.entries(summary.sourceCounts)
    .map(([src, n]) => `${src}: ${n}`)
    .join(" · ");

  els.cyclesSummary.innerHTML = `
    <div class="stat-tile"><div class="stat-tile__value">${summary.count}</div><div class="stat-tile__label">Cycles</div></div>
    <div class="stat-tile"><div class="stat-tile__value" style="color:${summary.flaggedCount ? "#c62828" : "#2e7d32"}">${summary.flaggedCount}</div><div class="stat-tile__label">Flagged ZUPT</div></div>
    <div class="stat-tile"><div class="stat-tile__value" style="color:${summary.noZuptDataCount ? "#e65100" : "#212121"}">${summary.noZuptDataCount}</div><div class="stat-tile__label">No ZUPT data</div></div>
    <div class="stat-tile"><div class="stat-tile__value" style="color:${summary.clampedCount ? "#e65100" : "#212121"}">${summary.clampedCount}</div><div class="stat-tile__label">Clamped</div></div>
    <div class="stat-tile"><div class="stat-tile__value">${formatNum(summary.meanAbsVEnd, 3)}</div><div class="stat-tile__label">Mean |vEnd| m/s${summary.noZuptDataCount ? " *" : ""}</div></div>
    <div class="stat-tile"><div class="stat-tile__value">${formatNum(summary.meanZuptAccelDeviationG, 3)}</div><div class="stat-tile__label">Mean accelDev g${summary.noZuptDataCount ? " *" : ""}</div></div>`;

  const globalT0Ms = state.globalT0Ms;
  const rows = cycles.map((c) => cycleRowHtml(c, globalT0Ms, state.zuptThresholds)).join("");
  const meanNote = summary.noZuptDataCount
    ? `<br>* ค่าเฉลี่ยคำนวณจาก cycle ที่มีข้อมูลจริงเท่านั้น (ไม่รวม ${summary.noZuptDataCount} cycle ที่ "no data")`
    : "";

  els.cyclesTableWrap.innerHTML = `
    <p class="chart-hint" style="margin:0 0 10px;">
      Window source: ${sourceBreakdown || "—"} ·
      flag เมื่อ |vEndPreDrift| &gt; ${state.zuptThresholds.vEndMps} m/s หรือ zuptAccelDeviationG &gt; ${state.zuptThresholds.accelDeviationG} g
      (ค่า default ยังไม่ผ่านการยืนยันด้วยข้อมูลเดินจริง — ปรับได้ที่ช่องด้านบน)${meanNote}
    </p>
    <div class="cycles-table-wrap">
      <table class="cycles-table">
        <thead>
          <tr>
            <th>Sensor</th><th>t (s)</th><th>cycleKey</th><th>stride (m)</th><th>clamp</th>
            <th>vStart</th><th>vEnd</th><th>window</th><th>accelDev (g)</th>
          </tr>
        </thead>
        <tbody>${rows}</tbody>
      </table>
    </div>`;
}

function renderNote(data) {
  els.noteText.textContent = data.note || "—";
}

// ========== MoCap summary + MoCap↔IMU compare ==========

function updateSlotUi(kind) {
  const loaded = kind === "imu" ? Boolean(state.raw) : Boolean(state.mocap);
  const slot = kind === "imu" ? els.imuSlot : els.mocapSlot;
  const nameEl = kind === "imu" ? els.imuFileName : els.mocapFileName;
  const fileName = kind === "imu" ? state.imuFileName : state.mocapFileName;
  slot.classList.toggle("loaded", loaded);
  if (loaded && fileName) {
    nameEl.textContent = fileName;
    nameEl.classList.remove("hidden");
  } else {
    nameEl.classList.add("hidden");
    nameEl.textContent = "";
  }
}

function renderMocapOnly() {
  if (!state.mocap) {
    els.mocapOnlySection.classList.add("hidden");
    return;
  }
  // ถ้ามี IMU ด้วย ให้ซ่อน mocap-only (ไปโชว์ใน compare แทน)
  if (state.raw) {
    els.mocapOnlySection.classList.add("hidden");
    return;
  }

  const m = state.mocap;
  const L = summarizeMocapSide(m.perSide?.L);
  const R = summarizeMocapSide(m.perSide?.R);

  els.mocapOnlyCard.innerHTML = `
    <div class="card-header">
      <span class="card-title">MoCap Ground Truth <span class="card-badge ok">loaded</span></span>
    </div>
    <div class="fact-grid" style="margin-bottom:14px;">
      ${fact("File", state.mocapFileName || "—")}
      ${fact("Duration", Number.isFinite(m.session?.durationS) ? `${formatNum(m.session.durationS, 1)} s` : "—")}
      ${fact("Pelvis net forward", Number.isFinite(m.session?.pelvisNetForwardDisplacementM) ? `${formatNum(m.session.pelvisNetForwardDisplacementM, 3)} m` : "—")}
      ${fact("Bilateral cadence", Number.isFinite(m.bilateral?.trueCadenceSpm) ? `${formatNum(m.bilateral.trueCadenceSpm, 1)} spm` : "—")}
      ${fact("sameSideRepeats", formatNum(m.bilateral?.sameSideRepeats ?? 0, 0))}
    </div>
    <table class="sensor-table">
      <thead>
        <tr><th>Side</th><th>Cycles</th><th>Mean stride</th><th>Mean cadence</th><th>Mean speed</th><th>Mean stance%</th></tr>
      </thead>
      <tbody>
        <tr>
          <td>${sideBadgeHtml("L")}Left</td>
          <td>${L.cycleCount}</td>
          <td>${formatNum(L.meanStrideLengthM, 3)} m</td>
          <td>${formatNum(L.meanCadenceSpm, 1)}</td>
          <td>${formatNum(L.meanWalkingSpeedMps, 3)}</td>
          <td>${formatNum(L.meanStancePct, 1)}</td>
        </tr>
        <tr>
          <td>${sideBadgeHtml("R")}Right</td>
          <td>${R.cycleCount}</td>
          <td>${formatNum(R.meanStrideLengthM, 3)} m</td>
          <td>${formatNum(R.meanCadenceSpm, 1)}</td>
          <td>${formatNum(R.meanWalkingSpeedMps, 3)}</td>
          <td>${formatNum(R.meanStancePct, 1)}</td>
        </tr>
      </tbody>
    </table>
    <p class="chart-hint" style="margin-top:12px;">อัปโหลด IMU trace ด้วยเพื่อเทียบ MoCap ↔ IMU อัตโนมัติ</p>`;
  els.mocapOnlySection.classList.remove("hidden");
}

function metricRowHtml(row) {
  const cls = row.comparable ? errorPctClass(row.errorPct) : "unreliable";
  return `<tr>
    <td>${row.metric}</td>
    <td>${row.unit}</td>
    <td>${formatNum(row.mocap, 3)}</td>
    <td>${formatNum(row.imu, 3)}</td>
    <td>${row.comparable ? formatSigned(row.error, 3) : "—"}</td>
    <td class="${cls}">${row.comparable ? formatSigned(row.errorPct, 1) + "%" : "—"}</td>
  </tr>`;
}

function sideCompareHtml(side, block) {
  if (!block?.present) {
    return `<div class="compare-side">
      <div class="compare-side__title">${sideBadgeHtml(side)}ขา ${side}</div>
      <div class="empty-note">ไม่มีข้อมูลทั้งสองฝั่ง</div>
    </div>`;
  }

    const rows = (block.metrics || []).map(metricRowHtml).join("");
    const align = block.alignment;
    const alignNote = align
      ? `<p class="chart-hint" style="margin:0 0 8px;">
          align: <b>${align.mode}</b>
          · source ${align.lagSource || "—"}
          · paired ${align.pairedCount}
          · lag ${formatNum(align.lagS, 2)} s
          · unpaired MoCap/IMU ${align.unpairedMocap}/${align.unpairedImu}
          ${align.timingMetricValid
            ? `· HS timing ${formatSigned(align.meanTimeErrorS * 1000, 0)} ms`
            : "· HS timing n/a (ไม่ใช่เมตริกเมื่อ sync จาก events)"}
        </p>`
      : "";
    return `<div class="compare-side">
    <div class="compare-side__title">
      ${sideBadgeHtml(side)}ขา ${side}
      <span class="card-badge neutral">MoCap ${block.mocapAll?.cycleCount ?? block.mocap.cycleCount} · IMU ${block.imuAll?.cycleCount ?? block.imu.cycleCount}</span>
    </div>
    ${alignNote}
    <div class="cycles-table-wrap">
      <table class="cycles-table compare-table">
        <thead>
          <tr><th>Metric</th><th>Unit</th><th>MoCap</th><th>IMU</th><th>Error</th><th>Error %</th></tr>
        </thead>
        <tbody>${rows}</tbody>
      </table>
    </div>
  </div>`;
}

function renderCompareReport(report) {
  if (!report?.ok) {
    els.compareBadge.textContent = "failed";
    els.compareBadge.className = "card-badge bad";
    els.compareBody.innerHTML = `<div class="empty-note">${report?.error || "เทียบไม่ได้"}</div>`;
    return;
  }

  els.compareBadge.textContent = report.imuSource === "reprocess" ? "reprocess" : "trace cycles";
  els.compareBadge.className = "card-badge ok";

  const warningsHtml = (report.warnings || []).length
    ? `<div class="warning-banner" style="margin-bottom:14px;">⚠️ ${(report.warnings || []).join(" · ")}</div>`
    : "";

  const session = report.session || {};
  const distanceRows = ["L", "R"]
    .map((side) => {
      const d = session.distanceBySide?.[side];
      if (!d || !Number.isFinite(d.imuSumStrideLengthM)) return "";
      return `<tr>
        <td>${sideBadgeHtml(side)}${side}</td>
        <td>${formatNum(d.imuSumStrideLengthM, 3)} m</td>
        <td class="${errorPctClass(d.vsMocapPelvisNetPct)}">${Number.isFinite(d.vsMocapPelvisNetPct) ? formatSigned(d.vsMocapPelvisNetPct, 1) + "%" : "—"}</td>
        <td class="${errorPctClass(d.vsGroundTruthPct)}">${Number.isFinite(d.vsGroundTruthPct) ? formatSigned(d.vsGroundTruthPct, 1) + "%" : "—"}</td>
      </tr>`;
    })
    .join("");

  els.compareBody.innerHTML = `
    ${warningsHtml}
    <div class="fact-grid" style="margin-bottom:16px;">
      ${fact("IMU source", report.imuSource)}
      ${fact("MoCap pelvis net", Number.isFinite(session.mocapPelvisNetForwardM) ? `${formatNum(session.mocapPelvisNetForwardM, 3)} m` : "—")}
      ${fact("MoCap cadence (bilateral)", Number.isFinite(session.mocapTrueCadenceSpm) ? `${formatNum(session.mocapTrueCadenceSpm, 1)} spm` : "—")}
      ${fact("IMU ground truth dist.", Number.isFinite(session.imuGroundTruthDistanceM) ? `${formatNum(session.imuGroundTruthDistanceM, 2)} m` : "—", !Number.isFinite(session.imuGroundTruthDistanceM))}
    </div>
    <div class="compare-sides">
      ${sideCompareHtml("L", report.sides.L)}
      ${sideCompareHtml("R", report.sides.R)}
    </div>
    <div style="margin-top:18px;">
      <div class="card-title" style="margin-bottom:10px;">Distance check (sum stride ต่อข้าง)</div>
      <table class="sensor-table">
        <thead>
          <tr><th>Side</th><th>IMU Σ stride</th><th>vs MoCap net</th><th>vs IMU GT dist</th></tr>
        </thead>
        <tbody>${distanceRows || `<tr><td colspan="4" class="fact-value muted">ไม่มีข้อมูล</td></tr>`}</tbody>
      </table>
    </div>
    <ul class="compare-notes">
      ${(report.notes || []).map((n) => `<li>${n}</li>`).join("")}
    </ul>`;
}

async function refreshCompare() {
  if (!state.raw || !state.mocap) {
    els.compareSection.classList.add("hidden");
    return;
  }

  els.compareSection.classList.remove("hidden");
  els.compareLoading.classList.remove("hidden");
  els.compareBody.innerHTML = "";
  els.compareBadge.textContent = "…";
  els.compareBadge.className = "card-badge neutral";

  // ให้ UI วาด loading ก่อน reprocess (อาจใช้เวลาสักครู่)
  await new Promise((r) => window.setTimeout(r, 30));

  let report;
  try {
    report = compareMocapToImu(state.mocap, state.raw);
  } catch (error) {
    report = { ok: false, error: error?.message || String(error) };
  }

  els.compareLoading.classList.add("hidden");
  renderCompareReport(report);
}

// ========== Load / wire up ==========

function loadImuPayload(data, fileName = null) {
  if (!data?.samples?.length && !(data?.cycles?.length)) {
    window.alert("ไฟล์ IMU ไม่มี samples หรือ cycles — รูปแบบไม่ถูกต้อง");
    return;
  }

  state.raw = data;
  state.imuFileName = fileName;
  updateSlotUi("imu");
  readThresholdInputs();

  if (data.samples?.length) {
    state.series = buildSeries(data.samples);
    state.globalT0Ms = computeGlobalT0Ms(data);
    const durations = Object.values(state.series).map((s) => s.t[s.t.length - 1] ?? 0);
    state.tMinSec = 0;
    state.tMaxSec = Math.max(...durations, 0);
    state.viewWindowSec = Math.min(30, state.tMaxSec || 30);
    state.viewStartSec = Math.max(0, state.tMaxSec - state.viewWindowSec);
  } else {
    state.series = null;
    state.globalT0Ms = computeGlobalT0Ms(data);
    state.tMinSec = 0;
    state.tMaxSec = 0;
  }

  els.content.classList.remove("hidden");
  renderWarnings(data);
  renderOverview(data);
  renderSampling(data);
  renderCalibration(data);
  renderGroundTruth(data);
  renderCycles(data);
  renderNote(data);

  if (state.series) {
    syncSliderFromState();
    renderChart();
    updateTable();
  }

  renderMocapOnly();
  void refreshCompare();
}

function loadMocapPayload(data, fileName = null) {
  if (!data?.perSide) {
    window.alert("ไฟล์ MoCap ไม่มี perSide — ควรเป็น *.gait-params.json จาก mocap-analysis/run.js");
    return;
  }

  state.mocap = data;
  state.mocapFileName = fileName;
  updateSlotUi("mocap");
  renderMocapOnly();
  void refreshCompare();
}

async function ingestJsonFile(file, preferredKind = null) {
  let data;
  try {
    data = JSON.parse(await file.text());
  } catch {
    window.alert("อ่าน JSON ไม่ได้ — ไฟล์เสียหรือไม่ใช่ JSON");
    return;
  }

  let kind = detectJsonKind(data);
  if (preferredKind && kind !== "unknown" && kind !== preferredKind) {
    const ok = window.confirm(
      `ไฟล์นี้ดูเหมือนเป็น ${kind.toUpperCase()} แต่คุณเลือกช่อง ${preferredKind.toUpperCase()}. โหลดตามชนิดที่ตรวจเจอไหม?`,
    );
    if (!ok) return;
  }
  if (kind === "unknown") {
    if (preferredKind === "imu") kind = "imu";
    else if (preferredKind === "mocap") kind = "mocap";
    else {
      window.alert("ไม่รู้จักรูปแบบไฟล์ — ต้องเป็น IMU trace หรือ MoCap gait-params.json");
      return;
    }
  }

  if (kind === "imu") loadImuPayload(data, file.name);
  else loadMocapPayload(data, file.name);
}

function wireUploadSlot({ slot, input, pickBtn, kind }) {
  const openPicker = () => input.click();
  pickBtn?.addEventListener("click", (e) => {
    e.stopPropagation();
    openPicker();
  });
  slot?.addEventListener("click", (e) => {
    if (e.target.closest("button")) return;
    openPicker();
  });
  input?.addEventListener("change", () => {
    const file = input.files?.[0];
    if (file) void ingestJsonFile(file, kind);
  });

  ["dragenter", "dragover"].forEach((type) => {
    slot?.addEventListener(type, (e) => {
      e.preventDefault();
      slot.classList.add("dragover");
    });
  });
  ["dragleave", "drop"].forEach((type) => {
    slot?.addEventListener(type, (e) => {
      e.preventDefault();
      slot.classList.remove("dragover");
    });
  });
  slot?.addEventListener("drop", (e) => {
    const file = e.dataTransfer?.files?.[0];
    if (file) void ingestJsonFile(file, kind);
  });
}

function wireSegmented(containerId, key, onChange) {
  const container = document.getElementById(containerId);
  container.addEventListener("click", (event) => {
    const btn = event.target.closest("button[data-" + key + "]");
    if (!btn) return;
    container.querySelectorAll("button").forEach((b) => b.classList.remove("active"));
    btn.classList.add("active");
    state[key] = btn.dataset[key];
    onChange();
  });
}

wireUploadSlot({
  slot: els.imuSlot,
  input: els.imuFileInput,
  pickBtn: els.imuPickBtn,
  kind: "imu",
});
wireUploadSlot({
  slot: els.mocapSlot,
  input: els.mocapFileInput,
  pickBtn: els.mocapPickBtn,
  kind: "mocap",
});

wireSegmented("frameToggle", "frame", () => { renderChart(); updateTable(); });
wireSegmented("sensorToggle", "sensor", () => { renderChart(); updateTable(); });
wireSegmented("axisToggle", "axis", () => renderChart());

[els.thresholdVEnd, els.thresholdAccelDev].forEach((input) => {
  input?.addEventListener("input", () => {
    readThresholdInputs();
    if (state.raw) renderCycles(state.raw);
  });
});

els.timeSlider?.addEventListener("input", () => {
  const maxStart = Math.max(0, state.tMaxSec - state.viewWindowSec);
  const pct = Number(els.timeSlider.value) / 100;
  state.viewStartSec = maxStart * pct;
  updateSliderLabels();
  renderChart();
  updateTable();
});

els.resetZoom?.addEventListener("click", () => {
  state.viewStartSec = 0;
  state.viewWindowSec = state.tMaxSec;
  syncSliderFromState();
  renderChart();
  updateTable();
});

els.fitWindow?.addEventListener("click", () => {
  state.viewWindowSec = Math.min(30, state.tMaxSec);
  state.viewStartSec = Math.max(0, state.tMaxSec - state.viewWindowSec);
  syncSliderFromState();
  renderChart();
  updateTable();
});

// Plotly ใช้ on() ไม่ใช่ addEventListener
if (els.chart && typeof els.chart.on === "function") {
  els.chart.on("plotly_relayout", (event) => {
    const x0 = event["xaxis.range[0]"] ?? event["xaxis.range"]?.[0];
    const x1 = event["xaxis.range[1]"] ?? event["xaxis.range"]?.[1];
    if (x0 == null || x1 == null) return;
    state.viewStartSec = Math.max(0, x0);
    state.viewWindowSec = Math.max(1, x1 - x0);
    syncSliderFromState();
    updateTable();
  });
}

// เผื่อมีไฟล์ตัวอย่างวางไว้ข้าง ๆ (เปิดผ่าน dev server เท่านั้น — file:// จะ fetch ไม่ได้)
const defaultFile = "gait-trace-20260715-215837.json";
fetch(defaultFile)
  .then((res) => (res.ok ? res.json() : null))
  .then((data) => { if (data) loadImuPayload(data, defaultFile); })
  .catch(() => {});

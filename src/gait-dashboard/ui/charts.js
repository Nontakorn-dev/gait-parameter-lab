/**
 * Charts Manager - Real-time Chart Rendering
 */

import {
  Chart,
  LineController,
  LineElement,
  PointElement,
  LinearScale,
  CategoryScale,
  Filler,
  Tooltip,
  Legend,
} from 'chart.js';

Chart.register(
  LineController,
  LineElement,
  PointElement,
  LinearScale,
  CategoryScale,
  Filler,
  Tooltip,
  Legend,
);

Chart.defaults.color = '#757575';
Chart.defaults.font.family = "'Inter', sans-serif";
Chart.defaults.font.size = 11;

const CHART_COLORS = {
  left: 'rgba(33, 150, 243, 1)',
  leftDim: 'rgba(33, 150, 243, 0.1)',
  right: 'rgba(239, 83, 80, 1)',
  rightDim: 'rgba(239, 83, 80, 0.08)',
  coral: 'rgba(239, 83, 80, 1)',
  amber: 'rgba(255, 152, 0, 1)',
  emerald: 'rgba(76, 175, 80, 1)',
  emeraldDim: 'rgba(76, 175, 80, 0.1)',
  violet: 'rgba(124, 77, 255, 1)',
  violetDim: 'rgba(124, 77, 255, 0.1)',
  gridColor: 'rgba(0, 0, 0, 0.06)',
};

const commonScaleOptions = {
  grid: {
    color: CHART_COLORS.gridColor,
    drawTicks: false,
  },
  ticks: {
    padding: 8,
    font: { size: 10 },
  },
  border: {
    display: false,
  },
};

const ANGULAR_VELOCITY_SCALE = {
  min: -400,
  max: 400,
  stepSize: 100,
};

const SHANK_ANGLE_SCALE = {
  min: -45,
  max: 45,
  stepSize: 15,
};

function formatTimeTick(value) {
  return Number.isFinite(value) ? value.toFixed(2) : value;
}

function buildSeriesPoints(timestamps = [], values = [], startIdx = 0) {
  const points = [];

  for (let i = startIdx; i < timestamps.length; i += 1) {
    if (!Number.isFinite(timestamps[i]) || !Number.isFinite(values[i])) {
      continue;
    }

    points.push({ x: timestamps[i], y: values[i] });
  }

  return points;
}

function buildVerticalLineData(xValue, yMin, yMax) {
  if (!Number.isFinite(xValue) || !Number.isFinite(yMin) || !Number.isFinite(yMax)) {
    return [];
  }

  return [
    { x: xValue, y: yMin },
    { x: xValue, y: yMax },
  ];
}

function getStableScaleRange(values, baseRange) {
  const numericValues = values.filter(Number.isFinite);

  if (!numericValues.length) {
    return baseRange;
  }

  let min = baseRange.min;
  let max = baseRange.max;

  const dataMin = Math.min(...numericValues);
  const dataMax = Math.max(...numericValues);

  if (dataMin < min) {
    min = Math.floor(dataMin / baseRange.stepSize) * baseRange.stepSize;
  }

  if (dataMax > max) {
    max = Math.ceil(dataMax / baseRange.stepSize) * baseRange.stepSize;
  }

  return {
    min,
    max,
    stepSize: baseRange.stepSize,
  };
}

function getEntryColor(entry) {
  if (entry?.side === 'R') {
    return {
      line: CHART_COLORS.right,
      fill: CHART_COLORS.rightDim,
      angle: CHART_COLORS.emerald,
      angleFill: CHART_COLORS.emeraldDim,
    };
  }

  return {
    line: CHART_COLORS.left,
    fill: CHART_COLORS.leftDim,
    angle: CHART_COLORS.violet,
    angleFill: CHART_COLORS.violetDim,
  };
}

function createGradient(ctx, colorStops) {
  const gradient = ctx.createLinearGradient(0, 0, 0, 220);
  colorStops.forEach(([offset, color]) => {
    gradient.addColorStop(offset, color);
  });
  return gradient;
}

export class ChartsManager {
  constructor() {
    this.angVelChart = null;
    this.angleChart = null;
  }

  init() {
    this._createAngVelChart();
    this._createAngleChart();
  }

  update(entries = []) {
    if (!Array.isArray(entries) || entries.length === 0) {
      this._clearCharts();
      return;
    }

    this._updateAngVelChart(entries);
    this._updateAngleChart(entries);
  }

  _clearCharts() {
    if (this.angVelChart) {
      this.angVelChart.data.datasets.forEach((dataset) => {
        dataset.data = [];
      });
      this.angVelChart.update('none');
    }

    if (this.angleChart) {
      this.angleChart.data.datasets.forEach((dataset) => {
        dataset.data = [];
      });
      this.angleChart.update('none');
    }
  }

  _createAngVelChart() {
    const ctx = document.getElementById('chart-angvel').getContext('2d');
    const leftGradient = createGradient(ctx, [
      [0, CHART_COLORS.leftDim],
      [1, 'rgba(33, 150, 243, 0.01)'],
    ]);
    const rightGradient = createGradient(ctx, [
      [0, CHART_COLORS.rightDim],
      [1, 'rgba(239, 83, 80, 0.01)'],
    ]);

    this.angVelChart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'Left Angular Velocity',
            data: [],
            borderColor: CHART_COLORS.left,
            backgroundColor: leftGradient,
            borderWidth: 1.5,
            pointRadius: 0,
            fill: true,
            tension: 0.3,
          },
          {
            label: 'Right Angular Velocity',
            data: [],
            borderColor: CHART_COLORS.right,
            backgroundColor: rightGradient,
            borderWidth: 1.5,
            pointRadius: 0,
            fill: false,
            tension: 0.3,
          },
          {
            label: 'Heel Strike',
            data: [],
            borderColor: CHART_COLORS.coral,
            backgroundColor: CHART_COLORS.coral,
            pointRadius: 6,
            pointStyle: 'triangle',
            showLine: false,
            pointBorderWidth: 0,
          },
          {
            label: 'Toe Off',
            data: [],
            borderColor: CHART_COLORS.amber,
            backgroundColor: CHART_COLORS.amber,
            pointRadius: 5,
            pointStyle: 'rect',
            showLine: false,
            pointBorderWidth: 0,
          },
          {
            label: 'Mid-Stance',
            data: [],
            borderColor: CHART_COLORS.emerald,
            backgroundColor: CHART_COLORS.emerald,
            pointRadius: 5,
            pointStyle: 'circle',
            showLine: false,
            pointBorderWidth: 0,
          },
          {
            label: 'Integration Start',
            data: [],
            borderColor: 'rgba(25, 118, 210, 0.85)',
            backgroundColor: 'rgba(25, 118, 210, 0.85)',
            borderWidth: 1.25,
            borderDash: [6, 4],
            pointRadius: 0,
            pointHoverRadius: 0,
            fill: false,
          },
          {
            label: 'Integration End',
            data: [],
            borderColor: 'rgba(56, 142, 60, 0.85)',
            backgroundColor: 'rgba(56, 142, 60, 0.85)',
            borderWidth: 1.25,
            borderDash: [6, 4],
            pointRadius: 0,
            pointHoverRadius: 0,
            fill: false,
          },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 300 },
        interaction: { intersect: false, mode: 'index' },
        plugins: {
          legend: {
            position: 'top',
            align: 'end',
            labels: {
              usePointStyle: true,
              pointStyle: 'circle',
              padding: 12,
              font: { size: 10, weight: '600' },
            },
          },
          tooltip: {
            backgroundColor: 'rgba(255, 255, 255, 0.95)',
            titleColor: '#212121',
            bodyColor: '#424242',
            borderColor: '#e0e0e0',
            borderWidth: 1,
            cornerRadius: 8,
            padding: 10,
            displayColors: true,
            callbacks: {
              label: (ctx) => `${ctx.dataset.label}: ${ctx.parsed.y?.toFixed(1)} deg/s`,
            },
          },
        },
        scales: {
          x: {
            type: 'linear',
            ...commonScaleOptions,
            title: { display: true, text: 'Time (s)', font: { size: 10, weight: '600' }, color: '#757575' },
            ticks: {
              ...commonScaleOptions.ticks,
              callback: (value) => formatTimeTick(value),
            },
          },
          y: {
            ...commonScaleOptions,
            title: { display: true, text: 'Angular Velocity (deg/s)', font: { size: 10, weight: '600' }, color: '#757575' },
            min: ANGULAR_VELOCITY_SCALE.min,
            max: ANGULAR_VELOCITY_SCALE.max,
            ticks: {
              ...commonScaleOptions.ticks,
              stepSize: ANGULAR_VELOCITY_SCALE.stepSize,
            },
          },
        },
      },
    });
  }

  _updateAngVelChart(entries) {
    if (!this.angVelChart || !entries.length) {
      return;
    }

    const [primaryEntry, secondaryEntry] = entries;
    const primaryData = primaryEntry?.processedData || { timestamps: [], angularVelocity: [], events: [], integrationWindow: null };
    const secondaryData = secondaryEntry?.processedData || { timestamps: [], angularVelocity: [] };

    if (!primaryData.timestamps.length) {
      return;
    }

    const showSamples = 400;
    const primaryStartIdx = Math.max(0, primaryData.timestamps.length - showSamples);
    const secondaryStartIdx = Math.max(0, secondaryData.timestamps.length - showSamples);
    const primaryPoints = buildSeriesPoints(primaryData.timestamps, primaryData.angularVelocity, primaryStartIdx);
    const secondaryPoints = buildSeriesPoints(secondaryData.timestamps, secondaryData.angularVelocity, secondaryStartIdx);
    const visibleStartTime = Math.min(
      primaryData.timestamps[primaryStartIdx] ?? Infinity,
      secondaryData.timestamps[secondaryStartIdx] ?? Infinity,
    );
    const visibleEndTime = Math.max(
      primaryData.timestamps[primaryData.timestamps.length - 1] ?? 0,
      secondaryData.timestamps[secondaryData.timestamps.length - 1] ?? 0,
    );

    const hsData = [];
    const toData = [];
    const msvData = [];

    for (const event of primaryData.events || []) {
      if (event.index < primaryStartIdx || event.index >= primaryData.timestamps.length) {
        continue;
      }

      const x = Number.isFinite(event.time) ? event.time : primaryData.timestamps[event.index];
      const y = primaryData.angularVelocity[event.index];

      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        continue;
      }

      const point = { x, y };
      if (event.type === 'HS') {
        hsData.push(point);
      } else if (event.type === 'TO') {
        toData.push(point);
      } else if (event.type === 'MSV') {
        msvData.push(point);
      }
    }

    this.angVelChart.data.labels = [];
    this.angVelChart.data.datasets[0].label = primaryEntry?.label ? `${primaryEntry.label} Angular Velocity` : 'Angular Velocity';
    this.angVelChart.data.datasets[0].data = primaryPoints;
    this.angVelChart.data.datasets[0].borderColor = getEntryColor(primaryEntry).line;
    this.angVelChart.data.datasets[1].label = secondaryEntry?.label ? `${secondaryEntry.label} Angular Velocity` : 'Secondary Angular Velocity';
    this.angVelChart.data.datasets[1].data = secondaryPoints;
    this.angVelChart.data.datasets[1].hidden = !secondaryEntry;
    this.angVelChart.data.datasets[2].data = hsData;
    this.angVelChart.data.datasets[3].data = toData;
    this.angVelChart.data.datasets[4].data = msvData;

    const angVelScale = getStableScaleRange([
      ...primaryPoints.map((point) => point.y),
      ...secondaryPoints.map((point) => point.y),
    ], ANGULAR_VELOCITY_SCALE);
    this.angVelChart.options.scales.y.min = angVelScale.min;
    this.angVelChart.options.scales.y.max = angVelScale.max;
    this.angVelChart.options.scales.x.min = Number.isFinite(visibleStartTime) ? visibleStartTime : null;
    this.angVelChart.options.scales.x.max = visibleEndTime;

    const integrationWindow = primaryData.integrationWindow ?? null;
    this.angVelChart.data.datasets[5].data = buildVerticalLineData(
      integrationWindow?.startTime,
      angVelScale.min,
      angVelScale.max,
    );
    this.angVelChart.data.datasets[6].data = buildVerticalLineData(
      integrationWindow?.endTime,
      angVelScale.min,
      angVelScale.max,
    );

    this.angVelChart.update('none');
  }

  _createAngleChart() {
    const ctx = document.getElementById('chart-angle').getContext('2d');
    const leftGradient = createGradient(ctx, [
      [0, CHART_COLORS.violetDim],
      [1, 'rgba(167, 139, 250, 0.01)'],
    ]);
    const rightGradient = createGradient(ctx, [
      [0, CHART_COLORS.emeraldDim],
      [1, 'rgba(76, 175, 80, 0.01)'],
    ]);

    this.angleChart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'Left Shank Angle',
            data: [],
            borderColor: CHART_COLORS.violet,
            backgroundColor: leftGradient,
            borderWidth: 1.5,
            pointRadius: 0,
            fill: true,
            tension: 0.3,
          },
          {
            label: 'Right Shank Angle',
            data: [],
            borderColor: CHART_COLORS.emerald,
            backgroundColor: rightGradient,
            borderWidth: 1.5,
            pointRadius: 0,
            fill: false,
            tension: 0.3,
          },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 300 },
        interaction: { intersect: false, mode: 'index' },
        plugins: {
          legend: {
            position: 'top',
            align: 'end',
            labels: {
              usePointStyle: true,
              pointStyle: 'circle',
              padding: 12,
              font: { size: 10, weight: '600' },
            },
          },
          tooltip: {
            backgroundColor: 'rgba(255, 255, 255, 0.95)',
            titleColor: '#212121',
            bodyColor: '#424242',
            borderColor: '#e0e0e0',
            borderWidth: 1,
            cornerRadius: 8,
            padding: 10,
            callbacks: {
              label: (ctx) => `${ctx.dataset.label}: ${ctx.parsed.y?.toFixed(1)} deg`,
            },
          },
        },
        scales: {
          x: {
            type: 'linear',
            ...commonScaleOptions,
            title: { display: true, text: 'Time (s)', font: { size: 10, weight: '600' }, color: '#757575' },
            ticks: {
              ...commonScaleOptions.ticks,
              callback: (value) => formatTimeTick(value),
            },
          },
          y: {
            ...commonScaleOptions,
            title: { display: true, text: 'Angle (deg)', font: { size: 10, weight: '600' }, color: '#757575' },
            min: SHANK_ANGLE_SCALE.min,
            max: SHANK_ANGLE_SCALE.max,
            ticks: {
              ...commonScaleOptions.ticks,
              stepSize: SHANK_ANGLE_SCALE.stepSize,
            },
          },
        },
      },
    });
  }

  _updateAngleChart(entries) {
    if (!this.angleChart || !entries.length) {
      return;
    }

    const [primaryEntry, secondaryEntry] = entries;
    const primaryData = primaryEntry?.processedData || { timestamps: [], shankAngle: [] };
    const secondaryData = secondaryEntry?.processedData || { timestamps: [], shankAngle: [] };

    if (!primaryData.timestamps.length) {
      return;
    }

    const showSamples = 400;
    const primaryStartIdx = Math.max(0, primaryData.timestamps.length - showSamples);
    const secondaryStartIdx = Math.max(0, secondaryData.timestamps.length - showSamples);
    const primaryPoints = buildSeriesPoints(primaryData.timestamps, primaryData.shankAngle, primaryStartIdx);
    const secondaryPoints = buildSeriesPoints(secondaryData.timestamps, secondaryData.shankAngle, secondaryStartIdx);
    const visibleStartTime = Math.min(
      primaryData.timestamps[primaryStartIdx] ?? Infinity,
      secondaryData.timestamps[secondaryStartIdx] ?? Infinity,
    );
    const visibleEndTime = Math.max(
      primaryData.timestamps[primaryData.timestamps.length - 1] ?? 0,
      secondaryData.timestamps[secondaryData.timestamps.length - 1] ?? 0,
    );

    this.angleChart.data.labels = [];
    this.angleChart.data.datasets[0].label = primaryEntry?.label ? `${primaryEntry.label} Angle` : 'Shank Angle';
    this.angleChart.data.datasets[0].data = primaryPoints;
    this.angleChart.data.datasets[0].borderColor = getEntryColor(primaryEntry).angle;
    this.angleChart.data.datasets[1].label = secondaryEntry?.label ? `${secondaryEntry.label} Angle` : 'Secondary Angle';
    this.angleChart.data.datasets[1].data = secondaryPoints;
    this.angleChart.data.datasets[1].hidden = !secondaryEntry;

    const angleScale = getStableScaleRange([
      ...primaryPoints.map((point) => point.y),
      ...secondaryPoints.map((point) => point.y),
    ], SHANK_ANGLE_SCALE);
    this.angleChart.options.scales.y.min = angleScale.min;
    this.angleChart.options.scales.y.max = angleScale.max;
    this.angleChart.options.scales.x.min = Number.isFinite(visibleStartTime) ? visibleStartTime : null;
    this.angleChart.options.scales.x.max = visibleEndTime;

    this.angleChart.update('none');
  }

  destroy() {
    this.angVelChart?.destroy();
    this.angleChart?.destroy();
  }
}
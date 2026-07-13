function averageValues(values) {
  const numericValues = values.filter(Number.isFinite);
  if (!numericValues.length) {
    return null;
  }

  return numericValues.reduce((total, value) => total + value, 0) / numericValues.length;
}

function normalizeParamEntries(entries = []) {
  return entries.map((entry) => (entry?.params ? entry : { params: entry }));
}

export function aggregateGaitParams(entries = []) {
  const normalizedEntries = normalizeParamEntries(entries);
  if (!normalizedEntries.length) {
    return null;
  }

  const averaged = {
    stepCount: 0,
    strideCount: 0,
    sessionDuration: 0,
    cadence: 0,
    stepTime: null,
    strideTime: 0,
    doubleSupport: null,
    stepLength: 0,
    strideLength: 0,
    walkingSpeed: 0,
    clearance: 0,
    peakShankAngle: 0,
    stancePct: 0,
    swingPct: 0,
  };

  const numericFields = [
    'sessionDuration',
    'cadence',
    'strideTime',
    'stepLength',
    'strideLength',
    'walkingSpeed',
    'clearance',
    'peakShankAngle',
    'stancePct',
    'swingPct',
  ];

  numericFields.forEach((field) => {
    const values = normalizedEntries
      .map((entry) => entry.params?.[field])
      .filter(Number.isFinite);
    const unresolvedIsMeaningful = field === 'stancePct' || field === 'swingPct';
    averaged[field] = values.length
      ? values.reduce((sum, value) => sum + value, 0) / values.length
      : (unresolvedIsMeaningful ? null : 0);
  });

  averaged.stepCount = Math.max(...normalizedEntries.map((entry) => entry.params?.stepCount || 0));
  averaged.strideCount = Math.max(...normalizedEntries.map((entry) => entry.params?.strideCount || 0));

  const stepTimeValues = normalizedEntries
    .map((entry) => entry.params?.stepTime)
    .filter(Number.isFinite);
  averaged.stepTime = stepTimeValues.length
    ? stepTimeValues.reduce((sum, value) => sum + value, 0) / stepTimeValues.length
    : null;

  const doubleSupportValues = normalizedEntries
    .map((entry) => entry.params?.doubleSupport)
    .filter(Number.isFinite);
  averaged.doubleSupport = doubleSupportValues.length
    ? doubleSupportValues.reduce((sum, value) => sum + value, 0) / doubleSupportValues.length
    : null;

  return averaged;
}

export function collectGaitMetricsRowsFromAnalyzers(analyzers = []) {
  return analyzers.flatMap((analyzer) => analyzer?.getGaitMetricsRows?.() || []);
}

export function buildSessionSummaryFromAnalyzers(analyzers = []) {
  const summaries = analyzers
    .map((analyzer) => analyzer?.getSummary?.())
    .filter(Boolean);
  const allRows = collectGaitMetricsRowsFromAnalyzers(analyzers);

  return {
    total_steps: summaries.length
      ? Math.max(...summaries.map((summary) => summary.totalSteps || 0))
      : 0,
    cadence_spm: averageValues(summaries.map((summary) => summary.cadenceSpm)) ?? 0,
    avg_step_length_m: averageValues(allRows.map((row) => row.step_length_m)) ?? 0,
    avg_stride_length_m: averageValues(allRows.map((row) => row.stride_length_m)) ?? 0,
    avg_clearance_m: averageValues(allRows.map((row) => row.max_clearance_m)) ?? 0,
    avg_speed_mps: averageValues(allRows.map((row) => row.speed_mps)) ?? 0,
  };
}

export function buildLiveSessionMetricsFromParams(aggregatedParams) {
  if (!aggregatedParams) {
    return {};
  }

  return {
    total_steps: aggregatedParams.stepCount || 0,
    cadence_spm: Number.isFinite(aggregatedParams.cadence) ? aggregatedParams.cadence : 0,
    avg_step_length_m: Number.isFinite(aggregatedParams.stepLength) ? aggregatedParams.stepLength : null,
    avg_stride_length_m: Number.isFinite(aggregatedParams.strideLength) ? aggregatedParams.strideLength : null,
    avg_clearance_m: Number.isFinite(aggregatedParams.clearance) ? aggregatedParams.clearance : null,
    avg_speed_mps: Number.isFinite(aggregatedParams.walkingSpeed) ? aggregatedParams.walkingSpeed : null,
  };
}

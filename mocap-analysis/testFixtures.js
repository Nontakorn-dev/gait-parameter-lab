// สร้างชุดข้อมูล marker สังเคราะห์ที่มี "ground truth" รู้ค่าแน่นอน (stride length,
// stance%, cadence, ankle clearance) — ใช้ร่วมกันระหว่าง test หลายไฟล์ใน mocap-analysis/
//
// รูปคลื่นความเร็วเชิงมุม (gx shape) จงใจ "มิเรอร์" ฟังก์ชัน generateShankAngularVelocity
// ใน src/gait-dashboard/data/demoDataGenerator.js (peak=350, hsDip=-180, stancePct=0.62)
// เพื่อให้ค่ามุมหน้าแข้งที่ derive ออกมาดูสมจริง (ไม่ได้มีผลต่อ event detection ในไฟล์นี้
// อีกต่อไปเพราะเปลี่ยนไปใช้ foot-velocity algorithm แล้ว แต่ยังใช้ทดสอบ peakShankAngleDeg)

function gxShape(phase, stancePct, peak, hsDip) {
  if (phase < 0.05) {
    const p = phase / 0.05;
    return hsDip * Math.exp(-p * 3) * Math.cos(p * Math.PI);
  }
  if (phase < 0.15) {
    const p = (phase - 0.05) / 0.10;
    return -20 + 60 * p;
  }
  if (phase < stancePct * 0.6) {
    const p = (phase - 0.15) / (stancePct * 0.6 - 0.15);
    return 40 + 20 * Math.sin(p * Math.PI);
  }
  if (phase < stancePct) {
    const p = (phase - stancePct * 0.6) / (stancePct - stancePct * 0.6);
    let v = 60 - 80 * Math.sin(p * Math.PI * 0.7);
    if (p > 0.3 && p < 0.7) v -= 40 * Math.exp(-1 * ((p - 0.5) / 0.1) ** 2);
    return v;
  }
  const swingPhase = (phase - stancePct) / (1.0 - stancePct);
  let v = peak * Math.sin(swingPhase * Math.PI);
  if (swingPhase > 0.75) {
    const termPhase = (swingPhase - 0.75) / 0.25;
    v -= (peak + Math.abs(hsDip)) * termPhase * termPhase;
  }
  return v;
}

export const FIXTURE = {
  SAMPLE_RATE: 120,
  STANCE_PCT: 0.62, // ground truth: ankle นิ่ง (v=0 เป๊ะ) ช่วง phase < 0.62 ของทุก stride
  STRIDE_TIME_S: 1.1, // -> cadence เดี่ยว ๆ ที่คาด = 2/1.1*60 = 109.09 spm
  STRIDE_LENGTH_M: 1.3,
  SHANK_LENGTH_M: 0.42,
  SWING_PEAK_HEIGHT_M: 0.05, // ground truth ankle clearance
  NUM_STRIDES: 8,
};
FIXTURE.WALK_SPEED_MPS = FIXTURE.STRIDE_LENGTH_M / FIXTURE.STRIDE_TIME_S;
FIXTURE.EXPECTED_CADENCE_SPM = (2 / FIXTURE.STRIDE_TIME_S) * 60;
FIXTURE.EXPECTED_STANCE_TIME_S = FIXTURE.STANCE_PCT * FIXTURE.STRIDE_TIME_S;

// per-stride velocity samples ถูก detrend (ลบ mean) ก่อน integrate เป็นมุม เพื่อไม่ให้มุม
// ไหลสะสมข้าม stride (ฟังก์ชันต้นทางออกแบบมาให้ signal สมจริงสำหรับ demo เท่านั้น ไม่ได้
// ออกแบบให้ผลรวมต่อรอบเป็นศูนย์พอดี — เดินจริงมุมหน้าแข้งกลับที่เดิมทุก heel-strike)
function buildDetrendedVelocityTemplate() {
  const samplesPerStride = Math.round(FIXTURE.STRIDE_TIME_S * FIXTURE.SAMPLE_RATE);
  const raw = [];
  for (let i = 0; i < samplesPerStride; i += 1) {
    raw.push(gxShape(i / samplesPerStride, FIXTURE.STANCE_PCT, 350, -180));
  }
  const meanV = raw.reduce((a, b) => a + b, 0) / raw.length;
  return raw.map((v) => v - meanV);
}

function buildLegTrajectory(phaseOffsetS, footprintStartX) {
  const velocityTemplate = buildDetrendedVelocityTemplate();
  const samplesPerStride = velocityTemplate.length;
  const totalDurationS = FIXTURE.NUM_STRIDES * FIXTURE.STRIDE_TIME_S + 1;
  const n = Math.round(totalDurationS * FIXTURE.SAMPLE_RATE);

  const traj = [];
  let angleDeg = 0;
  for (let i = 0; i < n; i += 1) {
    const t = i / FIXTURE.SAMPLE_RATE;
    const cyclePos = (t - phaseOffsetS) / FIXTURE.STRIDE_TIME_S;
    const cycleIndex = Math.floor(cyclePos);
    const phase = cyclePos - cycleIndex;
    const sampleIdx = ((Math.round(phase * samplesPerStride) % samplesPerStride) + samplesPerStride) % samplesPerStride;

    if (i > 0) angleDeg += velocityTemplate[sampleIdx] / FIXTURE.SAMPLE_RATE;

    const footprintX = footprintStartX + cycleIndex * FIXTURE.STRIDE_LENGTH_M;
    let ankleX;
    let ankleY;
    if (phase < FIXTURE.STANCE_PCT) {
      ankleX = footprintX;
      ankleY = 0;
    } else {
      const p = (phase - FIXTURE.STANCE_PCT) / (1 - FIXTURE.STANCE_PCT);
      const ease = (1 - Math.cos(p * Math.PI)) / 2;
      ankleX = footprintX + FIXTURE.STRIDE_LENGTH_M * ease;
      ankleY = FIXTURE.SWING_PEAK_HEIGHT_M * Math.sin(p * Math.PI);
    }

    const rad = (angleDeg * Math.PI) / 180;
    const kneeX = ankleX + FIXTURE.SHANK_LENGTH_M * Math.sin(rad);
    const kneeY = ankleY + FIXTURE.SHANK_LENGTH_M * Math.cos(rad);

    traj.push({ t, ankleX, ankleY, ankleZ: 0, kneeX, kneeY, kneeZ: 0, angleDeg });
  }
  return traj;
}

export function buildSyntheticCsv() {
  const left = buildLegTrajectory(0, 0);
  const right = buildLegTrajectory(FIXTURE.STRIDE_TIME_S / 2, FIXTURE.STRIDE_LENGTH_M / 2);
  const n = left.length;

  const markerDefs = [
    { id: 1, name: 'L_ASIS' },
    { id: 2, name: 'R_ASIS' },
    { id: 3, name: 'L_Knee' },
    { id: 4, name: 'R_Knee' },
    { id: 5, name: 'L_Ankle' },
    { id: 6, name: 'R_Ankle' },
  ];

  const lines = ['righthanded'];
  for (let i = 0; i < n; i += 1) {
    const t = i / FIXTURE.SAMPLE_RATE;
    const asisX = FIXTURE.WALK_SPEED_MPS * t;
    const values = {
      1: [asisX, 0.95, -0.1],
      2: [asisX, 0.95, 0.1],
      3: [left[i].kneeX, left[i].kneeY, -0.1],
      4: [right[i].kneeX, right[i].kneeY, 0.1],
      5: [left[i].ankleX, left[i].ankleY, -0.1],
      6: [right[i].ankleX, right[i].ankleY, 0.1],
    };
    const markerFields = markerDefs
      .map((m) => `${values[m.id][0].toFixed(6)},${values[m.id][1].toFixed(6)},${values[m.id][2].toFixed(6)},${m.id},${m.name}`)
      .join(',');
    lines.push(`frame,${i},${t.toFixed(8)},0,${markerDefs.length},${markerFields}`);
  }
  return { csv: lines.join('\n'), left, right };
}

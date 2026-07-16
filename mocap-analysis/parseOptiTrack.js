// Parser สำหรับ OptiTrack "Motive CSV export (row-typed)" format — รูปแบบเดียวกับที่เจอใน
// mocap.csv ตัวอย่าง (comment/info/righthanded/frame แถวต่อแถว, marker เป็น x,y,z,id,name)
// อ้างอิงจาก comment header ในไฟล์ตัวอย่างเอง ไม่ได้เดา:
//   frame: idx, time, rigidBodyCount, [rigidBody: id,x,y,z,qx,qy,qz,qw,yaw,pitch,roll]*,
//          markerCount, [marker: x,y,z,id,name]*

function parseFrameLine(parts) {
  const idx = Number(parts[1]);
  const time = Number(parts[2]);
  const rigidBodyCount = Number(parts[3]);
  let p = 4;

  const rigidBodies = [];
  for (let r = 0; r < rigidBodyCount; r += 1) {
    const id = Number(parts[p]);
    const x = Number(parts[p + 1]);
    const y = Number(parts[p + 2]);
    const z = Number(parts[p + 3]);
    const qx = Number(parts[p + 4]);
    const qy = Number(parts[p + 5]);
    const qz = Number(parts[p + 6]);
    const qw = Number(parts[p + 7]);
    const yaw = Number(parts[p + 8]);
    const pitch = Number(parts[p + 9]);
    const roll = Number(parts[p + 10]);
    rigidBodies.push({ id, x, y, z, qx, qy, qz, qw, yaw, pitch, roll });
    p += 11;
  }

  const markerCount = Number(parts[p]);
  p += 1;
  const markers = [];
  for (let m = 0; m < markerCount; m += 1) {
    const x = Number(parts[p]);
    const y = Number(parts[p + 1]);
    const z = Number(parts[p + 2]);
    const id = Number(parts[p + 3]);
    const name = parts[p + 4];
    markers.push({ id, name, x, y, z });
    p += 5;
  }

  return { idx, time, rigidBodies, markers };
}

// dt ต่อเฟรมควรมาจาก timestamp จริงเสมอ (ทนต่อเฟรมหาย) — ไม่ hardcode frame rate ตายตัว
// เหมือนหลักการเดียวกับที่ใช้ทั่ว gaitProcessor.js/traceRecorder.js ในโปรเจกต์นี้
function estimateFrameRateHz(frames) {
  if (frames.length < 2) return null;
  const deltas = [];
  for (let i = 1; i < frames.length; i += 1) {
    const d = frames[i].time - frames[i - 1].time;
    if (d > 0) deltas.push(d);
  }
  if (!deltas.length) return null;
  const sorted = [...deltas].sort((a, b) => a - b);
  const median = sorted[Math.floor(sorted.length / 2)];
  return median > 0 ? 1 / median : null;
}

export function parseOptiTrackCsv(text) {
  const lines = text.split(/\r?\n/);
  let handedness = null;
  const frames = [];

  for (const line of lines) {
    if (!line) continue;
    if (line.startsWith('righthanded')) { handedness = 'right'; continue; }
    if (line.startsWith('lefthanded')) { handedness = 'left'; continue; }
    if (!line.startsWith('frame,')) continue;
    frames.push(parseFrameLine(line.split(',')));
  }

  return {
    handedness,
    frameRateHz: estimateFrameRateHz(frames),
    frameCount: frames.length,
    durationS: frames.length ? frames[frames.length - 1].time - frames[0].time : 0,
    frames,
  };
}

// รวมชื่อ marker ทั้งหมดที่เคยปรากฏ (บาง marker อาจถูกบดบัง/reconstruct ไม่ได้บางเฟรม
// จึงต้องสแกนทุกเฟรม ไม่ใช่แค่เฟรมแรก) — คืน [{id, name, seenCount}]
export function listMarkers(parsed) {
  const seen = new Map();
  for (const frame of parsed.frames) {
    for (const marker of frame.markers) {
      const key = marker.id;
      if (!seen.has(key)) {
        seen.set(key, { id: marker.id, name: marker.name, seenCount: 0 });
      }
      seen.get(key).seenCount += 1;
    }
  }
  return [...seen.values()].sort((a, b) => a.id - b.id);
}

// ดึง time series ของ marker ตัวเดียวจาก id — ใส่ null ในเฟรมที่ marker หาย (ถูกบดบัง)
// แทนการข้ามเฟรมไปเฉย ๆ เพื่อให้ index ตรงกับ parsed.frames เสมอ (จำเป็นสำหรับ derivative)
export function extractMarkerSeries(parsed, markerId) {
  const t = new Array(parsed.frames.length);
  const x = new Array(parsed.frames.length);
  const y = new Array(parsed.frames.length);
  const z = new Array(parsed.frames.length);

  for (let i = 0; i < parsed.frames.length; i += 1) {
    const frame = parsed.frames[i];
    t[i] = frame.time;
    const marker = frame.markers.find((m) => m.id === markerId);
    if (marker && Number.isFinite(marker.x)) {
      x[i] = marker.x;
      y[i] = marker.y;
      z[i] = marker.z;
    } else {
      x[i] = null;
      y[i] = null;
      z[i] = null;
    }
  }

  return { t, x, y, z };
}

// เติมช่องว่างสั้น ๆ (marker หายไม่กี่เฟรมจากการบดบังชั่วคราว) ด้วย linear interpolation
// ช่องว่างที่ยาวเกิน maxGapFrames จะถูกปล่อยเป็น null ต่อไป + รายงานเป็น gap ให้ผู้ใช้ตรวจสอบ
// (ไม่เติมทับเงียบ ๆ เพราะถ้าโป่ง gap คร่อมจังหวะ heel-strike จะปั้นความเร็วเชิงมุมปลอมขึ้นมา)
export function interpolateGaps(series, maxGapFrames = 10) {
  const { t, x, y, z } = series;
  const n = t.length;
  const filled = { t, x: [...x], y: [...y], z: [...z] };
  const remainingGaps = [];

  let gapStart = null;
  for (let i = 0; i < n; i += 1) {
    const missing = x[i] === null;
    if (missing && gapStart === null) {
      gapStart = i;
    }
    if (!missing && gapStart !== null) {
      const gapEnd = i - 1;
      const gapLen = gapEnd - gapStart + 1;
      const prevIdx = gapStart - 1;
      if (prevIdx >= 0 && gapLen <= maxGapFrames) {
        for (const axis of ['x', 'y', 'z']) {
          const v0 = filled[axis][prevIdx];
          const v1 = filled[axis][i];
          for (let j = gapStart; j <= gapEnd; j += 1) {
            const frac = (j - prevIdx) / (i - prevIdx);
            filled[axis][j] = v0 + (v1 - v0) * frac;
          }
        }
      } else {
        remainingGaps.push({ startIdx: gapStart, endIdx: gapEnd, startTimeS: t[gapStart], endTimeS: t[gapEnd] });
      }
      gapStart = null;
    }
  }
  if (gapStart !== null) {
    remainingGaps.push({ startIdx: gapStart, endIdx: n - 1, startTimeS: t[gapStart], endTimeS: t[n - 1] });
  }

  return { series: filled, remainingGaps };
}

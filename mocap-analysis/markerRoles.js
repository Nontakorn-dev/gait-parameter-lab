// จับคู่ marker id -> บทบาททางกายวิภาค (6 จุดที่ lab แนะนำ: ASIS/Knee(Lateral Femoral
// Epicondyle)/Ankle(Lateral Malleolus) x ซ้าย-ขวา) จากชื่อ marker ที่ตั้งใน Motive
//
// ตั้งใจ "ไม่เดา" จากรูปแบบการเคลื่อนไหวเหมือนที่ทำกับไฟล์ตัวอย่างก่อนหน้า (เสี่ยงผิดสูง
// และช้า) — ครั้งนี้บังคับ resolve จากชื่อ marker เท่านั้น ถ้า auto-match ไม่ได้ ให้ fail
// ดัง ๆ พร้อมลิสต์ชื่อ/ไอดีที่เจอจริง แล้วให้ผู้ใช้ระบุ override map เอง ดีกว่าเดาแล้วผิดเงียบ ๆ

export const ROLES = ['L_ASIS', 'R_ASIS', 'L_Knee', 'R_Knee', 'L_Ankle', 'R_Ankle'];

// รูปแบบชื่อที่พบได้บ่อยต่อบทบาท (normalize เป็นตัวพิมพ์ใหญ่ ตัดอักขระที่ไม่ใช่ตัวอักษร/ตัวเลขออกก่อนเทียบ)
const ROLE_PATTERNS = {
  L_ASIS: [/^L.*ASIS?$/, /^LEFT.*ASIS?$/, /^LASI$/],
  R_ASIS: [/^R.*ASIS?$/, /^RIGHT.*ASIS?$/, /^RASI$/],
  L_Knee: [/^L.*KNEE$/, /^LEFT.*KNEE$/, /^LKNE$/, /^L.*LFE$/, /^L.*LATFEMEPI/, /^L.*FEMEPICONDYLE/],
  R_Knee: [/^R.*KNEE$/, /^RIGHT.*KNEE$/, /^RKNE$/, /^R.*LFE$/, /^R.*LATFEMEPI/, /^R.*FEMEPICONDYLE/],
  L_Ankle: [/^L.*ANKLE$/, /^LEFT.*ANKLE$/, /^LANK$/, /^L.*MALL/, /^L.*LATMALL/],
  R_Ankle: [/^R.*ANKLE$/, /^RIGHT.*ANKLE$/, /^RANK$/, /^R.*MALL/, /^R.*LATMALL/],
};

function normalizeName(name) {
  return String(name ?? '').toUpperCase().replace(/[^A-Z0-9]/g, '');
}

// markers: [{id, name}] จาก listMarkers(parsed)
// overrideMap: { roleName: markerId } — ใส่เองถ้า auto-match ไม่เจอหรือเจอผิด
export function resolveMarkerRoles(markers, overrideMap = {}) {
  const resolved = {};
  const unresolved = [];

  for (const role of ROLES) {
    if (Number.isFinite(overrideMap[role])) {
      resolved[role] = overrideMap[role];
      continue;
    }

    const patterns = ROLE_PATTERNS[role];
    const matches = markers.filter((m) => {
      const normalized = normalizeName(m.name);
      return patterns.some((re) => re.test(normalized));
    });

    if (matches.length === 1) {
      resolved[role] = matches[0].id;
    } else {
      unresolved.push({ role, matchCount: matches.length, candidates: matches.map((m) => ({ id: m.id, name: m.name })) });
    }
  }

  return { resolved, unresolved, ok: unresolved.length === 0 };
}

export function formatUnresolvedError(unresolved, allMarkers) {
  const lines = [
    `แก้ marker role ให้ครบไม่ได้ (${unresolved.length}/${ROLES.length} จุดยังไม่ชัดเจน):`,
    ...unresolved.map((u) => {
      if (u.matchCount === 0) return `  - ${u.role}: หาชื่อที่ match ไม่เจอเลย`;
      return `  - ${u.role}: match ${u.matchCount} ชื่อ (กำกวม) -> ${u.candidates.map((c) => `#${c.id} "${c.name}"`).join(', ')}`;
    }),
    '',
    'marker ทั้งหมดที่เจอในไฟล์:',
    ...allMarkers.map((m) => `  #${m.id} "${m.name}" (พบ ${m.seenCount} เฟรม)`),
    '',
    'แก้โดยส่ง overrideMap เข้า resolveMarkerRoles เช่น { L_ASIS: 2, R_ASIS: 5, ... } (ดู --map ใน run.js)',
  ];
  return lines.join('\n');
}

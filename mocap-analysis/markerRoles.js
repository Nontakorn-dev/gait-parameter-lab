// จับคู่ marker id -> บทบาททางกายวิภาค
//
// บทบาทหลัก (ต้องมี): ASIS / Knee × ซ้าย-ขวา + ankle สำหรับ HS อย่างน้อยหนึ่งชื่อต่อข้าง
//   L_Ankle / R_Ankle          — legacy (ใช้ทั้ง HS และมุม ถ้าไม่มี For* แยก)
//   L_AnkleForHS / R_AnkleForHS — ส้นเท้า/จุดที่ใช้ foot-velocity HS (heel ได้)
//   L_AnkleForAngle / R_AnkleForAngle — lateral malleolus สำหรับ shank angle / ω sync
//
// Heel ≠ malleolus: knee→heel ไม่ใช่ shank — ใช้หา HS ได้ แต่ signed ω มัก polarity-mismatch
// กับ IMU gx; trial ถัดไปควรติด malleolus แล้ว map เป็น *ForAngle

export const CORE_ROLES = ['L_ASIS', 'R_ASIS', 'L_Knee', 'R_Knee'];
/** ชื่อที่ยัง export เพื่อเทสต์/เอกสารเดิม — ไม่บังคับทุกตัวถ้ามี ForHS */
export const ROLES = [
  ...CORE_ROLES,
  'L_Ankle', 'R_Ankle',
  'L_AnkleForHS', 'R_AnkleForHS',
  'L_AnkleForAngle', 'R_AnkleForAngle',
];

const ROLE_PATTERNS = {
  L_ASIS: [/^L.*ASIS?$/, /^LEFT.*ASIS?$/, /^LASI$/],
  R_ASIS: [/^R.*ASIS?$/, /^RIGHT.*ASIS?$/, /^RASI$/],
  L_Knee: [/^L.*KNEE$/, /^LEFT.*KNEE$/, /^LKNE$/, /^L.*LFE$/, /^L.*LATFEMEPI/, /^L.*FEMEPICONDYLE/],
  R_Knee: [/^R.*KNEE$/, /^RIGHT.*KNEE$/, /^RKNE$/, /^R.*LFE$/, /^R.*LATFEMEPI/, /^R.*FEMEPICONDYLE/],
  // malleolus / ankle จริง — ไม่รวม heel
  L_Ankle: [/^L.*ANKLE$/, /^LEFT.*ANKLE$/, /^LANK$/, /^L.*MALL/, /^L.*LATMALL/],
  R_Ankle: [/^R.*ANKLE$/, /^RIGHT.*ANKLE$/, /^RANK$/, /^R.*MALL/, /^R.*LATMALL/],
  L_AnkleForHS: [
    /^L.*HEEL/, /^LEFT.*HEEL/, /^LHEE$/, /^L.*CALC/,
    /^L.*ANKLE$/, /^LEFT.*ANKLE$/, /^LANK$/, /^L.*MALL/, /^L.*LATMALL/,
  ],
  R_AnkleForHS: [
    /^R.*HEEL/, /^RIGHT.*HEEL/, /^RHEE$/, /^R.*CALC/,
    /^R.*ANKLE$/, /^RIGHT.*ANKLE$/, /^RANK$/, /^R.*MALL/, /^R.*LATMALL/,
  ],
  L_AnkleForAngle: [/^L.*ANKLE$/, /^LEFT.*ANKLE$/, /^LANK$/, /^L.*MALL/, /^L.*LATMALL/],
  R_AnkleForAngle: [/^R.*ANKLE$/, /^RIGHT.*ANKLE$/, /^RANK$/, /^R.*MALL/, /^R.*LATMALL/],
};

function normalizeName(name) {
  return String(name ?? '').toUpperCase().replace(/[^A-Z0-9]/g, '');
}

function matchRole(markers, role) {
  const patterns = ROLE_PATTERNS[role] || [];
  return markers.filter((m) => {
    const normalized = normalizeName(m.name);
    return patterns.some((re) => re.test(normalized));
  });
}

/**
 * เลือก series ต่อข้าง: HS (heel ได้) vs angle (ต้อง malleolus)
 * @returns {{ hsRole: string, angleRole: string|null, angleSource: 'malleolus'|'legacy-ankle'|'unavailable' }}
 */
export function pickAnkleRolePlan(resolved) {
  const plan = {};
  for (const side of ['L', 'R']) {
    const forHs = resolved[`${side}_AnkleForHS`];
    const forAngle = resolved[`${side}_AnkleForAngle`];
    const legacy = resolved[`${side}_Ankle`];
    const hasSplit = Number.isFinite(forHs) || Number.isFinite(forAngle);

    if (hasSplit) {
      const hsRole = Number.isFinite(forHs)
        ? `${side}_AnkleForHS`
        : (Number.isFinite(legacy) ? `${side}_Ankle` : null);
      const angleRole = Number.isFinite(forAngle) ? `${side}_AnkleForAngle` : null;
      plan[side] = {
        hsRole,
        angleRole,
        angleSource: angleRole ? 'malleolus' : 'unavailable',
      };
    } else {
      plan[side] = {
        hsRole: Number.isFinite(legacy) ? `${side}_Ankle` : null,
        angleRole: Number.isFinite(legacy) ? `${side}_Ankle` : null,
        angleSource: Number.isFinite(legacy) ? 'legacy-ankle' : 'unavailable',
      };
    }
  }
  return plan;
}

// markers: [{id, name}] จาก listMarkers(parsed)
// overrideMap: { roleName: markerId } — ใส่เองถ้า auto-match ไม่เจอหรือเจอผิด
export function resolveMarkerRoles(markers, overrideMap = {}) {
  const resolved = {};
  const unresolved = [];
  const cleanOverride = Object.fromEntries(
    Object.entries(overrideMap).filter(([k]) => !k.startsWith('_') && Number.isFinite(overrideMap[k])),
  );

  for (const role of CORE_ROLES) {
    if (Number.isFinite(cleanOverride[role])) {
      resolved[role] = cleanOverride[role];
      continue;
    }
    const matches = matchRole(markers, role);
    if (matches.length === 1) {
      resolved[role] = matches[0].id;
    } else {
      unresolved.push({
        role,
        matchCount: matches.length,
        candidates: matches.map((c) => ({ id: c.id, name: c.name })),
      });
    }
  }

  for (const side of ['L', 'R']) {
    const forHs = `${side}_AnkleForHS`;
    const forAngle = `${side}_AnkleForAngle`;
    const legacy = `${side}_Ankle`;

    for (const role of [forHs, forAngle, legacy]) {
      if (Number.isFinite(cleanOverride[role])) {
        resolved[role] = cleanOverride[role];
      }
    }

    // auto-match optional / legacy ถ้ายังไม่มีจาก override
    for (const role of [forHs, forAngle, legacy]) {
      if (Number.isFinite(resolved[role])) continue;
      // อย่า auto-map heel ชื่อ Marker-N — ต้อง override
      const matches = matchRole(markers, role);
      if (matches.length === 1) resolved[role] = matches[0].id;
    }

    const plan = pickAnkleRolePlan(resolved)[side];
    if (!plan.hsRole) {
      unresolved.push({
        role: `${side}_Ankle|${side}_AnkleForHS`,
        matchCount: 0,
        candidates: [],
        detail: 'ต้องมี ankle สำหรับ HS (legacy Ankle หรือ AnkleForHS)',
      });
    }

    // เติม legacy alias ให้โค้ดเก่าที่อ่าน L_Ankle
    if (!Number.isFinite(resolved[legacy]) && Number.isFinite(resolved[forHs])) {
      resolved[legacy] = resolved[forHs];
    }
  }

  return {
    resolved,
    unresolved,
    ok: unresolved.length === 0,
    anklePlan: pickAnkleRolePlan(resolved),
  };
}

export function formatUnresolvedError(unresolved, allMarkers) {
  const lines = [
    `แก้ marker role ให้ครบไม่ได้ (${unresolved.length} จุดยังไม่ชัดเจน):`,
    ...unresolved.map((u) => {
      if (u.detail) return `  - ${u.role}: ${u.detail}`;
      if (u.matchCount === 0) return `  - ${u.role}: หาชื่อที่ match ไม่เจอเลย`;
      return `  - ${u.role}: match ${u.matchCount} ชื่อ (กำกวม) -> ${u.candidates.map((c) => `#${c.id} "${c.name}"`).join(', ')}`;
    }),
    '',
    'marker ทั้งหมดที่เจอในไฟล์:',
    ...allMarkers.map((m) => `  #${m.id} "${m.name}" (พบ ${m.seenCount} เฟรม)`),
    '',
    'แก้โดยส่ง overrideMap เช่น { L_ASIS: 2, L_AnkleForHS: 6, L_AnkleForAngle: 9, ... }',
    'Heel → AnkleForHS ได้; มุม/ω sync ต้องการ lateral malleolus เป็น AnkleForAngle',
  ];
  return lines.join('\n');
}

# Test_3Aug — MoCap ↔ DernDee (3 Aug 2026)

## ไฟล์ต่อ trial

| | Test1 | Test2 |
|--|-------|-------|
| MoCap CSV | `TEST_DERNDEE.csv` | `DernDee3.csv` |
| Marker map | `marker-map.json` (จาก `position.md`) | เหมือนกัน แต่ **Heel ซ้าย/ขวา คนละหมายเลข** |
| IMU | `gait-trace-20260803-162019.json` | `gait-trace-20260803-163213.json` |
| Sync | ดู `SYNC.md` ในโฟลเดอร์นั้น | |

## Sync lag (IMU − MoCap)

| Trial | \|ω\| envelope (น่าเชื่อกว่า) | HS-circular (คลาดได้หลาย stride) |
|-------|-------------------------------|----------------------------------|
| **Test1** | **≈ 50.69 s** (L/R corr 0.99 / 0.94) | ≈ 53.41 s (spread L/R ใหญ่) |
| **Test2** | **≈ 3.16 s** (L/R corr 0.98 / 0.96) | ≈ 6.45 s |

- **signed ω** ไม่ correlate กับ IMU gx เมื่อ map **Heel → AnkleForHS** (polarity-mismatch / ambiguous)
- **\|ω\|** correlate สูงทั้งสองข้างชี้จุดเดียวกัน → exploratory clock sync ได้ แต่ **ไม่ใช่ lab gold-standard** (`syncTrusted=false`, `validationPublishable=false`)
- สาเหตุ: `knee→heel` ไม่ใช่ shank — ใช้ foot-velocity HS ได้ แต่ไม่ใช่มุมหน้าแข้ง / signed sync
- ระบบ fail-closed ถูกต้อง: ปฏิเสธ signed sync; envelope ใช้จับเวลาเท่านั้น ไม่เปิด agreement metrics

## ระยะทาง — อย่าสรุปจากตัวเลขที่เคยดู “แม่น”

`validationPublishable=false` ถูกต้องแล้ว

ตัวเลขที่เคยดูใกล้ `groundTruth.distanceM = 5 m` มาจาก **open stride + cycle ทับซ้อน + ค่าชน clamp** หักกลบกัน — ไม่ใช่ความแม่นจริง

หลังกรองคุณภาพ (ตัด clamp / untrusted / open / overlap): ข้อมูลที่เหลือยังน้อยเกินกว่าจะสรุปความแม่นของระยะทางจากชุดนี้

MoCap ครอบ pelvis max excursion ≈ **1.4 m** ของคลิปที่ IMU อ้าง 5 m — คนละช่วงเวลาของการเดิน

## Marker roles (สำคัญสำหรับ trial ถัดไป)

```json
{
  "L_AnkleForHS": <heel>,
  "R_AnkleForHS": <heel>,
  "L_AnkleForAngle": <lateral malleolus>,
  "R_AnkleForAngle": <lateral malleolus>
}
```

ชุดนี้มีแต่ Heel → map เป็น `*AnkleForHS` เท่านั้น (ไม่มี `*AnkleForAngle`)

**Trial ถัดไปต้องติด lateral malleolus** ถ้าต้องการ signed ω sync / peak shank angle

## รันซ้ำ

```bash
node mocap-analysis/run.js Test_3Aug/Test1/TEST_DERNDEE.csv \
  --map Test_3Aug/Test1/marker-map.json \
  --out Test_3Aug/Test1/mocap.gait-params.json

node mocap-analysis/syncSession.js \
  Test_3Aug/Test1/mocap.gait-params.json \
  Test_3Aug/Test1/gait-trace-20260803-162019.json \
  --out-dir Test_3Aug/Test1 --rival-scan 90

node mocap-analysis/analyzeSyncedStrideStep.js Test_3Aug
```

## ข้อจำกัดของชุดนี้

1. **Marker ชื่อ `Marker-1..8`** — ต้องมี `marker-map.json`; Heel ≠ malleolus
2. **เดินไป-กลับ** — ปิด `averageWalkingSpeedMps` / `vsMocapPelvisNetPct` เมื่อ `method !== 'net-start-end'`
3. **ไม่มี heel-tap / quiet onset** → `explorationOnly`; lab publishable = false
4. **ช่วงเวลาคนละส่วน** — MoCap ~1.4 m vs IMU GT 5 m
5. **`sameSideRepeats`** → `bilateral.reliable=false` ซ่อนตาราง step

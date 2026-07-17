# Lab validation SOP (SMART LAB / MoCap↔IMU)

ใช้เอกสารนี้ก่อนจองห้อง — เป้าหมายคือ **ไม่เสียเงินกับชุดข้อมูลที่เทียบไม่ได้**

## บังคับก่อนเดินทุก trial

1. **Calibrate** ทั้งสองข้าง: ยืนนิ่ง ≥3 s (gyro bias)
2. **ตรวจ axis map** ด้วย swing หน้า-หลัง — ถ้า polarity-mismatch ใน compare ต้องแก้ mount/map ก่อนเก็บชุดจริง
3. **Heel-tap 1 ครั้ง** ให้เห็น spike ทั้ง gyro และ marker velocity ก่อนเริ่มเดิน
4. **ยืนนิ่ง ≥0.4 s** ก่อนก้าวแรก (ช่วย onset coarse ถ้าไม่ได้ใส่ `--lag` มือ)
5. Firmware **100 Hz**, raw scale ตาม `raw/4096` (g) และ `raw/16.4` (dps)
6. ทุก sample ต้องมี **`t_ms`** — ห้าม trace ที่เวลา synthetic

## คำสั่งเทียบหลัง export

```bash
# 1) รัน mocap → mocap.gait-params.json
node mocap-analysis/run.js mocap.csv

# 2) เทียบแบบแลป (fail ถ้าใช้ผลไม่ได้)
node mocap-analysis/compare.js mocap.gait-params.json path/to/imu-trace.json \
  --lab --lag <SEC> --out report.json
```

`<SEC>` = เวลา IMU − MoCap จาก heel-tap (วัดจาก spike คู่เดียวกัน)

ถ้า `validationPublishable` ไม่ใช่ `true` → **ทิ้ง trial นั้น** อย่าเอา error% ไปใส่เปเปอร์

## สิ่งที่เคลมในเปเปอร์ได้ / ไม่ได้

| เมตริก | ใช้เป็น agreement ได้? |
|--------|-------------------------|
| strideLength | ได้ (หลัง paired + กรองคุณภาพ) |
| cadence / walkingSpeed | ได้ (ขึ้นกับ HS pairing ที่ trusted) |
| stancePct | **ไม่ได้** เป็น primary — คนละนิยาม event; MoCap มี bias ~−3% vs synthetic |
| peakShankAngleDeg | **ไม่ได้** จนกว่าจะมี angleOffset จากยืนนิ่ง |
| clearance | **ไม่ได้** — คนละจุดกายวิภาค |
| stepLength / doubleSupport | **ไม่ได้** กับ stroke — สูตรสมมาตร L/R |

## กรองคุณภาพอัตโนมัติแล้ว

ระบบตัดออกจาก agreement: stride clamp, ZUPT accel deviation > 0.25 g, stance จาก `previous-valid-ratio` / unresolved

## Force plate (ถ้ามี)

ใช้เป็น gold standard ของ HS/TO แทน ankle-velocity MoCap สำหรับ stance% — ลด systematic bias ของ reference

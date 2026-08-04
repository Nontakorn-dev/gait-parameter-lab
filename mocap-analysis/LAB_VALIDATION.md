# Lab validation SOP (SMART LAB / MoCap↔IMU)

ใช้เอกสารนี้ก่อนจองห้อง — เป้าหมายคือ **ไม่เสียเงินกับชุดข้อมูลที่เทียบไม่ได้**

## Pilot แรก (สำคัญกว่า agreement ทีละก้าว)

ก่อนจองห้องเต็มชุด — เดินตรง **วัดระยะจริง 10 m** แล้วเทียบ **`Σ strideLength clean`**
(`imuSumStrideLengthCleanM` / `sumStrideLengthCleanM` — ตัด clamp / untrusted / open / suspected-missed-HS)
กับ 10 m ผ่าน `groundTruth.distanceM` ใน trace

| ผล Σ clean | ความหมาย |
|------------|----------|
| ~8–12 m | ขยายไป MoCap agreement ได้ |
| ~1–2 m | double-integration พังบนข้อมูลจริง — หยุด; อย่าเก็บชุดใหญ่ |
| ติดลบเป็นระบบ | แกน/ขั้วผิด — ทำ swing test ใหม่ |

อย่าใช้ `imuSumStrideLengthM` (รวม clamp) เป็น pilot gate — ค่าเพดาน 1.80 m จะดึง % error หลอก

อย่าใช้ `strideLengthSignedM` เป็น auto-gate ของ axis map (residual จาก integration ทำให้ฟ้องเท็จได้) — ใช้ **swing test ด้วยมือ** ข้อ 2

## บังคับก่อนเดินทุก trial

1. **Calibrate** ทั้งสองข้าง: ยืนนิ่ง ≥3 s (gyro bias)
2. **ตรวจ axis map** ด้วย swing หน้า-หลัง (ขั้ว gx ตอนแกว่งขาไปหน้าควรเป็นบวก) — ถ้า polarity-mismatch ใน compare ต้องแก้ mount/map ก่อนเก็บชุดจริง
3. **Heel-tap 1 ครั้ง** ให้เห็น spike ทั้ง gyro และ marker velocity ก่อนเริ่มเดิน
4. **ยืนนิ่ง ≥0.4 s** ก่อนก้าวแรก (ช่วย onset coarse ถ้าไม่ได้ใส่ `--lag` มือ)
5. Firmware **100 Hz**, raw scale ตาม `raw/4096` (g) และ `raw/16.4` (dps)
6. ทุก sample ต้องมี **`t_ms`** — ห้าม trace ที่เวลา synthetic
7. ใส่ระยะเดินจริงใน `groundTruth.distanceM` เมื่อ export trace

## คำสั่งเทียบหลัง export

```bash
# 1) รัน mocap → mocap.gait-params.json
node mocap-analysis/run.js mocap.csv

# 2) เทียบแบบแลป (fail ถ้าใช้ผลไม่ได้)
node mocap-analysis/compare.js mocap.gait-params.json path/to/imu-trace.json \
  --lab --lag <SEC> --out report.json
```

`<SEC>` = เวลา IMU − MoCap จาก heel-tap (วัดจาก spike คู่เดียวกัน)

`--lag` ถูกตรวจด้วย corr ที่ค่าที่ใส่มาเอง **และ** สแกนเพื่อนบ้าน ±k·stride (ปฏิเสธเมื่อมี peak ดีกว่าชัด) + เทียบกับ free-scan — ถ้า corr อ่อน / เป็น period alias / ไม่มี MoCap ω ให้ตรวจ → `syncTrusted=false` (fail-closed)

ถ้า `validationPublishable` ไม่ใช่ `true` → **ทิ้ง trial นั้น** อย่าเอา error% ไปใส่เปเปอร์

## สิ่งที่เคลมในเปเปอร์ได้ / ไม่ได้

| เมตริก | ใช้เป็น agreement ได้? |
|--------|-------------------------|
| strideLength | ได้ (หลัง paired + กรองคุณภาพ) |
| cadence / walkingSpeed | ได้ (ขึ้นกับ HS pairing ที่ trusted) |
| stancePct | **ไม่ได้** เป็น primary — คนละนิยาม event; MoCap มี bias ~−3% vs synthetic |
| peakShankAngleDeg | **ไม่ได้** จนกว่าจะมี angleOffset จากยืนนิ่ง |
| clearance | **ไม่ได้** — คนละจุดกายวิภาค |
| stepLength / doubleSupport | **ไม่ได้** จากเซนเซอร์ข้างเดียว — ห้าม stride/2 หรือ 2·stance−100; ต้องมี HS สองข้าง |

## กรองคุณภาพอัตโนมัติแล้ว

ระบบตัดออกจาก agreement: stride clamp, ZUPT accel deviation > 0.25 g, stance จาก `previous-valid-ratio` / unresolved

## Force plate (ถ้ามี)

ใช้เป็น gold standard ของ HS/TO แทน ankle-velocity MoCap สำหรับ stance% — ลด systematic bias ของ reference

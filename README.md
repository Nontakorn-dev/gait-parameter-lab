# Gait Parameter Lab Dashboard (Standalone)

Standalone extraction ของ Gait Parameter Dashboard จาก `DernDee_Web-app` เพื่อใช้ทดสอบ/พัฒนากับ sensor lab จริง โดยไม่ต้องพึ่ง React app หรือ backend อื่น ๆ ของ DernDee

## รันโปรเจกต์

```bash
cd gait-parameter-lab
npm install
npm run dev
```

เปิด `http://localhost:5180` (ต้องใช้ Chrome/Edge เพื่อรองรับ Web Bluetooth — ทดสอบผ่าน `localhost` ถือว่าเป็น secure context แล้ว แต่ถ้าจะทดสอบจากมือถือ/เครื่องอื่นต้องรันผ่าน HTTPS)

## โครงสร้างไฟล์

```
src/
  main.js                     ← entry point, mount app + import CSS
  gait-dashboard/
    app.js                    ← GaitLabDashboardApp: orchestration (demo/BLE/gateway mode, calibration flow, sensor routing)
    ui/dashboard.js            ← DOM binding สำหรับแสดงตัวเลข gait parameter
    ui/charts.js                ← Chart.js realtime plots (angular velocity, shank angle)
    data/demoDataGenerator.js   ← จำลองข้อมูลเดินสำหรับ demo mode
    styles/main.css              ← สไตล์ dashboard
  gait/                         ← engine คำนวณ gait parameter (ไม่ผูกกับ UI)
    gaitProcessorClient.js       ← wrapper เลือกใช้ Worker หรือรันใน main thread
    gaitProcessor.js             ← core: cadence, step/stride length, walking speed, stance/swing %, double support ฯลฯ
    gaitProcessor.worker.js       ← รัน gaitProcessor.js ใน Web Worker
    gaitEventDetector.js          ← ตรวจจับ Heel Strike / stance-swing (adaptive threshold)
    kalmanFilter.js               ← ประมาณมุม shank จาก gyro+accel
    velocityIntegrator.js          ← อินทิเกรตความเร่ง → step length / clearance (พร้อม drift correction)
    signalUtils.js                 ← แปลงหน่วย raw IMU, moving average, trapezoidal integrate ฯลฯ
    gaitCalibration.js              ← gyro bias + shank length estimation
    gaitCalibrationCapture.js        ← flow เก็บข้อมูล calibration 3 วินาที
    gaitCalibrationStore.js           ← เก็บผล calibration ใน localStorage
    gaitRuntimeConfig.js               ← ค่าคงที่ (analysis interval, min samples, stale timeout)
    gaitSensorSelection.js              ← เลือกเซนเซอร์ซ้าย/ขวาที่จะแสดงผล
    gaitParamsAggregation.js             ← เฉลี่ยผลจากหลาย cycle/เซนเซอร์เป็น summary
  gateway/
    realtimeSensorUtils.js         ← normalize sample จากเซนเซอร์ (side, sensorKey, ฯลฯ)
    realtimeTransport.js            ← Web Bluetooth (BLE) client + WebSocket gateway client
```

ทุกไฟล์คัดลอกมาจาก `DernDee_Web-app/shared/{gait,gait-dashboard,gateway}` แบบ 1:1 (import path ระหว่างกันยังทำงานได้ เพราะจัดโฟลเดอร์เป็น sibling เหมือนเดิม)

## ปรับให้เข้ากับ lab จริง

- **BLE service/characteristic UUID**: แก้ที่ [src/gateway/realtimeTransport.js](src/gateway/realtimeTransport.js) — ค่า `BLE_SERVICE_UUID`, `BLE_CHARACTERISTIC_UUID`, `BLE_DEVICE_PREFIX`, และ `BODY_POSITIONS` (device id ต่อตำแหน่งเซนเซอร์)
- **รูปแบบ payload จากเซนเซอร์**: แก้ที่ [src/gateway/realtimeSensorUtils.js](src/gateway/realtimeSensorUtils.js) ฟังก์ชัน `normalizeRealtimeSensorSample` ให้ตรงกับ field ที่ฮาร์ดแวร์จริงส่งมา (raw accel/gyro, timestamp, side)
- **Sample rate/threshold ของอัลกอริทึม**: ค่าคงที่อยู่ต้นไฟล์ [src/gait/gaitProcessor.js](src/gait/gaitProcessor.js) (`SAMPLE_RATE`, `STEP_LENGTH_MIN/MAX_M`, threshold ต่าง ๆ) — ถ้าฮาร์ดแวร์จริงส่งข้อมูลคนละ sample rate ต้องแก้ตรงนี้
- **โหมด Demo**: ปุ่ม "Demo" ยังใช้ข้อมูลจำลองจาก `demoDataGenerator.js` ไว้ทดสอบ UI โดยไม่ต้องมีเซนเซอร์จริง

## หมายเหตุ

- โปรเจกต์นี้ไม่ได้พึ่ง Supabase, React, หรือ auth ใด ๆ ของ DernDee_Web-app — เป็นไฟล์ vanilla JS + Chart.js ล้วน ๆ
- Calibration profile และ transport mode เก็บอยู่ใน `localStorage`/`sessionStorage` ของ browser เท่านั้น

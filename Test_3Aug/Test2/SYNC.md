# Sync: Test2
- **lag (IMU − MoCap)** = **3.160 s** (`signal-xcorr-envelope`)
- HS-consensus lag (reference) = 6.450 s — คลาดจาก envelope ได้ ~1 stride เมื่อ Heel≠malleolus
- MoCap: `mocap.gait-params.json`
- IMU: `gait-trace-20260803-163213.json`
- ไม่ใช้ระยะ/จำนวนก้าวจาก IMU JSON — เทียบกับ MoCap โดยตรง
- ตารางจับคู่ใช้ lag = 3.160s
## Paired cycles

### L (lag=3.160s)

| MoCap HS (s) | IMU HS (s) | aligned IMU (s) | Δt (s) | MoCap stride (m) | IMU stride (m) |
|---:|---:|---:|---:|---:|---:|
| — | — | — | — | — | — |

### R (lag=3.160s)

| MoCap HS (s) | IMU HS (s) | aligned IMU (s) | Δt (s) | MoCap stride (m) | IMU stride (m) |
|---:|---:|---:|---:|---:|---:|
| — | — | — | — | — | — |

## Notes
- explorationOnly = true
- validationPublishable (lab gate) = false
- Heel = AnkleForHS เท่านั้น — ไม่มี malleolus → signed ω sync ไม่ใช้; |ω| envelope เป็น exploratory clock sync
- Σ stride clean หลังตัด clamp/open/overlap ยังน้อยกว่า groundTruth 5 m มาก — อย่าตีความ % ระยะเป็นความแม่น

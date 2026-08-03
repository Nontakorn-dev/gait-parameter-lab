# Sync: Test1
- **lag (IMU − MoCap)** = **50.685 s** (`signal-xcorr-envelope`)
- HS-consensus lag (reference) = 53.410 s — คลาดจาก envelope ได้ ~1 stride เมื่อ Heel≠malleolus
- MoCap: `mocap.gait-params.json`
- IMU: `gait-trace-20260803-162019.json`
- ไม่ใช้ระยะ/จำนวนก้าวจาก IMU JSON — เทียบกับ MoCap โดยตรง
- ตารางจับคู่ใช้ lag = 50.685s
## Paired cycles

### L (lag=50.685s)

| MoCap HS (s) | IMU HS (s) | aligned IMU (s) | Δt (s) | MoCap stride (m) | IMU stride (m) |
|---:|---:|---:|---:|---:|---:|
| 7.03 | 57.46 | 6.78 | -0.250 | 0.884 | 0.227 |

### R (lag=50.685s)

| MoCap HS (s) | IMU HS (s) | aligned IMU (s) | Δt (s) | MoCap stride (m) | IMU stride (m) |
|---:|---:|---:|---:|---:|---:|
| — | — | — | — | — | — |

## Notes
- explorationOnly = true
- validationPublishable (lab gate) = false
- Heel = AnkleForHS เท่านั้น — ไม่มี malleolus → signed ω sync ไม่ใช้; |ω| envelope เป็น exploratory clock sync
- Σ stride clean หลังตัด clamp/open/overlap ยังน้อยกว่า groundTruth 5 m มาก — อย่าตีความ % ระยะเป็นความแม่น

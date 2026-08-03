# Stride / Step analysis (หลัง sync)

Stride: เทียบ MoCap vs IMU จากคู่ HS หลัง sync. Step length: มีจาก MoCap (ระยะ marker ส้นเท้า HS→HS ข้างตรงข้าม). IMU step length = null — ห้าม stride/2 จากเซนเซอร์ข้างเดียว. Step time: เทียบได้เมื่อจับคู่ HS สลับข้างหลัง sync

## Test1

- Sync lag ≈ **50.685 s**
- Stride คู่ที่ใช้ได้: **2/6** (ตัดรอบกลับตัว MoCap < 0.30 m)
- Stride mean: MoCap **0.499 m** · IMU **1.479 m** · MAE **0.980 m** · bias **196.1%**
- ⛔ bilateral.reliable=false (sameSideRepeats=3) — ซ่อนตาราง step
- Pelvis max excursion ≈ **1.38 m** (net ไม่ใช้ — เดินไป-กลับ)

### Stride pairs

| Side | MoCap HS | IMU HS | MoCap stride (m) | IMU stride (m) | Error (m) | Error % | Use? |
|:---:|---:|---:|---:|---:|---:|---:|:---:|
| L | 2.24 | 55.74 | 0.495 | 1.157 | 0.662 | 134 | yes |
| L | 4.05 | 57.46 | 0.111 | 0.227 | 0.116 | 104 | no |
| L | 5.64 | 58.88 | 0.217 | 0.511 | 0.294 | 135 | no |
| L | 7.03 | 60.47 | 0.884 | — | — | — | no |
| R | 1.32 | 51.44 | 0.904 | 0.100 | -0.804 | -89 | no |
| R | 6.24 | 56.47 | 0.502 | 1.800 | 1.298 | 258 | yes |

### MoCap step length (bilateral)

_bilateral.reliable=false (sameSideRepeats=3) — ซ่อนตาราง step_


## Test2

- Sync lag ≈ **3.160 s**
- Stride คู่ที่ใช้ได้: **0/6** (ตัดรอบกลับตัว MoCap < 0.30 m)
- ไม่มีคู่ stride ที่ใช้ได้พอ
- ⛔ bilateral.reliable=false (sameSideRepeats=1) — ซ่อนตาราง step
- Pelvis max excursion ≈ **0.93 m** (net ไม่ใช้ — เดินไป-กลับ)

### Stride pairs

| Side | MoCap HS | IMU HS | MoCap stride (m) | IMU stride (m) | Error (m) | Error % | Use? |
|:---:|---:|---:|---:|---:|---:|---:|:---:|
| L | 0.00 | 5.94 | 0.338 | — | — | — | no |
| L | 7.69 | 13.57 | 0.045 | 0.312 | 0.267 | 587 | no |
| L | 9.10 | 15.25 | 0.260 | — | — | — | no |
| R | 0.00 | 6.87 | 0.649 | — | — | — | no |
| R | 4.17 | 11.13 | 0.216 | 1.067 | 0.851 | 393 | no |
| R | 6.93 | 14.29 | 0.043 | 0.100 | 0.057 | 130 | no |

### MoCap step length (bilateral)

_bilateral.reliable=false (sameSideRepeats=1) — ซ่อนตาราง step_


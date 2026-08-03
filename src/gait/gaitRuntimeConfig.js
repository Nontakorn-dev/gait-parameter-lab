export const GAIT_ANALYSIS_INTERVAL_MS = 400;
export const GAIT_MIN_ANALYSIS_SAMPLES = 300;
export const GAIT_SENSOR_STALE_AFTER_MS = 2500;
export const GAIT_MAX_VISIBLE_SHANK_SENSORS = 2;

/**
 * จำนวน sample ระหว่าง analyze() ใน reprocess mirror-live
 * ≈ GAIT_ANALYSIS_INTERVAL_MS @ 100 Hz (dashboard เรียก analyze ตาม timer 400ms)
 *
 * ห้ามเปลี่ยนหลัง campaign validate โดยไม่รัน pilot ใหม่ — Σclean / clamp
 * บน Test_3Aug ขึ้นกับค่านี้ (e=40 vs e=137 ให้ผลคนละชุด)
 */
export const GAIT_SAMPLE_RATE_HZ = 100;
export const GAIT_ANALYZE_EVERY_SAMPLES = 40;

/**
 * @file    fusion.h
 * @brief   Complementary Filter cho sensor fusion IMU — Pitch & Roll.
 *
 * ==========================================================================
 *  LÝ THUYẾT COMPLEMENTARY FILTER
 * ==========================================================================
 *
 *  Vấn đề với từng cảm biến riêng lẻ:
 *
 *    Gyroscope:
 *      angle(t) = angle(t-1) + gyro_rate × dt
 *      → Chính xác trong ngắn hạn (phản ứng nhanh, không bị lag)
 *      → Bị DRIFT (tích phân sai số nhỏ → tích lũy thành sai số lớn theo thời gian)
 *      → Đặc trưng High-Pass: tin tốt ở thay đổi nhanh, kém ở DC (tĩnh)
 *
 *    Accelerometer:
 *      accel_pitch = atan2(Ax, √(Ay²+Az²)) × (180/π)
 *      → Chính xác về lâu dài (không drift, có tham chiếu trọng lực)
 *      → Bị NHIỄU khi có gia tốc tuyến tính (rung cơ học, chuyển động nhanh)
 *      → Đặc trưng Low-Pass: tin tốt ở DC, kém ở thay đổi nhanh
 *
 *  Complementary Filter kết hợp hai nguồn:
 *
 *    angle = α × (angle + gyro × dt)   ← Phần High-Pass (Gyro)
 *          + (1 - α) × accel_angle     ← Phần Low-Pass (Accel)
 *
 *    α gần 1.0 → tin gyro nhiều (phản ứng nhanh, drift nhiều)
 *    α gần 0.0 → tin accel nhiều (ổn định, nhưng nhiễu cao tần)
 *
 *  Giá trị α điển hình cho servo gimbal @ 500Hz:
 *    α = 0.98 → time constant τ = α / ((1-α) × f) = 0.98 / (0.02 × 500) = 0.098s
 *    → Drift được bù sau ~0.1 giây
 *
 *  Lưu ý: Complementary Filter nhanh hơn Kalman nhưng không ước lượng
 *  được gyro bias tự động. Do đó PHẢI kết hợp với ImuDual_Calibrate().
 * ==========================================================================
 */

#ifndef __FUSION_H__
#define __FUSION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ===========================================================================
 *  DATA STRUCTURES
 * =========================================================================== */

/**
 * @brief  Handle cho Complementary Filter một IMU (2 trục: Pitch + Roll).
 */
typedef struct {
    float alpha;   /*!< Hệ số filter [0,1]. Mặc định: 0.98          */
    float pitch;   /*!< Góc Pitch ước lượng (°) — ngẩng/cúi đầu    */
    float roll;    /*!< Góc Roll  ước lượng (°) — nghiêng trái/phải */
    uint8_t initialized;  /*!< 0 = chưa có dữ liệu ban đầu          */
} CompFilter_t;

/* ===========================================================================
 *  PUBLIC API
 * =========================================================================== */

/**
 * @brief  Khởi tạo Complementary Filter.
 * @param  cf     Con trỏ handle.
 * @param  alpha  Hệ số filter (0.95–0.99 phù hợp cho gimbal servo).
 */
void CompFilter_Init(CompFilter_t *cf, float alpha);

/**
 * @brief  Cập nhật một bước filter với dữ liệu IMU mới.
 *
 *         Pitch:   accel_pitch = atan2(ax,  √(ay²+az²)) × (180/π)
 *         Roll:    accel_roll  = atan2(ay, az)           × (180/π)
 *
 *         angle_new = α × (angle_old + gyro × dt) + (1−α) × accel_angle
 *
 * @param  cf         Con trỏ handle.
 * @param  ax, ay, az Gia tốc (g), đã trừ calibration.
 * @param  gx, gy     Gyro rate (°/s) — gx=Pitch rate, gy=Roll rate.
 * @param  dt         Chu kỳ lấy mẫu (giây). Phải cố định.
 */
void CompFilter_Update(CompFilter_t *cf,
                       float ax, float ay, float az,
                       float gx, float gy,
                       float dt);

/**
 * @brief  Reset filter về góc ban đầu (tính từ accel).
 *         Gọi khi biết IMU đang ở vị trí tĩnh.
 * @param  cf         Con trỏ handle.
 * @param  ax, ay, az Gia tốc tại thời điểm reset (g).
 */
void CompFilter_Reset(CompFilter_t *cf, float ax, float ay, float az);

/**
 * @brief  Đặt lại hệ số alpha trong khi chạy (runtime tuning).
 */
void CompFilter_SetAlpha(CompFilter_t *cf, float alpha);

#ifdef __cplusplus
}
#endif

#endif /* __FUSION_H__ */

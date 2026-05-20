/**
 * @file    fusion.c
 * @brief   Complementary Filter implementation.
 */

#include "fusion.h"
#include <math.h>

/* Hằng số chuyển đổi */
#define RAD_TO_DEG  57.29577951308f

void CompFilter_Init(CompFilter_t *cf, float alpha)
{
    cf->alpha       = (alpha > 0.999f) ? 0.999f :
                      (alpha < 0.0f)   ? 0.0f   : alpha;
    cf->pitch       = 0.0f;
    cf->roll        = 0.0f;
    cf->initialized = 0;
}

void CompFilter_Update(CompFilter_t *cf,
                       float ax, float ay, float az,
                       float gx, float gy,
                       float dt)
{
    /* -------- Góc từ Accelerometer (long-term stable) --------
     *
     *  Pitch = atan2(Ax, √(Ay² + Az²))
     *    → góc giữa vector gia tốc và mặt phẳng YZ
     *    → = 0° khi chip nằm phẳng, +90° khi đứng thẳng mũi lên
     *
     *  Roll = atan2(Ay, Az)
     *    → góc nghiêng trái/phải
     */
    float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    float accel_roll  = atan2f(ay, az) * RAD_TO_DEG;

    /* -------- Lần đầu tiên: khởi tạo từ Accel --------
     * Tránh bump lớn khi bật gimbal do angle = 0 nhưng thực tế ≠ 0.
     */
    if (!cf->initialized) {
        cf->pitch       = accel_pitch;
        cf->roll        = accel_roll;
        cf->initialized = 1;
        return;
    }

    /* -------- Complementary Filter --------
     *
     *  angle = α × (angle + gyro × dt) + (1−α) × accel_angle
     *
     *  Phần thứ 1: tích phân gyro — phản ứng tức thì, ngắn hạn chính xác
     *  Phần thứ 2: bù bằng accel  — hiệu chỉnh drift dài hạn
     */
    float alpha = cf->alpha;

    cf->pitch = alpha * (cf->pitch + gx * dt) + (1.0f - alpha) * accel_pitch;
    cf->roll  = alpha * (cf->roll  + gy * dt) + (1.0f - alpha) * accel_roll;
}

void CompFilter_Reset(CompFilter_t *cf, float ax, float ay, float az)
{
    cf->pitch       = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    cf->roll        = atan2f(ay, az) * RAD_TO_DEG;
    cf->initialized = 1;
}

void CompFilter_SetAlpha(CompFilter_t *cf, float alpha)
{
    if (alpha > 0.999f) alpha = 0.999f;
    if (alpha < 0.0f)   alpha = 0.0f;
    cf->alpha = alpha;
}

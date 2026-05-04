/**
 * @file    servo_control.c
 * @brief   Điều khiển servo theo 3 vùng tín hiệu RC (incremental stepping).
 *
 *  Logic cốt lõi:
 *    - Vùng THẤP  → giảm CCR từng bước (SERVO_STEP_SIZE µs) mỗi SERVO_STEP_INTERVAL_MS ms
 *    - Vùng GIỮA  → giữ nguyên CCR (không thay đổi)
 *    - Vùng CAO   → tăng CCR từng bước mỗi SERVO_STEP_INTERVAL_MS ms
 *    - Mất tín hiệu (pulse == 0) → dừng tại vị trí hiện tại
 *
 *  Ưu điểm: servo không bao giờ nhảy đột ngột → không gây tiếng tạch tạch.
 */

#include "servo_control.h"

/* -------------------------------------------------------------------------- */
/* Trạng thái nội bộ (static)                                                 */
/* -------------------------------------------------------------------------- */

/* Vị trí hiện tại của từng servo (tính bằng µs CCR).
   Được cập nhật mỗi bước, luôn nằm trong [SERVO_PWM_MIN, SERVO_PWM_MAX]. */
static uint32_t s_posCh2 = SERVO_PWM_MID;
static uint32_t s_posCh3 = SERVO_PWM_MID;

/* Thời điểm bước cuối cùng – dùng để rate-limiting (non-blocking). */
static uint32_t s_lastStepTime = 0;

/* -------------------------------------------------------------------------- */
/* Helper: phân loại vùng tín hiệu                                            */
/* -------------------------------------------------------------------------- */
typedef enum {
    SERVO_ZONE_LOW,   /* Xoay nghịch */
    SERVO_ZONE_MID,   /* Dừng        */
    SERVO_ZONE_HIGH,  /* Xoay thuận  */
} ServoZone;

static inline ServoZone classifyZone(uint32_t pulse)
{
    if (pulse == 0 || (pulse >= SERVO_ZONE_LOW_MAX && pulse <= SERVO_ZONE_HIGH_MIN)) {
        return SERVO_ZONE_MID;   /* Mất tín hiệu hoặc vùng giữa → dừng */
    } else if (pulse < SERVO_ZONE_LOW_MAX) {
        return SERVO_ZONE_LOW;   /* Vùng thấp → xoay nghịch */
    } else {
        return SERVO_ZONE_HIGH;  /* Vùng cao  → xoay thuận  */
    }
}

/* -------------------------------------------------------------------------- */
/* Helper: cập nhật một kênh servo (tăng/giảm/giữ CCR theo zone)             */
/* -------------------------------------------------------------------------- */
static uint32_t stepServo(uint32_t currentPos, ServoZone zone)
{
    switch (zone)
    {
        case SERVO_ZONE_LOW:
            if (currentPos > SERVO_PWM_MIN + SERVO_STEP_SIZE) {
                currentPos -= SERVO_STEP_SIZE;
            } else {
                currentPos = SERVO_PWM_MIN;  /* Đã chạm giới hạn cuối */
            }
            break;

        case SERVO_ZONE_HIGH:
            if (currentPos < SERVO_PWM_MAX - SERVO_STEP_SIZE) {
                currentPos += SERVO_STEP_SIZE;
            } else {
                currentPos = SERVO_PWM_MAX;  /* Đã chạm giới hạn cuối */
            }
            break;

        case SERVO_ZONE_MID:
        default:
            /* Giữ nguyên vị trí – không làm gì */
            break;
    }
    return currentPos;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

void Servo_Init(void)
{
    /* Bật PWM cho cả hai kênh */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    /* Đặt servo về 90° (vị trí trung tâm) */
    s_posCh2 = SERVO_PWM_MID;
    s_posCh3 = SERVO_PWM_MID;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, s_posCh2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, s_posCh3);

    s_lastStepTime = HAL_GetTick();
}

void Servo_Update(uint32_t pulseCh2, uint32_t pulseCh3)
{
    /* Rate-limiting: chỉ thực hiện bước tiếp theo sau mỗi SERVO_STEP_INTERVAL_MS ms.
       Điều này đảm bảo tốc độ quay cố định, không phụ thuộc vào tốc độ vòng lặp. */
    uint32_t now = HAL_GetTick();
    if (now - s_lastStepTime < SERVO_STEP_INTERVAL_MS) {
        return;  /* Chưa đến lúc – bỏ qua lần gọi này */
    }
    s_lastStepTime = now;

    /* Phân vùng tín hiệu */
    ServoZone zoneCh2 = classifyZone(pulseCh2);
    ServoZone zoneCh3 = classifyZone(pulseCh3);

    /* Cập nhật vị trí theo bước */
    s_posCh2 = stepServo(s_posCh2, zoneCh2);
    s_posCh3 = stepServo(s_posCh3, zoneCh3);

    /* Ghi CCR ra timer – servo chỉ di chuyển đúng 1 bước nhỏ mỗi lần */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, s_posCh2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, s_posCh3);
}

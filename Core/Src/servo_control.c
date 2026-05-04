/**
 * @file    servo_control.c
 * @brief   Điều khiển servo qua TIM3 CH2 và CH3.
 *
 *  Ánh xạ: RC 1000–2000 µs → servo 0°–180° (pass-through 1:1).
 *  Failsafe: giữ 90° (1500 µs) khi pulseCh == 0 (mất tín hiệu).
 */

#include "servo_control.h"

void Servo_Init(void)
{
    /* Bật PWM cho cả hai kênh servo */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    /* Đặt servo về 90° (1500 µs) trước khi có tín hiệu RC */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_PWM_MID);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, SERVO_PWM_MID);
}

void Servo_Update(uint32_t pulseCh2, uint32_t pulseCh3)
{
    /* --- Servo kênh 2 --- */
    if (pulseCh2 > 0) {
        int32_t out = (int32_t)pulseCh2;
        if (out < (int32_t)SERVO_PWM_MIN) out = (int32_t)SERVO_PWM_MIN;  /* Giới hạn 0°   */
        if (out > (int32_t)SERVO_PWM_MAX) out = (int32_t)SERVO_PWM_MAX;  /* Giới hạn 180° */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)out);
    } else {
        /* Mất tín hiệu → failsafe 90° */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_PWM_MID);
    }

    /* --- Servo kênh 3 --- */
    if (pulseCh3 > 0) {
        int32_t out = (int32_t)pulseCh3;
        if (out < (int32_t)SERVO_PWM_MIN) out = (int32_t)SERVO_PWM_MIN;  /* Giới hạn 0°   */
        if (out > (int32_t)SERVO_PWM_MAX) out = (int32_t)SERVO_PWM_MAX;  /* Giới hạn 180° */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)out);
    } else {
        /* Mất tín hiệu → failsafe 90° */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, SERVO_PWM_MID);
    }
}

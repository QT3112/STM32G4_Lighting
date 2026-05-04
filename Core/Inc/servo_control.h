/**
 * @file    servo_control.h
 * @brief   Module điều khiển servo qua TIM3 CH2 và CH3.
 *          Ánh xạ RC 1000–2000 µs → servo 0°–180° (1000–2000 µs).
 *          Failsafe: giữ 90° (1500 µs) khi mất tín hiệu.
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "main.h"
#include "tim.h"

/* Dải xung đầu vào RC hợp lệ (µs) ---------------------------------------- */
#define SERVO_RC_MIN    1000U
#define SERVO_RC_MAX    2000U

/* Dải xung đầu ra servo (µs) ---------------------------------------------- */
#define SERVO_PWM_MIN   1000U   /* 0°   */
#define SERVO_PWM_MID   1500U   /* 90°  – vị trí failsafe */
#define SERVO_PWM_MAX   2000U   /* 180° */

/**
 * @brief Khởi tạo module servo: bật PWM và đặt về vị trí 90° (failsafe).
 *        Gọi sau MX_TIM3_Init().
 */
void Servo_Init(void);

/**
 * @brief Cập nhật vị trí servo dựa trên giá trị RC pulse.
 *        Ánh xạ 1000–2000 µs → 1000–2000 µs (pass-through).
 *        Nếu pulse == 0 (mất tín hiệu): giữ 90° (1500 µs).
 * @param pulseCh2  Pulse width kênh RC 2 (µs).
 * @param pulseCh3  Pulse width kênh RC 3 (µs).
 */
void Servo_Update(uint32_t pulseCh2, uint32_t pulseCh3);

#endif /* SERVO_CONTROL_H */

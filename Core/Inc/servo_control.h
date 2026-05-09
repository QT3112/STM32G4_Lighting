/**
 * @file    servo_control.h
 * @brief   Module điều khiển servo theo 3 vùng tín hiệu RC (TIM3 CH2/CH3).
 *
 *  Thay vì ánh xạ trực tiếp pulse → góc (gây giật khi tín hiệu thay đổi đột
 *  ngột), module này dùng phương pháp INCREMENTAL STEPPING:
 *
 *    - Vùng THẤP  (pulse < SERVO_ZONE_LOW_MAX)  : giảm CCR từng bước → xoay
 * nghịch
 *    - Vùng GIỮA  (LOW_MAX ≤ pulse ≤ HIGH_MIN)  : giữ CCR hiện tại → dừng
 *    - Vùng CAO   (pulse > SERVO_ZONE_HIGH_MIN) : tăng CCR từng bước → xoay
 * thuận
 *    - Mất tín hiệu (pulse == 0)                : dừng tại chỗ (giống vùng
 * giữa)
 *
 *  Tốc độ quay được điều chỉnh bằng SERVO_STEP_SIZE và SERVO_STEP_INTERVAL_MS.
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "main.h"
#include "tim.h"

/* Dải xung đầu ra servo (µs) ---------------------------------------------- */
#define SERVO_PWM_MIN 500U  /* Giới hạn cuối 0°   */
#define SERVO_PWM_MID 1500U /* Vị trí khởi động 90° */
#define SERVO_PWM_MAX 2500U /* Giới hạn cuối 180° */

/* Ngưỡng phân vùng tín hiệu RC (µs) --------------------------------------- */
#define SERVO_ZONE_LOW_MAX                                                     \
  1250U /* pulse < 1250  → Vùng THẤP  (xoay nghịch)                     \
         */
#define SERVO_ZONE_HIGH_MIN                                                    \
  1750U /* pulse > 1750  → Vùng CAO   (xoay thuận)                        \
         */
/* 1250 ≤ pulse ≤ 1750 → Vùng GIỮA (dừng)   */

/* Tham số tốc độ bước ------------------------------------------------------ */
/* Cứ mỗi SERVO_STEP_INTERVAL_MS ms, vị trí servo thay đổi SERVO_STEP_SIZE µs.
 */
/* Tốc độ quét toàn dải (1000→2000µs):                                        */
/*   t = (SERVO_PWM_MAX - SERVO_PWM_MIN) / SERVO_STEP_SIZE *
 * SERVO_STEP_INTERVAL_MS */
/*   Ví dụ: 20µs/bước × 20ms/bước = 1 giây để quét 180°                      */
#define SERVO_STEP_SIZE 20U        /* µs thay đổi mỗi bước       */
#define SERVO_STEP_INTERVAL_MS 20U /* ms giữa hai bước liên tiếp  */

/**
 * @brief Khởi tạo module servo: bật PWM và đặt về vị trí 90°.
 *        Gọi sau MX_TIM3_Init() và trước RC_Input_Init().
 */
void Servo_Init(void);

/**
 * @brief Cập nhật vị trí servo theo logic 3 vùng (non-blocking).
 *        Gọi liên tục trong vòng lặp while(1) của main.
 * @param pulseCh2  Pulse width kênh RC 2 (µs), 0 = mất tín hiệu → dừng.
 * @param pulseCh3  Pulse width kênh RC 3 (µs), 0 = mất tín hiệu → dừng.
 */
void Servo_Update(uint32_t pulseCh2, uint32_t pulseCh3);

#endif /* SERVO_CONTROL_H */

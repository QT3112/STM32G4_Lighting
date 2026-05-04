/**
 * @file    lighting_control.h
 * @brief   Module điều khiển đèn 3 vùng dựa trên RC CH1 (TIM2 CH1).
 *
 *  Vùng 1 – pulse > 1750 µs  : Hiệu ứng SOS (non-blocking)
 *  Vùng 2 – 1000 ≤ pulse < 1250 µs : Đèn sáng liên tục (ON)
 *  Vùng 3 – các giá trị còn lại / mất tín hiệu : Đèn tắt (OFF)
 */

#ifndef LIGHTING_CONTROL_H
#define LIGHTING_CONTROL_H

#include "main.h"

/* Ngưỡng điều khiển đèn (µs) ---------------------------------------------- */
#define LIGHT_ZONE_SOS_THRESHOLD    1750U   /* > giá trị này → SOS      */
#define LIGHT_ZONE_ON_MIN           1000U   /* ≥ giá trị này → ON zone  */
#define LIGHT_ZONE_ON_MAX           1250U   /* <  giá trị này → ON zone */

/* Chân GPIO đèn ------------------------------------------------------------ */
#define LIGHT_GPIO_PORT    GPIOA
#define LIGHT_GPIO_PIN     GPIO_PIN_6

/**
 * @brief Cập nhật trạng thái đèn theo giá trị pulse RC CH1.
 *        Gọi trong vòng lặp while(1) của main (non-blocking).
 * @param pulseCh1  Pulse width kênh RC 1 (µs), 0 = mất tín hiệu.
 */
void Lighting_Update(uint32_t pulseCh1);

#endif /* LIGHTING_CONTROL_H */

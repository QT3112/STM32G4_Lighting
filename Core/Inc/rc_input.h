/**
 * @file    rc_input.h
 * @brief   Module xử lý tín hiệu RC PWM Input Capture (TIM2 CH1/CH2/CH3).
 *          Cung cấp giá trị pulse width đã lọc nhiễu cho các module khác.
 */

#ifndef RC_INPUT_H
#define RC_INPUT_H

#include "main.h"
#include "tim.h"

/* Ngưỡng lọc nhiễu RC PWM hợp lệ (µs) ------------------------------------ */
#define RC_PULSE_MIN     800U   /* Giá trị tối thiểu chấp nhận (µs) */
#define RC_PULSE_MAX    2200U   /* Giá trị tối đa chấp nhận (µs)    */

/* Thời gian timeout: nếu không có xung trong RC_TIMEOUT_MS thì coi mất tín
   hiệu và reset pulse về 0. RC chuẩn 50 Hz → 20 ms/frame. Cho phép miss ~10
   frame liên tiếp trước khi kích hoạt failsafe. */
#define RC_TIMEOUT_MS   200U

/**
 * @brief Khởi tạo module RC Input.
 *        Gọi SAU HAL_Delay() khởi động để lastPulseTime không bị timeout ngay.
 */
void RC_Input_Init(void);

/**
 * @brief Kiểm tra timeout và reset pulse về 0 nếu quá RC_TIMEOUT_MS.
 *        Gọi trong vòng lặp while(1) của main.
 */
void RC_Input_Update(void);

/**
 * @brief Lấy giá trị pulse width kênh 1 (dùng cho đèn).
 * @return Độ rộng xung µs, hoặc 0 nếu mất tín hiệu.
 */
uint32_t RC_Input_GetCh1(void);

/**
 * @brief Lấy giá trị pulse width kênh 2 (dùng cho servo 1).
 * @return Độ rộng xung µs, hoặc 0 nếu mất tín hiệu.
 */
uint32_t RC_Input_GetCh2(void);

/**
 * @brief Lấy giá trị pulse width kênh 3 (dùng cho servo 2).
 * @return Độ rộng xung µs, hoặc 0 nếu mất tín hiệu.
 */
uint32_t RC_Input_GetCh3(void);

/**
 * @brief Hàm callback ISR – phải được gọi từ HAL_TIM_IC_CaptureCallback.
 *        Xử lý logic bắt cạnh lên/xuống và tính pulse width.
 * @param htim Con trỏ timer handle từ HAL callback.
 */
void RC_Input_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* RC_INPUT_H */

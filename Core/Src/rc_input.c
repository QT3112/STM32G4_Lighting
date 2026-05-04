/**
 * @file    rc_input.c
 * @brief   Xử lý tín hiệu RC PWM Input Capture trên TIM2 CH1/CH2/CH3.
 *
 *  Mỗi kênh dùng kỹ thuật đảo cực (polarity toggle):
 *    - Bắt cạnh LÊN  → ghi thời điểm bắt đầu xung.
 *    - Bắt cạnh XUỐNG → tính độ rộng xung, lọc nhiễu, lưu vào shared var.
 *
 *  Biến shared được bảo vệ bằng __disable_irq() / __enable_irq() khi đọc
 *  từ main loop.
 */

#include "rc_input.h"

/* -------------------------------------------------------------------------- */
/* Biến nội bộ (static)                                                       */
/* -------------------------------------------------------------------------- */

/* --- Kênh 1 (đèn) --- */
static volatile uint32_t s_capture1_ch1 = 0;
static volatile uint8_t  s_firstEdge_ch1 = 0;
static volatile uint32_t s_pulseWidth_ch1 = 0;
static volatile uint32_t s_lastPulseTime_ch1 = 0;

/* --- Kênh 2 (servo 1) --- */
static volatile uint32_t s_capture1_ch2 = 0;
static volatile uint8_t  s_firstEdge_ch2 = 0;
static volatile uint32_t s_pulseWidth_ch2 = 0;
static volatile uint32_t s_lastPulseTime_ch2 = 0;

/* --- Kênh 3 (servo 2) --- */
static volatile uint32_t s_capture1_ch3 = 0;
static volatile uint8_t  s_firstEdge_ch3 = 0;
static volatile uint32_t s_pulseWidth_ch3 = 0;
static volatile uint32_t s_lastPulseTime_ch3 = 0;

/* -------------------------------------------------------------------------- */
/* Helper: tính độ rộng xung có xử lý tràn 32-bit                            */
/* -------------------------------------------------------------------------- */
static inline uint32_t calcPulse(uint32_t rise, uint32_t fall)
{
    if (fall > rise) {
        return fall - rise;
    } else {
        return (0xFFFFFFFFUL - rise) + fall + 1UL; /* Xử lý overflow */
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

void RC_Input_Init(void)
{
    /* Khởi tạo lastPulseTime = HAL_GetTick() ngay sau delay khởi động.
       Nếu để = 0, timeout sẽ kích hoạt ngay lập tức trước khi có tín hiệu. */
    uint32_t now = HAL_GetTick();
    s_lastPulseTime_ch1 = now;
    s_lastPulseTime_ch2 = now;
    s_lastPulseTime_ch3 = now;
}

void RC_Input_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* Kênh 1 */
    if (now - s_lastPulseTime_ch1 > RC_TIMEOUT_MS) {
        __disable_irq();
        s_pulseWidth_ch1 = 0;
        __enable_irq();
    }

    /* Kênh 2 */
    if (now - s_lastPulseTime_ch2 > RC_TIMEOUT_MS) {
        __disable_irq();
        s_pulseWidth_ch2 = 0;
        __enable_irq();
    }

    /* Kênh 3 */
    if (now - s_lastPulseTime_ch3 > RC_TIMEOUT_MS) {
        __disable_irq();
        s_pulseWidth_ch3 = 0;
        __enable_irq();
    }
}

uint32_t RC_Input_GetCh1(void)
{
    __disable_irq();
    uint32_t val = s_pulseWidth_ch1;
    __enable_irq();
    return val;
}

uint32_t RC_Input_GetCh2(void)
{
    __disable_irq();
    uint32_t val = s_pulseWidth_ch2;
    __enable_irq();
    return val;
}

uint32_t RC_Input_GetCh3(void)
{
    __disable_irq();
    uint32_t val = s_pulseWidth_ch3;
    __enable_irq();
    return val;
}

void RC_Input_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* ------------------------------------------------------------------ */
    /* Kênh 1                                                             */
    /* ------------------------------------------------------------------ */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (s_firstEdge_ch1 == 0) /* Cạnh LÊN */
        {
            s_capture1_ch1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            s_firstEdge_ch1 = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else /* Cạnh XUỐNG */
        {
            uint32_t fall = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            uint32_t pulse = calcPulse(s_capture1_ch1, fall);

            if (pulse >= RC_PULSE_MIN && pulse <= RC_PULSE_MAX) {
                s_pulseWidth_ch1     = pulse;
                s_lastPulseTime_ch1  = HAL_GetTick();
            }

            s_firstEdge_ch1 = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }

    /* ------------------------------------------------------------------ */
    /* Kênh 2                                                             */
    /* ------------------------------------------------------------------ */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if (s_firstEdge_ch2 == 0) /* Cạnh LÊN */
        {
            s_capture1_ch2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            s_firstEdge_ch2 = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
                                          TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else /* Cạnh XUỐNG */
        {
            uint32_t fall = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            uint32_t pulse = calcPulse(s_capture1_ch2, fall);

            if (pulse >= RC_PULSE_MIN && pulse <= RC_PULSE_MAX) {
                s_pulseWidth_ch2     = pulse;
                s_lastPulseTime_ch2  = HAL_GetTick();
            }

            s_firstEdge_ch2 = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
                                          TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }

    /* ------------------------------------------------------------------ */
    /* Kênh 3                                                             */
    /* ------------------------------------------------------------------ */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        if (s_firstEdge_ch3 == 0) /* Cạnh LÊN */
        {
            s_capture1_ch3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            s_firstEdge_ch3 = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
                                          TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else /* Cạnh XUỐNG */
        {
            uint32_t fall = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            uint32_t pulse = calcPulse(s_capture1_ch3, fall);

            if (pulse >= RC_PULSE_MIN && pulse <= RC_PULSE_MAX) {
                s_pulseWidth_ch3     = pulse;
                s_lastPulseTime_ch3  = HAL_GetTick();
            }

            s_firstEdge_ch3 = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
                                          TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

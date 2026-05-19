/**
 * @file    pid.c
 * @brief   Implementation bộ điều khiển PID với anti-windup và D-term filter.
 */

#include "pid.h"

/* ============================================================================
 *  HELPER NỘI BỘ
 * ============================================================================ */

/**
 * @brief  Giới hạn (clamp) một giá trị float trong khoảng [lo, hi].
 */
static inline float _clamp(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ============================================================================
 *  PUBLIC API
 * ============================================================================ */

void PID_Init(PID_Handle_t *hpid,
              float Kp, float Ki, float Kd,
              float out_min, float out_max)
{
    hpid->Kp      = Kp;
    hpid->Ki      = Ki;
    hpid->Kd      = Kd;
    hpid->out_min = out_min;
    hpid->out_max = out_max;

    /* Mặc định: giới hạn tích phân bằng giới hạn đầu ra */
    hpid->int_min = out_min;
    hpid->int_max = out_max;

    /* Không lọc D-term theo mặc định */
    hpid->d_filter_alpha = 0.0f;
    hpid->d_filtered     = 0.0f;

    hpid->integral   = 0.0f;
    hpid->prev_error = 0.0f;
    hpid->enabled    = 1U;
}

float PID_Compute(PID_Handle_t *hpid, float setpoint, float feedback, float dt)
{
    /* Khi bị tắt hoặc dt không hợp lệ → trả về 0 */
    if (!hpid->enabled || dt <= 0.0f)
    {
        return 0.0f;
    }

    /* ---- P-term ---- */
    float error = setpoint - feedback;
    float P = hpid->Kp * error;

    /* ---- I-term với anti-windup (clamping trực tiếp trên integral) ---- */
    hpid->integral += hpid->Ki * error * dt;
    hpid->integral  = _clamp(hpid->integral, hpid->int_min, hpid->int_max);
    float I = hpid->integral;

    /* ---- D-term với bộ lọc thông thấp tùy chọn ---- */
    float d_raw = (error - hpid->prev_error) / dt;

    /* Exponential Moving Average: d_filtered = alpha*d_filtered + (1-alpha)*d_raw
       alpha = 0 → không lọc; alpha → 1 → rất mượt nhưng chậm phản ứng */
    float alpha = hpid->d_filter_alpha;
    hpid->d_filtered = alpha * hpid->d_filtered + (1.0f - alpha) * d_raw;

    float D = hpid->Kd * hpid->d_filtered;

    hpid->prev_error = error;

    /* ---- Tổng và clamp đầu ra ---- */
    float output = P + I + D;
    return _clamp(output, hpid->out_min, hpid->out_max);
}

void PID_Reset(PID_Handle_t *hpid)
{
    hpid->integral   = 0.0f;
    hpid->prev_error = 0.0f;
    hpid->d_filtered = 0.0f;
}

void PID_SetGains(PID_Handle_t *hpid, float Kp, float Ki, float Kd)
{
    hpid->Kp = Kp;
    hpid->Ki = Ki;
    hpid->Kd = Kd;
}

void PID_SetIntegralLimits(PID_Handle_t *hpid, float int_min, float int_max)
{
    hpid->int_min = int_min;
    hpid->int_max = int_max;
}

void PID_SetDerivativeFilter(PID_Handle_t *hpid, float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 0.99f) alpha = 0.99f;
    hpid->d_filter_alpha = alpha;
}

void PID_Enable(PID_Handle_t *hpid, uint8_t enable)
{
    hpid->enabled = enable ? 1U : 0U;
    if (!hpid->enabled)
    {
        PID_Reset(hpid);  /* Reset khi tắt để tránh tích lũy sai số */
    }
}

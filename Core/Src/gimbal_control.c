/**
 * @file    gimbal_control.c
 * @brief   Gimbal 2-trục (Pitch & Yaw) – Cascaded PID + Kalman Filter.
 *
 *  Luồng dữ liệu mỗi chu kỳ (Gimbal_Update):
 *
 *  MPU6050_Update()        → raw accel/gyro
 *  Kalman_Update(pitch)    → pitch_angle (°)    [Accel + Gyro X]
 *  ─────────────────────────────────────────────────────────────
 *  PITCH AXIS (Cascaded):
 *    Angle PID:   error = pitch_setpoint - pitch_angle
 *                 output → rate_setpoint (°/s)
 *    Rate PID:    error = rate_setpoint - gyro_x (°/s)
 *                 output → pitch_offset (µs)
 *    servo_pitch = CENTER + pitch_offset   [clamp 1000–2000]
 *
 *  YAW AXIS (Rate only):
 *    Rate PID:    error = yaw_rate_sp - gyro_z (°/s)
 *                 output → yaw_offset (µs)
 *    servo_yaw   = CENTER + yaw_offset    [clamp 1000–2000]
 *  ─────────────────────────────────────────────────────────────
 *  Ghi TIM3 CH2 (Pitch), TIM3 CH3 (Yaw)
 */

#include "gimbal_control.h"
#include <math.h>   /* atan2f, sqrtf */

/* ============================================================================
 *  MACRO NỘI BỘ
 * ============================================================================ */

/** Clamp uint32 trong khoảng [lo, hi] */
static inline uint32_t _clamp_u32(float val, uint32_t lo, uint32_t hi)
{
    if (val < (float)lo) return lo;
    if (val > (float)hi) return hi;
    return (uint32_t)val;
}

/* ============================================================================
 *  PUBLIC API
 * ============================================================================ */

void Gimbal_Init(GimbalControl_Handle_t *hgimbal)
{
    /* ---- Kalman Filter – Pitch ---- */
    Kalman_Init(&hgimbal->kalman_pitch,
                0.001f,   /* Q_angle  */
                0.003f,   /* Q_bias   */
                0.03f);   /* R_measure */

    /* ---- Cascaded PID – Pitch Angle (vòng ngoài) ---- */
    PID_Init(&hgimbal->pid_pitch_angle,
             GIMBAL_PITCH_ANGLE_KP,
             GIMBAL_PITCH_ANGLE_KI,
             GIMBAL_PITCH_ANGLE_KD,
             GIMBAL_RATE_SP_MIN,   /* out = rate setpoint */
             GIMBAL_RATE_SP_MAX);
    /* Bộ lọc D nhẹ để tránh nhiễu trên góc */
    PID_SetDerivativeFilter(&hgimbal->pid_pitch_angle, 0.15f);
    /* Giới hạn integral hẹp hơn để tránh windup khi servo bị chặn cơ học */
    PID_SetIntegralLimits(&hgimbal->pid_pitch_angle, -50.0f, 50.0f);

    /* ---- Cascaded PID – Pitch Rate (vòng trong) ---- */
    PID_Init(&hgimbal->pid_pitch_rate,
             GIMBAL_PITCH_RATE_KP,
             GIMBAL_PITCH_RATE_KI,
             GIMBAL_PITCH_RATE_KD,
             GIMBAL_PID_OUT_MIN,   /* out = servo offset µs */
             GIMBAL_PID_OUT_MAX);
    PID_SetDerivativeFilter(&hgimbal->pid_pitch_rate, 0.1f);
    PID_SetIntegralLimits(&hgimbal->pid_pitch_rate, -100.0f, 100.0f);

    /* ---- Rate PID – Yaw ---- */
    PID_Init(&hgimbal->pid_yaw_rate,
             GIMBAL_YAW_RATE_KP,
             GIMBAL_YAW_RATE_KI,
             GIMBAL_YAW_RATE_KD,
             GIMBAL_PID_OUT_MIN,
             GIMBAL_PID_OUT_MAX);
    PID_SetDerivativeFilter(&hgimbal->pid_yaw_rate, 0.1f);
    PID_SetIntegralLimits(&hgimbal->pid_yaw_rate, -80.0f, 80.0f);

    /* ---- Setpoint mặc định ---- */
    hgimbal->pitch_setpoint  = 0.0f;   /* Cân bằng ngang */
    hgimbal->yaw_rate_sp     = 0.0f;   /* Lock yaw */

    /* ---- Servo về vị trí trung tâm ---- */
    hgimbal->servo_pitch_us = GIMBAL_SERVO_CENTER_US;
    hgimbal->servo_yaw_us   = GIMBAL_SERVO_CENTER_US;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, GIMBAL_SERVO_CENTER_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, GIMBAL_SERVO_CENTER_US);

    hgimbal->last_tick   = HAL_GetTick();
    hgimbal->initialized = 1U;
}

void Gimbal_Reset(GimbalControl_Handle_t *hgimbal, MPU6050_Handle_t *hMpu)
{
    /* Lấy góc hiện tại từ Accel để khởi tạo Kalman (tránh bump khi bật) */
    float ax = hMpu->scaled.accel_x;
    float ay = hMpu->scaled.accel_y;
    float az = hMpu->scaled.accel_z;

    float init_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29577951f;

    Kalman_Reset(&hgimbal->kalman_pitch, init_pitch);

    /* Reset tất cả PID để xóa tích lũy cũ */
    PID_Reset(&hgimbal->pid_pitch_angle);
    PID_Reset(&hgimbal->pid_pitch_rate);
    PID_Reset(&hgimbal->pid_yaw_rate);

    /* Servo về trung tâm */
    hgimbal->servo_pitch_us = GIMBAL_SERVO_CENTER_US;
    hgimbal->servo_yaw_us   = GIMBAL_SERVO_CENTER_US;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, GIMBAL_SERVO_CENTER_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, GIMBAL_SERVO_CENTER_US);

    hgimbal->last_tick = HAL_GetTick();
}

void Gimbal_Update(GimbalControl_Handle_t *hgimbal, MPU6050_Handle_t *hMpu)
{
    if (!hgimbal->initialized) return;

    /* ---- Tính dt (giây) ---- */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - hgimbal->last_tick) / 1000.0f;
    hgimbal->last_tick = now;

    /* Bảo vệ: dt phải hợp lệ */
    if (dt <= 0.0f || dt > 0.1f) return;

    /* ---- Lấy dữ liệu cảm biến từ MPU6050 handle ---- */
    float ax = hMpu->scaled.accel_x;
    float ay = hMpu->scaled.accel_y;
    float az = hMpu->scaled.accel_z;

    /* Góc Pitch từ Accel (đo lường nhiễu, dài hạn ổn định) */
    float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29577951f;

    /* Tốc độ góc từ Gyro (°/s) */
    float gyro_x = hMpu->scaled.gyro_x;  /* Pitch rate */
    float gyro_z = hMpu->scaled.gyro_z;  /* Yaw rate   */

    /* ================================================================
     *  PITCH AXIS – Cascaded PID
     * ================================================================ */

    /* Bước 1: Kalman Filter → ước lượng góc Pitch tối ưu */
    float pitch_angle = Kalman_Update(&hgimbal->kalman_pitch,
                                       accel_pitch,
                                       gyro_x,
                                       dt);

    /* Bước 2: Angle PID (vòng ngoài)
     *   Input:  góc mong muốn (setpoint) vs góc thực (Kalman output)
     *   Output: tốc độ góc mong muốn (rate_setpoint) */
    float pitch_rate_sp = PID_Compute(&hgimbal->pid_pitch_angle,
                                       hgimbal->pitch_setpoint,
                                       pitch_angle,
                                       dt);

    /* Bước 3: Rate PID (vòng trong)
     *   Input:  tốc độ góc mong muốn (từ Angle PID) vs tốc độ góc thực (Gyro)
     *   Output: offset servo (µs) */
    float pitch_output = PID_Compute(&hgimbal->pid_pitch_rate,
                                      pitch_rate_sp,
                                      gyro_x,
                                      dt);

    /* Bước 4: Ghi servo Pitch = CENTER + offset */
    float servo_pitch_f = (float)GIMBAL_SERVO_CENTER_US + pitch_output;
    hgimbal->servo_pitch_us = _clamp_u32(servo_pitch_f,
                                          GIMBAL_SERVO_MIN_US,
                                          GIMBAL_SERVO_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, hgimbal->servo_pitch_us);

    /* ================================================================
     *  YAW AXIS – Rate PID đơn (không có Accel reference cho Yaw)
     * ================================================================ */

    /* Rate PID:  error = yaw_rate_sp - gyro_z
     *   setpoint = 0 → giữ nguyên hướng (lock yaw)
     *   Gyro Z là chiều quay ngang của gimbal                          */
    float yaw_output = PID_Compute(&hgimbal->pid_yaw_rate,
                                    hgimbal->yaw_rate_sp,
                                    gyro_z,
                                    dt);

    float servo_yaw_f = (float)GIMBAL_SERVO_CENTER_US + yaw_output;
    hgimbal->servo_yaw_us = _clamp_u32(servo_yaw_f,
                                        GIMBAL_SERVO_MIN_US,
                                        GIMBAL_SERVO_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, hgimbal->servo_yaw_us);
}

void Gimbal_SetPitchSetpoint(GimbalControl_Handle_t *hgimbal, float pitch_deg)
{
    hgimbal->pitch_setpoint = pitch_deg;
}

void Gimbal_SetYawRateSetpoint(GimbalControl_Handle_t *hgimbal, float yaw_rate_dps)
{
    hgimbal->yaw_rate_sp = yaw_rate_dps;
}

void Gimbal_TunePitchPID(GimbalControl_Handle_t *hgimbal,
                          float angle_Kp, float angle_Ki, float angle_Kd,
                          float rate_Kp,  float rate_Ki,  float rate_Kd)
{
    PID_SetGains(&hgimbal->pid_pitch_angle, angle_Kp, angle_Ki, angle_Kd);
    PID_SetGains(&hgimbal->pid_pitch_rate,  rate_Kp,  rate_Ki,  rate_Kd);
}

void Gimbal_TuneYawPID(GimbalControl_Handle_t *hgimbal,
                        float Kp, float Ki, float Kd)
{
    PID_SetGains(&hgimbal->pid_yaw_rate, Kp, Ki, Kd);
}

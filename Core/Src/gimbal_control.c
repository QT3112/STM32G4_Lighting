/**
 * @file    gimbal_control.c
 * @brief   Gimbal 2 trục — Dual IMU, Complementary Filter, Cascaded PID,
 *          Feedforward, 500Hz flag-based control loop.
 *
 *  Control flow (mỗi Gimbal_Tick() @ 500Hz):
 *
 *  1. Đọc cả 2 IMU qua I2C3 (blocking, ~400µs)
 *  2. Complementary Filter cho từng IMU → góc Pitch, Roll
 *  3. Tính relative error:
 *       err_pitch = (cam_pitch - frame_pitch) - setpoint_pitch
 *       err_yaw   = (cam_gyro_z - frame_gyro_z) - setpoint_yaw_rate
 *  4. Pitch Cascaded PID:
 *       rate_sp = AnglePID(err_pitch)
 *       offset  = RatePID(rate_sp - cam_gyro_x) + Kff × frame_gyro_x
 *  5. Yaw Rate PID:
 *       offset  = RatePID(err_yaw) + Kff × frame_gyro_z
 *  6. Servo output:
 *       CCR = SERVO_CENTER ± SIGN × offset (clamp 500–2500µs)
 *
 *  Notes:
 *  - Không gọi printf() từ Gimbal_Tick() (ISR-adjacent, không an toàn)
 *  - Telemetry được copy vào struct và in từ Gimbal_Update() ở main loop
 */

#include "gimbal_control.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* ===========================================================================
 *  PRIVATE HELPERS
 * =========================================================================== */

/** Clamp float trong khoảng [lo, hi] */
static inline float _clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/** Clamp và convert sang uint32_t cho CCR register */
static inline uint32_t _servo_clamp(float us)
{
    if (us < (float)GIMBAL_SERVO_MIN_US) return GIMBAL_SERVO_MIN_US;
    if (us > (float)GIMBAL_SERVO_MAX_US) return GIMBAL_SERVO_MAX_US;
    return (uint32_t)us;
}

/* ===========================================================================
 *  PUBLIC API
 * =========================================================================== */

void Gimbal_Init(GimbalControl_Handle_t *hg, ImuDual_Handle_t *hImu)
{
    memset(hg, 0, sizeof(GimbalControl_Handle_t));

    /* ---- Complementary Filters ---- */
    CompFilter_Init(&hg->cf_frame,  GIMBAL_CF_ALPHA);
    CompFilter_Init(&hg->cf_camera, GIMBAL_CF_ALPHA);

    /* ---- Pitch Angle PID (outer loop) ----
     *  Input:  relative pitch angle error (°)
     *  Output: desired pitch rate (°/s) → inner loop setpoint
     */
    PID_Init(&hg->pid_pitch_angle,
             GIMBAL_PITCH_ANGLE_KP,
             GIMBAL_PITCH_ANGLE_KI,
             GIMBAL_PITCH_ANGLE_KD,
             GIMBAL_RATE_SP_MIN,
             GIMBAL_RATE_SP_MAX);
    PID_SetDerivativeFilter(&hg->pid_pitch_angle, GIMBAL_D_FILTER_ANGLE);
    PID_SetIntegralLimits(&hg->pid_pitch_angle,
                          -GIMBAL_INT_LIMIT_ANGLE, GIMBAL_INT_LIMIT_ANGLE);

    /* ---- Pitch Rate PID (inner loop) ----
     *  Input:  rate error (°/s)
     *  Output: servo offset (µs)
     */
    PID_Init(&hg->pid_pitch_rate,
             GIMBAL_PITCH_RATE_KP,
             GIMBAL_PITCH_RATE_KI,
             GIMBAL_PITCH_RATE_KD,
             GIMBAL_PID_OUT_MIN,
             GIMBAL_PID_OUT_MAX);
    PID_SetDerivativeFilter(&hg->pid_pitch_rate, GIMBAL_D_FILTER_RATE);
    PID_SetIntegralLimits(&hg->pid_pitch_rate,
                          -GIMBAL_INT_LIMIT_RATE, GIMBAL_INT_LIMIT_RATE);

    /* ---- Yaw Rate PID (single loop) ----
     *  Input:  relative yaw rate error (°/s)
     *  Output: servo offset (µs)
     *  setpoint_yaw_rate = 0 → lock yaw (kháng lại xoay ngang)
     */
    PID_Init(&hg->pid_yaw_rate,
             GIMBAL_YAW_RATE_KP,
             GIMBAL_YAW_RATE_KI,
             GIMBAL_YAW_RATE_KD,
             GIMBAL_PID_OUT_MIN,
             GIMBAL_PID_OUT_MAX);
    PID_SetDerivativeFilter(&hg->pid_yaw_rate, GIMBAL_D_FILTER_RATE);
    PID_SetIntegralLimits(&hg->pid_yaw_rate,
                          -GIMBAL_INT_LIMIT_RATE, GIMBAL_INT_LIMIT_RATE);

    /* ---- Setpoints & Feedforward ---- */
    hg->pitch_setpoint     = 0.0f;
    hg->yaw_rate_setpoint  = 0.0f;
    hg->kff_pitch          = GIMBAL_KFF_PITCH;
    hg->kff_yaw            = GIMBAL_KFF_YAW;

    /* ---- Servo về center ---- */
    hg->servo_pitch_us = GIMBAL_SERVO_CENTER_US;
    hg->servo_yaw_us   = GIMBAL_SERVO_CENTER_US;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, GIMBAL_SERVO_CENTER_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, GIMBAL_SERVO_CENTER_US);

    hg->last_tick   = HAL_GetTick();
    hg->initialized = 1;

    /* Init filter với dữ liệu IMU hiện tại (nếu IMU đã init) */
    if (hImu->frame.initialized && hImu->camera.initialized) {
        ImuDual_Read(hImu);
        CompFilter_Reset(&hg->cf_frame,
                         hImu->frame.scaled.accel_x,
                         hImu->frame.scaled.accel_y,
                         hImu->frame.scaled.accel_z);
        CompFilter_Reset(&hg->cf_camera,
                         hImu->camera.scaled.accel_x,
                         hImu->camera.scaled.accel_y,
                         hImu->camera.scaled.accel_z);
    }

    printf("[GIMB] Init OK. Pitch SP=%.1f° | CF alpha=%.3f\r\n",
           hg->pitch_setpoint, GIMBAL_CF_ALPHA);
}

void Gimbal_Tick(GimbalControl_Handle_t *hg, ImuDual_Handle_t *hImu)
{
    if (!hg->initialized) return;

    /* ====================================================================
     *  Bước 1: Đo dt thực tế
     *  Mặc dù TIM6 kích hoạt đều đặn 2ms, đo dt thực để PID chính xác
     *  trong trường hợp main loop có jitter nhỏ.
     * ==================================================================== */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - hg->last_tick) * 0.001f;
    hg->last_tick = now;

    /* Bảo vệ: nếu dt quá lớn (hệ thống bị block), skip tick này */
    if (dt <= 0.0001f || dt > GIMBAL_CTRL_DT_MAX) {
        return;
    }

    /* ====================================================================
     *  Bước 2: Đọc cả 2 IMU
     * ==================================================================== */
    if (ImuDual_Read(hImu) != HAL_OK) {
        /* Nếu I2C fail, giữ nguyên servo ở vị trí cuối */
        return;
    }

    /* Shortcut aliases */
    ImuScaled_t *fr = &hImu->frame.scaled;
    ImuScaled_t *ca = &hImu->camera.scaled;

    /* ====================================================================
     *  Bước 3: Complementary Filter cho từng IMU
     *  cf_frame: pitch và roll của frame/body
     *  cf_camera: pitch và roll của camera/platform
     * ==================================================================== */
    CompFilter_Update(&hg->cf_frame,
                      fr->accel_x, fr->accel_y, fr->accel_z,
                      fr->gyro_x, fr->gyro_y,
                      dt);

    CompFilter_Update(&hg->cf_camera,
                      ca->accel_x, ca->accel_y, ca->accel_z,
                      ca->gyro_x, ca->gyro_y,
                      dt);

    /* ====================================================================
     *  Bước 4: Relative Error — Tim hiệu giữa camera và frame
     *
     *  err_pitch = camera_pitch - frame_pitch - setpoint_pitch
     *
     *  Diễn giải:
     *  - Nếu frame nghiêng +10° và camera vẫn ở 0°  → err = 0 - 10 - 0 = -10
     *    → Cần servo compensate +10° để camera luôn ở 0° tuyệt đối
     *  - Nếu setpoint = +5° (camera hướng xuống 5°)
     *    → err = camera_pitch - frame_pitch - 5
     * ==================================================================== */
    float err_pitch = (hg->cf_camera.pitch - hg->cf_frame.pitch)
                      - hg->pitch_setpoint;

    /* Yaw: dùng gyro rate (không có accel reference cho yaw)
     * err_yaw = (camera_gyro_z - frame_gyro_z) - setpoint_yaw_rate
     * setpoint_yaw_rate = 0 → lock yaw (kháng lại xoay ngang của frame) */
    float err_yaw_rate = (ca->gyro_z - fr->gyro_z) - hg->yaw_rate_setpoint;

    /* ====================================================================
     *  Bước 5: PITCH CASCADED PID
     *
     *  Vòng ngoài (Angle PID):
     *    Input:  err_pitch (°)
     *    Output: rate_setpoint (°/s) — muốn camera xoay bao nhanh
     *
     *  Vòng trong (Rate PID):
     *    Input:  rate_setpoint - camera_actual_pitch_rate
     *    Output: servo_offset (µs)
     * ==================================================================== */
    float pitch_rate_sp = PID_Compute(&hg->pid_pitch_angle,
                                       0.0f, -err_pitch,  /* error = 0 - (-err) */
                                       dt);

    /* Rate feedback là gyro của camera (tốc độ góc thực của platform) */
    float pitch_servo_offset = PID_Compute(&hg->pid_pitch_rate,
                                            pitch_rate_sp,
                                            ca->gyro_x,
                                            dt);

    /* Feedforward: bù ngay từ frame gyro, không đợi PID phản ứng */
    pitch_servo_offset += hg->kff_pitch * fr->gyro_x;

    /* ====================================================================
     *  Bước 6: YAW RATE PID (single loop)
     * ==================================================================== */
    float yaw_servo_offset = PID_Compute(&hg->pid_yaw_rate,
                                          0.0f, -err_yaw_rate,
                                          dt);

    yaw_servo_offset += hg->kff_yaw * fr->gyro_z;

    /* ====================================================================
     *  Bước 7: Servo Output
     *
     *  servo_us = CENTER + SIGN × offset
     *  Clamp vào [SERVO_MIN, SERVO_MAX]
     * ==================================================================== */
    float p_us = (float)GIMBAL_SERVO_CENTER_US
                 + GIMBAL_SERVO_PITCH_SIGN * pitch_servo_offset;
    float y_us = (float)GIMBAL_SERVO_CENTER_US
                 + GIMBAL_SERVO_YAW_SIGN * yaw_servo_offset;

    hg->servo_pitch_us = _servo_clamp(p_us);
    hg->servo_yaw_us   = _servo_clamp(y_us);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, hg->servo_pitch_us);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, hg->servo_yaw_us);

    /* ====================================================================
     *  Bước 8: Copy telemetry (atomic copy cho main loop logging)
     * ==================================================================== */
    hg->telem.frame_pitch    = hg->cf_frame.pitch;
    hg->telem.frame_roll     = hg->cf_frame.roll;
    hg->telem.cam_pitch      = hg->cf_camera.pitch;
    hg->telem.cam_roll       = hg->cf_camera.roll;
    hg->telem.err_pitch      = err_pitch;
    hg->telem.err_yaw_rate   = err_yaw_rate;
    hg->telem.servo_pitch_us = (float)hg->servo_pitch_us;
    hg->telem.servo_yaw_us   = (float)hg->servo_yaw_us;
    hg->telem.frame_gyro_z   = fr->gyro_z;
    hg->telem.loop_count++;
}

void Gimbal_Update(GimbalControl_Handle_t *hg)
{
    /* Hàm này gọi từ main loop — in telemetry mỗi 200ms
     * (không dùng HAL_Delay — dùng timestamp) */
    static uint32_t last_log = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_log >= 200) {
        last_log = now;
        /* Snapshot telemetry (không cần atomic vì float copy trong Cortex-M4) */
        GimbalTelemetry_t t = hg->telem;
        printf("[GIMB] F_P:%.1f F_R:%.1f | C_P:%.1f C_R:%.1f "
               "| eP:%.2f eY:%.2f | SrvP:%.0f SrvY:%.0f | N:%lu\r\n",
               t.frame_pitch, t.frame_roll,
               t.cam_pitch,   t.cam_roll,
               t.err_pitch,   t.err_yaw_rate,
               t.servo_pitch_us, t.servo_yaw_us,
               t.loop_count);
    }
}

void Gimbal_Reset(GimbalControl_Handle_t *hg, ImuDual_Handle_t *hImu)
{
    PID_Reset(&hg->pid_pitch_angle);
    PID_Reset(&hg->pid_pitch_rate);
    PID_Reset(&hg->pid_yaw_rate);

    if (hImu->frame.initialized && hImu->camera.initialized) {
        ImuDual_Read(hImu);
        CompFilter_Reset(&hg->cf_frame,
                         hImu->frame.scaled.accel_x,
                         hImu->frame.scaled.accel_y,
                         hImu->frame.scaled.accel_z);
        CompFilter_Reset(&hg->cf_camera,
                         hImu->camera.scaled.accel_x,
                         hImu->camera.scaled.accel_y,
                         hImu->camera.scaled.accel_z);
    }

    hg->servo_pitch_us = GIMBAL_SERVO_CENTER_US;
    hg->servo_yaw_us   = GIMBAL_SERVO_CENTER_US;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, GIMBAL_SERVO_CENTER_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, GIMBAL_SERVO_CENTER_US);

    hg->last_tick = HAL_GetTick();
    printf("[GIMB] Reset done.\r\n");
}

/* ---- Setpoint ---- */
void Gimbal_SetPitchDeg(GimbalControl_Handle_t *hg, float deg) {
    hg->pitch_setpoint = _clampf(deg, -60.0f, 60.0f);
}
void Gimbal_SetYawRateDps(GimbalControl_Handle_t *hg, float dps) {
    hg->yaw_rate_setpoint = _clampf(dps, -90.0f, 90.0f);
}

/* ---- Tuning ---- */
void Gimbal_TunePitch(GimbalControl_Handle_t *hg,
                      float aKp, float aKi, float aKd,
                      float rKp, float rKi, float rKd)
{
    PID_SetGains(&hg->pid_pitch_angle, aKp, aKi, aKd);
    PID_SetGains(&hg->pid_pitch_rate,  rKp, rKi, rKd);
}
void Gimbal_TuneYaw(GimbalControl_Handle_t *hg, float Kp, float Ki, float Kd) {
    PID_SetGains(&hg->pid_yaw_rate, Kp, Ki, Kd);
}
void Gimbal_SetFeedforward(GimbalControl_Handle_t *hg,
                            float kff_pitch, float kff_yaw)
{
    hg->kff_pitch = kff_pitch;
    hg->kff_yaw   = kff_yaw;
}
void Gimbal_SetFilterAlpha(GimbalControl_Handle_t *hg, float alpha) {
    CompFilter_SetAlpha(&hg->cf_frame,  alpha);
    CompFilter_SetAlpha(&hg->cf_camera, alpha);
}

/* ---- CLI ---- */
void Gimbal_CLI_Process(GimbalControl_Handle_t *hg, const char *line)
{
    if (!line || line[0] == '\0') return;

    float a, b, c, d, e, f;

    switch (line[0]) {
    case 'p':  /* p Kp Ki Kd — pitch angle PID */
        if (sscanf(line + 1, "%f %f %f", &a, &b, &c) == 3) {
            PID_SetGains(&hg->pid_pitch_angle, a, b, c);
            printf("[CLI] Pitch Angle PID: Kp=%.3f Ki=%.3f Kd=%.3f\r\n", a, b, c);
        }
        break;
    case 'P':  /* P Kp Ki Kd — pitch rate PID */
        if (sscanf(line + 1, "%f %f %f", &a, &b, &c) == 3) {
            PID_SetGains(&hg->pid_pitch_rate, a, b, c);
            printf("[CLI] Pitch Rate PID: Kp=%.3f Ki=%.3f Kd=%.3f\r\n", a, b, c);
        }
        break;
    case 'y':  /* y Kp Ki Kd — yaw rate PID */
        if (sscanf(line + 1, "%f %f %f", &a, &b, &c) == 3) {
            PID_SetGains(&hg->pid_yaw_rate, a, b, c);
            printf("[CLI] Yaw Rate PID: Kp=%.3f Ki=%.3f Kd=%.3f\r\n", a, b, c);
        }
        break;
    case 'f':  /* f kff_pitch kff_yaw — feedforward */
        if (sscanf(line + 1, "%f %f", &a, &b) == 2) {
            Gimbal_SetFeedforward(hg, a, b);
            printf("[CLI] Feedforward: kff_pitch=%.3f kff_yaw=%.3f\r\n", a, b);
        }
        break;
    case 'a':  /* a alpha — comp filter alpha */
        if (sscanf(line + 1, "%f", &a) == 1) {
            Gimbal_SetFilterAlpha(hg, a);
            printf("[CLI] CompFilter alpha=%.4f\r\n", a);
        }
        break;
    case 's':  /* s pitch_deg — set pitch setpoint */
        if (sscanf(line + 1, "%f", &a) == 1) {
            Gimbal_SetPitchDeg(hg, a);
            printf("[CLI] Pitch setpoint=%.2f°\r\n", hg->pitch_setpoint);
        }
        break;
    case 'r':  /* r — reset PIDs (không reset filter) */
        PID_Reset(&hg->pid_pitch_angle);
        PID_Reset(&hg->pid_pitch_rate);
        PID_Reset(&hg->pid_yaw_rate);
        printf("[CLI] PIDs reset.\r\n");
        break;
    case 'd':  /* d — dump current state */
        printf("[CLI] Pitch SP=%.2f | CF_alpha=%.3f | kff_p=%.3f kff_y=%.3f\r\n",
               hg->pitch_setpoint, hg->cf_frame.alpha,
               hg->kff_pitch, hg->kff_yaw);
        printf("[CLI] Pitch AnglePID: Kp=%.3f Ki=%.3f Kd=%.3f\r\n",
               hg->pid_pitch_angle.Kp, hg->pid_pitch_angle.Ki,
               hg->pid_pitch_angle.Kd);
        printf("[CLI] Pitch RatePID:  Kp=%.3f Ki=%.3f Kd=%.3f\r\n",
               hg->pid_pitch_rate.Kp, hg->pid_pitch_rate.Ki,
               hg->pid_pitch_rate.Kd);
        printf("[CLI] Yaw  RatePID:  Kp=%.3f Ki=%.3f Kd=%.3f\r\n",
               hg->pid_yaw_rate.Kp, hg->pid_yaw_rate.Ki,
               hg->pid_yaw_rate.Kd);
        printf("[CLI] Loop count: %lu\r\n", (uint32_t)hg->telem.loop_count);
        break;
    default:
        printf("[CLI] Unknown: '%c'. Commands: p P y f a s r d\r\n", line[0]);
        break;
    }

    (void)(d); (void)(e); (void)(f);  /* suppress unused warning */
}

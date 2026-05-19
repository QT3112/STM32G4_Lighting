/**
 * @file    gimbal_control.h
 * @brief   Module điều khiển Gimbal 2 trục (Pitch & Yaw) sử dụng Cascaded PID.
 *
 *  Kiến trúc Cascaded PID cho mỗi trục:
 *
 *    Setpoint (0°)                                           Servo CCR (µs)
 *        │                                                        ▲
 *        ▼                                                        │
 *   ┌──────────┐  rate_sp (°/s)   ┌──────────┐  raw_output      │
 *   │ANGLE PID │ ───────────────► │ RATE PID │ ─────────────────►│
 *   └──────────┘                  └──────────┘
 *        ▲                              ▲
 *        │ angle (°)                    │ gyro_rate (°/s)
 *    Kalman Filter                 MPU6050 Gyro
 *        ▲
 *        │ accel_angle + gyro_rate
 *    MPU6050 Accel + Gyro
 *
 *  Trục Pitch:  Servo 1 (TIM3 CH2) – ứng với góc gật gù (Pitch = Accel/Gyro X)
 *  Trục Yaw:    Servo 2 (TIM3 CH3) – ứng với góc xoay ngang (Yaw = Gyro Z)
 *               (Yaw không thể tính từ Accel → chỉ dùng Rate PID thuần)
 *
 *  Kích hoạt Gimbal Mode:
 *    Bật tắt đèn (toggle ON-zone) 5 lần trong GESTURE_WINDOW_MS ms.
 *    Cơ chế phát hiện tích hợp trong mode_manager.c.
 */

#ifndef __GIMBAL_CONTROL_H__
#define __GIMBAL_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"
#include "kalman.h"
#include "mpu6050.h"
#include "tim.h"

/* ============================================================================
 *  THAM SỐ CẤU HÌNH MẶC ĐỊNH
 * ============================================================================ */

/** Servo output base (µs) – điểm cân bằng cơ học (90°) */
#define GIMBAL_SERVO_CENTER_US   1500U

/** Giới hạn dải servo output (µs) */
#define GIMBAL_SERVO_MIN_US      1000U
#define GIMBAL_SERVO_MAX_US      2000U

/** Giới hạn rate setpoint từ Angle PID → Rate PID (°/s)
 *  Servo gimbal không cần tốc độ quá 150 °/s */
#define GIMBAL_RATE_SP_MAX        150.0f
#define GIMBAL_RATE_SP_MIN       -150.0f

/** Giới hạn output của Rate PID (µs – offset từ center) */
#define GIMBAL_PID_OUT_MAX        400.0f
#define GIMBAL_PID_OUT_MIN       -400.0f

/* ============================================================================
 *  HẰNG SỐ PID MẶC ĐỊNH (điểm khởi đầu, cần chỉnh thực tế)
 * ============================================================================ */

/* --- Pitch Angle PID --- */
#define GIMBAL_PITCH_ANGLE_KP    4.0f
#define GIMBAL_PITCH_ANGLE_KI    0.05f
#define GIMBAL_PITCH_ANGLE_KD    0.1f

/* --- Pitch Rate PID --- */
#define GIMBAL_PITCH_RATE_KP     2.0f
#define GIMBAL_PITCH_RATE_KI     0.2f
#define GIMBAL_PITCH_RATE_KD     0.02f

/* --- Yaw Rate PID (chỉ Rate PID vì Yaw không có Accel reference) --- */
#define GIMBAL_YAW_RATE_KP       1.5f
#define GIMBAL_YAW_RATE_KI       0.1f
#define GIMBAL_YAW_RATE_KD       0.01f

/* ============================================================================
 *  CẤU TRÚC DỮ LIỆU
 * ============================================================================ */

/**
 * @brief  Handle chính của Gimbal Controller.
 */
typedef struct {
    /* --- Bộ lọc Kalman (1 instance / trục) --- */
    Kalman_Handle_t kalman_pitch;  /*!< Kalman Filter cho trục Pitch */

    /* --- Cascaded PID cho Pitch --- */
    PID_Handle_t pid_pitch_angle;  /*!< Vòng ngoài: Angle PID (góc → rate_sp) */
    PID_Handle_t pid_pitch_rate;   /*!< Vòng trong: Rate PID  (rate_sp → servo) */

    /* --- Rate PID cho Yaw (chỉ 1 vòng) --- */
    PID_Handle_t pid_yaw_rate;     /*!< Rate PID: Yaw rate → servo             */

    /* --- Setpoint góc mong muốn (°) --- */
    float pitch_setpoint;  /*!< Mặc định 0.0° (cân bằng ngang) */
    float yaw_rate_sp;     /*!< Tốc độ quay yaw mong muốn (°/s), 0 = giữ nguyên */

    /* --- Đầu ra servo (µs) lưu để debug --- */
    uint32_t servo_pitch_us;  /*!< Giá trị CCR Servo Pitch (TIM3 CH2) */
    uint32_t servo_yaw_us;    /*!< Giá trị CCR Servo Yaw   (TIM3 CH3) */

    /* --- Thời gian chu kỳ --- */
    uint32_t last_tick;  /*!< HAL_GetTick() của lần cập nhật cuối */

    /* --- Trạng thái --- */
    uint8_t initialized;  /*!< 1 = đã init, 0 = chưa */
} GimbalControl_Handle_t;

/* ============================================================================
 *  PUBLIC API
 * ============================================================================ */

/**
 * @brief  Khởi tạo Gimbal Controller.
 *         Cài đặt Kalman Filter và tất cả bộ PID với tham số mặc định.
 *         Gọi sau MPU6050_Init() và Servo_Init().
 *
 * @param  hgimbal  Con trỏ handle Gimbal.
 */
void Gimbal_Init(GimbalControl_Handle_t *hgimbal);

/**
 * @brief  Cập nhật một chu kỳ điều khiển Gimbal.
 *         Đọc dữ liệu từ MPU6050 → Kalman Filter → Cascaded PID → ghi servo.
 *         Gọi trong main loop khi đang ở APP_MODE_GIMBAL.
 *
 * @param  hgimbal  Con trỏ handle Gimbal.
 * @param  hMpu     Con trỏ handle MPU6050 (đã gọi MPU6050_Update() trước đó).
 */
void Gimbal_Update(GimbalControl_Handle_t *hgimbal, MPU6050_Handle_t *hMpu);

/**
 * @brief  Reset tất cả PID và Kalman về trạng thái ban đầu.
 *         Gọi khi bật Gimbal Mode để tránh output đột ngột.
 *
 * @param  hgimbal  Con trỏ handle Gimbal.
 * @param  hMpu     Con trỏ handle MPU6050 để lấy góc khởi tạo.
 */
void Gimbal_Reset(GimbalControl_Handle_t *hgimbal, MPU6050_Handle_t *hMpu);

/**
 * @brief  Đặt setpoint góc Pitch (°).
 *         Mặc định = 0.0° (cân bằng hoàn toàn ngang).
 *         Cho phép bù gimbal ở góc tilt cố định nếu cần.
 *
 * @param  hgimbal        Con trỏ handle Gimbal.
 * @param  pitch_deg      Góc Pitch mong muốn (°).
 */
void Gimbal_SetPitchSetpoint(GimbalControl_Handle_t *hgimbal, float pitch_deg);

/**
 * @brief  Đặt setpoint tốc độ Yaw (°/s).
 *         0 = giữ nguyên hướng (lock Yaw). ≠ 0 = xoay chậm.
 *
 * @param  hgimbal       Con trỏ handle Gimbal.
 * @param  yaw_rate_dps  Tốc độ quay yaw mong muốn (°/s).
 */
void Gimbal_SetYawRateSetpoint(GimbalControl_Handle_t *hgimbal, float yaw_rate_dps);

/**
 * @brief  Chỉnh hệ số PID Pitch (cả Angle lẫn Rate) trong khi chạy.
 *
 * @param  hgimbal       Con trỏ handle Gimbal.
 * @param  angle_Kp/Ki/Kd  Hệ số Angle PID mới.
 * @param  rate_Kp/Ki/Kd   Hệ số Rate PID mới.
 */
void Gimbal_TunePitchPID(GimbalControl_Handle_t *hgimbal,
                          float angle_Kp, float angle_Ki, float angle_Kd,
                          float rate_Kp,  float rate_Ki,  float rate_Kd);

/**
 * @brief  Chỉnh hệ số PID Yaw (Rate PID) trong khi chạy.
 *
 * @param  hgimbal      Con trỏ handle Gimbal.
 * @param  Kp, Ki, Kd   Hệ số PID mới.
 */
void Gimbal_TuneYawPID(GimbalControl_Handle_t *hgimbal,
                        float Kp, float Ki, float Kd);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_CONTROL_H__ */

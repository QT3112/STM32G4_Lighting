/**
 * @file    gimbal_control.h
 * @brief   Gimbal 2 trục (Pitch + Yaw) — Dual IMU + Complementary Filter
 *          + Cascaded PID + Feedforward + 500Hz timer-triggered control loop.
 *
 * ==========================================================================
 *  KIẾN TRÚC ĐIỀU KHIỂN
 * ==========================================================================
 *
 *  IMU Frame (0x68)          IMU Camera (0x69)
 *       │                           │
 *  CompFilter                  CompFilter
 *  frame_pitch, frame_roll    cam_pitch, cam_roll
 *       │                           │
 *       └──────── RELATIVE ERROR ───┘
 *         err_pitch = cam_pitch - frame_pitch - setpoint_pitch
 *         err_yaw   = cam_yaw_rate - frame_yaw_rate - setpoint_yaw_rate
 *                          │
 *              FEEDFORWARD (từ frame gyro)
 *               ff_pitch = Kff_pitch × frame_gyro_x
 *               ff_yaw   = Kff_yaw   × frame_gyro_z
 *                          │
 *         ┌────────────────┴────────────────┐
 *    PITCH CASCADED PID                YAW RATE PID
 *    Angle PID → rate_sp           Rate PID → servo_offset
 *    Rate PID  → servo_offset
 *         │                                  │
 *    TIM3_CH2 (Servo Pitch)          TIM3_CH3 (Servo Yaw)
 *    Range: 500–2500µs               Range: 500–2500µs
 *
 * ==========================================================================
 *  TẠI SAO DÙNG 2 IMU — RELATIVE STABILIZATION
 * ==========================================================================
 *
 *  Với 1 IMU (camera): PID phải đợi camera bị lệch rồi mới phản ứng.
 *   → Latency = I2C read + PID compute + servo slew time
 *
 *  Với 2 IMU: Biết frame đang rung trước khi camera bị lệch.
 *   → Feedforward từ frame gyro phản ứng NGAY LẬP TỨC (không cần đợi error)
 *   → PID chỉ bù phần dư còn lại → hệ thống ổn định hơn và nhanh hơn
 *
 *  Error tương đối:
 *    Nếu frame nghiêng +10° và camera giữ được 0° → error = 0-10 = -10° (cần bù)
 *    Servo phải tạo ra +10° để camera vẫn ở 0° tuyệt đối.
 *
 * ==========================================================================
 *  FEEDFORWARD — CƠ CHẾ PHẢN ỨNG SỚM
 * ==========================================================================
 *
 *  Feedforward dùng gyro rate của frame (không phải angle) vì:
 *  - Gyro không có drift trong ngắn hạn → tin cậy hơn angle estimate
 *  - Rate → tức thời, không cần tích phân → phản ứng nhanh hơn 1 bước dt
 *
 *  output = PID_output + Kff × frame_gyro_rate
 *
 *  Tuning Kff:
 *  - Kff = 0.0: không feedforward (chỉ feedback)
 *  - Kff quá cao: servo giật khi frame rung nhẹ
 *  - Kff lý tưởng: servo bù đúng bằng chuyển động của frame
 * ==========================================================================
 *
 * ==========================================================================
 *  CONTROL LOOP @ 500Hz
 * ==========================================================================
 *
 *  Cơ chế:
 *    TIM6 ISR (500Hz) → set flag g_gimbal_tick = 1
 *    Main loop:         check flag → gọi Gimbal_Tick()
 *
 *  Tại sao dùng flag thay vì gọi thẳng từ ISR?
 *  - I2C blocking read (~400µs) không hoạt động trong ISR nếu I2C interrupt
 *    có priority thấp hơn TIM6 → deadlock
 *  - Flag approach: ISR nhẹ (chỉ set 1 bit), main loop làm việc nặng
 *  - Jitter < 10µs vì main loop rất nhanh khi không có delay
 *
 * ==========================================================================
 *  SERVO PWM: 500–2500µs (Full Hobby Range)
 * ==========================================================================
 *
 *  TIM3, Prescaler=169, F_tim=1MHz, Period=19999 → 50Hz servo
 *  CCR value = pulse width in µs (1µs resolution)
 *  Center: 1500µs, Min: 500µs, Max: 2500µs → ±1000µs range
 *
 *  Angle mapping:
 *    -90°  → 500µs
 *      0°  → 1500µs (center)
 *    +90°  → 2500µs
 * ==========================================================================
 */

#ifndef __GIMBAL_CONTROL_H__
#define __GIMBAL_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"
#include "fusion.h"
#include "imu_dual.h"
#include "tim.h"

/* ===========================================================================
 *  SERVO CONFIGURATION
 * =========================================================================== */

#define GIMBAL_SERVO_MIN_US     500U   /*!< Giới hạn cuối min (−90°)     */
#define GIMBAL_SERVO_CENTER_US  1500U  /*!< Vị trí trung tâm (0°)        */
#define GIMBAL_SERVO_MAX_US     2500U  /*!< Giới hạn cuối max (+90°)     */

/* Nếu servo quay ngược chiều mong muốn, đặt -1.0f */
#define GIMBAL_SERVO_PITCH_SIGN  1.0f  /*!< +1 = không đảo, -1 = đảo    */
#define GIMBAL_SERVO_YAW_SIGN    1.0f

/* ===========================================================================
 *  CONTROL LOOP
 * =========================================================================== */

#define GIMBAL_CTRL_DT          0.002f  /*!< Fixed dt = 2ms (500Hz)       */
#define GIMBAL_CTRL_DT_MAX      0.01f   /*!< Dt tối đa cho phép (10ms)    */

/* ===========================================================================
 *  COMPLEMENTARY FILTER
 * =========================================================================== */

/** α = 0.98: time constant ≈ 0.098s @ 500Hz */
#define GIMBAL_CF_ALPHA         0.98f

/* ===========================================================================
 *  PID DEFAULTS (điểm khởi đầu — cần tuning thực tế)
 *
 *  Tuning guide:
 *    1. Đặt Ki=0, Kd=0. Tăng Kp cho đến khi servo phản ứng rõ nhưng chưa dao động.
 *    2. Tăng nhỏ Ki (0.01–0.05) để loại steady-state error.
 *    3. Thêm Kd nhỏ (0.005–0.02) + D filter để giảm overshoot.
 *    4. Tuning Feedforward Kff riêng: tăng đến khi frame rung được bù tốt.
 * =========================================================================== */

/* --- Pitch Angle PID (outer loop: angle → rate_setpoint) --- */
#define GIMBAL_PITCH_ANGLE_KP   3.5f
#define GIMBAL_PITCH_ANGLE_KI   0.02f
#define GIMBAL_PITCH_ANGLE_KD   0.05f

/* --- Pitch Rate PID (inner loop: rate_setpoint → servo offset µs) --- */
#define GIMBAL_PITCH_RATE_KP    1.8f
#define GIMBAL_PITCH_RATE_KI    0.15f
#define GIMBAL_PITCH_RATE_KD    0.015f

/* --- Yaw Rate PID (single loop: yaw rate error → servo offset µs) --- */
#define GIMBAL_YAW_RATE_KP      1.5f
#define GIMBAL_YAW_RATE_KI      0.08f
#define GIMBAL_YAW_RATE_KD      0.01f

/* Giới hạn rate setpoint từ Angle PID (°/s) */
#define GIMBAL_RATE_SP_MAX       120.0f
#define GIMBAL_RATE_SP_MIN      -120.0f

/* Giới hạn output từ Rate PID (µs offset từ center) */
#define GIMBAL_PID_OUT_MAX       800.0f
#define GIMBAL_PID_OUT_MIN      -800.0f

/* Anti-windup giới hạn integral riêng */
#define GIMBAL_INT_LIMIT_ANGLE   40.0f
#define GIMBAL_INT_LIMIT_RATE    80.0f

/* D-term filter (EMA alpha: 0=không lọc, 0.9=lọc rất mạnh) */
#define GIMBAL_D_FILTER_ANGLE    0.15f
#define GIMBAL_D_FILTER_RATE     0.12f

/* ===========================================================================
 *  FEEDFORWARD
 * =========================================================================== */

/** Feedforward gain từ frame gyro rate → servo offset
 *  Tuning: bắt đầu từ 0, tăng dần cho đến khi frame jerk được bù tốt  */
#define GIMBAL_KFF_PITCH        0.0f   /*!< Tăng lên ~0.3–0.8 sau khi tuning */
#define GIMBAL_KFF_YAW          0.0f

/* ===========================================================================
 *  TELEMETRY STRUCT (để main loop in log an toàn)
 * =========================================================================== */

typedef struct {
    float frame_pitch;      /*!< Góc Pitch của frame (°)             */
    float frame_roll;       /*!< Góc Roll của frame (°)              */
    float cam_pitch;        /*!< Góc Pitch của camera (°)            */
    float cam_roll;         /*!< Góc Roll của camera (°)             */
    float err_pitch;        /*!< Relative pitch error (°)            */
    float err_yaw_rate;     /*!< Yaw rate error (°/s)                */
    float servo_pitch_us;   /*!< Pulse width servo Pitch (µs)        */
    float servo_yaw_us;     /*!< Pulse width servo Yaw (µs)          */
    float frame_gyro_z;     /*!< Frame yaw rate (°/s) — debug        */
    uint32_t loop_count;    /*!< Số lần Gimbal_Tick() đã chạy        */
} GimbalTelemetry_t;

/* ===========================================================================
 *  MAIN HANDLE
 * =========================================================================== */

typedef struct {
    /* ---- Sensor Fusion ---- */
    CompFilter_t  cf_frame;        /*!< Comp filter cho Frame IMU         */
    CompFilter_t  cf_camera;       /*!< Comp filter cho Camera IMU        */

    /* ---- PID Controllers ---- */
    PID_Handle_t  pid_pitch_angle; /*!< Pitch: outer loop (angle → rate)  */
    PID_Handle_t  pid_pitch_rate;  /*!< Pitch: inner loop (rate → servo)  */
    PID_Handle_t  pid_yaw_rate;    /*!< Yaw:   rate lock                  */

    /* ---- Setpoints ---- */
    float pitch_setpoint;          /*!< Góc Pitch mong muốn (°), default 0*/
    float yaw_rate_setpoint;       /*!< Yaw rate mong muốn (°/s), default 0*/

    /* ---- Feedforward gains ---- */
    float kff_pitch;               /*!< Pitch feedforward gain            */
    float kff_yaw;                 /*!< Yaw feedforward gain              */

    /* ---- Servo state ---- */
    uint32_t servo_pitch_us;       /*!< CCR Servo Pitch (TIM3 CH2)        */
    uint32_t servo_yaw_us;         /*!< CCR Servo Yaw   (TIM3 CH3)        */

    /* ---- Telemetry (updated by Tick, read by main loop) ---- */
    volatile GimbalTelemetry_t telem;

    /* ---- Timing ---- */
    uint32_t last_tick;            /*!< HAL_GetTick() of last Tick call   */

    /* ---- State ---- */
    uint8_t  initialized;
    uint8_t  lock_mode;            /*!< 1 = hold absolute angle setpoint  */
} GimbalControl_Handle_t;

/* ===========================================================================
 *  PUBLIC API
 * =========================================================================== */

/**
 * @brief  Khởi tạo gimbal controller và tất cả bộ PID/filter.
 *         Gọi SAU ImuDual_Init() và ImuDual_Calibrate().
 * @param  hg     Con trỏ handle gimbal.
 * @param  hImu   Con trỏ handle dual IMU.
 */
void Gimbal_Init(GimbalControl_Handle_t *hg, ImuDual_Handle_t *hImu);

/**
 * @brief  Một chu kỳ điều khiển 500Hz.
 *         Gọi từ main loop khi flag TIM6 được set VÀ mode == GIMBAL.
 *         Thực hiện: đọc IMU → filter → PID → ghi servo.
 * @param  hg     Con trỏ handle gimbal.
 * @param  hImu   Con trỏ handle dual IMU.
 */
void Gimbal_Tick(GimbalControl_Handle_t *hg, ImuDual_Handle_t *hImu);

/**
 * @brief  Gọi từ main loop (không phải ISR) để in telemetry & CLI.
 *         Không làm gì nặng — chỉ copy telemetry ra.
 * @param  hg  Con trỏ handle gimbal.
 */
void Gimbal_Update(GimbalControl_Handle_t *hg);

/**
 * @brief  Reset tất cả PID và filter khi bật Gimbal Mode.
 *         Tránh output đột ngột do integral cũ.
 * @param  hg     Con trỏ handle.
 * @param  hImu   Con trỏ dual IMU (cần dữ liệu hiện tại để init filter).
 */
void Gimbal_Reset(GimbalControl_Handle_t *hg, ImuDual_Handle_t *hImu);

/* ---- Setpoint API ---- */
void Gimbal_SetPitchDeg(GimbalControl_Handle_t *hg, float deg);
void Gimbal_SetYawRateDps(GimbalControl_Handle_t *hg, float dps);

/* ---- Runtime PID Tuning ---- */
void Gimbal_TunePitch(GimbalControl_Handle_t *hg,
                      float aKp, float aKi, float aKd,
                      float rKp, float rKi, float rKd);
void Gimbal_TuneYaw(GimbalControl_Handle_t *hg, float Kp, float Ki, float Kd);
void Gimbal_SetFeedforward(GimbalControl_Handle_t *hg, float kff_pitch, float kff_yaw);
void Gimbal_SetFilterAlpha(GimbalControl_Handle_t *hg, float alpha);

/* ---- CLI (gọi khi nhận được 1 dòng lệnh từ UART/USB CDC) ---- */
/**
 * @brief  Parse và thực thi CLI command.
 *
 *  Supported commands:
 *    p Kp Ki Kd   → pitch angle PID
 *    P Kp Ki Kd   → pitch rate PID
 *    y Kp Ki Kd   → yaw rate PID
 *    f kff_p kff_y→ feedforward gains
 *    a alpha      → comp filter alpha (cả 2 IMU)
 *    s pitch_deg  → pitch setpoint
 *    r            → reset PIDs
 *    c            → rerun calibration (hImu required, pass separately)
 *    d            → print current telemetry
 *
 * @param  hg   Con trỏ handle.
 * @param  line Chuỗi lệnh kết thúc bằng '\0'.
 */
void Gimbal_CLI_Process(GimbalControl_Handle_t *hg, const char *line);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_CONTROL_H__ */

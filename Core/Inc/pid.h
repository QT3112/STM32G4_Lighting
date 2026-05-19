/**
 * @file    pid.h
 * @brief   Thư viện điều khiển PID độc lập, hỗ trợ Cascaded PID.
 *
 *  Cascaded PID (PID vòng lồng):
 *    ┌──────────┐   rate_sp   ┌──────────┐   output
 *    │ Angle PID│ ──────────► │ Rate PID │ ──────────►
 *    └──────────┘             └──────────┘
 *     Setpoint=0°              Feedback = gyro (°/s)
 *     Feedback = angle (°)
 *
 *  Vòng ngoài (Angle PID):
 *    - Setpoint: góc mong muốn (°), thường là 0° (cân bằng)
 *    - Feedback: góc thực tế đo từ Kalman/Complementary Filter
 *    - Output:   tốc độ góc mong muốn (°/s) → trở thành Setpoint của vòng trong
 *
 *  Vòng trong (Rate PID):
 *    - Setpoint: tốc độ góc mong muốn (°/s) từ vòng ngoài
 *    - Feedback: tốc độ góc thực tế từ Gyroscope (°/s)
 *    - Output:   giá trị điều chỉnh servo (µs)
 */

#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ============================================================================
 *  CẤU TRÚC DỮ LIỆU
 * ============================================================================ */

/**
 * @brief  Handle cho một bộ điều khiển PID đơn.
 *         Khởi tạo bằng PID_Init(), gọi PID_Compute() mỗi chu kỳ điều khiển.
 */
typedef struct {
    /* Hệ số PID (người dùng chỉnh) */
    float Kp;       /*!< Hệ số tỉ lệ (Proportional gain)          */
    float Ki;       /*!< Hệ số tích phân (Integral gain)            */
    float Kd;       /*!< Hệ số vi phân (Derivative gain)            */

    /* Giới hạn đầu ra */
    float out_min;  /*!< Giá trị đầu ra tối thiểu (clamp)          */
    float out_max;  /*!< Giá trị đầu ra tối đa (clamp)             */

    /* Giới hạn tích phân (anti-windup) */
    float int_min;  /*!< Giới hạn thấp của tích phân               */
    float int_max;  /*!< Giới hạn cao của tích phân                */

    /* Trạng thái nội bộ (KHÔNG chỉnh thủ công) */
    float integral;     /*!< Tổng tích lũy sai số (I term)          */
    float prev_error;   /*!< Sai số chu kỳ trước (D term)           */

    /* Bộ lọc đạo hàm (tránh nhiễu cao tần trên D term) */
    float d_filtered;   /*!< Giá trị D đã qua bộ lọc thông thấp    */
    float d_filter_alpha; /*!< Hệ số lọc D (0=không lọc, 1=lọc mạnh) */

    /* Cờ bật/tắt */
    uint8_t enabled;    /*!< 1 = PID đang chạy, 0 = tạm dừng       */
} PID_Handle_t;

/* ============================================================================
 *  PUBLIC API
 * ============================================================================ */

/**
 * @brief  Khởi tạo handle PID với các hệ số và giới hạn.
 *
 * @param  hpid       Con trỏ handle PID.
 * @param  Kp         Hệ số tỉ lệ.
 * @param  Ki         Hệ số tích phân.
 * @param  Kd         Hệ số vi phân.
 * @param  out_min    Giới hạn đầu ra tối thiểu.
 * @param  out_max    Giới hạn đầu ra tối đa.
 */
void PID_Init(PID_Handle_t *hpid,
              float Kp, float Ki, float Kd,
              float out_min, float out_max);

/**
 * @brief  Tính đầu ra PID cho một chu kỳ điều khiển.
 *
 *         output = Kp*e + Ki*∫e*dt + Kd*(de/dt)
 *
 * @param  hpid      Con trỏ handle PID.
 * @param  setpoint  Giá trị mong muốn.
 * @param  feedback  Giá trị thực tế đo được.
 * @param  dt        Thời gian chu kỳ (giây).
 * @retval Giá trị đầu ra đã clamp vào [out_min, out_max].
 */
float PID_Compute(PID_Handle_t *hpid, float setpoint, float feedback, float dt);

/**
 * @brief  Reset trạng thái nội bộ (integral, prev_error) về 0.
 *         Gọi khi chuyển sang/ra khỏi Gimbal mode để tránh windup.
 *
 * @param  hpid  Con trỏ handle PID.
 */
void PID_Reset(PID_Handle_t *hpid);

/**
 * @brief  Đặt lại bộ hệ số Kp, Ki, Kd trong khi đang chạy (runtime tuning).
 *
 * @param  hpid  Con trỏ handle PID.
 * @param  Kp    Hệ số tỉ lệ mới.
 * @param  Ki    Hệ số tích phân mới.
 * @param  Kd    Hệ số vi phân mới.
 */
void PID_SetGains(PID_Handle_t *hpid, float Kp, float Ki, float Kd);

/**
 * @brief  Đặt giới hạn tích phân (anti-windup) riêng biệt.
 *         Mặc định = out_min/out_max. Gọi sau PID_Init() nếu cần thu hẹp.
 *
 * @param  hpid     Con trỏ handle PID.
 * @param  int_min  Giới hạn thấp tích phân.
 * @param  int_max  Giới hạn cao tích phân.
 */
void PID_SetIntegralLimits(PID_Handle_t *hpid, float int_min, float int_max);

/**
 * @brief  Bật/tắt bộ lọc thông thấp cho D-term.
 *         Alpha ≈ 0.1–0.3 là phù hợp cho hầu hết gimbal servo.
 *
 * @param  hpid   Con trỏ handle PID.
 * @param  alpha  Hệ số lọc [0.0, 1.0]. 0 = không lọc (D thô). 1 = không dùng D.
 */
void PID_SetDerivativeFilter(PID_Handle_t *hpid, float alpha);

/**
 * @brief  Bật hoặc tắt bộ điều khiển PID.
 *         Khi tắt, PID_Compute() luôn trả về 0.
 *
 * @param  hpid    Con trỏ handle PID.
 * @param  enable  1 = bật, 0 = tắt.
 */
void PID_Enable(PID_Handle_t *hpid, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */

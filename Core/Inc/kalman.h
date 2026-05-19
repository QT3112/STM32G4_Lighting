/**
 * @file    kalman.h
 * @brief   Kalman Filter 1 chiều (scalar) cho ước lượng góc IMU.
 *
 *  Mô hình trạng thái (State-Space):
 *
 *    State vector:  x = [ angle,  angle_bias ]ᵀ
 *                       (góc °)  (drift của gyro °/s)
 *
 *    Predict (Gyroscope):
 *      angle    = angle + dt * (gyro_rate - angle_bias)
 *      bias     = bias                    (không đổi giữa các bước)
 *
 *    Update (Accelerometer):
 *      Innovation y = accel_angle - angle_predicted
 *      Kalman gain K → chỉnh lại angle và bias
 *
 *  Ưu điểm so với Complementary Filter:
 *    - Ước lượng và bù được "gyro bias" (drift) tự động.
 *    - Hội tụ nhanh hơn và ổn định khi góc thay đổi nhanh.
 *
 *  Nhược điểm:
 *    - Nặng hơn về mặt tính toán (nhiều phép nhân float).
 *    - Cần cài đúng Q_angle, Q_bias, R_measure cho từng ứng dụng.
 *
 *  Tham khảo: Kalman, R.E. (1960). "A New Approach to Linear Filtering".
 *             Implemention inspired by: Lauszus/KalmanFilter (GitHub)
 */

#ifndef __KALMAN_H__
#define __KALMAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ============================================================================
 *  CẤU TRÚC DỮ LIỆU
 * ============================================================================ */

/**
 * @brief  Handle cho Kalman Filter 1 chiều.
 *         Khai báo 1 instance cho mỗi trục (pitch, roll).
 */
typedef struct {
    /* ------- Nhiễu hệ thống (Process Noise) ------- */
    float Q_angle;  /*!< Phương sai nhiễu mô hình góc.
                         Tăng → tin vào Accel hơn (phản ứng nhanh, nhiễu nhiều).
                         Giảm → tin vào Gyro hơn (mượt nhưng chậm).
                         Giá trị điển hình: 0.001                             */
    float Q_bias;   /*!< Phương sai nhiễu mô hình gyro bias.
                         Đặt nhỏ nếu gyro bias thay đổi chậm.
                         Giá trị điển hình: 0.003                             */

    /* ------- Nhiễu đo lường (Measurement Noise) ------- */
    float R_measure;/*!< Phương sai nhiễu của Accelerometer.
                         Tăng → tin vào Accel ít hơn (mượt hơn nhưng drift).
                         Giá trị điển hình: 0.03                              */

    /* ------- Trạng thái ước lượng (KHÔNG chỉnh thủ công) ------- */
    float angle;    /*!< Góc ước lượng hiện tại (°)               */
    float bias;     /*!< Ước lượng gyro bias hiện tại (°/s)       */

    /* ------- Ma trận hiệp phương sai (Error Covariance 2×2) ------- */
    float P[2][2];  /*!< P[0][0]=var(angle), P[1][1]=var(bias),
                         P[0][1]=P[1][0]=cov(angle,bias)           */
} Kalman_Handle_t;

/* ============================================================================
 *  PUBLIC API
 * ============================================================================ */

/**
 * @brief  Khởi tạo Kalman Filter với các tham số nhiễu.
 *
 *         Giá trị mặc định phù hợp cho MPU6050 + servo gimbal:
 *           Q_angle  = 0.001
 *           Q_bias   = 0.003
 *           R_measure = 0.03
 *
 * @param  hkal       Con trỏ handle Kalman.
 * @param  Q_angle    Nhiễu hệ thống góc.
 * @param  Q_bias     Nhiễu hệ thống bias.
 * @param  R_measure  Nhiễu đo lường accelerometer.
 */
void Kalman_Init(Kalman_Handle_t *hkal,
                 float Q_angle,
                 float Q_bias,
                 float R_measure);

/**
 * @brief  Cập nhật Kalman Filter cho một chu kỳ điều khiển.
 *         Gọi mỗi dt giây với dữ liệu mới từ MPU6050.
 *
 * @param  hkal         Con trỏ handle Kalman.
 * @param  accel_angle  Góc tính từ Accelerometer (°) – đo lường không chính xác.
 * @param  gyro_rate    Tốc độ góc từ Gyroscope (°/s) – tích phân bị drift.
 * @param  dt           Thời gian chu kỳ (giây).
 * @retval Góc ước lượng tối ưu (°) sau khi kết hợp cả hai nguồn.
 */
float Kalman_Update(Kalman_Handle_t *hkal,
                    float accel_angle,
                    float gyro_rate,
                    float dt);

/**
 * @brief  Reset Kalman Filter về trạng thái ban đầu.
 *         Gọi khi biết chắc góc hiện tại (vd: đặt thiết bị nằm bằng).
 *
 * @param  hkal         Con trỏ handle Kalman.
 * @param  init_angle   Góc ban đầu (°) để khởi tạo ước lượng.
 */
void Kalman_Reset(Kalman_Handle_t *hkal, float init_angle);

/**
 * @brief  Lấy góc ước lượng hiện tại (không cập nhật).
 *
 * @param  hkal  Con trỏ handle Kalman.
 * @retval Góc ước lượng (°).
 */
float Kalman_GetAngle(const Kalman_Handle_t *hkal);

/**
 * @brief  Lấy ước lượng gyro bias hiện tại (°/s).
 *         Hữu ích để debug hoặc lưu offset sau calibration.
 *
 * @param  hkal  Con trỏ handle Kalman.
 * @retval Gyro bias ước lượng (°/s).
 */
float Kalman_GetBias(const Kalman_Handle_t *hkal);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_H__ */

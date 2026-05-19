/**
 * @file    kalman.c
 * @brief   Kalman Filter 1D implementation cho ước lượng góc IMU.
 *
 *  Tham chiếu thuật toán:
 *    Predict step:
 *      angle_pred = angle + dt * (gyro_rate - bias)
 *      P[0][0] += dt*(dt*P[1][1] - P[0][1] - P[1][0] + Q_angle)
 *      P[0][1] -= dt * P[1][1]
 *      P[1][0] -= dt * P[1][1]
 *      P[1][1] += Q_bias * dt
 *
 *    Update step:
 *      S = P[0][0] + R_measure
 *      K[0] = P[0][0] / S    (Kalman gain cho angle)
 *      K[1] = P[1][0] / S    (Kalman gain cho bias)
 *      y = accel_angle - angle_pred  (Innovation)
 *      angle += K[0] * y
 *      bias  += K[1] * y
 *      P[0][0] -= K[0] * P[0][0]
 *      P[0][1] -= K[0] * P[0][1]
 *      P[1][0] -= K[1] * P[0][0]
 *      P[1][1] -= K[1] * P[0][1]
 */

#include "kalman.h"

/* ============================================================================
 *  PUBLIC API
 * ============================================================================ */

void Kalman_Init(Kalman_Handle_t *hkal,
                 float Q_angle,
                 float Q_bias,
                 float R_measure)
{
    hkal->Q_angle   = Q_angle;
    hkal->Q_bias    = Q_bias;
    hkal->R_measure = R_measure;

    hkal->angle = 0.0f;
    hkal->bias  = 0.0f;

    /* Khởi tạo ma trận P với giá trị không chắc chắn cao (uncertainty lớn) */
    hkal->P[0][0] = 1.0f;
    hkal->P[0][1] = 0.0f;
    hkal->P[1][0] = 0.0f;
    hkal->P[1][1] = 1.0f;
}

float Kalman_Update(Kalman_Handle_t *hkal,
                    float accel_angle,
                    float gyro_rate,
                    float dt)
{
    /* ============================================================
     *  BƯỚC 1: PREDICT (dự đoán trạng thái mới từ Gyroscope)
     * ============================================================ */

    /* Dự đoán góc mới: tích phân gyro, trừ bias ước lượng */
    hkal->angle += dt * (gyro_rate - hkal->bias);

    /* Cập nhật ma trận hiệp phương sai dự đoán P = A*P*Aᵀ + Q */
    float dt2 = dt * dt;
    hkal->P[0][0] += dt * (dt * hkal->P[1][1]
                           - hkal->P[0][1]
                           - hkal->P[1][0]
                           + hkal->Q_angle);
    hkal->P[0][1] -= dt * hkal->P[1][1];
    hkal->P[1][0] -= dt * hkal->P[1][1];
    hkal->P[1][1] += hkal->Q_bias * dt;

    /* Tránh rò rỉ số do dt nhỏ (P[1][1] luôn phải dương) */
    (void)dt2;  /* dt2 không dùng trực tiếp nhưng để tham chiếu trên */

    /* ============================================================
     *  BƯỚC 2: UPDATE (chỉnh bằng đo lường từ Accelerometer)
     * ============================================================ */

    /* Innovation covariance S = H*P*Hᵀ + R (H = [1, 0]) */
    float S = hkal->P[0][0] + hkal->R_measure;

    /* Kalman gain K = P*Hᵀ / S */
    float K0 = hkal->P[0][0] / S;
    float K1 = hkal->P[1][0] / S;

    /* Innovation y = accel_angle - angle_predicted */
    float y = accel_angle - hkal->angle;

    /* Cập nhật ước lượng trạng thái */
    hkal->angle += K0 * y;   /* Chỉnh góc ước lượng */
    hkal->bias  += K1 * y;   /* Chỉnh gyro bias ước lượng */

    /* Cập nhật ma trận hiệp phương sai P = (I - K*H)*P */
    float P00_tmp = hkal->P[0][0];
    float P01_tmp = hkal->P[0][1];

    hkal->P[0][0] -= K0 * P00_tmp;
    hkal->P[0][1] -= K0 * P01_tmp;
    hkal->P[1][0] -= K1 * P00_tmp;
    hkal->P[1][1] -= K1 * P01_tmp;

    return hkal->angle;
}

void Kalman_Reset(Kalman_Handle_t *hkal, float init_angle)
{
    hkal->angle   = init_angle;
    hkal->bias    = 0.0f;

    /* Reset covariance về trạng thái ban đầu – không chắc chắn */
    hkal->P[0][0] = 1.0f;
    hkal->P[0][1] = 0.0f;
    hkal->P[1][0] = 0.0f;
    hkal->P[1][1] = 1.0f;
}

float Kalman_GetAngle(const Kalman_Handle_t *hkal)
{
    return hkal->angle;
}

float Kalman_GetBias(const Kalman_Handle_t *hkal)
{
    return hkal->bias;
}

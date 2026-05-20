/**
 * @file    imu_dual.h
 * @brief   Dual MPU6050 driver — Frame IMU (0x68) + Camera IMU (0x69).
 *
 *  Cả hai IMU dùng chung I2C3 (PA8=SCL, PC11=SDA, 400kHz Fast Mode).
 *  AD0 của Frame IMU nối GND → 0x68.
 *  AD0 của Camera IMU nối VCC → 0x69.
 *
 *  Polling mode (không dùng DRDY interrupt) — đọc được gọi từ main loop
 *  khi flag từ TIM6 ISR @ 500Hz được set.
 *
 *  Gyro sensitivity: ±500°/s → 65.5 LSB/°/s
 *  Accel sensitivity: ±4g   → 8192 LSB/g
 *  DLPF: 42Hz bandwidth
 */

#ifndef __IMU_DUAL_H__
#define __IMU_DUAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"

/* ===========================================================================
 *  HARDWARE CONFIGURATION
 * =========================================================================== */

#define IMU_I2C_HANDLE        hi2c3
#define IMU_I2C_TIMEOUT_MS    5U       /*!< Timeout ngắn cho blocking read     */
#define IMU_CALIB_SAMPLES     500U     /*!< Số mẫu calibration gyro            */
#define IMU_WHOAMI_MPU6050    0x68U    /*!< WHO_AM_I của MPU6050 (chip ID nội bộ) */
#define IMU_WHOAMI_MPU6500    0x70U    /*!< WHO_AM_I của MPU6500 (chip ID nội bộ) */
/* NOTE: Đây là ID chip, KHÁC với địa chỉ I2C (0x68/0x69 do chân AD0 quyết định) */

/** Địa chỉ HAL 8-bit (7-bit << 1) */
#define IMU_FRAME_ADDR        (0x68U << 1)   /*!< AD0=GND → frame/body IMU    */
#define IMU_CAM_ADDR          (0x69U << 1)   /*!< AD0=VCC → camera/platform IMU */

#define IMU_GYRO_SENS         65.5f          /*!< LSB per °/s  (±500°/s range) */
#define IMU_ACCEL_SENS        8192.0f        /*!< LSB per g    (±4g range)     */

/* Register addresses */
#define IMU_REG_SMPLRT_DIV    0x19U
#define IMU_REG_CONFIG        0x1AU
#define IMU_REG_GYRO_CONFIG   0x1BU
#define IMU_REG_ACCEL_CONFIG  0x1CU
#define IMU_REG_INT_ENABLE    0x38U
#define IMU_REG_ACCEL_XOUT_H  0x3BU
#define IMU_REG_PWR_MGMT_1    0x6BU
#define IMU_REG_WHO_AM_I      0x75U

/* ===========================================================================
 *  DATA STRUCTURES
 * =========================================================================== */

/** Raw 16-bit sensor output */
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x,  gyro_y,  gyro_z;
} ImuRaw_t;

/** Chuyển đổi sang đơn vị vật lý, đã trừ calibration offset */
typedef struct {
    float accel_x, accel_y, accel_z;  /*!< Gia tốc (g)       */
    float gyro_x,  gyro_y,  gyro_z;   /*!< Tốc độ góc (°/s)  */
} ImuScaled_t;

/** Gyro bias offset tính từ calibration (trừ vào mỗi lần đọc) */
typedef struct {
    float gyro_x, gyro_y, gyro_z;     /*!< Đơn vị °/s        */
} ImuCalib_t;

/** Handle cho một MPU6050 đơn */
typedef struct {
    uint16_t    addr;           /*!< HAL 8-bit I2C address              */
    ImuRaw_t    raw;            /*!< Dữ liệu thô từ register            */
    ImuScaled_t scaled;         /*!< Đã scale + trừ offset              */
    ImuCalib_t  calib;          /*!< Gyro bias offset                   */
    uint8_t     initialized;    /*!< 1 = init thành công                */
    uint8_t     read_ok;        /*!< 1 = lần đọc cuối thành công        */
} ImuSingle_t;

/** Handle chính — chứa cả 2 IMU */
typedef struct {
    ImuSingle_t frame;          /*!< Frame/body IMU  (0x68)             */
    ImuSingle_t camera;         /*!< Camera IMU      (0x69)             */
} ImuDual_Handle_t;

/* ===========================================================================
 *  PUBLIC API
 * =========================================================================== */

/**
 * @brief  Khởi tạo cả hai MPU6050.
 *         Thực hiện WHO_AM_I check, reset, cấu hình DLPF/range/sample rate.
 * @param  h  Con trỏ handle.
 * @retval HAL_OK nếu cả hai thành công.
 */
HAL_StatusTypeDef ImuDual_Init(ImuDual_Handle_t *h);

/**
 * @brief  Đọc raw data từ cả hai IMU, scale, áp dụng calibration offset.
 *         Thời gian thực thi: ~400–500µs @ 400kHz I2C.
 * @param  h  Con trỏ handle.
 * @retval HAL_OK nếu cả hai IMU đọc thành công.
 */
HAL_StatusTypeDef ImuDual_Read(ImuDual_Handle_t *h);

/**
 * @brief  Calibrate gyro bias: lấy trung bình N mẫu ở trạng thái tĩnh.
 *         Gimbal phải đứng yên trong quá trình này (~1 giây blocking).
 * @param  h  Con trỏ handle.
 */
void ImuDual_Calibrate(ImuDual_Handle_t *h);

/**
 * @brief  Reset calibration offset về 0.
 */
void ImuDual_ResetCalib(ImuDual_Handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_DUAL_H__ */

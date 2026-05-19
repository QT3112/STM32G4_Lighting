/**
 * @file    mpu6050.c
 * @brief   Driver implementation cho cảm biến MPU6050.
 *
 * Giao tiếp:  I2C3  (PA8=SCL, PC11=SDA) – 400 kHz Fast Mode
 * Interrupt:  PA15  (EXTI15, Data-Ready)
 *
 * Luồng sử dụng điển hình trong ứng dụng:
 *   1. Gọi MPU6050_Init()  sau MX_I2C1_Init() trong main.c
 *   2. Trong HAL_GPIO_EXTI_Callback(): gọi MPU6050_DRDY_Callback()
 *   3. Trong main loop: if (MPU6050_IsDataReady()) { MPU6050_Update(); }
 *   4. Sử dụng hdev->angle.pitch / hdev->angle.roll cho thuật toán PID
 */

#include "mpu6050.h"
#include <math.h>   /* atan2f, sqrtf */
#include "stdio.h"
#include "stdint.h"
#include "string.h"

/* ============================================================================
 *  MACRO NỘI BỘ
 * ============================================================================ */

/** Ghi 1 byte vào thanh ghi */
#define MPU6050_WRITE_REG(reg, val) \
    _mpu6050_write_reg(hdev, (reg), (val))

/** Đọc N byte từ thanh ghi đầu tiên (burst read) */
#define MPU6050_READ_REGS(reg, buf, len) \
    HAL_I2C_Mem_Read(&MPU6050_I2C_HANDLE,          \
                     MPU6050_I2C_ADDR,             \
                     (reg),                        \
                     I2C_MEMADD_SIZE_8BIT,         \
                     (buf),                        \
                     (len),                        \
                     MPU6050_I2C_TIMEOUT_MS)

/** Hằng số chuyển đổi độ sang radian */
#define DEG_TO_RAD  0.01745329251994f
/** Hằng số chuyển đổi radian sang độ */
#define RAD_TO_DEG  57.29577951308f

/* ============================================================================
 *  HÀM HELPER NỘI BỘ (PRIVATE)
 * ============================================================================ */

/**
 * @brief  Ghi 1 byte dữ liệu vào thanh ghi MPU6050.
 */
static HAL_StatusTypeDef _mpu6050_write_reg(MPU6050_Handle_t *hdev,
                                            uint8_t reg,
                                            uint8_t value)
{
    (void)hdev;  /* Không dùng hdev trực tiếp ở đây, I2C handle dùng macro */
    return HAL_I2C_Mem_Write(&MPU6050_I2C_HANDLE,
                              MPU6050_I2C_ADDR,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              &value,
                              1,
                              MPU6050_I2C_TIMEOUT_MS);
}

/**
 * @brief  Tính hệ số nhạy Gyroscope từ FS_SEL.
 */
static float _get_gyro_sensitivity(MPU6050_GyroFS_t fs)
{
    switch (fs)
    {
        case MPU6050_GYRO_FS_250DPS:  return 131.0f;
        case MPU6050_GYRO_FS_500DPS:  return  65.5f;
        case MPU6050_GYRO_FS_1000DPS: return  32.8f;
        case MPU6050_GYRO_FS_2000DPS: return  16.4f;
        default:                      return  65.5f;
    }
}

/**
 * @brief  Tính hệ số nhạy Accelerometer từ AFS_SEL.
 */
static float _get_accel_sensitivity(MPU6050_AccelFS_t fs)
{
    switch (fs)
    {
        case MPU6050_ACCEL_FS_2G:  return 16384.0f;
        case MPU6050_ACCEL_FS_4G:  return  8192.0f;
        case MPU6050_ACCEL_FS_8G:  return  4096.0f;
        case MPU6050_ACCEL_FS_16G: return  2048.0f;
        default:                   return  8192.0f;
    }
}

/* ============================================================================
 *  PUBLIC API IMPLEMENTATION
 * ============================================================================ */

/**
 * @brief  Khởi tạo MPU6050.
 */
HAL_StatusTypeDef MPU6050_Init(MPU6050_Handle_t *hdev,
                                MPU6050_GyroFS_t  gyro_fs,
                                MPU6050_AccelFS_t accel_fs,
                                MPU6050_DLPF_t    dlpf)
{
    HAL_StatusTypeDef ret;
    uint8_t who_am_i = 0;

    /* ---- 0. Khởi tạo struct về giá trị mặc định ---- */
    hdev->gyro_fs            = gyro_fs;
    hdev->accel_fs           = accel_fs;
    hdev->gyro_sens          = _get_gyro_sensitivity(gyro_fs);
    hdev->accel_sens         = _get_accel_sensitivity(accel_fs);
    hdev->comp_filter_alpha  = 0.96f;   /* 96% Gyro + 4% Accel */
    hdev->last_tick          = HAL_GetTick();
    hdev->data_ready_flag    = 0;
    hdev->initialized        = 0;
    hdev->angle.pitch        = 0.0f;
    hdev->angle.roll         = 0.0f;

    /* ---- 1. Kiểm tra kết nối vật lý: đọc WHO_AM_I ---- */
    ret = HAL_I2C_Mem_Read(&MPU6050_I2C_HANDLE,
                            MPU6050_I2C_ADDR,
                            MPU6050_REG_WHO_AM_I,
                            I2C_MEMADD_SIZE_8BIT,
                            &who_am_i, 1,
                            MPU6050_I2C_TIMEOUT_MS);
    if (ret != HAL_OK || who_am_i != MPU6050_WHOAMI_VALUE)
    {
        /* Không nhận dạng được chip – kiểm tra kết nối vật lý */
        // printf("ret: %d, who_am_i: %x \n", ret, who_am_i);
        return HAL_ERROR;
    }

    /* ---- 2. Reset chip để đưa về trạng thái mặc định ---- */
    ret = MPU6050_WRITE_REG(MPU6050_REG_PWR_MGMT_1, 0x80);  /* DEVICE_RESET */
    if (ret != HAL_OK) return ret;
    HAL_Delay(100);  /* Chờ reset hoàn tất (~100ms theo datasheet) */

    /* ---- 3. Thoát chế độ Sleep – chọn clock PLL từ Gyro X ---- */
    /* CLK_SEL = 001 (PLL with X-axis gyro) → ổn định hơn internal 8MHz */
    ret = MPU6050_WRITE_REG(MPU6050_REG_PWR_MGMT_1, 0x01);
    if (ret != HAL_OK) return ret;

    /* ---- 4. Cài Sample Rate: 1kHz / (1 + SMPLRT_DIV) ---- */
    /* Với DLPF bật: F_gyro = 1kHz. SMPLRT_DIV=9 → 100 Hz sample rate */
    ret = MPU6050_WRITE_REG(MPU6050_REG_SMPLRT_DIV, 9U);
    if (ret != HAL_OK) return ret;

    /* ---- 5. Cài DLPF (Digital Low-Pass Filter) ---- */
    ret = MPU6050_WRITE_REG(MPU6050_REG_CONFIG, (uint8_t)dlpf);
    if (ret != HAL_OK) return ret;

    /* ---- 6. Cài dải đo Gyroscope ---- */
    ret = MPU6050_WRITE_REG(MPU6050_REG_GYRO_CONFIG, (uint8_t)gyro_fs);
    if (ret != HAL_OK) return ret;

    /* ---- 7. Cài dải đo Accelerometer ---- */
    ret = MPU6050_WRITE_REG(MPU6050_REG_ACCEL_CONFIG, (uint8_t)accel_fs);
    if (ret != HAL_OK) return ret;

    /* ---- 8. Cấu hình chân INT (PA15) ---- */
    /* Active-high, push-pull, latch cho đến khi đọc INT_STATUS, clear khi đọc data */
    ret = MPU6050_WRITE_REG(MPU6050_REG_INT_PIN_CFG,
                             MPU6050_INTCFG_ACTIVE_HIGH |  /* INT = high khi có data */
                             MPU6050_INTCFG_LATCH_EN    |  /* Giữ mức INT đến khi xóa */
                             MPU6050_INTCFG_RD_CLEAR);     /* Xóa cờ khi đọc INT_STATUS */
    if (ret != HAL_OK) return ret;

    /* ---- 9. Bật ngắt Data-Ready ---- */
    ret = MPU6050_WRITE_REG(MPU6050_REG_INT_ENABLE,
                             MPU6050_INT_ENABLE_DATA_RDY);
    if (ret != HAL_OK) return ret;

    /* ---- 10. Đánh dấu khởi tạo thành công ---- */
    hdev->initialized = 1;
    hdev->last_tick   = HAL_GetTick();

    return HAL_OK;
}

/**
 * @brief  Đọc 14 byte dữ liệu thô (Accel XYZ + Temp + Gyro XYZ).
 */
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Handle_t *hdev)
{
    uint8_t buf[14];
    HAL_StatusTypeDef ret;

    /* Burst read 14 byte bắt đầu từ ACCEL_XOUT_H (0x3B) */
    ret = MPU6050_READ_REGS(MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (ret != HAL_OK) return ret;

    /* Ghép byte cao và byte thấp (big-endian format của MPU6050) */
    hdev->raw.accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    hdev->raw.accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    hdev->raw.accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    hdev->raw.temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    hdev->raw.gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    hdev->raw.gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    hdev->raw.gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}

/**
 * @brief  Chuyển đổi dữ liệu thô sang đơn vị vật lý.
 */
void MPU6050_ConvertScaled(MPU6050_Handle_t *hdev)
{
    /* Gia tốc: đơn vị g (1g ≈ 9.81 m/s²) */
    hdev->scaled.accel_x = (float)hdev->raw.accel_x / hdev->accel_sens;
    hdev->scaled.accel_y = (float)hdev->raw.accel_y / hdev->accel_sens;
    hdev->scaled.accel_z = (float)hdev->raw.accel_z / hdev->accel_sens;

    /* Nhiệt độ: công thức từ datasheet MPU6050
       Temp(°C) = RAW / 340.0 + 36.53  */
    hdev->scaled.temp_c  = (float)hdev->raw.temp / 340.0f + 36.53f;

    /* Tốc độ góc: đơn vị °/s */
    hdev->scaled.gyro_x  = (float)hdev->raw.gyro_x / hdev->gyro_sens;
    hdev->scaled.gyro_y  = (float)hdev->raw.gyro_y / hdev->gyro_sens;
    hdev->scaled.gyro_z  = (float)hdev->raw.gyro_z / hdev->gyro_sens;
}

/**
 * @brief  Tính góc Pitch và Roll bằng Complementary Filter.
 *
 * Complementary Filter kết hợp:
 *   - Ưu điểm Gyro: phản ứng nhanh, không nhiễu trong thời gian ngắn
 *   - Ưu điểm Accel: chính xác về lâu dài, không bị drift
 *
 * Công thức:
 *   angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle
 *
 * Trong đó alpha (0.0–1.0), thường = 0.96 cho gimbal servo.
 */
void MPU6050_UpdateAngle(MPU6050_Handle_t *hdev)
{
    uint32_t now = HAL_GetTick();

    /* Tính dt (giây) từ lần gọi trước */
    float dt = (float)(now - hdev->last_tick) / 1000.0f;
    hdev->last_tick = now;

    /* Giới hạn dt để tránh nhảy vọt khi bị block lâu (tối đa 50ms) */
    if (dt > 0.05f) dt = 0.05f;
    if (dt <= 0.0f) return;

    /* ---- Góc từ Accelerometer (ổn định dài hạn, nhiễu ngắn hạn) ----
     *
     * Pitch: góc giữa vector gia tốc và mặt phẳng YZ
     *   pitch_acc = atan2(Ax, sqrt(Ay² + Az²))
     *
     * Roll:  góc giữa vector gia tốc và mặt phẳng XZ
     *   roll_acc  = atan2(Ay, Az)
     *
     * Chú ý: kết quả là góc so với trục trọng lực, phù hợp cho gimbal nằm ngang.
     */
    float ax = hdev->scaled.accel_x;
    float ay = hdev->scaled.accel_y;
    float az = hdev->scaled.accel_z;

    float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    float accel_roll  = atan2f(ay, az) * RAD_TO_DEG;

    /* ---- Complementary Filter ----
     *
     * Tích phân gyro + bù bằng góc accel để loại drift.
     * Hệ số alpha = 0.96: 96% tin vào Gyro trong ngắn hạn,
     *                      4%  tin vào Accel để chỉnh dài hạn.
     */
    float alpha = hdev->comp_filter_alpha;

    hdev->angle.pitch = alpha * (hdev->angle.pitch + hdev->scaled.gyro_x * dt)
                      + (1.0f - alpha) * accel_pitch;

    hdev->angle.roll  = alpha * (hdev->angle.roll  + hdev->scaled.gyro_y * dt)
                      + (1.0f - alpha) * accel_roll;
}

/**
 * @brief  Đọc, chuyển đổi và cập nhật góc trong một lệnh.
 */
HAL_StatusTypeDef MPU6050_Update(MPU6050_Handle_t *hdev)
{
    HAL_StatusTypeDef ret = MPU6050_ReadRaw(hdev);
    if (ret != HAL_OK) return ret;

    MPU6050_ConvertScaled(hdev);
    MPU6050_UpdateAngle(hdev);

    return HAL_OK;
}

/**
 * @brief  Callback ngắt Data-Ready (PA15, EXTI15).
 *         Gọi từ HAL_GPIO_EXTI_Callback() trong main.c hoặc stm32g4xx_it.c.
 */
void MPU6050_DRDY_Callback(MPU6050_Handle_t *hdev)
{
    hdev->data_ready_flag = 1;
}

/**
 * @brief  Kiểm tra và xóa cờ Data-Ready.
 */
uint8_t MPU6050_IsDataReady(MPU6050_Handle_t *hdev)
{
    if (hdev->data_ready_flag)
    {
        hdev->data_ready_flag = 0;  /* Xóa cờ ngay để tránh xử lý lại */
        return 1;
    }
    return 0;
}

/**
 * @brief  Đọc nhiệt độ chip (°C).
 */
float MPU6050_GetTemperature(MPU6050_Handle_t *hdev)
{
    return hdev->scaled.temp_c;
}

/**
 * @brief  Reset software cho MPU6050.
 */
HAL_StatusTypeDef MPU6050_Reset(MPU6050_Handle_t *hdev)
{
    hdev->initialized = 0;
    return MPU6050_WRITE_REG(MPU6050_REG_PWR_MGMT_1, 0x80);
}

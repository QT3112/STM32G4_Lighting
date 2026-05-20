/**
 * @file    imu_dual.c
 * @brief   Dual MPU6050 driver implementation.
 *
 *  Đọc tuần tự: Frame IMU trước, Camera IMU sau.
 *  Không dùng interrupt (polling) — an toàn khi gọi từ main loop.
 */

#include "imu_dual.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ===========================================================================
 *  PRIVATE HELPERS
 * =========================================================================== */

static HAL_StatusTypeDef _write(uint16_t addr, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&IMU_I2C_HANDLE, addr, reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1,
                              IMU_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef _read(uint16_t addr, uint8_t reg,
                                uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&IMU_I2C_HANDLE, addr, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, len,
                             IMU_I2C_TIMEOUT_MS);
}

/**
 * @brief  Init một MPU6050 theo địa chỉ đã cấu hình trong ImuSingle_t.
 */
static HAL_StatusTypeDef _init_one(ImuSingle_t *imu)
{
    HAL_StatusTypeDef ret;
    uint8_t who = 0;

    imu->initialized = 0;
    imu->read_ok     = 0;
    memset(&imu->calib, 0, sizeof(imu->calib));

    /* Bước 1: Kiểm tra WHO_AM_I
     *
     * WHO_AM_I (reg 0x75) là ID chip nội bộ, KHÔNG PHẢI địa chỉ I2C:
     *   Địa chỉ I2C (0x68/0x69) → do chân AD0 quyết định (GND=0x68, VCC=0x69)
     *   WHO_AM_I              → hằng số silicon không đổi:
     *                           MPU6050 = 0x68 | MPU6500 = 0x70
     * Chấp nhận cả hai vì register map tương thích nhau.
     */
    ret = _read(imu->addr, IMU_REG_WHO_AM_I, &who, 1);

    /* Luôn in ra để debug, kể cả khi thành công */
    printf("[IMUD] WHO_AM_I @ I2C:0x%02X  ret=%d  got=0x%02X",
           (imu->addr >> 1), ret, who);

    uint8_t valid = (who == IMU_WHOAMI_MPU6050 || who == IMU_WHOAMI_MPU6500);
    if (ret != HAL_OK || !valid) {
        printf(" -> FAIL (expected 0x68=MPU6050 or 0x70=MPU6500)\r\n");
        return HAL_ERROR;
    }
    printf(" -> OK (%s)\r\n", (who == IMU_WHOAMI_MPU6500) ? "MPU6500" : "MPU6050");


    /* Bước 2: Software reset */
    _write(imu->addr, IMU_REG_PWR_MGMT_1, 0x80U);
    HAL_Delay(100);

    /* Bước 3: Thoát Sleep, dùng PLL Gyro-X làm clock */
    ret = _write(imu->addr, IMU_REG_PWR_MGMT_1, 0x01U);
    if (ret != HAL_OK) return ret;

    /* Bước 4: Sample rate divider → 500Hz output rate
       F_sample = 1kHz / (1 + SMPLRT_DIV) = 1kHz / (1+1) = 500Hz */
    ret = _write(imu->addr, IMU_REG_SMPLRT_DIV, 0x01U);
    if (ret != HAL_OK) return ret;

    /* Bước 5: DLPF = 3 → Gyro BW 42Hz, Accel BW 44Hz
       Phù hợp với servo gimbal 50Hz, lọc tốt rung cơ học */
    ret = _write(imu->addr, IMU_REG_CONFIG, 0x03U);
    if (ret != HAL_OK) return ret;

    /* Bước 6: Gyro ±500°/s (FS_SEL = 1) */
    ret = _write(imu->addr, IMU_REG_GYRO_CONFIG, 0x08U);
    if (ret != HAL_OK) return ret;

    /* Bước 7: Accel ±4g (AFS_SEL = 1) */
    ret = _write(imu->addr, IMU_REG_ACCEL_CONFIG, 0x08U);
    if (ret != HAL_OK) return ret;

    /* Bước 8: Tắt interrupt output (dùng polling) */
    _write(imu->addr, IMU_REG_INT_ENABLE, 0x00U);

    imu->initialized = 1;
    printf("[IMUD] Init OK @ 0x%02X\r\n", imu->addr >> 1);
    return HAL_OK;
}

/**
 * @brief  Đọc 14 byte raw data từ một IMU, scale và trừ calibration.
 */
static HAL_StatusTypeDef _read_one(ImuSingle_t *imu)
{
    uint8_t buf[14];

    HAL_StatusTypeDef ret = _read(imu->addr, IMU_REG_ACCEL_XOUT_H, buf, 14);
    if (ret != HAL_OK) {
        imu->read_ok = 0;
        return ret;
    }

    /* Parse big-endian registers */
    imu->raw.accel_x = (int16_t)((uint16_t)(buf[0]  << 8) | buf[1]);
    imu->raw.accel_y = (int16_t)((uint16_t)(buf[2]  << 8) | buf[3]);
    imu->raw.accel_z = (int16_t)((uint16_t)(buf[4]  << 8) | buf[5]);
    /* buf[6-7] = temperature, bỏ qua */
    imu->raw.gyro_x  = (int16_t)((uint16_t)(buf[8]  << 8) | buf[9]);
    imu->raw.gyro_y  = (int16_t)((uint16_t)(buf[10] << 8) | buf[11]);
    imu->raw.gyro_z  = (int16_t)((uint16_t)(buf[12] << 8) | buf[13]);

    /* Scale sang đơn vị vật lý, trừ gyro bias calibration */
    imu->scaled.accel_x = (float)imu->raw.accel_x / IMU_ACCEL_SENS;
    imu->scaled.accel_y = (float)imu->raw.accel_y / IMU_ACCEL_SENS;
    imu->scaled.accel_z = (float)imu->raw.accel_z / IMU_ACCEL_SENS;

    imu->scaled.gyro_x = (float)imu->raw.gyro_x / IMU_GYRO_SENS - imu->calib.gyro_x;
    imu->scaled.gyro_y = (float)imu->raw.gyro_y / IMU_GYRO_SENS - imu->calib.gyro_y;
    imu->scaled.gyro_z = (float)imu->raw.gyro_z / IMU_GYRO_SENS - imu->calib.gyro_z;

    imu->read_ok = 1;
    return HAL_OK;
}

/* ===========================================================================
 *  PUBLIC API
 * =========================================================================== */

HAL_StatusTypeDef ImuDual_Init(ImuDual_Handle_t *h)
{
    HAL_StatusTypeDef ret_frame, ret_cam;

    h->frame.addr  = IMU_FRAME_ADDR;
    h->camera.addr = IMU_CAM_ADDR;

    ret_frame = _init_one(&h->frame);
    ret_cam   = _init_one(&h->camera);

    return (ret_frame == HAL_OK && ret_cam == HAL_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef ImuDual_Read(ImuDual_Handle_t *h)
{
    HAL_StatusTypeDef rf = _read_one(&h->frame);
    HAL_StatusTypeDef rc = _read_one(&h->camera);
    return (rf == HAL_OK && rc == HAL_OK) ? HAL_OK : HAL_ERROR;
}

void ImuDual_Calibrate(ImuDual_Handle_t *h)
{
    double sum_fx = 0, sum_fy = 0, sum_fz = 0;
    double sum_cx = 0, sum_cy = 0, sum_cz = 0;
    uint32_t n = IMU_CALIB_SAMPLES;

    printf("[IMUD] Calibrating gyro (%lu samples)... hold still!\r\n", n);

    for (uint32_t i = 0; i < n; i++) {
        /* Tạm thời clear calib để đọc raw */
        h->frame.calib.gyro_x = 0;
        h->frame.calib.gyro_y = 0;
        h->frame.calib.gyro_z = 0;
        h->camera.calib.gyro_x = 0;
        h->camera.calib.gyro_y = 0;
        h->camera.calib.gyro_z = 0;

        _read_one(&h->frame);
        _read_one(&h->camera);

        sum_fx += h->frame.scaled.gyro_x;
        sum_fy += h->frame.scaled.gyro_y;
        sum_fz += h->frame.scaled.gyro_z;

        sum_cx += h->camera.scaled.gyro_x;
        sum_cy += h->camera.scaled.gyro_y;
        sum_cz += h->camera.scaled.gyro_z;

        HAL_Delay(2);  /* ~500 samples × 2ms = 1 giây */
    }

    h->frame.calib.gyro_x  = (float)(sum_fx / n);
    h->frame.calib.gyro_y  = (float)(sum_fy / n);
    h->frame.calib.gyro_z  = (float)(sum_fz / n);
    h->camera.calib.gyro_x = (float)(sum_cx / n);
    h->camera.calib.gyro_y = (float)(sum_cy / n);
    h->camera.calib.gyro_z = (float)(sum_cz / n);

    printf("[IMUD] Calib done. Frame gyro offset: X=%.3f Y=%.3f Z=%.3f °/s\r\n",
           h->frame.calib.gyro_x, h->frame.calib.gyro_y, h->frame.calib.gyro_z);
    printf("[IMUD] Calib done. Cam   gyro offset: X=%.3f Y=%.3f Z=%.3f °/s\r\n",
           h->camera.calib.gyro_x, h->camera.calib.gyro_y, h->camera.calib.gyro_z);
}

void ImuDual_ResetCalib(ImuDual_Handle_t *h)
{
    memset(&h->frame.calib,  0, sizeof(ImuCalib_t));
    memset(&h->camera.calib, 0, sizeof(ImuCalib_t));
}

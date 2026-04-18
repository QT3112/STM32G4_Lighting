/**
 * @file    crsf.h
 * @brief   Thư viện CRSF (Crossfire Serial Protocol) cho STM32G4
 *
 * Hỗ trợ các frame type:
 *   0x02  GPS
 *   0x08  Battery Sensor
 *   0x14  Link Statistics
 *   0x16  RC Channels Packed (16 kênh × 11-bit)
 *   0x1E  Attitude
 *   0x21  Flight Mode
 *
 * Giao thức:
 *   UART 420000 baud, 8N1, half-duplex
 *   Frame: [Address][Length][Type][Payload...][CRC8/DVB-S2]
 *   CRC tính trên [Type]+[Payload]
 *
 * Tham khảo:
 *   - https://github.com/CapnBry/CRServoF
 *   - https://github.com/tbs-fpv/tbs-crsf-spec
 *   - ExpressLRS CRSF specification
 */

#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* ==========================================================================
 * Hằng số giao thức
 * ========================================================================== */
#define CRSF_BAUDRATE             420000U   ///< Tốc độ baud chuẩn CRSF
#define CRSF_MAX_CHANNELS         16U       ///< Số kênh RC tối đa
#define CRSF_FRAME_SIZE_MAX       64U       ///< Kích thước frame tối đa (bytes)
#define CRSF_RC_CHANNEL_MIN       172U      ///< Giá trị 11-bit nhỏ nhất (1000µs)
#define CRSF_RC_CHANNEL_MID       992U      ///< Giá trị 11-bit giữa    (1500µs)
#define CRSF_RC_CHANNEL_MAX       1811U     ///< Giá trị 11-bit lớn nhất (2000µs)
#define CRSF_TIMEOUT_MS           100U      ///< Timeout mất kết nối (ms)
#define CRSF_FLIGHT_MODE_STR_LEN  16U       ///< Chiều dài tối đa chuỗi flight mode

/* ==========================================================================
 * Địa chỉ thiết bị (Device Addresses / Sync Bytes)
 * ========================================================================== */
typedef enum {
    CRSF_ADDRESS_BROADCAST          = 0x00,
    CRSF_ADDRESS_USB                = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO   = 0x80,
    CRSF_ADDRESS_CURRENT_SENSOR     = 0xC0,
    CRSF_ADDRESS_GPS                = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX       = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER  = 0xC8,  ///< FC / Sync byte chính
    CRSF_ADDRESS_RACE_TAG           = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER  = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER      = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER   = 0xEE,
} CRSF_Address_e;

/* ==========================================================================
 * Loại frame (Frame Types)
 * ========================================================================== */
typedef enum {
    CRSF_FRAMETYPE_GPS              = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR   = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS  = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS      = 0x16,  ///< 16 kênh × 11-bit = 22 bytes
    CRSF_FRAMETYPE_ATTITUDE         = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE      = 0x21,
    CRSF_FRAMETYPE_DEVICE_PING      = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO      = 0x29,
} CRSF_FrameType_e;

/* ==========================================================================
 * Kích thước payload của từng loại frame
 * ========================================================================== */
#define CRSF_PAYLOAD_SIZE_RC_CHANNELS   22U  ///< 16ch × 11bit = 176bit = 22 byte
#define CRSF_PAYLOAD_SIZE_LINK_STATS    10U
#define CRSF_PAYLOAD_SIZE_BATTERY       8U
#define CRSF_PAYLOAD_SIZE_ATTITUDE      6U

/* ==========================================================================
 * Macro chuyển đổi đơn vị
 * Công thức tuyến tính: y = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
 * CRSF: 172 → 1000µs, 992 → 1500µs, 1811 → 2000µs
 * ========================================================================== */
/** Chuyển 11-bit CRSF value → µs (kết quả int32_t, nên clamp về [1000,2000]) */
#define CRSF_TO_US(v) \
    (int32_t)(((int32_t)(v) - CRSF_RC_CHANNEL_MIN) * \
    (2000 - 1000) / (CRSF_RC_CHANNEL_MAX - CRSF_RC_CHANNEL_MIN) + 1000)

/** Chuyển µs → 11-bit CRSF value */
#define US_TO_CRSF(us) \
    (uint16_t)(((int32_t)(us) - 1000) * \
    (CRSF_RC_CHANNEL_MAX - CRSF_RC_CHANNEL_MIN) / (2000 - 1000) + CRSF_RC_CHANNEL_MIN)

/* ==========================================================================
 * Struct: Link Statistics (0x14) – 10 bytes
 * ========================================================================== */
typedef struct {
    uint8_t uplink_rssi_ant1;   ///< Uplink RSSI Antenna 1 (dBm, không dấu, ~0-120)
    uint8_t uplink_rssi_ant2;   ///< Uplink RSSI Antenna 2
    uint8_t uplink_link_quality;///< Uplink Link Quality (0–100 %)
    int8_t  uplink_snr;         ///< Uplink SNR (dB)
    uint8_t active_antenna;     ///< Anten đang dùng (0 hoặc 1)
    uint8_t rf_mode;            ///< Chế độ RF (0=4Hz, 1=50Hz, 2=150Hz ...)
    uint8_t uplink_tx_power;    ///< Công suất TX (0=0mW, 1=10mW, 2=25mW, 3=100mW, 4=500mW, 5=1W, 6=2W)
    uint8_t downlink_rssi;      ///< Downlink RSSI (dBm, không dấu)
    uint8_t downlink_link_quality; ///< Downlink Link Quality (0–100 %)
    int8_t  downlink_snr;       ///< Downlink SNR (dB)
} CRSF_LinkStats_t;

/* ==========================================================================
 * Struct: Battery Sensor (0x08) – 8 bytes (Big-Endian từ receiver)
 * ========================================================================== */
typedef struct {
    uint16_t voltage_mv10;  ///< Điện áp × 10 mV (= mV / 100, BE → đổi về LE sau parse)
    uint16_t current_ma10;  ///< Dòng điện × 10 mA (BE)
    uint32_t capacity_mah : 24; ///< Dung lượng tiêu thụ (mAh, 3 bytes BE)
    uint8_t  remaining_pct; ///< Phần trăm pin còn lại (0–100)
} CRSF_Battery_t;

/* ==========================================================================
 * Struct: Attitude (0x1E) – 6 bytes (Big-Endian int16 × 10000 rad)
 * ========================================================================== */
typedef struct {
    int16_t pitch_rad10000;  ///< Pitch (×10000 rad, BE) – dương = mũi lên
    int16_t roll_rad10000;   ///< Roll  (×10000 rad, BE) – dương = nghiêng phải
    int16_t yaw_rad10000;    ///< Yaw   (×10000 rad, BE) – dương = xoay phải
} CRSF_Attitude_t;

/* ==========================================================================
 * Struct: Dữ liệu CRSF tổng hợp
 * ========================================================================== */
typedef struct {
    /* RC Channels (0x16) */
    uint16_t channels[CRSF_MAX_CHANNELS]; ///< Raw 11-bit (172..1811)

    /* Trạng thái kết nối */
    uint8_t  is_connected;          ///< 1 nếu đang kết nối (packet trong CRSF_TIMEOUT_MS)
    uint32_t last_packet_time;      ///< HAL_GetTick() của packet cuối (bất kỳ loại nào)

    /* Link Statistics (0x14) */
    CRSF_LinkStats_t link_stats;
    uint8_t has_link_stats;         ///< 1 nếu đã nhận được ít nhất 1 frame LinkStats

    /* Battery Sensor (0x08) */
    uint16_t battery_voltage_mv;    ///< Điện áp (mV) sau chuyển đổi
    uint16_t battery_current_ma;    ///< Dòng điện (mA) sau chuyển đổi
    uint32_t battery_capacity_mah;  ///< Dung lượng đã dùng (mAh)
    uint8_t  battery_remaining_pct; ///< % pin còn lại
    uint8_t  has_battery;           ///< 1 nếu đã nhận frame Battery

    /* Attitude (0x1E) */
    float pitch_rad;    ///< Pitch (radians)
    float roll_rad;     ///< Roll  (radians)
    float yaw_rad;      ///< Yaw   (radians)
    uint8_t has_attitude;

    /* Flight Mode (0x21) */
    char flight_mode[CRSF_FLIGHT_MODE_STR_LEN]; ///< Null-terminated string
    uint8_t has_flight_mode;
} CRSF_Data_t;

/* ==========================================================================
 * API công khai
 * ========================================================================== */

/**
 * @brief Khởi tạo thư viện CRSF (reset data, không cần thiết lập UART ở đây).
 * @param data Con trỏ tới CRSF_Data_t cần khởi tạo.
 */
void CRSF_Init(CRSF_Data_t *data);

/**
 * @brief Parse buffer DMA, nhận diện và giải mã tất cả các loại frame CRSF.
 * @param buffer  Con trỏ tới buffer DMA.
 * @param length  Số byte hợp lệ trong buffer.
 * @param data    Con trỏ tới CRSF_Data_t để lưu kết quả.
 * @return        Số frame hợp lệ đã parse được (≥1 nghĩa là có cập nhật).
 */
uint8_t CRSF_ParseFrame(const uint8_t *buffer, uint16_t length, CRSF_Data_t *data);

/**
 * @brief Lấy giá trị kênh RC đã chuyển đổi sang µs.
 * @param data  Con trỏ tới CRSF_Data_t.
 * @param ch    Số kênh (1–16).
 * @return      Giá trị µs (1000–2000), hoặc 0 nếu không kết nối / kênh không hợp lệ.
 */
uint16_t CRSF_GetChannelUs(const CRSF_Data_t *data, uint8_t ch);

/**
 * @brief Kiểm tra xem receiver có đang kết nối không (dựa trên timeout).
 * @param data  Con trỏ tới CRSF_Data_t.
 * @return      1 nếu đang kết nối, 0 nếu mất kết nối.
 */
uint8_t CRSF_IsConnected(CRSF_Data_t *data);

/**
 * @brief Reset toàn bộ dữ liệu CRSF về trạng thái ban đầu.
 * @param data  Con trỏ tới CRSF_Data_t.
 */
void CRSF_ResetData(CRSF_Data_t *data);

#endif /* CRSF_H */

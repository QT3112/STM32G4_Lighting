/**
 * @file    crsf.h
 * @brief   Thư viện CRSF (Crossfire Serial Protocol) cho STM32G4
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * Kiến trúc sử dụng:
 *   - USART3 ở tốc độ 420 000 baud, 8N1
 *   - DMA1_Channel1 Circular (RX) → buffer 128 byte không cần ngắt UART
 *   - CRSF_Input_Update() gọi trong main loop để parse frame từ DMA buffer
 *   - Hỗ trợ frame: RC Channels (0x16), Link Statistics (0x14),
 *                    Battery Sensor (0x08), Attitude (0x1E), Flight Mode (0x21)
 *
 * Cách tích hợp dual-mode (PWM / CRSF) trong main.c:
 *   1. Gọi CRSF_Input_Init() trong phần USER CODE BEGIN 2
 *   2. Trong while(1): gọi CRSF_Input_Update()
 *   3. Đọc tín hiệu qua CRSF_Input_IsConnected() + CRSF_Input_GetChX()
 *   4. Nếu CRSF kết nối → dùng kênh CRSF, không thì dùng RC_Input_GetChX()
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef CRSF_H
#define CRSF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* HAL, stdint */
#include "usart.h"  /* huart3       */

/* ==========================================================================
 * Thông số vật lý UART/DMA
 * ========================================================================== */

/** Kích thước DMA circular buffer (bytes). Phải là bội số 2. Tối thiểu 128 */
#define CRSF_DMA_BUF_SIZE       128U

/** Thời gian timeout kết nối (ms). Nếu không có packet hợp lệ trong khoảng
 *  này thì coi như mất tín hiệu và is_connected = 0.
 *  CRSF@420kbaud ở 100 Hz → 10 ms/frame. Chọn 300 ms ≈ miss 30 frame. */
#define CRSF_TIMEOUT_MS         300U

/* ==========================================================================
 * Hằng số giao thức CRSF
 * ========================================================================== */

/** Địa chỉ thiết bị */
#define CRSF_ADDRESS_BROADCAST          0x00U
#define CRSF_ADDRESS_FLIGHT_CONTROLLER  0xC8U
#define CRSF_ADDRESS_RADIO_TRANSMITTER  0xEAU
#define CRSF_ADDRESS_CRSF_RECEIVER      0xECU
#define CRSF_ADDRESS_CRSF_TRANSMITTER   0xEEU

/** Loại frame */
#define CRSF_FRAMETYPE_BATTERY_SENSOR   0x08U
#define CRSF_FRAMETYPE_LINK_STATISTICS  0x14U
#define CRSF_FRAMETYPE_RC_CHANNELS      0x16U
#define CRSF_FRAMETYPE_ATTITUDE         0x1EU
#define CRSF_FRAMETYPE_FLIGHT_MODE      0x21U

/** Kích thước payload cố định */
#define CRSF_PAYLOAD_SIZE_RC_CHANNELS   22U   /* 16 kênh × 11-bit = 22 byte */
#define CRSF_PAYLOAD_SIZE_LINK_STATS    10U
#define CRSF_PAYLOAD_SIZE_BATTERY       8U
#define CRSF_PAYLOAD_SIZE_ATTITUDE      6U
#define CRSF_FLIGHT_MODE_STR_LEN        16U   /* bao gồm null terminator */

/** Số kênh RC tối đa */
#define CRSF_MAX_CHANNELS               16U

/** Giới hạn giá trị kênh CRSF thô (11-bit: 0..2047) */
#define CRSF_CHANNEL_MIN                172U   /* ≈  988 µs */
#define CRSF_CHANNEL_CENTER             992U   /* = 1500 µs */
#define CRSF_CHANNEL_MAX               1811U   /* ≈ 2012 µs */

/**
 * @brief Chuyển đổi giá trị kênh CRSF thô (11-bit) sang µs.
 *
 *  Công thức chính thức theo tài liệu CRSF:
 *    µs = (raw - 992) * 5 / 8 + 1500
 *
 *  Kết quả được clamp về [1000, 2000] trong CRSF_Input_GetChX().
 */
#define CRSF_TO_US(raw) \
    ((int32_t)(((int32_t)(raw) - 992) * 5 / 8 + 1500))

/* ==========================================================================
 * Cấu trúc dữ liệu
 * ========================================================================== */

/** Link Statistics (frame 0x14) */
typedef struct {
    uint8_t  uplink_rssi_ant1;       /**< RSSI anten 1 (dBm * -1) */
    uint8_t  uplink_rssi_ant2;       /**< RSSI anten 2 (dBm * -1) */
    uint8_t  uplink_link_quality;    /**< Link quality uplink (%) */
    int8_t   uplink_snr;             /**< SNR uplink (dB) */
    uint8_t  active_antenna;         /**< Anten đang dùng (0/1) */
    uint8_t  rf_mode;                /**< Chế độ RF profile */
    uint8_t  uplink_tx_power;        /**< Công suất TX */
    uint8_t  downlink_rssi;          /**< RSSI downlink (dBm * -1) */
    uint8_t  downlink_link_quality;  /**< Link quality downlink (%) */
    int8_t   downlink_snr;           /**< SNR downlink (dB) */
} CRSF_LinkStats_t;

/**
 * @brief Cấu trúc chứa toàn bộ dữ liệu được giải mã từ các frame CRSF.
 *
 *  Các trường "has_*" cho biết liệu dữ liệu tương ứng đã được nhận
 *  hay chưa (0 = chưa, 1 = đã có giá trị hợp lệ ít nhất 1 lần).
 */
typedef struct {
    /* --- RC Channels (frame 0x16) --- */
    uint16_t channels[CRSF_MAX_CHANNELS]; /**< Giá trị thô 11-bit [0..2047] */

    /* --- Kết nối & thời gian --- */
    uint8_t  is_connected;          /**< 1 = có tín hiệu, 0 = mất tín hiệu */
    uint32_t last_packet_time;      /**< HAL_GetTick() của gói hợp lệ cuối   */

    /* --- Link Statistics (frame 0x14) --- */
    CRSF_LinkStats_t link_stats;
    uint8_t  has_link_stats;

    /* --- Battery Sensor (frame 0x08) --- */
    uint32_t battery_voltage_mv;    /**< Điện áp pin (mV) */
    uint32_t battery_current_ma;    /**< Dòng tiêu thụ (mA) */
    uint32_t battery_capacity_mah;  /**< Dung lượng đã dùng (mAh) */
    uint8_t  battery_remaining_pct; /**< Pin còn lại (%) */
    uint8_t  has_battery;

    /* --- Attitude (frame 0x1E) --- */
    float    pitch_rad;             /**< Pitch (radian) */
    float    roll_rad;              /**< Roll (radian) */
    float    yaw_rad;               /**< Yaw (radian) */
    uint8_t  has_attitude;

    /* --- Flight Mode (frame 0x21) --- */
    char     flight_mode[CRSF_FLIGHT_MODE_STR_LEN];
    uint8_t  has_flight_mode;

} CRSF_Data_t;

/* ==========================================================================
 * API cấp thấp (Low-level parser) — dùng nội bộ hoặc cho unit test
 * ========================================================================== */

/**
 * @brief Khởi tạo cấu trúc dữ liệu CRSF về trạng thái mặc định (all zero).
 * @param data Con trỏ tới CRSF_Data_t cần khởi tạo.
 */
void CRSF_Init(CRSF_Data_t *data);

/**
 * @brief Reset toàn bộ dữ liệu về zero (tương đương memset 0).
 * @param data Con trỏ tới CRSF_Data_t.
 */
void CRSF_ResetData(CRSF_Data_t *data);

/**
 * @brief Parse một hoặc nhiều frame CRSF từ buffer thô.
 *
 *  Hàm quét qua buffer, tìm địa chỉ hợp lệ, kiểm tra CRC-8/DVB-S2
 *  và gọi handler tương ứng theo frame type.
 *
 * @param buffer  Con trỏ vùng nhớ chứa dữ liệu nhận được từ UART/DMA.
 * @param length  Số byte hợp lệ trong buffer.
 * @param data    Con trỏ tới cấu trúc output CRSF_Data_t.
 * @return        Số frame hợp lệ đã parse được (0 nếu không có frame nào).
 */
uint8_t CRSF_ParseFrame(const uint8_t *buffer, uint16_t length,
                         CRSF_Data_t *data);

/**
 * @brief Chuyển đổi giá trị kênh thô sang µs, clamp về [1000, 2000].
 *
 * @param data  Con trỏ tới CRSF_Data_t đã có dữ liệu.
 * @param ch    Số kênh (1-based: 1..16).
 * @return      Giá trị µs [1000..2000], hoặc 0 nếu mất kết nối / kênh không
 *              hợp lệ.
 */
uint16_t CRSF_GetChannelUs(const CRSF_Data_t *data, uint8_t ch);

/**
 * @brief Kiểm tra trạng thái kết nối, tự động reset is_connected nếu timeout.
 *
 * @param data  Con trỏ tới CRSF_Data_t.
 * @return      1 nếu đang có tín hiệu hợp lệ, 0 nếu mất tín hiệu.
 */
uint8_t CRSF_IsConnected(CRSF_Data_t *data);

/* ==========================================================================
 * API cấp cao (High-level input module) — giao tiếp với main.c
 *
 *  Module này quản lý DMA buffer, gọi CRSF_ParseFrame tự động trong
 *  CRSF_Input_Update(), và cung cấp giao diện đọc kênh tương tự rc_input.h.
 * ========================================================================== */

/**
 * @brief Khởi tạo module CRSF Input.
 *
 *  - Gọi CRSF_Init() khởi tạo dữ liệu.
 *  - Bắt đầu nhận DMA: HAL_UART_Receive_DMA(&huart3, dma_buf, DMA_BUF_SIZE).
 *  - Phải gọi SAU khi MX_USART3_UART_Init() và MX_DMA_Init() đã chạy.
 */
void CRSF_Input_Init(void);

/**
 * @brief Xử lý dữ liệu DMA mới, parse frame, cập nhật is_connected.
 *
 *  Gọi liên tục trong vòng lặp while(1) của main.
 *  Không block; chỉ parse những byte DMA đã nạp kể từ lần gọi trước.
 */
void CRSF_Input_Update(void);

/**
 * @brief Kiểm tra xem thiết bị CRSF có đang kết nối không.
 * @return 1 nếu có tín hiệu CRSF hợp lệ, 0 nếu không.
 */
uint8_t CRSF_Input_IsConnected(void);

/**
 * @brief Lấy giá trị kênh 1 (µs) từ nguồn CRSF.
 *        Dùng cho điều khiển Servo 1 (tương đương RC_Input_GetCh1).
 * @return µs trong [1000..2000], hoặc 0 nếu mất tín hiệu.
 */
uint32_t CRSF_Input_GetCh1(void);

/**
 * @brief Lấy giá trị kênh 2 (µs) từ nguồn CRSF.
 *        Dùng cho điều khiển Servo 2 (tương đương RC_Input_GetCh2).
 * @return µs trong [1000..2000], hoặc 0 nếu mất tín hiệu.
 */
uint32_t CRSF_Input_GetCh2(void);

/**
 * @brief Lấy giá trị kênh 6 (µs) từ nguồn CRSF.
 *        Dùng cho điều khiển Đèn / Lighting (kênh công tắc trên tay cầm).
 * @return µs trong [1000..2000], hoặc 0 nếu mất tín hiệu.
 */
uint32_t CRSF_Input_GetCh6(void);

#ifdef __cplusplus
}
#endif

#endif /* CRSF_H */

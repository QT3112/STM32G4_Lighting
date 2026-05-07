/**
 * @file    crsf.c
 * @brief   Thư viện CRSF (Crossfire Serial Protocol) cho STM32G4
 *
 * Triển khai:
 *   - CRC-8/DVB-S2 qua lookup table 256 phần tử
 *   - Parser đa frame type: RC Channels (0x16), Link Statistics (0x14),
 *     Battery Sensor (0x08), Attitude (0x1E), Flight Mode (0x21)
 *   - Module CRSF_Input_*: quản lý DMA circular buffer, tự parse trong
 *     main loop, cung cấp API đọc kênh tương tự rc_input.h
 *
 * Kết nối phần cứng:
 *   PB10 → USART3_TX  (không dùng cho chức năng nhận, để dự phòng Telemetry)
 *   PB11 → USART3_RX  (nối vào chân TX của bộ thu CRSF / ELRS Receiver)
 *
 * Lưu ý baud rate:
 *   CRSF chuẩn yêu cầu 416 666 baud. STM32G4 với PCLK1 = 170 MHz,
 *   DIV1, Oversampling=16: giá trị BRR gần nhất cho 420 000 baud (sai số < 1%).
 *   Hầu hết bộ thu CRSF/ELRS đều chấp nhận sai số này.
 */

#include "crsf.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>  /* printf for debugging */
#include <string.h> /* memset, memcpy */

/* ==========================================================================
 * CRC-8/DVB-S2 Lookup Table (polynomial 0xD5, init = 0x00)
 *
 * CRC tính trên: [Type] + [Payload]  (KHÔNG bao gồm Sync và Length).
 * ========================================================================== */
static const uint8_t s_crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83,
    0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 0xA4, 0x71, 0xDB, 0x0E,
    0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75,
    0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65,
    0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA,
    0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45,
    0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E,
    0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3,
    0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1,
    0xA5, 0x70, 0xDA, 0x0F, 0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 0xD6, 0x03, 0xA9, 0x7C,
    0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07,
    0x53, 0x86, 0x2C, 0xF9,
};

/* ==========================================================================
 * Hàm nội bộ: CRC-8/DVB-S2
 * ========================================================================== */
static uint8_t crsf_crc8(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc = s_crc8_table[crc ^ *data++];
  }
  return crc;
}

/* ==========================================================================
 * Hàm nội bộ: Kiểm tra địa chỉ CRSF hợp lệ
 * ========================================================================== */
static uint8_t crsf_is_valid_address(uint8_t addr) {
  /* Chỉ chấp nhận các địa chỉ thiết bị thực sự của CRSF.
   * KHAI BÁO: Loại bỏ 0x00 (BROADCAST) khỏi danh sách hợp lệ.
   * Lý do: Byte 0x00 xuất hiện rất thường xuyên trong payload CRSF
   * (ví dụ: giá trị kênh RC nhỏ, SNR âm, ...). Nếu có trong danh sách,
   * parser sẽ nhầm là đầu frame mới và có thể dừng quét sớm (break),
   * bỏ sót frame 0x16 RC Channels thực sự phía sau. */
  switch (addr) {
  case CRSF_ADDRESS_FLIGHT_CONTROLLER: /* 0xC8 - FC nhận từ Receiver */
  case CRSF_ADDRESS_RADIO_TRANSMITTER: /* 0xEA */
  case CRSF_ADDRESS_CRSF_RECEIVER:     /* 0xEC */
  case CRSF_ADDRESS_CRSF_TRANSMITTER:  /* 0xEE */
    return 1;
  default:
    return 0;
  }
}

/* ==========================================================================
 * Hàm nội bộ: Giải nén 16 kênh 11-bit từ 22 byte payload (frame 0x16)
 *
 *  Dữ liệu được đóng gói liên tiếp theo thứ tự bit little-endian:
 *    Kênh N bắt đầu tại bit offset (N × 11)
 * ========================================================================== */
static void crsf_unpack_channels(const uint8_t *p, CRSF_Data_t *d) {
  /*
   * 16 kênh x 11-bit = 176 bit = 22 byte, đóng gói liên tiếp little-endian.
   * Kênh N bắt đầu tại bit offset = N*11.
   * Các cast sang uint32_t để tránh shift overflow khi vượt quá 16-bit.
   */
  d->channels[0] =
      ((uint32_t)p[0] | (uint32_t)p[1] << 8) & 0x07FFU; /* bits  0-10 */
  d->channels[1] =
      ((uint32_t)p[1] >> 3 | (uint32_t)p[2] << 5) & 0x07FFU; /* bits 11-21 */
  d->channels[2] =
      ((uint32_t)p[2] >> 6 | (uint32_t)p[3] << 2 | (uint32_t)p[4] << 10) &
      0x07FFU; /* bits 22-32 */
  d->channels[3] =
      ((uint32_t)p[4] >> 1 | (uint32_t)p[5] << 7) & 0x07FFU; /* bits 33-43 */
  d->channels[4] =
      ((uint32_t)p[5] >> 4 | (uint32_t)p[6] << 4) & 0x07FFU; /* bits 44-54 */
  d->channels[5] =
      ((uint32_t)p[6] >> 7 | (uint32_t)p[7] << 1 | (uint32_t)p[8] << 9) &
      0x07FFU; /* bits 55-65 */
  d->channels[6] =
      ((uint32_t)p[8] >> 2 | (uint32_t)p[9] << 6) & 0x07FFU; /* bits 66-76 */
  d->channels[7] =
      ((uint32_t)p[9] >> 5 | (uint32_t)p[10] << 3) & 0x07FFU; /* bits 77-87 */
  /* --- Bug fix: CH8 bắt đầu tại bit 88 = byte p[11] bit0 (không phải p[11]
   * skip) --- */
  d->channels[8] =
      ((uint32_t)p[11] | (uint32_t)p[12] << 8) & 0x07FFU; /* bits 88-98  */
  d->channels[9] =
      ((uint32_t)p[12] >> 3 | (uint32_t)p[13] << 5) & 0x07FFU; /* bits 99-109 */
  d->channels[10] =
      ((uint32_t)p[13] >> 6 | (uint32_t)p[14] << 2 | (uint32_t)p[15] << 10) &
      0x07FFU; /* bits 110-120 */
  d->channels[11] = ((uint32_t)p[15] >> 1 | (uint32_t)p[16] << 7) &
                    0x07FFU; /* bits 121-131 */
  d->channels[12] = ((uint32_t)p[16] >> 4 | (uint32_t)p[17] << 4) &
                    0x07FFU; /* bits 132-142 */
  d->channels[13] =
      ((uint32_t)p[17] >> 7 | (uint32_t)p[18] << 1 | (uint32_t)p[19] << 9) &
      0x07FFU; /* bits 143-153 */
  d->channels[14] = ((uint32_t)p[19] >> 2 | (uint32_t)p[20] << 6) &
                    0x07FFU; /* bits 154-164 */
  d->channels[15] = ((uint32_t)p[20] >> 5 | (uint32_t)p[21] << 3) &
                    0x07FFU; /* bits 165-175 */
}

/* ==========================================================================
 * Hàm nội bộ: Xử lý từng loại frame sau khi CRC đã được xác nhận đúng
 *
 * @param p        Trỏ vào đầu Payload (byte ngay sau [Type])
 * @param type     Loại frame
 * @param pay_len  Độ dài payload = frame_len - 2 (trừ type + crc)
 * @param data     Output data struct
 * ========================================================================== */
static void crsf_handle_frame(const uint8_t *p, uint8_t type, uint8_t pay_len,
                              CRSF_Data_t *data) {
  /* Ghi nhận thời điểm nhận packet hợp lệ (bất kể loại nào) */
  data->last_packet_time = HAL_GetTick();

  switch (type) {

  /* ------------------------------------------------------------------
   * RC Channels (0x16): 16 kênh × 11-bit, payload = 22 byte
   * ------------------------------------------------------------------ */
  case CRSF_FRAMETYPE_RC_CHANNELS:
    if (pay_len == CRSF_PAYLOAD_SIZE_RC_CHANNELS) {
      crsf_unpack_channels(p, data);
      data->is_connected = 1;

      /* [DEBUG] In ra giá trị của 3 kênh quan trọng mỗi 500ms để tránh lag
       * console */
      static uint32_t last_print = 0;
      if (HAL_GetTick() - last_print > 500) {
        printf(
            "[CRSF] CH1 (Servo1): %lu, CH2 (Servo2): %lu, CH6 (Light): %lu\n",
            (unsigned long)data->channels[0], (unsigned long)data->channels[1],
            (unsigned long)data->channels[5]);
        last_print = HAL_GetTick();
      }
    }
    break;

  /* ------------------------------------------------------------------
   * Link Statistics (0x14): 10 byte
   * [0] RSSI Ant1  [1] RSSI Ant2  [2] LQ  [3] SNR  [4] Active Ant
   * [5] RF Mode  [6] TX Power  [7] DL RSSI  [8] DL LQ  [9] DL SNR
   * ------------------------------------------------------------------ */
  case CRSF_FRAMETYPE_LINK_STATISTICS:
    if (pay_len >= CRSF_PAYLOAD_SIZE_LINK_STATS) {
      data->link_stats.uplink_rssi_ant1 = p[0];
      data->link_stats.uplink_rssi_ant2 = p[1];
      data->link_stats.uplink_link_quality = p[2];
      data->link_stats.uplink_snr = (int8_t)p[3];
      data->link_stats.active_antenna = p[4];
      data->link_stats.rf_mode = p[5];
      data->link_stats.uplink_tx_power = p[6];
      data->link_stats.downlink_rssi = p[7];
      data->link_stats.downlink_link_quality = p[8];
      data->link_stats.downlink_snr = (int8_t)p[9];
      data->has_link_stats = 1;
    }
    break;

  /* ------------------------------------------------------------------
   * Battery Sensor (0x08): 8 byte (Big-Endian)
   * [0-1] Voltage ×10 µV  [2-3] Current ×10 µA
   * [4-6] Capacity mAh (uint24)  [7] Remaining %
   * ------------------------------------------------------------------ */
  case CRSF_FRAMETYPE_BATTERY_SENSOR:
    if (pay_len >= CRSF_PAYLOAD_SIZE_BATTERY) {
      uint16_t raw_v = ((uint16_t)p[0] << 8) | p[1]; /* ×10 mV  */
      uint16_t raw_i = ((uint16_t)p[2] << 8) | p[3]; /* ×10 mA  */
      uint32_t raw_c =
          ((uint32_t)p[4] << 16) | ((uint32_t)p[5] << 8) | (uint32_t)p[6];

      data->battery_voltage_mv = (uint32_t)raw_v * 10U;
      data->battery_current_ma = (uint32_t)raw_i * 10U;
      data->battery_capacity_mah = raw_c;
      data->battery_remaining_pct = p[7];
      data->has_battery = 1;
    }
    break;

  /* ------------------------------------------------------------------
   * Attitude (0x1E): 6 byte (Big-Endian int16 × 10000 rad)
   * [0-1] Pitch  [2-3] Roll  [4-5] Yaw
   * ------------------------------------------------------------------ */
  case CRSF_FRAMETYPE_ATTITUDE:
    if (pay_len >= CRSF_PAYLOAD_SIZE_ATTITUDE) {
      int16_t raw_pitch = (int16_t)(((uint16_t)p[0] << 8) | p[1]);
      int16_t raw_roll = (int16_t)(((uint16_t)p[2] << 8) | p[3]);
      int16_t raw_yaw = (int16_t)(((uint16_t)p[4] << 8) | p[5]);

      data->pitch_rad = (float)raw_pitch / 10000.0f;
      data->roll_rad = (float)raw_roll / 10000.0f;
      data->yaw_rad = (float)raw_yaw / 10000.0f;
      data->has_attitude = 1;
    }
    break;

  /* ------------------------------------------------------------------
   * Flight Mode (0x21): null-terminated string, tối đa 15 ký tự + '\0'
   * ------------------------------------------------------------------ */
  case CRSF_FRAMETYPE_FLIGHT_MODE:
    if (pay_len > 0) {
      uint8_t copy_len = (pay_len < (CRSF_FLIGHT_MODE_STR_LEN - 1U))
                             ? pay_len
                             : (CRSF_FLIGHT_MODE_STR_LEN - 1U);
      memcpy(data->flight_mode, p, copy_len);
      data->flight_mode[copy_len] = '\0';
      data->has_flight_mode = 1;
    }
    break;

  default:
    /* Frame type không hỗ trợ: bỏ qua */
    break;
  }
}

/* ==========================================================================
 * API cấp thấp: CRSF_Init / CRSF_ResetData
 * ========================================================================== */
void CRSF_Init(CRSF_Data_t *data) { CRSF_ResetData(data); }

void CRSF_ResetData(CRSF_Data_t *data) { memset(data, 0, sizeof(CRSF_Data_t)); }

/* ==========================================================================
 * API cấp thấp: CRSF_ParseFrame
 *
 * Cấu trúc frame CRSF (Broadcast Frame):
 *   [i+0] Address/Sync  (1 byte) : 0xC8, 0xEA, 0xEC, 0xEE, 0x00
 *   [i+1] Frame Length  (1 byte) : Type(1) + Payload(N) + CRC(1) → N+2
 *   [i+2] Frame Type    (1 byte) : xem CRSF_FRAMETYPE_*
 *   [i+3 .. i+2+N] Payload       : N byte = frame_len - 2
 *   [i+1+frame_len] CRC8          : tính trên [Type]+[Payload]
 *
 * Trả về số frame hợp lệ đã parse được.
 * ========================================================================== */
uint8_t CRSF_ParseFrame(const uint8_t *buffer, uint16_t length,
                        CRSF_Data_t *data) {
  if (!buffer || !data || length < 4U) {
    return 0;
  }

  uint8_t frames_parsed = 0;

  for (uint16_t i = 0; i < length; i++) {

    /* Bỏ qua byte không phải địa chỉ hợp lệ */
    if (!crsf_is_valid_address(buffer[i])) {
      continue;
    }

    /* Cần ít nhất: Addr(1) + Len(1) + Type(1) → i+2 phải < length */
    if ((i + 2U) >= length) {
      break;
    }

    uint8_t frame_len = buffer[i + 1U];

    /* frame_len hợp lệ: 2 (type+crc) ≤ frame_len ≤ 62 */
    if (frame_len < 2U || frame_len > 62U) {
      continue;
    }

    /* Vị trí byte CRC = i + 1 (bỏ qua addr+len) + frame_len */
    uint16_t crc_idx = (uint16_t)(i + 1U + frame_len);
    /* Trường hợp chưa có đủ dữ liệu cho frame này:
     * Dùng continue thay vì break để tiếp tục tìm frame hợp lệ phía sau.
     * Byte sync này có thể là false positive (byte dữ liệu có giá trị trùng địa
     * chỉ). */
    if (crc_idx >= length) {
      continue;
    }

    uint8_t frame_type = buffer[i + 2U];
    const uint8_t *payload = &buffer[i + 3U];
    uint8_t pay_len = frame_len - 2U; /* trừ type(1) + crc(1) */

    /* Kiểm tra payload không vượt giới hạn buffer */
    if ((uint16_t)(i + 3U) + pay_len > length) {
      continue;
    }

    /* Tính CRC trên [Type] + [Payload] = (frame_len - 1) byte từ [i+2] */
    uint8_t crc_calc = crsf_crc8(&buffer[i + 2U], (uint16_t)(frame_len - 1U));
    uint8_t crc_received = buffer[crc_idx];

    if (crc_calc != crc_received) {
      /* [DEBUG] Báo lỗi CRC. In log có giới hạn (rate-limit) để không bị spam
       */
      static uint32_t last_crc_err = 0;
      if (HAL_GetTick() - last_crc_err > 1000) {
        printf("[CRSF Error] CRC Mismatch! Calc: 0x%02X, Recv: 0x%02X\n",
               crc_calc, crc_received);
        last_crc_err = HAL_GetTick();
      }
      continue; /* CRC không khớp → bỏ qua frame này */
    }

    /* Frame hợp lệ → xử lý */
    /* [DEBUG] In ra data thô của frame mỗi 1000ms để kiểm tra */
    static uint32_t last_raw_print = 0;
    if (HAL_GetTick() - last_raw_print > 1000) {
      printf("[CRSF RAW] ");
      for (uint16_t j = 0; j < frame_len + 2U; j++) {
        printf("%02X ", buffer[i + j]);
      }
      printf("\n");
      last_raw_print = HAL_GetTick();
    }

    crsf_handle_frame(payload, frame_type, pay_len, data);
    frames_parsed++;

    /* Nhảy qua toàn bộ frame: addr(1) + len_field(1) + frame_len bytes */
    i += (uint16_t)(1U + frame_len);
  }

  return frames_parsed;
}

/* ==========================================================================
 * API cấp thấp: CRSF_GetChannelUs
 * ========================================================================== */
uint16_t CRSF_GetChannelUs(const CRSF_Data_t *data, uint8_t ch) {
  if (!data || ch < 1U || ch > CRSF_MAX_CHANNELS) {
    return 0U;
  }
  if (!data->is_connected) {
    return 0U;
  }

  int32_t raw = (int32_t)data->channels[ch - 1U]; /* 0-based */
  int32_t us = CRSF_TO_US(raw);

  /* Clamp về [1000, 2000] */
  if (us < 1000)
    us = 1000;
  if (us > 2000)
    us = 2000;

  return (uint16_t)us;
}

/* ==========================================================================
 * API cấp thấp: CRSF_IsConnected
 * ========================================================================== */
uint8_t CRSF_IsConnected(CRSF_Data_t *data) {
  if (!data) {
    return 0U;
  }

  if (data->is_connected) {
    if ((HAL_GetTick() - data->last_packet_time) > CRSF_TIMEOUT_MS) {
      data->is_connected = 0;
    }
  }
  return data->is_connected;
}

/* ==========================================================================
 * Module High-Level: CRSF_Input_*
 *
 *  Quản lý DMA circular buffer nội bộ.
 *  Chiến lược:
 *    - DMA liên tục ghi vào s_dma_buf[CRSF_DMA_BUF_SIZE] theo vòng tròn.
 *    - CRSF_Input_Update() đọc vị trí đầu ghi hiện tại qua
 *      __HAL_DMA_GET_COUNTER và tính số byte mới kể từ lần cập nhật trước.
 *    - Copy dữ liệu mới vào parse_buf tuyến tính rồi gọi CRSF_ParseFrame.
 *
 *  Vì DMA circular mode không có ngắt bắt buộc, cách này hoàn toàn
 *  non-blocking và không cần HAL DMA callback.
 * ========================================================================== */

/** Buffer DMA nhận dữ liệu từ USART3 (circular, không cần ngắt) */
static uint8_t s_dma_buf[CRSF_DMA_BUF_SIZE];

/** Vị trí đọc cuối cùng trong s_dma_buf (0-based index) */
static uint16_t s_read_pos;

/** Cấu trúc dữ liệu CRSF được parse và cập nhật */
static CRSF_Data_t s_crsf_data;

/* -------------------------------------------------------------------------- */
void CRSF_Input_Init(void) {
  /* Khởi tạo dữ liệu về zero */
  CRSF_Init(&s_crsf_data);

  /* Đặt vị trí đọc ban đầu ở đầu buffer */
  s_read_pos = 0U;

  /* Bắt đầu nhận DMA Circular — UART liên tục ghi vào s_dma_buf */
  HAL_UART_Receive_DMA(&huart3, s_dma_buf, CRSF_DMA_BUF_SIZE);
}

/* -------------------------------------------------------------------------- */
void CRSF_Input_Update(void) {
  /* Lấy số byte DMA còn lại trong counter.
   * DMA_COUNTER = số byte chưa được ghi (đếm ngược từ CRSF_DMA_BUF_SIZE).
   * Vị trí đầu ghi hiện tại (write_pos) = CRSF_DMA_BUF_SIZE - COUNTER */
  uint16_t dma_counter = (uint16_t)__HAL_DMA_GET_COUNTER(huart3.hdmarx);
  uint16_t write_pos = CRSF_DMA_BUF_SIZE - dma_counter;

  if (write_pos != s_read_pos) {
    /* [DEBUG] Gom toàn bộ byte rác/thô vào buffer để in mỗi giây */
    static uint8_t debug_buf[512];
    static uint16_t debug_buf_idx = 0;
    
    uint16_t ptr_dbg = s_read_pos;
    while (ptr_dbg != write_pos) {
        if (debug_buf_idx < sizeof(debug_buf)) {
            debug_buf[debug_buf_idx++] = s_dma_buf[ptr_dbg];
        }
        ptr_dbg = (ptr_dbg + 1) % CRSF_DMA_BUF_SIZE;
    }

    static uint32_t last_dma_print = 0;
    if (HAL_GetTick() - last_dma_print > 1000) {
      if (debug_buf_idx > 0) {
        printf("[DMA RAW %d bytes] ", debug_buf_idx);
        for (uint16_t j = 0; j < debug_buf_idx; j++) {
            printf("%02X ", debug_buf[j]);
        }
        printf("\n");
        debug_buf_idx = 0;
      }
      last_dma_print = HAL_GetTick();
    }
    /* Kết thúc [DEBUG] */

    /* Có dữ liệu mới từ DMA, tiến hành parse */
    if (write_pos > s_read_pos) {
      /* Trường hợp đơn giản: không bị wrap */
      uint16_t new_bytes = write_pos - s_read_pos;
      CRSF_ParseFrame(&s_dma_buf[s_read_pos], new_bytes, &s_crsf_data);
    } else {
      /* Trường hợp wrap: dữ liệu bị chia làm 2 đoạn */
      uint16_t bytes_to_end = CRSF_DMA_BUF_SIZE - s_read_pos;
      uint16_t bytes_from_start = write_pos;

      /* Parse đoạn 1: từ s_read_pos đến cuối buffer */
      if (bytes_to_end > 0U) {
        CRSF_ParseFrame(&s_dma_buf[s_read_pos], bytes_to_end, &s_crsf_data);
      }
      /* Parse đoạn 2: từ đầu buffer đến write_pos */
      if (bytes_from_start > 0U) {
        CRSF_ParseFrame(&s_dma_buf[0U], bytes_from_start, &s_crsf_data);
      }
    }

    s_read_pos = write_pos;
  }

  /* Cập nhật trạng thái kết nối (xử lý timeout) */
  uint8_t was_connected = s_crsf_data.is_connected;
  CRSF_IsConnected(&s_crsf_data);

  /* [DEBUG] In log khi mất hoặc có lại kết nối CRSF */
  if (was_connected != s_crsf_data.is_connected) {
    if (s_crsf_data.is_connected) {
      printf("[CRSF Status] Connected to Receiver!\n");
    } else {
      printf("[CRSF Status] Connection LOST (Timeout)!\n");
    }
  }
}

/* -------------------------------------------------------------------------- */
uint8_t CRSF_Input_IsConnected(void) {
  /* Luôn chạy timeout check trước khi trả về trạng thái */
  return CRSF_IsConnected(&s_crsf_data);
}

/* -------------------------------------------------------------------------- */
uint32_t CRSF_Input_GetCh1(void) {
  return (uint32_t)CRSF_GetChannelUs(&s_crsf_data, 1U); /* Servo 1 */
}

/* -------------------------------------------------------------------------- */
uint32_t CRSF_Input_GetCh2(void) {
  return (uint32_t)CRSF_GetChannelUs(&s_crsf_data, 2U); /* Servo 2 */
}

/* -------------------------------------------------------------------------- */
uint32_t CRSF_Input_GetCh6(void) {
  return (uint32_t)CRSF_GetChannelUs(&s_crsf_data, 6U); /* Điều khiển đèn */
}

/*----- Đây là hàm ví dụ để kiểm tra file crsf.c có sử dụng được printf() hay
 * không-----*/
void CRSF_test_printf(void) { printf("[CRSF] Testing printf...\n"); }
/*----- Kết thúc hàm ví dụ ----*/
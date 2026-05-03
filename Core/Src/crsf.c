/**
 * @file    crsf.c
 * @brief   Thư viện CRSF (Crossfire Serial Protocol) cho STM32G4
 *
 * Triển khai:
 *   - CRC-8/DVB-S2 qua lookup table 256 phần tử (nhanh hơn ~8× so với tính từng
 * bit)
 *   - Parser đa frame type: RC Channels, Link Statistics, Battery, Attitude,
 * Flight Mode
 *   - API: CRSF_Init, CRSF_ParseFrame, CRSF_GetChannelUs, CRSF_IsConnected,
 * CRSF_ResetData
 */

#include "crsf.h"
#include <string.h> /* memset, memcpy, strncpy - có sẵn trong CMSIS/toolchain */
#include "usart.h"
#include "stdio.h"

/* ==========================================================================
 * CRC-8/DVB-S2 Lookup Table (polynomial 0xD5, init=0)
 *
 * Được tính sẵn tại compile time để tránh tính toán lặp lại ở runtime.
 * Cách dùng: crc = table[crc ^ byte], lặp qua toàn bộ dữ liệu.
 * ========================================================================== */
static const uint8_t crsf_crc8_table[256] = {
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

/* --------------------------------------------------------------------------
 * Tính CRC-8/DVB-S2 với lookup table
 * data: dữ liệu cần tính CRC (bắt đầu từ [Type], KHÔNG bao gồm Address/Length)
 * len : số byte cần tính
 * -------------------------------------------------------------------------- */
static uint8_t crsf_crc8(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc = crsf_crc8_table[crc ^ *data++];
  }
  return crc;
}

/* ==========================================================================
 * Giải nén (unpack) 16 kênh 11-bit từ 22 byte payload
 *
 * Dữ liệu được đóng gói liên tiếp theo thứ tự bit little-endian:
 *   Kênh N bắt đầu tại bit offset (N × 11)
 *   Byte index = bit_offset / 8
 *   Bit offset trong byte = bit_offset % 8
 * ========================================================================== */
static void crsf_unpack_channels(const uint8_t *p, CRSF_Data_t *d) {
  /* Mỗi dòng: lấy đúng 11 bit từ stream bit little-endian */
  d->channels[0] = ((uint16_t)p[0] | (uint16_t)p[1] << 8) & 0x07FF;
  d->channels[1] = ((uint16_t)p[1] >> 3 | (uint16_t)p[2] << 5) & 0x07FF;
  d->channels[2] =
      ((uint16_t)p[2] >> 6 | (uint16_t)p[3] << 2 | (uint16_t)p[4] << 10) &
      0x07FF;
  d->channels[3] = ((uint16_t)p[4] >> 1 | (uint16_t)p[5] << 7) & 0x07FF;
  d->channels[4] = ((uint16_t)p[5] >> 4 | (uint16_t)p[6] << 4) & 0x07FF;
  d->channels[5] =
      ((uint16_t)p[6] >> 7 | (uint16_t)p[7] << 1 | (uint16_t)p[8] << 9) &
      0x07FF;
  d->channels[6] = ((uint16_t)p[8] >> 2 | (uint16_t)p[9] << 6) & 0x07FF;
  d->channels[7] = ((uint16_t)p[9] >> 5 | (uint16_t)p[10] << 3) & 0x07FF;
  d->channels[8] = ((uint16_t)p[11] | (uint16_t)p[12] << 8) & 0x07FF;
  d->channels[9] = ((uint16_t)p[12] >> 3 | (uint16_t)p[13] << 5) & 0x07FF;
  d->channels[10] =
      ((uint16_t)p[13] >> 6 | (uint16_t)p[14] << 2 | (uint16_t)p[15] << 10) &
      0x07FF;
  d->channels[11] = ((uint16_t)p[15] >> 1 | (uint16_t)p[16] << 7) & 0x07FF;
  d->channels[12] = ((uint16_t)p[16] >> 4 | (uint16_t)p[17] << 4) & 0x07FF;
  d->channels[13] =
      ((uint16_t)p[17] >> 7 | (uint16_t)p[18] << 1 | (uint16_t)p[19] << 9) &
      0x07FF;
  d->channels[14] = ((uint16_t)p[19] >> 2 | (uint16_t)p[20] << 6) & 0x07FF;
  d->channels[15] = ((uint16_t)p[20] >> 5 | (uint16_t)p[21] << 3) & 0x07FF;
}

/* ==========================================================================
 * Kiểm tra địa chỉ có hợp lệ trong CRSF không
 * ========================================================================== */
static uint8_t crsf_is_valid_address(uint8_t addr) {
  switch (addr) {
  case CRSF_ADDRESS_FLIGHT_CONTROLLER: /* 0xC8 */
  case CRSF_ADDRESS_RADIO_TRANSMITTER: /* 0xEA */
  case CRSF_ADDRESS_CRSF_RECEIVER:     /* 0xEC */
  case CRSF_ADDRESS_CRSF_TRANSMITTER:  /* 0xEE */
  case CRSF_ADDRESS_BROADCAST:         /* 0x00 */
    return 1;
  default:
    return 0;
  }
}

/* ==========================================================================
 * Xử lý từng loại frame sau khi đã xác nhận CRC đúng
 * p        : trỏ vào đầu payload (byte sau [Type])
 * type     : loại frame
 * pay_len  : payload length = frame_len - 2 (trừ type + crc)
 * data     : output data struct
 * ========================================================================== */
static void crsf_handle_frame(const uint8_t *p, uint8_t type, uint8_t pay_len,
                              CRSF_Data_t *data) {
  data->last_packet_time = HAL_GetTick();

  switch (type) {

    printf("type: %d\n", type);

  /* ------------------------------------------------------------------
   * RC Channels (0x16): 16 kênh × 11-bit, payload = 22 bytes
   * ------------------------------------------------------------------ */
  case CRSF_FRAMETYPE_RC_CHANNELS:
    if (pay_len == CRSF_PAYLOAD_SIZE_RC_CHANNELS) {
      crsf_unpack_channels(p, data);
      data->is_connected = 1;
    }
    break;

  /* ------------------------------------------------------------------
   * Link Statistics (0x14): 10 bytes
   * [0] Uplink RSSI Ant1  [1] Uplink RSSI Ant2  [2] Uplink LQ
   * [3] Uplink SNR        [4] Active Antenna     [5] RF Mode
   * [6] Uplink TX Power   [7] Downlink RSSI      [8] Downlink LQ
   * [9] Downlink SNR
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
   * Battery Sensor (0x08): 8 bytes (Big-Endian)
   * [0-1] Voltage  ×10 mV  (BE uint16) → chia 10 → mV
   * [2-3] Current  ×10 mA  (BE uint16)
   * [4-6] Capacity mAh     (BE uint24)
   * [7]   Remaining %      (uint8)
   * ------------------------------------------------------------------ */
  case CRSF_FRAMETYPE_BATTERY_SENSOR:
    if (pay_len >= CRSF_PAYLOAD_SIZE_BATTERY) {
      /* Big-Endian → chuyển về Little-Endian */
      uint16_t raw_v = ((uint16_t)p[0] << 8) | p[1]; /* ×10 mV */
      uint16_t raw_i = ((uint16_t)p[2] << 8) | p[3]; /* ×10 mA */
      uint32_t raw_c = ((uint32_t)p[4] << 16) | ((uint32_t)p[5] << 8) | p[6];

      data->battery_voltage_mv = raw_v * 10U; /* chuyển về mV */
      data->battery_current_ma = raw_i * 10U; /* chuyển về mA */
      data->battery_capacity_mah = raw_c;
      data->battery_remaining_pct = p[7];
      data->has_battery = 1;
    }
    break;

  /* ------------------------------------------------------------------
   * Attitude (0x1E): 6 bytes (Big-Endian int16)
   * [0-1] Pitch × 10000 rad  [2-3] Roll × 10000 rad  [4-5] Yaw × 10000 rad
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
      uint8_t copy_len = (pay_len < (CRSF_FLIGHT_MODE_STR_LEN - 1))
                             ? pay_len
                             : (CRSF_FLIGHT_MODE_STR_LEN - 1);
      memcpy(data->flight_mode, p, copy_len);
      data->flight_mode[copy_len] = '\0'; /* đảm bảo null-terminated */
      data->has_flight_mode = 1;
    }
    break;

  default:
    /* Frame type không hỗ trợ: bỏ qua */
    break;
  }
}

/* ==========================================================================
 * API: CRSF_Init
 * ========================================================================== */
void CRSF_Init(CRSF_Data_t *data) { CRSF_ResetData(data); }

/* ==========================================================================
 * API: CRSF_ResetData
 * ========================================================================== */
void CRSF_ResetData(CRSF_Data_t *data) { memset(data, 0, sizeof(CRSF_Data_t)); }

/* ==========================================================================
 * API: CRSF_ParseFrame
 *
 * Cấu trúc frame CRSF:
 *   [i+0] Address/Sync  (1 byte) : 0xC8, 0xEA, 0xEC, 0xEE ...
 *   [i+1] Frame Length  (1 byte) : = sizeof(Type) + sizeof(Payload) +
 * sizeof(CRC) [i+2] Frame Type    (1 byte) : xem CRSF_FrameType_e
 *   [i+3..i+3+pay_len-1] Payload
 *   [i+1+frame_len] CRC8         : tính trên [Type]+[Payload]
 *
 * Trả về số frame hợp lệ đã parse được.
 * ========================================================================== */
uint8_t CRSF_ParseFrame(const uint8_t *buffer, uint16_t length,
                        CRSF_Data_t *data) {
  if (!buffer || !data || length < 4)
    return 0;

  uint8_t frames_parsed = 0;

  for (uint16_t i = 0; i < length; i++) {

    /* Bỏ qua các byte không phải địa chỉ hợp lệ */
    if (!crsf_is_valid_address(buffer[i]))
      continue;

    /* Kiểm tra còn đủ byte để đọc header */
    if (i + 2 >= length)
      break;

    uint8_t frame_len = buffer[i + 1];

    /* frame_len: ít nhất 2 (type + crc), tối đa 62 */
    if (frame_len < 2 || frame_len > 62)
      continue;

    /* Vị trí CRC = i + 1 (addr+len không tính) + frame_len = i + 1 + frame_len
     */
    uint16_t crc_idx = i + 1 + frame_len;
    if (crc_idx >= length)
      continue; /* chưa nhận đủ frame */

    /* Kiểm tra còn đủ buffer */
    if (crc_idx > length - 1)
      continue;

    uint8_t frame_type = buffer[i + 2];
    const uint8_t *payload = &buffer[i + 3];
    uint8_t pay_len = frame_len - 2; /* trừ type (1) + crc (1) */

    /* Kiểm tra payload không vượt buffer */
    if ((uint16_t)(i + 3) + pay_len > length)
      continue;

    /* Tính CRC trên: [Type] + [Payload] = frame_len - 1 bytes bắt đầu từ
     * buffer[i+2] */
    uint8_t crc_calc = crsf_crc8(&buffer[i + 2], (uint16_t)(frame_len - 1));
    uint8_t crc_received = buffer[crc_idx];

    if (crc_calc != crc_received)
      continue; /* CRC không khớp → bỏ qua */

    /* Frame hợp lệ: xử lý theo loại */
    crsf_handle_frame(payload, frame_type, pay_len, data);
    frames_parsed++;

    /* Nhảy qua frame này để tìm frame tiếp theo */
    i += (1 + frame_len); /* = addr(1) + frame_len bytes sau addr */
  }

  return frames_parsed;
}

/* ==========================================================================
 * API: CRSF_GetChannelUs
 * ========================================================================== */
uint16_t CRSF_GetChannelUs(const CRSF_Data_t *data, uint8_t ch) {
  if (!data || ch < 1 || ch > CRSF_MAX_CHANNELS)
    return 0;
  if (!data->is_connected)
    return 0;

  int32_t raw = (int32_t)data->channels[ch - 1]; /* 0-based index */
  int32_t us = CRSF_TO_US(raw);

  /* Clamp về [1000, 2000] */
  if (us < 1000)
    us = 1000;
  if (us > 2000)
    us = 2000;

  return (uint16_t)us;
}

/* ==========================================================================
 * API: CRSF_IsConnected
 * ========================================================================== */
uint8_t CRSF_IsConnected(CRSF_Data_t *data) {
  if (!data)
    return 0;

  if (data->is_connected) {
    /* Kiểm tra timeout: nếu quá CRSF_TIMEOUT_MS ms kể từ packet cuối */
    if ((HAL_GetTick() - data->last_packet_time) > CRSF_TIMEOUT_MS) {
      data->is_connected = 0;
    }
  }
  return data->is_connected;
}

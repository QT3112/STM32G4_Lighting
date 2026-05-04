/**
 * @file    mode_manager.h
 * @brief   Quản lý chế độ hoạt động: NORMAL ↔ DEMO.
 *
 *  Cơ chế kích hoạt: phát hiện cử chỉ "bật/tắt đèn nhanh 3 lần" trên RC CH1.
 *
 *  Thuật toán:
 *    - Theo dõi mỗi lần tín hiệu CH1 TIẾN VÀO vùng sáng (ON-zone rising edge).
 *    - Nếu 3 lần tiến vào ON-zone xảy ra trong vòng GESTURE_WINDOW_MS ms:
 *        → Toggle chế độ (NORMAL → DEMO hoặc DEMO → NORMAL).
 *    - Nếu cửa sổ thời gian hết hạn trước khi đủ 3 lần: reset bộ đếm.
 *
 *  Vùng ON được dùng để phát hiện cử chỉ:
 *    LIGHT_ZONE_ON_MIN ≤ pulse < LIGHT_ZONE_ON_MAX  (tái sử dụng từ lighting_control.h)
 */

#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include "main.h"
#include "lighting_control.h"   /* Dùng lại ngưỡng LIGHT_ZONE_ON_MIN/MAX */

/* Tham số phát hiện cử chỉ ------------------------------------------------- */
/* Số lần bật đèn (rising edge vào ON-zone) cần để kích hoạt toggle.          */
#define GESTURE_TRIGGER_COUNT   3U

/* Cửa sổ thời gian tối đa cho 3 lần bật (ms).                               */
/* Nếu các lần bật không hoàn thành trong khoảng này, bộ đếm reset.           */
#define GESTURE_WINDOW_MS       3000U

/* Kiểu enum chế độ hoạt động ----------------------------------------------- */
typedef enum {
    APP_MODE_NORMAL = 0,    /* Chế độ điều khiển bình thường (RC → servo + đèn) */
    APP_MODE_DEMO           /* Chế độ biểu diễn tự động (Demo_Performance)       */
} AppMode;

/**
 * @brief Khởi tạo module. Chế độ ban đầu là APP_MODE_NORMAL.
 */
void Mode_Init(void);

/**
 * @brief Cập nhật bộ phát hiện cử chỉ theo giá trị pulse RC CH1.
 *        Gọi liên tục trong vòng lặp while(1), trước khi dùng Mode_Get().
 * @param pulseCh1  Pulse width kênh RC 1 (µs), 0 = mất tín hiệu.
 */
void Mode_Update(uint32_t pulseCh1);

/**
 * @brief Lấy chế độ hoạt động hiện tại.
 * @return APP_MODE_NORMAL hoặc APP_MODE_DEMO.
 */
AppMode Mode_Get(void);

#endif /* MODE_MANAGER_H */

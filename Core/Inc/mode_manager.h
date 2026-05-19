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

/* Số lần bật đèn để toggle NORMAL ↔ DEMO */
#define GESTURE_TRIGGER_COUNT   3U

/* Số lần bật đèn để kích hoạt GIMBAL mode */
#define GESTURE_GIMBAL_COUNT    5U

/* Cửa sổ thời gian tối đa cho cả hai cử chỉ (ms). */
#define GESTURE_WINDOW_MS       3000U

/* Kiểu enum chế độ hoạt động ----------------------------------------------- */
typedef enum {
    APP_MODE_NORMAL = 0,    /*!< Chế độ điều khiển bình thường (RC → servo + đèn) */
    APP_MODE_DEMO,          /*!< Chế độ biểu diễn tự động (Demo_Performance)       */
    APP_MODE_GIMBAL         /*!< Chế độ cân bằng Gimbal (Cascaded PID + Kalman)    */
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
 * @return APP_MODE_NORMAL, APP_MODE_DEMO, hoặc APP_MODE_GIMBAL.
 */
AppMode Mode_Get(void);

/**
 * @brief Ép buộc đặt chế độ trực tiếp (dùng nội bộ hoặc test).
 * @param mode  Chế độ muốn chuyển sang.
 */
void Mode_Set(AppMode mode);

#endif /* MODE_MANAGER_H */

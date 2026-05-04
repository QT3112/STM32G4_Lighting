/**
 ******************************************************************************
 * @file    demo_performance.h
 * @brief   Header cho module biểu diễn thiết bị (Demo Performance).
 *
 * Module này cung cấp hàm Demo_Performance() – chạy một chuỗi hoạt động
 * demo 7 bước theo vòng lặp, hoàn toàn độc lập với tín hiệu điều khiển
 * ngõ vào (RC PWM). Gọi hàm này lặp lại trong while(1) của main.
 ******************************************************************************
 */

#ifndef DEMO_PERFORMANCE_H
#define DEMO_PERFORMANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"   /* HAL, TIM_HandleTypeDef, GPIO defs */
#include "tim.h"    /* htim3 extern */

/* Exported defines ----------------------------------------------------------*/

/** Biên độ servo: 500 µs (0°) – 2500 µs (180°) */
#define DEMO_SERVO_MIN          500     /**< Vị trí gốc / mặc định (µs) */
#define DEMO_SERVO_MAX          2500    /**< Vị trí tận cùng (µs)        */
#define DEMO_SERVO_MID          1500    /**< Giữa hành trình (µs)        */

/** Tốc độ dịch chuyển (µs mỗi lần cập nhật) */
#define DEMO_SPEED_SLOW         10      /**< Xoay từ từ – bước 2 & 3     */
#define DEMO_SPEED_FAST         20      /**< Xoay nhanh hơn – bước 5 & 6 */

/** Chu kỳ cập nhật servo (ms) */
#define DEMO_SERVO_INTERVAL_MS  20

/** Thời gian giữ nguyên ở đầu/cuối hành trình (ms) */
#define DEMO_HOLD_MS            300

/** Thời gian chạy chế độ SOS ở bước 7 trước khi quay về bước 1 (ms) */
#define DEMO_SOS_DURATION_MS    4000

/* Exported types ------------------------------------------------------------*/

/** Các bước trong trình tự biểu diễn */
typedef enum {
    DEMO_STEP1_LIGHT_OFF = 0,   /**< Bước 1: Đèn tắt, servo về gốc       */
    DEMO_STEP2_S1_SLOW,          /**< Bước 2: Servo1 xoay chậm ra & về    */
    DEMO_STEP3_S2_SLOW,          /**< Bước 3: Servo2 xoay chậm ra & về    */
    DEMO_STEP4_LIGHT_ON,         /**< Bước 4: Đèn sáng                    */
    DEMO_STEP5_S1_FAST,          /**< Bước 5: Servo1 xoay nhanh hơn ra & về */
    DEMO_STEP6_S2_FAST,          /**< Bước 6: Servo2 xoay nhanh hơn ra & về */
    DEMO_STEP7_SOS               /**< Bước 7: Chế độ SOS                  */
} DemoStep_t;

/** Hướng chuyển động của servo trong bước hiện tại */
typedef enum {
    SERVO_DIR_FORWARD = 0,       /**< Đang xoay ra (MIN → MAX) */
    SERVO_DIR_RETURN             /**< Đang quay về  (MAX → MIN) */
} ServoDir_t;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief  Chạy một tick của trình tự biểu diễn thiết bị.
 *
 * Gọi hàm này lặp lại trong vòng while(1) của main (không có HAL_Delay
 * blocking bên trong). Hàm tự quản lý trạng thái nội bộ và tự chuyển bước.
 *
 * Trình tự 7 bước (lặp vòng liên tục):
 *   Bước 1 – Đèn tắt, servo về gốc, giữ 500 ms
 *   Bước 2 – Servo1 xoay chậm toàn hành trình rồi về gốc
 *   Bước 3 – Servo2 xoay chậm toàn hành trình rồi về gốc
 *   Bước 4 – Đèn sáng, giữ 500 ms
 *   Bước 5 – Servo1 xoay nhanh hơn toàn hành trình rồi về gốc
 *   Bước 6 – Servo2 xoay nhanh hơn toàn hành trình rồi về gốc
 *   Bước 7 – Chế độ SOS trong DEMO_SOS_DURATION_MS ms
 */
void Demo_Performance(void);

#ifdef __cplusplus
}
#endif

#endif /* DEMO_PERFORMANCE_H */

/**
 * @file    mode_manager.c
 * @brief   Phát hiện cử chỉ bật/tắt đèn để chuyển chế độ NORMAL ↔ DEMO ↔ GIMBAL.
 *
 *  Cử chỉ nhận diện (CH1 rising-edge vào ON-zone):
 *    3 lần trong GESTURE_WINDOW_MS → Toggle NORMAL ↔ DEMO
 *    5 lần trong GESTURE_WINDOW_MS → Kích hoạt GIMBAL (hoặc tắt GIMBAL về NORMAL)
 *
 *  Ưu tiên: 5 lần được kiểm tra TRƯỚC 3 lần (vì 5 > 3, khi đếm đủ 5 thì
 *  bỏ qua hành động 3 lần nếu đã xảy ra trong cùng chuỗi).
 *
 *  Sơ đồ state machine bộ đếm:
 *
 *   pulse vào ON-zone (rising edge)
 *         │
 *         ▼
 *   s_onCount++
 *         │
 *         ├── s_onCount == GESTURE_GIMBAL_COUNT (5) → → Kích hoạt GIMBAL + Reset
 *         │
 *         └── s_onCount == GESTURE_TRIGGER_COUNT (3) → → Toggle NORMAL/DEMO + Reset
 *               (chỉ kích hoạt nếu s_onCount < GESTURE_GIMBAL_COUNT)
 */

#include "mode_manager.h"

/* -------------------------------------------------------------------------- */
/* Trạng thái nội bộ (static)                                                 */
/* -------------------------------------------------------------------------- */

static AppMode  s_mode         = APP_MODE_GIMBAL;

/* Bộ đếm và thời gian cho gesture detection */
static uint8_t  s_onCount      = 0;       /* Số lần rising edge vào ON-zone  */
static uint32_t s_windowStart  = 0;       /* HAL_GetTick() của lần đầu tiên  */
static uint8_t  s_wasInOnZone  = 0;       /* Trạng thái trước: 1=trong ON-zone */

/* -------------------------------------------------------------------------- */
/* Helper: kiểm tra pulse có trong vùng ON không                              */
/* -------------------------------------------------------------------------- */
static inline uint8_t isInOnZone(uint32_t pulse)
{
    return (pulse >= LIGHT_ZONE_ON_MIN && pulse < LIGHT_ZONE_ON_MAX) ? 1U : 0U;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

void Mode_Init(void)
{
    s_mode        = APP_MODE_GIMBAL;
    s_onCount     = 0;
    s_windowStart = 0;
    s_wasInOnZone = 0;
}

void Mode_Update(uint32_t pulseCh1)
{
    uint8_t inOnZone = isInOnZone(pulseCh1);

    /* Chỉ hành động khi phát hiện RISING EDGE vào ON-zone */
    if (inOnZone && !s_wasInOnZone)
    {
        uint32_t now = HAL_GetTick();

        if (s_onCount == 0)
        {
            /* Lần đầu tiên: bắt đầu cửa sổ thời gian */
            s_windowStart = now;
            s_onCount     = 1;
        }
        else if (now - s_windowStart <= GESTURE_WINDOW_MS)
        {
            /* Vẫn trong cửa sổ → đếm thêm */
            s_onCount++;

            /* --- Ưu tiên 1: Kiểm tra 5 lần → GIMBAL --- */
            if (s_onCount >= GESTURE_GIMBAL_COUNT)
            {
                if (s_mode == APP_MODE_GIMBAL)
                {
                    s_mode = APP_MODE_NORMAL;   /* Tắt Gimbal → về Normal */
                }
                else
                {
                    s_mode = APP_MODE_GIMBAL;   /* Bật Gimbal */
                }
                s_onCount = 0;  /* Reset sẵn sàng cử chỉ tiếp */
            }
            /* --- Ưu tiên 2: Kiểm tra 3 lần → DEMO (chỉ khi < 5) ---
             *   Điều kiện: đúng bằng 3 (không vượt qua), tránh kích hoạt
             *   nhầm khi đang trên đường đếm lên 5.                    */
            else if (s_onCount == GESTURE_TRIGGER_COUNT)
            {
                if (s_mode == APP_MODE_NORMAL)
                {
                    s_mode = APP_MODE_DEMO;
                }
                else if (s_mode == APP_MODE_DEMO)
                {
                    s_mode = APP_MODE_NORMAL;
                }
                /* Nếu đang GIMBAL: bỏ qua cử chỉ 3 lần,
                   để người dùng cần bật/tắt 5 lần để thoát Gimbal */
                s_onCount = 0;
            }
        }
        else
        {
            /* Cửa sổ hết hạn → lần này là đầu chuỗi mới */
            s_windowStart = now;
            s_onCount     = 1;
        }
    }

    s_wasInOnZone = inOnZone;
}

AppMode Mode_Get(void)
{
    return s_mode;
}

void Mode_Set(AppMode mode)
{
    s_mode    = mode;
    s_onCount = 0;  /* Reset gesture counter khi set mode trực tiếp */
}

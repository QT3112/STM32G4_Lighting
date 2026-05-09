/**
 * @file    mode_manager.c
 * @brief   Phát hiện cử chỉ bật/tắt đèn nhanh để toggle chế độ NORMAL ↔ DEMO.
 *
 *  Sơ đồ state machine phát hiện cử chỉ:
 *
 *   pulse ở ngoài ON-zone          pulse vào ON-zone (rising edge)
 *         │                                  │
 *         ▼                                  ▼
 *   [IDLE / OFF-zone]  ──────────────►  Tăng onCount
 *                                           │
 *                             ┌─────────────┴─────────────┐
 *                             │ Thời gian kể từ lần đầu   │
 *                             │ ≤ GESTURE_WINDOW_MS?       │
 *                             └──────────┬────────────────┘
 *                                        │ CÓ         KHÔNG
 *                                        ▼               ▼
 *                                   onCount++      Reset, bắt đầu lại
 *                                        │
 *                             ┌──────────┴──────────┐
 *                             │ onCount ≥            │
 *                             │ GESTURE_TRIGGER_COUNT│
 *                             └──────────┬──────────┘
 *                                  CÓ    │
 *                                        ▼
 *                               Toggle AppMode + Reset
 */

#include "mode_manager.h"

/* -------------------------------------------------------------------------- */
/* Trạng thái nội bộ (static)                                                 */
/* -------------------------------------------------------------------------- */

static AppMode  s_mode         = APP_MODE_NORMAL;

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
    s_mode        = APP_MODE_DEMO;
    s_onCount     = 0;
    s_windowStart = 0;
    s_wasInOnZone = 0;
}

void Mode_Update(uint32_t pulseCh1)
{
    uint8_t inOnZone = isInOnZone(pulseCh1);

    /* Chỉ hành động khi phát hiện RISING EDGE vào ON-zone
       (pulse vừa chuyển từ ngoài vào trong vùng sáng) */
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

            if (s_onCount >= GESTURE_TRIGGER_COUNT)
            {
                /* Đủ số lần → toggle chế độ */
                s_mode    = (s_mode == APP_MODE_NORMAL) ? APP_MODE_DEMO
                                                        : APP_MODE_NORMAL;
                s_onCount = 0;  /* Reset để sẵn sàng phát hiện cử chỉ tiếp */
            }
        }
        else
        {
            /* Cửa sổ hết hạn → coi lần này là lần đầu của chuỗi mới */
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

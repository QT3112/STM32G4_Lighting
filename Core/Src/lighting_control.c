/**
 * @file    lighting_control.c
 * @brief   Điều khiển đèn 3 vùng dựa trên RC CH1.
 *
 *  Vùng 1 – pulse > LIGHT_ZONE_SOS_THRESHOLD (1750 µs) : Hiệu ứng SOS
 *  Vùng 2 – LIGHT_ZONE_ON_MIN ≤ pulse < LIGHT_ZONE_ON_MAX (1000–1249 µs) : ON
 *  Vùng 3 – Còn lại / mất tín hiệu (pulse == 0) : OFF
 */

#include "lighting_control.h"

/* -------------------------------------------------------------------------- */
/* Biến SOS nội bộ                                                            */
/* -------------------------------------------------------------------------- */

/* Mỗi phần tử là thời gian ON/OFF xen kẽ (ms).
   Bước chẵn = ON, bước lẻ = OFF. */
static const uint16_t SOS_DELAYS[] = {
    150, 150, 150, 150, 150, 450,   /* S: . . . */
    450, 150, 450, 150, 450, 450,   /* O: - - - */
    150, 150, 150, 150, 150, 1050   /* S: . . . */
};
#define SOS_STEPS  (sizeof(SOS_DELAYS) / sizeof(SOS_DELAYS[0]))

static uint8_t  s_sosStep       = 0;
static uint32_t s_lastSosMillis = 0;

/* -------------------------------------------------------------------------- */
/* Helper SOS nội bộ                                                          */
/* -------------------------------------------------------------------------- */
static void handleSOS(void)
{
    if (HAL_GetTick() - s_lastSosMillis >= SOS_DELAYS[s_sosStep])
    {
        s_lastSosMillis = HAL_GetTick();

        /* Bước chẵn → sáng, bước lẻ → tắt */
        if (s_sosStep % 2 == 0) {
            HAL_GPIO_WritePin(LIGHT_GPIO_PORT, LIGHT_GPIO_PIN, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(LIGHT_GPIO_PORT, LIGHT_GPIO_PIN, GPIO_PIN_RESET);
        }

        s_sosStep++;
        if (s_sosStep >= SOS_STEPS) {
            s_sosStep = 0;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */
void Lighting_Update(uint32_t pulseCh1)
{
    if (pulseCh1 > LIGHT_ZONE_SOS_THRESHOLD)
    {
        /* Vùng 1: SOS */
        handleSOS();
    }
    else if (pulseCh1 >= LIGHT_ZONE_ON_MIN && pulseCh1 < LIGHT_ZONE_ON_MAX)
    {
        /* Vùng 2: Đèn sáng liên tục */
        HAL_GPIO_WritePin(LIGHT_GPIO_PORT, LIGHT_GPIO_PIN, GPIO_PIN_SET);
        s_sosStep = 0;  /* Reset SOS để bắt đầu lại từ đầu khi vào SOS lần sau */
    }
    else
    {
        /* Vùng 3: Tắt (bao gồm mất tín hiệu pulse == 0) */
        HAL_GPIO_WritePin(LIGHT_GPIO_PORT, LIGHT_GPIO_PIN, GPIO_PIN_RESET);
        s_sosStep = 0;
    }
}

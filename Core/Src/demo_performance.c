/**
 ******************************************************************************
 * @file    demo_performance.c
 * @brief   Implement hàm biểu diễn thiết bị (Demo Performance).
 *
 * Hàm Demo_Performance() thực thi một chuỗi 7 bước theo state-machine
 * không blocking, lặp vòng liên tục. Hoàn toàn độc lập với tín hiệu
 * điều khiển RC PWM ngõ vào.
 *
 * Phần cứng sử dụng:
 *   - Đèn     : GPIOA – GPIO_PIN_6
 *   - Servo 1 : TIM3 – TIM_CHANNEL_2  (PWM output)
 *   - Servo 2 : TIM3 – TIM_CHANNEL_3  (PWM output)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "demo_performance.h"

/* External variables --------------------------------------------------------*/
/* Biến SOS được định nghĩa trong main.c – dùng chung để tái sử dụng handleSOS() */
extern int      sosStep;
extern uint32_t lastSosMillis;

/* Biến SOS delays được định nghĩa trong main.c */
extern const unsigned int sosDelays[];

/* Private variables ---------------------------------------------------------*/
static DemoStep_t demoStep            = DEMO_STEP1_LIGHT_OFF;
static uint32_t   demoStepStart       = 0;
static uint32_t   demoLastServoUpdate = 0;
static int32_t    demoServo1Pos       = DEMO_SERVO_MIN;
static int32_t    demoServo2Pos       = DEMO_SERVO_MIN;
static ServoDir_t demoServoDir        = SERVO_DIR_FORWARD;

/* Private function prototypes -----------------------------------------------*/
static void handleSOS_local(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Xử lý nhấp nháy SOS không blocking (dùng nội bộ cho bước 7).
 *         Logic giống handleSOS() trong main.c, dùng chung biến sosStep
 *         và lastSosMillis thông qua extern.
 */
static void handleSOS_local(void)
{
    /* sosDelays[]: chẵn = sáng, lẻ = tắt */
    if (HAL_GetTick() - lastSosMillis >= sosDelays[sosStep])
    {
        lastSosMillis = HAL_GetTick();

        if (sosStep % 2 == 0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   /* Sáng */
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); /* Tắt  */
        }

        sosStep++;
        if (sosStep >= 18)   /* 18 phần tử = 1 chu kỳ S·O·S */
        {
            sosStep = 0;
        }
    }
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Chạy một tick của trình tự biểu diễn thiết bị.
 * @note   Gọi lặp lại trong while(1) của main, không có HAL_Delay bên trong.
 */
void Demo_Performance(void)
{
    uint32_t now = HAL_GetTick();

    switch (demoStep)
    {
        /* ------------------------------------------------------------------ */
        /* BƯỚC 1: Tắt đèn – giữ 500 ms rồi sang bước 2                      */
        /* ------------------------------------------------------------------ */
        case DEMO_STEP1_LIGHT_OFF:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); /* Tắt đèn */
            /* Đảm bảo cả hai servo ở vị trí gốc */
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, DEMO_SERVO_MIN);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, DEMO_SERVO_MIN);

            if (now - demoStepStart >= 500)
            {
                demoServo1Pos       = DEMO_SERVO_MIN;
                demoServoDir        = SERVO_DIR_FORWARD;
                demoLastServoUpdate = now;
                demoStepStart       = now;
                demoStep            = DEMO_STEP2_S1_SLOW;
            }
            break;

        /* ------------------------------------------------------------------ */
        /* BƯỚC 2: Servo1 xoay chậm (SLOW) ra tận cùng rồi quay về gốc       */
        /* ------------------------------------------------------------------ */
        case DEMO_STEP2_S1_SLOW:
            if (now - demoLastServoUpdate >= DEMO_SERVO_INTERVAL_MS)
            {
                demoLastServoUpdate = now;

                if (demoServoDir == SERVO_DIR_FORWARD)
                {
                    demoServo1Pos += DEMO_SPEED_SLOW;
                    if (demoServo1Pos >= DEMO_SERVO_MAX)
                    {
                        demoServo1Pos       = DEMO_SERVO_MAX;
                        demoServoDir        = SERVO_DIR_RETURN;
                        /* Dừng ở cuối hành trình một chút */
                        demoLastServoUpdate = now + DEMO_HOLD_MS;
                    }
                }
                else /* SERVO_DIR_RETURN */
                {
                    demoServo1Pos -= DEMO_SPEED_SLOW;
                    if (demoServo1Pos <= DEMO_SERVO_MIN)
                    {
                        demoServo1Pos       = DEMO_SERVO_MIN;
                        demoServo2Pos       = DEMO_SERVO_MIN;
                        demoServoDir        = SERVO_DIR_FORWARD;
                        demoLastServoUpdate = now;
                        demoStepStart       = now;
                        demoStep            = DEMO_STEP3_S2_SLOW;
                    }
                }
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)demoServo1Pos);
            }
            break;

        /* ------------------------------------------------------------------ */
        /* BƯỚC 3: Servo2 xoay chậm (SLOW) ra tận cùng rồi quay về gốc       */
        /* ------------------------------------------------------------------ */
        case DEMO_STEP3_S2_SLOW:
            if (now - demoLastServoUpdate >= DEMO_SERVO_INTERVAL_MS)
            {
                demoLastServoUpdate = now;

                if (demoServoDir == SERVO_DIR_FORWARD)
                {
                    demoServo2Pos += DEMO_SPEED_SLOW;
                    if (demoServo2Pos >= DEMO_SERVO_MAX)
                    {
                        demoServo2Pos       = DEMO_SERVO_MAX;
                        demoServoDir        = SERVO_DIR_RETURN;
                        demoLastServoUpdate = now + DEMO_HOLD_MS;
                    }
                }
                else
                {
                    demoServo2Pos -= DEMO_SPEED_SLOW;
                    if (demoServo2Pos <= DEMO_SERVO_MIN)
                    {
                        demoServo2Pos = DEMO_SERVO_MIN;
                        demoStepStart = now;
                        demoStep      = DEMO_STEP4_LIGHT_ON;
                    }
                }
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)demoServo2Pos);
            }
            break;

        /* ------------------------------------------------------------------ */
        /* BƯỚC 4: Bật đèn – giữ 500 ms rồi sang bước 5                       */
        /* ------------------------------------------------------------------ */
        case DEMO_STEP4_LIGHT_ON:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); /* Bật đèn */

            if (now - demoStepStart >= 500)
            {
                demoServo1Pos       = DEMO_SERVO_MIN;
                demoServoDir        = SERVO_DIR_FORWARD;
                demoLastServoUpdate = now;
                demoStepStart       = now;
                demoStep            = DEMO_STEP5_S1_FAST;
            }
            break;

        /* ------------------------------------------------------------------ */
        /* BƯỚC 5: Servo1 xoay nhanh hơn (FAST) ra tận cùng rồi quay về gốc  */
        /* ------------------------------------------------------------------ */
        case DEMO_STEP5_S1_FAST:
            if (now - demoLastServoUpdate >= DEMO_SERVO_INTERVAL_MS)
            {
                demoLastServoUpdate = now;

                if (demoServoDir == SERVO_DIR_FORWARD)
                {
                    demoServo1Pos += DEMO_SPEED_FAST;
                    if (demoServo1Pos >= DEMO_SERVO_MAX)
                    {
                        demoServo1Pos       = DEMO_SERVO_MAX;
                        demoServoDir        = SERVO_DIR_RETURN;
                        demoLastServoUpdate = now + DEMO_HOLD_MS;
                    }
                }
                else
                {
                    demoServo1Pos -= DEMO_SPEED_FAST;
                    if (demoServo1Pos <= DEMO_SERVO_MIN)
                    {
                        demoServo1Pos       = DEMO_SERVO_MIN;
                        demoServo2Pos       = DEMO_SERVO_MIN;
                        demoServoDir        = SERVO_DIR_FORWARD;
                        demoLastServoUpdate = now;
                        demoStepStart       = now;
                        demoStep            = DEMO_STEP6_S2_FAST;
                    }
                }
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)demoServo1Pos);
            }
            break;

        /* ------------------------------------------------------------------ */
        /* BƯỚC 6: Servo2 xoay nhanh hơn (FAST) ra tận cùng rồi quay về gốc  */
        /* ------------------------------------------------------------------ */
        case DEMO_STEP6_S2_FAST:
            if (now - demoLastServoUpdate >= DEMO_SERVO_INTERVAL_MS)
            {
                demoLastServoUpdate = now;

                if (demoServoDir == SERVO_DIR_FORWARD)
                {
                    demoServo2Pos += DEMO_SPEED_FAST;
                    if (demoServo2Pos >= DEMO_SERVO_MAX)
                    {
                        demoServo2Pos       = DEMO_SERVO_MAX;
                        demoServoDir        = SERVO_DIR_RETURN;
                        demoLastServoUpdate = now + DEMO_HOLD_MS;
                    }
                }
                else
                {
                    demoServo2Pos -= DEMO_SPEED_FAST;
                    if (demoServo2Pos <= DEMO_SERVO_MIN)
                    {
                        demoServo2Pos = DEMO_SERVO_MIN;
                        /* Bắt đầu bước 7: SOS */
                        sosStep       = 0;
                        lastSosMillis = now;
                        demoStepStart = now;
                        demoStep      = DEMO_STEP7_SOS;
                    }
                }
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)demoServo2Pos);
            }
            break;

        /* ------------------------------------------------------------------ */
        /* BƯỚC 7: Chế độ SOS – chạy DEMO_SOS_DURATION_MS ms rồi về bước 1   */
        /* ------------------------------------------------------------------ */
        case DEMO_STEP7_SOS:
            handleSOS_local(); /* Nhấp nháy SOS không blocking */

            if (now - demoStepStart >= DEMO_SOS_DURATION_MS)
            {
                /* Kết thúc SOS: tắt đèn, quay về bước 1 */
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
                sosStep       = 0;
                demoStep      = DEMO_STEP1_LIGHT_OFF;
                demoStepStart = now;
            }
            break;

        default:
            demoStep      = DEMO_STEP1_LIGHT_OFF;
            demoStepStart = now;
            break;
    }
}

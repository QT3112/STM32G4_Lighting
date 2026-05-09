/**
 ******************************************************************************
 * @file    demo_performance.c
 * @brief   Implement hàm biểu diễn thiết bị (Demo Performance).
 ******************************************************************************
 */

#include "demo_performance.h"

static int sosStep = 0;
static uint32_t lastSosMillis = 0;
static const unsigned int sosDelays[] = {
    150, 150, 150, 150, 150, 450, /* S */
    450, 150, 450, 150, 450, 450, /* O */
    150, 150, 150, 150, 150, 1050 /* S */
};

static DemoStep_t demoStep = DEMO_STEP1_LIGHT_OFF;
static uint32_t demoStepStart = 0;
static uint32_t demoLastServoUpdate = 0;
static int32_t demoServo1Pos = DEMO_SERVO_MID;
static int32_t demoServo2Pos = DEMO_SERVO_MID;
static ServoDir_t demoServoDir = SERVO_DIR_FORWARD;
static int demoStep8Phase = 0;

static void handleSOS_local(void) {
  if (HAL_GetTick() - lastSosMillis >= sosDelays[sosStep]) {
    lastSosMillis = HAL_GetTick();

    if (sosStep % 2 == 0) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    }

    sosStep++;
    if (sosStep >= 18) {
      sosStep = 0;
    }
  }
}

void Demo_Performance(void) {
  uint32_t now = HAL_GetTick();

  switch (demoStep) {
  case DEMO_STEP1_LIGHT_OFF:
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, DEMO_SERVO_MID);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, DEMO_SERVO_MID);

    if (now - demoStepStart >= 500) {
      demoServo1Pos = DEMO_SERVO_MID;
      demoServoDir = SERVO_DIR_FORWARD;
      demoLastServoUpdate = now;
      demoStepStart = now;
      demoStep = DEMO_STEP2_S1_SLOW;
    }
    break;

  case DEMO_STEP2_S1_SLOW:
    if (now - demoLastServoUpdate >= DEMO_SERVO_INTERVAL_MS) {
      demoLastServoUpdate = now;

      if (demoServoDir == SERVO_DIR_FORWARD) {
        demoServo1Pos += DEMO_SPEED_SLOW;
        if (demoServo1Pos >= DEMO_SERVO_MAX) {
          demoServo1Pos = DEMO_SERVO_MAX;
          demoServoDir = SERVO_DIR_RETURN;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else if (demoServoDir == SERVO_DIR_RETURN) {
        demoServo1Pos -= DEMO_SPEED_SLOW;
        if (demoServo1Pos <= DEMO_SERVO_MIN) {
          demoServo1Pos = DEMO_SERVO_MIN;
          demoServoDir = SERVO_DIR_TO_MID;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else /* SERVO_DIR_TO_MID */
      {
        demoServo1Pos += DEMO_SPEED_SLOW;
        if (demoServo1Pos >= DEMO_SERVO_MID) {
          demoServo1Pos = DEMO_SERVO_MID;
          demoServo2Pos = DEMO_SERVO_MID;
          demoServoDir = SERVO_DIR_FORWARD;
          demoLastServoUpdate = now;
          demoStepStart = now;
          demoStep = DEMO_STEP3_S2_SLOW;
        }
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)demoServo1Pos);
    }
    break;

  case DEMO_STEP3_S2_SLOW:
    if (now - demoLastServoUpdate >= DEMO_SERVO_INTERVAL_MS) {
      demoLastServoUpdate = now;

      if (demoServoDir == SERVO_DIR_FORWARD) {
        demoServo2Pos += DEMO_SPEED_SLOW;
        if (demoServo2Pos >= DEMO_SERVO_MAX) {
          demoServo2Pos = DEMO_SERVO_MAX;
          demoServoDir = SERVO_DIR_RETURN;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else if (demoServoDir == SERVO_DIR_RETURN) {
        demoServo2Pos -= DEMO_SPEED_SLOW;
        if (demoServo2Pos <= DEMO_SERVO_MIN) {
          demoServo2Pos = DEMO_SERVO_MIN;
          demoServoDir = SERVO_DIR_TO_MID;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else /* SERVO_DIR_TO_MID */
      {
        demoServo2Pos += DEMO_SPEED_SLOW;
        if (demoServo2Pos >= DEMO_SERVO_MID) {
          demoServo2Pos = DEMO_SERVO_MID;
          demoStepStart = now;
          demoStep = DEMO_STEP4_LIGHT_ON;
        }
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)demoServo2Pos);
    }
    break;

  case DEMO_STEP4_LIGHT_ON:
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    if (now - demoStepStart >= 500) {
      demoServo1Pos = DEMO_SERVO_MID;
      demoServoDir = SERVO_DIR_FORWARD;
      demoLastServoUpdate = now;
      demoStepStart = now;
      demoStep = DEMO_STEP5_S1_FAST;
    }
    break;

  case DEMO_STEP5_S1_FAST:
    if (now - demoLastServoUpdate >= DEMO_SERVO_INTERVAL_MS) {
      demoLastServoUpdate = now;

      if (demoServoDir == SERVO_DIR_FORWARD) {
        demoServo1Pos += DEMO_SPEED_FAST;
        if (demoServo1Pos >= DEMO_SERVO_MAX) {
          demoServo1Pos = DEMO_SERVO_MAX;
          demoServoDir = SERVO_DIR_RETURN;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else if (demoServoDir == SERVO_DIR_RETURN) {
        demoServo1Pos -= DEMO_SPEED_FAST;
        if (demoServo1Pos <= DEMO_SERVO_MIN) {
          demoServo1Pos = DEMO_SERVO_MIN;
          demoServoDir = SERVO_DIR_TO_MID;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else /* SERVO_DIR_TO_MID */
      {
        demoServo1Pos += DEMO_SPEED_FAST;
        if (demoServo1Pos >= DEMO_SERVO_MID) {
          demoServo1Pos = DEMO_SERVO_MID;
          demoServo2Pos = DEMO_SERVO_MID;
          demoServoDir = SERVO_DIR_FORWARD;
          demoLastServoUpdate = now;
          demoStepStart = now;
          demoStep = DEMO_STEP6_S2_FAST;
        }
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)demoServo1Pos);
    }
    break;

  case DEMO_STEP6_S2_FAST:
    if (now - demoLastServoUpdate >= DEMO_SERVO_INTERVAL_MS) {
      demoLastServoUpdate = now;

      if (demoServoDir == SERVO_DIR_FORWARD) {
        demoServo2Pos += DEMO_SPEED_FAST;
        if (demoServo2Pos >= DEMO_SERVO_MAX) {
          demoServo2Pos = DEMO_SERVO_MAX;
          demoServoDir = SERVO_DIR_RETURN;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else if (demoServoDir == SERVO_DIR_RETURN) {
        demoServo2Pos -= DEMO_SPEED_FAST;
        if (demoServo2Pos <= DEMO_SERVO_MIN) {
          demoServo2Pos = DEMO_SERVO_MIN;
          demoServoDir = SERVO_DIR_TO_MID;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else /* SERVO_DIR_TO_MID */
      {
        demoServo2Pos += DEMO_SPEED_FAST;
        if (demoServo2Pos >= DEMO_SERVO_MID) {
          demoServo2Pos = DEMO_SERVO_MID;
          sosStep = 0;
          lastSosMillis = now;
          demoStepStart = now;
          demoStep = DEMO_STEP7_SOS;
        }
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)demoServo2Pos);
    }
    break;

  case DEMO_STEP7_SOS:
    // handleSOS_local();
    demoStep = DEMO_STEP8_BOTH_SERVOS;

    // if (now - demoStepStart >= DEMO_SOS_DURATION_MS) {
    //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    //   sosStep = 0;
    //   demoStep8Phase = 0;
    //   demoServo1Pos = DEMO_SERVO_MID;
    //   demoServo2Pos = DEMO_SERVO_MID;
    //   demoLastServoUpdate = now;
    //   demoStep = DEMO_STEP8_BOTH_SERVOS;
    //   demoStepStart = now;
    // }
    break;

  case DEMO_STEP8_BOTH_SERVOS:
    if (now - demoLastServoUpdate >= DEMO_SERVO_INTERVAL_MS) {
      demoLastServoUpdate = now;

      if (demoStep8Phase == 0) /* Phase 0: S1 MID->MAX, S2 MID->MIN */
      {
        demoServo1Pos += DEMO_SPEED_FAST;
        demoServo2Pos -= DEMO_SPEED_FAST;
        if (demoServo1Pos >= DEMO_SERVO_MAX) {
          demoServo1Pos = DEMO_SERVO_MAX;
          demoServo2Pos = DEMO_SERVO_MIN;
          demoStep8Phase = 1;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else if (demoStep8Phase == 1) /* Phase 1: S1 MAX->MIN, S2 MIN->MAX */
      {
        demoServo1Pos -= DEMO_SPEED_FAST;
        demoServo2Pos += DEMO_SPEED_FAST;
        if (demoServo1Pos <= DEMO_SERVO_MIN) {
          demoServo1Pos = DEMO_SERVO_MIN;
          demoServo2Pos = DEMO_SERVO_MAX;
          demoStep8Phase = 2;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else if (demoStep8Phase == 2) /* Phase 2: S1 MIN->MAX, S2 MAX->MIN */
      {
        demoServo1Pos += DEMO_SPEED_FAST;
        demoServo2Pos -= DEMO_SPEED_FAST;
        if (demoServo1Pos >= DEMO_SERVO_MAX) {
          demoServo1Pos = DEMO_SERVO_MAX;
          demoServo2Pos = DEMO_SERVO_MIN;
          demoStep8Phase = 3;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else if (demoStep8Phase == 3) /* Phase 3: S1 MAX->MID, S2 MIN->MID */
      {
        demoServo1Pos -= DEMO_SPEED_FAST;
        demoServo2Pos += DEMO_SPEED_FAST;
        if (demoServo1Pos <= DEMO_SERVO_MID) {
          demoServo1Pos = DEMO_SERVO_MID;
          demoServo2Pos = DEMO_SERVO_MID;
          demoStep8Phase = 4;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else if (demoStep8Phase == 4) /* Phase 4: Both MID->MAX */
      {
        demoServo1Pos += DEMO_SPEED_FAST;
        demoServo2Pos += DEMO_SPEED_FAST;
        if (demoServo1Pos >= DEMO_SERVO_MAX) {
          demoServo1Pos = DEMO_SERVO_MAX;
          demoServo2Pos = DEMO_SERVO_MAX;
          demoStep8Phase = 5;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else if (demoStep8Phase == 5) /* Phase 5: Both MAX->MIN */
      {
        demoServo1Pos -= DEMO_SPEED_FAST;
        demoServo2Pos -= DEMO_SPEED_FAST;
        if (demoServo1Pos <= DEMO_SERVO_MIN) {
          demoServo1Pos = DEMO_SERVO_MIN;
          demoServo2Pos = DEMO_SERVO_MIN;
          demoStep8Phase = 6;
          demoLastServoUpdate = now + DEMO_HOLD_MS;
        }
      } else if (demoStep8Phase == 6) /* Phase 6: Both MIN->MID */
      {
        demoServo1Pos += DEMO_SPEED_FAST;
        demoServo2Pos += DEMO_SPEED_FAST;
        if (demoServo1Pos >= DEMO_SERVO_MID) {
          demoServo1Pos = DEMO_SERVO_MID;
          demoServo2Pos = DEMO_SERVO_MID;

          demoStep8Phase = 0;
          demoStepStart = now;
          demoStep = DEMO_STEP1_LIGHT_OFF;
        }
      }

      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)demoServo1Pos);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)demoServo2Pos);
    }
    break;

  default:
    demoStep = DEMO_STEP1_LIGHT_OFF;
    demoStepStart = now;
    break;
  }
}

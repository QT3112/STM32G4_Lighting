/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "gpio.h"
#include "stm32g4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "crsf.h" /* Chế độ nhận tín hiệu CRSF qua UART3 */
#include "demo_performance.h"
#include "lighting_control.h"
#include "mode_manager.h"
#include "rc_input.h"
#include "servo_control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Tất cả biến shared đã được chuyển vào rc_input.c (static nội bộ).       */
/* Truy xuất qua RC_Input_GetCh1/2/3().                                      */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Demo_Performance() được khai báo trong demo_performance.h */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief HAL callback cho TIM2 Input Capture – ủy quyền sang rc_input.c.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  RC_Input_CaptureCallback(htim);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_Device_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 1. Tắt đèn mặc định */
  HAL_GPIO_WritePin(LIGHT_GPIO_PORT, LIGHT_GPIO_PIN, GPIO_PIN_RESET);

  /* 2. Khởi tạo servo: bật PWM và về 90° */
  Servo_Init();

  /* 3. Cấp xung 1 giây để servo ổn định về vị trí gốc */
  HAL_Delay(1000);

  /* 4. Khởi tạo RC Input (ghi lastPulseTime = now sau delay, tránh timeout sớm)
   */
  RC_Input_Init();

  /* 5. Khởi tạo CRSF Input (DMA circular Rx trên USART3 @ 420000 baud)
        Nếu bộ thu CRSF không được cắm, module vẫn hoạt động bình thường,
        CRSF_Input_IsConnected() chỉ trả về 0 → chương trình tự dùng PWM. */
  CRSF_Input_Init();

  /* 6. Khởi tạo bộ quản lý chế độ (mặc định: NORMAL) */
  Mode_Init();

  /* 7. Khởi động Input Capture ngắt (chế độ PWM – luôn hoạt động) */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* 1. Cập nhật timeout PWM và parse frame CRSF mới từ DMA buffer */
    // CRSF_test_printf();
    RC_Input_Update();
    CRSF_Input_Update();

    /* 2. Chọn nguồn tín hiệu điều khiển:
     *      - Ưu tiên CRSF nếu bộ thu CRSF đang kết nối (cắm cụm dây CRSF)
     *      - Nếu không → dùng PWM thông thường (cắm cụm dây PWM)
     *
     * Ánh xạ kênh (Channel Mapping):
     *   CH1 → Servo 1
     *   CH2 → Servo 2
     *   CH6 → Đèn / Lighting (công tắc trên tay cầm) */
    uint32_t ch_servo1, ch_servo2, ch_light;

    if (CRSF_Input_IsConnected()) {
      /* CHẾ ĐỘ CRSF: đọc kênh từ bộ thu CRSF qua UART3 */
      ch_servo1 = CRSF_Input_GetCh1(); /* CH1 → Servo 1 */
      ch_servo2 = CRSF_Input_GetCh2(); /* CH2 → Servo 2 */
      ch_light = CRSF_Input_GetCh6();  /* CH6 → Đèn     */
    } else {
      /* CHẾ ĐỘ PWM: đọc kênh từ TIM2 Input Capture (RC Receiver truyền thống)
       * PWM chỉ có 3 kênh vật lý (PA0/PA1/PA2):
       *   TIM2 CH1 (PA0) → Servo 1
       *   TIM2 CH2 (PA1) → Servo 2
       *   TIM2 CH3 (PA2) → Đèn     */
      ch_servo1 = RC_Input_GetCh1(); /* PA0 → Servo 1 */
      ch_servo2 = RC_Input_GetCh2(); /* PA1 → Servo 2 */
      ch_light = RC_Input_GetCh3();  /* PA2 → Đèn     */
    }

    /* 3. Cập nhật bộ phát hiện cử chỉ toggle chế độ NORMAL/DEMO
          (luôn chạy dù đang dùng nguồn tín hiệu nào) */
    Mode_Update(ch_light);

    /* 4. Điều phối theo chế độ hoạt động */
    if (Mode_Get() == APP_MODE_DEMO) {
      /* CHẾ ĐỘ DEMO: đèn do Demo_Performance() điều khiển tự động */
      Demo_Performance();
    } else {
      /* CHẾ ĐỘ NORMAL: servo + đèn theo tín hiệu điều khiển (PWM hoặc CRSF) */
      Servo_Update(ch_servo1, ch_servo2);
      Lighting_Update(ch_light);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

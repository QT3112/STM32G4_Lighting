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
#include "i2c.h"
#include "stm32g4xx_hal.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "demo_performance.h"
#include "rc_input.h"
#include "servo_control.h"
#include "lighting_control.h"
#include "mode_manager.h"
#include "mpu6050.h"
#include "gimbal_control.h"
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
/* Handle MPU6050 và Gimbal Controller (khai báo tại đây để chia sẻ với callback) */
MPU6050_Handle_t    hMpu;
GimbalControl_Handle_t hGimbal;
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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  RC_Input_CaptureCallback(htim);
}

/**
 * @brief HAL callback cho EXTI (PA15) – MPU6050 Data-Ready Interrupt.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MPU6050_INT_PIN)
  {
    MPU6050_DRDY_Callback(&hMpu);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_Device_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  /* 1. Tắt đèn mặc định */
  HAL_GPIO_WritePin(LIGHT_GPIO_PORT, LIGHT_GPIO_PIN, GPIO_PIN_RESET);

  /* 2. Khởi tạo servo: bật PWM và về 90° */
  Servo_Init();

  /* 3. Cấp xung 1 giây để servo ổn định về vị trí gốc */
  HAL_Delay(1000);

  /* 4. Khởi tạo RC Input */
  RC_Input_Init();

  /* 5. Khởi tạo bộ quản lý chế độ (mặc định: NORMAL) */
  Mode_Init();

  /* 6. Khởi tạo MPU6050 (I2C3, DRDY trên PA15) */
  uint8_t check_connect_MPU;
  check_connect_MPU = MPU6050_Init(&hMpu,
                    MPU6050_GYRO_FS_500DPS,
                    MPU6050_ACCEL_FS_4G,
                    MPU6050_DLPF_BW_044HZ);
  if (check_connect_MPU != HAL_OK)
  {
    /* Không kết nối được MPU6050 – kiểm tra dây I2C3 */
    printf("MPU6050 Init: FAIL! Check I2C3 wiring (PA8, PC11).\r\n");
    /* Khong goi Error_Handler() vi no se tat ngat (__disable_irq), lam chet USB CDC */
  }
  else
  {
    printf("MPU6050 Init: OK!\r\n");
  }

  /* 7. Khởi tạo Gimbal Controller (tham số PID mặc định) */
  Gimbal_Init(&hGimbal);

  /* 8. Khởi động Input Capture ngắt */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1. Cập nhật timeout, reset pulse nếu mất tín hiệu */
    // printf("check MPU: %d \n", check_connect_MPU);
    // HAL_Delay(1000);
    
    RC_Input_Update();

    /* 2. Đọc giá trị xung an toàn từ ISR */
    uint32_t ch1 = RC_Input_GetCh1();
    uint32_t ch2 = RC_Input_GetCh2();
    uint32_t ch3 = RC_Input_GetCh3();

    /* 3. Cập nhật bộ phát hiện cử chỉ toggle chế độ
          (luôn chạy dù đang ở chế độ nào) */
    Mode_Update(ch1);

    /* 4. Điều phối theo chế độ hiện tại */
    if (Mode_Get() == APP_MODE_DEMO)
    {
      /* CHẾ ĐỘ DEMO: đèn do Demo_Performance() điều khiển */
      Demo_Performance();
    }
    else if (Mode_Get() == APP_MODE_GIMBAL)
    {
      /* CHẾ ĐỘ GIMBAL: đọc IMU + chạy Cascaded PID ổn định 2 trục */
      if (MPU6050_IsDataReady(&hMpu))
      {
        MPU6050_Update(&hMpu);
        Gimbal_Update(&hGimbal, &hMpu);
      }
    }
    else
    {
      /* CHẾ ĐỘ NORMAL: servo + đèn theo tín hiệu RC */
      Servo_Update(ch2, ch3);
      Lighting_Update(ch1);
    }

    /* 5. In log định kỳ mỗi 500ms để kiểm tra */
    static uint32_t last_print_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_print_tick >= 500)
    {
      last_print_tick = now;
      AppMode mode = Mode_Get();
      if (mode == APP_MODE_NORMAL)
      {
        printf("[MODE] NORMAL - RC CH1:%lu CH2:%lu CH3:%lu (MPU status: %d)\r\n", ch1, ch2, ch3, check_connect_MPU);
      }
      else if (mode == APP_MODE_DEMO)
      {
        printf("[MODE] DEMO - Playing sequence...\r\n");
      }
      else if (mode == APP_MODE_GIMBAL)
      {
        printf("[MODE] GIMBAL - Pitch(Kal): %.2f | Roll(Comp): %.2f | Servo P:%lu Y:%lu\r\n",
               hGimbal.kalman_pitch.angle, hMpu.angle.roll,
               hGimbal.servo_pitch_us, hGimbal.servo_yaw_us);
      }
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

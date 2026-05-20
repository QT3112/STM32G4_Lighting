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
#include "imu_dual.h"         /* Driver dual MPU6050 (0x68 + 0x69) */
#include "fusion.h"           /* Complementary Filter               */
#include "gimbal_control.h"   /* Gimbal controller (rewritten)      */
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

/* Dual IMU handle: Frame(0x68) + Camera(0x69) */
ImuDual_Handle_t     hImuDual;

/* Gimbal controller handle */
GimbalControl_Handle_t hGimbal;

/* Extern flag set bởi TIM6 ISR @ 500Hz (khai báo trong stm32g4xx_it.c) */
extern volatile uint8_t g_gimbal_tick_flag;

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

  /* 2. Khởi tạo servo: bật PWM và về 90° (center = 1500µs) */
  Servo_Init();

  /* 3. Cấp xung 1 giây để servo ổn định */
  HAL_Delay(1000);

  /* 4. Khởi tạo RC Input */
  RC_Input_Init();

  /* 5. Khởi tạo bộ quản lý chế độ (mặc định: GIMBAL) */
  Mode_Init();

  /* 6. Khởi tạo Dual IMU (Frame @ 0x68 + Camera @ 0x69 trên I2C3) */
  if (ImuDual_Init(&hImuDual) != HAL_OK)
  {
    printf("[MAIN] Dual IMU init FAIL — check I2C3 wiring & AD0 pins.\r\n");
    /* Không gọi Error_Handler() để USB CDC vẫn hoạt động cho debug */
  }
  else
  {
    /* 7. Calibrate gyro bias (~1 giây, gimbal phải bất động) */
    ImuDual_Calibrate(&hImuDual);

    /* 8. Khởi tạo Gimbal Controller */
    Gimbal_Init(&hGimbal, &hImuDual);
  }

  /* 9. Khởi động Input Capture interrupt (RC) */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

  /* 10. Khởi động TIM6 @ 500Hz — kích hoạt g_gimbal_tick_flag trong ISR */
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1. Cập nhật timeout, reset pulse nếu mất tín hiệu */
    RC_Input_Update();

    /* 2. Đọc giá trị xung an toàn từ ISR */
    uint32_t ch1 = RC_Input_GetCh1();
    uint32_t ch2 = RC_Input_GetCh2();
    uint32_t ch3 = RC_Input_GetCh3();

    /* 3. Cập nhật bộ phát hiện cử chỉ toggle chế độ */
    Mode_Update(ch1);

    /* 4. Điều phối theo chế độ hiện tại */
    if (Mode_Get() == APP_MODE_DEMO)
    {
      /* CHẾ ĐỘ DEMO */
      Demo_Performance();
    }
    else if (Mode_Get() == APP_MODE_GIMBAL)
    {
      /* CHẾ ĐỘ GIMBAL — 500Hz flag-based control loop
       *
       * TIM6 ISR set g_gimbal_tick_flag = 1 mỗi 2ms (500Hz).
       * Main loop kiểm tra flag, xóa và gọi Gimbal_Tick().
       *
       * Tại sao không gọi trong ISR?
       *   Gimbal_Tick() thực hiện HAL_I2C_Mem_Read() (blocking ~400µs).
       *   Nếu I2C3 EV interrupt có priority thấp hơn TIM6, I2C sẽ timeout
       *   vì EV không thể preempt TIM6 ISR → deadlock.
       *   Giải pháp: ISR chỉ set flag (< 1µs), main loop làm việc nặng.
       */
      if (g_gimbal_tick_flag)
      {
        g_gimbal_tick_flag = 0;   /* Clear flag trước khi xử lý */
        Gimbal_Tick(&hGimbal, &hImuDual);
      }

      /* Telemetry & CLI (200ms interval, không blocking) */
      Gimbal_Update(&hGimbal);
    }
    else
    {
      /* CHẾ ĐỘ NORMAL: servo + đèn theo tín hiệu RC */
      Servo_Update(ch2, ch3);
      Lighting_Update(ch1);
    }

    /* 5. Mode status log mỗi 500ms */
    static uint32_t last_print_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_print_tick >= 500)
    {
      last_print_tick = now;
      AppMode mode = Mode_Get();
      if (mode == APP_MODE_NORMAL)
      {
        printf("[MODE] NORMAL - RC CH1:%lu CH2:%lu CH3:%lu\r\n", ch1, ch2, ch3);
      }
      else if (mode == APP_MODE_DEMO)
      {
        printf("[MODE] DEMO\r\n");
      }
      else if (mode == APP_MODE_GIMBAL)
      {
        /* In trạng thái giao tiếp và dữ liệu thô để kiểm tra MPU6050 */
        printf("[DEBUG IMU] Frame(0x68) Init:%d Read:%d | Cam(0x69) Init:%d Read:%d\r\n", 
               hImuDual.frame.initialized, hImuDual.frame.read_ok,
               hImuDual.camera.initialized, hImuDual.camera.read_ok);
               
        printf("[DEBUG IMU] Frame Accel: X=%.2f Y=%.2f Z=%.2f | Cam Accel: X=%.2f Y=%.2f Z=%.2f\r\n",
               hImuDual.frame.scaled.accel_x, hImuDual.frame.scaled.accel_y, hImuDual.frame.scaled.accel_z,
               hImuDual.camera.scaled.accel_x, hImuDual.camera.scaled.accel_y, hImuDual.camera.scaled.accel_z);
      }
      /* GIMBAL mode: telemetry đã được in bởi Gimbal_Update() @ 200ms */
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

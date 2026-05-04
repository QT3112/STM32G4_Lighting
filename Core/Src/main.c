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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "demo_performance.h"
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
volatile uint32_t capture1 = 0;
volatile uint32_t sharedPulseWidth = 0;
volatile uint8_t isFirstCaptured = 0;
volatile uint32_t lastPulseTime = 0;  // Thời điểm nhận được xung PWM cuối cùng

volatile uint32_t capture1_ch2 = 0;
volatile uint32_t sharedPulseWidth_ch2 = 0;
volatile uint8_t isFirstCaptured_ch2 = 0;
volatile uint32_t lastPulseTime_ch2 = 0;

volatile uint32_t capture1_ch3 = 0;
volatile uint32_t sharedPulseWidth_ch3 = 0;
volatile uint8_t isFirstCaptured_ch3 = 0;
volatile uint32_t lastPulseTime_ch3 = 0;

// Mảng thời gian SOS [cite: 4]
const unsigned int sosDelays[] = {
  150, 150, 150, 150, 150, 450,  // S
  450, 150, 450, 150, 450, 450,  // O
  150, 150, 150, 150, 150, 1050  // S
};
int sosStep = 0;
uint32_t lastSosMillis = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Demo_Performance() được khai báo trong demo_performance.h */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Hàm ngắt này tự động được gọi khi chân TIM2_CH1 bắt được cạnh xung
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if (isFirstCaptured == 0) // Bắt được cạnh LÊN
    {
      capture1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      isFirstCaptured = 1;
      // Đảo cực để chờ bắt cạnh XUỐNG
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else // Bắt được cạnh XUỐNG
    {
      uint32_t capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      uint32_t pulse;
      
      if (capture2 > capture1) {
        pulse = capture2 - capture1;
      } else {
        pulse = (0xFFFFFFFF - capture1) + capture2 + 1; // Xử lý tràn (overflow)
      }
      
      // Chỉ chấp nhận xung nằm trong dải hợp lệ RC PWM (800–2200us), loại bỏ nhiễu
      if (pulse >= 800 && pulse <= 2200) {
        sharedPulseWidth = pulse;
        lastPulseTime = HAL_GetTick();
      }
      
      isFirstCaptured = 0;
      // Đảo cực lại để chờ cạnh LÊN của chu kỳ tiếp theo
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    if (isFirstCaptured_ch2 == 0) // Bắt được cạnh LÊN
    {
      capture1_ch2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
      isFirstCaptured_ch2 = 1;
      // Đảo cực để chờ bắt cạnh XUỐNG
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else // Bắt được cạnh XUỐNG
    {
      uint32_t capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
      uint32_t pulse_ch2;

      if (capture2 > capture1_ch2) {
        pulse_ch2 = capture2 - capture1_ch2;
      } else {
        pulse_ch2 = (0xFFFFFFFF - capture1_ch2) + capture2 + 1; // Xử lý tràn (overflow)
      }

      // Chỉ chấp nhận xung nằm trong dải hợp lệ RC PWM (800–2200us), loại bỏ nhiễu
      if (pulse_ch2 >= 800 && pulse_ch2 <= 2200) {
        sharedPulseWidth_ch2 = pulse_ch2;
        lastPulseTime_ch2 = HAL_GetTick();
      }

      isFirstCaptured_ch2 = 0;
      // Đảo cực lại để chờ cạnh LÊN của chu kỳ tiếp theo
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    if (isFirstCaptured_ch3 == 0) // Bắt được cạnh LÊN
    {
      capture1_ch3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
      isFirstCaptured_ch3 = 1;
      // Đảo cực để chờ bắt cạnh XUỐNG
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else // Bắt được cạnh XUỐNG
    {
      uint32_t capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
      uint32_t pulse_ch3;

      if (capture2 > capture1_ch3) {
        pulse_ch3 = capture2 - capture1_ch3;
      } else {
        pulse_ch3 = (0xFFFFFFFF - capture1_ch3) + capture2 + 1; // Xử lý tràn (overflow)
      }

      // Chỉ chấp nhận xung nằm trong dải hợp lệ RC PWM (800–2200us), loại bỏ nhiễu
      if (pulse_ch3 >= 800 && pulse_ch3 <= 2200) {
        sharedPulseWidth_ch3 = pulse_ch3;
        lastPulseTime_ch3 = HAL_GetTick();
      }

      isFirstCaptured_ch3 = 0;
      // Đảo cực lại để chờ cạnh LÊN của chu kỳ tiếp theo
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
}

// Hàm xử lý SOS không nghẽn [cite: 16, 17, 18, 19]
void handleSOS() {
  if (HAL_GetTick() - lastSosMillis >= sosDelays[sosStep]) {
    lastSosMillis = HAL_GetTick();
    
    // Bước chẵn là sáng (ON), lẻ là tắt (OFF)
    if (sosStep % 2 == 0) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // sáng hoàn toàn
    } else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // tắt hoàn toàn
    }

    sosStep++;
    if (sosStep >= 18) { // Lặp lại chu kỳ [cite: 19, 20]
      sosStep = 0;
    }
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
  /* USER CODE BEGIN 2 */
  // Khởi động Timer xuất PWM cho servo
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // Tắt đèn mặc định và đưa servo về 0 độ (tương ứng xung 500us)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 500);

  // Cấp xung trong 1 giây để servo có đủ thời gian quay về gốc
  HAL_Delay(1000);

  // Khởi tạo lastPulseTime SAU delay để tránh timeout kích hoạt ngay lập tức
  // (nếu để = 0, điều kiện HAL_GetTick() - 0 > 50 đúng ngay, gây reset sai)
  uint32_t now = HAL_GetTick();
  lastPulseTime     = now;
  lastPulseTime_ch2 = now;
  lastPulseTime_ch3 = now;

  // Sau đó mới khởi động Timer ngắt Input Capture
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // ==============================================================
    // CHẾ ĐỘ DEMO – Gọi hàm biểu diễn, không phụ thuộc tín hiệu ngõ vào.
    // Khi muốn quay lại chế độ điều khiển bình thường, comment dòng
    // này và bỏ comment khối điều khiển phía dưới.
    Demo_Performance();

    /* ------ KHỐI ĐIỀU KHIỂN BÌNH THƯỜNG (tạm comment để test demo) ------

    // Kiểm tra timeout: nếu không có xung trong 50ms thì reset về 0
    if (HAL_GetTick() - lastPulseTime > 50) {
      __disable_irq();
      sharedPulseWidth = 0;
      __enable_irq();
    }
    if (HAL_GetTick() - lastPulseTime_ch2 > 50) {
      __disable_irq();
      sharedPulseWidth_ch2 = 0;
      __enable_irq();
    }
    if (HAL_GetTick() - lastPulseTime_ch3 > 50) {
      __disable_irq();
      sharedPulseWidth_ch3 = 0;
      __enable_irq();
    }

    // Truy xuất an toàn giá trị xung [cite: 10]
    __disable_irq();
    uint32_t currentPulse     = sharedPulseWidth;
    uint32_t currentPulse_ch2 = sharedPulseWidth_ch2;
    uint32_t currentPulse_ch3 = sharedPulseWidth_ch3;
    __enable_irq();

    // Xuất xung ra PWM cho servo CH2 (ánh xạ 1000-2000us sang 500-2500us)
    if (currentPulse_ch2 > 0) {
      int32_t outPWM2 = ((int32_t)currentPulse_ch2 - 1000) * 2 + 500;
      if (outPWM2 < 500)  outPWM2 = 500;
      if (outPWM2 > 2500) outPWM2 = 2500;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)outPWM2);
    } else {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }

    // Xuất xung ra PWM cho servo CH3 (ánh xạ 1000-2000us sang 500-2500us)
    if (currentPulse_ch3 > 0) {
      int32_t outPWM3 = ((int32_t)currentPulse_ch3 - 1000) * 2 + 500;
      if (outPWM3 < 500)  outPWM3 = 500;
      if (outPWM3 > 2500) outPWM3 = 2500;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)outPWM3);
    } else {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    }

    // 1. Mức cao nhất (>1750us): Chế độ SOS [cite: 11]
    if (currentPulse > 1750) {
      handleSOS();
    }
    // 2. Mức thấp nhất (1000us – 1250us): Bật sáng (ON)
    else if (currentPulse >= 1000 && currentPulse < 1250) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
      sosStep = 0;
    }
    // 3. Mức giữa hoặc mất tín hiệu: Tắt (OFF) [cite: 14]
    else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
      sosStep = 0;
    }
    ---------------------------------------------------------------------- */

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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Dual Mode Control (PWM / CRSF)
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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "crsf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * ==============================================================================
 * LỰA CHỌN CHẾ ĐỘ ĐIỀU KHIỂN
 * ==============================================================================
 * Thay đổi giá trị CONTROL_MODE để chọn chế độ hoạt động:
 *
 *   #define CONTROL_MODE  PWM_MODE
 *     → Chế độ 1: Đọc tín hiệu PWM đầu vào từ Timer2 (TIM2_CH1/2/3)
 *       Phù hợp khi nhận xung PWM trực tiếp từ bộ phát tín hiệu ngoài.
 *
 *   #define CONTROL_MODE  CRSF_MODE
 *     → Chế độ 2: Đọc dữ liệu từ bộ thu RC BetaFPV ELRS qua UART3 (giao thức CRSF)
 *       Phù hợp khi dùng tay cầm điều khiển RC không dây ELRS.
 * ==============================================================================
 */
#define PWM_MODE   1
#define CRSF_MODE  2
#define CONTROL_MODE  CRSF_MODE   /* <<< THAY ĐỔI TẠI ĐÂY */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* --------------------------------------------------------------------------
 * [CHẾ ĐỘ 1 - PWM] Biến bắt xung đầu vào từ Timer2 Input Capture
 * -------------------------------------------------------------------------- */
volatile uint32_t capture1 = 0;
volatile uint32_t sharedPulseWidth = 0;       // Xung CH1: điều khiển đèn (µs)
volatile uint8_t  isFirstCaptured = 0;
volatile uint32_t lastPulseTime = 0;           // Timestamp nhận xung CH1 cuối

volatile uint32_t capture1_ch2 = 0;
volatile uint32_t sharedPulseWidth_ch2 = 0;   // Xung CH2: điều khiển servo 2 (µs)
volatile uint8_t  isFirstCaptured_ch2 = 0;
volatile uint32_t lastPulseTime_ch2 = 0;

volatile uint32_t capture1_ch3 = 0;
volatile uint32_t sharedPulseWidth_ch3 = 0;   // Xung CH3: điều khiển servo 3 (µs)
volatile uint8_t  isFirstCaptured_ch3 = 0;
volatile uint32_t lastPulseTime_ch3 = 0;

/* --------------------------------------------------------------------------
 * [CHẾ ĐỘ 2 - CRSF] Bộ đệm DMA và dữ liệu CRSF từ UART3
 * -------------------------------------------------------------------------- */
uint8_t      crsf_rx_buffer[CRSF_FRAME_SIZE_MAX]; // Bộ đệm nhận DMA
CRSF_Data_t  crsf_data;                           // Dữ liệu sau khi parse CRSF
extern DMA_HandleTypeDef hdma_usart3_rx;

/* --------------------------------------------------------------------------
 * [DÙNG CHUNG] Biến đầu ra chuẩn hoá (µs) - cầu nối giữa nguồn đầu vào và đầu ra
 * Cả 2 chế độ đều đổ dữ liệu vào 3 biến này trước khi xử lý đầu ra.
 * -------------------------------------------------------------------------- */
uint32_t ctrl_light  = 0;   // Giá trị điều khiển đèn  (1000..2000µs, 0 = mất tín hiệu)
uint32_t ctrl_servo2 = 0;   // Giá trị điều khiển servo 2 (1000..2000µs)
uint32_t ctrl_servo3 = 0;   // Giá trị điều khiển servo 3 (1000..2000µs)

/* --------------------------------------------------------------------------
 * [DÙNG CHUNG] Hiệu ứng đèn SOS
 * -------------------------------------------------------------------------- */
// Mảng thời gian các bước SOS: S=3 chớp ngắn, O=3 chớp dài, S=3 chớp ngắn
// Mỗi cặp (ON_time, OFF_time) ứng với 1 ký tự Morse
const unsigned int sosDelays[] = {
  150, 150, 150, 150, 150, 450,  // S: . . .
  450, 150, 450, 150, 450, 450,  // O: - - -
  150, 150, 150, 150, 150, 1050  // S: . . .
};
int      sosStep      = 0;
uint32_t lastSosMillis = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==========================================================================
 * INPUT CAPTURE CALLBACK - Chế độ 1: Đọc xung PWM vào từ TIM2
 * Hàm này được gọi tự động bởi HAL mỗi khi Timer2 bắt được cạnh xung.
 * Ba kênh TIM2_CH1/2/3 tương ứng với đèn/servo2/servo3.
 * ========================================================================== */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* --- Kênh 1: Điều khiển Đèn (PA0 / TIM2_CH1) --- */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if (isFirstCaptured == 0) // Cạnh LÊN: ghi lại thời điểm bắt đầu xung
    {
      capture1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      isFirstCaptured = 1;
      // Chuyển cực để bắt cạnh XUỐNG tiếp theo
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else // Cạnh XUỐNG: tính độ rộng xung (µs)
    {
      uint32_t capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      if (capture2 > capture1) {
        sharedPulseWidth = capture2 - capture1;
      } else {
        sharedPulseWidth = (0xFFFFFFFF - capture1) + capture2 + 1; // Xử lý tràn timer
      }
      lastPulseTime   = HAL_GetTick();
      isFirstCaptured = 0;
      // Chuyển cực để bắt cạnh LÊN của chu kỳ tiếp theo
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }

  /* --- Kênh 2: Điều khiển Servo 2 (TIM2_CH2) --- */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    if (isFirstCaptured_ch2 == 0)
    {
      capture1_ch2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
      isFirstCaptured_ch2 = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
      uint32_t capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
      if (capture2 > capture1_ch2) {
        sharedPulseWidth_ch2 = capture2 - capture1_ch2;
      } else {
        sharedPulseWidth_ch2 = (0xFFFFFFFF - capture1_ch2) + capture2 + 1;
      }
      lastPulseTime_ch2   = HAL_GetTick();
      isFirstCaptured_ch2 = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }

  /* --- Kênh 3: Điều khiển Servo 3 (TIM2_CH3) --- */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    if (isFirstCaptured_ch3 == 0)
    {
      capture1_ch3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
      isFirstCaptured_ch3 = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
      uint32_t capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
      if (capture2 > capture1_ch3) {
        sharedPulseWidth_ch3 = capture2 - capture1_ch3;
      } else {
        sharedPulseWidth_ch3 = (0xFFFFFFFF - capture1_ch3) + capture2 + 1;
      }
      lastPulseTime_ch3   = HAL_GetTick();
      isFirstCaptured_ch3 = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
}

/* ==========================================================================
 * HÀM TIỆN ÍCH - Ánh xạ (map) giá trị từ một khoảng sang khoảng khác
 * Tương tự hàm map() trong Arduino.
 * Ví dụ: crsf_map(1000, 172, 1811, 1000, 2000) → 1495
 * ========================================================================== */
static int32_t map_range(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* ==========================================================================
 * HÀM TIỆN ÍCH - Giới hạn (clamp) giá trị trong khoảng [lo, hi]
 * ========================================================================== */
static int32_t clamp(int32_t val, int32_t lo, int32_t hi)
{
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

/* ==========================================================================
 * HÀM ĐIỀU KHIỂN ĐÈN - Logic dùng chung cho cả 2 chế độ
 * Dùng biến ctrl_light (µs) để quyết định trạng thái đèn PA6:
 *   > 1750µs  → Chế độ SOS (nhấp nháy khẩn cấp)
 *   500-1250µs → Đèn BẬT (sáng liên tục)
 *   Còn lại   → Đèn TẮT (mất tín hiệu hoặc mức giữa)
 * ========================================================================== */
static void handleLightOutput(void)
{
  if (ctrl_light > 1750)
  {
    // SOS: Chạy máy trạng thái SOS không chặn (non-blocking)
    if (HAL_GetTick() - lastSosMillis >= sosDelays[sosStep]) {
      lastSosMillis = HAL_GetTick();
      // Bước chẵn → sáng, bước lẻ → tắt
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (sosStep % 2 == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      sosStep++;
      if (sosStep >= 18) sosStep = 0; // Lặp lại chu kỳ SOS
    }
  }
  else if (ctrl_light > 500 && ctrl_light < 1250)
  {
    // Đèn BẬT liên tục
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    sosStep = 0; // Reset trạng thái SOS để sau này bắt đầu từ đầu
  }
  else
  {
    // Đèn TẮT (mất tín hiệu hoặc joystick ở mức giữa)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    sosStep = 0;
  }
}

/* ==========================================================================
 * HÀM ĐIỀU KHIỂN SERVO - Logic dùng chung cho cả 2 chế độ
 * Dùng ctrl_servo2/3 (đơn vị µs, 1000..2000) để xuất xung PWM TIM3.
 * Ánh xạ 1000-2000µs → 500-2500µs để phù hợp với servo 180°.
 * ========================================================================== */
static void handleServoOutput(void)
{
  /* Servo 2 (TIM3_CH2) */
  if (ctrl_servo2 > 0)
  {
    // Ánh xạ 1000-2000µs (RC standard) sang 500-2500µs (dải đầy đủ của servo)
    int32_t pwm2 = map_range((int32_t)ctrl_servo2, 1000, 2000, 500, 2500);
    pwm2 = clamp(pwm2, 500, 2500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)pwm2);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); // Mất tín hiệu → tắt PWM
  }

  /* Servo 3 (TIM3_CH3) */
  if (ctrl_servo3 > 0)
  {
    int32_t pwm3 = map_range((int32_t)ctrl_servo3, 1000, 2000, 500, 2500);
    pwm3 = clamp(pwm3, 500, 2500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)pwm3);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
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
  MX_DMA_Init();
  MX_USB_Device_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Khởi động Timer xuất PWM (TIM3) trước tiên
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // Đưa servo về vị trí giữa (1500µs → pwm ≈ 1500us) để an toàn
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);

  // Chờ servo có thời gian về vị trí trước khi nhận lệnh mới
  HAL_Delay(1000);

#if CONTROL_MODE == PWM_MODE
  /* -----------------------------------------------------------------------
   * CHẾ ĐỘ 1: Khởi động Timer2 Input Capture Interrupt để đọc xung PWM vào
   * ----------------------------------------------------------------------- */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // Đèn
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); // Servo 2
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3); // Servo 3

#elif CONTROL_MODE == CRSF_MODE
  /* -----------------------------------------------------------------------
   * CHẾ ĐỘ 2: Khởi động nhận DMA + Idle Interrupt cho CRSF trên UART3
   * ----------------------------------------------------------------------- */
  CRSF_Init(&crsf_data);  /* Khởi tạo state + reset toàn bộ data */
  // Bắt đầu nhận DMA; khi UART rảnh (Idle) HAL_UARTEx_RxEventCallback sẽ được gọi
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, crsf_rx_buffer, CRSF_FRAME_SIZE_MAX);
  // Tắt ngắt Half Transfer để tránh callback sớm khi mới nhận được nửa frame
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* ======================================================================
     * BƯỚC 1: ĐỌC ĐẦU VÀO - Thu thập dữ liệu theo chế độ đã chọn
     * ====================================================================== */

#if CONTROL_MODE == PWM_MODE
    /* -----------------------------------------------------------------------
     * CHẾ ĐỘ 1 - PWM Input: Kiểm tra timeout và đọc xung từ TIM2
     * Nếu không nhận được xung trong 50ms → coi như mất tín hiệu (= 0)
     * ----------------------------------------------------------------------- */

    // Timeout kênh đèn
    if (HAL_GetTick() - lastPulseTime > 50) {
      __disable_irq();
      sharedPulseWidth = 0;
      __enable_irq();
    }
    // Timeout kênh servo 2
    if (HAL_GetTick() - lastPulseTime_ch2 > 50) {
      __disable_irq();
      sharedPulseWidth_ch2 = 0;
      __enable_irq();
    }
    // Timeout kênh servo 3
    if (HAL_GetTick() - lastPulseTime_ch3 > 50) {
      __disable_irq();
      sharedPulseWidth_ch3 = 0;
      __enable_irq();
    }

    // Đọc an toàn giá trị xung (tắt ngắt để tránh race condition)
    __disable_irq();
    uint32_t raw_light  = sharedPulseWidth;
    uint32_t raw_servo2 = sharedPulseWidth_ch2;
    uint32_t raw_servo3 = sharedPulseWidth_ch3;
    __enable_irq();

    // Gán trực tiếp vào biến chuẩn hoá (xung TIM2 đã là đơn vị µs)
    ctrl_light  = raw_light;
    ctrl_servo2 = raw_servo2;
    ctrl_servo3 = raw_servo3;

#elif CONTROL_MODE == CRSF_MODE
    /* -----------------------------------------------------------------------
     * CHẾ ĐỘ 2 - CRSF Input: Dữ liệu đã được parse trong callback ngắt.
     * Tại đây chỉ cần kiểm tra timeout và chuyển đổi đơn vị kênh CRSF → µs.
     *
     * Quy ước kênh CRSF:
     *   CH1 → ctrl_light  (đèn)
     *   CH2 → ctrl_servo2 (servo 2)
     *   CH3 → ctrl_servo3 (servo 3)
     *
     * Giá trị CRSF 11-bit: 172 (min) | 992 (mid) | 1811 (max)
     * Ánh xạ → chuẩn RC µs:  1000µs  | 1500µs   | 2000µs
     * ----------------------------------------------------------------------- */

    // Kiểm tra kết nối và timeout tự động (hàm CRSF_IsConnected cập nhật is_connected)
    if (CRSF_IsConnected(&crsf_data)) {
      // Dùng CRSF_GetChannelUs() để chuyển đổi 11-bit → µs (đã clamp 1000-2000)
      // CH6 (index 5) → đèn | CH2 (index 1) → servo2 | CH3 (index 2) → servo3
      ctrl_light  = CRSF_GetChannelUs(&crsf_data, 6);
      ctrl_servo2 = CRSF_GetChannelUs(&crsf_data, 2);
      ctrl_servo3 = CRSF_GetChannelUs(&crsf_data, 3);
    } else {
      // Mất kết nối: đặt tất cả về 0 để vô hiệu hoá đầu ra
      ctrl_light  = 0;
      ctrl_servo2 = 0;
      ctrl_servo3 = 0;
    }
#endif

    /* ======================================================================
     * BƯỚC 2: XỬ LÝ ĐẦU RA - Giống hệt nhau cho cả 2 chế độ
     * Sử dụng ctrl_light, ctrl_servo2, ctrl_servo3 (đơn vị µs)
     * ====================================================================== */

    handleLightOutput();   // Điều khiển đèn PA6 (SOS / ON / OFF)
    handleServoOutput();   // Xuất xung PWM TIM3 cho 2 servo
    
    // In thông tin qua Serial (500ms một lần)
    static uint32_t lastPrintTime = 0;
    if (HAL_GetTick() - lastPrintTime >= 500) {
      lastPrintTime = HAL_GetTick();
      #if CONTROL_MODE == CRSF_MODE
      if (crsf_data.is_connected) {
         printf("CRSF Mode (Connected) | Light(CH6): %u us | Servo2(CH2): %u us | Servo3(CH3): %u us\r\n", 
                 (unsigned int)ctrl_light, (unsigned int)ctrl_servo2, (unsigned int)ctrl_servo3);
      } else {
         printf("CRSF Mode (Disconnected)\r\n");
      }
      #elif CONTROL_MODE == PWM_MODE
      printf("PWM Mode | Light: %u us | Servo2: %u us | Servo3: %u us\r\n", 
             (unsigned int)ctrl_light, (unsigned int)ctrl_servo2, (unsigned int)ctrl_servo3);
      #endif
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

/* ==========================================================================
 * UART RX EVENT CALLBACK - Chế độ 2: Nhận frame CRSF qua DMA + Idle Interrupt
 * Hàm này được HAL gọi tự động khi đường UART3 rảnh sau khi nhận dữ liệu.
 * Tham số Size: số byte thực sự nhận được trong bộ đệm DMA.
 * ========================================================================== */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3)
  {
    // Parse buffer vừa nhận, cập nhật crsf_data nếu frame hợp lệ (CRC đúng)
    CRSF_ParseFrame(crsf_rx_buffer, Size, &crsf_data);

    // Khởi động lại việc nhận DMA cho frame tiếp theo
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, crsf_rx_buffer, CRSF_FRAME_SIZE_MAX);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); // Tắt ngắt Half Transfer
  }
}

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

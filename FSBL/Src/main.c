/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MAVLink GPS STM32N6570-DK ↔ Cube Orange
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#define MAVLINK_COMM_NUM_BUFFERS 1
#include "mavlink/common/mavlink.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DBG(fmt, ...) do {                                          \
    char _b[128];                                                   \
    snprintf(_b, sizeof(_b), fmt "\r\n", ##__VA_ARGS__);           \
    HAL_UART_Transmit(&huart1, (uint8_t*)_b, strlen(_b), 100);     \
} while(0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t rx_byte;

static volatile float   gps_lat     = 0;
static volatile float   gps_lon     = 0;
static volatile float   gps_alt     = 0;
static volatile uint8_t gps_updated = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  SCB_EnableICache();
  SCB_EnableDCache();
  HAL_Init();

  /* USER CODE BEGIN Init */
  RCC->BUSENSR = 0xFFFFFFFF;
  HAL_PWREx_EnableVddA();
  HAL_PWREx_EnableVddIO2();
  HAL_PWREx_EnableVddIO3();
  HAL_PWREx_EnableVddIO4();
  HAL_PWREx_EnableVddIO5();
  __HAL_RCC_PWR_CLK_ENABLE();
  if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK) while(1);
  if (HAL_PWREx_GetSupplyConfig() != PWR_EXTERNAL_SOURCE_SUPPLY)    while(1);
  __HAL_RCC_CPUCLK_CONFIG(RCC_CPUCLKSOURCE_HSI);
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  DBG("===GPS MAVLink STM32N6570-DK ===");
  DBG("USART1=PC(VCP)  USART2=CubeOrange");
  /* USER CODE END 2 */


  /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
    	if (gps_updated == 2)
    	{
    	  gps_updated = 0;
    	  DBG("START BYTE recu !");
    	}
    	else if (gps_updated == 3)
    	{
    	  gps_updated = 0;
    	  DBG("MSG complet ! id=%d", 0);
    	}
    	else if (gps_updated == 1)
    	{
    	  __disable_irq();
    	  float lat = gps_lat;
    	  float lon = gps_lon;
    	  float alt = gps_alt;
    	  gps_updated = 0;
    	  __enable_irq();
    	  DBG("[GPS] lat=%.6f lon=%.6f alt=%.1f m", lat, lon, alt);
    	}
    }
    /* USER CODE END 3 */


}

/* USER CODE BEGIN CLK 1 */
/* USER CODE END CLK 1 */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK)
    Error_Handler();
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    Error_Handler();

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv              = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL1.PLLState       = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState       = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState       = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState       = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
  if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
     (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
  {
    RCC_ClkInitStruct.ClockType    = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
    RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
      Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType     = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL1.PLLState      = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource     = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL1.PLLM          = 4;
  RCC_OscInitStruct.PLL1.PLLN          = 75;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL1.PLLP1         = 1;
  RCC_OscInitStruct.PLL1.PLLP2         = 1;
  RCC_OscInitStruct.PLL2.PLLState      = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState      = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState      = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_HCLK
                                   | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
                                   | RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_PCLK5
                                   | RCC_CLOCKTYPE_PCLK4;
  RCC_ClkInitStruct.CPUCLKSource   = RCC_CPUCLKSOURCE_IC1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;
  RCC_ClkInitStruct.IC1Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC1Selection.ClockDivider    = 2;
  RCC_ClkInitStruct.IC2Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC2Selection.ClockDivider    = 3;
  RCC_ClkInitStruct.IC6Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC6Selection.ClockDivider    = 4;
  RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC11Selection.ClockDivider   = 3;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
    Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance            = USART1;
  huart1.Init.BaudRate       = 115200;
  huart1.Init.WordLength     = UART_WORDLENGTH_8B;
  huart1.Init.StopBits       = UART_STOPBITS_1;
  huart1.Init.Parity         = UART_PARITY_ODD;
  huart1.Init.Mode           = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance            = USART2;
  huart2.Init.BaudRate       = 57600;
  huart2.Init.WordLength     = UART_WORDLENGTH_8B;
  huart2.Init.StopBits       = UART_STOPBITS_1;
  huart2.Init.Parity         = UART_PARITY_NONE;
  huart2.Init.Mode           = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();

  HAL_Delay(2000);  /* attendre 2 secondes que le Cube soit prêt */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOE_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if (UartHandle->Instance == USART2)
  {
    mavlink_message_t msg;
    mavlink_status_t  status;

    uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &msg, &status);

    /* Debug — affiche seulement les bytes MAVLink valides */
    if (rx_byte == 0xFD || rx_byte == 0xFE)  /* start bytes MAVLink */
    {
      gps_updated = 2;  /* signal spécial pour le while(1) */
    }

    if (result)
    {
      gps_updated = 3;  /* message complet reçu */
      if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
      {
        mavlink_global_position_int_t gps;
        mavlink_msg_global_position_int_decode(&msg, &gps);
        gps_lat     = gps.lat / 1e7f;
        gps_lon     = gps.lon / 1e7f;
        gps_alt     = gps.alt / 1000.0f;
        gps_updated = 1;
        BSP_LED_Toggle(LED2);
      }
    }
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	/* Relance la réception au lieu d'aller en erreur */
	  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}
/* USER CODE END 4 */

void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	  while (1)
	  {
	    BSP_LED_Toggle(LED1);
	    BSP_LED_Toggle(LED2);
	    HAL_Delay(200);
	  }
	  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  UNUSED(file);
  UNUSED(line);
  while (1) {}
  /* USER CODE END 6 */
}
#endif

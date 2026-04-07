/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MAVLink GPS STM32N6570-DK ↔ Cube Orange
  *                   Réception USART2 par interruption (sans DMA)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#define MAVLINK_COMM_NUM_BUFFERS 1
#include "mavlink/common/mavlink.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* USER CODE BEGIN PM */
/*
 * DBG : trace sur USART1 (VCP debug).
 * NE PAS appeler depuis HAL_UART_ErrorCallback.
 */
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

/* Buffer de réception : 1 octet à la fois */
static uint8_t rx_byte;

/* Compteurs diagnostic */
static volatile uint32_t byte_count         = 0;
static volatile uint32_t mavlink_msg_count  = 0;
static volatile uint32_t mavlink_last_msgid = 0;
static volatile uint32_t uart2_errors       = 0;

/* Données GPS */
static volatile float   gps_lat     = 0.0f;
static volatile float   gps_lon     = 0.0f;
static volatile float   gps_alt     = 0.0f;
static volatile uint8_t gps_fix     = 0;
static volatile uint8_t gps_updated = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void request_mavlink_streams(void);
static void send_heartbeat(void);
/* USER CODE END PFP */

/* ==========================================================================
   MAIN
   ========================================================================== */
int main(void)
{
  SCB_EnableICache();
  /* D-Cache volontairement désactivée — pas de problème de cohérence avec IT */

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
  DBG("=== GPS MAVLink STM32N6570-DK ===");
  DBG("USART1=VCP  USART2=CubeOrange IT 57600");

  __HAL_UART_FLUSH_DRREGISTER(&huart2);
  HAL_Delay(200);

  request_mavlink_streams();
  DBG("Streams GPS requested");
  /* Lancer la réception IT — UNE SEULE FOIS, elle se relance dans le callback */
  if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1) != HAL_OK)
  {
      DBG("ERREUR : Receive_IT failed !");
      Error_Handler();
  }
  //HAL_Delay(500);


  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  uint32_t last_hb        = 0;
  uint32_t last_print     = 0;
  uint32_t last_diag      = 0;
  uint32_t last_byte_snap = 0;

  while (1)
  {
      uint32_t now = HAL_GetTick();

      /* -- Heartbeat toutes les secondes ---------------------------------- */
      if (now - last_hb >= 1000)
      {
          last_hb = now;
          send_heartbeat();
          BSP_LED_Toggle(LED1);
      }

      /* -- Affichage GPS quand données fraîches --------------------------- */
      if (gps_updated && (now - last_print >= 200))
      {
          last_print  = now;
          gps_updated = 0;

          int32_t lat_d  = (int32_t)gps_lat;
          int32_t lat_f  = (int32_t)((gps_lat - (float)lat_d) * 1000000.0f);
          int32_t lon_d  = (int32_t)gps_lon;
          int32_t lon_f  = (int32_t)((gps_lon - (float)lon_d) * 1000000.0f);
          int32_t alt_cm = (int32_t)(gps_alt * 100.0f);
          if (lat_f < 0) lat_f = -lat_f;
          if (lon_f < 0) lon_f = -lon_f;

          DBG("GPS fix=%d  lat=%ld.%06ld  lon=%ld.%06ld  alt=%ldcm",
              (int)gps_fix,
              (long)lat_d, (long)lat_f,
              (long)lon_d, (long)lon_f,
              (long)alt_cm);
      }

      /* -- Diagnostic toutes les 5 secondes ------------------------------- */
      if (now - last_diag >= 5000)
      {
          last_diag = now;
          uint32_t delta = byte_count - last_byte_snap;
          last_byte_snap = byte_count;

          DBG("Diag: bytes/5s=%lu  mavlink_msgs=%lu  lastid=%lu  errors=%lu",
              (unsigned long)delta,
              (unsigned long)mavlink_msg_count,
              (unsigned long)mavlink_last_msgid,
              (unsigned long)uart2_errors);

          if (delta == 0)
              DBG("WARN: 0 octet recu — verif cablage/baud/SERIAL2_PROTOCOL=2");
          else if (mavlink_msg_count == 0)
              DBG("WARN: octets recus mais 0 msg MAVLink — verif baud rate");
      }

      HAL_Delay(10);
  }
  /* USER CODE END WHILE */
}

/* ==========================================================================
   CLOCK CONFIG
   ========================================================================== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK) Error_Handler();
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) Error_Handler();

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv              = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL1.PLLState       = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState       = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState       = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState       = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
  if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
      (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
  {
      RCC_ClkInitStruct.ClockType    = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK;
      RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL1.PLLState       = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource      = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL1.PLLM          = 4;
  RCC_OscInitStruct.PLL1.PLLN          = 75;
  RCC_OscInitStruct.PLL1.PLLFractional  = 0;
  RCC_OscInitStruct.PLL1.PLLP1         = 1;
  RCC_OscInitStruct.PLL1.PLLP2         = 1;
  RCC_OscInitStruct.PLL2.PLLState       = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState       = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState       = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType          = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_HCLK
                                       | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
                                       | RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_PCLK5
                                       | RCC_CLOCKTYPE_PCLK4;
  RCC_ClkInitStruct.CPUCLKSource       = RCC_CPUCLKSOURCE_IC1;
  RCC_ClkInitStruct.SYSCLKSource       = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
  RCC_ClkInitStruct.AHBCLKDivider     = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider    = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider    = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider    = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5CLKDivider    = RCC_APB5_DIV1;
  RCC_ClkInitStruct.IC1Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC1Selection.ClockDivider    = 2;
  RCC_ClkInitStruct.IC2Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC2Selection.ClockDivider    = 3;
  RCC_ClkInitStruct.IC6Selection.ClockSelection  = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC6Selection.ClockDivider    = 4;
  RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC11Selection.ClockDivider   = 3;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK) Error_Handler();
}

/* ==========================================================================
   UART / GPIO INITS
   ========================================================================== */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance                     = USART1;
  huart1.Init.BaudRate               = 115200;
  huart1.Init.WordLength             = UART_WORDLENGTH_8B;
  huart1.Init.StopBits               = UART_STOPBITS_1;
  huart1.Init.Parity                 = UART_PARITY_ODD;
  huart1.Init.Mode                   = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit  = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)                                            Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)                               Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance                     = USART2;
  huart2.Init.BaudRate               = 57600;   /* Cube Orange TELEM = 57600 par défaut */
  huart2.Init.WordLength             = UART_WORDLENGTH_8B;
  huart2.Init.StopBits               = UART_STOPBITS_1;
  huart2.Init.Parity                 = UART_PARITY_NONE;
  huart2.Init.Mode                   = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit  = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)                                            Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)                               Error_Handler();
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
}

/* ==========================================================================
   USER CODE 4
   ========================================================================== */
/* USER CODE BEGIN 4 */

/**
  * @brief Callback RX IT — appelé pour chaque octet reçu sur USART2
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if (UartHandle->Instance != USART2) return;

    mavlink_message_t msg;
    mavlink_status_t  status;
    static uint8_t rx_byte_array[256] = {0};


    mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &msg, &status);
    rx_byte_array[byte_count] = rx_byte;

    byte_count++;
    if(byte_count == 256) {
    	for(int i = 0 ; i < 20 ; i++) {
    		DBG("Value %d = %d", i, rx_byte_array[i]);
    	}
    	DBG("\n");
    	byte_count = 0;
    }



    /* Parser MAVLink octet par octet */
        /*
    if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &msg, &status))
    {
        mavlink_msg_count++;

        mavlink_last_msgid = msg.msgid;

        switch (msg.msgid)
        {
            case MAVLINK_MSG_ID_GPS_RAW_INT:
            {
                mavlink_gps_raw_int_t gps;
                mavlink_msg_gps_raw_int_decode(&msg, &gps);
                gps_lat     = gps.lat  / 1e7f;
                gps_lon     = gps.lon  / 1e7f;
                gps_alt     = gps.alt  / 1000.0f;
                gps_fix     = gps.fix_type;
                gps_updated = 1;
                break;
            }
            case MAVLINK_MSG_ID_HEARTBEAT:
                BSP_LED_Toggle(LED2);  // LED2 = Cube vivant
                break;
            default:
                break;
        }
    }*/

    /* Relancer pour l'octet suivant — OBLIGATOIRE */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

/**
  * @brief Erreur UART — reset silencieux + relance
  *        PAS de DBG ici (bloquant → overrun en cascade)
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    if (UartHandle->Instance == USART2)
    {
        uart2_errors++;
        huart2.ErrorCode = HAL_UART_ERROR_NONE;
        huart2.RxState   = HAL_UART_STATE_READY;
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) { (void)UartHandle; }

/* ---- MAVLink helpers ----------------------------------------------------- */

static void request_mavlink_streams(void)
{
    mavlink_message_t msg;
    uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;

    /* POSITION → GLOBAL_POSITION_INT (msgid=33) à 2 Hz */
    mavlink_msg_request_data_stream_pack(
        255, 0, &msg, 1, 1,
        MAV_DATA_STREAM_POSITION, 2, 1
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart2, buf, len, 100);

    /* RAW_SENSORS → GPS_RAW_INT (msgid=24) à 2 Hz */
    mavlink_msg_request_data_stream_pack(
        255, 0, &msg, 1, 1,
        MAV_DATA_STREAM_RAW_SENSORS, 2, 1
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart2, buf, len, 100);
}

static void send_heartbeat(void)
{
    mavlink_message_t msg;
    uint8_t  buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        255, 0, &msg,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE
    );
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart2, buf, len, 100);
}

/* USER CODE END 4 */

/* ==========================================================================
   ERROR HANDLER
   ========================================================================== */
void Error_Handler(void)
{
    while (1)
    {
        BSP_LED_Toggle(LED1);
        BSP_LED_Toggle(LED2);
        HAL_Delay(200);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    UNUSED(file);
    UNUSED(line);
    while (1) {}
}
#endif

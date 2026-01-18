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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void SetSpeed_L(int16_t Speed)
{
    if (Speed>=0)
    {
      HAL_GPIO_WritePin(LAIN1_GPIO_Port, LAIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LAIN2_GPIO_Port, LAIN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LBIN1_GPIO_Port, LBIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LBIN2_GPIO_Port, LBIN2_Pin, GPIO_PIN_RESET);
      /* clip to timer period */
      int16_t absSpeed = (Speed>0)?Speed:0;
      if (htim3.Init.Period > 0 && absSpeed > (int16_t)htim3.Init.Period) 
      {
        absSpeed = (int16_t)htim3.Init.Period;
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)absSpeed);
    }
    else
    {
      HAL_GPIO_WritePin(LAIN1_GPIO_Port, LAIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LAIN2_GPIO_Port, LAIN2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LBIN1_GPIO_Port, LBIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LBIN2_GPIO_Port, LBIN2_Pin, GPIO_PIN_SET);
      int absVal = abs((int)Speed);
      if (htim3.Init.Period > 0 && absVal > (int16_t)htim3.Init.Period)
      {
        absVal = (int16_t)htim3.Init.Period;
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)absVal);
    }
}

void SetSpeed_R(int16_t Speed)
{
    if (Speed>=0)
    {
      HAL_GPIO_WritePin(RAIN1_GPIO_Port, RAIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RAIN2_GPIO_Port, RAIN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RBIN1_GPIO_Port, RBIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RBIN2_GPIO_Port, RBIN2_Pin, GPIO_PIN_RESET);
      /* clip to timer period */
      int16_t absSpeed = (Speed>0)?Speed:0;
      if (htim4.Init.Period > 0 && absSpeed > (int16_t)htim4.Init.Period) 
      {
        absSpeed = (int16_t)htim4.Init.Period;
      }
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t)absSpeed);
    }
    else
    {
      HAL_GPIO_WritePin(RAIN1_GPIO_Port, RAIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RAIN2_GPIO_Port, RAIN2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RBIN1_GPIO_Port, RBIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RBIN2_GPIO_Port, RBIN2_Pin, GPIO_PIN_SET);
      int absVal = abs((int)Speed);
      if (htim4.Init.Period > 0 && absVal > (int16_t)htim4.Init.Period)
      {
        absVal = (int16_t)htim4.Init.Period;
      }
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t)absVal);
    }
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_data = 0;

int16_t SpeedA = 0;
int16_t SpeedB = 0;
/* UART packet buffer for assembling incoming bytes */
static char uart_rx_buf[64];
static uint8_t uart_rx_idx = 0;
/* joystick filters/state */
#define JOY_FILTER_SIZE 2
static int16_t joyLyBuf[JOY_FILTER_SIZE];
static int16_t joyRyBuf[JOY_FILTER_SIZE];
static uint8_t joyFilterPos = 0;
static uint8_t joyFilterCount = 0;
/* last applied mapped speeds (timer units) */
static int16_t lastAppliedA = 0;
static int16_t lastAppliedB = 0;
/* consecutive zero counters to avoid single-packet zeroing */
static uint8_t zeroCountA = 0;
static uint8_t zeroCountB = 0;
/* watchdog timeout (ms): if no valid joystick packet within this, stop motors */
#define JOY_TIMEOUT_MS 500U
static uint32_t last_valid_rx_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /*
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(1, 1, "Speed A:");
  */
  
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  
  for (uint8_t i = 0; i < 10; i++)
  {
    HAL_GPIO_WritePin(Lazer_GPIO_Port, Lazer_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(Lazer_GPIO_Port, Lazer_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
  }


  SetSpeed_L(25);
  SetSpeed_R(-25);
  HAL_Delay(1000);
  SetSpeed_L(0);
  SetSpeed_R(0);

  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* watchdog: if no valid joystick packet within JOY_TIMEOUT_MS, stop motors */
    if ((HAL_GetTick() - last_valid_rx_tick) > JOY_TIMEOUT_MS)
    {
      if (lastAppliedA != 0 || lastAppliedB != 0)
      {
        SetSpeed_L(0);
        SetSpeed_R(0);
        lastAppliedA = 0;
        lastAppliedB = 0;
        SpeedA = 0;
        SpeedB = 0;
        /*
        OLED_ShowString(1, 8, "   0");
        OLED_ShowString(2, 8, "   0");
        */
      }
    }
    HAL_Delay(1);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Parse joystick packet: format [joystick,Lx,Ly,Rx,Ry] where values are -100..100 */
static void ProcessJoystickPacket(char *buf)
{
  int Lx, Ly, Rx, Ry;
  /* find last occurrence of header to avoid misalignment */
  char *start = NULL;
  char *psearch = buf;
  while (1)
  {
    char *f = strstr(psearch, "[joystick,");
    if (!f) break;
    start = f;
    psearch = f + 1;
  }
  if (!start) return;
  char *end = strchr(start, ']');
  if (!end) return;
  /* parse only the substring from start to end */
  char tmpBuf[48];
  size_t len = (size_t)(end - (start + 10)); /* after header */
  if (len >= sizeof(tmpBuf)) return;
  memcpy(tmpBuf, start + 10, len);
  tmpBuf[len] = '\0';
  if (sscanf(tmpBuf, "%d,%d,%d,%d", &Lx, &Ly, &Rx, &Ry) != 4) return;
  /* validate ranges */
  if (Lx < -100 || Lx > 100 || Ly < -100 || Ly > 100 || Rx < -100 || Rx > 100 || Ry < -100 || Ry > 100) return;

  /* valid packet received: update last valid timestamp */
  last_valid_rx_tick = HAL_GetTick();

  /* push into moving average buffers */
  joyLyBuf[joyFilterPos] = (int16_t)Ly;
  joyRyBuf[joyFilterPos] = (int16_t)Ry;
  joyFilterPos = (joyFilterPos + 1) % JOY_FILTER_SIZE;
  if (joyFilterCount < JOY_FILTER_SIZE) joyFilterCount++;

  int sumLy = 0, sumRy = 0;
  for (uint8_t i = 0; i < joyFilterCount; i++)
  {
    sumLy += joyLyBuf[i];
    sumRy += joyRyBuf[i];
  }
  int16_t avgLy = (int16_t)(sumLy / (int)joyFilterCount);
  int16_t avgRy = (int16_t)(sumRy / (int)joyFilterCount);

  /* apply deadzone to avoid small jitter around 0 */
  const int deadzone = 5; /* joystick units */
  if (avgLy > -deadzone && avgLy < deadzone) avgLy = 0;
  if (avgRy > -deadzone && avgRy < deadzone) avgRy = 0;

  /* map -100..100 to -Period..Period */
  int16_t periodL = (int16_t)(htim3.Init.Period);
  int16_t periodR = (int16_t)(htim4.Init.Period);
  int16_t sA = (int16_t)((avgLy * periodL) / 100);
  int16_t sB = (int16_t)((avgRy * periodR) / 100);

  /* avoid single-packet zero glitches: require two consecutive zero packets to force zero */
  if (sA == 0)
  {
    if (lastAppliedA != 0)
    {
      zeroCountA++;
      if (zeroCountA < 2) sA = lastAppliedA; /* ignore this zero */
      else zeroCountA = 0; /* accept zero and reset counter */
    }
  }
  else
  {
    zeroCountA = 0;
  }
  if (sB == 0)
  {
    if (lastAppliedB != 0)
    {
      zeroCountB++;
      if (zeroCountB < 2) sB = lastAppliedB;
      else zeroCountB = 0;
    }
  }
  else
  {
    zeroCountB = 0;
  }

  /* apply small hysteresis: only update if change is significant */
  int16_t deltaThresholdA = (int16_t)( (periodL>0) ? (periodL / 100) : 1 ); /* ~1% */
  int16_t deltaThresholdB = (int16_t)( (periodR>0) ? (periodR / 100) : 1 );
  if (deltaThresholdA < 1) deltaThresholdA = 1;
  if (deltaThresholdB < 1) deltaThresholdB = 1;
  if ( (sA != lastAppliedA) && (abs(sA - lastAppliedA) >= deltaThresholdA) )
  {
    SetSpeed_L(sA);
    lastAppliedA = sA;
  }
  if ( (sB != lastAppliedB) && (abs(sB - lastAppliedB) >= deltaThresholdB) )
  {
    SetSpeed_R(sB);
    lastAppliedB = sB;
  }

  SpeedA = lastAppliedA;
  SpeedB = lastAppliedB;
  /* update OLED numeric display (overwrite after labels) */
  char out[12];
  snprintf(out, sizeof(out), "%4d", lastAppliedA);
  OLED_ShowString(1, 8, out);
  snprintf(out, sizeof(out), "%4d", lastAppliedB);
  OLED_ShowString(2, 8, out);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
    /* append received byte to buffer and check for packet end ']' */
    if (uart_rx_idx < sizeof(uart_rx_buf) - 1)
    {
      uart_rx_buf[uart_rx_idx++] = (char)rx_data;
      uart_rx_buf[uart_rx_idx] = '\0';
      if ((char)rx_data == ']')
      {
        ProcessJoystickPacket(uart_rx_buf);
        uart_rx_idx = 0;
        uart_rx_buf[0] = '\0';
      }
    }
    else
    {
      /* overflow, reset */
      uart_rx_idx = 0;
      uart_rx_buf[0] = '\0';
    }
    /* restart receive */
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);
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

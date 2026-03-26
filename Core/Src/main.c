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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "pca9685.h"
#include "wheel_control.h"
/* #include "oled.h" */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OLED_UPDATE_MS 100U
/* Encoder calibration: left motor max measured speed in counts per second. */
#define LEFT_ENCODER_MAX_CPS 9400
#define RIGHT_ENCODER_MAX_CPS 9400
#define JOY_DEADZONE_PCT 4
#define JOY_TIMEOUT_MS 500U

/* Servo calibration test mode: set to 1 to run standalone servo test loop. */
#define SERVO_TEST_ENABLE 0
#define SERVO_TEST_ADDR_7BIT 0x40U
#define SERVO_TEST_CHANNEL 0U
#define SERVO_TEST_PWM_HZ 50U
/* Calibrate with a wide range first, then narrow if mechanical limit is reached. */
#define SERVO_TEST_MIN_PULSE_US 500U
#define SERVO_TEST_MAX_PULSE_US 2500U
/* Logical angle is forced to 0~180 for standard positional servo. */
#define SERVO_TEST_LOGICAL_MAX_ANGLE 180
#define SERVO_TEST_MIN_ANGLE 0
#define SERVO_TEST_MID_ANGLE 90
#define SERVO_TEST_MAX_ANGLE 180
#define SERVO_TEST_SWEEP_STEP 10
#define SERVO_TEST_HOLD_MS 2000U
#define SERVO_TEST_STEP_DELAY_MS 500U

/* [k,x,y] dual-servo control: x is q/a, y is d/u. */
#define K_DUAL_SERVO_CHANNEL_A 4U
#define K_DUAL_SERVO_CHANNEL_B 5U
#define K_DUAL_SERVO_COMP_SUM_ANGLE 180
#define K_DUAL_SERVO_CTRL_MIN_ANGLE 30
#define K_DUAL_SERVO_CTRL_MAX_ANGLE 115
#define K_DUAL_SERVO_INIT_ANGLE 30
#define K_DUAL_SERVO_SPEED_DEG_PER_SEC 120
#define K_DUAL_SERVO_STEP_PERIOD_MS 20U

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
      HAL_GPIO_WritePin(RAIN1_GPIO_Port, RAIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RAIN2_GPIO_Port, RAIN2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RBIN1_GPIO_Port, RBIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RBIN2_GPIO_Port, RBIN2_Pin, GPIO_PIN_SET);
      /* clip to timer period */
      int16_t absSpeed = (Speed>0)?Speed:0;
      if (htim3.Init.Period > 0 && absSpeed > (int16_t)htim3.Init.Period) 
      {
        absSpeed = (int16_t)htim3.Init.Period;
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)absSpeed);
    }
    else
    {
      HAL_GPIO_WritePin(RAIN1_GPIO_Port, RAIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RAIN2_GPIO_Port, RAIN2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RBIN1_GPIO_Port, RBIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RBIN2_GPIO_Port, RBIN2_Pin, GPIO_PIN_RESET);
      int absVal = abs((int)Speed);
      if (htim3.Init.Period > 0 && absVal > (int16_t)htim3.Init.Period)
      {
        absVal = (int16_t)htim3.Init.Period;
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)absVal);
    }
}

void SetSpeed_H(int16_t Speed)
{
    if (Speed > 0)
    {
      /* H motor forward */
      HAL_GPIO_WritePin(HIN1_GPIO_Port, HIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(HIN2_GPIO_Port, HIN2_Pin, GPIO_PIN_RESET);
    }
    else if (Speed < 0)
    {
      /* H motor reverse */
      HAL_GPIO_WritePin(HIN1_GPIO_Port, HIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(HIN2_GPIO_Port, HIN2_Pin, GPIO_PIN_SET);
    }
    else
    {
      /* stop: both low */
      HAL_GPIO_WritePin(HIN1_GPIO_Port, HIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(HIN2_GPIO_Port, HIN2_Pin, GPIO_PIN_RESET);
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
static uint8_t uart_in_frame = 0;
/* mailbox: always keep only latest completed frame */
static volatile char uart_latest_frame[64];
static volatile uint8_t uart_latest_ready = 0;
/* joystick filters/state */
#define JOY_FILTER_SIZE 1
static int16_t joyLyBuf[JOY_FILTER_SIZE];
static int16_t joyRyBuf[JOY_FILTER_SIZE];
static uint8_t joyFilterPos = 0;
static uint8_t joyFilterCount = 0;
static int16_t lastServoAngle = -1;
static WheelControl_t wheelControl = {0};
static int32_t dualServoAngleA_mdeg = 0;
static int16_t dualServoLastAppliedA = -1;
static int8_t dualServoDir = 0;
static uint8_t dualServoRunning = 0;
static uint32_t dualServoLastTick = 0;
// static uint32_t last_oled_update_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ProcessJoystickPacket(char *buf);
static void ProcessServoPacket(char *buf);
static void ProcessDualServoPacket(char *buf);
static void ProcessLatestUartFrame(void);
static HAL_StatusTypeDef ServoSetAngle180(uint16_t angleDeg);
static HAL_StatusTypeDef ServoSetAngle180ByChannel(uint8_t channel, uint16_t angleDeg);
static HAL_StatusTypeDef DualServoApplyComplementAngleA(int16_t angleA);
static void DualServoSyncStep(void);
static void RunServoCalibrationTest(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void ProcessUartBytes(void)
{
  ProcessLatestUartFrame();
}

static void ProcessLatestUartFrame(void)
{
  char frame[64];
  if (!uart_latest_ready)
  {
    return;
  }

  __disable_irq();
  strncpy(frame, (const char *)uart_latest_frame, sizeof(frame) - 1);
  frame[sizeof(frame) - 1] = '\0';
  uart_latest_ready = 0;
  __enable_irq();

  if (frame[0] == '[' && frame[1] != '\0')
  {
    if (frame[1] == 'j')
    {
      ProcessJoystickPacket(frame);
    }
    else if (frame[1] == 's')
    {
      ProcessServoPacket(frame);
    }
    else if (frame[1] == 'k')
    {
      ProcessDualServoPacket(frame);
    }
  }
}

static HAL_StatusTypeDef ServoSetAngle180(uint16_t angleDeg)
{
  return ServoSetAngle180ByChannel(SERVO_TEST_CHANNEL, angleDeg);
}

static HAL_StatusTypeDef ServoSetAngle180ByChannel(uint8_t channel, uint16_t angleDeg)
{
  uint32_t pulseUs;

  if (angleDeg > SERVO_TEST_LOGICAL_MAX_ANGLE)
  {
    angleDeg = SERVO_TEST_LOGICAL_MAX_ANGLE;
  }

  pulseUs = SERVO_TEST_MIN_PULSE_US +
            (((uint32_t)(SERVO_TEST_MAX_PULSE_US - SERVO_TEST_MIN_PULSE_US) * angleDeg) /
             SERVO_TEST_LOGICAL_MAX_ANGLE);

  return PCA9685_SetServoPulseUs(&hi2c2,
                                 SERVO_TEST_ADDR_7BIT,
                                 channel,
                                 (uint16_t)pulseUs,
                                 SERVO_TEST_PWM_HZ);
}

static HAL_StatusTypeDef DualServoApplyComplementAngleA(int16_t angleA)
{
  HAL_StatusTypeDef stA;
  HAL_StatusTypeDef stB;
  int16_t angleB;

  if (angleA < K_DUAL_SERVO_CTRL_MIN_ANGLE)
  {
    angleA = K_DUAL_SERVO_CTRL_MIN_ANGLE;
  }
  if (angleA > K_DUAL_SERVO_CTRL_MAX_ANGLE)
  {
    angleA = K_DUAL_SERVO_CTRL_MAX_ANGLE;
  }

  angleB = (int16_t)(K_DUAL_SERVO_COMP_SUM_ANGLE - angleA);

  stA = ServoSetAngle180ByChannel(K_DUAL_SERVO_CHANNEL_A, (uint16_t)angleA);
  stB = ServoSetAngle180ByChannel(K_DUAL_SERVO_CHANNEL_B, (uint16_t)angleB);

  if (stA != HAL_OK)
  {
    return stA;
  }
  return stB;
}

static void DualServoSyncStep(void)
{
  uint32_t nowTick;
  uint32_t elapsedMs;
  int32_t deltaMdeg;
  int32_t nextMdeg;
  int16_t nextAngleA;

  if (dualServoRunning == 0U || dualServoDir == 0)
  {
    return;
  }

  nowTick = HAL_GetTick();
  elapsedMs = nowTick - dualServoLastTick;
  if (elapsedMs < K_DUAL_SERVO_STEP_PERIOD_MS)
  {
    return;
  }
  dualServoLastTick = nowTick;

  deltaMdeg = (int32_t)dualServoDir * (int32_t)K_DUAL_SERVO_SPEED_DEG_PER_SEC * (int32_t)elapsedMs;
  nextMdeg = dualServoAngleA_mdeg + deltaMdeg;

  if (nextMdeg <= ((int32_t)K_DUAL_SERVO_CTRL_MIN_ANGLE * 1000))
  {
    nextMdeg = (int32_t)K_DUAL_SERVO_CTRL_MIN_ANGLE * 1000;
    dualServoRunning = 0;
    dualServoDir = 0;
  }
  else if (nextMdeg >= ((int32_t)K_DUAL_SERVO_CTRL_MAX_ANGLE * 1000))
  {
    nextMdeg = (int32_t)K_DUAL_SERVO_CTRL_MAX_ANGLE * 1000;
    dualServoRunning = 0;
    dualServoDir = 0;
  }

  dualServoAngleA_mdeg = nextMdeg;
  nextAngleA = (int16_t)(dualServoAngleA_mdeg / 1000);

  if (nextAngleA != dualServoLastAppliedA)
  {
    (void)DualServoApplyComplementAngleA(nextAngleA);
    dualServoLastAppliedA = nextAngleA;
  }
}

static void RunServoCalibrationTest(void)
{
  int16_t angle;

  /* Step 1: hold min/mid/max to quickly observe center and endpoints. */
  (void)ServoSetAngle180(SERVO_TEST_MIN_ANGLE);
  HAL_Delay(SERVO_TEST_HOLD_MS);

  (void)ServoSetAngle180(SERVO_TEST_MID_ANGLE);
  HAL_Delay(SERVO_TEST_HOLD_MS);

  (void)ServoSetAngle180(SERVO_TEST_MAX_ANGLE);
  HAL_Delay(SERVO_TEST_HOLD_MS);

  /* Step 2: sweep up/down so you can find the practical limits. */
  for (angle = SERVO_TEST_MIN_ANGLE; angle <= SERVO_TEST_MAX_ANGLE; angle += SERVO_TEST_SWEEP_STEP)
  {
    (void)ServoSetAngle180((uint16_t)angle);
    HAL_Delay(SERVO_TEST_STEP_DELAY_MS);
  }

  for (angle = SERVO_TEST_MAX_ANGLE; angle >= SERVO_TEST_MIN_ANGLE; angle -= SERVO_TEST_SWEEP_STEP)
  {
    (void)ServoSetAngle180((uint16_t)angle);
    HAL_Delay(SERVO_TEST_STEP_DELAY_MS);
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /*
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(1, 1, "Speed A:");
  */
  
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  WheelControl_Init(
    &wheelControl,
    LEFT_ENCODER_MAX_CPS,
    RIGHT_ENCODER_MAX_CPS,
    (uint16_t)htim3.Init.Period,
    JOY_TIMEOUT_MS,
    HAL_GetTick());
  
  HAL_StatusTypeDef pca_status = PCA9685_Init(&hi2c2, 0x40, 50);
  (void)pca_status;

#if SERVO_TEST_ENABLE
  while (1)
  {
    RunServoCalibrationTest();
  }
#endif

  SetSpeed_L(0);
  SetSpeed_R(0);
  SetSpeed_H(0);
  (void)DualServoApplyComplementAngleA(K_DUAL_SERVO_INIT_ANGLE);
  dualServoAngleA_mdeg = (int32_t)K_DUAL_SERVO_INIT_ANGLE * 1000;
  dualServoLastAppliedA = K_DUAL_SERVO_INIT_ANGLE;
  dualServoRunning = 0;
  dualServoDir = 0;
  dualServoLastTick = HAL_GetTick();
  


  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* process UART bytes outside ISR to reduce latency */
    ProcessUartBytes();
    int16_t cmdL = 0;
    int16_t cmdR = 0;
    WheelControl_Step(&wheelControl, &htim2, &htim4, HAL_GetTick(), &cmdL, &cmdR);
    SetSpeed_L(cmdL);
    SetSpeed_R(cmdR);
    DualServoSyncStep();
    SpeedA = cmdL;
    SpeedB = cmdR;

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
/* Parse joystick packet: format [j,Lx,Ly,Rx,Ry] where values are -100..100 */
static void ProcessJoystickPacket(char *buf)
{
  int Lx, Ly, Rx, Ry;
  if (buf[0] != '[' || buf[1] != 'j' || buf[2] != ',') return;
  char *end = strchr(buf, ']');
  if (!end) return;
  /* parse only the substring from start to end */
  char tmpBuf[48];
  size_t len = (size_t)(end - (buf + 3)); /* after header */
  if (len >= sizeof(tmpBuf)) return;
  memcpy(tmpBuf, buf + 3, len);
  tmpBuf[len] = '\0';
  if (sscanf(tmpBuf, "%d,%d,%d,%d", &Lx, &Ly, &Rx, &Ry) != 4) return;
  /* validate ranges */
  if (Lx < -100 || Lx > 100 || Ly < -100 || Ly > 100 || Rx < -100 || Rx > 100 || Ry < -100 || Ry > 100) return;

  /* valid packet received: update watchdog */
  WheelControl_MarkValidRx(&wheelControl, HAL_GetTick());

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
  const int deadzone = JOY_DEADZONE_PCT; /* joystick percentage */
  if (avgLy > -deadzone && avgLy < deadzone) avgLy = 0;
  if (avgRy > -deadzone && avgRy < deadzone) avgRy = 0;

  /* map -100..100 to target speed percentage for closed-loop controller */
  WheelControl_SetTargetsPercent(&wheelControl, avgLy, avgRy);
  /* OLED output disabled (hardware not installed) */
  /*
  if ((HAL_GetTick() - last_oled_update_tick) >= OLED_UPDATE_MS)
  {
    last_oled_update_tick = HAL_GetTick();
    char out[12];
    snprintf(out, sizeof(out), "%4d", lastAppliedA);
    OLED_ShowString(1, 8, out);
    snprintf(out, sizeof(out), "%4d", lastAppliedB);
    OLED_ShowString(2, 8, out);
  }
  */
}

/* Parse servo packet: format [s,1,x], where x is angle 0~180 for servo CH0. */
static void ProcessServoPacket(char *buf)
{
  int servoId;
  int angleVal;
  if (buf[0] != '[' || buf[1] != 's' || buf[2] != ',') return;
  char *end = strchr(buf, ']');
  if (!end) return;

  char tmpBuf[32];
  size_t len = (size_t)(end - (buf + 3));
  if (len >= sizeof(tmpBuf)) return;
  memcpy(tmpBuf, buf + 3, len);
  tmpBuf[len] = '\0';

  if (sscanf(tmpBuf, "%d,%d", &servoId, &angleVal) != 2) return;

  /* For now only packet [s,1,x] is accepted and mapped to PCA9685 CH0. */
  if (servoId != 1)
  {
    return;
  }

  /* second value controls CH0 angle: clamp to 0~180 for positional servo. */
  int16_t targetAngle = (angleVal < 0) ? 0 : (int16_t)angleVal;
  if (targetAngle > SERVO_TEST_LOGICAL_MAX_ANGLE)
  {
    targetAngle = SERVO_TEST_LOGICAL_MAX_ANGLE;
  }

  if (targetAngle != lastServoAngle)
  {
    (void)ServoSetAngle180((uint16_t)targetAngle);
    lastServoAngle = targetAngle;
  }
}

/* Parse dual-servo packet: format [k,x,y]
 * x = q: A angle up (+), B angle down (-)
 * x = a: A angle down (-), B angle up (+)
 * y = d: run continuously with fixed speed
 * y = u: stop at current angle
 */
static void ProcessDualServoPacket(char *buf)
{
  char dirCmdRaw;
  char stateCmdRaw;
  char dirCmd;
  char stateCmd;

  if (buf[0] != '[' || buf[1] != 'k' || buf[2] != ',') return;
  if (sscanf(buf, "[k,%c,%c]", &dirCmdRaw, &stateCmdRaw) != 2) return;

  dirCmd = (char)tolower((int)dirCmdRaw);
  stateCmd = (char)tolower((int)stateCmdRaw);

  if (stateCmd == 'u')
  {
    dualServoRunning = 0;
    dualServoDir = 0;
    dualServoLastTick = HAL_GetTick();
    return;
  }

  if (stateCmd != 'd')
  {
    return;
  }

  if (dirCmd == 'q')
  {
    dualServoDir = 1;
  }
  else if (dirCmd == 'a')
  {
    dualServoDir = -1;
  }
  else
  {
    return;
  }

  dualServoRunning = 1;
  dualServoLastTick = HAL_GetTick();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
    uint8_t byte = rx_data;

    if (byte == '[')
    {
      uart_in_frame = 1;
      uart_rx_idx = 0;
    }

    if (uart_in_frame)
    {
      if (uart_rx_idx < (uint8_t)(sizeof(uart_rx_buf) - 1))
      {
        uart_rx_buf[uart_rx_idx++] = (char)byte;
        uart_rx_buf[uart_rx_idx] = '\0';

        if (byte == ']')
        {
          strncpy((char *)uart_latest_frame, uart_rx_buf, sizeof(uart_latest_frame) - 1);
          uart_latest_frame[sizeof(uart_latest_frame) - 1] = '\0';
          uart_latest_ready = 1;
          uart_in_frame = 0;
          uart_rx_idx = 0;
          uart_rx_buf[0] = '\0';
        }
      }
      else
      {
        uart_in_frame = 0;
        uart_rx_idx = 0;
        uart_rx_buf[0] = '\0';
      }
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

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
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
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
#define JOY_Y_ACTIVE_THRESHOLD_PCT 75
#define JOY_X_ACTIVE_THRESHOLD_PCT 80
#define JOY_TIMEOUT_MS 500U
#define UART_FRAME_BUF_SIZE 64U

#define SERVO_ADDR_7BIT 0x40U
#define SERVO_PWM_HZ 50U
#define SERVO_MIN_PULSE_US 500U
#define SERVO_MAX_PULSE_US 2500U
#define SERVO_ANGLE_180_MAX 180U
#define SERVO_ANGLE_270_MAX 270U

#define SERVO_SLIDER_GIMBAL_ID 1
#define SERVO_SLIDER_SPEED_ID 2
#define SERVO_SLIDER_UPPER_ID 3
#define SERVO_SLIDER_MIDDLE_ID 4
#define SERVO_SLIDER_LOWER_ID 5

#define SERVO_CH_TRUNK 8U
#define SERVO_CH_GIMBAL 9U
#define SERVO_CH_LOWER 10U
#define SERVO_CH_MIDDLE 11U
#define SERVO_CH_UPPER 12U
#define SERVO_CH_FLOWER 14U
#define SERVO_CH_CLAMP 15U

#define FLOWER_MIN_ANGLE 0
#define FLOWER_MAX_ANGLE 90
#define FLOWER_INIT_ANGLE 45
#define FLOWER_SPEED_DEG_PER_SEC 20
#define FLOWER_STEP_PERIOD_MS 20U

#define CLAMP_CLOSE_ANGLE 100
#define CLAMP_OPEN_ANGLE 33
#define TRUNK_RETRACT_ANGLE 33
#define TRUNK_OPEN_ANGLE 120

#define GIMBAL_INIT_ANGLE 135U
#define UPPER_INIT_ANGLE 90U
#define MIDDLE_INIT_ANGLE 90U
#define LOWER_INIT_ANGLE 90U

/* Mecanum lateral speed ratio: strafe = forward/backward * 3/4. */
#define K_MECANUM_LATERAL_SCALE_NUM 3
#define K_MECANUM_LATERAL_SCALE_DEN 4
#define K_FB_LOW_SPEED_DIRECT_THRESHOLD_PCT 30

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static int16_t ClampSpeedPercent(int16_t speedPct)
{
  if (speedPct > 100)
  {
    return 100;
  }
  if (speedPct < -100)
  {
    return -100;
  }
  return speedPct;
}

static uint16_t PercentToPwmAbs(int16_t speedPct, uint32_t period)
{
  int32_t absPct = (speedPct >= 0) ? (int32_t)speedPct : -(int32_t)speedPct;
  if (period == 0U)
  {
    return 0U;
  }

  if (absPct > 100)
  {
    absPct = 100;
  }

  uint32_t duty = ((uint32_t)absPct * period) / 100U;
  if (duty > period)
  {
    duty = period;
  }

  return (uint16_t)duty;
}

static uint16_t PwmCompareForComplementary(uint16_t duty, uint32_t period)
{
  if (period == 0U)
  {
    return 0U;
  }
  if (duty > period)
  {
    duty = (uint16_t)period;
  }
  return (uint16_t)(period - duty);
}

static int16_t PwmCmdToPercent(int16_t pwmCmd, uint32_t period)
{
  if (period == 0U)
  {
    return 0;
  }

  int32_t pct = ((int32_t)pwmCmd * 100) / (int32_t)period;
  if (pct > 100)
  {
    pct = 100;
  }
  else if (pct < -100)
  {
    pct = -100;
  }
  return (int16_t)pct;
}

void SetSpeed_LA(int16_t Speed)
{
  int16_t speedPct = ClampSpeedPercent(Speed);
  uint16_t duty = PercentToPwmAbs(speedPct, htim3.Init.Period);

  if (speedPct > 0)
  {
    HAL_GPIO_WritePin(LAIN1_GPIO_Port, LAIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LAIN2_GPIO_Port, LAIN2_Pin, GPIO_PIN_RESET);
  }
  else if (speedPct < 0)
  {
    HAL_GPIO_WritePin(LAIN1_GPIO_Port, LAIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LAIN2_GPIO_Port, LAIN2_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(LAIN1_GPIO_Port, LAIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LAIN2_GPIO_Port, LAIN2_Pin, GPIO_PIN_RESET);
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)duty);
}

void SetSpeed_LB(int16_t Speed)
{
  int16_t speedPct = ClampSpeedPercent(Speed);
  uint16_t duty = PercentToPwmAbs(speedPct, htim1.Init.Period);
  uint16_t cmp = PwmCompareForComplementary(duty, htim1.Init.Period);

  if (speedPct > 0)
  {
    HAL_GPIO_WritePin(LBIN1_GPIO_Port, LBIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LBIN2_GPIO_Port, LBIN2_Pin, GPIO_PIN_RESET);
  }
  else if (speedPct < 0)
  {
    HAL_GPIO_WritePin(LBIN1_GPIO_Port, LBIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LBIN2_GPIO_Port, LBIN2_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(LBIN1_GPIO_Port, LBIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LBIN2_GPIO_Port, LBIN2_Pin, GPIO_PIN_RESET);
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)cmp);
}

void SetSpeed_RA(int16_t Speed)
{
  int16_t speedPct = ClampSpeedPercent(Speed);
  uint16_t duty = PercentToPwmAbs(speedPct, htim3.Init.Period);

  if (speedPct > 0)
  {
    HAL_GPIO_WritePin(RAIN1_GPIO_Port, RAIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RAIN2_GPIO_Port, RAIN2_Pin, GPIO_PIN_SET);
  }
  else if (speedPct < 0)
  {
    HAL_GPIO_WritePin(RAIN1_GPIO_Port, RAIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RAIN2_GPIO_Port, RAIN2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(RAIN1_GPIO_Port, RAIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RAIN2_GPIO_Port, RAIN2_Pin, GPIO_PIN_RESET);
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)duty);
}

void SetSpeed_RB(int16_t Speed)
{
  int16_t speedPct = ClampSpeedPercent(Speed);
  uint16_t duty = PercentToPwmAbs(speedPct, htim1.Init.Period);
  uint16_t cmp = PwmCompareForComplementary(duty, htim1.Init.Period);

  if (speedPct > 0)
  {
    HAL_GPIO_WritePin(RBIN1_GPIO_Port, RBIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RBIN2_GPIO_Port, RBIN2_Pin, GPIO_PIN_SET);
  }
  else if (speedPct < 0)
  {
    HAL_GPIO_WritePin(RBIN1_GPIO_Port, RBIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RBIN2_GPIO_Port, RBIN2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(RBIN1_GPIO_Port, RBIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RBIN2_GPIO_Port, RBIN2_Pin, GPIO_PIN_RESET);
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)cmp);
}

void SetSpeed_L(int16_t Speed)
{
  SetSpeed_LA(Speed);
  SetSpeed_LB(Speed);
}

void SetSpeed_R(int16_t Speed)
{
  SetSpeed_RA(Speed);
  SetSpeed_RB(Speed);
}

void SetSpeed_H(int16_t Speed)
{
    int16_t speedPct = ClampSpeedPercent(Speed);
    if (speedPct > 0)
    {
      /* H motor forward */
      HAL_GPIO_WritePin(HIN1_GPIO_Port, HIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(HIN2_GPIO_Port, HIN2_Pin, GPIO_PIN_RESET);
    }
    else if (speedPct < 0)
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
static char uart_rx_buf[UART_FRAME_BUF_SIZE];
static uint8_t uart_rx_idx = 0;
static uint8_t uart_in_frame = 0;
/* mailbox: always keep only latest completed frame */
static volatile char uart_latest_frame[UART_FRAME_BUF_SIZE];
static volatile uint8_t uart_latest_ready = 0;
static volatile uint8_t uart_latest_len = 0;
/* joystick filters/state */
#define JOY_FILTER_SIZE 1
static int16_t joyLxBuf[JOY_FILTER_SIZE];
static int16_t joyLyBuf[JOY_FILTER_SIZE];
static int16_t joyRxBuf[JOY_FILTER_SIZE];
static uint8_t joyFilterPos = 0;
static uint8_t joyFilterCount = 0;
static int16_t keyDriveSpeedPct = 50;
static WheelControl_t wheelControl = {0};
static int16_t gimbalLastApplied = -1;
static int16_t upperLastApplied = -1;
static int16_t middleLastApplied = -1;
static int16_t lowerLastApplied = -1;
static int16_t clampLastApplied = -1;
static int16_t trunkLastApplied = -1;
static int32_t flowerAngle_mdeg = 0;
static int16_t flowerLastApplied = -1;
static int8_t flowerDir = 0;
static uint8_t flowerRunning = 0;
static uint32_t flowerLastTick = 0;
/* -1: left strafe, 0: stop/normal mode, +1: right strafe */
static volatile int8_t mecanumLateralCmd = 0;
/* Direct-drive bypass for modes that should not pass through S-curve. */
static int16_t directDriveLeftPct = 0;
static int16_t directDriveRightPct = 0;
static uint8_t directDriveEnabled = 0;
// static uint32_t last_oled_update_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ProcessJoystickPacket(char *buf);
static void ProcessServoPacket(char *buf);
static void ProcessDualServoPacket(char *buf);
static void ProcessUartBytes(void);
static void ProcessLatestUartFrame(void);
static uint8_t ParseSignedInt16Fast(const char **cursor, int16_t *outValue);
static char AsciiToLowerFast(char ch);
static void StartUartReceive(void);
static HAL_StatusTypeDef ServoSetAngle180ByChannel(uint8_t channel, uint16_t angleDeg);
static HAL_StatusTypeDef ServoSetAngle270ByChannel(uint8_t channel, uint16_t angleDeg);
static HAL_StatusTypeDef ServoSetPulseUsByChannel(uint8_t channel, uint16_t pulseUs);
static void FlowerHandSyncStep(void);
static void ApplyMecanumLateralCommand(int8_t lateralCmd, int16_t fallbackLeftPct, int16_t fallbackRightPct);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static char AsciiToLowerFast(char ch)
{
  if (ch >= 'A' && ch <= 'Z')
  {
    return (char)(ch + ('a' - 'A'));
  }
  return ch;
}

static uint8_t ParseSignedInt16Fast(const char **cursor, int16_t *outValue)
{
  const char *p;
  int32_t sign = 1;
  int32_t value = 0;
  uint8_t hasDigit = 0U;

  if (cursor == NULL || *cursor == NULL || outValue == NULL)
  {
    return 0U;
  }

  p = *cursor;

  if (*p == '+')
  {
    p++;
  }
  else if (*p == '-')
  {
    sign = -1;
    p++;
  }

  while (*p >= '0' && *p <= '9')
  {
    hasDigit = 1U;
    value = (value * 10) + (int32_t)(*p - '0');
    if (value > 32768)
    {
      return 0U;
    }
    p++;
  }

  if (hasDigit == 0U)
  {
    return 0U;
  }

  value *= sign;
  if (value < -32768 || value > 32767)
  {
    return 0U;
  }

  *outValue = (int16_t)value;
  *cursor = p;
  return 1U;
}

static void StartUartReceive(void)
{
  (void)HAL_UART_Receive_IT(&huart2, &rx_data, 1);
}

static void ProcessUartBytes(void)
{
  ProcessLatestUartFrame();
}

static void ApplyMecanumLateralCommand(int8_t lateralCmd, int16_t fallbackLeftPct, int16_t fallbackRightPct)
{
  int16_t lateralPct = (int16_t)((((int32_t)keyDriveSpeedPct * K_MECANUM_LATERAL_SCALE_NUM) + (K_MECANUM_LATERAL_SCALE_DEN / 2)) / K_MECANUM_LATERAL_SCALE_DEN);
  lateralPct = ClampSpeedPercent(lateralPct);

  if (lateralCmd < 0)
  {
    /* Left strafe: FL-, RL+, FR+, RR- */
    SetSpeed_LA((int16_t)(-lateralPct));
    SetSpeed_LB(lateralPct);
    SetSpeed_RA(lateralPct);
    SetSpeed_RB((int16_t)(-lateralPct));
    return;
  }

  if (lateralCmd > 0)
  {
    /* Right strafe: FL+, RL-, FR-, RR+ */
    SetSpeed_LA(lateralPct);
    SetSpeed_LB((int16_t)(-lateralPct));
    SetSpeed_RA((int16_t)(-lateralPct));
    SetSpeed_RB(lateralPct);
    return;
  }

  SetSpeed_L(fallbackLeftPct);
  SetSpeed_R(fallbackRightPct);
}

static void ProcessLatestUartFrame(void)
{
  char frame[UART_FRAME_BUF_SIZE];
  uint8_t frameLen;

  if (!uart_latest_ready)
  {
    return;
  }

  __disable_irq();
  frameLen = uart_latest_len;
  if (frameLen >= (uint8_t)sizeof(frame))
  {
    frameLen = (uint8_t)(sizeof(frame) - 1U);
  }
  memcpy(frame, (const void *)uart_latest_frame, frameLen);
  frame[frameLen] = '\0';
  uart_latest_ready = 0;
  __enable_irq();

  if (frameLen < 4U || frame[0] != '[' || frame[1] == '\0')
  {
    return;
  }

  if (AsciiToLowerFast(frame[1]) == 'j')
  {
    ProcessJoystickPacket(frame);
  }
  else if (AsciiToLowerFast(frame[1]) == 's')
  {
    ProcessServoPacket(frame);
  }
  else if (AsciiToLowerFast(frame[1]) == 'k')
  {
    ProcessDualServoPacket(frame);
  }
}

static HAL_StatusTypeDef ServoSetPulseUsByChannel(uint8_t channel, uint16_t pulseUs)
{
  if (pulseUs < SERVO_MIN_PULSE_US)
  {
    pulseUs = SERVO_MIN_PULSE_US;
  }
  else if (pulseUs > SERVO_MAX_PULSE_US)
  {
    pulseUs = SERVO_MAX_PULSE_US;
  }

  return PCA9685_SetServoPulseUs(&hi2c2, SERVO_ADDR_7BIT, channel, pulseUs, SERVO_PWM_HZ);
}

static HAL_StatusTypeDef ServoSetAngle180ByChannel(uint8_t channel, uint16_t angleDeg)
{
  uint32_t pulseUs;

  if (angleDeg > SERVO_ANGLE_180_MAX)
  {
    angleDeg = SERVO_ANGLE_180_MAX;
  }

  pulseUs = SERVO_MIN_PULSE_US +
            (((uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angleDeg) / SERVO_ANGLE_180_MAX);

  return ServoSetPulseUsByChannel(channel, (uint16_t)pulseUs);
}

static HAL_StatusTypeDef ServoSetAngle270ByChannel(uint8_t channel, uint16_t angleDeg)
{
  uint32_t pulseUs;

  if (angleDeg > SERVO_ANGLE_270_MAX)
  {
    angleDeg = SERVO_ANGLE_270_MAX;
  }

  pulseUs = SERVO_MIN_PULSE_US +
            (((uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angleDeg) /
             SERVO_ANGLE_270_MAX);

  return ServoSetPulseUsByChannel(channel, (uint16_t)pulseUs);
}

static void FlowerHandSyncStep(void)
{
  uint32_t nowTick;
  uint32_t elapsedMs;
  int32_t deltaMdeg;
  int32_t nextMdeg;
  int16_t nextAngle;

  if (flowerRunning == 0U || flowerDir == 0)
  {
    return;
  }

  nowTick = HAL_GetTick();
  elapsedMs = nowTick - flowerLastTick;
  if (elapsedMs < FLOWER_STEP_PERIOD_MS)
  {
    return;
  }
  flowerLastTick = nowTick;

  deltaMdeg = (int32_t)flowerDir * (int32_t)FLOWER_SPEED_DEG_PER_SEC * (int32_t)elapsedMs;
  nextMdeg = flowerAngle_mdeg + deltaMdeg;

  if (nextMdeg <= ((int32_t)FLOWER_MIN_ANGLE * 1000))
  {
    nextMdeg = (int32_t)FLOWER_MIN_ANGLE * 1000;
    flowerRunning = 0;
    flowerDir = 0;
  }
  else if (nextMdeg >= ((int32_t)FLOWER_MAX_ANGLE * 1000))
  {
    nextMdeg = (int32_t)FLOWER_MAX_ANGLE * 1000;
    flowerRunning = 0;
    flowerDir = 0;
  }

  flowerAngle_mdeg = nextMdeg;
  nextAngle = (int16_t)(flowerAngle_mdeg / 1000);

  if (nextAngle != flowerLastApplied)
  {
    (void)ServoSetAngle180ByChannel(SERVO_CH_FLOWER, (uint16_t)nextAngle);
    flowerLastApplied = nextAngle;
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
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
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
  
  HAL_StatusTypeDef pca_status = PCA9685_Init(&hi2c2, SERVO_ADDR_7BIT, SERVO_PWM_HZ);
  (void)pca_status;

  SetSpeed_L(0);
  SetSpeed_R(0);
  SetSpeed_H(0);

  (void)ServoSetAngle270ByChannel(SERVO_CH_GIMBAL, GIMBAL_INIT_ANGLE);
  gimbalLastApplied = (int16_t)GIMBAL_INIT_ANGLE;
  (void)ServoSetAngle180ByChannel(SERVO_CH_UPPER, UPPER_INIT_ANGLE);
  upperLastApplied = (int16_t)UPPER_INIT_ANGLE;
  (void)ServoSetAngle180ByChannel(SERVO_CH_MIDDLE, MIDDLE_INIT_ANGLE);
  middleLastApplied = (int16_t)MIDDLE_INIT_ANGLE;
  (void)ServoSetAngle180ByChannel(SERVO_CH_LOWER, LOWER_INIT_ANGLE);
  lowerLastApplied = (int16_t)LOWER_INIT_ANGLE;

  flowerAngle_mdeg = (int32_t)FLOWER_INIT_ANGLE * 1000;
  flowerLastApplied = FLOWER_INIT_ANGLE;
  flowerDir = 0;
  flowerRunning = 0;
  flowerLastTick = HAL_GetTick();
  (void)ServoSetAngle180ByChannel(SERVO_CH_FLOWER, (uint16_t)FLOWER_INIT_ANGLE);

  (void)ServoSetAngle180ByChannel(SERVO_CH_CLAMP, (uint16_t)CLAMP_OPEN_ANGLE);
  clampLastApplied = CLAMP_OPEN_ANGLE;
  (void)ServoSetAngle180ByChannel(SERVO_CH_TRUNK, (uint16_t)TRUNK_RETRACT_ANGLE);
  trunkLastApplied = TRUNK_RETRACT_ANGLE;

  mecanumLateralCmd = 0;
  StartUartReceive();
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ProcessUartBytes();
    uint32_t nowTick = HAL_GetTick();
    int16_t cmdL = 0;
    int16_t cmdR = 0;
    WheelControl_Step(&wheelControl, &htim2, &htim4, nowTick, &cmdL, &cmdR);
    int16_t cmdLPct = PwmCmdToPercent(cmdL, htim3.Init.Period);
    int16_t cmdRPct = PwmCmdToPercent(cmdR, htim3.Init.Period);
    int16_t applyLeftPct = cmdLPct;
    int16_t applyRightPct = cmdRPct;

    if ((nowTick - wheelControl.last_valid_rx_tick) > JOY_TIMEOUT_MS)
    {
      mecanumLateralCmd = 0;
      directDriveEnabled = 0U;
      directDriveLeftPct = 0;
      directDriveRightPct = 0;
      applyLeftPct = 0;
      applyRightPct = 0;
      WheelControl_SetTargetsPercent(&wheelControl, 0, 0);
    }
    else if (directDriveEnabled != 0U)
    {
      applyLeftPct = directDriveLeftPct;
      applyRightPct = directDriveRightPct;
    }

    ApplyMecanumLateralCommand(mecanumLateralCmd, applyLeftPct, applyRightPct);
    FlowerHandSyncStep();
    SpeedA = applyLeftPct;
    SpeedB = applyRightPct;

    __WFI();
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
/* Parse joystick packet: format [j,Lx,Ly,Rx,Ry].
 * Ly: forward/backward constant speed trigger by threshold.
 * Lx: rotate (left=CCW, right=CW) by threshold.
 * Rx: mecanum strafe (left/right) by threshold.
 * Ry: reserved for future use.
 */
static void ProcessJoystickPacket(char *buf)
{
  const char *p = buf;
  int16_t Lx;
  int16_t Ly;
  int16_t Rx;
  int16_t Ry;

  if (p[0] != '[' || AsciiToLowerFast(p[1]) != 'j' || p[2] != ',') return;

  p += 3;
  if (ParseSignedInt16Fast(&p, &Lx) == 0U || *p != ',') return;
  p++;
  if (ParseSignedInt16Fast(&p, &Ly) == 0U || *p != ',') return;
  p++;
  if (ParseSignedInt16Fast(&p, &Rx) == 0U || *p != ',') return;
  p++;
  if (ParseSignedInt16Fast(&p, &Ry) == 0U || *p != ']' || p[1] != '\0') return;

  /* validate ranges */
  if (Lx < -100 || Lx > 100 || Ly < -100 || Ly > 100 || Rx < -100 || Rx > 100 || Ry < -100 || Ry > 100) return;

  /* valid packet received: update watchdog */
  WheelControl_MarkValidRx(&wheelControl, HAL_GetTick());

  /* push into moving average buffers */
  joyLxBuf[joyFilterPos] = (int16_t)Lx;
  joyLyBuf[joyFilterPos] = (int16_t)Ly;
  joyRxBuf[joyFilterPos] = (int16_t)Rx;
  joyFilterPos = (joyFilterPos + 1) % JOY_FILTER_SIZE;
  if (joyFilterCount < JOY_FILTER_SIZE) joyFilterCount++;

  int sumLx = 0;
  int sumLy = 0;
  int sumRx = 0;
  for (uint8_t i = 0; i < joyFilterCount; i++)
  {
    sumLx += joyLxBuf[i];
    sumLy += joyLyBuf[i];
    sumRx += joyRxBuf[i];
  }
  int16_t avgLx = (int16_t)(sumLx / (int)joyFilterCount);
  int16_t avgLy = (int16_t)(sumLy / (int)joyFilterCount);
  int16_t avgRx = (int16_t)(sumRx / (int)joyFilterCount);

  int16_t leftTargetPct = 0;
  int16_t rightTargetPct = 0;
  uint8_t forwardBackActive = 0U;
  uint8_t rotateActive = 0U;

  if (abs((int)avgLy) >= JOY_Y_ACTIVE_THRESHOLD_PCT)
  {
    int16_t fbPct = (avgLy > 0) ? keyDriveSpeedPct : (int16_t)(-keyDriveSpeedPct);
    leftTargetPct = fbPct;
    rightTargetPct = fbPct;
    forwardBackActive = 1U;
  }

  if (avgLx <= -JOY_X_ACTIVE_THRESHOLD_PCT)
  {
    leftTargetPct = (int16_t)(-keyDriveSpeedPct);
    rightTargetPct = keyDriveSpeedPct;
    rotateActive = 1U;
  }
  else if (avgLx >= JOY_X_ACTIVE_THRESHOLD_PCT)
  {
    leftTargetPct = keyDriveSpeedPct;
    rightTargetPct = (int16_t)(-keyDriveSpeedPct);
    rotateActive = 1U;
  }

  if (avgRx <= -JOY_X_ACTIVE_THRESHOLD_PCT)
  {
    mecanumLateralCmd = -1;
  }
  else if (avgRx >= JOY_X_ACTIVE_THRESHOLD_PCT)
  {
    mecanumLateralCmd = 1;
  }
  else
  {
    mecanumLateralCmd = 0;
  }

  if (mecanumLateralCmd != 0)
  {
    directDriveEnabled = 0U;
    WheelControl_SetTargetsPercent(&wheelControl, 0, 0);
    return;
  }

  if (rotateActive)
  {
    directDriveEnabled = 1U;
    directDriveLeftPct = leftTargetPct;
    directDriveRightPct = rightTargetPct;
    WheelControl_SetTargetsPercent(&wheelControl, 0, 0);
    return;
  }

  if (forwardBackActive && keyDriveSpeedPct < K_FB_LOW_SPEED_DIRECT_THRESHOLD_PCT)
  {
    directDriveEnabled = 1U;
    directDriveLeftPct = leftTargetPct;
    directDriveRightPct = rightTargetPct;
    WheelControl_SetTargetsPercent(&wheelControl, 0, 0);
    return;
  }

  directDriveEnabled = 0U;
  /* Forward/backward in normal speed range still uses S-curve tracking. */
  WheelControl_SetTargetsPercent(&wheelControl, leftTargetPct, rightTargetPct);
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

/* Parse [s,id,x]:
 * id=1 -> gimbal (CH9, 0~270), id=2 -> speed percent, id=3 -> upper(CH12),
 * id=4 -> middle(CH11), id=5 -> lower(CH10).
 */
static void ProcessServoPacket(char *buf)
{
  const char *p = buf;
  int16_t servoIdVal;
  int16_t angleVal;
  int servoId;
  int16_t targetAngle;

  if (p[0] != '[' || AsciiToLowerFast(p[1]) != 's' || p[2] != ',') return;

  p += 3;
  if (ParseSignedInt16Fast(&p, &servoIdVal) == 0U || *p != ',') return;
  p++;
  if (ParseSignedInt16Fast(&p, &angleVal) == 0U || *p != ']' || p[1] != '\0') return;

  servoId = (int)servoIdVal;

  if (servoId == SERVO_SLIDER_SPEED_ID)
  {
    int32_t speedPct = (int32_t)angleVal;
    if (speedPct < 0)
    {
      speedPct = -speedPct;
    }
    if (speedPct > 100)
    {
      speedPct = 100;
    }
    keyDriveSpeedPct = (int16_t)speedPct;
    return;
  }

  targetAngle = (angleVal < 0) ? 0 : (int16_t)angleVal;

  if (servoId == SERVO_SLIDER_GIMBAL_ID)
  {
    if (targetAngle > (int16_t)SERVO_ANGLE_270_MAX)
    {
      targetAngle = (int16_t)SERVO_ANGLE_270_MAX;
    }

    if (targetAngle != gimbalLastApplied)
    {
      (void)ServoSetAngle270ByChannel(SERVO_CH_GIMBAL, (uint16_t)targetAngle);
      gimbalLastApplied = targetAngle;
    }
    return;
  }

  if (targetAngle > (int16_t)SERVO_ANGLE_180_MAX)
  {
    targetAngle = (int16_t)SERVO_ANGLE_180_MAX;
  }

  if (servoId == SERVO_SLIDER_UPPER_ID)
  {
    if (targetAngle != upperLastApplied)
    {
      (void)ServoSetAngle180ByChannel(SERVO_CH_UPPER, (uint16_t)targetAngle);
      upperLastApplied = targetAngle;
    }
    return;
  }

  if (servoId == SERVO_SLIDER_MIDDLE_ID)
  {
    if (targetAngle != middleLastApplied)
    {
      (void)ServoSetAngle180ByChannel(SERVO_CH_MIDDLE, (uint16_t)targetAngle);
      middleLastApplied = targetAngle;
    }
    return;
  }

  if (servoId == SERVO_SLIDER_LOWER_ID)
  {
    if (targetAngle != lowerLastApplied)
    {
      (void)ServoSetAngle180ByChannel(SERVO_CH_LOWER, (uint16_t)targetAngle);
      lowerLastApplied = targetAngle;
    }
  }
}

/* Parse shared packet: format [k,x,y]
 * q/e: flower hand angle increase/decrease (continuous while pressed)
 * u/d: clamp close/open (dual state)
 * i/o: trunk retract/open (dual state)
 * y: d/u, d = press, u = release
 */
static void ProcessDualServoPacket(char *buf)
{
  char moveCmd;
  char stateCmd;

  if (buf[0] != '[' || AsciiToLowerFast(buf[1]) != 'k' || buf[2] != ',') return;
  if (buf[4] != ',' || buf[6] != ']' || buf[7] != '\0') return;

  moveCmd = AsciiToLowerFast(buf[3]);
  stateCmd = AsciiToLowerFast(buf[5]);

  if (stateCmd == 'u')
  {
    if (moveCmd == 'q' || moveCmd == 'e')
    {
      flowerRunning = 0;
      flowerDir = 0;
    }

    return;
  }

  if (stateCmd != 'd')
  {
    return;
  }

  if (moveCmd == 'u')
  {
    if (clampLastApplied != CLAMP_CLOSE_ANGLE)
    {
      (void)ServoSetAngle180ByChannel(SERVO_CH_CLAMP, (uint16_t)CLAMP_CLOSE_ANGLE);
      clampLastApplied = CLAMP_CLOSE_ANGLE;
    }
    return;
  }

  if (moveCmd == 'd')
  {
    if (clampLastApplied != CLAMP_OPEN_ANGLE)
    {
      (void)ServoSetAngle180ByChannel(SERVO_CH_CLAMP, (uint16_t)CLAMP_OPEN_ANGLE);
      clampLastApplied = CLAMP_OPEN_ANGLE;
    }
    return;
  }

  if (moveCmd == 'i')
  {
    if (trunkLastApplied != TRUNK_RETRACT_ANGLE)
    {
      (void)ServoSetAngle180ByChannel(SERVO_CH_TRUNK, (uint16_t)TRUNK_RETRACT_ANGLE);
      trunkLastApplied = TRUNK_RETRACT_ANGLE;
    }
    return;
  }

  if (moveCmd == 'o')
  {
    if (trunkLastApplied != TRUNK_OPEN_ANGLE)
    {
      (void)ServoSetAngle180ByChannel(SERVO_CH_TRUNK, (uint16_t)TRUNK_OPEN_ANGLE);
      trunkLastApplied = TRUNK_OPEN_ANGLE;
    }
    return;
  }

  if (moveCmd == 'q')
  {
    flowerDir = 1;
    flowerRunning = 1;
    flowerLastTick = HAL_GetTick();
    return;
  }

  if (moveCmd == 'e')
  {
    flowerDir = -1;
    flowerRunning = 1;
    flowerLastTick = HAL_GetTick();
    return;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
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
          uint8_t frameLen = uart_rx_idx;
          if (frameLen >= (uint8_t)sizeof(uart_latest_frame))
          {
            frameLen = (uint8_t)(sizeof(uart_latest_frame) - 1U);
          }
          memcpy((void *)uart_latest_frame, uart_rx_buf, frameLen);
          uart_latest_frame[frameLen] = '\0';
          uart_latest_len = frameLen;
          uart_latest_ready = 1U;
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
    StartUartReceive();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart_in_frame = 0U;
    uart_rx_idx = 0U;
    uart_rx_buf[0] = '\0';
    StartUartReceive();
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

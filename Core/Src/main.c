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
typedef struct
{
  float filteredInputRaw;
  uint16_t targetInputRaw;
  uint8_t initialized;
} ServoSliderLpfState_t;

typedef struct
{
  uint16_t stateId;
  uint16_t gimbalAngleDeg;
  uint16_t upperAngleDeg;
  uint16_t middleAngleDeg;
  uint16_t lowerAngleDeg;
  uint16_t flowerAngleDeg;
  uint16_t clampAngleDeg;
  uint16_t trunkAngleDeg;
} ServoStatePreset_t;

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
/* Slider input range from Bluetooth packets [s,id,x]/[f,id,x].
 * Default uses angle-like range 0~180; if app sends ADC raw set MAX to 4095.
 */
#define SERVO_SLIDER_INPUT_MIN 0U
#define SERVO_SLIDER_INPUT_MAX 180U
/* 0~1: smaller alpha gives smoother but slower response. */
#define SERVO_SLIDER_LPF_ALPHA 0.18f

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

#define FLOWER_MIN_ANGLE 90
#define FLOWER_MAX_ANGLE 180
#define FLOWER_INIT_ANGLE 180
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

#define SERVO_STATE_KEEP_ANGLE 0xFFFFU

/* Bluetooth state packet: (x), x is state id. */
/* Add more states with SERVO_STATE_n_* macros, then append into servoStatePresets[]. */
#define SERVO_STATE_1_ID 1U
#define SERVO_STATE_1_GIMBAL_ANGLE SERVO_STATE_KEEP_ANGLE
#define SERVO_STATE_1_UPPER_ANGLE 132U
#define SERVO_STATE_1_MIDDLE_ANGLE 14U
#define SERVO_STATE_1_LOWER_ANGLE 90U
#define SERVO_STATE_1_FLOWER_ANGLE SERVO_STATE_KEEP_ANGLE
#define SERVO_STATE_1_CLAMP_ANGLE SERVO_STATE_KEEP_ANGLE
#define SERVO_STATE_1_TRUNK_ANGLE SERVO_STATE_KEEP_ANGLE

/* State transition smoothing: servo angle changes use limited speed instead of jump. */
#define SERVO_STATE_TRANSITION_SPEED_DPS 45U
#define SERVO_STATE_TRANSITION_PERIOD_MS 20U
#define SERVO_STATE_TARGET_INVALID (-1)

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
static uint8_t uart_frame_end = 0;
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
static ServoSliderLpfState_t upperSliderLpf = {0};
static ServoSliderLpfState_t middleSliderLpf = {0};
static ServoSliderLpfState_t lowerSliderLpf = {0};
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
static int16_t stateTargetGimbal = SERVO_STATE_TARGET_INVALID;
static int16_t stateTargetUpper = SERVO_STATE_TARGET_INVALID;
static int16_t stateTargetMiddle = SERVO_STATE_TARGET_INVALID;
static int16_t stateTargetLower = SERVO_STATE_TARGET_INVALID;
static int16_t stateTargetFlower = SERVO_STATE_TARGET_INVALID;
static int16_t stateTargetClamp = SERVO_STATE_TARGET_INVALID;
static int16_t stateTargetTrunk = SERVO_STATE_TARGET_INVALID;
static uint32_t stateTransitionLastTick = 0;

static const ServoStatePreset_t servoStatePresets[] =
{
  {
    SERVO_STATE_1_ID,
    SERVO_STATE_1_GIMBAL_ANGLE,
    SERVO_STATE_1_UPPER_ANGLE,
    SERVO_STATE_1_MIDDLE_ANGLE,
    SERVO_STATE_1_LOWER_ANGLE,
    SERVO_STATE_1_FLOWER_ANGLE,
    SERVO_STATE_1_CLAMP_ANGLE,
    SERVO_STATE_1_TRUNK_ANGLE
  }
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ProcessJoystickPacket(char *buf);
static void ProcessServoPacket(char *buf);
static void ProcessServoFinalPacket(char *buf);
static void ProcessDualServoPacket(char *buf);
static void ProcessStatePacket(const char *buf);
static void ProcessUartBytes(void);
static void ProcessLatestUartFrame(void);
static uint8_t ParseSignedInt16Fast(const char **cursor, int16_t *outValue);
static char AsciiToLowerFast(char ch);
static void StartUartReceive(void);
static HAL_StatusTypeDef ServoSetAngle180ByChannel(uint8_t channel, uint16_t angleDeg);
static HAL_StatusTypeDef ServoSetAngle270ByChannel(uint8_t channel, uint16_t angleDeg);
static HAL_StatusTypeDef ServoSetPulseUsByChannel(uint8_t channel, uint16_t pulseUs);
static uint16_t ClampServoSliderInputRaw(int16_t inputRaw);
static uint16_t ServoSliderLowPassUpdate(ServoSliderLpfState_t *state);
static uint16_t ServoSliderInputToAngle180(uint16_t inputRaw);
static uint16_t ServoSliderAngle180ToInput(uint16_t angleDeg);
static void ServoSliderSetTargetInput(ServoSliderLpfState_t *state,
                                      int16_t inputRaw,
                                      int16_t currentAngleDeg);
static void ServoSliderApplySmoothedAngle180(uint8_t channel,
                                             ServoSliderLpfState_t *state,
                                             int16_t *lastAppliedAngle);
static void UpdateServo180ByFilteredSliderInput(uint8_t channel,
                                              int16_t inputRaw,
                                              ServoSliderLpfState_t *filterState,
                                              int16_t *lastAppliedAngle);
static void UpdateServo180ByFinalSliderInput(uint8_t channel,
                                           int16_t inputRaw,
                                           ServoSliderLpfState_t *filterState,
                                           int16_t *lastAppliedAngle);
static void FlowerHandSyncStep(void);
static void ApplyMecanumLateralCommand(int8_t lateralCmd, int16_t fallbackLeftPct, int16_t fallbackRightPct);
static const ServoStatePreset_t *FindServoStatePreset(uint16_t stateId);
static void ApplyStateServoAngle180(uint8_t channel,
                                    uint16_t targetAngle,
                                    int16_t *lastAppliedAngle,
                                    ServoSliderLpfState_t *filterState);
static void ApplyStateServoAngle270(uint8_t channel,
                                    uint16_t targetAngle,
                                    int16_t *lastAppliedAngle);
static void ApplyServoStatePreset(const ServoStatePreset_t *preset);
static int16_t MoveAngleTowardTarget(int16_t currentAngle,
                                     int16_t targetAngle,
                                     uint16_t maxStepDeg);
static void ServoStateTransitionStep(void);

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

static const ServoStatePreset_t *FindServoStatePreset(uint16_t stateId)
{
  uint32_t i;

  for (i = 0U; i < (uint32_t)(sizeof(servoStatePresets) / sizeof(servoStatePresets[0])); i++)
  {
    if (servoStatePresets[i].stateId == stateId)
    {
      return &servoStatePresets[i];
    }
  }

  return NULL;
}

static void ApplyStateServoAngle180(uint8_t channel,
                                    uint16_t targetAngle,
                                    int16_t *lastAppliedAngle,
                                    ServoSliderLpfState_t *filterState)
{
  uint16_t angleDeg;
  uint16_t inputRaw;

  if (targetAngle == SERVO_STATE_KEEP_ANGLE || lastAppliedAngle == NULL)
  {
    return;
  }

  angleDeg = targetAngle;
  if (angleDeg > SERVO_ANGLE_180_MAX)
  {
    angleDeg = SERVO_ANGLE_180_MAX;
  }

  if ((int16_t)angleDeg != *lastAppliedAngle)
  {
    (void)ServoSetAngle180ByChannel(channel, angleDeg);
    *lastAppliedAngle = (int16_t)angleDeg;
  }

  if (filterState != NULL)
  {
    inputRaw = ServoSliderAngle180ToInput(angleDeg);
    filterState->targetInputRaw = inputRaw;
    filterState->filteredInputRaw = (float)inputRaw;
    filterState->initialized = 1U;
  }
}

static void ApplyStateServoAngle270(uint8_t channel,
                                    uint16_t targetAngle,
                                    int16_t *lastAppliedAngle)
{
  uint16_t angleDeg;

  if (targetAngle == SERVO_STATE_KEEP_ANGLE || lastAppliedAngle == NULL)
  {
    return;
  }

  angleDeg = targetAngle;
  if (angleDeg > SERVO_ANGLE_270_MAX)
  {
    angleDeg = SERVO_ANGLE_270_MAX;
  }

  if ((int16_t)angleDeg != *lastAppliedAngle)
  {
    (void)ServoSetAngle270ByChannel(channel, angleDeg);
    *lastAppliedAngle = (int16_t)angleDeg;
  }
}

static int16_t MoveAngleTowardTarget(int16_t currentAngle,
                                     int16_t targetAngle,
                                     uint16_t maxStepDeg)
{
  if (maxStepDeg == 0U || currentAngle == targetAngle)
  {
    return currentAngle;
  }

  if (currentAngle < targetAngle)
  {
    int32_t nextAngle = (int32_t)currentAngle + (int32_t)maxStepDeg;
    if (nextAngle > (int32_t)targetAngle)
    {
      nextAngle = (int32_t)targetAngle;
    }
    return (int16_t)nextAngle;
  }

  {
    int32_t nextAngle = (int32_t)currentAngle - (int32_t)maxStepDeg;
    if (nextAngle < (int32_t)targetAngle)
    {
      nextAngle = (int32_t)targetAngle;
    }
    return (int16_t)nextAngle;
  }
}

static void ServoStateTransitionStep(void)
{
  uint32_t nowTick;
  uint32_t elapsedMs;
  uint32_t stepDeg;
  int16_t nextAngle;

  nowTick = HAL_GetTick();
  elapsedMs = nowTick - stateTransitionLastTick;
  if (elapsedMs < SERVO_STATE_TRANSITION_PERIOD_MS)
  {
    return;
  }
  stateTransitionLastTick = nowTick;

  stepDeg = ((uint32_t)SERVO_STATE_TRANSITION_SPEED_DPS * elapsedMs + 999U) / 1000U;
  if (stepDeg == 0U)
  {
    stepDeg = 1U;
  }

  if (stateTargetGimbal >= 0 && gimbalLastApplied >= 0)
  {
    nextAngle = MoveAngleTowardTarget(gimbalLastApplied, stateTargetGimbal, (uint16_t)stepDeg);
    ApplyStateServoAngle270(SERVO_CH_GIMBAL, (uint16_t)nextAngle, &gimbalLastApplied);
    if (nextAngle == stateTargetGimbal)
    {
      stateTargetGimbal = SERVO_STATE_TARGET_INVALID;
    }
  }

  if (stateTargetUpper >= 0 && upperLastApplied >= 0)
  {
    nextAngle = MoveAngleTowardTarget(upperLastApplied, stateTargetUpper, (uint16_t)stepDeg);
    ApplyStateServoAngle180(SERVO_CH_UPPER, (uint16_t)nextAngle, &upperLastApplied, &upperSliderLpf);
    if (nextAngle == stateTargetUpper)
    {
      stateTargetUpper = SERVO_STATE_TARGET_INVALID;
    }
  }

  if (stateTargetMiddle >= 0 && middleLastApplied >= 0)
  {
    nextAngle = MoveAngleTowardTarget(middleLastApplied, stateTargetMiddle, (uint16_t)stepDeg);
    ApplyStateServoAngle180(SERVO_CH_MIDDLE, (uint16_t)nextAngle, &middleLastApplied, &middleSliderLpf);
    if (nextAngle == stateTargetMiddle)
    {
      stateTargetMiddle = SERVO_STATE_TARGET_INVALID;
    }
  }

  if (stateTargetLower >= 0 && lowerLastApplied >= 0)
  {
    nextAngle = MoveAngleTowardTarget(lowerLastApplied, stateTargetLower, (uint16_t)stepDeg);
    ApplyStateServoAngle180(SERVO_CH_LOWER, (uint16_t)nextAngle, &lowerLastApplied, &lowerSliderLpf);
    if (nextAngle == stateTargetLower)
    {
      stateTargetLower = SERVO_STATE_TARGET_INVALID;
    }
  }

  if (stateTargetFlower >= 0 && flowerLastApplied >= 0)
  {
    nextAngle = MoveAngleTowardTarget(flowerLastApplied, stateTargetFlower, (uint16_t)stepDeg);
    ApplyStateServoAngle180(SERVO_CH_FLOWER, (uint16_t)nextAngle, &flowerLastApplied, NULL);
    flowerAngle_mdeg = (int32_t)flowerLastApplied * 1000;
    if (nextAngle == stateTargetFlower)
    {
      stateTargetFlower = SERVO_STATE_TARGET_INVALID;
    }
  }

  if (stateTargetClamp >= 0 && clampLastApplied >= 0)
  {
    nextAngle = MoveAngleTowardTarget(clampLastApplied, stateTargetClamp, (uint16_t)stepDeg);
    ApplyStateServoAngle180(SERVO_CH_CLAMP, (uint16_t)nextAngle, &clampLastApplied, NULL);
    if (nextAngle == stateTargetClamp)
    {
      stateTargetClamp = SERVO_STATE_TARGET_INVALID;
    }
  }

  if (stateTargetTrunk >= 0 && trunkLastApplied >= 0)
  {
    nextAngle = MoveAngleTowardTarget(trunkLastApplied, stateTargetTrunk, (uint16_t)stepDeg);
    ApplyStateServoAngle180(SERVO_CH_TRUNK, (uint16_t)nextAngle, &trunkLastApplied, NULL);
    if (nextAngle == stateTargetTrunk)
    {
      stateTargetTrunk = SERVO_STATE_TARGET_INVALID;
    }
  }
}

static void ApplyServoStatePreset(const ServoStatePreset_t *preset)
{
  if (preset == NULL)
  {
    return;
  }

  if (preset->gimbalAngleDeg != SERVO_STATE_KEEP_ANGLE)
  {
    uint16_t target = preset->gimbalAngleDeg;
    if (target > SERVO_ANGLE_270_MAX)
    {
      target = SERVO_ANGLE_270_MAX;
    }
    stateTargetGimbal = (int16_t)target;
  }
  else
  {
    stateTargetGimbal = SERVO_STATE_TARGET_INVALID;
  }

  if (preset->upperAngleDeg != SERVO_STATE_KEEP_ANGLE)
  {
    uint16_t target = preset->upperAngleDeg;
    if (target > SERVO_ANGLE_180_MAX)
    {
      target = SERVO_ANGLE_180_MAX;
    }
    stateTargetUpper = (int16_t)target;
  }
  else
  {
    stateTargetUpper = SERVO_STATE_TARGET_INVALID;
  }

  if (preset->middleAngleDeg != SERVO_STATE_KEEP_ANGLE)
  {
    uint16_t target = preset->middleAngleDeg;
    if (target > SERVO_ANGLE_180_MAX)
    {
      target = SERVO_ANGLE_180_MAX;
    }
    stateTargetMiddle = (int16_t)target;
  }
  else
  {
    stateTargetMiddle = SERVO_STATE_TARGET_INVALID;
  }

  if (preset->lowerAngleDeg != SERVO_STATE_KEEP_ANGLE)
  {
    uint16_t target = preset->lowerAngleDeg;
    if (target > SERVO_ANGLE_180_MAX)
    {
      target = SERVO_ANGLE_180_MAX;
    }
    stateTargetLower = (int16_t)target;
  }
  else
  {
    stateTargetLower = SERVO_STATE_TARGET_INVALID;
  }

  if (preset->flowerAngleDeg != SERVO_STATE_KEEP_ANGLE)
  {
    uint16_t target = preset->flowerAngleDeg;
    if (target > SERVO_ANGLE_180_MAX)
    {
      target = SERVO_ANGLE_180_MAX;
    }
    flowerRunning = 0U;
    flowerDir = 0;
    stateTargetFlower = (int16_t)target;
  }
  else
  {
    stateTargetFlower = SERVO_STATE_TARGET_INVALID;
  }

  if (preset->clampAngleDeg != SERVO_STATE_KEEP_ANGLE)
  {
    uint16_t target = preset->clampAngleDeg;
    if (target > SERVO_ANGLE_180_MAX)
    {
      target = SERVO_ANGLE_180_MAX;
    }
    stateTargetClamp = (int16_t)target;
  }
  else
  {
    stateTargetClamp = SERVO_STATE_TARGET_INVALID;
  }

  if (preset->trunkAngleDeg != SERVO_STATE_KEEP_ANGLE)
  {
    uint16_t target = preset->trunkAngleDeg;
    if (target > SERVO_ANGLE_180_MAX)
    {
      target = SERVO_ANGLE_180_MAX;
    }
    stateTargetTrunk = (int16_t)target;
  }
  else
  {
    stateTargetTrunk = SERVO_STATE_TARGET_INVALID;
  }
}

static void ProcessStatePacket(const char *buf)
{
  const char *p = buf;
  int16_t stateIdRaw;
  const ServoStatePreset_t *preset;

  if (p == NULL || p[0] != '(')
  {
    return;
  }

  p++;
  if (ParseSignedInt16Fast(&p, &stateIdRaw) == 0U || *p != ')' || p[1] != '\0')
  {
    return;
  }

  if (stateIdRaw < 0)
  {
    return;
  }

  preset = FindServoStatePreset((uint16_t)stateIdRaw);
  if (preset == NULL)
  {
    return;
  }

  ApplyServoStatePreset(preset);
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

  if (frameLen < 3U || frame[0] == '\0')
  {
    return;
  }

  if (frame[0] == '(')
  {
    ProcessStatePacket(frame);
    return;
  }

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
  else if (AsciiToLowerFast(frame[1]) == 'f')
  {
    ProcessServoFinalPacket(frame);
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

static uint16_t ClampServoSliderInputRaw(int16_t inputRaw)
{
  if (inputRaw <= (int16_t)SERVO_SLIDER_INPUT_MIN)
  {
    return SERVO_SLIDER_INPUT_MIN;
  }

  if (inputRaw >= (int16_t)SERVO_SLIDER_INPUT_MAX)
  {
    return SERVO_SLIDER_INPUT_MAX;
  }

  return (uint16_t)inputRaw;
}

static uint16_t ServoSliderLowPassUpdate(ServoSliderLpfState_t *state)
{
  float alpha = SERVO_SLIDER_LPF_ALPHA;
  float filtered;

  if (state == NULL || state->initialized == 0U)
  {
    return SERVO_SLIDER_INPUT_MIN;
  }

  if (alpha < 0.0f)
  {
    alpha = 0.0f;
  }
  else if (alpha > 1.0f)
  {
    alpha = 1.0f;
  }

  state->filteredInputRaw += alpha * ((float)state->targetInputRaw - state->filteredInputRaw);

  filtered = state->filteredInputRaw;
  if (filtered < (float)SERVO_SLIDER_INPUT_MIN)
  {
    filtered = (float)SERVO_SLIDER_INPUT_MIN;
  }
  else if (filtered > (float)SERVO_SLIDER_INPUT_MAX)
  {
    filtered = (float)SERVO_SLIDER_INPUT_MAX;
  }

  return (uint16_t)(filtered + 0.5f);
}

static uint16_t ServoSliderInputToAngle180(uint16_t inputRaw)
{
  uint32_t inputSpan = (uint32_t)(SERVO_SLIDER_INPUT_MAX - SERVO_SLIDER_INPUT_MIN);
  uint32_t angle;

  if (inputRaw <= SERVO_SLIDER_INPUT_MIN)
  {
    return 0U;
  }

  if (inputRaw >= SERVO_SLIDER_INPUT_MAX)
  {
    return SERVO_ANGLE_180_MAX;
  }

  angle = ((uint32_t)(inputRaw - SERVO_SLIDER_INPUT_MIN) * SERVO_ANGLE_180_MAX + (inputSpan / 2U)) / inputSpan;
  return (uint16_t)angle;
}

static uint16_t ServoSliderAngle180ToInput(uint16_t angleDeg)
{
  uint32_t inputSpan = (uint32_t)(SERVO_SLIDER_INPUT_MAX - SERVO_SLIDER_INPUT_MIN);
  uint32_t inputRaw;

  if (angleDeg > SERVO_ANGLE_180_MAX)
  {
    angleDeg = SERVO_ANGLE_180_MAX;
  }

  inputRaw = SERVO_SLIDER_INPUT_MIN +
             ((inputSpan * (uint32_t)angleDeg + (SERVO_ANGLE_180_MAX / 2U)) / SERVO_ANGLE_180_MAX);
  return (uint16_t)inputRaw;
}

static void ServoSliderSetTargetInput(ServoSliderLpfState_t *state,
                                      int16_t inputRaw,
                                      int16_t currentAngleDeg)
{
  uint16_t inputRawClamped;
  uint16_t startAngle;

  if (state == NULL)
  {
    return;
  }

  inputRawClamped = ClampServoSliderInputRaw(inputRaw);

  if (state->initialized == 0U)
  {
    if (currentAngleDeg < 0)
    {
      startAngle = 0U;
    }
    else
    {
      startAngle = (uint16_t)currentAngleDeg;
      if (startAngle > SERVO_ANGLE_180_MAX)
      {
        startAngle = SERVO_ANGLE_180_MAX;
      }
    }

    state->filteredInputRaw = (float)ServoSliderAngle180ToInput(startAngle);
    state->initialized = 1U;
  }

  state->targetInputRaw = inputRawClamped;
}

static void ServoSliderApplySmoothedAngle180(uint8_t channel,
                                             ServoSliderLpfState_t *state,
                                             int16_t *lastAppliedAngle)
{
  uint16_t inputRawFiltered;
  uint16_t targetAngle;

  if (state == NULL || lastAppliedAngle == NULL || state->initialized == 0U)
  {
    return;
  }

  inputRawFiltered = ServoSliderLowPassUpdate(state);
  targetAngle = ServoSliderInputToAngle180(inputRawFiltered);

  if ((int16_t)targetAngle != *lastAppliedAngle)
  {
    (void)ServoSetAngle180ByChannel(channel, targetAngle);
    *lastAppliedAngle = (int16_t)targetAngle;
  }
}

static void UpdateServo180ByFilteredSliderInput(uint8_t channel,
                                              int16_t inputRaw,
                                              ServoSliderLpfState_t *filterState,
                                              int16_t *lastAppliedAngle)
{
  (void)channel;

  if (lastAppliedAngle == NULL)
  {
    return;
  }

  /* Process packet only updates target; smoothing step runs in main loop. */
  ServoSliderSetTargetInput(filterState, inputRaw, *lastAppliedAngle);
}

static void UpdateServo180ByFinalSliderInput(uint8_t channel,
                                           int16_t inputRaw,
                                           ServoSliderLpfState_t *filterState,
                                           int16_t *lastAppliedAngle)
{
  (void)channel;

  if (lastAppliedAngle == NULL)
  {
    return;
  }

  /* Final packet also updates target; motion stays smooth and converges. */
  ServoSliderSetTargetInput(filterState, inputRaw, *lastAppliedAngle);
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
  stateTransitionLastTick = HAL_GetTick();
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
    ServoStateTransitionStep();
    if (stateTargetUpper == SERVO_STATE_TARGET_INVALID)
    {
      ServoSliderApplySmoothedAngle180(SERVO_CH_UPPER, &upperSliderLpf, &upperLastApplied);
    }
    if (stateTargetMiddle == SERVO_STATE_TARGET_INVALID)
    {
      ServoSliderApplySmoothedAngle180(SERVO_CH_MIDDLE, &middleSliderLpf, &middleLastApplied);
    }
    if (stateTargetLower == SERVO_STATE_TARGET_INVALID)
    {
      ServoSliderApplySmoothedAngle180(SERVO_CH_LOWER, &lowerSliderLpf, &lowerLastApplied);
    }
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

/* Parse process packet [s,id,x]:
 * id=1 -> gimbal angle (CH9, 0~270), id=2 -> speed percent,
 * id=3/4/5 -> upper/middle/lower slider input value.
 */
static void ProcessServoPacket(char *buf)
{
  const char *p = buf;
  int16_t servoIdVal;
  int16_t inputVal;
  int servoId;
  int16_t targetAngle;

  if (p[0] != '[' || AsciiToLowerFast(p[1]) != 's' || p[2] != ',') return;

  p += 3;
  if (ParseSignedInt16Fast(&p, &servoIdVal) == 0U || *p != ',') return;
  p++;
  if (ParseSignedInt16Fast(&p, &inputVal) == 0U || *p != ']' || p[1] != '\0') return;

  servoId = (int)servoIdVal;

  if (servoId == SERVO_SLIDER_SPEED_ID)
  {
    int32_t speedPct = (int32_t)inputVal;
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

  if (servoId == SERVO_SLIDER_GIMBAL_ID)
  {
    stateTargetGimbal = SERVO_STATE_TARGET_INVALID;
    targetAngle = (inputVal < 0) ? 0 : (int16_t)inputVal;

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

  if (servoId == SERVO_SLIDER_UPPER_ID)
  {
    stateTargetUpper = SERVO_STATE_TARGET_INVALID;
    UpdateServo180ByFilteredSliderInput(SERVO_CH_UPPER,
                                      inputVal,
                                      &upperSliderLpf,
                                      &upperLastApplied);
    return;
  }

  if (servoId == SERVO_SLIDER_MIDDLE_ID)
  {
    stateTargetMiddle = SERVO_STATE_TARGET_INVALID;
    UpdateServo180ByFilteredSliderInput(SERVO_CH_MIDDLE,
                                      inputVal,
                                      &middleSliderLpf,
                                      &middleLastApplied);
    return;
  }

  if (servoId == SERVO_SLIDER_LOWER_ID)
  {
    stateTargetLower = SERVO_STATE_TARGET_INVALID;
    UpdateServo180ByFilteredSliderInput(SERVO_CH_LOWER,
                                      inputVal,
                                      &lowerSliderLpf,
                                      &lowerLastApplied);
  }
}

/* Parse final packet [f,id,x]:
 * id=3/4/5 -> upper/middle/lower final slider input value.
 */
static void ProcessServoFinalPacket(char *buf)
{
  const char *p = buf;
  int16_t servoIdVal;
  int16_t inputVal;
  int servoId;

  if (p[0] != '[' || AsciiToLowerFast(p[1]) != 'f' || p[2] != ',') return;

  p += 3;
  if (ParseSignedInt16Fast(&p, &servoIdVal) == 0U || *p != ',') return;
  p++;
  if (ParseSignedInt16Fast(&p, &inputVal) == 0U || *p != ']' || p[1] != '\0') return;

  servoId = (int)servoIdVal;

  if (servoId == SERVO_SLIDER_UPPER_ID)
  {
    stateTargetUpper = SERVO_STATE_TARGET_INVALID;
    UpdateServo180ByFinalSliderInput(SERVO_CH_UPPER,
                                   inputVal,
                                   &upperSliderLpf,
                                   &upperLastApplied);
    return;
  }

  if (servoId == SERVO_SLIDER_MIDDLE_ID)
  {
    stateTargetMiddle = SERVO_STATE_TARGET_INVALID;
    UpdateServo180ByFinalSliderInput(SERVO_CH_MIDDLE,
                                   inputVal,
                                   &middleSliderLpf,
                                   &middleLastApplied);
    return;
  }

  if (servoId == SERVO_SLIDER_LOWER_ID)
  {
    stateTargetLower = SERVO_STATE_TARGET_INVALID;
    UpdateServo180ByFinalSliderInput(SERVO_CH_LOWER,
                                   inputVal,
                                   &lowerSliderLpf,
                                   &lowerLastApplied);
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
    stateTargetClamp = SERVO_STATE_TARGET_INVALID;
    if (clampLastApplied != CLAMP_CLOSE_ANGLE)
    {
      (void)ServoSetAngle180ByChannel(SERVO_CH_CLAMP, (uint16_t)CLAMP_CLOSE_ANGLE);
      clampLastApplied = CLAMP_CLOSE_ANGLE;
    }
    return;
  }

  if (moveCmd == 'd')
  {
    stateTargetClamp = SERVO_STATE_TARGET_INVALID;
    if (clampLastApplied != CLAMP_OPEN_ANGLE)
    {
      (void)ServoSetAngle180ByChannel(SERVO_CH_CLAMP, (uint16_t)CLAMP_OPEN_ANGLE);
      clampLastApplied = CLAMP_OPEN_ANGLE;
    }
    return;
  }

  if (moveCmd == 'i')
  {
    stateTargetTrunk = SERVO_STATE_TARGET_INVALID;
    if (trunkLastApplied != TRUNK_RETRACT_ANGLE)
    {
      (void)ServoSetAngle180ByChannel(SERVO_CH_TRUNK, (uint16_t)TRUNK_RETRACT_ANGLE);
      trunkLastApplied = TRUNK_RETRACT_ANGLE;
    }
    return;
  }

  if (moveCmd == 'o')
  {
    stateTargetTrunk = SERVO_STATE_TARGET_INVALID;
    if (trunkLastApplied != TRUNK_OPEN_ANGLE)
    {
      (void)ServoSetAngle180ByChannel(SERVO_CH_TRUNK, (uint16_t)TRUNK_OPEN_ANGLE);
      trunkLastApplied = TRUNK_OPEN_ANGLE;
    }
    return;
  }

  if (moveCmd == 'q')
  {
    stateTargetFlower = SERVO_STATE_TARGET_INVALID;
    flowerDir = 1;
    flowerRunning = 1;
    flowerLastTick = HAL_GetTick();
    return;
  }

  if (moveCmd == 'e')
  {
    stateTargetFlower = SERVO_STATE_TARGET_INVALID;
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

    if (byte == '[' || byte == '(')
    {
      uart_in_frame = 1U;
      uart_rx_idx = 0U;
      uart_frame_end = (byte == '[') ? (uint8_t)']' : (uint8_t)')';
    }

    if (uart_in_frame)
    {
      if (uart_rx_idx < (uint8_t)(sizeof(uart_rx_buf) - 1))
      {
        uart_rx_buf[uart_rx_idx++] = (char)byte;
        uart_rx_buf[uart_rx_idx] = '\0';

        if (byte == uart_frame_end)
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
          uart_frame_end = 0U;
          uart_rx_buf[0] = '\0';
        }
      }
      else
      {
        uart_in_frame = 0;
        uart_rx_idx = 0;
        uart_frame_end = 0U;
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
    uart_frame_end = 0U;
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

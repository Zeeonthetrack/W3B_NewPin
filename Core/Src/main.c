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
#include <math.h>
#include "pca9685.h"
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

/* Two-level speed control: 10ms control period + safety limits. */
#define CONTROL_PERIOD_MS 10U
#define SCURVE_MAX_ACC_CPS2 120000
#define SCURVE_MAX_DEC_CPS2 110000
#define SCURVE_MAX_JERK_CPS3 1800000
#define SCURVE_TRACK_GAIN_NUM 8
#define SCURVE_TRACK_GAIN_DEN 10
#define SCURVE_FAST_ERR_CPS 1000
#define SCURVE_MIN_ACC_CPS2 600
#define SCURVE_SNAP_ERR_CPS 6
#define SPEED_EPS_CPS 20
#define ACC_EPS_CPS2 100
#define STOP_HOLD_CPS 80
#define JOY_DEADZONE_PCT 4
#define PWM_OUT_SLEW_UP_UNITS_PER_SEC 15000
#define PWM_OUT_SLEW_DOWN_UNITS_PER_SEC 9000

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
/* Speed slew limiter in PWM-units per second (tune this): larger=faster response. */
#define MOTOR_SLEW_RATE_UNITS_PER_SEC 200
static int16_t joyLyBuf[JOY_FILTER_SIZE];
static int16_t joyRyBuf[JOY_FILTER_SIZE];
static uint8_t joyFilterPos = 0;
static uint8_t joyFilterCount = 0;
/* minimum joystick delta (raw units) to treat as a meaningful new command */
#define JOY_INPUT_CHANGE_THRESHOLD 2
static int16_t lastInputLy = 32767;
static int16_t lastInputRy = 32767;
/* last applied mapped speeds (timer units) */
static int16_t lastAppliedA = 0;
static int16_t lastAppliedB = 0;
static int16_t targetPctA = 0;
static int16_t targetPctB = 0;
static int16_t lastServoAngle = -1;
static uint32_t last_ctrl_tick = 0;

typedef struct
{
  int32_t v_ref_cps;
  int32_t a_ref_cps2;
  uint8_t phase;
} Scurve7Planner_t;

typedef struct
{
  uint8_t inited;
  uint16_t last_cnt;
  uint32_t last_tick;
  int32_t last_speed_cps;
} EncoderObs_t;

typedef struct
{
  Scurve7Planner_t planner;
  EncoderObs_t enc;
  int32_t max_cps;
} WheelCtrl_t;

static WheelCtrl_t wheelL = {0};
static WheelCtrl_t wheelR = {0};
/* watchdog timeout (ms): if no valid joystick packet within this, stop motors */
#define JOY_TIMEOUT_MS 500U
static uint32_t last_valid_rx_tick = 0;
// static uint32_t last_oled_update_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ProcessJoystickPacket(char *buf);
static void ProcessServoPacket(char *buf);
static void ProcessLatestUartFrame(void);
static int32_t ClampI32(int32_t x, int32_t lo, int32_t hi);
static int32_t EncoderUpdateSignedCps(TIM_HandleTypeDef *htim, EncoderObs_t *obs, uint32_t minDtMs);
static void Scurve7Step(Scurve7Planner_t *p, int32_t target_cps, uint32_t dtMs);
static void ControlLoopStep(uint32_t nowTick);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void ProcessUartBytes(void)
{
  ProcessLatestUartFrame();
}

static int32_t ClampI32(int32_t x, int32_t lo, int32_t hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static int32_t SignI32(int32_t x)
{
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

static void WheelControllerResetToZero(WheelCtrl_t *w)
{
  w->planner.v_ref_cps = 0;
  w->planner.a_ref_cps2 = 0;
  w->planner.phase = 4;
}

static int32_t EncoderUpdateSignedCps(TIM_HandleTypeDef *htim, EncoderObs_t *obs, uint32_t minDtMs)
{
  uint32_t now = HAL_GetTick();
  uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(htim);

  if (!obs->inited)
  {
    obs->inited = 1;
    obs->last_cnt = cnt;
    obs->last_tick = now;
    obs->last_speed_cps = 0;
    return 0;
  }

  uint32_t dtMs = now - obs->last_tick;
  if (dtMs < minDtMs)
  {
    return obs->last_speed_cps;
  }

  int16_t deltaCnt = (int16_t)(cnt - obs->last_cnt);
  int32_t cps = ((int32_t)deltaCnt * 1000) / (int32_t)dtMs;

  obs->last_cnt = cnt;
  obs->last_tick = now;
  obs->last_speed_cps = cps;
  return cps;
}

static void Scurve7Step(Scurve7Planner_t *p, int32_t target_cps, uint32_t dtMs)
{
  int32_t dt = (int32_t)dtMs;
  int32_t err = target_cps - p->v_ref_cps;

  /* Snap near target to remove exponential-like tail when error is tiny. */
  if (abs(err) <= SCURVE_SNAP_ERR_CPS)
  {
    p->v_ref_cps = target_cps;
    p->a_ref_cps2 = 0;
    p->phase = 4;
    return;
  }

  int32_t acc_limit = SCURVE_MAX_ACC_CPS2;
  /* If current speed and error have opposite sign, this is a braking phase. */
  if ((p->v_ref_cps > 0 && err < 0) || (p->v_ref_cps < 0 && err > 0))
  {
    acc_limit = SCURVE_MAX_DEC_CPS2;
  }

  int32_t absErr = abs(err);
  int32_t desired_acc;
  if (absErr >= SCURVE_FAST_ERR_CPS)
  {
    desired_acc = SignI32(err) * acc_limit;
  }
  else
  {
    desired_acc = (err * SCURVE_TRACK_GAIN_NUM) / SCURVE_TRACK_GAIN_DEN;
    if (abs(desired_acc) < SCURVE_MIN_ACC_CPS2)
    {
      desired_acc = SignI32(err) * SCURVE_MIN_ACC_CPS2;
    }
  }
  desired_acc = ClampI32(desired_acc, -acc_limit, acc_limit);

  int32_t jerk_step = (SCURVE_MAX_JERK_CPS3 * dt) / 1000;
  if (jerk_step < 1)
  {
    jerk_step = 1;
  }

  if (p->a_ref_cps2 < desired_acc)
  {
    p->a_ref_cps2 += jerk_step;
    if (p->a_ref_cps2 > desired_acc) p->a_ref_cps2 = desired_acc;
  }
  else if (p->a_ref_cps2 > desired_acc)
  {
    p->a_ref_cps2 -= jerk_step;
    if (p->a_ref_cps2 < desired_acc) p->a_ref_cps2 = desired_acc;
  }

  p->a_ref_cps2 = ClampI32(p->a_ref_cps2, -acc_limit, acc_limit);
  p->v_ref_cps += (p->a_ref_cps2 * dt) / 1000;

  if ((err > 0 && p->v_ref_cps > target_cps) || (err < 0 && p->v_ref_cps < target_cps))
  {
    p->v_ref_cps = target_cps;
  }

  if (abs(err) <= SPEED_EPS_CPS && abs(p->a_ref_cps2) <= ACC_EPS_CPS2)
  {
    p->phase = 4;
  }
  else if (err >= 0)
  {
    if (p->a_ref_cps2 < 0) p->phase = 7;
    else if (p->a_ref_cps2 < desired_acc) p->phase = 1;
    else if (p->a_ref_cps2 > desired_acc) p->phase = 3;
    else p->phase = 2;
  }
  else
  {
    if (p->a_ref_cps2 > 0) p->phase = 5;
    else if (p->a_ref_cps2 > desired_acc) p->phase = 5;
    else if (p->a_ref_cps2 < desired_acc) p->phase = 7;
    else p->phase = 6;
  }
}

static void ControlLoopStep(uint32_t nowTick)
{
  if (last_ctrl_tick == 0U)
  {
    last_ctrl_tick = nowTick;
    return;
  }

  uint32_t dtMs = nowTick - last_ctrl_tick;
  if (dtMs < CONTROL_PERIOD_MS)
  {
    return;
  }
  if (dtMs > 100U)
  {
    dtMs = CONTROL_PERIOD_MS;
  }
  last_ctrl_tick = nowTick;

  if ((HAL_GetTick() - last_valid_rx_tick) > JOY_TIMEOUT_MS)
  {
    targetPctA = 0;
    targetPctB = 0;
  }

  int32_t targetL = ((int32_t)targetPctA * wheelL.max_cps) / 100;
  int32_t targetR = ((int32_t)targetPctB * wheelR.max_cps) / 100;
  targetL = ClampI32(targetL, -wheelL.max_cps, wheelL.max_cps);
  targetR = ClampI32(targetR, -wheelR.max_cps, wheelR.max_cps);

  int32_t fbL = EncoderUpdateSignedCps(&htim2, &wheelL.enc, CONTROL_PERIOD_MS);
  int32_t fbR = EncoderUpdateSignedCps(&htim4, &wheelR.enc, CONTROL_PERIOD_MS);

  if (targetPctA == 0 && targetPctB == 0 && abs(fbL) <= STOP_HOLD_CPS && abs(fbR) <= STOP_HOLD_CPS)
  {
    WheelControllerResetToZero(&wheelL);
    WheelControllerResetToZero(&wheelR);
    SetSpeed_L(0);
    SetSpeed_R(0);
    lastAppliedA = 0;
    lastAppliedB = 0;
    SpeedA = 0;
    SpeedB = 0;
    return;
  }

  Scurve7Step(&wheelL.planner, targetL, dtMs);
  Scurve7Step(&wheelR.planner, targetR, dtMs);

  int16_t pwmLimit = (int16_t)htim3.Init.Period;
  int32_t cmdL32 = (wheelL.max_cps != 0) ? ((wheelL.planner.v_ref_cps * (int32_t)pwmLimit) / wheelL.max_cps) : 0;
  int32_t cmdR32 = (wheelR.max_cps != 0) ? ((wheelR.planner.v_ref_cps * (int32_t)pwmLimit) / wheelR.max_cps) : 0;
  int32_t cmdL = ClampI32(cmdL32, -pwmLimit, pwmLimit);
  int32_t cmdR = ClampI32(cmdR32, -pwmLimit, pwmLimit);

  /* Final PWM slew limiter to suppress visible step changes from quantization/noise. */
  int32_t upStep = (PWM_OUT_SLEW_UP_UNITS_PER_SEC * (int32_t)dtMs) / 1000;
  int32_t downStep = (PWM_OUT_SLEW_DOWN_UNITS_PER_SEC * (int32_t)dtMs) / 1000;
  if (upStep < 1)
  {
    upStep = 1;
  }
  if (downStep < 1)
  {
    downStep = 1;
  }

  int32_t prevL = (int32_t)lastAppliedA;
  int32_t prevR = (int32_t)lastAppliedB;
  int32_t stepL = ((abs(cmdL) > abs(prevL)) && (SignI32(cmdL) == SignI32(prevL))) ? upStep : downStep;
  int32_t stepR = ((abs(cmdR) > abs(prevR)) && (SignI32(cmdR) == SignI32(prevR))) ? upStep : downStep;

  cmdL = ClampI32(cmdL, prevL - stepL, prevL + stepL);
  cmdR = ClampI32(cmdR, prevR - stepR, prevR + stepR);

  SetSpeed_L((int16_t)cmdL);
  SetSpeed_R((int16_t)cmdR);

  lastAppliedA = (int16_t)cmdL;
  lastAppliedB = (int16_t)cmdR;
  SpeedA = (int16_t)cmdL;
  SpeedB = (int16_t)cmdR;
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

  wheelL.max_cps = LEFT_ENCODER_MAX_CPS;
  wheelR.max_cps = RIGHT_ENCODER_MAX_CPS;
  
  HAL_StatusTypeDef pca_status = PCA9685_Init(&hi2c2, 0x40, 50);
  (void)pca_status;

  // PCA9685_SetServoAngle(&hi2c2, 0x40, 0, 0, 500, 2500, 50);
  // HAL_Delay(3000);
  // PCA9685_SetServoAngle(&hi2c2, 0x40, 0, 30, 500, 2500, 50);
  // HAL_Delay(3000); 

  SetSpeed_L(0);
  SetSpeed_R(0);
  SetSpeed_H(0);
  


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
    /* 20ms dual-loop control: S-curve planner + incremental PID speed loop. */
    ControlLoopStep(HAL_GetTick());

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

  /* valid packet received: update last valid timestamp */
  last_valid_rx_tick = HAL_GetTick();

  lastInputLy = (int16_t)Ly;
  lastInputRy = (int16_t)Ry;

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
  targetPctA = (int16_t)ClampI32(avgLy, -100, 100);
  targetPctB = (int16_t)ClampI32(avgRy, -100, 100);
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

/* Parse servo packet: format [s,x,angle] */
static void ProcessServoPacket(char *buf)
{
  int secondVal;
  if (buf[0] != '[' || buf[1] != 's' || buf[2] != ',') return;
  char *end = strchr(buf, ']');
  if (!end) return;

  char tmpBuf[32];
  size_t len = (size_t)(end - (buf + 3));
  if (len >= sizeof(tmpBuf)) return;
  memcpy(tmpBuf, buf + 3, len);
  tmpBuf[len] = '\0';

  if (sscanf(tmpBuf, "%*d,%d", &secondVal) != 1) return;

  /* second value controls CH0 angle: negative->0, positive keeps raw value. */
  int16_t targetAngle = (secondVal < 0) ? 0 : (int16_t)secondVal;
  if (targetAngle > 270)
  {
    targetAngle = 270;
  }

  if (targetAngle != lastServoAngle)
  {
    PCA9685_SetServoAngle(&hi2c2, 0x40, 0, targetAngle, 500, 2500, 50);
    lastServoAngle = targetAngle;
  }
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

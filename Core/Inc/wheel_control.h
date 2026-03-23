#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

typedef struct
{
  int32_t v_ref_cps;
  int32_t a_ref_cps2;
  uint8_t phase;
} WheelControlPlanner_t;

typedef struct
{
  uint8_t inited;
  uint16_t last_cnt;
  uint32_t last_tick;
  int32_t last_speed_cps;
} WheelControlEncoderObs_t;

typedef struct
{
  WheelControlPlanner_t planner;
  WheelControlEncoderObs_t enc;
  int32_t max_cps;
} WheelControlWheel_t;

typedef struct
{
  WheelControlWheel_t wheel_l;
  WheelControlWheel_t wheel_r;
  int16_t target_pct_l;
  int16_t target_pct_r;
  int16_t last_cmd_l;
  int16_t last_cmd_r;
  uint16_t pwm_limit;
  uint32_t joy_timeout_ms;
  uint32_t last_valid_rx_tick;
  uint32_t last_ctrl_tick;
} WheelControl_t;

void WheelControl_Init(
  WheelControl_t *ctx,
  int32_t leftMaxCps,
  int32_t rightMaxCps,
  uint16_t pwmLimit,
  uint32_t joyTimeoutMs,
  uint32_t nowTick);

void WheelControl_SetTargetsPercent(WheelControl_t *ctx, int16_t leftPct, int16_t rightPct);
void WheelControl_MarkValidRx(WheelControl_t *ctx, uint32_t nowTick);

void WheelControl_Step(
  WheelControl_t *ctx,
  TIM_HandleTypeDef *encLeftTim,
  TIM_HandleTypeDef *encRightTim,
  uint32_t nowTick,
  int16_t *outLeftCmd,
  int16_t *outRightCmd);

#ifdef __cplusplus
}
#endif

#endif

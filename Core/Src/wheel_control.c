#include "wheel_control.h"

#include <stdlib.h>

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
#define PWM_OUT_SLEW_UP_UNITS_PER_SEC 15000
#define PWM_OUT_SLEW_DOWN_UNITS_PER_SEC 9000

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

static void WheelControllerResetToZero(WheelControlWheel_t *w)
{
  w->planner.v_ref_cps = 0;
  w->planner.a_ref_cps2 = 0;
  w->planner.phase = 4;
}

static int32_t EncoderUpdateSignedCps(
  TIM_HandleTypeDef *htim,
  WheelControlEncoderObs_t *obs,
  uint32_t nowTick,
  uint32_t minDtMs)
{
  uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(htim);

  if (!obs->inited)
  {
    obs->inited = 1;
    obs->last_cnt = cnt;
    obs->last_tick = nowTick;
    obs->last_speed_cps = 0;
    return 0;
  }

  uint32_t dtMs = nowTick - obs->last_tick;
  if (dtMs < minDtMs)
  {
    return obs->last_speed_cps;
  }

  int16_t deltaCnt = (int16_t)(cnt - obs->last_cnt);
  int32_t cps = ((int32_t)deltaCnt * 1000) / (int32_t)dtMs;

  obs->last_cnt = cnt;
  obs->last_tick = nowTick;
  obs->last_speed_cps = cps;
  return cps;
}

static void Scurve7Step(WheelControlPlanner_t *p, int32_t target_cps, uint32_t dtMs)
{
  int32_t dt = (int32_t)dtMs;
  int32_t err = target_cps - p->v_ref_cps;

  if (abs(err) <= SCURVE_SNAP_ERR_CPS)
  {
    p->v_ref_cps = target_cps;
    p->a_ref_cps2 = 0;
    p->phase = 4;
    return;
  }

  int32_t acc_limit = SCURVE_MAX_ACC_CPS2;
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

void WheelControl_Init(
  WheelControl_t *ctx,
  int32_t leftMaxCps,
  int32_t rightMaxCps,
  uint16_t pwmLimit,
  uint32_t joyTimeoutMs,
  uint32_t nowTick)
{
  ctx->wheel_l.max_cps = leftMaxCps;
  ctx->wheel_r.max_cps = rightMaxCps;
  ctx->target_pct_l = 0;
  ctx->target_pct_r = 0;
  ctx->last_cmd_l = 0;
  ctx->last_cmd_r = 0;
  ctx->pwm_limit = pwmLimit;
  ctx->joy_timeout_ms = joyTimeoutMs;
  ctx->last_valid_rx_tick = nowTick;
  ctx->last_ctrl_tick = 0;
  WheelControllerResetToZero(&ctx->wheel_l);
  WheelControllerResetToZero(&ctx->wheel_r);
}

void WheelControl_SetTargetsPercent(WheelControl_t *ctx, int16_t leftPct, int16_t rightPct)
{
  ctx->target_pct_l = (int16_t)ClampI32(leftPct, -100, 100);
  ctx->target_pct_r = (int16_t)ClampI32(rightPct, -100, 100);
}

void WheelControl_MarkValidRx(WheelControl_t *ctx, uint32_t nowTick)
{
  ctx->last_valid_rx_tick = nowTick;
}

void WheelControl_Step(
  WheelControl_t *ctx,
  TIM_HandleTypeDef *encLeftTim,
  TIM_HandleTypeDef *encRightTim,
  uint32_t nowTick,
  int16_t *outLeftCmd,
  int16_t *outRightCmd)
{
  int16_t targetPctL = ctx->target_pct_l;
  int16_t targetPctR = ctx->target_pct_r;

  if ((nowTick - ctx->last_valid_rx_tick) > ctx->joy_timeout_ms)
  {
    targetPctL = 0;
    targetPctR = 0;
  }

  if (ctx->last_ctrl_tick == 0U)
  {
    ctx->last_ctrl_tick = nowTick;
    *outLeftCmd = ctx->last_cmd_l;
    *outRightCmd = ctx->last_cmd_r;
    return;
  }

  uint32_t dtMs = nowTick - ctx->last_ctrl_tick;
  if (dtMs < CONTROL_PERIOD_MS)
  {
    *outLeftCmd = ctx->last_cmd_l;
    *outRightCmd = ctx->last_cmd_r;
    return;
  }
  if (dtMs > 100U)
  {
    dtMs = CONTROL_PERIOD_MS;
  }
  ctx->last_ctrl_tick = nowTick;

  int32_t targetL = ((int32_t)targetPctL * ctx->wheel_l.max_cps) / 100;
  int32_t targetR = ((int32_t)targetPctR * ctx->wheel_r.max_cps) / 100;
  targetL = ClampI32(targetL, -ctx->wheel_l.max_cps, ctx->wheel_l.max_cps);
  targetR = ClampI32(targetR, -ctx->wheel_r.max_cps, ctx->wheel_r.max_cps);

  int32_t fbL = EncoderUpdateSignedCps(encLeftTim, &ctx->wheel_l.enc, nowTick, CONTROL_PERIOD_MS);
  int32_t fbR = EncoderUpdateSignedCps(encRightTim, &ctx->wheel_r.enc, nowTick, CONTROL_PERIOD_MS);

  if (targetPctL == 0 && targetPctR == 0 && abs(fbL) <= STOP_HOLD_CPS && abs(fbR) <= STOP_HOLD_CPS)
  {
    WheelControllerResetToZero(&ctx->wheel_l);
    WheelControllerResetToZero(&ctx->wheel_r);
    ctx->last_cmd_l = 0;
    ctx->last_cmd_r = 0;
    *outLeftCmd = 0;
    *outRightCmd = 0;
    return;
  }

  Scurve7Step(&ctx->wheel_l.planner, targetL, dtMs);
  Scurve7Step(&ctx->wheel_r.planner, targetR, dtMs);

  int32_t cmdL32 = (ctx->wheel_l.max_cps != 0) ? ((ctx->wheel_l.planner.v_ref_cps * (int32_t)ctx->pwm_limit) / ctx->wheel_l.max_cps) : 0;
  int32_t cmdR32 = (ctx->wheel_r.max_cps != 0) ? ((ctx->wheel_r.planner.v_ref_cps * (int32_t)ctx->pwm_limit) / ctx->wheel_r.max_cps) : 0;
  int32_t cmdL = ClampI32(cmdL32, -(int32_t)ctx->pwm_limit, (int32_t)ctx->pwm_limit);
  int32_t cmdR = ClampI32(cmdR32, -(int32_t)ctx->pwm_limit, (int32_t)ctx->pwm_limit);

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

  int32_t prevL = (int32_t)ctx->last_cmd_l;
  int32_t prevR = (int32_t)ctx->last_cmd_r;
  int32_t stepL = ((abs(cmdL) > abs(prevL)) && (SignI32(cmdL) == SignI32(prevL))) ? upStep : downStep;
  int32_t stepR = ((abs(cmdR) > abs(prevR)) && (SignI32(cmdR) == SignI32(prevR))) ? upStep : downStep;

  cmdL = ClampI32(cmdL, prevL - stepL, prevL + stepL);
  cmdR = ClampI32(cmdR, prevR - stepR, prevR + stepR);

  ctx->last_cmd_l = (int16_t)cmdL;
  ctx->last_cmd_r = (int16_t)cmdR;
  *outLeftCmd = ctx->last_cmd_l;
  *outRightCmd = ctx->last_cmd_r;
}

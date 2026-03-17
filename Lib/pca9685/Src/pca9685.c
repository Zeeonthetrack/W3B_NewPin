#include "pca9685.h"
#include "i2c.h"

#define PCA9685_FIXED_I2C_HANDLE (&hi2c2)

#define PCA9685_REG_MODE1         (0x00U)
#define PCA9685_REG_MODE2         (0x01U)
#define PCA9685_REG_LED0_ON_L     (0x06U)
#define PCA9685_REG_PRESCALE      (0xFEU)

#define PCA9685_MODE1_RESTART     (0x80U)
#define PCA9685_MODE1_AI          (0x20U)
#define PCA9685_MODE1_SLEEP       (0x10U)

#define PCA9685_MODE2_OUTDRV      (0x04U)

#define PCA9685_OSC_HZ            (25000000UL)
#define PCA9685_MIN_FREQ_HZ       (24U)
#define PCA9685_MAX_FREQ_HZ       (1526U)
#define PCA9685_I2C_TIMEOUT_MS    (100U)

static uint16_t pca9685_get_addr(uint8_t addr7bit)
{
  return (uint16_t)(addr7bit << 1);
}

static HAL_StatusTypeDef pca9685_write_reg(I2C_HandleTypeDef *hi2c,
                                           uint8_t addr7bit,
                                           uint8_t reg,
                                           uint8_t value)
{
  return HAL_I2C_Mem_Write(hi2c,
                           pca9685_get_addr(addr7bit),
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &value,
                           1U,
                           PCA9685_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef pca9685_read_reg(I2C_HandleTypeDef *hi2c,
                                          uint8_t addr7bit,
                                          uint8_t reg,
                                          uint8_t *value)
{
  return HAL_I2C_Mem_Read(hi2c,
                          pca9685_get_addr(addr7bit),
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          value,
                          1U,
                          PCA9685_I2C_TIMEOUT_MS);
}

static uint8_t pca9685_get_prescale(uint16_t pwmHz)
{
  uint32_t denom = 4096UL * (uint32_t)pwmHz;
  uint32_t prescale = (PCA9685_OSC_HZ + (denom / 2UL)) / denom;

  if (prescale > 0UL)
  {
    prescale -= 1UL;
  }

  if (prescale > 255UL)
  {
    prescale = 255UL;
  }

  return (uint8_t)prescale;
}

HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7bit, uint16_t pwmHz)
{
  HAL_StatusTypeDef st;
  uint8_t prescale;
  uint8_t oldMode;
  uint8_t wakeMode;

  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  if (pwmHz < PCA9685_MIN_FREQ_HZ || pwmHz > PCA9685_MAX_FREQ_HZ)
  {
    return HAL_ERROR;
  }

  prescale = pca9685_get_prescale(pwmHz);

  if (HAL_I2C_IsDeviceReady(hi2c,
                            pca9685_get_addr(addr7bit),
                            2U,
                            PCA9685_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  st = pca9685_read_reg(hi2c, addr7bit, PCA9685_REG_MODE1, &oldMode);
  if (st != HAL_OK)
  {
    return st;
  }

  wakeMode = (uint8_t)((oldMode | PCA9685_MODE1_AI) & (~PCA9685_MODE1_SLEEP));

  st = pca9685_write_reg(hi2c,
                         addr7bit,
                         PCA9685_REG_MODE1,
                         (uint8_t)((oldMode & (~PCA9685_MODE1_RESTART)) | PCA9685_MODE1_SLEEP));
  if (st != HAL_OK)
  {
    return st;
  }

  st = pca9685_write_reg(hi2c, addr7bit, PCA9685_REG_PRESCALE, prescale);
  if (st != HAL_OK)
  {
    return st;
  }

  st = pca9685_write_reg(hi2c, addr7bit, PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV);
  if (st != HAL_OK)
  {
    return st;
  }

  st = pca9685_write_reg(hi2c, addr7bit, PCA9685_REG_MODE1, wakeMode);
  if (st != HAL_OK)
  {
    return st;
  }

  HAL_Delay(1);

  st = pca9685_write_reg(hi2c,
                         addr7bit,
                         PCA9685_REG_MODE1,
                         (uint8_t)(wakeMode | PCA9685_MODE1_RESTART));
  if (st != HAL_OK)
  {
    return st;
  }

  return HAL_OK;
}

HAL_StatusTypeDef PCA9685_SetPwmCounts(I2C_HandleTypeDef *hi2c,
                                        uint8_t addr7bit,
                                        uint8_t channel,
                                        uint16_t onCount,
                                        uint16_t offCount)
{
  uint8_t reg;
  uint8_t data[4];

  if (hi2c == NULL || channel >= PCA9685_CHANNEL_COUNT)
  {
    return HAL_ERROR;
  }

  onCount &= PCA9685_COUNT_MAX;
  offCount &= PCA9685_COUNT_MAX;

  reg = (uint8_t)(PCA9685_REG_LED0_ON_L + (4U * channel));
  data[0] = (uint8_t)(onCount & 0xFFU);
  data[1] = (uint8_t)((onCount >> 8) & 0x0FU);
  data[2] = (uint8_t)(offCount & 0xFFU);
  data[3] = (uint8_t)((offCount >> 8) & 0x0FU);

  return HAL_I2C_Mem_Write(hi2c,
                           pca9685_get_addr(addr7bit),
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           data,
                           4U,
                           PCA9685_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef PCA9685_SetDutyPercent(I2C_HandleTypeDef *hi2c,
                                          uint8_t addr7bit,
                                          uint8_t channel,
                                          uint8_t dutyPercent)
{
  uint16_t offCount;

  if (dutyPercent > 100U)
  {
    dutyPercent = 100U;
  }

  offCount = (uint16_t)(((uint32_t)dutyPercent * (PCA9685_COUNT_MAX + 1U)) / 100U);
  if (offCount > PCA9685_COUNT_MAX)
  {
    offCount = PCA9685_COUNT_MAX;
  }

  return PCA9685_SetPwmCounts(hi2c, addr7bit, channel, 0U, offCount);
}

HAL_StatusTypeDef PCA9685_SetServoPulseUs(I2C_HandleTypeDef *hi2c,
                                           uint8_t addr7bit,
                                           uint8_t channel,
                                           uint16_t pulseUs,
                                           uint16_t pwmHz)
{
  uint32_t cycleUs;
  uint32_t offCount;

  if (pwmHz == 0U)
  {
    return HAL_ERROR;
  }

  cycleUs = 1000000UL / (uint32_t)pwmHz;
  if (pulseUs > cycleUs)
  {
    pulseUs = (uint16_t)cycleUs;
  }

  offCount = ((uint32_t)pulseUs * (PCA9685_COUNT_MAX + 1UL)) / cycleUs;
  if (offCount > PCA9685_COUNT_MAX)
  {
    offCount = PCA9685_COUNT_MAX;
  }

  return PCA9685_SetPwmCounts(hi2c, addr7bit, channel, 0U, (uint16_t)offCount);
}

HAL_StatusTypeDef PCA9685_SetServoAngle(I2C_HandleTypeDef *hi2c,
                                         uint8_t addr7bit,
                                         uint8_t channel,
                                         uint16_t angleDeg,
                                         uint16_t minPulseUs,
                                         uint16_t maxPulseUs,
                                         uint16_t pwmHz)
{
  uint32_t pulseUs;

  if (minPulseUs >= maxPulseUs)
  {
    return HAL_ERROR;
  }

  if (angleDeg > PCA9685_SERVO_ANGLE_MAX_DEG)
  {
    angleDeg = PCA9685_SERVO_ANGLE_MAX_DEG;
  }

  pulseUs = minPulseUs + (((uint32_t)(maxPulseUs - minPulseUs) * angleDeg) / (uint32_t)PCA9685_SERVO_ANGLE_MAX_DEG);

  return PCA9685_SetServoPulseUs(hi2c,
                                  addr7bit,
                                  channel,
                                  (uint16_t)pulseUs,
                                  pwmHz);
}

HAL_StatusTypeDef PCA9685_SetServoSpeed(I2C_HandleTypeDef *hi2c,
                                         uint8_t addr7bit,
                                         uint8_t channel,
                                         int16_t speed)
{
  int32_t pulseUs;

  if (speed > 100)
  {
    speed = 100;
  }
  else if (speed < -100)
  {
    speed = -100;
  }

  /* Continuous rotation mapping:
     speed =  100 -> 500us (forward fastest)
     speed =    0 -> 1500us (stop)
     speed = -100 -> 2500us (reverse fastest) */
  pulseUs = (int32_t)PCA9685_SERVO_STOP_US - (((int32_t)speed * 1000) / 100);

  if (pulseUs < (int32_t)PCA9685_SERVO_MIN_US)
  {
    pulseUs = (int32_t)PCA9685_SERVO_MIN_US;
  }
  else if (pulseUs > (int32_t)PCA9685_SERVO_MAX_US)
  {
    pulseUs = (int32_t)PCA9685_SERVO_MAX_US;
  }

  return PCA9685_SetServoPulseUs(hi2c,
                                  addr7bit,
                                  channel,
                                  (uint16_t)pulseUs,
                                  PCA9685_SERVO_DEFAULT_HZ);
}

HAL_StatusTypeDef PCA9685_Init_Simple(void)
{
  return PCA9685_Init(PCA9685_FIXED_I2C_HANDLE,
                      PCA9685_FIXED_ADDR_7BIT,
                      PCA9685_FIXED_PWM_HZ);
}

HAL_StatusTypeDef PCA9685_SetServoAngle_Simple(uint8_t channel, uint16_t angleDeg)
{
  return PCA9685_SetServoAngle(PCA9685_FIXED_I2C_HANDLE,
                               PCA9685_FIXED_ADDR_7BIT,
                               channel,
                               angleDeg,
                               PCA9685_SG90_MIN_PULSE_US,
                               PCA9685_SG90_MAX_PULSE_US,
                               PCA9685_FIXED_PWM_HZ);
}

HAL_StatusTypeDef PCA9685_SetServoSpeed_Simple(uint8_t channel, int16_t speed)
{
  return PCA9685_SetServoSpeed(PCA9685_FIXED_I2C_HANDLE,
                               PCA9685_FIXED_ADDR_7BIT,
                               channel,
                               speed);
}

HAL_StatusTypeDef PCA9685_SetLED_Simple(uint8_t channel, uint8_t dutyPercent)
{
  return PCA9685_SetDutyPercent(PCA9685_FIXED_I2C_HANDLE,
                                PCA9685_FIXED_ADDR_7BIT,
                                channel,
                                dutyPercent);
}

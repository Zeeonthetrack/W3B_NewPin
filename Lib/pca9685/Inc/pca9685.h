#ifndef __PCA9685_H__
#define __PCA9685_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define PCA9685_DEFAULT_ADDR_7BIT (0x40U)
#define PCA9685_CHANNEL_COUNT      (16U)
#define PCA9685_COUNT_MAX          (4095U)
#define PCA9685_SERVO_MIN_US       (500U)
#define PCA9685_SERVO_STOP_US      (1500U)
#define PCA9685_SERVO_MAX_US       (2500U)
#define PCA9685_SERVO_DEFAULT_HZ   (50U)

HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7bit, uint16_t pwmHz);
HAL_StatusTypeDef PCA9685_SetPwmCounts(I2C_HandleTypeDef *hi2c,
                                        uint8_t addr7bit,
                                        uint8_t channel,
                                        uint16_t onCount,
                                        uint16_t offCount);
HAL_StatusTypeDef PCA9685_SetDutyPercent(I2C_HandleTypeDef *hi2c,
                                          uint8_t addr7bit,
                                          uint8_t channel,
                                          uint8_t dutyPercent);
HAL_StatusTypeDef PCA9685_SetServoPulseUs(I2C_HandleTypeDef *hi2c,
                                           uint8_t addr7bit,
                                           uint8_t channel,
                                           uint16_t pulseUs,
                                           uint16_t pwmHz);
HAL_StatusTypeDef PCA9685_SetServoAngle(I2C_HandleTypeDef *hi2c,
                                         uint8_t addr7bit,
                                         uint8_t channel,
                                         uint16_t angleDeg,
                                         uint16_t minPulseUs,
                                         uint16_t maxPulseUs,
                                         uint16_t pwmHz);
HAL_StatusTypeDef PCA9685_SetServoSpeed(I2C_HandleTypeDef *hi2c,
                                         uint8_t addr7bit,
                                         uint8_t channel,
                                         int16_t speed);

#ifdef __cplusplus
}
#endif

#endif /* __PCA9685_H__ */

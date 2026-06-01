#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include <stdint.h>

/*
 * Motor PWM driver — dual-channel H-bridge control.
 * 电机 PWM 驱动——双通道 H 桥控制。
 *
 * Hardware resources are injected via chassis_config.h (or fallback defaults).
 * 硬件资源通过 chassis_config.h 注入（或使用以下 fallback 默认值）。
 */

/* ---- hardware defaults (override via chassis_config.h) ---- */

#ifndef HW_MOTOR_HTIM
#define HW_MOTOR_HTIM    (&htim4)
#endif
#ifndef HW_MOTOR_L_CH1
#define HW_MOTOR_L_CH1   TIM_CHANNEL_1
#endif
#ifndef HW_MOTOR_L_CH2
#define HW_MOTOR_L_CH2   TIM_CHANNEL_2
#endif
#ifndef HW_MOTOR_R_CH1
#define HW_MOTOR_R_CH1   TIM_CHANNEL_3
#endif
#ifndef HW_MOTOR_R_CH2
#define HW_MOTOR_R_CH2   TIM_CHANNEL_4
#endif
#ifndef MOTOR_LEFT_DIR
#define MOTOR_LEFT_DIR   (-1)
#endif
#ifndef MOTOR_RIGHT_DIR
#define MOTOR_RIGHT_DIR  (-1)
#endif
#ifndef MOTOR_PWM_MAX
#define MOTOR_PWM_MAX    3600
#endif

/* ---- API ---- */

void motor_init(void);
void motor_set_pwm(uint8_t motor_id, int16_t pwm_val);

#endif /* __MOTOR_H */

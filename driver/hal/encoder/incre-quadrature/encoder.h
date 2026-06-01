#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include <stdint.h>

/*
 * Encoder driver — quadrature encoder pulse counting + RPM conversion.
 * 编码器驱动——正交编码器脉冲计数 + 转速换算。
 *
 * Hardware resources are injected via chassis_config.h (or fallback defaults).
 * 硬件资源通过 chassis_config.h 注入（或使用以下 fallback 默认值）。
 */

/* ---- hardware defaults (override via chassis_config.h) ---- */

#ifndef HW_ENC_L_HTIM
#define HW_ENC_L_HTIM      (&htim2)
#endif
#ifndef HW_ENC_L_TIM
#define HW_ENC_L_TIM       TIM2
#endif
#ifndef HW_ENC_R_HTIM
#define HW_ENC_R_HTIM      (&htim3)
#endif
#ifndef HW_ENC_R_TIM
#define HW_ENC_R_TIM       TIM3
#endif
#ifndef ENCODER_LEFT_DIR
#define ENCODER_LEFT_DIR   (-1)
#endif
#ifndef ENCODER_RIGHT_DIR
#define ENCODER_RIGHT_DIR  (1)
#endif
#ifndef ENCODER_PPR
#define ENCODER_PPR         1560.0f
#endif

/* ---- API ---- */

void encoder_init(void);
float encoder_get_rpm(uint8_t motor_id, float dt);
int32_t encoder_get_location(uint8_t motor_id);

#endif /* __ENCODER_H */

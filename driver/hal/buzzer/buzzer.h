#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"

/*
 * Buzzer driver — GPIO on/off control + non-blocking beep state machine.
 * 蜂鸣器驱动——GPIO 开关控制 + 非阻塞蜂鸣状态机。
 *
 * Hardware resources are injected via chassis_config.h (or fallback defaults).
 * 硬件资源通过 chassis_config.h 注入（或使用以下 fallback 默认值）。
 */

/* ---- hardware defaults (override via chassis_config.h) ---- */

#ifndef HW_BUZZER_PORT
#define HW_BUZZER_PORT     GPIOB
#endif
#ifndef HW_BUZZER_PIN
#define HW_BUZZER_PIN      GPIO_PIN_1
#endif
#ifndef HW_BUZZER_ACTIVE
#define HW_BUZZER_ACTIVE   0           // 0 = active-low, 1 = active-high
#endif

/* ---- API ---- */

void buzzer_init(void);
void buzzer_set(uint8_t state);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_beep(uint8_t count, uint16_t duration_ms);
void buzzer_update(void);
uint8_t buzzer_is_busy(void);

#endif

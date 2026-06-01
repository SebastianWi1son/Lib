#ifndef __KEY_H
#define __KEY_H

#include "main.h"
#include <stdint.h>

/*
 * Key input driver — debounced button read with release-edge detection.
 * 按键驱动——消抖按键读取，松手触发检测。
 *
 * key_tick() must be called from a 1 ms periodic ISR (SysTick).
 * key_tick() 必须放在 1ms 定时中断 (SysTick) 中调用。
 *
 * Hardware resources are injected via chassis_config.h (or fallback defaults).
 * 硬件资源通过 chassis_config.h 注入（或使用以下 fallback 默认值）。
 */

/* ---- hardware defaults (override via chassis_config.h) ---- */

#ifndef HW_KEY1_PORT
#define HW_KEY1_PORT       GPIOB
#endif
#ifndef HW_KEY1_PIN
#define HW_KEY1_PIN        GPIO_PIN_3
#endif
#ifndef HW_KEY2_PORT
#define HW_KEY2_PORT       GPIOB
#endif
#ifndef HW_KEY2_PIN
#define HW_KEY2_PIN        GPIO_PIN_4
#endif
#ifndef HW_KEY_DEBOUNCE_MS
#define HW_KEY_DEBOUNCE_MS 20
#endif

/* ---- API ---- */

void key_init(void);
uint8_t key_get_num(void);
void key_tick(void);

#endif

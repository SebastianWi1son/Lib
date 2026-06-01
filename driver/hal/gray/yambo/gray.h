#ifndef __GRAY_H
#define __GRAY_H

#include "main.h"
#include <stdint.h>

/*
 * Gray-scale line sensor driver — 4051 multiplexer + comparator readout.
 * 灰度传感器驱动——4051 多路复用器 + 比较器读取。
 *
 * Uses direct GPIO register access (BSRR/IDR) for multiplexer bit-bang speed.
 * Porting to non-STM32: replace the register-access macros in gray.c.
 * 使用 GPIO 寄存器直接操作以保证多路复用器切换速度。
 * 移植到非 STM32 平台：替换 gray.c 中的寄存器操作宏。
 *
 * Hardware resources are injected via chassis_config.h (or fallback defaults).
 * 硬件资源通过 chassis_config.h 注入（或使用以下 fallback 默认值）。
 */

/* ---- hardware defaults (override via chassis_config.h) ---- */

#ifndef HW_GRAY_AD0_PORT
#define HW_GRAY_AD0_PORT   GPIOA
#endif
#ifndef HW_GRAY_AD0_PIN
#define HW_GRAY_AD0_PIN    GPIO_PIN_3
#endif
#ifndef HW_GRAY_AD1_PORT
#define HW_GRAY_AD1_PORT   GPIOA
#endif
#ifndef HW_GRAY_AD1_PIN
#define HW_GRAY_AD1_PIN    GPIO_PIN_4
#endif
#ifndef HW_GRAY_AD2_PORT
#define HW_GRAY_AD2_PORT   GPIOA
#endif
#ifndef HW_GRAY_AD2_PIN
#define HW_GRAY_AD2_PIN    GPIO_PIN_5
#endif
#ifndef HW_GRAY_OUT_PORT
#define HW_GRAY_OUT_PORT   GPIOA
#endif
#ifndef HW_GRAY_OUT_PIN
#define HW_GRAY_OUT_PIN    GPIO_PIN_2
#endif
#ifndef HW_GRAY_MUX_DELAY
#define HW_GRAY_MUX_DELAY  200
#endif

/* ---- API ---- */

uint8_t gray_get_data(void);

#endif /* __GRAY_H */

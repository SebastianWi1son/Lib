#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include <stdint.h>

/*
 * OLED SSD1306 driver — software I2C bit-bang, 128x64 monochrome.
 * OLED SSD1306 驱动——软件 I2C 模拟，128x64 单色显示。
 *
 * Hardware resources are injected via chassis_config.h (or fallback defaults).
 * 硬件资源通过 chassis_config.h 注入（或使用以下 fallback 默认值）。
 */

/* ---- hardware defaults (override via chassis_config.h) ---- */

#ifndef HW_OLED_SCL_PORT
#define HW_OLED_SCL_PORT   GPIOA
#endif
#ifndef HW_OLED_SCL_PIN
#define HW_OLED_SCL_PIN    GPIO_PIN_8
#endif
#ifndef HW_OLED_SDA_PORT
#define HW_OLED_SDA_PORT   GPIOB
#endif
#ifndef HW_OLED_SDA_PIN
#define HW_OLED_SDA_PIN    GPIO_PIN_5
#endif
#ifndef HW_OLED_ADDR
#define HW_OLED_ADDR       0x78
#endif

/* ---- API ---- */

void oled_init(void);
void oled_clear(void);
void oled_show_string(uint8_t x, uint8_t y, const char *str);
void oled_printf(uint8_t x, uint8_t y, const char *fmt, ...);

#endif

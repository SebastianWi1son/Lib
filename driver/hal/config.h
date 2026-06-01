#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#include "main.h"
#include "usart.h"

/*
 * Chassis Hardware Configuration — single source of truth.
 * 底盘硬件配置文件——唯一的硬件资源定义入口。
 *
 * Include this ONCE in main.c BEFORE any driver header.
 * All HW_ macros defined here take priority over driver-internal fallbacks.
 * 在 main.c 中所有 driver 头文件之前 include 本文件一次即可。
 * 此处定义的所有 HW_ 宏优先级高于驱动内部的 fallback 默认值。
 */

/* ==================================================================== */
/* === 1. Motor — 电机 PWM                                            === */
/* ==================================================================== */
#define HW_MOTOR_HTIM      (&htim4)
#define HW_MOTOR_L_CH1     TIM_CHANNEL_1   // 左轮正转
#define HW_MOTOR_L_CH2     TIM_CHANNEL_2   // 左轮反转
#define HW_MOTOR_R_CH1     TIM_CHANNEL_3   // 右轮正转
#define HW_MOTOR_R_CH2     TIM_CHANNEL_4   // 右轮反转
#define MOTOR_LEFT_DIR     (-1)            // 左轮极性 (1 正向, -1 反向)
#define MOTOR_RIGHT_DIR    (-1)            // 右轮极性

/* ==================================================================== */
/* === 2. Encoder — 编码器                                             === */
/* ==================================================================== */
#define HW_ENC_L_HTIM      (&htim2)        // 左轮编码器定时器句柄
#define HW_ENC_L_TIM       TIM2            // 左轮编码器定时器寄存器
#define HW_ENC_R_HTIM      (&htim3)        // 右轮编码器定时器句柄
#define HW_ENC_R_TIM       TIM3            // 右轮编码器定时器寄存器
#define ENCODER_LEFT_DIR    (-1)           // 左编码器极性
#define ENCODER_RIGHT_DIR   (1)            // 右编码器极性
#define ENCODER_PPR         1560.0f        // 每圈脉冲数 (随电机/编码器更换)

/* ==================================================================== */
/* === 3. Gray Sensor — 灰度传感器 (4051 多路复用器)                  === */
/* ==================================================================== */
#define HW_GRAY_AD0_PORT   GPIOA
#define HW_GRAY_AD0_PIN    GPIO_PIN_3
#define HW_GRAY_AD1_PORT   GPIOA
#define HW_GRAY_AD1_PIN    GPIO_PIN_4
#define HW_GRAY_AD2_PORT   GPIOA
#define HW_GRAY_AD2_PIN    GPIO_PIN_5
#define HW_GRAY_OUT_PORT   GPIOA
#define HW_GRAY_OUT_PIN    GPIO_PIN_2
#define HW_GRAY_MUX_DELAY  200             // 多路复用器建立等待 (循环次数)

/* ==================================================================== */
/* === 4. ICM-42605 / ICM-20948 IMU — 陀螺仪                          === */
/* ==================================================================== */
#define HW_ICM_SPI         (&hspi2)
#define HW_ICM_CS_PORT     ICM_CS_GPIO_Port
#define HW_ICM_CS_PIN      ICM_CS_Pin
#define GYRO_Z_DIR         (1.0f)          // 陀螺 Z 轴极性

/* ==================================================================== */
/* === 5. Buzzer — 蜂鸣器                                              === */
/* ==================================================================== */
#define HW_BUZZER_PORT     GPIOB
#define HW_BUZZER_PIN      GPIO_PIN_1
#define HW_BUZZER_ACTIVE   0               // 0 = 低电平触发, 1 = 高电平触发

/* ==================================================================== */
/* === 6. Keys — 按键                                                  === */
/* ==================================================================== */
#define HW_KEY1_PORT       GPIOB
#define HW_KEY1_PIN        GPIO_PIN_3      // CT_DEBUG
#define HW_KEY2_PORT       GPIOB
#define HW_KEY2_PIN        GPIO_PIN_4      // CR_DEBUG
#define HW_KEY_DEBOUNCE_MS 20              // 消抖周期 (ms)

/* ==================================================================== */
/* === 7. OLED SSD1306 — 软件 I2C                                     === */
/* ==================================================================== */
#define HW_OLED_SCL_PORT   GPIOA
#define HW_OLED_SCL_PIN    GPIO_PIN_8
#define HW_OLED_SDA_PORT   GPIOB
#define HW_OLED_SDA_PIN    GPIO_PIN_5
#define HW_OLED_ADDR       0x78

/* ==================================================================== */
/* === 8. USART / 通信                                                 === */
/* ==================================================================== */
#define HW_GIMBAL_UART     (&huart1)

/* ==================================================================== */
/* === 9. Navigation Polarity — 导航运动学极性 (Algorithm Layer)      === */
/* ==================================================================== */
#define NAV_LINE_DIR       (1.0f)          // 巡线误差极性
#define CHASSIS_STEER_DIR  (1.0f)          // 差速转向极性

#endif /* __CHASSIS_CONFIG_H */

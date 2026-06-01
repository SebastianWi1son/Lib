#ifndef __CURRENT_SENSE_H
#define __CURRENT_SENSE_H

/**
 * @file    current_sense.h
 * @brief   Inline 相电流采样驱动 — 硬件解耦
 *
 * 架构位置: bsp/driver/adc/current_sense/ (与 encoder 同级)
 * 注入方式: foc_hardware_t.get_current_cb = current_sense_get
 *
 * ADC 由 PWM 定时器 TRGO 硬件触发 (中心点采样, 避开开关噪声),
 * DMA 自动搬运结果到 SRAM. 本驱动只读 DMA buffer 最新值, 不做阻塞传输.
 *
 * 使用前需在 STM32CubeMX 中配置:
 *   - ADC1 (或 ADC2) INJ0/INJ1: 两路相电流, 由 TIM1_TRGO 触发
 *   - DMA: ADC 结果 → SRAM buffer (D2 域, 0x30000000 区, 避开 DCache)
 *   - TIM1: Master TRGO → Update Event (PWM 中心点)
 *   - 运放 + 采样电阻: 下桥臂串联 → 差分放大 → ADC 引脚
 */

#include <stdint.h>

/* ================================================================
 * 硬件配置 (根据实际 PCB 修改)
 * ================================================================ */

/* 采样电阻 (Ω) — 典型值 0.005~0.02Ω */
#define CURR_SENSE_SHUNT_RES   0.01f

/* 运放增益 (V/V) — 差分放大倍数, 典型值 20~100 */
#define CURR_SENSE_AMP_GAIN    50.0f

/* ADC 参考电压 (V) — STM32H7 通常 3.3V */
#define CURR_SENSE_VREF        3.3f

/* ADC 分辨率 — 16-bit = 65536, 12-bit = 4096 */
#define CURR_SENSE_ADC_RES     65536.0f

/* ADC 读数中点 — 16-bit 时零电流对应 32768 */
#define CURR_SENSE_ADC_MID     32768.0f

/* ── 电参数换算 ── */
/* ADC 每 LSB 对应电压: VREF / ADC_RES */
#define CURR_SENSE_V_PER_LSB   (CURR_SENSE_VREF / CURR_SENSE_ADC_RES)
/* 电流转换系数: V_PER_LSB / (SHUNT * GAIN)  → A/LSB */
#define CURR_SENSE_A_PER_LSB   (CURR_SENSE_V_PER_LSB / (CURR_SENSE_SHUNT_RES * CURR_SENSE_AMP_GAIN))

/* ================================================================
 * API
 * ================================================================ */

/**
 * @brief 初始化电流采样
 *
 * 在 CubeMX 生成的 MX_ADC1_Init() / MX_DMA_Init() 之后调用.
 * 执行:
 *   - 启动 ADC 注入通道的 DMA 传输
 *   - 自动校准零电流偏移 (电机未使能时读取 100 次取平均)
 *
 * 偏移值存入静态变量, 后续 current_sense_get() 自动扣除.
 */
void current_sense_init(void);

/**
 * @brief 读取相电流 Ia, Ib (A) — inline 非阻塞
 *
 * 作为 foc_hardware_t.get_current_cb 的回调.
 * 只读 DMA buffer 最新值, 不发起新的 ADC 转换.
 *
 * Ic 由上层 (foc_current_update) 根据 Ia+Ib+Ic=0 推导.
 *
 * @param ia  [out] A 相电流 (A)
 * @param ib  [out] B 相电流 (A)
 */
void current_sense_get(float *ia, float *ib);

/**
 * @brief 过流标志 — 供 app 层轮询
 *
 * 任一相电流绝对值超过阈值时置 1.
 * 清除: 调用 current_sense_clear_overcurrent().
 *
 * @return 1 = 过流, 0 = 正常
 */
uint8_t current_sense_is_overcurrent(void);
void current_sense_clear_overcurrent(void);

/**
 * @brief 设置过流阈值 (A)
 *
 * @param threshold  相电流幅值上限, 超过此值触发过流标志
 */
void current_sense_set_overcurrent_threshold(float threshold);

#endif /* __CURRENT_SENSE_H */

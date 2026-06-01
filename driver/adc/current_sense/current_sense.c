/**
 * @file    current_sense.c
 * @brief   Inline 相电流采样驱动 — 实现
 *
 * 硬件解耦: 所有外设引用集中在文件顶部的 #define 块.
 * 适配实际硬件时只需修改这些宏, 无需改动函数逻辑.
 *
 * CubeMX 配置要求:
 *   ADC1:
 *     - INJ0: Ia 通道 (对应 GPIO 引脚, 查阅 H743 数据手册 ADC1_INJ0)
 *     - INJ1: Ib 通道
 *     - External Trigger: TIM1_TRGO (PWM 中心点触发)
 *     - Injected Conversion Mode: Enabled
 *   DMA (ADC1):
 *     - Circular mode, Half-Word
 *     - Destination: adc_dma_buffer (D2 SRAM)
 *   TIM1 (PWM):
 *     - Master Mode: Update Event → TRGO
 *
 * 数据流:
 *   TIM1 PWM 中心点 → TRGO → ADC1 注入采样 → DMA → adc_dma_buffer[]
 *                                                    ↓
 *                                        current_sense_get() 只读 buffer
 *
 * 参考: bsp/driver/encoder/as5048a.c (同样的硬件解耦模式)
 */

#include "current_sense.h"
#include <math.h>

/* ================================================================
 * 硬件映射 — 适配时修改这里
 *
 * CubeMX 生成的外设句柄名:
 *   在 Core/Inc/adc.h 中找到 extern ADC_HandleTypeDef hadc1;
 *   在 Core/Inc/dma.h 中找到 extern DMA_HandleTypeDef hdma_adc1;
 *
 * ADC 通道:
 *   在 .ioc 中分配 INJ0 (Ia) 和 INJ1 (Ib) 的 GPIO 引脚后,
 *   确认 Channel 编号 (ADC_CHANNEL_0 ~ ADC_CHANNEL_19)
 * ================================================================ */

#define CURR_SENSE_ADC        hadc1          /* ADC 句柄 */
#define CURR_SENSE_DMA        hdma_adc1      /* DMA 句柄 */

/* 注入通道编号 (在 CubeMX 中配置) */
#define CURR_SENSE_INJ_CH_IA  ADC_INJECTED_RANK_1
#define CURR_SENSE_INJ_CH_IB  ADC_INJECTED_RANK_2

/* DMA buffer — 必须放在 D2 SRAM (0x30000000) 避开 DCache
 *
 * CubeMX 中 DMA 的 Destination Address 配为 (uint32_t)&adc_dma_buffer.
 * 或者使用 CubeMX 默认生成在 main.c 的 buffer, 修改此处指向它即可.
 *
 * Layout: [0]=Ia, [1]=Ib (按 INJ Rank 顺序)
 */
#define CURR_SENSE_DMA_BUF_SIZE  2
static volatile uint16_t adc_dma_buffer[CURR_SENSE_DMA_BUF_SIZE]
    __attribute__((section(".dma_buffer")));  /* 链接脚本中映射到 D2 SRAM */

/* 零电流 ADC 偏移 — 上电校准 */
static float offset_ia = CURR_SENSE_ADC_MID;
static float offset_ib = CURR_SENSE_ADC_MID;

/* 过流保护 */
static float overcurrent_limit = 10.0f;  /* 默认 10A */
static uint8_t overcurrent_flag = 0;

/* ================================================================
 * 初始化
 * ================================================================ */

void current_sense_init(void) {
  /* ── 启动 ADC 注入 DMA (CubeMX 已配置, 这里只需使能) ── */
  /*
   * 实际集成时取消注释:
   *
   *   HAL_ADCEx_InjectedStart_DMA(&CURR_SENSE_ADC,
   *                                adc_dma_buffer,
   *                                CURR_SENSE_DMA_BUF_SIZE,
   *                                CURR_SENSE_INJ_CH_IA,
   *                                CURR_SENSE_INJ_CH_IB);
   *
   * 注意: STM32H7 的 HAL_ADCEx_InjectedStart_DMA 参数签名可能因 HAL 版本而异,
   * 请参考具体 HAL 版本的文档.
   */

  /* ── 零电流偏移自动校准 ──
   * 电机未使能, 相电流应为 0. 采集 N 次取平均. */
  /* 注意: 校准需要 DMA 已启动 (HAL_ADCEx_InjectedStart_DMA 已调用).
   * 当前 HAL 调用被注释, 集成时需取消注释并确保 PWM+ADC 已运行.
   * 忙等延时需根据实际 ADC 采样率校准, 当前值仅作占位. */
  #define CALIB_SAMPLES 200
  float sum_ia = 0.0f, sum_ib = 0.0f;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    sum_ia += (float)adc_dma_buffer[0];
    sum_ib += (float)adc_dma_buffer[1];
    /* 简单的忙等延时 (~100µs 让 ADC 完成一轮采样) */
    for (volatile int d = 0; d < 1000; d++) {}
  }

  offset_ia = sum_ia / (float)CALIB_SAMPLES;
  offset_ib = sum_ib / (float)CALIB_SAMPLES;

  overcurrent_flag = 0;
}

/* ================================================================
 * Inline 读取 — 作为 foc_hardware_t.get_current_cb
 *
 * 只读 DMA buffer 最新值, 不发起 ADC 转换, 不阻塞.
 * ADC 由 TIM1_TRGO 硬件自动触发, DMA 自动搬运 → 完全硬件解耦.
 * ================================================================ */

void current_sense_get(float *ia, float *ib) {
  /* 读取 DMA buffer 最新值 (非阻塞, <1µs) */
  uint16_t raw_ia = adc_dma_buffer[0];
  uint16_t raw_ib = adc_dma_buffer[1];

  /* ADC 原始值 → 安培 */
  *ia = ((float)raw_ia - offset_ia) * CURR_SENSE_A_PER_LSB;
  *ib = ((float)raw_ib - offset_ib) * CURR_SENSE_A_PER_LSB;

  /* 过流检测 */
  if (fabsf(*ia) > overcurrent_limit || fabsf(*ib) > overcurrent_limit) {
    overcurrent_flag = 1;
  }
}

/* ================================================================
 * 过流检测
 * ================================================================ */

uint8_t current_sense_is_overcurrent(void) {
  return overcurrent_flag;
}

void current_sense_clear_overcurrent(void) {
  overcurrent_flag = 0;
}

void current_sense_set_overcurrent_threshold(float threshold) {
  if (threshold > 0.0f)
    overcurrent_limit = threshold;
}

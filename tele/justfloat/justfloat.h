#ifndef __JUSTFLOAT_H
#define __JUSTFLOAT_H

/**
 * @file    justfloat.h
 * @brief   JustFloat JustFloat 协议层 — 硬件解耦, 可移植
 *
 * 本文件是 bsp/middleware/telemetry/justfloat.h 的重构版.
 * 关键变化:
 *   - 不再硬编码 UART 句柄 (&huart4) — 通过 justfloat_port_t 注入
 *   - 不再硬编码 DMA buffer 地址 — 通过 justfloat_set_*_buffer() 注入
 *   - 不包含任何 HAL 头文件
 *   - 命令字符串名统一由 motor_param bounds_table 提供 (tele_parser 层)
 *
 * 集成方式:
 *   1. 创建 justfloat_port_t, 填入 STM32 端口函数 (见 justfloat_port_stm32.c)
 *   2. justfloat_init(&port)
 *   3. 分配 TX/RX buffer (D2 SRAM, 避开 DCache)
 *   4. ISR 中: justfloat_rx_feed() 喂入接收数据
 *   5. main loop:  justfloat_send() 发送遥测 / justfloat_cmd_ready() 处理命令
 */

#include <stdint.h>

/* ================================================================
 * 端口注入接口 — 替代 HAL 直接调用
 * ================================================================ */

typedef struct {
  /* TX: 启动 DMA 发送, data 为字节数组, len 为字节数 */
  void (*tx_dma_start)(const uint8_t *data, uint16_t len);

  /* RX: 启动 DMA 循环接收, buf 为接收缓冲, len 为缓冲大小 */
  void (*rx_dma_start)(uint8_t *buf, uint16_t len);

  /* RX: 查询 DMA 已接收字节数 (用于 IDLE 中断后处理) */
  uint16_t (*rx_dma_get_count)(void);

  /* RX: 错误恢复 — 中止 DMA, 清除标志, 重新启动 */
  void (*rx_error_recover)(void);

  /* 获取毫秒时间戳 — 用于遥测发送节流 */
  uint32_t (*get_tick_ms)(void);
} justfloat_port_t;

/* ================================================================
 * 协议 API
 * ================================================================ */

/* 初始化 — 注入端口, 启动 RX DMA */
void justfloat_init(justfloat_port_t *port);

/* 设置 TX/RX 缓冲区 (替代硬编码地址) */
void justfloat_set_tx_buf(float *buf);
void justfloat_set_rx_buf(char *buf, uint16_t max_len);

/* ── TX (遥测发送) ── */

/* 以 JustFloat 格式发送 float 数组.
 * interval_ms: 最小发送间隔 (节流), data: float 数组, count: float 个数 */
void justfloat_send(uint16_t interval_ms, const float *data, uint16_t count);

/* ── RX (命令接收) — ISR 回调 ── */

/* ISR 中调用: 将收到的原始字节拷入 RX buffer, 设 cmd_ready */
void justfloat_rx_feed(uint16_t size);

/* ── RX (命令读取) — main loop 调用 ── */

/* 是否有新命令到达 */
uint8_t justfloat_cmd_ready(void);

/* 获取命令缓冲 (以 '\0' 结尾的字符串) */
const char *justfloat_cmd_buf(void);

/* 清除命令标志 (处理完命令后调用) */
void justfloat_flush(void);

#endif /* __JUSTFLOAT_H */

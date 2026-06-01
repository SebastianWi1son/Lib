/**
 * @file    justfloat.c
 * @brief   JustFloat JustFloat 协议实现 — 纯协议层, 硬件解耦
 *
 * 所有硬件操作通过 justfloat_port_t 函数指针, 不直接调用 HAL.
 * 可在任何有 DMA UART 的 MCU 上使用, 只需提供端口实现.
 */

#include "justfloat.h"
#include <string.h>
#include <stdlib.h>

/* ================================================================
 * 内部状态
 * ================================================================ */

static justfloat_port_t *port = NULL;

/* 缓冲区 (由上层通过 justfloat_set_*_buffer 注入) */
static float  *tx_buffer = NULL;
static char   *rx_buffer = NULL;
static uint16_t rx_buf_max = 0;

/* 命令就绪标志 */
static volatile uint8_t cmd_ready = 0;

/* TX 节流 */
static uint32_t last_tx_tick = 0;

/* JustFloat 帧尾: IEEE 754 正无穷 = 0x7F800000 */
static const uint32_t JUSTFLOAT_TAIL = 0x7F800000;

/* ================================================================
 * 初始化
 * ================================================================ */

void justfloat_init(justfloat_port_t *p) {
  port = p;
  last_tx_tick = 0;
  cmd_ready = 0;
}

void justfloat_set_tx_buf(float *buf) {
  tx_buffer = buf;
}

void justfloat_set_rx_buf(char *buf, uint16_t max_len) {
  rx_buffer = buf;
  rx_buf_max = max_len;

  /* 启动 DMA 循环接收 */
  if (port && rx_buffer)
    port->rx_dma_start((uint8_t *)rx_buffer, rx_buf_max);
}

/* ================================================================
 * TX — JustFloat 遥测发送
 * ================================================================ */

void justfloat_send(uint16_t interval_ms, const float *data, uint16_t count) {
  if (!port || !tx_buffer || !data) return;
  if (count == 0) return;

  /* 节流 */
  uint32_t now = port->get_tick_ms();
  if (now - last_tx_tick < interval_ms) return;
  last_tx_tick = now;

  /* 组装 JustFloat 帧: N 个 float + 1 个 tail marker */
  uint16_t float_bytes = count * sizeof(float);
  memcpy(tx_buffer, data, float_bytes);

  /* 追加 tail (IEEE 754 +Inf) */
  uint8_t *tail_ptr = ((uint8_t *)tx_buffer) + float_bytes;
  memcpy(tail_ptr, &JUSTFLOAT_TAIL, sizeof(JUSTFLOAT_TAIL));

  /* DMA 发送 */
  uint16_t total_bytes = float_bytes + sizeof(JUSTFLOAT_TAIL);
  port->tx_dma_start((const uint8_t *)tx_buffer, total_bytes);
}

/* ================================================================
 * RX — 命令接收 (ISR 回调)
 * ================================================================ */

void justfloat_rx_feed(uint16_t size) {
  if (!rx_buffer) return;

  /* 拷贝到接收处理缓冲 (避免 DMA 正在写入时解析) */
  /* 注意: rx_buffer 同时用作 DMA 目标和命令缓冲.
   * 实际使用时应分配独立的 cmd_parse_buffer.
   * 这里简化: 假设接收完成时 DMA 已停止, 直接加 '\0' 即可. */

  if (size >= rx_buf_max) size = rx_buf_max - 1;
  rx_buffer[size] = '\0';
  /* 命令就绪 — 下一轮 main loop 中解析 */
  cmd_ready = 1;
}

/* ================================================================
 * RX — 命令读取 (main loop 调用)
 * ================================================================ */

uint8_t justfloat_cmd_ready(void) {
  return cmd_ready;
}

const char *justfloat_cmd_buf(void) {
  return rx_buffer;
}

void justfloat_flush(void) {
  cmd_ready = 0;

  /* 重新启动 DMA 接收 */
  if (port && rx_buffer)
    port->rx_dma_start((uint8_t *)rx_buffer, rx_buf_max);
}

/**
 * @file    justfloat.c
 * @brief   JustFloat 遥测协议 — 纯可移植实现
 *
 * 本文件包含零个平台相关调用. 所有硬件操作通过 justfloat_port_t 回调.
 *
 * 依赖: <string.h> (memcpy), <stdint.h> (uint*_t)
 *   不依赖: HAL, FreeRTOS, CMSIS, 任何 vendor SDK
 *
 * 许可: MIT
 */

#include "justfloat.h"
#include <string.h>

/* ================================================================
 * 内部状态 (static — 单例)
 * ================================================================ */

static justfloat_port_t port;          /* 值拷贝, 不持有外部指针 */
static uint8_t          port_ok = 0;   /* justfloat_init 已调用 */

/* 缓冲区指针 */
static float   *tx_buf     = NULL;
static char    *rx_buf     = NULL;
static uint16_t rx_buf_cap = 0;

/* 命令就绪标志 (ISR 写, 主循环读 → volatile 保证可见性) */
static volatile uint8_t cmd_ready = 0;

/* TX 节流: 上次发送的 tick 值 */
static uint32_t tx_last_tick = 0;

/* JustFloat 帧尾: IEEE 754 正无穷 = 0x7F800000 (小端) */
static const uint32_t JF_TAIL = 0x7F800000;

/* ── 安全上限 ── */
#define JF_MAX_CHANNELS  30u   /* 单帧最多 30 个 float (120 字节 payload) */
#define JF_RX_BUF_MIN    32u   /* RX 缓冲区最小尺寸 */

/* ================================================================
 * 初始化
 * ================================================================ */

void justfloat_init(const justfloat_port_t *p) {
  if (!p) return;

  port    = *p;          /* 值拷贝 — 调用方可安全销毁局部变量 */
  port_ok = 1;

  tx_last_tick = 0;
  cmd_ready    = 0;
}

void justfloat_set_tx_buf(float *buf) {
  tx_buf = buf;
}

void justfloat_set_rx_buf(char *buf, uint16_t max_len) {
  rx_buf     = buf;
  rx_buf_cap = (max_len >= JF_RX_BUF_MIN) ? max_len : JF_RX_BUF_MIN;

  /* 启动 DMA 接收 */
  if (port_ok && rx_buf) {
    port.rx_start((uint8_t *)rx_buf, rx_buf_cap);
  }
}

/* ================================================================
 * TX — 遥测发送
 * ================================================================ */

void justfloat_send(uint16_t interval_ms, const float *data, uint16_t count) {
  if (!port_ok || !tx_buf || !data) return;
  if (count == 0 || count > JF_MAX_CHANNELS) return;

  /* ── 节流 ── */
  if (interval_ms > 0) {
    uint32_t now = port.get_tick_ms();
    if (now - tx_last_tick < interval_ms) return;
    tx_last_tick = now;
  }

  /* ── 组装 JustFloat 帧 ──
   *
   * tx_buf 布局: [float_0] [float_1] ... [float_{N-1}] [0x7F800000]
   *
   * 如果 tx_buf 位于 cacheable 内存, 调用方必须在 justfloat_set_tx_buf()
   * 之前通过 MPU 将其设为 non-cacheable, 或在 memcpy 之后手动
   * SCB_CleanDCache_by_Addr(). 本层不做 cache 操作 — 这是 BSP 的职责.
   */
  uint16_t data_bytes = (uint16_t)(count * sizeof(float));
  memcpy(tx_buf, data, data_bytes);

  /* 追加 tail marker */
  uint8_t *p = ((uint8_t *)tx_buf) + data_bytes;
  memcpy(p, &JF_TAIL, sizeof(JF_TAIL));

  /* DMA 发送 — port->tx_start 内部检查 UART 状态,
   * 如果忙碌则静默丢弃本帧 (遥测容忍丢帧) */
  uint16_t total = data_bytes + (uint16_t)sizeof(JF_TAIL);
  port.tx_start((const uint8_t *)tx_buf, total);
}

/* ================================================================
 * RX — 命令接收 (ISR → 喂入)
 * ================================================================ */

void justfloat_rx_feed(uint16_t size) {
  if (!rx_buf || rx_buf_cap == 0) return;

  /* 安全截断: 为 '\0' 保留 1 字节 */
  if (size >= rx_buf_cap) {
    size = rx_buf_cap - 1;
  }

  /* 0 字节接收可能是噪声或线路空闲 — 不处理 */
  if (size == 0) return;

  /* '\0' 结尾 — 保证字符串操作安全 */
  rx_buf[size] = '\0';

  /* 通知主循环 */
  cmd_ready = 1;

  /*
   * 立即重启 DMA 接收 — 对标 vofa.c tele_rx_process() 的行为.
   * 不在 rx_feed → flush 之间留空窗期, 确保下一帧命令不会丢失.
   *
   * port->rx_start() 内部会:
   *   1. 注册缓冲区 (更新保存的 buf/len)
   *   2. 清除 Overrun 标志
   *   3. 启动 HAL_UARTEx_ReceiveToIdle_DMA
   */
  if (port_ok) {
    port.rx_start((uint8_t *)rx_buf, rx_buf_cap);
  }
}

/* ================================================================
 * RX — 命令读取 (主循环 → 消费)
 * ================================================================ */

uint8_t justfloat_cmd_ready(void) {
  return cmd_ready;
}

const char *justfloat_cmd_buf(void) {
  return rx_buf;
}

void justfloat_flush(void) {
  cmd_ready = 0;
  /* DMA 已在 rx_feed() 中重启, 不需要再碰硬件 */
}

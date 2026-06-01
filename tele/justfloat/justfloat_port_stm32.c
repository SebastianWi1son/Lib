/**
 * @file    justfloat_port_stm32.c
 * @brief   JustFloat 端口 — STM32 实现, 复用 uart_comm_port 抽象
 *
 * 将 uart_comm_port 的函数适配为 justfloat_port_t 签名.
 * justfloat_port_t 函数无 port_handle 参数, 这里通过静态闭包存储.
 *
 * 这个文件是 JustFloat 协议和 STM32 HAL 之间唯一的耦合点.
 * 移植到其他 MCU 时只需复制此文件并按 port 接口重新实现.
 */

#include "justfloat.h"
#include "uart_comm_port.h"
#include "main.h"           /* HAL_GetTick, UART_HandleTypeDef */

/* ================================================================
 * 闭包 — 存储 UART 句柄和缓冲信息
 * ================================================================ */

static void  *tele_uart  = NULL;
static uint8_t *tele_rx_buf = NULL;
static uint16_t tele_rx_size = 0;

/* ================================================================
 * 包装函数 — 匹配 justfloat_port_t 签名
 * ================================================================ */

static void tele_tx_dma_start(const uint8_t *data, uint16_t len) {
  if (tele_uart)
    port_dma_tx_start(tele_uart, data, len);
}

static void tele_rx_dma_start(uint8_t *buf, uint16_t len) {
  tele_rx_buf  = buf;
  tele_rx_size = len;
  if (tele_uart)
    port_dma_rx_setup(tele_uart, buf, len);
}

static uint16_t tele_rx_dma_get_count(void) {
  if (tele_uart)
    return port_dma_rx_pos(tele_uart, tele_rx_size);
  return 0;
}

static void tele_rx_error_recover(void) {
  if (tele_uart && tele_rx_buf)
    port_rx_error_recover(tele_uart, tele_rx_buf, tele_rx_size);
}

/* ================================================================
 * 公开 API
 * ================================================================ */

/**
 * @brief 创建绑定到指定 UART 的 justfloat_port_t
 *
 * @param huart  要绑定的 UART 句柄 (如 &huart4)
 * @return       填入 justfloat_port_t 字段, 可直接传给 justfloat_init()
 *
 * 用法:
 *   justfloat_port_t port = justfloat_port_create(&huart4);
 *   justfloat_init(&port);
 */
justfloat_port_t justfloat_port_create(void *huart) {
  tele_uart    = huart;
  tele_rx_buf  = NULL;
  tele_rx_size = 0;

  justfloat_port_t p = {
    .tx_dma_start     = tele_tx_dma_start,
    .rx_dma_start     = tele_rx_dma_start,
    .rx_dma_get_count = tele_rx_dma_get_count,
    .rx_error_recover = tele_rx_error_recover,
    .get_tick_ms      = HAL_GetTick,   /* STM32 专有, 其他平台替换 */
  };
  return p;
}

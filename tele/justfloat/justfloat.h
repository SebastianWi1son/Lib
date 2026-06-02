/**
 * @file    justfloat.h
 * @brief   JustFloat 遥测协议 — 纯可移植协议层, 零硬件依赖
 *
 * ## 协议说明
 *
 * JustFloat (Vofa+ / 伏特加兼容) 是一种极简浮点遥测协议:
 *   帧格式: [float_0] [float_1] ... [float_N] [0x7F800000]
 *   每个元素是 IEEE 754 单精度浮点 (4 字节, 小端).
 *   帧尾 0x7F800000 是 IEEE 754 正无穷的位表示, 用作帧同步标记.
 *
 * ## 架构
 *
 *   ┌────────────────────────────────────────────┐
 *   │  justfloat.h / justfloat.c  (本文件)        │
 *   │  纯 C99, 零依赖. 不包含任何 HAL/OS 头文件.   │
 *   │  所有硬件操作通过 justfloat_port_t 回调.     │
 *   └──────────────┬─────────────────────────────┘
 *                  │ justfloat_port_t (4 个函数指针)
 *   ┌──────────────┴─────────────────────────────┐
 *   │  bsp/tele/justfloat_port_<mcu>.c            │
 *   │  板级实现: STM32H7 / STM32F4 / ESP32 / ... │
 *   │  这是唯一包含 HAL 头文件的编译单元.          │
 *   └────────────────────────────────────────────┘
 *
 * ## 集成步骤
 *
 *   1. 实现 justfloat_port_t 的 4 个函数 (参考 bsp/tele/justfloat_port_stm32.c)
 *   2. 分配 TX/RX 缓冲区 — 必须位于 DMA 可访问的 non-cacheable 内存
 *   3. justfloat_init(&port) → justfloat_set_tx_buf / set_rx_buf
 *   4. ISR 回调中调用 justfloat_rx_feed(size)
 *   5. main loop 中调用 justfloat_send() 发送遥测
 *
 * ## 缓冲区要求
 *
 *   TX buffer: (max_channels + 1) × sizeof(float) 字节
 *   RX buffer: 命令字符串最大长度 (建议 128~256 字节)
 *   两个 buffer 必须满足:
 *     - DMA 可访问 (STM32H7: D2/D3 SRAM, 不能是 DTCM)
 *     - Non-cacheable, 或每次访问前手动 cache clean/invalidate
 *     - 32 字节对齐 (推荐, 避免 cache line 撕裂)
 *
 * ## 线程安全
 *
 *   - justfloat_send():     主循环调用, 不可重入
 *   - justfloat_rx_feed():  ISR 上下文调用, 与 send 共享 cmd_ready 标志
 *   - justfloat_cmd_ready / justfloat_flush: 主循环调用
 *   - cmd_ready 使用 volatile 保证单生产者-单消费者可见性
 *
 * 许可: MIT
 */

#ifndef __JUSTFLOAT_H
#define __JUSTFLOAT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 硬件抽象接口 (Port Interface)
 *
 * 移植到新平台只需实现这 4 个函数.
 * 所有函数在调用时保证 port 已通过 justfloat_init() 注册.
 * ================================================================ */

typedef struct {
  /* ── TX: 启动异步 DMA 发送 ──
   *
   * 非阻塞. data 指向待发送数据的原始字节, len 为字节数.
   * data 在发送完成前必须保持有效 (调用方保证).
   *
   * 实现参考 (STM32):
   *   if (huart->gState == HAL_UART_STATE_READY)
   *       HAL_UART_Transmit_DMA(huart, (uint8_t *)data, len);
   */
  void (*tx_start)(const uint8_t *data, uint16_t len);

  /* ── RX: 启动 DMA 接收 (注册缓冲区 + 开始接收) ──
   *
   * buf   — 接收缓冲区 (DMA 将把收到的字节写入此处)
   * len   — 缓冲区大小 (字节)
   *
   * 实现必须:
   *   1. 保存 buf/len 用于后续 rx_restart() 恢复
   *   2. 启动带 IDLE 检测的 DMA 接收 (STM32: HAL_UARTEx_ReceiveToIdle_DMA)
   *   3. 清除 Overrun 标志 (防止启动时的残留错误)
   *
   * 由 justfloat_set_rx_buf() 和 justfloat_rx_feed() 内部调用.
   */
  void (*rx_start)(uint8_t *buf, uint16_t len);

  /* ── RX: 重启接收 (错误恢复 / TX 完成后恢复) ──
   *
   * 中止当前 DMA 接收, 清除所有 UART 错误标志,
   * 使用上次 rx_start() 保存的 buf/len 重新启动 DMA.
   *
   * 由 ISR 回调中调用 (HAL_UART_ErrorCallback / HAL_UART_TxCpltCallback).
   * 如果 buf/len 未设置 (rx_start 未被调用过), 此函数应安全返回.
   */
  void (*rx_restart)(void);

  /* ── 时间戳: 获取单调递增的毫秒计数器 ──
   *
   * 用于 TX 节流, 不需要精确到 ms (误差 ±1ms 可接受).
   * 典型实现: HAL_GetTick() / xTaskGetTickCount() / millis()
   */
  uint32_t (*get_tick_ms)(void);

} justfloat_port_t;

/* ================================================================
 * 公共 API
 * ================================================================ */

/* ── 初始化 ── */

/**
 * @brief 注册端口实现, 初始化协议状态
 * @param p  指向调用方填充的 justfloat_port_t (内部值拷贝, 调用后可销毁)
 */
void justfloat_init(const justfloat_port_t *p);

/**
 * @brief 设置 TX 缓冲区
 * @param buf  (max_channels + 1) × 4 字节, DMA 可访问, non-cacheable
 *
 * 必须在 justfloat_init() 之后, justfloat_send() 之前调用.
 */
void justfloat_set_tx_buf(float *buf);

/**
 * @brief 设置 RX 缓冲区并启动 DMA 接收
 * @param buf      接收缓冲区, DMA 可访问, non-cacheable
 * @param max_len  缓冲区大小 (字节), 建议 128~256
 *
 * 内部调用 port->rx_start() 启动 DMA 接收.
 * 必须在 justfloat_init() 之后调用.
 */
void justfloat_set_rx_buf(char *buf, uint16_t max_len);

/* ── TX (遥测发送, 主循环上下文) ── */

/**
 * @brief 以 JustFloat 格式发送遥测数据
 *
 * @param interval_ms  最小发送间隔 (节流), 0 = 每次调用都发送
 * @param data         float 数组, N 个通道
 * @param count        通道数 (float 个数), 最大建议 30
 *
 * 内部流程:
 *   1. 节流检查 (距上次发送 < interval_ms 则跳过)
 *   2. memcpy(data) → tx_buffer
 *   3. 追加 0x7F800000 tail marker
 *   4. port->tx_start() 启动 DMA 发送
 *
 * data 可以是栈上的临时数组, 内部会 memcpy 到 DMA 缓冲区.
 * 如果上一次 DMA 发送未完成 (port->tx_start 内部检查 gState),
 * 本次发送被静默丢弃 (对于遥测数据, 丢帧优于排队).
 */
void justfloat_send(uint16_t interval_ms, const float *data, uint16_t count);

/* ── RX (命令接收, ISR 上下文) ── */

/**
 * @brief 喂入接收到的命令数据 (ISR 回调中调用)
 *
 * @param size  DMA 收到的字节数 (来自 UART IDLE 中断的 Size 参数)
 *
 * 内部流程:
 *   1. 安全截断 + '\0' 结尾 (保证字符串安全)
 *   2. 设置 cmd_ready 标志 (主循环通过 justfloat_cmd_ready() 检查)
 *   3. 立即调用 port->rx_start() 重新启动 DMA 接收
 *
 * 对标 vofa.c 的 tele_rx_process(): 收到数据后立刻重启 DMA,
 * 不在 rx_feed → flush 之间留窗口.
 */
void justfloat_rx_feed(uint16_t size);

/* ── RX (命令读取, 主循环上下文) ── */

/**
 * @brief 检查是否有待处理的命令
 * @return 0 = 无命令, 非 0 = 有命令 (可通过 justfloat_cmd_buf() 获取)
 */
uint8_t justfloat_cmd_ready(void);

/**
 * @brief 获取命令缓冲区
 * @return 指向 '\0' 结尾的命令字符串的指针
 *         仅在 justfloat_cmd_ready() 返回非 0 时有效
 */
const char *justfloat_cmd_buf(void);

/**
 * @brief 清除命令就绪标志
 *
 * 主循环处理完命令后调用. 对标 vofa.c 的 tele_flush_buffer().
 * 注意: RX DMA 已在 rx_feed() 中重启, 本函数不操作硬件.
 */
void justfloat_flush(void);

#ifdef __cplusplus
}
#endif

#endif /* __JUSTFLOAT_H */

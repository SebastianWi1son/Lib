/**
 * @file    uart_comm.h
 * @brief   Universal UART Communication Transceiver — Public API
 *
 * Portable non-blocking UART transceiver with DMA circular RX and TX ring buffer.
 *
 * Platform coupling: NONE in this header.
 * Platform-specific DMA/UART calls are isolated in uart_comm_port_xxx.c.
 * The only platform-dependent type passed through is port_handle (void*).
 *
 * Protocol frame format:
 *   [0xAA] [payload[0..N-1]] [checksum] [0x55]
 * Checksum = sum of payload bytes, truncated to uint8_t.
 *
 * Integration:
 *   1. Provide buffers (rx: power-of-2 size, tx: any size) in non-cacheable SRAM.
 *   2. Fill uart_comm_cfg_t with port_handle (= &huart1 on STM32) + buffers + payload_len.
 *   3. Call uart_comm_init().
 *   4. In main loop: uart_comm_poll().
 *   5. To receive: uart_comm_get_packet().
 *   6. To send: uart_comm_send_packet() (non-blocking).
 *   7. In ISR: call uart_comm_tx_cplt_handler() from platform UART TX complete ISR.
 *   8. In ISR: call uart_comm_error_handler() from platform UART error ISR.
 */

#ifndef __UART_COMM_H
#define __UART_COMM_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 * Platform detection (for buffer alignment hints only)
 * ================================================================ */

#if defined(STM32H743xx) || defined(STM32H750xx)
  #define UART_COMM_DMA_ALIGN    32    /* H7 cache line */
#elif defined(STM32F1xx)
  #define UART_COMM_DMA_ALIGN    4     /* F1: word alignment is enough */
#else
  #define UART_COMM_DMA_ALIGN    4     /* F4, G4, unknown */
#endif

/* ================================================================
 * Constants
 * ================================================================ */

#define UART_COMM_MAX_PAYLOAD    16    /**< Maximum payload length per frame */
#define UART_COMM_FRAME_OVERHEAD 3     /**< 0xAA + checksum + 0x55 */
#define UART_COMM_ERROR_THRESHOLD 5    /**< Consecutive checksum failures before reset */

/* ================================================================
 * Types
 * ================================================================ */

/** Heartbeat configuration */
typedef struct {
    uint8_t   enabled;
    uint16_t  interval_ms;
    uint8_t   payload[8];
    uint8_t   payload_len;
} uart_comm_heartbeat_cfg_t;

/**
 * @brief Instance configuration — caller-provided, HAL-free.
 *
 * port_handle: on STM32, pass &huart1 / &huart2 etc.
 *              on other platforms, pass whatever the port file expects.
 */
typedef struct {
    void    *port_handle;        /**< Platform UART/DMA handle (e.g. &huart1) */
    uint8_t *rx_buf;             /**< DMA circular RX buffer (caller alloc, power-of-2 size) */
    uint16_t rx_buf_size;        /**< Must be power of 2 (256, 512, 1024) */
    uint8_t *tx_buf;             /**< TX ring buffer (caller alloc) */
    uint16_t tx_buf_size;        /**< TX ring capacity in bytes */
    uint8_t  payload_len;        /**< Fixed payload length for this channel (1..16) */

    uart_comm_heartbeat_cfg_t heartbeat; /**< Optional heartbeat */

    /** Optional: called from uart_comm_poll() when valid frame received (main loop, not ISR) */
    void (*on_packet)(uint8_t *payload, uint8_t len);
} uart_comm_cfg_t;

/**
 * @brief Instance — caller allocates (static/global), do NOT access fields directly.
 */
typedef struct {
    /* --- Config --- */
    void    *port_handle;
    uint8_t *rx_buf;
    uint16_t rx_buf_size;
    uint8_t *tx_buf;
    uint16_t tx_buf_size;
    uint8_t  payload_len;
    uint8_t  frame_len;
    uart_comm_heartbeat_cfg_t heartbeat;
    void   (*on_packet)(uint8_t *payload, uint8_t len);

    /* --- RX state --- */
    uint16_t  rx_read_pos;
    uint8_t   rx_error_count;
    uint8_t   rx_pkt_buf[UART_COMM_MAX_PAYLOAD];
    uint8_t   rx_pkt_len;
    volatile uint8_t rx_pkt_ready;

    /* --- TX state --- */
    uint16_t  tx_head;
    uint16_t  tx_tail;
    volatile uint8_t tx_dma_busy;
    uint8_t   tx_dma_buf[32];   /**< TX DMA staging buffer */

    /* --- Heartbeat --- */
    uint32_t  hb_last_tick;

    /* --- Stats --- */
    volatile uint32_t stat_rx_frames;
    volatile uint32_t stat_tx_frames;
    volatile uint32_t stat_rx_errors;
} uart_comm_t;

/* ================================================================
 * Public API
 * ================================================================ */

void uart_comm_init(uart_comm_t *c, const uart_comm_cfg_t *cfg);
void uart_comm_poll(uart_comm_t *c);
bool uart_comm_get_packet(uart_comm_t *c, uint8_t *out, uint8_t *len);
void uart_comm_send_packet(uart_comm_t *c, const uint8_t *payload, uint8_t len);
void uart_comm_tx_cplt_handler(uart_comm_t *c, void *port_handle);
void uart_comm_error_handler(uart_comm_t *c, void *port_handle);
void uart_comm_reset(uart_comm_t *c);
void uart_comm_stats(uart_comm_t *c, uint32_t *rx, uint32_t *tx, uint32_t *err);

#endif /* __UART_COMM_H */

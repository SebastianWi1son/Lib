/**
 * @file    uart_comm.c
 * @brief   Universal UART Communication Transceiver — Core Logic
 *
 * This file contains ZERO vendor HAL dependencies.
 * All platform-specific DMA/UART operations go through the port interface
 * defined in uart_comm_port.h.
 *
 * To build for a platform, compile this file together with the matching
 * uart_comm_port_xxx.c (e.g. uart_comm_port_stm32.c for STM32).
 *
 * Architecture:
 *   RX: DMA circular → port_dma_rx_pos() in poll → byte scan →
 *       memcpy-safe extraction → internal pkt_buf → get_packet()
 *   TX: Non-blocking. Frame → internal tx_dma_buf → port_dma_tx_start().
 *       If busy → queue to TX ring → tx_cplt_handler pops next.
 *   Heartbeat: sys_millis() based periodic send in poll().
 *   Error recovery: auto-reset parser after N consecutive checksum failures.
 */

#include "uart_comm.h"
#include "uart_comm_port.h"
#include "sys_time.h"
#include <string.h>

/* ================================================================
 * Internal Helpers — Protocol Framing
 * ================================================================ */

static uint8_t compute_checksum(const uint8_t *payload, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += payload[i];
    return sum;
}

/* ================================================================
 * Internal Helpers — TX Ring Buffer
 * ================================================================ */

static inline uint16_t tx_ring_used(uart_comm_t *c) {
    if (c->tx_head >= c->tx_tail) return c->tx_head - c->tx_tail;
    return c->tx_buf_size - c->tx_tail + c->tx_head;
}

static inline uint16_t tx_ring_free(uart_comm_t *c) {
    return c->tx_buf_size - tx_ring_used(c) - 1;
}

static inline void tx_ring_push_byte(uart_comm_t *c, uint8_t b) {
    c->tx_buf[c->tx_head] = b;
    if (++c->tx_head >= c->tx_buf_size) c->tx_head = 0;
}

static inline uint8_t tx_ring_pop_byte(uart_comm_t *c) {
    uint8_t b = c->tx_buf[c->tx_tail];
    if (++c->tx_tail >= c->tx_buf_size) c->tx_tail = 0;
    return b;
}

static void tx_dma_kick(uart_comm_t *c, const uint8_t *data, uint16_t len) {
    c->tx_dma_busy = 1;
    if (len <= sizeof(c->tx_dma_buf) && data != c->tx_dma_buf) {
        memcpy(c->tx_dma_buf, data, len);
        data = c->tx_dma_buf;
    }
    port_dma_tx_start(c->port_handle, data, len);
}

static void tx_dequeue_and_send(uart_comm_t *c) {
    if (tx_ring_used(c) < c->frame_len) { c->tx_dma_busy = 0; return; }
    for (uint8_t i = 0; i < c->frame_len; i++)
        c->tx_dma_buf[i] = tx_ring_pop_byte(c);
    port_dma_tx_start(c->port_handle, c->tx_dma_buf, c->frame_len);
    c->tx_dma_busy = 1;
}

/* ================================================================
 * Internal Helpers — RX Frame Scanner
 * ================================================================ */

/**
 * @brief Scan a contiguous buffer region for valid frames.
 *
 * Bytes consumed (up to len). Partial frame at end is left for next poll.
 * Field extraction uses memcpy for Cortex-M3/M4/M7 portability (no unaligned access).
 *
 * @return number of bytes consumed (may be < len if partial frame at end)
 */
static uint16_t scan_rx_region(uart_comm_t *c, const uint8_t *buf, uint16_t len) {
    uint16_t i = 0;
    uint8_t  fl = c->frame_len;

    while (i + fl <= len) {
        /* Header check */
        if (buf[i] != 0xAA) { i++; c->rx_error_count++; continue; }

        /* Footer check (fast rejection) */
        if (buf[i + fl - 1] != 0x55) { i++; c->rx_error_count++; continue; }

        /* Checksum validation */
        uint8_t  rx_cksum = buf[i + fl - 2];
        uint8_t  calc_cksum = 0;
        for (uint8_t j = 0; j < c->payload_len; j++)
            calc_cksum += buf[i + 1 + j];

        if (rx_cksum != calc_cksum) {
            i++;
            c->rx_error_count++;
            c->stat_rx_errors++;
            continue;
        }

        /* Valid frame — copy payload via memcpy (safe on all Cortex-M) */
        c->rx_error_count = 0;
        c->stat_rx_frames++;
        memcpy(c->rx_pkt_buf, &buf[i + 1], c->payload_len);
        c->rx_pkt_len = c->payload_len;
        c->rx_pkt_ready = 1;

        if (c->on_packet) c->on_packet(c->rx_pkt_buf, c->rx_pkt_len);

        i += fl; /* skip entire frame */
    }

    return i;
}

/**
 * @brief Process new RX data between read_pos and write_pos (handles wrap).
 */
static void process_rx_data(uart_comm_t *c, uint16_t write_pos) {
    uint16_t read_pos = c->rx_read_pos;

    if (write_pos > read_pos) {
        /* Contiguous region */
        uint16_t n = write_pos - read_pos;
        uint16_t consumed = scan_rx_region(c, &c->rx_buf[read_pos], n);
        c->rx_read_pos = read_pos + consumed;
    } else if (write_pos < read_pos) {
        /* Wrap: two segments [read_pos..end] + [0..write_pos-1] */
        uint16_t seg1 = c->rx_buf_size - read_pos;
        uint16_t c1 = scan_rx_region(c, &c->rx_buf[read_pos], seg1);
        if (c1 == seg1) {
            uint16_t c2 = scan_rx_region(c, c->rx_buf, write_pos);
            c->rx_read_pos = c2;
        } else {
            c->rx_read_pos = read_pos + c1;
        }
    }
    /* write_pos == read_pos: nothing to do */
}

/* ================================================================
 * Public API
 * ================================================================ */

void uart_comm_init(uart_comm_t *c, const uart_comm_cfg_t *cfg) {
    if (!c || !cfg || !cfg->port_handle || !cfg->rx_buf || !cfg->tx_buf) return;
    if (cfg->payload_len == 0 || cfg->payload_len > UART_COMM_MAX_PAYLOAD) return;

    memset(c, 0, sizeof(*c));

    c->port_handle = cfg->port_handle;
    c->rx_buf      = cfg->rx_buf;
    c->rx_buf_size = cfg->rx_buf_size;
    c->tx_buf      = cfg->tx_buf;
    c->tx_buf_size = cfg->tx_buf_size;
    c->payload_len = cfg->payload_len;
    c->frame_len   = cfg->payload_len + UART_COMM_FRAME_OVERHEAD;
    c->on_packet   = cfg->on_packet;
    c->heartbeat   = cfg->heartbeat;

    /* Platform-specific: set up DMA circular RX */
    port_dma_rx_setup(cfg->port_handle, cfg->rx_buf, cfg->rx_buf_size);
    port_idle_it_enable(cfg->port_handle);
}

void uart_comm_poll(uart_comm_t *c) {
    if (!c || !c->port_handle) return;

    /* RX: check for new data */
    uint16_t write_pos = port_dma_rx_pos(c->port_handle, c->rx_buf_size);
    if (write_pos != c->rx_read_pos) {
        process_rx_data(c, write_pos);
    }

    /* Error recovery: too many consecutive failures → reset parser */
    if (c->rx_error_count >= UART_COMM_ERROR_THRESHOLD) {
        c->rx_error_count = 0;
        c->rx_read_pos = port_dma_rx_pos(c->port_handle, c->rx_buf_size);
        c->rx_pkt_ready = 0;
    }

    /* Heartbeat */
    if (c->heartbeat.enabled && c->heartbeat.interval_ms > 0) {
        uint32_t now = sys_millis();
        if (now - c->hb_last_tick >= c->heartbeat.interval_ms) {
            c->hb_last_tick = now;
            uart_comm_send_packet(c, c->heartbeat.payload, c->heartbeat.payload_len);
        }
    }
}

bool uart_comm_get_packet(uart_comm_t *c, uint8_t *out, uint8_t *len) {
    if (!c || !out || !len || !c->rx_pkt_ready) return false;
    memcpy(out, c->rx_pkt_buf, c->rx_pkt_len);
    *len = c->rx_pkt_len;
    c->rx_pkt_ready = 0;
    return true;
}

void uart_comm_send_packet(uart_comm_t *c, const uint8_t *payload, uint8_t len) {
    if (!c || !payload || len == 0 || len > UART_COMM_MAX_PAYLOAD) return;

    /* Build frame: [0xAA] [payload...] [checksum] [0x55] */
    uint8_t frame[UART_COMM_MAX_PAYLOAD + UART_COMM_FRAME_OVERHEAD];
    frame[0] = 0xAA;
    memcpy(&frame[1], payload, len);
    frame[1 + len] = compute_checksum(payload, len);
    frame[2 + len] = 0x55;
    uint8_t fl = len + UART_COMM_FRAME_OVERHEAD;

    if (!c->tx_dma_busy) {
        tx_dma_kick(c, frame, fl);
        c->stat_tx_frames++;
    } else if (tx_ring_free(c) >= fl) {
        for (uint8_t i = 0; i < fl; i++) tx_ring_push_byte(c, frame[i]);
        c->stat_tx_frames++;
    }
    /* else: ring full → frame dropped (acceptable for gimbal: latest-command-wins) */
}

void uart_comm_tx_cplt_handler(uart_comm_t *c, void *port_handle) {
    if (!c || c->port_handle != port_handle) return;

    if (tx_ring_used(c) >= c->frame_len)
        tx_dequeue_and_send(c);
    else
        c->tx_dma_busy = 0;
}

void uart_comm_error_handler(uart_comm_t *c, void *port_handle) {
    if (!c || c->port_handle != port_handle) return;

    port_rx_error_recover(port_handle, c->rx_buf, c->rx_buf_size);
    port_idle_it_enable(port_handle);

    c->rx_read_pos   = port_dma_rx_pos(port_handle, c->rx_buf_size);
    c->rx_error_count = 0;
    c->rx_pkt_ready   = 0;
}

void uart_comm_reset(uart_comm_t *c) {
    if (!c) return;
    c->rx_read_pos   = port_dma_rx_pos(c->port_handle, c->rx_buf_size);
    c->rx_error_count = 0;
    c->rx_pkt_ready   = 0;
    c->tx_dma_busy    = 0;
    c->tx_head        = 0;
    c->tx_tail        = 0;
}

void uart_comm_stats(uart_comm_t *c, uint32_t *rx, uint32_t *tx, uint32_t *err) {
    if (!c) return;
    if (rx)  *rx  = c->stat_rx_frames;
    if (tx)  *tx  = c->stat_tx_frames;
    if (err) *err = c->stat_rx_errors;
}

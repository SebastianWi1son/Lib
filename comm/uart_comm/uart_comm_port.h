/**
 * @file    uart_comm_port.h
 * @brief   Port interface for uart_comm — platform-specific DMA/UART glue
 *
 * Each supported platform provides one implementation of these functions.
 * The core uart_comm.c logic calls only through this interface and never
 * touches vendor HAL directly.
 *
 * To add a new platform (e.g. TI TM4C):
 *   1. Copy uart_comm_port_stm32.c → uart_comm_port_ti.c
 *   2. Implement all functions using TI's uDMA + UART API
 *   3. Include the correct port file in your build
 *
 * port_handle type: void* — cast to platform UART handle type inside port functions.
 * On STM32: (UART_HandleTypeDef*)port_handle
 * On TI:    (uint32_t)port_handle (UART base address)
 */

#ifndef __UART_COMM_PORT_H
#define __UART_COMM_PORT_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 * Port API — implement these 6 functions per platform
 * ================================================================ */

/**
 * @brief Initialize DMA receive in circular mode.
 *
 * Called once from uart_comm_init(). After this call, DMA continuously
 * writes received bytes into buf in a circular fashion.
 *
 * On platforms without DMA CIRCULAR support (very rare), implement as
 * periodic re-start in port_dma_rx_pos().
 */
void port_dma_rx_setup(void *port_handle, uint8_t *buf, uint16_t size);

/**
 * @brief Get the number of bytes DMA has written since setup.
 *
 * Wraps around with the buffer. The core logic handles wrap detection.
 *
 * Implementation: on STM32, reads NDTR register.
 * Returns position in [0, buf_size).
 */
uint16_t port_dma_rx_pos(void *port_handle, uint16_t buf_size);

/**
 * @brief Start a DMA transmit.
 *
 * Non-blocking. The core logic tracks busy state via port_dma_tx_busy().
 * The platform must arrange for uart_comm_tx_cplt_handler() to be called
 * when transmission completes (typically via UART TX complete ISR).
 */
void port_dma_tx_start(void *port_handle, const uint8_t *data, uint16_t len);

/**
 * @brief Check if a DMA TX is currently in progress.
 *
 * The core logic uses software flag c->tx_dma_busy, but this function
 * provides a hardware-level check for error recovery.
 */
bool port_dma_tx_busy(void *port_handle);

/**
 * @brief Recover from UART error (overrun, noise, framing, parity).
 *
 * Must: abort current RX DMA, clear all UART error flags,
 * restart circular DMA reception.
 */
void port_rx_error_recover(void *port_handle, uint8_t *buf, uint16_t size);

/**
 * @brief Enable UART IDLE line interrupt.
 *
 * Used for frame-boundary detection. No-op on platforms without IDLE
 * interrupt (e.g. STM32F103) — those platforms rely on poll-only mode.
 */
void port_idle_it_enable(void *port_handle);

#endif /* __UART_COMM_PORT_H */

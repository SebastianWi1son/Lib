/**
 * @file    uart_comm_port_stm32.c
 * @brief   STM32 HAL port implementation for uart_comm
 *
 * This is the ONLY file in uart_comm that includes STM32 HAL headers.
 * When porting to a different MCU family, replace this file entirely.
 *
 * All functions cast port_handle → UART_HandleTypeDef* internally.
 * The core uart_comm.c never sees STM32 types.
 */

#include "uart_comm_port.h"

/* ================================================================
 * STM32 HAL includes — only in this file
 * ================================================================ */

/* Auto-detect STM32 family for correct HAL include */
#if defined(STM32H7xx)
  #include "stm32h7xx_hal.h"
  #include "stm32h7xx_hal_dma.h"
#elif defined(STM32F4xx)
  #include "stm32f4xx_hal.h"
  #include "stm32f4xx_hal_dma.h"
#elif defined(STM32F1xx)
  #include "stm32f1xx_hal.h"
  #include "stm32f1xx_hal_dma.h"
#elif defined(STM32G4xx)
  #include "stm32g4xx_hal.h"
  #include "stm32g4xx_hal_dma.h"
#else
  #include "stm32h7xx_hal.h"   /* fallback */
  #include "stm32h7xx_hal_dma.h"
#endif

/* ================================================================
 * Helper: cast void* → UART_HandleTypeDef*
 * ================================================================ */
#define H(x)  ((UART_HandleTypeDef *)(x))

/* ================================================================
 * Port Implementation
 * ================================================================ */

void port_dma_rx_setup(void *port_handle, uint8_t *buf, uint16_t size) {
    UART_HandleTypeDef *h = H(port_handle);

    /* Start DMA reception — continuous (circular) */
    HAL_UART_Receive_DMA(h, buf, size);

    /* Enable circular mode on the DMA stream itself */
    if (h->hdmarx != NULL) {
        SET_BIT(h->hdmarx->Instance->CR, DMA_SxCR_CIRC);
    }
}

uint16_t port_dma_rx_pos(void *port_handle, uint16_t buf_size) {
    UART_HandleTypeDef *h = H(port_handle);
    if (h->hdmarx == NULL) return 0;

    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(h->hdmarx);
    return buf_size - remaining;
}

void port_dma_tx_start(void *port_handle, const uint8_t *data, uint16_t len) {
    HAL_UART_Transmit_DMA(H(port_handle), (uint8_t *)data, len);
}

bool port_dma_tx_busy(void *port_handle) {
    UART_HandleTypeDef *h = H(port_handle);
    /* Check HAL state: if not READY, TX is ongoing */
    return (h->gState != HAL_UART_STATE_READY);
}

void port_rx_error_recover(void *port_handle, uint8_t *buf, uint16_t size) {
    UART_HandleTypeDef *h = H(port_handle);

    /* Abort current reception */
    HAL_UART_AbortReceive(h);

    /* Clear all error flags */
    __HAL_UART_CLEAR_FLAG(h, UART_CLEAR_OREF | UART_CLEAR_NEF |
                              UART_CLEAR_PEF  | UART_CLEAR_FEF);

    /* Restart circular DMA reception */
    HAL_UART_Receive_DMA(h, buf, size);
    if (h->hdmarx != NULL) {
        SET_BIT(h->hdmarx->Instance->CR, DMA_SxCR_CIRC);
    }
}

void port_idle_it_enable(void *port_handle) {
    UART_HandleTypeDef *h = H(port_handle);
#if defined(STM32F1xx)
    /* STM32F1 has no IDLE interrupt — no-op */
    (void)h;
#else
    __HAL_UART_ENABLE_IT(h, UART_IT_IDLE);
#endif
}

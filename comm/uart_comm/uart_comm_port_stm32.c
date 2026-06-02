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

    /* Set CIRC bit BEFORE HAL_UART_Receive_DMA.
       At this point the DMA stream is disabled (from MX_DMA_Init / prior abort),
       so the write to CR takes effect.
       HAL_DMA_Start_IT internally disables→DMA_SetConfig(doesn't touch CIRC)→enables.
       Writing CIRC AFTER Start_IT (EN=1) is silently ignored on STM32H7/F7. */
    if (h->hdmarx != NULL) {
        SET_BIT(((DMA_Stream_TypeDef *)h->hdmarx->Instance)->CR, DMA_SxCR_CIRC);
    }

    HAL_UART_Receive_DMA(h, buf, size);
}

uint16_t port_dma_rx_pos(void *port_handle, uint16_t buf_size) {
    UART_HandleTypeDef *h = H(port_handle);
    if (h->hdmarx == NULL) return 0;

    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(h->hdmarx);
    return buf_size - remaining;
}

void port_dma_tx_start(void *port_handle, const uint8_t *data, uint16_t len) {
    UART_HandleTypeDef *h = H(port_handle);
    /* Match old vofa.c pattern: only TX when UART is ready */
    if (h->gState != HAL_UART_STATE_READY) return;
    HAL_UART_Transmit_DMA(h, (uint8_t *)data, len);
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

    /* Set CIRC before restarting — AbortReceive disables DMA, so write takes effect */
    if (h->hdmarx != NULL) {
        SET_BIT(((DMA_Stream_TypeDef *)h->hdmarx->Instance)->CR, DMA_SxCR_CIRC);
    }
    HAL_UART_Receive_DMA(h, buf, size);
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

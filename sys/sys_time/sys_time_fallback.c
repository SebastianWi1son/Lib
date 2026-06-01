/**
 * @file    sys_time_fallback.c
 * @brief   Generic SysTick-based fallback (no DWT dependency)
 *
 * Use this when DWT cycle counter is unavailable (e.g., Cortex-M0, non-ARM).
 * The caller must provide a 1kHz tick increment function.
 *
 * Usage:
 *   // In SysTick_Handler (or equivalent timer ISR):
 *   void SysTick_Handler(void) {
 *       sys_time_tick_inc();  // Call every 1ms
 *   }
 */

#include "sys_time.h"

/* ================================================================
 * Static state
 * ================================================================ */

static volatile uint32_t s_tick_ms = 0;

/* ================================================================
 * API
 * ================================================================ */

void sys_time_init(const sys_time_cfg_t *cfg) {
    (void)cfg; /* cpu_freq_hz not needed for tick-based fallback */
    s_tick_ms = 0;
}

/**
 * @brief Increment millisecond counter — call from 1kHz timer ISR.
 */
void sys_time_tick_inc(void) {
    s_tick_ms++;
}

uint32_t sys_micros(void) {
    /* Coarse: millis * 1000. Resolution limited to 1ms.
     * Acceptable for heartbeat and timeout purposes. */
    return s_tick_ms * 1000UL;
}

uint32_t sys_millis(void) {
    return s_tick_ms;
}

float sys_dt(volatile uint32_t *last_us) {
    if (last_us == NULL) return 0.0f;
    uint32_t now = sys_micros();
    uint32_t prev = *last_us;
    *last_us = now;
    if (now >= prev) {
        return (float)(now - prev) / 1000000.0f;
    } else {
        return (float)((0xFFFFFFFFUL - prev) + now + 1UL) / 1000000.0f;
    }
}

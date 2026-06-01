/**
 * @file    sys_time.h
 * @brief   Portable microsecond/millisecond time abstraction
 *
 * Design pattern: callback injection (same as FOC_Hardware_t).
 * Platform-specific timer implementation is injected at init time.
 *
 * Default backend: DWT cycle counter (available on all Cortex-M3/M4/M7/M33).
 * For non-Cortex-M targets, provide a custom implementation file.
 *
 * Usage:
 *   sys_time_cfg_t cfg = { .cpu_freq_hz = 480000000 };
 *   sys_time_init(&cfg);
 *   // then use sys_micros(), sys_millis(), sys_dt() anywhere
 *
 * Key insight:
 *   - sys_micros() wraps DWT->CYCCNT (or equivalent) → independent of HAL tick
 *   - sys_millis() wraps sys_micros() → independent of HAL_GetTick()
 *   - sys_dt() measures actual elapsed time → no hardcoded "dt = 0.001f"
 *   - All three work identically on STM32, TI Tiva C, NXP Kinetis, etc.
 */

#ifndef __SYS_TIME_H
#define __SYS_TIME_H

#include <stdint.h>

/* ================================================================
 * Configuration
 * ================================================================ */

typedef struct {
    uint32_t cpu_freq_hz;   /**< CPU core clock in Hz (e.g. 480000000 for H7 @ 480MHz) */
} sys_time_cfg_t;

/* ================================================================
 * Public API
 * ================================================================ */

/**
 * @brief Initialize the time system.
 *
 * Must be called once at startup, before any other sys_time functions.
 * For DWT-based implementations: unlocks DWT peripheral, resets cycle counter.
 *
 * @param cfg  CPU frequency configuration
 */
void sys_time_init(const sys_time_cfg_t *cfg);

/**
 * @brief Get elapsed microseconds since sys_time_init().
 *
 * Resolution: 1 μs (at 480 MHz, ±0.002 μs accuracy).
 * Rollover: ~71 minutes at 32-bit. Wraps correctly via 64-bit internal accumulator.
 *
 * @return Microseconds since init (uint32_t, wraps at ~71 min for callers that care)
 */
uint32_t sys_micros(void);

/**
 * @brief Get elapsed milliseconds since sys_time_init().
 *
 * Wraps sys_micros(). Safe to use as HAL_GetTick() replacement.
 * Rollover: ~49 days at 32-bit.
 *
 * @return Milliseconds since init
 */
uint32_t sys_millis(void);

/**
 * @brief Compute delta-time in seconds since last call.
 *
 * Typical usage in a periodic ISR:
 *   static uint32_t last_us = 0;
 *   float dt = sys_dt(&last_us);  // returns actual elapsed seconds
 *
 * This replaces "dt = 0.001f" with actual measurement:
 *   - If ISR jitter causes a 1.2ms gap, dt = 0.0012 (correct)
 *   - If ISR fires exactly at 1kHz, dt ≈ 0.001 (same as before)
 *
 * @param last_us  Pointer to static/global variable holding previous micros() value.
 *                 Updated in-place on each call.
 * @return         Elapsed time in seconds (float).
 */
float sys_dt(volatile uint32_t *last_us);

#endif /* __SYS_TIME_H */

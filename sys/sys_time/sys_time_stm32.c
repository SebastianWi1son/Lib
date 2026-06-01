/**
 * @file    sys_time_stm32.c
 * @brief   DWT cycle counter implementation (Cortex-M3/M4/M7/M33)
 *
 * Uses the ARM CoreSight DWT (Data Watchpoint and Trace) cycle counter.
 * This peripheral is present on ALL Cortex-M3/M4/M7/M33 processors,
 * regardless of vendor (ST, TI, NXP, Microchip, etc.).
 *
 * The ONLY vendor-specific input is cpu_freq_hz, provided by the caller at init.
 *
 * Integration (replace existing micros.c):
 *   #include "sys_time.h"
 *   sys_time_cfg_t cfg = { .cpu_freq_hz = HAL_RCC_GetHCLKFreq() };
 *   sys_time_init(&cfg);
 *   // Then use sys_micros() / sys_millis() everywhere
 *
 * For TI processors:
 *   sys_time_cfg_t cfg = { .cpu_freq_hz = 120000000 }; // TI Tiva C @ 120MHz
 *   sys_time_init(&cfg);
 *   // Everything else is identical
 */

#include "sys_time.h"
#include <stdint.h>

/* ================================================================
 * ARM CoreSight DWT registers (standard on Cortex-M3+)
 * These are NOT vendor-specific — they work on all Cortex-M.
 * ================================================================ */

/** Core Debug registers */
#define CORE_DEBUG_BASE  0xE000EDF0UL
#define CoreDebug        ((CoreDebug_Type *)CORE_DEBUG_BASE)

/** DWT (Data Watchpoint and Trace) registers */
#define DWT_BASE         0xE0001000UL
#define DWT              ((DWT_Type *)DWT_BASE)

typedef struct {
    volatile uint32_t DEMCR;  /**< Debug Exception and Monitor Control */
} CoreDebug_Type;

typedef struct {
    volatile uint32_t CTRL;   /**< Control register */
    volatile uint32_t CYCCNT; /**< Cycle count register */
    volatile uint32_t CPICNT; /**< CPI count register */
    volatile uint32_t EXCCNT; /**< Exception overhead count register */
    volatile uint32_t SLEEPCNT;/**< Sleep count register */
    volatile uint32_t LSUCNT; /**< LSU count register */
    volatile uint32_t FOLDCNT;/**< Folded-instruction count register */
    volatile uint32_t PCSR;   /**< Program Counter Sample register */
    volatile uint32_t COMP0;  /**< Comparator 0 */
    volatile uint32_t MASK0;  /**< Mask 0 */
    volatile uint32_t FUNCTION0;/**< Function 0 */
    /* ... more comparators follow, but we only need CTRL + CYCCNT */
} DWT_Type;

/* ================================================================
 * Bit definitions
 * ================================================================ */

#define CoreDebug_DEMCR_TRCENA_Msk   (1UL << 24)  /**< Trace enable */
#define DWT_CTRL_CYCCNTENA_Msk       (1UL << 0)   /**< Cycle counter enable */
#define DWT_LAR_ACCESS_KEY           0xC5ACCE55UL /**< Unlock key for DWT */

/* DWT Lock Access Register (not in standard CMSIS structs, defined inline) */
#define DWT_LAR_ADDR  (*(volatile uint32_t *)(DWT_BASE + 0xFB0))

/* ================================================================
 * Static state
 * ================================================================ */

static uint32_t s_cpu_freq_hz   = 0;
static uint32_t s_cpu_freq_mhz  = 0;  /* Pre-computed for division */
static uint32_t s_last_cycle    = 0;
static uint32_t s_cycle_high    = 0;  /* Upper 32 bits of 64-bit accumulator */
static uint32_t s_micros_base   = 0;  /* Accumulated microseconds at last rollover */

/* ================================================================
 * Implementation
 * ================================================================ */

void sys_time_init(const sys_time_cfg_t *cfg) {
    if (cfg == NULL) return;

    s_cpu_freq_hz  = cfg->cpu_freq_hz;
    s_cpu_freq_mhz = cfg->cpu_freq_hz / 1000000UL;
    s_last_cycle   = 0;
    s_cycle_high   = 0;
    s_micros_base  = 0;

    /* Enable DWT trace — standard on all Cortex-M3+ */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Unlock DWT registers (required on Cortex-M7, harmless on M3/M4) */
    DWT_LAR_ADDR = DWT_LAR_ACCESS_KEY;

    /* Reset and start cycle counter */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t sys_micros(void) {
    if (s_cpu_freq_hz == 0) return 0;

    uint32_t current_cycle = DWT->CYCCNT;

    /* Detect 32-bit rollover: CYCCNT wraps every ~8.9s at 480MHz */
    if (current_cycle < s_last_cycle) {
        s_cycle_high++;
        /* Accumulate microseconds from completed 32-bit cycle spans */
        s_micros_base += (0xFFFFFFFFUL / s_cpu_freq_mhz);
    }
    s_last_cycle = current_cycle;

    /* Compute microseconds from current 64-bit cycle count */
    uint64_t cycles_since_wrap = (uint64_t)current_cycle;

    /* Handle the corner case: carry from high word accumulation */
    uint32_t us = s_micros_base + (uint32_t)(cycles_since_wrap / s_cpu_freq_mhz);

    return us;
}

uint32_t sys_millis(void) {
    /* Simple: divide microseconds by 1000.
     * At 49.7-day rollover, this wraps correctly (uint32_t modulo). */
    return sys_micros() / 1000UL;
}

float sys_dt(volatile uint32_t *last_us) {
    if (last_us == NULL) return 0.0f;

    uint32_t now = sys_micros();
    uint32_t prev = *last_us;
    *last_us = now;

    if (now >= prev) {
        return (float)(now - prev) / 1000000.0f;
    } else {
        /* Handle micros() wrap (~71 min). The elapsed time is:
         *   (0xFFFFFFFF - prev) + now + 1
         * This is correct because micros() wraps at uint32_t max. */
        return (float)((0xFFFFFFFFUL - prev) + now + 1UL) / 1000000.0f;
    }
}

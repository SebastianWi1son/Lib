#ifndef __ERROR_REPORT_H
#define __ERROR_REPORT_H

/**
 * @file    error_report.h
 * @brief   Error report system — error codes, assertion macros, telemetry status channel
 *
 * Usage:
 *   - CHECK(condition, err)   — runtime check, always active, records error
 *   - ASSERT(condition, err)  — debug-only check, compiles to nothing in Release
 *   - PANIC(condition, err)   — hardware safety底线: record + emergency stop
 *
 * Error codes are grouped by module: high nibble = module, low nibble = specific.
 * In JustFloat telemetry frames, channel 0 carries error_report_get().
 *   0.0 = normal, any other value = error — look up in the enum below.
 */

#include <stdint.h>

/* ================================================================
 * Error Code Enum — grouped by module
 * ================================================================ */

typedef enum {
    ERR_NONE                = 0x00,  /* No error */

    /* ── Encoder (0x1x) ── */
    ERR_ENC_YAW_SPI_FAIL    = 0x10,  /* Yaw encoder SPI communication failed */
    ERR_ENC_PIT_I2C_FAIL    = 0x11,  /* Pitch encoder I2C communication failed */
    ERR_ENC_ANGLE_INVALID   = 0x12,  /* Encoder angle out of [0, 2π) range */
    ERR_ENC_CALIB_LOST      = 0x13,  /* Encoder calibration data lost */

    /* ── FOC (0x2x) ── */
    ERR_FOC_OVERCURRENT     = 0x20,  /* Motor overcurrent detected */
    ERR_FOC_OVERSPEED       = 0x21,  /* Motor overspeed detected */
    ERR_FOC_STARTUP_FAIL    = 0x22,  /* Alignment sequence failed */
    ERR_FOC_CURRENT_SENSOR_LOST = 0x23, /* Current sensor offline (auto-fallback) */

    /* ── Communication (0x3x) ── */
    ERR_COMM_VISION_RX_OVF   = 0x30, /* Vision channel (USART1) RX overflow */
    ERR_COMM_CHASSIS_TIMEOUT = 0x31, /* Chassis channel (USART2) timeout */
    ERR_COMM_CHECKSUM_FAIL   = 0x32, /* Frame checksum failures exceeded threshold */

    /* ── System (0x4x) ── */
    ERR_SYS_DT_ANOMALY       = 0x40, /* Control period dt out of valid range */
    ERR_SYS_WATCHDOG         = 0x41, /* Watchdog about to fire */

    /* ── Gimbal (0x5x) ── */
    ERR_GIMBAL_PITCH_LIMIT   = 0x50, /* Pitch hard limit triggered (±45°) */
    ERR_GIMBAL_YAW_JUMP      = 0x51, /* Yaw angle jump detected (sensor anomaly) */
    ERR_GIMBAL_EMERGENCY     = 0x52, /* Emergency stop triggered */

    ERR_COUNT
} error_code_t;

/* ================================================================
 * Assertion Macros
 * ================================================================ */

/**
 * @brief Runtime check — always active. Records error but does NOT halt.
 *        Use at every critical code path: sensor read, communication, etc.
 */
#define CHECK(cond, err) \
    do { if (!(cond)) { error_report_set(err); } } while(0)

/**
 * @brief Debug assertion — only active in Debug builds.
 *        On failure: record error, print file+line, enter infinite loop.
 *        On Release: compiles to nothing.
 */
#ifdef DEBUG
  #define ASSERT(cond, err) \
      do { if (!(cond)) { error_report_assert(err, __FILE__, __LINE__); } } while(0)
#else
  #define ASSERT(cond, err) ((void)0)
#endif

/**
 * @brief Hardware safety panic — always active.
 *        On failure: record error, THEN trigger emergency stop.
 *        Use for: overcurrent, overspeed, pitch limit exceeded, etc.
 */
#define PANIC(cond, err) \
    do { \
        if (!(cond)) { \
            error_report_set(err); \
            error_report_emergency_stop(); \
        } \
    } while(0)

/* ================================================================
 * Public API
 * ================================================================ */

/** Initialize error report system — clears all state */
void error_report_init(void);

/** Record an error. ISR-safe (only sets volatile flags, no blocking). */
void error_report_set(error_code_t err);

/** Assert failure handler — records error, prints diagnostics, loops forever */
void error_report_assert(error_code_t err, const char *file, int line);

/** Get current error code (0x00 = normal). Call from main loop or telemetry. */
error_code_t error_report_get(void);

/** Get cumulative warning count since last init */
uint32_t error_report_warning_count(void);

/** Get human-readable name string for an error code */
const char *error_report_name(error_code_t err);

/** Emergency stop callback — set by application layer at init */
void error_report_set_emergency_cb(void (*cb)(void));

/** Called by PANIC — invokes the emergency stop callback */
void error_report_emergency_stop(void);

#endif /* __ERROR_REPORT_H */

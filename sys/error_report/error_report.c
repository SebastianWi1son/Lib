/**
 * @file    error_report.c
 * @brief   Error report system implementation — ISR-safe, no blocking
 */

#include "error_report.h"

/* ================================================================
 * Static state — all volatile for ISR access
 * ================================================================ */

static volatile error_code_t s_current_error   = ERR_NONE;
static volatile uint32_t     s_warning_count   = 0;
static volatile uint8_t      s_error_latched   = 0;  /* 1 = error has been logged */

/* Emergency stop callback — registered by application layer */
static void (*s_emergency_cb)(void) = NULL;

/* ================================================================
 * Error name lookup table
 * ================================================================ */

static const char *s_error_names[] = {
    [ERR_NONE]                  = "NONE",
    [ERR_ENC_YAW_SPI_FAIL]      = "ENC_YAW_SPI_FAIL",
    [ERR_ENC_PIT_I2C_FAIL]      = "ENC_PIT_I2C_FAIL",
    [ERR_ENC_ANGLE_INVALID]     = "ENC_ANGLE_INVALID",
    [ERR_ENC_CALIB_LOST]        = "ENC_CALIB_LOST",
    [ERR_FOC_OVERCURRENT]       = "FOC_OVERCURRENT",
    [ERR_FOC_OVERSPEED]         = "FOC_OVERSPEED",
    [ERR_FOC_STARTUP_FAIL]      = "FOC_STARTUP_FAIL",
    [ERR_FOC_CURRENT_SENSOR_LOST] = "FOC_CURRENT_SENSOR_LOST",
    [ERR_COMM_VISION_RX_OVF]    = "COMM_VISION_RX_OVF",
    [ERR_COMM_CHASSIS_TIMEOUT]  = "COMM_CHASSIS_TIMEOUT",
    [ERR_COMM_CHECKSUM_FAIL]    = "COMM_CHECKSUM_FAIL",
    [ERR_SYS_DT_ANOMALY]        = "SYS_DT_ANOMALY",
    [ERR_SYS_WATCHDOG]          = "SYS_WATCHDOG",
    [ERR_GIMBAL_PITCH_LIMIT]    = "GIMBAL_PITCH_LIMIT",
    [ERR_GIMBAL_YAW_JUMP]       = "GIMBAL_YAW_JUMP",
    [ERR_GIMBAL_EMERGENCY]      = "GIMBAL_EMERGENCY",
};

/* ================================================================
 * Public API
 * ================================================================ */

void error_report_init(void) {
    s_current_error = ERR_NONE;
    s_warning_count = 0;
    s_error_latched = 0;
    s_emergency_cb  = NULL;
}

void error_report_set(error_code_t err) {
    if (err == ERR_NONE) return;

    /* Only count first occurrence — prevents spam in ISR */
    if (s_error_latched == 0) {
        s_error_latched = 1;
        s_current_error = err;
    }

    s_warning_count++;
}

void error_report_assert(error_code_t err, const char *file, int line) {
    s_current_error = err;
    s_error_latched = 1;

    /* On assert failure: print file+line, loop forever.
     * file/line are compile-time constants — stored in flash, not stack. */
    (void)file;
    (void)line;

    /* Disable interrupts and spin — this is a fatal error */
    __disable_irq();
    while (1) {
        /* If a debugger is attached, break here.
         * In production, the watchdog will reset the MCU. */
    }
}

error_code_t error_report_get(void) {
    return (error_code_t)s_current_error;
}

uint32_t error_report_warning_count(void) {
    return s_warning_count;
}

const char *error_report_name(error_code_t err) {
    if ((uint32_t)err >= ERR_COUNT) return "UNKNOWN";
    if (err == ERR_NONE) return s_error_names[ERR_NONE];
    return s_error_names[err];
}

void error_report_set_emergency_cb(void (*cb)(void)) {
    s_emergency_cb = cb;
}

void error_report_emergency_stop(void) {
    if (s_emergency_cb != NULL) {
        s_emergency_cb();
    }
}

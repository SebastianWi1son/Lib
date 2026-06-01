/**
 * @file    gimbal_controller.h
 * @brief   Reusable gimbal control API
 *
 * Provides a single entry point for gimbal angle/speed commands.
 * Multiple callers (UART, telemetry, CAN, local buttons) can share this interface
 * without directly manipulating FOC global variables.
 *
 * Dependencies: app_gimbal.h (for App_Gimbal_Set_Target, FOC_Motor_Handle_t externs)
 */

#ifndef __GIMBAL_CONTROLLER_H
#define __GIMBAL_CONTROLLER_H

#include <stdint.h>

/* ================================================================
 * Public API — callable from any context
 * ================================================================ */

/**
 * @brief Set gimbal target angles (logical coordinates).
 *
 * (0, 0) = center forward. Positive yaw = left, positive pitch = up.
 * Internally applies hard limits:
 *   Yaw:   no hard limit (slip ring, continuous)
 *   Pitch: ±45° (ribbon cable constraint)
 *
 * @param yaw_rad    Logical yaw target in radians
 * @param pitch_rad  Logical pitch target in radians
 */
void gimbal_ctrl_set_target(float yaw_rad, float pitch_rad);

/**
 * @brief Set gimbal speed limits (trajectory planner vmax).
 *
 * Affects both axes simultaneously. Values are in rad/s.
 * Internally clamped to safe hardware limits.
 *
 * @param yaw_max    Yaw max speed (rad/s), 0 = no change
 * @param pitch_max  Pitch max speed (rad/s), 0 = no change
 */
void gimbal_ctrl_set_speed_limit(float yaw_max, float pitch_max);

/**
 * @brief Get current gimbal state.
 *
 * Returns both target and actual angles in logical coordinates.
 *
 * @param yaw         [out] Current actual yaw angle (logical)
 * @param pitch       [out] Current actual pitch angle (logical)
 * @param target_yaw  [out] Current target yaw angle (logical)
 * @param target_pitch[out] Current target pitch angle (logical)
 */
void gimbal_ctrl_get_state(float *yaw, float *pitch,
                           float *target_yaw, float *target_pitch);

/**
 * @brief Emergency stop — immediately disable both motors.
 *
 * Sets system_runflag to 0, which triggers motor disable in ISR.
 * Callable from any context (ISR-safe).
 */
void gimbal_ctrl_emergency_stop(void);

/**
 * @brief Re-enable motors after emergency stop.
 *
 * Sets system_runflag to 1, re-enabling the ISR control loop.
 * Motors will hold current encoder position.
 */
void gimbal_ctrl_emergency_resume(void);

/**
 * @brief Check if gimbal is currently enabled.
 *
 * @return 1 if motors are running, 0 if stopped.
 */
uint8_t gimbal_ctrl_is_running(void);

#endif /* __GIMBAL_CONTROLLER_H */

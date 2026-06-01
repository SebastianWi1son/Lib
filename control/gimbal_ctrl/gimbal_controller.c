/**
 * @file    gimbal_controller.c
 * @brief   Reusable gimbal control API — Implementation
 *
 * Wraps the project's App_Gimbal_Set_Target() with:
 *   - Coordinate validation and clamping
 *   - Emergency stop/resume
 *   - State query in logical coordinates
 *
 * This module depends on the existing app_gimbal.c globals:
 *   gimbal_yaw, gimbal_pitch      (FOC_Motor_Handle_t)
 *   target_angle_yaw/pitch        (volatile float)
 *   yaw_center_angle/pitch_center_angle (volatile float)
 *   system_runflag                (volatile uint8_t)
 *   pitch_center_angle, yaw_center_angle
 *
 * These externs are resolved at link time when integrated into the project.
 */

#include "gimbal_controller.h"
#include <math.h>

/* ================================================================
 * External references — resolved by app_gimbal.c at link time
 * ================================================================ */

/* FOC motor instances (for reading current angles, modifying planner) */
struct FOC_Motor_Handle_s;
extern struct FOC_Motor_Handle_s gimbal_yaw;
extern struct FOC_Motor_Handle_s gimbal_pitch;

/* Target angles (written by set_target, read by ISR) */
extern volatile float target_angle_yaw;
extern volatile float target_angle_pitch;

/* Center offsets (set at init, used for logical↔physical conversion) */
extern volatile float yaw_center_angle;
extern volatile float pitch_center_angle;

/* System run flag (controls ISR motor enable/disable) */
extern volatile uint8_t system_runflag;

/* App_Gimbal_Set_Target — the existing function that does shortest-path
   math, hard limits for pitch, and jump filtering for yaw. */
extern void App_Gimbal_Set_Target(float cmd_yaw, float cmd_pitch);

/* ================================================================
 * Constants
 * ================================================================ */

/** Pitch hard limit (±45°) — ribbon cable constraint */
#define PITCH_LIMIT_RAD  0.785398163f   /* 45° = π/4 */

/** Speed limit bounds (rad/s) */
#define SPEED_MIN_RAD_S  0.1f
#define SPEED_MAX_RAD_S  50.0f

/* ================================================================
 * Public API
 * ================================================================ */

void gimbal_ctrl_set_target(float yaw_rad, float pitch_rad) {
    /* Clamp pitch to hardware limits (±45°).
     * Yaw is continuous via slip ring — no hard limit.
     * App_Gimbal_Set_Target internally handles shortest-path and jump filtering. */

    if (pitch_rad > PITCH_LIMIT_RAD)  pitch_rad = PITCH_LIMIT_RAD;
    if (pitch_rad < -PITCH_LIMIT_RAD) pitch_rad = -PITCH_LIMIT_RAD;

    App_Gimbal_Set_Target(yaw_rad, pitch_rad);
}

void gimbal_ctrl_set_speed_limit(float yaw_max, float pitch_max) {
    /* Update yaw planner */
    if (yaw_max > 0.0f) {
        if (yaw_max < SPEED_MIN_RAD_S) yaw_max = SPEED_MIN_RAD_S;
        if (yaw_max > SPEED_MAX_RAD_S) yaw_max = SPEED_MAX_RAD_S;
        gimbal_yaw.planner.max_speed = yaw_max;
        gimbal_yaw.cfg.traj_vmax     = yaw_max;
    }

    /* Update pitch planner */
    if (pitch_max > 0.0f) {
        if (pitch_max < SPEED_MIN_RAD_S) pitch_max = SPEED_MIN_RAD_S;
        if (pitch_max > SPEED_MAX_RAD_S) pitch_max = SPEED_MAX_RAD_S;
        gimbal_pitch.planner.max_speed = pitch_max;
        gimbal_pitch.cfg.traj_vmax     = pitch_max;
    }
}

void gimbal_ctrl_get_state(float *yaw, float *pitch,
                           float *target_yaw, float *target_pitch) {
    /* Convert from physical absolute angles to logical coordinates
       (0 = center forward, positive = left/up) */
    if (yaw)         *yaw         = gimbal_yaw.abs_angle - yaw_center_angle;
    if (pitch)       *pitch       = gimbal_pitch.abs_angle - pitch_center_angle;
    if (target_yaw)  *target_yaw  = target_angle_yaw - yaw_center_angle;
    if (target_pitch)*target_pitch = target_angle_pitch - pitch_center_angle;
}

void gimbal_ctrl_emergency_stop(void) {
    system_runflag = 0;
}

void gimbal_ctrl_emergency_resume(void) {
    /* Re-enable: ISR will call FOC_Enable on next tick */
    system_runflag = 1;
}

uint8_t gimbal_ctrl_is_running(void) {
    return (system_runflag != 0) ? 1 : 0;
}

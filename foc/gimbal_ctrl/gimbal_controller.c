/**
 * @file    gimbal_controller.c
 * @brief   云台控制 API — 通过 motor_param 操作 FOC 参数
 *
 * 所有参数写入通过 motor_param 层, 获得:
 *   - 统一范围校验
 *   - cfg + 运行时双写一致性
 *
 * 与 app_gimbal.c 的 globals 通过 extern 连接.
 * 迁移到 output/foc/ 后, 类型从 FOC_Motor_Handle_t 变为 foc_motor_t.
 */

#include "gimbal_controller.h"
#include "foc.h"
#include "motor_param.h"
#include <math.h>

/* ================================================================
 * External references — 由 app_gimbal.c 提供 (link-time)
 *
 * 迁移到 output/foc/ 后, 类型从 struct FOC_Motor_Handle_s 变为 foc_motor_t.
 * 两种声明等价 (二进制兼容), 选择一种即可.
 * ================================================================ */

extern foc_motor_t gimbal_yaw;
extern foc_motor_t gimbal_pitch;

extern volatile float target_angle_yaw;
extern volatile float target_angle_pitch;
extern volatile float yaw_center_angle;
extern volatile float pitch_center_angle;
extern volatile uint8_t system_runflag;

/* ================================================================
 * Constants
 * ================================================================ */

#define PITCH_LIMIT_RAD  0.785398163f   /* ±45° */
#define TWO_PI           6.283185307f
#define PI               3.141592654f

/* ================================================================
 * Public API
 * ================================================================ */

void gimbal_ctrl_set_target(float yaw_rad, float pitch_rad) {
    /* 逻辑坐标 → 物理坐标 */
    float physical_pitch = pitch_rad + pitch_center_angle;
    float physical_yaw   = yaw_rad   + yaw_center_angle;

    /* Pitch 硬限位 (±45°, 排线约束) */
    float limit_p = PITCH_LIMIT_RAD;
    if (physical_pitch > pitch_center_angle + limit_p)
        physical_pitch = pitch_center_angle + limit_p;
    if (physical_pitch < pitch_center_angle - limit_p)
        physical_pitch = pitch_center_angle - limit_p;

    target_angle_pitch = physical_pitch;

    /* Yaw 最短路径 (滑环, 连续旋转) */
    float raw_diff = physical_yaw - gimbal_yaw.abs_angle;

    /* 跳变滤波器 — 归一化前检查原始差值, 超过 180° 视为传感器异常 */
    float diff_y = fmodf(raw_diff, TWO_PI);
    if (fabsf(raw_diff) > PI && fabsf(diff_y) > PI) return;

    if (diff_y >  PI) diff_y -= TWO_PI;
    if (diff_y < -PI) diff_y += TWO_PI;

    target_angle_yaw = gimbal_yaw.abs_angle + diff_y;
}

void gimbal_ctrl_set_speed_limit(float yaw_max, float pitch_max) {
    /* 通过 motor_param 写入 — 自动校验 + cfg/planner 双写 */
    if (yaw_max > 0.0f)
        motor_param_set_traj_vmax(&gimbal_yaw, yaw_max);
    if (pitch_max > 0.0f)
        motor_param_set_traj_vmax(&gimbal_pitch, pitch_max);
}

void gimbal_ctrl_get_state(float *yaw, float *pitch,
                           float *target_yaw, float *target_pitch) {
    if (yaw)          *yaw          = gimbal_yaw.abs_angle - yaw_center_angle;
    if (pitch)        *pitch        = gimbal_pitch.abs_angle - pitch_center_angle;
    if (target_yaw)   *target_yaw   = target_angle_yaw - yaw_center_angle;
    if (target_pitch) *target_pitch = target_angle_pitch - pitch_center_angle;
}

void gimbal_ctrl_emergency_stop(void) {
    system_runflag = 0;
}

void gimbal_ctrl_emergency_resume(void) {
    system_runflag = 1;
}

uint8_t gimbal_ctrl_is_running(void) {
    return (system_runflag != 0) ? 1 : 0;
}

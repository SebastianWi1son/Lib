#ifndef __MOTOR_PARAM_H
#define __MOTOR_PARAM_H

/**
 * @file    motor_param.h
 * @brief   FOC 参数统一访问层 — set/get + 范围校验 + 快照
 *
 * 所有对 foc_motor_t 内部可调参数的读写都通过本模块。
 * 控制器 (gimbal_ctrl)、调试器 (motor_tune)、遥测 (app_tele)
 * 都是本模块的客户端。
 *
 * 参考: STM32 Motor Control SDK 的 MC_Protocol register map
 *        ODrive 的 endpoint_mapper
 */

#include <stdint.h>
#include <stdbool.h>
#include "foc.h"

/* ================================================================
 * 参数 ID 枚举 — 集中管理的注册表
 * ================================================================ */

typedef enum {
    /* ── 角度环 PID ── */
    PARAM_ANGLE_KP = 0,
    PARAM_ANGLE_KI,
    PARAM_ANGLE_KD,
    PARAM_ANGLE_LIMIT_OUT,
    PARAM_ANGLE_LIMIT_I,
    PARAM_ANGLE_RAMP,
    PARAM_ANGLE_SEP_ERR,

    /* ── 速度环 PID ── */
    PARAM_VEL_KP,
    PARAM_VEL_KI,
    PARAM_VEL_KD,
    PARAM_VEL_LIMIT_OUT,
    PARAM_VEL_LIMIT_I,
    PARAM_VEL_RAMP,
    PARAM_VEL_SEP_ERR,

    /* ── Iq 电流环 PID ── */
    PARAM_IQ_KP,
    PARAM_IQ_KI,
    PARAM_IQ_KD,
    PARAM_IQ_LIMIT_OUT,
    PARAM_IQ_LIMIT_I,
    PARAM_IQ_RAMP,
    PARAM_IQ_SEP_ERR,

    /* ── Id 电流环 PID ── */
    PARAM_ID_KP,
    PARAM_ID_KI,
    PARAM_ID_KD,
    PARAM_ID_LIMIT_OUT,
    PARAM_ID_LIMIT_I,
    PARAM_ID_RAMP,
    PARAM_ID_SEP_ERR,

    /* ── 系统参数 ── */
    PARAM_VOLTAGE_LIMIT,
    PARAM_DEADZONE,
    PARAM_TRAJ_VMAX,
    PARAM_TRAJ_TF,
    PARAM_VEL_LPF_TF,
    PARAM_CTRL_MODE,
    PARAM_MAX_CURRENT,

    PARAM_COUNT          /* 参数总数 (自动维护) */
} foc_param_id_t;

/* ================================================================
 * PID 三元组 — 原子批量写入用
 * ================================================================ */

typedef enum {
    PID_WHICH_ANGLE = 0,
    PID_WHICH_VEL   = 1,
    PID_WHICH_IQ    = 2,
    PID_WHICH_ID    = 3,
} motor_pid_which_t;

typedef struct {
    float kp, ki, kd;
} motor_pid_triple_t;

/* ================================================================
 * 参数快照 — 调试回滚 & 出厂默认
 * ================================================================ */

typedef struct {
    foc_pid_param_t angle_pid;
    foc_pid_param_t vel_pid;
    foc_pid_param_t iq_pid;
    foc_pid_param_t id_pid;
    float voltage_limit;
    float deadzone;
    float traj_vmax;
    float traj_tf;
    float vel_lpf_tf;
    foc_ctrl_mode_t ctrl_mode;
    float max_current;
} motor_param_snapshot_t;

/* ================================================================
 * 单参数 Set — 返回 false = 校验失败, 参数未修改
 * ================================================================ */

bool motor_param_set_angle_kp(foc_motor_t *m, float v);
bool motor_param_set_angle_ki(foc_motor_t *m, float v);
bool motor_param_set_angle_kd(foc_motor_t *m, float v);
bool motor_param_set_angle_limit_out(foc_motor_t *m, float v);
bool motor_param_set_angle_limit_i(foc_motor_t *m, float v);
bool motor_param_set_angle_ramp(foc_motor_t *m, float v);
bool motor_param_set_angle_sep_err(foc_motor_t *m, float v);

bool motor_param_set_vel_kp(foc_motor_t *m, float v);
bool motor_param_set_vel_ki(foc_motor_t *m, float v);
bool motor_param_set_vel_kd(foc_motor_t *m, float v);
bool motor_param_set_vel_limit_out(foc_motor_t *m, float v);
bool motor_param_set_vel_limit_i(foc_motor_t *m, float v);
bool motor_param_set_vel_ramp(foc_motor_t *m, float v);
bool motor_param_set_vel_sep_err(foc_motor_t *m, float v);

bool motor_param_set_iq_kp(foc_motor_t *m, float v);
bool motor_param_set_iq_ki(foc_motor_t *m, float v);
bool motor_param_set_iq_kd(foc_motor_t *m, float v);
bool motor_param_set_iq_limit_out(foc_motor_t *m, float v);
bool motor_param_set_iq_limit_i(foc_motor_t *m, float v);
bool motor_param_set_iq_ramp(foc_motor_t *m, float v);
bool motor_param_set_iq_sep_err(foc_motor_t *m, float v);

bool motor_param_set_id_kp(foc_motor_t *m, float v);
bool motor_param_set_id_ki(foc_motor_t *m, float v);
bool motor_param_set_id_kd(foc_motor_t *m, float v);
bool motor_param_set_id_limit_out(foc_motor_t *m, float v);
bool motor_param_set_id_limit_i(foc_motor_t *m, float v);
bool motor_param_set_id_ramp(foc_motor_t *m, float v);
bool motor_param_set_id_sep_err(foc_motor_t *m, float v);

bool motor_param_set_voltage_limit(foc_motor_t *m, float v);
bool motor_param_set_deadzone(foc_motor_t *m, float v);
bool motor_param_set_traj_vmax(foc_motor_t *m, float v);
bool motor_param_set_traj_tf(foc_motor_t *m, float v);
bool motor_param_set_vel_lpf_tf(foc_motor_t *m, float v);
bool motor_param_set_ctrl_mode(foc_motor_t *m, foc_ctrl_mode_t mode);
bool motor_param_set_max_current(foc_motor_t *m, float v);

/* ================================================================
 * 单参数 Get — 只读, 不需要校验
 * ================================================================ */

float motor_param_get_angle_kp(foc_motor_t *m);
float motor_param_get_angle_ki(foc_motor_t *m);
float motor_param_get_angle_kd(foc_motor_t *m);
float motor_param_get_vel_kp(foc_motor_t *m);
float motor_param_get_vel_ki(foc_motor_t *m);
float motor_param_get_vel_kd(foc_motor_t *m);
float motor_param_get_iq_kp(foc_motor_t *m);
float motor_param_get_iq_ki(foc_motor_t *m);
float motor_param_get_iq_kd(foc_motor_t *m);
float motor_param_get_id_kp(foc_motor_t *m);
float motor_param_get_id_ki(foc_motor_t *m);
float motor_param_get_id_kd(foc_motor_t *m);
float motor_param_get_voltage_limit(foc_motor_t *m);
float motor_param_get_deadzone(foc_motor_t *m);
float motor_param_get_traj_vmax(foc_motor_t *m);
float motor_param_get_traj_tf(foc_motor_t *m);
float motor_param_get_vel_lpf_tf(foc_motor_t *m);
foc_ctrl_mode_t motor_param_get_ctrl_mode(foc_motor_t *m);
float motor_param_get_max_current(foc_motor_t *m);

/* ================================================================
 * 批量设置 PID (kp/ki/kd 原子写入)
 * ================================================================ */

bool motor_param_set_pid_batch(foc_motor_t *m, motor_pid_triple_t *t,
                                motor_pid_which_t which);

/* ================================================================
 * 通用 Set/Get — 通过 param_id + value 访问 (协议/上位机用)
 * ================================================================ */

bool motor_param_set_by_id(foc_motor_t *m, foc_param_id_t id, float value);
float motor_param_get_by_id(foc_motor_t *m, foc_param_id_t id);

/* ================================================================
 * 范围查询 — 客户端可用作 UI 提示或校验
 * ================================================================ */

typedef struct {
    float min;
    float max;
    const char *name;       /* 上位机命令名 */
} motor_param_bounds_t;

const motor_param_bounds_t *motor_param_get_bounds(foc_param_id_t id);

/* ================================================================
 * 快照 & 回滚
 * ================================================================ */

void motor_param_snapshot_save(foc_motor_t *m, motor_param_snapshot_t *snap);
void motor_param_snapshot_restore(foc_motor_t *m, motor_param_snapshot_t *snap);

/* 出厂默认: motor_param 内部保存一份默认值 (首次 init 时自动保存) */
void motor_param_snapshot_save_default(foc_motor_t *m);
void motor_param_snapshot_restore_default(foc_motor_t *m);

#endif /* __MOTOR_PARAM_H */

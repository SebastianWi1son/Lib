#include "motor_param.h"
#include <math.h>

/* ================================================================
 * 参数范围校验表 — 集中管理, 单一事实来源
 * ================================================================ */

static const motor_param_bounds_t bounds_table[PARAM_COUNT] = {
    [PARAM_ANGLE_KP]        = {0.0f,   100.0f, "ang_Kp:"},
    [PARAM_ANGLE_KI]        = {0.0f,    50.0f, "ang_Ki:"},
    [PARAM_ANGLE_KD]        = {0.0f,    10.0f, "ang_Kd:"},
    [PARAM_ANGLE_LIMIT_OUT] = {0.1f,   100.0f, "ang_MaxOut:"},
    [PARAM_ANGLE_LIMIT_I]   = {0.0f,    50.0f, "ang_MaxI:"},
    [PARAM_ANGLE_RAMP]      = {0.0f,  2000.0f, "ang_Ramp:"},
    [PARAM_ANGLE_SEP_ERR]   = {0.0f,     5.0f, "ang_SepErr:"},

    [PARAM_VEL_KP]          = {0.0f,    10.0f, "vel_Kp:"},
    [PARAM_VEL_KI]          = {0.0f,    20.0f, "vel_Ki:"},
    [PARAM_VEL_KD]          = {0.0f,     1.0f, "vel_Kd:"},
    [PARAM_VEL_LIMIT_OUT]   = {0.1f,    12.0f, "vel_MaxOut:"},
    [PARAM_VEL_LIMIT_I]     = {0.0f,    12.0f, "vel_MaxI:"},
    [PARAM_VEL_RAMP]        = {0.0f,  2000.0f, "vel_Ramp:"},
    [PARAM_VEL_SEP_ERR]     = {0.0f,    10.0f, "vel_SepErr:"},

    [PARAM_IQ_KP]           = {0.0f,     5.0f, "iq_Kp:"},
    [PARAM_IQ_KI]           = {0.0f,    50.0f, "iq_Ki:"},
    [PARAM_IQ_KD]           = {0.0f,     0.5f, "iq_Kd:"},
    [PARAM_IQ_LIMIT_OUT]    = {0.1f,    12.0f, "iq_MaxOut:"},
    [PARAM_IQ_LIMIT_I]      = {0.0f,     6.0f, "iq_MaxI:"},
    [PARAM_IQ_RAMP]         = {0.0f,  2000.0f, "iq_Ramp:"},
    [PARAM_IQ_SEP_ERR]      = {0.0f,    10.0f, "iq_SepErr:"},

    [PARAM_ID_KP]           = {0.0f,     5.0f, "id_Kp:"},
    [PARAM_ID_KI]           = {0.0f,    50.0f, "id_Ki:"},
    [PARAM_ID_KD]           = {0.0f,     0.5f, "id_Kd:"},
    [PARAM_ID_LIMIT_OUT]    = {0.1f,    12.0f, "id_MaxOut:"},
    [PARAM_ID_LIMIT_I]      = {0.0f,     6.0f, "id_MaxI:"},
    [PARAM_ID_RAMP]         = {0.0f,  2000.0f, "id_Ramp:"},
    [PARAM_ID_SEP_ERR]      = {0.0f,    10.0f, "id_SepErr:"},

    [PARAM_VOLTAGE_LIMIT]   = {0.1f,    12.0f, "volt_limit:"},
    [PARAM_DEADZONE]        = {0.0f,     0.5f, "deadzone:"},
    [PARAM_TRAJ_VMAX]       = {0.1f,   100.0f, "traj_Vmax:"},
    [PARAM_TRAJ_TF]         = {0.001f,   1.0f, "traj_Tf:"},
    [PARAM_VEL_LPF_TF]      = {0.001f,   0.1f, "lpf_Tf:"},
    [PARAM_CTRL_MODE]       = {0.0f,     2.0f, "ctrl_mode:"},
    [PARAM_MAX_CURRENT]     = {0.0f,    10.0f, "max_current:"},
};

/* ================================================================
 * 出厂默认快照 (静态, 首次 save_default 时填充)
 * ================================================================ */

static motor_param_snapshot_t default_snapshot;
static bool default_saved = false;

/* ================================================================
 * 内部辅助: 范围校验 + 裁剪
 * ================================================================ */

static inline float clamp_to_bounds(foc_param_id_t id, float value, bool *clamped) {
    const motor_param_bounds_t *b = &bounds_table[id];
    if (value > b->max) { *clamped = true; return b->max; }
    if (value < b->min) { *clamped = true; return b->min; }
    *clamped = false;
    return value;
}

/* ================================================================
 * 单参数 Set — 返回 false 表示值被裁剪 (超出范围)
 * ================================================================ */

#define DEFINE_PARAM_SET(name, field, id)                               \
    bool motor_param_set_##name(foc_motor_t *m, float v) {             \
        bool clamped;                                                    \
        v = clamp_to_bounds(id, v, &clamped);                           \
        m->field = v;                                                    \
        return !clamped;                                                 \
    }

/* ── 角度环 PID (写 cfg 模板 + pid_instance_t 运行时) ── */

bool motor_param_set_angle_kp(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ANGLE_KP, v, &clamped);
    m->cfg.angle_pid.kp = v; m->angle_pid.kp = v; return !clamped;
}
bool motor_param_set_angle_ki(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ANGLE_KI, v, &clamped);
    m->cfg.angle_pid.ki = v; m->angle_pid.ki = v; return !clamped;
}
bool motor_param_set_angle_kd(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ANGLE_KD, v, &clamped);
    m->cfg.angle_pid.kd = v; m->angle_pid.kd = v; return !clamped;
}
bool motor_param_set_angle_limit_out(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ANGLE_LIMIT_OUT, v, &clamped);
    m->cfg.angle_pid.limit_out = v; m->angle_pid.limit_out = v; return !clamped;
}
bool motor_param_set_angle_limit_i(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ANGLE_LIMIT_I, v, &clamped);
    m->cfg.angle_pid.limit_i_out = v; m->angle_pid.limit_i_out = v; return !clamped;
}
bool motor_param_set_angle_ramp(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ANGLE_RAMP, v, &clamped);
    m->cfg.angle_pid.ramp = v; m->angle_pid.output_ramp = v; return !clamped;
}
bool motor_param_set_angle_sep_err(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ANGLE_SEP_ERR, v, &clamped);
    m->cfg.angle_pid.sep_err = v; m->angle_pid.separation_err = v; return !clamped;
}

/* ── 速度环 PID ── */

bool motor_param_set_vel_kp(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_VEL_KP, v, &clamped);
    m->cfg.vel_pid.kp = v; m->vel_pid.kp = v; return !clamped;
}
bool motor_param_set_vel_ki(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_VEL_KI, v, &clamped);
    m->cfg.vel_pid.ki = v; m->vel_pid.ki = v; return !clamped;
}
bool motor_param_set_vel_kd(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_VEL_KD, v, &clamped);
    m->cfg.vel_pid.kd = v; m->vel_pid.kd = v; return !clamped;
}
bool motor_param_set_vel_limit_out(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_VEL_LIMIT_OUT, v, &clamped);
    m->cfg.vel_pid.limit_out = v; m->vel_pid.limit_out = v; return !clamped;
}
bool motor_param_set_vel_limit_i(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_VEL_LIMIT_I, v, &clamped);
    m->cfg.vel_pid.limit_i_out = v; m->vel_pid.limit_i_out = v; return !clamped;
}
bool motor_param_set_vel_ramp(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_VEL_RAMP, v, &clamped);
    m->cfg.vel_pid.ramp = v; m->vel_pid.output_ramp = v; return !clamped;
}
bool motor_param_set_vel_sep_err(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_VEL_SEP_ERR, v, &clamped);
    m->cfg.vel_pid.sep_err = v; m->vel_pid.separation_err = v; return !clamped;
}

/* ── Iq 电流环 PID ── */

bool motor_param_set_iq_kp(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_IQ_KP, v, &clamped);
    m->cfg.iq_pid.kp = v; m->iq_pid.kp = v; return !clamped;
}
bool motor_param_set_iq_ki(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_IQ_KI, v, &clamped);
    m->cfg.iq_pid.ki = v; m->iq_pid.ki = v; return !clamped;
}
bool motor_param_set_iq_kd(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_IQ_KD, v, &clamped);
    m->cfg.iq_pid.kd = v; m->iq_pid.kd = v; return !clamped;
}
bool motor_param_set_iq_limit_out(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_IQ_LIMIT_OUT, v, &clamped);
    m->cfg.iq_pid.limit_out = v; m->iq_pid.limit_out = v; return !clamped;
}
bool motor_param_set_iq_limit_i(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_IQ_LIMIT_I, v, &clamped);
    m->cfg.iq_pid.limit_i_out = v; m->iq_pid.limit_i_out = v; return !clamped;
}
bool motor_param_set_iq_ramp(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_IQ_RAMP, v, &clamped);
    m->cfg.iq_pid.ramp = v; m->iq_pid.output_ramp = v; return !clamped;
}
bool motor_param_set_iq_sep_err(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_IQ_SEP_ERR, v, &clamped);
    m->cfg.iq_pid.sep_err = v; m->iq_pid.separation_err = v; return !clamped;
}

/* ── Id 电流环 PID ── */

bool motor_param_set_id_kp(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ID_KP, v, &clamped);
    m->cfg.id_pid.kp = v; m->id_pid.kp = v; return !clamped;
}
bool motor_param_set_id_ki(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ID_KI, v, &clamped);
    m->cfg.id_pid.ki = v; m->id_pid.ki = v; return !clamped;
}
bool motor_param_set_id_kd(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ID_KD, v, &clamped);
    m->cfg.id_pid.kd = v; m->id_pid.kd = v; return !clamped;
}
bool motor_param_set_id_limit_out(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ID_LIMIT_OUT, v, &clamped);
    m->cfg.id_pid.limit_out = v; m->id_pid.limit_out = v; return !clamped;
}
bool motor_param_set_id_limit_i(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ID_LIMIT_I, v, &clamped);
    m->cfg.id_pid.limit_i_out = v; m->id_pid.limit_i_out = v; return !clamped;
}
bool motor_param_set_id_ramp(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ID_RAMP, v, &clamped);
    m->cfg.id_pid.ramp = v; m->id_pid.output_ramp = v; return !clamped;
}
bool motor_param_set_id_sep_err(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_ID_SEP_ERR, v, &clamped);
    m->cfg.id_pid.sep_err = v; m->id_pid.separation_err = v; return !clamped;
}

/* ── 系统参数 (cfg + 运行时双写, 保持一致性) ── */

bool motor_param_set_voltage_limit(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_VOLTAGE_LIMIT, v, &clamped);
    m->cfg.voltage_limit = v; return !clamped;
}
bool motor_param_set_deadzone(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_DEADZONE, v, &clamped);
    m->cfg.deadzone = v; return !clamped;
}
bool motor_param_set_traj_vmax(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_TRAJ_VMAX, v, &clamped);
    m->cfg.traj_vmax = v;
    m->planner.max_speed = v;  /* 与 gimbal_ctrl 保持一致: cfg + planner 双写 */
    return !clamped;
}
bool motor_param_set_traj_tf(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_TRAJ_TF, v, &clamped);
    m->cfg.traj_tf = v;
    m->planner.Tf = v;
    return !clamped;
}
bool motor_param_set_vel_lpf_tf(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_VEL_LPF_TF, v, &clamped);
    m->cfg.vel_lpf_tf = v;
    m->vel_lpf.Tf = v;
    return !clamped;
}
bool motor_param_set_ctrl_mode(foc_motor_t *m, foc_ctrl_mode_t mode) {
    bool clamped; float v = (float)mode;
    v = clamp_to_bounds(PARAM_CTRL_MODE, v, &clamped);
    m->cfg.ctrl_mode = (foc_ctrl_mode_t)v; return !clamped;
}
bool motor_param_set_max_current(foc_motor_t *m, float v) {
    bool clamped; v = clamp_to_bounds(PARAM_MAX_CURRENT, v, &clamped);
    m->cfg.max_current = v; return !clamped;
}

/* ================================================================
 * 单参数 Get — 从 cfg 模板读取
 * ================================================================ */

#define DEFINE_PARAM_GET(name, field)                                   \
    float motor_param_get_##name(foc_motor_t *m) { return m->field; }

DEFINE_PARAM_GET(angle_kp,       cfg.angle_pid.kp)
DEFINE_PARAM_GET(angle_ki,       cfg.angle_pid.ki)
DEFINE_PARAM_GET(angle_kd,       cfg.angle_pid.kd)
DEFINE_PARAM_GET(vel_kp,         cfg.vel_pid.kp)
DEFINE_PARAM_GET(vel_ki,         cfg.vel_pid.ki)
DEFINE_PARAM_GET(vel_kd,         cfg.vel_pid.kd)
DEFINE_PARAM_GET(iq_kp,          cfg.iq_pid.kp)
DEFINE_PARAM_GET(iq_ki,          cfg.iq_pid.ki)
DEFINE_PARAM_GET(iq_kd,          cfg.iq_pid.kd)
DEFINE_PARAM_GET(id_kp,          cfg.id_pid.kp)
DEFINE_PARAM_GET(id_ki,          cfg.id_pid.ki)
DEFINE_PARAM_GET(id_kd,          cfg.id_pid.kd)
DEFINE_PARAM_GET(voltage_limit,  cfg.voltage_limit)
DEFINE_PARAM_GET(deadzone,       cfg.deadzone)
DEFINE_PARAM_GET(traj_vmax,      cfg.traj_vmax)
DEFINE_PARAM_GET(traj_tf,        cfg.traj_tf)
DEFINE_PARAM_GET(vel_lpf_tf,     cfg.vel_lpf_tf)
DEFINE_PARAM_GET(ctrl_mode,      cfg.ctrl_mode)
DEFINE_PARAM_GET(max_current,    cfg.max_current)

/* ================================================================
 * 批量设置 PID — 三个值在一次调用中原子写入
 * ================================================================ */

bool motor_param_set_pid_batch(foc_motor_t *m, motor_pid_triple_t *t,
                                motor_pid_which_t which) {
    bool ok = true;
    switch (which) {
    case PID_WHICH_ANGLE:
        ok &= motor_param_set_angle_kp(m, t->kp);
        ok &= motor_param_set_angle_ki(m, t->ki);
        ok &= motor_param_set_angle_kd(m, t->kd);
        break;
    case PID_WHICH_VEL:
        ok &= motor_param_set_vel_kp(m, t->kp);
        ok &= motor_param_set_vel_ki(m, t->ki);
        ok &= motor_param_set_vel_kd(m, t->kd);
        break;
    case PID_WHICH_IQ:
        ok &= motor_param_set_iq_kp(m, t->kp);
        ok &= motor_param_set_iq_ki(m, t->ki);
        ok &= motor_param_set_iq_kd(m, t->kd);
        break;
    case PID_WHICH_ID:
        ok &= motor_param_set_id_kp(m, t->kp);
        ok &= motor_param_set_id_ki(m, t->ki);
        ok &= motor_param_set_id_kd(m, t->kd);
        break;
    }
    return ok;
}

/* ================================================================
 * 通用 Set/Get by ID — 协议/上位机通信用
 * ================================================================ */

bool motor_param_set_by_id(foc_motor_t *m, foc_param_id_t id, float value) {
    switch (id) {
    case PARAM_ANGLE_KP:        return motor_param_set_angle_kp(m, value);
    case PARAM_ANGLE_KI:        return motor_param_set_angle_ki(m, value);
    case PARAM_ANGLE_KD:        return motor_param_set_angle_kd(m, value);
    case PARAM_ANGLE_LIMIT_OUT: return motor_param_set_angle_limit_out(m, value);
    case PARAM_ANGLE_LIMIT_I:   return motor_param_set_angle_limit_i(m, value);
    case PARAM_ANGLE_RAMP:      return motor_param_set_angle_ramp(m, value);
    case PARAM_ANGLE_SEP_ERR:   return motor_param_set_angle_sep_err(m, value);
    case PARAM_VEL_KP:          return motor_param_set_vel_kp(m, value);
    case PARAM_VEL_KI:          return motor_param_set_vel_ki(m, value);
    case PARAM_VEL_KD:          return motor_param_set_vel_kd(m, value);
    case PARAM_VEL_LIMIT_OUT:   return motor_param_set_vel_limit_out(m, value);
    case PARAM_VEL_LIMIT_I:     return motor_param_set_vel_limit_i(m, value);
    case PARAM_VEL_RAMP:        return motor_param_set_vel_ramp(m, value);
    case PARAM_VEL_SEP_ERR:     return motor_param_set_vel_sep_err(m, value);
    case PARAM_IQ_KP:           return motor_param_set_iq_kp(m, value);
    case PARAM_IQ_KI:           return motor_param_set_iq_ki(m, value);
    case PARAM_IQ_KD:           return motor_param_set_iq_kd(m, value);
    case PARAM_IQ_LIMIT_OUT:    return motor_param_set_iq_limit_out(m, value);
    case PARAM_IQ_LIMIT_I:      return motor_param_set_iq_limit_i(m, value);
    case PARAM_IQ_RAMP:         return motor_param_set_iq_ramp(m, value);
    case PARAM_IQ_SEP_ERR:      return motor_param_set_iq_sep_err(m, value);
    case PARAM_ID_KP:           return motor_param_set_id_kp(m, value);
    case PARAM_ID_KI:           return motor_param_set_id_ki(m, value);
    case PARAM_ID_KD:           return motor_param_set_id_kd(m, value);
    case PARAM_ID_LIMIT_OUT:    return motor_param_set_id_limit_out(m, value);
    case PARAM_ID_LIMIT_I:      return motor_param_set_id_limit_i(m, value);
    case PARAM_ID_RAMP:         return motor_param_set_id_ramp(m, value);
    case PARAM_ID_SEP_ERR:      return motor_param_set_id_sep_err(m, value);
    case PARAM_VOLTAGE_LIMIT:   return motor_param_set_voltage_limit(m, value);
    case PARAM_DEADZONE:        return motor_param_set_deadzone(m, value);
    case PARAM_TRAJ_VMAX:       return motor_param_set_traj_vmax(m, value);
    case PARAM_TRAJ_TF:         return motor_param_set_traj_tf(m, value);
    case PARAM_VEL_LPF_TF:      return motor_param_set_vel_lpf_tf(m, value);
    case PARAM_CTRL_MODE:       return motor_param_set_ctrl_mode(m, (foc_ctrl_mode_t)value);
    case PARAM_MAX_CURRENT:     return motor_param_set_max_current(m, value);
    default: return false;
    }
}

float motor_param_get_by_id(foc_motor_t *m, foc_param_id_t id) {
    switch (id) {
    case PARAM_ANGLE_KP:        return motor_param_get_angle_kp(m);
    case PARAM_ANGLE_KI:        return motor_param_get_angle_ki(m);
    case PARAM_ANGLE_KD:        return motor_param_get_angle_kd(m);
    case PARAM_VEL_KP:          return motor_param_get_vel_kp(m);
    case PARAM_VEL_KI:          return motor_param_get_vel_ki(m);
    case PARAM_VEL_KD:          return motor_param_get_vel_kd(m);
    case PARAM_IQ_KP:           return motor_param_get_iq_kp(m);
    case PARAM_IQ_KI:           return motor_param_get_iq_ki(m);
    case PARAM_IQ_KD:           return motor_param_get_iq_kd(m);
    case PARAM_ID_KP:           return motor_param_get_id_kp(m);
    case PARAM_ID_KI:           return motor_param_get_id_ki(m);
    case PARAM_ID_KD:           return motor_param_get_id_kd(m);
    case PARAM_VOLTAGE_LIMIT:   return motor_param_get_voltage_limit(m);
    case PARAM_DEADZONE:        return motor_param_get_deadzone(m);
    case PARAM_TRAJ_VMAX:       return motor_param_get_traj_vmax(m);
    case PARAM_TRAJ_TF:         return motor_param_get_traj_tf(m);
    case PARAM_VEL_LPF_TF:      return motor_param_get_vel_lpf_tf(m);
    case PARAM_CTRL_MODE:       return (float)motor_param_get_ctrl_mode(m);
    case PARAM_MAX_CURRENT:     return motor_param_get_max_current(m);
    default: return 0.0f;
    }
}

/* ================================================================
 * 范围查询
 * ================================================================ */

const motor_param_bounds_t *motor_param_get_bounds(foc_param_id_t id) {
    if (id >= PARAM_COUNT) return NULL;
    return &bounds_table[id];
}

/* ================================================================
 * 快照 & 回滚
 * ================================================================ */

void motor_param_snapshot_save(foc_motor_t *m, motor_param_snapshot_t *snap) {
    snap->angle_pid     = m->cfg.angle_pid;
    snap->vel_pid       = m->cfg.vel_pid;
    snap->iq_pid        = m->cfg.iq_pid;
    snap->id_pid        = m->cfg.id_pid;
    snap->voltage_limit = m->cfg.voltage_limit;
    snap->deadzone      = m->cfg.deadzone;
    snap->traj_vmax     = m->cfg.traj_vmax;
    snap->traj_tf       = m->cfg.traj_tf;
    snap->vel_lpf_tf    = m->cfg.vel_lpf_tf;
    snap->ctrl_mode     = m->cfg.ctrl_mode;
    snap->max_current   = m->cfg.max_current;
}

void motor_param_snapshot_restore(foc_motor_t *m, motor_param_snapshot_t *snap) {
    /* 恢复到 config 模板 */
    m->cfg.angle_pid     = snap->angle_pid;
    m->cfg.vel_pid       = snap->vel_pid;
    m->cfg.iq_pid        = snap->iq_pid;
    m->cfg.id_pid        = snap->id_pid;
    m->cfg.voltage_limit = snap->voltage_limit;
    m->cfg.deadzone      = snap->deadzone;
    m->cfg.traj_vmax     = snap->traj_vmax;
    m->cfg.traj_tf       = snap->traj_tf;
    m->cfg.vel_lpf_tf    = snap->vel_lpf_tf;
    m->cfg.ctrl_mode     = snap->ctrl_mode;
    m->cfg.max_current   = snap->max_current;

    /* 同步到运行时 PID 实例 (ISR 立即生效) */
    m->angle_pid.kp = snap->angle_pid.kp;
    m->angle_pid.ki = snap->angle_pid.ki;
    m->angle_pid.kd = snap->angle_pid.kd;
    m->angle_pid.limit_out   = snap->angle_pid.limit_out;
    m->angle_pid.limit_i_out = snap->angle_pid.limit_i_out;
    m->angle_pid.output_ramp = snap->angle_pid.ramp;
    m->angle_pid.separation_err = snap->angle_pid.sep_err;

    m->vel_pid.kp = snap->vel_pid.kp;
    m->vel_pid.ki = snap->vel_pid.ki;
    m->vel_pid.kd = snap->vel_pid.kd;
    m->vel_pid.limit_out   = snap->vel_pid.limit_out;
    m->vel_pid.limit_i_out = snap->vel_pid.limit_i_out;
    m->vel_pid.output_ramp = snap->vel_pid.ramp;
    m->vel_pid.separation_err = snap->vel_pid.sep_err;

    m->iq_pid.kp = snap->iq_pid.kp;
    m->iq_pid.ki = snap->iq_pid.ki;
    m->iq_pid.kd = snap->iq_pid.kd;
    m->iq_pid.limit_out   = snap->iq_pid.limit_out;
    m->iq_pid.limit_i_out = snap->iq_pid.limit_i_out;
    m->iq_pid.output_ramp = snap->iq_pid.ramp;
    m->iq_pid.separation_err = snap->iq_pid.sep_err;

    m->id_pid.kp = snap->id_pid.kp;
    m->id_pid.ki = snap->id_pid.ki;
    m->id_pid.kd = snap->id_pid.kd;
    m->id_pid.limit_out   = snap->id_pid.limit_out;
    m->id_pid.limit_i_out = snap->id_pid.limit_i_out;
    m->id_pid.output_ramp = snap->id_pid.ramp;
    m->id_pid.separation_err = snap->id_pid.sep_err;

    /* 同步到运行时滤波器/规划器 */
    m->planner.max_speed = snap->traj_vmax;
    m->planner.Tf        = snap->traj_tf;
    m->vel_lpf.Tf        = snap->vel_lpf_tf;
}

void motor_param_snapshot_save_default(foc_motor_t *m) {
    motor_param_snapshot_save(m, &default_snapshot);
    default_saved = true;
}

void motor_param_snapshot_restore_default(foc_motor_t *m) {
    if (default_saved)
        motor_param_snapshot_restore(m, &default_snapshot);
}

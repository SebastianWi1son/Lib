#ifndef __FOC_H
#define __FOC_H

/**
 * @file    foc.h
 * @brief   Field-Oriented Control — snake_case, 运行时模式切换
 *
 * 控制模式 (运行时可选, 可通过 上位机 或 API 切换):
 *   FOC_CTRL_VOLTAGE   — 电压模式: 位置→速度→Uq→SVPWM (默认, 无需电流传感器)
 *   FOC_CTRL_CURRENT   — 电流模式: 位置→速度→Iq_ref→IqPID→Uq→SVPWM + Id→0
 *   FOC_CTRL_OPEN_LOOP — 开环: 强制旋转磁场 (测试极对数/方向用)
 *
 * 自动探测:
 *   foc_init() 检测 get_current_cb, 非空→CURRENT, 空→VOLTAGE.
 *   用户可在 init 后覆盖 motor->cfg.ctrl_mode 手动切换.
 *   电流传感器离线时 CURRENT 自动回退 VOLTAGE (无感切换).
 *
 * @migration 从 bsp/middleware/simple_foc/foc.h:
 *   FOC_Motor_Handle_t → foc_motor_t
 *   FOC_Init() → foc_init(),  etc.
 */

#include <stdint.h>
#include "dsp.h"
#include "pid.h"

/* ================================================================
 * 控制模式
 * ================================================================ */

typedef enum {
  FOC_CTRL_VOLTAGE   = 0,  /* 电压模式 — 无电流传感器可用 */
  FOC_CTRL_CURRENT   = 1,  /* 电流模式 — 需 get_current_cb 硬件支持 */
  FOC_CTRL_OPEN_LOOP = 2,  /* 开环测试 — 仅 foc_open_loop_velocity_tick 使用 */
} foc_ctrl_mode_t;

/* ================================================================
 * 硬件抽象回调
 * ================================================================ */

typedef struct {
  /* 传感器读取 — 返回物理角度 (rad), 范围 [0, 2PI) */
  float (*get_angle_cb)(void);

  /* 相电流读取 — 返回 Ia, Ib (A). 可选, 不用时置 NULL.
   * Ic 由 Ia + Ib + Ic = 0 推出, 无需额外 ADC 通道 */
  void (*get_current_cb)(float *ia, float *ib);

  /* 三相 PWM 输出 — Ua, Ub, Uc (V), 0 ~ voltage_supply */
  void (*set_pwm_cb)(float ua, float ub, float uc);

  /* 阻塞延时 (ms) — 仅在对齐阶段使用 */
  void (*delay_ms_cb)(uint32_t ms);

  /* 电机使能 — 1 = 开 PWM, 0 = 关 PWM */
  void (*enable_cb)(uint8_t state);
} foc_hardware_t;

/* ================================================================
 * PID 参数模板
 * ================================================================ */

typedef struct {
  float kp, ki, kd;
  float limit_out, limit_i_out;
  float ramp, sep_err;
} foc_pid_param_t;

/* ================================================================
 * FOC 配置 (只读模板)
 * ================================================================ */

typedef struct {
  /* ── 电压参数 ── */
  float voltage_supply;    /* 母线电压 (V) */
  float voltage_limit;     /* 最大输出电压 (V) */
  int pole_pairs;          /* 极对数 */
  int direction;           /* 旋转方向: 1 / -1 */
  float deadzone;          /* 死区宽度 (rad) */
  float vel_lpf_tf;        /* 速度 LPF 时间常数 (s) */
  float align_voltage;     /* 对齐电压 (V): 自由轴 3.0, 受限轴 1.0 */

  /* ── 轨迹规划器 ── */
  float traj_vmax;         /* 最大速度 (rad/s) */
  float traj_tf;           /* 平滑时间常数 (s) */

  /* ── 控制模式 — init 自动探测, 运行时可改 ── */
  foc_ctrl_mode_t ctrl_mode;

  /* ── PID 参数 ── */
  foc_pid_param_t vel_pid;      /* 速度环 */
  foc_pid_param_t angle_pid;    /* 位置环 */
  foc_pid_param_t iq_pid;       /* Iq 电流环 (转矩) — ctrl_mode=CURRENT 时生效 */
  foc_pid_param_t id_pid;       /* Id 电流环 (磁通) — ctrl_mode=CURRENT 时生效 */

  /* ── 电流保护 ── */
  float max_current;       /* 最大相电流 (A), 过流检测阈值 */
} foc_config_t;

/* ================================================================
 * FOC 电机实例 (运行时状态)
 * ================================================================ */

typedef struct {
  foc_hardware_t hw;
  foc_config_t cfg;

  /* 电气对齐偏移 (rad, electrical) */
  float zero_offset_elec;

  /* 开环虚拟角度 (rad, physical) */
  float open_loop_angle;

  /* PID 实例 */
  pid_instance_t vel_pid;
  pid_instance_t angle_pid;
  pid_instance_t iq_pid;       /* Iq 电流环 — 始终初始化, ctrl_mode=CURRENT 时使用 */
  pid_instance_t id_pid;       /* Id 电流环 — 同上 */

  /* 轨迹规划器 & 滤波器 */
  dsp_traj_t planner;
  dsp_lpf_t vel_lpf;

  /* 传感器状态 */
  float raw_angle;            /* 原始角度 [0, 2PI) */
  float abs_angle;            /* 绝对角度 (多圈展开) */
  float velocity;             /* 滤波后速度 (rad/s) */

  /* 速度前馈 */
  float planned_target_prev;

  /* 多圈计数器 */
  int full_rotations;

  /* 电流状态 — ctrl_mode=CURRENT 时由 foc_current_update 填充 */
  float ia, ib, ic;           /* 相电流 (A) */
  float i_alpha, i_beta;      /* Clarke 变换后 */
  float iq, id;               /* Park 变换后 (转矩 / 磁通) */
} foc_motor_t;

/* ================================================================
 * API
 * ================================================================ */

void foc_init(foc_motor_t *motor, foc_config_t *cfg, foc_hardware_t *hw);
void foc_enable(foc_motor_t *motor);
void foc_disable(foc_motor_t *motor);
void foc_start_and_sync(foc_motor_t *motor, volatile float *cmd_target);
void foc_open_loop_velocity_tick(foc_motor_t *motor, float target_vel,
                                  float limit_voltage, float dt);
void foc_sensor_update(foc_motor_t *motor, float dt);
void foc_angle_control_tick(foc_motor_t *motor, float target_angle, float dt);

/* 过流检测 — app 层轮询用, 超 max_current 返回 1 */
int foc_current_overlimit(foc_motor_t *motor);

#endif /* __FOC_H */

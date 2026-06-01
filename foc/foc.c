#include "foc.h"
#include "foc_transform.h"    /* Clarke / Park / InvPark */
#include <math.h>

#define TWO_PI  6.28318530718f
#define SQRT3   1.73205080757f
#define PI      3.14159265359f

/* ================================================================
 * 初始化 — 含自动模式探测
 * ================================================================ */

void foc_init(foc_motor_t *motor, foc_config_t *cfg, foc_hardware_t *hw) {
  motor->cfg = *cfg;
  motor->hw = *hw;

  /* ── 滤波器 & 规划器 ── */
  dsp_lpf_init(&motor->vel_lpf, cfg->vel_lpf_tf);
  dsp_traj_init(&motor->planner, cfg->traj_vmax, cfg->traj_tf);

  /* ── 位置/速度 PID ── */
  pid_init(&motor->vel_pid,   cfg->vel_pid.kp,   cfg->vel_pid.ki,   cfg->vel_pid.kd,
           cfg->vel_pid.limit_out,   cfg->vel_pid.limit_i_out,   cfg->vel_pid.ramp,
           cfg->vel_pid.sep_err);
  pid_init(&motor->angle_pid, cfg->angle_pid.kp, cfg->angle_pid.ki, cfg->angle_pid.kd,
           cfg->angle_pid.limit_out, cfg->angle_pid.limit_i_out, cfg->angle_pid.ramp,
           cfg->angle_pid.sep_err);

  /* ── 电流 PID — 始终初始化, ctrl_mode=CURRENT 时才使用 ── */
  pid_init(&motor->iq_pid, cfg->iq_pid.kp, cfg->iq_pid.ki, cfg->iq_pid.kd,
           cfg->iq_pid.limit_out, cfg->iq_pid.limit_i_out, cfg->iq_pid.ramp,
           cfg->iq_pid.sep_err);
  pid_init(&motor->id_pid, cfg->id_pid.kp, cfg->id_pid.ki, cfg->id_pid.kd,
           cfg->id_pid.limit_out, cfg->id_pid.limit_i_out, cfg->id_pid.ramp,
           cfg->id_pid.sep_err);

  /* ── 传感器状态: 显式初始化, 防止 Sync 前被调用时误触发圈数跳变 ── */
  motor->raw_angle  = 0.0f;
  motor->abs_angle  = 0.0f;
  motor->velocity   = 0.0f;
  motor->full_rotations    = 0;
  motor->zero_offset_elec  = 0.0f;
  motor->open_loop_angle   = 0.0f;
  motor->planned_target_prev = 0.0f;

  /* ── 电流状态 ── */
  motor->ia = 0.0f;  motor->ib = 0.0f;  motor->ic = 0.0f;
  motor->i_alpha = 0.0f;  motor->i_beta = 0.0f;
  motor->iq = 0.0f;  motor->id = 0.0f;

  /* ── 自动探测控制模式 ──
   * 若用户已预设 ctrl_mode (非 0), 保留用户选择.
   * 否则根据电流传感器硬件能力自动决策. */
  if (motor->cfg.ctrl_mode == FOC_CTRL_VOLTAGE
      && motor->hw.get_current_cb != NULL) {
    motor->cfg.ctrl_mode = FOC_CTRL_CURRENT;
  }
}

/* ================================================================
 * 使能 / 关闭
 * ================================================================ */

void foc_enable(foc_motor_t *motor) {
  if (motor->hw.enable_cb)
    motor->hw.enable_cb(1);
}

void foc_disable(foc_motor_t *motor) {
  if (motor->hw.enable_cb)
    motor->hw.enable_cb(0);
  if (motor->hw.set_pwm_cb)
    motor->hw.set_pwm_cb(0.0f, 0.0f, 0.0f);
}

/* ================================================================
 * SVPWM 发波 (内部函数, 电压/电流模式共用)
 * ================================================================ */

static void foc_svpwm_write(foc_motor_t *motor, float uq, float ud,
                             float angle_elec) {
  float limit = motor->cfg.voltage_limit;

  uq = fmaxf(-limit, fminf(limit, uq));
  ud = fmaxf(-limit, fminf(limit, ud));

  /* 电气角度归一化 */
  float angle = fmodf(angle_elec - motor->zero_offset_elec, TWO_PI);
  if (angle < 0.0f)
    angle += TWO_PI;

  /* Inverse Park — 复用通用变换 */
  float u_alpha, u_beta;
  foc_inv_park_transform(uq, ud, angle, &u_alpha, &u_beta);

  /* SVPWM 中心对齐 */
  float center = motor->cfg.voltage_supply / 2.0f;
  float ua = u_alpha + center;
  float ub = (SQRT3 * u_beta - u_alpha) / 2.0f + center;
  float uc = (-u_alpha - SQRT3 * u_beta) / 2.0f + center;

  if (motor->hw.set_pwm_cb)
    motor->hw.set_pwm_cb(ua, ub, uc);
}

/* ================================================================
 * 开环速度控制 (测试用)
 * ================================================================ */

void foc_open_loop_velocity_tick(foc_motor_t *motor, float target_vel,
                                  float limit_voltage, float dt) {
  if (dt <= 0.0f)
    return;

  if (limit_voltage > motor->cfg.voltage_limit)
    limit_voltage = motor->cfg.voltage_limit;
  if (limit_voltage < 0.0f)
    limit_voltage = 0.0f;

  if (fabsf(target_vel) < 0.05f) {
    if (motor->hw.set_pwm_cb)
      motor->hw.set_pwm_cb(0.0f, 0.0f, 0.0f);
    return;
  }

  motor->open_loop_angle += target_vel * dt;
  motor->open_loop_angle = fmodf(motor->open_loop_angle, TWO_PI);
  if (motor->open_loop_angle < 0.0f)
    motor->open_loop_angle += TWO_PI;

  float angle_elec = motor->open_loop_angle * motor->cfg.pole_pairs;
  foc_svpwm_write(motor, limit_voltage, 0.0f, angle_elec);
}

/* ================================================================
 * 传感器更新
 * ================================================================ */

void foc_sensor_update(foc_motor_t *motor, float dt) {
  if (dt <= 0.0f || !motor->hw.get_angle_cb)
    return;

  float raw = motor->hw.get_angle_cb();
  float d_raw = raw - motor->raw_angle;

  if (fabsf(d_raw) > (0.8f * TWO_PI))
    motor->full_rotations += (d_raw > 0.0f) ? -1 : 1;

  motor->raw_angle = raw;

  float last_abs = motor->abs_angle;
  motor->abs_angle = (float)motor->full_rotations * TWO_PI + raw;

  float raw_vel = (motor->abs_angle - last_abs) / dt;
  motor->velocity = dsp_lpf_calc(&motor->vel_lpf, raw_vel, dt);
}

/* ================================================================
 * 电流闭环子步骤 (仅在 ctrl_mode=CURRENT 时调用)
 * ================================================================ */

static void foc_current_control_step(foc_motor_t *motor, float uq_ref,
                                      float angle_elec, float dt) {
  /* 读取相电流 (inline — ADC 由 TIM TRGO 硬件触发, 这里只读 DMA buffer) */
  if (motor->hw.get_current_cb) {
    motor->hw.get_current_cb(&motor->ia, &motor->ib);
    motor->ic = -motor->ia - motor->ib;
  }

  foc_clarke_transform(motor->ia, motor->ib,
                       &motor->i_alpha, &motor->i_beta);
  foc_park_transform(motor->i_alpha, motor->i_beta, angle_elec,
                     &motor->iq, &motor->id);

  float iq_err = uq_ref - motor->iq;
  float id_err = 0.0f - motor->id;
  float uq = pid_calculate(&motor->iq_pid, iq_err, dt);
  float ud = pid_calculate(&motor->id_pid, id_err, dt);

  foc_svpwm_write(motor, uq, ud, angle_elec);
}

/* ================================================================
 * 角度闭环控制 (核心)
 *
 * 控制链路:
 *   target → traj planner → pos PID → vel PID → [Iq/Id PID] → SVPWM
 *
 * 模式选择 (运行时, 通过 motor->cfg.ctrl_mode):
 *   VOLTAGE:   速度 PID → Uq → SVPWM  (默认)
 *   CURRENT:   速度 PID → Iq_ref → Iq PID → Uq → SVPWM
 *              Id → 0  (去磁控制)
 *
 * 故障回退: 电流传感器离线时 CURRENT 自动退为 VOLTAGE
 * ================================================================ */

void foc_angle_control_tick(foc_motor_t *motor, float target_angle, float dt) {
  if (dt <= 0.0f)
    return;

  /* Step 1-6: 轨迹规划 → 位置环 → 速度环 (所有模式共用) */
  float planned_target = dsp_traj_calc(&motor->planner, target_angle, dt);

  float err_ang = planned_target - motor->abs_angle;
  err_ang = dsp_soft_deadzone(err_ang, motor->cfg.deadzone);

  float target_vel = pid_calculate(&motor->angle_pid, err_ang, dt);

  target_vel += (planned_target - motor->planned_target_prev) / dt;
  motor->planned_target_prev = planned_target;

  if (target_vel > motor->cfg.traj_vmax)
    target_vel = motor->cfg.traj_vmax;
  else if (target_vel < -motor->cfg.traj_vmax)
    target_vel = -motor->cfg.traj_vmax;

  float err_vel = target_vel - motor->velocity;
  float uq_ref = pid_calculate(&motor->vel_pid, err_vel, dt);

  /* 电角度 */
  float angle_elec =
      motor->abs_angle * motor->cfg.pole_pairs * motor->cfg.direction;

  /* Step 7: 根据控制模式选择发波路径 */
  switch (motor->cfg.ctrl_mode) {

  case FOC_CTRL_CURRENT:
    if (motor->hw.get_current_cb) {
      /* ─── 电流闭环 ─── */
      foc_current_control_step(motor, uq_ref, angle_elec, dt);
      break;
    }
    /* 电流传感器离线 → 自动回退电压模式 */
    /* fallthrough */

  case FOC_CTRL_VOLTAGE:
  default:
    /* ─── 电压模式 ─── */
    foc_svpwm_write(motor, uq_ref, 0.0f, angle_elec);
    break;
  }
}

/* ================================================================
 * 电机对齐 & 同步 (启动时调用, 阻塞 ~2s)
 * ================================================================ */

void foc_start_and_sync(foc_motor_t *motor, volatile float *cmd_target) {
  motor->zero_offset_elec = 0.0f;

  float v_align = motor->cfg.align_voltage;
  if (v_align <= 0.0f)
    v_align = 3.0f;

  int steps = 1000;
  int delay_per_step = 2;

  for (int i = 1; i <= steps; i++) {
    float current_v = v_align * ((float)i / (float)steps);
    foc_svpwm_write(motor, current_v, 0.0f, 1.5f * PI);
    if (motor->hw.delay_ms_cb)
      motor->hw.delay_ms_cb(delay_per_step);
  }

  /* [Bugfix B2] get_angle_cb 非空检查 — 否则硬故障 */
  if (!motor->hw.get_angle_cb) return;

  float a1, a2;
  int retry = 0;
  do {
    if (motor->hw.delay_ms_cb)
      motor->hw.delay_ms_cb(50);
    a1 = motor->hw.get_angle_cb();
    if (motor->hw.delay_ms_cb)
      motor->hw.delay_ms_cb(50);
    a2 = motor->hw.get_angle_cb();
    retry++;
  } while (fabsf(a1 - a2) > 0.1f && retry < 3);

  float settled = a2;

  motor->zero_offset_elec =
      settled * motor->cfg.pole_pairs * motor->cfg.direction;

  /* 对齐完成 → 断电, 交由 ISR 闭环接管 */
  foc_svpwm_write(motor, 0.0f, 0.0f, 0.0f);

  motor->abs_angle = settled;
  motor->raw_angle = settled;
  motor->planned_target_prev = settled;
  *cmd_target = settled;

  /* 规划器状态同步到对齐后角度 */
  motor->planner.ramp_target = settled;
  motor->planner.filter1 = settled;
  motor->planner.filter2 = settled;
}

/* ================================================================
 * 过流检测 — app 层轮询用
 * 仅在 FOC_CTRL_CURRENT 模式下有效 (VOLTAGE 模式无电流测量).
 * VOLTAGE 模式返回 0, 此时应依赖 I²t 热保护或电压限幅.
 * ================================================================ */

int foc_current_overlimit(foc_motor_t *motor) {
  if (motor->cfg.ctrl_mode != FOC_CTRL_CURRENT) return 0;
  float amp = sqrtf(motor->iq * motor->iq + motor->id * motor->id);
  return (amp > motor->cfg.max_current) ? 1 : 0;
}

/**
 ******************************************************************************
 * @file    pid.c
 * @brief   Industrial-grade PID controller with derivative-on-measurement,
 *          trapezoidal integration, anti-windup, and integral separation.
 *          工业级 PID 控制器：微分先行 / 梯形积分 / 抗饱和 / 积分分离。
 * @note    Optimised for STM32F103 (no FPU): reciprocal pre-computation turns
 *          divides into multiplies. On FPU-equipped targets the reciprocal
 *          step is skipped — the FPU divides in one cycle anyway.
 *          针对 F103（无 FPU）优化：预计算倒数变除为乘；有 FPU 的平台跳过此步骤。
 ******************************************************************************
 */

#include "pid.h"

/* ==================================================================== */
/* === Private Helpers                                                 === */
/* ==================================================================== */

/** Float absolute value. */
static inline float pid_fabs(float x) {
  return (x < 0.0f) ? -x : x;
}

/** Symmetric clamp: constrain val to [-limit, +limit]. */
static inline float pid_constrain(float val, float limit) {
  if (val > limit)  return limit;
  if (val < -limit) return -limit;
  return val;
}

/* ==================================================================== */
/* === 1. Initialisation                                               === */
/* ==================================================================== */

/**
 * @brief   Initialise a PID controller instance.
 *          初始化 PID 控制器实例。
 * @param   pid                PID instance pointer
 * @param   kp                 Proportional gain
 * @param   ki                 Integral gain
 * @param   kd                 Derivative gain
 * @param   limit_out          Total output clamp (symmetric)
 * @param   limit_integral     Integral anti-windup clamp (symmetric)
 * @param   integral_sep_thresh Integral separation threshold (0 = disabled)
 * @param   output_max_rate    Output slew rate limit in units/s (0 = disabled)
 * @param   d_filter_Tf        D-term low-pass filter time constant
 */
void pid_init(pid_instance_t *pid, float kp, float ki, float kd,
              float limit_out, float limit_integral,
              float integral_sep_thresh, float output_max_rate,
              float d_filter_Tf) {

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->limit_out = limit_out;
  pid->limit_integral = limit_integral;
  pid->integral_sep_thresh = integral_sep_thresh;

  pid->integral = 0.0f;
  pid->error_prev = 0.0f;
  pid->measure_prev = 0.0f;

  dsp_lpf_init(&pid->d_filter, d_filter_Tf);
  dsp_ramp_init(&pid->output_ramp, output_max_rate);
}

/* ==================================================================== */
/* === 2. PID Calculation                                              === */
/* ==================================================================== */

/**
 * @brief   Compute one PID iteration.
 *          计算一次 PID 迭代。
 *
 *          Algorithm stages:
 *          1. P — proportional on error.
 *          2. I — trapezoidal (Tustin) integration with anti-windup clamping
 *             and integral separation.
 *          3. D — derivative-on-measurement (eliminates derivative kick).
 *             The raw derivative is low-pass filtered via dsp_lpf_t.
 *          4. Output — symmetric clamping + slew rate limiting via dsp_ramp_t.
 *          5. State update.
 *
 *          算法流程：P 比例 → I 梯形积分+抗饱和+积分分离 → D 微分先行+低通滤波
 *          → 输出对称限幅+斜率限制 → 状态更新。
 *
 * @param   pid      PID instance pointer
 * @param   target   Setpoint (desired value)
 * @param   measure  Current measurement (actual value)
 * @param   dt       Delta time in seconds since last call
 * @return           Control output
 */
float pid_calculate(pid_instance_t *pid, float target, float measure, float dt) {
  if (dt <= 0.0f || dt > 0.5f)
    dt = 0.001f;

  float error = target - measure;

  /* P */
  float proportional = pid->kp * error;

  /* I — trapezoidal integration, anti-windup, separation */
  float temp_integral = pid->integral
                      + pid->ki * dt * 0.5f * (error + pid->error_prev);
  temp_integral = pid_constrain(temp_integral, pid->limit_integral);

  if (pid->integral_sep_thresh <= 0.0f
      || pid_fabs(error) <= pid->integral_sep_thresh) {
    pid->integral = temp_integral;
  }

  /* D — derivative on measurement, low-pass filtered */
#if defined(__FPU_PRESENT) && (__FPU_PRESENT == 1U)
  float raw_derivative = -pid->kd * (measure - pid->measure_prev) / dt;
#else
  float inv_dt = 1.0f / dt;
  float raw_derivative = -pid->kd * (measure - pid->measure_prev) * inv_dt;
#endif
  float derivative = dsp_lpf_calc(&pid->d_filter, raw_derivative, dt);

  /* Output — clamp + slew rate limit */
  float output = proportional + pid->integral + derivative;
  output = pid_constrain(output, pid->limit_out);

  if (pid->output_ramp.max_rate > 0.0f)
    output = dsp_ramp_calc(&pid->output_ramp, output, dt);

  /* State update */
  pid->error_prev = error;
  pid->measure_prev = measure;
  return output;
}

/* ==================================================================== */
/* === 3. DLC: Cubic-Kp PID (三次方动态Kp)                            === */
/* ==================================================================== */

/**
 * @brief   PID calculation with cubic Kp scaling.
 *          Kp is dynamically scaled by 1.0 + r³, where r = |error| / error_range
 *          (clamped to [0,1]). At small errors Kp stays near baseline,
 *          suppressing overshoot; at large errors Kp ramps up to 2× for
 *          aggressive correction. The cubic curve gives a gentle transition.
 *          三次方动态Kp PID：小误差时Kp贴近基准值减少过调，大误差时Kp平滑升至2倍
 *          快速修正。三次曲线确保过渡柔和，消除分段阈值引起的力跳变。
 *
 *          Usage — replace this:
 *            float saved = pid->kp;
 *            pid->kp = saved * (1.0f + r*r*r);
 *            float out = pid_calculate(pid, target, measure, dt);
 *            pid->kp = saved;
 *          With this one-liner:
 *            float out = pid_calculate_cubic_kp(pid, target, measure, dt, range);
 *
 * @param   pid         PID instance pointer
 * @param   target      Setpoint
 * @param   measure     Current measurement
 * @param   dt          Delta time in seconds
 * @param   error_range Normalisation range: |error| at which r = 1.0
 *                      (i.e. the max error you expect in normal operation)
 * @return              Control output
 */
float pid_calculate_cubic_kp(pid_instance_t *pid, float target, float measure,
                             float dt, float error_range) {
  float error = target - measure;
  float r = pid_constrain(pid_fabs(error) / error_range, 1.0f);
  float scale = 1.0f + r * r * r;

  float saved_kp = pid->kp;
  pid->kp = saved_kp * scale;
  float output = pid_calculate(pid, target, measure, dt);
  pid->kp = saved_kp;
  return output;
}

/* ==================================================================== */
/* === 4. Reset                                                        === */
/* ==================================================================== */

/**
 * @brief   Reset PID internal state (integral, history, D-filter, ramp).
 *          Call on restart or emergency stop recovery.
 *          重置 PID 内部状态（积分/历史值/D项滤波器/斜率限制器），用于重启或急停恢复。
 * @param   pid  PID instance pointer
 */
void pid_reset(pid_instance_t *pid) {
  pid->integral = 0.0f;
  pid->error_prev = 0.0f;
  pid->measure_prev = 0.0f;

  dsp_lpf_init(&pid->d_filter, pid->d_filter.Tf);
  dsp_ramp_init(&pid->output_ramp, pid->output_ramp.max_rate);
}

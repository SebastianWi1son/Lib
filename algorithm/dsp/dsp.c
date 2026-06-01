/**
 ******************************************************************************
 * @file    dsp.c
 * @brief   Digital Signal Processing (DSP) toolbox for motor control loops.
 *          电机控制环路数字信号处理 (DSP) 工具箱。
 * @note    All functions mandate a dt (delta-time) parameter for complete
 *          time decoupling — no blocking delays, no hardware timers assumed.
 *          所有函数强制传入 dt 实现时间解耦，不依赖阻塞延时或硬件定时器。
 ******************************************************************************
 */

#include "dsp.h"

/* ==================================================================== */
/* === Private Helpers                                                 === */
/* ==================================================================== */

/** Float absolute value — avoids external math.h dependency. */
static inline float dsp_fabs(float x) {
  return (x < 0.0f) ? -x : x;
}

/** First-order LPF smoothing factor: alpha = dt / (Tf + dt). */
static inline float dsp_alpha(float dt, float Tf) {
  return dt / (Tf + dt);
}

/* ==================================================================== */
/* === 1. First-Order Low-Pass Filter                                  === */
/* ==================================================================== */

/**
 * @brief   Initialize the first-order low-pass filter.
 *          初始化一阶低通滤波器。
 * @param   lpf           LPF instance pointer
 * @param   time_constant Filter time constant (Tf). Larger = stronger
 *                        filtering but more phase delay.
 */
void dsp_lpf_init(dsp_lpf_t *lpf, float time_constant) {
  lpf->Tf = time_constant;
  lpf->prev_out = 0.0f;
}

/**
 * @brief   Compute one LPF iteration.
 *          计算一次低通滤波迭代。
 * @param   lpf  LPF instance pointer
 * @param   raw  Raw input sample
 * @param   dt   Delta time in seconds since last call
 * @return       Filtered output
 */
float dsp_lpf_calc(dsp_lpf_t *lpf, float raw, float dt) {
  if (dt <= 0.0f)
    return raw;

  float alpha = dsp_alpha(dt, lpf->Tf);
  float output = alpha * raw + (1.0f - alpha) * lpf->prev_out;

  lpf->prev_out = output;
  return output;
}

/* ==================================================================== */
/* === 2. Slew Rate Limiter                                            === */
/* ==================================================================== */

/**
 * @brief   Initialize the slew rate limiter.
 *          初始化斜率限制器。
 * @param   ramp     Ramp limiter instance pointer
 * @param   max_rate Maximum allowable rate of change per second
 */
void dsp_ramp_init(dsp_ramp_t *ramp, float max_rate) {
  ramp->max_rate = max_rate;
  ramp->prev_out = 0.0f;
}

/**
 * @brief   Compute one rate-limited iteration.
 *          计算一次斜率限制迭代。
 * @param   ramp   Ramp limiter instance pointer
 * @param   target Desired target value
 * @param   dt     Delta time in seconds since last call
 * @return         Rate-limited output
 */
float dsp_ramp_calc(dsp_ramp_t *ramp, float target, float dt) {
  float max_step = ramp->max_rate * dt;
  float out = target;

  if (target > ramp->prev_out + max_step)
    out = ramp->prev_out + max_step;
  else if (target < ramp->prev_out - max_step)
    out = ramp->prev_out - max_step;

  ramp->prev_out = out;
  return out;
}

/* ==================================================================== */
/* === 3. 2nd-Order Trajectory Planner                                 === */
/* ==================================================================== */

/**
 * @brief   Initialize the 2nd-order trajectory planner for S-curve
 *          smoothing.
 *          初始化二阶轨迹规划器，生成 S 型曲线。
 * @param   traj          Trajectory planner instance pointer
 * @param   max_rate      Maximum allowable rate of change (trapezoidal slope)
 * @param   time_constant Filter time constant for the two LPF stages
 */
void dsp_traj_init(dsp_traj_t *traj, float max_rate, float time_constant) {
  traj->Tf = time_constant;
  dsp_ramp_init(&traj->inner_ramp, max_rate);
  traj->filter1 = 0.0f;
  traj->filter2 = 0.0f;
}

/**
 * @brief   Compute one trajectory-planning iteration.
 *          Stage 1: dsp_ramp_calc (trapezoidal rate limiting).
 *          Stage 2: two cascaded LPFs (S-curve smoothing).
 *          计算一次轨迹规划迭代：斜坡限速 + 两级LPF S曲线平滑。
 * @param   traj   Trajectory planner instance pointer
 * @param   target Desired final value
 * @param   dt     Delta time in seconds since last call
 * @return         Smoothed trajectory value
 */
float dsp_traj_calc(dsp_traj_t *traj, float target, float dt) {
  if (dt <= 0.0f)
    return target;

  float ramped = dsp_ramp_calc(&traj->inner_ramp, target, dt);

  float alpha = dsp_alpha(dt, traj->Tf);
  traj->filter1 += alpha * (ramped - traj->filter1);
  traj->filter2 += alpha * (traj->filter1 - traj->filter2);

  return traj->filter2;
}

/**
 * @brief   Reset trajectory planner internal state (ramp, both filter stages).
 *          Preserves configuration (Tf, max_rate).
 *          重置轨迹规划器内部状态，保留配置参数。
 * @param   traj  Trajectory planner instance pointer
 */
void dsp_traj_reset(dsp_traj_t *traj) {
  dsp_ramp_init(&traj->inner_ramp, traj->inner_ramp.max_rate);
  traj->filter1 = 0.0f;
  traj->filter2 = 0.0f;
}

/* ==================================================================== */
/* === 4. Soft Deadzone                                                === */
/* ==================================================================== */

/**
 * @brief   Apply a parabolic smooth-deadzone to suppress static overshoot
 *          and low-frequency hunting near the setpoint.
 *          抛物线软死区，抑制稳态超调和低频震荡。
 * @param   error          Current system error
 * @param   deadzone_range Threshold below which the output is smoothly
 *                         attenuated (parabolic profile)
 * @return                 Processed error (attenuated inside the deadzone,
 *                         unchanged outside)
 */
float dsp_soft_deadzone(float error, float deadzone_range) {
  float abs_error = dsp_fabs(error);
  if (abs_error < deadzone_range)
    return error * (abs_error / deadzone_range);
  return error;
}

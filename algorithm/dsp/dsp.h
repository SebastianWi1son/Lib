#ifndef __DSP_H
#define __DSP_H

#include "main.h"

/* ==================================================================== */
/* === DSP Toolbox for Motor Control                                  === */
/* ==================================================================== */

/* ---- Filter & Control Structures ---- */

/*
 * First-Order Low-Pass Filter — eliminates high-frequency noise.
 * 一阶低通滤波器，滤除高频噪声。
 */
typedef struct {
  float Tf;       // filter time constant (larger → stronger filtering, more delay)
  float prev_out; // previous output
} dsp_lpf_t;

void dsp_lpf_init(dsp_lpf_t *lpf, float time_constant);
float dsp_lpf_calc(dsp_lpf_t *lpf, float raw, float dt);

/*
 * Slew Rate Limiter — clamps transient spikes.
 * 斜率限制器，抑制瞬态尖峰。
 */
typedef struct {
  float max_rate; // maximum allowable rate of change
  float prev_out; // previous output
} dsp_ramp_t;

void dsp_ramp_init(dsp_ramp_t *ramp, float max_rate);
float dsp_ramp_calc(dsp_ramp_t *ramp, float target, float dt);

/*
 * 2nd-Order Trajectory Planner — S-curve smoothing, suppresses startup kickback.
 * Built from: ramp (rate limiter) + 2 cascaded LPFs (S-curve shaping).
 * 二阶轨迹规划器，S型曲线平滑，抑制启动冲击。
 * 由斜坡限制器 + 两级低通滤波器级联构成。
 */
typedef struct {
  float Tf;              // filter time constant for the two LPF stages
  dsp_ramp_t inner_ramp; // stage 1: rate limiter (trapezoidal profile)
  float filter1;         // stage 2: first LPF state
  float filter2;         // stage 2: second LPF state
} dsp_traj_t;

void dsp_traj_init(dsp_traj_t *traj, float max_rate, float time_constant);
float dsp_traj_calc(dsp_traj_t *traj, float target, float dt);
void dsp_traj_reset(dsp_traj_t *traj);

/*
 * Soft Deadzone — parabolic smooth decay, restrains static overshoot.
 * 抛物线软死区，抑制稳态超调和低频震荡。
 */
float dsp_soft_deadzone(float error, float deadzone_range);

#endif

#ifndef __PID_H
#define __PID_H

#include "dsp.h"
#include "main.h"

/*
 * Industrial-grade PID controller with derivative-on-measurement,
 * trapezoidal integration, and integral separation.
 * 工业级PID控制器：微分先行 + 梯形积分 + 积分分离。
 */
typedef struct pid_instance_s {
  /* ---- gains ---- */
  float kp;
  float ki;
  float kd;

  /* ---- limits ---- */
  float limit_out;           // total output clamp (symmetric)
  float limit_integral;      // integral anti-windup clamp (symmetric)
  float integral_sep_thresh; // integral separation threshold (0 = disabled)

  /* ---- state ---- */
  float integral;
  float error_prev;
  float measure_prev;

  /* ---- D-term low-pass filter ---- */
  dsp_lpf_t d_filter;

  /* ---- output slew rate limiter (0 max_rate = disabled) ---- */
  dsp_ramp_t output_ramp;
} pid_instance_t;

void pid_init(pid_instance_t *pid, float kp, float ki, float kd,
              float limit_out, float limit_integral,
              float integral_sep_thresh, float output_max_rate,
              float d_filter_Tf);

float pid_calculate(pid_instance_t *pid, float target, float measure, float dt);

/* ---- DLC: optional extension functions ---- */
float pid_calculate_cubic_kp(pid_instance_t *pid, float target, float measure,
                             float dt, float error_range);

void pid_reset(pid_instance_t *pid);

#endif

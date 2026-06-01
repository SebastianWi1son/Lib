#include "wheel.h"

void wheel_init(wheel_t *wheel, uint8_t id, float kp, float ki, float kd,
                wheel_get_rpm_cb_t get_cb, wheel_set_pwm_cb_t set_cb) {
  wheel->motor_id = id;
  wheel->target_rpm = 0.0f;
  wheel->current_rpm = 0.0f;

  // get cb linked
  wheel->get_rpm_cb = get_cb;
  wheel->set_pwm_cb = set_cb;

  // DSP traj tools
  dsp_traj_init(&wheel->target_planner, 1500.0f, 0.02f);

  // PID speed loop
  pid_init(&wheel->speed_pid, kp, ki, kd, 3600.0f, 1200.0f, 20.0f, 10000.0f, 0.005f);
}

void wheel_set_pid_params(wheel_t *wheel, float kp, float ki, float kd) {
  // 直接更新内部 PID 控制器的参数
  wheel->speed_pid.kp = kp;
  wheel->speed_pid.ki = ki;
  wheel->speed_pid.kd = kd;

  pid_reset(&wheel->speed_pid);
}

void wheel_set_target(wheel_t *wheel, float target_rpm) {
  wheel->target_rpm = target_rpm;
}

void wheel_update(wheel_t *wheel, float dt) {
  // 1. 测速：通过回调函数调用底层的 encoder_get_rpm
  // 只要绑定了回调，即使指针为空也能做个保护 (企业级防呆)
  if (wheel->get_rpm_cb != 0) {
    wheel->current_rpm = wheel->get_rpm_cb(wheel->motor_id, dt);
  }

  // 2. 规划：S 型曲线滤网
  float planned_rpm =
      dsp_traj_calc(&wheel->target_planner, wheel->target_rpm, dt);

  // 3. PID 计算
  float pwm_out = pid_calculate(&wheel->speed_pid, planned_rpm,
                                wheel->current_rpm, dt);

  // 4. 执行：通过回调函数调用底层的 motor_set_pwm
  if (wheel->set_pwm_cb != 0) {
    wheel->set_pwm_cb(wheel->motor_id, (int16_t)pwm_out);
  }
}

void wheel_stop(wheel_t *wheel) {
  wheel->target_rpm = 0.0f;
  pid_reset(&wheel->speed_pid);
  dsp_traj_reset(&wheel->target_planner);

  // 紧急制动
  if (wheel->set_pwm_cb != 0) {
    wheel->set_pwm_cb(wheel->motor_id, 0);
  }
}
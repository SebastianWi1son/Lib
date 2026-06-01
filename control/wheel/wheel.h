#ifndef __WHEEL_H
#define __WHEEL_H

#include "dsp.h"
#include "pid.h"
#include <stdint.h>


// ==========================================
// 定义硬件接口的回调函数类型
// ==========================================
typedef float (*wheel_get_rpm_cb_t)(uint8_t motor_id, float dt);
typedef float (*wheel_get_current_cb_t)(uint8_t motor_id);

typedef void (*wheel_set_pwm_cb_t)(uint8_t motor_id, int16_t pwm_val);

// 轮子对象结构体
typedef struct {
    uint8_t motor_id;
    float target_rpm;
    float current_rpm;

    float target_current;   // 速度环的输出作为电流环的输入
    float actual_current;   // 实际采样电流

  dsp_traj_t target_planner;
  pid_instance_t speed_pid;

  // ✨ 回调函数指针 (轮子的神经接口)
  wheel_get_rpm_cb_t get_rpm_cb;
  wheel_set_pwm_cb_t set_pwm_cb;
} wheel_t;

// --- API 接口 (注意 init 函数的参数变了) ---
void wheel_set_pid_params(wheel_t *wheel, float kp, float ki, float kd);
void wheel_init(wheel_t *wheel, uint8_t id, float kp, float ki, float kd,
                wheel_get_rpm_cb_t get_cb, wheel_set_pwm_cb_t set_cb);

void wheel_set_target(wheel_t *wheel, float target_rpm);
void wheel_update(wheel_t *wheel, float dt);
void wheel_stop(wheel_t *wheel);

#endif /* __CTRL_WHEEL_H */
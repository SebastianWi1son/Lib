#include "chassis_config.h"
#include "motor.h"
#include "tim.h"

void motor_init(void) {
  HAL_TIM_PWM_Start(HW_MOTOR_HTIM, HW_MOTOR_L_CH1);
  HAL_TIM_PWM_Start(HW_MOTOR_HTIM, HW_MOTOR_L_CH2);
  HAL_TIM_PWM_Start(HW_MOTOR_HTIM, HW_MOTOR_R_CH1);
  HAL_TIM_PWM_Start(HW_MOTOR_HTIM, HW_MOTOR_R_CH2);
}

void motor_set_pwm(uint8_t motor_id, int16_t pwm_val) {
  if (motor_id == 1)
    pwm_val = (int16_t)(pwm_val * MOTOR_LEFT_DIR);
  else if (motor_id == 2)
    pwm_val = (int16_t)(pwm_val * MOTOR_RIGHT_DIR);

  if (pwm_val > MOTOR_PWM_MAX)
    pwm_val = MOTOR_PWM_MAX;
  else if (pwm_val < -MOTOR_PWM_MAX)
    pwm_val = -MOTOR_PWM_MAX;

  if (motor_id == 1) {
    if (pwm_val >= 0) {
      __HAL_TIM_SET_COMPARE(HW_MOTOR_HTIM, HW_MOTOR_L_CH1, MOTOR_PWM_MAX);
      __HAL_TIM_SET_COMPARE(HW_MOTOR_HTIM, HW_MOTOR_L_CH2,
                            MOTOR_PWM_MAX - pwm_val);
    } else {
      __HAL_TIM_SET_COMPARE(HW_MOTOR_HTIM, HW_MOTOR_L_CH1,
                            MOTOR_PWM_MAX + pwm_val);
      __HAL_TIM_SET_COMPARE(HW_MOTOR_HTIM, HW_MOTOR_L_CH2, MOTOR_PWM_MAX);
    }
  } else if (motor_id == 2) {
    if (pwm_val >= 0) {
      __HAL_TIM_SET_COMPARE(HW_MOTOR_HTIM, HW_MOTOR_R_CH1, MOTOR_PWM_MAX);
      __HAL_TIM_SET_COMPARE(HW_MOTOR_HTIM, HW_MOTOR_R_CH2,
                            MOTOR_PWM_MAX - pwm_val);
    } else {
      __HAL_TIM_SET_COMPARE(HW_MOTOR_HTIM, HW_MOTOR_R_CH1,
                            MOTOR_PWM_MAX + pwm_val);
      __HAL_TIM_SET_COMPARE(HW_MOTOR_HTIM, HW_MOTOR_R_CH2, MOTOR_PWM_MAX);
    }
  }
}

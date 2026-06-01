#include "m2804.h"
#include "tim.h"

#define M2804_TIM (&htim8)
#define M2804_U_BUS 12.6f
#define M2804_PWM_PERIOD 6000

void m2804_run_sw(uint8_t runflag) {
  if (runflag) {
    HAL_TIM_PWM_Start(M2804_TIM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(M2804_TIM, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(M2804_TIM, TIM_CHANNEL_3);
  } else {
    HAL_TIM_PWM_Stop(M2804_TIM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(M2804_TIM, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(M2804_TIM, TIM_CHANNEL_3);
  }
}

void m2804_set_pwm(float Ua, float Ub, float Uc) {
  __HAL_TIM_SET_COMPARE(M2804_TIM, TIM_CHANNEL_1,
                        (uint32_t)(Ua / M2804_U_BUS * M2804_PWM_PERIOD));
  __HAL_TIM_SET_COMPARE(M2804_TIM, TIM_CHANNEL_2,
                        (uint32_t)(Ub / M2804_U_BUS * M2804_PWM_PERIOD));
  __HAL_TIM_SET_COMPARE(M2804_TIM, TIM_CHANNEL_3,
                        (uint32_t)(Uc / M2804_U_BUS * M2804_PWM_PERIOD));
}

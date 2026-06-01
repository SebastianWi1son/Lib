#include "m3510.h"
#include "tim.h"

#define M3510_TIM (&htim1)
#define M3510_U_BUS 12.6f
#define M3510_PWM_PERIOD 6000

void m3510_run_sw(uint8_t runflag) {
  if (runflag) {
    HAL_TIM_PWM_Start(M3510_TIM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(M3510_TIM, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(M3510_TIM, TIM_CHANNEL_3);
  } else {
    HAL_TIM_PWM_Stop(M3510_TIM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(M3510_TIM, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(M3510_TIM, TIM_CHANNEL_3);
  }
}

void m3510_set_pwm(float Ua, float Ub, float Uc) {
  __HAL_TIM_SET_COMPARE(M3510_TIM, TIM_CHANNEL_1,
                        (uint32_t)(Ua / M3510_U_BUS * M3510_PWM_PERIOD));
  __HAL_TIM_SET_COMPARE(M3510_TIM, TIM_CHANNEL_2,
                        (uint32_t)(Ub / M3510_U_BUS * M3510_PWM_PERIOD));
  __HAL_TIM_SET_COMPARE(M3510_TIM, TIM_CHANNEL_3,
                        (uint32_t)(Uc / M3510_U_BUS * M3510_PWM_PERIOD));
}
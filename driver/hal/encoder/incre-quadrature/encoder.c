#include "chassis_config.h"
#include "encoder.h"
#include "tim.h"

static uint16_t encoder_last_cnt1 = 0;
static uint16_t encoder_last_cnt2 = 0;
static int32_t encoder_absolute_loc1 = 0;
static int32_t encoder_absolute_loc2 = 0;

void encoder_init(void) {
  HAL_TIM_Encoder_Start(HW_ENC_L_HTIM, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(HW_ENC_R_HTIM, TIM_CHANNEL_ALL);
  encoder_last_cnt1 = HW_ENC_L_TIM->CNT;
  encoder_last_cnt2 = HW_ENC_R_TIM->CNT;
}

float encoder_get_rpm(uint8_t motor_id, float dt) {
  int16_t delta_pulses = 0;
  uint16_t current_cnt = 0;

  if (dt <= 0.0f)
    return 0.0f;

  if (motor_id == 1) {
    current_cnt = HW_ENC_L_TIM->CNT;
    int16_t raw_delta = (int16_t)(current_cnt - encoder_last_cnt1);
    delta_pulses = raw_delta * ENCODER_LEFT_DIR;
    encoder_last_cnt1 = current_cnt;
    encoder_absolute_loc1 += delta_pulses;
  } else if (motor_id == 2) {
    current_cnt = HW_ENC_R_TIM->CNT;
    int16_t raw_delta = (int16_t)(current_cnt - encoder_last_cnt2);
    delta_pulses = raw_delta * ENCODER_RIGHT_DIR;
    encoder_last_cnt2 = current_cnt;
    encoder_absolute_loc2 += delta_pulses;
  }

  return ((float)delta_pulses / ENCODER_PPR) * (1.0f / dt) * 60.0f;
}

int32_t encoder_get_location(uint8_t motor_id) {
  if (motor_id == 1)
    return encoder_absolute_loc1;
  else if (motor_id == 2)
    return encoder_absolute_loc2;
  return 0;
}

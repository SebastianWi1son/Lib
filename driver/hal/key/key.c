#include "chassis_config.h"
#include "key.h"

static uint8_t key_num = 0;

void key_init(void) {
  // GPIO configured by CubeMX — nothing to do
}

static uint8_t key_get_state(void) {
  if (HAL_GPIO_ReadPin(HW_KEY1_PORT, HW_KEY1_PIN) == GPIO_PIN_RESET)
    return 1;
  if (HAL_GPIO_ReadPin(HW_KEY2_PORT, HW_KEY2_PIN) == GPIO_PIN_RESET)
    return 2;
  return 0;
}

void key_tick(void) {
  static uint8_t count = 0;
  static uint8_t curr_state = 0, prev_state = 0;

  count++;
  if (count >= HW_KEY_DEBOUNCE_MS) {
    count = 0;

    prev_state = curr_state;
    curr_state = key_get_state();

    // Release-edge detection: fire on release, immune to press bounce
    if (curr_state == 0 && prev_state != 0)
      key_num = prev_state;
  }
}

uint8_t key_get_num(void) {
  uint8_t temp = 0;
  if (key_num) {
    temp = key_num;
    key_num = 0;
    return temp;
  }
  return 0;
}

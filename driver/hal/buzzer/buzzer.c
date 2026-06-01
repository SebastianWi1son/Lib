#include "chassis_config.h"
#include "buzzer.h"

// Non-blocking beep state machine
static uint8_t target_beeps = 0;
static uint8_t current_beeps = 0;
static uint16_t beep_interval = 0;
static uint32_t last_toggle_tick = 0;
static uint8_t buzzer_state = 0; // 0=off, 1=on
static uint8_t is_active = 0;    // 0=idle, 1=running

void buzzer_init(void) {
#if HW_BUZZER_ACTIVE == 0
  HAL_GPIO_WritePin(HW_BUZZER_PORT, HW_BUZZER_PIN, GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(HW_BUZZER_PORT, HW_BUZZER_PIN, GPIO_PIN_RESET);
#endif
  is_active = 0;
}

void buzzer_set(uint8_t state) {
  if (state == 1) {
#if HW_BUZZER_ACTIVE == 0
    HAL_GPIO_WritePin(HW_BUZZER_PORT, HW_BUZZER_PIN, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(HW_BUZZER_PORT, HW_BUZZER_PIN, GPIO_PIN_SET);
#endif
  } else {
#if HW_BUZZER_ACTIVE == 0
    HAL_GPIO_WritePin(HW_BUZZER_PORT, HW_BUZZER_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(HW_BUZZER_PORT, HW_BUZZER_PIN, GPIO_PIN_RESET);
#endif
  }
}

void buzzer_on(void) { buzzer_set(1); }

void buzzer_off(void) { buzzer_set(0); }

// Non-blocking beep: starts a sequence of `count` toggles, `duration_ms` each half-period.
// 非阻塞蜂鸣：启动 count 次翻转的序列，每次半周期为 duration_ms。
void buzzer_beep(uint8_t count, uint16_t duration_ms) {
  if (count == 0) return;

  target_beeps = count;
  current_beeps = 0;
  beep_interval = duration_ms;

  is_active = 1;
  buzzer_state = 1;
  buzzer_set(1);
  last_toggle_tick = HAL_GetTick();
}

// Tick function — call from main loop or a timer ISR.
// 状态机刷新函数——在主循环或定时器中断中调用。
void buzzer_update(void) {
  if (!is_active) return;

  if (HAL_GetTick() - last_toggle_tick >= beep_interval) {
    last_toggle_tick = HAL_GetTick();

    if (buzzer_state == 1) {
      // Currently on — turn off and count one completed beep
      buzzer_state = 0;
      buzzer_set(0);
      current_beeps++;

      if (current_beeps >= target_beeps) {
        is_active = 0;
      }
    } else {
      // Currently off — turn on for the next beep
      buzzer_state = 1;
      buzzer_set(1);
    }
  }
}

uint8_t buzzer_is_busy(void) { return is_active; }

#include "chassis_config.h"
#include "gray.h"

#define GRAY_AD0_HIGH()  (HW_GRAY_AD0_PORT->BSRR = HW_GRAY_AD0_PIN)
#define GRAY_AD0_LOW()   (HW_GRAY_AD0_PORT->BSRR = (uint32_t)HW_GRAY_AD0_PIN << 16U)
#define GRAY_AD1_HIGH()  (HW_GRAY_AD1_PORT->BSRR = HW_GRAY_AD1_PIN)
#define GRAY_AD1_LOW()   (HW_GRAY_AD1_PORT->BSRR = (uint32_t)HW_GRAY_AD1_PIN << 16U)
#define GRAY_AD2_HIGH()  (HW_GRAY_AD2_PORT->BSRR = HW_GRAY_AD2_PIN)
#define GRAY_AD2_LOW()   (HW_GRAY_AD2_PORT->BSRR = (uint32_t)HW_GRAY_AD2_PIN << 16U)
#define GRAY_OUT_READ()  ((HW_GRAY_OUT_PORT->IDR & HW_GRAY_OUT_PIN) ? 1 : 0)

static const uint8_t sensor_mapping[8] = {0, 1, 2, 3, 4, 5, 6, 7};

uint8_t gray_get_data(void) {
  uint8_t final_data = 0;
  for (int phys_pos = 0; phys_pos < 8; phys_pos++) {
    uint8_t i = sensor_mapping[phys_pos];

    (i & 0x01) ? GRAY_AD0_HIGH() : GRAY_AD0_LOW();
    (i & 0x02) ? GRAY_AD1_HIGH() : GRAY_AD1_LOW();
    (i & 0x04) ? GRAY_AD2_HIGH() : GRAY_AD2_LOW();

    for (volatile int delay = 0; delay < HW_GRAY_MUX_DELAY; delay++)
      ;

    if (GRAY_OUT_READ())
      final_data |= (1 << phys_pos);
  }
  return final_data;
}

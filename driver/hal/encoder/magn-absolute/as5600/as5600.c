#include "as5600.h"
#include "i2c.h"

#define AS5600_I2C_HANDLE hi2c1
#define AS5600_ADDR 0x36
#define AS5600_ANGLE_REG 0x0C
#define AS5600_RESOLUTION 4096.0f
#define AS5600_MASK 0x0FFF
#define AS5600_TIMEOUT_MS 1
#define AS5600_FAIL_COOLDOWN 50

static volatile float as5600_latest_angle = 0.0f;
static uint16_t as5600_rx = 0;
static uint8_t as5600_err_count = 0;
static uint8_t as5600_cooldown = 0;

float as5600_get_angle_cb(void) {
  if (as5600_cooldown > 0) {
    as5600_cooldown--;
    return as5600_latest_angle;
  }

  uint8_t reg = AS5600_ANGLE_REG;
  uint8_t data[2];

  if (HAL_I2C_Mem_Read(&AS5600_I2C_HANDLE, AS5600_ADDR << 1, reg,
                       I2C_MEMADD_SIZE_8BIT, data, 2,
                       AS5600_TIMEOUT_MS) == HAL_OK) {
    as5600_rx = ((uint16_t)data[0] << 8) | data[1];
    as5600_latest_angle =
        ((float)(as5600_rx & AS5600_MASK) / AS5600_RESOLUTION) * 6.283185307f;
    as5600_err_count = 0;
  } else {
    as5600_err_count++;
    if (as5600_err_count >= 3)
      as5600_cooldown = AS5600_FAIL_COOLDOWN;
  }
  return as5600_latest_angle;
}

uint8_t as5600_is_healthy(void) { return as5600_err_count < 3; }

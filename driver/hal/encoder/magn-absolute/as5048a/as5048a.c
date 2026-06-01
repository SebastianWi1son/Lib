#include "as5048a.h"
#include "spi.h"

#define AS5048_SPI_HANDLE hspi1
#define AS5048_CS_PORT GPIOA
#define AS5048_CS_PIN GPIO_PIN_4

#define AS5048A_RESOLUTION 16384.0f
#define AS5048A_MASK 0x3FFF

static volatile float as5048a_latest_angle = 0.0f;
static uint16_t as5048a_rx = 0;

float as5048a_get_angle_cb(void) {
  uint16_t tx = 0xFFFF;
  HAL_GPIO_WritePin(AS5048_CS_PORT, AS5048_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&AS5048_SPI_HANDLE, (uint8_t *)&tx,
                          (uint8_t *)&as5048a_rx, 1, 1);
  HAL_GPIO_WritePin(AS5048_CS_PORT, AS5048_CS_PIN, GPIO_PIN_SET);
  as5048a_latest_angle =
      ((float)(as5048a_rx & AS5048A_MASK) / AS5048A_RESOLUTION) * 6.283185307f;
  return as5048a_latest_angle;
}

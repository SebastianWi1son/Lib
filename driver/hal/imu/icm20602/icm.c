#include "icm.h"
#include "chassis_config.h"
#include "gpio.h"
#include "spi.h"

#define ICM_CS_ON()                                                            \
  HAL_GPIO_WritePin(HW_ICM_CS_PORT, HW_ICM_CS_PIN, GPIO_PIN_RESET)
#define ICM_CS_OFF()                                                           \
  HAL_GPIO_WritePin(HW_ICM_CS_PORT, HW_ICM_CS_PIN, GPIO_PIN_SET)

#define ICM_REG_CONFIG 0x1A
#define ICM_REG_GYRO_CONFIG 0x1B
#define ICM_REG_ACCEL_XOUT_H 0x3B
#define ICM_REG_PWR_MGMT_1 0x6B
#define ICM_REG_WHO_AM_I 0x75

// =====================================================================
// 私有状态 (Private State)
// =====================================================================
static float s_gyro_z_bias = 0.0f; // 开机零偏
static float s_yaw_dps = 0.0f;     // 扣除零偏后的实时角速度
static float s_yaw_angle = 0.0f;   // 积分后的绝对航向角

// =====================================================================
// 硬件底层通信 (Private Methods)
// =====================================================================
static void icm_write_reg(uint8_t reg, uint8_t value) {
  uint8_t tx_data[2];
  tx_data[0] = reg & 0x7F;
  tx_data[1] = value;

  ICM_CS_ON();
  HAL_SPI_Transmit(HW_ICM_SPI, tx_data, 2, 10);
  ICM_CS_OFF();
}

static void icm_read_regs(uint8_t reg, uint8_t *buf, uint16_t len) {
  uint8_t tx_reg = reg | 0x80;

  ICM_CS_ON();
  HAL_SPI_Transmit(HW_ICM_SPI, &tx_reg, 1, 10);
  HAL_SPI_Receive(HW_ICM_SPI, buf, len, 10);
  ICM_CS_OFF();
}

static void icm_get_raw_data(icm_data_t *data) {
  uint8_t buffer[14];
  icm_read_regs(ICM_REG_ACCEL_XOUT_H, buffer, 14);

  data->accel_x = (buffer[0] << 8 | buffer[1]);
  data->accel_y = (buffer[2] << 8 | buffer[3]);
  data->accel_z = (buffer[4] << 8 | buffer[5]);
  data->gyro_x = (buffer[8] << 8 | buffer[9]);
  data->gyro_y = (buffer[10] << 8 | buffer[11]);
  data->gyro_z = (buffer[12] << 8 | buffer[13]);
}

// 内部函数：仅读取原始 Z 轴角速度 (带方向修正)
static float read_raw_gyro_z_dps(void) {
  icm_data_t icm_data;
  icm_get_raw_data(&icm_data);
  return ((float)icm_data.gyro_z / 16.4f) * GYRO_Z_DIR;
}

// =====================================================================
// 对外 API (Public Methods)
// =====================================================================

uint8_t icm_init(void) {
  uint8_t who_am_i = 0;
  HAL_Delay(50);
  icm_read_regs(ICM_REG_WHO_AM_I, &who_am_i, 1);

  if (who_am_i != 0x12)
    return 1;

  icm_write_reg(ICM_REG_PWR_MGMT_1, 0x01);
  HAL_Delay(10);
  icm_write_reg(ICM_REG_CONFIG, 0x00);
  icm_write_reg(ICM_REG_GYRO_CONFIG, 0x18);
  return 0;
}

void icm_calibrate_z_bias(void) {
  float sum = 0;
  int sample_count = 500;
  
  for(int i = 0; i < 50; i++) {
      read_raw_gyro_z_dps();
      HAL_Delay(2);
  }
  
  for(int i = 0; i < sample_count; i++) {
      sum += read_raw_gyro_z_dps();
      HAL_Delay(2); 
  }
  s_gyro_z_bias = sum / sample_count;
}

// 核心更新函数：读取总线、去零偏、滤波、积分。
// 必须且仅能在唯一的定时器中断中调用一次。
void icm_update_z(float dt) {
    float raw_dps = read_raw_gyro_z_dps();
    
    // 1. 计算去零偏后的真实角速度
    s_yaw_dps = raw_dps - s_gyro_z_bias;
    
    // 2. 底噪死区屏蔽
    if (s_yaw_dps > -0.5f && s_yaw_dps < 0.5f) {
        s_yaw_dps = 0.0f; 
    }
    
    // 3. 积分累计角度
    s_yaw_angle += s_yaw_dps * dt;
}

// 供 PID 内环 (速度环) 或 UI 读取
float icm_get_dps(void) {
    return s_yaw_dps;
}

// 供 PID 外环 (位置环/角度环) 或 UI 读取
float icm_get_angle(void) {
    return s_yaw_angle;
}

// 供地标识别中断调用，强行修正当前航向角
void icm_set_angle(float true_angle) {
    s_yaw_angle = true_angle;
}
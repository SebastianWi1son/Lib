#ifndef __ICM_H__
#define __ICM_H__

#include "main.h"

// 数据结构定义 (可按需决定是否保留，目前已被底层封装，外部不可见)
typedef struct {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} icm_data_t;

// 硬件初始化
uint8_t icm_init(void);

// 姿态解算 API
void icm_calibrate_z_bias(void);       // 开机静止时调用，采集零偏
void icm_update_z(float dt);           // 定时器中调用 (如传 0.005f)，更新数据
float icm_get_dps(void);               // 获取当前角速度 (已扣除零偏)
float icm_get_angle(void);             // 获取当前绝对航向角 (积分结果)
void icm_set_angle(float true_angle);  // 强行设定角度 (用于地标纠偏)

#endif /* __ICM_H__ */
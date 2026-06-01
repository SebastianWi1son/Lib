#ifndef __APP_TELE_H
#define __APP_TELE_H

/**
 * @file    app_tele.h
 * @brief   遥测薄胶水 — 命令分派 + 遥测数据填充
 *
 * 这是 bsp/middleware/telemetry/ + app/tele/ 重构后的统一入口.
 *
 * 职责: 只做胶水 — 不解析协议, 不直接写 FOC 字段.
 *   - 命令: tele_parser → motor_param / motor_tune
 *   - 数据: motor_param_get_*() → justfloat_send()
 *
 * 编译依赖 (link-time extern):
 *   - foc_motor_t gimbal_yaw, gimbal_pitch  (app_gimbal.c)
 *   - volatile float target_angle_yaw/pitch  (app_gimbal.c)
 *   - volatile float yaw_center_angle, pitch_center_angle
 *   - volatile uint8_t system_runflag
 */

#include <stdint.h>

void app_tele_init(void);
void app_tele_process(void);

#endif /* __APP_TELE_H */

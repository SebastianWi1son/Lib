#ifndef __MOTOR_TUNE_H
#define __MOTOR_TUNE_H

/**
 * @file    motor_tune.h
 * @brief   电机调试域控制器 — 定义调试协议 + 数据采集 + 快照管理
 *
 * 本模块定义调试命令协议, 所有传输层 (tele, UART, CAN) 必须适配此协议.
 * 依赖方向: 传输层 → motor_tune ← (协议定义者)
 *
 * 内部包含:
 *   - 数据采集引擎 (环形缓冲, ISR capture)
 *   - 命令状态机 (STEP, HOLD)
 *   - 快照 & 预设管理 (通过 motor_param)
 */

#include <stdint.h>
#include <stdbool.h>
#include "foc.h"
#include "motor_param.h"

/* ================================================================
 * 调试命令协议 — 传输层必须映射到此枚举
 * ================================================================ */

typedef enum {
  TUNE_CMD_NONE = 0,
  TUNE_CMD_STEP,              /* 阶跃响应: param={amplitude_rad, duration_ms} */
  TUNE_CMD_HOLD,              /* 被动采集: param={duration_ms} */
  TUNE_CMD_SNAPSHOT_SAVE,     /* 保存当前参数为默认 */
  TUNE_CMD_SNAPSHOT_RESTORE,  /* 恢复默认参数 */
} tune_cmd_id_t;

typedef struct { float a; float b; } tune_param_t;

/* ================================================================
 * 采样数据 — ISR 中填充
 * ================================================================ */

#define TUNE_BUF_SAMPLES 2048

typedef struct {
  float abs_angle;
  float target_angle;
  float velocity;
  float uq;
  float angle_pid_out;
  float vel_pid_out;
  float iq, id;
  uint32_t tick;
} tune_sample_t;

/* ================================================================
 * 环形缓冲 — ISR / main loop 共享
 * ================================================================ */

typedef struct {
  tune_sample_t buf[TUNE_BUF_SAMPLES];
  volatile uint32_t head;
  volatile uint32_t count;
  volatile uint8_t enabled;
  volatile uint8_t triggered;
  uint32_t decimation;
  uint32_t decim_counter;
} tune_buffer_t;

/* ================================================================
 * 命令状态
 * ================================================================ */

typedef enum {
  TUNE_STATE_IDLE = 0,
  TUNE_STATE_ARMING,
  TUNE_STATE_RUNNING,
  TUNE_STATE_DONE,
} tune_state_t;

/* ================================================================
 * 每电机一个 motor_tune 实例
 * ================================================================ */

typedef struct {
  foc_motor_t  *motor;          /* 绑定的电机 */
  volatile float *cmd_target;   /* ISR 读取的目标角指针 */
  tune_buffer_t buffer;
  tune_cmd_id_t  active_cmd;
  tune_state_t   state;
  tune_param_t   cmd_param;
  uint32_t       cmd_duration_ms;
  uint32_t       arm_ticks;
  uint32_t       total_ticks;
  uint32_t       elapsed_ticks;
  float          saved_target;
  motor_param_snapshot_t pre_snapshot;
} motor_tune_t;

/* ================================================================
 * API
 * ================================================================ */

/* 初始化 — 绑定电机, 设置采样频率 (1=1kHz, 10=100Hz) */
void motor_tune_init(motor_tune_t *t, foc_motor_t *motor,
                      volatile float *cmd_target, uint32_t decimation);

/* ── 命令接口 — 传输层调用 ── */

/* 启动调试命令, 返回 false=前一个命令未完成 */
bool motor_tune_cmd_start(motor_tune_t *t, tune_cmd_id_t cmd,
                           tune_param_t param, uint32_t duration_ms);

/* 中断当前命令 */
void motor_tune_cmd_stop(motor_tune_t *t);

/* ── main loop 轮询 ── */
void motor_tune_poll(motor_tune_t *t, float dt);

/* ── ISR 数据采集 (在 foc_angle_control_tick 末尾调用) ── */
void motor_tune_capture_isr(motor_tune_t *t);

/* ── 数据回传 (app_tele 调用) ── */
uint32_t motor_tune_buffer_read(motor_tune_t *t, tune_sample_t *out,
                                 uint32_t max_count);
uint8_t motor_tune_is_done(motor_tune_t *t);

#endif /* __MOTOR_TUNE_H */

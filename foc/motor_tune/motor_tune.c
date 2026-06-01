#include "motor_tune.h"
#include <string.h>
#include <math.h>

/* ================================================================
 * 初始化
 * ================================================================ */

void motor_tune_init(motor_tune_t *t, foc_motor_t *motor,
                      volatile float *cmd_target, uint32_t decimation) {
  memset(t, 0, sizeof(*t));
  t->motor       = motor;
  t->cmd_target  = cmd_target;
  t->buffer.decimation = (decimation > 0) ? decimation : 1;
  t->state = TUNE_STATE_IDLE;
}

/* ================================================================
 * ISR 数据采集 — 轻量, 零阻塞
 * ================================================================ */

void motor_tune_capture_isr(motor_tune_t *t) {
  tune_buffer_t *b = &t->buffer;
  if (!b->enabled || !b->triggered) return;
  if (++b->decim_counter < b->decimation) return;
  b->decim_counter = 0;

  foc_motor_t *m = t->motor;
  tune_sample_t *s = &b->buf[b->head];

  s->abs_angle     = m->abs_angle;
  s->target_angle  = m->planned_target_prev;
  s->velocity      = m->velocity;
  /* 电压模式: vel_pid_out = Uq; 电流模式: uq = iq_pid.output_prev */
  s->uq            = m->vel_pid.output_prev;
  s->angle_pid_out = m->angle_pid.output_prev;
  s->vel_pid_out   = m->vel_pid.output_prev;
  s->iq = m->iq;
  s->id = m->id;
  s->tick = b->count;

  b->head = (b->head + 1) % TUNE_BUF_SAMPLES;
  b->count++;
}

/* ================================================================
 * 命令接口
 * ================================================================ */

bool motor_tune_cmd_start(motor_tune_t *t, tune_cmd_id_t cmd,
                           tune_param_t param, uint32_t duration_ms) {
  if (t->state != TUNE_STATE_IDLE && t->state != TUNE_STATE_DONE)
    return false;

  /* 快照 & 非采集类命令直接执行 */
  switch (cmd) {
  case TUNE_CMD_SNAPSHOT_SAVE:
    motor_param_snapshot_save_default(t->motor);
    return true;
  case TUNE_CMD_SNAPSHOT_RESTORE:
    motor_param_snapshot_restore_default(t->motor);
    return true;
  default:
    break;
  }

  /* 采集类命令: 保存快照, 初始化缓冲 */
  motor_param_snapshot_save(t->motor, &t->pre_snapshot);

  t->active_cmd      = cmd;
  t->cmd_param       = param;
  t->cmd_duration_ms = (duration_ms > 0) ? duration_ms : 1000;
  t->elapsed_ticks   = 0;
  t->arm_ticks       = 100;   /* 100ms 预触发 */
  t->total_ticks     = t->arm_ticks + t->cmd_duration_ms;

  t->buffer.head     = 0;
  t->buffer.count    = 0;
  t->buffer.decim_counter = 0;
  t->buffer.enabled  = 1;
  t->buffer.triggered = 0;

  t->state = TUNE_STATE_ARMING;
  return true;
}

void motor_tune_cmd_stop(motor_tune_t *t) {
  t->buffer.triggered = 0;
  t->buffer.enabled   = 0;
  t->state = TUNE_STATE_IDLE;
}

/* ================================================================
 * main loop 轮询 — 状态机推进
 * ================================================================ */

void motor_tune_poll(motor_tune_t *t, float dt) {
  if (t->state == TUNE_STATE_IDLE || t->state == TUNE_STATE_DONE)
    return;

  /* tick 估推 (主循环 ~100Hz, ISR @ 1kHz) */
  uint32_t inc = (uint32_t)(dt * 1000.0f + 0.5f);
  if (inc < 1) inc = 1;
  t->elapsed_ticks += inc;

  switch (t->state) {

  case TUNE_STATE_ARMING:
    if (t->elapsed_ticks >= t->arm_ticks) {
      /* 触发: 注入激励信号 */
      t->buffer.triggered = 1;

      if (t->active_cmd == TUNE_CMD_STEP && t->cmd_target) {
        t->saved_target = *t->cmd_target;
        *t->cmd_target = t->motor->abs_angle + t->cmd_param.a;
      }
      /* HOLD: 不注入, 只被动记录 */

      t->state = TUNE_STATE_RUNNING;
    }
    break;

  case TUNE_STATE_RUNNING:
    if (t->elapsed_ticks >= t->total_ticks) {
      /* 完成 */
      t->buffer.triggered = 0;

      if (t->active_cmd == TUNE_CMD_STEP && t->cmd_target)
        *t->cmd_target = t->saved_target;

      t->state = TUNE_STATE_DONE;
    }
    break;

  default: break;
  }
}

/* ================================================================
 * 数据回传
 * ================================================================ */

uint8_t motor_tune_is_done(motor_tune_t *t) {
  return (t->state == TUNE_STATE_DONE) ? 1 : 0;
}

uint32_t motor_tune_buffer_read(motor_tune_t *t, tune_sample_t *out,
                                 uint32_t max_count) {
  tune_buffer_t *b = &t->buffer;
  uint32_t count_snap = b->count;  /* 快照一次, 防止 ISR 并发更新 */
  uint32_t avail = count_snap;
  if (avail > TUNE_BUF_SAMPLES) avail = TUNE_BUF_SAMPLES;
  if (avail > max_count) avail = max_count;
  if (avail == 0) return 0;

  uint32_t start = (count_snap >= TUNE_BUF_SAMPLES) ? b->head : 0;
  for (uint32_t i = 0; i < avail; i++)
    out[i] = b->buf[(start + i) % TUNE_BUF_SAMPLES];

  return avail;
}

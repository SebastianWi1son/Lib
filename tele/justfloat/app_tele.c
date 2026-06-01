/**
 * @file    app_tele.c
 * @brief   遥测薄桥接 — 命令分派 + 遥测填充
 *
 * 只做分派: 普通参数→motor_param, 调试命令→motor_tune.
 * 不解析协议 (tele_parser), 不直接写 FOC (motor_param), 不碰 HAL (justfloat_port).
 */

#include "app_tele.h"
#include "justfloat.h"
#include "tele_parser.h"
#include "motor_param.h"
#include "motor_tune.h"
#include "foc.h"
#include "gimbal_controller.h"
#include <string.h>

/* ================================================================
 * Config
 * ================================================================ */

#define PLOT_CHANNELS   20
#define TELE_INTERVAL_MS 10

/* DMA buffer — D2 SRAM, 避开 DCache */
static float  tele_tx_buf[PLOT_CHANNELS + 1]
    __attribute__((section(".dma_buffer")));
static char   tele_rx_buf[256]
    __attribute__((section(".dma_buffer")));

/* ── extern (link-time) ── */
extern foc_motor_t gimbal_yaw;
extern foc_motor_t gimbal_pitch;
extern volatile float target_angle_yaw;
extern volatile float target_angle_pitch;
extern volatile float yaw_center_angle;
extern volatile float pitch_center_angle;
extern volatile uint8_t system_runflag;

/* ── 状态 ── */
static foc_motor_t *active_motor = NULL;
static float active_motor_id = 1.0f;
static float plot_snapshot[PLOT_CHANNELS];

/* 每电机一个 motor_tune 实例 */
static motor_tune_t tune_yaw;
static motor_tune_t tune_pitch;

/* ================================================================
 * 初始化
 * ================================app================================ */

void app_tele_init(void) {
  extern void *huart4;
  justfloat_port_t port = justfloat_port_create(&huart4);
  justfloat_init(&port);
  justfloat_set_tx_buf(tele_tx_buf);
  justfloat_set_rx_buf(tele_rx_buf, sizeof(tele_rx_buf));

  active_motor = &gimbal_yaw;
  active_motor_id = 1.0f;

  motor_tune_init(&tune_yaw,   &gimbal_yaw,   &target_angle_yaw,   10);
  motor_tune_init(&tune_pitch, &gimbal_pitch, &target_angle_pitch, 10);
}

/* ================================================================
 * 命令分派
 * ================================================================ */

static void tele_dispatch(tele_cmd_t *cmd) {
  if (!active_motor) return;

  /* 普通参数 → motor_param */
  if (cmd->cmd_id >= 0 && cmd->cmd_id < PARAM_COUNT) {
    motor_param_set_by_id(active_motor, (foc_param_id_t)cmd->cmd_id, cmd->value);
    return;
  }

  /* 特殊命令 */
  motor_tune_t *t = (active_motor == &gimbal_yaw) ? &tune_yaw : &tune_pitch;

  switch (cmd->cmd_id) {
  case TELE_CMD_MOTOR_SWITCH:
    active_motor_id = cmd->value;
    active_motor = ((int)cmd->value == 0) ? &gimbal_pitch : &gimbal_yaw;
    break;

  case TELE_CMD_MOTOR_RUN:
    system_runflag = (uint8_t)cmd->value;
    break;

  case TELE_CMD_TARGET_YAW:
    gimbal_ctrl_set_target(cmd->value, target_angle_pitch - pitch_center_angle);
    break;

  case TELE_CMD_TARGET_PITCH:
    gimbal_ctrl_set_target(target_angle_yaw - yaw_center_angle, cmd->value);
    break;

  /* ── 调试命令 → motor_tune (协议由 motor_tune 定义) ── */
  case TELE_CMD_DEBUG_STEP:
    motor_tune_cmd_start(t, TUNE_CMD_STEP,
                          (tune_param_t){cmd->value, 2000}, 2000);
    break;

  case TELE_CMD_DEBUG_HOLD:
    motor_tune_cmd_start(t, TUNE_CMD_HOLD,
                          (tune_param_t){cmd->value, 0}, (uint32_t)cmd->value);
    break;

  case TELE_CMD_SNAPSHOT_SAVE:
    motor_tune_cmd_start(t, TUNE_CMD_SNAPSHOT_SAVE, (tune_param_t){0, 0}, 0);
    break;

  case TELE_CMD_SNAPSHOT_REST:
    motor_tune_cmd_start(t, TUNE_CMD_SNAPSHOT_RESTORE, (tune_param_t){0, 0}, 0);
    break;
  }
}

/* ================================================================
 * 遥测填充
 * ================================================================ */

static void tele_fill_snapshot(void) {
  if (!active_motor) return;

  plot_snapshot[0]  = motor_param_get_voltage_limit(active_motor);
  plot_snapshot[1]  = active_motor->planner.filter2;
  plot_snapshot[2]  = active_motor->velocity;
  plot_snapshot[3]  = active_motor->angle_pid.output_prev;
  plot_snapshot[4]  = active_motor->vel_pid.output_prev;

  plot_snapshot[5]  = target_angle_pitch - pitch_center_angle;
  plot_snapshot[6]  = gimbal_pitch.abs_angle - pitch_center_angle;
  plot_snapshot[7]  = target_angle_yaw - yaw_center_angle;
  plot_snapshot[8]  = gimbal_yaw.abs_angle - yaw_center_angle;

  plot_snapshot[9]  = active_motor_id;
  plot_snapshot[10] = (float)system_runflag;

  plot_snapshot[11] = motor_param_get_angle_kp(active_motor);
  plot_snapshot[12] = motor_param_get_angle_ki(active_motor);
  plot_snapshot[13] = motor_param_get_angle_kd(active_motor);
  plot_snapshot[14] = motor_param_get_vel_kp(active_motor);
  plot_snapshot[15] = motor_param_get_vel_ki(active_motor);
  plot_snapshot[16] = motor_param_get_vel_kd(active_motor);
  plot_snapshot[17] = motor_param_get_deadzone(active_motor);
  plot_snapshot[18] = motor_param_get_traj_vmax(active_motor);

  plot_snapshot[19] = active_motor->iq;
}

/* ================================================================
 * 主循环
 * ================================================================ */

void app_tele_process(void) {
  /* A. 命令处理 */
  if (justfloat_cmd_ready()) {
    const char *buf = justfloat_cmd_buf();
    tele_cmd_t cmd;
    const char *scan = buf;
    while (*scan) {
      if (tele_parse_command(scan, &cmd))
        tele_dispatch(&cmd);
      const char *next = strpbrk(scan, " \n\r");
      scan = next ? next + 1 : scan + strlen(scan);
    }
    justfloat_flush();
  }

  /* B. 调试状态机推进 */
  motor_tune_poll(&tune_yaw,   0.01f);
  motor_tune_poll(&tune_pitch, 0.01f);

  /* C. 遥测发送 */
  tele_fill_snapshot();
  justfloat_send(TELE_INTERVAL_MS, plot_snapshot, PLOT_CHANNELS);
}

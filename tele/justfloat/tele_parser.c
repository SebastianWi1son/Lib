#include "tele_parser.h"
#include <string.h>
#include <stdlib.h>

/* ================================================================
 * 特殊命令名表 — 与 TELE_CMD_* 对应
 * ================================================================ */

typedef struct {
  const char *name;
  int   cmd_id;
} tele_special_cmd_t;

static const tele_special_cmd_t special_cmds[] = {
  {"debug_step:",    TELE_CMD_DEBUG_STEP},
  {"debug_hold:",    TELE_CMD_DEBUG_HOLD},
  {"snapshot_save",  TELE_CMD_SNAPSHOT_SAVE},
  {"snapshot_rest",  TELE_CMD_SNAPSHOT_REST},
  {"MotorID:",       TELE_CMD_MOTOR_SWITCH},
  {"MotorRun:",      TELE_CMD_MOTOR_RUN},
  {"YawTarget:",     TELE_CMD_TARGET_YAW},
  {"PitchTarget:",   TELE_CMD_TARGET_PITCH},
  {NULL, 0}
};

/* ================================================================
 * 解析
 * ================================================================ */

int tele_parse_command(const char *text, tele_cmd_t *cmd) {
  if (!text || !cmd) return 0;

  /* ── 1. 先匹配 motor_param 参数命令 (复用 bounds_table) ── */
  for (int i = 0; i < PARAM_COUNT; i++) {
    const motor_param_bounds_t *b = motor_param_get_bounds((foc_param_id_t)i);
    if (!b || !b->name) continue;

    const char *pos = strstr(text, b->name);
    if (pos) {
      cmd->cmd_id = i;
      cmd->value  = (float)atof(pos + strlen(b->name));
      return 1;
    }
  }

  /* ── 2. 匹配特殊命令 ── */
  for (int i = 0; special_cmds[i].name != NULL; i++) {
    const char *pos = strstr(text, special_cmds[i].name);
    if (pos) {
      cmd->cmd_id = special_cmds[i].cmd_id;
      /* 有参数的命令 (如 debug_step:0.5, MotorID:1) */
      if (special_cmds[i].name[strlen(special_cmds[i].name)-1] == ':') {
        cmd->value = (float)atof(pos + strlen(special_cmds[i].name));
      } else {
        cmd->value = 1.0f;  /* 无参数命令 (如 snapshot_save) */
      }
      return 1;
    }
  }

  return 0;
}

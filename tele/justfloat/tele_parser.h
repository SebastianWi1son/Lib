#ifndef __TELE_PARSER_H
#define __TELE_PARSER_H

/**
 * @file    tele_parser.h
 * @brief   文本命令 → 结构化命令解析
 *
 * 复用 motor_param 的 bounds_table[].name 做命令名匹配,
 * 不再需要在 app_tele.c 中硬编码 "ang_Kp:" 字符串.
 *
 * 支持:
 *   - 普通参数: "ang_Kp:12.5" → tele_cmd_t { PARAM_ANGLE_KP, 12.5 }
 *   - 调试命令: "debug_step:0.5" → tele_cmd_t { PARAM_COUNT+1, 0.5 }
 *   - 特殊命令: "MotorRun:1"    → tele_cmd_t { special_id, 1.0 }
 */

#include <stdint.h>
#include "motor_param.h"

/* ================================================================
 * 结构化命令
 * ================================================================ */

/* PARAM_COUNT 以上的 ID 用于特殊命令 */
#define TELE_CMD_DEBUG_STEP     (PARAM_COUNT + 1)
#define TELE_CMD_DEBUG_HOLD     (PARAM_COUNT + 2)
#define TELE_CMD_MOTOR_SWITCH   (PARAM_COUNT + 3)   /* MotorID:0/1 */
#define TELE_CMD_MOTOR_RUN      (PARAM_COUNT + 4)   /* MotorRun:0/1 */
#define TELE_CMD_TARGET_YAW     (PARAM_COUNT + 5)   /* YawTarget: */
#define TELE_CMD_TARGET_PITCH   (PARAM_COUNT + 6)   /* PitchTarget: */
#define TELE_CMD_SNAPSHOT_SAVE  (PARAM_COUNT + 7)   /* snapshot_save */
#define TELE_CMD_SNAPSHOT_REST  (PARAM_COUNT + 8)   /* snapshot_restore */

typedef struct {
  int   cmd_id;    /* 参数: foc_param_id_t; 特殊: TELE_CMD_* */
  float value;
} tele_cmd_t;

/* ================================================================
 * 解析器
 * ================================================================ */

/**
 * @brief 解析一行 JustFloat 文本 → 结构化命令
 * @param text   以 '\0' 结尾的文本 (来自 justfloat_cmd_buf)
 * @param cmd    输出: 解析出的命令
 * @return       1 = 解析成功, 0 = 无匹配 (需调用多次以解析多条命令)
 *
 * 用法:
 *   const char *buf = justfloat_cmd_buf();
 *   tele_cmd_t cmd;
 *   while (tele_parse_command(buf, &cmd)) {
 *       dispatch(cmd);
 *       buf += ...;  // 跳过已解析部分
 *   }
 */
int tele_parse_command(const char *text, tele_cmd_t *cmd);

#endif /* __TELE_PARSER_H */

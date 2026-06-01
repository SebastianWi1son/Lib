/* app_menu.c — User Configuration
 *
 * 后端 shell_core.c/h 封装了全部菜单逻辑、状态机和渲染, 无需阅读。
 * 本文件是你唯一需要编辑的配置文件, 分 4 段填空即可。
 *
 * Quick start / 快速上手:
 *   1. 填 g_cfg 里的 3 个必填函数指针 (disp_clear, disp_text, key_read)
 *   2. 填可选指针 (buzzer / mission_done / on_ext_cmd / render_custom)
 *   3. 声明你的动作函数 (段落 2)
 *   4. 声明你的菜单树 (段落 3)
 *   5. 主循环调 app_menu_init() → app_menu_tick()
 */
#include "shell_core.h"

// ================================================================
// 段落 1: 硬件回调
//         Hardware Callbacks
//
// 直接填函数指针。签名匹配就直接填平台函数名; 不匹配才写 wrapper。
// 不用的填 NULL, 后端会安全跳过。
// ================================================================

/*
 * 回调签名速查:
 *
 *   void     disp_clear(void)
 *   void     disp_text(uint8_t col, uint8_t row, const char *str)
 *   uint8_t  key_read(void)                  // 必须非阻塞。返回 0=无,1=K1,2=K2
 *   void     buzzer_beep(uint8_t n, uint16_t ms)
 *   bool     buzzer_busy(void)
 *   bool     mission_done(void)              // true=任务完成, shell 自动跳到 FINISHED
 *   void     on_ext_cmd(uint8_t data)        // shell_inject(data) 时触发
 *   void     render_custom(uint8_t id)       // 自定义状态渲染
 *
 * FAQ:
 *   Q: 我的 OLED_ShowStr(col, row, str) 参数是 uint8_t*, 不是 const char*
 *   A: 在这里写一个一行 wrapper, 然后填 wrapper 的名字:
 *        static void my_text(uint8_t c, uint8_t r, const char *s) {
 *            OLED_ShowStr(c, r, (uint8_t*)s);
 *        }
 *      .disp_text = my_text,
 *
 *   Q: 我的 key_get_num() 返回的不是 0/1/2
 *   A: 同理写 wrapper 转换。
 */

/* [YOUR WRAPPERS — only if platform signatures don't match] */

static const shell_cfg_t g_cfg = {
    .disp_clear    = /* [TODO] */ NULL,    // 例: OLED_Clear
    .disp_text     = /* [TODO] */ NULL,    // 例: OLED_ShowStr 或 my_text (wrapper)
    .key_read      = /* [TODO] */ NULL,    // 例: key_get_num 或 my_key (wrapper)
    .buzzer_beep   = NULL,                 // 例: App_Buzzer_Beep
    .buzzer_busy   = NULL,                 // 例: App_Buzzer_IsBusy
    .mission_done  = NULL,                 // 例: nav_done_check
    .on_ext_cmd    = NULL,                 // 例: on_pi_route
    .render_custom = NULL,                 // 例: render_comm_test
};

// ================================================================
// 段落 2: 任务动作 (菜单终点选中后执行的函数)
//         Task Actions
//
// 每个函数内可用的 shell API:
//   shell_set_running()            — 进入 RUNNING 状态 (配合 mission_done 自动结束)
//   shell_wait_ext()               — 进入 WAIT_EXT 状态 (配合 shell_inject)
//   shell_nav_to(&某菜单)          — 跳转到指定菜单
//   shell_defer_beep(N, ms, fn)    — 先蜂鸣 N 次, 然后自动调 fn
//   shell_goto_custom(id)          — 进入自定义渲染状态 (配合 render_custom)
//
// ⚠ defer_beep 是异步的 — shell_set_running 应放在 defer 的回调里, 不在 action 里。
// ================================================================

/* [YOUR ACTIONS] */

// 例 — 无蜂鸣动作 (确认页 → START → 直接执行):
// static void enter_vision(void) {
//     send_start_cmd_to_pi();
//     shell_wait_ext();
// }

// 例 — 带蜂鸣动作 (确认页 → START → 蜂鸣 3 声 → 发车):
// static void do_launch(void) {
//     strategy_select(&strat, TASK_SPEED);
//     strategy_start(&strat, get_yaw());
//     shell_set_running();                 // ← 在 defer 回调里切换状态
// }
// static void act_speed(void) {
//     shell_defer_beep(3, 500, do_launch); // ← action 里只触发蜂鸣
// }

// 例 — 自定义状态 (如通信测试):
// static void enter_comm_test(void) { shell_goto_custom(0); }
// 然后在 render_custom 回调里根据 id==0 绘制调试信息。

// ================================================================
// 段落 3: 菜单树
//         Menu Tree
//
// 每个条目 5 个字段 (只写用到的即可, 未提及字段自动归 0/NULL):
//   .text     显示文字
//   .sub_menu 子菜单指针 (非 NULL = 选中后跳到该菜单)
//   .action   动作函数   (非 NULL = 终点条目)
//   .beeps    蜂鸣次数   (0=不蜂鸣, 默认 0)
//   .beep_ms  每次时长ms (默认 0)
//
// 规则 (后端自动判断):
//   .sub_menu != NULL        →  选中即跳转 (子菜单入口 / 返回上级)
//   .action != NULL, .beeps=0 →  弹出确认页 → START 才执行
//   .action != NULL, .beeps>0 →  蜂鸣后立即执行 (跳过确认页, 快速启动)
//
// 菜单可无限嵌套 — 子菜单的条目可再指向孙菜单。
// ================================================================

/* ⚠ 必须! 前向声明所有会被循环引用的菜单 (C 编译要求) */
extern const shell_menu_t main_menu;

/* [YOUR MENUS] */

// 例 — 二级菜单:
// static const shell_menu_item_t sub_qt_items[] = {
//     { .text = "T1 Ortho", .action = act_ortho },
//     { .text = "T2 Arc A", .action = act_arc_a, .beeps = 3, .beep_ms = 500 },
//     { .text = "返回",     .sub_menu = &main_menu },
// };
// static const shell_menu_t sub_qt = {
//     .title = "Quick Test", .items = sub_qt_items, .count = 3
// };

// 例 — 一级菜单:
// static const shell_menu_item_t main_items[] = {
//     { .text = "1. Speed Run",  .action = act_speed },
//     { .text = "2. Vision Trk", .action = enter_vision },
//     { .text = "3. Quick Test", .sub_menu = &sub_qt },
//     { .text = "4. Comm Test",  .action = enter_comm_test },
// };
// static const shell_menu_t main_menu = {
//     .title = "Mode Select", .items = main_items, .count = 4
// };

// ================================================================
// 段落 4: 入口函数 [固定 — 不要修改]
//         Entry Points
// ================================================================

void app_menu_init(void) {
    shell_init(&g_cfg, &main_menu);
}

void app_menu_tick(void) {
    shell_tick();
}

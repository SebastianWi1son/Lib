#ifndef SHELL_CORE_H
#define SHELL_CORE_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Shell Core — Portable Application Menu State Machine
 * 可移植应用菜单状态机（后端）
 *
 * Zero hardware dependencies. Singleton pattern.
 * Put this file + shell_core.c into any project, then fill in app_menu.c.
 * 零硬件依赖，单例模式。将本文件 + shell_core.c 放入项目，然后填写 app_menu.c。
 */

// ============================================================
// Platform Constants — adjust per display hardware
// 平台常量 — 按显示硬件调整
// ============================================================

/** Number of visible text rows on the display. 显示屏可见文本行数。
 *  OLED 128×64 → 8 rows.  LCD 2×16 → 2 rows. */
#define SHELL_DISPLAY_ROWS      8

/** Max length of a single menu item string (including null terminator).
 *  单个菜单项字符串最大长度（含终止符）。 */
#define SHELL_ITEM_TEXT_MAX     32

/** Max depth of deferred action queue. 延迟动作队列最大深度。 */
#define SHELL_DEFERRED_QUEUE    4

// ============================================================
// Display Text Strings — customise per application
// 显示文案 — 按应用自定义
// ============================================================

#define SHELL_TEXT_WELCOME      "Mode Select"
#define SHELL_TEXT_FINISHED     "FINISHED!"
#define SHELL_TEXT_PRESS_ANY    "Press any key.."
#define SHELL_TEXT_WAITING      "Waiting..."
#define SHELL_TEXT_RUNNING      ">>> RUNNING"
#define SHELL_TEXT_CONFIRM_SEL  "Select:"
#define SHELL_TEXT_START        "> START"
#define SHELL_TEXT_START_OFF    "  START"
#define SHELL_TEXT_BACK         "> Back"
#define SHELL_TEXT_BACK_OFF     "  Back"

// ============================================================
// Types
// ============================================================

/* Forward declaration for self-referencing menus. 前向声明。 */
typedef struct shell_menu shell_menu_t;

/** One entry in a menu list. 菜单中的一个条目。
 *
 *  C99 designated initialisers zero all omitted fields.
 *  C99 指定初始化器自动归零所有未提及字段。
 *
 *  Usage / 用法:
 *    { .text="Speed Run", .action=my_fn }              // confirm → exec
 *    { .text="Quick Test", .sub_menu=&sub_qt }          // jump to sub-menu
 *    { .text="T2 Arc A", .action=my_fn, .beeps=3, .beep_ms=500 }  // beep → exec
 */
typedef struct {
    const char         *text;       // display text / 显示文字
    const shell_menu_t *sub_menu;   // sub-menu pointer (NULL = terminal item)
    void               (*action)(void);  // action callback (NULL = no action)
    uint8_t             beeps;      // beep count before action (0 = no beep)
    uint16_t            beep_ms;    // beep duration per beep in ms
} shell_menu_item_t;

/** A menu page — title + item array. 一个菜单页 — 标题 + 条目数组。 */
struct shell_menu {
    const char              *title;  // optional title (NULL = none)
    const shell_menu_item_t *items;  // item array (ROM)
    uint8_t                  count;  // number of items
};

/** Callback configuration block — fill with your platform functions.
 *  回调配置块 — 填入你的平台函数指针。 NULL fields are safely ignored.
 */
typedef struct {
    /* ---- Required / 必填 ---- */

    /** Clear the entire display. 清空显示屏。 */
    void     (*disp_clear)(void);

    /** Draw text at logical (col, row). 在逻辑 (col, row) 处绘制文本。
     *  col=0 left edge, row=0 top edge.  col=0 左边缘, row=0 上边缘。
     *  If your display lib uses uint8_t* instead of const char*,
     *  write a one-line wrapper. 若显示库用 uint8_t*, 写一行 wrapper。 */
    void     (*disp_text)(uint8_t col, uint8_t row, const char *str);

    /** Read key event. Must be non-blocking. 读取按键，必须非阻塞。
     *  @return 0=none, 1=K1 (next/cycle), 2=K2 (confirm/select) */
    uint8_t  (*key_read)(void);

    /* ---- Optional / 可选 (set NULL if unused) ---- */

    /** Fire buzzer: @p n beeps, each @p ms long. 蜂鸣 n 次，每次 ms 毫秒。 */
    void     (*buzzer_beep)(uint8_t n, uint16_t ms);

    /** Is buzzer still active? 蜂鸣器是否仍在响？ */
    bool     (*buzzer_busy)(void);

    /** Has the mission finished? 任务是否已完成？
     *  When this returns true, shell auto-transitions RUNNING → FINISHED.
     *  当返回 true 时，shell 自动从 RUNNING → FINISHED。 */
    bool     (*mission_done)(void);

    /** External command received (e.g. from UART). 收到外部指令。
     *  Called when shell_inject(data) fires in WAIT_EXT state.
     *  当 shell 处于 WAIT_EXT 状态且 shell_inject(data) 被调用时触发。 */
    void     (*on_ext_cmd)(uint8_t data);

    /** Render a custom state screen (SHELL_STATE_CUSTOM + id).
     *  渲染自定义状态界面。Called every tick while in custom state.
     *  在自定义状态中每 tick 调用。 */
    void     (*render_custom)(uint8_t id);

} shell_cfg_t;

/** Shell runtime state. 外壳运行时状态。 */
typedef enum {
    SHELL_STATE_IDLE = 0,       // uninitialised / 未初始化
    SHELL_STATE_WELCOME,        // splash screen / 欢迎页
    SHELL_STATE_MENU,           // menu active / 菜单中
    SHELL_STATE_CONFIRM,        // confirm dialog / 确认页
    SHELL_STATE_WAIT_EXT,       // waiting for external trigger / 等待外部触发
    SHELL_STATE_RUNNING,        // mission in progress / 任务运行中
    SHELL_STATE_FINISHED,       // mission complete / 任务完成
    SHELL_STATE_CUSTOM = 20     // base for user custom states / 用户自定义状态起始
} shell_state_e;

// ============================================================
// Public API
// ============================================================

/** Initialise shell with callbacks and start menu. Call once at boot.
 *  初始化外壳：传入回调配置和起始菜单。启动时调用一次。 */
void shell_init(const shell_cfg_t *cfg, const shell_menu_t *start);

/** Per-frame tick. Call from main loop at ~20–100 Hz.
 *  每帧心跳。在主循环中以约 20–100 Hz 调用。 */
void shell_tick(void);

/** Enter RUNNING state (mission started). 进入运行状态。 */
void shell_set_running(void);

/** Enter WAIT_EXT state (waiting for external command). 进入等待外部指令状态。 */
void shell_wait_ext(void);

/** Inject an external command byte. 注入一个外部指令字节。
 *  If shell is in WAIT_EXT, triggers on_ext_cmd(data).
 *  若 shell 处于 WAIT_EXT 状态，触发 on_ext_cmd(data)。 */
void shell_inject(uint8_t data);

/** Navigate to a specific menu immediately. 立即跳转到指定菜单。 */
void shell_nav_to(const shell_menu_t *menu);

/** Enter custom state (SHELL_STATE_CUSTOM + @p id). 进入自定义状态。 */
void shell_goto_custom(uint8_t id);

/** Schedule: beep @p count times, then call @p fn.
 *  调度：蜂鸣 count 次后调用 fn。
 *  fn(NULL) means no follow-up action.  fn 为 NULL 表示无后续动作。
 *  fn should call shell_set_running() / shell_nav_to() etc. if needed.
 *  fn 内如需切换状态，调用 shell_set_running() 等 API。 */
void shell_defer_beep(uint8_t count, uint16_t ms, void (*fn)(void));

/** Return current shell state. 返回当前外壳状态。 */
shell_state_e shell_state(void);

#endif /* SHELL_CORE_H */

# Shell Module — 前后端分离的嵌入式菜单框架

## 文件职责

| 文件 | 行数 | 你是做什么的 |
|------|------|------------|
| `shell_core.h` | ~130 | 读一眼回调签名和条目字段含义 |
| `shell_core.c` | ~330 | **永远不读** — 后端封装 |
| `app_menu.c` | ~120 | **你唯一编辑的文件** |

## 架构

```
app_menu.c (前端 — 你编辑)        shell_core.c (后端 — 你不动)
─────────────────────────────     ─────────────────────────────
段落 1: g_cfg 填函数指针 ─────────→  存储回调, 每 tick 判空调用
段落 2: 动作函数 ─────────────────→  菜单终点触发, defer 异步链
段落 3: 菜单数组 ─────────────────→  视口滚动渲染, 状态机驱动
段落 4: init() + tick() ──────────→  单例初始化, 5 阶段管线
```

## 快速移植到新项目

**Step 1** — 复制 `shell_core.h` + `shell_core.c` + `app_menu.c` 到项目

**Step 2** — 打开 `app_menu.c`，填段落 1 的 `g_cfg`:

```c
static const shell_cfg_t g_cfg = {
    .disp_clear    = OLED_Clear,        // ← 把你的清屏函数填这里
    .disp_text     = OLED_ShowStr,      // ← 把你的显示函数填这里
    .key_read      = key_get_num,       // ← 把你的按键函数填这里
    .buzzer_beep   = App_Buzzer_Beep,   // ← 或填 NULL
    .buzzer_busy   = App_Buzzer_IsBusy, // ← 或填 NULL
    .mission_done  = NULL,
    .on_ext_cmd    = NULL,
    .render_custom = NULL,
};
```

**Step 3** — 填段落 2 动作函数 + 段落 3 菜单数组

**Step 4** — 主循环调 `app_menu_init()` + `app_menu_tick()`

## 菜单条目规则一览

| 你写的 | shell 行为 |
|--------|-----------|
| `{ .text="...", .sub_menu=&sub }` | 选中即跳转子菜单 |
| `{ .text="...", .action=fn }` | 弹出确认页 → START 执行 |
| `{ .text="...", .action=fn, .beeps=3, .beep_ms=500 }` | 蜂鸣 3 声 → 立即执行 (跳过确认) |
| `{ .text="返回", .sub_menu=&parent }` | 回到上级菜单 |

## 菜单无限嵌套

```
main_menu
 ├─ "视觉循迹" ──→ sub_vision
 │                 ├─ "路线A" ──→ sub_route_a
 │                 │              ├─ "启动" → action
 │                 │              └─ "返回" → &sub_vision
 │                 └─ "返回" → &main_menu
 └─ "快速测试" ──→ sub_quick
                    └─ ...
```

无深度限制。返回上级通过显式指针。

## 蜂鸣时序

```c
// 确认页 → START → 蜂鸣 3 声 → 发车
static void do_launch(void) {
    strategy_start(&strat, get_yaw());
    shell_set_running();                // ← 在 defer 回调里切状态
}
static void act_speed(void) {
    shell_defer_beep(3, 500, do_launch); // ← action 只管触发蜂鸣
}
```

## 外部指令 (如树莓派下发的路线 ID)

```c
// 在动作回调里进入等待:
static void enter_vision(void) {
    send_start_cmd();
    shell_wait_ext();
}

// UART ISR 中注入指令:
void on_pi_data(uint8_t route_id) {
    shell_inject(route_id);  // → 触发 on_ext_cmd(route_id)
}
```

## 内存

`shell_ctx_t` 单例约 180 bytes (含延迟队列)。菜单数据全在 ROM (`const`)。

## 注意事项

- 单例模式 — 一个固件只有一个 shell 实例
- 所有回调在调用前判空，NULL 安全
- `SHELL_DISPLAY_ROWS` 在 `shell_core.h` 顶部，按屏幕行数修改
- WELCOME/FINISHED 文案同样在 `shell_core.h` 顶部 `#define`

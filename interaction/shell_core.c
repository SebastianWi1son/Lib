#include "shell_core.h"
#include <stddef.h>

/*
 * shell_core.c — Application Menu State Machine (Backend)
 * 应用菜单状态机（后端）
 *
 * Singleton — one global instance. All callbacks NULL-guarded.
 * 单例模式，所有回调判空后调用。
 */

// ============================================================
// Internal Types
// ============================================================

typedef enum {
    DQ_NONE = 0,
    DQ_BEEP_EXEC,       // buzzer → then call fn
    DQ_EXEC_ONLY,       // call fn next tick
    DQ_BEEP_ONLY,       // buzzer only
} dq_type_e;

typedef struct {
    dq_type_e  type;
    void      (*fn)(void);
    uint8_t    beep_n;
    uint16_t   beep_ms;
} dq_item_t;

typedef struct {
    shell_cfg_t        cfg;
    const shell_menu_t *menu;

    shell_state_e  state;
    shell_state_e  last_state;
    uint8_t        cursor;
    uint8_t        saved_idx;      // menu index when entering CONFIRM
    void           (*defer_fn)(void);
    bool           ui_dirty;

    dq_item_t      dq[SHELL_DEFERRED_QUEUE];
    uint8_t        dq_head, dq_tail;
    bool           dq_busy;

    bool           ext_pending;
    uint8_t        ext_data;
    uint8_t        custom_id;
    uint32_t       tick;
} sh_t;

static sh_t g;

// ============================================================
// Helpers
// ============================================================

static inline void dirty(void) { g.ui_dirty = true; }

static void set_state(shell_state_e s) {
    g.state = s;
    dirty();
}

// ============================================================
// Deferred Queue (ring buffer)
// ============================================================

static bool dq_push(dq_type_e t, void (*fn)(void), uint8_t n, uint16_t ms) {
    uint8_t nx = (g.dq_tail + 1) % SHELL_DEFERRED_QUEUE;
    if (nx == g.dq_head) return false;
    g.dq[g.dq_tail].type    = t;
    g.dq[g.dq_tail].fn      = fn;
    g.dq[g.dq_tail].beep_n  = n;
    g.dq[g.dq_tail].beep_ms = ms;
    g.dq_tail = nx;
    return true;
}

static bool dq_pop(dq_item_t *out) {
    if (g.dq_head == g.dq_tail) return false;
    *out = g.dq[g.dq_head];
    g.dq_head = (g.dq_head + 1) % SHELL_DEFERRED_QUEUE;
    return true;
}

static void dq_tick(void) {
    /* Wait for buzzer to finish. */
    if (g.dq_busy) {
        if (g.cfg.buzzer_busy != NULL && g.cfg.buzzer_busy())
            return;
        g.dq_busy = false;
        if (g.defer_fn != NULL) {
            void (*fn)(void) = g.defer_fn;
            g.defer_fn = NULL;
            fn();
        }
        return;
    }

    /* Fire deferred fn from a prior beepless completion. */
    if (g.defer_fn != NULL) {
        void (*fn)(void) = g.defer_fn;
        g.defer_fn = NULL;
        fn();
        return;
    }

    /* Process queue. */
    dq_item_t it;
    while (dq_pop(&it)) {
        switch (it.type) {
        case DQ_BEEP_EXEC:
            if (g.cfg.buzzer_beep != NULL) {
                g.cfg.buzzer_beep(it.beep_n, it.beep_ms);
                g.dq_busy   = true;
                g.defer_fn  = it.fn;
            } else if (it.fn != NULL) {
                it.fn();
            }
            return;
        case DQ_EXEC_ONLY:
            if (it.fn != NULL) it.fn();
            break;
        case DQ_BEEP_ONLY:
            if (g.cfg.buzzer_beep != NULL) {
                g.cfg.buzzer_beep(it.beep_n, it.beep_ms);
                g.dq_busy = true;
            }
            return;
        default: break;
        }
    }
}

// ============================================================
// Rendering
// ============================================================

static void render_menu(void) {
    const shell_menu_t *m = g.menu;
    if (m == NULL || g.cfg.disp_text == NULL) return;

    uint8_t vis    = SHELL_DISPLAY_ROWS;
    uint8_t title_r = (m->title != NULL) ? 1 : 0;
    uint8_t rows   = vis - title_r;

    uint8_t start = 0;
    if (g.cursor >= rows && m->count > rows) {
        start = g.cursor - rows + 1;
        if (start > m->count - rows) start = m->count - rows;
    }

    if (m->title != NULL) g.cfg.disp_text(0, 0, m->title);

    for (uint8_t i = 0; i < rows; i++) {
        uint8_t idx = start + i;
        if (idx >= m->count) break;

        char line[SHELL_ITEM_TEXT_MAX + 4];
        uint8_t p = 0;
        line[p++] = (idx == g.cursor) ? '>' : ' ';
        line[p++] = ' ';
        const char *s = m->items[idx].text;
        while (s != NULL && *s && p < sizeof(line) - 1) line[p++] = *s++;
        line[p] = '\0';
        g.cfg.disp_text(0, (uint8_t)(title_r + i), line);
    }
}

static void render_confirm(void) {
    if (g.cfg.disp_text == NULL) return;

    /* "Select: N" */
    char buf[20];
    uint8_t p = 0;
    const char *pfx = SHELL_TEXT_CONFIRM_SEL " ";
    while (*pfx && p < sizeof(buf) - 1) buf[p++] = *pfx++;
    uint8_t id = g.saved_idx + 1;  /* 1-based for display */
    if (id >= 10) buf[p++] = (char)('0' + id / 10);
    buf[p++] = (char)('0' + id % 10);
    buf[p] = '\0';
    g.cfg.disp_text(0, 0, buf);

    g.cfg.disp_text(16, 4, (g.cursor == 0) ? SHELL_TEXT_START
                                            : SHELL_TEXT_START_OFF);
    g.cfg.disp_text(16, 6, (g.cursor == 1) ? SHELL_TEXT_BACK
                                            : SHELL_TEXT_BACK_OFF);
}

static void render_state(void) {
    switch (g.state) {
    case SHELL_STATE_WELCOME:
        if (g.cfg.disp_text != NULL)
            g.cfg.disp_text(12, 3, SHELL_TEXT_WELCOME);
        break;
    case SHELL_STATE_MENU:     render_menu();    break;
    case SHELL_STATE_CONFIRM:  render_confirm();  break;
    case SHELL_STATE_WAIT_EXT:
        if (g.cfg.disp_text != NULL)
            g.cfg.disp_text(16, 2, SHELL_TEXT_WAITING);
        if (g.cfg.render_custom != NULL) g.cfg.render_custom(0);
        break;
    case SHELL_STATE_RUNNING:
        if (g.cfg.disp_text != NULL)
            g.cfg.disp_text(24, 1, SHELL_TEXT_RUNNING);
        if (g.cfg.render_custom != NULL) g.cfg.render_custom(0);
        break;
    case SHELL_STATE_FINISHED:
        if (g.cfg.disp_text != NULL) {
            g.cfg.disp_text(32, 2, SHELL_TEXT_FINISHED);
            g.cfg.disp_text(8,  5, SHELL_TEXT_PRESS_ANY);
        }
        break;
    default:
        if (g.cfg.render_custom != NULL) g.cfg.render_custom(g.custom_id);
        break;
    }
}

// ============================================================
// Input
// ============================================================

static void input(uint8_t key) {
    if (key == 0) return;

    /* ---- Global: RUNNING / WAIT_EXT → abort → FINISHED ---- */
    if (g.state == SHELL_STATE_RUNNING || g.state == SHELL_STATE_WAIT_EXT) {
        set_state(SHELL_STATE_FINISHED); return;
    }
    /* ---- Global: WELCOME → MENU ---- */
    if (g.state == SHELL_STATE_WELCOME) {
        if (g.menu != NULL) { g.state = SHELL_STATE_MENU; g.cursor = 0; }
        dirty(); return;
    }
    /* ---- Global: FINISHED → WELCOME ---- */
    if (g.state == SHELL_STATE_FINISHED) {
        set_state(SHELL_STATE_WELCOME); return;
    }
    /* ---- Global: CUSTOM → WELCOME ---- */
    if (g.state >= SHELL_STATE_CUSTOM) {
        set_state(SHELL_STATE_WELCOME); return;
    }

    /* ---- K1: NEXT ---- */
    if (key == 1) {
        if (g.state == SHELL_STATE_MENU) {
            if (g.menu != NULL && g.menu->count > 0)
                g.cursor = (g.cursor + 1) % g.menu->count;
        } else if (g.state == SHELL_STATE_CONFIRM) {
            g.cursor = (g.cursor + 1) % 2;
        }
        dirty(); return;
    }

    /* ---- K2: CONFIRM ---- */
    if (key == 2) {
        if (g.state == SHELL_STATE_MENU && g.menu != NULL) {
            uint8_t idx = g.cursor;
            if (idx >= g.menu->count) return;

            const shell_menu_item_t *it = &g.menu->items[idx];

            if (it->sub_menu != NULL) {
                /* Navigate to sub-menu. */
                g.menu   = it->sub_menu;
                g.cursor = 0;
                g.state  = SHELL_STATE_MENU;
                dirty(); return;
            }

            if (it->action != NULL) {
                if (it->beeps > 0) {
                    /* Quick-launch: beep → exec. */
                    dq_push(DQ_BEEP_EXEC, it->action, it->beeps, it->beep_ms);
                } else {
                    /* Go to confirm screen. */
                    g.saved_idx = idx;
                    g.cursor    = 0;
                    g.state     = SHELL_STATE_CONFIRM;
                }
                dirty();
            }
            return;
        }

        if (g.state == SHELL_STATE_CONFIRM) {
            if (g.cursor == 1) {
                /* Back → return to menu. */
                g.state = SHELL_STATE_MENU;
            } else {
                /* START → fire action from saved menu index. */
                if (g.menu != NULL && g.saved_idx < g.menu->count) {
                    const shell_menu_item_t *it = &g.menu->items[g.saved_idx];
                    if (it->action != NULL) it->action();
                }
            }
            dirty(); return;
        }
    }
}

// ============================================================
// Public API
// ============================================================

void shell_init(const shell_cfg_t *cfg, const shell_menu_t *start) {
    if (cfg == NULL) return;
    g.cfg       = *cfg;
    g.menu      = start;
    g.state     = SHELL_STATE_WELCOME;
    g.last_state= SHELL_STATE_IDLE;
    g.cursor    = 0;
    g.saved_idx = 0;
    g.defer_fn  = NULL;
    g.ui_dirty  = true;
    g.dq_head   = 0;
    g.dq_tail   = 0;
    g.dq_busy   = false;
    g.ext_pending = false;
    g.ext_data  = 0;
    g.custom_id = 0;
    g.tick      = 0;
}

void shell_tick(void) {
    g.tick++;

    /* 1. Deferred actions (beep → exec chain). */
    dq_tick();

    /* 2. External trigger. */
    if (g.ext_pending) {
        g.ext_pending = false;
        if (g.state == SHELL_STATE_WAIT_EXT && g.cfg.on_ext_cmd != NULL)
            g.cfg.on_ext_cmd(g.ext_data);
    }

    /* 3. Input. */
    if (g.cfg.key_read != NULL)
        input(g.cfg.key_read());

    /* 4. Auto-complete: mission_done → FINISHED. */
    if (g.state == SHELL_STATE_RUNNING && g.cfg.mission_done != NULL) {
        if (g.cfg.mission_done()) set_state(SHELL_STATE_FINISHED);
    }

    /* 5. Render. */
    if (g.ui_dirty || g.state != g.last_state) {
        if (g.state != g.last_state) {
            if (g.cfg.disp_clear != NULL) g.cfg.disp_clear();
            g.last_state = g.state;
        }
        render_state();
        g.ui_dirty = false;
    }
}

void shell_set_running(void)  { set_state(SHELL_STATE_RUNNING); }
void shell_wait_ext(void)     { set_state(SHELL_STATE_WAIT_EXT); }

void shell_inject(uint8_t data) {
    g.ext_pending = true;
    g.ext_data    = data;
}

void shell_nav_to(const shell_menu_t *menu) {
    if (menu == NULL) return;
    g.menu   = menu;
    g.cursor = 0;
    set_state(SHELL_STATE_MENU);
}

void shell_goto_custom(uint8_t id) {
    g.custom_id = id;
    set_state((shell_state_e)(SHELL_STATE_CUSTOM + id));
}

void shell_defer_beep(uint8_t count, uint16_t ms, void (*fn)(void)) {
    if (count > 0)
        dq_push(DQ_BEEP_EXEC, fn, count, ms);
    else if (fn != NULL)
        dq_push(DQ_EXEC_ONLY, fn, 0, 0);
}

shell_state_e shell_state(void) { return g.state; }

# FOC 模块 — snake_case 修复版, 运行时模式切换

## 文件职责

```
output/foc/
├── foc.h              # 类型定义 + API 声明 + foc_ctrl_mode_t 枚举
├── foc.c              # 实现: 控制编排 + SVPWM + 对齐 + 7 个 bugfix
├── foc_transform.h    # 通用数学: Clarke / Park / Inverse Park (电压/电流模式共用)
└── README.md

output/foc_current/
├── foc_current.h      # 电流专用: foc_current_update / foc_current_overlimit
├── foc_current.c      # 占位 (后续: 校准、诊断、I²t 保护)
└── README.md
```

## 控制模式 (运行时切换)

```c
typedef enum {
    FOC_CTRL_VOLTAGE   = 0,  // 电压模式: 位置→速度→Uq (默认, 无需电流传感器)
    FOC_CTRL_CURRENT   = 1,  // 电流模式: 位置→速度→Iq→IqPID→Uq + Id→0
    FOC_CTRL_OPEN_LOOP = 2,  // 开环 (测试用)
} foc_ctrl_mode_t;
```

### 自动探测 (init-time)

```c
foc_init(&motor, &cfg, &hw);
// 若 get_current_cb != NULL → 自动设为 FOC_CTRL_CURRENT
// 否则保持 FOC_CTRL_VOLTAGE
```

### 运行时切换

```c
// Vofa+ / API 直接改
motor.cfg.ctrl_mode = FOC_CTRL_CURRENT;   // 切电流模式
motor.cfg.ctrl_mode = FOC_CTRL_VOLTAGE;   // 切回电压模式
```

### 故障自动回退

```c
// foc_angle_control_tick 中:
case FOC_CTRL_CURRENT:
    if (motor->hw.get_current_cb) {
        foc_current_control_step(...);  // 正常电流闭环
        break;
    }
    // 传感器离线 → 自动回退电压模式 (无感, 不丢控制)
    /* fallthrough */
case FOC_CTRL_VOLTAGE:
    foc_svpwm_write(motor, uq_ref, 0, angle_elec);
    break;
```

## 控制架构

### VOLTAGE 模式 (默认)
```
target → traj planner → pos PID → vel PID → Uq ·→ SVPWM
           ↑                        ↑
     encoder angle             encoder velocity (LPF)
```

### CURRENT 模式
```
target → traj planner → pos PID → vel PID → Iq_ref
                                             ↓
                      foc_current_update ← ADC Ia,Ib
                             ↓
                      Clarke → Park
                             ↓
                    [iq, id] ─→ Iq PID → Uq ┐
                    0 ────────→ Id PID → Ud ├→ InvPark → SVPWM
                                            │
                              electrical angle
```

## Bug 修复清单

| # | 位置 | 问题 | 修复 |
|---|------|------|------|
| 1 | `foc_angle_control_tick` | 前馈除法未检查 dt≤0 | 函数头 `if (dt <= 0.0f) return;` |
| 2 | `foc_open_loop_velocity_tick` | 开环积分未检查 dt | 同上 |
| 3 | `pid.c` (上游) | 积分分离时积分器仍累积 | 记录, 建议修复见下方 |
| 4 | `foc_init` | raw_angle/abs_angle 未初始化 | 显式赋值 0.0f |
| 5 | `foc_svpwm_write` | Ud 未限幅 | 加 Ud clamp |
| 6 | `app_gimbal.c` (上游) | 传感器故障不清零 PWM | 记录, 建议修复见下方 |
| 7 | `foc_start_and_sync` | planner.ramp_target 未同步 | 赋值 settled |

### 建议上游修复 (Bug 3 — pid.c)

```c
// 仅在非分离期间累加积分
if (pid->separation_err <= 0.0f || fabsf(error) <= pid->separation_err) {
    pid->integral += pid->ki * dt * 0.5f * (error + pid->error_prev);
}
```

### 建议上游修复 (Bug 6 — app_gimbal.c)

```c
if (system_runflag == 2) {
    foc_open_loop_velocity_tick(&gimbal_pitch, 2.0f, 1.5f, dt);
} else if (as5600_is_healthy()) {
    foc_sensor_update(&gimbal_pitch, dt);
    foc_angle_control_tick(&gimbal_pitch, target_angle_pitch, dt);
} else {
    gimbal_pitch.hw.set_pwm_cb(0.0f, 0.0f, 0.0f);  // 传感器故障 → 断电
}
```

## 命名映射 (从 bsp/middleware/simple_foc/ 迁移)

| 旧 | 新 |
|----|----|
| `FOC_Motor_Handle_t` | `foc_motor_t` |
| `FOC_Hardware_t` | `foc_hardware_t` |
| `FOC_Config_t` | `foc_config_t` |
| `FOC_PID_Param_t` | `foc_pid_param_t` |
| `FOC_Init()` | `foc_init()` |
| `FOC_Enable()` / `FOC_Disable()` | `foc_enable()` / `foc_disable()` |
| `FOC_Start_And_Sync()` | `foc_start_and_sync()` |
| `FOC_Sensor_Update()` | `foc_sensor_update()` |
| `FOC_Angle_Control_Tick()` | `foc_angle_control_tick()` |
| `FOC_Open_Loop_Velocity_Tick()` | `foc_open_loop_velocity_tick()` |
| `_2PI` / `_SQRT3` | `TWO_PI` / `SQRT3` |

`pid_instance_t`、`dsp_traj_t`、`dsp_lpf_t` 不变 (来自 bsp/middleware/math_algo/).

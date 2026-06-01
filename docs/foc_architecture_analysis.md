# FOC 架构分析: Inline 电流采样位于哪一层?

## 问题

当前项目使用电压模式 FOC（无电流传感器）。未来加入电流闭环时，
inline 电流采样代码应放在三层架构的哪一层？

## 结论

**电流采样应放在 `bsp/driver/adc/` 层**，与编码器驱动同级。
通过 `foc_hardware_t` 回调注入 FOC 中间件，与现有的 `get_angle_cb` 模式完全一致。

## 理由

### 1. 硬件层级对等

| 外设 | 驱动层位置 | 回调 | 数据类型 |
|------|-----------|------|---------|
| SPI (AS5048A 编码器) | `bsp/driver/encoder/as5048a/` | `get_angle_cb` | `float` (rad) |
| I2C (AS5600 编码器) | `bsp/driver/encoder/as5600/` | `get_angle_cb` | `float` (rad) |
| ADC (电流采样) | `bsp/driver/adc/current_sense/` | `get_current_cb` | `float[2]` (A) |
| TIM1/8 PWM (电机) | `bsp/driver/motor/m3510/` `m2804/` | `set_pwm_cb` | `float[3]` (V) |

ADC 是标准的硬件外设，与 SPI、I2C、TIM 同级。不应该放在中间件层。

### 2. 与现有回调模式一致

当前 `foc_hardware_t` 已经有 4 个回调:

```c
typedef struct {
  float (*get_angle_cb)(void);        // 编码器 → 角度
  void (*get_current_cb)(float*,float*); // ADC → 电流 (新增)
  void (*set_pwm_cb)(float,float,float);  // 电压 → PWM
  void (*delay_ms_cb)(uint32_t);       // 阻塞延时
  void (*enable_cb)(uint8_t);          // 使能控制
} foc_hardware_t;
```

`get_current_cb` 与 `get_angle_cb` 在架构上是完全对称的:
- 都是从硬件外设读取物理量
- 都转换为工程单位 (rad, A)
- 都在 ISR 中调用
- 都不应发起阻塞操作 (只读 DMA 缓冲)

### 3. FOC 中间件保持硬件无关

`bsp/middleware/simple_foc/` 层不包含任何 `#include "main.h"` 或 HAL 引用。
它只通过 `foc_hardware_t` 回调与硬件交互。这个设计原则保证了:

- FOC 算法可以在任何 MCU 上编译 (STM32, ESP32, RP2040...)
- FOC 算法可以在 PC 上进行 SIL (Software-In-the-Loop) 仿真
- 更换 ADC 芯片时只需修改驱动层，FOC 算法不受影响

如果把 ADC 读取代码放入中间件层 (如 `foc.c` 直接调用 `HAL_ADC_*`)，
上述所有优势都会丢失。

### 4. 时序耦合由硬件处理，不由中间件处理

关键认知: **ADC 采样必须与 PWM 同步** (在 PWM 中心点触发，避开开关噪声)。
但这个同步是**硬件层面的**:

```
TIM1 (PWM) ──TRGO──> ADC1 (注入采样) ──DMA──> SRAM buffer
```

软件只需要从 SRAM buffer 读取最新值。软件不需要关心触发时机。

因此:
- **驱动层** (`current_sense.c`): 配置 TIM1→ADC1 触发链，配置 DMA
- **回调** (`get_current_cb`): 从 DMA buffer 读到最新电流值
- **FOC 中间件**: 完全不知道触发链的存在

### 5. 校准属于驱动层

电流采样的校准 (零电流偏移、增益误差) 是硬件相关的:
- 不同采样电阻值 → 不同增益
- 不同运放 → 不同偏移
- 不同温度 → 不同漂移

这些校准逻辑应封装在 `current_sense.c` 中，对上层透明。

## 推荐驱动层接口

```c
// bsp/driver/adc/current_sense/current_sense.h

/* 初始化 ADC + DMA + TIM 触发链 */
void current_sense_init(void);

/* 读取相电流 Ia, Ib (A) — 作为 foc_hardware_t.get_current_cb */
void current_sense_get(float *ia, float *ib);

/* 上电校准: 记录零电流时的 ADC 偏移 */
void current_sense_calibrate_offset(void);

/* 过流标志 (可由 app 层轮询) */
uint8_t current_sense_is_overcurrent(void);
```

## 层级全景图

```
┌──────────────────────────────────────────────────────────┐
│  Application Layer (app/)                                 │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────────┐  │
│  │ app_gimbal.c │  │  app_tele.c  │  │   app_com.c    │  │
│  │ 接线回调     │  │  Vofa+遥测   │  │  协议通信      │  │
│  └──────┬───────┘  └──────┬───────┘  └───────┬────────┘  │
├─────────┼─────────────────┼───────────────────┼──────────┤
│  Middleware (bsp/middleware/)                  │          │
│  ┌──────┴──────────────────┴───────────────────┴────────┐ │
│  │              simple_foc/                              │ │
│  │  foc.h: foc_hardware_t { get_angle_cb,               │ │
│  │                          get_current_cb,  ← 新增     │ │
│  │                          set_pwm_cb, ... }           │ │
│  │  foc.c: FOC 控制循环, 与硬件零耦合                    │ │
│  │  foc_current.h: Clarke/Park 变换 (static inline)     │ │
│  └──────────────────────────────────────────────────────┘ │
├──────────────────────────────────────────────────────────┤
│  BSP Drivers (bsp/driver/) — 硬件抽象                    │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────────┐ │
│  │ encoder/ │ │ encoder/ │ │ motor/   │ │ adc/        │ │
│  │ as5048a/ │ │ as5600/  │ │ m3510/   │ │ current_    │ │
│  │ (SPI)    │ │ (I2C)    │ │ m2804/   │ │ sense/  ← 新│ │
│  └──────────┘ └──────────┘ └──────────┘ └────────────┘ │
├──────────────────────────────────────────────────────────┤
│  Core/ — STM32CubeMX HAL                                 │
│  adc.c  spi.c  i2c.c  tim.c  dma.c  gpio.c             │
└──────────────────────────────────────────────────────────┘
```

## 关于 "Inline" 采样的说明

"Inline" 在此语境下指 **在 ISR 内部直接读取传感器数据**，而非通过 RTOS 任务或 DMA
中断回调异步处理。当前项目已使用 inline 模式读取编码器:

```c
// ISR 中直接读编码器 (inline)
FOC_Sensor_Update(&motor, dt)
  → motor->hw.get_angle_cb()  // 阻塞 SPI 读取
```

电流采样的 inline 模式更优:
```c
// ISR 中直接读 DMA buffer (inline, 非阻塞)
FOC_Current_Update(&motor)
  → motor->hw.get_current_cb()  // 读 DMA buffer, <1μs
```

因为 ADC 由 PWM 定时器硬件触发 + DMA 自动搬运，`get_current_cb` 只需读内存，
不需要发起外设传输。这比当前编码器的阻塞 SPI 读取更高效。

## 进一步优化建议

1. **编码器也改成 DMA 模式**: 当前 `as5048a_get_angle_cb` 在 ISR 中做阻塞 SPI 传输。
   可参考电流采样的模式，SPI 由定时器周期性触发 DMA 读取，ISR 只读 DMA buffer。
2. **统一的传感器更新**: 如果编码器和电流都走 DMA，`FOC_Sensor_Update` 和
   `FOC_Current_Update` 都是纯内存读取，可以合并为一个函数，减少 ISR 中的函数调用。

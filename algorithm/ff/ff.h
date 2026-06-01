#ifndef __FF_H
#define __FF_H

/*
 * Feedforward (FF) Toolbox — header-only, zero overhead.
 * 前馈工具箱（纯头文件，零调用开销）。
 *
 * Feedforward acts on measured or known quantities BEFORE error develops,
 * complementing the PID feedback loop which reacts AFTER error appears:
 *
 *   output = pid_calculate(pid, target, measure, dt)    // feedback
 *          + ff_rate(base, measured_rate, gain)         // feedforward
 *
 * 前馈在误差产生之前基于已知/测量量预判补偿，与事后修正的 PID 反馈互补。
 */

/* ==================================================================== */
/* === 1. Rate Feedforward — 速率比例前馈                             === */
/* ==================================================================== */

/**
 * @brief   Rate-proportional feedforward: output = base + gain * rate.
 *          速率比例前馈：输出 = 基值 + 增益 × 测量速率。
 *
 *          Usage scenarios / 使用场景:
 *          - Gyro angular-velocity damping on a line-following chassis:
 *            ff_rate(pid_output, gyro_deg_per_s, gyro_gain)
 *            → faster rotation → stronger counter-steer, suppresses hunting.
 *            寻迹底盘陀螺角速度阻尼：转速越快反向抑制力越大，抑制蛇形摆动。
 *
 *          - Motor velocity FF (back-EMF compensation):
 *            ff_rate(pid_output, measured_rpm, kv)
 *            → pre-empts the voltage drop caused by motor speed.
 *            电机速度前馈（反电动势补偿）：预判转速引起的电压降。
 *
 *          - Any system where a measured rate predicts the required
 *            control effort before the PID error builds up.
 *            任何能用实测速率预判控制量的系统。
 *
 * @param   base  Base signal (typically PID output)
 * @param   rate  Measured rate (angular velocity, linear speed, etc.)
 * @param   gain  Feedforward gain — 0 = disabled
 * @return        base + gain * rate
 */
static inline float ff_rate(float base, float rate, float gain) {
  return base + gain * rate;
}

/* ==================================================================== */
/* === 2. Fixed Bias — 固定偏置前馈                                   === */
/* ==================================================================== */

/**
 * @brief   Fixed-bias feedforward: output = base + bias.
 *          Adds a constant offset regardless of system state.
 *          固定偏置前馈：叠加一个与系统状态无关的恒定偏置。
 *
 *          Usage scenarios / 使用场景:
 *          - Arc-direction compensation in differential-drive navigation:
 *            ff_bias(pid_output, direction * arc_gain)
 *            → a known curvature demands a persistent steer offset.
 *            差速底盘圆弧导航：已知曲率方向需要持续的转向偏置。
 *
 *          - Static load compensation:
 *            ff_bias(pid_output, gravity_offset)
 *            → counter a constant external force (e.g. inclined surface).
 *            静态负载补偿：抵消恒定外力（如斜面重力分量）。
 *
 * @param   base  Base signal
 * @param   bias  Constant offset (signed)
 * @return        base + bias
 */
static inline float ff_bias(float base, float bias) {
  return base + bias;
}

/* ==================================================================== */
/* === 3. Derivative Feedforward — 目标变化率前馈 (DF, 未使用)        === */
/* ==================================================================== */

/**
 * @brief   Derivative feedforward based on target rate of change:
 *          output = gain * d(target)/dt.
 *          目标变化率微分前馈：输出 = 增益 × 目标变化速率。
 *
 *          Usage scenarios / 使用场景:
 *          - Servo motor tracking: pre-emptively compensates for the
 *            inertia / back-EMF of accelerating the load before the
 *            position error builds up. Industry standard in CNC and
 *            robot joint control.
 *            伺服电机跟踪：在位置误差产生之前预判加速负载所需的力矩，
 *            CNC 和机器人关节控制的工业标准做法。
 *
 *          NOT currently used in this chassis project.
 *          当前寻迹底盘项目未使用此函数。
 *
 *          Caller maintains prev_target across iterations:
 *          调用方自行维护 prev_target 跨迭代:
 *          @code
 *            float out = pid_calculate(pid, target, measure, dt)
 *                      + ff_derivative(target, prev_target, dt, ff_gain);
 *            prev_target = target;
 *          @endcode
 *
 * @param   target      Current setpoint
 * @param   prev_target Setpoint from previous iteration
 * @param   dt          Delta time in seconds
 * @param   gain        Feedforward gain
 * @return              gain * (target - prev_target) / dt
 */
static inline float ff_derivative(float target, float prev_target,
                                   float dt, float gain) {
  return gain * (target - prev_target) / dt;
}

/* ==================================================================== */
/* === Project Wrappers — 项目专用语义封装                            === */
/* ==================================================================== */
/*
 * Thin wrappers over the primitives above. They exist purely for call-site
 * readability — the function name alone tells you what physical effect is
 * being compensated, without tracing parameter names or consulting comments.
 * 以下是对基元函数的薄封装，仅为提升调用点的可读性——函数名本身就能说明
 * 正在补偿什么物理效应，无需追溯参数名或查阅注释。
 *
 * Zero runtime overhead: all are static inline, the compiler folds them
 * into the same code as calling the primitive directly.
 * 零运行时开销：全部 static inline，编译器会将其折叠为直接调用基元的代码。
 */

/**
 * @brief   Gyro angular-velocity damping.
 *          陀螺角速度阻尼前馈。
 *          ff_gyro_damp(p, gyro_rate, kd) ≡ ff_rate(p, gyro_rate, kd)
 *          Call site / 调用点: nav_action_tracking
 */
static inline float ff_gyro_damp(float base, float gyro_rate, float kd) {
  return ff_rate(base, gyro_rate, kd);
}

/**
 * @brief   Arc-direction steering bias for differential-drive navigation.
 *          差速底盘圆弧方向转向偏置前馈。
 *          ff_arc_bias(p, direction, gain) ≡ ff_bias(p, -direction * gain)
 *          Call site / 调用点: nav_action_arc_enter, nav_action_arc_track
 */
static inline float ff_arc_bias(float base, float direction, float gain) {
  return ff_bias(base, -direction * gain);
}

#endif

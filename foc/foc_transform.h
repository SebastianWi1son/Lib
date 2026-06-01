#ifndef __FOC_TRANSFORM_H
#define __FOC_TRANSFORM_H

/**
 * @file    foc_transform.h
 * @brief   FOC 通用坐标变换 — Clarke / Park / Inverse Park
 *
 * 这些变换是 FOC 算法的数学基础, 与电流环、电压环的具体实现无关。
 * 全部为 static inline, 编译到调用方的编译单元中, 零调用开销。
 *
 * 使用方:
 *   foc_svpwm_write()       → foc_inv_park_transform()  (电压模式 & 电流模式)
 *   foc_angle_control_tick() → foc_clarke_transform()    (电流模式)
 *                            → foc_park_transform()      (电流模式)
 *
 * 变换链路:
 *   ia, ib, ic (三相静止)
 *     → Clarke  → i_alpha, i_beta (两相静止)
 *     → Park    → iq, id (两相旋转, dq 坐标系)
 *     → PID     → uq, ud
 *     → InvPark → u_alpha, u_beta
 *     → SVPWM   → ua, ub, uc
 */

#include <math.h>

/* ================================================================
 * Clarke 变换 — 三相静止 → 两相静止
 *
 *   i_alpha = ia
 *   i_beta  = (ia + 2*ib) / sqrt(3)
 *
 * 等幅值变换: |I_alpha_beta| = |I_abc|
 * Ic 不需要作为输入 (由 Ia + Ib + Ic = 0 隐式得到)
 * ================================================================ */

static inline void foc_clarke_transform(float ia, float ib,
                                         float *i_alpha, float *i_beta) {
  *i_alpha = ia;
  *i_beta  = (ia + 2.0f * ib) / 1.73205080757f;
}

/* ================================================================
 * Park 变换 — 两相静止 → 两相旋转
 *
 *   iq =  i_alpha * cos(θ) + i_beta * sin(θ)
 *   id = -i_alpha * sin(θ) + i_beta * cos(θ)
 *
 * θ = 电角度 (rad), 由 abs_angle * pole_pairs * direction 计算
 * ================================================================ */

static inline void foc_park_transform(float i_alpha, float i_beta,
                                       float angle_elec,
                                       float *iq, float *id) {
  float cos_a = cosf(angle_elec);
  float sin_a = sinf(angle_elec);
  *id =  i_alpha * cos_a + i_beta * sin_a;   /* d = flux */
  *iq = -i_alpha * sin_a + i_beta * cos_a;   /* q = torque */
}

/* ================================================================
 * Inverse Park 变换 — 两相旋转 → 两相静止
 *
 *   u_alpha = -uq * sin(θ) + ud * cos(θ)
 *   u_beta  =  uq * cos(θ) + ud * sin(θ)
 *
 * 电压模式和电流模式共用:
 *   电压模式: uq = 速度 PID 输出, ud = 0
 *   电流模式: uq = Iq PID 输出,  ud = Id PID 输出
 * ================================================================ */

static inline void foc_inv_park_transform(float uq, float ud,
                                           float angle_elec,
                                           float *u_alpha, float *u_beta) {
  float cos_a = cosf(angle_elec);
  float sin_a = sinf(angle_elec);
  *u_alpha = -uq * sin_a + ud * cos_a;
  *u_beta  =  uq * cos_a + ud * sin_a;
}

#endif /* __FOC_TRANSFORM_H */

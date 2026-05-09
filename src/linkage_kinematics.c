/**
 * @file linkage_kinematics.c
 * @brief 2-DOF 腿机构运动学: IK/FK 实现 (CMSIS-DSP 加速)
 *
 * STM32F4 硬件 FPU + CMSIS-DSP 三角函数加速。
 * arm_sin_cos_f32 单次调用同时得 sin/cos, 比两次标准库调用省 ~30% 周期。
 * arm_sqrt_f32 使用 FPU VSQRT 指令, 比 sqrtf 快 ~15%。
 * atan2f / acosf 在 CMSIS-DSP 中无对应, 保留 math.h。
 */

#include "linkage_kinematics.h"
#include <math.h>  /* acosf, atan2f, fabsf — CMSIS-DSP 无等价函数 */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* --------------------------------------------------
 *  IK:  h, phi  →  theta_a, theta_b
 *
 *  机构坐标系: +X 右 +Y 上, O 为电机同轴原点
 *  vertical-down = (-π/2), phi 从垂直向下顺时针为正
 *  angle(OP7) = -π/2 - phi
 *  P7 = h * (cos(angle), sin(angle))
 *     = h * (-sin(phi), -cos(phi))
 * -------------------------------------------------- */

lk_error_t lk_inverse(float h, float phi, int elbow,
                      const Kinematics_Config *cfg,
                      float *out_theta_a, float *out_theta_b)
{
	/* --- 范围检查 --- */
	if (h < LK_H_MIN_MM - 1e-4f || h > LK_H_MAX_MM + 1e-4f) {
		return LK_ERR_H_OUT_OF_RANGE;
	}

	const float L1 = LK_L1_MM;
	const float L2 = LK_L2_MM;

	/* P7 in mechanism frame: phi=0 → OP7 = (0, -h) */
	float s_phi, c_phi;
	arm_sin_cos_f32(phi, &s_phi, &c_phi);
	float Px = -h * s_phi;
	float Py = -h * c_phi;

	float r;
	arm_sqrt_f32(Px * Px + Py * Py, &r);

	/* degenerate: r ≈ 0 */
	if (r < 1e-6f) {
		return LK_ERR_UNREACHABLE;
	}

	/* Cosine law: (r² + L1² - L2²) / (2·L1·r) */
	float r2 = r * r;
	float L1_2 = L1 * L1;
	float L2_2 = L2 * L2;
	float cos_alpha = (r2 + L1_2 - L2_2) / (2.0f * L1 * r);

	cos_alpha = lk_clamp(cos_alpha, -1.0f, 1.0f);
	float alpha = acosf(cos_alpha);

	float phi_vec = atan2f(Py, Px);

	/* 选择 elbow 分支 */
	float sign = (elbow >= 0) ? 1.0f : -1.0f;
	float theta_a = phi_vec + sign * alpha;
	theta_a = lk_wrap_pi(theta_a);

	/* theta_b from geometry */
	float sin_a, cos_a;
	arm_sin_cos_f32(theta_a, &sin_a, &cos_a);
	float theta_b = atan2f(Py - L1 * sin_a,
	                       Px - L1 * cos_a);

	*out_theta_a = theta_a;
	*out_theta_b = theta_b;

	(void)cfg; /* IK 返回机构帧角度, offset 由调用方处理 */
	return LK_OK;
}

lk_error_t lk_inverse_continuous(float h, float phi,
                                 float prev_theta_a, float prev_theta_b,
                                 const Kinematics_Config *cfg,
                                 float *out_theta_a, float *out_theta_b)
{
	float ta_up, tb_up, ta_dn, tb_dn;
	lk_error_t e_up = lk_inverse(h, phi, +1, cfg, &ta_up, &tb_up);
	lk_error_t e_dn = lk_inverse(h, phi, -1, cfg, &ta_dn, &tb_dn);

	if (e_up != LK_OK && e_dn != LK_OK) {
		return LK_ERR_UNREACHABLE;
	}
	if (e_up != LK_OK) {
		*out_theta_a = ta_dn; *out_theta_b = tb_dn;
		return LK_OK;
	}
	if (e_dn != LK_OK) {
		*out_theta_a = ta_up; *out_theta_b = tb_up;
		return LK_OK;
	}

	/* both valid → 选离 prev 最近的分支 */
	float da_up  = lk_wrap_pi(ta_up - prev_theta_a);
	float db_up  = lk_wrap_pi(tb_up - prev_theta_b);
	float da_dn  = lk_wrap_pi(ta_dn - prev_theta_a);
	float db_dn  = lk_wrap_pi(tb_dn - prev_theta_b);

	float dist_up = da_up * da_up + db_up * db_up;
	float dist_dn = da_dn * da_dn + db_dn * db_dn;

	if (dist_up <= dist_dn) {
		*out_theta_a = ta_up; *out_theta_b = tb_up;
	} else {
		*out_theta_a = ta_dn; *out_theta_b = tb_dn;
	}
	return LK_OK;
}

/* --------------------------------------------------
 *  FK:  theta_a, theta_b  →  h, phi
 *
 *  P7 = L1*[cos(θa), sin(θa)] + L2*[cos(θb), sin(θb)]
 *  h = |P7|
 *  phi = -atan2(P7_y, P7_x) - π/2   (clockwise from vertical-down)
 * -------------------------------------------------- */

lk_error_t lk_forward(float theta_a, float theta_b,
                      const Kinematics_Config *cfg,
                      float *out_h, float *out_phi)
{
	const float L1 = LK_L1_MM;
	const float L2 = LK_L2_MM;

	float sin_a, cos_a, sin_b, cos_b;
	arm_sin_cos_f32(theta_a, &sin_a, &cos_a);
	arm_sin_cos_f32(theta_b, &sin_b, &cos_b);

	float Px = L1 * cos_a + L2 * cos_b;
	float Py = L1 * sin_a + L2 * sin_b;

	arm_sqrt_f32(Px * Px + Py * Py, out_h);

	/* angle(OP7) = atan2(Py, Px)
	 * phi = -(angle + π/2)  (clockwise from vertical-down) */
	*out_phi = -atan2f(Py, Px) - (M_PI / 2.0f);
	*out_phi = lk_wrap_pi(*out_phi);

	(void)cfg;
	return LK_OK;
}

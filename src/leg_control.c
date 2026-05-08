/**
 * @file leg_control.c
 * @brief 腿足控制胶水层实现
 *
 * IK 解算 → 软限位 → 力矩前馈 → dm4310_set_pos_with_offset
 * 不使用 lk_motor_a/b() 以避免与 g_dm_offset 双重叠加。
 */
#include "leg_control.h"
#include "linkage_kinematics.h"
#include "dm4310_motor.h"
#include "motor_debug.h"

#include <math.h>
#include <zephyr/sys/printk.h>

/* 足端垂直力估计 (N), 每条腿约车重一半 */
#define EST_FOOT_FORCE_N  20.0f

/* 软限位: h 低于此值进入近奇异区, 扭矩放大 >3.5x */
#define H_SOFT_LIMIT_MM    45.0f

static void compute_feedforward(float h_mm, float phi_rad,
				float theta_a, float theta_b,
				uint8_t m1, uint8_t m2)
{
	float sin_phi, cos_phi;
	float Px, cos_tb;

	/* arm_sin_cos_f32 不可用时的回退 */
	sin_phi = sinf(phi_rad);
	cos_phi = cosf(phi_rad);

	/* Px: 末端水平偏移 (mm) */
	Px = -h_mm * sin_phi;

	/* τa ∝ Px (水平偏移越大, 关节 a 力矩越大) */
	g_dm4310.feedforward_tau[m1 - 1] = -Px * EST_FOOT_FORCE_N / 1000.0f;

	/* τb ∝ L2·cos(θb) (蹲低时最大, 站立时减小) */
	cos_tb = cosf(theta_b);
	g_dm4310.feedforward_tau[m2 - 1] = -LK_L2_MM * cos_tb * EST_FOOT_FORCE_N / 1000.0f;
}

/* 左腿: M1=θa, M2=θb */
int leg_move_to_left(float h_mm, float phi_rad)
{
	float ta, tb;
	float h_clamped = h_mm;

	if (h_clamped < H_SOFT_LIMIT_MM) {
		static uint32_t warn_cnt;
		if ((warn_cnt++ % 500) == 0) {
			printk("WARNING: h=%.1f < %dmm, clamping (near-singular)\n",
			       (double)h_mm, (int)H_SOFT_LIMIT_MM);
		}
		h_clamped = H_SOFT_LIMIT_MM;
	}

	lk_error_t e = lk_inverse(h_clamped, phi_rad, -1, NULL, &ta, &tb);
	if (e != LK_OK) {
		return -(int)e;
	}

	/* 左腿: M1→θa, M2→θb */
	float motor_a = lk_theta_a_to_m1(ta);
	float motor_b = lk_theta_b_to_m2(tb);

	compute_feedforward(h_clamped, phi_rad, ta, tb, 1, 2);
	dm4310_set_pos_with_offset(1, motor_a);
	dm4310_set_pos_with_offset(2, motor_b);
	telemetry_record_cmd(h_clamped, phi_rad);
	return 0;
}

/* 右腿: M4=θa, M3=θb (M3/M4对调), φ 取反以实现镜像对称 */
int leg_move_to_right(float h_mm, float phi_rad)
{
	float ta, tb;
	float phi_mirror = -phi_rad;
	float h_clamped = h_mm;

	if (h_clamped < H_SOFT_LIMIT_MM) {
		printk("WARNING: h=%.1f < %dmm, clamping to limit (near-singular)\n",
		       (double)h_mm, (int)H_SOFT_LIMIT_MM);
		h_clamped = H_SOFT_LIMIT_MM;
	}

	lk_error_t e = lk_inverse(h_clamped, phi_mirror, -1, NULL, &ta, &tb);
	if (e != LK_OK) {
		return -(int)e;
	}

	float motor_a = lk_theta_a_to_m4(ta);
	float motor_b = lk_theta_b_to_m3(tb);

	compute_feedforward(h_clamped, phi_mirror, ta, tb, 3, 4);
	dm4310_set_pos_with_offset(3, motor_b);
	dm4310_set_pos_with_offset(4, motor_a);
	return 0;
}

int leg_move_all(float h_mm, float phi_rad)
{
	int ret_l = leg_move_to_left(h_mm, phi_rad);
	int ret_r = leg_move_to_right(h_mm, phi_rad);
	return (ret_l != 0) ? ret_l : ret_r;
}

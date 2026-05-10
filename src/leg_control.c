/**
 * @file leg_control.c
 * @brief 腿足控制胶水层实现
 *
 * IK 解算 → 关节限位检查 → 力矩前馈 → dm4310_set_pos_with_offset
 * 对两个 elbow 分支都求解，只选关节限位内的，避免 wrap_pi 跨 ±180° 误判。
 */

#include "leg_control.h"
#include "linkage_kinematics.h"
#include "dm4310_motor.h"
#include "motor_debug.h"

#include <math.h>
#include <zephyr/sys/printk.h>

#define EST_FOOT_FORCE_N  0.0f  /* 暂时关闭前馈, 纯位置环测试直线 */
#define MOTOR_DELTA_LIMIT_RAD LK_MOTOR_MAX_DELTA_RAD
#define JOINT_MARGIN_RAD   LK_THETA_MARGIN_RAD

static float prev_ta_left, prev_tb_left;
static float prev_ta_right, prev_tb_right;

static void compute_feedforward(float h_mm, float phi_rad,
				float theta_a, float theta_b,
				uint8_t m1, uint8_t m2)
{
	float Px = -h_mm * sinf(phi_rad);
	float cos_tb = cosf(theta_b);
	dm4310_set_feedforward_tau(m1, -Px * EST_FOOT_FORCE_N / 1000.0f);
	dm4310_set_feedforward_tau(m2, -LK_L2_MM * cos_tb * EST_FOOT_FORCE_N / 1000.0f);
}

static bool joint_within_limits(float ta, float tb)
{
	if (ta < LK_THETA_A_MIN + JOINT_MARGIN_RAD || ta > LK_THETA_A_MAX - JOINT_MARGIN_RAD)
		return false;
	if (tb < LK_THETA_B_MIN + JOINT_MARGIN_RAD || tb > LK_THETA_B_MAX - JOINT_MARGIN_RAD)
		return false;
	return true;
}

/*
 * 弹性 IK: 尝试两个 elbow 分支，只保留关节限位内的解，
 * 选离 prev 最近的分支（不用 wrap，直接比绝对差）。
 */
static lk_error_t ik_safe(float h, float phi, float prev_ta, float prev_tb,
			  float *out_ta, float *out_tb)
{
	float ta_up, tb_up, ta_dn, tb_dn;
	bool up_ok = false, dn_ok = false;

	if (lk_inverse(h, phi, +1, NULL, &ta_up, &tb_up) == LK_OK &&
	    joint_within_limits(ta_up, tb_up))
		up_ok = true;

	if (lk_inverse(h, phi, -1, NULL, &ta_dn, &tb_dn) == LK_OK &&
	    joint_within_limits(ta_dn, tb_dn))
		dn_ok = true;

	if (!up_ok && !dn_ok)
		return LK_ERR_UNREACHABLE;

	if (up_ok && !dn_ok) {
		*out_ta = ta_up; *out_tb = tb_up;
		return LK_OK;
	}
	if (dn_ok && !up_ok) {
		*out_ta = ta_dn; *out_tb = tb_dn;
		return LK_OK;
	}

	/* 两个都合规 → 选离 prev 更近的 (绝对值距离, 不 wrap) */
	float d_up = fabsf(ta_up - prev_ta) + fabsf(tb_up - prev_tb);
	float d_dn = fabsf(ta_dn - prev_ta) + fabsf(tb_dn - prev_tb);

	if (d_up <= d_dn) {
		*out_ta = ta_up; *out_tb = tb_up;
	} else {
		*out_ta = ta_dn; *out_tb = tb_dn;
	}
	return LK_OK;
}

int leg_move_to_left(float h_mm, float phi_rad)
{
	float ta, tb;
	float h_clamped = h_mm;

	if (h_clamped < LK_H_SAFE_MIN) {
		static uint32_t warn_cnt;
		if ((warn_cnt++ % 500) == 0) {
			printk("WARNING: h=%.1f < %.0fmm, clamping\n",
			       (double)h_mm, (double)LK_H_SAFE_MIN);
		}
		h_clamped = LK_H_SAFE_MIN;
	}

	if (ik_safe(h_clamped, phi_rad, prev_ta_left, prev_tb_left, &ta, &tb) != LK_OK) {
		printk("IK FAIL L: h=%.1f phi=%.2f\n",
		       (double)h_clamped, (double)phi_rad);
		return -1;
	}

	prev_ta_left = ta;
	prev_tb_left = tb;

	float motor_a = lk_theta_a_to_m1(ta);
	float motor_b = lk_theta_b_to_m2(tb);

	float prev_a = dm4310_get_hold_pos(1);
	float prev_b = dm4310_get_hold_pos(2);
	/* 同比例缩放: 防止独立截断破坏 A/B 运动比例 → 末端走弧线 */
	float da = motor_a - prev_a;
	float db = motor_b - prev_b;
	float m = fmaxf(fabsf(da), fabsf(db));
	if (m > MOTOR_DELTA_LIMIT_RAD) {
		float s = MOTOR_DELTA_LIMIT_RAD / m;
		motor_a = prev_a + da * s;
		motor_b = prev_b + db * s;
	}

	compute_feedforward(h_clamped, phi_rad, ta, tb, 1, 2);
	dm4310_set_pos_with_offset(1, motor_a);
	dm4310_set_pos_with_offset(2, motor_b);
	telemetry_record_cmd(h_clamped, phi_rad);
	return 0;
}

int leg_move_to_right(float h_mm, float phi_rad)
{
	float ta, tb;
	float h_clamped = h_mm;

	if (h_clamped < LK_H_SAFE_MIN) {
		static uint32_t warn_cnt;
		if ((warn_cnt++ % 500) == 0) {
			printk("WARNING: h=%.1f < %dmm, clamping\n",
			       (double)h_mm, (int)LK_H_SAFE_MIN);
		}
		h_clamped = LK_H_SAFE_MIN;
	}

	/*
	 * 双腿 mechanism frame 的 +X 都指向身体内侧，脚在中间 → Px 同号。
	 * 不需要镜像 φ，直接用同一个 phi_rad。
	 */
	if (ik_safe(h_clamped, phi_rad, prev_ta_right, prev_tb_right, &ta, &tb) != LK_OK) {
		printk("IK FAIL R: h=%.1f phi=%.2f\n",
		       (double)h_clamped, (double)phi_rad);
		return -1;
	}

	prev_ta_right = ta;
	prev_tb_right = tb;

	float motor_a = lk_theta_a_to_m3(ta);
	float motor_b = lk_theta_b_to_m4(tb);

	float prev_a = dm4310_get_hold_pos(3);
	float prev_b = dm4310_get_hold_pos(4);
	float da = motor_a - prev_a;
	float db = motor_b - prev_b;
	float m = fmaxf(fabsf(da), fabsf(db));
	if (m > MOTOR_DELTA_LIMIT_RAD) {
		float s = MOTOR_DELTA_LIMIT_RAD / m;
		motor_a = prev_a + da * s;
		motor_b = prev_b + db * s;
	}

	compute_feedforward(h_clamped, phi_rad, ta, tb, 3, 4);
	dm4310_set_pos_with_offset(3, motor_a);
	dm4310_set_pos_with_offset(4, motor_b);
	return 0;
}

int leg_move_all(float h_mm, float phi_rad)
{
	int ret_l = leg_move_to_left(h_mm, phi_rad);
	int ret_r = leg_move_to_right(h_mm, phi_rad);

	static uint32_t dbg_cnt;
	if ((dbg_cnt++ % 50) == 0) {
		printk("LEG: h=%.1f phi=%.1f° | tgt M1=%.4f M2=%.4f M3=%.4f M4=%.4f\n",
		       (double)h_mm, (double)(phi_rad*180/M_PI),
		       (double)dm4310_get_hold_pos(1), (double)dm4310_get_hold_pos(2),
		       (double)dm4310_get_hold_pos(3), (double)dm4310_get_hold_pos(4));
	}

	return (ret_l != 0) ? ret_l : ret_r;
}

/* 直接设置腿位置 (无增量限制, Shell 调试用) */
int leg_set_left(float h_mm, float phi_rad)
{
	float ta, tb;
	float h_clamped = (h_mm < LK_H_SAFE_MIN) ? LK_H_SAFE_MIN : h_mm;

	if (ik_safe(h_clamped, phi_rad, prev_ta_left, prev_tb_left, &ta, &tb) != LK_OK)
		return -1;

	prev_ta_left = ta;
	prev_tb_left = tb;

	float motor_a = lk_theta_a_to_m1(ta);
	float motor_b = lk_theta_b_to_m2(tb);

	compute_feedforward(h_clamped, phi_rad, ta, tb, 1, 2);
	dm4310_set_pos_with_offset(1, motor_a);
	dm4310_set_pos_with_offset(2, motor_b);
	return 0;
}

int leg_set_right(float h_mm, float phi_rad)
{
	float ta, tb;
	float h_clamped = (h_mm < LK_H_SAFE_MIN) ? LK_H_SAFE_MIN : h_mm;

	if (ik_safe(h_clamped, phi_rad, prev_ta_right, prev_tb_right, &ta, &tb) != LK_OK)
		return -1;

	prev_ta_right = ta;
	prev_tb_right = tb;

	float motor_a = lk_theta_a_to_m3(ta);
	float motor_b = lk_theta_b_to_m4(tb);

	compute_feedforward(h_clamped, phi_rad, ta, tb, 3, 4);
	dm4310_set_pos_with_offset(3, motor_a);
	dm4310_set_pos_with_offset(4, motor_b);
	return 0;
}

void leg_init_prev_left(float ta, float tb)
{
	prev_ta_left = ta;
	prev_tb_left = tb;
}

void leg_init_prev_right(float ta, float tb)
{
	prev_ta_right = ta;
	prev_tb_right = tb;
}

/* 诊断: 只算不跑, 打印全链路 */
void leg_diag(float h_mm, float phi_rad)
{
	float h = (h_mm < LK_H_SAFE_MIN) ? LK_H_SAFE_MIN : h_mm;
	float ta_L, tb_L, ta_R, tb_R;

	/* 当前 FK */
	float cur_ta_L = lk_m1_to_theta_a(g_dm4310.motor[0].pos_rad);
	float cur_tb_L = lk_m2_to_theta_b(g_dm4310.motor[1].pos_rad);
	float cur_ta_R = lk_m3_to_theta_a(g_dm4310.motor[2].pos_rad);
	float cur_tb_R = lk_m4_to_theta_b(g_dm4310.motor[3].pos_rad);

	printk("=== DIAG h=%.1f phi=%.1f° ===\n",
	       (double)h_mm, (double)(phi_rad * 180.0 / M_PI));
	printk("Current FK:\n");
	printk("  L: ta=%.1f° tb=%.1f°  |  R: ta=%.1f° tb=%.1f°\n",
	       (double)(cur_ta_L * 180.0/M_PI), (double)(cur_tb_L * 180.0/M_PI),
	       (double)(cur_ta_R * 180.0/M_PI), (double)(cur_tb_R * 180.0/M_PI));
	printk("  L: ab=%.1f°  |  R: ab=%.1f°\n",
	       (double)((cur_tb_L-cur_ta_L)*180.0/M_PI),
	       (double)((cur_tb_R-cur_ta_R)*180.0/M_PI));
	printk("  M1=%.4f M2=%.4f | M3=%.4f M4=%.4f (motor rad)\n",
	       (double)g_dm4310.motor[0].pos_rad, (double)g_dm4310.motor[1].pos_rad,
	       (double)g_dm4310.motor[2].pos_rad, (double)g_dm4310.motor[3].pos_rad);

	/* IK: 左腿 */
	printk("IK Left:\n");
	float ta_up, tb_up, ta_dn, tb_dn;
	lk_error_t e_up = lk_inverse(h, phi_rad, +1, NULL, &ta_up, &tb_up);
	lk_error_t e_dn = lk_inverse(h, phi_rad, -1, NULL, &ta_dn, &tb_dn);
	printk("  up(+1): e=%d ta=%.1f° tb=%.1f° lim=%c\n",
	       e_up, (double)(ta_up*180/M_PI), (double)(tb_up*180/M_PI),
	       joint_within_limits(ta_up, tb_up) ? 'Y' : 'N');
	printk("  dn(-1): e=%d ta=%.1f° tb=%.1f° lim=%c\n",
	       e_dn, (double)(ta_dn*180/M_PI), (double)(tb_dn*180/M_PI),
	       joint_within_limits(ta_dn, tb_dn) ? 'Y' : 'N');

	lk_error_t eL = ik_safe(h, phi_rad, prev_ta_left, prev_tb_left, &ta_L, &tb_L);
	if (eL == LK_OK) {
		float m1 = lk_theta_a_to_m1(ta_L);
		float m2 = lk_theta_b_to_m2(tb_L);
		printk("  -> sel: ta=%.1f° tb=%.1f° ab=%.1f°\n",
		       (double)(ta_L*180/M_PI), (double)(tb_L*180/M_PI),
		       (double)((tb_L-ta_L)*180/M_PI));
		printk("  -> M1=%.4f M2=%.4f (motor rad)\n",
		       (double)m1, (double)m2);
	} else {
		printk("  -> IK FAIL: %d\n", eL);
	}

	/* IK: 右腿 */
	printk("IK Right:\n");
	e_up = lk_inverse(h, phi_rad, +1, NULL, &ta_up, &tb_up);
	e_dn = lk_inverse(h, phi_rad, -1, NULL, &ta_dn, &tb_dn);
	printk("  up(+1): e=%d ta=%.1f° tb=%.1f° lim=%c\n",
	       e_up, (double)(ta_up*180/M_PI), (double)(tb_up*180/M_PI),
	       joint_within_limits(ta_up, tb_up) ? 'Y' : 'N');
	printk("  dn(-1): e=%d ta=%.1f° tb=%.1f° lim=%c\n",
	       e_dn, (double)(ta_dn*180/M_PI), (double)(tb_dn*180/M_PI),
	       joint_within_limits(ta_dn, tb_dn) ? 'Y' : 'N');

	lk_error_t eR = ik_safe(h, phi_rad, prev_ta_right, prev_tb_right, &ta_R, &tb_R);
	if (eR == LK_OK) {
		float m3 = lk_theta_a_to_m3(ta_R);
		float m4 = lk_theta_b_to_m4(tb_R);
		printk("  -> sel: ta=%.1f° tb=%.1f° ab=%.1f°\n",
		       (double)(ta_R*180/M_PI), (double)(tb_R*180/M_PI),
		       (double)((tb_R-ta_R)*180/M_PI));
		printk("  -> M3=%.4f M4=%.4f (motor rad)\n",
		       (double)m3, (double)m4);
	} else {
		printk("  -> IK FAIL: %d\n", eR);
	}
	printk("=== END DIAG ===\n");
}

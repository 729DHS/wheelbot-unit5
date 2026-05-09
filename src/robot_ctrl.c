/**
 * @file robot_ctrl.c
 * @brief 轨迹插值 (梯形加减速) + jog 微调 + 堵转保护
 *
 * 500Hz 控制循环内执行:
 *   1. 梯形加减速轨迹 (加速→匀速→减速)
 *   2. 调 leg_move_all() 下发目标
 *   3. 堵转/碰撞检测 (100ms debounce → 自动 stop)
 *
 * 加减速算法:
 *   每 tick 根据剩余距离计算目标速度 v_target = sign(d) * min(v_max, sqrt(2*a*|d|))
 *   速度变化率限制在 ±a*dt 内, 自然形成梯形轮廓。
 */

#include "robot_ctrl.h"
#include "leg_control.h"
#include "linkage_kinematics.h"
#include "dm4310_motor.h"

#include <math.h>
#include <zephyr/sys/printk.h>

struct robot_ctrl_state g_robot;

/* 控制周期 2ms */
#define TICK_S  0.002f

/* ================================================================
 *  梯形加减速轨迹
 * ================================================================ */

/** @brief 单轴加速度限制步进
 *  @param cur      当前位置
 *  @param target   目标位置
 *  @param vel      当前速度 (in/out)
 *  @param v_max    最大速度 (正数)
 *  @param a_max    最大加速度 (正数)
 *  @return 新位置
 */
static float accel_limited_step(float cur, float target,
				float *vel, float v_max, float a_max)
{
	float dist = target - cur;

	if (fabsf(dist) < 1e-6f) {
		*vel = 0.0f;
		return target;
	}

	int sign = (dist > 0.0f) ? 1 : -1;

	/* 目标速度: min(匀速上限, 按 a_max 能在剩余距离内停下) */
	float v_stop = sqrtf(2.0f * a_max * fabsf(dist));
	float v_target = (v_stop < v_max) ? v_stop : v_max;
	v_target *= (float)sign;

	/* 速度变化率限制 */
	float a_step = a_max * TICK_S;
	float dv = v_target - *vel;
	if (dv >  a_step) dv =  a_step;
	if (dv < -a_step) dv = -a_step;

	*vel += dv;
	return cur + *vel * TICK_S;
}

static void start_trajectory(float h_from, float phi_from,
			     float h_to, float phi_to)
{
	g_robot.traj_h_target = h_to;
	g_robot.traj_phi_target = phi_to;
	g_robot.traj_h_current = h_from;
	g_robot.traj_phi_current = phi_from;
	g_robot.traj_h_vel = 0.0f;
	g_robot.traj_phi_vel = 0.0f;
	g_robot.traj_active = true;

	printk("TRAJ START: h %.1f→%.1f mm  phi %.1f→%.1f deg  "
	       "(v_max=%.0fmm/s a=%.0fmm/s²)\n",
	       (double)h_from, (double)h_to,
	       (double)phi_from, (double)phi_to,
	       (double)TRAJ_H_SPEED_MM_PER_S,
	       (double)TRAJ_H_ACCEL_MM_PER_S2);
}

/** @brief 梯形加减速一步插值 */
static void trajectory_step(void)
{
	float h = accel_limited_step(g_robot.traj_h_current,
				      g_robot.traj_h_target,
				      &g_robot.traj_h_vel,
				      TRAJ_H_SPEED_MM_PER_S,
				      TRAJ_H_ACCEL_MM_PER_S2);

	float phi = accel_limited_step(g_robot.traj_phi_current,
					g_robot.traj_phi_target,
					&g_robot.traj_phi_vel,
					TRAJ_PHI_SPEED_DEG_PER_S,
					TRAJ_PHI_ACCEL_DEG_PER_S2);

	g_robot.traj_h_current = h;
	g_robot.traj_phi_current = phi;

	/* 下发当前插值位置 */
	int ret = leg_move_all(h, phi * 3.1415926535f / 180.0f);

	if (ret != 0) {
		printk("TRAJ leg_move_all FAIL: h=%.1f phi=%.1f ret=%d -> STOP\n",
		       (double)h, (double)phi, ret);
		robot_ctrl_stop();
		return;
	}

	/* 检查到达: 位置逼近且速度为零 */
	bool h_done = (fabsf(g_robot.traj_h_target - h) < 0.5f)
		      && (fabsf(g_robot.traj_h_vel) < 1.0f);
	bool phi_done = (fabsf(g_robot.traj_phi_target - phi) < 0.1f)
			&& (fabsf(g_robot.traj_phi_vel) < 0.2f);

	if (h_done && phi_done) {
		g_robot.traj_active = false;
		g_robot.traj_h_vel = 0.0f;
		g_robot.traj_phi_vel = 0.0f;
		printk("TRAJ DONE: h=%.1f phi=%.1f\n",
		       (double)g_robot.traj_h_target,
		       (double)g_robot.traj_phi_target);
	}
}

/* ================================================================
 *  堵转保护
 * ================================================================ */

static void stall_check(void)
{
	bool cond = false;

	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		float pos_err = fabsf(g_dm4310.hold_pos_rad[i] -
				      g_dm4310.motor[i].pos_rad);
		float vel = fabsf(g_dm4310.motor[i].vel_radps);

		if (pos_err > STALL_POS_ERR_RAD && vel < STALL_VEL_THRESHOLD) {
			cond = true;
		}

		if (fabsf(g_dm4310.motor[i].torque_nm) > STALL_TORQUE_NM) {
			cond = true;
		}
	}

	float t[DM4310_MOTOR_COUNT];
	float t_max = 0.0f, t_sum = 0.0f;
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		t[i] = fabsf(g_dm4310.motor[i].torque_nm);
		t_sum += t[i];
		if (t[i] > t_max) { t_max = t[i]; }
	}
	float t_avg_others = (t_sum - t_max) / 3.0f;
	if (t_max > STALL_TORQUE_RATIO * t_avg_others && t_max > 1.0f) {
		cond = true;
	}

	if (cond) {
		g_robot.stall_counter++;
		if (g_robot.stall_counter >= STALL_DEBOUNCE_TICKS) {
			printk("!!! STALL PROTECTION: auto stop !!!\n");
			printk("  M1: err=%.3f tq=%.3f | M2: err=%.3f tq=%.3f\n",
			       (double)fabsf(g_dm4310.hold_pos_rad[0] - g_dm4310.motor[0].pos_rad),
			       (double)g_dm4310.motor[0].torque_nm,
			       (double)fabsf(g_dm4310.hold_pos_rad[1] - g_dm4310.motor[1].pos_rad),
			       (double)g_dm4310.motor[1].torque_nm);
			printk("  M3: err=%.3f tq=%.3f | M4: err=%.3f tq=%.3f\n",
			       (double)fabsf(g_dm4310.hold_pos_rad[2] - g_dm4310.motor[2].pos_rad),
			       (double)g_dm4310.motor[2].torque_nm,
			       (double)fabsf(g_dm4310.hold_pos_rad[3] - g_dm4310.motor[3].pos_rad),
			       (double)g_dm4310.motor[3].torque_nm);
			printk("  h_tgt=%.1f h_cur=%.1f phi_tgt=%.1f phi_cur=%.1f\n",
			       (double)g_robot.traj_h_target, (double)g_robot.traj_h_current,
			       (double)g_robot.traj_phi_target, (double)g_robot.traj_phi_current);
			robot_ctrl_stop();
			g_robot.stall_triggered = true;
		}
	} else {
		g_robot.stall_counter = 0;
	}
}

/* ================================================================
 *  公开接口
 * ================================================================ */

void robot_ctrl_tick(void)
{
	if (g_robot.traj_active) {
		trajectory_step();
		stall_check();
	}
}

int robot_ctrl_move_to(float h_mm, float phi_deg)
{
	if (!g_dm4310.bringup_done) {
		return -1;
	}

	start_trajectory(g_robot.traj_h_current, g_robot.traj_phi_current,
			 h_mm, phi_deg);

	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		g_dm4310.hold_kp[i] = TRAJ_KP;
		g_dm4310.hold_kd[i] = TRAJ_KD;
		g_dm4310.feedforward_tau[i] = 0.0f;
	}

	g_robot.stall_triggered = false;
	g_robot.stall_counter = 0;
	leg_move_all(g_robot.traj_h_current,
		     g_robot.traj_phi_current * 3.1415926535f / 180.0f);
	return 0;
}

int robot_ctrl_jog_h(float delta_mm)
{
	float new_target = g_robot.traj_h_current + delta_mm;

	if (new_target < ROBOT_H_USER_MIN_MM || new_target > ROBOT_H_USER_MAX_MM) {
		return -1;
	}

	if (g_robot.traj_active) {
		g_robot.traj_h_target = new_target;
	} else {
		start_trajectory(g_robot.traj_h_current, g_robot.traj_phi_current,
				 new_target, g_robot.traj_phi_current);
		for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
			g_dm4310.hold_kp[i] = TRAJ_KP;
			g_dm4310.hold_kd[i] = TRAJ_KD;
		}
		g_robot.stall_triggered = false;
		g_robot.stall_counter = 0;
	}
	return 0;
}

int robot_ctrl_jog_phi(float delta_deg)
{
	float new_target = g_robot.traj_phi_current + delta_deg;

	if (new_target < -ROBOT_PHI_USER_MAX_DEG || new_target > ROBOT_PHI_USER_MAX_DEG) {
		return -1;
	}

	if (g_robot.traj_active) {
		g_robot.traj_phi_target = new_target;
	} else {
		start_trajectory(g_robot.traj_h_current, g_robot.traj_phi_current,
				 g_robot.traj_h_current, new_target);
		for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
			g_dm4310.hold_kp[i] = TRAJ_KP;
			g_dm4310.hold_kd[i] = TRAJ_KD;
		}
		g_robot.stall_triggered = false;
		g_robot.stall_counter = 0;
	}
	return 0;
}

void robot_ctrl_stop(void)
{
	g_robot.traj_active = false;
	g_robot.traj_h_vel = 0.0f;
	g_robot.traj_phi_vel = 0.0f;
	g_robot.stall_counter = 0;

	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		g_dm4310.hold_kp[i] = 0.01f;
		g_dm4310.hold_kd[i] = 0.001f;
		g_dm4310.feedforward_tau[i] = 0.0f;
		g_dm4310.hold_pos_rad[i] = g_dm4310.motor[i].pos_rad;
	}
	g_dm4310.hold_updates = 1U;
	g_dm4310.balance_ramp_remaining = 0U;
}

/**
 * @file robot_ctrl.c
 * @brief 轨迹插值 + jog 微调 + 堵转保护
 *
 * 500Hz 控制循环内执行:
 *   1. 轨迹插值 (h ≤10mm/s, phi ≤2deg/s)
 *   2. 调 leg_move_all() 下发目标
 *   3. 堵转/碰撞检测 (100ms debounce → 自动 stop)
 */
#include "robot_ctrl.h"
#include "leg_control.h"
#include "dm4310_motor.h"

#include <math.h>
#include <zephyr/sys/printk.h>

struct robot_ctrl_state g_robot;

/* 控制周期 2ms */
#define TICK_S  0.002f

static void start_trajectory(float h_from, float phi_from,
			     float h_to, float phi_to)
{
	float dh = h_to - h_from;
	float dphi = phi_to - phi_from;

	g_robot.traj_h_target = h_to;
	g_robot.traj_phi_target = phi_to;
	g_robot.traj_h_current = h_from;
	g_robot.traj_phi_current = phi_from;

	/* 每 tick 步长 (带符号方向) */
	float max_h_step = TRAJ_H_SPEED_MM_PER_S * TICK_S;
	float max_phi_step = TRAJ_PHI_SPEED_DEG_PER_S * TICK_S;

	g_robot.traj_h_step_per_tick = (dh > 0 ? 1.0f : -1.0f) * max_h_step;
	g_robot.traj_phi_step_per_tick = (dphi > 0 ? 1.0f : -1.0f) * max_phi_step;

	g_robot.traj_active = true;
}

/* 一步插值: 向目标逼近, 到达后清零 traj_active */
static void trajectory_step(void)
{
	float h = g_robot.traj_h_current;
	float phi = g_robot.traj_phi_current;
	bool h_done = false, phi_done = false;

	if (fabsf(g_robot.traj_h_target - h) <= fabsf(g_robot.traj_h_step_per_tick)) {
		h = g_robot.traj_h_target;
		h_done = true;
	} else {
		h += g_robot.traj_h_step_per_tick;
	}

	if (fabsf(g_robot.traj_phi_target - phi) <= fabsf(g_robot.traj_phi_step_per_tick)) {
		phi = g_robot.traj_phi_target;
		phi_done = true;
	} else {
		phi += g_robot.traj_phi_step_per_tick;
	}

	g_robot.traj_h_current = h;
	g_robot.traj_phi_current = phi;

	/* 下发当前插值位置 */
	leg_move_all(h, phi * 3.1415926535f / 180.0f);

	if (h_done && phi_done) {
		g_robot.traj_active = false;
	}
}

/* 堵转/碰撞检测 (每 tick) */
static void stall_check(void)
{
	bool cond = false;

	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		float pos_err = fabsf(g_dm4310.hold_pos_rad[i] -
				      g_dm4310.motor[i].pos_rad);
		float vel = fabsf(g_dm4310.motor[i].vel_radps);

		/* 条件 1: 位置误差大 + 电机几乎不动 → 堵转 */
		if (pos_err > STALL_POS_ERR_RAD && vel < STALL_VEL_THRESHOLD) {
			cond = true;
		}

		/* 条件 2: 力矩超安全阈值 */
		if (fabsf(g_dm4310.motor[i].torque_nm) > STALL_TORQUE_NM) {
			cond = true;
		}
	}

	/* 条件 3: 单电机力矩明显大于其他三台 */
	float t[DM4310_MOTOR_COUNT];
	float t_max = 0.0f, t_sum = 0.0f;
	int max_i = 0;
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		t[i] = fabsf(g_dm4310.motor[i].torque_nm);
		t_sum += t[i];
		if (t[i] > t_max) { t_max = t[i]; max_i = i; }
	}
	float t_avg_others = (t_sum - t_max) / 3.0f;
	if (t_max > STALL_TORQUE_RATIO * t_avg_others && t_max > 1.0f) {
		cond = true;
	}

	if (cond) {
		g_robot.stall_counter++;
		if (g_robot.stall_counter >= STALL_DEBOUNCE_TICKS) {
			printk("!!! STALL PROTECTION: auto stop !!!\n");
			robot_ctrl_stop();
			g_robot.stall_triggered = true;
		}
	} else {
		g_robot.stall_counter = 0;
	}
}

void robot_ctrl_tick(void)
{
	if (g_robot.traj_active) {
		trajectory_step();
	}

	/* 堵转检测仅轨迹运动时生效 (拖拽模式手推自然产生误差, 无误触发) */
	if (g_robot.traj_active) {
		stall_check();
	}
}

int robot_ctrl_move_to(float h_mm, float phi_deg)
{
	if (!g_dm4310.bringup_done) {
		return -1;
	}

	/* 从当前位置开始插值 */
	start_trajectory(g_robot.traj_h_current, g_robot.traj_phi_current,
			 h_mm, phi_deg);

	/* 设轨迹模式增益 */
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

	if (new_target < 120.0f || new_target > 200.0f) {
		return -1;
	}

	if (g_robot.traj_active) {
		/* 更新目标, 轨迹继续 */
		g_robot.traj_h_target = new_target;
		float dh = new_target - g_robot.traj_h_current;
		float max_step = TRAJ_H_SPEED_MM_PER_S * TICK_S;
		g_robot.traj_h_step_per_tick = (dh > 0 ? 1.0f : -1.0f) * max_step;
	} else {
		/* 从当前位置启动新轨迹 */
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

	if (new_target < -8.0f || new_target > 8.0f) {
		return -1;
	}

	if (g_robot.traj_active) {
		g_robot.traj_phi_target = new_target;
		float dphi = new_target - g_robot.traj_phi_current;
		float max_step = TRAJ_PHI_SPEED_DEG_PER_S * TICK_S;
		g_robot.traj_phi_step_per_tick = (dphi > 0 ? 1.0f : -1.0f) * max_step;
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

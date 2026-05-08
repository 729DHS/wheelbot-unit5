/**
 * @file robot_ctrl.h
 * @brief 机器人控制状态机 — 轨迹插值、jog 微调、堵转保护
 *
 * 轨迹限速: h ≤ 10mm/s, phi ≤ 2deg/s
 * 控制周期 500Hz (2ms), 每 tick 插值一步。
 * 堵转保护: 位置误差/力矩/单电机异常 → 100ms debounce → 自动 stop。
 */
#ifndef ROBOT_CTRL_H_
#define ROBOT_CTRL_H_

#include <stdbool.h>
#include <stdint.h>

/* 轨迹限速 */
#define TRAJ_H_SPEED_MM_PER_S      10.0f
#define TRAJ_PHI_SPEED_DEG_PER_S   2.0f

/* 轨迹模式位置增益 (中等刚度, 避免抖动) */
#define TRAJ_KP  20.0f
#define TRAJ_KD  0.5f

/* 堵转保护阈值 (bring-up 保守值) */
#define STALL_POS_ERR_RAD      0.15f
#define STALL_VEL_THRESHOLD    0.05f
#define STALL_TORQUE_NM        3.0f
#define STALL_DEBOUNCE_TICKS   50
#define STALL_TORQUE_RATIO     3.0f

struct robot_ctrl_state {
	bool traj_active;
	float traj_h_target;          /* 最终目标 h (mm) */
	float traj_phi_target;        /* 最终目标 phi (deg) */
	float traj_h_current;         /* 当前插值 h */
	float traj_phi_current;       /* 当前插值 phi */
	float traj_h_step_per_tick;   /* h 步长 (mm/tick, 带符号) */
	float traj_phi_step_per_tick; /* phi 步长 (deg/tick, 带符号) */

	uint16_t stall_counter;       /* 堵转 debounce 计数 */
	bool stall_triggered;         /* 堵转保护已触发 */
};

extern struct robot_ctrl_state g_robot;

/**
 * @brief 每控制 tick 调用 (500Hz)
 *
 * 轨迹插值 → leg_move_all → 堵转检测。
 * 主循环 bringup_done 后调用，替代 dm4310_hold_positions。
 */
void robot_ctrl_tick(void);

/* Shell 接口 */
int  robot_ctrl_move_to(float h_mm, float phi_deg);
int  robot_ctrl_jog_h(float delta_mm);
int  robot_ctrl_jog_phi(float delta_deg);
void robot_ctrl_stop(void);

#endif /* ROBOT_CTRL_H_ */

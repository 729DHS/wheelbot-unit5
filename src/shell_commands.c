	/* robot cali — 一键标定: re-ZERO 电机 + 记录位姿

/**
 * @file shell_commands.c
 * @brief 串口 Shell 电机调试命令
 *
 * 通过 USART6 串口 (uart:~$ 提示符) 输入命令控制电机:
 *   motor enable <1-4|all>   使能电机
 *   motor disable <1-4|all>  失能电机
 *   motor zero <1-4|all>     设置零点
 *   motor status             查看状态
 *   motor kp <1-4> <value>   设置 KP
 *   motor kd <1-4> <value>   设置 KD
 */

#include "dm4310_motor.h"
#include "motor_debug.h"
#include "leg_control.h"
#include "robot_ctrl.h"
#include "linkage_kinematics.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

/* Shell 实例指针 (供 main.c CSV 输出使用) */
const struct shell *g_motor_shell;
/* CSV 流开关 */
bool g_csv_enabled;

/* limit_scan 状态 (robot limit_scan 用) */
struct limit_scan_s {
	bool init;
	float min_m[4], max_m[4];
	float min_ta[2], max_ta[2];
	float min_tb[2], max_tb[2];
	float min_h[2], max_h[2];
	float min_phi[2], max_phi[2];
};
static struct limit_scan_s g_scan;

static float dm4310_motor_calibrated_pos(int idx)
{
	return g_dm4310.motor[idx].pos_rad - g_dm_offset[idx];
}

static void scan_update(struct limit_scan_s *s)
{
	float m[4], ta[2], tb[2], h[2], phi[2];
	for (int i = 0; i < 4; i++) m[i] = dm4310_motor_calibrated_pos(i);
	ta[0] = lk_m1_to_theta_a(m[0]); tb[0] = lk_m2_to_theta_b(m[1]);
	ta[1] = lk_m3_to_theta_a(m[2]); tb[1] = lk_m4_to_theta_b(m[3]);
	lk_forward(ta[0], tb[0], NULL, &h[0], &phi[0]);
	lk_forward(ta[1], tb[1], NULL, &h[1], &phi[1]);

	if (!s->init) {
		s->init = true;
		for (int i = 0; i < 4; i++) s->min_m[i] = s->max_m[i] = m[i];
		for (int i = 0; i < 2; i++) {
			s->min_ta[i] = s->max_ta[i] = ta[i];
			s->min_tb[i] = s->max_tb[i] = tb[i];
			s->min_h[i]  = s->max_h[i]  = h[i];
			s->min_phi[i]= s->max_phi[i]= phi[i];
		}
		return;
	}

	for (int i = 0; i < 4; i++) {
		if (m[i] < s->min_m[i]) s->min_m[i] = m[i];
		if (m[i] > s->max_m[i]) s->max_m[i] = m[i];
	}
	for (int i = 0; i < 2; i++) {
		if (ta[i] < s->min_ta[i]) s->min_ta[i] = ta[i];
		if (ta[i] > s->max_ta[i]) s->max_ta[i] = ta[i];
		if (tb[i] < s->min_tb[i]) s->min_tb[i] = tb[i];
		if (tb[i] > s->max_tb[i]) s->max_tb[i] = tb[i];
		if (h[i]  < s->min_h[i])  s->min_h[i]  = h[i];
		if (h[i]  > s->max_h[i])  s->max_h[i]  = h[i];
		if (phi[i]< s->min_phi[i]) s->min_phi[i]= phi[i];
		if (phi[i]> s->max_phi[i]) s->max_phi[i]= phi[i];
	}
}

static void scan_print(const struct shell *sh, struct limit_scan_s *s, uint32_t iter)
{
	float m[4], ta[2], tb[2], h[2], phi[2], tq[4];
	for (int i = 0; i < 4; i++) {
		m[i]  = dm4310_motor_calibrated_pos(i);
		tq[i] = g_dm4310.motor[i].torque_nm;
	}
	ta[0] = lk_m1_to_theta_a(m[0]); tb[0] = lk_m2_to_theta_b(m[1]);
	ta[1] = lk_m3_to_theta_a(m[2]); tb[1] = lk_m4_to_theta_b(m[3]);
	lk_forward(ta[0], tb[0], NULL, &h[0], &phi[0]);
	lk_forward(ta[1], tb[1], NULL, &h[1], &phi[1]);

#define D(x) (double)(x)
#define R2D(x) ((double)(x) * 180.0 / M_PI)

	shell_print(sh, "=== LIMIT SCAN #%u @ 5Hz | t=%u ms ===", iter, k_uptime_get_32());

	shell_print(sh, "Motor(rad): M1      M2      M3      M4");
	shell_print(sh, "  cur       %+7.4f %+7.4f %+7.4f %+7.4f", D(m[0]),D(m[1]),D(m[2]),D(m[3]));
	shell_print(sh, "  min       %+7.4f %+7.4f %+7.4f %+7.4f", D(s->min_m[0]),D(s->min_m[1]),D(s->min_m[2]),D(s->min_m[3]));
	shell_print(sh, "  max       %+7.4f %+7.4f %+7.4f %+7.4f", D(s->max_m[0]),D(s->max_m[1]),D(s->max_m[2]),D(s->max_m[3]));

	shell_print(sh, "Theta(deg): θa_L    θb_L    θa_R    θb_R");
	shell_print(sh, "  cur        %+7.2f %+7.2f %+7.2f %+7.2f", R2D(ta[0]),R2D(tb[0]),R2D(ta[1]),R2D(tb[1]));
	shell_print(sh, "  min        %+7.2f %+7.2f %+7.2f %+7.2f", R2D(s->min_ta[0]),R2D(s->min_tb[0]),R2D(s->min_ta[1]),R2D(s->min_tb[1]));
	shell_print(sh, "  max        %+7.2f %+7.2f %+7.2f %+7.2f", R2D(s->max_ta[0]),R2D(s->max_tb[0]),R2D(s->max_ta[1]),R2D(s->max_tb[1]));

	shell_print(sh, "Task Space:  h_L     φ_L      h_R     φ_R   (mm/deg)");
	shell_print(sh, "  cur       %+7.1f %+7.1f %+7.1f %+7.1f", D(h[0]),R2D(phi[0]),D(h[1]),R2D(phi[1]));
	shell_print(sh, "  min       %+7.1f %+7.1f %+7.1f %+7.1f", D(s->min_h[0]),R2D(s->min_phi[0]),D(s->min_h[1]),R2D(s->min_phi[1]));
	shell_print(sh, "  max       %+7.1f %+7.1f %+7.1f %+7.1f", D(s->max_h[0]),R2D(s->max_phi[0]),D(s->max_h[1]),R2D(s->max_phi[1]));

	shell_print(sh, "Torque(Nm):  t1      t2      t3      t4");
	shell_print(sh, "  cur       %+7.3f %+7.3f %+7.3f %+7.3f", D(tq[0]),D(tq[1]),D(tq[2]),D(tq[3]));

	/* 理论限位 (来自 linkage_kinematics.h) */
	shell_print(sh, "TheoryLim: h[%.0f..%.0f] φ[%.0f..%.0f] θa[%.0f..%.0f] θb[%.0f..%.0f] (mm/deg)",
		    (double)LK_H_SOFT_MIN, (double)LK_H_SOFT_MAX,
		    R2D(LK_PHI_SOFT_MIN), R2D(LK_PHI_SOFT_MAX),
		    R2D(LK_THETA_A_MIN), R2D(LK_THETA_A_MAX),
		    R2D(LK_THETA_B_MIN), R2D(LK_THETA_B_MAX));

	/* 限位检测 (含关节空间) */
	const char *hl = (h[0] <= LK_H_SOFT_MIN || h[0] >= LK_H_SOFT_MAX) ? "!!" : "OK";
	const char *pl = (phi[0] <= LK_PHI_SOFT_MIN || phi[0] >= LK_PHI_SOFT_MAX) ? "!!" : "OK";
	const char *hr = (h[1] <= LK_H_SOFT_MIN || h[1] >= LK_H_SOFT_MAX) ? "!!" : "OK";
	const char *pr = (phi[1] <= LK_PHI_SOFT_MIN || phi[1] >= LK_PHI_SOFT_MAX) ? "!!" : "OK";
	bool jt_near = (ta[0] <= LK_THETA_A_MIN || ta[0] >= LK_THETA_A_MAX ||
			ta[1] <= LK_THETA_A_MIN || ta[1] >= LK_THETA_A_MAX ||
			tb[0] <= LK_THETA_B_MIN || tb[0] >= LK_THETA_B_MAX ||
			tb[1] <= LK_THETA_B_MIN || tb[1] >= LK_THETA_B_MAX);
	shell_print(sh, "LIMIT: h_L[%s] φ_L[%s] h_R[%s] φ_R[%s] joint[%s]",
		    hl, pl, hr, pr, jt_near ? "!!" : "OK");

	bool any_tq = false;
	for (int i = 0; i < 4; i++) {
		if (fabsf(tq[i]) > 1.5f) any_tq = true;
	}
	if (any_tq) shell_print(sh, "  -> High torque detected, possible mech limit!");

#undef D
#undef R2D
}

/* motor csv <on|off|once> */
static int cmd_motor_csv(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: motor csv <on|off|once>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "on") == 0) {
		g_motor_shell = sh;
		g_csv_enabled = true;
		shell_print(sh, "t_ms,m1,m2,m3,m4,t1,t2,t3,t4,pitch,pitch_rate,dt_us");
		shell_print(sh, "CSV: on");
	} else if (strcmp(argv[1], "off") == 0) {
		g_csv_enabled = false;
		shell_print(sh, "CSV: off");
	} else if (strcmp(argv[1], "once") == 0) {
		shell_print(sh, "%u,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,0.0000,0.00,0",
			    k_uptime_get_32(),
			    (double)g_dm4310.motor[0].pos_rad,
			    (double)g_dm4310.motor[1].pos_rad,
			    (double)g_dm4310.motor[2].pos_rad,
			    (double)g_dm4310.motor[3].pos_rad,
			    (double)g_dm4310.motor[0].torque_nm,
			    (double)g_dm4310.motor[1].torque_nm,
			    (double)g_dm4310.motor[2].torque_nm,
			    (double)g_dm4310.motor[3].torque_nm);
	} else {
		shell_print(sh, "Usage: motor csv <on|off|once>");
		return -EINVAL;
	}
	return 0;
}

/* motor enable <1-4|all> */
static int cmd_motor_enable(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: motor enable <1-4|all>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "all") == 0) {
		for (uint8_t id = 1; id <= DM4310_MOTOR_COUNT; id++) {
			int ret = dm4310_enable_motor(id);
			shell_print(sh, "M%d ENABLE  ret=%d", id, ret);
		}
		return 0;
	}

	int id = atoi(argv[1]);
	if (id < 1 || id > DM4310_MOTOR_COUNT) {
		shell_print(sh, "Invalid motor ID: %d (1-%d)", id, DM4310_MOTOR_COUNT);
		return -EINVAL;
	}
	int ret = dm4310_enable_motor((uint8_t)id);
	shell_print(sh, "M%d ENABLE  ret=%d", id, ret);
	return ret;
}

/* motor disable <1-4|all> */
static int cmd_motor_disable(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: motor disable <1-4|all>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "all") == 0) {
		for (uint8_t id = 1; id <= DM4310_MOTOR_COUNT; id++) {
			int ret = dm4310_disable_motor(id);
			shell_print(sh, "M%d DISABLE ret=%d", id, ret);
		}
		return 0;
	}

	int id = atoi(argv[1]);
	if (id < 1 || id > DM4310_MOTOR_COUNT) {
		shell_print(sh, "Invalid motor ID: %d (1-%d)", id, DM4310_MOTOR_COUNT);
		return -EINVAL;
	}
	int ret = dm4310_disable_motor((uint8_t)id);
	shell_print(sh, "M%d DISABLE ret=%d", id, ret);
	return ret;
}

/* motor zero <1-4|all> */
static int cmd_motor_zero(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: motor zero <1-4|all>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "all") == 0) {
		for (uint8_t id = 1; id <= DM4310_MOTOR_COUNT; id++) {
			int ret = dm4310_zero_motor(id);
			shell_print(sh, "M%d ZERO    ret=%d", id, ret);
		}
		return 0;
	}

	int id = atoi(argv[1]);
	if (id < 1 || id > DM4310_MOTOR_COUNT) {
		shell_print(sh, "Invalid motor ID: %d (1-%d)", id, DM4310_MOTOR_COUNT);
		return -EINVAL;
	}
	int ret = dm4310_zero_motor((uint8_t)id);
	shell_print(sh, "M%d ZERO    ret=%d", id, ret);
	return ret;
}

/* motor status */
static int cmd_motor_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "online=0x%x loops=%u bringup=%s",
		    g_dm4310.online_mask, g_dm4310.loops,
		    g_dm4310.bringup_done ? "done" : "no");

	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		const volatile struct dm4310_motor_status *m = &g_dm4310.motor[i];
		shell_print(sh,
			    "  M%d: %s  pos=%+.4f rad (%+.1f°)  vel=%+.2f  "
			    "t_mos=%d t_coil=%d rx=%u kp=%.3f kd=%.3f",
			    i + 1,
			    m->online ? "ON " : "OFF",
			    (double)m->pos_rad,
			    (double)m->pos_rad * 180.0 / 3.1415926535,
			    (double)m->vel_radps,
			    m->mos_temp, m->coil_temp, m->rx_count,
			    (double)g_dm4310.hold_kp[i],
			    (double)g_dm4310.hold_kd[i]);
	}
	return 0;
}

/* motor kp <1-4> <value> */
static int cmd_motor_kp(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_print(sh, "Usage: motor kp <1-4> <value>");
		return -EINVAL;
	}

	int id = atoi(argv[1]);
	if (id < 1 || id > DM4310_MOTOR_COUNT) {
		shell_print(sh, "Invalid motor ID: %d", id);
		return -EINVAL;
	}

	float val = (float)atof(argv[2]);
	g_dm4310.hold_kp[id - 1] = val;
	g_dm4310.hold_updates = 1U;
	shell_print(sh, "M%d KP=%.3f", id, (double)val);
	return 0;
}

/* motor kd <1-4> <value> */
static int cmd_motor_kd(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_print(sh, "Usage: motor kd <1-4> <value>");
		return -EINVAL;
	}

	int id = atoi(argv[1]);
	if (id < 1 || id > DM4310_MOTOR_COUNT) {
		shell_print(sh, "Invalid motor ID: %d", id);
		return -EINVAL;
	}

	float val = (float)atof(argv[2]);
	g_dm4310.hold_kd[id - 1] = val;
	g_dm4310.hold_updates = 1U;
	shell_print(sh, "M%d KD=%.3f", id, (double)val);
	return 0;
}

/* motor torque <1-4> <Nm> — 直接设置力矩前馈 (调试/Unit6 用) */
static int cmd_motor_torque(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_print(sh, "Usage: motor torque <1-4> <Nm>");
		return -EINVAL;
	}

	int id = atoi(argv[1]);
	if (id < 1 || id > DM4310_MOTOR_COUNT) {
		shell_print(sh, "Invalid motor ID: %d", id);
		return -EINVAL;
	}

	float val = (float)atof(argv[2]);
	dm4310_set_feedforward_tau((uint8_t)id, val);
	g_dm4310.hold_updates = 1U;
	shell_print(sh, "M%d feedforward_tau=%.3f Nm", id, (double)val);
	return 0;
}

/* motor lock — 锁定当前位置 (高 KP/KD) */
static int cmd_motor_lock(const struct shell *sh, size_t argc, char **argv)
{
	float kp = 80.0f, kd = 1.5f;
	if (argc >= 2) kp = (float)atof(argv[1]);
	if (argc >= 3) kd = (float)atof(argv[2]);

	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		g_dm4310.hold_pos_rad[i] = g_dm4310.motor[i].pos_rad;
		g_dm4310.hold_kp[i] = kp;
		g_dm4310.hold_kd[i] = kd;
		g_dm4310.feedforward_tau[i] = 0.0f;
	}
	g_dm4310.hold_updates = 1U;
	shell_print(sh, "LOCKED at current pos | KP=%.1f KD=%.2f", (double)kp, (double)kd);
	shell_print(sh, "  M1=%.4f M2=%.4f M3=%.4f M4=%.4f",
		    (double)g_dm4310.motor[0].pos_rad,
		    (double)g_dm4310.motor[1].pos_rad,
		    (double)g_dm4310.motor[2].pos_rad,
		    (double)g_dm4310.motor[3].pos_rad);
	return 0;
}

/* motor unlock — 回到拖动模式 */
static int cmd_motor_unlock(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		g_dm4310.hold_pos_rad[i] = g_dm4310.motor[i].pos_rad;
		g_dm4310.hold_kp[i] = 0.01f;
		g_dm4310.hold_kd[i] = 0.001f;
		g_dm4310.feedforward_tau[i] = 0.0f;
	}
	g_dm4310.hold_updates = 1U;
	shell_print(sh, "UNLOCKED → drag mode (KP=0.01 KD=0.001)");
	return 0;
}

/* robot cali — 一键软件标定: 记录当前原始编码器位置为固件零点
 * 标定位姿: θa = LK_OFFSET_A = -162.4°, θb = LK_OFFSET_B = -10.0°
 * 不向电机发送 ZERO(0xFE), 避免覆盖 DM4310 内部绝对零点。*/
static int cmd_robot_cali(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* 1. 获取当前原始电机角, 作为固件软件零点 */
	float raw_m[4];
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		raw_m[i] = g_dm4310.motor[i].pos_rad;
	}

	/* 2. Re-ZERO 四台电机 (当前位置 → encoder=0) */
	for (uint8_t id = 1; id <= DM4310_MOTOR_COUNT; id++) {
		int ret = dm4310_zero_motor(id);
		shell_print(sh, "M%d ZERO  ret=%d", id, ret);
	}

	/* 3. 清零 g_dm_offset + 同步 hold_pos_rad */
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		g_dm_offset[i] = 0.0f;
		g_dm4310.hold_pos_rad[i] = 0.0f; /* ZERO后encoder归零 */
	}
	g_dm4310.hold_updates = 1U;

	/*
	 * 4. FK from post-ZERO (motor=0 → θa=OFFSET_A, θb=OFFSET_B)。
	 *    轨迹起点必须和 robot raw 的 FK 一致，不用 pre-ZERO 的 raw_m。
	 */
	float ta0 = LK_OFFSET_A;
	float tb0 = LK_OFFSET_B;
	float h0, phi0;
	lk_forward(ta0, tb0, NULL, &h0, &phi0);
	g_robot.traj_h_current = h0;
	g_robot.traj_phi_current = phi0 * 180.0f / M_PI;

	/* 初始化 IK 分支追踪 (机构帧 θa/θb) */
	leg_init_prev_left(ta0, tb0);
	leg_init_prev_right(ta0, tb0);

	shell_print(sh, "Software calibration done. No DM4310 ZERO command sent.");
	shell_print(sh, "Raw encoder zero offset: M1=%.4f M2=%.4f M3=%.4f M4=%.4f rad",
		    (double)raw_m[0], (double)raw_m[1],
		    (double)raw_m[2], (double)raw_m[3]);
	shell_print(sh, "Task space (FK from ZERO): h0=%.1f mm phi0=%.1f deg",
		    (double)h0, (double)(phi0 * 180.0 / M_PI));
	shell_print(sh, "Use 'robot raw' to verify angles/h/phi.");
	return 0;
}

/* robot diag <h_mm> <phi_deg> */
static int cmd_robot_diag(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_print(sh, "Usage: robot diag <h_mm> <phi_deg>");
		return -EINVAL;
	}
	float h_mm = (float)atof(argv[1]);
	float phi_deg = (float)atof(argv[2]);
	float phi_rad = phi_deg * 3.1415926535f / 180.0f;
	shell_print(sh, "Running diag h=%.1f phi=%.1f deg -> see serial",
		    (double)h_mm, (double)phi_deg);
	leg_diag(h_mm, phi_rad);
	shell_print(sh, "Diag done.");
	return 0;
}

/* robot move <h_mm> <phi_deg> — 轨迹插值到绝对目标 (限速: h≤10mm/s, phi≤2deg/s) */
static int cmd_robot_move(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_print(sh, "Usage: robot move <h_mm> <phi_deg>");
		shell_print(sh, "  h_mm:   足端高度 (允许 45.0-100.0 mm, 限速 10mm/s)");
		shell_print(sh, "  phi_deg: 摆动角 (允许 ±30°, 限速 2deg/s)");
		return -EINVAL;
	}

	if (!g_dm4310.bringup_done) {
		shell_print(sh, "Bringup not done, cannot move");
		return -EAGAIN;
	}

	if (g_dm4310.online_mask != 0x0F) {
		shell_print(sh, "REJECTED: not all motors online (mask=0x%x)",
			    g_dm4310.online_mask);
		return -ENOTCONN;
	}

	float h_mm = (float)atof(argv[1]);
	float phi_deg = (float)atof(argv[2]);

	if (h_mm < ROBOT_H_USER_MIN_MM || h_mm > ROBOT_H_USER_MAX_MM) {
		shell_print(sh, "REJECTED: h=%.1f mm out of range (%.0f-%.0f mm)",
			    (double)h_mm,
			    (double)ROBOT_H_USER_MIN_MM, (double)ROBOT_H_USER_MAX_MM);
		return -EINVAL;
	}
	if (phi_deg < -ROBOT_PHI_USER_MAX_DEG || phi_deg > ROBOT_PHI_USER_MAX_DEG) {
		shell_print(sh, "REJECTED: phi=%.1f deg out of range (±%.0f deg)",
			    (double)phi_deg,
			    (double)ROBOT_PHI_USER_MAX_DEG);
		return -EINVAL;
	}

	int ret = robot_ctrl_move_to(h_mm, phi_deg);
	if (ret == 0) {
		shell_print(sh, "Move traj: %.1f→%.1f mm, %.1f→%.1f deg (≤10mm/s, ≤2deg/s)",
			    (double)g_robot.traj_h_current,
			    (double)h_mm,
			    (double)g_robot.traj_phi_current,
			    (double)phi_deg);
	} else {
		shell_print(sh, "Move failed: ret=%d", ret);
	}
	return ret;
}

/* robot jog h <delta_mm> — h 向小步微调 (单次 ≤5mm) */
static int cmd_robot_jog_h(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: robot jog h <delta_mm> (|delta|≤1mm)");
		return -EINVAL;
	}

	if (!g_dm4310.bringup_done || g_dm4310.online_mask != 0x0F) {
		shell_print(sh, "REJECTED: bringup not done or motors offline");
		return -EAGAIN;
	}

	float delta = (float)atof(argv[1]);
	if (fabsf(delta) > 1.0f) {
		shell_print(sh, "REJECTED: |delta|=%.1f mm > 1mm", (double)fabsf(delta));
		return -EINVAL;
	}

	float new_h = g_robot.traj_h_current + delta;
	if (new_h < ROBOT_H_USER_MIN_MM || new_h > ROBOT_H_USER_MAX_MM) {
		shell_print(sh, "REJECTED: target h=%.1f mm out of range (%.0f-%.0f)",
			    (double)new_h,
			    (double)ROBOT_H_USER_MIN_MM, (double)ROBOT_H_USER_MAX_MM);
		return -EINVAL;
	}

	int ret = robot_ctrl_jog_h(delta);
	if (ret == 0) {
		shell_print(sh, "Jog h: %+.1f mm → %.1f mm",
			    (double)delta, (double)g_robot.traj_h_target);
	} else {
		shell_print(sh, "Jog h failed: ret=%d", ret);
	}
	return ret;
}

/* robot jog phi <delta_deg> — phi 向小步微调 (单次 ≤1deg) */
static int cmd_robot_jog_phi(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: robot jog phi <delta_deg> (|delta|≤0.2deg)");
		return -EINVAL;
	}

	if (!g_dm4310.bringup_done || g_dm4310.online_mask != 0x0F) {
		shell_print(sh, "REJECTED: bringup not done or motors offline");
		return -EAGAIN;
	}

	float delta = (float)atof(argv[1]);
	if (fabsf(delta) > 0.2f) {
		shell_print(sh, "REJECTED: |delta|=%.1f deg > 0.2deg", (double)fabsf(delta));
		return -EINVAL;
	}

	float new_phi = g_robot.traj_phi_current + delta;
	if (new_phi < -ROBOT_PHI_USER_MAX_DEG || new_phi > ROBOT_PHI_USER_MAX_DEG) {
		shell_print(sh, "REJECTED: target phi=%.1f deg out of range (±%.0f)",
			    (double)new_phi,
			    (double)ROBOT_PHI_USER_MAX_DEG);
		return -EINVAL;
	}

	int ret = robot_ctrl_jog_phi(delta);
	if (ret == 0) {
		shell_print(sh, "Jog phi: %+.1f deg → %.1f deg",
			    (double)delta, (double)g_robot.traj_phi_target);
	} else {
		shell_print(sh, "Jog phi failed: ret=%d", ret);
	}
	return ret;
}

/* leg left <h_mm> <phi_deg> — 仅左腿 (M1+M2) */
static int cmd_leg_left(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_print(sh, "Usage: leg left <h_mm> <phi_deg>");
		return -EINVAL;
	}
	if (!g_dm4310.bringup_done) {
		shell_print(sh, "Bringup not done");
		return -EAGAIN;
	}

	float h = (float)atof(argv[1]);
	float phi = (float)atof(argv[2]) * 3.1415926535f / 180.0f;

	/* 调试模式使用直接设置 (不走 delta 限幅), 使用 TRAJ_KP/KD */
	for (int i = 0; i < 2; i++) {
		g_dm4310.hold_kp[i] = TRAJ_KP;
		g_dm4310.hold_kd[i] = TRAJ_KD;
	}
	g_dm4310.hold_updates = 1U;

	int ret = leg_set_left(h, phi);
	if (ret != 0) {
		shell_print(sh, "IK FAIL L: h=%.1f phi=%.1f°", (double)h, (double)(phi*180/M_PI));
		return ret;
	}
	shell_print(sh, "Leg L: h=%.1f phi=%.1f° → M1=%.3f M2=%.3f",
		    (double)h, (double)(phi*180/M_PI),
		    (double)dm4310_get_hold_pos(1), (double)dm4310_get_hold_pos(2));
	return 0;
}

/* leg right <h_mm> <phi_deg> — 仅右腿 (M3+M4) */
static int cmd_leg_right(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_print(sh, "Usage: leg right <h_mm> <phi_deg>");
		return -EINVAL;
	}
	if (!g_dm4310.bringup_done) {
		shell_print(sh, "Bringup not done");
		return -EAGAIN;
	}

	float h = (float)atof(argv[1]);
	float phi = (float)atof(argv[2]) * 3.1415926535f / 180.0f;

	for (int i = 2; i < 4; i++) {
		g_dm4310.hold_kp[i] = TRAJ_KP;
		g_dm4310.hold_kd[i] = TRAJ_KD;
	}
	g_dm4310.hold_updates = 1U;

	int ret = leg_set_right(h, phi);
	if (ret != 0) {
		shell_print(sh, "IK FAIL R: h=%.1f phi=%.1f°", (double)h, (double)(phi*180/M_PI));
		return ret;
	}
	shell_print(sh, "Leg R: h=%.1f phi=%.1f° → M3=%.3f M4=%.3f",
		    (double)h, (double)(phi*180/M_PI),
		    (double)dm4310_get_hold_pos(3), (double)dm4310_get_hold_pos(4));
	return 0;
}

/* robot stop — 回到拖动模式 (KP=0.01/KD=0.001, 终止轨迹) */
	/* leg left_xy <x_mm> <y_mm> — XY直角坐标 (X=前后, Y=高度向下) */
	static int cmd_leg_left_xy(const struct shell *sh, size_t argc, char **argv)
	{
		if (argc < 3) {
			shell_print(sh, "Usage: leg left_xy <x_mm> <y_mm>");
			return -EINVAL;
		}
		if (!g_dm4310.bringup_done) {
			shell_print(sh, "Bringup not done");
			return -EAGAIN;
		}

		float x = (float)atof(argv[1]);
		float y = (float)atof(argv[2]);
		float h = sqrtf(x * x + y * y);
		float phi = atan2f(x, y);

		for (int i = 0; i < 2; i++) {
			g_dm4310.hold_kp[i] = TRAJ_KP;
			g_dm4310.hold_kd[i] = TRAJ_KD;
		}
		g_dm4310.hold_updates = 1U;

		int ret = leg_move_to_left(h, phi);
		if (ret != 0) {
			shell_print(sh, "IK FAIL L: x=%.1f y=%.1f",
				    (double)x, (double)y);
			return ret;
		}
		shell_print(sh, "Leg L XY(%.1f,%.1f) h=%.1f phi=%.1f° M1=%.3f M2=%.3f",
			    (double)x, (double)y, (double)h, (double)(phi*180/M_PI),
			    (double)dm4310_get_hold_pos(1), (double)dm4310_get_hold_pos(2));
		return 0;
	}

	/* leg right_xy <x_mm> <y_mm> */
	static int cmd_leg_right_xy(const struct shell *sh, size_t argc, char **argv)
	{
		if (argc < 3) {
			shell_print(sh, "Usage: leg right_xy <x_mm> <y_mm>");
			return -EINVAL;
		}
		if (!g_dm4310.bringup_done) {
			shell_print(sh, "Bringup not done");
			return -EAGAIN;
		}

		float x = (float)atof(argv[1]);
		float y = (float)atof(argv[2]);
		float h = sqrtf(x * x + y * y);
		float phi = atan2f(x, y);

		for (int i = 2; i < 4; i++) {
			g_dm4310.hold_kp[i] = TRAJ_KP;
			g_dm4310.hold_kd[i] = TRAJ_KD;
		}
		g_dm4310.hold_updates = 1U;

		int ret = leg_move_to_right(h, phi);
		if (ret != 0) {
			shell_print(sh, "IK FAIL R: x=%.1f y=%.1f",
				    (double)x, (double)y);
			return ret;
		}
		shell_print(sh, "Leg R XY(%.1f,%.1f) h=%.1f phi=%.1f° M3=%.3f M4=%.3f",
			    (double)x, (double)y, (double)h, (double)(phi*180/M_PI),
			    (double)dm4310_get_hold_pos(3), (double)dm4310_get_hold_pos(4));
		return 0;
	}

static int cmd_robot_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	robot_ctrl_stop();
	shell_print(sh, "Robot stopped: back to drag mode (KP=0.01 KD=0.001)");
	return 0;
}


/* robot raw — 诊断输出 (M1-M4, theta_a/b, h/phi, 力矩) */
static int cmd_robot_raw(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	float m[4];
	for (int i = 0; i < 4; i++) {
		m[i] = dm4310_motor_calibrated_pos(i);
	}

	float ta_L = lk_m1_to_theta_a(m[0]);
	float tb_L = lk_m2_to_theta_b(m[1]);
	float ta_R = lk_m3_to_theta_a(m[2]);
	float tb_R = lk_m4_to_theta_b(m[3]);

	float h_L = 0, phi_L = 0, h_R = 0, phi_R = 0;
	lk_forward(ta_L, tb_L, NULL, &h_L, &phi_L);
	lk_forward(ta_R, tb_R, NULL, &h_R, &phi_R);

	shell_print(sh, "M1_cal=%.4f M2_cal=%.4f M3_cal=%.4f M4_cal=%.4f rad",
		    (double)m[0], (double)m[1], (double)m[2], (double)m[3]);
	shell_print(sh, "M1_raw=%.4f M2_raw=%.4f M3_raw=%.4f M4_raw=%.4f rad",
		    (double)g_dm4310.motor[0].pos_rad,
		    (double)g_dm4310.motor[1].pos_rad,
		    (double)g_dm4310.motor[2].pos_rad,
		    (double)g_dm4310.motor[3].pos_rad);
	shell_print(sh, "theta_a_L=%.2f theta_b_L=%.2f theta_a_R=%.2f theta_b_R=%.2f deg",
		    (double)(ta_L * 180.0 / M_PI),
		    (double)(tb_L * 180.0 / M_PI),
		    (double)(ta_R * 180.0 / M_PI),
		    (double)(tb_R * 180.0 / M_PI));
	shell_print(sh, "h_L=%.1f mm phi_L=%.1f deg  h_R=%.1f mm phi_R=%.1f deg",
		    (double)h_L, (double)(phi_L * 180.0 / M_PI),
		    (double)h_R, (double)(phi_R * 180.0 / M_PI));
	shell_print(sh, "t1=%.3f t2=%.3f t3=%.3f t4=%.3f Nm",
		    (double)g_dm4310.motor[0].torque_nm,
		    (double)g_dm4310.motor[1].torque_nm,
		    (double)g_dm4310.motor[2].torque_nm,
		    (double)g_dm4310.motor[3].torque_nm);
	return 0;
}

/* robot viz — ASCII 侧视简图 (带车体边界/O/P标记/限位指示) */
static int cmd_robot_viz(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	float ta_L = lk_m1_to_theta_a(dm4310_motor_calibrated_pos(0));
	float tb_L = lk_m2_to_theta_b(dm4310_motor_calibrated_pos(1));
	float ta_R = lk_m3_to_theta_a(dm4310_motor_calibrated_pos(2));
	float tb_R = lk_m4_to_theta_b(dm4310_motor_calibrated_pos(3));

	float h_L, phi_L, h_R, phi_R;
	if (lk_forward(ta_L, tb_L, NULL, &h_L, &phi_L) != LK_OK) { h_L = 0; phi_L = 0; }
	if (lk_forward(ta_R, tb_R, NULL, &h_R, &phi_R) != LK_OK) { h_R = 0; phi_R = 0; }

	float phi_L_d = (double)(phi_L * 180.0 / M_PI);
	float phi_R_d = (double)(phi_R * 180.0 / M_PI);
	float ta_L_d = (double)(ta_L * 180.0 / M_PI);
	float tb_L_d = (double)(tb_L * 180.0 / M_PI);
	float ta_R_d = (double)(ta_R * 180.0 / M_PI);
	float tb_R_d = (double)(tb_R * 180.0 / M_PI);

	/* 限位检测 (任务空间 + 关节空间) */
	bool h_near = (h_L <= LK_H_SOFT_MIN || h_L >= LK_H_SOFT_MAX ||
		       h_R <= LK_H_SOFT_MIN || h_R >= LK_H_SOFT_MAX);
	bool phi_near = (phi_L <= LK_PHI_SOFT_MIN || phi_L >= LK_PHI_SOFT_MAX ||
			 phi_R <= LK_PHI_SOFT_MIN || phi_R >= LK_PHI_SOFT_MAX);
	bool jt_near = (ta_L <= LK_THETA_A_MIN || ta_L >= LK_THETA_A_MAX ||
			ta_R <= LK_THETA_A_MIN || ta_R >= LK_THETA_A_MAX ||
			tb_L <= LK_THETA_B_MIN || tb_L >= LK_THETA_B_MAX ||
			tb_R <= LK_THETA_B_MIN || tb_R >= LK_THETA_B_MAX);
	const char *limit_str = (h_near || phi_near || jt_near)
				? "!! NEAR LIMIT !!" : "OK";

	/* 机构坐标系: +X右 +Y上, O 在原点 */
	/* P7 = h * [-sin(phi), -cos(phi)] */
	float Px_L = h_L * sinf(-phi_L);
	float Py_L = -h_L * cosf(-phi_L);
	float P2x_L = LK_L1_MM * cosf(ta_L);
	float P2y_L = LK_L1_MM * sinf(ta_L);

	shell_print(sh, "=== Robot Viz ===  limit: %s", limit_str);
	shell_print(sh, "Left:  h=%.1fmm phi=%.1f°  θa=%.1f° θb=%.1f°",
		    (double)h_L, phi_L_d, ta_L_d, tb_L_d);
	shell_print(sh, "Right: h=%.1fmm phi=%.1f°  θa=%.1f° θb=%.1f°",
		    (double)h_R, phi_R_d, ta_R_d, tb_R_d);
	shell_print(sh, "Motor torques: M1=%.3f M2=%.3f M3=%.3f M4=%.3f Nm",
		    (double)g_dm4310.motor[0].torque_nm,
		    (double)g_dm4310.motor[1].torque_nm,
		    (double)g_dm4310.motor[2].torque_nm,
		    (double)g_dm4310.motor[3].torque_nm);
	shell_print(sh, "");
	shell_print(sh, "            +============+  ← body");
	shell_print(sh, "   O───────+   CHASSIS   +───────O  O=motor");
	shell_print(sh, "  /                              \\");
	shell_print(sh, " /                                \\");
	shell_print(sh, "P2(%.0f,%.0f)                  P2", (double)P2x_L, (double)P2y_L);
	shell_print(sh, " \\                                /");
	shell_print(sh, "  \\                              /");
	shell_print(sh, "   P7@(%.0f,%.0f)               P7  P=foot",
		    (double)Px_L, (double)Py_L);
	shell_print(sh, "");

	if (h_near)   shell_print(sh, "  -> h near limit (%.0f-%.0fmm)", (double)LK_H_SOFT_MIN, (double)LK_H_SOFT_MAX);
	if (phi_near) shell_print(sh, "  -> phi near limit (±%.0f°)", (double)(LK_PHI_SOFT_MAX * 180.0 / M_PI));
	if (jt_near)  shell_print(sh, "  -> joint near limit (θa:%.0f~%.0f θb:%.0f~%.0f°)",
				    (double)(LK_THETA_A_MIN * 180.0 / M_PI),
				    (double)(LK_THETA_A_MAX * 180.0 / M_PI),
				    (double)(LK_THETA_B_MIN * 180.0 / M_PI),
				    (double)(LK_THETA_B_MAX * 180.0 / M_PI));

	return 0;
}

/* robot limit_scan — 只读连续扫描 (5Hz), 手推机构时追踪 min/max + 限位检测 */
static int cmd_robot_limit_scan(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Limit scan starting @ 5Hz. Close terminal to stop.");
	shell_print(sh, "Tracking: M1-M4, θa/θb, h/φ (L+R), torque, min/max, limits.");

	/* 重置扫描状态 */
	memset(&g_scan, 0, sizeof(g_scan));

	uint32_t iter = 0;
	while (1) {
		scan_update(&g_scan);
		scan_print(sh, &g_scan, ++iter);
		k_msleep(200); /* 5Hz */
	}
	return 0;
}

/* robot set_offset <1-4> <deg> — 手动微调安装角偏置 */
static int cmd_robot_set_offset(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_print(sh, "Usage: robot set_offset <1-4> <deg>");
		return -EINVAL;
	}
	int id = atoi(argv[1]);
	if (id < 1 || id > DM4310_MOTOR_COUNT) {
		shell_print(sh, "Invalid motor ID: %d", id);
		return -EINVAL;
	}
	float delta_rad = (float)atof(argv[2]) * 3.1415926535f / 180.0f;
	float old = g_dm_offset[id - 1];
	g_dm_offset[id - 1] -= delta_rad;
	shell_print(sh, "M%d offset: %.4f -> %.4f rad (delta=%.1f deg)",
		    id, (double)old, (double)g_dm_offset[id-1],
		    (double)atof(argv[2]));
	return 0;
}
/* balance pitch_zero [value] — 设置 IMU Pitch 零点偏置 */
static int cmd_balance_pitch_zero(const struct shell *sh, size_t argc, char **argv)
{
	if (argc >= 2) {
		g_balance_pitch_zero_rad = (float)atof(argv[1]);
		shell_print(sh, "Pitch zero set: %.4f rad (%.1f deg)",
			    (double)g_balance_pitch_zero_rad,
			    (double)g_balance_pitch_zero_rad * 180.0 / 3.1415926535);
	} else {
		shell_print(sh, "Pitch zero: %.4f rad (%.1f deg)",
			    (double)g_balance_pitch_zero_rad,
			    (double)g_balance_pitch_zero_rad * 180.0 / 3.1415926535);
		shell_print(sh, "Usage: balance pitch_zero <rad>");
	}
	return 0;
}

/* balance enable — 一键切入站立模式 */
static int cmd_balance_enable(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t ramp = 100; /* 默认 100 tick (500ms @ 5ms) */

	if (argc >= 2) {
		ramp = (uint32_t)atoi(argv[1]);
	}

	int ret = dm4310_balance_enable(ramp);
	if (ret == 0) {
		shell_print(sh, "Balance enable: ramp=%u ticks, target KP=80 KD=1.5",
			    ramp);
	} else {
		shell_print(sh, "Balance enable failed: bringup not done (ret=%d)",
			    ret);
	}
	return ret;
}

/* motor telemetry <on|off> — 高速遥测开关 (Unit6 系统辨识) */
static int cmd_motor_telemetry(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: motor telemetry <on|off>");
		return -EINVAL;
	}
	if (strcmp(argv[1], "on") == 0) {
		g_telemetry_enabled = true;
		shell_print(sh, "Telemetry: on");
	} else if (strcmp(argv[1], "off") == 0) {
		g_telemetry_enabled = false;
		shell_print(sh, "Telemetry: off");
	} else {
		shell_print(sh, "Usage: motor telemetry <on|off>");
		return -EINVAL;
	}
	return 0;
}

/* motor raw — 显示四台电机最近一帧原始 CAN 数据 */
static int cmd_motor_raw(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		const uint8_t *d = g_dm4310.motor[i].raw_frame;
		shell_print(sh, "M%d: %02x %02x %02x %02x %02x %02x %02x %02x  rx=%u",
			    i + 1,
			    d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7],
			    g_dm4310.motor[i].rx_count);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(motor_cmds,
	SHELL_CMD_ARG(enable,    NULL, "motor enable <1-4|all>",    cmd_motor_enable,    2, 0),
	SHELL_CMD_ARG(disable, NULL, "motor disable <1-4|all>", cmd_motor_disable, 2, 0),
	SHELL_CMD_ARG(zero,    NULL, "motor zero <1-4|all>",    cmd_motor_zero,    2, 0),
	SHELL_CMD_ARG(csv,     NULL, "motor csv <on|off|once>", cmd_motor_csv,     2, 0),
	SHELL_CMD_ARG(status,  NULL, "motor status",             cmd_motor_status,  1, 0),
	SHELL_CMD_ARG(raw,     NULL, "motor raw — show CAN raw frames", cmd_motor_raw, 1, 0),
	SHELL_CMD_ARG(kp,      NULL, "motor kp <1-4> <value>",  cmd_motor_kp,      3, 0),
	SHELL_CMD_ARG(kd,        NULL, "motor kd <1-4> <value>",      cmd_motor_kd,        3, 0),
	SHELL_CMD_ARG(torque,    NULL, "motor torque <1-4> <Nm>",     cmd_motor_torque,    3, 0),
	SHELL_CMD_ARG(lock,      NULL, "motor lock [kp] [kd] — lock at current pos", cmd_motor_lock, 1, 2),
	SHELL_CMD_ARG(unlock,    NULL, "motor unlock — back to drag mode", cmd_motor_unlock, 1, 0),
	SHELL_CMD_ARG(telemetry, NULL, "motor telemetry <on|off>",    cmd_motor_telemetry, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(motor, &motor_cmds, "DM4310 motor debug commands", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(balance_cmds,
	SHELL_CMD_ARG(pitch_zero, NULL, "balance pitch_zero [rad]", cmd_balance_pitch_zero, 1, 1),
	SHELL_CMD_ARG(enable,     NULL, "balance enable [ramp_ticks]", cmd_balance_enable,   1, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(balance, &balance_cmds, "Balance control commands", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(jog_cmds,
	SHELL_CMD_ARG(h,   NULL, "robot jog h <delta_mm> (|delta|≤1mm)", cmd_robot_jog_h,   2, 0),
	SHELL_CMD_ARG(phi, NULL, "robot jog phi <delta_deg> (|delta|≤0.2deg)", cmd_robot_jog_phi, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(leg_cmds,
	SHELL_CMD_ARG(left,     NULL, "leg left <h_mm> <phi_deg>",         cmd_leg_left,     3, 0),
	SHELL_CMD_ARG(right,    NULL, "leg right <h_mm> <phi_deg>",        cmd_leg_right,    3, 0),
	SHELL_CMD_ARG(left_xy,  NULL, "leg left_xy <x_mm> <y_mm>",         cmd_leg_left_xy,  3, 0),
	SHELL_CMD_ARG(right_xy, NULL, "leg right_xy <x_mm> <y_mm>",        cmd_leg_right_xy, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(leg, &leg_cmds, "Single leg control", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(robot_cmds,
	SHELL_CMD_ARG(diag, NULL, "robot diag <h_mm> <phi_deg>", cmd_robot_diag, 3, 0),
	SHELL_CMD_ARG(cali, NULL, "robot cali — save current pose as zero + track h0/phi0", cmd_robot_cali, 1, 0),
	SHELL_CMD_ARG(move, NULL, "robot move <h_mm> <phi_deg> (traj limited)", cmd_robot_move, 3, 0),
	SHELL_CMD(jog, &jog_cmds, "robot jog — small incremental move", NULL),
	SHELL_CMD_ARG(stop, NULL, "robot stop — back to drag mode + cancel traj", cmd_robot_stop, 1, 0),
	SHELL_CMD_ARG(raw, NULL, "robot raw — show motor angles + FK + torque", cmd_robot_raw, 1, 0),
	SHELL_CMD_ARG(viz, NULL, "robot viz — ASCII side view with limits", cmd_robot_viz, 1, 0),
	SHELL_CMD_ARG(limit_scan, NULL, "robot limit_scan — 5Hz read-only sweep, min/max + limits", cmd_robot_limit_scan, 1, 0),
	SHELL_CMD_ARG(set_offset, NULL, "robot set_offset <1-4> <deg>", cmd_robot_set_offset, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(robot, &robot_cmds, "Robot calibration commands", NULL);

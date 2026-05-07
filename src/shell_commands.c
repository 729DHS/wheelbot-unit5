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

#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

/* Shell 实例指针 (供 main.c CSV 输出使用) */
const struct shell *g_motor_shell;
/* CSV 流开关 */
bool g_csv_enabled;

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
	g_dm4310.feedforward_tau[id - 1] = val;
	g_dm4310.hold_updates = 1U;
	shell_print(sh, "M%d feedforward_tau=%.3f Nm", id, (double)val);
	return 0;
}

/* robot cali — 一键找零: 将当前电机位置保存为零点偏置 */
static int cmd_robot_cali(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		g_dm_offset[i] = g_dm4310.motor[i].pos_rad;
	}
	shell_print(sh, "Calibration saved: M1=%.4f M2=%.4f M3=%.4f M4=%.4f rad",
		    (double)g_dm_offset[0], (double)g_dm_offset[1],
		    (double)g_dm_offset[2], (double)g_dm_offset[3]);
	shell_print(sh, "Current pose is now the zero reference.");
	return 0;
}

/* robot move <h_mm> <phi_deg> — 悬空小角度验证 (调用 leg_move_all) */
static int cmd_robot_move(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_print(sh, "Usage: robot move <h_mm> <phi_deg>");
		shell_print(sh, "  h_mm:   足端高度 (允许 120-200 mm)");
		shell_print(sh, "  phi_deg: 摆动角 (允许 ±8°)");
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

	if (h_mm < 120.0f || h_mm > 200.0f) {
		shell_print(sh, "REJECTED: h=%.1f mm out of range (120-200 mm)",
			    (double)h_mm);
		return -EINVAL;
	}
	if (phi_deg < -8.0f || phi_deg > 8.0f) {
		shell_print(sh, "REJECTED: phi=%.1f deg out of range (±8 deg)",
			    (double)phi_deg);
		return -EINVAL;
	}

	float phi_rad = phi_deg * 3.1415926535f / 180.0f;
	int ret = leg_move_all(h_mm, phi_rad);
	if (ret == 0) {
		shell_print(sh, "Move: h=%.1f mm phi=%.1f deg",
			    (double)h_mm, (double)phi_deg);
	} else {
		shell_print(sh, "Move failed: ret=%d", ret);
	}
	return ret;
}

/* robot stop — 回到拖动模式 (KP=0.01/KD=0.001, 清零前馈) */
static int cmd_robot_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		g_dm4310.hold_kp[i] = 0.01f;
		g_dm4310.hold_kd[i] = 0.001f;
		g_dm4310.feedforward_tau[i] = 0.0f;
		g_dm4310.hold_pos_rad[i] = g_dm4310.motor[i].pos_rad;
	}
	g_dm4310.hold_updates = 1U;
	g_dm4310.balance_ramp_remaining = 0U;
	shell_print(sh, "Robot stopped: back to drag mode (KP=0.01 KD=0.001)");
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

SHELL_STATIC_SUBCMD_SET_CREATE(motor_cmds,
	SHELL_CMD_ARG(enable,    NULL, "motor enable <1-4|all>",    cmd_motor_enable,    2, 0),
	SHELL_CMD_ARG(disable, NULL, "motor disable <1-4|all>", cmd_motor_disable, 2, 0),
	SHELL_CMD_ARG(zero,    NULL, "motor zero <1-4|all>",    cmd_motor_zero,    2, 0),
	SHELL_CMD_ARG(csv,     NULL, "motor csv <on|off|once>", cmd_motor_csv,     2, 0),
	SHELL_CMD_ARG(status,  NULL, "motor status",             cmd_motor_status,  1, 0),
	SHELL_CMD_ARG(kp,      NULL, "motor kp <1-4> <value>",  cmd_motor_kp,      3, 0),
	SHELL_CMD_ARG(kd,        NULL, "motor kd <1-4> <value>",      cmd_motor_kd,        3, 0),
	SHELL_CMD_ARG(torque,    NULL, "motor torque <1-4> <Nm>",     cmd_motor_torque,    3, 0),
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

SHELL_STATIC_SUBCMD_SET_CREATE(robot_cmds,
	SHELL_CMD_ARG(cali, NULL, "robot cali — save current pose as zero", cmd_robot_cali, 1, 0),
	SHELL_CMD_ARG(move, NULL, "robot move <h_mm> <phi_deg>", cmd_robot_move, 3, 0),
	SHELL_CMD_ARG(stop, NULL, "robot stop — back to drag mode", cmd_robot_stop, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(robot, &robot_cmds, "Robot calibration commands", NULL);

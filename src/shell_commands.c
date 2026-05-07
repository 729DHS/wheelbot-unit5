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
		shell_print(sh, "CSV: on");
	} else if (strcmp(argv[1], "off") == 0) {
		g_csv_enabled = false;
		shell_print(sh, "CSV: off");
	} else if (strcmp(argv[1], "once") == 0) {
		shell_print(sh, "%u,%.4f,%.4f,%.4f,%.4f",
			    k_uptime_get_32(),
			    (double)g_dm4310.motor[0].pos_rad,
			    (double)g_dm4310.motor[1].pos_rad,
			    (double)g_dm4310.motor[2].pos_rad,
			    (double)g_dm4310.motor[3].pos_rad);
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

SHELL_STATIC_SUBCMD_SET_CREATE(motor_cmds,
	SHELL_CMD_ARG(enable,  NULL, "motor enable <1-4|all>",  cmd_motor_enable,  2, 0),
	SHELL_CMD_ARG(disable, NULL, "motor disable <1-4|all>", cmd_motor_disable, 2, 0),
	SHELL_CMD_ARG(zero,    NULL, "motor zero <1-4|all>",    cmd_motor_zero,    2, 0),
	SHELL_CMD_ARG(csv,     NULL, "motor csv <on|off|once>", cmd_motor_csv,     2, 0),
	SHELL_CMD_ARG(status,  NULL, "motor status",             cmd_motor_status,  1, 0),
	SHELL_CMD_ARG(kp,      NULL, "motor kp <1-4> <value>",  cmd_motor_kp,      3, 0),
	SHELL_CMD_ARG(kd,      NULL, "motor kd <1-4> <value>",  cmd_motor_kd,      3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(motor, &motor_cmds, "DM4310 motor debug commands", NULL);

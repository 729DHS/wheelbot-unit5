/**
 * @file main.c
 * @brief Unit5 — DM4310 串口反馈测试 + Shell 控制
 *
 * 上电 bringup 四台电机后，以极小增益保持 MIT 使能状态。
 * Shell 命令 "motor csv on" 可开启 CSV 角度流输出。
 * USART6 (3-pin, PG14/PG9) = 板子丝印 UART1。
 */

#include "dm4310_motor.h"

#include <stdio.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>

/* 来自 shell_commands.c 的全局变量 */
extern const struct shell *g_motor_shell;
extern bool g_csv_enabled;

#define CTRL_PERIOD_MS  5
#define PRINT_PERIOD_MS 10

int main(void)
{
	printk("=== Unit5 DM4310 Feedback Test ===\n");

	int ret = dm4310_init();
	if (ret < 0) {
		printk("DM4310 init failed: %d\n", ret);
		return ret;
	}
	printk("DM4310 driver initialized\n");

	/* 极小增益 → 电机保持使能但力矩≈0，可被自由拖动 */
	g_dm4310.hold_kp[0] = 0.01f; g_dm4310.hold_kd[0] = 0.001f;
	g_dm4310.hold_kp[1] = 0.01f; g_dm4310.hold_kd[1] = 0.001f;
	g_dm4310.hold_kp[2] = 0.01f; g_dm4310.hold_kd[2] = 0.001f;
	g_dm4310.hold_kp[3] = 0.01f; g_dm4310.hold_kd[3] = 0.001f;

	uint32_t last_print = 0;
	bool bringup_reported = false;

	while (1) {
		dm4310_poll_rx();

		/* GDB 调试命令处理 (主循环消费，GDB set 写入) */
		for (int g = 0; g < DM4310_MOTOR_COUNT; g++) {
			uint8_t cmd = g_gdb_cmd[g];
			if (cmd == GDB_CMD_ENABLE) {
				dm4310_enable_motor((uint8_t)(g + 1));
				g_gdb_cmd[g] = GDB_CMD_NONE;
			} else if (cmd == GDB_CMD_DISABLE) {
				dm4310_disable_motor((uint8_t)(g + 1));
				g_gdb_cmd[g] = GDB_CMD_NONE;
			} else if (cmd == GDB_CMD_ZERO) {
				dm4310_zero_motor((uint8_t)(g + 1));
				g_gdb_cmd[g] = GDB_CMD_NONE;
			}
		}

		if (g_dm4310.bringup_done) {
			if (!bringup_reported) {
				printk("Bringup done. online=0x%x\n",
				       g_dm4310.online_mask);
				bringup_reported = true;
			}

			/* 保持当前位置，极小力矩 */
			float target[DM4310_MOTOR_COUNT];
			for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
				target[i] = g_dm4310.motor[i].pos_rad;
			}
			dm4310_hold_positions(target);
		}

		dm4310_tick();

		/* CSV 角度流 (通过 Shell 输出，避免与 uart:~$ 冲突) */
		if (g_csv_enabled && g_motor_shell != NULL) {
			uint32_t now = k_uptime_get_32();
			if (now - last_print >= PRINT_PERIOD_MS) {
				last_print = now;
				shell_print(g_motor_shell,
					    "%u,%.4f,%.4f,%.4f,%.4f",
					    now,
					    (double)g_dm4310.motor[0].pos_rad,
					    (double)g_dm4310.motor[1].pos_rad,
					    (double)g_dm4310.motor[2].pos_rad,
					    (double)g_dm4310.motor[3].pos_rad);
			}
		}

		k_sleep(K_MSEC(CTRL_PERIOD_MS));
	}
}

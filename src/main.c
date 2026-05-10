/**
 * @file main.c
 * @brief Unit5 — DM4310 500Hz 控制固件 + DWT 性能分析
 *
 * 500Hz 固定周期控制循环 (DWT CYCCNT 忙等)。
 * CSV 角度流输出降频至 50Hz (每 10 tick 输出一行)。
 * GPIO PH13 用作分析引脚: 控制循环入口 HIGH, 出口 LOW。
 *
 * CSV 格式: t_ms,m1,m2,m3,m4,t1,t2,t3,t4,pitch,pitch_rate,dt_us
 *   pitch / pitch_rate: BMI088 暂未接入, 填 0
 *   dt_us: 本 tick 控制循环耗时 (DWT 实测, us)
 */

#include "dm4310_motor.h"
#include "motor_debug.h"
#include "robot_ctrl.h"
#include "protocol_service.h"
#include "leg_control.h"
#include "linkage_kinematics.h"

#include <stdio.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>

extern const struct shell *g_motor_shell;
extern bool g_csv_enabled;

/* === DWT 周期计数器 (CMSIS 提供 DWT/CoreDebug 寄存器定义) === */

/* === GPIO 分析引脚 PH13 ===
 * 控制循环入口 HIGH, 出口 LOW, 示波器测量真实控制耗时  */
#define PROFILE_GPIO_BASE  0x40021C00UL
#define PROFILE_MODER  (*(volatile uint32_t *)(PROFILE_GPIO_BASE))
#define PROFILE_BSRR   (*(volatile uint32_t *)(PROFILE_GPIO_BASE + 0x18))
#define PROFILE_PIN    13

#define PROFILE_HIGH()  (PROFILE_BSRR = (1U << PROFILE_PIN))
#define PROFILE_LOW()   (PROFILE_BSRR = (1U << (PROFILE_PIN + 16)))

/* === 控制周期 === */
#define CTRL_FREQ_HZ    500
#define SYS_CORE_FREQ   168000000UL
#define CYCLES_PER_TICK (SYS_CORE_FREQ / CTRL_FREQ_HZ)  /* 336000 = 2ms */
#define CSV_PERIOD_TICKS 10   /* 500Hz / 10 = 50Hz CSV */
#define TLM_PERIOD_TICKS 20   /* 500Hz / 20 = 25Hz 遥测 */
#define TLM_OFFSET_TICKS 5    /* 遥测与 CSV 错开 10ms, 避免同时 shell_print */

static void dwt_init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void profile_pin_init(void)
{
	/* PH13: output push-pull */
	PROFILE_MODER = (PROFILE_MODER & ~(3U << (PROFILE_PIN * 2))) |
			(1U << (PROFILE_PIN * 2));
}

int main(void)
{
	printk("=== Unit5 DM4310 500Hz Control ===\n");

	dwt_init();
	profile_pin_init();

	int ret = dm4310_init();
	if (ret < 0) {
		printk("DM4310 init failed: %d\n", ret);
		return ret;
	}
	printk("DM4310 driver initialized\n");

	proto_init();

	uint32_t next_wake = DWT->CYCCNT;
	uint16_t tick_count = 0;
	bool bringup_reported = false;
	uint32_t loop_start, loop_end, loop_dt_us = 0;

	while (1) {
		PROFILE_HIGH();
		loop_start = DWT->CYCCNT;

		dm4310_poll_rx();

		/* GDB 调试命令 */
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
				/* 拖动模式: 极小增益, 可手拽 */
				for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
					g_dm4310.hold_pos_rad[i] = g_dm4310.motor[i].pos_rad;
					g_dm4310.hold_kp[i] = 0.01f;
					g_dm4310.hold_kd[i] = 0.001f;
				}
				g_dm4310.hold_updates = 1U;
				printk("DRAG mode ready. Run 'robot cali' to calibrate.\n");
				bringup_reported = true;
			}
		}

		dm4310_tick();

		loop_end = DWT->CYCCNT;
		/* DWT 实测本 tick 控制耗时 (含 CAN 发送, 不含忙等) */
		loop_dt_us = (loop_end - loop_start) * 1000000UL / SYS_CORE_FREQ;

		/* CSV 50Hz (tick 0,10,20...) + 遥测 25Hz (tick 5,25,45...) */
		if (tick_count % CSV_PERIOD_TICKS == 0) {
			uint32_t now = k_uptime_get_32();

			if (g_csv_enabled && g_motor_shell != NULL) {
				shell_print(g_motor_shell,
					"%u,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.4f,%.2f,%u",
					now,
					(double)g_dm4310.motor[0].pos_rad,
					(double)g_dm4310.motor[1].pos_rad,
					(double)g_dm4310.motor[2].pos_rad,
					(double)g_dm4310.motor[3].pos_rad,
					(double)g_dm4310.motor[0].torque_nm,
					(double)g_dm4310.motor[1].torque_nm,
					(double)g_dm4310.motor[2].torque_nm,
					(double)g_dm4310.motor[3].torque_nm,
					0.0, 0.0,   /* pitch, pitch_rate: BMI088 暂未接入 */
					loop_dt_us);
			}
		}

		if ((tick_count + TLM_OFFSET_TICKS) % TLM_PERIOD_TICKS == 0) {
			if (g_telemetry_enabled && g_motor_shell != NULL) {
				uint32_t now = k_uptime_get_32();
				telemetry_print(g_motor_shell, now, 0.0f, 0.0f, 0.0f);
			}
			proto_telemetry_tick();
		}

		tick_count++;

		PROFILE_LOW();

		/* DWT 忙等至下一个 2ms 边界 (处理 32-bit 翻转) */
		next_wake += CYCLES_PER_TICK;
		while ((int32_t)(DWT->CYCCNT - next_wake) < 0) {
			/* busy-wait */
		}
	}
}

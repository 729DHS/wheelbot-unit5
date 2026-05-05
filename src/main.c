/**
 * @file main.c
 * @brief Unit5 — DM4310 串口反馈测试
 *
 * 上电 bringup 四台电机后，以极小增益保持 MIT 使能状态，
 * 每 50ms 通过 USART6 (PG14/PG9 USB-TTL) 输出 CSV 格式反馈数据。
 * 电机可被自由拖动，不做任何运动控制。
 *
 * 串口 USART6 @ 115200 baud（USB-TTL CH340 直连），不做任何解算。
 * 注意: 板子丝印 UART1 (3-pin) 实际对应 STM32 USART6 (PG14/PG9)。
 */
#include "dm4310_motor.h"

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define CTRL_PERIOD_MS  5
#define PRINT_PERIOD_MS 50

int main(void)
{
	printk("=== Unit5 DM4310 Feedback Test ===\n");

	int ret = dm4310_init();
	if (ret < 0) {
		printk("DM4310 init failed: %d\n", ret);
		return ret;
	}
	printk("DM4310 driver initialized\n");

	printk("Bringing up motors...\n");
	while (!g_dm4310.bringup_done) {
		dm4310_poll_rx();
		dm4310_tick();
		k_sleep(K_MSEC(2));
	}

	printk("Bringup done. online=0x%x\n", g_dm4310.online_mask);

	if (g_dm4310.online_mask != 0x0FU) {
		printk("ERROR: not all motors online, halt\n");
		dm4310_stop_all();
		return -1;
	}

	/* 极小增益 → 电机保持使能但力矩≈0，可被自由拖动
	 * 不使用 KP=KD=0（会触发 DISABLE 而非 MIT 控制帧） */
	g_dm4310.hold_kp[0] = 0.01f; g_dm4310.hold_kd[0] = 0.001f;
	g_dm4310.hold_kp[1] = 0.01f; g_dm4310.hold_kd[1] = 0.001f;
	g_dm4310.hold_kp[2] = 0.01f; g_dm4310.hold_kd[2] = 0.001f;
	g_dm4310.hold_kp[3] = 0.01f; g_dm4310.hold_kd[3] = 0.001f;

	printk("# t_ms,M1_rad,M1_vel,M2_rad,M2_vel,M3_rad,M3_vel,M4_rad,M4_vel\n");

	uint32_t last_print = k_uptime_get_32();

	while (1) {
		dm4310_poll_rx();

		float target[DM4310_MOTOR_COUNT];
		for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
			target[i] = g_dm4310.motor[i].pos_rad;
		}
		dm4310_hold_positions(target);
		dm4310_tick();

		uint32_t now = k_uptime_get_32();
		if (now - last_print < PRINT_PERIOD_MS) {
			k_sleep(K_MSEC(CTRL_PERIOD_MS));
			continue;
		}
		last_print = now;

		printk("%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
		       now,
		       (double)g_dm4310.motor[0].pos_rad,
		       (double)g_dm4310.motor[0].vel_radps,
		       (double)g_dm4310.motor[1].pos_rad,
		       (double)g_dm4310.motor[1].vel_radps,
		       (double)g_dm4310.motor[2].pos_rad,
		       (double)g_dm4310.motor[2].vel_radps,
		       (double)g_dm4310.motor[3].pos_rad,
		       (double)g_dm4310.motor[3].vel_radps);

		k_sleep(K_MSEC(CTRL_PERIOD_MS));
	}
}

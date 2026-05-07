/**
 * @file motor_debug.c
 * @brief 高速遥测 — 为 Unit6 系统辨识提供结构化数据流
 *
 * 每行格式:
 *   Time_ms,Pitch_rad,Gyro_dps,h_cmd_mm,phi_cmd_rad,T_out_Nm
 *
 * Pitch/Gyro 字段当前为 0 (BMI088 接入后填入真实值)。
 * 与 CSV 角度流 (motor csv) 共用 USART6，由独立 Shell 开关控制。
 */
#include "motor_debug.h"

#include <stdio.h>
#include <zephyr/shell/shell.h>

bool g_telemetry_enabled;

static float g_h_cmd_mm;
static float g_phi_cmd_rad;

void telemetry_record_cmd(float h_mm, float phi_rad)
{
	g_h_cmd_mm = h_mm;
	g_phi_cmd_rad = phi_rad;
}

void telemetry_print(const struct shell *sh, uint32_t now_ms,
		     float pitch, float gyro, float torque)
{
	if (!g_telemetry_enabled || sh == NULL) {
		return;
	}

	shell_print(sh, "%u,%.4f,%.2f,%.1f,%.4f,%.3f",
		    now_ms,
		    (double)pitch,
		    (double)gyro,
		    (double)g_h_cmd_mm,
		    (double)g_phi_cmd_rad,
		    (double)torque);
}

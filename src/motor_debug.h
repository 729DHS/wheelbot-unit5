/**
 * @file motor_debug.h
 * @brief 高速遥测接口 — 供 Unit6 系统辨识用
 *
 * 输出格式: Time_ms,Pitch_rad,Gyro_dps,h_cmd_mm,phi_cmd_rad,T_out_Nm
 *
 * Pitch/Gyro 字段当前为占位 (无 IMU)，对接 BMI088 后填入真实值。
 */
#ifndef MOTOR_DEBUG_H_
#define MOTOR_DEBUG_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/shell/shell.h>

/* 遥测开关 (Shell 命令控制) */
extern bool g_telemetry_enabled;

/**
 * @brief 记录最近一次控制指令 (h, phi)
 *
 * 由控制环 (leg_move_to_*) 在每次下发目标前调用。
 */
void telemetry_record_cmd(float h_mm, float phi_rad);

/**
 * @brief 输出一行遥测数据
 *
 * @param sh      Shell 实例 (用于 shell_print)
 * @param now_ms  当前时间戳 (ms)
 * @param pitch   当前 Pitch 角 (rad), 无 IMU 时传 0
 * @param gyro    Gyro 角速度 (dps), 无 IMU 时传 0
 * @param torque  估算输出力矩 (Nm), 暂传 0
 */
void telemetry_print(const struct shell *sh, uint32_t now_ms,
		     float pitch, float gyro, float torque);

#endif /* MOTOR_DEBUG_H_ */

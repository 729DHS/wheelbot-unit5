/**
 * @file protocol_service.c
 * @brief PC 控制协议实现 — USART1 中断驱动 + k_msgq + 工作线程
 *
 * USART1 (PA9 TX / PB7 RX, 115200) 专用于上位机通信。
 * ISR 按字节收帧, 以 \n 为帧尾, k_msgq 异步投递到工作线程。
 * 工作线程解析文本命令, 派发到 robot_ctrl / leg_control / dm4310 层。
 *
 * 线程安全: 控制线程写 g_robot.traj_*, 协议线程通过 robot_ctrl_* API
 * 间接修改 (robot_ctrl 函数内部不持锁, 依赖单生产者模式)。
 */

#include "protocol_service.h"
#include "linkage_kinematics.h"
#include "leg_control.h"
#include "robot_ctrl.h"
#include "dm4310_motor.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

/* ================================================================
 *  UART 层
 * ================================================================ */

/** @brief USART1 设备绑定 */
static const struct device *uart_dev;

/** @brief RX 帧组装缓冲区 (ISR 上下文写入) */
static char rx_buf[PROTO_FRAME_MAX];
static uint8_t rx_pos;

/** @brief 消息队列: ISR → 工作线程 */
K_MSGQ_DEFINE(proto_msgq, PROTO_FRAME_MAX, PROTO_MSGQ_DEPTH, 1);

/** @brief 流式遥测开关 */
static bool stream_on;

/* UART ISR 回调 */
static void uart_isr_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_rx_ready(dev)) continue;

		uint8_t byte;
		while (uart_fifo_read(dev, &byte, 1) == 1) {
			/* 忽略 \r (兼容 CRLF) */
			if (byte == '\r') continue;

			if (byte == '\n') {
				/* 帧结束, 投递到消息队列 */
				if (rx_pos > 0 && rx_pos < PROTO_FRAME_MAX) {
					rx_buf[rx_pos] = '\0';
					k_msgq_put(&proto_msgq, rx_buf, K_NO_WAIT);
				}
				rx_pos = 0;
				continue;
			}

			/* 累积字节, 溢出保护 */
			if (rx_pos < PROTO_FRAME_MAX - 1) {
				rx_buf[rx_pos++] = (char)byte;
			} else {
				rx_pos = 0; /* 丢弃溢出帧 */
			}
		}
	}
}

static int uart_init(void)
{
	uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));
	if (!device_is_ready(uart_dev)) {
		printk("PROTO: USART1 not ready\n");
		return -ENODEV;
	}

	uart_irq_callback_set(uart_dev, uart_isr_cb);
	uart_irq_rx_enable(uart_dev);
	printk("PROTO: USART1 initialized (PA9 TX, PB7 RX, 115200)\n");
	return 0;
}

/* 从线程上下文发送原始字节 */
static void uart_tx_raw(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, data[i]);
	}
}

/* ================================================================
 *  协议层
 * ================================================================ */

/* 发送格式化的协议响应 */
void proto_send(const char *fmt, ...)
{
	char buf[PROTO_FRAME_MAX];
	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (len < 0) return;

	/* 确保换行 */
	if ((size_t)len < sizeof(buf) - 2) {
		buf[len]     = '\n';
		buf[len + 1] = '\0';
		uart_tx_raw((const uint8_t *)buf, (size_t)len + 1);
	} else {
		uart_tx_raw((const uint8_t *)buf, (size_t)len);
		uart_tx_raw((const uint8_t *)"\n", 1);
	}
}

bool proto_stream_enabled(void)
{
	return stream_on;
}

/* ---------- 命令处理函数 ---------- */

static void handle_move(const char *args)
{
	float h_mm, phi_deg;
	if (sscanf(args, "%f:%f", &h_mm, &phi_deg) != 2) {
		proto_send("ERR:PARSE:MOVE");
		return;
	}

	/* 限位检查 */
	if (h_mm < ROBOT_H_USER_MIN_MM || h_mm > ROBOT_H_USER_MAX_MM) {
		proto_send("ERR:RANGE:h=%.1f out of [%.0f,%.0f]",
			   (double)h_mm,
			   (double)ROBOT_H_USER_MIN_MM,
			   (double)ROBOT_H_USER_MAX_MM);
		return;
	}
	if (phi_deg < -ROBOT_PHI_USER_MAX_DEG || phi_deg > ROBOT_PHI_USER_MAX_DEG) {
		proto_send("ERR:RANGE:phi=%.1f out of [+/-%.0f]",
			   (double)phi_deg, (double)ROBOT_PHI_USER_MAX_DEG);
		return;
	}

	if (!g_dm4310.bringup_done) {
		proto_send("ERR:NOT_READY:bringup not done");
		return;
	}
	if (g_dm4310.online_mask != 0x0F) {
		proto_send("ERR:OFFLINE:mask=0x%x", g_dm4310.online_mask);
		return;
	}

	int ret = robot_ctrl_move_to(h_mm, phi_deg);
	if (ret == 0) {
		proto_send("OK:MOVE:%.1f:%.1f", (double)h_mm, (double)phi_deg);
	} else {
		proto_send("ERR:MOVE_FAIL:%d", ret);
	}
}

static void handle_jog_h(const char *args)
{
	float delta = atof(args);

	if (fabsf(delta) > 1.0f) {
		proto_send("ERR:RANGE:|delta|=%.1f > 1mm", (double)fabsf(delta));
		return;
	}
	if (!g_dm4310.bringup_done || g_dm4310.online_mask != 0x0F) {
		proto_send("ERR:NOT_READY");
		return;
	}

	int ret = robot_ctrl_jog_h(delta);
	if (ret == 0) {
		proto_send("OK:JOG_H:%+.1f", (double)delta);
	} else {
		proto_send("ERR:JOG_H_FAIL:%d", ret);
	}
}

static void handle_jog_phi(const char *args)
{
	float delta = atof(args);

	if (fabsf(delta) > 0.2f) {
		proto_send("ERR:RANGE:|delta|=%.1f > 0.2deg", (double)fabsf(delta));
		return;
	}
	if (!g_dm4310.bringup_done || g_dm4310.online_mask != 0x0F) {
		proto_send("ERR:NOT_READY");
		return;
	}

	int ret = robot_ctrl_jog_phi(delta);
	if (ret == 0) {
		proto_send("OK:JOG_PHI:%+.1f", (double)delta);
	} else {
		proto_send("ERR:JOG_PHI_FAIL:%d", ret);
	}
}

static void handle_stop(void)
{
	robot_ctrl_stop();
	proto_send("OK:STOP");
}

static void handle_status(void)
{
	float m[4], tq[4];
	for (int i = 0; i < 4; i++) {
		m[i]  = g_dm4310.motor[i].pos_rad;
		tq[i] = g_dm4310.motor[i].torque_nm;
	}

	/* FK 计算末端位姿 */
	float ta_L = lk_m1_to_theta_a(m[0]);
	float tb_L = lk_m2_to_theta_b(m[1]);
	float ta_R = lk_m3_to_theta_a(m[2]);
	float tb_R = lk_m4_to_theta_b(m[3]);
	float h_L, phi_L, h_R, phi_R;
	lk_forward(ta_L, tb_L, NULL, &h_L, &phi_L);
	lk_forward(ta_R, tb_R, NULL, &h_R, &phi_R);

	proto_send("OK:STATUS:"
		   "h_L=%.1f,phi_L=%.1f,h_R=%.1f,phi_R=%.1f,"
		   "m1=%.4f,m2=%.4f,m3=%.4f,m4=%.4f,"
		   "t1=%.3f,t2=%.3f,t3=%.3f,t4=%.3f,"
		   "cmd_h=%.1f,cmd_phi=%.1f,traj=%d,online=0x%x",
		   (double)h_L, (double)(phi_L * 180.0 / M_PI),
		   (double)h_R, (double)(phi_R * 180.0 / M_PI),
		   (double)m[0], (double)m[1], (double)m[2], (double)m[3],
		   (double)tq[0], (double)tq[1], (double)tq[2], (double)tq[3],
		   (double)g_robot.traj_h_current,
		   (double)g_robot.traj_phi_current,
		   g_robot.traj_active ? 1 : 0,
		   g_dm4310.online_mask);
}

static void handle_stream(const char *args)
{
	if (strcmp(args, "ON") == 0) {
		stream_on = true;
		proto_send("OK:STREAM:ON");
	} else if (strcmp(args, "OFF") == 0) {
		stream_on = false;
		proto_send("OK:STREAM:OFF");
	} else {
		proto_send("ERR:STREAM:use ON or OFF");
	}
}

static void handle_mode(const char *args)
{
	if (strcmp(args, "DRAG") == 0) {
		robot_ctrl_stop();
		for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
			g_dm4310.hold_kp[i] = 0.01f;
			g_dm4310.hold_kd[i] = 0.001f;
		}
		g_dm4310.hold_updates = 1U;
		proto_send("OK:MODE:DRAG");
	} else if (strcmp(args, "HOLD") == 0) {
		/* 以当前位置为目标, 中等刚度保持 */
		for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
			g_dm4310.hold_pos_rad[i] = g_dm4310.motor[i].pos_rad;
			g_dm4310.hold_kp[i] = 0.5f;
			g_dm4310.hold_kd[i] = 0.05f;
			g_dm4310.feedforward_tau[i] = 0.0f;
		}
		g_dm4310.hold_updates = 1U;
		proto_send("OK:MODE:HOLD");
	} else {
		proto_send("ERR:MODE:use DRAG or HOLD");
	}
}

/* ---------- 帧解析 ---------- */

static void dispatch_frame(const char *frame)
{
	/* 提取命令部分 (到首个 ':' 或行尾) */
	char cmd[16];
	const char *args = frame;
	size_t cmd_len = 0;

	while (*args != '\0' && *args != ':') {
		if (cmd_len < sizeof(cmd) - 1) {
			cmd[cmd_len++] = *args;
		}
		args++;
	}
	cmd[cmd_len] = '\0';

	/* 跳过 ':' */
	if (*args == ':') args++;

	/* 派发命令 */
	if (strcmp(cmd, "MOVE") == 0) {
		handle_move(args);
	} else if (strcmp(cmd, "JOG_H") == 0) {
		handle_jog_h(args);
	} else if (strcmp(cmd, "JOG_PHI") == 0) {
		handle_jog_phi(args);
	} else if (strcmp(cmd, "STOP") == 0) {
		handle_stop();
	} else if (strcmp(cmd, "STATUS") == 0) {
		handle_status();
	} else if (strcmp(cmd, "STREAM") == 0) {
		handle_stream(args);
	} else if (strcmp(cmd, "MODE") == 0) {
		handle_mode(args);
	} else {
		proto_send("ERR:UNKNOWN_CMD:%s", cmd);
	}
}

/* ---------- 遥测输出 ---------- */

/**
 * @brief 输出一行遥测 (主循环 25Hz 调用)
 *
 * 格式: T:ts,h_L,phi_L,h_R,phi_R,m1,m2,m3,m4,t1,t2,t3,t4\n
 */
void proto_telemetry_tick(void)
{
	if (!stream_on) return;

	float m[4], tq[4];
	for (int i = 0; i < 4; i++) {
		m[i]  = g_dm4310.motor[i].pos_rad;
		tq[i] = g_dm4310.motor[i].torque_nm;
	}
	float ta_L = lk_m1_to_theta_a(m[0]);
	float tb_L = lk_m2_to_theta_b(m[1]);
	float ta_R = lk_m3_to_theta_a(m[2]);
	float tb_R = lk_m4_to_theta_b(m[3]);
	float h_L, phi_L, h_R, phi_R;
	lk_forward(ta_L, tb_L, NULL, &h_L, &phi_L);
	lk_forward(ta_R, tb_R, NULL, &h_R, &phi_R);

	proto_send("T:%u,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f",
		   k_uptime_get_32(),
		   (double)h_L, (double)(phi_L * 180.0 / M_PI),
		   (double)h_R, (double)(phi_R * 180.0 / M_PI),
		   (double)m[0], (double)m[1], (double)m[2], (double)m[3],
		   (double)tq[0], (double)tq[1], (double)tq[2], (double)tq[3]);
}

/* ================================================================
 *  工作线程
 * ================================================================ */

static void proto_thread_fn(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	char frame[PROTO_FRAME_MAX];

	while (1) {
		/* 阻塞等待下一帧 */
		int ret = k_msgq_get(&proto_msgq, frame, K_FOREVER);
		if (ret == 0) {
			dispatch_frame(frame);
		}
	}
}

#define PROTO_THREAD_STACK 1024
#define PROTO_THREAD_PRIO  5

K_THREAD_DEFINE(proto_thread, PROTO_THREAD_STACK,
		proto_thread_fn, NULL, NULL, NULL,
		PROTO_THREAD_PRIO, 0, 0);

/* ================================================================
 *  公开初始化
 * ================================================================ */

int proto_init(void)
{
	int ret = uart_init();
	if (ret < 0) return ret;

	/* 工作线程由 K_THREAD_DEFINE 在系统启动时自动创建,
	 * 此处仅初始化硬件 */
	stream_on = false;
	rx_pos = 0;

	printk("PROTO: service ready\n");
	return 0;
}

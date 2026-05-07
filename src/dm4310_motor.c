/**
 * @file dm4310_motor.c
 * @brief DM-J4310-2EC 电机 MIT 协议驱动实现
 *
 * 基于 Zephyr RTOS CAN 子系统实现的 DM4310 关节电机驱动。
 * 支持四台电机（CAN1: 电机1-2 左腿 / CAN2: 电机3-4 右腿），
 * MIT 位置控制协议，bringup 状态机，位置保持及零点标定。
 *
 * Bringup 流程（每台电机 staggered 执行，每 tick 只处理一台）：
 * Step 0: 写 MIT 模式寄存器（StdId 0x7FF）
 * Step 1: 发送 DISABLE（10 tick）
 * Step 2: 发送 ZERO（10 tick）
 * Step 3: 发送 ENABLE（10 tick）
 * 总计约 124 tick × 2ms = ~250ms
 *
 * MIT 控制律：
 *   torque = KP × (p_des - p_act) + KD × (v_des - v_act) + T_ff
 * KP=0, KD=0 时输出零力矩，电机可被外力自由拖动且不会回弹。
 */

#include "dm4310_motor.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

K_MSGQ_DEFINE(dm4310_can_rx_msgq, sizeof(struct can_frame), 64, 4);

#define DM4310_OUTPUT_ENABLED 1
#define DM4310_CAN1_HOME_ENABLED 0
#define DM4310_HOME_START_IDX 0U
#define DM4310_NORMAL_OUTPUT_ENABLED 1
#define DM4310_CAN1_HOME_USE_FLASH 0
#define DM4310_CAN1_HOME_NVS_ID 1U
#define DM4310_CAN1_HOME_MAGIC 0x4331484dU
#define DM4310_CAN1_HOME_VERSION 4U
#define DM4310_CAN1_HOME_ENABLE_TICKS 200U
#define DM4310_CAN1_HOME_MODE_TICKS 100U
#define DM4310_CAN1_HOLD_KP 100.0f
#define DM4310_CAN1_HOLD_KD 2.2f
#define DM4310_CAN2_HOLD_KP 100.0f
#define DM4310_CAN2_HOLD_KD 2.2f
#define DM4310_HOME_DEADBAND_RAD 0.015f
#define DM4310_CTRL_MODE_MIT 1U
#define DM4310_REG_CTRL_MODE 0x0AU
#define DM4310_REG_WRITE_ID 0x7FFU

struct dm4310_can1_home_record {
	uint32_t magic;
	uint32_t version;
	float pos_rad[DM4310_HOME_MOTOR_COUNT];
};

volatile struct dm4310_driver g_dm4310 = {
	.magic = DM4310_MAGIC,
};

/* GDB 命令队列，主循环消费 */
volatile uint8_t g_gdb_cmd[DM4310_MOTOR_COUNT];

static const struct device *can1_dev;
static const struct device *can2_dev;

static int dm4310_send_raw(uint16_t std_id, const uint8_t data[8]);

static inline const struct device *motor_idx_to_can(uint8_t motor_idx)
{
	return (motor_idx < DM4310_MOTORS_ON_CAN1) ? can1_dev : can2_dev;
}

static void put_le_u32(uint8_t data[4], uint32_t value)
{
	data[0] = (uint8_t)value;
	data[1] = (uint8_t)(value >> 8);
	data[2] = (uint8_t)(value >> 16);
	data[3] = (uint8_t)(value >> 24);
}

static int dm4310_write_u32_register(uint8_t motor_id, uint8_t reg, uint32_t value)
{
	uint8_t data[8];
	struct can_frame frame;
	int ret;

	data[0] = motor_id;
	data[1] = 0U;
	data[2] = 0x55U;
	data[3] = reg;
	put_le_u32(&data[4], value);

	memset(&frame, 0, sizeof(frame));
	frame.id = DM4310_REG_WRITE_ID;
	frame.dlc = 8;
	memcpy(frame.data, data, 8);

	ret = can_send(motor_idx_to_can(motor_id - 1U), &frame, K_MSEC(2), NULL, NULL);
	g_dm4310.last_send_ret = ret;

	return ret;
}

static uint8_t dm4310_home_motors_online(void)
{
	for (int motor = 0; motor < DM4310_HOME_MOTOR_COUNT; motor++) {
		if (g_dm4310.motor[motor].online == 0U) {
			return 0U;
		}
	}
	return 1U;
}

static float dm4310_home_kp(uint8_t motor_idx)
{
	return motor_idx < DM4310_MOTORS_ON_CAN1 ? DM4310_CAN1_HOLD_KP : DM4310_CAN2_HOLD_KP;
}

static float dm4310_home_kd(uint8_t motor_idx)
{
	return motor_idx < DM4310_MOTORS_ON_CAN1 ? DM4310_CAN1_HOLD_KD : DM4310_CAN2_HOLD_KD;
}

static uint8_t dm4310_home_in_deadband(uint8_t motor_idx)
{
	float target = g_dm4310.can1_home_pos_rad[motor_idx];
	float actual = g_dm4310.motor[motor_idx].pos_rad;

	return fabsf(target - actual) < DM4310_HOME_DEADBAND_RAD ? 1U : 0U;
}

static float clampf(float v, float lo, float hi)
{
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span, offset, scaled;
	uint32_t levels;

	x = clampf(x, x_min, x_max);
	span = x_max - x_min;
	offset = x - x_min;
	levels = ((uint32_t)1u << (uint32_t)bits) - 1u;
	scaled = offset * (float)levels / span;
	return (uint16_t)(scaled + 0.5f);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span;
	uint32_t levels;

	span = x_max - x_min;
	levels = ((uint32_t)1u << (uint32_t)bits) - 1u;
	return ((float)x_int * span / (float)levels) + x_min;
}

/**
 * @brief 打包 MIT 协议特殊指令帧
 *
 * 前 7 字节填充 0xFF，第 8 字节为指令尾字节。
 * 用于 DISABLE(0xFD)、ENABLE(0xFC)、ZERO(0xFE) 等非数值控制指令。
 *
 * @param tail 指令尾字节
 * @param[out] data 8 字节帧缓冲区
 */
void dm4310_pack_special(uint8_t tail, uint8_t data[8])
{
	memset(data, 0xFF, 7);
	data[7] = tail;
}

/**
 * @brief 打包 MIT 协议位置控制帧
 *
 * 将浮点控制参数按 MIT 协议位宽编码为 8 字节 CAN 帧：
 * - 位置 (16-bit): data[0:1]
 * - 速度 (12-bit): data[2:3] 高 4 位
 * - KP   (12-bit): data[3] 低 4 位 + data[4]
 * - KD   (12-bit): data[5:6] 高 4 位
 * - 力矩 (12-bit): data[6] 低 4 位 + data[7]
 *
 * @param pos 目标位置（弧度, -12.5 ~ +12.5）
 * @param vel 目标速度（弧度/秒, -30 ~ +30）
 * @param kp  位置刚度（0 ~ 500）
 * @param kd  速度阻尼（0 ~ 5）
 * @param tor 前馈力矩（Nm, -10 ~ +10）
 * @param[out] data 8 字节帧缓冲区
 */
void dm4310_pack_control(float pos, float vel, float kp, float kd, float tor,
			 uint8_t data[8])
{
	uint16_t p_int, v_int, kp_int, kd_int, t_int;

	p_int = float_to_uint(pos, DM4310_P_MIN, DM4310_P_MAX, 16);
	v_int = float_to_uint(vel, DM4310_V_MIN, DM4310_V_MAX, 12);
	kp_int = float_to_uint(kp, DM4310_KP_MIN, DM4310_KP_MAX, 12);
	kd_int = float_to_uint(kd, DM4310_KD_MIN, DM4310_KD_MAX, 12);
	t_int = float_to_uint(tor, DM4310_T_MIN, DM4310_T_MAX, 12);

	data[0] = (uint8_t)(p_int >> 8);
	data[1] = (uint8_t)(p_int & 0xFFU);
	data[2] = (uint8_t)(v_int >> 4);
	data[3] = (uint8_t)(((v_int & 0x0FU) << 4) | (uint8_t)(kp_int >> 8));
	data[4] = (uint8_t)(kp_int & 0xFFU);
	data[5] = (uint8_t)(kd_int >> 4);
	data[6] = (uint8_t)(((kd_int & 0x0FU) << 4) | (uint8_t)(t_int >> 8));
	data[7] = (uint8_t)(t_int & 0xFFU);
}

/**
 * @brief 解码 DM4310 反馈帧
 *
 * 解析 8 字节 CAN 数据帧为电机状态结构体。
 * 反馈帧布局（实际协议，非 datasheet 描述）：
 * - D0[3:0] = 电机 ID
 * - D0[7:4] = 电机状态
 * - D1-D2 = 位置（16-bit 大端）
 * - D3-D4[7:4] = 速度（12-bit）
 * - D4[3:0]-D5 = 力矩（12-bit）
 * - D6 = MOS 温度
 * - D7 = 线圈温度
 *
 * @param data 8 字节 CAN 帧数据
 * @param[out] out 解码后的电机状态
 * @return true 解码成功，false ID 无效或超出范围
 */
bool dm4310_decode_feedback(const uint8_t data[8],
			    struct dm4310_motor_status *out)
{
	uint8_t motor_id;
	int p_int, v_int, t_int;

	motor_id = (uint8_t)(data[0] & 0x0FU);
	if (motor_id == 0U || motor_id > DM4310_MOTOR_COUNT) {
		return false;
	}

	p_int = ((int)data[1] << 8) | (int)data[2];
	v_int = ((int)data[3] << 4) | ((int)data[4] >> 4);
	t_int = (((int)data[4] & 0x0F) << 8) | (int)data[5];

	out->motor_state = (uint8_t)(data[0] >> 4);
	out->pos_rad = uint_to_float(p_int, DM4310_P_MIN, DM4310_P_MAX, 16);
	out->vel_radps = uint_to_float(v_int, DM4310_V_MIN, DM4310_V_MAX, 12);
	out->torque_nm = uint_to_float(t_int, DM4310_T_MIN, DM4310_T_MAX, 12);
	out->mos_temp = data[6];
	out->coil_temp = data[7];
	out->rx_count++;
	out->last_ms = k_uptime_get_32();
	out->online = 1U;

	return true;
}

static int dm4310_send_raw(uint16_t std_id, const uint8_t data[8])
{
	struct can_frame frame;
	int ret;
	uint8_t motor_idx = (uint8_t)(std_id - DM4310_CAN_TX_ID_BASE);

	memset(&frame, 0, sizeof(frame));
	frame.id = std_id;
	frame.dlc = 8;
	memcpy(frame.data, data, 8);

	ret = can_send(motor_idx_to_can(motor_idx), &frame, K_MSEC(2), NULL, NULL);
	g_dm4310.last_send_ret = ret;

	return ret;
}

/**
 * @brief 从消息队列取出所有待处理 CAN 帧并解析
 *
 * 跳过寄存器读写/保存响应帧（data[2] = 0x55/0x33/0xAA），
 * 其余帧作为电机反馈解码并更新对应电机状态。
 */
static void drain_rx(void)
{
	struct can_frame frame;

	while (k_msgq_get(&dm4310_can_rx_msgq, &frame, K_NO_WAIT) == 0) {
		struct dm4310_motor_status status = { 0 };
		uint8_t mid;

		/* M3508/M2006 反馈帧 CAN ID 在 0x200-0x2FF 范围，必须滤掉，
		 * 否则 data[0] 碰巧落入 1-4 会覆盖 DM4310 电机状态导致跳变。
		 * DM4310 反馈帧 ID 不受此限制（可能是 0x00, 0x11+, 1-4 等）。 */
		if (frame.id >= 0x200U && frame.id <= 0x2FFU) {
			continue;
		}

		if (frame.data[2] == 0x55U || frame.data[2] == 0x33U || frame.data[2] == 0xAAU) {
			continue;
		}

		if (!dm4310_decode_feedback(frame.data, &status)) {
			continue;
		}

		mid = (uint8_t)(frame.data[0] & 0x0FU);
		if (mid >= 1U && mid <= DM4310_MOTOR_COUNT) {
			g_dm4310.motor[mid - 1U] = status;
		}
	}
}

/**
 * @brief 刷新电机在线位掩码
 *
 * 检查每台电机：从未收到反馈或距上次反馈超过 DM4310_ONLINE_TIMEOUT_MS
 * 则标记为离线。更新 g_dm4310.online_mask 位掩码。
 */
static void refresh_online_mask(void)
{
	uint32_t mask = 0;
	uint32_t now = k_uptime_get_32();

	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		if (g_dm4310.motor[i].rx_count == 0U ||
		    (now - g_dm4310.motor[i].last_ms) > DM4310_ONLINE_TIMEOUT_MS) {
			g_dm4310.motor[i].online = 0;
		}
		if (g_dm4310.motor[i].online) {
			mask |= BIT(i);
		}
	}
	g_dm4310.online_mask = mask;
}

static int dm4310_init_bus(const struct device *dev)
{
	const struct can_filter filter = {
		.id = 0x000U,
		.mask = 0x000U,
		.flags = 0,
	};
	int ret;

	ret = can_stop(dev);
	if (ret < 0 && ret != -EALREADY && ret != -ENETDOWN) {
		return ret;
	}

	ret = can_set_mode(dev, CAN_MODE_ONE_SHOT);
	if (ret < 0) {
		return ret;
	}

	ret = can_start(dev);
	if (ret < 0) {
		return ret;
	}

	ret = can_add_rx_filter_msgq(dev, &dm4310_can_rx_msgq, &filter);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/**
 * @brief 初始化 DM4310 驱动及双路 CAN 总线
 *
 * 初始化 CAN1 和 CAN2（1Mbps, ONE_SHOT 模式, 全通滤波器），
 * 所有电机默认 KP=90, KD=1.8。
 *
 * @return 0 成功，-ENODEV 表示 CAN 设备未就绪，其他负值表示 CAN 配置失败
 */
int dm4310_init(void)
{
	memset((void *)&g_dm4310, 0, sizeof(g_dm4310));
	g_dm4310.magic = DM4310_MAGIC;

	can1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));
	if (!device_is_ready(can1_dev)) {
		g_dm4310.init_ret = -ENODEV;
		return -ENODEV;
	}

	can2_dev = DEVICE_DT_GET(DT_NODELABEL(can2));
	if (!device_is_ready(can2_dev)) {
		g_dm4310.init_ret = -ENODEV;
		return -ENODEV;
	}

	g_dm4310.init_ret = dm4310_init_bus(can1_dev);
	if (g_dm4310.init_ret < 0) {
		return g_dm4310.init_ret;
	}

	g_dm4310.init_ret = dm4310_init_bus(can2_dev);
	if (g_dm4310.init_ret < 0) {
		return g_dm4310.init_ret;
	}

	g_dm4310.can1_home_load_ret = -ENOTSUP;
#if DM4310_CAN1_HOME_USE_FLASH != 0
	g_dm4310.can1_home_load_ret = dm4310_nvs_init();
	if (g_dm4310.can1_home_load_ret == 0) {
		g_dm4310.can1_home_load_ret = dm4310_load_can1_home();
	}
#endif

	g_dm4310.ready = 1U;
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		g_dm4310.hold_kp[i] = 90.0f;
		g_dm4310.hold_kd[i] = 1.8f;
	}

	return 0;
}

/**
 * @brief 轮询接收 CAN 反馈帧并更新在线状态
 *
 * 从消息队列中取出所有待处理 CAN 帧，解析电机反馈数据，
 * 更新各电机状态及在线掩码。
 */
void dm4310_poll_rx(void)
{
	drain_rx();
	refresh_online_mask();

#if DM4310_CAN1_HOME_ENABLED != 0
	if (g_dm4310.can1_home_valid == 0U && g_dm4310.can1_home_enable_ticks == 0U) {
		g_dm4310.can1_home_enable_ticks = DM4310_CAN1_HOME_MODE_TICKS +
						       DM4310_CAN1_HOME_ENABLE_TICKS;
	}

#endif
}

/**
 * @brief 执行一个控制 tick
 *
 * 根据当前状态分阶段执行：
 * - Bringup 阶段：每次处理一台电机的 bringup 步骤（MIT 模式 → DISABLE → ZERO → ENABLE），
 *   四台电机 staggered 推进，避免 CAN 总线拥塞。
 * - 正常控制阶段：每次发送全部四台电机的 MIT 位置控制帧（10ms 间隔）。
 *
 * 正常控制模式下，通过 hold_updates 标志判断是否有待发送的目标：
 * - hold_updates > 0：使用 hold_pos_rad + hold_kp/hold_kd 发送位置保持指令
 * - hold_updates == 0：发送全零控制帧（电机失能等待）
 *
 * @return 最后一次 CAN 发送的返回值
 */
int dm4310_tick(void)
{
	uint8_t id;
	uint8_t idx;
	uint32_t step;
	uint32_t tick;
	uint8_t data[8];
	int ret = 0;

	drain_rx();
	refresh_online_mask();

	#if DM4310_OUTPUT_ENABLED == 0
	dm4310_hold_reset();
	g_dm4310.loops++;
	return 0;
	#endif

#if DM4310_CAN1_HOME_ENABLED != 0
	if (g_dm4310.can1_home_valid != 0U) {
		g_dm4310.can1_home_active = 1U;
		for (int n = 0; n < DM4310_MOTOR_COUNT; n++) {
			if (g_dm4310.tx_index >= DM4310_MOTOR_COUNT) {
				g_dm4310.tx_index = 0;
			}
			idx = g_dm4310.tx_index;
			if (idx >= DM4310_HOME_START_IDX && idx < DM4310_HOME_MOTOR_COUNT) {
				if (g_dm4310.can1_home_enable_ticks > 0U) {
					dm4310_pack_special(DM4310_CMD_ENABLE_TAIL, data);
					g_dm4310.can1_home_enable_ticks--;
				} else if (dm4310_home_in_deadband(idx) != 0U) {
					dm4310_pack_control(g_dm4310.motor[idx].pos_rad, 0.0f,
							    0.0f, dm4310_home_kd(idx), 0.0f, data);
				} else {
					dm4310_pack_control(g_dm4310.can1_home_pos_rad[idx], 0.0f,
							    dm4310_home_kp(idx), dm4310_home_kd(idx), 0.0f, data);
				}
			} else {
				dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
			}
			ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
			g_dm4310.tx_index++;
		}
		g_dm4310.loops++;
		return ret;
	}

	for (int n = 0; n < DM4310_MOTOR_COUNT; n++) {
		if (g_dm4310.tx_index >= DM4310_MOTOR_COUNT) {
			g_dm4310.tx_index = 0;
		}
		idx = g_dm4310.tx_index;
		if (idx >= DM4310_HOME_START_IDX && idx < DM4310_HOME_MOTOR_COUNT) {
			if (g_dm4310.can1_home_enable_ticks > DM4310_CAN1_HOME_ENABLE_TICKS) {
				ret = dm4310_write_u32_register(idx + 1U, DM4310_REG_CTRL_MODE,
								DM4310_CTRL_MODE_MIT);
				g_dm4310.can1_home_enable_ticks--;
				g_dm4310.tx_index++;
				continue;
			}
			dm4310_pack_special(DM4310_CMD_ENABLE_TAIL, data);
			if (g_dm4310.can1_home_enable_ticks > 0U) {
				g_dm4310.can1_home_enable_ticks--;
			}
		} else {
			dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
		}
		ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
		g_dm4310.tx_index++;
	}
	g_dm4310.loops++;
	return ret;
#endif

#if DM4310_NORMAL_OUTPUT_ENABLED == 0
	dm4310_hold_reset();
	for (int n = 0; n < DM4310_MOTOR_COUNT; n++) {
		if (g_dm4310.tx_index >= DM4310_MOTOR_COUNT) {
			g_dm4310.tx_index = 0;
		}
		idx = g_dm4310.tx_index;
		dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
		ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
		g_dm4310.tx_index++;
	}
	g_dm4310.loops++;
	return ret;
#endif

	if (g_dm4310.bringup_done) {
		/* 每 tick 发送全部 4 台电机，控制间隔 10ms（DM4310 要求 <5ms，20ms 间隔会导致抖动） */
		for (int n = 0; n < DM4310_MOTOR_COUNT; n++) {
			if (g_dm4310.tx_index >= DM4310_MOTOR_COUNT) {
				g_dm4310.tx_index = 0;
			}
			idx = g_dm4310.tx_index;
			id = idx + 1U;

			/* KP=0 且 KD=0：禁用电机（比 MIT 零力矩帧更彻底，避免电机内部默认参数导致抖动） */
			if (g_dm4310.hold_kp[idx] == 0.0f && g_dm4310.hold_kd[idx] == 0.0f) {
				dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
			} else if (g_dm4310.hold_updates > 0U) {
				dm4310_pack_control(g_dm4310.hold_pos_rad[idx],
						    0.0f, g_dm4310.hold_kp[idx],
						    g_dm4310.hold_kd[idx], 0.0f, data);
			} else {
				dm4310_pack_control(0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
						    data);
			}
			ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
			g_dm4310.tx_index++;
		}
		g_dm4310.loops++;
		return ret;
	}

	/* Bringup: MIT register write -> DISABLE -> ZERO -> ENABLE */
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		step = g_dm4310.bringup_step[i];
		tick = g_dm4310.bringup_tick[i];

		if (step >= 4U) {
			continue;
		}

		/* Step 0: write MIT mode register via StdId 0x7FF */
		if (step == 0U) {
			ret = dm4310_write_u32_register((uint8_t)(i + 1U),
				DM4310_REG_CTRL_MODE, DM4310_CTRL_MODE_MIT);
			g_dm4310.bringup_step[i] = 1U;
			g_dm4310.bringup_tick[i] = 0U;
			g_dm4310.loops++;
			return ret;
		}

		if (step == 1U) {
			dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
		} else if (step == 2U) {
			dm4310_pack_special(DM4310_CMD_ZERO_TAIL, data);
		} else {
			dm4310_pack_special(DM4310_CMD_ENABLE_TAIL, data);
		}

		ret = dm4310_send_raw(DM4310_CAN_TX_ID_BASE + (uint16_t)i, data);
		tick++;
		if (tick >= DM4310_BRINGUP_TICKS) {
			g_dm4310.bringup_step[i] = step + 1U;
			if (g_dm4310.bringup_step[i] >= 4U) {
				continue;
			}
			g_dm4310.bringup_tick[i] = 0U;
		} else {
			g_dm4310.bringup_tick[i] = tick;
		}
		g_dm4310.loops++;
		return ret;
	}

	/*
	 * 检查所有电机是否完成 bringup。
	 * 注意：最后一个电机 ENABLE 完成后，需额外一个 tick 才能在这里检测到并设置 bringup_done。
	 */
	uint8_t all_done = 1U;
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		if (g_dm4310.bringup_step[i] < 4U) {
			all_done = 0U;
			break;
		}
	}
	g_dm4310.bringup_done = all_done;
	g_dm4310.loops++;
	return ret;
}

int dm4310_save_can1_home_current(void)
{
#if DM4310_CAN1_HOME_USE_FLASH != 0
	struct dm4310_can1_home_record record = {
		.magic = DM4310_CAN1_HOME_MAGIC,
		.version = DM4310_CAN1_HOME_VERSION,
	};
	struct dm4310_can1_home_record old_record;
	int ret;

	if (dm4310_nvs_ready == 0U) {
		ret = dm4310_nvs_init();
		if (ret < 0) {
			g_dm4310.can1_home_save_ret = ret;
			return ret;
		}
	}

	if (dm4310_home_motors_online() == 0U) {
		g_dm4310.can1_home_save_ret = -ENOTCONN;
		return g_dm4310.can1_home_save_ret;
	}

	ret = flash_read(dm4310_nvs.flash_device, dm4310_nvs.offset,
			 &old_record, sizeof(old_record));
	if (ret == 0 && old_record.magic == DM4310_CAN1_HOME_MAGIC &&
	    old_record.version == DM4310_CAN1_HOME_VERSION) {
		for (int motor = 0; motor < DM4310_HOME_MOTOR_COUNT; motor++) {
			record.pos_rad[motor] = old_record.pos_rad[motor];
		}
	} else {
		for (int motor = 0; motor < DM4310_HOME_MOTOR_COUNT; motor++) {
			record.pos_rad[motor] = g_dm4310.motor[motor].pos_rad;
		}
	}

	for (int motor = 0; motor < DM4310_HOME_MOTOR_COUNT; motor++) {
		record.pos_rad[motor] = g_dm4310.motor[motor].pos_rad;
	}

	ret = flash_erase(dm4310_nvs.flash_device, dm4310_nvs.offset, dm4310_storage_sector_size);
	if (ret < 0) {
		g_dm4310.can1_home_save_ret = ret;
		return ret;
	}

	ret = flash_write(dm4310_nvs.flash_device, dm4310_nvs.offset, &record, sizeof(record));
	if (ret < 0) {
		g_dm4310.can1_home_save_ret = ret;
		return ret;
	}

	for (int motor = 0; motor < DM4310_HOME_MOTOR_COUNT; motor++) {
		g_dm4310.can1_home_pos_rad[motor] = record.pos_rad[motor];
	}
	g_dm4310.can1_home_valid = 1U;
	g_dm4310.can1_home_enable_ticks = DM4310_CAN1_HOME_MODE_TICKS +
					       DM4310_CAN1_HOME_ENABLE_TICKS;
	g_dm4310.can1_home_active = 1U;
	g_dm4310.can1_home_auto_saved = 1U;
	g_dm4310.can1_home_save_ret = 0;

	return 0;
#else
	ARG_UNUSED(g_dm4310);
	g_dm4310.can1_home_save_ret = -ENOTSUP;
	return -ENOTSUP;
#endif
}

int dm4310_hold_positions(const float target[DM4310_MOTOR_COUNT])
{
#if DM4310_NORMAL_OUTPUT_ENABLED == 0
	ARG_UNUSED(target);
	g_dm4310.hold_updates = 0U;
	return -EACCES;
#else
	/*
	 * 使用饱和赋值而非自增，避免 uint32_t 溢出归零时
	 * 产生一帧全零控制命令导致电机瞬间失能。
	 */
	g_dm4310.hold_updates = 1U;
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		g_dm4310.hold_pos_rad[i] = target[i];
	}
	return 0;
#endif
}

void dm4310_hold_reset(void)
{
	g_dm4310.hold_updates = 0;
	memset((void *)g_dm4310.hold_pos_rad, 0, sizeof(g_dm4310.hold_pos_rad));
}

/**
 * @brief 紧急停止所有电机
 *
 * 向四台电机发送 DISABLE 指令并清除位置保持目标。
 * 用于异常停机或安全保护场景。
 */
void dm4310_stop_all(void)
{
	uint8_t data[8];

	dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
	for (int i = 0; i < DM4310_MOTOR_COUNT; i++) {
		(void)dm4310_send_raw(DM4310_CAN_TX_ID_BASE + (uint16_t)i, data);
	}
	dm4310_hold_reset();
}

int dm4310_enable_motor(uint8_t motor_id)
{
	if (motor_id < 1U || motor_id > DM4310_MOTOR_COUNT) {
		return -EINVAL;
	}
	uint8_t idx = motor_id - 1U;
	g_dm4310.hold_kp[idx] = 0.01f;
	g_dm4310.hold_kd[idx] = 0.001f;
	g_dm4310.hold_pos_rad[idx] = g_dm4310.motor[idx].pos_rad;
	g_dm4310.hold_updates = 1U;

	uint8_t data[8];
	dm4310_pack_special(DM4310_CMD_ENABLE_TAIL, data);
	return dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
}

int dm4310_disable_motor(uint8_t motor_id)
{
	if (motor_id < 1U || motor_id > DM4310_MOTOR_COUNT) {
		return -EINVAL;
	}
	uint8_t idx = motor_id - 1U;
	g_dm4310.hold_kp[idx] = 0.0f;
	g_dm4310.hold_kd[idx] = 0.0f;

	uint8_t data[8];
	dm4310_pack_special(DM4310_CMD_DISABLE_TAIL, data);
	return dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
}

int dm4310_zero_motor(uint8_t motor_id)
{
	if (motor_id < 1U || motor_id > DM4310_MOTOR_COUNT) {
		return -EINVAL;
	}
	uint8_t idx = motor_id - 1U;
	g_dm4310.hold_kp[idx] = 0.01f;
	g_dm4310.hold_kd[idx] = 0.001f;
	g_dm4310.hold_pos_rad[idx] = 0.0f;

	uint8_t data[8];
	dm4310_pack_special(DM4310_CMD_ZERO_TAIL, data);
	return dm4310_send_raw(DM4310_CAN_TX_ID_BASE + idx, data);
}

bool dm4310_is_online(uint8_t motor_id)
{
	if (motor_id < 1U || motor_id > DM4310_MOTOR_COUNT) {
		return false;
	}
	return g_dm4310.motor[motor_id - 1U].online != 0U;
}

const volatile struct dm4310_motor_status *dm4310_get(uint8_t motor_id)
{
	if (motor_id < 1U || motor_id > DM4310_MOTOR_COUNT) {
		return NULL;
	}
	return &g_dm4310.motor[motor_id - 1U];
}

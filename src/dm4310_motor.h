/**
 * @file dm4310_motor.h
 * @brief DM-J4310-2EC 电机 MIT 协议驱动
 *
 * 基于 Zephyr RTOS 的 DM4310 关节电机 CAN 总线驱动。
 * 支持四台电机（CAN1 两台 + CAN2 两台），实现 MIT 位置控制协议，
 * 包含上电 bringup 序列、位置保持控制、零点标定等功能。
 *
 * MIT 协议数值范围：
 * - 位置：-12.5 ~ +12.5 rad
 * - 速度：-30.0 ~ +30.0 rad/s
 * - KP：0.0 ~ 500.0
 * - KD：0.0 ~ 5.0
 * - 力矩：-10.0 ~ +10.0 Nm
 *
 * @note 控制模式：MIT 位置控制（寄存器 0x0A = 1）
 * @note CAN 波特率：1 Mbps
 */

#ifndef DM4310_MOTOR_H_
#define DM4310_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

/** @brief 驱动魔术字，用于运行时校验结构体完整性 */
#define DM4310_MAGIC 0x444d3433

/** @brief 电机总数 */
#define DM4310_MOTOR_COUNT 4

/** @brief 零点标定涉及的电机数 */
#define DM4310_HOME_MOTOR_COUNT 4

/** @brief 控制帧 CAN ID 基址（CAN ID = BASE + motor_index）
 *  M1..M4 对应 CAN ID 5..8 (避开 3508 的 1..4) */
#define DM4310_CAN_TX_ID_BASE 5U

/** @brief 反馈帧 Master ID 基址（建议 Master ID = CAN ID + 0x10, M1..M4 = 0x15..0x18） */
#define DM4310_MASTER_ID_BASE 0x15U

/** @brief CAN1 上的电机数量（左腿） */
#define DM4310_MOTORS_ON_CAN1  2U

/** @brief 电机在线超时时间（毫秒），超时未收到反馈则标记离线 */
#define DM4310_ONLINE_TIMEOUT_MS 500

/* === MIT 协议数值范围 === */

/** @brief 位置下限（弧度） */
#define DM4310_P_MIN  (-12.5f)
/** @brief 位置上限（弧度） */
#define DM4310_P_MAX  12.5f
/** @brief 速度下限（弧度/秒） */
#define DM4310_V_MIN  (-30.0f)
/** @brief 速度上限（弧度/秒） */
#define DM4310_V_MAX  30.0f
/** @brief KP 下限 */
#define DM4310_KP_MIN 0.0f
/** @brief KP 上限 */
#define DM4310_KP_MAX 500.0f
/** @brief KD 下限 */
#define DM4310_KD_MIN 0.0f
/** @brief KD 上限 */
#define DM4310_KD_MAX 5.0f
/** @brief 力矩下限（Nm） */
#define DM4310_T_MIN  (-10.0f)
/** @brief 力矩上限（Nm） */
#define DM4310_T_MAX  10.0f

/* === MIT 协议特殊指令尾字节 === */

/** @brief 电机使能指令尾字节 */
#define DM4310_CMD_ENABLE_TAIL  0xFC
/** @brief 电机禁用指令尾字节 */
#define DM4310_CMD_DISABLE_TAIL 0xFD
/** @brief 电机零点标定指令尾字节 */
#define DM4310_CMD_ZERO_TAIL    0xFE

/**
 * @brief Bringup 每步骤持续 tick 数
 *
 * 每 tick 向单台电机发送一帧指令，10 tick 确保电机充分响应。
 * 四台电机 staggered 执行，bringup 总耗时约 4×31×2ms ≈ 250ms。
 */
#define DM4310_BRINGUP_TICKS 10

/**
 * @brief 单台电机状态
 *
 * 由反馈帧解析填充，记录电机的实时位置、速度、力矩及温度信息。
 */
struct dm4310_motor_status {
	uint32_t rx_count;      /**< 累计接收反馈帧数 */
	uint32_t last_ms;       /**< 最后一次收到反馈的时间戳（毫秒） */
	uint8_t online;         /**< 在线标志（0 = 离线, 1 = 在线） */
	uint8_t motor_state;    /**< 电机状态码（反馈帧 D0[7:4]） */
	uint8_t mos_temp;       /**< MOS 管温度（摄氏度） */
	uint8_t coil_temp;      /**< 线圈温度（摄氏度） */
	float pos_rad;          /**< 当前位置（弧度） */
	float vel_radps;        /**< 当前速度（弧度/秒） */
	float torque_nm;        /**< 当前力矩（Nm） */
	uint8_t raw_frame[8];   /**< 最近一帧原始 CAN 数据 (诊断用) */
};

/**
 * @brief DM4310 驱动全局状态
 *
 * 所有电机控制均通过此结构体的全局实例 g_dm4310 进行。
 * 包含 bringup 状态机、位置保持目标及每台电机的实时反馈数据。
 */
struct dm4310_driver {
	uint32_t magic;         /**< 魔术字 DM4310_MAGIC */
	uint32_t ready;         /**< 驱动就绪标志 */
	int32_t init_ret;       /**< 初始化返回值 */
	int32_t last_send_ret;  /**< 最后一次 CAN 发送返回值 */
	uint32_t loops;         /**< 主循环计数器 */
	uint32_t online_mask;   /**< 在线电机位掩码（BIT(0) = 电机1, ...） */
	uint32_t bringup_step[DM4310_MOTOR_COUNT];  /**< 每台电机 bringup 当前步骤（0-3, ≥4=完成） */
	uint32_t bringup_tick[DM4310_MOTOR_COUNT];  /**< 每台电机 bringup 步骤内 tick 计数 */
	uint8_t bringup_done;   /**< Bringup 完成标志（0 = 未完成, 1 = 完成） */
	uint8_t tx_index;       /**< 轮询发送索引（staggered 发送用） */
	uint8_t can1_home_valid;    /**< CAN1 零点是否有效 */
	uint8_t can1_home_auto_saved; /**< CAN1 零点是否已自动保存 */
	uint8_t can1_home_active;    /**< CAN1 零点标定是否激活 */
	uint16_t can1_home_enable_ticks; /**< CAN1 零点使能剩余 tick */
	int32_t can1_home_load_ret;  /**< 零点加载返回值 */
	int32_t can1_home_save_ret;  /**< 零点保存返回值 */
	float can1_home_pos_rad[DM4310_HOME_MOTOR_COUNT]; /**< CAN1 零点位置（弧度） */
	uint32_t hold_updates;  /**< 位置保持更新计数（>0 表示有待发送的目标位置） */
	float hold_kp[DM4310_MOTOR_COUNT];   /**< 每台电机保持 KP */
	float hold_kd[DM4310_MOTOR_COUNT];   /**< 每台电机保持 KD */
	float hold_pos_rad[DM4310_MOTOR_COUNT]; /**< 每台电机目标位置（弧度） */
	float feedforward_tau[DM4310_MOTOR_COUNT]; /**< 力矩前馈 (Nm, 由运动学计算) */
	float balance_target_kp;          /**< 站立模式目标 KP (斜坡终点) */
	float balance_target_kd;          /**< 站立模式目标 KD (斜坡终点) */
	uint32_t balance_ramp_remaining;  /**< 站立模式增益爬坡剩余 tick */
	uint32_t balance_ramp_total;      /**< 站立模式增益爬坡总 tick (线性插值分母) */
	struct dm4310_motor_status motor[DM4310_MOTOR_COUNT]; /**< 四台电机状态 */
};

/** @brief DM4310 驱动全局实例 */
extern volatile struct dm4310_driver g_dm4310;

/** @brief 电机零点偏置 (robot cali 设置)，下发目标时叠加 */
extern float g_dm_offset[DM4310_MOTOR_COUNT];

/** @brief 平衡环 Pitch 零点偏置 (balance pitch_zero 设置) */
extern float g_balance_pitch_zero_rad;

/* GDB 调试命令队列 (主循环消费，可被 GDB set 触发) */
#define GDB_CMD_NONE    0
#define GDB_CMD_ENABLE  1
#define GDB_CMD_DISABLE 2
#define GDB_CMD_ZERO    3
extern volatile uint8_t g_gdb_cmd[DM4310_MOTOR_COUNT];

/**
 * @brief 初始化 DM4310 驱动
 *
 * 初始化 CAN1/CAN2 总线（1Mbps, ONE_SHOT 模式, 全通滤波器）。
 * 所有电机默认 KP=90, KD=1.8。
 *
 * @return 0 成功，负值表示错误码（-ENODEV: CAN 设备未就绪）
 */
int dm4310_init(void);

/**
 * @brief 轮询接收 CAN 反馈帧
 *
 * 从消息队列中取出所有待处理的 CAN 帧，解析电机反馈数据，
 * 更新各电机状态及在线掩码。
 */
void dm4310_poll_rx(void);

/**
 * @brief 执行一个控制 tick
 *
 * Bringup 阶段：每次处理一台电机的 bringup 步骤（MIT 模式 → DISABLE → ZERO → ENABLE）。
 * 正常控制阶段：每次 staggered 发送两台电机的 MIT 位置控制帧。
 *
 * @return 最后一次 CAN 发送的返回值
 */
int dm4310_tick(void);

/**
 * @brief 保存 CAN1 零点位置到 Flash（当前项目禁用）
 * @return 0 成功，负值错误码
 */
int dm4310_save_can1_home_current(void);

/**
 * @brief 重置位置保持状态（清零所有目标位置和更新计数）
 */
void dm4310_hold_reset(void);

/**
 * @brief 紧急停止所有电机（向四台电机发送 DISABLE 指令）
 */
void dm4310_stop_all(void);

/**
 * @brief 使能单台电机（发送 ENABLE 指令 + 设置极小 hold 增益）
 * @param motor_id 电机 ID (1-4)
 * @return CAN 发送返回值，0 成功
 */
int dm4310_enable_motor(uint8_t motor_id);

/**
 * @brief 失能单台电机（发送 DISABLE 指令 + 清零 hold 增益）
 * @param motor_id 电机 ID (1-4)
 * @return CAN 发送返回值，0 成功
 */
int dm4310_disable_motor(uint8_t motor_id);

/**
 * @brief 设置单台电机零点（发送 ZERO 指令）
 * @param motor_id 电机 ID (1-4)
 * @return CAN 发送返回值，0 成功
 */
int dm4310_zero_motor(uint8_t motor_id);

/**
 * @brief 查询指定电机是否在线
 * @param motor_id 电机 ID（1-4）
 * @return true 在线，false 离线或 ID 无效
 */
bool dm4310_is_online(uint8_t motor_id);

/**
 * @brief 获取指定电机的状态指针
 * @param motor_id 电机 ID（1-4）
 * @return 电机状态结构体指针，ID 无效时返回 NULL
 */
const volatile struct dm4310_motor_status *dm4310_get(uint8_t motor_id);

/**
 * @brief 下发单台电机目标位置（叠加零点偏置 + 物理限幅）
 * @param motor_id  电机 ID（1-4）
 * @param target_kin 运动学目标位置（弧度），内部自动叠加 g_dm_offset
 * @return 0 成功，-EINVAL 表示 ID 无效
 */
int dm4310_set_pos_with_offset(uint8_t motor_id, float target_kin);

/**
 * @brief 设置单台电机力矩前馈
 * @param motor_id  电机 ID（1-4）
 * @param tau_nm    前馈力矩 (Nm)
 */
void dm4310_set_feedforward_tau(uint8_t motor_id, float tau_nm);

/**
 * @brief 获取单台电机当前位置保持目标
 * @param motor_id  电机 ID（1-4）
 * @return 目标位置 (rad)
 */
float dm4310_get_hold_pos(uint8_t motor_id);

/**
 * @brief 一键切入站立模式
 *
 * 将四台电机从拖动模式 (KP=0.01/KD=0.001) 切换到高刚度站立模式。
 * 以当前位置为初始目标，逐 tick 阶梯提升 KP/KD 至目标值 (默认 80/1.5)，
 * 避免增益突变导致过流或抖动。
 *
 * @param ramp_ticks  增益爬坡 tick 数（0=瞬切，建议 50-100）
 * @return 0 成功
 */
int dm4310_balance_enable(uint32_t ramp_ticks);

/* === 协议编解码辅助函数（暴露供测试使用） === */

void dm4310_pack_special(uint8_t tail, uint8_t data[8]);
void dm4310_pack_control(float pos, float vel, float kp, float kd, float tor, uint8_t data[8]);

#endif

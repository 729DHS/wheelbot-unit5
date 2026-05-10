/**
 * @file leg_control.h
 * @brief 腿足控制胶水层: 运动学 IK → 电机目标下发
 *
 * 使用 linkage_kinematics 的 lk_inverse() 解算关节角，
 * 通过 dm4310_set_pos_with_offset() 下发给 DM 电机（自动叠加 g_dm_offset）。
 *
 * 偏移策略: 不使用 lk_motor_a/b()，偏移量由固件 g_dm_offset 统一管理，
 * 通过 robot cali 标定。Kinematics_Config 用于仿真/显示，固件不依赖。
 *
 * 电机映射:
 *   左腿: M1=θa, M2=θb  (CAN1)
 *   右腿: M3=θa, M4=θb  (CAN2, φ 符号镜像)
 */
#ifndef LEG_CONTROL_H_
#define LEG_CONTROL_H_

#include <stdint.h>

/**
 * @brief 左腿移动到指定高度和角度 (M1+M2)
 * @param h_mm    末端到原点距离 (mm, 20.6~235.4)
 * @param phi_rad OP7 与垂直向下方向的夹角 (rad, φ=0 垂直向下)
 * @return 0 成功，负值 = 运动学错误码
 */
int leg_move_to_left(float h_mm, float phi_rad);

/**
 * @brief 右腿移动到指定高度和角度 (M3+M4)
 * @note 右腿 φ 方向为镜像对称，内部自动取反
 */
int leg_move_to_right(float h_mm, float phi_rad);

/**
 * @brief 双腿同时移动到相同高度和角度
 */
int leg_move_all(float h_mm, float phi_rad);

/**
 * @brief 直接设置腿位置 (无增量限制, 用于 Shell 调试)
 * @param h_mm    足端距离 (mm)
 * @param phi_rad 摆动角 (rad)
 * @return 0 成功, 负值错误
 */
int leg_set_left(float h_mm, float phi_rad);
int leg_set_right(float h_mm, float phi_rad);

void leg_init_prev_left(float ta, float tb);
void leg_init_prev_right(float ta, float tb);

void leg_diag(float h_mm, float phi_rad);

#endif /* LEG_CONTROL_H_ */

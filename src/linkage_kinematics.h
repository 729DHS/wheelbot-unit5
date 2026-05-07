/**
 * @file linkage_kinematics.h
 * @brief 五连杆正逆运动学解析解 (等价 2R 机械臂)
 *
 * L1 = 107.4 mm  (O→P2, 第一等效连杆长)
 * L2 = 128.0 mm  (P2→P7, 第二等效连杆长)
 */
#ifndef LINKAGE_KINEMATICS_H_
#define LINKAGE_KINEMATICS_H_

#define LK_L1_MM 107.4f
#define LK_L2_MM 128.0f
#define LK_WORKSPACE_MIN_MM 20.6f
#define LK_WORKSPACE_MAX_MM 235.4f

/* 逆解肘部配置 */
#define LK_ELBOW_DOWN (-1)
#define LK_ELBOW_UP    1

struct lk_pose {
	float x_mm;
	float y_mm;
};

struct lk_joint_angles {
	float theta_a_rad;
	float theta_b_rad;
};

/**
 * @brief 正运动学: 关节角 → 末端位置
 * @param angles  等效关节角 (theta_a, theta_b) in rad
 * @param out     末端 P7 坐标 (mm)
 */
void lk_forward(const struct lk_joint_angles *angles, struct lk_pose *out);

/**
 * @brief 逆运动学: 末端位置 → 关节角
 * @param target  目标末端坐标 P7 (mm)
 * @param elbow   LK_ELBOW_UP 或 LK_ELBOW_DOWN
 * @param out     解出的关节角 (theta_a, theta_b) in rad
 * @return 0 成功, -1 不可达 (超出工作空间)
 */
int lk_inverse(const struct lk_pose *target, int elbow, struct lk_joint_angles *out);

#endif

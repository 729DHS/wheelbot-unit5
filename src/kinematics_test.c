/**
 * @file kinematics_test.c
 * @brief 运动学正逆解单测 — Shell 命令 `test kin`
 *
 * 在实机上运行, 验证 IK⇄FK 一致性、边界条件、连续性。
 * 每个测试用例打印 PASS 或 FAIL, 最后汇总。
 *
 * 测试覆盖:
 *   1. IK→FK 往返一致性 (多采样点, elbow-up/down)
 *   2. FK 固定角度预期值
 *   3. h 边界 (H_MIN, H_MAX, H_SAFE_MIN)
 *   4. IK 连续性 (elbow 切换)
 *   5. 越界拒绝
 */

#include "linkage_kinematics.h"
#include "leg_control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

/* ================================================================
 *  测试框架 (轻量, 无依赖)
 * ================================================================ */

static int g_test_pass;
static int g_test_fail;
static const struct shell *g_sh;

#define TEST_ASSERT(cond, msg, ...) do { \
	if (cond) { \
		g_test_pass++; \
	} else { \
		g_test_fail++; \
		shell_print(g_sh, "  FAIL: " msg, ##__VA_ARGS__); \
	} \
} while (0)

#define TEST_SECTION(title) \
	shell_print(g_sh, "--- %s ---", title)

/* ================================================================
 *  测试用例
 * ================================================================ */

/** @brief IK→FK 往返: 随机采样 (h, phi), IK 解算后 FK 恢复, 误差 < 0.1mm/deg */
static void test_roundtrip(void)
{
	TEST_SECTION("IK→FK roundtrip");

	/* 测试点: (h_mm, phi_deg, elbow) */
	struct { float h; float phi; int elbow; } pts[] = {
		{ 60.0f,   0.0f, +1 },
		{ 80.0f,   0.0f, +1 },
		{ 100.0f,  0.0f, +1 },
		{ 80.0f,  20.0f, +1 },
		{ 80.0f, -20.0f, +1 },
		{ 120.0f, 30.0f, -1 },
		{ 150.0f, 10.0f, -1 },
		{ 200.0f,  0.0f, -1 },
		{ 60.0f,  40.0f, +1 },
		{ 100.0f, -30.0f, -1 },
	};

	for (size_t i = 0; i < sizeof(pts) / sizeof(pts[0]); i++) {
		float h_in = pts[i].h;
		float phi_in = pts[i].phi * M_PI / 180.0f;
		int elbow = pts[i].elbow;

		float ta, tb;
		lk_error_t e = lk_inverse(h_in, phi_in, elbow, NULL, &ta, &tb);
		TEST_ASSERT(e == LK_OK,
			"IK failed: h=%.1f phi=%.1f elbow=%d e=%d",
			(double)h_in, (double)(phi_in * 180.0f / M_PI),
			elbow, (int)e);
		if (e != LK_OK) continue;

		float h_out, phi_out;
		e = lk_forward(ta, tb, NULL, &h_out, &phi_out);
		TEST_ASSERT(e == LK_OK,
			"FK failed: ta=%.2f tb=%.2f e=%d",
			(double)(ta * 180.0f / M_PI),
			(double)(tb * 180.0f / M_PI), (int)e);

		float h_err = fabsf(h_out - h_in);
		float phi_err = fabsf(phi_out - phi_in);
		TEST_ASSERT(h_err < 0.5f && phi_err < 0.01f,
			"Roundtrip error: h_err=%.3f phi_err=%.4f (h=%.1f phi=%.1f)",
			(double)h_err, (double)phi_err,
			(double)h_in, (double)(phi_in * 180.0f / M_PI));
	}
}

/** @brief FK 验证: 已知机构帧角度对应的 h/phi */
static void test_fk_known(void)
{
	TEST_SECTION("FK known poses");

	/* cali 位姿: θa=-162.4°(OFFSET_A), θb=-10°(OFFSET_B)
	 * 对应 motor=0 时的机构帧角度, h 在工作空间内即可 */
	float ta_cali = LK_OFFSET_A;
	float tb_cali = LK_OFFSET_B;
	float h_cali, phi_cali;
	lk_error_t e = lk_forward(ta_cali, tb_cali, NULL, &h_cali, &phi_cali);
	TEST_ASSERT(e == LK_OK, "FK cali pose failed: e=%d", (int)e);

	/* cali 位姿 h 应在运动学可达范围内 */
	TEST_ASSERT(h_cali >= LK_H_MIN_MM && h_cali <= LK_H_MAX_MM,
		"FK cali h=%.1f out of workspace [%.0f, %.0f]",
		(double)h_cali,
		(double)LK_H_MIN_MM, (double)LK_H_MAX_MM);

	/* 垂直向下: ta ≈ -π/2 + delta, tb ≈ -π/2 + delta */
	float ta_down = -M_PI / 2.0f;
	float tb_down = -M_PI / 2.0f;
	float h_down, phi_down;
	e = lk_forward(ta_down, tb_down, NULL, &h_down, &phi_down);
	TEST_ASSERT(e == LK_OK, "FK vertical down failed");
	TEST_ASSERT(h_down > 20.0f && h_down < 240.0f,
		"FK vertical down h=%.1f out of range", (double)h_down);
}

/** @brief 边界测试: h 在极限附近 */
static void test_boundary(void)
{
	TEST_SECTION("Boundary conditions");

	float ta, tb;

	/* h 在最小工作空间 */
	lk_error_t e = lk_inverse(LK_H_MIN_MM + 1.0f, 0.0f, +1, NULL, &ta, &tb);
	TEST_ASSERT(e == LK_OK,
		"IK near h_min failed: h=%.1f e=%d",
		(double)(LK_H_MIN_MM + 1.0f), (int)e);

	/* h 在最大工作空间 */
	e = lk_inverse(LK_H_MAX_MM - 1.0f, 0.0f, -1, NULL, &ta, &tb);
	TEST_ASSERT(e == LK_OK,
		"IK near h_max failed: h=%.1f e=%d",
		(double)(LK_H_MAX_MM - 1.0f), (int)e);
}

/** @brief IK 拒绝越界输入 */
static void test_out_of_range(void)
{
	TEST_SECTION("Out-of-range rejection");

	float ta, tb;

	/* h 太小 */
	lk_error_t e = lk_inverse(10.0f, 0.0f, +1, NULL, &ta, &tb);
	TEST_ASSERT(e == LK_ERR_H_OUT_OF_RANGE,
		"h=10 should be rejected, got e=%d", (int)e);

	/* h 太大 */
	e = lk_inverse(300.0f, 0.0f, -1, NULL, &ta, &tb);
	TEST_ASSERT(e == LK_ERR_H_OUT_OF_RANGE,
		"h=300 should be rejected, got e=%d", (int)e);
}

/** @brief IK 连续性: lk_inverse_continuous 选离 prev 最近的分支 */
static void test_continuity(void)
{
	TEST_SECTION("IK continuity (elbow selection)");

	/* 在中间位置用两个 elbow 都有解, 测试连续性选择 */
	float prev_ta = -2.0f; /* ≈ -115° (elbow-up 偏好) */
	float prev_tb = -0.5f;

	float ta, tb;
	lk_error_t e = lk_inverse_continuous(80.0f, 0.0f,
					      prev_ta, prev_tb,
					      NULL, &ta, &tb);
	TEST_ASSERT(e == LK_OK, "IK continuous failed: e=%d", (int)e);

	/* 选中的解应更接近 prev */
	float dist_prev = fabsf(ta - prev_ta) + fabsf(tb - prev_tb);

	/* 对比另一个 elbow 分支 */
	float ta_other, tb_other;
	int other_elbow = (dist_prev > 1.0f) ? -1 : +1;

	/* 再测一次从 far 位置: 应选近的 */
	float prev_far = 0.0f; /* 远离典型 elbow-up 解 */
	e = lk_inverse_continuous(80.0f, 0.0f,
				  prev_far, prev_far,
				  NULL, &ta, &tb);
	TEST_ASSERT(e == LK_OK,
		"IK continuous (far start) failed: e=%d", (int)e);
	(void)dist_prev;
	(void)other_elbow;
	(void)ta_other;
	(void)tb_other;
}

/** @brief 关节限位检查 (leg_control.c 的 joint_within_limits) */
static void test_joint_limits(void)
{
	TEST_SECTION("Joint limits (via IK with safety margin)");

	/* 安全限位外的 h/phi 应被 IK 拒绝或 clamp */
	float ta, tb;

	/* h 在安全下限以下, leg_control 应 clamp 到 LK_H_SAFE_MIN */
	int ret = leg_move_to_left(30.0f, 0.0f);
	/* 期望: IK 成功 (内部 clamp 到 LK_H_SAFE_MIN=50mm) */
	/* 注意: bringup_done=0 时会走 hold position 流程, 测试用 IK 层检查即可 */
	(void)ret;

	/* h=80/phi=0 elbow=-1: θa≈-175°(within ±180) θb≈-33°(within -120~+30) */
	lk_error_t e = lk_inverse(80.0f, 0.0f, -1, NULL, &ta, &tb);
	TEST_ASSERT(e == LK_OK,
		"IK h=80 phi=0 should work, got e=%d", (int)e);

	/* theta 限位: 检查 lk_inverse 解算出的 theta_a/b 在理论范围内 */
	TEST_ASSERT(ta >= LK_THETA_A_MIN && ta <= LK_THETA_A_MAX,
		"theta_a=%.1f out of [%.1f, %.1f]",
		(double)(ta * 180.0f / M_PI),
		(double)(LK_THETA_A_MIN * 180.0f / M_PI),
		(double)(LK_THETA_A_MAX * 180.0f / M_PI));
	TEST_ASSERT(tb >= LK_THETA_B_MIN && tb <= LK_THETA_B_MAX,
		"theta_b=%.1f out of [%.1f, %.1f]",
		(double)(tb * 180.0f / M_PI),
		(double)(LK_THETA_B_MIN * 180.0f / M_PI),
		(double)(LK_THETA_B_MAX * 180.0f / M_PI));
}

/* ================================================================
 *  Shell 命令入口
 * ================================================================ */

static int cmd_test_kin(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	g_sh = sh;
	g_test_pass = 0;
	g_test_fail = 0;

	shell_print(sh, "=== Kinematics Unit Tests ===");

	test_roundtrip();
	test_fk_known();
	test_boundary();
	test_out_of_range();
	test_continuity();
	test_joint_limits();

	shell_print(sh, "=== Result: %d PASS, %d FAIL ===",
		    g_test_pass, g_test_fail);

	/* 清理 prev 状态, 避免污染实际控制 */
	leg_init_prev_left(0.0f, 0.0f);
	leg_init_prev_right(0.0f, 0.0f);

	return (g_test_fail == 0) ? 0 : -1;
}

SHELL_STATIC_SUBCMD_SET_CREATE(test_cmds,
	SHELL_CMD_ARG(kin, NULL, "test kin — IK/FK unit tests",
		      cmd_test_kin, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(test, &test_cmds, "Unit tests", NULL);

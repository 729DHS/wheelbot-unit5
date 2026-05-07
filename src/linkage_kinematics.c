#include "linkage_kinematics.h"

#include <math.h>

void lk_forward(const struct lk_joint_angles *angles, struct lk_pose *out)
{
	out->x_mm = LK_L1_MM * cosf(angles->theta_a_rad) +
		    LK_L2_MM * cosf(angles->theta_b_rad);
	out->y_mm = LK_L1_MM * sinf(angles->theta_a_rad) +
		    LK_L2_MM * sinf(angles->theta_b_rad);
}

int lk_inverse(const struct lk_pose *target, int elbow, struct lk_joint_angles *out)
{
	float r = sqrtf(target->x_mm * target->x_mm + target->y_mm * target->y_mm);
	float cos_alpha;
	float alpha;
	float theta_a, theta_b;

	if (r < LK_WORKSPACE_MIN_MM || r > LK_WORKSPACE_MAX_MM) {
		return -1;
	}

	cos_alpha = (r * r + LK_L1_MM * LK_L1_MM - LK_L2_MM * LK_L2_MM) /
		    (2.0f * LK_L1_MM * r);

	/* 浮点误差钳位 */
	if (cos_alpha > 1.0f) {
		cos_alpha = 1.0f;
	} else if (cos_alpha < -1.0f) {
		cos_alpha = -1.0f;
	}

	alpha = acosf(cos_alpha);

	if (elbow == LK_ELBOW_UP) {
		theta_a = atan2f(target->y_mm, target->x_mm) + alpha;
	} else {
		theta_a = atan2f(target->y_mm, target->x_mm) - alpha;
	}

	theta_b = atan2f(target->y_mm - LK_L1_MM * sinf(theta_a),
			  target->x_mm - LK_L1_MM * cosf(theta_a));

	out->theta_a_rad = theta_a;
	out->theta_b_rad = theta_b;

	return 0;
}

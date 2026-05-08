/**
 * linkage_kinematics.h — 2-DOF leg linkage inverse/forward kinematics
 *
 * Coordinate conventions (mechanism frame, +X right +Y up):
 *   O  — motor coaxial center (origin)
 *   P7 — wheel hub center (end-effector)
 *
 * API parameters:
 *   h   — distance |O->P7|, unit: mm
 *   phi — angle of vector OP7 measured clockwise from vertical-down,
 *          unit: rad.  phi=0 → leg points straight down.
 *
 *   theta_a — bar_a angle measured CCW from +X, unit: rad
 *   theta_b — bar_b angle measured CCW from +X, unit: rad
 *
 * Equivalent 2R arm model:
 *   P7 = L1*[cos(θa), sin(θa)] + L2*[cos(θb), sin(θb)]
 *   L1 = |O-P2| = 107.4 mm
 *   L2 = |P2-P7| = 128.0 mm
 *
 * Workspace: h ∈ [abs(L1-L2), L1+L2] = [20.6, 235.4] mm
 *
 * Motor target (DM motor zero = cad_angle_at_zero):
 *   motor_target_a = θa - offset_a
 *   motor_target_b = θb - offset_b
 */

#ifndef LINKAGE_KINEMATICS_H
#define LINKAGE_KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>
#include <arm_math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- constants ---------- */
#define LK_L1_MM   (107.4f)   /* |O->P2| */
#define LK_L2_MM   (128.0f)   /* |P2->P7| */

/* 理论工作空间 (运动学可达) */
#define LK_H_MIN_MM  (20.6f)  /* abs(L1 - L2) */
#define LK_H_MAX_MM  (235.4f) /* L1 + L2 */

/* 固定机械偏置: θ = LK_Mx_DIR * motor + OFFSET (机构帧, rad)
 * cali 位姿: motor=0 时 θa = LK_OFFSET_A, θb = LK_OFFSET_B
 *
 * 电机→关节映射: M1=θa_L, M2=θb_L, M3=θb_R, M4=θa_R (M3/M4 对调) */
#define LK_OFFSET_A  (-2.834415f)   /* -162.4 deg */
#define LK_OFFSET_B  (-0.174533f)   /*  -10.0 deg */

/* 逐电机编码器方向: M4 反向安装 */
#define LK_M1_DIR  ( 1.0f)
#define LK_M2_DIR  ( 1.0f)
#define LK_M3_DIR  ( 1.0f)
#define LK_M4_DIR  (-1.0f)

/* 左腿: M1→θa, M2→θb */
static inline float lk_m1_to_theta_a(float m) { return LK_M1_DIR * m + LK_OFFSET_A; }
static inline float lk_m2_to_theta_b(float m) { return LK_M2_DIR * m + LK_OFFSET_B; }
static inline float lk_theta_a_to_m1(float t) { return LK_M1_DIR * (t - LK_OFFSET_A); }
static inline float lk_theta_b_to_m2(float t) { return LK_M2_DIR * (t - LK_OFFSET_B); }

/* 右腿: M4→θa, M3→θb */
static inline float lk_m4_to_theta_a(float m) { return LK_M4_DIR * m + LK_OFFSET_A; }
static inline float lk_m3_to_theta_b(float m) { return LK_M3_DIR * m + LK_OFFSET_B; }
static inline float lk_theta_a_to_m4(float t) { return LK_M4_DIR * (t - LK_OFFSET_A); }
static inline float lk_theta_b_to_m3(float t) { return LK_M3_DIR * (t - LK_OFFSET_B); }

/* 关节空间限位 (机构帧, 编码器+offset后, rad) */
#define LK_THETA_A_MIN  (-3.141593f)  /* -180 deg */
#define LK_THETA_A_MAX  ( 0.349066f)  /*  +20 deg */
#define LK_THETA_B_MIN  (-2.094395f)  /* -120 deg */
#define LK_THETA_B_MAX  ( 0.523599f)  /*  +30 deg */

/* 任务空间软限位 (h/φ 空间) */
#define LK_H_SOFT_MIN     (45.0f)       /* mm, κ<5 近奇异区 */
#define LK_H_SOFT_MAX     (235.4f)      /* mm, L1+L2 奇异 */
#define LK_PHI_SOFT_MIN   (-1.047198f)  /* -60 deg */
#define LK_PHI_SOFT_MAX   ( 1.047198f)  /* +60 deg */

/* ---------- error codes ---------- */
typedef enum {
    LK_OK = 0,
    LK_ERR_H_OUT_OF_RANGE,   /* h outside [LK_H_MIN_MM, LK_H_MAX_MM] */
    LK_ERR_UNREACHABLE,      /* internal constraint violation */
} lk_error_t;

/* ---------- zero-offset configuration ---------- */
typedef struct {
    float theta_a_offset;  /* rad, cad_angle_at_zero for bar_a */
    float theta_b_offset;  /* rad, cad_angle_at_zero for bar_b */
} Kinematics_Config;

/* default offsets: left leg standing posture */
#define LK_DEFAULT_OFFSET_A  (-2.83442f) /* -162.4 deg */
#define LK_DEFAULT_OFFSET_B  (-0.17453f) /*  -10.0 deg */

static inline void lk_config_default(Kinematics_Config *cfg) {
    cfg->theta_a_offset = LK_DEFAULT_OFFSET_A;
    cfg->theta_b_offset = LK_DEFAULT_OFFSET_B;
}

/* ---------- inverse kinematics ---------- */
/**
 * IK: given (h, phi), compute theta_a, theta_b.
 *
 * h   — |O->P7| [mm],  clamped to [20.6, 235.4]
 * phi — OP7 angle clockwise from vertical-down [rad]
 * elbow — +1 for elbow-up, -1 for elbow-down
 * cfg — zero-offset config (may be NULL → use defaults)
 *
 * out_theta_a, out_theta_b — joint angles in mechanism frame [rad]
 *
 * Returns LK_OK on success, or error code.
 */
lk_error_t lk_inverse(float h, float phi, int elbow,
                      const Kinematics_Config *cfg,
                      float *out_theta_a, float *out_theta_b);

/**
 * IK with auto elbow selection: picks the solution closest to the
 * previous theta values (for continuity).
 */
lk_error_t lk_inverse_continuous(float h, float phi,
                                 float prev_theta_a, float prev_theta_b,
                                 const Kinematics_Config *cfg,
                                 float *out_theta_a, float *out_theta_b);

/* ---------- forward kinematics ---------- */
/**
 * FK: given motor angles, compute (h, phi) of the end-effector.
 *
 * theta_a, theta_b — joint angles in mechanism frame [rad]
 * cfg — zero-offset config (may be NULL → use defaults)
 *
 * out_h, out_phi — end-effector position in (h, phi) space
 *
 * Returns LK_OK on success.
 */
lk_error_t lk_forward(float theta_a, float theta_b,
                      const Kinematics_Config *cfg,
                      float *out_h, float *out_phi);

/* ---------- motor-target helpers ---------- */
/**
 * Convert mechanism-frame joint angles to DM motor position commands.
 * motor = theta - offset
 */
static inline float lk_motor_a(float theta_a, const Kinematics_Config *cfg) {
    return LK_M1_DIR * (theta_a - cfg->theta_a_offset);
}
static inline float lk_motor_b(float theta_b, const Kinematics_Config *cfg) {
    return LK_M2_DIR * (theta_b - cfg->theta_b_offset);
}

/* ---------- utility ---------- */
/** Clamp value to [lo, hi]. */
static inline float lk_clamp(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/** Wrap angle to [-pi, pi). */
static inline float lk_wrap_pi(float a) {
    while (a >=  M_PI) a -= 2.0f * M_PI;
    while (a <  -M_PI) a += 2.0f * M_PI;
    return a;
}

#ifdef __cplusplus
}
#endif

#endif /* LINKAGE_KINEMATICS_H */

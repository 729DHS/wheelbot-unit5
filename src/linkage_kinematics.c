/**
 * linkage_kinematics.c — implementation
 *
 * All trig functions use arm_math.h (CMSIS-DSP) for STM32F4 FPU acceleration.
 * arm_sin_f32 / arm_cos_f32 take radians, return [-1, 1].
 */

#include "linkage_kinematics.h"
#include <math.h>  /* acosf, atan2f, sqrtf, fabsf — not available in arm_math */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* --------------------------------------------------
 *  IK:  h, phi  →  theta_a, theta_b
 *
 *  In mechanism frame (+X right +Y up):
 *    vertical-down = direction (-π/2).
 *    phi measured clockwise from vertical-down.
 *    clockwise in standard math = negative.
 *
 *    angle(OP7) = -π/2 - phi
 *    P7 = h * (cos(angle), sin(angle))
 *       = h * (-sin(phi), -cos(phi))
 * -------------------------------------------------- */

lk_error_t lk_inverse(float h, float phi, int elbow,
                      const Kinematics_Config *cfg,
                      float *out_theta_a, float *out_theta_b)
{
    /* --- range check --- */
    if (h < LK_H_MIN_MM - 1e-4f || h > LK_H_MAX_MM + 1e-4f) {
        return LK_ERR_H_OUT_OF_RANGE;
    }

    const float L1 = LK_L1_MM;
    const float L2 = LK_L2_MM;

    /* P7 in mechanism frame: phi=0 → OP7 = (0, -h) */
    float s_phi, c_phi;
    s_phi = sinf(phi); c_phi = cosf(phi);
    float Px = -h * s_phi;
    float Py = -h * c_phi;

    float r = sqrtf(Px * Px + Py * Py);

    /* degenerate: r ≈ 0 */
    if (r < 1e-6f) {
        return LK_ERR_UNREACHABLE;
    }

    /* Cosine law: (r² + L1² - L2²) / (2·L1·r) */
    float r2 = r * r;
    float L1_2 = L1 * L1;
    float L2_2 = L2 * L2;
    float cos_alpha = (r2 + L1_2 - L2_2) / (2.0f * L1 * r);

    /* Clamp for float rounding safety */
    cos_alpha = lk_clamp(cos_alpha, -1.0f, 1.0f);
    float alpha = acosf(cos_alpha);

    float phi_vec = atan2f(Py, Px);

    /* Select elbow branch */
    float sign = (elbow >= 0) ? 1.0f : -1.0f;
    float theta_a = phi_vec + sign * alpha;
    theta_a = lk_wrap_pi(theta_a);

    /* theta_b from geometry */
    float sin_a, cos_a;
    sin_a = sinf(theta_a); cos_a = cosf(theta_a);
    float theta_b = atan2f(Py - L1 * sin_a,
                           Px - L1 * cos_a);

    *out_theta_a = theta_a;
    *out_theta_b = theta_b;

    (void)cfg; /* IK returns mechanism-frame angles, caller applies offset */
    return LK_OK;
}

lk_error_t lk_inverse_continuous(float h, float phi,
                                 float prev_theta_a, float prev_theta_b,
                                 const Kinematics_Config *cfg,
                                 float *out_theta_a, float *out_theta_b)
{
    float ta_up, tb_up, ta_dn, tb_dn;
    lk_error_t e_up = lk_inverse(h, phi, +1, cfg, &ta_up, &tb_up);
    lk_error_t e_dn = lk_inverse(h, phi, -1, cfg, &ta_dn, &tb_dn);

    if (e_up != LK_OK && e_dn != LK_OK) {
        return LK_ERR_UNREACHABLE;
    }
    if (e_up != LK_OK) {
        *out_theta_a = ta_dn; *out_theta_b = tb_dn;
        return LK_OK;
    }
    if (e_dn != LK_OK) {
        *out_theta_a = ta_up; *out_theta_b = tb_up;
        return LK_OK;
    }

    /* both valid → pick closest to previous */
    float da_up  = lk_wrap_pi(ta_up - prev_theta_a);
    float db_up  = lk_wrap_pi(tb_up - prev_theta_b);
    float da_dn  = lk_wrap_pi(ta_dn - prev_theta_a);
    float db_dn  = lk_wrap_pi(tb_dn - prev_theta_b);

    float dist_up = da_up * da_up + db_up * db_up;
    float dist_dn = da_dn * da_dn + db_dn * db_dn;

    if (dist_up <= dist_dn) {
        *out_theta_a = ta_up; *out_theta_b = tb_up;
    } else {
        *out_theta_a = ta_dn; *out_theta_b = tb_dn;
    }
    return LK_OK;
}

/* --------------------------------------------------
 *  FK:  theta_a, theta_b  →  h, phi
 *
 *  P7 = L1*[cos(θa), sin(θa)] + L2*[cos(θb), sin(θb)]
 *  h = |P7|
 *  phi = -atan2(P7_y, P7_x) - π/2   (clockwise from vertical-down)
 * -------------------------------------------------- */

lk_error_t lk_forward(float theta_a, float theta_b,
                      const Kinematics_Config *cfg,
                      float *out_h, float *out_phi)
{
    const float L1 = LK_L1_MM;
    const float L2 = LK_L2_MM;

    float sin_a, cos_a, sin_b, cos_b;
    sin_a = sinf(theta_a); cos_a = cosf(theta_a);
    arm_sin_cos_f32(theta_b, &sin_b, &cos_b);

    float Px = L1 * cos_a + L2 * cos_b;
    float Py = L1 * sin_a + L2 * sin_b;

    *out_h = sqrtf(Px * Px + Py * Py);

    /* angle(OP7) in math frame = atan2(Py, Px)
     * phi = -(angle + π/2)   (clockwise from vertical-down) */
    *out_phi = -atan2f(Py, Px) - (M_PI / 2.0f);
    *out_phi = lk_wrap_pi(*out_phi);

    (void)cfg;
    return LK_OK;
}

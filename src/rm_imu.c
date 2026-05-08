#include "rm_imu.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#define G_MPS2 9.80665f
#define RAD_TO_DEG 57.295779513f
#define MAHONY_KP 1.0f
#define MAHONY_KI 0.0f
#define GYRO_CALIB_SAMPLES 500
#define IMU_OFFSET_X_M -0.175f
#define IMU_OFFSET_Z_M -0.080f
#define IMU_PITCH_ALPHA_LPF_A 0.70f

volatile struct rm_imu_attitude g_rm_imu = {
	.magic = RM_IMU_MAGIC,
	.q = { 1.0f, 0.0f, 0.0f, 0.0f },
};

static const struct device *const accel_dev = DEVICE_DT_GET(DT_NODELABEL(bmi088_accel));
static const struct device *const gyro_dev = DEVICE_DT_GET(DT_NODELABEL(bmi088_gyro));

static uint32_t last_update_ms;
static float integral_fb[3];
static float last_pitch_gyro_rps;
static float pitch_alpha_lpf_rps2;

static float rm_sensor_value_to_float(const struct sensor_value *value)
{
	return (float)value->val1 + (float)value->val2 / 1000000.0f;
}

static float inv_sqrt(float x)
{
	return 1.0f / sqrtf(x);
}

static int mahony_update_6axis(float *q, float ax, float ay, float az, float gx, float gy,
			       float gz, float dt)
{
	float recip_norm;
	float q0q0;
	float q0q1;
	float q0q2;
	float q1q3;
	float q2q3;
	float q3q3;
	float halfvx;
	float halfvy;
	float halfvz;
	float halfex;
	float halfey;
	float halfez;
	float qa;
	float qb;
	float qc;

	if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
		return -ENODATA;
	}

	recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
	ax *= recip_norm;
	ay *= recip_norm;
	az *= recip_norm;

	q0q0 = q[0] * q[0];
	q0q1 = q[0] * q[1];
	q0q2 = q[0] * q[2];
	q1q3 = q[1] * q[3];
	q2q3 = q[2] * q[3];
	q3q3 = q[3] * q[3];

	halfvx = q1q3 - q0q2;
	halfvy = q0q1 + q2q3;
	halfvz = q0q0 - 0.5f + q3q3;
	halfex = ay * halfvz - az * halfvy;
	halfey = az * halfvx - ax * halfvz;
	halfez = ax * halfvy - ay * halfvx;

	if (MAHONY_KI > 0.0f) {
		integral_fb[0] += MAHONY_KI * halfex * dt;
		integral_fb[1] += MAHONY_KI * halfey * dt;
		integral_fb[2] += MAHONY_KI * halfez * dt;
		gx += integral_fb[0];
		gy += integral_fb[1];
		gz += integral_fb[2];
	} else {
		integral_fb[0] = 0.0f;
		integral_fb[1] = 0.0f;
		integral_fb[2] = 0.0f;
	}

	gx += MAHONY_KP * halfex;
	gy += MAHONY_KP * halfey;
	gz += MAHONY_KP * halfez;

	gx *= 0.5f * dt;
	gy *= 0.5f * dt;
	gz *= 0.5f * dt;
	qa = q[0];
	qb = q[1];
	qc = q[2];

	q[0] += -qb * gx - qc * gy - q[3] * gz;
	q[1] += qa * gx + qc * gz - q[3] * gy;
	q[2] += qa * gy - qb * gz + q[3] * gx;
	q[3] += qa * gz + qb * gy - qc * gx;

	recip_norm = inv_sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recip_norm;
	q[1] *= recip_norm;
	q[2] *= recip_norm;
	q[3] *= recip_norm;

	return 0;
}

static void update_euler(void)
{
	float *q = (float *)g_rm_imu.q;

	g_rm_imu.pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * RAD_TO_DEG;
	g_rm_imu.roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
			       2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * RAD_TO_DEG;
}

static int read_sensors(float accel[3], float gyro[3], float *temp_c)
{
	struct sensor_value acc_val[3];
	struct sensor_value gyr_val[3];
	struct sensor_value temp;
	int ret;

	ret = sensor_sample_fetch(accel_dev);
	if (ret != 0) {
		return ret;
	}
	ret = sensor_sample_fetch(gyro_dev);
	if (ret != 0) {
		return ret;
	}
	ret = sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, acc_val);
	if (ret != 0) {
		return ret;
	}
	ret = sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyr_val);
	if (ret != 0) {
		return ret;
	}
	ret = sensor_channel_get(accel_dev, SENSOR_CHAN_DIE_TEMP, &temp);
	if (ret != 0) {
		return ret;
	}

	for (int i = 0; i < RM_IMU_AXIS_COUNT; i++) {
		accel[i] = rm_sensor_value_to_float(&acc_val[i]);
		gyro[i] = rm_sensor_value_to_float(&gyr_val[i]);
	}
	*temp_c = rm_sensor_value_to_float(&temp);

	return 0;
}

static void compensate_imu_offset(float accel[3], const float gyro[3], float dt)
{
	float pitch_gyro = gyro[1];
	float pitch_alpha;
	float accel_dyn_x;
	float accel_dyn_z;

	pitch_alpha = (pitch_gyro - last_pitch_gyro_rps) / dt;
	last_pitch_gyro_rps = pitch_gyro;
	pitch_alpha_lpf_rps2 = IMU_PITCH_ALPHA_LPF_A * pitch_alpha_lpf_rps2 +
			       (1.0f - IMU_PITCH_ALPHA_LPF_A) * pitch_alpha;
	g_rm_imu.pitch_alpha_rps2 = pitch_alpha_lpf_rps2;

	accel_dyn_x = pitch_alpha_lpf_rps2 * IMU_OFFSET_Z_M -
		      pitch_gyro * pitch_gyro * IMU_OFFSET_X_M;
	accel_dyn_z = -pitch_alpha_lpf_rps2 * IMU_OFFSET_X_M -
		      pitch_gyro * pitch_gyro * IMU_OFFSET_Z_M;

	accel[0] -= accel_dyn_x;
	accel[2] -= accel_dyn_z;
}

int rm_imu_init(void)
{
	float gyro_sum[RM_IMU_AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
	float accel[RM_IMU_AXIS_COUNT];
	float gyro[RM_IMU_AXIS_COUNT];
	float temp;
	int ret;

	memset((void *)&g_rm_imu, 0, sizeof(g_rm_imu));
	g_rm_imu.magic = RM_IMU_MAGIC;
	g_rm_imu.q[0] = 1.0f;

	if (!device_is_ready(accel_dev) || !device_is_ready(gyro_dev)) {
		g_rm_imu.last_error = -ENODEV;
		return -ENODEV;
	}

	for (int sample = 0; sample < GYRO_CALIB_SAMPLES; sample++) {
		ret = read_sensors(accel, gyro, &temp);
		if (ret != 0) {
			g_rm_imu.last_error = ret;
			return ret;
		}
		for (int i = 0; i < RM_IMU_AXIS_COUNT; i++) {
			gyro_sum[i] += gyro[i];
		}
		k_sleep(K_MSEC(2));
	}

	for (int i = 0; i < RM_IMU_AXIS_COUNT; i++) {
		g_rm_imu.gyro_bias[i] = gyro_sum[i] / GYRO_CALIB_SAMPLES;
	}
	g_rm_imu.temp_c = temp;
	g_rm_imu.calibrated = 1U;
	last_update_ms = k_uptime_get_32();
	last_pitch_gyro_rps = 0.0f;
	pitch_alpha_lpf_rps2 = 0.0f;

	return 0;
}

int rm_imu_update(void)
{
	float accel[RM_IMU_AXIS_COUNT];
	float gyro[RM_IMU_AXIS_COUNT];
	float temp;
	uint32_t now;
	float dt;
	float gyro_norm;
	float acc_norm;
	int ret;

	ret = read_sensors(accel, gyro, &temp);
	if (ret != 0) {
		g_rm_imu.last_error = ret;
		return ret;
	}

	now = k_uptime_get_32();
	dt = (now - last_update_ms) / 1000.0f;
	if (dt <= 0.0f || dt > 0.05f) {
		dt = 0.001f;
	}
	last_update_ms = now;

	for (int i = 0; i < RM_IMU_AXIS_COUNT; i++) {
		gyro[i] -= g_rm_imu.gyro_bias[i];
		g_rm_imu.accel_raw[i] = accel[i];
		g_rm_imu.gyro[i] = gyro[i];
	}

	compensate_imu_offset(accel, gyro, dt);
	for (int i = 0; i < RM_IMU_AXIS_COUNT; i++) {
		g_rm_imu.accel_comp[i] = accel[i];
		g_rm_imu.accel[i] = accel[i];
	}

	ret = mahony_update_6axis((float *)g_rm_imu.q, accel[0], accel[1], accel[2],
				  gyro[0], gyro[1], gyro[2], dt);
	if (ret != 0) {
		g_rm_imu.last_error = ret;
		return ret;
	}
	update_euler();

	gyro_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
	acc_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	g_rm_imu.stable = (gyro_norm < 0.05f && acc_norm > G_MPS2 - 0.8f &&
			   acc_norm < G_MPS2 + 0.8f) ? 1U : 0U;
	g_rm_imu.temp_c = temp;
	g_rm_imu.dt = dt;
	g_rm_imu.last_error = 0;
	g_rm_imu.update_count++;

	return 0;
}

const volatile struct rm_imu_attitude *rm_imu_get(void)
{
	return &g_rm_imu;
}

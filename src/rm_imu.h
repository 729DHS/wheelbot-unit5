#ifndef RM_IMU_H_
#define RM_IMU_H_

#include <stdint.h>

#define RM_IMU_MAGIC 0x494d5541
#define RM_IMU_AXIS_COUNT 3

struct rm_imu_attitude {
	uint32_t magic;
	uint32_t update_count;
	int32_t last_error;
	float dt;
	float accel[RM_IMU_AXIS_COUNT];
	float accel_raw[RM_IMU_AXIS_COUNT];
	float accel_comp[RM_IMU_AXIS_COUNT];
	float gyro[RM_IMU_AXIS_COUNT];
	float q[4];
	float pitch;
	float roll;
	float temp_c;
	float gyro_bias[RM_IMU_AXIS_COUNT];
	float pitch_alpha_rps2;
	uint8_t stable;
	uint8_t calibrated;
	uint8_t reserved[2];
};

extern volatile struct rm_imu_attitude g_rm_imu;

int rm_imu_init(void);
int rm_imu_update(void);
const volatile struct rm_imu_attitude *rm_imu_get(void);

#endif

#ifndef RM_M3508_H_
#define RM_M3508_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

#define RM_M3508_MAGIC 0x4d333530
#define RM_M3508_CONTROL_ID 0x200
#define RM_M3508_FIRST_FEEDBACK_ID 0x201
#define RM_M3508_FEEDBACK_ID_COUNT 4
#define RM_M3508_MOTOR_COUNT 2
#define RM_M3508_CURRENT_SLOT_COUNT 4
#define RM_M3508_BUS_COUNT 2
#define RM_M3508_ONLINE_TIMEOUT_MS 500

enum rm_m3508_id {
	RM_M3508_CAN1_ID201 = 0,
	RM_M3508_CAN2_ID202 = 1,
};

struct rm_m3508_motor {
	uint32_t rx_count;
	uint32_t last_ms;
	uint32_t last_can_id;
	uint8_t last_data[8];
	uint8_t online;
	uint8_t temperature;
	uint16_t angle_raw;
	int16_t speed_rpm;
	int16_t current_raw;
};

struct rm_m3508_bus_state {
	uint32_t tx_count;
	uint32_t rx_count;
	uint32_t parsed_count;
	uint32_t online_mask;
	uint32_t last_tx_ms;
	uint32_t last_rx_ms;
	int32_t set_mode_ret;
	int32_t start_ret;
	int32_t filter_ret[RM_M3508_FEEDBACK_ID_COUNT];
	int32_t last_send_ret;
	int16_t current_cmd[RM_M3508_CURRENT_SLOT_COUNT];
};

struct rm_m3508_driver {
	uint32_t magic;
	uint32_t ready_mask;
	uint32_t loops;
	struct rm_m3508_bus_state bus[RM_M3508_BUS_COUNT];
	struct rm_m3508_motor motor[RM_M3508_MOTOR_COUNT];
};

extern volatile struct rm_m3508_driver g_rm_m3508;

int rm_m3508_init(void);
void rm_m3508_poll(void);
int rm_m3508_send_currents(void);
int rm_m3508_set_current(enum rm_m3508_id id, int16_t current);
int rm_m3508_set_all_current(int16_t can1_id201_current, int16_t can2_id202_current);
void rm_m3508_stop(void);
bool rm_m3508_is_online(enum rm_m3508_id id);
const volatile struct rm_m3508_motor *rm_m3508_get(enum rm_m3508_id id);

void rm_m3508_make_zero_current_frame(struct can_frame *frame);
void rm_m3508_make_current_frame(struct can_frame *frame, const int16_t current[RM_M3508_CURRENT_SLOT_COUNT]);

#endif

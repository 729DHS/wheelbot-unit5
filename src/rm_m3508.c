#include "rm_m3508.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

K_MSGQ_DEFINE(m3508_can1_rx_msgq, sizeof(struct can_frame), 64, 4);
K_MSGQ_DEFINE(m3508_can2_rx_msgq, sizeof(struct can_frame), 64, 4);

volatile struct rm_m3508_driver g_rm_m3508 = {
	.magic = RM_M3508_MAGIC,
};

static const struct device *const can_dev[RM_M3508_BUS_COUNT] = {
	DEVICE_DT_GET(DT_NODELABEL(can1)),
	DEVICE_DT_GET(DT_NODELABEL(can2)),
};

static struct k_msgq *const can_rx_msgq[RM_M3508_BUS_COUNT] = {
	&m3508_can1_rx_msgq,
	&m3508_can2_rx_msgq,
};

static uint16_t be16u(const uint8_t *data)
{
	return ((uint16_t)data[0] << 8) | data[1];
}

static int16_t be16s(const uint8_t *data)
{
	return (int16_t)be16u(data);
}

static void put_be16(uint8_t *data, int16_t value)
{
	data[0] = (uint8_t)((uint16_t)value >> 8);
	data[1] = (uint8_t)value;
}

static bool valid_id(enum rm_m3508_id id)
{
	return id >= RM_M3508_CAN1_ID201 && id < RM_M3508_MOTOR_COUNT;
}

static uint8_t motor_bus(enum rm_m3508_id id)
{
	return id == RM_M3508_CAN1_ID201 ? 0U : 1U;
}

static uint8_t motor_feedback_id(enum rm_m3508_id id)
{
	return id == RM_M3508_CAN1_ID201 ? 0x03U : 0x01U;
}

static uint8_t motor_current_slot(enum rm_m3508_id id)
{
	return motor_feedback_id(id) - 1U;
}

static int16_t motor_direction(enum rm_m3508_id id)
{
	return id == RM_M3508_CAN2_ID202 ? -1 : 1;
}

static int send_current_bus(uint8_t bus)
{
	struct can_frame frame;
	int ret;

	if (bus >= RM_M3508_BUS_COUNT) {
		return -EINVAL;
	}

	rm_m3508_make_current_frame(&frame,
				    (const int16_t *)g_rm_m3508.bus[bus].current_cmd);
	ret = can_send(can_dev[bus], &frame, K_NO_WAIT, NULL, NULL);
	g_rm_m3508.bus[bus].last_send_ret = ret;
	g_rm_m3508.bus[bus].last_tx_ms = k_uptime_get_32();
	if (ret == 0) {
		g_rm_m3508.bus[bus].tx_count++;
	}

	return ret;
}

static int start_can_bus(uint8_t bus)
{
	const struct device *dev = can_dev[bus];
	int ret;

	if (!device_is_ready(dev)) {
		g_rm_m3508.bus[bus].start_ret = -ENODEV;
		return -ENODEV;
	}

	g_rm_m3508.ready_mask |= BIT(bus);

	g_rm_m3508.bus[bus].set_mode_ret = can_set_mode(dev, CAN_MODE_ONE_SHOT);
	if (g_rm_m3508.bus[bus].set_mode_ret < 0) {
		g_rm_m3508.bus[bus].set_mode_ret = can_set_mode(dev, CAN_MODE_NORMAL);
	}

	ret = can_start(dev);
	g_rm_m3508.bus[bus].start_ret = ret;
	if (ret < 0 && ret != -EALREADY) {
		return ret;
	}

	return 0;
}

static int add_feedback_filters(uint8_t bus)
{
	for (int i = 0; i < RM_M3508_FEEDBACK_ID_COUNT; i++) {
		const struct can_filter filter = {
			.id = RM_M3508_FIRST_FEEDBACK_ID + i,
			.mask = CAN_STD_ID_MASK,
			.flags = 0,
		};
		int ret = can_add_rx_filter_msgq(can_dev[bus], can_rx_msgq[bus], &filter);

		g_rm_m3508.bus[bus].filter_ret[i] = ret;
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static bool parse_frame(uint8_t bus, const struct can_frame *frame)
{
	enum rm_m3508_id id;
	struct rm_m3508_motor *status;

	if (bus >= RM_M3508_BUS_COUNT ||
	    (frame->flags & CAN_FRAME_IDE) != 0U || frame->dlc != 8U ||
	    frame->id < RM_M3508_FIRST_FEEDBACK_ID ||
	    frame->id >= RM_M3508_FIRST_FEEDBACK_ID + RM_M3508_FEEDBACK_ID_COUNT) {
		return false;
	}

	id = bus == 0U ? RM_M3508_CAN1_ID201 : RM_M3508_CAN2_ID202;

	status = (struct rm_m3508_motor *)&g_rm_m3508.motor[id];
	status->rx_count++;
	status->last_ms = k_uptime_get_32();
	status->last_can_id = frame->id;
	memcpy(status->last_data, frame->data, sizeof(status->last_data));
	status->online = 1U;
	status->angle_raw = be16u(&frame->data[0]);
	status->speed_rpm = be16s(&frame->data[2]) * motor_direction(id);
	status->current_raw = be16s(&frame->data[4]) * motor_direction(id);
	status->temperature = frame->data[6];

	return true;
}

static void drain_rx(uint8_t bus)
{
	struct can_frame frame;

	while (k_msgq_get(can_rx_msgq[bus], &frame, K_NO_WAIT) == 0) {
		g_rm_m3508.bus[bus].rx_count++;
		g_rm_m3508.bus[bus].last_rx_ms = k_uptime_get_32();

		if (parse_frame(bus, &frame)) {
			g_rm_m3508.bus[bus].parsed_count++;
		}
	}
}

static void refresh_online(void)
{
	const uint32_t now = k_uptime_get_32();

	g_rm_m3508.bus[0].online_mask = 0U;
	g_rm_m3508.bus[1].online_mask = 0U;

	for (int i = 0; i < RM_M3508_MOTOR_COUNT; i++) {
		struct rm_m3508_motor *motor = (struct rm_m3508_motor *)&g_rm_m3508.motor[i];
		const uint8_t bus = motor_bus(i);

		if (motor->rx_count == 0U || now - motor->last_ms > RM_M3508_ONLINE_TIMEOUT_MS) {
			motor->online = 0U;
		}
		if (motor->online != 0U) {
			g_rm_m3508.bus[bus].online_mask |= BIT(motor_feedback_id(i) - 1U);
		}
	}
}

int rm_m3508_init(void)
{
	int ret;

	memset((void *)&g_rm_m3508, 0, sizeof(g_rm_m3508));
	g_rm_m3508.magic = RM_M3508_MAGIC;

	for (int bus = 0; bus < RM_M3508_BUS_COUNT; bus++) {
		ret = start_can_bus(bus);
		if (ret < 0) {
			return ret;
		}

		ret = add_feedback_filters(bus);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

void rm_m3508_poll(void)
{
	drain_rx(0);
	drain_rx(1);
	refresh_online();
	g_rm_m3508.loops++;
}

int rm_m3508_send_currents(void)
{
	int ret = 0;

	for (uint8_t bus = 0U; bus < RM_M3508_BUS_COUNT; bus++) {
		int r = send_current_bus(bus);

		if (r != 0 && ret == 0) {
			ret = r;
		}
	}

	return ret;
}

int rm_m3508_set_current(enum rm_m3508_id id, int16_t current)
{
	if (!valid_id(id)) {
		return -EINVAL;
	}

	g_rm_m3508.bus[motor_bus(id)].current_cmd[motor_current_slot(id)] =
		current * motor_direction(id);
	return 0;
}

int rm_m3508_set_all_current(int16_t can1_id201_current, int16_t can2_id202_current)
{
	int ret;

	ret = rm_m3508_set_current(RM_M3508_CAN1_ID201, can1_id201_current);
	if (ret < 0) {
		return ret;
	}

	return rm_m3508_set_current(RM_M3508_CAN2_ID202, can2_id202_current);
}

void rm_m3508_stop(void)
{
	(void)rm_m3508_set_all_current(0, 0);
	for (uint8_t bus = 0U; bus < RM_M3508_BUS_COUNT; bus++) {
		(void)send_current_bus(bus);
	}
}

bool rm_m3508_is_online(enum rm_m3508_id id)
{
	if (!valid_id(id)) {
		return false;
	}

	return g_rm_m3508.motor[id].online != 0U;
}

const volatile struct rm_m3508_motor *rm_m3508_get(enum rm_m3508_id id)
{
	if (!valid_id(id)) {
		return NULL;
	}

	return &g_rm_m3508.motor[id];
}

void rm_m3508_make_zero_current_frame(struct can_frame *frame)
{
	const int16_t zero_current[RM_M3508_CURRENT_SLOT_COUNT] = { 0 };

	rm_m3508_make_current_frame(frame, zero_current);
}

void rm_m3508_make_current_frame(struct can_frame *frame,
				 const int16_t current[RM_M3508_CURRENT_SLOT_COUNT])
{
	memset(frame, 0, sizeof(*frame));
	frame->id = RM_M3508_CONTROL_ID;
	frame->dlc = 8;
	frame->flags = 0;

	for (int i = 0; i < RM_M3508_CURRENT_SLOT_COUNT; i++) {
		put_be16(&frame->data[i * 2], current[i]);
	}
}

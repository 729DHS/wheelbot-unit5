#include "rm_sbus.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#define RM_SBUS_RX_TIMEOUT_US 0

volatile struct rm_sbus_snapshot g_sbus_snapshot = {
	.magic = RM_SBUS_MAGIC,
};

static const struct device *const sbus_uart = DEVICE_DT_GET(DT_NODELABEL(usart3));
static uint8_t sbus_dma_buf[RM_SBUS_DMA_BUF_LEN];

static int32_t map_stick(uint16_t raw)
{
	return (int32_t)((165 * (int32_t)raw + 7936) / 196);
}

static int32_t map_knob(uint16_t raw)
{
	return (int32_t)(((int32_t)raw - 240) * 4962 / 1567 - 3278);
}

static uint8_t map_switch(uint16_t raw)
{
	switch (raw) {
	case 0x00f0:
		return 1;
	case 0x0400:
		return 3;
	case 0x070f:
		return 2;
	default:
		return 0;
	}
}

static int32_t map_thumbwheel(uint16_t raw)
{
	switch (raw) {
	case 0x00f0:
		return 1684;
	case 0x0400:
		return 1024;
	case 0x070f:
		return -3278;
	default:
		return (int32_t)raw;
	}
}

static bool frame_is_valid(const uint8_t *data)
{
	return data[0] == 0x0f && data[RM_SBUS_FRAME_LEN - 1] == 0x00;
}

static void parse_frame(const uint8_t *data)
{
	uint16_t raw[RM_SBUS_CHANNELS];
	uint8_t switches[3];
	int32_t channel[RM_SBUS_CHANNELS];

	raw[0] = ((((uint16_t)data[2] << 8) | data[1]) & 0x07ff);
	raw[1] = ((((uint16_t)data[3] << 5) | (data[2] >> 3)) & 0x07ff);
	raw[2] = ((((uint16_t)data[5] << 10) | ((uint16_t)data[4] << 2) |
		   (data[3] >> 6)) &
		  0x07ff);
	raw[3] = ((((uint16_t)data[6] << 7) | (data[5] >> 1)) & 0x07ff);
	raw[4] = ((((uint16_t)data[7] << 4) | (data[6] >> 4)) & 0x07ff);
	raw[5] = ((((uint16_t)data[9] << 9) | ((uint16_t)data[8] << 1) |
		   (data[7] >> 7)) &
		  0x07ff);
	raw[6] = ((((uint16_t)data[10] << 6) | (data[9] >> 2)) & 0x07ff);
	raw[7] = ((((uint16_t)data[11] << 3) | (data[10] >> 5)) & 0x07ff);
	raw[8] = ((((uint16_t)data[13] << 8) | data[12]) & 0x07ff);
	raw[9] = ((((uint16_t)data[14] << 5) | (data[13] >> 3)) & 0x07ff);

	for (int i = 0; i < 4; i++) {
		channel[i] = map_stick(raw[i]);
	}

	switches[0] = map_switch(raw[4]);
	switches[1] = map_switch(raw[5]);
	switches[2] = map_switch(raw[7]);

	channel[4] = switches[0];
	channel[5] = switches[1];
	channel[6] = map_thumbwheel(raw[6]);
	channel[7] = switches[2];
	channel[8] = map_knob(raw[8]);
	channel[9] = map_knob(raw[9]);

	g_sbus_snapshot.seq++;
	g_sbus_snapshot.last_ms = k_uptime_get_32();
	g_sbus_snapshot.header = data[0];
	g_sbus_snapshot.flags = data[23];
	g_sbus_snapshot.footer = data[24];
	g_sbus_snapshot.connected = data[23] == 0x00 ? 1U : 0U;
	memcpy((void *)g_sbus_snapshot.raw, raw, sizeof(raw));
	memcpy((void *)g_sbus_snapshot.channel, channel, sizeof(channel));
	memcpy((void *)g_sbus_snapshot.switches, switches, sizeof(switches));
	g_sbus_snapshot.seq++;
}

static void process_byte(uint8_t byte)
{
	g_sbus_snapshot.rx_tail[g_sbus_snapshot.rx_tail_pos++ % RM_SBUS_RX_TAIL_LEN] = byte;
	g_sbus_snapshot.bytes++;

	if (g_sbus_snapshot.frame_index == 0U && byte != 0x0fU) {
		g_sbus_snapshot.sync_drops++;
		return;
	}

	g_sbus_snapshot.frame[g_sbus_snapshot.frame_index++] = byte;

	if (g_sbus_snapshot.frame_index != RM_SBUS_FRAME_LEN) {
		return;
	}

	g_sbus_snapshot.frame_index = 0U;
	g_sbus_snapshot.frames++;

	if (!frame_is_valid((const uint8_t *)g_sbus_snapshot.frame)) {
		g_sbus_snapshot.invalid_frames++;
		return;
	}

	g_sbus_snapshot.valid_frames++;
	parse_frame((const uint8_t *)g_sbus_snapshot.frame);
}

static void process_chunk(const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		process_byte(buf[i]);
	}
}

static int start_rx_dma(void)
{
	int ret;

	ret = uart_rx_enable(sbus_uart, sbus_dma_buf, sizeof(sbus_dma_buf),
			     RM_SBUS_RX_TIMEOUT_US);
	if (ret == 0) {
		g_sbus_snapshot.dma_restarts++;
	}

	return ret;
}

static void sbus_uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case UART_RX_RDY:
		g_sbus_snapshot.dma_rdy_events++;
		process_chunk(evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);
		break;
	case UART_RX_BUF_REQUEST:
		g_sbus_snapshot.dma_buf_requests++;
		break;
	case UART_RX_BUF_RELEASED:
		g_sbus_snapshot.dma_buf_released++;
		break;
	case UART_RX_DISABLED:
		g_sbus_snapshot.dma_disabled++;
		g_sbus_snapshot.init_ret = start_rx_dma();
		break;
	case UART_RX_STOPPED:
		g_sbus_snapshot.dma_stopped++;
		break;
	default:
		break;
	}
}

int rm_sbus_init(void)
{
	const struct uart_config config = {
		.baudrate = 100000,
		.parity = UART_CFG_PARITY_EVEN,
		.stop_bits = UART_CFG_STOP_BITS_2,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};
	int ret;

	g_sbus_snapshot.magic = RM_SBUS_MAGIC;
	g_sbus_snapshot.ready = device_is_ready(sbus_uart) ? 1U : 0U;

	if (g_sbus_snapshot.ready == 0U) {
		g_sbus_snapshot.init_ret = -ENODEV;
		return -ENODEV;
	}

	ret = uart_configure(sbus_uart, &config);
	if (ret < 0) {
		g_sbus_snapshot.init_ret = ret;
		return ret;
	}

	ret = uart_callback_set(sbus_uart, sbus_uart_callback, NULL);
	if (ret < 0) {
		g_sbus_snapshot.init_ret = ret;
		return ret;
	}

	ret = start_rx_dma();
	g_sbus_snapshot.init_ret = ret;

	return ret;
}

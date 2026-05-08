#ifndef RM_SBUS_H_
#define RM_SBUS_H_

#include <stdint.h>

#define RM_SBUS_CHANNELS 10
#define RM_SBUS_FRAME_LEN 25
#define RM_SBUS_RX_TAIL_LEN 64
#define RM_SBUS_DMA_BUF_LEN 64
#define RM_SBUS_MAGIC 0x53425553

struct rm_sbus_snapshot {
	uint32_t magic;
	uint32_t ready;
	int32_t init_ret;
	uint32_t seq;
	uint32_t bytes;
	uint32_t frames;
	uint32_t valid_frames;
	uint32_t invalid_frames;
	uint32_t sync_drops;
	uint32_t last_ms;
	uint32_t dma_restarts;
	uint32_t dma_rdy_events;
	uint32_t dma_buf_requests;
	uint32_t dma_buf_released;
	uint32_t dma_disabled;
	uint32_t dma_stopped;
	uint8_t frame[RM_SBUS_FRAME_LEN];
	uint8_t rx_tail[RM_SBUS_RX_TAIL_LEN];
	uint32_t rx_tail_pos;
	uint8_t frame_index;
	uint8_t header;
	uint8_t flags;
	uint8_t footer;
	uint8_t connected;
	uint8_t switches[3];
	uint8_t reserved[3];
	uint16_t raw[RM_SBUS_CHANNELS];
	int32_t channel[RM_SBUS_CHANNELS];
};

extern volatile struct rm_sbus_snapshot g_sbus_snapshot;

int rm_sbus_init(void);

#endif

/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _JZ4750_PCM_H
#define _JZ4750_PCM_H

#define ST_RUNNING		(1<<0)
#define ST_OPENED		(1<<1)

#define AIC_START_DMA           (1<<0)
#define AIC_END_DMA             (1<<1)

struct jz4750_pcm_dma_params {
	int channel;				/* Channel ID */
	dma_addr_t dma_addr;
	int dma_size;			/* Size of the DMA transfer */
};

#endif

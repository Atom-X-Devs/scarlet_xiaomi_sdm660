/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 MediaTek Inc.
 */

#ifndef __RPROC_MTK_COMMON_H
#define __RPROC_MTK_COMMON_H

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>

#define MT8183_SW_RSTN			0x0
#define MT8183_SW_RSTN_BIT		BIT(0)
#define MT8183_SCP_TO_HOST		0x1C
#define MT8183_SCP_IPC_INT_BIT		BIT(0)
#define MT8183_SCP_WDT_INT_BIT		BIT(8)
#define MT8183_HOST_TO_SCP		0x28
#define MT8183_HOST_IPC_INT_BIT		BIT(0)
#define MT8183_WDT_CFG			0x84
#define MT8183_SCP_CLK_SW_SEL		0x4000
#define MT8183_SCP_CLK_DIV_SEL		0x4024
#define MT8183_SCP_SRAM_PDN		0x402C
#define MT8183_SCP_L1_SRAM_PD		0x4080
#define MT8183_SCP_TCM_TAIL_SRAM_PD	0x4094

#define SCP_FW_VER_LEN		32

struct scp_run {
	u32 signaled;
	s8 fw_ver[SCP_FW_VER_LEN];
	u32 dec_capability;
	u32 enc_capability;
	wait_queue_head_t wq;
};

struct scp_ipi_desc {
	scp_ipi_handler_t handler;
	void *priv;
};

struct mtk_scp {
	struct device *dev;
	struct rproc *rproc;
	struct clk *clk;
	void __iomem *reg_base;
	void __iomem *sram_base;
	size_t sram_size;

	struct share_obj *recv_buf;
	struct share_obj *send_buf;
	struct scp_run run;
	struct mutex lock; /* for protecting mtk_scp data structure */
	struct scp_ipi_desc ipi_desc[SCP_IPI_MAX];
	bool ipi_id_ack[SCP_IPI_MAX];
	wait_queue_head_t ack_wq;

	void __iomem *cpu_addr;
	phys_addr_t phys_addr;
	size_t dram_size;

	struct rproc_subdev *rpmsg_subdev;
};

/**
 * struct share_obj - SRAM buffer shared with
 *		      AP and SCP
 *
 * @id:		IPI id
 * @len:	share buffer length
 * @share_buf:	share buffer data
 */
struct share_obj {
	s32 id;
	u32 len;
	u8 share_buf[288];
};

void scp_memcpy_aligned(void *dst, const void *src, unsigned int len);

#endif

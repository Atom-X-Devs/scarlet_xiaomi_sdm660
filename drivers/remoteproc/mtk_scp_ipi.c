// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2018 MediaTek Inc.

#include <asm/barrier.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_data/mtk_scp.h>
#include <linux/platform_device.h>

#include "mtk_common.h"

int scp_ipi_register(struct platform_device *pdev,
		     enum scp_ipi_id id,
		     scp_ipi_handler_t handler,
		     void *priv)
{
	struct mtk_scp *scp = platform_get_drvdata(pdev);
	struct scp_ipi_desc *ipi_desc;

	if (!scp) {
		dev_err(&pdev->dev, "scp device is not ready\n");
		return -EPROBE_DEFER;
	}

	if (WARN_ON(id < 0) || WARN_ON(id >= SCP_IPI_MAX) ||
	    WARN_ON(handler == NULL))
		return -EINVAL;

	ipi_desc = scp->ipi_desc;
	ipi_desc[id].handler = handler;
	ipi_desc[id].priv = priv;

	return 0;
}
EXPORT_SYMBOL_GPL(scp_ipi_register);

void scp_ipi_unregister(struct platform_device *pdev, enum scp_ipi_id id)
{
	struct mtk_scp *scp = platform_get_drvdata(pdev);
	struct scp_ipi_desc *ipi_desc;

	if (!scp)
		return;

	if (WARN_ON(id < 0) || WARN_ON(id >= SCP_IPI_MAX))
		return;

	ipi_desc = scp->ipi_desc;
	ipi_desc[id].handler = NULL;
	ipi_desc[id].priv = NULL;
}
EXPORT_SYMBOL_GPL(scp_ipi_unregister);

/*
 * Copy src to dst, where dst is in SCP SRAM region.
 * Since AP access of SCP SRAM don't support byte write, this always write a
 * full word at a time, and may cause some extra bytes to be written at the
 * beginning & ending of dst.
 */
void scp_memcpy_aligned(void *dst, const void *src, unsigned int len)
{
	void *ptr;
	u32 val;
	unsigned int i = 0;

	if (!IS_ALIGNED((unsigned long)dst, 4)) {
		ptr = (void *)ALIGN_DOWN((unsigned long)dst, 4);
		i = 4 - (dst - ptr);
		val = readl_relaxed(ptr);
		memcpy((u8 *)&val + (4 - i), src, i);
		writel_relaxed(val, ptr);
	}

	while (i + 4 <= len) {
		val = *((u32 *)(src + i));
		writel_relaxed(val, dst + i);
		i += 4;
	}
	if (i < len) {
		val = readl_relaxed(dst + i);
		memcpy(&val, src + i, len - i);
		writel_relaxed(val, dst + i);
	}
}
EXPORT_SYMBOL_GPL(scp_memcpy_aligned);

int scp_ipi_send(struct platform_device *pdev,
		 enum scp_ipi_id id,
		 void *buf,
		 unsigned int len,
		 unsigned int wait)
{
	struct mtk_scp *scp = platform_get_drvdata(pdev);
	struct share_obj *send_obj = scp->send_buf;
	unsigned long timeout;
	int ret;

	if (WARN_ON(id <= SCP_IPI_INIT) || WARN_ON(id >= SCP_IPI_MAX) ||
	    WARN_ON(id == SCP_IPI_NS_SERVICE) ||
	    WARN_ON(len > sizeof(send_obj->share_buf)) || WARN_ON(!buf))
		return -EINVAL;

	mutex_lock(&scp->lock);

	ret = clk_prepare_enable(scp->clk);
	if (ret) {
		dev_err(scp->dev, "failed to enable clock\n");
		return ret;
	}

	 /* Wait until SCP receives the last command */
	timeout = jiffies + msecs_to_jiffies(2000);
	do {
		if (time_after(jiffies, timeout)) {
			dev_err(scp->dev, "%s: IPI timeout!\n", __func__);
			ret = -EIO;
			mutex_unlock(&scp->lock);
			goto clock_disable;
		}
	} while (readl(scp->reg_base + MT8183_HOST_TO_SCP));

	scp_memcpy_aligned(send_obj->share_buf, buf, len);

	send_obj->len = len;
	send_obj->id = id;

	scp->ipi_id_ack[id] = false;
	/*
	 * Ensure that all writes to SRAM are committed before sending the
	 * interrupt to SCP.
	 */
	mb();
	/* send the command to SCP */
	writel(MT8183_HOST_IPC_INT_BIT, scp->reg_base + MT8183_HOST_TO_SCP);

	mutex_unlock(&scp->lock);

	if (wait) {
		/* wait for SCP's ACK */
		timeout = msecs_to_jiffies(wait);
		ret = wait_event_timeout(scp->ack_wq,
					 scp->ipi_id_ack[id],
					 timeout);
		scp->ipi_id_ack[id] = false;
		if (WARN(!ret,
			 "scp ipi %d ack time out !", id))
			ret = -EIO;
		else
			ret = 0;
	}

clock_disable:
	clk_disable_unprepare(scp->clk);

	return ret;
}
EXPORT_SYMBOL_GPL(scp_ipi_send);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek scp IPI interface");

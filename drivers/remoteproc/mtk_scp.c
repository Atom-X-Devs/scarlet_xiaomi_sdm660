// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2018 MediaTek Inc.

#include <asm/barrier.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_data/mtk_scp.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/rpmsg/mtk_rpmsg.h>

#include "mtk_common.h"
#include "remoteproc_internal.h"

#define MAX_CODE_SIZE 0x500000
#define SCP_FW_END 0x7C000

struct platform_device *scp_get_pdev(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *scp_node;
	struct platform_device *scp_pdev;

	scp_node = of_parse_phandle(dev->of_node, "mediatek,scp", 0);
	if (!scp_node) {
		dev_err(dev, "can't get SCP node\n");
		return NULL;
	}

	scp_pdev = of_find_device_by_node(scp_node);
	if (WARN_ON(!scp_pdev)) {
		dev_err(dev, "SCP pdev failed\n");
		of_node_put(scp_node);
		return NULL;
	}

	return scp_pdev;
}
EXPORT_SYMBOL_GPL(scp_get_pdev);

static void scp_wdt_handler(struct mtk_scp *scp)
{
	rproc_report_crash(scp->rproc, RPROC_WATCHDOG);
}

static void scp_init_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct mtk_scp *scp = (struct mtk_scp *)priv;
	struct scp_run *run = (struct scp_run *)data;

	scp->run.signaled = run->signaled;
	strscpy(scp->run.fw_ver, run->fw_ver, SCP_FW_VER_LEN);
	scp->run.dec_capability = run->dec_capability;
	scp->run.enc_capability = run->enc_capability;
	wake_up_interruptible(&scp->run.wq);
}

static void scp_ipi_handler(struct mtk_scp *scp)
{
	struct share_obj *rcv_obj = scp->recv_buf;
	struct scp_ipi_desc *ipi_desc = scp->ipi_desc;
	u8 tmp_data[288];

	if (rcv_obj->id >= SCP_IPI_MAX || !ipi_desc[rcv_obj->id].handler) {
		dev_err(scp->dev, "No such ipi id = %d\n", rcv_obj->id);
		return;
	}

	memcpy_fromio(tmp_data, &rcv_obj->share_buf, rcv_obj->len);
	ipi_desc[rcv_obj->id].handler(tmp_data,
				      rcv_obj->len,
				      ipi_desc[rcv_obj->id].priv);
	scp->ipi_id_ack[rcv_obj->id] = true;
	wake_up(&scp->ack_wq);
}

static int scp_ipi_init(struct mtk_scp *scp)
{
	size_t send_offset = SCP_FW_END - sizeof(struct share_obj);
	size_t recv_offset = send_offset - sizeof(struct share_obj);

	/* Disable SCP to host interrupt */
	writel(MT8183_SCP_IPC_INT_BIT, scp->reg_base + MT8183_SCP_TO_HOST);

	/* shared buffer initialization */
	scp->recv_buf = (__force struct share_obj *)(scp->sram_base +
						recv_offset);
	scp->send_buf = (__force struct share_obj *)(scp->sram_base +
						send_offset);
	memset_io(scp->recv_buf, 0, sizeof(scp->recv_buf));
	memset_io(scp->send_buf, 0, sizeof(scp->send_buf));

	return 0;
}

static void scp_reset_assert(const struct mtk_scp *scp)
{
	u32 val;

	val = readl(scp->reg_base + MT8183_SW_RSTN);
	val &= ~MT8183_SW_RSTN_BIT;
	writel(val, scp->reg_base + MT8183_SW_RSTN);
}

static void scp_reset_deassert(const struct mtk_scp *scp)
{
	u32 val;

	val = readl(scp->reg_base + MT8183_SW_RSTN);
	val |= MT8183_SW_RSTN_BIT;
	writel(val, scp->reg_base + MT8183_SW_RSTN);
}

static irqreturn_t scp_irq_handler(int irq, void *priv)
{
	struct mtk_scp *scp = priv;
	u32 scp_to_host;
	int ret;

	ret = clk_prepare_enable(scp->clk);
	if (ret) {
		dev_err(scp->dev, "failed to enable clocks\n");
		return IRQ_NONE;
	}

	scp_to_host = readl(scp->reg_base + MT8183_SCP_TO_HOST);
	if (scp_to_host & MT8183_SCP_IPC_INT_BIT) {
		scp_ipi_handler(scp);
	} else {
		dev_err(scp->dev, "SCP watchdog timeout! 0x%x", scp_to_host);
		scp_wdt_handler(scp);
	}

	/*
	 * Ensure that all writes to SRAM are committed before another
	 * interrupt.
	 */
	mb();
	/* SCP won't send another interrupt until we set SCP_TO_HOST to 0. */
	writel(MT8183_SCP_IPC_INT_BIT | MT8183_SCP_WDT_INT_BIT,
	       scp->reg_base + MT8183_SCP_TO_HOST);
	clk_disable_unprepare(scp->clk);

	return IRQ_HANDLED;
}

static int scp_elf_load_segments(struct rproc *rproc, const struct firmware *fw)
{
	struct device *dev = &rproc->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;
		void __iomem *ptr;

		if (phdr->p_type != PT_LOAD)
			continue;

		dev_dbg(dev, "phdr: type %d da 0x%x memsz 0x%x filesz 0x%x\n",
			phdr->p_type, da, memsz, filesz);

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* grab the kernel address for this device address */
		ptr = rproc_da_to_va(rproc, da, memsz);
		if (!ptr) {
			dev_err(dev, "bad phdr da 0x%x mem 0x%x\n", da, memsz);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		if (phdr->p_filesz)
			scp_memcpy_aligned(ptr, elf_data + phdr->p_offset,
					   filesz);
	}

	return ret;
}

static int scp_load(struct rproc *rproc, const struct firmware *fw)
{
	const struct mtk_scp *scp = rproc->priv;
	struct device *dev = scp->dev;
	int ret;

	ret = clk_prepare_enable(scp->clk);
	if (ret) {
		dev_err(dev, "failed to enable clocks\n");
		return ret;
	}

	/* Hold SCP in reset while loading FW. */
	scp_reset_assert(scp);

	/* Reset clocks before loading FW */
	writel(0x0, scp->reg_base + MT8183_SCP_CLK_SW_SEL);
	writel(0x0, scp->reg_base + MT8183_SCP_CLK_DIV_SEL);

	/* Initialize TCM before loading FW. */
	writel(0x0, scp->reg_base + MT8183_SCP_L1_SRAM_PD);
	writel(0x0, scp->reg_base + MT8183_SCP_TCM_TAIL_SRAM_PD);

	/* Turn on the power of SCP's SRAM before using it. */
	writel(0x0, scp->reg_base + MT8183_SCP_SRAM_PDN);
	ret = scp_elf_load_segments(rproc, fw);
	clk_disable_unprepare(scp->clk);

	return ret;
}

static int scp_start(struct rproc *rproc)
{
	struct mtk_scp *scp = (struct mtk_scp *)rproc->priv;
	struct device *dev = scp->dev;
	struct scp_run *run;
	int ret;

	ret = clk_prepare_enable(scp->clk);
	if (ret) {
		dev_err(dev, "failed to enable clocks\n");
		return ret;
	}

	run = &scp->run;
	run->signaled = false;

	scp_reset_deassert(scp);

	ret = wait_event_interruptible_timeout(
					run->wq,
					run->signaled,
					msecs_to_jiffies(2000));

	if (ret == 0) {
		dev_err(dev, "wait SCP initialization timeout!\n");
		ret = -ETIME;
		goto stop;
	}
	if (ret == -ERESTARTSYS) {
		dev_err(dev, "wait SCP interrupted by a signal!\n");
		goto stop;
	}
	clk_disable_unprepare(scp->clk);
	dev_info(dev, "SCP is ready. FW version %s\n", run->fw_ver);

	return 0;

stop:
	scp_reset_assert(scp);
	clk_disable_unprepare(scp->clk);
	return ret;
}

static void *scp_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct mtk_scp *scp = (struct mtk_scp *)rproc->priv;
	int offset;

	if (da < scp->sram_size) {
		offset = da;
		if (offset >= 0 && ((offset + len) < scp->sram_size))
			return (__force void *)(scp->sram_base + offset);
	} else if (da >= scp->sram_size &&
		   da < (scp->sram_size + MAX_CODE_SIZE)) {
		offset = da;
		if (offset >= 0 && (offset + len) < MAX_CODE_SIZE)
			return scp->cpu_addr + offset;
	} else {
		offset = da - scp->phys_addr;
		if (offset >= 0 &&
		    (offset + len) < (scp->dram_size - MAX_CODE_SIZE))
			return scp->cpu_addr + offset;
	}

	return NULL;
}

static int scp_stop(struct rproc *rproc)
{
	struct mtk_scp *scp = (struct mtk_scp *)rproc->priv;
	int ret;

	ret = clk_prepare_enable(scp->clk);
	if (ret) {
		dev_err(scp->dev, "failed to enable clocks\n");
		return ret;
	}

	scp_reset_assert(scp);
	/* Disable SCP watchdog */
	writel(0, scp->reg_base + MT8183_WDT_CFG);
	clk_disable_unprepare(scp->clk);

	return 0;
}

static const struct rproc_ops scp_ops = {
	.start		= scp_start,
	.stop		= scp_stop,
	.load		= scp_load,
	.da_to_va	= scp_da_to_va,
};

unsigned int scp_get_vdec_hw_capa(struct platform_device *pdev)
{
	struct mtk_scp *scp = platform_get_drvdata(pdev);

	return scp->run.dec_capability;
}
EXPORT_SYMBOL_GPL(scp_get_vdec_hw_capa);

unsigned int scp_get_venc_hw_capa(struct platform_device *pdev)
{
	struct mtk_scp *scp = platform_get_drvdata(pdev);

	return scp->run.enc_capability;
}
EXPORT_SYMBOL_GPL(scp_get_venc_hw_capa);

void *scp_mapping_dm_addr(struct platform_device *pdev, u32 mem_addr)
{
	struct mtk_scp *scp = platform_get_drvdata(pdev);
	void *ptr;

	ptr = scp_da_to_va(scp->rproc, mem_addr, 0);
	if (!ptr)
		return ERR_PTR(-EINVAL);

	return ptr;
}
EXPORT_SYMBOL_GPL(scp_mapping_dm_addr);

#if SCP_RESERVED_MEM
phys_addr_t scp_mem_base_phys;
phys_addr_t scp_mem_base_virt;
phys_addr_t scp_mem_size;

static struct scp_reserve_mblock scp_reserve_mblock[] = {
	{
		.num = SCP_ISP_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x200000, /*2MB*/
	},
	{
		.num = SCP_ISP_MEM2_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x800000, /*8MB*/
	},
	{
		.num = SCP_DIP_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x900000, /*9MB*/
	},
	{
		.num = SCP_MDP_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x600000, /*6MB*/
	},
	{
		.num = SCP_FD_MEM_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.size = 0x100000, /*1MB*/
	},
};

static int scp_reserve_mem_init(struct mtk_scp *scp)
{
	enum scp_reserve_mem_id_t id;
	phys_addr_t accumlate_memory_size = 0;

	scp_mem_base_phys = (phys_addr_t) (scp->phys_addr + MAX_CODE_SIZE);
	scp_mem_size = (phys_addr_t) (scp->dram_size - MAX_CODE_SIZE);

	dev_info(scp->dev,
		 "phys:0x%llx - 0x%llx (0x%llx)\n",
		 (unsigned long long)scp_mem_base_phys,
		 (unsigned long long)(scp_mem_base_phys + scp_mem_size),
		 (unsigned long long)scp_mem_size);
	accumlate_memory_size = 0;
	for (id = 0; id < SCP_NUMS_MEM_ID; id++) {
		scp_reserve_mblock[id].start_phys =
			scp_mem_base_phys + accumlate_memory_size;
		accumlate_memory_size += scp_reserve_mblock[id].size;
		dev_info(
			scp->dev,
			"[reserve_mem:%d]: phys:0x%llx - 0x%llx (0x%llx)\n", id,
			(unsigned long long)scp_reserve_mblock[id].start_phys,
			(unsigned long long)(scp_reserve_mblock[id].start_phys +
					     scp_reserve_mblock[id].size),
			(unsigned long long)scp_reserve_mblock[id].size);
	}
	return 0;
}

static int scp_reserve_memory_ioremap(struct mtk_scp *scp)
{
	enum scp_reserve_mem_id_t id;
	phys_addr_t accumlate_memory_size = 0;

	scp_mem_base_virt = (phys_addr_t)(size_t)ioremap_wc(scp_mem_base_phys,
							    scp_mem_size);

	dev_info(scp->dev,
		 "virt:0x%llx - 0x%llx (0x%llx)\n",
		(unsigned long long)scp_mem_base_virt,
		(unsigned long long)(scp_mem_base_virt + scp_mem_size),
		(unsigned long long)scp_mem_size);
	for (id = 0; id < SCP_NUMS_MEM_ID; id++) {
		scp_reserve_mblock[id].start_virt =
			scp_mem_base_virt + accumlate_memory_size;
		accumlate_memory_size += scp_reserve_mblock[id].size;
	}
	/* the reserved memory should be larger then expected memory
	 * or scp_reserve_mblock does not match dts
	 */
	WARN_ON(accumlate_memory_size > scp_mem_size);
#ifdef DEBUG
	for (id = 0; id < NUMS_MEM_ID; id++) {
		dev_info(scp->dev,
			 "[mem_reserve-%d] phys:0x%llx,virt:0x%llx,size:0x%llx\n",
			 id,
			 scp_get_reserve_mem_phys(id),
			 scp_get_reserve_mem_virt(id),
			 scp_get_reserve_mem_size(id));
	}
#endif
	return 0;
}
phys_addr_t scp_get_reserve_mem_phys(enum scp_reserve_mem_id_t id)
{
	if (id >= SCP_NUMS_MEM_ID) {
		pr_err("[SCP] no reserve memory for %d", id);
		return 0;
	} else
		return scp_reserve_mblock[id].start_phys;
}
EXPORT_SYMBOL_GPL(scp_get_reserve_mem_phys);

phys_addr_t scp_get_reserve_mem_virt(enum scp_reserve_mem_id_t id)
{
	if (id >= SCP_NUMS_MEM_ID) {
		pr_err("[SCP] no reserve memory for %d", id);
		return 0;
	} else
		return scp_reserve_mblock[id].start_virt;
}
EXPORT_SYMBOL_GPL(scp_get_reserve_mem_virt);

phys_addr_t scp_get_reserve_mem_size(enum scp_reserve_mem_id_t id)
{
	if (id >= SCP_NUMS_MEM_ID) {
		pr_err("[SCP] no reserve memory for %d", id);
		return 0;
	} else
		return scp_reserve_mblock[id].size;
}
EXPORT_SYMBOL_GPL(scp_get_reserve_mem_size);
#endif

static int scp_map_memory_region(struct mtk_scp *scp)
{
	struct device_node *node;
	struct resource r;
	int ret;

	node = of_parse_phandle(scp->dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(scp->dev, "no memory-region specified\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &r);
	if (ret)
		return ret;

	scp->phys_addr = r.start;
	scp->dram_size = resource_size(&r);
	scp->cpu_addr =
		devm_ioremap_wc(scp->dev, scp->phys_addr, scp->dram_size);

	if (!scp->cpu_addr) {
		dev_err(scp->dev, "unable to map memory region: %pa+%zx\n",
			&r.start, scp->dram_size);
		return -EBUSY;
	}

#if SCP_RESERVED_MEM
	scp_reserve_mem_init(scp);
	scp_reserve_memory_ioremap(scp);
#endif
	return 0;
}

static struct mtk_rpmsg_info mtk_scp_rpmsg_info = {
	.send_ipi = scp_ipi_send,
	.register_ipi = scp_ipi_register,
	.unregister_ipi = scp_ipi_unregister,
	.ns_ipi_id = SCP_IPI_NS_SERVICE,
};

static void scp_add_rpmsg_subdev(struct mtk_scp *scp)
{
	scp->rpmsg_subdev =
		mtk_rpmsg_create_rproc_subdev(to_platform_device(scp->dev),
					      &mtk_scp_rpmsg_info);
	if (scp->rpmsg_subdev)
		rproc_add_subdev(scp->rproc, scp->rpmsg_subdev);
}

static void scp_remove_rpmsg_subdev(struct mtk_scp *scp)
{
	if (scp->rpmsg_subdev) {
		rproc_remove_subdev(scp->rproc, scp->rpmsg_subdev);
		mtk_rpmsg_destroy_rproc_subdev(scp->rpmsg_subdev);
		scp->rpmsg_subdev = NULL;
	}
}

static int scp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mtk_scp *scp;
	struct rproc *rproc;
	struct resource *res;
	char *fw_name = "scp.img";
	int ret;

	rproc = rproc_alloc(dev,
			    np->name,
			    &scp_ops,
			    fw_name,
			    sizeof(*scp));
	if (!rproc) {
		dev_err(dev, "unable to allocate remoteproc\n");
		return -ENOMEM;
	}

	scp = (struct mtk_scp *)rproc->priv;
	scp->rproc = rproc;
	scp->dev = dev;
	platform_set_drvdata(pdev, scp);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	scp->sram_base = devm_ioremap_resource(dev, res);
	if (IS_ERR((__force void *)scp->sram_base)) {
		dev_err(dev, "Failed to parse and map sram memory\n");
		ret = PTR_ERR((__force void *)scp->sram_base);
		goto free_rproc;
	}
	scp->sram_size = resource_size(res);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg");
	scp->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR((__force void *)scp->reg_base)) {
		dev_err(dev, "Failed to parse and map cfg memory\n");
		ret = PTR_ERR((__force void *)scp->reg_base);
		goto free_rproc;
	}

	ret = scp_map_memory_region(scp);
	if (ret)
		goto free_rproc;

	scp->clk = devm_clk_get(dev, "main");
	if (IS_ERR(scp->clk)) {
		dev_err(dev, "Failed to get clock\n");
		ret = PTR_ERR(scp->clk);
		goto free_rproc;
	}

	ret = clk_prepare_enable(scp->clk);
	if (ret) {
		dev_err(dev, "failed to enable clocks\n");
		goto free_rproc;
	}

	ret = scp_ipi_init(scp);
	clk_disable_unprepare(scp->clk);
	if (ret) {
		dev_err(dev, "Failed to init ipi\n");
		goto free_rproc;
	}

	/* register SCP initialization IPI */
	ret = scp_ipi_register(pdev,
			       SCP_IPI_INIT,
			       scp_init_ipi_handler,
			       scp);
	if (ret) {
		dev_err(dev, "Failed to register IPI_SCP_INIT\n");
		goto free_rproc;
	}

	mutex_init(&scp->lock);

	init_waitqueue_head(&scp->run.wq);
	init_waitqueue_head(&scp->ack_wq);

	scp_add_rpmsg_subdev(scp);

	ret = devm_request_threaded_irq(dev, platform_get_irq(pdev, 0), NULL,
					scp_irq_handler, IRQF_ONESHOT,
					pdev->name, scp);

	if (ret) {
		dev_err(dev, "failed to request irq\n");
		goto remove_subdev;
	}

	ret = rproc_add(rproc);
	if (ret)
		goto remove_subdev;

	return 0;

remove_subdev:
	scp_remove_rpmsg_subdev(scp);
	mutex_destroy(&scp->lock);
free_rproc:
	rproc_free(rproc);

	return ret;
}

static int scp_remove(struct platform_device *pdev)
{
	struct mtk_scp *scp = platform_get_drvdata(pdev);

	scp_remove_rpmsg_subdev(scp);
	rproc_del(scp->rproc);
	rproc_free(scp->rproc);

	return 0;
}

static const struct of_device_id mtk_scp_of_match[] = {
	{ .compatible = "mediatek,mt8183-scp"},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_scp_of_match);

static struct platform_driver mtk_scp_driver = {
	.probe = scp_probe,
	.remove = scp_remove,
	.driver = {
		.name = "mtk-scp",
		.of_match_table = of_match_ptr(mtk_scp_of_match),
	},
};

module_platform_driver(mtk_scp_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek SCP control driver");

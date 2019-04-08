// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include "governor.h"

struct cci_devfreq {
	struct devfreq		*devfreq;
	struct regulator	*proc_reg;
	unsigned long           proc_reg_uV;
	struct clk		*cci_clk;
	struct notifier_block	nb;
};

static int cci_devfreq_regulator_notifier(struct notifier_block *nb,
					  unsigned long val, void *data)
{
	struct cci_devfreq *cci_df =
		container_of(nb, struct cci_devfreq, nb);

	/* deal with reduce frequency */
	if (val & REGULATOR_EVENT_PRE_VOLTAGE_CHANGE) {
		struct pre_voltage_change_data *pvc_data = data;

		if (pvc_data->min_uV < pvc_data->old_uV) {
			cci_df->proc_reg_uV =
				(unsigned long)(pvc_data->min_uV);
			mutex_lock(&cci_df->devfreq->lock);
			update_devfreq(cci_df->devfreq);
			mutex_unlock(&cci_df->devfreq->lock);
		}
	}

	if ((val & REGULATOR_EVENT_ABORT_VOLTAGE_CHANGE) &&
	    ((unsigned long)data > cci_df->proc_reg_uV)) {
		cci_df->proc_reg_uV = (unsigned long)data;
		mutex_lock(&cci_df->devfreq->lock);
		update_devfreq(cci_df->devfreq);
		mutex_unlock(&cci_df->devfreq->lock);
	}

	/* deal with increase frequency */
	if ((val & REGULATOR_EVENT_VOLTAGE_CHANGE) &&
	    (cci_df->proc_reg_uV < (unsigned long)data)) {
		cci_df->proc_reg_uV = (unsigned long)data;
		mutex_lock(&cci_df->devfreq->lock);
		update_devfreq(cci_df->devfreq);
		mutex_unlock(&cci_df->devfreq->lock);
	}

	return 0;
}

static int mtk_cci_governor_get_target(struct devfreq *devfreq,
				       unsigned long *freq)
{
	struct cci_devfreq *cci_df;
	struct dev_pm_opp *opp;

	cci_df = dev_get_drvdata(devfreq->dev.parent);

	/* find available frequency */
	opp = dev_pm_opp_find_max_freq_by_volt(devfreq->dev.parent,
						 cci_df->proc_reg_uV);
	*freq = dev_pm_opp_get_freq(opp);

	return 0;
}

static int mtk_cci_governor_event_handler(struct devfreq *devfreq,
					  unsigned int event, void *data)
{
	struct cci_devfreq *cci_df;
	struct notifier_block *nb;

	cci_df = dev_get_drvdata(devfreq->dev.parent);
	nb = &cci_df->nb;

	switch (event) {
	case DEVFREQ_GOV_START:
	case DEVFREQ_GOV_RESUME:
		nb->notifier_call = cci_devfreq_regulator_notifier;
		regulator_register_notifier(cci_df->proc_reg,
					    nb);
		break;

	case DEVFREQ_GOV_STOP:
	case DEVFREQ_GOV_SUSPEND:
		regulator_unregister_notifier(cci_df->proc_reg,
					    nb);

		break;

	default:
		break;
	}

	return 0;
}

static struct devfreq_governor mtk_cci_devfreq_governor = {
	.name = "mtk_cci_vmon",
	.get_target_freq = mtk_cci_governor_get_target,
	.event_handler = mtk_cci_governor_event_handler,
};

static int mtk_cci_devfreq_target(struct device *dev, unsigned long *freq,
				  u32 flags)
{
	struct cci_devfreq *cci_df = dev_get_drvdata(dev);

	if (!cci_df)
		return -EINVAL;

	clk_set_rate(cci_df->cci_clk, *freq);

	return 0;
}

static struct devfreq_dev_profile cci_devfreq_profile = {
	.target		= mtk_cci_devfreq_target,
};

static int mtk_cci_devfreq_probe(struct platform_device *pdev)
{
	struct device *cci_dev = &pdev->dev;
	struct cci_devfreq *cci_df;
	int ret;

	cci_df = devm_kzalloc(cci_dev, sizeof(*cci_df), GFP_KERNEL);
	if (!cci_df)
		return -ENOMEM;

	cci_df->cci_clk = clk_get(cci_dev, "cci_clock");
	ret = PTR_ERR_OR_ZERO(cci_df->cci_clk);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(cci_dev, "failed to get clock for CCI: %d\n",
				ret);

		return ret;
	}

	cci_df->proc_reg = regulator_get_optional(cci_dev, "proc");
	ret = PTR_ERR_OR_ZERO(cci_df->proc_reg);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(cci_dev, "failed to get regulator for CCI: %d\n",
				ret);

		goto err_put_clk;
	}

	ret = dev_pm_opp_of_add_table(&pdev->dev);
	if (ret) {
		dev_err(cci_dev, "Fail to init CCI OPP table\n");
		goto err_put_reg;
	}

	platform_set_drvdata(pdev, cci_df);

	cci_df->devfreq = devm_devfreq_add_device(cci_dev,
						       &cci_devfreq_profile,
						       "mtk_cci_vmon",
						       NULL);
	if (IS_ERR(cci_df->devfreq)) {
		ret = PTR_ERR(cci_df->devfreq);
		dev_err(cci_dev, "cannot create cci devfreq device: %d\n", ret);
		goto err_put_reg;
	}

	return 0;

err_put_reg:
	regulator_put(cci_df->proc_reg);
err_put_clk:
	clk_put(cci_df->cci_clk);

	return ret;
}

static const struct of_device_id mediatek_cci_devfreq_of_match[] = {
	{ .compatible = "mediatek,mt8183-cci" },
	{ },
};
MODULE_DEVICE_TABLE(of, mediatek_cci_devfreq_of_match);

static struct platform_driver cci_devfreq_driver = {
	.probe	= mtk_cci_devfreq_probe,
	.driver = {
		.name = "mediatek-cci-devfreq",
		.of_match_table = mediatek_cci_devfreq_of_match,
	},
};

static int __init mtk_cci_devfreq_init(void)
{
	int ret;

	ret = devfreq_add_governor(&mtk_cci_devfreq_governor);
	if (ret) {
		pr_err("%s: failed to add governor: %d\n", __func__, ret);
		return ret;
	}

	ret = platform_driver_register(&cci_devfreq_driver);
	if (ret)
		devfreq_remove_governor(&mtk_cci_devfreq_governor);

	return ret;
}
module_init(mtk_cci_devfreq_init)

static void __exit mtk_cci_devfreq_exit(void)
{
	int ret;

	ret = devfreq_remove_governor(&mtk_cci_devfreq_governor);
	if (ret)
		pr_err("%s: failed to remove governor: %d\n", __func__, ret);

	platform_driver_unregister(&cci_devfreq_driver);
}
module_exit(mtk_cci_devfreq_exit)

MODULE_DESCRIPTION("Mediatek CCI devfreq driver");
MODULE_AUTHOR("Andrew-sh.Cheng <andrew-sh.cheng@mediatek.com>");
MODULE_LICENSE("GPL v2");

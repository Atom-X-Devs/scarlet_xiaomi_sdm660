/*
 * linux/drivers/devfreq/governor_passive.c
 *
 * Copyright (C) 2016 Samsung Electronics
 * Author: Chanwoo Choi <cw00.choi@samsung.com>
 * Author: MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/device.h>
#include <linux/devfreq.h>
#include <linux/of.h>
#include <linux/slab.h>
#include "governor.h"

static unsigned int xlate_cpufreq_to_devfreq(struct devfreq_passive_data *data,
					     unsigned int cpu)
{
	unsigned int cpu_min, cpu_max;
	struct devfreq *devfreq = (struct devfreq *)data->this;
	unsigned int dev_min, dev_max, cpu_percent, cpu_freq = 0, freq = 0;
	unsigned long *freq_table = devfreq->profile->freq_table;
	struct device *dev = devfreq->dev.parent;
	struct devfreq_map *map;
	int opp_cnt, i;

	if (!data->state[cpu] || data->state[cpu]->first_cpu != cpu) {
		freq = 0;
		goto out;
	}

	/* Use Interpolation if map is not available */
	cpu_freq = data->state[cpu]->freq;
	if (!data->map) {
		cpu_min = data->state[cpu]->min_freq;
		cpu_max = data->state[cpu]->max_freq;
		if (freq_table) {
			dev_min = freq_table[0];
			dev_max = freq_table[devfreq->profile->max_state - 1];
		} else {
			if (devfreq->max_freq <= devfreq->min_freq)
				return 0;
			dev_min = devfreq->min_freq;
			dev_max = devfreq->max_freq;
		}

		cpu_percent = ((cpu_freq - cpu_min) * 100) / cpu_max - cpu_min;
		freq = dev_min + mult_frac(dev_max - dev_min, cpu_percent, 100);
		goto out;
	}

	map = data->map[cpu];
	opp_cnt = dev_pm_opp_get_opp_count(dev);
	for (i = 0; i < opp_cnt; i++) {
		freq = max(freq, map[i].dev_hz);
		if (map[i].cpu_khz >= cpu_freq)
			break;
	}
out:
	dev_dbg(dev, "CPU%u: %d -> dev: %u\n", cpu, cpu_freq, freq);
	return freq;
}

static int devfreq_passive_get_target_freq(struct devfreq *devfreq,
					unsigned long *freq)
{
	struct devfreq_passive_data *p_data
			= (struct devfreq_passive_data *)devfreq->data;
	struct devfreq *parent_devfreq = (struct devfreq *)p_data->parent;
	unsigned long child_freq = ULONG_MAX;
	struct dev_pm_opp *opp;
	unsigned int cpu, tgt_freq = 0;
	int i, count, ret = 0;

	/*
	 * If the devfreq device with passive governor has the specific method
	 * to determine the next frequency, should use the get_target_freq()
	 * of struct devfreq_passive_data.
	 */
	if (p_data->get_target_freq) {
		ret = p_data->get_target_freq(devfreq, freq);
		goto out;
	}

	if (p_data->cpufreq_type) {
		for_each_possible_cpu(cpu)
			tgt_freq = max(tgt_freq,
				       xlate_cpufreq_to_devfreq(p_data, cpu));
		*freq = tgt_freq;
		goto out;
	}

	/*
	 * If the parent and passive devfreq device uses the OPP table,
	 * get the next frequency by using the OPP table.
	 */

	/*
	 * - parent devfreq device uses the governors except for passive.
	 * - passive devfreq device uses the passive governor.
	 *
	 * Each devfreq has the OPP table. After deciding the new frequency
	 * from the governor of parent devfreq device, the passive governor
	 * need to get the index of new frequency on OPP table of parent
	 * device. And then the index is used for getting the suitable
	 * new frequency for passive devfreq device.
	 */
	if (!devfreq->profile || !devfreq->profile->freq_table
		|| devfreq->profile->max_state <= 0)
		return -EINVAL;

	/*
	 * The passive governor have to get the correct frequency from OPP
	 * list of parent device. Because in this case, *freq is temporary
	 * value which is decided by ondemand governor.
	 */
	opp = devfreq_recommended_opp(parent_devfreq->dev.parent, freq, 0);
	if (IS_ERR(opp)) {
		ret = PTR_ERR(opp);
		goto out;
	}

	dev_pm_opp_put(opp);

	/*
	 * Get the OPP table's index of decided freqeuncy by governor
	 * of parent device.
	 */
	for (i = 0; i < parent_devfreq->profile->max_state; i++)
		if (parent_devfreq->profile->freq_table[i] == *freq)
			break;

	if (i == parent_devfreq->profile->max_state) {
		ret = -EINVAL;
		goto out;
	}

	/* Get the suitable frequency by using index of parent device. */
	if (i < devfreq->profile->max_state) {
		child_freq = devfreq->profile->freq_table[i];
	} else {
		count = devfreq->profile->max_state;
		child_freq = devfreq->profile->freq_table[count - 1];
	}

	/* Return the suitable frequency for passive device. */
	*freq = child_freq;

out:
	return ret;
}

static int update_devfreq_passive(struct devfreq *devfreq, unsigned long freq)
{
	int ret;

	if (!devfreq->governor)
		return -EINVAL;

	mutex_lock_nested(&devfreq->lock, SINGLE_DEPTH_NESTING);

	ret = devfreq->governor->get_target_freq(devfreq, &freq);
	if (ret < 0)
		goto out;

	ret = devfreq->profile->target(devfreq->dev.parent, &freq, 0);
	if (ret < 0)
		goto out;

	if (devfreq->profile->freq_table
		&& (devfreq_update_status(devfreq, freq)))
		dev_err(&devfreq->dev,
			"Couldn't update frequency transition information.\n");

	devfreq->previous_freq = freq;

out:
	mutex_unlock(&devfreq->lock);

	return 0;
}

static int devfreq_passive_notifier_call(struct notifier_block *nb,
				unsigned long event, void *ptr)
{
	struct devfreq_passive_data *data
			= container_of(nb, struct devfreq_passive_data, nb);
	struct devfreq *devfreq = (struct devfreq *)data->this;
	struct devfreq *parent = (struct devfreq *)data->parent;
	struct devfreq_freqs *freqs = (struct devfreq_freqs *)ptr;
	unsigned long freq = freqs->new;

	switch (event) {
	case DEVFREQ_PRECHANGE:
		if (parent->previous_freq > freq)
			update_devfreq_passive(devfreq, freq);
		break;
	case DEVFREQ_POSTCHANGE:
		if (parent->previous_freq < freq)
			update_devfreq_passive(devfreq, freq);
		break;
	}

	return NOTIFY_DONE;
}

static int cpufreq_passive_notifier_call(struct notifier_block *nb,
					 unsigned long event, void *ptr)
{
	struct devfreq_passive_data *data =
			container_of(nb, struct devfreq_passive_data, nb);
	struct devfreq *devfreq = (struct devfreq *)data->this;
	struct cpufreq_freqs *freq = ptr;
	struct devfreq_cpu_state *state;
	int ret = 0;

	if (event != CPUFREQ_POSTCHANGE)
		goto out;

	state = data->state[freq->cpu];
	if (!state)
		goto out;

	if (state->freq != freq->new) {
		state->freq = freq->new;
		mutex_lock(&devfreq->lock);
		ret = update_devfreq(devfreq);
		mutex_unlock(&devfreq->lock);
		if (ret)
			dev_err(&devfreq->dev, "Frequency update failed.\n");
	}
out:
	return ret;
}

static int cpufreq_passive_register(struct devfreq_passive_data **p_data)
{
	unsigned int cpu;
	struct devfreq_map **cpu_map;
	struct devfreq_map *map, *per_cpu_map;
	struct devfreq_passive_data *data = *p_data;
	struct devfreq *devfreq = (struct devfreq *)data->this;
	int i, count = 0, opp_cnt = 0, ret = 0, iter_val = 0;
	struct device_node *np, *opp_table_np, *cpu_np;
	struct opp_table *opp_table, *cpu_opp_tbl;
	struct device *dev = devfreq->dev.parent;
	struct devfreq_cpu_state *state;
	struct dev_pm_opp *opp, *cpu_opp;
	struct cpufreq_policy *policy;
	struct device *cpu_dev;
	u64 cpu_khz, dev_hz;

	get_online_cpus();
	data->nb.notifier_call = cpufreq_passive_notifier_call;
	ret = cpufreq_register_notifier(&data->nb,
					CPUFREQ_TRANSITION_NOTIFIER);
	if (ret)
		return ret;

	/* Populate devfreq_cpu_state */
	for_each_online_cpu(cpu) {
		if (data->state[cpu])
			continue;

		policy = cpufreq_cpu_get(cpu);
		if (policy) {
			state = kzalloc(sizeof(*state), GFP_KERNEL);
			if (!state)
				return -ENOMEM;

			state->first_cpu = cpumask_first(policy->related_cpus);
			state->freq = policy->cur;
			state->min_freq = policy->cpuinfo.min_freq;
			state->max_freq = policy->cpuinfo.max_freq;
			data->state[cpu] = state;
			cpufreq_cpu_put(policy);
		} else {
			return -EPROBE_DEFER;
		}
	}

	opp_table_np = dev_pm_opp_of_get_opp_desc_node(dev);
	if (!opp_table_np)
		goto out;

	opp_cnt = dev_pm_opp_get_opp_count(dev);
	if (opp_cnt <= 0)
		goto put_opp_table;

	/* Allocate memory for devfreq_map*/
	cpu_map = kcalloc(num_possible_cpus(), sizeof(*cpu_map), GFP_KERNEL);
	if (!cpu_map)
		return -ENOMEM;

	for_each_possible_cpu(cpu) {
		per_cpu_map = kcalloc(opp_cnt, sizeof(*per_cpu_map),
				      GFP_KERNEL);
		if (!per_cpu_map)
			return -ENOMEM;
		cpu_map[cpu] = per_cpu_map;
	}
	data->map = cpu_map;

	/* Populate devfreq_map */
	opp_table = dev_pm_opp_get_opp_table(dev);
	if (!opp_table)
		return -ENOMEM;

	for_each_available_child_of_node(opp_table_np, np) {
		opp = dev_pm_opp_find_opp_of_np(opp_table, np);
		if (IS_ERR(opp))
			continue;

		dev_hz = dev_pm_opp_get_freq(opp);
		dev_pm_opp_put(opp);

		count = of_count_phandle_with_args(np, "required-opps", NULL);
		for (i = 0; i < count; i++) {
			for_each_possible_cpu(cpu) {
				cpu_dev = get_cpu_device(cpu);
				if (!cpu_dev) {
					dev_err(dev, "CPU get device failed.\n");
					continue;
				}

				cpu_np = of_parse_required_opp(np, i);
				if (!cpu_np) {
					dev_err(dev, "Parsing required opp failed.\n");
					continue;
				}

				/* Get cpu opp-table */
				cpu_opp_tbl = dev_pm_opp_get_opp_table(cpu_dev);
				if (!cpu_opp_tbl) {
					dev_err(dev, "CPU opp table get failed.\n");
					goto put_cpu_node;
				}

				/* Match the cpu opp node from required-opp with
				 * the cpu-opp table */
				cpu_opp = dev_pm_opp_find_opp_of_np(cpu_opp_tbl,
								    cpu_np);
				if (!cpu_opp) {
					dev_dbg(dev, "CPU opp get failed.\n");
					goto put_cpu_opp_table;
				}

				cpu_khz = dev_pm_opp_get_freq(cpu_opp);
				if (cpu_opp && cpu_khz) {
					/* Update freq-map if not already set */
					map = cpu_map[cpu];
					map[iter_val].cpu_khz = cpu_khz / 1000;
					map[iter_val].dev_hz = dev_hz;
				}
				dev_pm_opp_put(cpu_opp);
put_cpu_opp_table:
				dev_pm_opp_put_opp_table(cpu_opp_tbl);
put_cpu_node:
				of_node_put(cpu_np);
			}
		}
		iter_val++;
	}
	dev_pm_opp_put_opp_table(opp_table);
put_opp_table:
	of_node_put(opp_table_np);
out:
	put_online_cpus();

	/* Update devfreq */
	mutex_lock(&devfreq->lock);
	ret = update_devfreq(devfreq);
	mutex_unlock(&devfreq->lock);
	if (ret)
		dev_err(dev, "Frequency update failed.\n");

	return ret;
}

static int cpufreq_passive_unregister(struct devfreq_passive_data **p_data)
{
	int cpu;
	struct devfreq_passive_data *data = *p_data;

	cpufreq_unregister_notifier(&data->nb,
				    CPUFREQ_TRANSITION_NOTIFIER);

	for_each_possible_cpu(cpu) {
		kfree(data->state[cpu]);
		kfree(data->map[cpu]);
		data->state[cpu] = NULL;
		data->map[cpu] = NULL;
	}

	kfree(data->map);
	data->map = NULL;

	return 0;
}

static int devfreq_passive_event_handler(struct devfreq *devfreq,
				unsigned int event, void *data)
{
	struct device *dev = devfreq->dev.parent;
	struct devfreq_passive_data *p_data
			= (struct devfreq_passive_data *)devfreq->data;
	struct devfreq *parent = (struct devfreq *)p_data->parent;
	struct notifier_block *nb = &p_data->nb;
	int ret = 0;

	if (!parent && !p_data->cpufreq_type)
		return -EPROBE_DEFER;

	switch (event) {
	case DEVFREQ_GOV_START:
		if (!p_data->this)
			p_data->this = devfreq;

		if (p_data->cpufreq_type) {
			ret = cpufreq_passive_register(&p_data);
		} else {
			nb->notifier_call = devfreq_passive_notifier_call;
			ret = devm_devfreq_register_notifier(dev, parent, nb,
						DEVFREQ_TRANSITION_NOTIFIER);
		}
		break;
	case DEVFREQ_GOV_STOP:
		if (p_data->cpufreq_type) {
			cpufreq_passive_unregister(&p_data);
		} else {
			devm_devfreq_unregister_notifier(dev, parent, nb,
						DEVFREQ_TRANSITION_NOTIFIER);
		}
		break;
	default:
		break;
	}

	return ret;
}

static struct devfreq_governor devfreq_passive = {
	.name = DEVFREQ_GOV_PASSIVE,
	.immutable = 1,
	.get_target_freq = devfreq_passive_get_target_freq,
	.event_handler = devfreq_passive_event_handler,
};

static int __init devfreq_passive_init(void)
{
	return devfreq_add_governor(&devfreq_passive);
}
subsys_initcall(devfreq_passive_init);

static void __exit devfreq_passive_exit(void)
{
	int ret;

	ret = devfreq_remove_governor(&devfreq_passive);
	if (ret)
		pr_err("%s: failed remove governor %d\n", __func__, ret);
}
module_exit(devfreq_passive_exit);

MODULE_AUTHOR("Chanwoo Choi <cw00.choi@samsung.com>");
MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_DESCRIPTION("DEVFREQ Passive governor");
MODULE_LICENSE("GPL v2");

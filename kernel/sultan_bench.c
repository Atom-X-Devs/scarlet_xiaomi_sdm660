#define pr_fmt(fmt) "sultan_bench: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/power_supply.h>
#include <linux/random.h>
#include <linux/kthread.h>

/**************************** CONFIGURATION BEGIN ****************************/
static const int little_cpu_freqs[] = {
#if 0
	300000,
	364800,
	441600,
	518400,
	595200,
	672000,
	748800,
	825600,
	883200,
	960000,
	1036800,
	1094400,
	1171200,
	1248000,
	1324800,
	1401600,
	1478400,
	1555200,
	1670400,
	1747200,
	1824000,
	1900800
#endif
};

static const int big_cpu_freqs[] = {
#if 0
	300000,
	345600,
	422400,
	499200,
	576000,
	652800,
	729600,
	806400,
	902400,
	979200,
	1056000,
	1132800,
	1190400,
	1267200,
	1344000,
	1420800,
	1497600,
	1574400,
	1651200,
	1728000,
	1804800,
	1881600,
	1958400,
	2035200,
	2112000,
	2208000,
	2265600,
	2323200,
	2342400,
	2361600,
	2457600
#endif
};

/* Uncomment to disable power readings */
#define MEASURE_POWER

/* WARNING: Don't bench both clusters at the same time */
const unsigned long cpu_bench_mask = 0b11110000;
/***************************** CONFIGURATION END *****************************/

/* Delay before starting to ensure nothing left from init will interfere */
#define START_DELAY_MS 10000

struct work_data {
	const int *freq_tbl;
	int nr_freqs;
};

static int max_cpu_freq[NR_CPUS];
static volatile int active_count;
static volatile int sub_active_count;
static DECLARE_COMPLETION(bench_round_comp);
static DECLARE_COMPLETION(benchmark_done);
static DEFINE_SPINLOCK(add_lock);

static int add_with_lock(volatile int *dest, int val)
{
	int result;

	spin_lock(&add_lock);
	*dest += val;
	result = *dest;
	spin_unlock(&add_lock);

	return result;
}

static int cpu_notifier_cb(struct notifier_block *nb,
			   unsigned long action, void *data)
{
	struct cpufreq_policy *policy = data;
	int freq;

	if (action != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	if (add_with_lock(&active_count, 0)) {
		freq = max_cpu_freq[policy->cpu];
		if (!freq)
			freq = policy->cpuinfo.min_freq;
	} else {
		freq = policy->cpuinfo.max_freq;
	}

	policy->min = freq;
	policy->max = freq;

	return NOTIFY_OK;
}

static struct notifier_block cpu_notif = {
	.notifier_call = cpu_notifier_cb,
	.priority = INT_MIN
};

static void __set_cpu_freq(int cpu, int freq)
{
	if (cpumask_test_cpu(cpu, cpu_lp_mask))
		cpu = cpumask_first(cpu_lp_mask);
	else
		cpu = cpumask_first(cpu_perf_mask);

	max_cpu_freq[cpu] = freq;
	cpufreq_update_policy(cpumask_first(cpu_lp_mask));
	cpufreq_update_policy(cpumask_first(cpu_perf_mask));
	msleep(100);
}

static void set_cpu_freq(int cpu, int freq)
{
	struct cpufreq_policy policy;

	/* Set CPU freq to absolute min first to reset voltage */
	cpufreq_get_policy(&policy, cpu);
	__set_cpu_freq(cpu, policy.cpuinfo.min_freq);
	__set_cpu_freq(cpu, freq);
}

static void rc4(const u8 *key, size_t key_len, u8 *out, size_t out_len,
		size_t rounds)
{
	u8 i, j, s[256];
	size_t k;

	for (i = 0;; i++) {
		s[i] = i;
		if (i == 255)
			break;
	}

	for (i = 0, j = 0;; i++) {
		j += s[i] + key[i % key_len];
		swap(s[i], s[j]);
		if (i == 255)
			break;
	}

	for (i = 0, j = 0, k = 0; k < out_len * rounds; k++) {
		i += 1;
		j += s[i];
		swap(s[i], s[j]);
		out[k % out_len] = s[(s[i] + s[j]) % 256];
	}
}

static void bench_func(int cpu, int freq)
{
	volatile ktime_t start, stop;
	u8 key[16], out[SZ_1K];

	get_random_bytes(key, sizeof(key));

	pr_info("START: CPU%d: [%7d kHz]\n", cpu, freq);
	start = ktime_get();

	/* Benchmark routine */
	rc4(key, sizeof(key), out, sizeof(out), SZ_4M);

	stop = ktime_get();
	pr_info("STOP: CPU%d: [%7d kHz] [%7lld us]\n", cpu, freq,
		ktime_us_delta(stop, start));
}

static int cpu_bench_thread(void *data)
{
	static const struct sched_param sched_max_rt_prio = {
		.sched_priority = MAX_RT_PRIO - 1
	};
	const int cpu = smp_processor_id();
	struct work_data *w = data;
	int i;

	sched_setscheduler_nocheck(current, SCHED_FIFO, &sched_max_rt_prio);

	/* Migrate all IRQs off the bench CPUs since they can't service them */
	irq_migrate_all_off_this_cpu();
	local_irq_disable();

	for (i = 0; i < w->nr_freqs; i++) {
		/* Synchronize all the benching CPUs */
		add_with_lock(&sub_active_count, -1);
		wait_for_completion(&bench_round_comp);

		set_cpu_freq(cpu, w->freq_tbl[i]);
		bench_func(cpu, cpufreq_get(cpu));
	}

	add_with_lock(&active_count, -1);
	local_irq_enable();

	return 0;
}

static void print_power_usage(struct power_supply *batt_psy)
{
#ifdef MEASURE_POWER
	union power_supply_propval current_val, voltage_val;
	s64 current_ua, voltage_uv, power_mw;
	int ret;

	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW,
					&current_val);
	if (ret)
		return;

	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&voltage_val);
	if (ret)
		return;

	current_ua = current_val.intval;
	voltage_uv = voltage_val.intval;
	power_mw = current_ua * voltage_uv / 1000000000ULL;
	pr_info("power usage [%6lld mW]\n", power_mw);
#endif
}

static int master_thread(void *data)
{
	static const struct sched_param sched_max_rt_prio = {
		.sched_priority = MAX_RT_PRIO - 1
	};
	const int cpu_bench_count = __builtin_popcount(cpu_bench_mask) - 1;
	struct power_supply *batt_psy;
	struct work_data cpuwork[NR_CPUS];
	int i, cpu;

	sched_setscheduler_nocheck(current, SCHED_FIFO, &sched_max_rt_prio);

#ifdef MEASURE_POWER
	batt_psy = power_supply_get_by_name("bms");
	if (!batt_psy) {
		pr_err("failed to get batt supply\n");
		goto exit;
	}
#endif

	if (cpufreq_register_notifier(&cpu_notif, CPUFREQ_POLICY_NOTIFIER)) {
		pr_err("failed to register cpu notifier\n");
		goto exit;
	}

	for_each_cpu_not(cpu, to_cpumask(&cpu_bench_mask))
		BUG_ON(cpu_online(cpu) && cpu_down(cpu));

	for_each_cpu(cpu, to_cpumask(&cpu_bench_mask)) {
		struct work_data *w = &cpuwork[cpu];
		struct task_struct *thread;

		if (cpu == smp_processor_id())
			continue;

		if (cpumask_test_cpu(cpu, cpu_lp_mask)) {
			w->nr_freqs = ARRAY_SIZE(little_cpu_freqs);
			w->freq_tbl = little_cpu_freqs;
		} else {
			w->nr_freqs = ARRAY_SIZE(big_cpu_freqs);
			w->freq_tbl = big_cpu_freqs;
		}

		thread = kthread_create(cpu_bench_thread, w, "bench_thread");
		BUG_ON(IS_ERR(thread));
		kthread_bind(thread, cpu);
		wake_up_process(thread);
	}

	add_with_lock(&active_count, cpu_bench_count);
	add_with_lock(&sub_active_count, cpu_bench_count);

	msleep(START_DELAY_MS);

	while (add_with_lock(&active_count, 0)) {
		if (!add_with_lock(&sub_active_count, 0)) {
			add_with_lock(&sub_active_count, cpu_bench_count);
			for (i = 0; i < cpu_bench_count; i++)
				complete(&bench_round_comp);
		}
		print_power_usage(batt_psy);
		msleep(250);
	}

	/* Reset the CPU freqs to the max before trashing the notifier */
	for_each_cpu(cpu, to_cpumask(&cpu_bench_mask))
		cpufreq_update_policy(cpu);
	cpufreq_unregister_notifier(&cpu_notif, CPUFREQ_POLICY_NOTIFIER);

	for_each_cpu_not(cpu, to_cpumask(&cpu_bench_mask))
		cpu_up(cpu);

exit:
	complete(&benchmark_done);
	return 0;
}

static int __init sultan_bench_init(void)
{
	struct task_struct *thread;
	int cpu;

	for_each_cpu(cpu, to_cpumask(&cpu_bench_mask))
		BUG_ON(!cpu_online(cpu) && cpu_up(cpu));

	thread = kthread_create(master_thread, NULL, "bench_master");
	BUG_ON(IS_ERR(thread));
	kthread_bind(thread, cpumask_first(to_cpumask(&cpu_bench_mask)));
	wake_up_process(thread);
	wait_for_completion(&benchmark_done);

	return 0;
}
late_initcall_sync(sultan_bench_init);

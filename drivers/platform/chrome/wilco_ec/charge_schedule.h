/* SPDX-License-Identifier: GPL-2.0 */
/*
 * EC communication for Peak Shift and Advanced Battery Charging schedules.
 *
 * Copyright 2019 Google LLC
 *
 * See Documentation/ABI/testing/sysfs-platform-wilco-ec for more info.
 */
#ifndef WILCO_EC_CHARGE_SCHEDULE_H
#define WILCO_EC_CHARGE_SCHEDULE_H

#include <linux/platform_data/wilco-ec.h>

/* Property IDs and related EC constants */
#define PID_ADV_CHARGING_SUNDAY			0x04EE
#define PID_ADV_CHARGING			0x04ED
#define PID_PEAK_SHIFT				0x0412
#define PID_PEAK_SHIFT_SUNDAY			0x04F5
#define PID_PEAK_SHIFT_BATTERY_THRESHOLD	0x04EB

#define PEAK_SHIFT_BATTERY_THRESHOLD_MIN	15
#define PEAK_SHIFT_BATTERY_THRESHOLD_MAX	100

struct peak_shift_schedule {
	int day_of_week;		/* 0==Sunday, 1==Monday, ... */
	int start_hours;		/* 0..23 */
	int start_minutes;		/* One of {0, 15, 30, 45} */
	int end_hours;			/* 0..23 */
	int end_minutes;		/* One of {0, 15, 30, 45} */
	int charge_start_hours;		/* 0..23 */
	int charge_start_minutes;	/* One of {0, 15, 30, 45} */
};

struct adv_charge_schedule {
	int day_of_week;	/* 0==Sunday, 1==Monday, ... */
	int start_hours;	/* 0..23 */
	int start_minutes;	/* One of {0, 15, 30, 45} */
	int duration_hours;	/* 0..23 */
	int duration_minutes;	/* One of {0, 15, 30, 45} */
};

/*
 * Return 0 on success, negative error code on failure. For the getters()
 * the sched.day_of_week field should be filled before use. For the setters()
 * all of the sched fields should be filled before use.
 */
int wilco_ec_get_adv_charge_schedule(struct wilco_ec_device *ec,
				     struct adv_charge_schedule *sched);
int wilco_ec_set_adv_charge_schedule(struct wilco_ec_device *ec,
				     const struct adv_charge_schedule *sched);
int wilco_ec_get_peak_shift_schedule(struct wilco_ec_device *ec,
				     struct peak_shift_schedule *sched);
int wilco_ec_set_peak_shift_schedule(struct wilco_ec_device *ec,
				     const struct peak_shift_schedule *sched);

#endif /* WILCO_EC_CHARGE_SCHEDULE_H */

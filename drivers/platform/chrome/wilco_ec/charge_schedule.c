// SPDX-License-Identifier: GPL-2.0
/*
 * EC communication for Peak Shift and Advanced Battery Charging schedules.
 *
 * Copyright 2019 Google LLC
 *
 * See Documentation/ABI/testing/sysfs-platform-wilco-ec for more info.
 */

#include <linux/platform_data/wilco-ec.h>
#include "charge_schedule.h"

/*
 * Date and hour information is passed to/from the EC using packed bytes,
 * where each byte represents an hour and a minute that some event occurs.
 * The minute field supports quarter-hour intervals, so either
 * 0, 15, 30, or 45. This allows this info to be packed within 2 bits.
 * Along with the 5 bits of hour info [0-23], this gives us 7 used bits
 * within each packed byte.
 */

struct adv_charging_payload {
	u16 RESERVED;
	u8 duration_time;
	u8 start_time;
} __packed;

struct peak_shift_payload {
	u8 start_time;
	u8 end_time;
	u8 charge_start_time;
	u8 RESERVED;
} __packed;

/*
 * Peak Shift and Advanced Charging properties each pack the hour and
 * minute info into the bytes differently, hence this struct. For instance,
 * with Peak Shift, the minute info is bits 6 and 7 and the hour info is
 * bits 1 through 5, but with Advanced Charging the minute info is bits 0 and 1
 * and the hour info is bits 2 through 6.
 */
struct time_bcd_format {
	u8 hour_position;
	u8 minute_position;
};

const struct time_bcd_format ADV_CHARGING_BCD_FORMAT = {
	.minute_position = 0,	// bits[0:1]
	.hour_position = 2	// bits[2:6]
				// bit[7] is unused
};

const struct time_bcd_format PEAK_SHIFT_BCD_FORMAT = {
				// bit[0] is unused
	.hour_position = 1,	// bits[1:5]
	.minute_position = 6	// bits[6:7]
};

/* Pack hour and minute info into a byte. */
static u8 pack_field(struct time_bcd_format fmt, int hours, int quarter_hour)
{
	int result = 0;

	quarter_hour /= 15;
	result |= hours << fmt.hour_position;
	result |= quarter_hour << fmt.minute_position;
	return (u8)result;
}

/* Extract hour and minute info from a byte. */
static void unpack_field(struct time_bcd_format fmt, u8 field, int *hour,
			 int *quarter_hour)
{
	*hour =		(field >> fmt.hour_position)   & 0x1f; // 00011111
	*quarter_hour = (field >> fmt.minute_position) & 0x03; // 00000011
	*quarter_hour *= 15;
}

#define hour_valid(h)   (h >= 0 && h < 24)
#define minute_valid(m) (m >= 0 && m < 60 && (m % 15 == 0))

static bool
is_adv_charging_sched_valid(const struct adv_charge_schedule *sched)
{
	return (hour_valid(sched->start_hours) &&
		hour_valid(sched->duration_hours) &&
		minute_valid(sched->start_minutes) &&
		minute_valid(sched->duration_minutes));
}

static bool
is_peak_shift_schedule_valid(const struct peak_shift_schedule *sched)
{
	return (hour_valid(sched->start_hours) &&
		hour_valid(sched->end_hours) &&
		hour_valid(sched->charge_start_hours) &&
		minute_valid(sched->start_minutes) &&
		minute_valid(sched->end_minutes) &&
		minute_valid(sched->charge_start_minutes));
}

int wilco_ec_get_adv_charge_schedule(struct wilco_ec_device *ec,
				     struct adv_charge_schedule *sched)
{
	struct wilco_ec_property_msg msg;
	struct adv_charging_payload *payload;
	int ret;

	msg.property_id = PID_ADV_CHARGING_SUNDAY + sched->day_of_week;
	ret = wilco_ec_get_property(ec, &msg);
	if (ret)
		return ret;

	payload = (struct adv_charging_payload *) msg.data;
	unpack_field(ADV_CHARGING_BCD_FORMAT, payload->start_time,
		     &sched->start_hours, &sched->start_minutes);
	unpack_field(ADV_CHARGING_BCD_FORMAT, payload->duration_time,
		     &sched->duration_hours, &sched->duration_minutes);

	return 0;
}

int wilco_ec_set_adv_charge_schedule(struct wilco_ec_device *ec,
				     const struct adv_charge_schedule *sched)
{
	struct adv_charging_payload *payload;
	struct wilco_ec_property_msg msg;

	if (!is_adv_charging_sched_valid(sched))
		return -EINVAL;

	payload = (struct adv_charging_payload *)msg.data;
	memset(payload, 0, sizeof(*payload));
	payload->start_time = pack_field(ADV_CHARGING_BCD_FORMAT,
					 sched->start_hours,
					 sched->start_minutes);
	payload->duration_time = pack_field(ADV_CHARGING_BCD_FORMAT,
					    sched->duration_hours,
					    sched->duration_minutes);
	msg.length = sizeof(*payload);
	msg.property_id = PID_ADV_CHARGING_SUNDAY + sched->day_of_week;

	return wilco_ec_set_property(ec, &msg);
}

int wilco_ec_get_peak_shift_schedule(struct wilco_ec_device *ec,
				     struct peak_shift_schedule *sched)
{
	struct wilco_ec_property_msg msg;
	struct peak_shift_payload *payload;
	int ret;

	msg.property_id = PID_PEAK_SHIFT_SUNDAY + sched->day_of_week;
	ret = wilco_ec_get_property(ec, &msg);
	if (ret)
		return ret;

	payload = (struct peak_shift_payload *) msg.data;
	unpack_field(PEAK_SHIFT_BCD_FORMAT, payload->start_time,
		     &sched->start_hours, &sched->start_minutes);
	unpack_field(PEAK_SHIFT_BCD_FORMAT, payload->end_time,
		     &sched->end_hours, &sched->end_minutes);
	unpack_field(PEAK_SHIFT_BCD_FORMAT, payload->charge_start_time,
		     &sched->charge_start_hours, &sched->charge_start_minutes);

	return 0;
}

int wilco_ec_set_peak_shift_schedule(struct wilco_ec_device *ec,
				     const struct peak_shift_schedule *sched)
{
	struct peak_shift_payload *payload;
	struct wilco_ec_property_msg msg;

	if (!is_peak_shift_schedule_valid(sched))
		return -EINVAL;

	payload = (struct peak_shift_payload *)msg.data;
	memset(payload, 0, sizeof(*payload));
	payload->start_time = pack_field(PEAK_SHIFT_BCD_FORMAT,
					 sched->start_hours,
					 sched->start_minutes);
	payload->end_time = pack_field(PEAK_SHIFT_BCD_FORMAT,
				       sched->end_hours,
				       sched->end_minutes);
	payload->charge_start_time = pack_field(PEAK_SHIFT_BCD_FORMAT,
						sched->charge_start_hours,
						sched->charge_start_minutes);
	msg.length = sizeof(*payload);
	msg.property_id = PID_PEAK_SHIFT_SUNDAY + sched->day_of_week;

	return wilco_ec_set_property(ec, &msg);
}

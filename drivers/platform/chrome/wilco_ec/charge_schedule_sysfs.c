// SPDX-License-Identifier: GPL-2.0
/*
 * Sysfs interface for Peak Shift and Advanced Battery Charging schedules.
 *
 * Copyright 2019 Google LLC
 *
 * See Documentation/ABI/testing/sysfs-platform-wilco-ec for more info.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_data/wilco-ec.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "charge_schedule.h"

#define DRV_NAME "wilco-charge-schedule"

struct wilco_property_attribute {
	struct device_attribute dev_attr;
	u32 property_id;
};

static ssize_t
bool_property_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct wilco_ec_device *ec = dev_get_platdata(dev);
	struct wilco_property_attribute *wpa;
	u8 val;
	int ret;

	wpa = container_of(attr, struct wilco_property_attribute, dev_attr);
	ret = wilco_ec_get_byte_property(ec, wpa->property_id, &val);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t
bool_property_store(struct device *dev, struct device_attribute *attr,
		    const char *buf, size_t count)
{
	struct wilco_ec_device *ec = dev_get_platdata(dev);
	struct wilco_property_attribute *wpa;
	bool val;
	int ret;

	if (strtobool(buf, &val) < 0)
		return -EINVAL;

	wpa = container_of(attr, struct wilco_property_attribute, dev_attr);
	ret = wilco_ec_set_byte_property(ec, wpa->property_id, (u8) val);
	if (ret < 0)
		return ret;

	return count;
}

#define BOOL_PROPERTY_ATTR(_name, _property_id)				\
	struct wilco_property_attribute _name =	{			\
		__ATTR(_name, 0644, bool_property_show, bool_property_store), \
		_property_id						\
	}

BOOL_PROPERTY_ATTR(peak_shift_enable, PID_PEAK_SHIFT);
BOOL_PROPERTY_ATTR(advanced_charging_enable, PID_ADV_CHARGING);

static ssize_t
peak_shift_battery_threshold_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct wilco_ec_device *ec = dev_get_platdata(dev);
	u8 val;
	int ret;

	ret = wilco_ec_get_byte_property(ec, PID_PEAK_SHIFT_BATTERY_THRESHOLD,
					 &val);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t
peak_shift_battery_threshold_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct wilco_ec_device *ec = dev_get_platdata(dev);
	u8 val;
	int ret;

	if (kstrtou8(buf, 10, &val) < 0)
		return -EINVAL;
	if (val < PEAK_SHIFT_BATTERY_THRESHOLD_MIN ||
	    val > PEAK_SHIFT_BATTERY_THRESHOLD_MAX)
		return -EINVAL;

	ret = wilco_ec_set_byte_property(ec, PID_PEAK_SHIFT_BATTERY_THRESHOLD,
					 val);
	if (ret < 0)
		return ret;

	return count;
}

DEVICE_ATTR_RW(peak_shift_battery_threshold);

struct wilco_schedule_attribute {
	struct device_attribute dev_attr;
	int day_of_week;	/* 0==Sunday, 1==Monday, ... */
};

#define to_wilco_schedule_attr(_dev_attr) \
	container_of(_dev_attr, struct wilco_schedule_attribute, dev_attr)

static ssize_t
advanced_charging_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct wilco_ec_device *ec = dev_get_platdata(dev);
	struct wilco_schedule_attribute *wsa;
	struct adv_charge_schedule sched;
	int ret;

	wsa = to_wilco_schedule_attr(attr);
	sched.day_of_week = wsa->day_of_week;
	ret = wilco_ec_get_adv_charge_schedule(ec, &sched);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%02d:%02d %02d:%02d\n",
			sched.start_hours, sched.start_minutes,
			sched.duration_hours, sched.duration_minutes);
}

static ssize_t
advanced_charging_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct wilco_ec_device *ec = dev_get_platdata(dev);
	struct wilco_schedule_attribute *wsa;
	struct adv_charge_schedule sched;
	int ret;

	ret = sscanf(buf, "%d:%d %d:%d",
		     &sched.start_hours, &sched.start_minutes,
		     &sched.duration_hours, &sched.duration_minutes);
	if (ret != 4)
		return -EINVAL;

	wsa = to_wilco_schedule_attr(attr);
	sched.day_of_week = wsa->day_of_week;
	ret = wilco_ec_set_adv_charge_schedule(ec, &sched);
	if (ret < 0)
		return ret;

	return count;
}

#define ADVANCED_CHARGING_ATTR(_name, _day_of_week)			\
	struct wilco_property_attribute adv_charging_attr_##_name = {	\
		__ATTR(advanced_charging_##_name, 0644,			\
		       advanced_charging_show, advanced_charging_store),\
		_day_of_week						\
	}

ADVANCED_CHARGING_ATTR(sunday, 0);
ADVANCED_CHARGING_ATTR(monday, 1);
ADVANCED_CHARGING_ATTR(tuesday, 2);
ADVANCED_CHARGING_ATTR(wednesday, 3);
ADVANCED_CHARGING_ATTR(thursday, 4);
ADVANCED_CHARGING_ATTR(friday, 5);
ADVANCED_CHARGING_ATTR(saturday, 6);

static ssize_t
peak_shift_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct wilco_ec_device *ec = dev_get_platdata(dev);
	struct wilco_schedule_attribute *wsa;
	struct peak_shift_schedule sched;
	int ret;

	wsa = to_wilco_schedule_attr(attr);
	sched.day_of_week = wsa->day_of_week;
	ret = wilco_ec_get_peak_shift_schedule(ec, &sched);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%02d:%02d %02d:%02d %02d:%02d\n",
			sched.start_hours, sched.start_minutes,
			sched.end_hours, sched.end_minutes,
			sched.charge_start_hours, sched.charge_start_minutes);
}

static ssize_t
peak_shift_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct wilco_ec_device *ec = dev_get_platdata(dev);
	struct wilco_schedule_attribute *wsa;
	struct peak_shift_schedule sched;
	int ret;

	ret = sscanf(buf, "%d:%d %d:%d %d:%d",
		     &sched.start_hours, &sched.start_minutes,
		     &sched.end_hours, &sched.end_minutes,
		     &sched.charge_start_hours, &sched.charge_start_minutes);
	if (ret != 6)
		return -EINVAL;

	wsa = to_wilco_schedule_attr(attr);
	sched.day_of_week = wsa->day_of_week;
	ret = wilco_ec_set_peak_shift_schedule(ec, &sched);
	if (ret < 0)
		return ret;

	return count;
}

#define PEAK_SHIFT_ATTR(_name, _day_of_week)				\
	struct wilco_property_attribute peak_shift_attr_##_name = {	\
		__ATTR(peak_shift_##_name, 0644,			\
		       peak_shift_show, peak_shift_store),		\
		_day_of_week						\
	}

PEAK_SHIFT_ATTR(sunday, 0);
PEAK_SHIFT_ATTR(monday, 1);
PEAK_SHIFT_ATTR(tuesday, 2);
PEAK_SHIFT_ATTR(wednesday, 3);
PEAK_SHIFT_ATTR(thursday, 4);
PEAK_SHIFT_ATTR(friday, 5);
PEAK_SHIFT_ATTR(saturday, 6);

static struct attribute *wilco_charge_schedule_attrs[] = {
	&advanced_charging_enable.dev_attr.attr,
	&adv_charging_attr_sunday.dev_attr.attr,
	&adv_charging_attr_monday.dev_attr.attr,
	&adv_charging_attr_tuesday.dev_attr.attr,
	&adv_charging_attr_wednesday.dev_attr.attr,
	&adv_charging_attr_thursday.dev_attr.attr,
	&adv_charging_attr_friday.dev_attr.attr,
	&adv_charging_attr_saturday.dev_attr.attr,

	&peak_shift_enable.dev_attr.attr,
	&dev_attr_peak_shift_battery_threshold.attr,
	&peak_shift_attr_sunday.dev_attr.attr,
	&peak_shift_attr_monday.dev_attr.attr,
	&peak_shift_attr_tuesday.dev_attr.attr,
	&peak_shift_attr_wednesday.dev_attr.attr,
	&peak_shift_attr_thursday.dev_attr.attr,
	&peak_shift_attr_friday.dev_attr.attr,
	&peak_shift_attr_saturday.dev_attr.attr,
	NULL,
};

static struct attribute_group wilco_charge_schedule_attr_group = {
	.attrs = wilco_charge_schedule_attrs,
};

static int wilco_charge_schedule_probe(struct platform_device *pdev)
{
	return devm_device_add_group(&pdev->dev,
				     &wilco_charge_schedule_attr_group);
}

static struct platform_driver wilco_charge_schedule_driver = {
	.probe	= wilco_charge_schedule_probe,
	.driver = {
		.name = DRV_NAME,
	}
};
module_platform_driver(wilco_charge_schedule_driver);

MODULE_ALIAS("platform:" DRV_NAME);
MODULE_AUTHOR("Nick Crews <ncrews@chromium.org>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Wilco EC charge scheduling driver");

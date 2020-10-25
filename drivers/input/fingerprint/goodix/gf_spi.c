/*
 * TEE driver for goodix fingerprint sensor
 * Copyright (C) 2016 Goodix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_TOUCHSCREEN_COMMON
#include <linux/input/tp_common.h>
#endif
#include <net/sock.h>
#include <net/netlink.h>
#include "gf_spi.h"

#define WAKELOCK_HOLD_TIME		1000	/* in ms */
#define GF_SPIDEV_NAME			"goodix,fingerprint"
#define GF_DEV_NAME			"goodix_fp"
#define GF_INPUT_NAME			"uinput-goodix"
#define CHRD_DRIVER_NAME		"goodix_fp_spi"
#define CLASS_NAME			"goodix_fp"
#define N_SPI_MINORS			32	/* ... up to 256 */

static int SPIDEV_MAJOR;
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct wakeup_source *fp_wakelock;
static struct gf_dev gf;
static int pid = -1;
static struct sock *nl_sk = NULL;
extern int fpsensor;

static struct gf_key_map maps[] = {
	{ EV_KEY, GF_KEY_INPUT_HOME },
	{ EV_KEY, GF_KEY_INPUT_MENU },
	{ EV_KEY, GF_KEY_INPUT_BACK },
	{ EV_KEY, GF_KEY_INPUT_POWER },
#if defined(SUPPORT_NAV_EVENT)
	{ EV_KEY, GF_NAV_INPUT_UP },
	{ EV_KEY, GF_NAV_INPUT_DOWN },
	{ EV_KEY, GF_NAV_INPUT_RIGHT },
	{ EV_KEY, GF_NAV_INPUT_LEFT },
	{ EV_KEY, GF_KEY_INPUT_CAMERA },
	{ EV_KEY, GF_NAV_INPUT_CLICK },
	{ EV_KEY, GF_NAV_INPUT_DOUBLE_CLICK },
	{ EV_KEY, GF_NAV_INPUT_LONG_PRESS },
	{ EV_KEY, GF_NAV_INPUT_HEAVY },
#endif
};

static inline void sendnlmsg(char *msg)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int len = NLMSG_SPACE(MAX_MSGSIZE);
	int ret = 0;

	if (!msg || !nl_sk || !pid)
		return;

	skb = alloc_skb(len, GFP_KERNEL);
	if (!skb)
		return;

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_MSGSIZE, 0);
	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;
	memcpy(NLMSG_DATA(nlh), msg, sizeof(char));
	ret = netlink_unicast(nl_sk, skb, pid, MSG_DONTWAIT);
}

static inline void nl_data_ready(struct sk_buff *__skb)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	char str[100];

	skb = skb_get(__skb);
	if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
		memcpy(str, NLMSG_DATA(nlh), sizeof(str));
		pid = nlh->nlmsg_pid;
		kfree_skb(skb);
	}
}

static inline int netlink_init(void)
{
	struct netlink_kernel_cfg netlink_cfg;

	memset(&netlink_cfg, 0, sizeof(struct netlink_kernel_cfg));
	netlink_cfg.groups = 0;
	netlink_cfg.flags = 0;
	netlink_cfg.input = nl_data_ready;
	netlink_cfg.cb_mutex = NULL;
	nl_sk = netlink_kernel_create(&init_net, NETLINK_TEST, &netlink_cfg);
	if (!nl_sk)
		return 1;

	return 0;
}

static inline void netlink_exit(void)
{
	if (nl_sk != NULL) {
		netlink_kernel_release(nl_sk);
		nl_sk = NULL;
	}
}

static inline int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;

	gf_dev->reset_gpio = of_get_named_gpio(np, "fp-gpio-reset", 0);
	if (gf_dev->reset_gpio < 0)
		return gf_dev->reset_gpio;

	rc = devm_gpio_request(dev, gf_dev->reset_gpio, "goodix_reset");
	if (rc)
		goto err_reset;

	gpio_direction_output(gf_dev->reset_gpio, 0);

	gf_dev->irq_gpio = of_get_named_gpio(np, "fp-gpio-irq", 0);
	if (gf_dev->irq_gpio < 0)
		return gf_dev->irq_gpio;

	rc = devm_gpio_request(dev, gf_dev->irq_gpio, "goodix_irq");
	if (rc)
		goto err_irq;

	gpio_direction_input(gf_dev->irq_gpio);

err_irq:
	devm_gpio_free(dev, gf_dev->irq_gpio);
err_reset:
	devm_gpio_free(dev, gf_dev->reset_gpio);
	return rc;
}

static inline int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (gf_dev == NULL)
		return -1;

	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);

	return 0;
}

static inline void gf_enable_irq(struct gf_dev *gf_dev)
{
	if (!(gf_dev->irq_enabled)) {
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 1;
	}
}

static inline void gf_disable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		gf_dev->irq_enabled = 0;
		disable_irq(gf_dev->irq);
	}
}

static inline irqreturn_t gf_irq(int irq, void *handle)
{
	char msg = GF_NET_EVENT_IRQ;
	struct gf_dev *gf_dev = &gf;

	__pm_wakeup_event(fp_wakelock, WAKELOCK_HOLD_TIME);
	sendnlmsg(&msg);

	return IRQ_HANDLED;
}

static inline int irq_setup(struct gf_dev *gf_dev)
{
	int status;

	gf_dev->irq = gpio_to_irq(gf_dev->irq_gpio);
	status = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
				      IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				      "gf", gf_dev);

	if (status)
		return status;

	enable_irq_wake(gf_dev->irq);
	gf_dev->irq_enabled = 1;

	return status;
}

static inline void irq_cleanup(struct gf_dev *gf_dev)
{
	gf_dev->irq_enabled = 0;
	disable_irq(gf_dev->irq);
	disable_irq_wake(gf_dev->irq);
	free_irq(gf_dev->irq, gf_dev);
}

static inline void nav_event_input(struct gf_dev *gf_dev, gf_nav_event_t nav_event)
{
	uint32_t nav_input = 0;

	switch (nav_event) {
	case GF_NAV_FINGER_DOWN:
		break;
	case GF_NAV_FINGER_UP:
		break;
	case GF_NAV_DOWN:
		nav_input = GF_NAV_INPUT_DOWN;
		break;
	case GF_NAV_UP:
		nav_input = GF_NAV_INPUT_UP;
		break;
	case GF_NAV_LEFT:
		nav_input = GF_NAV_INPUT_LEFT;
		break;
	case GF_NAV_RIGHT:
		nav_input = GF_NAV_INPUT_RIGHT;
		break;
	case GF_NAV_CLICK:
		nav_input = GF_NAV_INPUT_CLICK;
		break;
	case GF_NAV_HEAVY:
		nav_input = GF_NAV_INPUT_HEAVY;
		break;
	case GF_NAV_LONG_PRESS:
		nav_input = GF_NAV_INPUT_LONG_PRESS;
		break;
	case GF_NAV_DOUBLE_CLICK:
		nav_input = GF_NAV_INPUT_DOUBLE_CLICK;
		break;
	default:
		break;
	}

	if ((nav_event != GF_NAV_FINGER_DOWN) &&
	    (nav_event != GF_NAV_FINGER_UP)) {
		input_report_key(gf_dev->input, nav_input, 1);
		input_sync(gf_dev->input);
		input_report_key(gf_dev->input, nav_input, 0);
		input_sync(gf_dev->input);
	}
}

static inline void gf_kernel_key_input(struct gf_dev *gf_dev, struct gf_key *gf_key)
{
	uint32_t key_input = 0;

	switch (gf_key->key) {
	case GF_KEY_HOME:
#ifdef CONFIG_TOUCHSCREEN_COMMON
		if (!capacitive_keys_enabled)
			return;
#endif
		key_input = GF_KEY_INPUT_HOME;
		break;
	case GF_KEY_POWER:
		key_input = GF_KEY_INPUT_POWER;
		break;
	case GF_KEY_CAMERA:
		key_input = GF_KEY_INPUT_CAMERA;
		break;
	default:
		/* add special key define */
		key_input = gf_key->key;
	}

	if ((GF_KEY_POWER == gf_key->key || GF_KEY_CAMERA == gf_key->key) &&
	    (gf_key->value == 1)) {
		input_report_key(gf_dev->input, key_input, 1);
		input_sync(gf_dev->input);
		input_report_key(gf_dev->input, key_input, 0);
		input_sync(gf_dev->input);
	}

	if (gf_key->key == GF_KEY_HOME) {
		input_report_key(gf_dev->input, key_input, gf_key->value);
		input_sync(gf_dev->input);
	}
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = &gf;
	struct gf_key gf_key;
#ifdef SUPPORT_NAV_EVENT
	gf_nav_event_t nav_event = GF_NAV_NONE;
#endif
	int retval = 0;
	u8 netlink_route = NETLINK_TEST;
	struct gf_ioc_chip_info info;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENODEV;

	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (retval)
		return -EFAULT;

	switch (cmd) {
	case GF_IOC_INIT:
		if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8))) {
			retval = -EFAULT;
			break;
		}
		break;
	case GF_IOC_EXIT:
		break;
	case GF_IOC_DISABLE_IRQ:
		gf_disable_irq(gf_dev);
		break;
	case GF_IOC_ENABLE_IRQ:
		gf_enable_irq(gf_dev);
		break;
	case GF_IOC_RESET:
		gf_hw_reset(gf_dev, 3);
		break;
	case GF_IOC_INPUT_KEY_EVENT:
		if (copy_from_user(&gf_key, (void __user *)arg, sizeof(struct gf_key))) {
			retval = -EFAULT;
			break;
		}
		gf_kernel_key_input(gf_dev, &gf_key);
		break;
#ifdef SUPPORT_NAV_EVENT
	case GF_IOC_NAV_EVENT:
		if (copy_from_user(&nav_event, (void __user *)arg, sizeof(gf_nav_event_t))) {
			retval = -EFAULT;
			break;
		}
		nav_event_input(gf_dev, nav_event);
		break;
#endif
	case GF_IOC_ENABLE_SPI_CLK:
		break;
	case GF_IOC_DISABLE_SPI_CLK:
		break;
	case GF_IOC_ENABLE_POWER:
		gf_power_on(gf_dev);
		break;
	case GF_IOC_DISABLE_POWER:
		gf_power_off(gf_dev);
		break;
	case GF_IOC_ENTER_SLEEP_MODE:
		break;
	case GF_IOC_GET_FW_INFO:
		break;
	case GF_IOC_REMOVE:
		break;
	case GF_IOC_CHIP_INFO:
		if (copy_from_user(&info, (void __user *)arg, sizeof(struct gf_ioc_chip_info))) {
			retval = -EFAULT;
			break;
		}
		break;
	default:
		break;
	}

	return retval;
}

#ifdef CONFIG_COMPAT
static inline long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT*/

static inline int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev = &gf;
	int status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		gf_dev->users++;
		filp->private_data = gf_dev;
		nonseekable_open(inode, filp);
		if (gf_dev->users == 1) {
			status = gf_parse_dts(gf_dev);
			if (status)
				goto err_parse_dt;

			status = irq_setup(gf_dev);
			if (status)
				goto err_irq;
		}
		gf_hw_reset(gf_dev, 3);
		gf_dev->device_available = 1;
	}
	mutex_unlock(&device_list_lock);

	return status;
err_irq:
	gf_cleanup(gf_dev);
err_parse_dt:
	return status;
}

static inline int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev = &gf;
	int status = 0;

	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;

	gf_dev->users--;
	if (!gf_dev->users) {
		irq_cleanup(gf_dev);
		gf_cleanup(gf_dev);
		gf_dev->device_available = 0;
		gf_power_off(gf_dev);
	}
	mutex_unlock(&device_list_lock);
	return status;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT*/
	.open = gf_open,
	.release = gf_release,
};

static inline int goodix_fb_state_chg_callback(struct notifier_block *nb,
					       unsigned long val, void *data)
{
	struct gf_dev *gf_dev;
	struct fb_event *evdata = data;
	unsigned int blank;
	char msg = 0;

	if (val != FB_EVENT_BLANK)
		return 0;

	gf_dev = container_of(nb, struct gf_dev, notifier);
	if (evdata && evdata->data && val == FB_EVENT_BLANK && gf_dev) {
		blank = *(int *)(evdata->data);
		switch (blank) {
		case FB_BLANK_POWERDOWN:
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 1;
				gf_dev->wait_finger_down = true;
				if (gf_dev->proximity_state)
					gf_disable_irq(gf_dev);
				msg = GF_NET_EVENT_FB_BLACK;
				sendnlmsg(&msg);
			}
			break;
		case FB_BLANK_UNBLANK:
		case FB_BLANK_NORMAL:
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 0;
				gf_enable_irq(gf_dev);
				msg = GF_NET_EVENT_FB_UNBLACK;
				sendnlmsg(&msg);
			}
			break;
		default:
			break;
		}
	}
	return NOTIFY_OK;
}

static struct notifier_block goodix_noti_block = {
	.notifier_call = goodix_fb_state_chg_callback,
};

static inline ssize_t proximity_state_set(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct gf_dev *gf_dev = dev_get_drvdata(dev);
	int rc, val;

	rc = kstrtoint(buf, 10, &val);
	if (rc)
		return -EINVAL;

	gf_dev->proximity_state = !!val;

	if (gf_dev->fb_black) {
		if (gf_dev->proximity_state)
			/* Disable IRQ when screen is off and proximity sensor is covered */
			gf_disable_irq(gf_dev);
		else
			/* Enable IRQ when screen is off and proximity sensor is uncovered */
			gf_enable_irq(gf_dev);
	}

	return count;
}
static DEVICE_ATTR(proximity_state, S_IWUSR, NULL, proximity_state_set);

static struct attribute *attrs[] = {
	&dev_attr_proximity_state.attr,
	NULL
};

static const struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct class *gf_class;
static int gf_probe(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = &gf;
	int status = -EINVAL;
	unsigned long minor;
	int i;
#if defined(CONFIG_MACH_XIAOMI_LAVENDER) || defined(CONFIG_MACH_XIAOMI_WAYNE)
	struct regulator *vreg;
#endif

	/* Initialize the driver data */
	INIT_LIST_HEAD(&gf_dev->device_entry);
	gf_dev->spi = pdev;
	gf_dev->irq_gpio = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;
	gf_dev->pwr_gpio = -EINVAL;
	gf_dev->device_available = 0;
	gf_dev->fb_black = 0;
	gf_dev->wait_finger_down = false;

#if defined(CONFIG_MACH_XIAOMI_LAVENDER) || defined(CONFIG_MACH_XIAOMI_WAYNE)
	vreg = regulator_get(&gf_dev->spi->dev, "vcc_ana");
	if (!vreg) {
		dev_err(&gf_dev->spi->dev, "Unable to get vdd_ana\n");
		goto error_hw;
	}

	status = regulator_enable(vreg);
	if (status) {
		dev_err(&gf_dev->spi->dev, "error enabling vdd_ana %d\n", status);
		regulator_put(vreg);
		vreg = NULL;
		goto error_hw;
	}
	pr_info("Macle Set voltage on vdd_ana for goodix fingerprint");

	msleep(11);
#endif

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
				    gf_dev, GF_DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		status = -ENODEV;
		mutex_unlock(&device_list_lock);
		goto error_hw;
	}

	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	} else {
		gf_dev->devt = 0;
		goto error_hw;
	}
	mutex_unlock(&device_list_lock);

	gf_dev->input = input_allocate_device();
	if (gf_dev->input == NULL) {
		status = -ENOMEM;
		goto error_dev;
	}
	for (i = 0; i < ARRAY_SIZE(maps); i++)
		input_set_capability(gf_dev->input, maps[i].type, maps[i].code);

	gf_dev->input->name = GF_INPUT_NAME;
	status = input_register_device(gf_dev->input);
	if (status)
		goto error_input;

	gf_dev->notifier = goodix_noti_block;
	fb_register_client(&gf_dev->notifier);

	dev_set_drvdata(&gf_dev->spi->dev, gf_dev);

	status = sysfs_create_group(&gf_dev->spi->dev.kobj, &attr_group);
	if (status)
		goto error_sysfs;

	fp_wakelock = wakeup_source_register(NULL, "fp_wakelock");

	return status;

error_sysfs:
	sysfs_remove_group(&gf_dev->spi->dev.kobj, &attr_group);
error_input:
	if (gf_dev->input != NULL)
		input_free_device(gf_dev->input);
error_dev:
	if (gf_dev->devt != 0) {
		mutex_lock(&device_list_lock);
		list_del(&gf_dev->device_entry);
		device_destroy(gf_class, gf_dev->devt);
		clear_bit(MINOR(gf_dev->devt), minors);
		mutex_unlock(&device_list_lock);
	}
error_hw:
	gf_dev->device_available = 0;

	return status;
}

static inline int gf_remove(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = &gf;

	wakeup_source_unregister(fp_wakelock);
	fb_unregister_client(&gf_dev->notifier);

	if (gf_dev->input)
		input_unregister_device(gf_dev->input);

	input_free_device(gf_dev->input);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct of_device_id gx_match_table[] = {
	{ .compatible = GF_SPIDEV_NAME },
	{},
};

static struct platform_driver gf_driver = {
	.driver = {
		.name = GF_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gx_match_table,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = gf_probe,
	.remove = gf_remove,
};

static inline int __init gf_init(void)
{
	int status;

	if (fpsensor != 2) {
		pr_err("Macle gf_init failed as fpsensor = %d(2=goodix)\n", fpsensor);
		return -1;
	}

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
	if (status < 0) {
		pr_warn("Failed to register char device!\n");
		return status;
	}

	SPIDEV_MAJOR = status;
	gf_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("Failed to create class.\n");
		return PTR_ERR(gf_class);
	}

	status = platform_driver_register(&gf_driver);
	if (status < 0) {
		class_destroy(gf_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("Failed to register SPI driver.\n");
	}
	netlink_init();
	pr_info("status = 0x%x\n", status);
	return 0;
}
module_init(gf_init);

static inline void __exit gf_exit(void)
{
	netlink_exit();
	platform_driver_unregister(&gf_driver);
	class_destroy(gf_class);
	unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_AUTHOR("Jandy Gou, <gouqingsong@goodix.com>");
MODULE_DESCRIPTION("goodix fingerprint sensor device driver");
MODULE_LICENSE("GPL");

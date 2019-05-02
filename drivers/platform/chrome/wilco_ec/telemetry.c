// SPDX-License-Identifier: GPL-2.0
/*
 * Telemetry communication for Wilco EC
 *
 * Copyright 2019 Google LLC
 *
 * The Wilco Embedded Controller is able to send telemetry data
 * which is useful for enterprise applications. A daemon running on
 * the OS sends a command to the EC via a write() to a char device,
 * and can read the response with a read(). The write() request is
 * verified by the driver to ensure that it is performing only one
 * of the whitelisted commands, and that no extraneous data is
 * being transmitted to the EC. The response is passed directly
 * back to the reader with no modification.
 *
 * The character device will appear as /dev/wilco_telemN, where N
 * is some small non-negative integer, starting with 0. Only one
 * process may have the file descriptor open at a time. The calling
 * userspace program needs to keep the device file descriptor open
 * between the calls to write() and read() in order to preserve the
 * response. 32 bytes of data are expected for arguments, and 32
 * bytes will be available for reading.
 *
 * For testing purposes, try requesting the EC's firmware build
 * date, by sending the WILCO_EC_TELEM_GET_VERSION command with
 * argument index=3. i.e. write [0x38, 0x00, 0x03, ...(29 more 0s)]
 * to the device node. An ASCII string of the build date is
 * returned.
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/platform_data/wilco-ec.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define TELEM_DEV_NAME		"wilco_telem"
#define TELEM_CLASS_NAME	TELEM_DEV_NAME
#define DRV_NAME		TELEM_DEV_NAME
#define TELEM_DEV_NAME_FMT	(TELEM_DEV_NAME "%d")
static struct class telem_class = {
	.owner	= THIS_MODULE,
	.name	= TELEM_CLASS_NAME,
};

/* Keep track of all the device numbers used. */
#define TELEM_MAX_DEV 128
static int telem_major;
static DEFINE_IDA(telem_ida);

/* EC telemetry command codes */
#define WILCO_EC_TELEM_GET_LOG			0x99
#define WILCO_EC_TELEM_GET_VERSION		0x38
#define WILCO_EC_TELEM_GET_FAN_INFO		0x2E
#define WILCO_EC_TELEM_GET_DIAG_INFO		0xFA
#define WILCO_EC_TELEM_GET_TEMP_INFO		0x95
#define WILCO_EC_TELEM_GET_TEMP_READ		0x2C
#define WILCO_EC_TELEM_GET_BATT_EXT_INFO	0x07

#define TELEM_ARGS_SIZE_MAX	30

/**
 * struct wilco_ec_telem_request - Telemetry command and arguments sent to EC.
 * @command: One of WILCO_EC_TELEM_GET_* command codes.
 * @reserved: Must be 0.
 * @args: The first N bytes are one of telem_args_get_* structs, the rest is 0.
 */
struct wilco_ec_telem_request {
	u8 command;
	u8 reserved;
	u8 args[TELEM_ARGS_SIZE_MAX];
} __packed;

/*
 * The following telem_args_get_* structs are embedded within the |args| field
 * of wilco_ec_telem_request.
 */

struct telem_args_get_log {
	u8 log_type;
	u8 log_index;
} __packed;

/*
 * Get a piece of info about the EC firmware version:
 * 0 = label
 * 1 = svn_rev
 * 2 = model_no
 * 3 = build_date
 * 4 = frio_version
 */
struct telem_args_get_version {
	u8 index;
} __packed;

struct telem_args_get_fan_info {
	u8 command;
	u8 fan_number;
	u8 arg;
} __packed;

struct telem_args_get_diag_info {
	u8 type;
} __packed;

struct telem_args_get_temp_info {
	u8 command;
	u8 index;
	u8 field;
	u8 zone;
} __packed;

struct telem_args_get_temp_read {
	u8 sensor_index;
} __packed;

struct telem_args_get_batt_ext_info {
	u8 var_args[5];
} __packed;

static const char TELEM_ARGS_ZERO[TELEM_ARGS_SIZE_MAX] = {0};

/**
 * check_args_length() - Ensure that un-needed argument bytes are 0.
 * @rq:	Request to be validated.
 * @arg_size: Number of bytes of rq->args that are allowed to be non-zero.
 */
static int check_args_length(struct wilco_ec_telem_request *rq, size_t arg_size)
{
	if (memcmp(rq->args + arg_size, TELEM_ARGS_ZERO,
		   TELEM_ARGS_SIZE_MAX - arg_size) != 0)
		return -EINVAL;

	return 0;
}

/*
 * We do not want to allow userspace to send arbitrary telemetry commands to
 * the EC. Therefore we check to ensure that
 * 1. The request follows the format of struct wilco_ec_telem_request.
 * 2. The supplied command code is one of the whitelisted commands.
 * 3. The arguments only contain the necessary data. Each command takes a
 *    different format of arguments, and only these arguments can be non-zero.
 *    For instance, if the request uses command WILCO_EC_TELEM_GET_LOG, then
 *    the command uses the argument format of telem_args_get_log, which
 *    contains 2 bytes of arguments. Thus, everything in the args besides
 *    these two bytes had better be zero.
 */
static int check_telem_request(struct wilco_ec_telem_request *rq)
{
	if (rq->reserved)
		return -EINVAL;

	switch (rq->command) {
	case WILCO_EC_TELEM_GET_LOG:
		return check_args_length(rq,
				sizeof(struct telem_args_get_log));
	case WILCO_EC_TELEM_GET_VERSION:
		return check_args_length(rq,
				sizeof(struct telem_args_get_version));
	case WILCO_EC_TELEM_GET_FAN_INFO:
		return check_args_length(rq,
				sizeof(struct telem_args_get_fan_info));
	case WILCO_EC_TELEM_GET_DIAG_INFO:
		return check_args_length(rq,
				sizeof(struct telem_args_get_diag_info));
	case WILCO_EC_TELEM_GET_TEMP_INFO:
		return check_args_length(rq,
				sizeof(struct telem_args_get_temp_info));
	case WILCO_EC_TELEM_GET_TEMP_READ:
		return check_args_length(rq,
				sizeof(struct telem_args_get_temp_read));
	case WILCO_EC_TELEM_GET_BATT_EXT_INFO:
		return check_args_length(rq,
				sizeof(struct telem_args_get_batt_ext_info));
	default:
		return -EINVAL;
	}
}

/**
 * struct telem_device_data - Data for a Wilco EC device that queries telemetry.
 * @cdev: Char dev that userspace reads and polls from.
 * @dev: Device associated with the %cdev.
 * @ec: Wilco EC that we will be communicating with using the mailbox interface.
 * @available: Boolean of if the device can be opened.
 */
struct telem_device_data {
	struct device dev;
	struct cdev cdev;
	struct wilco_ec_device *ec;
	atomic_t available;
};

#define TELEM_RESPONSE_SIZE	EC_MAILBOX_DATA_SIZE

/**
 * struct telem_session_data - Data that exists between open() and release().
 * @dev_data: Pointer to get back to the device data and EC.
 * @request: Command and arguments sent to EC.
 * @response: Response buffer of data from EC.
 * @has_msg: Is there data available to read from a previous write?
 */
struct telem_session_data {
	struct telem_device_data *dev_data;
	struct wilco_ec_telem_request request;
	u8 response[TELEM_RESPONSE_SIZE];
	bool has_msg;
};

/**
 * telem_open() - Callback for when the device node is opened.
 * @inode: inode for this char device node.
 * @filp: file for this char device node.
 *
 * We need to ensure that after writing a command to the device,
 * the same userspace process reads the corresponding result.
 * Therefore, we increment a refcount on opening the device, so that
 * only one process can communicate with the EC at a time.
 *
 * Return: 0 on success, or negative error code on failure.
 */
static int telem_open(struct inode *inode, struct file *filp)
{
	struct telem_device_data *dev_data;
	struct telem_session_data *sess_data;

	/* Ensure device isn't already open */
	dev_data = container_of(inode->i_cdev, struct telem_device_data, cdev);
	if (atomic_cmpxchg(&dev_data->available, 1, 0) == 0)
		return -EBUSY;

	sess_data = kzalloc(sizeof(*sess_data), GFP_KERNEL);
	if (!sess_data) {
		atomic_set(&dev_data->available, 1);
		return -ENOMEM;
	}
	sess_data->dev_data = dev_data;
	sess_data->has_msg = false;

	nonseekable_open(inode, filp);
	filp->private_data = sess_data;

	return 0;
}

static ssize_t telem_write(struct file *filp, const char __user *buf,
			   size_t count, loff_t *pos)
{
	struct telem_session_data *sess_data = filp->private_data;
	struct wilco_ec_message msg = {};
	int ret;

	if (count != sizeof(sess_data->request))
		return -EINVAL;
	if (copy_from_user(&sess_data->request, buf, count))
		return -EFAULT;
	ret = check_telem_request(&sess_data->request);
	if (ret < 0)
		return ret;

	msg.type = WILCO_EC_MSG_TELEMETRY;
	msg.request_data = &sess_data->request;
	msg.request_size = sizeof(sess_data->request);
	msg.response_data = sess_data->response;
	msg.response_size = sizeof(sess_data->response);

	ret = wilco_ec_mailbox(sess_data->dev_data->ec, &msg);
	if (ret < 0)
		return ret;
	if (ret != sizeof(sess_data->response))
		return -EMSGSIZE;

	sess_data->has_msg = true;

	return count;
}

static ssize_t telem_read(struct file *filp, char __user *buf, size_t count,
			  loff_t *pos)
{
	struct telem_session_data *sess_data = filp->private_data;

	if (!sess_data->has_msg)
		return -ENODATA;
	if (count > sizeof(sess_data->response))
		return -EINVAL;

	if (copy_to_user(buf, sess_data->response, count))
		return -EFAULT;

	sess_data->has_msg = false;

	return count;
}

static int telem_release(struct inode *inode, struct file *filp)
{
	struct telem_session_data *sess_data = filp->private_data;

	atomic_set(&sess_data->dev_data->available, 1);
	kfree(sess_data);

	return 0;
}

static const struct file_operations telem_fops = {
	.open = telem_open,
	.write = telem_write,
	.read = telem_read,
	.release = telem_release,
	.llseek = no_llseek,
	.owner = THIS_MODULE,
};

/**
 * telem_device_probe() - Callback when creating a new device.
 * @pdev: platform device that we will be receiving telems from.
 *
 * This finds a free minor number for the device, allocates and initializes
 * some device data, and creates a new device and char dev node.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int telem_device_probe(struct platform_device *pdev)
{
	struct telem_device_data *dev_data;
	dev_t dev_num;
	int error, minor;

	/* Get the next available device number */
	minor = ida_alloc_max(&telem_ida, TELEM_MAX_DEV-1, GFP_KERNEL);
	if (minor < 0) {
		error = minor;
		dev_err(&pdev->dev, "Failed to find minor number: %d", error);
		return error;
	}

	dev_data = devm_kzalloc(&pdev->dev, sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data) {
		ida_simple_remove(&telem_ida, minor);
		return -ENOMEM;
	}

	/* Initialize the device data */
	dev_data->ec = dev_get_platdata(&pdev->dev);
	atomic_set(&dev_data->available, 1);
	platform_set_drvdata(pdev, dev_data);

	/* Initialize the device */
	dev_num = MKDEV(telem_major, minor);
	dev_data->dev.devt = dev_num;
	dev_data->dev.class = &telem_class;
	dev_set_name(&dev_data->dev, TELEM_DEV_NAME_FMT, minor);
	device_initialize(&dev_data->dev);

	/* Initialize the character device and add it to userspace */;
	cdev_init(&dev_data->cdev, &telem_fops);
	error = cdev_device_add(&dev_data->cdev, &dev_data->dev);
	if (error) {
		ida_simple_remove(&telem_ida, minor);
		return error;
	}

	return 0;
}

static int telem_device_remove(struct platform_device *pdev)
{
	struct telem_device_data *dev_data = platform_get_drvdata(pdev);

	cdev_device_del(&dev_data->cdev, &dev_data->dev);
	ida_simple_remove(&telem_ida, MINOR(dev_data->dev.devt));

	return 0;
}

static struct platform_driver telem_driver = {
	.probe = telem_device_probe,
	.remove = telem_device_remove,
	.driver = {
		.name = DRV_NAME,
	},
};

static int __init telem_module_init(void)
{
	dev_t dev_num = 0;
	int ret;

	ret = class_register(&telem_class);
	if (ret) {
		pr_warn(DRV_NAME ": Failed registering class: %d", ret);
		return ret;
	}

	/* Request the kernel for device numbers, starting with minor=0 */
	ret = alloc_chrdev_region(&dev_num, 0, TELEM_MAX_DEV, TELEM_DEV_NAME);
	if (ret) {
		pr_warn(DRV_NAME ": Failed allocating dev numbers: %d", ret);
		goto destroy_class;
	}
	telem_major = MAJOR(dev_num);

	ret = platform_driver_register(&telem_driver);
	if (ret < 0) {
		pr_warn(DRV_NAME ": Failed registering driver: %d\n", ret);
		goto unregister_region;
	}

	return 0;

unregister_region:
	unregister_chrdev_region(MKDEV(telem_major, 0), TELEM_MAX_DEV);
destroy_class:
	class_unregister(&telem_class);
	ida_destroy(&telem_ida);
	return ret;
}

static void __exit telem_module_exit(void)
{
	platform_driver_unregister(&telem_driver);
	unregister_chrdev_region(MKDEV(telem_major, 0), TELEM_MAX_DEV);
	class_unregister(&telem_class);
	ida_destroy(&telem_ida);
}

module_init(telem_module_init);
module_exit(telem_module_exit);

MODULE_AUTHOR("Nick Crews <ncrews@chromium.org>");
MODULE_DESCRIPTION("Wilco EC telemetry driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);

// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 *
 * $Revision: 47247 $
 * $Date: 2019-07-10 10:41:36 +0800 (Wed, 10 Jul 2019) $
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#ifdef CONFIG_DRM_MSM
#include <linux/msm_drm_notify.h>
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include "nt36xxx.h"

#define POINT_DATA_LEN 65

struct nvt_ts_data *ts;
static uint8_t bTouchIsAwake = 0;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

#ifdef CONFIG_DRM_MSM
static int nvt_drm_notifier_callback(struct notifier_block *self,
				     unsigned long event, void *data);
#elif defined(CONFIG_FB)
static int nvt_fb_notifier_callback(struct notifier_block *self,
				    unsigned long event, void *data);
#endif

#if WAKEUP_GESTURE
#define GESTURE_WORD_C 12
#define GESTURE_WORD_W 13
#define GESTURE_WORD_V 14
#define GESTURE_DOUBLE_CLICK 15
#define GESTURE_WORD_Z 16
#define GESTURE_WORD_M 17
#define GESTURE_WORD_O 18
#define GESTURE_WORD_E 19
#define GESTURE_WORD_S 20
#define GESTURE_SLIDE_UP 21
#define GESTURE_SLIDE_DOWN 22
#define GESTURE_SLIDE_LEFT 23
#define GESTURE_SLIDE_RIGHT 24
#define DATA_PROTOCOL 30
#define FUNCPAGE_GESTURE 1

const uint16_t gesture_key_array[] = {
	KEY_POWER, /* GESTURE_WORD_C */
	KEY_POWER, /* GESTURE_WORD_W */
	KEY_POWER, /* GESTURE_WORD_V */
	KEY_POWER, /* GESTURE_DOUBLE_CLICK */
	KEY_POWER, /* GESTURE_WORD_Z */
	KEY_POWER, /* GESTURE_WORD_M */
	KEY_POWER, /* GESTURE_WORD_O */
	KEY_POWER, /* GESTURE_WORD_E */
	KEY_POWER, /* GESTURE_WORD_S */
	KEY_POWER, /* GESTURE_SLIDE_UP */
	KEY_POWER, /* GESTURE_SLIDE_DOWN */
	KEY_POWER, /* GESTURE_SLIDE_LEFT */
	KEY_POWER, /* GESTURE_SLIDE_RIGHT */
};

inline void nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];

	/* support fw specifal data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE))
		gesture_id = func_id;
	else if (gesture_id > DATA_PROTOCOL)
		return;

	switch (gesture_id) {
	case GESTURE_WORD_C:
		keycode = gesture_key_array[0];
		break;
	case GESTURE_WORD_W:
		keycode = gesture_key_array[1];
		break;
	case GESTURE_WORD_V:
		keycode = gesture_key_array[2];
		break;
	case GESTURE_DOUBLE_CLICK:
		keycode = gesture_key_array[3];
		break;
	case GESTURE_WORD_Z:
		keycode = gesture_key_array[4];
		break;
	case GESTURE_WORD_M:
		keycode = gesture_key_array[5];
		break;
	case GESTURE_WORD_O:
		keycode = gesture_key_array[6];
		break;
	case GESTURE_WORD_E:
		keycode = gesture_key_array[7];
		break;
	case GESTURE_WORD_S:
		keycode = gesture_key_array[8];
		break;
	case GESTURE_SLIDE_UP:
		keycode = gesture_key_array[9];
		break;
	case GESTURE_SLIDE_DOWN:
		keycode = gesture_key_array[10];
		break;
	case GESTURE_SLIDE_LEFT:
		keycode = gesture_key_array[11];
		break;
	case GESTURE_SLIDE_RIGHT:
		keycode = gesture_key_array[12];
		break;
	default:
		break;
	}

	if (keycode > 0) {
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}
}
#endif

static inline void nvt_irq_enable(bool enable)
{
	if (enable) {
		if (!ts->irq_enabled) {
			enable_irq(ts->client->irq);
			ts->irq_enabled = true;
		}
	} else {
		if (ts->irq_enabled) {
			disable_irq(ts->client->irq);
			ts->irq_enabled = false;
		}
	}
}

inline int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf,
			    uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;

	mutex_lock(&ts->xbuf_lock);

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = address;
	msgs[0].len = 1;
	msgs[0].buf = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = address;
	msgs[1].len = len - 1;
	msgs[1].buf = ts->xbuf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	memcpy(buf + 1, ts->xbuf, len - 1);

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

inline int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf,
			     uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;

	mutex_lock(&ts->xbuf_lock);

	msg.flags = !I2C_M_RD;
	msg.addr = address;
	msg.len = len;
	memcpy(ts->xbuf, buf, len);
	msg.buf = ts->xbuf;

	ret = i2c_transfer(client->adapter, &msg, 1);

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

inline int32_t nvt_set_page(uint16_t i2c_addr, uint32_t addr)
{
	uint8_t buf[4] = { 0 };

	buf[0] = 0xFF;
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;

	return CTP_I2C_WRITE(ts->client, i2c_addr, buf, 3);
}

inline void nvt_sw_reset_idle(void)
{
	uint8_t buf[4] = { 0 };

	buf[0] = 0x00;
	buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(15);
}

inline void nvt_bootloader_reset(void)
{
	uint8_t buf[8] = { 0 };

	buf[0] = 0x00;
	buf[1] = 0x69;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(35);
}

inline int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = { 0 };

	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR |
		     EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0xFF;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

	return 0;
}

inline int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = { 0 };

	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR |
		     EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

	return 0;
}

inline int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = { 0 };

	buf[0] = EVENT_MAP_RESET_COMPLETE;
	buf[1] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 6);

	return 0;
}

inline int32_t nvt_read_pid(void)
{
	uint8_t buf[3] = { 0 };

	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	return 0;
}

inline int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = { 0 };
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

	buf[0] = EVENT_MAP_FWINFO;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 17);
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);

	if ((buf[1] + buf[2]) != 0xFF) {
		ts->x_num = 18;
		ts->y_num = 32;
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;

		if (retry_count < 3) {
			retry_count++;
			goto info_retry;
		} else
			ret = -1;
	}

	nvt_read_pid();
	return ret;
}

static inline void nvt_parse_dt(struct device *dev)
{
#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;

#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0,
						 &ts->reset_flags);
#endif
	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0,
					       &ts->irq_flags);

#else
#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = NVT_TOUCH_RST_PIN;
#endif
	ts->irq_gpio = NVT_TOUCH_INT_PIN;
#endif
}

static inline int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_HIGH,
				       "NVT-tp-rst");
		if (ret)
			goto err_request_reset_gpio;
	}
#endif

	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret)
			goto err_request_irq_gpio;
	}

	return ret;

err_request_irq_gpio:
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_free(ts->reset_gpio);
err_request_reset_gpio:
#endif
	return ret;
}

static inline void nvt_gpio_deconfig(struct nvt_ts_data *ts)
{
	if (gpio_is_valid(ts->irq_gpio))
		gpio_free(ts->irq_gpio);
#if NVT_TOUCH_SUPPORT_HW_RST
	if (gpio_is_valid(ts->reset_gpio))
		gpio_free(ts->reset_gpio);
#endif
}

static inline uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	for (i = 1; i < 7; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

static inline irqreturn_t nvt_ts_work_func(int irq, void *data)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 1] = { 0 };
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint8_t input_id = 0;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = { 0 };
	int32_t i = 0;
	int32_t finger_cnt = 0;

#if WAKEUP_GESTURE
	if (unlikely(!bTouchIsAwake))
		pm_wakeup_event(&ts->input_dev->dev, 5000);
#endif

	mutex_lock(&ts->lock);

	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data,
			   POINT_DATA_LEN + 1);
	if (unlikely(ret < 0))
		goto XFER_ERROR;

	if (nvt_fw_recovery(point_data))
		goto XFER_ERROR;

#if WAKEUP_GESTURE
	if (unlikely(!bTouchIsAwake)) {
		input_id = (uint8_t)(point_data[1] >> 3);
		nvt_ts_wakeup_gesture_report(input_id, point_data);
		goto XFER_ERROR;
	}
#endif

	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position] >> 3);

		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		if (likely(((point_data[position] & 0x07) == 0x01) ||
			   ((point_data[position] & 0x07) == 0x02))) {
			input_x = (uint32_t)(point_data[position + 1] << 4) +
				  (uint32_t)(point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) +
				  (uint32_t)(point_data[position + 3] & 0x0F);
			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, TOUCH_FORCE_NUM);
			finger_cnt++;
		}
	}

	for (i = 0; i < ts->max_touch_num; i++) {
		if (likely(press_id[i] != 1)) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
	input_sync(ts->input_dev);

XFER_ERROR:
	mutex_unlock(&ts->lock);
	return IRQ_HANDLED;
}

inline void nvt_stop_crc_reboot(void)
{
	uint8_t buf[8] = { 0 };
	int32_t retry = 0;

	nvt_set_page(I2C_BLDR_Address, 0x1F64E);

	buf[0] = 0x4E;
	CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 4);

	if ((buf[1] == 0xFC) ||
	    ((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {
		for (retry = 5; retry > 0; retry--) {
			buf[0] = 0x00;
			buf[1] = 0xA5;
			CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

			buf[0] = 0x00;
			buf[1] = 0xA5;
			CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
			msleep(1);

			nvt_set_page(I2C_BLDR_Address, 0x3F135);

			buf[0] = 0x35;
			buf[1] = 0xA5;
			CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 2);

			nvt_set_page(I2C_BLDR_Address, 0x3F135);

			buf[0] = 0x35;
			buf[1] = 0x00;
			CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 2);

			if (buf[1] == 0xA5)
				break;
		}
	}

	return;
}

static inline int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = { 0 };
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	nvt_bootloader_reset();

	for (retry = 5; retry > 0; retry--) {
		nvt_sw_reset_idle();

		buf[0] = 0x00;
		buf[1] = 0x35;
		CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		msleep(10);

		nvt_set_page(I2C_BLDR_Address, 0x1F64E);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 7);

		if ((buf[1] == 0xFC) || ((buf[1] == 0xFF) && (buf[2] == 0xFF) &&
		    (buf[3] == 0xFF))) {
			nvt_stop_crc_reboot();
			continue;
		}

		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table));
		     list++) {
			found_nvt_chip = 0;

			for (i = 0; i < NVT_ID_BYTE_MAX; i++)
				if (trim_id_table[list].mask[i])
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;

			if (i == NVT_ID_BYTE_MAX)
				found_nvt_chip = 1;

			if (found_nvt_chip) {
				ts->mmap = trim_id_table[list].mmap;
				ts->carrier_system = trim_id_table[list].hwinfo->carrier_system;
				ret = 0;
				goto out;
			} else
				ts->mmap = NULL;
		}

		msleep(10);
	}

out:
	return ret;
}

static inline int32_t nvt_ts_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	int32_t ret = 0;
#if WAKEUP_GESTURE
	int32_t retry = 0;
#endif

	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	nvt_parse_dt(&client->dev);

	ret = nvt_gpio_config(ts);
	if (ret)
		goto err_gpio_config_failed;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);

	msleep(10);

#ifdef CONFIG_XIAOMI_SDM660
	ts->vcc_i2c = regulator_get(&client->dev, "vcc_i2c-supply");
	if (!IS_ERR(ts->vcc_i2c)) {
		ret = regulator_set_voltage(ts->vcc_i2c, 1800000, 1800000);
		if (!ret) {
			ret = regulator_enable(ts->vcc_i2c);
			if (ret) {
				pr_err("%s: Enabling regulator vcc_i2c failed\n", __func__);
				regulator_put(ts->vcc_i2c);
			}
		}
	}
#endif

	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	nvt_get_fw_info();

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;
	ts->int_trigger_type = INT_TRIGGER_TYPE;
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
#endif

#if WAKEUP_GESTURE
	for (retry = 0; retry < ARRAY_SIZE(gesture_key_array); retry++)
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);
#endif

	snprintf(ts->phys, sizeof(ts->phys), "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	ret = input_register_device(ts->input_dev);
	if (ret)
		goto err_input_register_device_failed;

	client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
		ts->irq_enabled = true;
		ret = request_threaded_irq(client->irq, NULL, nvt_ts_work_func,
					   ts->int_trigger_type | IRQF_ONESHOT,
					   NVT_I2C_NAME, ts);
		if (ret != 0)
			goto err_int_request_failed;
		else
			nvt_irq_enable(false);
	}

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 1);
#endif

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = alloc_workqueue("nvt_fwu_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!nvt_fwu_wq) {
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));
#endif

#ifdef CONFIG_DRM_MSM
	ts->drm_notif.notifier_call = nvt_drm_notifier_callback;
	ret = msm_drm_register_client(&ts->drm_notif);
	if (ret)
		goto err_register_drm_notif_failed;
#elif defined(CONFIG_FB)
	ts->fb_notif.notifier_call = nvt_fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		goto err_register_fb_notif_failed;
#endif

	bTouchIsAwake = 1;

	nvt_irq_enable(true);
	return 0;

#ifdef CONFIG_DRM_MSM
err_register_drm_notif_failed:
#elif defined(CONFIG_FB)
err_register_fb_notif_failed:
#endif
#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
err_create_nvt_fwu_wq_failed:
#endif
#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif
	free_irq(client->irq, ts);
err_int_request_failed:
	input_unregister_device(ts->input_dev);
	ts->input_dev = NULL;
err_input_register_device_failed:
	if (ts->input_dev) {
		input_free_device(ts->input_dev);
		ts->input_dev = NULL;
	}
err_input_dev_alloc_failed:
err_chipvertrim_failed:
	nvt_gpio_deconfig(ts);
	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);
err_check_functionality_failed:
	nvt_gpio_deconfig(ts);
err_gpio_config_failed:
	i2c_set_clientdata(client, NULL);
	if (ts) {
		kfree(ts);
		ts = NULL;
	}
	return ret;
}

static inline int32_t nvt_ts_remove(struct i2c_client *client)
{
#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if WAKEUP_GESTURE
	if (ts->input_dev)
		device_init_wakeup(&ts->input_dev->dev, 0);
#endif

	nvt_irq_enable(false);
	free_irq(client->irq, ts);

	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);

	nvt_gpio_deconfig(ts);

	if (ts->input_dev) {
		input_unregister_device(ts->input_dev);
		ts->input_dev = NULL;
	}

	i2c_set_clientdata(client, NULL);

	if (ts) {
		kfree(ts);
		ts = NULL;
	}

	return 0;
}

static inline void nvt_ts_shutdown(struct i2c_client *client)
{
	nvt_irq_enable(false);

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if WAKEUP_GESTURE
	if (ts->input_dev)
		device_init_wakeup(&ts->input_dev->dev, 0);
#endif
}

static inline int32_t nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = { 0 };
	uint32_t i = 0;

	if (!bTouchIsAwake)
		return 0;

#if !WAKEUP_GESTURE
	nvt_irq_enable(false);
#endif

	mutex_lock(&ts->lock);

	bTouchIsAwake = 0;

#if WAKEUP_GESTURE
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x13;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

	enable_irq_wake(ts->client->irq);
#else
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

	nvt_set_page(I2C_FW_Address, 0x11a50);
	buf[0] = 0x11a50 & 0xff;
	buf[1] = 0x11;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#endif

	mutex_unlock(&ts->lock);

	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);

	msleep(50);
	return 0;
}

static inline int32_t nvt_ts_resume(struct device *dev)
{
	if (bTouchIsAwake)
		return 0;

	mutex_lock(&ts->lock);

#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif

	if (nvt_check_fw_reset_state(RESET_STATE_REK)) {
		nvt_bootloader_reset();
		nvt_check_fw_reset_state(RESET_STATE_REK);
	}

#if !WAKEUP_GESTURE
	nvt_irq_enable(true);
#endif

	bTouchIsAwake = 1;

	mutex_unlock(&ts->lock);
	return 0;
}

#ifdef CONFIG_DRM_MSM
static inline int nvt_drm_notifier_callback(struct notifier_block *self,
					    unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	unsigned int *blank = evdata->data;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, drm_notif);

	if (evdata->id != 0)
		return 0;

	if (evdata && evdata->data) {
		switch (event) {
		case MSM_DRM_EARLY_EVENT_BLANK:
			if (*blank == MSM_DRM_BLANK_POWERDOWN)
				nvt_ts_suspend(&ts->client->dev);
			break;
		case MSM_DRM_EVENT_BLANK:
			if (*blank == MSM_DRM_BLANK_UNBLANK)
				nvt_ts_resume(&ts->client->dev);
			break;
		default:
			break;
		}
	}

	return 0;
}

#elif defined(CONFIG_FB)
static inline int nvt_fb_notifier_callback(struct notifier_block *self,
					   unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	unsigned int *blank = evdata->data;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, fb_notif);

	if (evdata && evdata->data) {
		switch (event) {
		case FB_EARLY_EVENT_BLANK:
			if (*blank == FB_BLANK_POWERDOWN)
				nvt_ts_suspend(&ts->client->dev);
			break;
		case FB_EVENT_BLANK:
			if (*blank == FB_BLANK_UNBLANK)
				nvt_ts_resume(&ts->client->dev);
			break;
		default:
			break;
		}
	}

	return 0;
}
#endif

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{},
};

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts", },
	{},
};
#endif

static struct i2c_driver nvt_i2c_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.shutdown	= nvt_ts_shutdown,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_I2C_NAME,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

static inline int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	ret = i2c_add_driver(&nvt_i2c_driver);
	if (ret)
		return ret;

	return ret;
}

static inline void __exit nvt_driver_exit(void)
{
	i2c_del_driver(&nvt_i2c_driver);
}

module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");

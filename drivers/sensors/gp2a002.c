/*
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>

#include <linux/sensor/sensors_core.h>
#include "gp2a002.h"

#define REGS_PROX		0x0 /* Read  Only */
#define REGS_GAIN		0x1 /* Write Only */
#define REGS_HYS		0x2 /* Write Only */
#define REGS_CYCLE		0x3 /* Write Only */
#define REGS_OPMOD		0x4 /* Write Only */
#define REGS_CON		0x6 /* Write Only */

#if defined(CONFIG_SEC_FORTUNA_PROJECT)
#define PROX_NONDETECT			0x40
#define PROX_DETECT				0x20
#else
#define PROX_NONDETECT			0x2F
#define PROX_DETECT				0x0F
#endif
#define PROX_NONDETECT_MODE1	0x43
#define PROX_DETECT_MODE1		0x28
#define PROX_NONDETECT_MODE2	0x48
#define PROX_DETECT_MODE2		0x42
#define OFFSET_FILE_PATH		"/efs/prox_cal"

#define PROXIMITY	1
#define CHIP_DEV_NAME	"GP2AP002"
#define CHIP_DEV_VENDOR	"SHARP"

struct gp2a_data {
#if defined(CONFIG_SENSORS_IIO)
	struct iio_dev *indio_dev;
#else
	struct input_dev *input;
#endif
	struct device *dev;
	struct gp2a_platform_data *pdata;
	struct i2c_client *i2c_client;
	struct mutex power_lock;
	struct wake_lock prx_wake_lock;
	struct workqueue_struct *wq;
	struct work_struct work_prox;

	int irq;
	int power_state;
	char val_state;
	char cal_mode;

	u8 detect;
	u8 nondetect;
#if defined(CONFIG_SENSORS_IIO)
	u64 timestamp;
#endif
};

#if defined(CONFIG_SENSORS_IIO)
#define IIO_BUFFER_1_BYTES	9 /* data 1 bytes + timestamp 8 bytes */

static const struct iio_chan_spec gp2a_channels[] = {
	{
		.type = IIO_PROXIMITY,
		.channel = -1,
		.scan_index = 0,
		.scan_type = IIO_ST('s', IIO_BUFFER_1_BYTES * 8,
			IIO_BUFFER_1_BYTES * 8, 0)
	}
};
#endif

int gp2a_i2c_read(struct gp2a_data *gp2a, u8 reg, u8 *val)
{
	int err = 0;
	unsigned char data[2] = {reg, 0};
	int retry = 10;
	struct i2c_msg msg[2] = {};
	struct i2c_client *client = gp2a->i2c_client;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = data;

	msg[1].addr = client->addr;
	msg[1].flags = 1;
	msg[1].len = 2;
	msg[1].buf = data;

	while (retry--) {
		data[0] = reg;

		err = i2c_transfer(client->adapter, msg, 2);

		if (err >= 0) {
			*val = data[1];
			return 0;
		}
	}

	pr_err("%s : i2c transfer error ret = %d\n", __func__, err);

	return err;
}

int gp2a_i2c_write(struct gp2a_data *gp2a, u8 reg, u8 val)
{
	int err = -1;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 10;
	struct i2c_client *client = gp2a->i2c_client;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	while (retry--) {
		data[0] = reg;
		data[1] = val;

		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 2;
		msg->buf = data;

		err = i2c_transfer(client->adapter, msg, 1);

		if (err >= 0)
			return 0;
	}

	pr_err("%s : i2c transfer error ret= %d\n", __func__, err);

	return err;
}
static int gp2a_leda_onoff(struct gp2a_data *gp2a, int power)
{
	int ret;

#if defined(CONFIG_SENSORS_LEDA_EN_GPIO)
	if (power)
		ret = gpio_direction_output(gp2a->pdata->power_en, 1);
	else
		ret = gpio_direction_output(gp2a->pdata->power_en, 0);

	if (ret < 0)
		pr_err("%s, error for direction\n", __func__);
#else
	struct regulator *gp2a_leda;

	pr_info("%s %s\n", __func__, (power) ? "on" : "off");

	gp2a_leda = devm_regulator_get(&gp2a->i2c_client->dev, "gp2a-leda");
	if (IS_ERR(gp2a_leda)) {
		pr_err("[SENSOR]: %s - cannot get gp2a_leda\n", __func__);
		return -ENOMEM;
	}

	if (power) {
		ret = regulator_enable(gp2a_leda);
		if (ret) {
			pr_err("%s: enable gp2a_leda failed, rc=%d\n",
				__func__, ret);
			return ret;
		}
	} else {
		ret = regulator_disable(gp2a_leda);
		if (ret) {
			pr_err("%s: disable gp2a_leda failed, rc=%d\n",
				__func__, ret);
			return ret;
		}
	}

	devm_regulator_put(gp2a_leda);
	msleep(20);
#endif
	return 0;
}
static int gp2a_power_onoff(struct gp2a_data *gp2a, int power)
{
	u8 value;
	pr_info("%s,status(%d)\n", __func__, power);

	if (power) {
		gp2a_leda_onoff(gp2a, power);
		value = 0x18;
		gp2a_i2c_write(gp2a, REGS_CON, value);
		value = 0x08;
		gp2a_i2c_write(gp2a, REGS_GAIN, value);
		value = gp2a->nondetect;
		gp2a_i2c_write(gp2a, REGS_HYS, value);
		value = 0x04;
		gp2a_i2c_write(gp2a, REGS_CYCLE, value);
		value = 0x03;
		gp2a_i2c_write(gp2a, REGS_OPMOD, value);

		enable_irq_wake(gp2a->irq);
		enable_irq(gp2a->irq);

		value = 0x00;
		gp2a_i2c_write(gp2a, REGS_CON, value);
	} else {
		disable_irq_wake(gp2a->irq);
		disable_irq(gp2a->irq);

		value = 0x02;
		gp2a_i2c_write(gp2a, REGS_OPMOD, value);
		gp2a_leda_onoff(gp2a, power);
	}
	return 0;
}

static ssize_t adc_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", gp2a->val_state);
}

static ssize_t state_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", gp2a->val_state);
}

static ssize_t name_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_DEV_NAME);
}

static ssize_t vendor_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_DEV_VENDOR);
}

static int gp2a_cal_mode_read_file(struct gp2a_data *gp2a)
{
	int err = 0;
	mm_segment_t old_fs;
	struct file *cal_mode_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_mode_filp = filp_open(OFFSET_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_mode_filp)) {
		err = PTR_ERR(cal_mode_filp);
		if (err != -ENOENT)
			pr_err("%s,Can't open cal_mode file\n", __func__);
		set_fs(old_fs);
		return err;
	}
	err = cal_mode_filp->f_op->read(cal_mode_filp,
		(char *)&gp2a->cal_mode,
		sizeof(u8), &cal_mode_filp->f_pos);

	if (err != sizeof(u8)) {
		pr_err("%s,Can't read the cal_mode from file\n",
			__func__);
		filp_close(cal_mode_filp, current->files);
		set_fs(old_fs);
		return -EIO;
	}

	filp_close(cal_mode_filp, current->files);
	set_fs(old_fs);

	return err;
}

static int gp2a_cal_mode_save_file(char mode)
{
	struct file *cal_mode_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_mode_filp = filp_open(OFFSET_FILE_PATH,
		O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_mode_filp)) {
		pr_err("%s,Can't open cal_mode file\n",
			__func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_mode_filp);
		pr_err("%s,err = %d\n",
			__func__, err);
		return err;
	}

	err = cal_mode_filp->f_op->write(cal_mode_filp,
		(char *)&mode, sizeof(u8), &cal_mode_filp->f_pos);
	if (err != sizeof(u8)) {
		pr_err("%s,Can't read the cal_mode from file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_mode_filp, current->files);
	set_fs(old_fs);

	return err;
}

static ssize_t prox_cal_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", gp2a->cal_mode);
}

static ssize_t prox_cal_write(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);
	int err;

	if (sysfs_streq(buf, "1")) {
		gp2a->cal_mode = 1;
		gp2a->nondetect = PROX_NONDETECT_MODE1;
		gp2a->detect = PROX_DETECT_MODE1;
	} else if (sysfs_streq(buf, "2")) {
		gp2a->cal_mode = 2;
		gp2a->nondetect = PROX_NONDETECT_MODE2;
		gp2a->detect = PROX_DETECT_MODE2;
	} else if (sysfs_streq(buf, "0")) {
		gp2a->cal_mode = 0;
		gp2a->nondetect = PROX_NONDETECT;
		gp2a->detect = PROX_DETECT;
	} else {
		pr_err("%s,invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	if (gp2a->power_state == 1) {
		gp2a_power_onoff(gp2a, 0);
		msleep(20);
		gp2a_power_onoff(gp2a, 1);
	}

	err = gp2a_cal_mode_save_file(gp2a->cal_mode);
	if (err < 0) {
		pr_err("%s,prox_cal_write() failed\n", __func__);
		return err;
	}

	return size;
}

static DEVICE_ATTR(adc, 0440, adc_read, NULL);
static DEVICE_ATTR(state, 0440, state_read, NULL);
static DEVICE_ATTR(name, 0440, name_read, NULL);
static DEVICE_ATTR(vendor, 0440, vendor_read, NULL);
static DEVICE_ATTR(prox_cal, 0664, prox_cal_read, prox_cal_write);

static struct device_attribute *proxi_attrs[] = {
	&dev_attr_adc,
	&dev_attr_state,
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_prox_cal,
	NULL,
};
static int gp2a_regulator_onoff(struct device *dev, bool onoff)
{
	/*struct regulator *gp2a_vio;*/
	struct regulator *gp2a_vdd;
	int ret;

	pr_info("%s %s\n", __func__, (onoff) ? "on" : "off");

	gp2a_vdd = devm_regulator_get(dev, "gp2a-vdd");
	if (IS_ERR(gp2a_vdd)) {
		pr_err("[SENSOR]: %s - cannot get gp2a_vdd\n", __func__);
		return -ENOMEM;
	}

	regulator_set_voltage(gp2a_vdd, 2850000, 2850000);
/*
	gp2a_vio = devm_regulator_get(dev, "gp2a-vio");
	if (IS_ERR(gp2a_vio)) {
		pr_err("%s: cannot get gp2a_vio\n", __func__);
		return -ENOMEM;
	}
*/
	if (onoff) {
		ret = regulator_enable(gp2a_vdd);
		if (ret) {
			pr_err("%s: enable gp2a_vdd failed, rc=%d\n",
				__func__, ret);
			return ret;
		}
/*
		ret = regulator_enable(gp2a_vio);
		if (ret) {
			pr_err("%s: enable gp2a_vio failed, rc=%d\n",
				__func__, ret);
			return ret;
		}
*/
	} else {
		ret = regulator_disable(gp2a_vdd);
		if (ret) {
			pr_err("%s: disable gp2a_vdd failed, rc=%d\n",
				__func__, ret);
			return ret;
		}
/*
		ret = regulator_disable(gp2a_vio);
		if (ret) {
			pr_err("%s: disable gp2a_vio failed, rc=%d\n",
				__func__, ret);
			return ret;
		}
*/
	}

	devm_regulator_put(gp2a_vdd);

	/*devm_regulator_put(gp2a_vio);*/

	msleep(20);

	return 0;
}

static ssize_t proximity_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#if defined(CONFIG_SENSORS_IIO)
	struct gp2a_data *gp2a = iio_priv(dev_get_drvdata(dev));
#else
	struct gp2a_data *gp2a = dev_get_drvdata(dev);
#endif

	return snprintf(buf, PAGE_SIZE, "%d\n", gp2a->power_state);
}

static ssize_t proximity_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
#if defined(CONFIG_SENSORS_IIO)
	struct gp2a_data *gp2a = iio_priv(dev_get_drvdata(dev));
#else
	struct gp2a_data *gp2a = dev_get_drvdata(dev);
#endif
	int value = 0;
	int err = 0;

	err = kstrtoint(buf, 10, &value);
	if (err) {
		pr_err("%s,kstrtoint failed.", __func__);
		goto done;
	}
	if (value != 0 && value != 1) {
		pr_err("%s,wrong value(%d)\n", __func__, value);
		goto done;
	}

	mutex_lock(&gp2a->power_lock);

	if (gp2a->power_state != value) {
		pr_info("%s,enable(%d)\n", __func__, value);
		if (value) {
			err = gp2a_cal_mode_read_file(gp2a);
			if (err < 0 && err != -ENOENT)
				pr_err("%s,cal_mode file read fail\n",
					__func__);

			pr_info("%s,mode(%d)\n", __func__, gp2a->cal_mode);
			if (gp2a->cal_mode == 2) {
				gp2a->nondetect = PROX_NONDETECT_MODE2;
				gp2a->detect = PROX_DETECT_MODE2;
			} else if (gp2a->cal_mode == 1) {
				gp2a->nondetect = PROX_NONDETECT_MODE1;
				gp2a->detect = PROX_DETECT_MODE1;
			} else {
				gp2a->nondetect = PROX_NONDETECT;
				gp2a->detect = PROX_DETECT;
			}
			gp2a_regulator_onoff(&gp2a->i2c_client->dev, true);
			gp2a_power_onoff(gp2a, 1);
			gp2a->power_state = value;

			gp2a->val_state = value;
#if !defined(CONFIG_SENSORS_IIO)
			input_report_abs(gp2a->input, ABS_DISTANCE,
				gp2a->val_state);
			input_sync(gp2a->input);
#endif
		} else {
			gp2a_power_onoff(gp2a, 0);
			gp2a_regulator_onoff(&gp2a->i2c_client->dev, false);
			gp2a->power_state = value;
		}

	} else {
		pr_err("%s,wrong cmd for enable\n", __func__);
	}

	mutex_unlock(&gp2a->power_lock);
done:
	return size;
}

#if defined(CONFIG_SENSORS_IIO)
static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	proximity_enable_show, proximity_enable_store, 0);

static struct attribute *proximity_sysfs_attrs[] = {
	&iio_dev_attr_enable.dev_attr.attr,
	NULL
};
#else

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	proximity_enable_show, proximity_enable_store);

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_enable.attr,
	NULL
};
#endif

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

#if defined(CONFIG_SENSORS_IIO)
static int gp2a_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan,
	int *val, int *val2, long mask)
{
	struct gp2a_data *data = iio_priv(indio_dev);
	int ret = -EINVAL;

	switch (chan->type) {
	case IIO_PROXIMITY:
		*val = data->val_state;
		break;
	default:
		pr_err("%s, invalied channel\n", __func__);
		return ret;
	}

	return IIO_VAL_INT;
}

static const struct iio_info gp2a_info = {
	.attrs = &proximity_attribute_group,
	.driver_module = THIS_MODULE,
	.read_raw = gp2a_read_raw,
};
#endif

static void gp2a_prox_work_func(struct work_struct *work)
{
	struct gp2a_data *gp2a = container_of(work,
		struct gp2a_data, work_prox);
#if defined(CONFIG_SENSORS_IIO)
	struct iio_dev *indio_dev = iio_priv_to_dev(gp2a);
	struct timespec ts;
	u8 buf[IIO_BUFFER_1_BYTES];
#endif
	u8 vo, value;

	gp2a_i2c_read(gp2a, REGS_PROX, &vo);
	vo = 0x01 & vo;

	value = 0x18;
	gp2a_i2c_write(gp2a, REGS_CON, value);

	if (!vo) {
		gp2a->val_state = 0x01;
		value = gp2a->nondetect;
	} else {
		gp2a->val_state = 0x00;
		value = gp2a->detect;
	}
	gp2a_i2c_write(gp2a, REGS_HYS, value);

	pr_info("%s,%d\n", __func__, gp2a->val_state);

#if defined(CONFIG_SENSORS_IIO)
	ts = ktime_to_timespec(ktime_get_boottime());
	gp2a->timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;

	memcpy(buf, &gp2a->val_state, sizeof(gp2a->val_state));
	memcpy(buf + 1, &gp2a->timestamp, sizeof(gp2a->timestamp));

	iio_push_to_buffers(indio_dev, buf);
#else
	input_report_abs(gp2a->input, ABS_DISTANCE, gp2a->val_state);
	input_sync(gp2a->input);
#endif
	msleep(20);

	value = 0x00;
	gp2a_i2c_write(gp2a, REGS_CON, value);
}

irqreturn_t gp2a_irq_handler(int irq, void *data)
{
	struct gp2a_data *gp2a = data;
	pr_info("%s,%d\n", __func__, irq);

	schedule_work((struct work_struct *)&gp2a->work_prox);
	wake_lock_timeout(&gp2a->prx_wake_lock, 3*HZ);

	return IRQ_HANDLED;
}

static int gp2a_setup_irq(struct gp2a_data *gp2a)
{
	int rc;
	struct gp2a_platform_data *pdata = gp2a->pdata;
	int irq = -1;
	u8 value;

	rc = gpio_request(pdata->p_out, "gpio_proximity_out");
	if (rc < 0) {
		pr_err("%s,gpio %d request failed (%d)\n",
			__func__, pdata->p_out, rc);
		return rc;
	}

	rc = gpio_direction_input(pdata->p_out);
	if (rc < 0) {
		pr_err("%s,failed gpio %d as input (%d)\n",
			__func__, pdata->p_out, rc);
		goto err_gpio_direction_input;
	}

	value = 0x18;
	gp2a_i2c_write(gp2a, REGS_CON, value);
	irq = gpio_to_irq(pdata->p_out);
	rc = request_irq(irq, gp2a_irq_handler, IRQF_TRIGGER_FALLING |
		IRQF_ONESHOT | IRQF_NO_SUSPEND, "proximity_int", gp2a);

	if (rc < 0) {
		pr_err("%s,request_irq(%d) failed for gpio %d (%d)\n",
			__func__, irq,
			pdata->p_out, rc);
		goto err_request_irq;
	} else{
		pr_info("%s,request_irq(%d) success for gpio %d\n",
			__func__, irq, pdata->p_out);
	}
	disable_irq(irq);
	gp2a->irq = irq;

	value = 0x02;
	gp2a_i2c_write(gp2a, REGS_OPMOD, value);
	goto done;

err_request_irq:
err_gpio_direction_input:
	gpio_free(pdata->p_out);
done:
	return rc;
}

#if defined(CONFIG_SENSORS_LEDA_EN_GPIO)
static int gp2a_request_gpio(struct gp2a_platform_data *pdata)
{
	int ret;
	ret = gpio_request(pdata->power_en, "prox_en");
	if (ret) {
		pr_err("%s: gpio request fail\n", __func__);
		return ret;
	}

	ret = gpio_direction_output(pdata->power_en, 0);
	if (ret) {
		pr_err("%s: unable to set_direction [%d]\n", __func__,
			pdata->power_en);
		return ret;
	}
	return 0;
}
#endif


static int gp2a_parse_dt(struct device *dev, struct gp2a_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;

	pdata->p_out = of_get_named_gpio_flags(np, "gp2a-i2c,irq-gpio",
				0, &flags);
	if (pdata->p_out < 0) {
		pr_err("%s : get irq_gpio(%d) error\n", __func__, pdata->p_out);
		return -ENODEV;
	}
#if defined(CONFIG_SENSORS_LEDA_EN_GPIO)
	pdata->power_en = of_get_named_gpio_flags(np, "gp2a-i2c,en-gpio",
				0, &flags);
	if (pdata->power_en < 0) {
		pr_err("%s : get power_en(%d) error\n", __func__,
			pdata->power_en);
		return -ENODEV;
	}
#endif
	return 0;
}


static int gp2a_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret = 0;
	struct gp2a_data *gp2a;
	struct gp2a_platform_data *pdata = client->dev.platform_data;
#if defined(CONFIG_SENSORS_IIO)
	struct iio_dev *indio_dev;
#endif

	pr_info("%s, start\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s, i2c functionality failed\n", __func__);
		return -ENOMEM;
	}

	ret = gp2a_regulator_onoff(&client->dev, true);
	if (ret) {
		pr_err("%s, Power Up Failed\n", __func__);
		goto err_regulator_onoff;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct gp2a_platform_data), GFP_KERNEL);
		if (!pdata) {
			pr_err("%s, Failed to allocate memory\n", __func__);
			goto err_alloc_pdata;
		}
		ret = gp2a_parse_dt(&client->dev, pdata);
		if (ret < 0)
			goto err_parse_dt;
		ret = gp2a_request_gpio(pdata);
		if (ret < 0)
			goto err_request_gpio;
	}

#if defined(CONFIG_SENSORS_IIO)
	/* Configure IIO device */
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*gp2a));
	if (!indio_dev) {
		pr_err("%s, iio_device_alloc failed\n", __func__);
		ret = -ENOMEM;
		goto err_mem_alloc;
	}

	gp2a = iio_priv(indio_dev);
	dev_set_name(&indio_dev->dev, "proximity_sensor");
	i2c_set_clientdata(client, indio_dev);

	/* Init IIO device */
	indio_dev->name = client->name;
	indio_dev->channels = gp2a_channels;
	indio_dev->num_channels = ARRAY_SIZE(gp2a_channels);
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &gp2a_info;

	ret = sensors_iio_configure_buffer(indio_dev);
	if (ret) {
		pr_err("%s, configure ring buffer failed\n",
			__func__);
		goto err_iio_configure_buffer_failed;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		pr_err("%s: iio_device_register failed(%d)\n", __func__, ret);
		goto err_iio_register_failed;
	}

	gp2a->pdata = pdata;
	gp2a->i2c_client = client;

#else	/* !defined(CONFIG_SENSORS_IIO)*/

	/* Allocate memory for driver data */
	gp2a = kzalloc(sizeof(struct gp2a_data), GFP_KERNEL);
	if (!gp2a) {
		pr_err("%s,failed memory alloc\n", __func__);
		ret = -ENOMEM;
		goto err_mem_alloc;
	}
	i2c_set_clientdata(client, gp2a);

	gp2a->pdata = pdata;
	gp2a->i2c_client = client;

	gp2a->input = input_allocate_device();
	if (!gp2a->input) {
		pr_err("%s,could not allocate input device\n", __func__);
		goto err_input_allocate_device_proximity;
	}

	input_set_drvdata(gp2a->input, gp2a);
	gp2a->input->name = "proximity_sensor";
	input_set_capability(gp2a->input, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(gp2a->input, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(gp2a->input);
	if (ret < 0) {
		pr_err("%s,could not register input device\n",
			__func__);
		goto err_input_register_device_proximity;
	}
	ret = sensors_create_symlink(&gp2a->input->dev.kobj,
		gp2a->input->name);
	if (ret < 0) {
		pr_err("%s,create sysfs symlink error\n", __func__);
		goto err_sysfs_create_symlink_proximity;
	}

	ret = sysfs_create_group(&gp2a->input->dev.kobj,
		&proximity_attribute_group);
	if (ret) {
		pr_err("%s,create sysfs group error\n", __func__);
		goto err_sysfs_create_group_proximity;
	}
#endif

	wake_lock_init(&gp2a->prx_wake_lock, WAKE_LOCK_SUSPEND,
		"prx_wake_lock");
	mutex_init(&gp2a->power_lock);

	INIT_WORK(&gp2a->work_prox, gp2a_prox_work_func);

	gp2a_leda_onoff(gp2a, 1);

	ret = gp2a_setup_irq(gp2a);
	if (ret) {
		pr_err("%s,could not setup irq\n", __func__);
		goto err_setup_irq;
	}

	gp2a_leda_onoff(gp2a, 0);

	ret = sensors_register(gp2a->dev, gp2a, proxi_attrs,
		"proximity_sensor");
	if (ret < 0) {
		pr_info("%s,could not sensors_register\n", __func__);
		goto err_sensors_register;
	}

	pr_info("%s done\n", __func__);
	return 0;

err_sensors_register:
	free_irq(gp2a->irq, gp2a);
	gpio_free(gp2a->pdata->p_out);
err_setup_irq:
	gp2a_leda_onoff(gp2a, 0);
	mutex_destroy(&gp2a->power_lock);
	wake_lock_destroy(&gp2a->prx_wake_lock);
#if defined(CONFIG_SENSORS_IIO)
	iio_device_unregister(indio_dev);
err_iio_register_failed:
	sensors_iio_unconfigure_buffer(indio_dev);
err_iio_configure_buffer_failed:
#else
	sysfs_remove_group(&gp2a->input->dev.kobj,
		&proximity_attribute_group);
err_sysfs_create_group_proximity:
	sensors_remove_symlink(&gp2a->input->dev.kobj,
		gp2a->input->name);
err_sysfs_create_symlink_proximity:
	input_unregister_device(gp2a->input);
err_input_register_device_proximity:
	input_free_device(gp2a->input);
err_input_allocate_device_proximity:
	kfree(gp2a);
#endif
err_mem_alloc:
err_request_gpio:
err_parse_dt:
err_alloc_pdata:
	gp2a_regulator_onoff(&client->dev, false);
err_regulator_onoff:
	pr_err("%s is FAILED (%d)\n", __func__, ret);
	return ret;
}

static const struct i2c_device_id gp2a_device_id[] = {
	{"gp2a", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, gp2a_device_id);

static struct of_device_id gp2a_i2c_match_table[] = {
	{ .compatible = "gp2a-i2c",},
	{},
};

MODULE_DEVICE_TABLE(of, gp2a_i2c_match_table);

static struct i2c_driver gp2a_i2c_driver = {
	.driver = {
		.name = "gp2a",
		.owner = THIS_MODULE,
		.of_match_table = gp2a_i2c_match_table,

	},
	.probe		= gp2a_i2c_probe,
	.id_table	= gp2a_device_id,
};

module_i2c_driver(gp2a_i2c_driver);

MODULE_AUTHOR("mjchen@sta.samsung.com");
MODULE_DESCRIPTION("Optical Sensor driver for gp2ap002");
MODULE_LICENSE("GPL");

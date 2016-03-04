/*
 * Copyright (c) 2014 Samsung Electronics Co, Ltd.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#define DEBUG

#include <linux/battery/sec_charging_common.h>
#include <linux/delay.h>
#include <linux/edge_wakeup_mmp.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio-pxa.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/pm_wakeup.h>
#include <linux/platform_device.h>
#include <linux/sec-common.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/pm_qos.h>
#include <linux/switch.h>

#include <linux/extcon/s2mm001.h>
#include <linux/extcon/extcon-samsung.h>

enum s2mm001_muic_reg {
	S2MM001_MUIC_REG_DEVID		= 0x01,
	S2MM001_MUIC_REG_CTRL1		= 0x02,
	S2MM001_MUIC_REG_INT1		= 0x03,
	S2MM001_MUIC_REG_INT2		= 0x04,
	S2MM001_MUIC_REG_INTMASK1	= 0x05,
	S2MM001_MUIC_REG_INTMASK2	= 0x06,
	S2MM001_MUIC_REG_ADC		= 0x07,
	S2MM001_MUIC_REG_TIMING1	= 0x08,
	S2MM001_MUIC_REG_TIMING2	= 0x09,
	S2MM001_MUIC_REG_DEV_T1		= 0x0a,
	S2MM001_MUIC_REG_DEV_T2		= 0x0b,
	S2MM001_MUIC_REG_BUTTON1	= 0X0c,
	S2MM001_MUIC_REG_BUTTON2	= 0X0d,
	S2MM001_MUIC_REG_MANSW1		= 0x13,
	S2MM001_MUIC_REG_MANSW2		= 0x14,
	S2MM001_MUIC_REG_DEV_T3		= 0x15,
	S2MM001_MUIC_REG_RESET		= 0x1B,
	S2MM001_MUIC_REG_TIMING3	= 0x20,
	S2MM001_MUIC_REG_OCP_SET	= 0x22,
	S2MM001_MUIC_REG_CTRL2		= 0x23,
	S2MM001_MUIC_REG_ADC_MODE	= 0x25,
	S2MM001_MUIC_REG_END,
};

/* S2MM001 Control register */
#define CTRL1_SWITCH_OPEN		(1 << 4)
#define CTRL1_RAW_DATA			(1 << 3)
#define CTRL1_MANUAL_SW			(1 << 2)
#define CTRL1_WAIT			(1 << 1)
#define CTRL1_INT_MASK			(1 << 0)
#define CTRL1_INIT			(CTRL1_SWITCH_OPEN | CTRL1_RAW_DATA |\
						CTRL1_MANUAL_SW | CTRL1_WAIT |\
						!CTRL1_INT_MASK)
#define CTRL1_DEF_INT			0x1F

/* S2MM001 Interrupt 1 register */
#define INT1_OVP_OCP			(1 << 7)
#define INT1_VBUS_OCP			(1 << 6)
#define INT1_VBUS_OVP			(1 << 5)
#define INT1_LKR			(1 << 4) /* Long Key Released */
#define INT1_LKP			(1 << 3) /* Long Key Pressed */
#define INT1_KP				(1 << 2) /* Key Pressed */
#define INT1_DETACH			(1 << 1)
#define INT1_ATTACH			(1 << 0)
#define INT1_MASK_INIT			(INT1_OVP_OCP | INT1_VBUS_OCP | \
					INT1_VBUS_OVP | INT1_LKR | INT1_LKP |\
					INT1_KP | !INT1_DETACH | !INT1_ATTACH)

/* S2MM001 Interrupt 2 register */
#define INT2_AV_CHARGE			(1 << 6)
#define INT2_MHDL			(1 << 5)
#define INT2_STUCK_RCV			(1 << 4)
#define INT2_STUCK			(1 << 3)
#define INT2_ADC_CHANGE			(1 << 2)
#define INT2_RSV_ATTACH			(1 << 1) /* Reseved Accessory */
#define INT2_VBUS_DET			(1 << 0)
#define INT2_MASK_INIT			(INT2_AV_CHARGE | INT2_MHDL | \
					INT2_STUCK_RCV | INT2_STUCK | \
					INT2_ADC_CHANGE | INT2_RSV_ATTACH | \
					!INT2_VBUS_DET)

/* S2MM001 Device Type 1 register */
#define DEV_TYPE1_USB_OTG		(1 << 7)
#define DEV_TYPE1_DEDICATED_CHG		(1 << 6)
#define DEV_TYPE1_CDP			(1 << 5)
#define DEV_TYPE1_CARKIT		(1 << 4)
#define DEV_TYPE1_UART			(1 << 3)
#define DEV_TYPE1_USB			(1 << 2)
#define DEV_TYPE1_AUDIO_2		(1 << 1)
#define DEV_TYPE1_AUDIO_1		(1 << 0)
#define DEV_TYPE1_USB_TYPES		(DEV_TYPE1_USB_OTG | \
						DEV_TYPE1_USB)
#define DEV_TYPE1_CHG_TYPES		(DEV_TYPE1_DEDICATED_CHG | \
						DEV_TYPE1_CDP)

/* S2MM001 Device Type 2 register */
#define DEV_TYPE2_AV			(1 << 6)
#define DEV_TYPE2_TTY			(1 << 5)
#define DEV_TYPE2_PPD			(1 << 4)
#define DEV_TYPE2_JIG_UART_OFF		(1 << 3)
#define DEV_TYPE2_JIG_UART_ON		(1 << 2)
#define DEV_TYPE2_JIG_USB_OFF		(1 << 1)
#define DEV_TYPE2_JIG_USB_ON		(1 << 0)
#define DEV_TYPE2_JIG_USB_TYPES		(DEV_TYPE2_JIG_USB_OFF | \
						DEV_TYPE2_JIG_USB_ON)
#define DEV_TYPE2_JIG_UART_TYPES	(DEV_TYPE2_JIG_UART_OFF | \
						DEV_TYPE2_JIG_UART_ON)
#define DEV_TYPE2_JIG_TYPES		(DEV_TYPE2_JIG_UART_TYPES | \
						DEV_TYPE2_JIG_USB_TYPES)

/* S2MM001 Device Type 3 register */
#define DEV_TYPE3_AV75			(1 << 7) /* A/V Cable 75 ohm */
#define DEV_TYPE3_U200_CHG		(1 << 6)
#define DEV_TYPE3_APPLE_CHG		(1 << 5)
#define DEV_TYPE3_AV_WITH_VBUS		(1 << 4)
#define DEV_TYPE3_NO_STD_CHG		(1 << 2)
#define DEV_TYPE3_VBUS_VALID		(1 << 1)
#define DEV_TYPE3_MHL			(1 << 0)
#define DEV_TYPE3_CHG_TYPE		(DEV_TYPE3_U200_CHG | \
						DEV_TYPE3_NO_STD_CHG | \
						DEV_TYPE3_APPLE_CHG)
/*
 * Manual Switch
 * D- [7:5] / D+ [4:2] / CHARGER[1] / OTGEN[0]
 * 000: Open all / 001: USB / 010: AUDIO / 011: UART / 100: V_AUDIO
 * 00: Vbus to Open / 01: Vbus to Charger / 10: Vbus to MIC / 11: Vbus to VBout
 */
#define MANUAL_SW1_DM_SHIFT		5
#define MANUAL_SW1_DP_SHIFT		2
#define MANUAL_SW1_VBUS_SHIFT		0
#define MANUAL_SW1_D_OPEN		(0x0)
#define MANUAL_SW1_D_USB		(0x1)
#define MANUAL_SW1_D_AUDIO		(0x2)
#define MANUAL_SW1_D_UART		(0x3)
#define MANUAL_SW1_V_OPEN		(0x0)
#define MANUAL_SW1_V_CHARGER		(0x2)
#define MANUAL_SW1_V_OTGEN		(0x1)

/* MANUAL SWITCH 1 */
#define CON_TO_USB	(MANUAL_SW1_D_USB << MANUAL_SW1_DM_SHIFT) | \
			(MANUAL_SW1_D_USB << MANUAL_SW1_DP_SHIFT) | \
			(MANUAL_SW1_V_CHARGER << MANUAL_SW1_VBUS_SHIFT)
#define CON_TO_UART	(MANUAL_SW1_D_UART << MANUAL_SW1_DM_SHIFT) | \
			(MANUAL_SW1_D_UART << MANUAL_SW1_DP_SHIFT) | \
			(MANUAL_SW1_V_CHARGER << MANUAL_SW1_VBUS_SHIFT)
#define CON_OPEN	(MANUAL_SW1_D_OPEN << MANUAL_SW1_DM_SHIFT) | \
			(MANUAL_SW1_D_OPEN << MANUAL_SW1_DP_SHIFT) | \
			(MANUAL_SW1_V_OPEN << MANUAL_SW1_VBUS_SHIFT)

#define RESET_BIT			(1 << 0)

#define I2C_RW_RETRY_MAX    3
#define I2C_RW_RETRY_DELAY  15

struct s2mm001_usbsw {
	struct i2c_client *client;
	struct s2mm001_platform_data *pdata;
	struct work_struct work;
	struct pm_qos_request qos_idle;
	struct switch_dev dock_dev;
	struct mutex mutex;
	u8 id;
	u8 dev1;
	u8 dev2;
	u8 dev3;
	u8 adc;
	u8 chg_type;
	u8 intr1;
	u8 intr2;
};

struct ic_vendor {
	u8 id;
	char *part_num;
};

static struct s2mm001_usbsw *chip;
static struct wakeup_source jig_suspend_wake;
static int jig_wakelock_acq;
static int probing = 1;
static int attached_dev;
//static int en_uart = 1;

static int read_reg(struct i2c_client *client, u8 reg, u8 * data)
{
	int ret = 0;
	u8 buf[1];
	struct i2c_msg msg[2];

	buf[0] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "I2C Read Failed REG[0x%2x] (ret=%d)\n",
			reg, ret);
		return -EIO;
	}
	*data = buf[0];

	dev_info(&client->dev, "I2C Read REG[0x%2x] DATA[0x%2x]\n", reg,
		 buf[0]);
	return 0;
}

static int write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int ret = 0;
	u8 buf[2];
	struct i2c_msg msg[1];

	buf[0] = reg;
	buf[1] = data;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	dev_info(&client->dev, "I2C Write REG[0x%2x] DATA[0x%2x]\n", buf[0],
		 buf[1]);

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "I2C Write Failed (ret=%d) \n", ret);
		return -EIO;
	}

	return ret;
}

#if 0	/* FIX ME */
static int retry_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int result = -1, i;

	for (i = 0; i < I2C_RW_RETRY_MAX && (result < 0); i++) {
		result = write_reg(client, reg, data);
		if (result < 0)
			msleep(I2C_RW_RETRY_DELAY);
	}
	return result;
}
#endif

static int retry_read_reg(struct i2c_client *client, u8 reg, u8 * data)
{
	int result = -1, i;

	for (i = 0; i < I2C_RW_RETRY_MAX && (result < 0); i++) {
		result = read_reg(client, reg , data);
		if (result < 0)
			msleep(I2C_RW_RETRY_DELAY);
	}
	return result;
}

static ssize_t adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct s2mm001_usbsw *usbsw = chip;
	u8 adc_value[] = "1C";
	u8 adc_fail = 0;

	if (usbsw->dev2 & DEV_TYPE2_JIG_TYPES) {
		pr_info("adc_show JIG_UART_OFF\n");
		return sprintf(buf, "%s\n", adc_value);
	} else {
		pr_info("adc_show no detect\n");
		return sprintf(buf, "%d\n", adc_fail);
	}
}

static ssize_t s2mm001_set_syssleep(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct s2mm001_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	dev_info(&client->dev, "%s+\n", __func__);
	if (!strncmp(buf, "1", 1)) {
		dev_info(&client->dev, "%s:release wake lock\n", __func__);
		pm_qos_update_request(&usbsw->qos_idle,
				      PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
		__pm_relax(&jig_suspend_wake);
	} else if (!strncmp(buf, "0", 1)) {
		dev_info(&client->dev, "%s:get wake lock\n", __func__);
		__pm_stay_awake(&jig_suspend_wake);
		pm_qos_update_request(&usbsw->qos_idle, PM_QOS_DEFAULT_VALUE);
	}
	dev_info(&client->dev, "%s-\n", __func__);

	return count;
}

static ssize_t usb_state_show_attrs(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t usb_sel_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "PDA");
}

static ssize_t uart_sel_show_attrs(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t uart_sel_store_attrs(struct device *dev,
				    struct device_attribute *attr, const char *buf,
				    size_t count)
{
	return count;
}

static DEVICE_ATTR(adc, (S_IRUGO | S_IXOTH), adc_show, NULL);
static DEVICE_ATTR(syssleep, (S_IWUSR | S_IWGRP), NULL, s2mm001_set_syssleep);
static DEVICE_ATTR(usb_state, (S_IRUGO), usb_state_show_attrs, NULL);
static DEVICE_ATTR(usb_sel, (S_IRUGO), usb_sel_show_attrs, NULL);
static DEVICE_ATTR(uart_sel, (S_IRUGO | S_IWUSR | S_IWGRP),
				uart_sel_show_attrs, uart_sel_store_attrs);
static DEVICE_ATTR(attached_dev, (S_IRUGO), attached_dev_attrs, NULL);

static void manual_path_to_usb(int on)
{
	struct s2mm001_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	u8 ctrl_reg, mansw1;

	read_reg(client, S2MM001_MUIC_REG_MANSW1, &mansw1);
	read_reg(client, S2MM001_MUIC_REG_CTRL1, &ctrl_reg);

	mansw1 = on ? CON_TO_USB : CON_OPEN;
	on ? (ctrl_reg &= ~CTRL1_MANUAL_SW) : (ctrl_reg |= CTRL1_MANUAL_SW);

	write_reg(client, S2MM001_MUIC_REG_MANSW1, mansw1);
	write_reg(client, S2MM001_MUIC_REG_CTRL1, ctrl_reg);
}

#if 0	/* FIX ME */
static int s2mm001_ic_reset(void)
{
	struct s2mm001_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	u8 sintm1, sintm2, smansw1, sctrl;

	mutex_lock(&usbsw->mutex);
	disable_irq(client->irq);

	read_reg(client, S2MM001_MUIC_REG_INTMASK1, &sintm1);
	read_reg(client, S2MM001_MUIC_REG_INTMASK2, &sintm2);
	read_reg(client, S2MM001_MUIC_REG_MANSW1, &smansw1);
	read_reg(client, S2MM001_MUIC_REG_CTRL1, &sctrl);

	write_reg(client, S2MM001_MUIC_REG_RESET, 1);
	msleep(20);

	write_reg(client, S2MM001_MUIC_REG_INTMASK1, sintm1);
	write_reg(client, S2MM001_MUIC_REG_INTMASK2, sintm2);
	write_reg(client, S2MM001_MUIC_REG_MANSW1, smansw1);
	write_reg(client, S2MM001_MUIC_REG_CTRL1, sctrl);

	dev_info(&client->dev, "s2mm001 was reset!\n");

	enable_irq(client->irq);
	mutex_unlock(&usbsw->mutex);

	return 0;
}
#endif

static irqreturn_t microusb_irq_handler(int irq, void *data)
{
	struct s2mm001_usbsw *usbsw = data;

	dev_info(&usbsw->client->dev, "%s\n", __func__);
	schedule_work(&usbsw->work);

	return IRQ_HANDLED;
}

static int s2mm001_reg_init(struct s2mm001_usbsw *usbsw)
{
	int i, ret;
	struct i2c_client *client = usbsw->client;
	struct ic_vendor muic_list[] = {
		{0x00, "s2mm001"},
	};

	ret = read_reg(client, S2MM001_MUIC_REG_DEVID, &usbsw->id);
	if (ret < 0)
		return ret;
	for (i = 0; i < ARRAY_SIZE(muic_list); i++) {
		if (usbsw->id == muic_list[i].id)
			dev_info(&client->dev, "PartNum : %s\n",
				 muic_list[i].part_num);
	}

	/* INT MASK1, 2 */
	ret = write_reg(client, S2MM001_MUIC_REG_INTMASK1, INT1_MASK_INIT);
	if (ret < 0)
		return ret;
	ret = write_reg(client, S2MM001_MUIC_REG_INTMASK2, INT2_MASK_INIT);
	if (ret < 0)
		return ret;
	/* CONTROL */
	ret = write_reg(client, S2MM001_MUIC_REG_CTRL1, CTRL1_INIT);
	if (ret < 0)
		return ret;

	return 0;
}

static void detect_dev_s2mm001(struct s2mm001_usbsw *usbsw, u8 intr1, u8 intr2,
			      void *data)
{
	struct s2mm001_platform_data *pdata = usbsw->pdata;
	struct i2c_client *client = usbsw->client;
	u8 val1, val2, val3, adc;
	int dev_classifi = CABLE_UNKNOWN;

	read_reg(client, S2MM001_MUIC_REG_DEV_T1, &val1);
	read_reg(client, S2MM001_MUIC_REG_DEV_T2, &val2);
	read_reg(client, S2MM001_MUIC_REG_DEV_T3, &val3);
	read_reg(client, S2MM001_MUIC_REG_ADC, &adc);

	if (probing == 1)
		read_reg(client, S2MM001_MUIC_REG_INT2, &intr2);

	if (intr1 & INT1_ATTACH) {
		/* Attached */
		if (val1 & DEV_TYPE1_USB_OTG) {
			dev_classifi = S2MM001_CABLE_TYPE1_OTG;
			dev_info(&client->dev, "OTG ATTACHED*****\n");
		}
		if (val1 & DEV_TYPE1_DEDICATED_CHG) {
			dev_classifi = S2MM001_CABLE_TYPE1_TA;
			dev_info(&client->dev, "TA ATTACHED*****\n");
		}
		if (val1 & DEV_TYPE1_CDP) {
			dev_classifi = S2MM001_CABLE_TYPE1_TA;
			dev_info(&client->dev, "TA ATTACHED*****\n");
		}
		if (val1 & DEV_TYPE1_CARKIT) {
			dev_classifi = S2MM001_CABLE_TYPE1_CARKIT;
			manual_path_to_usb(1);
			dev_info(&client->dev, "CARKIT ATTACHED*****\n");
		}
		if (val1 & DEV_TYPE1_UART) {
			dev_classifi = S2MM001_CABLE_TYPE1_UART;
			dev_info(&client->dev, "UART ATTACHED*****\n");
		}
		if (val1 & DEV_TYPE1_USB) {
			dev_classifi = S2MM001_CABLE_TYPE1_USB;
			dev_info(&client->dev, "USB Cable ATTACHED*****\n");
		}
		if (val2 & DEV_TYPE2_JIG_UART_OFF) {
			if (val3 & DEV_TYPE3_VBUS_VALID) {
				dev_classifi = S2MM001_CABLE_TYPE2_JIG_UART_OFF_VB;
				dev_info(&client->dev,
						 "JIG_UARTOFF + VBUS ATTACHED*****\n");
			} else {	
				dev_classifi = S2MM001_CABLE_TYPE2_JIG_UART_OFF;
				dev_info(&client->dev,
						 "JIG_UARTOFF ATTACHED*****\n");
			}
		}
		if (val2 & DEV_TYPE2_JIG_UART_ON) {
			dev_classifi = S2MM001_CABLE_TYPE2_JIG_UART_ON;
			dev_info(&client->dev,
					 "JIG_UARTON ATTACHED*****\n");
		}
		if (val2 & DEV_TYPE2_JIG_USB_OFF) {
			dev_classifi = S2MM001_CABLE_TYPE2_JIG_USB_OFF;
			dev_info(&client->dev, "JIG_USB_OFF ATTACHED*****\n");
		}
		if (val2 & DEV_TYPE2_JIG_USB_ON) {
			dev_classifi = S2MM001_CABLE_TYPE2_JIG_USB_ON;
			dev_info(&client->dev, "JIG_USB_ON ATTACHED*****\n");
		}
		if (val2 & DEV_TYPE2_PPD) {
			if (!jig_wakelock_acq) {
				__pm_stay_awake(&jig_suspend_wake);
				pm_qos_update_request(&usbsw->qos_idle,
						      pdata->qos_val);
				jig_wakelock_acq = 1;
				dev_info(&client->dev,
					 "AP WakeLock for FactoryTest *****\n");
			}
		}
		if (val3 & DEV_TYPE3_CHG_TYPE) {
			dev_classifi = S2MM001_CABLE_TYPE1_TA;
			dev_info(&client->dev, "TA ATTACHED*****\n");
		}

		if (probing == 1)
			*(int *)data = dev_classifi;
#ifdef CONFIG_EXTCON_SAMSUNG
		extcon_samsung_call_chain(dev_classifi);
#endif
		usbsw->dev1 = val1;
		usbsw->dev2 = val2;
		usbsw->dev3 = val3;
		usbsw->adc = adc;
		usbsw->intr1 = intr1;
		usbsw->intr2 = intr2;
		attached_dev = dev_classifi;
	}

	if (intr1 & INT1_DETACH) {
		/* Detached */
		if (usbsw->dev1 & DEV_TYPE1_USB_OTG) {
			dev_info(&client->dev, "OTG DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_TYPE1_DEDICATED_CHG) {
			dev_info(&client->dev, "TA DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_TYPE1_CDP) {
			dev_info(&client->dev, "TA DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_TYPE1_CARKIT) {
			manual_path_to_usb(0);
			dev_info(&client->dev, "CARKIT DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_TYPE1_UART) {
			dev_info(&client->dev, "UART DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_TYPE1_USB) {
			dev_info(&client->dev, "USB Cable DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_TYPE2_JIG_UART_OFF) {
			dev_info(&client->dev, "JIG_UARTOFF DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_TYPE2_JIG_UART_ON) {
			dev_info(&client->dev, "JIG_UARTON DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_TYPE2_JIG_USB_OFF) {
			dev_info(&client->dev, "JIG_USB_OFF DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_TYPE2_JIG_USB_ON) {
			dev_info(&client->dev, "JIG_USB_ON DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_TYPE2_PPD) {
			if (jig_wakelock_acq) {
				__pm_relax(&jig_suspend_wake);
				pm_qos_update_request(&usbsw->qos_idle,
						      PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);

				jig_wakelock_acq = 0;
				dev_info(&client->dev,
					 "AP WakeLock Release *****\n");
			}
		}
#ifdef CONFIG_EXTCON_SAMSUNG
		extcon_samsung_call_chain(S2MM001_CABLE_TYPE_NONE);
#endif
		attached_dev = S2MM001_CABLE_TYPE_NONE;
	}

	dev_info(&client->dev, "connector detect done.\n");

	return;
}

static void s2mm001_work_cb(struct work_struct *work)
{
	u8 intr1, intr2, val1;

	struct s2mm001_usbsw *usbsw =
	    container_of(work, struct s2mm001_usbsw, work);
	struct i2c_client *client = usbsw->client;

	msleep(50);

	mutex_lock(&usbsw->mutex);
	disable_irq(client->irq);

	/* Read and Clear Interrupt1/2 */
	retry_read_reg(client, S2MM001_MUIC_REG_INT1, &intr1);
	retry_read_reg(client, S2MM001_MUIC_REG_INT2, &intr2);

	retry_read_reg(client, S2MM001_MUIC_REG_RESET, &val1);
	if (val1 == RESET_BIT)
		s2mm001_reg_init(usbsw);

	detect_dev_s2mm001(usbsw, intr1, intr2, NULL);

	enable_irq(client->irq);
	mutex_unlock(&usbsw->mutex);
}

static int s2mm001_int_init(struct device_node *np, struct s2mm001_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;
	int ret, irq;
	u8 intr1, intr2;

	INIT_WORK(&usbsw->work, s2mm001_work_cb);

	irq = of_get_named_gpio(np, "connector-gpio", 0);
	if (irq < 0) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__, irq);
		return irq;
	}

	client->irq = gpio_to_irq(irq);
	ret =
	    request_irq(client->irq, microusb_irq_handler,
			IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING,
			"s2mm001 micro USB", usbsw);
	if (ret) {
		dev_err(&client->dev, "Unable to get IRQ %d\n", client->irq);
		return ret;
	}

	/* Read and Clear INTERRUPT1,2 REGS */
	mutex_lock(&usbsw->mutex);
	disable_irq(client->irq);
	read_reg(client, S2MM001_MUIC_REG_INT1, &intr1);
	read_reg(client, S2MM001_MUIC_REG_INT2, &intr2);
	enable_irq(client->irq);
	mutex_unlock(&usbsw->mutex);

	return 0;
}

static const struct of_device_id s2mm001_dt_ids[] = {
	{.compatible = "samsung,s2mm001",},
	{}
};

MODULE_DEVICE_TABLE(of, sec_charger_dt_ids);

static int s2mm001_probe_dt(struct device_node *np,
			   struct device *dev,
			   struct s2mm001_platform_data *pdata)
{
	int ret = 0;
	int connector_gpio = 0;
	const struct of_device_id *match;
	u32 lpm;

	if (!np)
		return -EINVAL;

	match = of_match_device(s2mm001_dt_ids, dev);
	if (!match)
		return -EINVAL;

	connector_gpio = of_get_named_gpio(np, "connector-gpio", 0);
	if (connector_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
		       connector_gpio);
		return connector_gpio;
	}
	ret = gpio_request(connector_gpio, "connector-gpio");
	if (ret) {
		pr_err("%s:%d gpio_request failed: connector_gpio %d\n",
		       __func__, __LINE__, connector_gpio);
		return ret;
	}

	gpio_direction_input(connector_gpio);

	ret = request_mfp_edge_wakeup(connector_gpio, NULL, pdata, dev);
	if (ret)
		pr_err("%s: failed to request edge wakeup.\n", __func__);

	if (!of_property_read_u32(np, "lpm-qos", &lpm))
		pdata->qos_val = lpm;
	else {
		pr_err("s2mm001: failed to get 'lpm-qos' from dt\n");
		return -EINVAL;
	}

	pr_info("%s: irq gpio no: %d", __func__, connector_gpio);

	return 0;
}

static struct s2mm001_platform_data s2mm001_info = {
	.charger_cb = sec_charger_cb,
};

static int s2mm001_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct s2mm001_usbsw *usbsw;
	struct device *switch_dev;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct s2mm001_platform_data *pdata = client->dev.platform_data;
	struct device_node *np = client->dev.of_node;
	int ret = 0;

	dev_info(&client->dev, "probe start\n");
	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata)
			pdata = &s2mm001_info;

		ret = s2mm001_probe_dt(np, &client->dev, pdata);
		if (ret)
			return ret;
	} else if (!pdata) {
		dev_err(&client->dev, "%s: no platform data defined\n",
			__func__);
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Not compatible i2c function\n");
		return -EIO;
	}
	/* For AT Command FactoryTest */
	wakeup_source_init(&jig_suspend_wake, "JIG_UART Connect suspend wake");

	usbsw = kzalloc(sizeof(struct s2mm001_usbsw), GFP_KERNEL);
	if (!usbsw) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	chip = usbsw;
	usbsw->client = client;
	usbsw->pdata = pdata;

	i2c_set_clientdata(client, usbsw);

	mutex_init(&usbsw->mutex);

	/* DeskTop Dock  */
	usbsw->dock_dev.name = "dock";
	ret = switch_dev_register(&usbsw->dock_dev);
	if (ret < 0)
		dev_err(&client->dev, "dock_dev_register error !!\n");

	if (!sec_class) {
		sec_class = class_create(THIS_MODULE, "sec");
	}
	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");
	if (device_create_file(switch_dev, &dev_attr_adc) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_adc.attr.name);
	if (device_create_file(switch_dev, &dev_attr_usb_state) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_usb_state.attr.name);
	if (device_create_file(switch_dev, &dev_attr_usb_sel) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_usb_sel.attr.name);
	if (device_create_file(switch_dev, &dev_attr_uart_sel) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_uart_sel.attr.name);
	if (device_create_file(switch_dev, &dev_attr_syssleep) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_syssleep.attr.name);
	if (device_create_file(switch_dev, &dev_attr_attached_dev) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_attached_dev.attr.name);
	dev_set_drvdata(switch_dev, usbsw);

	usbsw->qos_idle.name = "Jig driver";
	pm_qos_add_request(&usbsw->qos_idle, PM_QOS_CPUIDLE_BLOCK,
			   PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);

	ret = s2mm001_reg_init(usbsw);
	if (ret)
		goto s2mm001_probe_fail;

	/* device detection */
	dev_info(&client->dev, "First Detection\n");
	detect_dev_s2mm001(usbsw, INT1_ATTACH, 0, &first_acce);

	ret = s2mm001_int_init(np, usbsw);
	if (ret)
		goto s2mm001_probe_fail;

	probing = 0;
	dev_info(&client->dev, "PROBE Done.\n");

	return 0;

s2mm001_probe_fail:
	device_destroy(sec_class, 0);
	i2c_set_clientdata(client, NULL);
	kfree(usbsw);
	return ret;
}

static int s2mm001_resume(struct i2c_client *client)
{
	struct s2mm001_usbsw *usbsw = chip;
	u8 val1;

	dev_info(&client->dev, "s2mm001_resume\n");

	retry_read_reg (client, S2MM001_MUIC_REG_CTRL1, &val1);
	if ( val1 == CTRL1_DEF_INT)
		s2mm001_reg_init(usbsw);

	return 0;
}

static const struct i2c_device_id s2mm001_id[] = {
	{"s2mm001", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, s2mm001_id);

static struct i2c_driver s2mm001_i2c_driver = {
	.driver = {
		   .name = "s2mm001",
		   .of_match_table = s2mm001_dt_ids,
		   },
	.probe = s2mm001_probe,
	.resume = s2mm001_resume,
	.id_table = s2mm001_id,
};
#if 0 /* FIX ME */
static int __init en_uart_setup(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	en_uart = n;
	printk(KERN_INFO "en_uart = %d\n", en_uart);

	return 1;
}
__setup("uart_swc_en=", en_uart_setup);
#endif
static int __init s2mm001_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&s2mm001_i2c_driver);
}

module_init(s2mm001_init);

MODULE_LICENSE("GPL");

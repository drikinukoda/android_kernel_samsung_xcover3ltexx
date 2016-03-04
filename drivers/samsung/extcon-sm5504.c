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
#include <linux/regmap.h>
#include <linux/switch.h>

#include <linux/extcon/sm5504-muic.h>
#include <linux/extcon/extcon-samsung.h>

/* REGISTER */
enum sm5504_register {
	REG_DEVID		= 0x01,
	REG_CTRL		= 0x02,
	REG_INT1		= 0x03,
	REG_INT2		= 0x04,
	REG_INT1_MASK		= 0x05,
	REG_INT2_MASK		= 0x06,
	REG_ADC			= 0x07,
	REG_DEV_T1		= 0x0A,
	REG_DEV_T2		= 0x0B,
	REG_MANSW1		= 0x13,
	REG_MANSW2		= 0x14,
	REG_RESET		= 0x1B,
	REG_CHG_TYPE		= 0x24,
	REG_CHG_PUMP		= 0x3A,
	REG_END
};

/* CONTROL : REG_CTRL */
#define ADC_EN			(1 << 7)
#define USBCHDEN		(1 << 6)
#define CHGTYP			(1 << 5)
#define SWITCH_OPEN		(1 << 4)
#define MANUAL_SWITCH		(1 << 2)
#define MASK_INT		(1 << 0)
#define CTRL_INIT		(ADC_EN | USBCHDEN | CHGTYP | MANUAL_SWITCH)
#define RESET_DEFAULT		(ADC_EN | USBCHDEN | CHGTYP | MANUAL_SWITCH | MASK_INT)

/* INTERRUPT 1 : REG_INT1 */
#define ADC_CHG			(1 << 6)
#define CONNECT			(1 << 5)
#define OVP			(1 << 4)
#define DCD_OUT			(1 << 3)
#define CHGDET			(1 << 2)
#define DETACH			(1 << 1)
#define ATTACH			(1 << 0)

/* INTERRUPT 2 : REG_INT2 */
#define OVP_OCP			(1 << 7)
#define OCP			(1 << 6)
#define OCP_LATCH		(1 << 5)
#define OVP_FET			(1 << 4)
#define POR			(1 << 2)
#define UVLO			(1 << 1)
#define RID_CHARGER		(1 << 0)

/* INTMASK 1 : REG_INT1_MASK */
#define ADC_CHG_M		(1 << 6)
#define CONNECT_M		(1 << 5)
#define OVP_M			(1 << 4)
#define DCD_OUT_M		(1 << 3)
#define CHGDET_M		(1 << 2)
#define DETACH_M		(1 << 1)
#define ATTACH_M		(1 << 0)
#define INTMASK1_INIT		(ADC_CHG_M | CONNECT_M | OVP_M | DCD_OUT_M | CHGDET_M)

/* INTMASK 2 : REG_INT2_MASK */
#define OVP_OCP_M		(1 << 7)
#define OCP_M			(1 << 6)
#define OCP_LATCH_M		(1 << 5)
#define OVP_FET_M		(1 << 4)
#define POR_M			(1 << 2)
#define UVLO_M			(1 << 1)
#define RID_CHARGER_M		(1 << 0)
#define INTMASK2_INIT		(OVP_OCP_M | POR_M | !UVLO_M | RID_CHARGER_M)

/* DEVICE TYPE 1 : REG_DEV_T1 */
#define DEV_DCP			(1 << 6)	//Max 1.5A
#define DEV_CDP			(1 << 5)	//Max 1.5A with Data
#define DEV_CARKIT_T1		(1 << 4)
#define DEV_UART		(1 << 3)
#define DEV_SDP			(1 << 2)	//Max 500mA with Data
#define DEV_OTG			(1 << 0)
#define DEV_CHARGER		(DEV_DEDICATED_CHG | DEV_USB_CHG)

/* DEVICE TYPE 2 : REG_DEV_T2 */
#define DEV_UNKNOWN		(1 << 7)
#define DEV_JIG_UART_OFF	(1 << 3)
#define DEV_JIG_UART_ON		(1 << 2)
#define DEV_JIG_USB_OFF		(1 << 1)
#define DEV_JIG_USB_ON		(1 << 0)
#define DEV_JIG_ALL		(DEV_JIG_UART_OFF | DEV_JIG_UART_ON | DEV_JIG_USB_OFF | DEV_JIG_USB_ON)
#define DEV_JIG_WAKEUP		(DEV_JIG_UART_OFF | DEV_JIG_UART_ON | DEV_JIG_USB_ON)

/* MANUAL SWITCH 1 : REG_MANSW1 */
#define CON_TO_USB		0x24	/* 0010 0100 */
#define CON_TO_UART		0x6C	/* 0110 1100 */
#define CON_OPEN		0x00
#define MANUAL_SW1_MASK		0xFC

/* MANUAL SWITCH 2 : REG_MANSW2 */
#define BOOT_SW			(1 << 3)
#define JIG_ON			(1 << 2)
#define VBUS_FET_ONOFF		(1 << 0)

/* RESET : REG_RESET */
#define IC_RESET		(1 << 0)

/* REG_CHG_TYPE : REG_CHG_TYPE */
#define TIMEOUT_SDP		(1 << 3)
#define SDP			(1 << 2)
#define CDP			(1 << 1)
#define DCP			(1 << 0)

#define I2C_RW_RETRY_MAX    3
#define I2C_RW_RETRY_DELAY  15

static struct sm5504_usbsw {
	struct device *dev;
	struct sm5504_platform_data *pdata;
	struct work_struct work;
	struct pm_qos_request qos_idle;
	struct switch_dev dock_dev;
	struct mutex mutex;
	struct regmap *regmap;
	int irq;
	u8 id;
	u8 dev1;
	u8 dev2;
	u8 adc;
	u8 chg_type;
	u8 intr1;
	u8 intr2;
} *chip;

struct ic_vendor {
	u8 id;
	char *part_num;
};

static struct wakeup_source jig_suspend_wake;
static int jig_wakelock_acq;
static int probing = 1;
static int en_uart = 1;
static struct regmap_config sm5504_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_END,
};

static int read_reg(struct sm5504_usbsw *usbsw, u8 reg, u8 *data)
{
	int ret;
	unsigned int buf;

	ret = regmap_read(usbsw->regmap, (unsigned int)reg, &buf);
	if (ret < 0) {
		dev_err(usbsw->dev,"I2C Read REG[0x%.2X] failed\n", reg);
		return ret;
	}

	*data = (u8)buf;

	dev_info(usbsw->dev, "I2C Read REG[0x%.2X] DATA[0x%.2X]\n", reg, *data);
	return ret;
}

static int write_reg(struct sm5504_usbsw *usbsw, u8 reg, u8 data)
{
	int ret;

	ret = regmap_write(usbsw->regmap, (unsigned int)reg, (unsigned int) data);
	if (ret < 0) {
		dev_err(usbsw->dev,"I2C write REG[0x%.2X] failed\n", reg);
		return ret;
	}

	dev_info(usbsw->dev, "I2C Write REG[0x%.2X] DATA[0x%.2X]\n", reg, data);
	return ret;
}

static int retry_read_reg(struct sm5504_usbsw *usbsw, u8 reg, u8 * data)
{
	int result = -1, i;

	for (i = 0; i < I2C_RW_RETRY_MAX && (result < 0); i++) {
		result = read_reg(usbsw, reg , data);
		if (result < 0)
			msleep(I2C_RW_RETRY_DELAY);
	}
	return result;
}

static ssize_t adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sm5504_usbsw *usbsw = chip;
	u8 adc_value[] = "1C";
	u8 adc_fail = 0;

	if (usbsw->dev2 & DEV_JIG_ALL) {
		pr_info("adc_show JIG_UART_OFF\n");
		return sprintf(buf, "%s\n", adc_value);
	} else {
		pr_info("adc_show no detect\n");
		return sprintf(buf, "%d\n", adc_fail);
	}
}

static ssize_t sm5504_set_syssleep(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct sm5504_usbsw *usbsw = chip;

	dev_info(usbsw->dev, "%s+\n", __func__);
	if (!strncmp(buf, "1", 1)) {
		dev_info(usbsw->dev, "%s:release wake lock\n", __func__);
		pm_qos_update_request(&usbsw->qos_idle,
				      PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
		__pm_relax(&jig_suspend_wake);
	} else if (!strncmp(buf, "0", 1)) {
		dev_info(usbsw->dev, "%s:get wake lock\n", __func__);
		__pm_stay_awake(&jig_suspend_wake);
		pm_qos_update_request(&usbsw->qos_idle, PM_QOS_DEFAULT_VALUE);
	}
	dev_info(usbsw->dev, "%s-\n", __func__);

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
static DEVICE_ATTR(adc, S_IRUGO | S_IXOTH, adc_show, NULL);
static DEVICE_ATTR(syssleep, (S_IWUSR | S_IWGRP), NULL, sm5504_set_syssleep);
static DEVICE_ATTR(usb_state, S_IRUGO, usb_state_show_attrs, NULL);
static DEVICE_ATTR(usb_sel, S_IRUGO, usb_sel_show_attrs, NULL);
static DEVICE_ATTR(uart_sel, S_IRUGO | S_IWUSR | S_IWGRP, uart_sel_show_attrs,
		   uart_sel_store_attrs);
static DEVICE_ATTR(attached_dev, S_IRUGO, attached_dev_attrs, NULL);

static void manual_path_to_usb(int on)
{
	struct sm5504_usbsw *usbsw = chip;

	regmap_update_bits(usbsw->regmap, REG_MANSW1,
			MANUAL_SW1_MASK, on ? CON_TO_USB : CON_OPEN);
	regmap_update_bits(usbsw->regmap, REG_CTRL,
			MANUAL_SWITCH, on ? !MANUAL_SWITCH : MANUAL_SWITCH);
	dev_info(usbsw->dev, "maunal path to %s\n", on ? "USB" : "OPEN");
}

static void cutoff_path(int cut)
{
	struct sm5504_usbsw *usbsw = chip;

	regmap_update_bits(usbsw->regmap, REG_MANSW1,
			MANUAL_SW1_MASK, CON_OPEN);
	regmap_update_bits(usbsw->regmap, REG_CTRL,
			MANUAL_SWITCH, cut ? !MANUAL_SWITCH : MANUAL_SWITCH);
	dev_info(usbsw->dev, "cutoff the patch %s\n", cut ? "ON" : "OFF");
}

#if 0 /* FIX ME (unused) */
static void additional_uvlo_int(int on)
{
	struct sm5504_usbsw *usbsw = chip;

	regmap_update_bits(usbsw->regmap, REG_INT2_MASK,
				UVLO_M, on ? !UVLO_M : UVLO_M);
	dev_info(usbsw->dev, "uvlo interrupt %s\n", on ? "unmasked" : "masked");
}

static int sm5504_ic_reset(void)
{
	struct sm5504_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	u8 sintm1, sintm2, smansw1, sctrl;

	mutex_lock(&usbsw->mutex);
	disable_irq(client->irq);

	read_reg(regmap, REG_INT1_MASK, &sintm1);
	read_reg(regmap, REG_INT2_MASK, &sintm2);
	read_reg(regmap, REG_MANSW1, &smansw1);
	read_reg(regmap, REG_CTRL, &sctrl);

	write_reg(regmap, REG_RESET, IC_RESET);
	msleep(20);

	write_reg(regmap, REG_INT1_MASK, sintm1);
	write_reg(regmap, REG_INT2_MASK, sintm2);
	write_reg(regmap, REG_MANSW1, smansw1);
	write_reg(regmap, REG_CTRL, sctrl);

	dev_info(&client->dev, "sm5504 was reset!\n");

	enable_irq(client->irq);
	mutex_unlock(&usbsw->mutex);

	return 0;
}
#endif

static irqreturn_t microusb_irq_handler(int irq, void *data)
{
	struct sm5504_usbsw *usbsw = data;

	dev_info(usbsw->dev, "%s\n", __func__);
	schedule_work(&usbsw->work);

	return IRQ_HANDLED;
}

static int sm5504_reg_init(struct sm5504_usbsw *usbsw)
{
	int i, ret;
	struct ic_vendor muic_list[] = {
		{0x01, "SM5504"},
	};

	ret = read_reg(usbsw, REG_DEVID, &usbsw->id);
	if (ret < 0)
		return ret;
	for (i = 0; i < ARRAY_SIZE(muic_list); i++) {
		if (usbsw->id == muic_list[i].id)
			dev_info(usbsw->dev, "PartNum : %s\n",
				 muic_list[i].part_num);
	}

	/* INT MASK1, 2 */
	ret = write_reg(usbsw, REG_INT1_MASK, INTMASK1_INIT);
	if (ret < 0)
		return ret;
	ret = write_reg(usbsw, REG_INT2_MASK, INTMASK2_INIT);
	if (ret < 0)
		return ret;
	/* CONTROL */
	ret = write_reg(usbsw, REG_CTRL, CTRL_INIT);
	if (ret < 0)
		return ret;

	/* Charge Pump Enable*/
	ret = write_reg(usbsw, REG_CHG_PUMP, 0x00);
	if (ret < 0)
		return ret;

	return 0;
}

static void detect_dev_sm5504(struct sm5504_usbsw *usbsw, u8 intr1, u8 intr2,
			      void *data)
{
	struct sm5504_platform_data *pdata = usbsw->pdata;
	u8 val1, val2, adc, chg_type;
	int battery = 0;
	int dev_classifi = SM5504_CABLE_NONE_MUIC;

	read_reg(usbsw, REG_DEV_T1, &val1);
	read_reg(usbsw, REG_DEV_T2, &val2);
	read_reg(usbsw, REG_ADC, &adc);
	read_reg(usbsw, REG_CHG_TYPE, &chg_type);

	dev_info(usbsw->dev, "REG_INT1 : %.2X", intr1);
	dev_info(usbsw->dev, "REG_INT2 : %.2X", intr2);
	dev_info(usbsw->dev, "REG_DEV_T1 : %.2X", val1);
	dev_info(usbsw->dev, "REG_DEV_T2 : %.2X", val2);
	dev_info(usbsw->dev, "REG_ADC : %.2X", adc);
	dev_info(usbsw->dev, "REG_CHG_TYPE : %.2X", chg_type);

	if (probing == 1)
		read_reg(usbsw, REG_INT2, &intr2);

	if (intr1 & (ATTACH | CONNECT)) {
		dev_classifi = SM5504_CABLE_UNKNOWN;
		/* Attached */
		if (val1 & DEV_DCP && chg_type & DCP) {
			/* TA */
			dev_classifi = SM5504_CABLE_DCP_MUIC;
			dev_info(usbsw->dev, "DCP ATTACHED*****\n");
		}
		if (val1 & DEV_CDP && chg_type & CDP) {
			/* TA */
			dev_classifi = SM5504_CABLE_CDP_MUIC;
			dev_info(usbsw->dev, "CDP ATTACHED*****\n");
		}
		if (val1 & DEV_SDP && chg_type & SDP) {
			/* USB */
			dev_classifi = SM5504_CABLE_SDP_MUIC;
			dev_info(usbsw->dev, "SDP ATTACHED*****\n");
		}
		if (val1 & DEV_OTG) {
			dev_classifi = SM5504_CABLE_OTG_MUIC;
			dev_info(usbsw->dev, "OTG ATTACHED*****\n");
		}
		if (val1 & DEV_UART) {
			dev_classifi = SM5504_CABLE_UART_MUIC;
			dev_info(usbsw->dev, "UART ATTACHED*****\n");
		}
		if (val1 & DEV_CARKIT_T1) {
			manual_path_to_usb(1);
			dev_classifi = SM5504_CABLE_CARKIT_T1_MUIC;
			dev_info(usbsw->dev,
				 "CARKIT or L USB Cable ATTACHED*****\n");
		}
		if (val2 & DEV_JIG_UART_OFF) {
			if (intr2 & UVLO) {
				dev_classifi = SM5504_CABLE_JIG_UART_OFF_MUIC;
				dev_info(usbsw->dev,
					 "JIG_UARTOFF ATTACHED*****\n");
			} else {
				dev_classifi = SM5504_CABLE_JIG_UART_OFF_VB_MUIC;
				dev_info(usbsw->dev,
					 "JIG_UARTOFF + VBUS ATTACHED*****\n");
			}
#ifndef CONFIG_SEC_FACTORY
			battery = is_battery_connected();
			if (battery > 0 && en_uart == 0) {
				dev_info(usbsw->dev, "++ Manually cut off*****\n");
				cutoff_path(1);
				dev_info(usbsw->dev, "-- Manually cut off*****\n");
			}
#endif
		}
		if (val2 & DEV_JIG_UART_ON) {
			if (intr2 & UVLO) {
				dev_classifi = SM5504_CABLE_JIG_UART_ON_MUIC;
				dev_info(usbsw->dev,
					 "JIG_UARTON ATTACHED*****\n");
			} else {
				dev_classifi = SM5504_CABLE_JIG_UART_ON_VB_MUIC;
				dev_info(usbsw->dev,
					 "JIG_UARTON + VBUS ATTACHED*****\n");
			}
#ifndef CONFIG_SEC_FACTORY
			battery = is_battery_connected();
			if (battery > 0 && en_uart == 0) {
				dev_info(usbsw->dev, "++ Manually cut off*****\n");
				cutoff_path(1);
				dev_info(usbsw->dev, "-- Manually cut off*****\n");
			}
#endif
		}
		if (val2 & DEV_JIG_USB_OFF) {
			dev_classifi = SM5504_CABLE_JIG_USB_OFF_MUIC;
			dev_info(usbsw->dev, "JIG_USB_OFF ATTACHED*****\n");
		}
		if (val2 & DEV_JIG_USB_ON) {
			dev_classifi = SM5504_CABLE_JIG_USB_ON_MUIC;
			dev_info(usbsw->dev, "JIG_USB_ON ATTACHED*****\n");
		}
		if (val2 & DEV_JIG_WAKEUP) {
			if (!jig_wakelock_acq) {
				__pm_stay_awake(&jig_suspend_wake);
				pm_qos_update_request(&usbsw->qos_idle,
						      pdata->qos_val);
				jig_wakelock_acq = 1;
				dev_info(usbsw->dev,
					 "AP WakeLock for FactoryTest *****\n");
			}
		}
		if (val2 & DEV_UNKNOWN && adc == 0x1A) {
			if (intr2 & UVLO) {
				dev_classifi = SM5504_CABLE_DESKTOP_DOCK_MUIC;
				dev_info(usbsw->dev, "DESKTOP DOCK ATTACHED*****\n");
			} else {
				dev_classifi = SM5504_CABLE_DESKTOP_DOCK_VB_MUIC;
				dev_info(usbsw->dev, "DESKTOP DOCK + VBUS ATTACHED*****\n");
			}
		}

#ifdef CONFIG_EXTCON_SAMSUNG
		extcon_samsung_call_chain(dev_classifi);
#endif
		if (probing == 1)
			*(int *)data = dev_classifi;

		usbsw->dev1 = val1;
		usbsw->dev2 = val2;
		usbsw->adc = adc;
		usbsw->chg_type = chg_type;
		usbsw->intr1 = intr1;
		usbsw->intr2 = intr2;
	}

	if (intr1 & DETACH) {
		/* Detached */
		if (usbsw->dev1 & DEV_DCP && usbsw->chg_type & DCP) {
			dev_info(usbsw->dev, "DCP DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_CDP && usbsw->chg_type & CDP) {
			dev_info(usbsw->dev, "CDP DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_SDP && usbsw->chg_type & SDP) {
			dev_info(usbsw->dev, "SDP DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_OTG) {
			dev_info(usbsw->dev, "OTG DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_UART) {
			dev_info(usbsw->dev, "UART DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_CARKIT_T1) {
			manual_path_to_usb(0);
			dev_info(usbsw->dev,
				 "CARKIT or L USB Cable DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_JIG_UART_OFF) {
			if (usbsw->intr2 & UVLO) {
				dev_info(usbsw->dev,
					 "JIG_UARTOFF DETACHED*****\n");
			} else {
				dev_info(usbsw->dev,
					 "JIG_UARTOFF + VBUS DETACHED*****\n");
			}
#ifndef CONFIG_SEC_FACTORY
			dev_info(usbsw->dev, "++ Automatic switching*****\n");
			cutoff_path(0);
			dev_info(usbsw->dev, "-- Automatic switching*****\n");
#endif
		}
		if (usbsw->dev2 & DEV_JIG_UART_ON) {
			if (usbsw->intr2 & UVLO) {
				dev_info(usbsw->dev,
					 "JIG_UARTON DETACHED*****\n");
			} else {
				dev_info(usbsw->dev,
					 "JIG_UARTON + VBUS DETACHED*****\n");
			}
#ifndef CONFIG_SEC_FACTORY
			dev_info(usbsw->dev, "++ Automatic switching*****\n");
			cutoff_path(0);
			dev_info(usbsw->dev, "-- Automatic switching*****\n");
#endif
		}
		if (usbsw->dev2 & DEV_JIG_USB_OFF) {
			dev_info(usbsw->dev, "JIG_USB_OFF DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_JIG_USB_ON) {
			dev_info(usbsw->dev, "JIG_USB_ON DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_JIG_WAKEUP) {
			if (jig_wakelock_acq) {
				__pm_relax(&jig_suspend_wake);
				pm_qos_update_request(&usbsw->qos_idle,
						      PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);

				jig_wakelock_acq = 0;
				dev_info(usbsw->dev,
					 "AP WakeLock Release *****\n");
			}
		}
		if (usbsw->dev2 & DEV_UNKNOWN && usbsw->adc == 0x1A) {
			if (usbsw->intr2 & UVLO) {
				dev_info(usbsw->dev, "DESKTOP DOCK DETACHED*****\n");
			} else {
				dev_info(usbsw->dev, "DESKTOP DOCK + VBUS DETACHED*****\n");
			}
		}
#ifdef CONFIG_EXTCON_SAMSUNG
		extcon_samsung_call_chain(SM5504_CABLE_NONE_MUIC);
#endif
	}

	/*
	 * SM5504 doesn't support DESKTOP-DOCK officially
	 * So, it doesn't report the attaching event in case of VBUS IN&OUT to DESKTOP-DOCK
	 */
	if (intr1 == 0x0) {
		if (val2 & DEV_UNKNOWN && adc == 0x1A) {
			if (intr2 & UVLO) {
				dev_classifi = SM5504_CABLE_DESKTOP_DOCK_MUIC;
				dev_info(usbsw->dev, "DESKTOP DOCK ATTACHED*****\n");
			} else {
				dev_classifi = SM5504_CABLE_DESKTOP_DOCK_VB_MUIC;
				dev_info(usbsw->dev, "DESKTOP DOCK + VBUS ATTACHED*****\n");
			}
		}

		if (probing == 1)
			*(int *)data = dev_classifi;
#ifdef CONFIG_EXTCON_SAMSUNG
			extcon_samsung_call_chain(dev_classifi);
#endif
		usbsw->dev1 = val1;
		usbsw->dev2 = val2;
		usbsw->adc = adc;
		usbsw->chg_type = chg_type;
		usbsw->intr1 = intr1;
		usbsw->intr2 = intr2;
	}

	if (intr2 & (OCP | OCP_LATCH | OVP_FET)) {
		if (intr2 & OCP)
			dev_info(usbsw->dev, "OCP *****\n");
		if (intr2 & OCP_LATCH)
			dev_info(usbsw->dev, "OCP_LATCH *****\n");
		if (intr2 & OVP_FET)
			dev_info(usbsw->dev, "OVP_FET *****\n");
	}

	return;
}

static void sm5504_work_cb(struct work_struct *work)
{
	u8 intr1, intr2, val1;

	struct sm5504_usbsw *usbsw =
	    container_of(work, struct sm5504_usbsw, work);
	msleep(50);

	mutex_lock(&usbsw->mutex);
	disable_irq(usbsw->irq);

	/* Read and Clear Interrupt1/2 */
	retry_read_reg(usbsw, REG_INT1, &intr1);
	retry_read_reg(usbsw, REG_INT2, &intr2);

	retry_read_reg(usbsw, REG_CTRL, &val1);
	if (val1 == RESET_DEFAULT)
		sm5504_reg_init(usbsw);

	extcon_samsung_vbus_call_chain((intr2 & UVLO) ? false : true);

	detect_dev_sm5504(usbsw, intr1, intr2, NULL);

	enable_irq(usbsw->irq);
	mutex_unlock(&usbsw->mutex);
}

static int sm5504_int_init(struct device_node *np, struct sm5504_usbsw *usbsw)
{
	int ret, irq;
	u8 intr1, intr2;

	INIT_WORK(&usbsw->work, sm5504_work_cb);

	irq = of_get_named_gpio(np, "connector-gpio", 0);
	if (irq < 0) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__, irq);
		return irq;
	}

	usbsw->irq = gpio_to_irq(irq);
	ret =
	    request_irq(usbsw->irq, microusb_irq_handler,
			IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING,
			"sm5504 micro USB", usbsw);
	if (ret) {
		dev_err(usbsw->dev, "Unable to get IRQ %d\n", usbsw->irq);
		return ret;
	}

	/* Read and Clear INTERRUPT1,2 REGS */
	mutex_lock(&usbsw->mutex);
	disable_irq(usbsw->irq);
	read_reg(usbsw, REG_INT1, &intr1);
	read_reg(usbsw, REG_INT2, &intr2);
	enable_irq(usbsw->irq);
	mutex_unlock(&usbsw->mutex);

	return 0;
}

static const struct of_device_id sm5504_dt_ids[] = {
	{.compatible = "samsung,sm5504",},
	{}
};

MODULE_DEVICE_TABLE(of, sec_charger_dt_ids);

static int sm5504_probe_dt(struct device_node *np,
			   struct device *dev,
			   struct sm5504_platform_data *pdata)
{
	int ret = 0;
	int connector_gpio = 0;
	const struct of_device_id *match;
	u32 lpm;

	if (!np)
		return -EINVAL;

	match = of_match_device(sm5504_dt_ids, dev);
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
		pr_err("sm5504: failed to get 'lpm-qos' from dt\n");
		return -EINVAL;
	}

	return 0;
}

static struct sm5504_platform_data sm5504_info = {
	.charger_cb = sec_charger_cb,
};

static int sm5504_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sm5504_usbsw *usbsw;
	struct device *switch_dev;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct sm5504_platform_data *pdata = client->dev.platform_data;
	struct device_node *np = client->dev.of_node;
	int ret = 0;

	dev_info(&client->dev, "probe start\n");
	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata)
			pdata = &sm5504_info;

		ret = sm5504_probe_dt(np, &client->dev, pdata);
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

	usbsw = kzalloc(sizeof(struct sm5504_usbsw), GFP_KERNEL);
	if (!usbsw) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	chip = usbsw;
	usbsw->dev = &client->dev;
	usbsw->pdata = pdata;
	usbsw->regmap = devm_regmap_init_i2c(client, &sm5504_regmap_config);
	if (IS_ERR_OR_NULL(usbsw->regmap)) {
		dev_err(&client->dev, "Failed to allocate register map\n");
		return -ENOMEM;
	}

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

	ret = sm5504_reg_init(usbsw);
	if (ret)
		goto sm5504_probe_fail;

	/* device detection */
	dev_info(&client->dev, "First Detection\n");
	detect_dev_sm5504(usbsw, ATTACH, 0, &first_acce);

	ret = sm5504_int_init(np, usbsw);
	if (ret)
		goto sm5504_probe_fail;

	probing = 0;
	dev_info(&client->dev, "PROBE Done.\n");

	return 0;

sm5504_probe_fail:
	device_destroy(sec_class, 0);
	i2c_set_clientdata(client, NULL);
	kfree(usbsw);
	return ret;
}

static int sm5504_resume(struct i2c_client *client)
{
	struct sm5504_usbsw *usbsw = chip;
	u8 val1;

	dev_info(usbsw->dev, "sm5504_resume\n");

	retry_read_reg (chip, REG_CTRL, &val1);
	if ( val1 == RESET_DEFAULT)
		sm5504_reg_init(usbsw);

	return 0;
}

static const struct i2c_device_id sm5504_id[] = {
	{"sm5504", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sm5504_id);

static struct i2c_driver sm5504_i2c_driver = {
	.driver = {
		   .name = "sm5504",
		   .of_match_table = sm5504_dt_ids,
		   },
	.probe = sm5504_probe,
	.resume = sm5504_resume,
	.id_table = sm5504_id,
};

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

static int __init sm5504_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&sm5504_i2c_driver);
}

module_init(sm5504_init);

MODULE_LICENSE("GPL");

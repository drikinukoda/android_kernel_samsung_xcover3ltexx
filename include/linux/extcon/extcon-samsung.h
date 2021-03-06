/*
 *
 * Copyright (c) Samsung Electronics Co, Ltd.
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

#ifndef _EXTCON_SAMSUNG_H_
#define _EXTCON_SAMSUNG_H_

#include <linux/notifier.h>

enum samsung_cable_type {
	CABLE_NONE = 0,
	CABLE_TA,
	CABLE_USB_SDP,
	CABLE_USB_CDP,
	CABLE_UART,
	CABLE_OTG,
	CABLE_MHL,
	CABLE_DESKDOCK,
	CABLE_AUDIODOCK,
	CABLE_SMARTDOCK_TA,
	CABLE_SMARTDOCK_USB,
	CABLE_JIG_USB_ON,
	CABLE_JIG_USB_OFF,
	CABLE_JIG_UART_ON,
	CABLE_JIG_UART_ON_VB,
	CABLE_JIG_UART_OFF,
	CABLE_JIG_UART_OFF_VB,
	CABLE_UNKNOWN,
};

struct extcon_cable
{
	int id;
	char *name;
};

#define DEFINE_EXTCON_CABLE(cable_name)	\
{					\
	.id = cable_name,		\
	.name = #cable_name"\n",		\
}

#if defined(CONFIG_EXTCON_SAMSUNG)
extern int first_acce;
int usb_switch_register_notify(struct notifier_block *nb);
int usb_switch_unregister_notify(struct notifier_block *nb);
void extcon_samsung_call_chain(const int cable_type);
int vbus_register_notify(struct notifier_block *nb);
int vbus_unregister_notify(struct notifier_block *nb);
void extcon_samsung_vbus_call_chain(const bool vbus_in);
bool get_jig_state(void);
void muic_attached_accessory_inquire(void);
ssize_t attached_dev_attrs(struct device *dev,
			struct device_attribute *attr, char *buf);
#else
static int first_acce;

static int usb_switch_register_notify(struct notifier_block *nb)
{
	return 0;
}

static int usb_switch_unregister_notify(struct notifier_block *nb)
{
	return 0;
}

static void extcon_samsung_call_chain(const int cable_type) { }

static int vbus_register_notify(struct notifier_block *nb)
{
	return 0;
}

static int vbus_unregister_notify(struct notifier_block *nb)
{
	return 0;
}

static void extcon_samsung_vbus_call_chain(const bool vbus_in) { };

static bool get_jig_state(void)
{
	return false;
}

static void muic_attached_accessory_inquire(void) { }

static ssize_t attached_dev_attrs(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "\n");
}
#endif
#endif

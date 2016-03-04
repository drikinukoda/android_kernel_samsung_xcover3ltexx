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

#ifndef _SM5502_MUIC_H_
#define _SM5502_MUIC_H_

#include <linux/types.h>

enum sm5502_cable_type {
	SM5502_CABLE_TYPE_NONE = 0,
	SM5502_CABLE_TYPE1_AUDIO1,
	SM5502_CABLE_TYPE1_AUDIO2,
	SM5502_CABLE_TYPE1_USB,
	SM5502_CABLE_TYPE1_UART,
	SM5502_CABLE_TYPE1_CARKIT,
	SM5502_CABLE_TYPE1_TA,
	SM5502_CABLE_TYPE1_OTG,
	SM5502_CABLE_TYPE2_JIG_USB_ON,
	SM5502_CABLE_TYPE2_JIG_USB_OFF,
	SM5502_CABLE_TYPE2_JIG_UART_ON,
	SM5502_CABLE_TYPE2_JIG_UART_ON_VB,
	SM5502_CABLE_TYPE2_JIG_UART_OFF,
	SM5502_CABLE_TYPE2_JIG_UART_OFF_VB,
	SM5502_CABLE_TYPE2_DESKDOCK,
	SM5502_CABLE_TYPE3_MHL_VB,
	SM5502_CABLE_TYPE3_MHL,
	SM5502_CABLE_TYPE3_DESKDOCK_VB,
	SM5502_CABLE_TYPE3_U200CHG,
	SM5502_CABLE_TYPE3_NONSTD_SDP,
	SM5502_CABLE_UNKNOWN,
};

struct sm5502_platform_data {
	int intb_gpio;
	void (*usb_cb) (u8 attached);
	void (*uart_cb) (u8 attached);
	void (*charger_cb) (u8 attached);
	void (*jig_cb) (u8 attached);
	void (*reset_cb) (void);
	void (*mhl_cb) (u8 attached);
	void (*cardock_cb) (bool attached);
	void (*deskdock_cb) (bool attached);
	void (*id_open_cb) (void);
	u32 qos_val;
};

/* Control for Analog Audio Dock device */
extern void sm5502_dock_audiopath_ctrl(int on);
extern void sm5502_chgpump_ctrl(int enable);
extern int sm5502_ic_reset(void);

#endif /* _SM5502_MUIC_H_ */

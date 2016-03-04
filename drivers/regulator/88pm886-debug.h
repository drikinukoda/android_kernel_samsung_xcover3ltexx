/*
 * 88pm886-debug.h - driver for taking 88pm886 register dump
 *
 * Copyright (C) 2014, Samsung Electronics, Co., Ltd. All Rights Reserved.
 * Abilash Chandrasekar <abilash.c@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __LINUX_REGULATOR_88PM886_DEBUG
#define __LINUX_REGULATOR_88PM886_DEBUG

#define START_ADDR 0x00
#define TOTAL_REGS 191

/* ldo voltage registers addresses
   ldox_set_slp bits[7:4]
   ldox_set bits[3:0]
*/

#define PM886_LDO1_VOUT		(0x20)
#define PM886_LDO2_VOUT		(0x26)
#define PM886_LDO3_VOUT		(0x2c)
#define PM886_LDO4_VOUT		(0x32)
#define PM886_LDO5_VOUT		(0x38)
#define PM886_LDO6_VOUT		(0x3e)
#define PM886_LDO7_VOUT		(0x44)
#define PM886_LDO8_VOUT		(0x4a)
#define PM886_LDO9_VOUT		(0x50)
#define PM886_LDO10_VOUT	(0x56)
#define PM886_LDO11_VOUT	(0x5c)
#define PM886_LDO12_VOUT	(0x62)
#define PM886_LDO13_VOUT	(0x68)
#define PM886_LDO14_VOUT	(0x6e)
#define PM886_LDO15_VOUT	(0x74)
#define PM886_LDO16_VOUT	(0x7a)

/* ldo enable registers addresses*/
#define PM886_LDO_EN1		(0x09)
#define PM886_LDO_EN2		(0x0a)


static const unsigned int vset1_3[] = {
	1700000, 1800000, 1900000, 2500000, 2800000, 2900000, 3100000, 3300000,
};

static const unsigned int vset4_15[] = {
	1200000, 1250000, 1700000, 1800000, 1850000, 1900000, 2500000, 2600000,
	2700000, 2750000, 2800000, 2850000, 2900000, 3000000, 3100000, 3300000,
};

static const unsigned int vset16[] = {
	1700000, 1800000, 1900000, 2000000, 2100000, 2500000, 2700000, 2800000,
};

char *names_registers[] = {
		" LDO1",
		" LDO2",
		" LDO3",
		" LDO4",
		" LDO5",
		" LDO6",
		" LDO7",
		" LDO8",
		" LDO9",
		" LDO10",
		" LDO11",
		" LDO12",
		" LDO13",
		" LDO14",
		" LDO15",
		" LDO16",
};

#endif

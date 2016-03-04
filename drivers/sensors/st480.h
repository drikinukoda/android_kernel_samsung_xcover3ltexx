/*
 * Copyright (C) 2012 Senodia.
 *
 * Author: Tori Xu <xuezhi_xu@senodia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Definitions for senodia compass chip.
 */
#ifndef ST480_H
#define ST480_H

/*
 * IC Package size(choose your ic model number)
 * ST480MB: 2X2_BGA
 * ST480MF: 1_6X1_6_LGA
 * ST480MW: 1_6X1_6_BGA
 * ST480MC: 1_2X1_2
 */
#define ST480_SIZE_2X2_BGA	0
#define ST480_SIZE_1_6X1_6_LGA	0
#define ST480_SIZE_1_6X1_6_BGA	0
#define ST480_SIZE_1_2X1_2	1

/*
 * IC position
 * ( Top : ST480_BOARD_LOCATION_FRONT )
 * ( Bottom: ST480_BOARD_LOCATION_BACK)
 * ( Upper Left: ST480_BOARD_LOCATION_DEGREE_0)
 * ( Upper Right: ST480_BOARD_LOCATION_DEGREE_90 )
 * ( Lower Right: ST480_BOARD_LOCATION_DEGREE_180)
 * ( Lower Left: ST480_BOARD_LOCATION_DEGREE_270)
 */
#define ST480_BOARD_LOCATION_FRONT 1
#define ST480_BOARD_LOCATION_BACK 0
#define ST480_BOARD_LOCATION_DEGREE_0 0
#define ST480_BOARD_LOCATION_DEGREE_90 0
#define ST480_BOARD_LOCATION_DEGREE_180 0
#define ST480_BOARD_LOCATION_DEGREE_270 1

/*
 * register shift
 */
#define ST480_REG_DRR_SHIFT 2

/*
 * BURST MODE(INT)
 */
#define ST480_BURST_MODE 0
#define BURST_MODE_CMD 0x1F
#define BURST_MODE_DATA_LOW 0x01

/*
 * SINGLE MODE
 */
#define ST480_SINGLE_MODE 1
#define SINGLE_MEASUREMENT_MODE_CMD 0x3F

/*
 * register
 */
#define READ_MEASUREMENT_CMD 0x4F
#define WRITE_REGISTER_CMD 0x60
#define READ_REGISTER_CMD 0x50
#define EXIT_REGISTER_CMD 0x80
#define MEMORY_RECALL_CMD 0xD0
#define MEMORY_STORE_CMD 0xE0
#define RESET_CMD 0xF0
#define BIST_SINGLE_MEASUREMENT_MODE_CMD 0x38
#define BIST_READ_MEASUREMENT_CMD 0x48

#define REGISTER_ZERO_DEFAULT_VALUE 0x007c
#define REGISTER_ONE_DEFAULT_VALUE 0x0000

#define DEVICE_ID_REG (0x10 << ST480_REG_DRR_SHIFT)
#define DEVICE_ID  0x3A3A

#define CALIBRATION_REG (0x02 << ST480_REG_DRR_SHIFT)
#define CALIBRATION_DATA_LOW 0x1C
#define CALIBRATION_DATA_HIGH 0x00

#define ONE_INIT_DATA_LOW 0x7C
#define ONE_INIT_DATA_HIGH 0x00
#define ONE_INIT_BIST_TEST 0x01
#define ONE_INIT_REG (0x00 << ST480_REG_DRR_SHIFT)

#define TWO_INIT_DATA_LOW 0x00
#define TWO_INIT_DATA_HIGH 0x00
#define TWO_INIT_REG (0x02 << ST480_REG_DRR_SHIFT)

#define REGISTER_ONE (0x01 << ST480_REG_DRR_SHIFT)

/*
 * Miscellaneous set.
 */
#define MAX_FAILURE_COUNT 3
#define ST480_DEFAULT_DELAY 200000000LL

/*
 * Xcover3 TA charging compensation
 */
#define TA_X_CHARGE_COMPE -67
#define TA_Y_CHARGE_COMPE 20
#define TA_Z_CHARGE_COMPE 34

/*
 * Xcover3 USB charging compensation
 */
#define USB_X_CHARGE_COMPE -43
#define USB_Y_CHARGE_COMPE 18
#define USB_Z_CHARGE_COMPE 20

enum {
	CABLE_STATUS_NONE = 0,
	CABLE_STATUS_TA,
	CABLE_STATUS_USB,
};

#endif


/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * STMicroelectronics st_lsm6dsrx mlc preload config
 *
 * Copyright 2022 STMicroelectronics Inc.
 */

#ifndef ST_LSM6DSRX_PRELOAD_MLC_H
#define ST_LSM6DSRX_PRELOAD_MLC_H

static const u8 mlcdata[] = {
	/*
	 * fill data from MLC / FSM stored in:
	 * https://github.com/STMicroelectronics/STMems_Finite_State_Machine
	 *
	 * or
	 *
	 * https://github.com/STMicroelectronics/STMems_Machine_Learning_Core
	 *
	 * based on the needs and type of device (lsm6dsr or lsm6dsrx)
	 * Follow an example for lsm6dsr FSM free fall detection.
	 */
	/* lsm6dsr_freefall_detection.ucf */
	0x01, 0x80, 0x04, 0x00, 0x05, 0x00, 0x5f, 0x0a, 0x46, 0x01,
	0x47, 0x00, 0x0a, 0x00, 0x0b, 0x01, 0x0c, 0x00, 0x0e, 0x00,
	0x0f, 0x00, 0x10, 0x00, 0x02, 0x01, 0x17, 0x40, 0x09, 0x00,
	0x02, 0x11, 0x08, 0x7a, 0x09, 0x00, 0x09, 0x00, 0x09, 0x01,
	0x09, 0x01, 0x09, 0x00, 0x09, 0x04, 0x02, 0x41, 0x08, 0x00,
	0x09, 0x71, 0x09, 0x10, 0x09, 0x1c, 0x09, 0x00, 0x09, 0x11,
	0x09, 0x00, 0x09, 0xcd, 0x09, 0x34, 0x09, 0xa8, 0x09, 0x00,
	0x09, 0x00, 0x09, 0x00, 0x09, 0x01, 0x09, 0x00, 0x09, 0x00,
	0x09, 0x03, 0x09, 0x00, 0x09, 0xf5, 0x09, 0x77, 0x09, 0x99,
	0x09, 0x12, 0x09, 0x66, 0x09, 0x53, 0x09, 0xc7, 0x09, 0x88,
	0x09, 0x99, 0x09, 0x50, 0x09, 0x00, 0x04, 0x00, 0x05, 0x01,
	0x17, 0x00, 0x02, 0x01, 0x01, 0x00, 0x14, 0x80, 0x10, 0x28,
	0x11, 0x00, 0x5e, 0x02,
};

static struct firmware st_lsm6dsrx_mlc_preload = {
		.size = sizeof(mlcdata),
		.data = mlcdata
};

#endif /* ST_LSM6DSRX_PRELOAD_MLC_H */

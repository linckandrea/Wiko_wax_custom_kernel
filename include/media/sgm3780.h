/*
* sgm3780.h - sgm3780 sensor driver
*
* Copyright (c) 2012-2013 NVIDIA Corporation. All Rights Reserved.
*
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any
* kind, whether express or implied.
*/

#ifndef __SGM3780_H__
#define __SGM3780_H__

#include <media/nvc_torch.h>
#include <media/nvc_image.h>

struct tinno_flash_power_rail {
	/* to enable the module power */
	struct regulator *vin;
	/* to enable the host interface power */
	struct regulator *vdd;
};

struct tinno_flash_led_config {
	u16 flash_torch_ratio;	/* max flash to max torch ratio, in 1/1000 */
	u16 granularity;	/* 1, 10, 100, ... to carry float settings */
	u16 flash_levels;	/* calibrated flash levels < 32 */
	/* this table contains the calibrated flash level - luminance pair */
	struct nvc_torch_lumi_level_v1 *lumi_levels;
};

struct tinno_flash_config {
	struct tinno_flash_led_config led_config[1];
};

struct tinno_flash_platform_data {
	struct tinno_flash_config config;
	const char *dev_name; /* see implementation notes in driver */
	struct nvc_torch_pin_state pinstate; /* see notes in driver */
	struct edp_client edpc_config;
	int cfg;
	int num;
	int (*poweron_callback)(struct tinno_flash_power_rail *pw);
	int (*poweroff_callback)(struct tinno_flash_power_rail *pw);
	int gpio_en_torch;
	int gpio_en_flash;
	int edp_state_flash;
	int edp_state_torch;
	int timeout_ms;
	void (*flash_dev_cb)(struct device *, union nvc_imager_flash_control *);
};

#endif


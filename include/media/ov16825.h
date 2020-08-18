/*
 * nvc_ov16825.h - ov16825 sensor driver
 *
 *  * Copyright (c) 2012 NVIDIA Corporation.  All rights reserved.
 */

#ifndef __OV16825_H__
#define __OV16825_H__

#include <media/nvc.h>
#include <media/nvc_image.h>
#include <linux/edp.h>

/* See notes in the nvc.h file on the GPIO usage */
enum ov16825_gpio_type {
	OV16825_GPIO_TYPE_SHTDN = 0,
	OV16825_GPIO_TYPE_PWRDN,
	OV16825_CUSTOMER_BOARD_GPIO,
};

struct ov16825_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *dovdd;
};

struct ov16825_platform_data {
	unsigned cfg;
	unsigned num;
	unsigned sync;
	const char *dev_name;
	unsigned gpio_count;
	struct nvc_gpio_pdata *gpio; /* see nvc.h GPIO notes */
	struct nvc_imager_cap *cap;
	unsigned lens_focal_length; /* / _INT2FLOAT_DIVISOR */
	unsigned lens_max_aperture; /* / _INT2FLOAT_DIVISOR */
	unsigned lens_fnumber; /* / _INT2FLOAT_DIVISOR */
	unsigned lens_view_angle_h; /* / _INT2FLOAT_DIVISOR */
	unsigned lens_view_angle_v; /* / _INT2FLOAT_DIVISOR */
	int (*probe_clock)(unsigned long);
	int (*power_on)(struct ov16825_power_rail *, struct nvc_gpio_pdata *, int size_gpio);
	int (*power_off)(struct ov16825_power_rail *, struct nvc_gpio_pdata *, int size_gpio);
	char *clk_name;
	struct edp_client edpc_config;
};
#endif  /* __OV16825_H__ */

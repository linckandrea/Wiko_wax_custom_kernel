/*
 * arch/arm/mach-tegra/board-touch-raydium_spi.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 * Copyright (c) 2012, Synaptics Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include "board-touch-320x.h"
#include <linux/input/syn320x_pdata.h>
#include "board-ceres.h"

/*
#define SYNAPTICS_BUTTON_CODES {KEY_HOME, KEY_BACK,}

static unsigned char synaptics_button_codes[] = SYNAPTICS_BUTTON_CODES;

struct rmi_button_map synaptics_button_map = {
	.nbuttons = ARRAY_SIZE(synaptics_button_codes),
	.map = synaptics_button_codes,
};

int synaptics_touchpad_gpio_setup(void *gpio_data, bool configure)
{
	struct synaptics_gpio_data *syna_gpio_data =
		(struct synaptics_gpio_data *)gpio_data;

	if (!gpio_data)
		return -EINVAL;

	pr_info("%s: called with configure=%d, SYNAPTICS_ATTN_GPIO=%d\n",
		__func__, configure, syna_gpio_data->attn_gpio);

	if (configure) {
		gpio_request(syna_gpio_data->attn_gpio, "synaptics-irq");
		gpio_direction_input(syna_gpio_data->attn_gpio);

		gpio_request(syna_gpio_data->reset_gpio, "synaptics-reset");
		gpio_direction_output(syna_gpio_data->reset_gpio, 0);

		msleep(20);
		gpio_set_value(syna_gpio_data->reset_gpio, 1);
		msleep(100);
	} else {
		gpio_free(syna_gpio_data->attn_gpio);
		gpio_free(syna_gpio_data->reset_gpio);
	}
	return 0;
}

*/


static struct syn320x_platform_data syn320x_pdata = {
	  .irq_pin = TEGRA_GPIO_PN1,
	  .reset_pin = TEGRA_GPIO_PN2,				//Ivan CAM_RSTN
	  .pdown_pin = 0,				//Ivan CAM1_POWER_DWN_GPIO
	  .scl_pin = TEGRA_GPIO_PR2,
	  .sda_pin = TEGRA_GPIO_PR3,
	  .addr = SYN_320X_ADDR,
	};

static struct i2c_board_info __initdata ceres_i2c_bus2_syn320x_info[] = {
	{
		I2C_BOARD_INFO("s320_tp", SYN_320X_ADDR),
		.platform_data = &syn320x_pdata,
	},
};


int __init touch_init_syn320x(void)
{

//	spi_register_board_info(board_info, board_info_size);z
	i2c_register_board_info(0, ceres_i2c_bus2_syn320x_info,
		ARRAY_SIZE(ceres_i2c_bus2_syn320x_info));	
	return 0;
}

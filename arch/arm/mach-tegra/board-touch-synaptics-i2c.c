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
#include "board-touch-synaptics-i2c.h"
#include "gpio-names.h"
#include <linux/input/synaptics_dsx.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#ifdef CONFIG_MACH_S8515
#include "board-s8515.h"
#endif   
#ifdef CONFIG_MACH_S9321
#include "board-s9321.h"
#endif   

#define TM_SAMPLE1_ADDR 0x72 >> 1
#define TM_SAMPLE1_ATTN 130

static unsigned char TM_SAMPLE1_f1a_button_codes[] = {KEY_MENU, KEY_HOMEPAGE, KEY_BACK};

static int synaptics_gpio_setup(unsigned gpio, bool configure)
{
	int retval=0;
	if (configure)
	{
		retval = gpio_request(gpio, "synaptics-irq");
		if (retval) {
			pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			       __func__, gpio, retval);
			return retval;
		}
		//omap_mux_init_signal("sdmmc2_clk.gpio_130", OMAP_PIN_INPUT_PULLUP);

		retval = gpio_direction_input(gpio);
		if (retval) {
			pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			       __func__, gpio, retval);
			gpio_free(gpio);
		}
	} else {
		pr_warn("%s: No way to deconfigure gpio %d.",
		       __func__, gpio);
	}

	return retval;
}

static struct synaptics_dsx_cap_button_map TM_SAMPLE1_cap_button_map = {
	.nbuttons = ARRAY_SIZE(TM_SAMPLE1_f1a_button_codes),
	.map = TM_SAMPLE1_f1a_button_codes,
};

static struct synaptics_dsx_platform_data dsx_platformdata = {
	.irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT,//IRQF_TRIGGER_FALLING,
	.irq_gpio = TEGRA_GPIO_PN1,//TM_SAMPLE1_ATTN,
	.reset_gpio = TEGRA_GPIO_PN2,
 	.gpio_config = synaptics_gpio_setup,
	.cap_button_map = &TM_SAMPLE1_cap_button_map,
	#ifdef TINNO_TP_2_CAM
	.regulator_en = 1,
	#else
	.regulator_en = 0,
	#endif  
};

static struct i2c_board_info bus_i2c_devices[] = {
 	{
 		I2C_BOARD_INFO("synaptics_dsx_i2c", TM_SAMPLE1_ADDR),
 		.platform_data = &dsx_platformdata,
     	},	
};

int __init touch_init_synaptics_i2c(void)
{
	int ret;
	if (ARRAY_SIZE(bus_i2c_devices)) {
#if (CONFIG_S8515_PR_VERSION == 2)
	ret = i2c_register_board_info(1, bus_i2c_devices,ARRAY_SIZE(bus_i2c_devices));	
#else
	  #ifdef TINNO_TP_2_CAM
			pr_info("Magnum >>>>I2C device setup, i2c adapter == 2");
			ret = i2c_register_board_info(2, bus_i2c_devices,ARRAY_SIZE(bus_i2c_devices));
		#else
			pr_info("Magnum >>>>I2C device setup, i2c adapter == 0");
			ret = i2c_register_board_info(0, bus_i2c_devices,ARRAY_SIZE(bus_i2c_devices));
		#endif
#endif
	}
	return ret;
}



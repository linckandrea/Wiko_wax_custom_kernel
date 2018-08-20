/*
 * arch/arm/mach-tegra/board-s9321-kbc.c
 * Keys configuration for Nvidia tegra3 ceres platform.
 *
 * Copyright (C) 2013 NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/io.h>
#include <linux/io.h>
#include <mach/iomap.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/max77660/max77660-core.h>
#include <linux/mfd/palmas.h>

#include "tegra-board-id.h"
#include "board.h"
#include "board-s9321.h"
#include "board-atlantis.h"
#include "devices.h"
#include "wakeups-t14x.h"

#define CERES_POWER_ON_INT (MAX77660_IRQ_BASE + MAX77660_IRQ_GLBL_EN0_F)
#define CERES_POWER_LONGPRESS_INT (MAX77660_IRQ_BASE + MAX77660_IRQ_GLBL_EN0_1SEC)

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

#define GPIO_IKEY(_id, _irq, _iswake, _deb)	\
	{					\
		.code = _id,			\
		.gpio = -1,			\
		.irq = _irq,			\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = _deb,	\
	}

#define GPIO_HKEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 0,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

#if 0 //ljs s9321
static struct gpio_keys_button ceres_int_keys[] = {
	[0] = GPIO_KEY(KEY_HOME, PJ1, 0),
	[1] = GPIO_KEY(KEY_MENU, PJ2, 0),
	[2] = GPIO_HKEY(KEY_BACK, PJ3, 0),
	[3] = GPIO_IKEY(KEY_POWER, CERES_POWER_ON_INT, 1, 100),
	[4] = GPIO_KEY(KEY_VOLUMEUP, PJ6, 1),
	[5] = GPIO_KEY(KEY_VOLUMEDOWN, PJ5, 1),
	[6] = GPIO_IKEY(KEY_POWER, CERES_POWER_LONGPRESS_INT, 0, 1500),
};
#else
static struct gpio_keys_button ceres_int_keys[] = {
	[0] = GPIO_IKEY(KEY_POWER, CERES_POWER_ON_INT, 1, 100),
	[1] = GPIO_KEY(KEY_VOLUMEUP, PJ6, 1),
	[2] = GPIO_KEY(KEY_VOLUMEDOWN, PJ5, 1),
	[3] = GPIO_IKEY(KEY_POWER, CERES_POWER_LONGPRESS_INT, 0, 1500),
//Ivan	[4] = GPIO_KEY(KEY_WAKEUP, PJ1, 0),		//Low Battery Alert
};
#endif
static int atlantis_wakeup_key(void)
{
	int wakeup_key;
	u64 status = readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS)
		| (u64)readl(IO_ADDRESS(TEGRA_PMC_BASE)
		+ PMC_WAKE2_STATUS) << 32;

	if (status & ((u64)1 << TEGRA_WAKE_GPIO_PJ4))
		wakeup_key = KEY_POWER;
	else
		wakeup_key = KEY_RESERVED;

	return wakeup_key;
}

static struct gpio_keys_platform_data ceres_int_keys_pdata = {
	.buttons	= ceres_int_keys,
	.nbuttons	= ARRAY_SIZE(ceres_int_keys),
};

static struct platform_device ceres_int_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data  = &ceres_int_keys_pdata,
	},
};

int __init ceres_keys_init(void)
{
	struct board_info bi;

	tegra_get_board_info(&bi);

	if ((bi.board_id == BOARD_E1670) || (bi.board_id == BOARD_E1680) ||
		 (bi.board_id == BOARD_E1671) || (bi.board_id == BOARD_E1740)) {
#if 0 //ljs s9321
		ceres_int_keys[3].gpio = TEGRA_GPIO_PJ4;
		ceres_int_keys[3].active_low = 1;
		ceres_int_keys[3].debounce_interval = 30;
		ceres_int_keys_pdata.wakeup_key	= atlantis_wakeup_key;
#else
		ceres_int_keys[0].gpio = TEGRA_GPIO_PJ4;
		ceres_int_keys[0].active_low = 1;
		ceres_int_keys[0].debounce_interval = 30;
		ceres_int_keys_pdata.wakeup_key	= atlantis_wakeup_key;
#endif
	}

	platform_device_register(&ceres_int_keys_device);

	return 0;
}


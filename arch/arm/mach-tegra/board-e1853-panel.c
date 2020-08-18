/*
 * arch/arm/mach-tegra/board-e1853-panel.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <linux/i2c/ds90uh925q_ser.h>
#include <linux/of.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/board_id.h>

#include "board.h"
#include "devices.h"
#include "tegra3_host1x_devices.h"
#include "gpio-names.h"
#include "board-e1853.h"

#define E1853_HDMI_HPD TEGRA_GPIO_PB2

/* dc related */
static int e1853_lvds_enable(struct device *dev)
{
	return 0;
}

static int e1853_lvds_disable(void)
{
	return 0;
}

static struct ds90uh925q_platform_data lvds_ser_platform = {
	.has_lvds_en_gpio = false,
	.is_fpdlinkII  = true,
	.support_hdcp  = false,
	.clk_rise_edge = true,
};

static struct i2c_board_info __initdata lvds_ser_info[] = {
	{
		I2C_BOARD_INFO("ds90uh925q", 0xd),
		.platform_data = &lvds_ser_platform,
	}
};

static struct tegra_dc_mode e1853_CLAA101WB03_panel_modes[] = {
	{
		/* 1366x768@60Hz */
		.pclk = 74180000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 30,
		.v_sync_width = 5,
		.h_back_porch = 52,
		.v_back_porch = 20,
		.h_active = 1366,
		.v_active = 768,
		.h_front_porch = 64,
		.v_front_porch = 25,
	},
};

static struct tegra_fb_data e1853_CLAA101WB03_fb_data = {
	.win        = 0,
	.xres       = 1366,
	.yres       = 768,
	.bits_per_pixel = 32,
};

static struct tegra_dc_mode e1853_panel_modes[] = {
	{
		/* 800x480@60 */
		.pclk = 32460000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 64,
		.v_sync_width = 3,
		.h_back_porch = 128,
		.v_back_porch = 22,
		.h_front_porch = 64,
		.v_front_porch = 20,
		.h_active = 800,
		.v_active = 480,
	},
};

static struct tegra_fb_data e1853_fb_data = {
	.win		= 0,
	.xres		= 800,
	.yres		= 480,
	.bits_per_pixel	= 32,
};

static struct tegra_dc_out_pin e1853_dc_out_pins[] = {
	{
		.name	= TEGRA_DC_OUT_PIN_H_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_V_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name   = TEGRA_DC_OUT_PIN_DATA_ENABLE,
		.pol    = TEGRA_DC_OUT_PIN_POL_HIGH,
	},
};

static struct tegra_dc_out e1853_ser_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.parent_clk	= "pll_d_out0",
	.type		= TEGRA_DC_OUT_RGB,
	.modes		= e1853_panel_modes,
	.n_modes	= ARRAY_SIZE(e1853_panel_modes),
	.enable		= e1853_lvds_enable,
	.disable	= e1853_lvds_disable,
	.out_pins	= e1853_dc_out_pins,
	.n_out_pins	= ARRAY_SIZE(e1853_dc_out_pins),
};

static struct tegra_dc_platform_data e1853_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &e1853_ser_out,
	.emc_clk_rate	= 300000000,
	.fb		= &e1853_fb_data,
};

static int e1853_hdmi_enable(struct device *dev)
{
	return 0;
}

static int e1853_hdmi_disable(void)
{
	return 0;
}

static struct tegra_fb_data e1853_hdmi_fb_data = {
	.win            = 0,
	.xres           = 800,
	.yres           = 480,
	.bits_per_pixel = 32,
	.flags          = TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out e1853_hdmi_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.parent_clk     = "pll_d2_out0",
	.type		= TEGRA_DC_OUT_HDMI,
	.flags          = TEGRA_DC_OUT_HOTPLUG_LOW |
			  TEGRA_DC_OUT_NVHDCP_POLICY_ON_DEMAND,
	.max_pixclock   = KHZ2PICOS(148500),
	.hotplug_gpio   = E1853_HDMI_HPD,
	.enable		= e1853_hdmi_enable,
	.disable	= e1853_hdmi_disable,
	.ddc_bus        = 3,
};

static struct tegra_dc_platform_data e1853_hdmi_pdata = {
	.flags           = 0,
	.default_out     = &e1853_hdmi_out,
	.emc_clk_rate    = 300000000,
	.fb              = &e1853_hdmi_fb_data,
};

#if defined(CONFIG_TEGRA_NVMAP)
static struct nvmap_platform_carveout e1853_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
		.size		= TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
		.buddy_size	= 0, /* no buddy allocation for IRAM */
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by e1853_panel_init() */
		.size		= 0,	/* Filled in by e1853_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data e1853_nvmap_data = {
	.carveouts	= e1853_carveouts,
	.nr_carveouts	= ARRAY_SIZE(e1853_carveouts),
};

static struct platform_device e1853_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &e1853_nvmap_data,
	},
};
#endif

static struct platform_device *e1853_gfx_devices[] __initdata = {
#if defined(CONFIG_TEGRA_NVMAP)
	&e1853_nvmap_device,
#endif
};

static void e1853_config_CLAA101WB03_lcd(void)
{
	e1853_disp1_pdata.default_out->modes =
		e1853_CLAA101WB03_panel_modes;
	e1853_disp1_pdata.default_out->n_modes =
		ARRAY_SIZE(e1853_CLAA101WB03_panel_modes);
	e1853_disp1_pdata.fb = &e1853_CLAA101WB03_fb_data;
}

int __init e1853_panel_init(void)
{
	bool has_ebb = false;
	int err;
#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	struct resource *res;
	struct platform_device *phost1x = NULL;
#endif
	bool is_dt = of_have_populated_dt();

	if (tegra_is_board(NULL, "61861", NULL, NULL, NULL)) {
		has_ebb = true;
		if (tegra_is_board(NULL, "61227", NULL, NULL, NULL)) {
			e1853_config_CLAA101WB03_lcd();
			e1853_touch_init();
		}
	}

	e1853_carveouts[1].base = tegra_carveout_start;
	e1853_carveouts[1].size = tegra_carveout_size;
	tegra_disp1_device.dev.platform_data = &e1853_disp1_pdata;
	tegra_disp2_device.dev.platform_data = &e1853_hdmi_pdata;

	err = platform_add_devices(e1853_gfx_devices,
		ARRAY_SIZE(e1853_gfx_devices));

#ifdef CONFIG_TEGRA_GRHOST
	if (!is_dt)
		phost1x = tegra3_register_host1x_devices();
	else
		phost1x = to_platform_device(bus_find_device_by_name(
			&platform_bus_type, NULL, "host1x"));
	if (!phost1x) {
		pr_err("host1x devices registration failed\n");
		return -EINVAL;
	}
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = platform_get_resource_byname(&tegra_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	if (res) {
		res->start = tegra_fb_start;
		res->end = tegra_fb_start + tegra_fb_size - 1;
	}

	/*
	 * If the bootloader fb is valid, copy it to the fb, or else
	 * clear fb to avoid garbage on dispaly1.
	 */
	if (tegra_bootloader_fb_size)
		__tegra_move_framebuffer(&e1853_nvmap_device,
				tegra_fb_start, tegra_bootloader_fb_start,
				min(tegra_fb_size, tegra_bootloader_fb_size));
	else
		__tegra_clear_framebuffer(&e1853_nvmap_device,
					  tegra_fb_start, tegra_fb_size);

	if (!err) {
		tegra_disp1_device.dev.parent = &phost1x->dev;
		err = platform_device_register(&tegra_disp1_device);
	}

	res = platform_get_resource_byname(&tegra_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	if (res) {
		res->start = tegra_fb2_start;
		res->end = tegra_fb2_start + tegra_fb2_size - 1;
	}

	/*
	 * If the bootloader fb2 is valid, copy it to the fb2, or else
	 * clear fb2 to avoid garbage on dispaly2.
	 */
	if (tegra_bootloader_fb2_size)
		__tegra_move_framebuffer(&e1853_nvmap_device,
			tegra_fb2_start, tegra_bootloader_fb2_start,
			min(tegra_fb2_size, tegra_bootloader_fb2_size));
	else
		__tegra_clear_framebuffer(&e1853_nvmap_device,
					  tegra_fb2_start, tegra_fb2_size);

	if (!err) {
		tegra_disp2_device.dev.parent = &phost1x->dev;
		err = platform_device_register(&tegra_disp2_device);
	}
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err) {
		nvavp_device.dev.parent = &phost1x->dev;
		err = platform_device_register(&nvavp_device);
	}
#endif

	if (has_ebb) {
		if (!err)
			i2c_register_board_info(1, lvds_ser_info, 1);
	}

	return err;
}

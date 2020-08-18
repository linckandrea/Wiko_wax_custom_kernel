/*
 * arch/arm/mach-tegra/board-dolak-panel.c
 *
 * Copyright (c) 2011-2013, NVIDIA Corporation.
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <linux/of.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra14_host1x_devices.h"

#define PANEL_ENABLE	1

#if PANEL_ENABLE

#define DSI_PANEL_218	1

#define DSI_PANEL_RESET	1
#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE

static atomic_t __maybe_unused sd_brightness = ATOMIC_INIT(255);

static int dolak_backlight_init(struct device *dev)
{
#if DSI_PANEL_218
	/* TODO: Enable backlight for dsi panel */
#endif
	return -ENODEV;
}

static void dolak_backlight_exit(struct device *dev)
{
#if DSI_PANEL_218
	/* TODO: Exit backlight for dsi panel */
#endif
}

static int dolak_backlight_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");

	/* TODO: add bl_out table here if needed */

	return brightness;
}

static struct platform_pwm_backlight_data dolak_backlight_data = {
	.pwm_id		= 2,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 5000000,
	.init		= dolak_backlight_init,
	.exit		= dolak_backlight_exit,
	.notify		= dolak_backlight_notify,
};

static struct platform_device dolak_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &dolak_backlight_data,
	},
};

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
static int dolak_panel_enable(struct device *dev)
{
#if DSI_PANEL_218
	/* TODO: DSI panel enable */
#endif
	return -ENODEV;
}

static int dolak_panel_disable(void)
{
#if DSI_PANEL_218
	/* TODO: DSI panel disable */
#endif
	return -ENODEV;
}

static struct resource dolak_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by dolak_panel_init() */
		.end	= 0,	/* Filled in by dolak_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "mipi_cal",
		.start  = TEGRA_MIPI_CAL_BASE,
		.end    = TEGRA_MIPI_CAL_BASE + TEGRA_MIPI_CAL_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode dolak_panel_modes[] = {
#if defined(CONFIG_TEGRA_SIMULATION_PLATFORM)
	{
		.pclk = 18000000,
		.h_ref_to_sync = 11,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 4,
		.h_back_porch = 16,
		.v_back_porch = 4,
		.h_active = 240,
		.v_active = 320,
		.h_front_porch = 16,
		.v_front_porch = 4,
	},
#else
	{
		.pclk = 323000000,
		.h_ref_to_sync = 11,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 4,
		.h_back_porch = 16,
		.v_back_porch = 4,
		.h_active = 864,
		.v_active = 480,
		.h_front_porch = 16,
		.v_front_porch = 4,
	},
#endif
};

static struct tegra_fb_data dolak_fb_data = {
	.win		= 0,
#if defined(CONFIG_TEGRA_SIMULATION_PLATFORM)
	.xres		= 240,
	.yres		= 320,
	.bits_per_pixel = 16,
	.flags		= 0,
#else
	.xres		= 864,
	.yres		= 480,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
#endif
};

static struct tegra_dsi_cmd dsi_init_cmd[] = {
#if DSI_PANEL_218
	DSI_CMD_SHORT(0x05, 0x11, 0x00),
	DSI_DLY_MS(20),
#if (DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(0x15, 0x35, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(20),
#endif
};

static struct tegra_dsi_cmd dsi_early_suspend_cmd[] = {
#if DSI_PANEL_218
	DSI_CMD_SHORT(0x05, 0x28, 0x00),
	DSI_DLY_MS(20),
#if (DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(0x05, 0x34, 0x00),
#endif
#endif
};

static struct tegra_dsi_cmd dsi_late_resume_cmd[] = {
#if DSI_PANEL_218
#if (DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(0x15, 0x35, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(20),
#endif
};

static struct tegra_dsi_cmd dsi_suspend_cmd[] = {
#if DSI_PANEL_218
	DSI_CMD_SHORT(0x05, 0x28, 0x00),
	DSI_DLY_MS(20),
#if (DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(0x05, 0x34, 0x00),
#endif
	DSI_CMD_SHORT(0x05, 0x10, 0x00),
	DSI_DLY_MS(5),
#endif
};

static struct tegra_dsi_out dolak_dsi = {
#if DSI_PANEL_218
	.n_data_lanes = 2,
#else
	.n_data_lanes = 4,
#endif
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.refresh_rate = 60,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.dsi_instance = 0,
	.controller_vs = DSI_VS_1,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,

	.n_init_cmd = ARRAY_SIZE(dsi_init_cmd),
	.dsi_init_cmd = dsi_init_cmd,

	.n_early_suspend_cmd = ARRAY_SIZE(dsi_early_suspend_cmd),
	.dsi_early_suspend_cmd = dsi_early_suspend_cmd,

	.n_late_resume_cmd = ARRAY_SIZE(dsi_late_resume_cmd),
	.dsi_late_resume_cmd = dsi_late_resume_cmd,

	.n_suspend_cmd = ARRAY_SIZE(dsi_suspend_cmd),
	.dsi_suspend_cmd = dsi_suspend_cmd,

	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
	.lp_cmd_mode_freq_khz = 20000,

	/* TODO: Get the vender recommended freq */
	.lp_read_cmd_mode_freq_khz = 200000,
};

#if !defined(CONFIG_TEGRA_SIMULATION_PLATFORM)
static struct tegra_dc_sd_settings dolak_sd_settings = {
	.enable = 0, /* disabled by default */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = 4,
	.aggressiveness = 1,
	.use_vid_luma = false,
	.phase_in_adjustments = false,
	.k_limit_enable = false,
	/* Aggressive k_limit */
	.k_limit = 180,
	.sd_window_enable = false,
	.soft_clipping_enable = false,
	/* Low soft clipping threshold to compensate for aggressive k_limit */
	.soft_clipping_threshold = 128,
	.smooth_k_enable = false,
	.smooth_k_incr = 64,
	/* Default video coefficients */
	.coeff = {5, 9, 2},
	.fc = {0, 0},
	/* Immediate backlight changes */
	.blp = {1024, 255},
	/* Gammas: R: 2.2 G: 2.2 B: 2.2 */
	/* Default BL TF */
	.bltf = {
			{
				{57, 65, 73, 82},
				{92, 103, 114, 125},
				{138, 150, 164, 178},
				{193, 208, 224, 241},
			},
		},
	/* Default LUT */
	.lut = {
			{
				{255, 255, 255},
				{199, 199, 199},
				{153, 153, 153},
				{116, 116, 116},
				{85, 85, 85},
				{59, 59, 59},
				{36, 36, 36},
				{17, 17, 17},
				{0, 0, 0},
			},
		},
	.sd_brightness = &sd_brightness,
	.bl_device = (struct backlight_device *)&dolak_backlight_device,
};
#endif

static struct tegra_dc_out dolak_disp1_out = {
#if defined(CONFIG_TEGRA_SIMULATION_PLATFORM)
	.type		= TEGRA_DC_OUT_RGB,
#else
	.type		= TEGRA_DC_OUT_DSI,
	.sd_settings	= &dolak_sd_settings,
#endif
	.dsi		= &dolak_dsi,

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.flags		= DC_CTRL_MODE,

	.modes		= dolak_panel_modes,
	.n_modes	= ARRAY_SIZE(dolak_panel_modes),

	.enable		= dolak_panel_enable,
	.disable	= dolak_panel_disable,
};

static struct tegra_dc_platform_data dolak_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &dolak_disp1_out,
	.fb		= &dolak_fb_data,
};

static struct platform_device dolak_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= dolak_disp1_resources,
	.num_resources	= ARRAY_SIZE(dolak_disp1_resources),
	.dev = {
		.platform_data = &dolak_disp1_pdata,
	},
};
#endif

static struct nvmap_platform_carveout dolak_carveouts[] = {
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
		.base		= 0,	/* Filled in by dolak_panel_init() */
		.size		= 0,	/* Filled in by dolak_panel_init() */
		.buddy_size	= SZ_32K,
	},
	[2] = {
		.name		= "vpr",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_VPR,
		.base		= 0,	/* Filled in by dolak_panel_init() */
		.size		= 0,	/* Filled in by dolak_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data dolak_nvmap_data = {
	.carveouts	= dolak_carveouts,
	.nr_carveouts	= ARRAY_SIZE(dolak_carveouts),
};

static struct platform_device dolak_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &dolak_nvmap_data,
	},
};

static struct platform_device *dolak_gfx_devices[] __initdata = {
	&dolak_nvmap_device,
	&tegra_pwfm2_device,
	&dolak_backlight_device,
};

int __init dolak_panel_init(void)
{
	int err;
#if defined(CONFIG_TEGRA_GRHOST)
#if defined(CONFIG_TEGRA_DC)
	struct resource *res;
#endif
	struct platform_device *phost1x = NULL;
#endif
	bool is_dt = of_have_populated_dt();

	dolak_carveouts[1].base = tegra_carveout_start;
	dolak_carveouts[1].size = tegra_carveout_size;
	dolak_carveouts[2].base = tegra_vpr_start;
	dolak_carveouts[2].size = tegra_vpr_size;

	err = platform_add_devices(dolak_gfx_devices,
				   ARRAY_SIZE(dolak_gfx_devices));

#ifdef CONFIG_TEGRA_GRHOST
	if (!is_dt)
		phost1x = tegra14_register_host1x_devices();
	else
		phost1x = to_platform_device(bus_find_device_by_name(
			&platform_bus_type, NULL, "host1x"));
	if (!phost1x) {
		pr_err("host1x devices registration failed\n");
		return -EINVAL;
	}
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = platform_get_resource_byname(&dolak_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	if (!err) {
		dolak_disp1_device.dev.parent = &phost1x->dev;
		err = platform_device_register(&dolak_disp1_device);
	}
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err) {
		nvavp_device.dev.parent = &phost1x->dev;
		err = platform_device_register(&nvavp_device);
	}
#endif
	return err;
}
#else
int __init dolak_panel_init(void)
{
	return -ENODEV;
}
#endif

/*
 * arch/arm/mach-tegra/board-s8515-panel.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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
 */

#include <linux/ioport.h>
#include <linux/fb.h>
#include <linux/nvmap.h>
#include <linux/nvhost.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/mfd/max77660/max77660-core.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>

#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "board-s8515.h"
#include "board-panel.h"
#include "common.h"
#include "tegra-board-id.h"
#include "board-atlantis.h"

#include "tegra14_host1x_devices.h"

#define DSI_PANEL_RST_GPIO	TEGRA_GPIO_PG4
#define DSI_PANEL_BL_EN_GPIO	TEGRA_GPIO_PG2	//TEGRA_GPIO_PG2 -> PWM
#define DSI_PANEL_BL_PWM_GPIO	TEGRA_GPIO_PG3		//TEGRA_GPIO_PG3
#define TE_GPIO			TEGRA_GPIO_PG1

struct platform_device * __init ceres_host1x_init(void)
{
	struct platform_device *pdev = NULL;
#ifdef CONFIG_TEGRA_GRHOST
	pdev = tegra14_register_host1x_devices();
	if (!pdev) {
		pr_err("host1x devices registration failed\n");
		return NULL;
	}
#endif
	return pdev;
}

#ifdef CONFIG_TEGRA_DC

/* hdmi pins for hotplug */
#define ceres_hdmi_hpd		(MAX77660_GPIO_BASE + MAX77660_GPIO2)

/* hdmi related regulators */
static struct regulator *ceres_hdmi_reg;
static struct regulator *ceres_hdmi_pll;
static struct regulator *ceres_hdmi_vddio;
static struct regulator *ceres_hdmi_1v8;
char *panel_name = "unnknow";
static struct resource ceres_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0, /* Filled in by ceres_panel_init() */
		.end	= 0, /* Filled in by ceres_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dsi_regs",
		.start	= 0, /* Filled in by tegra_dsi_resources_init() */
		.end	= 0, /* Filled in by tegra_dsi_resources_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "mipi_cal",
		.start	= TEGRA_MIPI_CAL_BASE,
		.end	= TEGRA_MIPI_CAL_BASE + TEGRA_MIPI_CAL_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource ceres_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0, /* Filled in by ceres_panel_init() */
		.end	= 0, /* Filled in by ceres_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_sd_settings sd_settings;

static struct tegra_dc_out ceres_disp1_out = {
	.type		= TEGRA_DC_OUT_DSI,
	.sd_settings	= &sd_settings,
	.refresh_rate   = 60000,	/* 60Hz */
};

static int ceres_hdmi_enable(struct device *dev)
{
	int ret;

	if (!ceres_hdmi_reg) {
			ceres_hdmi_reg = regulator_get(dev, "avdd_hdmi");
			if (IS_ERR_OR_NULL(ceres_hdmi_reg)) {
				pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
				ceres_hdmi_reg = NULL;
				return PTR_ERR(ceres_hdmi_reg);
			}
	}
	ret = regulator_enable(ceres_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!ceres_hdmi_pll) {
		ceres_hdmi_pll = regulator_get(dev, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(ceres_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			ceres_hdmi_pll = NULL;
			regulator_put(ceres_hdmi_reg);
			ceres_hdmi_reg = NULL;
			return PTR_ERR(ceres_hdmi_pll);
		}
	}
	ret = regulator_enable(ceres_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}

	if (!ceres_hdmi_1v8) {
		ceres_hdmi_1v8 = regulator_get(dev, "vdd_1v8_hdmi");
		if (IS_ERR_OR_NULL(ceres_hdmi_1v8)) {
			pr_err("hdmi: couldn't get regulator vdd_1v8_hdmi\n");
			ceres_hdmi_1v8 = NULL;
			regulator_put(ceres_hdmi_pll);
			ceres_hdmi_pll = NULL;
			regulator_put(ceres_hdmi_reg);
			ceres_hdmi_reg = NULL;
			return PTR_ERR(ceres_hdmi_1v8);
		}
	}
	ret = regulator_enable(ceres_hdmi_1v8);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator vdd_1v8_hdmi\n");
		return ret;
	}

	return 0;
}

static int ceres_hdmi_disable(void)
{
	if (ceres_hdmi_reg) {
		regulator_disable(ceres_hdmi_reg);
		regulator_put(ceres_hdmi_reg);
		ceres_hdmi_reg = NULL;
	}

	if (ceres_hdmi_pll) {
		regulator_disable(ceres_hdmi_pll);
		regulator_put(ceres_hdmi_pll);
		ceres_hdmi_pll = NULL;
	}

	if (ceres_hdmi_1v8) {
		regulator_disable(ceres_hdmi_1v8);
		regulator_put(ceres_hdmi_1v8);
		ceres_hdmi_1v8 = NULL;
	}

	return 0;
}

static int ceres_hdmi_postsuspend(void)
{
	if (ceres_hdmi_vddio) {
		regulator_disable(ceres_hdmi_vddio);
		regulator_put(ceres_hdmi_vddio);
		ceres_hdmi_vddio = NULL;
	}
	return 0;
}

static int ceres_hdmi_hotplug_init(struct device *dev)
{
	int ret = 0;
	if (!ceres_hdmi_vddio) {
		ceres_hdmi_vddio = regulator_get(dev, "vdd_hdmi_5v0");
		if (IS_ERR_OR_NULL(ceres_hdmi_vddio)) {
			ret = PTR_ERR(ceres_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator vdd_hdmi_5v0\n");
			ceres_hdmi_vddio = NULL;
			return ret;
		}
	}
	ret = regulator_enable(ceres_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator vdd_hdmi_5v0\n");
		regulator_put(ceres_hdmi_vddio);
		ceres_hdmi_vddio = NULL;
		return ret;
	}
	return ret;
}

static struct tegra_dc_out ceres_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,
	.parent_clk	= "pll_d2_out0",

	.ddc_bus	= 3,
	.hotplug_gpio	= ceres_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.enable		= ceres_hdmi_enable,
	.disable	= ceres_hdmi_disable,
	.postsuspend	= ceres_hdmi_postsuspend,
	.hotplug_init	= ceres_hdmi_hotplug_init,
};

static struct tegra_fb_data ceres_disp1_fb_data = {
	.win		= 0,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data ceres_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &ceres_disp1_out,
	.fb		= &ceres_disp1_fb_data,
	.emc_clk_rate	= 204000000,
#ifdef CONFIG_TEGRA_DC_CMU
	.cmu_enable	= 1,
#endif
};

static struct tegra_fb_data ceres_disp2_fb_data = {
	.win		= 0,
	.xres		= 1024,
	.yres		= 600,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data ceres_disp2_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &ceres_disp2_out,
	.fb		= &ceres_disp2_fb_data,
	.emc_clk_rate	= 300000000,
#ifdef CONFIG_TEGRA_DC_CMU
	.cmu_enable	= 1,
#endif
};

static struct platform_device ceres_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= ceres_disp2_resources,
	.num_resources	= ARRAY_SIZE(ceres_disp2_resources),
	.dev = {
		.platform_data = &ceres_disp2_pdata,
	},
};

static struct platform_device ceres_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= ceres_disp1_resources,
	.num_resources	= ARRAY_SIZE(ceres_disp1_resources),
	.dev = {
		.platform_data = &ceres_disp1_pdata,
	},
};

static struct nvmap_platform_carveout ceres_carveouts[] = {
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
		.base		= 0, /* Filled in by ceres_panel_init() */
		.size		= 0, /* Filled in by ceres_panel_init() */
		.buddy_size	= SZ_32K,
	},
	[2] = {
		.name		= "vpr",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_VPR,
		.base		= 0, /* Filled in by ceres_panel_init() */
		.size		= 0, /* Filled in by ceres_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data ceres_nvmap_data = {
	.carveouts	= ceres_carveouts,
	.nr_carveouts	= ARRAY_SIZE(ceres_carveouts),
};

static struct platform_device ceres_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &ceres_nvmap_data,
	},
};

static struct tegra_dc_sd_settings ceres_sd_settings = {
	.enable = 1, /* enabled by default */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 5,
	.use_vid_luma = false,
	.phase_in_adjustments = 0,
	.k_limit_enable = true,
	/* Aggressive k_limit */
	.k_limit = 180,
	.sd_window_enable = false,
	.soft_clipping_enable = true,
	/* Low soft clipping threshold to compensate for aggressive k_limit */
	.soft_clipping_threshold = 128,
	.smooth_k_enable = true,
	.smooth_k_incr = 4,
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
//	.use_vpulse2 = true,
	.bl_device_name = "pwm-backlight",	
};

char *get_panel_name(void){
        return panel_name;
}

void tegra_fb_data_get(struct tegra_fb_data **fb_data)
{
	*fb_data = &ceres_disp1_fb_data;
}
EXPORT_SYMBOL(tegra_fb_data_get);

static void ceres_panel_select(void)
{
	struct tegra_panel *panel;
	u8 dsi_instance = 0;

//Ivan
    //panel = &dsi_otm1283a_720p;

	if (tegra_get_board_panel_id()==11) {  //LIUJ201140504RELE1315ADDO adc select lcd
		panel = &dsi_hx8394a_720p;
                panel_name = "Tcl_hx8394_p3_HD_video_24bit\n";
	}else if(tegra_get_board_panel_id()==12) {

                panel = &dsi_otm1283a_otp_720p;
                panel_name = "Truly_otm1283a_otp_HD_video_24bit\n";
        }else {
		panel = &dsi_otm1283a_720p;
                panel_name = "Truly_otm1283a_HD_video_24bit\n";
	}
	dsi_instance = DSI_INSTANCE_0;	


	if (panel->init_sd_settings)
		panel->init_sd_settings(&sd_settings);

	if (panel->init_dc_out) {
		panel->init_dc_out(&ceres_disp1_out);
		ceres_disp1_out.dsi->dsi_instance = dsi_instance;
		ceres_disp1_out.dsi->dsi_panel_rst_gpio = DSI_PANEL_RST_GPIO;
		ceres_disp1_out.dsi->dsi_panel_bl_en_gpio =
			DSI_PANEL_BL_EN_GPIO;
		ceres_disp1_out.dsi->dsi_panel_bl_pwm_gpio =
			DSI_PANEL_BL_PWM_GPIO;
		ceres_disp1_out.dsi->te_gpio = TE_GPIO;
		/* update the init cmd if dependent on reset GPIO */
		tegra_dsi_update_init_cmd_gpio_rst(&ceres_disp1_out);
	}

	if (panel->init_fb_data)
		panel->init_fb_data(&ceres_disp1_fb_data);

	if (panel->init_cmu_data)
		panel->init_cmu_data(&ceres_disp1_pdata);

	if (panel->set_disp_device)
		panel->set_disp_device(&ceres_disp1_device);

	tegra_dsi_resources_init(dsi_instance, ceres_disp1_resources,
		ARRAY_SIZE(ceres_disp1_resources));

	if (panel->register_bl_dev)
		panel->register_bl_dev();

}

void ceres_set_hotplug_gpio(void)
{
	struct tegra_dc_platform_data *pdata;
	struct board_info board_info;
	int hdmi_hpd_gpio = ceres_hdmi_hpd;

	pdata = ceres_disp2_device.dev.platform_data;
	tegra_get_board_info(&board_info);

	if ((board_info.board_id == BOARD_E1670) ||
		 (board_info.board_id == BOARD_E1740)) {
		hdmi_hpd_gpio = (PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO14);
	} else if ((board_info.fab > BOARD_FAB_A00) ||
		(board_info.board_id == BOARD_E1690) ||
		(board_info.board_id == BOARD_E1683)) {
			hdmi_hpd_gpio = (MAX77660_GPIO_BASE + MAX77660_GPIO1);
	}

	pdata->default_out->hotplug_gpio = hdmi_hpd_gpio;
}

int __init ceres_panel_init(void)
{
	int err = 0;
	struct resource __maybe_unused *res;
	struct platform_device *phost1x;

	sd_settings = ceres_sd_settings;

	ceres_panel_select();

#ifdef CONFIG_TEGRA_NVMAP
	ceres_carveouts[1].base = tegra_carveout_start;
	ceres_carveouts[1].size = tegra_carveout_size;
	ceres_carveouts[2].base = tegra_vpr_start;
	ceres_carveouts[2].size = tegra_vpr_size;

	err = platform_device_register(&ceres_nvmap_device);
	if (err) {
		pr_err("nvmap device registration failed\n");
		return err;
	}
#endif

	phost1x = ceres_host1x_init();
	if (!phost1x) {
		pr_err("host1x devices registration failed\n");
		return -EINVAL;
	}

	res = platform_get_resource_byname(&ceres_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	/* Copy the bootloader fb to the fb. */
	__tegra_move_framebuffer(&ceres_nvmap_device,
		tegra_fb_start, tegra_bootloader_fb_start,
			min(tegra_fb_size, tegra_bootloader_fb_size));

	ceres_disp1_device.dev.parent = &phost1x->dev;
	err = platform_device_register(&ceres_disp1_device);
	if (err) {
		pr_err("disp1 device registration failed\n");
		return err;
	}

//Ivan removed
#if 0
	ceres_set_hotplug_gpio();

	err = tegra_init_hdmi(&ceres_disp2_device, phost1x);
	if (err)
		return err;
#endif

#ifdef CONFIG_TEGRA_NVAVP
	nvavp_device.dev.parent = &phost1x->dev;
	err = platform_device_register(&nvavp_device);
	if (err) {
		pr_err("nvavp device registration failed\n");
		return err;
	}
#endif
	return err;
}
#else
int __init ceres_panel_init(void)
{
	if (ceres_host1x_init())
		return 0;
	else
		return -EINVAL;
}
#endif

/*
 * arch/arm/mach-tegra/board-s9321-power.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/edp.h>
#include <linux/edpdev.h>
#include <linux/i2c.h>
#include "linux/iio/machine.h"
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max77660/max77660-core.h>
#include <linux/platform_data/lp8755.h>
#include <linux/pid_thermal_gov.h>
#include <linux/generic_adc_thermal.h>
#include <linux/power/power_supply_extcon.h>

#include <asm/mach-types.h>

#include <mach/iomap.h>
#include <mach/edp.h>
#include <mach/irqs.h>
#include <mach/gpio-tegra.h>
#include <mach/hardware.h>

#include "cpu-tegra.h"
#include "pm.h"
#include "board.h"
#include "board-common.h"
#include "board-s9321.h"
#include "tegra11_soctherm.h"
#include "tegra3_tsensor.h"
#include "tegra-board-id.h"
#include "tegra_cl_dvfs.h"
#include "devices.h"
#include "board-atlantis.h"

#define PMC_CTRL                0x0
#define PMC_CTRL_INTR_LOW       (1 << 17)

/* max77660 consumer rails */
static struct regulator_consumer_supply max77660_unused_supply[] = {
	REGULATOR_SUPPLY("unused_reg", NULL),
};

static struct regulator_consumer_supply max77660_buck1_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
	REGULATOR_SUPPLY("vdd_core", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("vdd_core_dbg", NULL),
	REGULATOR_SUPPLY("vdd_core", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("vdd_core", "sdhci-tegra.3"),
};

static struct regulator_consumer_supply max77660_buck2_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
	REGULATOR_SUPPLY("vdd_cpu_dbg", NULL),
};

static struct regulator_consumer_supply max77660_buck3_a02_supply[] = {
	REGULATOR_SUPPLY("vdd2_lpddr3", NULL),
	REGULATOR_SUPPLY("vddca_lpddr3", NULL),
	REGULATOR_SUPPLY("vrefca_lpddr3", NULL),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.1"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.2"),
	REGULATOR_SUPPLY("vddio_hsic_modem2", NULL),
};

static struct regulator_consumer_supply max77660_buck3_supply[] = {
	REGULATOR_SUPPLY("vdd2_lpddr3", NULL),
	REGULATOR_SUPPLY("vddca_lpddr3", NULL),
	REGULATOR_SUPPLY("vrefca_lpddr3", NULL),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.1"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.2"),
	REGULATOR_SUPPLY("vddio_hsic_modem2", NULL),
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddq_lpddr3", NULL),
	REGULATOR_SUPPLY("vrefdq_lpddr3", NULL),
};

static struct regulator_consumer_supply max77660_buck4_supply[] = {
	REGULATOR_SUPPLY("vdd_bb", NULL),
};

static struct regulator_consumer_supply max77660_buck5_supply[] = {
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.1"),
	REGULATOR_SUPPLY("vddio_cam", "vi"),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("vdd_1v8_pmic", NULL),
	REGULATOR_SUPPLY("vdd_1v8_cpu_reg", NULL),
	REGULATOR_SUPPLY("vdd_sys_mb", NULL),
	REGULATOR_SUPPLY("vdd_1v8_eeprom", NULL),
	REGULATOR_SUPPLY("vdd_1v8_hdmi", "tegradc.1"),
	REGULATOR_SUPPLY("vdd_1v8_com", NULL),
	REGULATOR_SUPPLY("vddio_sim_bb", NULL),
	REGULATOR_SUPPLY("dvdd", "1-0077"),
	REGULATOR_SUPPLY("vdd_dtv", NULL),
	REGULATOR_SUPPLY("vdd_modem2", NULL),
	REGULATOR_SUPPLY("vdd_dbg", NULL),
	REGULATOR_SUPPLY("vio_tp", "1-0039"),
};

static struct regulator_consumer_supply max77660_buck6_supply[] = {
	REGULATOR_SUPPLY("vdd_1v7_rf", "tegra_bbc_proxy"),
};

static struct regulator_consumer_supply max77660_buck7_supply[] = {
	REGULATOR_SUPPLY("vdd_2v65_rf", "tegra_bbc_proxy"),
};

static struct regulator_consumer_supply max77660_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply max77660_ldo2_supply[] = {
	REGULATOR_SUPPLY("avdd_2v8_cam_af", NULL),
	REGULATOR_SUPPLY("avdd_cam2", NULL),
	REGULATOR_SUPPLY("vana_imx132", "2-0036"),
	REGULATOR_SUPPLY("avdd", "2-003c"),
	REGULATOR_SUPPLY("vdd", "2-000e"),
	REGULATOR_SUPPLY("vdd", "2-000c"),
	REGULATOR_SUPPLY("avdd_cam_ldo2", NULL),
	REGULATOR_SUPPLY("vana", "2-0010"),
	//luis
	REGULATOR_SUPPLY("avdd_cam1_ldo3", "2-0010"),
};

static struct regulator_consumer_supply max77660_ldo3_supply[] = {
	//REGULATOR_SUPPLY("vdd_cam_1v2", NULL),
	REGULATOR_SUPPLY("vana", "2-0036"),
	REGULATOR_SUPPLY("vif", "2-0010"),
	//luis
	//REGULATOR_SUPPLY("avdd", "2-0036"),
	//luis
	REGULATOR_SUPPLY("dvdd", "2-0036"),
	REGULATOR_SUPPLY("vdd_cam_1v2", "2-0010"),
};

static struct regulator_consumer_supply max77660_ldo4_supply[] = {
	 REGULATOR_SUPPLY("avdd_lcd", NULL),
	 REGULATOR_SUPPLY("avdd", "spi2.0"),
	 REGULATOR_SUPPLY("vin", "2-004a"),
	 REGULATOR_SUPPLY("vdd_tp", "1-0039"),
};

static struct regulator_consumer_supply max77660_ldo4_display_config0_supply[] = {
	 REGULATOR_SUPPLY("avdd", "spi2.0"),
	 REGULATOR_SUPPLY("vin", "2-004a"),
};

static struct regulator_consumer_supply max77660_ldo5_supply[] = {
	//REGULATOR_SUPPLY("avdd_cam1_ldo3", NULL),
	REGULATOR_SUPPLY("avdd_aud", NULL),
	//luis
	REGULATOR_SUPPLY("avdd", "2-0036"),
};

static struct regulator_consumer_supply max77660_ldo6_supply[] = {
	//REGULATOR_SUPPLY("vdd_cam_1v2_old", NULL),
	//luis
	//REGULATOR_SUPPLY("dvdd", "2-0036"),
};

static struct regulator_consumer_supply max77660_ldo7_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_m_p", NULL),
	REGULATOR_SUPPLY("avdd_pllc", NULL),
	REGULATOR_SUPPLY("avdd_ddr_hs", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_mipi_pllu", NULL),

};

static struct regulator_consumer_supply max77660_ldo8_supply[] = {
	REGULATOR_SUPPLY("dvdd_bb_pll", NULL),
	REGULATOR_SUPPLY("avdd_bb_pll", NULL),
};

static struct regulator_consumer_supply max77660_ldo9_supply[] = {
	REGULATOR_SUPPLY("vdd_mb", NULL),
	REGULATOR_SUPPLY("vdd_temp", NULL),
	REGULATOR_SUPPLY("avdd", "1-0077"),
	REGULATOR_SUPPLY("vdd_irled", NULL),
	REGULATOR_SUPPLY("vdd_sensor_2v8", NULL),
	REGULATOR_SUPPLY("vdd_pm_2v8", NULL),
	REGULATOR_SUPPLY("vdd", "0-004c"),
	REGULATOR_SUPPLY("vdd", "0-0068"),
	REGULATOR_SUPPLY("vdd", "0-000e"),
	REGULATOR_SUPPLY("vdd", "0-0078"),
	REGULATOR_SUPPLY("vdd_prox", "0-001c"),	
};

static struct regulator_consumer_supply max77660_ldo10_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
	REGULATOR_SUPPLY("vpp_bb_fuse", "tegra_bbc_proxy"),
};

static struct regulator_consumer_supply max77660_ldo11_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avdd_hdmi", "tegradc.1"),
	REGULATOR_SUPPLY("vdd_sys_dtv_3v3", NULL),
};

static struct regulator_consumer_supply max77660_ldo12_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
};

static struct regulator_consumer_supply max77660_ldo13_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
};

static struct regulator_consumer_supply max77660_ldo14_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.3"),
};

static struct regulator_consumer_supply max77660_ldo15_supply[] = {
	REGULATOR_SUPPLY("vddio_sim0", "tegra_bbc_proxy"),
};

static struct regulator_consumer_supply max77660_ldo16_supply[] = {
	REGULATOR_SUPPLY("vddio_sim1", "tegra_bbc_proxy"),
};

static struct regulator_consumer_supply max77660_ldo17_supply[] = {
	REGULATOR_SUPPLY("avdd_1v8_rf", NULL),
};

static struct regulator_consumer_supply max77660_ldo18_supply[] = {
	REGULATOR_SUPPLY("avdd_2v7_rf", NULL),
};

static struct regulator_consumer_supply max77660_sw1_supply[] = {
	 REGULATOR_SUPPLY("vdd_dis_lcd", NULL),
	 REGULATOR_SUPPLY("vdd_dis_ts", NULL),
	 REGULATOR_SUPPLY("dvdd", "spi2.0"),
	 REGULATOR_SUPPLY("vdd_lcd_1v8_s", NULL),
};

static struct regulator_consumer_supply max77660_sw2_supply[] = {
	REGULATOR_SUPPLY("vdd_cam_1v8", "2-0010"),
	REGULATOR_SUPPLY("vdd_cam_1v8", "2-0036"),
	//luis
	REGULATOR_SUPPLY("dovdd", "2-0036"),
};

static struct regulator_consumer_supply max77660_sw3_supply[] = {
	REGULATOR_SUPPLY("vdd_aud_dgtl", NULL),
	REGULATOR_SUPPLY("vdd_aud_anlg", NULL),
	REGULATOR_SUPPLY("vdd_aud_mic", NULL),
	REGULATOR_SUPPLY("vdd", "0-0044"),
	REGULATOR_SUPPLY("vlogic", "0-0068"),
	REGULATOR_SUPPLY("vid", "0-000e"),
	REGULATOR_SUPPLY("vddio", "0-0078"),
	//edit by Magnum 2013-11-13
#ifndef TINNO_TP_2_CAM	    
	REGULATOR_SUPPLY("vio_tp", "2-0039"),	
#endif 
	REGULATOR_SUPPLY("vdd_sensor", "0-0019"),	
	REGULATOR_SUPPLY("vio_prox", "0-001c"),		
};

static struct regulator_consumer_supply max77660_sw4_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc_alt1", NULL),
};

static struct regulator_consumer_supply max77660_sw5_a02_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddq_lpddr3", NULL),
	REGULATOR_SUPPLY("vrefdq_lpddr3", NULL),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "vi"),
	REGULATOR_SUPPLY("vdd_1v2_lcd", NULL),
	REGULATOR_SUPPLY("vdd_1v2_cdc", NULL),
	REGULATOR_SUPPLY("vdd_prox", "0-0044"),
};

static struct regulator_consumer_supply max77660_sw5_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "vi"),
	REGULATOR_SUPPLY("vdd_1v2_lcd", NULL),
	REGULATOR_SUPPLY("vdd_1v2_cdc", NULL),
	REGULATOR_SUPPLY("vdd_prox", "0-0044"),
};

static struct max77660_regulator_fps_cfg max77660_fps_cfgs[] = {
};

#define MAX77660_PDATA_INIT(_rid, _id, _min_uV, _max_uV, _supply_reg,	\
		_always_on, _boot_on, _apply_uV,			\
		_fps_src, _fps_pu_period, _fps_pd_period, _flags)	\
	static struct regulator_init_data max77660_regulator_idata_##_id = {   \
		.supply_regulator = _supply_reg,			\
		.constraints = {					\
			.name = max77660_rails(_id),			\
			.min_uV = _min_uV*1000,				\
			.max_uV = _max_uV*1000,				\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					     REGULATOR_MODE_FAST |	\
					     REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					   REGULATOR_CHANGE_STATUS |	\
					   REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on || (_flags & DISABLE_DVFS), \
			.boot_on = _boot_on,				\
			.apply_uV = _apply_uV,				\
		},							\
		.num_consumer_supplies =				\
			ARRAY_SIZE(max77660_##_id##_supply),		\
		.consumer_supplies = max77660_##_id##_supply,		\
	};								\
static struct max77660_regulator_platform_data max77660_regulator_pdata_##_id =\
{									\
		.reg_init_data = &max77660_regulator_idata_##_id,	\
		.fps_src = _fps_src,					\
		.fps_pu_period = _fps_pu_period,			\
		.fps_pd_period = _fps_pd_period,			\
		.num_fps_cfgs = ARRAY_SIZE(max77660_fps_cfgs),		\
		.fps_cfgs = max77660_fps_cfgs,				\
		.flags = _flags,					\
	}


MAX77660_PDATA_INIT(BUCK1, buck1,  650, 1400, NULL,
		1, 1, 0, FPS_SRC_DEF, -1, -1, MAX77660_EXT_ENABLE_EN1);

MAX77660_PDATA_INIT(BUCK2, buck2,  650, 1300, NULL,
		1, 1, 0, FPS_SRC_DEF, 0, 0, MAX77660_EXT_ENABLE_EN2);

MAX77660_PDATA_INIT(BUCK3, buck3,  1200, 1200, NULL,
		0, 0, 0, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(BUCK4, buck4,  1100, 1100, NULL,
		0, 0, 0, FPS_SRC_DEF, -1, -1, MAX77660_EXT_ENABLE_EN3);

MAX77660_PDATA_INIT(BUCK5, buck5,  1800, 1800, NULL,
		1, 1, 0, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(BUCK6, buck6,  1700, 1800, NULL,
		0, 0, 0, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(BUCK7, buck7,  2650, 2800, NULL,
		0, 0, 0, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO1, ldo1, 800, 800, max77660_rails(buck3),
		1, 0, 1, FPS_SRC_DEF, -1, -1, 0);

//luis
#if 1
MAX77660_PDATA_INIT(LDO2, ldo2, 2800, 2800, NULL,
		0, 0, 1, FPS_SRC_DEF, -1, -1, 0);
#else
MAX77660_PDATA_INIT(LDO2, ldo2, 2800, 2800, NULL,
		0, 0, 1, FPS_SRC_DEF, -1, -1, 0);
#endif

//luis
#if 0
MAX77660_PDATA_INIT(LDO3, ldo3, 2800, 2800, NULL,
		0, 0, 1, FPS_SRC_DEF, -1, -1, 0);
#else
MAX77660_PDATA_INIT(LDO3, ldo3, 1200, 1200, NULL,
		0, 0, 1, FPS_SRC_DEF, -1, -1, 0);
#endif

MAX77660_PDATA_INIT(LDO4, ldo4, 3000, 3000, NULL,
		1, 1, 1, FPS_SRC_DEF, -1, -1, 0);

//luis
#if 0
MAX77660_PDATA_INIT(LDO5, ldo5, 1800, 1800, NULL,
		1, 0, 1, FPS_SRC_DEF, -1, -1, 0);
#else
MAX77660_PDATA_INIT(LDO5, ldo5, 2800, 2800, NULL,
		1, 0, 1, FPS_SRC_DEF, -1, -1, 0);
#endif

MAX77660_PDATA_INIT(LDO6, ldo6, 1200, 1200, max77660_rails(buck5),
		0, 0, 0, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO7, ldo7, 1050, 1050, max77660_rails(buck3),
		1, 1, 0, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO8, ldo8, 900, 1100, max77660_rails(buck3),
		0, 0, 0, FPS_SRC_DEF, -1, -1, MAX77660_EXT_ENABLE_EN3);

MAX77660_PDATA_INIT(LDO9, ldo9, 2800, 2800, NULL,
		1, 1, 1, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO10, ldo10, 1800, 1800, NULL,
		0, 0, 1, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO11, ldo11, 3300, 3300, NULL,
		0, 0, 1, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO12, ldo12, 1800, 3300, NULL,
		0, 0, 1, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO13, ldo13, 2850, 2850, NULL,
		0, 0, 1, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO14, ldo14, 2850, 2850, NULL,
		0, 0, 1, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO15, ldo15, 1800, 3000, NULL,
		0, 0, 0, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO16, ldo16, 1800, 3000, NULL,
		0, 0, 0, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO17, ldo17, 1800, 1800, max77660_rails(buck7),
		1, 1, 1, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(LDO18, ldo18, 2700, 2700, NULL,
		1, 0, 1, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(SW1, sw1, 1800, 1800, max77660_rails(buck5),
		0, 1, 0, FPS_SRC_DEF, -1, -1, 0);

//luis
#if 0
MAX77660_PDATA_INIT(SW2, sw2, 1800, 1800, max77660_rails(buck5),
		0, 0, 0, FPS_SRC_DEF, -1, -1, 0);
#else
MAX77660_PDATA_INIT(SW2, sw2, 1800, 1800, max77660_rails(buck5),
		0, 1, 0, FPS_SRC_DEF, -1, -1, 0);
#endif

MAX77660_PDATA_INIT(SW3, sw3, 1800, 1800, max77660_rails(buck5),
		0, 1, 0, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(SW4, sw4, 1100, 1100, max77660_rails(buck1),
		0, 0, 0, FPS_SRC_DEF, -1, -1, 0);

MAX77660_PDATA_INIT(SW5, sw5, 1200, 1200, max77660_rails(buck3),
		0, 0, 0, FPS_SRC_DEF, -1, -1, 0);

#define MAX77660_REG(_id, _data) 	\
	[MAX77660_REGULATOR_ID_##_id] = (&max77660_regulator_pdata_##_data)

static struct max77660_regulator_platform_data *max77660_reg_pdata[] = {
	MAX77660_REG(BUCK1, buck1),
	MAX77660_REG(BUCK2, buck2),
	MAX77660_REG(BUCK3, buck3),
	MAX77660_REG(BUCK4, buck4),
	MAX77660_REG(BUCK5, buck5),
	MAX77660_REG(BUCK6, buck6),
	MAX77660_REG(BUCK7, buck7),
	MAX77660_REG(LDO1, ldo1),
	MAX77660_REG(LDO2, ldo2),
	MAX77660_REG(LDO3, ldo3),
	MAX77660_REG(LDO4, ldo4),
	MAX77660_REG(LDO5, ldo5),
	MAX77660_REG(LDO6, ldo6),
	MAX77660_REG(LDO7, ldo7),
	MAX77660_REG(LDO8, ldo8),
	MAX77660_REG(LDO9, ldo9),
	MAX77660_REG(LDO10, ldo10),
	MAX77660_REG(LDO11, ldo11),
	MAX77660_REG(LDO12, ldo12),
	MAX77660_REG(LDO13, ldo13),
	MAX77660_REG(LDO14, ldo14),
	MAX77660_REG(LDO15, ldo15),
	MAX77660_REG(LDO16, ldo16),
	MAX77660_REG(LDO17, ldo17),
	MAX77660_REG(LDO18, ldo18),
	MAX77660_REG(SW1, sw1),
	MAX77660_REG(SW2, sw2),
	MAX77660_REG(SW3, sw3),
	MAX77660_REG(SW4, sw4),
	MAX77660_REG(SW5, sw5),
};

#define MAX77660_INIT_PINS(_id, _gpio, _od, _up_dn, _flags)	\
{								\
	.pin_id			= MAX77660_PINS_##_id,		\
	.gpio_pin_mode		= _gpio,			\
	.open_drain		= _od,				\
	.pullup_dn_normal	= MAX77660_PIN_##_up_dn,	\
	.gpio_init_flag		= _flags,			\
}

static struct max77660_pinctrl_platform_data max77660_pinctrl_pdata[] = {
	MAX77660_INIT_PINS(GPIO0, 0, 1, DEFAULT, 0),
	MAX77660_INIT_PINS(GPIO1, 1, 0, PULL_NORMAL, GPIOF_IN),
	MAX77660_INIT_PINS(GPIO2, 1, 1, PULL_UP, GPIOF_OUT_INIT_HIGH),
	MAX77660_INIT_PINS(GPIO3, 1, 0, PULL_NORMAL, GPIOF_IN),
	MAX77660_INIT_PINS(GPIO4, 1, 0, PULL_NORMAL, GPIOF_IN),
	MAX77660_INIT_PINS(GPIO5, 1, 1, PULL_UP, GPIOF_IN),
	MAX77660_INIT_PINS(GPIO6, 1, 1, PULL_UP, GPIOF_IN),
	MAX77660_INIT_PINS(GPIO7, 0, 0, PULL_NORMAL, 0),
	MAX77660_INIT_PINS(GPIO8, 0, 1, PULL_UP, 0),
	MAX77660_INIT_PINS(GPIO9, 1, 1, PULL_UP, GPIOF_IN),
	//MAX77660_INIT_PINS(GPIO9, 0, 0, PULL_UP, GPIOF_IN),
};

static struct regulator_consumer_supply max77660_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.0"),
	REGULATOR_SUPPLY("usb_vbus", "tegra-otg"),
};

static struct regulator_consumer_supply max77660_batt_supply[] = {
	REGULATOR_SUPPLY("usb_bat_chg", "tegra-udc.0"),
};

struct max77660_vbus_platform_data max77660_vbus_pdata = {
	.num_consumer_supplies = ARRAY_SIZE(max77660_vbus_supply),
	.consumer_supplies = max77660_vbus_supply,
};

//125 ~ -40
//this is for tinno board with 10k pullup
uint32_t max77660_adc_temperature_lookup_table[] = {
	214, 219, 223, 228, 233, 238, 243, 248,
	253, 259, 264, 270, 276, 282, 288, 294,
	301, 308, 314, 321, 329, 336, 343, 351,
	359, 367, 376, 384, 393, 402, 411, 421,
	431, 441, 451, 461, 472, 483, 495, 506,
	518, 531, 543, 556, 569, 583, 597, 611,
	626, 641, 657, 672, 689, 705, 722, 740,
	758, 776, 795, 814, 834, 854, 875, 896,
	918, 940, 962, 986, 1009, 1034, 1058, 1084,
	1110, 1136, 1163, 1191, 1219, 1247, 1276, 1306,
	1337, 1367, 1399, 1431, 1463, 1496, 1530, 1564,
	1599, 1634, 1669, 1705, 1742, 1779, 1816, 1854,
	1892, 1930, 1969, 2009, 2048, 2088, 2128, 2168,
	2208, 2249, 2289, 2330, 2371, 2412, 2452, 2493,
	2534, 2574, 2615, 2655, 2695, 2734, 2774, 2813,
	2852, 2890, 2928, 2965, 3002, 3039, 3074, 3110,
	3145, 3179, 3212, 3245, 3277, 3309, 3339, 3369,
	3399, 3427, 3455, 3482, 3509, 3534, 3559, 3583,
	3606, 3629, 3651, 3672, 3692, 3712, 3731, 3749,
	3767, 3783, 3800, 3815, 3830, 3844, 3858, 3871,
	3884, 3896, 3907, 3918, 3928, 3938
};


/*
* Values calculated using formula to calculate Resistance given Vthm values
* and temperature resistance correspondence obatined from thermistor
* datasheet
*/

static struct max77660_bcharger_platform_data max77660_bcharger_pdata = {
//Ivan	.tz_name = "battery-temp",
	.tz_name = "generic-adc-thermal",  	//Ivan added
	.max_charge_current_mA = 3000,
	.consumer_supplies = max77660_batt_supply,
	.num_consumer_supplies = ARRAY_SIZE(max77660_batt_supply),
	.wdt_timeout    = 32,
#if (CONFIG_MACH_S9321 == 1)
	.max_term_vol_mV = 4400,	
#else
	.max_term_vol_mV = 4300,
#endif
	.temperature_poll_period_secs = 5,
	.oc_thresh = OC_THRESH_4A0
};

static struct max77660_charger_platform_data max77660_charger_pdata = {
	.ext_conn_name = "max77660-extcon",
	.bcharger_pdata = &max77660_bcharger_pdata,
	.vbus_pdata = &max77660_vbus_pdata,
};

static struct power_supply_extcon_plat_data extcon_pdata = {
	.extcon_name = "tegra-udc",
};

static struct platform_device power_supply_extcon_device = {
	.name           = "power-supply-extcon",
	.id             = -1,
	.dev		= {
		.platform_data = &extcon_pdata,
	},
};

static struct iio_map max77660_iio_map[] = {
	MAX77660_GPADC_IIO_MAP(VTHM, "generic-adc-thermal.0", "vthm_channel"),
	MAX77660_GPADC_IIO_MAP(NULL, NULL, NULL),
};

static struct gadc_thermal_platform_data gadc_thermal_battery_pdata = {
	.iio_channel_name = "vthm_channel",
//	.tz_name = "battery-temp",
	.tz_name = "generic-adc-thermal",	
	.temp_offset = 0,
	.adc_to_temp = NULL,
	.adc_temp_lookup = max77660_adc_temperature_lookup_table,
	.lookup_table_size = ARRAY_SIZE(max77660_adc_temperature_lookup_table),
	.first_index_temp = 125,
	.last_index_temp = -40,
};

static struct platform_device gadc_thermal_battery = {
	.name   = "generic-adc-thermal",
	.id     = 0,
	.dev    = {
		.platform_data = &gadc_thermal_battery_pdata,
	},
};

struct max77660_adc_platform_data max77660_adc_pdata = {
	.adc_current_uA = 10,
	.adc_avg_sample = 2,
	.adc_ref_enabled = 0,
	.channel_mapping = max77660_iio_map,
};

struct max77660_clk32k_platform_data clk32_pdata = {
	.en_clk32out1 = true,
	.en_clk32out2 = true,
	.clk32k_mode = MAX77660_CLK_MODE_LOW_JITTER,
	.clk32k_load_cap = MAX77660_CLK_LOAD_CAP_22pF,
};

struct max77660_sim_platform_data max77660_sim_pdata = {
	.sim_reg = {
		{
			.detect_en = 1,
			.batremove_en = 0,
			.det_debouncecnt = 0x10,
#ifdef CONFIG_SIM_MAX77660_HOTSWAP
			.auto_pwrdn_en = 1,
#else
			.auto_pwrdn_en = 0,
#endif
			.inst_pol = 0,			//Ivan 1 -> 0
			.sim_puen = 1,
			.pwrdn_debouncecnt = 0x20,
		},
		{
			.detect_en = 1,
			.batremove_en = 0,
			.det_debouncecnt = 0x10,
#ifdef CONFIG_SIM_MAX77660_HOTSWAP
			.auto_pwrdn_en = 1,
#else
			.auto_pwrdn_en = 0,
#endif
			.inst_pol = 0,
			.sim_puen = 1,
			.pwrdn_debouncecnt = 0x20,
		},
	},
};

static struct max77660_haptic_platform_data max77660_haptic_pdata = {
	.type = MAX77660_HAPTIC_ERM,
	.mode = MAX77660_INTERNAL_MODE,
	.internal_mode_pattern = 0,
	.pattern_cycle = 10,
	.pattern_signal_period = 0xD0,
	.feedback_duty_cycle = 12,
	.invert = MAX77660_INVERT_OFF,
	.cont_mode = MAX77660_CONT_MODE,
	.motor_startup_val = 0,
	.scf_val = 7,
	.edp_states = { 180, 0 }
};

static struct max77660_haptic_platform_data_encl max77660_haptic_pdata_encl = {
	.pdata = &max77660_haptic_pdata,
	.size = sizeof(max77660_haptic_pdata),
};

static struct max77660_platform_data max77660_pdata = {
	.irq_base	= MAX77660_IRQ_BASE,
	.gpio_base	= MAX77660_GPIO_BASE,

	.pinctrl_pdata	= max77660_pinctrl_pdata,
	.num_pinctrl	= ARRAY_SIZE(max77660_pinctrl_pdata),

	.charger_pdata = &max77660_charger_pdata,

	.clk32k_pdata = &clk32_pdata,

	.adc_pdata = &max77660_adc_pdata,

	.sim_pdata = &max77660_sim_pdata,

	.haptic_pdata = &max77660_haptic_pdata_encl,

	.flags	= 0x00,
	.use_power_off	= true,
#if !defined(CONFIG_TEGRA_FIQ_DEBUGGER)
	.system_watchdog_timeout = 32,
#else
	.system_watchdog_timeout = 0,
#endif
	.system_watchdog_reset_timeout = 20,
	.dvfs_pd = {
		.en_pwm = true,
		.step_voltage_uV = 25000,
		.default_voltage_uV = 1100000,
		.base_voltage_uV = 600000,
		.max_voltage_uV = 1200000,
	},
	.led_disable = true,
};

static struct i2c_board_info __initdata max77660_regulators[] = {
	{
		/* The I2C address was determined by OTP factory setting */
		I2C_BOARD_INFO("max77660", MAX77660_PWR_I2C_ADDR),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &max77660_pdata,
	},
};

/* LP8755 DC-DC converter */
static struct regulator_consumer_supply lp8755_buck0_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator *s_lp8755_regulator;
static struct work_struct lp8755_work;
static struct workqueue_struct *workqueue;


#define lp8755_rail(_id) "lp8755_"#_id

#define LP8755_PDATA_INIT(_name, _minmv, _maxmv, _supply_reg, _always_on, \
	_boot_on, _apply_uv)						\
	static struct regulator_init_data reg_idata_##_name = {		\
		.constraints = {					\
			.name = lp8755_rail(_name),			\
			.min_uV = (_minmv)*1000,			\
			.max_uV = (_maxmv)*1000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,			\
			.boot_on = _boot_on,				\
			.apply_uV = _apply_uv,				\
		},							\
		.num_consumer_supplies =				\
			ARRAY_SIZE(lp8755_##_name##_supply),		\
		.consumer_supplies = lp8755_##_name##_supply,		\
		.supply_regulator = _supply_reg,			\
	}

LP8755_PDATA_INIT(buck0, 500, 1670, NULL, 0, 0, 0);

#define LP8755_REG_PDATA(_sname) &reg_idata_##_sname

static struct regulator_init_data *lp8755_reg_data[LP8755_BUCK_MAX] = {
	LP8755_REG_PDATA(buck0),
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

static struct lp8755_platform_data lp8755_pdata;

static struct i2c_board_info lp8755_regulators[] = {
	{
		I2C_BOARD_INFO(LP8755_NAME, 0x60),
		.irq		= 0,
		.platform_data	= &lp8755_pdata,
	},
};

static void lp8755_regulator_init(void)
{
	lp8755_pdata.mphase = MPHASE_CONF6;
	lp8755_pdata.buck_data[LP8755_BUCK0] = lp8755_reg_data[LP8755_BUCK0];
	lp8755_pdata.ramp_us[LP8755_BUCK0] = 230;

	i2c_register_board_info(4, lp8755_regulators,
			ARRAY_SIZE(lp8755_regulators));
}

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
/* VDD_CPU can source from extrenal CPU regulator or from one of the  *
 * voltage rail of the PMIC. For different boards, the configuration  *
 * used as below:
 *  E1690(CERES FFD): CPU_VDD is from Buck2 of Max 77660
 *  E1680(CERES)    : For SKU 1001 CPU_VDD is from Buck2 of Max 77660
 *		      For SKU 1000 CPU_VDD is from LP8755LME
 *  E1683(CERES)    : Same as E1680 A04
 *  E1740(ATLANTIS FFD) : CPU_VDD is from SMPS12 of TPS80036
 *  E1670(ATLANTIS) : For SKU 100 CPU_VDD is from SMPS12 of TPS80036
 *		      For SKU 120 CPU_VDD is from LP8755LME	     */

/* LP8755LME: fixed 10mV steps from 500mV to 1670mV, with offset 0x80 */
#define LP8755_CPU_VDD_MAP_SIZE ((1670000 - 500000) / 10000 + 1)
static struct voltage_reg_map  lp8755_cpu_vdd_map[LP8755_CPU_VDD_MAP_SIZE];
static inline void lp8755_vdd_cpu_fill_reg_map(void)
{
	int i;
	for (i = 0; i < LP8755_CPU_VDD_MAP_SIZE; i++) {
		lp8755_cpu_vdd_map[i].reg_value = i + 0x80;
		lp8755_cpu_vdd_map[i].reg_uV = 500000 + 10000 * i;
	}
}

/* board parameters for cpu dfll */
static struct tegra_cl_dvfs_cfg_param ceres_cl_dvfs_param = {
	.sample_rate = 12500,
	.force_mode = TEGRA_CL_DVFS_FORCE_FIXED,
	.cf = 10,
	.ci = 0,
	.cg = 2,

	.droop_cut_value = 0xF,
	.droop_restore_ramp = 0x0,
	.scale_out_ramp = 0x0,
};

static struct tegra_cl_dvfs_platform_data ceres_cl_dvfs_lp8755_vdd_cpu_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_I2C,
	.u.pmu_i2c = {
		.fs_rate = 400000,
		.slave_addr = 0xc0,
		.reg = 0x00,
	},
	.vdd_map = lp8755_cpu_vdd_map,
	.vdd_map_size = LP8755_CPU_VDD_MAP_SIZE,

	.cfg_param = &ceres_cl_dvfs_param,
};

/* Maxim 77660 Buck2: fixed 6.25mV steps from 600mV to 1500mV, with offset 0 */
#define MAX77660_CPU_VDD_MAP_SIZE ((1500000 - 600000) / 6250 + 1)
static struct voltage_reg_map max77660_cpu_vdd_map[MAX77660_CPU_VDD_MAP_SIZE];
static inline void max77660_vdd_cpu_fill_reg_map(void)
{
	int i;
	for (i = 0; i < MAX77660_CPU_VDD_MAP_SIZE; i++) {
		max77660_cpu_vdd_map[i].reg_value = i + 0x0;
		max77660_cpu_vdd_map[i].reg_uV = 600000 + 6250 * i;
	}
}

static struct tegra_cl_dvfs_platform_data \
				ceres_cl_dvfs_max77660_vdd_cpu_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_I2C,
	.u.pmu_i2c = {
		.fs_rate = 400000,
		.slave_addr = 0x46,
		.reg = 0x47,
	},
	.vdd_map = max77660_cpu_vdd_map,
	.vdd_map_size = MAX77660_CPU_VDD_MAP_SIZE,

	.cfg_param = &ceres_cl_dvfs_param,
};

/* TPS80036 SMPS12 fixed 10mV steps from 500mV to 1650mV, with offset 6 */
#define TPS80036_CPU_VDD_MAP_SIZE ((1650000 - 500000) / 10000 + 1)
static struct voltage_reg_map tps80036_cpu_vdd_map[TPS80036_CPU_VDD_MAP_SIZE];
static inline void tps80036_vdd_cpu_fill_reg_map(void)
{
	int i;
	for (i = 0; i < TPS80036_CPU_VDD_MAP_SIZE; i++) {
		tps80036_cpu_vdd_map[i].reg_value = i + 0x6;
		tps80036_cpu_vdd_map[i].reg_uV = 500000 + 10000 * i;
	}
}

static struct tegra_cl_dvfs_platform_data \
				atlantis_cl_dvfs_tps80036_vdd_cpu_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_I2C,
	.u.pmu_i2c = {
		.fs_rate = 400000,
		.slave_addr = 0xb0,
		.reg = 0x23,
	},
	.vdd_map = tps80036_cpu_vdd_map,
	.vdd_map_size = TPS80036_CPU_VDD_MAP_SIZE,

	.cfg_param = &ceres_cl_dvfs_param,
};

int tegra_get_cvb_alignment_uV(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_E1690) ||
	    (board_info.board_id == BOARD_E1680 && board_info.sku == 1001) ||
	    (board_info.board_id == BOARD_E1683 && board_info.sku == 1001))
		return 6250;
	else
		return 10000;
}

static void lp8755_slew_rate_work(struct work_struct *work)
{
	if (s_lp8755_regulator == NULL)
		s_lp8755_regulator = regulator_get(NULL, "vdd_cpu");

	if (!IS_ERR(s_lp8755_regulator))
		regulator_set_ramp_delay(s_lp8755_regulator, 230);
}

void tegra_set_vdd_cpu_ramp_rate(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_E1680 && board_info.sku == 1000) ||
	   (board_info.board_id == BOARD_E1683 && board_info.sku == 1000) ||
		(board_info.board_id == BOARD_E1670 && board_info.sku == 120)) {
		if (workqueue != NULL)
			queue_work(workqueue, &lp8755_work);
	}
}


static int __init ceres_cl_dvfs_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_E1690) ||
	    (board_info.board_id == BOARD_E1680 && board_info.sku == 1001) ||
	    (board_info.board_id == BOARD_E1683 && board_info.sku == 1001)) {
		max77660_vdd_cpu_fill_reg_map();
		tegra_cl_dvfs_device.dev.platform_data =
					&ceres_cl_dvfs_max77660_vdd_cpu_data;
	} else if (((board_info.board_id == BOARD_E1670) &&
			(board_info.sku == 100)) ||
			(board_info.board_id == BOARD_E1740)) {
		tps80036_vdd_cpu_fill_reg_map();
		tegra_cl_dvfs_device.dev.platform_data =
					&atlantis_cl_dvfs_tps80036_vdd_cpu_data;
	} else {
		lp8755_vdd_cpu_fill_reg_map();
		tegra_cl_dvfs_device.dev.platform_data =
					&ceres_cl_dvfs_lp8755_vdd_cpu_data;
	}
	platform_device_register(&tegra_cl_dvfs_device);

	return 0;
}
#endif


int __init ceres_regulator_init(void)
{
	struct board_info board_info;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
	int id;
	bool wdt_disable;
	int modem_id = tegra_get_modem_id();

	tegra_get_board_info(&board_info);
	pr_info("board_info: id:sku:fab:major:minor = 0x%04x:0x%04x:0x%02x:0x%02x:0x%02x\n",
		board_info.board_id, board_info.sku,
		board_info.fab, board_info.major_revision,
		board_info.minor_revision);

	if ((board_info.board_id == BOARD_E1680 && board_info.sku == 1000) ||
	    (board_info.board_id == BOARD_E1683 && board_info.sku == 1000) ||
		(board_info.board_id == BOARD_E1670 && board_info.sku == 120)) {
		workqueue = create_singlethread_workqueue("lp8755_wq");
		if (!workqueue)
			pr_err("cannot create workqueue\n");
		INIT_WORK(&lp8755_work, lp8755_slew_rate_work);
	}

	if (board_info.board_id == BOARD_E1670 || board_info.board_id == BOARD_E1740) {
		atlantis_regulator_init();
		atlantis_pwm_init();
		atlantis_vibrator_init();
		goto reg_populate_done;
	}

	/* configure the power management controller to trigger PMU
	 * interrupts when low */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	regulator_has_full_constraints();
	for (id = 0; id < MAX77660_REGULATOR_ID_NR; ++id)
		max77660_pdata.regulator_pdata[id] = max77660_reg_pdata[id];

	/* set ldo1 to 0.85v from 0.80v */
	max77660_regulator_idata_ldo1.constraints.min_uV = 850000;
	max77660_regulator_idata_ldo1.constraints.max_uV = 850000;

	switch (board_info.board_id) {
	case BOARD_E1680:
	case BOARD_E1681:
	case BOARD_E1683:
	case BOARD_E1671:
		max77660_pinctrl_pdata[MAX77660_PINS_GPIO1].pullup_dn_normal =
					MAX77660_PIN_PULL_UP;
		max77660_pinctrl_pdata[MAX77660_PINS_GPIO1].open_drain = 1;
		max77660_pinctrl_pdata[MAX77660_PINS_GPIO2].pullup_dn_normal =
					MAX77660_PIN_PULL_NORMAL;
		max77660_pinctrl_pdata[MAX77660_PINS_GPIO2].open_drain = 0;

		max77660_regulator_idata_ldo13.constraints.boot_off = 1;
		if (board_info.sku == 1001) {
			max77660_regulator_pdata_buck1.fps_src = FPS_SRC_3;
			max77660_regulator_pdata_buck6.fps_src = FPS_SRC_3;
			max77660_regulator_pdata_buck7.fps_src = FPS_SRC_3;
			max77660_regulator_pdata_ldo17.fps_src = FPS_SRC_NONE;
			max77660_regulator_pdata_ldo18.fps_src = FPS_SRC_NONE;
		} else {
			max77660_regulator_pdata_buck2.flags = MAX77660_EXT_ENABLE_EN1;
			max77660_regulator_pdata_buck1.flags = 0;
			max77660_regulator_idata_buck1.consumer_supplies = max77660_unused_supply;
			max77660_regulator_idata_buck1.num_consumer_supplies =
					ARRAY_SIZE(max77660_unused_supply);
			max77660_regulator_idata_buck2.consumer_supplies = max77660_buck1_supply;
			max77660_regulator_idata_buck2.num_consumer_supplies =
							ARRAY_SIZE(max77660_buck1_supply);

			max77660_regulator_pdata_buck1.fps_src = FPS_SRC_NONE;
			max77660_regulator_pdata_buck2.fps_src = FPS_SRC_3;
			max77660_regulator_pdata_buck6.fps_src = FPS_SRC_3;
			max77660_regulator_pdata_buck7.fps_src = FPS_SRC_3;
			max77660_regulator_pdata_ldo17.fps_src = FPS_SRC_NONE;
			max77660_regulator_pdata_ldo18.fps_src = FPS_SRC_NONE;

			lp8755_regulator_init();
		}

		if (((board_info.board_id == BOARD_E1680) ||
		     (board_info.board_id == BOARD_E1683)) &&
			(tegra_get_modem_config() & BIT(0))) {
			/*
			 * When bit0 is set it indicates that BB_PWR_REQ is
			 * connected to EN5 and that LDO17/18 should be
			 * part of FPS6. This allows LDO17/18 to turn on/off
			 * as the modem exits/enters hibernate
			 */
			max77660_regulator_pdata_ldo17.flags =
						MAX77660_EXT_ENABLE_EN5;
			max77660_regulator_pdata_ldo17.fps_src = FPS_SRC_6;
			max77660_regulator_pdata_ldo17.fps_pu_period =
						FPS_POWER_PERIOD_0;
			max77660_regulator_pdata_ldo17.fps_pd_period =
						FPS_POWER_PERIOD_0;

			max77660_regulator_pdata_ldo18.flags =
						MAX77660_EXT_ENABLE_EN5;
			max77660_regulator_pdata_ldo18.fps_src = FPS_SRC_6;
			max77660_regulator_pdata_ldo18.fps_pu_period =
						FPS_POWER_PERIOD_0;
			max77660_regulator_pdata_ldo18.fps_pd_period =
						FPS_POWER_PERIOD_0;
		}

		break;

	case BOARD_E1690:
		max77660_regulator_pdata_buck1.fps_src = FPS_SRC_3;
		max77660_regulator_pdata_buck6.fps_src = FPS_SRC_3;
		max77660_regulator_pdata_buck7.fps_src = FPS_SRC_3;
		max77660_regulator_pdata_ldo17.fps_src = FPS_SRC_NONE;
		max77660_regulator_pdata_ldo18.fps_src = FPS_SRC_NONE;
		break;

	default:
		pr_info("UNSUPPORTED BOARD\n");
		return -1;
	}

	if (get_display_config() == 0) {
		max77660_regulator_idata_ldo4.consumer_supplies =
			max77660_ldo4_display_config0_supply;
		max77660_regulator_idata_ldo4.num_consumer_supplies =
			ARRAY_SIZE(max77660_ldo4_display_config0_supply);
	}

	wdt_disable = is_pmic_wdt_disabled_at_boot();
	if (wdt_disable) {
		max77660_pdata.system_watchdog_timeout = 0;
		max77660_bcharger_pdata.wdt_timeout = 0;
	}

	/* Pass NULL bcharger platform data for Adapter source */
	if (get_power_supply_type() == POWER_SUPPLY_TYPE_MAINS) {
		max77660_charger_pdata.bcharger_pdata = NULL;
	}

	/* Ceres FAB <= A02 LPDDR3 regulators on SW5 so keep it always_on */
	if (tegra_revision <= TEGRA_REVISION_A02) {
		max77660_regulator_pdata_sw5.fps_src = FPS_SRC_NONE;
		max77660_regulator_idata_sw5.constraints.always_on = 1;
		max77660_regulator_idata_sw5.consumer_supplies =
						max77660_sw5_a02_supply;
		max77660_regulator_idata_sw5.num_consumer_supplies =
					ARRAY_SIZE(max77660_sw5_a02_supply);
		max77660_regulator_idata_buck3.consumer_supplies =
						max77660_buck3_a02_supply;
		max77660_regulator_idata_buck3.num_consumer_supplies =
					ARRAY_SIZE(max77660_buck3_a02_supply);
	}

	if (((board_info.board_id == BOARD_E1680) &&
	    (board_info.fab >= BOARD_FAB_A04)) ||
	    (board_info.board_id == BOARD_E1683))
		max77660_pdata.use_rcm_reset = true;

	if (modem_id == TEGRA_BB_INTEGRATED_DISABLED) {
		max77660_regulator_pdata_buck4.flags = 0;
		max77660_regulator_idata_buck4.constraints.always_on = 1;
		max77660_regulator_pdata_ldo8.flags = 0;
	}
       /* Set FPS_SRC to none of sw5 */
        max77660_regulator_pdata_sw5.fps_src = FPS_SRC_0;

	i2c_register_board_info(4, max77660_regulators,
				ARRAY_SIZE(max77660_regulators));

	platform_device_register(&power_supply_extcon_device);
	platform_device_register(&gadc_thermal_battery);

reg_populate_done:
#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
	ceres_cl_dvfs_init();
#endif

	return 0;
}

/* Always ON /Battery regulator */
static struct regulator_consumer_supply fixed_reg_battery_supply[] = {
	REGULATOR_SUPPLY("vdd_sys_bl", NULL),
	REGULATOR_SUPPLY("vdd_sys_cam", NULL),
	REGULATOR_SUPPLY("vdd_vbrtr", NULL),
};

/* LCD_AVDD_EN From PMU GP6 */
static struct regulator_consumer_supply fixed_reg_avdd_lcd_supply[] = {
	REGULATOR_SUPPLY("avdd_lcd", NULL),
};

static struct regulator_consumer_supply fixed_reg_vdd_hdmi_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_5v0", "tegradc.1"),
};

/* Macro for defining fixed regulator sub device data */
#define FIXED_SUPPLY(_name) "fixed_reg_"#_name
#define FIXED_REG(_id, _var, _name, _in_supply, _always_on, _boot_on,	\
	_gpio_nr, _open_drain, _active_high, _boot_state, _millivolts,	\
	_delay)	\
	static struct regulator_init_data ri_data_##_var =		\
	{								\
		.supply_regulator = _in_supply,				\
		.num_consumer_supplies =				\
			ARRAY_SIZE(fixed_reg_##_name##_supply),		\
		.consumer_supplies = fixed_reg_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,			\
			.boot_on = _boot_on,				\
		},							\
	};								\
	static struct fixed_voltage_config fixed_reg_##_var##_pdata =	\
	{								\
		.supply_name = FIXED_SUPPLY(_name),			\
		.microvolts = _millivolts * 1000,			\
		.gpio = _gpio_nr,					\
		.gpio_is_open_drain = _open_drain,			\
		.enable_high = _active_high,				\
		.enabled_at_boot = _boot_state,				\
		.init_data = &ri_data_##_var,				\
		.startup_delay = _delay,				\
	};								\
	static struct platform_device fixed_reg_##_var##_dev = {	\
		.name = "reg-fixed-voltage",				\
		.id = _id,						\
		.dev = {						\
			.platform_data = &fixed_reg_##_var##_pdata,	\
		},							\
	}

FIXED_REG(0,	battery,	battery,
	NULL,	0,	0,
	-1,	false, true,	0,	3300, 0);

FIXED_REG(1,	avdd_lcd,	avdd_lcd,
	NULL,	0,	0,
	MAX77660_GPIO_BASE + MAX77660_GPIO6,	false,	true,
	1,	2800, 0);
FIXED_REG(2,	vdd_hdmi_5v0,	vdd_hdmi_5v0,
	NULL,	0,	0,
	MAX77660_GPIO_BASE + MAX77660_GPIO3,	false,	true,
	0,	5000, 250000);
/*
 * Creating the fixed regulator device tables
 */

#define ADD_FIXED_REG(_name)    (&fixed_reg_##_name##_dev)

#define CERES_COMMON_FIXED_REG		\
	ADD_FIXED_REG(battery),		\
	ADD_FIXED_REG(vdd_hdmi_5v0)	\


/*Gpio switch regulator platform data for Ceres E1680 */
static struct platform_device *fixed_reg_devs_e1680[] = {
	CERES_COMMON_FIXED_REG,
};

static struct platform_device *fixed_reg_devs_e1680_display_config[] = {
	ADD_FIXED_REG(avdd_lcd),
};

static int __init ceres_fixed_regulator_init(void)
{
	struct board_info board_info;
	if (!of_machine_is_compatible("nvidia,ceres"))
		return 0;

	tegra_get_board_info(&board_info);
	if (board_info.board_id == BOARD_E1670 || board_info.board_id == BOARD_E1740)
		return atlantis_fixed_regulator_init();

	if(get_display_config() == 0)
		platform_add_devices(fixed_reg_devs_e1680_display_config,
				ARRAY_SIZE(fixed_reg_devs_e1680_display_config));

	return platform_add_devices(fixed_reg_devs_e1680,
				ARRAY_SIZE(fixed_reg_devs_e1680));
}
subsys_initcall_sync(ceres_fixed_regulator_init);

static void ceres_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP0) &&
		(stg == TEGRA_SUSPEND_BEFORE_PERIPHERAL))
		ceres_pinmux_suspend();
}

static void ceres_board_resume(int lp_state, enum resume_stage stg)
{
}

#ifdef CONFIG_TEGRA_LP1_LOW_COREVOLTAGE
static struct lpx_corev_reg_lookup ceres_reg_lookup[] = {
	{   0, 0x20}, /* No voltage is no lp1bb, set lowest voltage 0.8V */
	{ 800, 0x20},
	{ 825, 0x24},
	{ 850, 0x28},
	{ 900, 0x30},
	{ 950, 0x38},
	{1000, 0x40},
	{1050, 0x48},
	{1100, 0x50},
	{1150, 0x58},
	{1200, 0x60},
	{1230, 0x65},
	{INT_MAX, 0x68},
};

static int ceres_lp1_reg_lookup(int voltage)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(ceres_reg_lookup); i++)
		if (voltage <= ceres_reg_lookup[i].core_voltage)
			return ceres_reg_lookup[i].reg_value;
	return 0x65; /* Default Voltage: 1.230V */
}
#endif

static struct tegra_suspend_platform_data ceres_suspend_data = {
	.cpu_timer	= 300,
	.cpu_off_timer	= 300,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x157e,
	.core_off_timer = 10,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.cpu_lp2_min_residency = 1000,
	.board_suspend = ceres_board_suspend,
	.board_resume = ceres_board_resume,
#ifdef CONFIG_TEGRA_LP1_LOW_COREVOLTAGE
	/* Settings for Ceres */
	.lp1_lowvolt_support = true,
	.i2c_base_addr = TEGRA_I2C5_BASE,
	.pmuslave_addr = 0x46,
	.core_reg_addr = 0x46,
	.lp1_core_volt_low_cold = 0x20,
	.lp1_core_volt_low = 0x20,
	.lp1_core_volt_high = 0x58,
	.lp1_lookup_reg = ceres_lp1_reg_lookup,
#endif
#ifdef CONFIG_TEGRA_WDT_CLEAR_EARLY
	/* Settings for Ceres */
	.wdt_clear_early_support = true,
	.i2c_clk_offset = (1 << 15),
	.wdt_clr_reg = { 0x20, 1 },
	.wdt_int_status_regs = {
		{0x5, 1 << 7},
		{0x8, 1 << 1},
		{-1, -1},
	},
#endif
};


int __init ceres_suspend_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
#ifdef CONFIG_TEGRA_LP1_LOW_COREVOLTAGE
	if ((board_info.board_id == BOARD_E1680) ||
	    (board_info.board_id == BOARD_E1683)) {
		if (board_info.sku == 1000)
			ceres_suspend_data.core_reg_addr = 0x47;
	} else {
		ceres_suspend_data.lp1_lowvolt_support = false;
#ifdef CONFIG_TEGRA_WDT_CLEAR_EARLY
		ceres_suspend_data.wdt_clear_early_support = false;
#endif
	}
#endif
	tegra_init_suspend(&ceres_suspend_data);

	return 0;
}

int __init ceres_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA)
		regulator_mA = 9000;

	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);
	tegra_init_cpu_edp_limits(regulator_mA);

	regulator_mA = get_maximum_core_current_supported();
	if (!regulator_mA)
		regulator_mA = 4000;

	pr_info("%s: core regulator %d mA\n", __func__, regulator_mA);
	tegra_init_core_edp_limits(regulator_mA);

	return 0;
}

static struct tegra_tsensor_pmu_data tpdata_max77660 = {
	.reset_tegra = 1,
	.pmu_16bit_ops = 0,
	.controller_type = 0,
	.pmu_i2c_addr = 0x23,
	.i2c_controller_id = 4,
	.poweroff_reg_addr = 0x1c,
	.poweroff_reg_data = 0x4,
};

static struct pid_thermal_gov_params soctherm_pid_params = {
	.max_err_temp = 9000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 20,
	.down_compensation = 20,
};

static struct thermal_zone_params soctherm_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &soctherm_pid_params,
};

static struct soctherm_platform_data ceres_soctherm_data = {
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 4,
			.trips = {
				{
					/*camera cooling device trip point*/
					.cdev_type = "camera-throttle",
					.trip_temp = 76000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 86000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 96000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 98000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 4,
			.trips = {
				{
					/*camera cooling device trip point*/
					.cdev_type = "camera-throttle",
					.trip_temp = 78000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 88000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 98000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 100000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
		[THERM_PLL] = {
			.zone_enable = true,
			.num_trips = 1,
			.trips = {
				{
					.cdev_type = "tegra-dram",
					.trip_temp = 78000,
					.trip_type = THERMAL_TRIP_ACTIVE,
					.upper = 1,
					.lower = 1,
				},
			},

		},
	},
	.throttle = {
		[THROTTLE_HEAVY] = {
			.priority = 100,
			.devs = {
				[THROTTLE_DEV_CPU] = {
					.enable = true,
					.depth = 80,
				},
				[THROTTLE_DEV_GPU] = {
					.enable = true,
					.depth = 80,
				},
			},
		},
		[THROTTLE_OC1] = {
			.throt_mode = BRIEF,
			.polarity = 1,
			.pgmask = 1,
			.devs = {
				[THROTTLE_DEV_CPU] = {
					.enable = true,
					.depth = 50,
				},
				[THROTTLE_DEV_GPU] = {
					.enable = true,
					.depth = 50,
				},
			},
		},
		[THROTTLE_OC2] = {
			.throt_mode = BRIEF,
			.devs = {
				[THROTTLE_DEV_CPU] = {
					.enable = true,
					.depth = 50,
				},
				[THROTTLE_DEV_GPU] = {
					.enable = true,
					.depth = 50,
				},
			},
		},

	},
	.tshut_pmu_trip_data = &tpdata_max77660,
};

int __init ceres_soctherm_init(void)
{
	struct board_info board_info;

	/* atlantis soctherm init uses a different config */
	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_E1670) ||
		(board_info.board_id == BOARD_E1740))
		return atlantis_soctherm_init();

	tegra_platform_edp_init(ceres_soctherm_data.therm[THERM_CPU].trips,
				&ceres_soctherm_data.therm[THERM_CPU].num_trips,
				8000); /* edp temperature margin */
	tegra_add_tj_trips(ceres_soctherm_data.therm[THERM_CPU].trips,
			   &ceres_soctherm_data.therm[THERM_CPU].num_trips);

	return tegra11_soctherm_init(&ceres_soctherm_data);
}

static struct edp_manager ceres_sysedp_manager = {
	.name = "battery",
	/* .max = 18500 */
//Ivan	.max = 13500 /* 3.8*2.350*1.5 */
	.max = 22215,
};

void __init ceres_sysedp_init(void)
{
	struct edp_governor *g;
	int r;

	if (!IS_ENABLED(CONFIG_EDP_FRAMEWORK))
		return;

	if (get_power_supply_type() != POWER_SUPPLY_TYPE_BATTERY)
		ceres_sysedp_manager.max = INT_MAX;

	r = edp_register_manager(&ceres_sysedp_manager);
	WARN_ON(r);
	if (r)
		return;

	/* start with priority governor */
	g = edp_get_governor("priority");
	WARN_ON(!g);
	if (!g)
		return;

	r = edp_set_governor(&ceres_sysedp_manager, g);
	WARN_ON(r);
}

/* change this table for battery usecase
 * make sure the depletion will not occupy too
 * much power that the edp update request stops executing
 * and hang the system
*/
#if 1
static unsigned int ceres_psydepl_states[] = {
	9900, 9600, 9300, 9000, 8700, 8400, 8100, 7800,
	7500, 7200, 6900, 6600, 6300, 6000, 5800, 5600,
	5400, 5200, 5000, 4800, 4600, 4400, 4200, 4000,
	3800, 3600, 3400, 3200, 3000, 2800, 2600, 2400,
	2200, 2000, 1900, 1800, 1700, 1600, 1500, 1400,
	1300, 1200, 1100, 1000,  900,  800,  700,  600,
	 500,  400,  300,  200,  100,    0
};
#else
static unsigned int ceres_psydepl_states[] = {
	/* 9900, 9600, 9300, 9000, 8700, 8400, 8100, 7800,
	7500, 7200, 6900, 6600, 6300, 6000, 5800, 5600, */
	5400, 5200, 5000, 4800, 4600, 4400, 4200, 4000,
	3800, 3600, 3400, 3200, 3000, 2800, 2600, 2400,
	2200, 2000, 1900, 1800, 1700, 1600, 1500, 1400,
	1300, 1200, 1100, 1000,  900,  800,  700,  600,
	 500,  400,  300,  200,  100,    0
};
#endif

#if 0
/* Temperature in deci-celcius */
static struct psy_depletion_ibat_lut ceres_ibat_lut[] = {
	{  600,  500 },
	{  450, 4300 },
	{    0, 4300 },
	{ -300,    0 }
};

static struct psy_depletion_rbat_lut ceres_rbat_lut[] = {
	{ 100, 136000 },
	{  90, 108000 },
	{  80,  96000 },
	{  70,  84000 },
	{  60,  80000 },
	{  50,  80000 },
	{  40,  80000 },
	{  30,  84000 },
	{  20,  92000 },
	{  10, 108000 },
	{   0, 136000 }
};

static struct psy_depletion_ocv_lut ceres_ocv_lut[] = {
	{ 100, 4200000 },
	{  90, 4151388 },
	{  80, 4064953 },
	{  70, 3990914 },
	{  60, 3916230 },
	{  50, 3863778 },
	{  40, 3807535 },
	{  30, 3781554 },
	{  20, 3761117 },
	{  10, 3663381 },
	{   0, 3514236 }
};

static struct psy_depletion_platform_data ceres_psydepl_pdata = {
	.power_supply = "battery",
	.states = ceres_psydepl_states,
	.num_states = ARRAY_SIZE(ceres_psydepl_states),
	.e0_index = 26,
	.r_const = 141600,
	.vsys_min = 2900000,
	.vcharge = 4200000,
	.ibat_nom = 4300,
	.ibat_lut = ceres_ibat_lut,
	.ocv_lut = ceres_ocv_lut,
	.rbat_lut = ceres_rbat_lut
};
#else

/* Temperature in deci-celcius */
static struct psy_depletion_ibat_lut ceres_ibat_lut[] = {
	{  600, 2350 },
	{  500, 2350 },
	{  250, 2350 },
	{  0, 2350 },
	{ -100, 2350 },
	{ -300,	0 }
};

static struct psy_depletion_rbat_lut ceres_rbat_lut[] = {
	{ 100, 165000 },
	{  90, 142500 },
	{  80, 150000 },
	{  70, 160000 },
	{  60, 147500 },
	{  50, 165000 },
	{  40, 165000 },
	{  30, 172500 },
	{  20, 172500 },
	{  10, 177500 },
	{   0, 152500 }
};

/* measured */
static struct psy_depletion_ocv_lut ceres_ocv_lut[] = {
	{ 100, 4350000 },
	{  93, 4201250 },
	{  90, 4167500 },
	{  88, 4151250 },
	{  85, 4118750 },
	{  82, 4097500 },
	{  80, 4081250 },
	{  77, 4048750 },
	{  75, 4021250 },
	{  72, 4005000 },
	{  70, 3988750 },
	{  68, 3970000 },
	{  65, 3940000 },
	{  62, 3911250 },
	{  60, 3895000 },
	{  58, 3883750 },
	{  55, 3860000 },
	{  52, 3846250 },
	{  50, 3837500 },
	{  48, 3833750 },
	{  45, 3817500 },
	{  40, 3800000 },
	{  35, 3786250 },
	{  30, 3770000 },
	{  25, 3733000 },
	{  20, 3725000 },
	{  15, 3697000 },
	{  10, 3677000 },
	{   5, 3588000 },
	{   0, 3254000 }
};

static struct psy_depletion_platform_data ceres_psydepl_pdata = {
	.power_supply = "battery",
	.states = ceres_psydepl_states,
	.num_states = ARRAY_SIZE(ceres_psydepl_states),
	.e0_index = ARRAY_SIZE(ceres_psydepl_states) - 7,
	/* .r_const = 230000, */
	.r_const = 060000,
	.vsys_min = 3250000,
	/* .vcharge = 4200000, */
	.vcharge = 4350000,
	/* .ibat_nom = 2000, */
	.ibat_nom = 2350,
	.ibat_lut = ceres_ibat_lut,
	.ocv_lut = ceres_ocv_lut,
	.rbat_lut = ceres_rbat_lut
};
#endif

#if defined(CONFIG_MACH_S9321) && CONFIG_MACH_S9321
struct psy_depletion_platform_data * psy_get_pdata(void)
{
	return &ceres_psydepl_pdata;
}
#endif

static struct platform_device ceres_psydepl_device = {
	.name = "psy_depletion",
	.id = -1,
	.dev = { .platform_data = &ceres_psydepl_pdata }
};

void __init ceres_sysedp_psydepl_init(void)
{
	int r;

	r = platform_device_register(&ceres_psydepl_device);
	WARN_ON(r);
}

static struct tegra_sysedp_corecap ceres_sysedp_corecap[] = {
	{  1000, {  700, 192000, 768000 }, {  700, 192000, 768000 } },
	{  2000, {  700, 192000, 768000 }, {  700, 192000, 768000 } },
	{  3000, {  700, 192000, 768000 }, {  700, 268800, 768000 } },
	{  4000, { 1200, 192000, 768000 }, {  700, 346000, 768000 } },
	{  4500, { 1700, 192000, 768000 }, { 1200, 595200, 768000 } },
	{  5000, { 2200, 192000, 768000 }, { 1700, 595200, 768000 } },
	{  5500, { 2700, 192000, 768000 }, { 1700, 710400, 768000 } },
	{  6000, { 3200, 192000, 768000 }, { 2200, 710400, 921600 } },
	{  6500, { 3700, 192000, 768000 }, { 2700, 710400, 921600 } },
	{  7000, { 4200, 192000, 768000 }, { 3200, 710400, 921600 } },
	{  7500, { 4700, 192000, 768000 }, { 3700, 710400, 921600 } },
	{  8000, { 5200, 268800, 921600 }, { 4200, 710400, 921600 } },
	{  8500, { 5500, 595200, 921600 }, { 4700, 710400, 921600 } },
	{  9000, { 6000, 595200, 921600 }, { 5200, 710400, 921600 } },
	{  9500, { 6000, 710400, 921600 }, { 6000, 710400, 921600 } },
	{ 10000, { 6000, 710400, 921600 }, { 6000, 710400, 921600 } },
	{ 11000, { 6000, 710400, 921600 }, { 6000, 710400, 921600 } }	
};

static struct tegra_sysedp_platform_data ceres_sysedp_platdata = {
	.corecap = ceres_sysedp_corecap,
	.corecap_size = ARRAY_SIZE(ceres_sysedp_corecap),
	.core_gain = 100,
	.init_req_watts = 11000,
	.bbc = "bbc"
};

static struct platform_device ceres_sysedp_device = {
	.name = "tegra_sysedp",
	.id = -1,
	.dev = { .platform_data = &ceres_sysedp_platdata }
};

void __init ceres_sysedp_core_init(void)
{
	int r;

	ceres_sysedp_platdata.cpufreq_lim = tegra_get_system_edp_entries(
			&ceres_sysedp_platdata.cpufreq_lim_size);
	if (!ceres_sysedp_platdata.cpufreq_lim) {
		WARN_ON(1);
		return;
	}

	r = platform_device_register(&ceres_sysedp_device);
	WARN_ON(r);
}

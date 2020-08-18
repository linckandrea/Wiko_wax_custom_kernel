/*
 * arch/arm/mach-tegra/board-atlantis-power.c
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
 */

#include <linux/edp.h>
#include <linux/edpdev.h>
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/palmas.h>
#include <linux/power/bq2419x-charger.h>
#include <linux/platform_data/lp8755.h>
#include <linux/pid_thermal_gov.h>
#include <linux/irq.h>
#include <linux/input/drv2603-vibrator.h>
#include <linux/generic_adc_thermal.h>
#include <linux/power/power_supply_extcon.h>

#include <asm/mach-types.h>

#include <mach/iomap.h>
#include <mach/edp.h>
#include <mach/irqs.h>
#include <mach/gpio-tegra.h>

#include "pm.h"
#include "board.h"
#include "board-common.h"
#include "tegra-board-id.h"
#include "board-atlantis.h"
#include "tegra11_soctherm.h"
#include "tegra3_tsensor.h"
#include "board-pmu-defines.h"
#include "devices.h"

#define PMC_CTRL                0x0
#define PMC_CTRL_INTR_LOW       (1 << 17)
#define BOARD_SKU_100		100
#define BOARD_SKU_110		110
#define BOARD_SKU_120		120

/* TPS80036 consumer rails */
static struct regulator_consumer_supply palmas_smps12_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
	REGULATOR_SUPPLY("vdd_cpu_dbg", NULL),
};

static struct regulator_consumer_supply palmas_smps3_supply[] = {
	REGULATOR_SUPPLY("vdd_bb", NULL),
};

static struct regulator_consumer_supply palmas_smps6_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
	REGULATOR_SUPPLY("vdd_core_dbg", NULL),
	REGULATOR_SUPPLY("vdd_core", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("vdd_core", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("vdd_core", "sdhci-tegra.3"),
};

static struct regulator_consumer_supply palmas_smps7_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.3"),
};

static struct regulator_consumer_supply palmas_smps8_supply[] = {
	REGULATOR_SUPPLY("vdd2_lpddr3", NULL),
	REGULATOR_SUPPLY("vddca_lpddr3", NULL),
	REGULATOR_SUPPLY("vrefca_lpddr3", NULL),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.1"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.2"),
	REGULATOR_SUPPLY("vdd_hsic_modem2", NULL),
};

static struct regulator_consumer_supply palmas_smps9_supply[] = {
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.1"),
	REGULATOR_SUPPLY("vddio_cam", "tegra_camera"),
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
	REGULATOR_SUPPLY("vdd_1v8_sdmmc_emmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vdd_ddr1", NULL),
	REGULATOR_SUPPLY("vdd_1v8_hdmi", "tegradc.1"),
	REGULATOR_SUPPLY("vdd_ts", NULL),
	REGULATOR_SUPPLY("vdd_1v8_audio_mb", NULL),
	REGULATOR_SUPPLY("vdd_1v8_uart_mb", NULL),
	REGULATOR_SUPPLY("vdd_1v8_com", NULL),
	REGULATOR_SUPPLY("dvdd", "1-0077"),
	REGULATOR_SUPPLY("vdd_dtv", NULL),
	REGULATOR_SUPPLY("vdd_1v8_bb_mb", NULL),
	REGULATOR_SUPPLY("avdd_1v8_rf", NULL),
	REGULATOR_SUPPLY("vdd_modem2", NULL),
	REGULATOR_SUPPLY("vdd_dbg", NULL),
	REGULATOR_SUPPLY("vdd_1v8_ldo_ddr_hs", NULL),
};

static struct regulator_consumer_supply palmas_ldo1_supply[] = {
	REGULATOR_SUPPLY("unused", NULL),
};

static struct regulator_consumer_supply palmas_ldo2_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
};

static struct regulator_consumer_supply palmas_ldo3_supply[] = {
	REGULATOR_SUPPLY("vddio_sim0", "tegra_bbc_proxy"),
};

static struct regulator_consumer_supply palmas_ldo4_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_m_p", NULL),
	REGULATOR_SUPPLY("avdd_pllc", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_mipi_pllu", NULL),
	REGULATOR_SUPPLY("avdd_ddr_hs", NULL),
};

/* powered by vdd_soc_ldo5 - smps6 or vdd_1v2_ldo5 - smps 8 */
static struct regulator_consumer_supply palmas_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply palmas_ldo6_supply[] = {
	REGULATOR_SUPPLY("avdd_cam1", NULL),
	REGULATOR_SUPPLY("vana", "2-0010"),
	REGULATOR_SUPPLY("avdd_ov5693", "2-0010"),
};

static struct regulator_consumer_supply palmas_ldo7_supply[] = {
	REGULATOR_SUPPLY("vddio_sim1", "tegra_bbc_proxy"),
};

static struct regulator_consumer_supply palmas_ldo8_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
	REGULATOR_SUPPLY("vpp_bb_fuse", "tegra_bbc_proxy"),
};

static struct regulator_consumer_supply palmas_ldo9_supply[] = {
	REGULATOR_SUPPLY("dvdd_bb_pll", NULL),
	REGULATOR_SUPPLY("avdd_bb_pll", NULL),
};

static struct regulator_consumer_supply palmas_ldo10_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
};

static struct regulator_consumer_supply palmas_ldo11_supply[] = {
	REGULATOR_SUPPLY("vdd_mb", NULL),
	REGULATOR_SUPPLY("vdd_temp", NULL),
	REGULATOR_SUPPLY("avdd", "1-0077"),
	REGULATOR_SUPPLY("vdd_irled", NULL),
	REGULATOR_SUPPLY("vdd_sensor_2v8", NULL),
	REGULATOR_SUPPLY("vdd_pm_2v8", NULL),
	REGULATOR_SUPPLY("vdd", "0-0029"),
	REGULATOR_SUPPLY("vdd", "0-004c"),
	REGULATOR_SUPPLY("vdd", "0-0069"),
	REGULATOR_SUPPLY("vdd", "0-000d"),
	REGULATOR_SUPPLY("vdd", "0-0078"),
};

static struct regulator_consumer_supply palmas_ldo12_supply[] = {
	REGULATOR_SUPPLY("vdd_dis_lcd", NULL),
	REGULATOR_SUPPLY("vdd_dis_ts", NULL),
	REGULATOR_SUPPLY("dvdd", "spi2.0"),
	REGULATOR_SUPPLY("vdd_lcd_1v8_s", NULL),
};

static struct regulator_consumer_supply palmas_ldo13_supply[] = {
	REGULATOR_SUPPLY("vdd_cam_1v8", NULL),
	REGULATOR_SUPPLY("vif", "2-0010"),
	REGULATOR_SUPPLY("vif", "2-0036"),
	REGULATOR_SUPPLY("vdd_i2c", "2-000e"),
	REGULATOR_SUPPLY("vi2c", "2-0030"),
	REGULATOR_SUPPLY("vdd", "2-004a"),
	REGULATOR_SUPPLY("dovdd", "2-0010"),
	REGULATOR_SUPPLY("vdd_i2c", "2-000c"),
};

static struct regulator_consumer_supply palmas_ldo14_supply[] = {
	REGULATOR_SUPPLY("avdd_2v8_cam_af", NULL),
	REGULATOR_SUPPLY("avdd_cam2", NULL),
	REGULATOR_SUPPLY("vana", "2-0036"),
	REGULATOR_SUPPLY("vdd", "2-000e"),
	REGULATOR_SUPPLY("vana_imx132", "2-0036"),
	REGULATOR_SUPPLY("af_vdd", "2-0010"),
	REGULATOR_SUPPLY("vdd", "2-000c"),
};

static struct regulator_consumer_supply palmas_ldoln_supply[] = {
	REGULATOR_SUPPLY("avdd_aud", NULL),
};

static struct regulator_consumer_supply palmas_ldousb_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avdd_hdmi", "tegradc.1"),
	REGULATOR_SUPPLY("vdd_sys_dtv_3v3", NULL),
};

static struct regulator_consumer_supply palmas_regen1_supply[] = {
	REGULATOR_SUPPLY("amic_bias_en", NULL),
};

static struct regulator_consumer_supply palmas_regen2_supply[] = {
	REGULATOR_SUPPLY("vdd_cam_1v2", NULL),
	REGULATOR_SUPPLY("vdig", "2-0010"),
	REGULATOR_SUPPLY("vdig", "2-0036"),
	REGULATOR_SUPPLY("dvdd", "2-0010"),
};

static struct regulator_consumer_supply palmas_regen4_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "vi"),
	REGULATOR_SUPPLY("vdd_1v2_lcd", NULL),
	REGULATOR_SUPPLY("vdd_1v2_cdc", NULL),
	REGULATOR_SUPPLY("vrefdq_lpddr3", NULL),
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddq_lpddr3", NULL),
};

static struct regulator_consumer_supply palmas_regen5_supply[] = {
	REGULATOR_SUPPLY("vdd_aud_dgtl", NULL),
	REGULATOR_SUPPLY("vdd_aud_anlg", NULL),
	REGULATOR_SUPPLY("vdd_aud_mic", NULL),
	REGULATOR_SUPPLY("vdd_sw_1v8_snsr", NULL),
	REGULATOR_SUPPLY("vlogic", "0-0069"),
	REGULATOR_SUPPLY("vid", "0-000d"),
	REGULATOR_SUPPLY("vddio", "0-0078"),
};

static struct regulator_consumer_supply palmas_regen7_supply[] = {
	REGULATOR_SUPPLY("avdd_lcd", NULL),
	REGULATOR_SUPPLY("avdd", "spi2.0"),
};

static struct regulator_consumer_supply palmas_chargerpump_supply[] = {
};

PALMAS_REGS_PDATA(smps12, 650,  1300, NULL, 0, 0, 0, 0,
	0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0, 0);
PALMAS_REGS_PDATA(smps3, 1100,  1100, NULL, 0, 0, 0, 0,
	0, PALMAS_EXT_CONTROL_ENABLE2, 0, 10000, 0);
PALMAS_REGS_PDATA(smps6, 650,  1400, NULL, 0, 0, 0, 0,
	0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0, 0);
PALMAS_REGS_PDATA(smps7, 2840,  2860, NULL, 0, 0, 0, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(smps8, 1200,  1200, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(smps9, 1800,  1800, NULL, 1, 1, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo1, 3200,  3200, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo2, 2850,  2850, palmas_rails(smps7), 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo3, 1800,  3000, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo4, 1050,  1050, palmas_rails(smps8), 1, 0, 1, 0,
	0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0, 0);
PALMAS_REGS_PDATA(ldo5, 800,  800, palmas_rails(smps8), 1, 0, 1, 0,
	0, PALMAS_EXT_CONTROL_NSLEEP, 1, 0, 0);
PALMAS_REGS_PDATA(ldo6, 2700,  2700, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo7, 1800,  3000, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo8, 1800,  1800, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo9, 900,  1100, palmas_rails(smps8), 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo10, 1800,  3300, palmas_rails(smps7), 0, 0, 0, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo11, 2800,  2800, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo12, 1800,  1800, palmas_rails(smps9), 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo13, 1800,  1800, palmas_rails(smps9), 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo14, 2800,  2800, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldoln, 2800,  2800, NULL, 0, 0, 0, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldousb, 3300,  3300, NULL, 0, 0, 0, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(regen1, 4300,  4300, NULL, 1, 1, 0, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(regen2, 1200,  1200, palmas_rails(smps8), 0, 0, 0, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(regen4, 1200,  1200, palmas_rails(smps9), 1, 1, 0, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(regen5, 1800,  1800, palmas_rails(smps8), 1, 1, 0, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(regen7, 2800,  2800, NULL, 1, 1, 0, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(chargerpump, 5000,  5000, NULL, 0, 0, 0, 0,
	0, 0, 0, 0, 0);


#define PALMAS_REG_PDATA(_sname) &reg_idata_##_sname

static struct regulator_init_data *atlantis_reg_data[PALMAS_NUM_REGS] = {
	PALMAS_REG_PDATA(smps12),
	NULL,
	PALMAS_REG_PDATA(smps3),
	NULL,
	NULL,
	PALMAS_REG_PDATA(smps6),
	PALMAS_REG_PDATA(smps7),
	PALMAS_REG_PDATA(smps8),
	PALMAS_REG_PDATA(smps9),
	NULL,
	PALMAS_REG_PDATA(ldo1),
	PALMAS_REG_PDATA(ldo2),
	PALMAS_REG_PDATA(ldo3),
	PALMAS_REG_PDATA(ldo4),
	PALMAS_REG_PDATA(ldo5),
	PALMAS_REG_PDATA(ldo6),
	PALMAS_REG_PDATA(ldo7),
	PALMAS_REG_PDATA(ldo8),
	PALMAS_REG_PDATA(ldo9),
	PALMAS_REG_PDATA(ldo10),
	PALMAS_REG_PDATA(ldo11),
	PALMAS_REG_PDATA(ldo12),
	PALMAS_REG_PDATA(ldo13),
	PALMAS_REG_PDATA(ldo14),
	PALMAS_REG_PDATA(ldoln),
	PALMAS_REG_PDATA(ldousb),
	PALMAS_REG_PDATA(regen1),
	PALMAS_REG_PDATA(regen2),
	NULL,
	PALMAS_REG_PDATA(regen4),
	PALMAS_REG_PDATA(regen5),
	PALMAS_REG_PDATA(regen7),
	NULL,
	NULL,
	PALMAS_REG_PDATA(chargerpump),
};

#define PALMAS_REG_INIT_DATA(_sname) &reg_init_data_##_sname

static struct palmas_reg_init *atlantis_reg_init[PALMAS_NUM_REGS] = {
	PALMAS_REG_INIT_DATA(smps12),
	NULL,
	PALMAS_REG_INIT_DATA(smps3),
	NULL,
	NULL,
	PALMAS_REG_INIT_DATA(smps6),
	PALMAS_REG_INIT_DATA(smps7),
	PALMAS_REG_INIT_DATA(smps8),
	PALMAS_REG_INIT_DATA(smps9),
	NULL,
	PALMAS_REG_INIT_DATA(ldo1),
	PALMAS_REG_INIT_DATA(ldo2),
	PALMAS_REG_INIT_DATA(ldo3),
	PALMAS_REG_INIT_DATA(ldo4),
	PALMAS_REG_INIT_DATA(ldo5),
	PALMAS_REG_INIT_DATA(ldo6),
	PALMAS_REG_INIT_DATA(ldo7),
	PALMAS_REG_INIT_DATA(ldo8),
	PALMAS_REG_INIT_DATA(ldo9),
	PALMAS_REG_INIT_DATA(ldo10),
	PALMAS_REG_INIT_DATA(ldo11),
	PALMAS_REG_INIT_DATA(ldo12),
	PALMAS_REG_INIT_DATA(ldo13),
	PALMAS_REG_INIT_DATA(ldo14),
	PALMAS_REG_INIT_DATA(ldoln),
	PALMAS_REG_INIT_DATA(ldousb),
	PALMAS_REG_INIT_DATA(regen1),
	PALMAS_REG_INIT_DATA(regen2),
	NULL,
	PALMAS_REG_INIT_DATA(regen4),
	PALMAS_REG_INIT_DATA(regen5),
	PALMAS_REG_INIT_DATA(regen7),
	NULL,
	NULL,
	PALMAS_REG_INIT_DATA(chargerpump),
};

static struct regulator_consumer_supply fixed_reg_battery_supply[] = {
	REGULATOR_SUPPLY("vdd_sys_cam", NULL),
	REGULATOR_SUPPLY("vdd_sys_bl", NULL),
	REGULATOR_SUPPLY("vdd_sys_com", NULL),
	REGULATOR_SUPPLY("vdd_sys_gps", NULL),
	REGULATOR_SUPPLY("vdd_vbrtr", NULL),
	REGULATOR_SUPPLY("vdd_sys_kb", NULL),
	REGULATOR_SUPPLY("vdd_sys_flash", NULL),
	REGULATOR_SUPPLY("vdd_sys_lcd", NULL),
	REGULATOR_SUPPLY("vdd_sys_cdc", NULL),
	REGULATOR_SUPPLY("vdd_sys_spkr", NULL),
	REGULATOR_SUPPLY("vin", "1-0036"),
	REGULATOR_SUPPLY("vin", "2-0030"),
	REGULATOR_SUPPLY("vin", "2-004a"),
};

static struct regulator_consumer_supply fixed_reg_avdd_lcd_supply[] = {
	REGULATOR_SUPPLY("avdd_lcd", NULL),
	REGULATOR_SUPPLY("avdd", "spi2.0"),
};

static struct regulator_consumer_supply fixed_reg_vdd_hdmi_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_5v0", "tegradc.1"),
};

/* Macro for defining fixed regulator sub device data */
#define FIXED_SUPPLY(_name) "fixed_reg_"#_name
#define FIXED_REG(_id, _var, _name, _in_supply, _always_on, _boot_on,	\
	_gpio_nr, _open_drain, _active_high, _boot_state, _millivolts,	\
	_sdelay)							\
	static struct regulator_init_data ri_data_##_var =		\
	{								\
		.supply_regulator = _in_supply,				\
		.num_consumer_supplies =				\
			ARRAY_SIZE(fixed_reg_##_name##_supply),	\
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
	static struct fixed_voltage_config fixed_reg_##_var##_pdata = \
	{								\
		.supply_name = FIXED_SUPPLY(_name),			\
		.microvolts = _millivolts * 1000,			\
		.gpio = _gpio_nr,					\
		.gpio_is_open_drain = _open_drain,			\
		.enable_high = _active_high,				\
		.enabled_at_boot = _boot_state,				\
		.init_data = &ri_data_##_var,				\
		.startup_delay = _sdelay				\
	};								\
	static struct platform_device fixed_reg_##_var##_dev = {	\
		.name = "reg-fixed-voltage",				\
		.id = _id,						\
		.dev = {						\
			.platform_data = &fixed_reg_##_var##_pdata,	\
		},							\
	}

FIXED_REG(0,    battery,        battery,
	NULL,   0,      0,
	-1,     false, true,    0,      4200,   0);

FIXED_REG(1, avdd_lcd, avdd_lcd,
	palmas_rails(ldo1), 1, 1,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO3,
	false, true, 1, 3200, 0);

FIXED_REG(2, vdd_hdmi_5v0, vdd_hdmi_5v0,
	palmas_rails(chargerpump), 0, 0, PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO9,
	false, true, 0, 5000, 250000);

#define ADD_FIXED_REG(_name)    (&fixed_reg_##_name##_dev)

#define ATLANTIS_COMMON_FIXED_REG	\
	ADD_FIXED_REG(battery),		\
	ADD_FIXED_REG(avdd_lcd),	\
	ADD_FIXED_REG(vdd_hdmi_5v0)

static struct platform_device *fixed_reg_devs_e1670[] = {
	ATLANTIS_COMMON_FIXED_REG
};

int __init atlantis_fixed_regulator_init(void)
{
	struct board_info board_info;
	int num_fixed_regulators = ARRAY_SIZE(fixed_reg_devs_e1670);

	tegra_get_board_info(&board_info);
	if ((board_info.fab == BOARD_FAB_A00) &&
		 (board_info.board_id != BOARD_E1740))
		num_fixed_regulators--;
	return platform_add_devices(fixed_reg_devs_e1670,
					num_fixed_regulators);
}

static struct palmas_dvfs_init_data palmas_dvfs_idata[] = {
	{
		.en_pwm = true,
		.ext_ctrl = PALMAS_EXT_CONTROL_ENABLE2,
		.reg_id = PALMAS_REG_SMPS3,
		.step_20mV = true,
		.base_voltage_uV = 560000,
		.max_voltage_uV = 1200000,
		.smps3_ctrl = true,
	}, {
		.en_pwm = false,
	},
};

static struct palmas_pmic_platform_data pmic_platform = {
	.dvfs_init_data = palmas_dvfs_idata,
	.dvfs_init_data_size = ARRAY_SIZE(palmas_dvfs_idata),
};

static struct palmas_pinctrl_config palmas_pincfg[] = {
	PALMAS_PINMUX(POWERGOOD, POWERGOOD, DEFAULT, DEFAULT),
	PALMAS_PINMUX(VAC, VAC, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO0, ID, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO1, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO2, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO3, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO4, GPIO, PULL_UP, DEFAULT),
	PALMAS_PINMUX(GPIO5, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO6, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO7, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO8, SIM1RSTI, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO9, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO10, GPIO, PULL_UP, DEFAULT),
	PALMAS_PINMUX(GPIO11, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO12, SIM2RSTO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO13, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO14, GPIO, PULL_DOWN, DEFAULT),
	PALMAS_PINMUX(GPIO15, SIM2RSTI, DEFAULT, DEFAULT),
};

static struct palmas_pinctrl_platform_data palmas_pinctrl_pdata = {
	.pincfg = palmas_pincfg,
	.num_pinctrl = ARRAY_SIZE(palmas_pincfg),
	.dvfs1_enable = true,
	.dvfs2_enable = false,
};

static struct palmas_extcon_platform_data palmas_extcon_pdata = {
	.connection_name = "palmas-extcon",
	.enable_vbus_detection = true,
	.enable_id_pin_detection = true,
};

struct palmas_clk32k_init_data atlantis_palmas_clk32k_idata[] = {
	{
		.clk32k_id = PALMAS_CLOCK32KG,
		.enable = true,
	}, {
		.clk32k_id = PALMAS_CLOCK32KG_AUDIO,
		.enable = true,
	},
};

/* OCV Configuration */
static struct ocv_config ocv_cfg = {
	.voltage_diff = 75,
	.current_diff = 30,

	.sleep_enter_current = 60,
	.sleep_enter_samples = 3,

	.sleep_exit_current = 100,
	.sleep_exit_samples = 3,

	.long_sleep_current = 500,
	.ocv_period = 300,
	.relax_period = 600,

	.flat_zone_low = 35,
	.flat_zone_high = 65,

	.max_ocv_discharge = 1300,

	.table = {
		3300, 3603, 3650, 3662, 3700,
		3723, 3734, 3746, 3756, 3769,
		3786, 3807, 3850, 3884, 3916,
		3949, 3990, 4033, 4077, 4129,
		4193
	},
};

/* EDV Configuration */
static struct edv_config edv_cfg = {
	.averaging = true,
	.seq_edv = 5,
	.filter_light = 155,
	.filter_heavy = 199,
	.overload_current = 1000,
	.edv = {
		{3150, 0},
		{3450, 4},
		{3600, 10},
	},
};

/* General Battery Cell Configuration */
static struct cell_config cell_cfg =  {
	.technology = POWER_SUPPLY_TECHNOLOGY_LION,
	.cc_voltage = 4175,
	.cc_current = 250,
	.cc_capacity = 15,
	.seq_cc = 5,

	.cc_polarity = true,
	.cc_out = true,
	.ocv_below_edv1 = false,

	.design_capacity = 4000,
	.design_qmax = 4100,
	.r_sense = 10,

	.qmax_adjust = 1,
	.fcc_adjust = 2,

	.max_overcharge = 100,
	.electronics_load = 200, /* *10 uAh */

	.max_increment = 150,
	.max_decrement = 150,
	.low_temp = 119,
	.deep_dsg_voltage = 30,
	.max_dsg_estimate = 300,
	.light_load = 100,
	.near_full = 500,
	.cycle_threshold = 3500,
	.recharge = 300,

	.mode_switch_capacity = 5,

	.call_period = 2,

	.ocv = &ocv_cfg,
	.edv = &edv_cfg,
};

static struct palmas_battery_platform_data battery_pdata = {
	.therm_zone_name = "battery-temp",
	 /* time in seconds for current average readings */
	.current_avg_interval = 10,

	/* Battery is soldered in with no detection resistor */
	.battery_soldered = 0,

	/* How often we should update the info about charge status */
	.battery_status_interval = 10000,

	/* Battery Configuration for Fuel Guage */
	.cell_cfg = &cell_cfg,
	.gpadc_retry_count = 5,
	.is_battery_present = false,
	.enable_ovc_alarm = true,
	.ovc_period = 3900,
	.ovc_threshold = 3000
};

static struct iio_map palmas_iio_map[] = {
	PALMAS_GPADC_IIO_MAP(IN1, "generic-adc-thermal.0", "battery-temp-channel"),
	PALMAS_GPADC_IIO_MAP(IN6, "palmas-battery", "vbat_channel"),
	PALMAS_GPADC_IIO_MAP(NULL, NULL, NULL),
};

struct palmas_gpadc_platform_data gpadc_pdata = {
	.channel0_current_uA = 0,
	.channel3_current_uA = 0,
	.iio_maps = palmas_iio_map,
};

static struct regulator_consumer_supply palmas_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.0"),
	REGULATOR_SUPPLY("usb_vbus", "tegra-otg"),
};

static struct regulator_consumer_supply palmas_batt_supply[] = {
	REGULATOR_SUPPLY("usb_bat_chg", "tegra-udc.0"),
};

static struct palmas_vbus_platform_data palmas_vbus_pdata = {
	.num_consumer_supplies = ARRAY_SIZE(palmas_vbus_supply),
	.consumer_supplies = palmas_vbus_supply,
};

struct palmas_bcharger_platform_data palmas_bcharger_pdata = {
	.battery_tz_name = "battery-temp",
	.max_charge_current_mA = 3000,
	.charging_term_current_mA = 100,
	.consumer_supplies = palmas_batt_supply,
	.num_consumer_supplies = ARRAY_SIZE(palmas_batt_supply),
	.wdt_timeout    = 40,
	.rtc_alarm_time = 3600,
	.chg_restart_time = 1800,
	.temperature_poll_period_secs = 5,
};

static struct power_supply_extcon_plat_data extcon_pdata = {
	.extcon_name = "tegra-udc",
};

static struct platform_device power_supply_extcon_device = {
	.name           = "power-supply-extcon",
	.id             = -1,
	.dev            = {
		.platform_data = &extcon_pdata,
	},
};

struct palmas_charger_platform_data palmas_charger_pdata = {
	.vbus_pdata = &palmas_vbus_pdata,
};

static struct palmas_sim_platform_data sim_platform = {
	.dbcnt = 0x10,
	.pwrdncnt = 0x10,
	.pwrdnen1 = 0,
	.pwrdnen2 = 0,
	.det_polarity = 0,
	.det1_pu = 1,
	.det1_pd = 0,
	.det2_pu = 1,
	.det2_pd = 0,
};

static struct palmas_platform_data palmas_pdata = {
	.gpio_base = PALMAS_TEGRA_GPIO_BASE,
	.irq_base = PALMAS_TEGRA_IRQ_BASE,
	.pmic_pdata = &pmic_platform,
	.clk32k_init_data =  atlantis_palmas_clk32k_idata,
	.clk32k_init_data_size = ARRAY_SIZE(atlantis_palmas_clk32k_idata),
	.sim_pdata = &sim_platform,
	.irq_type = IRQ_TYPE_LEVEL_HIGH,
	.use_power_off = true,
	.watchdog_timer_initial_period = 128,
	.pinctrl_pdata = &palmas_pinctrl_pdata,
	.extcon_pdata = &palmas_extcon_pdata,
	.charger_pdata = &palmas_charger_pdata,
	.adc_pdata = &gpadc_pdata,
	.battery_pdata = &battery_pdata,
};


/* -40 to 119 degC */
static int atlanties_ers_batt_temperature_table[] = {
	277, 284, 291, 298, 305, 313, 321, 329,
	337, 345, 354, 363, 372, 381, 390, 400,
	410, 420, 431, 442, 453, 464, 476, 487,
	500, 512, 525, 538, 551, 565, 579, 593,
	607, 622, 637, 653, 669, 685, 702, 718,
	736, 753, 771, 789, 808, 827, 846, 865,
	885, 906, 926, 947, 968, 990, 1011, 1033,
	1056, 1078, 1101, 1124, 1148, 1171, 1195, 1219,
	1243, 1268, 1292, 1317, 1342, 1367, 1392, 1417,
	1442, 1467, 1493, 1518, 1543, 1568, 1594, 1619,
	1644, 1669, 1693, 1718, 1742, 1767, 1791, 1814,
	1838, 1861, 1884, 1907, 1930, 1952, 1973, 1995,
	2016, 2036, 2057, 2077, 2096, 2115, 2134, 2152,
	2170, 2187, 2204, 2221, 2237, 2253, 2268, 2282,
	2297, 2311, 2324, 2337, 2350, 2362, 2374, 2385,
	2396, 2407, 2417, 2427, 2436, 2445, 2454, 2462,
	2471, 2478, 2486, 2493, 2500, 2506, 2512, 2518,
	2524, 2529, 2534, 2539, 2544, 2548, 2553, 2557,
	2561, 2564, 2568, 2571, 2574, 2577, 2580, 2582,
	2585, 2587, 2590, 2592, 2594, 2595, 2597, 2599,
};

/* -40 to 125 degC */
static int atlantis_ffd_batt_temperature_table[] = {
	259, 266, 272, 279, 286, 293, 301, 308,
	316, 324, 332, 340, 349, 358, 367, 376,
	386, 395, 405, 416, 426, 437, 448, 459,
	471, 483, 495, 508, 520, 533, 547, 561,
	575, 589, 604, 619, 634, 650, 666, 682,
	699, 716, 733, 751, 769, 787, 806, 825,
	845, 865, 885, 905, 926, 947, 969, 990,
	1013, 1035, 1058, 1081, 1104, 1127, 1151, 1175,
	1199, 1224, 1249, 1273, 1298, 1324, 1349, 1374,
	1400, 1426, 1451, 1477, 1503, 1529, 1554, 1580,
	1606, 1631, 1657, 1682, 1707, 1732, 1757, 1782,
	1807, 1831, 1855, 1878, 1902, 1925, 1948, 1970,
	1992, 2014, 2036, 2057, 2077, 2097, 2117, 2136,
	2155, 2174, 2192, 2209, 2227, 2243, 2259, 2275,
	2291, 2305, 2320, 2334, 2347, 2361, 2373, 2386,
	2397, 2409, 2420, 2430, 2441, 2450, 2460, 2469,
	2478, 2486, 2494, 2502, 2509, 2516, 2523, 2529,
	2535, 2541, 2547, 2552, 2557, 2562, 2567, 2571,
	2575, 2579, 2583, 2587, 2590, 2593, 2596, 2599,
	2602, 2605, 2607, 2609, 2611, 2614, 2615, 2617,
	2619, 2621, 2622, 2624, 2625, 2626,
};

static struct gadc_thermal_platform_data gadc_thermal_battery_pdata = {
	.iio_channel_name = "battery-temp-channel",
	.tz_name = "battery-temp",
	.temp_offset = 0,
	.adc_to_temp = NULL,
	.adc_temp_lookup = atlantis_ffd_batt_temperature_table,
	.lookup_table_size = ARRAY_SIZE(atlantis_ffd_batt_temperature_table),
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

static struct i2c_board_info palma_device[] = {
	{
		I2C_BOARD_INFO("tps80036", 0x58),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &palmas_pdata,
	},
};

/* LP8755 DC-DC converter */
static struct regulator_consumer_supply lp8755_buck0_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

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

static struct drv2603_platform_data atlantis_vibrator_pdata = {
	.pwm_id = 0,
	.vibrator_mode = ERM_MODE,
	.gpio = PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO6,
	.duty_cycle = 80,
	.edp_states = {198, 0},
};

static struct platform_device atlantis_vibrator_device = {
	.name = "drv2603-vibrator",
	.id = -1,
	.dev = {
		.platform_data = &atlantis_vibrator_pdata,
	},
};

int atlantis_pwm_init(void)
{
	return platform_device_register(&tegra_pwfm0_device);
}

int atlantis_vibrator_init(void)
{
	return platform_device_register(&atlantis_vibrator_device);
}

int __init atlantis_regulator_init(void)
{
	struct board_info board_info;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
	int i;
	bool wdt_disable;

	/* configure the power management controller to trigger PMU
	 * interrupts when high */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl & ~PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	tegra_get_board_info(&board_info);
	for (i = 0; i < PALMAS_NUM_REGS ; i++) {
		pmic_platform.reg_data[i] = atlantis_reg_data[i];
		pmic_platform.reg_init[i] = atlantis_reg_init[i];
	}

	reg_init_data_ldo5.tracking_regulator = PALMAS_REG_SMPS6;
	/* Enable full constraints to disable unused rails */
	regulator_has_full_constraints();

	/* Set LDO11 startup time to 600us */
	reg_idata_ldo11.constraints.startup_delay = 600;

	if (((board_info.fab != BOARD_FAB_A00) ||
		 (board_info.board_id == BOARD_E1740)))
			reg_idata_regen7.num_consumer_supplies = 0;

	if (board_info.sku == BOARD_SKU_110) {
		pmic_platform.reg_data[PALMAS_REG_SMPS12] = NULL;
		pmic_platform.reg_init[PALMAS_REG_SMPS12] = NULL;

		lp8755_regulator_init();
	} else if (board_info.sku == BOARD_SKU_100 ||
		 board_info.board_id == BOARD_E1740) {
		reg_init_data_smps12.roof_floor = PALMAS_EXT_CONTROL_ENABLE1;
	} else if (board_info.sku == BOARD_SKU_120) {
		pmic_platform.reg_data[PALMAS_REG_SMPS12] =
				atlantis_reg_data[PALMAS_REG_SMPS6];
		pmic_platform.reg_data[PALMAS_REG_SMPS12]->constraints.name =
				palmas_rails(smps12);
		pmic_platform.reg_init[PALMAS_REG_SMPS12] =
				atlantis_reg_init[PALMAS_REG_SMPS6];
		pmic_platform.reg_data[PALMAS_REG_SMPS6] = NULL;
		pmic_platform.reg_init[PALMAS_REG_SMPS6] = NULL;
		reg_init_data_ldo5.config_flags =
				PALMAS_REGULATOR_CONFIG_TRACKING_ENABLE;
		reg_init_data_ldo5.tracking_regulator = PALMAS_REG_SMPS12;

		lp8755_regulator_init();
	}

	platform_device_register(&power_supply_extcon_device);
	if (board_info.board_id == BOARD_E1670) {
		gadc_thermal_battery_pdata.adc_temp_lookup =
				atlanties_ers_batt_temperature_table;
		gadc_thermal_battery_pdata.lookup_table_size =
				ARRAY_SIZE(atlanties_ers_batt_temperature_table);
		gadc_thermal_battery_pdata.first_index_temp = 119;
		gadc_thermal_battery_pdata.last_index_temp = -40;
	} else if (board_info.board_id == BOARD_E1740) {
		gadc_thermal_battery_pdata.adc_temp_lookup = atlantis_ffd_batt_temperature_table;
		gadc_thermal_battery_pdata.lookup_table_size = ARRAY_SIZE(atlantis_ffd_batt_temperature_table);
		gadc_thermal_battery_pdata.first_index_temp = 125;
		gadc_thermal_battery_pdata.last_index_temp = -40;
	}

	if (get_power_supply_type() == POWER_SUPPLY_TYPE_BATTERY) {
		palmas_pdata.battery_pdata->is_battery_present = true;
		palmas_pdata.charger_pdata->bcharger_pdata =
						&palmas_bcharger_pdata;
		platform_device_register(&gadc_thermal_battery);
	}

	wdt_disable = is_pmic_wdt_disabled_at_boot();
	if (wdt_disable)
		palmas_pdata.watchdog_timer_initial_period = 0;

	i2c_register_board_info(4, palma_device,
			ARRAY_SIZE(palma_device));

	return 0;
}

static unsigned int atlantis_psydepl_states[] = {
	9900, 9600, 9300, 9000, 8700, 8400, 8100, 7800,
	7500, 7200, 6900, 6600, 6300, 6000, 5800, 5600,
	5400, 5200, 5000, 4800, 4600, 4400, 4200, 4000,
	3800, 3600, 3400, 3200, 3000, 2800, 2600, 2400,
	2200, 2000, 1900, 1800, 1700, 1600, 1500, 1400,
	1300, 1200, 1100, 1000,  900,  800,  700,  600,
	 500,  400,  300,  200,  100,    0
};

/* Temperature in deci-celcius */
static struct psy_depletion_ibat_lut atlantis_ibat_lut[] = {
	{  900, 3000 },
	{  400, 3000 },
	{    0, 3000 },
	{ -300,    0 }
};

static struct psy_depletion_rbat_lut atlantis_rbat_lut[] = {
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

static struct psy_depletion_ocv_lut atlantis_ocv_lut[] = {
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

static struct psy_depletion_platform_data atlantis_psydepl_pdata = {
	.power_supply = "palmas-battery",
	.states = atlantis_psydepl_states,
	.num_states = ARRAY_SIZE(atlantis_psydepl_states),
	.e0_index = 26,
	.r_const = 140600,
	.vsys_min = 3250000,
	.vcharge = 4200000,
	.ibat_nom = 3000,
	.ibat_lut = atlantis_ibat_lut,
	.rbat_lut = atlantis_rbat_lut,
	.ocv_lut = atlantis_ocv_lut
};

static struct platform_device atlantis_psydepl_device = {
	.name = "psy_depletion",
	.id = -1,
	.dev = { .platform_data = &atlantis_psydepl_pdata }
};

void __init atlantis_sysedp_psydepl_init(void)
{
	int r;

	r = platform_device_register(&atlantis_psydepl_device);
	WARN_ON(r);
}

static struct tegra_tsensor_pmu_data tpdata_palmas = {
	.reset_tegra = 1,
	.pmu_16bit_ops = 0,
	.controller_type = 0,
	.pmu_i2c_addr = 0x58,
	.i2c_controller_id = 4,
	.poweroff_reg_addr = 0xa0,
	.poweroff_reg_data = 0x0,
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

/* atlantis has POP package, for which the trip points are different */
static struct soctherm_platform_data atlantis_soctherm_data = {
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 80000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 92000,
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
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 82000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 92000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 94000,
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
	.tshut_pmu_trip_data = &tpdata_palmas,
};

int __init atlantis_soctherm_init(void)
{
	struct board_info board_info;

	/* atlantis ERS and FFD */
	tegra_get_board_info(&board_info);
	if (!((board_info.board_id == BOARD_E1670) ||
		(board_info.board_id == BOARD_E1740)))
		return -EINVAL;

	tegra_platform_edp_init(atlantis_soctherm_data.therm[THERM_CPU].trips,
				&atlantis_soctherm_data.therm[THERM_CPU].num_trips,
				14000); /* edp temperature margin */
	tegra_add_tj_trips(atlantis_soctherm_data.therm[THERM_CPU].trips,
			   &atlantis_soctherm_data.therm[THERM_CPU].num_trips);

	return tegra11_soctherm_init(&atlantis_soctherm_data);
}

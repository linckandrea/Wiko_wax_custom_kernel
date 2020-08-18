/*
 * arch/arm/mach-tegra/board-e1853-power.c
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

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps6591x.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6591x-regulator.h>
#include <linux/delay.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "gpio-names.h"
#include "board-e1853.h"
#include "pm.h"
#include "wakeups-t3.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)
#define CPU_SOFTRST_CTRL		0x380

#define CLK_RST_CONTROLLER_CLK_OUT_ENB_H	0x14
#define CLK_RST_CONTROLLER_CLK_SOURCE_DVC_I2C	0x128
#define CLK_RST_CONTROLLER_RST_DEV_H_CLR	0x08
#define I2C_CNFG				0x0
#define I2C_CMD_ADDR0			0x4
#define I2C_CMD_ADDR1			0x8
#define I2C_CMD_DATA1			0xc
#define I2C_CMD_DATA2			0x10
#define I2C_STATUS			0x1c
#define I2C_INT				0x074
static void __iomem *reg_clk_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
static void __iomem *reg_i2c5_base = IO_ADDRESS(TEGRA_I2C5_BASE);

#define clk_writel(value, reg) \
	__raw_writel(value, (u32)reg_clk_base + (reg))

static struct regulator_consumer_supply tps6591x_vdd1_supply_0[] = {
	REGULATOR_SUPPLY("vdd_pexb", NULL),
	REGULATOR_SUPPLY("avdd_plle", NULL),
	REGULATOR_SUPPLY("avdd_sata", NULL),
	REGULATOR_SUPPLY("vdd_sata", NULL),
	REGULATOR_SUPPLY("avdd_sata_pll", NULL),
	REGULATOR_SUPPLY("avdd_pexb", NULL),
	REGULATOR_SUPPLY("avdd_pex_pll", NULL),
	REGULATOR_SUPPLY("avdd_pexa", NULL),
	REGULATOR_SUPPLY("vdd_pexa", NULL),
};

static struct regulator_consumer_supply tps6591x_vdd2_supply_0[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
};

static struct regulator_consumer_supply tps6591x_vddctrl_supply_0[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply tps6591x_vio_supply_0[] = {
	REGULATOR_SUPPLY("vddio_sdmmc4", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("vddio_vi", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("ldo1", NULL),
	REGULATOR_SUPPLY("ldo2", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo1_supply_0[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo2_supply_0[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d2", NULL),
	REGULATOR_SUPPLY("avdd_dsi_csi", "vi"),
};

static struct regulator_consumer_supply tps6591x_ldo3_supply_0[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo4_supply_0[] = {
	REGULATOR_SUPPLY("vdd_1v8_sys", NULL),
	REGULATOR_SUPPLY("vmmc", "sdhci-tegra.1"),
};

static struct regulator_consumer_supply tps6591x_ldo5_supply_0[] = {
	REGULATOR_SUPPLY("vdd_fuse", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo6_supply_0[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("vmmc", "sdhci-tegra.0"),
};

static struct regulator_consumer_supply tps6591x_ldo7_supply_0[] = {
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo8_supply_0[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
};

#define TPS_PDATA_INIT(_name, _sname, _minmv, _maxmv, _supply_reg, _always_on, \
		_ectrl, _flags) \
static struct tps6591x_regulator_platform_data pdata_##_name##_##_sname = \
{									\
	.regulator = {							\
		.constraints = {					\
			.min_uV = (_minmv)*1000,			\
			.max_uV = (_maxmv)*1000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,			\
			.boot_on = 0,					\
			.apply_uV = 0,					\
		},							\
		.num_consumer_supplies =				\
			ARRAY_SIZE(tps6591x_##_name##_supply_##_sname), \
		.consumer_supplies = tps6591x_##_name##_supply_##_sname,\
		.supply_regulator = _supply_reg,			\
	},								\
	.init_uV =  -1 * 1000,						\
	.init_enable = 0,						\
	.init_apply = 0,						\
	.ectrl = _ectrl,						\
	.flags = _flags,						\
}

TPS_PDATA_INIT(vdd1, 0, 600,  1500, 0, 0, EXT_CTRL_SLEEP_OFF, 0);
TPS_PDATA_INIT(vdd2, 0, 600,  1500, 0, 1, 0, 0);
TPS_PDATA_INIT(vddctrl, 0, 600,  1400, 0, 1, EXT_CTRL_EN1, 0);
TPS_PDATA_INIT(vio,  0, 1500, 3300, 0, 1, EXT_CTRL_SLEEP_OFF, 0);

TPS_PDATA_INIT(ldo1, 0, 1000, 1000, 0, 1, EXT_CTRL_SLEEP_OFF, 0);
TPS_PDATA_INIT(ldo2, 0, 1200, 1200, 0, 1, EXT_CTRL_SLEEP_OFF, 0);

TPS_PDATA_INIT(ldo3, 0, 1000, 3300, 0, 1, 0, 0);
TPS_PDATA_INIT(ldo4, 0, 1000, 3300, 0, 1, 0, 0);
TPS_PDATA_INIT(ldo5, 0, 1000, 3300, 0, 0, EXT_CTRL_SLEEP_OFF, 0);

TPS_PDATA_INIT(ldo6, 0, 1000, 3300, 0, 1, EXT_CTRL_SLEEP_OFF, 0);
TPS_PDATA_INIT(ldo7, 0, 1000, 3300, 0, 1, EXT_CTRL_SLEEP_OFF, 0);
TPS_PDATA_INIT(ldo8, 0, 1000, 3300, 0, 0, EXT_CTRL_SLEEP_OFF, 0);

#define TPS_REG(_id, _name, _sname)			\
{							\
	.id	= TPS6591X_ID_##_id,			\
	.name	= "tps6591x-regulator",			\
	.platform_data	= &pdata_##_name##_##_sname,	\
}

static struct tps6591x_subdev_info tps_devs_e1853[] = {
	TPS_REG(VIO, vio, 0),
	TPS_REG(VDD_1, vdd1, 0),
	TPS_REG(VDD_2, vdd2, 0),
	TPS_REG(VDDCTRL, vddctrl, 0),
	TPS_REG(LDO_1, ldo1, 0),
	TPS_REG(LDO_2, ldo2, 0),
	TPS_REG(LDO_3, ldo3, 0),
	TPS_REG(LDO_4, ldo4, 0),
	TPS_REG(LDO_5, ldo5, 0),
	TPS_REG(LDO_6, ldo6, 0),
	TPS_REG(LDO_7, ldo7, 0),
	TPS_REG(LDO_8, ldo8, 0)
};

#define TPS_GPIO_INIT_PDATA(gpio_nr, _init_apply, _sleep_en, _pulldn_en,\
		_output_en, _output_val)	\
	[gpio_nr] = {				\
		.sleep_en	= _sleep_en,	\
		.pulldn_en	= _pulldn_en,	\
		.output_mode_en	= _output_en,	\
		.output_val	= _output_val,	\
		.init_apply	= _init_apply,	\
}

static struct tps6591x_gpio_init_data tps_gpio_pdata[] =  {
	TPS_GPIO_INIT_PDATA(2, 1, 0, 0, 1, 1),
	TPS_GPIO_INIT_PDATA(6, 1, 0, 0, 1, 1),
};

static struct tps6591x_sleep_keepon_data tps_slp_keepon = {
	.clkout32k_keepon = 1,
};

static struct tps6591x_platform_data tps_platform = {
	.irq_base	= TPS6591X_IRQ_BASE,
	.gpio_base	= TPS6591X_GPIO_BASE,
	.dev_slp_en	= true,
	.slp_keepon	= &tps_slp_keepon,
	.use_power_off	= true,
	.dev_slp_delayed = true,
};

static struct i2c_board_info __initdata e1853_regulators[] = {
	{
		I2C_BOARD_INFO("tps6591x", 0x2D),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

int __init e1853_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	tps_platform.num_subdevs =
			ARRAY_SIZE(tps_devs_e1853);
	tps_platform.subdevs = tps_devs_e1853;

	tps_platform.dev_slp_en = true;
	tps_platform.gpio_init_data = tps_gpio_pdata;
	tps_platform.num_gpioinit_data =
		ARRAY_SIZE(tps_gpio_pdata);

	i2c_register_board_info(4, e1853_regulators, 1);
	return 0;
}

/* EN_3V3_TEGRA From PMU GP2 */
static struct regulator_consumer_supply fixed_reg_en_3v3_supply[] = {
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-udc.0"),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("AVDD", NULL),
	REGULATOR_SUPPLY("HPVDD", NULL),
	REGULATOR_SUPPLY("DCVDD", NULL),
	REGULATOR_SUPPLY("DBVDD", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("hvdd_sata", NULL),
	REGULATOR_SUPPLY("hvdd_pex", NULL),
	REGULATOR_SUPPLY("vddio_pex_ctl", NULL),
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("ldo6", NULL),
	REGULATOR_SUPPLY("ldo7", NULL),
	REGULATOR_SUPPLY("ldo8", NULL),
};

/* Macro for defining fixed regulator sub device data */
#define FIXED_SUPPLY(_name) "fixed_reg_"#_name
#define FIXED_REG_OD(_id, _var, _name, _in_supply, _always_on,		\
		_boot_on, _gpio_nr, _active_high, _boot_state,		\
		_millivolts, _od_state)					\
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
		.enable_high = _active_high,				\
		.enabled_at_boot = _boot_state,				\
		.init_data = &ri_data_##_var,				\
		.gpio_is_open_drain = _od_state,			\
	};								\
	static struct platform_device fixed_reg_##_var##_dev = {	\
		.name   = "reg-fixed-voltage",				\
		.id     = _id,						\
		.dev    = {						\
			.platform_data = &fixed_reg_##_var##_pdata,	\
		},							\
	}

#define FIXED_REG(_id, _var, _name, _in_supply, _always_on, _boot_on,	\
		 _gpio_nr, _active_high, _boot_state, _millivolts)	\
	FIXED_REG_OD(_id, _var, _name, _in_supply, _always_on, _boot_on,  \
		_gpio_nr, _active_high, _boot_state, _millivolts, false)

/* common to most of boards*/
FIXED_REG(0, en_3v3, en_3v3, NULL, 1, 1, TPS6591X_GPIO_2, true, 1, 3300);

#define ADD_FIXED_REG(_name)	(&fixed_reg_##_name##_dev)

static struct platform_device *fixed_reg_devs_e1853[] = {
	ADD_FIXED_REG(en_3v3),
};

int __init e1853_fixed_regulator_init(void)
{
	struct platform_device **fixed_reg_devs;
	int    nfixreg_devs;
	fixed_reg_devs = fixed_reg_devs_e1853;
	nfixreg_devs = ARRAY_SIZE(fixed_reg_devs_e1853);
	return platform_add_devices(fixed_reg_devs, nfixreg_devs);
}
subsys_initcall_sync(e1853_fixed_regulator_init);

static int I2cStatusPoll(void)
{
	int status = 0;
	int timeout = 0;

#define I2C_POLL_TIMEOUT_USEC   (10 * 1000)
#define I2C_POLL_STEP_USEC  250
	while (timeout < I2C_POLL_TIMEOUT_USEC) {
		status = readl(reg_i2c5_base + I2C_STATUS);
		if (status)
			udelay(I2C_POLL_STEP_USEC);
		else
			return 0;

		timeout += I2C_POLL_STEP_USEC;
	}
	return -1;
#undef  I2C_POLL_STEP_USEC
#undef  I2C_POLL_TIMEOUT_USEC
}

/* Addr -> device addr in 8bit, offset -> reg offset, data -> 1 byte data */
static int I2CWrite(u32 Addr, u8 Offset, u8 Data)
{
	u32 OffsetData = 0;
	/* Pack offset and data as it go to same controller's reg */
	OffsetData = Data;
	OffsetData = (OffsetData << 8) | Offset;
	writel(Addr, reg_i2c5_base + I2C_CMD_ADDR0);
	udelay(10);
	writel(0x82, reg_i2c5_base + I2C_CNFG);
	writel(OffsetData, reg_i2c5_base + I2C_CMD_DATA1);
	udelay(10);
	readl(reg_i2c5_base + I2C_CNFG);
	writel(0xA82, reg_i2c5_base + I2C_CNFG);
	return I2cStatusPoll();
}

#if 0
/* Addr -> device addr in 8bit, offset-> 1st byte (LSB) offset */
static int  I2CRead(u32 Addr, u32 Offset, u8 *Data)
{
	int e;
	*Data = 0;
	writel((Addr & 0xFE), reg_i2c5_base + I2C_CMD_ADDR0);
	writel((Addr | 0x1), reg_i2c5_base + I2C_CMD_ADDR1);
	writel(Offset, reg_i2c5_base + I2C_CMD_DATA1);
	writel(0x290, reg_i2c5_base + I2C_CNFG);
	e = I2cStatusPoll();
	if (!e)
		*Data = readl(reg_i2c5_base + I2C_CMD_DATA2);
	return e;
}
#endif

static void configure_i2c_clk(void)
{
	u32 reg;

	/* Disable the DVC I2C Module */
	reg = readl(reg_clk_base + CLK_RST_CONTROLLER_RST_DEV_H_CLR);
	reg |= 1 << 15;
	writel(reg, reg_clk_base + CLK_RST_CONTROLLER_RST_DEV_H_CLR);

	/* Disable DVC I2C Module clock */
	reg = readl(reg_clk_base + CLK_RST_CONTROLLER_CLK_OUT_ENB_H);
	reg &= ~(1 << 15);
	writel(reg, reg_clk_base + CLK_RST_CONTROLLER_CLK_OUT_ENB_H);

	udelay(10);

	/* Enable DVC I2C Module clock */
	reg = readl(reg_clk_base + CLK_RST_CONTROLLER_CLK_OUT_ENB_H);
	reg |= 1 << 15;
	writel(reg, reg_clk_base + CLK_RST_CONTROLLER_CLK_OUT_ENB_H);

	/* Set the divider to 16, so that we run at 705KHz (and internal
	 * divider of 8), so we are effective running at 100KHz as
	 * requested by Syseng
	 */
	udelay(10);
	reg = readl(reg_clk_base + CLK_RST_CONTROLLER_CLK_SOURCE_DVC_I2C);
	reg = 16;
	reg |= 3 << 30;
	writel(reg, reg_clk_base +
			CLK_RST_CONTROLLER_CLK_SOURCE_DVC_I2C);

	/* Stablization delay of 10 usec */
	udelay(10);

	/* ENABLE the DVC I2C Module */
	reg = readl(reg_clk_base + CLK_RST_CONTROLLER_RST_DEV_H_CLR);
	reg &= ~(1 << 15);
	writel(reg, reg_clk_base + CLK_RST_CONTROLLER_RST_DEV_H_CLR);
	udelay(10);

}


static void e1853_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
	else if ((lp_state == TEGRA_SUSPEND_LP0) && (stg == TEGRA_SUSPEND_BEFORE_CPU)) {
		configure_i2c_clk();

		/* Program PMU-GPIO6, disable output */
		I2CWrite(0x5A, 0x66, 0x86);
		udelay(1000);
		/* Program PMU-GPIO2, disable output */
		I2CWrite(0x5A, 0x62, 0x86);
		udelay(1000);
	}
}

static void e1853_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
	else if ((lp_state == TEGRA_SUSPEND_LP0)  && (stg == TEGRA_RESUME_AFTER_CPU)) {
		configure_i2c_clk();

		/* Program PMU-GPIO2, enable output */
		I2CWrite(0x5A, 0x62, 0x87);
		udelay(1000);
		/* Program PMU-GPIO6, enable output */
		I2CWrite(0x5A, 0x66, 0x87);
		udelay(1000);
	}
}

static struct tegra_suspend_platform_data e1853_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= false,
	.cpu_lp2_min_residency = 2000,
	.board_suspend = e1853_board_suspend,
	.board_resume = e1853_board_resume,
};

int __init e1853_suspend_init(void)
{
	/* FIXME Get correct value from sys-eng */
	clk_writel(0x8040, CPU_SOFTRST_CTRL);
	tegra_init_suspend(&e1853_suspend_data);
	return 0;
}

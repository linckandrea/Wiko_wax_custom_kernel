/*
 * arch/arm/mach-tegra/board-p1852-power.c
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
#include <linux/io.h>
#include <linux/regulator/tps6591x-regulator.h>

#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps6591x.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/regulator/tps6591x-regulator.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "board-p1852.h"
#include "pm.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)
#define CPU_SOFTRST_CTRL		0x380

static void __iomem *reg_clk_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);

#define clk_writel(value, reg) \
	__raw_writel(value, (u32)reg_clk_base + (reg))

static struct regulator_consumer_supply tps6591x_vdd1_supply_0[] = {
	REGULATOR_SUPPLY("vdd_pexb", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo1_supply_0[] = {
	REGULATOR_SUPPLY("avdd_plle", NULL),
};

#define TPS_PDATA_INIT(_name, _sname, _minmv, _maxmv, _supply_reg, _always_on, \
	_boot_on, _apply_uv, _init_mV, _init_enable, _init_apply) \
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
			.boot_on = _boot_on,				\
			.apply_uV = _apply_uv,				\
		},							\
		.num_consumer_supplies =				\
			ARRAY_SIZE(tps6591x_##_name##_supply_##_sname),	\
		.consumer_supplies = tps6591x_##_name##_supply_##_sname, \
		.supply_regulator = _supply_reg,			\
	},								\
	.init_uV =  _init_mV * 1000,					\
	.init_enable = _init_enable,					\
	.init_apply = _init_apply,					\
	.ectrl = 0,							\
	.flags = 0,							\
}


TPS_PDATA_INIT(vdd1, 0, 600,  1500, 0, 0, 0, 0, -1, 0, 0);
TPS_PDATA_INIT(ldo1, 0, 1000, 3300, 0, 0, 0, 0, -1, 0, 0);

#define TPS_REG(_id, _name, _sname)			\
{							\
	.id	= TPS6591X_ID_##_id,			\
	.name	= "tps6591x-regulator",			\
	.platform_data	= &pdata_##_name##_##_sname,	\
}

static struct tps6591x_subdev_info tps_devs_p1852[] = {
	TPS_REG(VDD_1, vdd1, 0),
	TPS_REG(LDO_1, ldo1, 0),
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
};

static struct i2c_board_info __initdata p1852_regulators[] = {
	{
		I2C_BOARD_INFO("tps6591x", 0x2D),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

int __init p1852_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	tps_platform.num_subdevs =
			ARRAY_SIZE(tps_devs_p1852);
	tps_platform.subdevs = tps_devs_p1852;
	tps_platform.gpio_init_data = NULL;
	tps_platform.num_gpioinit_data = 0;

	i2c_register_board_info(4, p1852_regulators, 1);
	return 0;
}

static void p1852_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == 1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void p1852_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == 1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data p1852_suspend_data = {
	/*  FIXME: This value needs to come from SysEng  */
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP1,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= false,
	.cpu_lp2_min_residency = 2000,
	.board_suspend = p1852_board_suspend,
	.board_resume = p1852_board_resume,
};

int __init p1852_suspend_init(void)
{
	/* FIXME Get correct value from sys-eng */
	clk_writel(0x8040, CPU_SOFTRST_CTRL);
	tegra_init_suspend(&p1852_suspend_data);
	return 0;
}

/*
 * arch/arm/mach-tegra/board-s8515-powermon.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/ina219.h>
#include <linux/platform_data/ina230.h>
#include <linux/i2c/pca954x.h>

#include "board.h"
#include "board-s8515.h"
#include "tegra-board-id.h"

#define PRECISION_MULTIPLIER_CERES	1000

enum {
	UNUSED_RAIL,
};

/* following rails are present on Ceres-ERS */
enum {
	VDD_SYS_CELL,
	VDD_SYS_BUCK1,
	VDD_SOC,
	VDD_SYS_BUCK2,
	VDD_CPU_AP,
	VDD_1V8_BUCK5,
	VDD_SYS_BUCK3,
	VDD_SW_1V2_DSI_CSI_AP,
};

enum {
	VDD_SYS_BL,
	AVDD_DIS_LDO4,
};

/* following rails are present on Ceres-ERS A03 on i2c2_1 */
enum {
	VDD_SYS_CELL_A03,
	VDD_RTC_AP_A03,
	VDD_SYS_BUCK2_A03,
	VDD_SOC_A03,
	VDD_SYS_REG_A03,
	VDD_CPU_AP_A03,
	VDD_SYS_BUCK5_A03,
	VDD_1V8_BUCK5_A03,
	VDD_1V8_AP_A03,
	VDD_SYS_BUCK3_A03,
	VDD_1V2_BUCK3_A03,
	VDD_SW_1V2_MUX_A03,
	VDD_SW_1V2_DSI_CSI_AP_A03,
	AVDD_1V05_LDO7_A03,
};

/* following rails are present on Ceres-FFD */
enum {
	VDD_CELL,
	VDD_CPU_BUCK2,
	VDD_SOC_BUCK1,
};

/* following rails are present on Atlantis-FFD */
enum {
	PM_VDD_CELL,
	VDD_CPU_SMPS1_2,
	VDD_SOC_SMPS6,
};

/* following rails are present on Atlantis-ERS */
enum {
	ATL_VDD_SYS_CELL,
	ATL_VDD_SYS_SMPS8,
	ATL_VDD_SYS_SMPS6,
	ATL_VDD_SOC,
	ATL_VDD_SYS_SMPS1_2,
	ATL_VDD_CPU,
	ATL_VDD_SW_1V2_DSI_CSI_AP,
	ATL_VDD_1V8_SMPS9,
};

enum {
	ATL_VDD_SYS_BL,
	ATL_AVDD_DIS_LDO1,
};

/* following rails are present on Atlantis A02 on i2c2_1 */
enum {
	ATL_A02_VDD_SYS_CELL,
	ATL_A02_VDD_RTC_LDO5,
	ATL_A02_VDD_SYS_SMPS1_2,
	ATL_A02_VDD_SOC,
	ATL_A02_VDD_SYS_REG,
	ATL_A02_VDD_CPU,
	ATL_A02_VDD_SYS_SMPS9,
	ATL_A02_VDD_1V8_SMPS9,
	ATL_A02_VDD_1V8_AP,
	ATL_A02_VDD_SYS_SMPS8,
	ATL_A02_VDD_1V2_SMPS8,
	ATL_A02_VDD_SW_1V2_MUX,
	ATL_A02_VDD_SW_1V2_DSI_AP,
	ATL_A02_AVDD_1V05_LDO4
};

static struct ina219_platform_data power_mon_info_0[] = {
	/* All unused INA219 devices use below data*/
	[UNUSED_RAIL] = {
		.calibration_data = 0x369C,
		.power_lsb = 3.051979018 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "unused_rail",
		.divisor = 20,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
	},
};

/* following are power monitor parameters for Ceres-ERS */
static struct ina230_platform_data power_mon_info_1[] = {
	[VDD_SYS_CELL] = {
		.calibration_data  = 0x1062,
		.power_lsb = 3.051979018 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_CELL",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[VDD_SYS_BUCK1] = {
		.calibration_data  = 0x3E79,
		.power_lsb = 0.800350153 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_BUCK1",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[VDD_SOC] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 3.906369213 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SOC",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	[VDD_SYS_BUCK2] = {
		.calibration_data  = 0x3A90,
		.power_lsb = 1.707577375 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_BUCK2",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 5,
	},

	[VDD_CPU_AP] = {
		.calibration_data  = 0x6665,
		.power_lsb = 4.883073284 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_CPU_AP",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	[VDD_1V8_BUCK5] = {
		.calibration_data  = 0x5692,
		.power_lsb = 0.577565202 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_1V8_BUCK5",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[VDD_SYS_BUCK3] = {
		.calibration_data  = 0xE90,
		.power_lsb = 0.343347639 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_BUCK3",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 100,
	},

	[VDD_SW_1V2_DSI_CSI_AP] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 0.019531846 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SW_1V2_DSI_CSI_AP",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 200,
	},
};

static struct ina230_platform_data power_mon_info_2[] = {
	[VDD_SYS_BL] = {
		.calibration_data  = 0x4188,
		.power_lsb = 0.152598951 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_BL",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 50,
	},

	[AVDD_DIS_LDO4] = {
		.calibration_data  = 0x48D0,
		.power_lsb = 0.068669528 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "AVDD_DIS_LDO4",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 100,
	},
};

/* following are power monitor parameters for Ceres-ERS A03 on i2c2_1 */
static struct ina230_platform_data ceres_A03_power_mon_info_1[] = {
	[VDD_SYS_CELL_A03] = {
		.calibration_data  = 0xAEC,
		.power_lsb = 4.577968526 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_CELL",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[VDD_RTC_AP_A03] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 0.195318461 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_RTC_AP",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 20,
	},

	[VDD_SYS_BUCK2_A03] = {
		.calibration_data  = 0x3A90,
		.power_lsb = 1.707577375 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_BUCK2",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 5,
	},

	[VDD_SOC_A03] = {
		.calibration_data  = 0x6665,
		.power_lsb = 4.883073284 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SOC",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	[VDD_SYS_REG_A03] = {
		.calibration_data  = 0x1F3A,
		.power_lsb = 3.202401801 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_REG",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 5,
	},

	[VDD_CPU_AP_A03] = {
		.calibration_data  = 0x369C,
		.power_lsb = 9.155937053 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_CPU_AP",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	[VDD_SYS_BUCK5_A03] = {
		.calibration_data  = 0xF22,
		.power_lsb = 0.330407847 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_BUCK5",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 100,
	},

	[VDD_1V8_BUCK5_A03] = {
		.calibration_data  = 0x5692,
		.power_lsb = 0.577565202 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_1V8_BUCK5",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[VDD_1V8_AP_A03] = {
		.calibration_data  = 0x74D0,
		.power_lsb = 0.142678794 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_1V8_AP",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 30,
	},

	[VDD_SYS_BUCK3_A03] = {
		.calibration_data  = 0xE90,
		.power_lsb = 0.343347639 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_BUCK3",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 100,
	},

	[VDD_1V2_BUCK3_A03] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 3.906369213 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_1V2_BUCK3",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	[VDD_SW_1V2_MUX_A03] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 3.906369213 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SW_1V2_MUX",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	/*
	 * mark following rails as unused until i2c address conflict
	 * is resolved
	 */
	[VDD_SW_1V2_DSI_CSI_AP_A03] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 0.019531846 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "unused_rail", /* VDD_SW_1V2_DSI_CSI_AP */
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 200,
	},

	[AVDD_1V05_LDO7_A03] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 0.130212307 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "unused_rail", /* AVDD_1V05_LDO7 */
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 30,
	},
};

/* following are power monitor parameters for Ceres-FFD */
static struct ina230_platform_data power_mon_info_ffd[] = {
	[VDD_CELL] = {
		.calibration_data  = 0x20C4,
		.power_lsb = 3.051979018 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_CELL",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 5,
	},

	[VDD_CPU_BUCK2] = {
		.calibration_data  = 0x1D48,
		.power_lsb = 1.707577375 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_CPU_BUCK2",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[VDD_SOC_BUCK1] = {
		.calibration_data  = 0x3E79,
		.power_lsb = 0.800350153 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SOC_BUCK1",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},
};

/* following are power monitor parameters for Atlantis-FFD */
static struct ina230_platform_data atl_ffd_power_mon_info[] = {
	[PM_VDD_CELL] = {
		.rail_name = "VDD_CELL",
		.resistor = 5,
	},

	[VDD_CPU_SMPS1_2] = {
		.rail_name = "VDD_CPU_SMPS1_2",
		.resistor = 10,
	},

	[VDD_SOC_SMPS6] = {
		.rail_name = "VDD_SOC_SMPS6",
		.resistor = 10,
	},
};

/* following are the power monitor parameters for Atlantis */
static struct ina230_platform_data atl_power_mon_info_1[] = {
	[ATL_VDD_SYS_CELL] = {
		.calibration_data  = 0x1062,
		.power_lsb = 3.051979018 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_CELL",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[ATL_VDD_SYS_SMPS8] = {
		.calibration_data  = 0xB42,
		.power_lsb = 0.444136017 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_SMPS8",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 100,
	},

	[ATL_VDD_SYS_SMPS6] = {
		.calibration_data  = 0x2ED7,
		.power_lsb = 1.067467267 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_SMPS6",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[ATL_VDD_SOC] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 3.906369213 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SOC",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	[ATL_VDD_SYS_SMPS1_2] = {
		.calibration_data  = 0x176B,
		.power_lsb = 2.135112594 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_SMPS1_2",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[ATL_VDD_CPU] = {
		.calibration_data  = 0x51EA,
		.power_lsb = 6.103958035 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_CPU",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	[ATL_VDD_SW_1V2_DSI_CSI_AP] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 0.019531846 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SW_1V2_DSI_CSI_AP",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 200,
	},

	[ATL_VDD_1V8_SMPS9] = {
		.calibration_data  = 0x6240,
		.power_lsb = 0.508905852 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_1V8_SMPS9",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},
};

static struct ina230_platform_data atl_power_mon_info_2[] = {
	[ATL_VDD_SYS_BL] = {
		.calibration_data  = 0x4188,
		.power_lsb = 0.152598951 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_BL",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 50,
	},

	[ATL_AVDD_DIS_LDO1] = {
		.calibration_data  = 0x48D0,
		.power_lsb = 0.068669528 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "AVDD_DIS_LDO1",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 100,
	},
};

/* following are power monitor parameters for Atlantis A02 on i2c2_1 */
static struct ina230_platform_data atl_A02_power_mon_info_1[] = {
	[ATL_A02_VDD_SYS_CELL] = {
		.calibration_data  = 0x0AEC,
		.power_lsb = 4.577968526 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_CELL",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[ATL_A02_VDD_RTC_LDO5] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 0.078127384 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_RTC_LDO5",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 50,
	},

	[ATL_A02_VDD_SYS_SMPS1_2] = {
		.calibration_data  = 0x176B,
		.power_lsb = 2.135112594 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_SMPS1_2",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[ATL_A02_VDD_SOC] = {
		.calibration_data  = 0x51EA,
		.power_lsb = 6.103958035 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SOC",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	[ATL_A02_VDD_SYS_REG] = {
		.calibration_data  = 0x1F3A,
		.power_lsb = 3.202401801 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_REG",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 5,
	},

	[ATL_A02_VDD_CPU] = {
		.calibration_data  = 0x369C,
		.power_lsb = 9.155937053 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_CPU",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	[ATL_A02_VDD_SYS_SMPS9] = {
		.calibration_data  = 0x1127,
		.power_lsb = 0.291505352 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_SMPS9",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 100,
	},

	[ATL_A02_VDD_1V8_SMPS9] = {
		.calibration_data  = 0x6240,
		.power_lsb = 0.508905852 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_1V8_SMPS9",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 10,
	},

	[ATL_A02_VDD_1V8_AP] = {
		.calibration_data  = 0x74D0,
		.power_lsb = 0.142678794 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_1V8_AP",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 30,
	},

	[ATL_A02_VDD_SYS_SMPS8] = {
		.calibration_data  = 0x0B42,
		.power_lsb = 0.444136017 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SYS_SMPS8",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 100,
	},

	[ATL_A02_VDD_1V2_SMPS8] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 3.906369213 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_1V2_SMPS8",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	[ATL_A02_VDD_SW_1V2_MUX] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 3.906369213 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "VDD_SW_1V2_MUX",
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 1,
	},

	/*
	 * mark following rails as unused until i2c address conflict
	 * is resolved
	 */
	[ATL_A02_VDD_SW_1V2_DSI_AP] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 0.019531846 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "unused_rail", /* VDD_SW_1V2_DSI_CSI_AP */
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 200,
	},

	[ATL_A02_AVDD_1V05_LDO4] = {
		.calibration_data  = 0x7FFF,
		.power_lsb = 0.130212307 * PRECISION_MULTIPLIER_CERES,
		.rail_name = "unused_rail", /* AVDD_1V05_LDO4 */
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_CERES,
		.resistor = 30,
	},
};

/* i2c addresses of rails present on Ceres-ERS */
enum {
	INA_I2C_2_0_ADDR_40,
	INA_I2C_2_0_ADDR_41,
	INA_I2C_2_0_ADDR_42,
};

enum {
	INA_I2C_2_1_ADDR_40,
	INA_I2C_2_1_ADDR_41,
	INA_I2C_2_1_ADDR_42,
	INA_I2C_2_1_ADDR_43,
	INA_I2C_2_1_ADDR_44,
	INA_I2C_2_1_ADDR_45,
	INA_I2C_2_1_ADDR_46,
	INA_I2C_2_1_ADDR_47,
};

enum {
	INA_I2C_2_2_ADDR_41,
	INA_I2C_2_2_ADDR_44,
};

/* i2c2_1 addresses of rails present on Ceres-ERS A03 */
enum {
	CERES_A03_INA_I2C_2_1_ADDR_40,
	CERES_A03_INA_I2C_2_1_ADDR_41,
	CERES_A03_INA_I2C_2_1_ADDR_42,
	CERES_A03_INA_I2C_2_1_ADDR_43,
	CERES_A03_INA_I2C_2_1_ADDR_44,
	CERES_A03_INA_I2C_2_1_ADDR_45,
	CERES_A03_INA_I2C_2_1_ADDR_46,
	CERES_A03_INA_I2C_2_1_ADDR_47,
	CERES_A03_INA_I2C_2_1_ADDR_48,
	CERES_A03_INA_I2C_2_1_ADDR_49,
	CERES_A03_INA_I2C_2_1_ADDR_4B,
	CERES_A03_INA_I2C_2_1_ADDR_4C,
	CERES_A03_INA_I2C_2_1_ADDR_4E,
	CERES_A03_INA_I2C_2_1_ADDR_4F,
};

/*
 * the I2C device addresses on branch 2 are different between
 * Ceres and Atlantis
 */
enum {
	ATL_INA_I2C_2_2_ADDR_49,
	ATL_INA_I2C_2_2_ADDR_4C,
};

/* i2c2_1 addresses of rails present on Atlantis A02 */
enum {
	ATL_A02_INA_I2C_2_1_ADDR_40,
	ATL_A02_INA_I2C_2_1_ADDR_41,
	ATL_A02_INA_I2C_2_1_ADDR_42,
	ATL_A02_INA_I2C_2_1_ADDR_43,
	ATL_A02_INA_I2C_2_1_ADDR_44,
	ATL_A02_INA_I2C_2_1_ADDR_45,
	ATL_A02_INA_I2C_2_1_ADDR_46,
	ATL_A02_INA_I2C_2_1_ADDR_47,
	ATL_A02_INA_I2C_2_1_ADDR_48,
	ATL_A02_INA_I2C_2_1_ADDR_49,
	ATL_A02_INA_I2C_2_1_ADDR_4B,
	ATL_A02_INA_I2C_2_1_ADDR_4C,
	ATL_A02_INA_I2C_2_1_ADDR_4E,
	ATL_A02_INA_I2C_2_1_ADDR_4F,
};

/* i2c addresses of rails present on Ceres-FFD and Atlantis-FFD */
enum {
	INA_I2C_2_ADDR_40,
	INA_I2C_2_ADDR_41,
	INA_I2C_2_ADDR_42,
};

/* following are the i2c board info for Ceres-ERS */
static struct i2c_board_info ceres_i2c2_0_ina219_board_info[] = {
	[INA_I2C_2_0_ADDR_40] = {
		I2C_BOARD_INFO("ina219", 0x40),
		.platform_data = &power_mon_info_0[UNUSED_RAIL],
		.irq = -1,
	},

	[INA_I2C_2_0_ADDR_41] = {
		I2C_BOARD_INFO("ina219", 0x41),
		.platform_data = &power_mon_info_0[UNUSED_RAIL],
		.irq = -1,
	},

	[INA_I2C_2_0_ADDR_42] = {
		I2C_BOARD_INFO("ina219", 0x42),
		.platform_data = &power_mon_info_0[UNUSED_RAIL],
		.irq = -1,
	},
};

static struct i2c_board_info ceres_i2c2_1_ina230_board_info[] = {
	[INA_I2C_2_1_ADDR_40] = {
		I2C_BOARD_INFO("ina230", 0x40),
		.platform_data = &power_mon_info_1[VDD_SYS_CELL],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_41] = {
		I2C_BOARD_INFO("ina230", 0x41),
		.platform_data = &power_mon_info_1[VDD_SYS_BUCK3],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_42] = {
		I2C_BOARD_INFO("ina230", 0x42),
		.platform_data = &power_mon_info_1[VDD_SYS_BUCK1],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_43] = {
		I2C_BOARD_INFO("ina230", 0x43),
		.platform_data = &power_mon_info_1[VDD_SOC],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_44] = {
		I2C_BOARD_INFO("ina230", 0x44),
		.platform_data = &power_mon_info_1[VDD_SYS_BUCK2],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_45] = {
		I2C_BOARD_INFO("ina230", 0x45),
		.platform_data = &power_mon_info_1[VDD_CPU_AP],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_46] = {
		I2C_BOARD_INFO("ina230", 0x46),
		.platform_data = &power_mon_info_1[VDD_SW_1V2_DSI_CSI_AP],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_47] = {
		I2C_BOARD_INFO("ina230", 0x47),
		.platform_data = &power_mon_info_1[VDD_1V8_BUCK5],
		.irq = -1,
	},
};

static struct i2c_board_info ceres_i2c2_2_ina230_board_info[] = {
	[INA_I2C_2_2_ADDR_41] = {
		I2C_BOARD_INFO("ina230", 0x41),
		.platform_data = &power_mon_info_2[VDD_SYS_BL],
		.irq = -1,
	},

	[INA_I2C_2_2_ADDR_44] = {
		I2C_BOARD_INFO("ina230", 0x44),
		.platform_data = &power_mon_info_2[AVDD_DIS_LDO4],
		.irq = -1,
	},
};

/* following are the i2c2_1 board info for Ceres-ERS A03 */
static struct i2c_board_info ceres_A03_i2c2_1_ina230_board_info[] = {
	[CERES_A03_INA_I2C_2_1_ADDR_40] = {
		I2C_BOARD_INFO("ina230", 0x40),
		.platform_data = &ceres_A03_power_mon_info_1[VDD_SYS_CELL_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_41] = {
		I2C_BOARD_INFO("ina230", 0x41),
		.platform_data = &ceres_A03_power_mon_info_1[VDD_RTC_AP_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_42] = {
		I2C_BOARD_INFO("ina230", 0x42),
		.platform_data = &ceres_A03_power_mon_info_1[VDD_SYS_BUCK2_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_43] = {
		I2C_BOARD_INFO("ina230", 0x43),
		.platform_data = &ceres_A03_power_mon_info_1[VDD_SOC_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_44] = {
		I2C_BOARD_INFO("ina230", 0x44),
		.platform_data = &ceres_A03_power_mon_info_1[VDD_SYS_REG_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_45] = {
		I2C_BOARD_INFO("ina230", 0x45),
		.platform_data = &ceres_A03_power_mon_info_1[VDD_CPU_AP_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_46] = {
		I2C_BOARD_INFO("ina230", 0x46),
		.platform_data = &ceres_A03_power_mon_info_1[VDD_SYS_BUCK5_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_47] = {
		I2C_BOARD_INFO("ina230", 0x47),
		.platform_data = &ceres_A03_power_mon_info_1[VDD_1V8_BUCK5_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_48] = {
		I2C_BOARD_INFO("ina230", 0x48),
		.platform_data = &ceres_A03_power_mon_info_1[VDD_1V8_AP_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_49] = {
		I2C_BOARD_INFO("ina230", 0x49),
		.platform_data = &ceres_A03_power_mon_info_1[VDD_SYS_BUCK3_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_4B] = {
		I2C_BOARD_INFO("ina230", 0x4B),
		.platform_data = &ceres_A03_power_mon_info_1[VDD_1V2_BUCK3_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_4C] = {
		I2C_BOARD_INFO("ina230", 0x4C),
		.platform_data =
			&ceres_A03_power_mon_info_1[VDD_SW_1V2_MUX_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_4E] = {
		I2C_BOARD_INFO("ina230", 0x4E),
		.platform_data =
			&ceres_A03_power_mon_info_1[VDD_SW_1V2_DSI_CSI_AP_A03],
		.irq = -1,
	},

	[CERES_A03_INA_I2C_2_1_ADDR_4F] = {
		I2C_BOARD_INFO("ina230", 0x4F),
		.platform_data =
			&ceres_A03_power_mon_info_1[AVDD_1V05_LDO7_A03],
		.irq = -1,
	},
};

/* following are the i2c board info for Ceres-FFD */
static struct i2c_board_info ceres_i2c2_ina230_board_info_ffd[] = {
	[INA_I2C_2_ADDR_40] = {
		I2C_BOARD_INFO("ina230", 0x40),
		.platform_data = &power_mon_info_ffd[VDD_CELL],
		.irq = -1,
	},

	[INA_I2C_2_ADDR_41] = {
		I2C_BOARD_INFO("ina230", 0x41),
		.platform_data = &power_mon_info_ffd[VDD_CPU_BUCK2],
		.irq = -1,
	},

	[INA_I2C_2_ADDR_42] = {
		I2C_BOARD_INFO("ina230", 0x42),
		.platform_data = &power_mon_info_ffd[VDD_SOC_BUCK1],
		.irq = -1,
	},
};

/* following are the i2c board info for Atlantis-FFD */
static struct i2c_board_info atl_ffd_ina230_board_info[] = {
	[INA_I2C_2_ADDR_40] = {
		I2C_BOARD_INFO("ina230", 0x40),
		.platform_data = &atl_ffd_power_mon_info[PM_VDD_CELL],
		.irq = -1,
	},

	[INA_I2C_2_ADDR_41] = {
		I2C_BOARD_INFO("ina230", 0x41),
		.platform_data = &atl_ffd_power_mon_info[VDD_CPU_SMPS1_2],
		.irq = -1,
	},

	[INA_I2C_2_ADDR_42] = {
		I2C_BOARD_INFO("ina230", 0x42),
		.platform_data = &atl_ffd_power_mon_info[VDD_SOC_SMPS6],
		.irq = -1,
	},
};

/* following are the i2c board info for Atlantis */
static struct i2c_board_info atlantis_i2c2_1_ina230_board_info[] = {
	[INA_I2C_2_1_ADDR_40] = {
		I2C_BOARD_INFO("ina230", 0x40),
		.platform_data = &atl_power_mon_info_1[ATL_VDD_SYS_CELL],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_41] = {
		I2C_BOARD_INFO("ina230", 0x41),
		.platform_data = &atl_power_mon_info_1[ATL_VDD_SYS_SMPS8],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_42] = {
		I2C_BOARD_INFO("ina230", 0x42),
		.platform_data = &atl_power_mon_info_1[ATL_VDD_SYS_SMPS6],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_43] = {
		I2C_BOARD_INFO("ina230", 0x43),
		.platform_data = &atl_power_mon_info_1[ATL_VDD_SOC],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_44] = {
		I2C_BOARD_INFO("ina230", 0x44),
		.platform_data = &atl_power_mon_info_1[ATL_VDD_SYS_SMPS1_2],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_45] = {
		I2C_BOARD_INFO("ina230", 0x45),
		.platform_data = &atl_power_mon_info_1[ATL_VDD_CPU],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_46] = {
		I2C_BOARD_INFO("ina230", 0x46),
		.platform_data =
			&atl_power_mon_info_1[ATL_VDD_SW_1V2_DSI_CSI_AP],
		.irq = -1,
	},

	[INA_I2C_2_1_ADDR_47] = {
		I2C_BOARD_INFO("ina230", 0x47),
		.platform_data = &atl_power_mon_info_1[ATL_VDD_1V8_SMPS9],
		.irq = -1,
	},
};

static struct i2c_board_info atlantis_i2c2_2_ina230_board_info[] = {
	[ATL_INA_I2C_2_2_ADDR_49] = {
		I2C_BOARD_INFO("ina230", 0x49),
		.platform_data = &atl_power_mon_info_2[ATL_VDD_SYS_BL],
		.irq = -1,
	},

	[ATL_INA_I2C_2_2_ADDR_4C] = {
		I2C_BOARD_INFO("ina230", 0x4C),
		.platform_data = &atl_power_mon_info_2[ATL_AVDD_DIS_LDO1],
		.irq = -1,
	},
};

/* following are the i2c2_1 board info for Atlantis A02 */
static struct i2c_board_info atlantis_A02_i2c2_1_ina230_board_info[] = {
	[ATL_A02_INA_I2C_2_1_ADDR_40] = {
		I2C_BOARD_INFO("ina230", 0x40),
		.platform_data =
			&atl_A02_power_mon_info_1[ATL_A02_VDD_SYS_CELL],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_41] = {
		I2C_BOARD_INFO("ina230", 0x41),
		.platform_data =
			&atl_A02_power_mon_info_1[ATL_A02_VDD_RTC_LDO5],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_42] = {
		I2C_BOARD_INFO("ina230", 0x42),
		.platform_data =
			&atl_A02_power_mon_info_1[ATL_A02_VDD_SYS_SMPS1_2],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_43] = {
		I2C_BOARD_INFO("ina230", 0x43),
		.platform_data = &atl_A02_power_mon_info_1[ATL_A02_VDD_SOC],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_44] = {
		I2C_BOARD_INFO("ina230", 0x44),
		.platform_data = &atl_A02_power_mon_info_1[ATL_A02_VDD_SYS_REG],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_45] = {
		I2C_BOARD_INFO("ina230", 0x45),
		.platform_data = &atl_A02_power_mon_info_1[ATL_A02_VDD_CPU],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_46] = {
		I2C_BOARD_INFO("ina230", 0x46),
		.platform_data =
			&atl_A02_power_mon_info_1[ATL_A02_VDD_SYS_SMPS9],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_47] = {
		I2C_BOARD_INFO("ina230", 0x47),
		.platform_data =
			&atl_A02_power_mon_info_1[ATL_A02_VDD_1V8_SMPS9],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_48] = {
		I2C_BOARD_INFO("ina230", 0x48),
		.platform_data = &atl_A02_power_mon_info_1[ATL_A02_VDD_1V8_AP],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_49] = {
		I2C_BOARD_INFO("ina230", 0x49),
		.platform_data =
			&atl_A02_power_mon_info_1[ATL_A02_VDD_SYS_SMPS8],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_4B] = {
		I2C_BOARD_INFO("ina230", 0x4B),
		.platform_data =
			&atl_A02_power_mon_info_1[ATL_A02_VDD_1V2_SMPS8],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_4C] = {
		I2C_BOARD_INFO("ina230", 0x4C),
		.platform_data =
			&atl_A02_power_mon_info_1[ATL_A02_VDD_SW_1V2_MUX],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_4E] = {
		I2C_BOARD_INFO("ina230", 0x4E),
		.platform_data =
			&atl_A02_power_mon_info_1[ATL_A02_VDD_SW_1V2_DSI_AP],
		.irq = -1,
	},

	[ATL_A02_INA_I2C_2_1_ADDR_4F] = {
		I2C_BOARD_INFO("ina230", 0x4F),
		.platform_data =
			&atl_A02_power_mon_info_1[ATL_A02_AVDD_1V05_LDO4],
		.irq = -1,
	},
};

static struct pca954x_platform_mode ceres_pca954x_modes[] = {
	{ .adap_id = PCA954x_I2C_BUS0, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS1, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS2, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS3, .deselect_on_exit = true, },
};

static struct pca954x_platform_data ceres_pca954x_data = {
	.modes    = ceres_pca954x_modes,
	.num_modes      = ARRAY_SIZE(ceres_pca954x_modes),
};

static const struct i2c_board_info ceres_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("pca9546", 0x71),
		.platform_data = &ceres_pca954x_data,
	},
};

static void __init register_devices_E1690(void)
{
	i2c_register_board_info(1, ceres_i2c2_ina230_board_info_ffd,
		ARRAY_SIZE(ceres_i2c2_ina230_board_info_ffd));
}

static void __init register_devices_E1740(void)
{
	i2c_register_board_info(1, atl_ffd_ina230_board_info,
		ARRAY_SIZE(atl_ffd_ina230_board_info));
}

static void __init register_devices_E1670(void)
{
	i2c_register_board_info(PCA954x_I2C_BUS1,
		atlantis_i2c2_1_ina230_board_info,
		ARRAY_SIZE(atlantis_i2c2_1_ina230_board_info));

	i2c_register_board_info(PCA954x_I2C_BUS2,
		atlantis_i2c2_2_ina230_board_info,
		ARRAY_SIZE(atlantis_i2c2_2_ina230_board_info));
}

static void __init register_devices_E1670_A02(void)
{
	i2c_register_board_info(PCA954x_I2C_BUS1,
		atlantis_A02_i2c2_1_ina230_board_info,
		ARRAY_SIZE(atlantis_A02_i2c2_1_ina230_board_info));

	/* i2c2_2 on A02 is same as previous versions */
	i2c_register_board_info(PCA954x_I2C_BUS2,
		atlantis_i2c2_2_ina230_board_info,
		ARRAY_SIZE(atlantis_i2c2_2_ina230_board_info));
}

static void __init register_devices_E1680(void)
{
	i2c_register_board_info(PCA954x_I2C_BUS1,
		ceres_i2c2_1_ina230_board_info,
		ARRAY_SIZE(ceres_i2c2_1_ina230_board_info));

	i2c_register_board_info(PCA954x_I2C_BUS2,
		ceres_i2c2_2_ina230_board_info,
		ARRAY_SIZE(ceres_i2c2_2_ina230_board_info));
}

static void __init register_devices_E1680_A03(void)
{
	i2c_register_board_info(PCA954x_I2C_BUS1,
		ceres_A03_i2c2_1_ina230_board_info,
		ARRAY_SIZE(ceres_A03_i2c2_1_ina230_board_info));

	/* i2c2_2 on A03 is same as previous versions */
	i2c_register_board_info(PCA954x_I2C_BUS2,
		ceres_i2c2_2_ina230_board_info,
		ARRAY_SIZE(ceres_i2c2_2_ina230_board_info));
}

int __init ceres_pmon_init(void)
{
	struct board_info bi;

	tegra_get_board_info(&bi);

	if (bi.board_id == BOARD_E1690) {
		/* There are only 3 devices on Ceres-FFD
		 * so register only those if board is E1690
		 */
		register_devices_E1690();
	} else if (bi.board_id == BOARD_E1740) {
			/* There are only 3 devices on Atlantis-FFD
			 * so register only those if board is E1740
			 */
			register_devices_E1740();
	} else {
		/* register devices common to ceres/atlantis ERS */
		i2c_register_board_info(1, ceres_i2c2_board_info,
			ARRAY_SIZE(ceres_i2c2_board_info));

		i2c_register_board_info(PCA954x_I2C_BUS0,
			ceres_i2c2_0_ina219_board_info,
			ARRAY_SIZE(ceres_i2c2_0_ina219_board_info));

		if ((bi.board_id == BOARD_E1670) ||
		    (bi.board_id == BOARD_E1671)) {
			/* register devices specific to Atlantis-ERS */
			if (bi.fab >= BOARD_FAB_A02)
				register_devices_E1670_A02();
			else
				register_devices_E1670();
		} else {
			/* register devices specific to Ceres-ERS */
			if ((bi.fab >= BOARD_FAB_A03) ||
			    (bi.board_id == BOARD_E1683))
				register_devices_E1680_A03();
			else
				register_devices_E1680();
		}
	}

	return 0;
}


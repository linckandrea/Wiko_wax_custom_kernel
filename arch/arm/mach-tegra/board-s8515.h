/*
 * arch/arm/mach-tegra/board-s8515.h
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#ifndef _MACH_TEGRA_BOARD_S8515_H
#define _MACH_TEGRA_BOARD_S8515_H

#include <mach/gpio-tegra.h>
#include <mach/irqs.h>
#include <linux/mfd/palmas.h>
#include <linux/mfd/max77660/max77660-core.h>
#include "gpio-names.h"

//Ivan added
#define TINNO_TP_2_CAM

/* MAX77660_GPIO Base Address */
#define MAX77660_GPIO_BASE      TEGRA_NR_GPIOS
#define MAX77660_IRQ_BASE	TEGRA_NR_IRQS

/* I2C related GPIOs */
#define TEGRA_GPIO_I2C1_SCL		TEGRA_GPIO_PF1
#define TEGRA_GPIO_I2C1_SDA		TEGRA_GPIO_PF2
#define TEGRA_GPIO_I2C2_SCL		TEGRA_GPIO_PT5
#define TEGRA_GPIO_I2C2_SDA		TEGRA_GPIO_PT6
#define TEGRA_GPIO_I2C3_SCL		TEGRA_GPIO_PR2
#define TEGRA_GPIO_I2C3_SDA		TEGRA_GPIO_PR3
#define TEGRA_GPIO_I2C4_SCL		TEGRA_GPIO_PV4
#define TEGRA_GPIO_I2C4_SDA		TEGRA_GPIO_PV5
#define TEGRA_GPIO_I2C5_SCL		TEGRA_GPIO_PZ6
#define TEGRA_GPIO_I2C5_SDA		TEGRA_GPIO_PZ7

/* Camera related GPIOs */
#define CAM_RSTN			TEGRA_GPIO_PS0
#define CAM_FLASH_STROBE		TEGRA_GPIO_PS1
#define CAM1_POWER_DWN_GPIO		TEGRA_GPIO_PS2
#define CAM2_POWER_DWN_GPIO		TEGRA_GPIO_PS3
#define CAM_AF_PWDN			TEGRA_GPIO_PS4
#define CAM_TORCH_EN			TEGRA_GPIO_PS5


//luis
//definitions for tinno
#if (CONFIG_S8515_PR_VERSION == 2)
#define CAM_RSTN_TINNO			TEGRA_GPIO_PS2
//#define CAM_FLASH_STROBE		TEGRA_GPIO_PS1
#define CAM_PWDN_TINNO  		TEGRA_GPIO_PS2
#else
#define CAM_RSTN_TINNO			TEGRA_GPIO_PS0
//#define CAM_FLASH_STROBE		TEGRA_GPIO_PS1
#define CAM_PWDN_TINNO  		TEGRA_GPIO_PS2
#endif
//#define CAM2_POWER_DWN_GPIO		TEGRA_GPIO_PS3
#define CAM_CHOS_TINNO			TEGRA_GPIO_PS4
//#define CAM_TORCH_EN			TEGRA_GPIO_PS5

/* Touchscreen definitions */
#define SYNAPTICS_ATTN_GPIO		TEGRA_GPIO_PN1
#define SYNAPTICS_RESET_GPIO		TEGRA_GPIO_PN2

#define TOUCH_GPIO_IRQ_RAYDIUM_SPI	TEGRA_GPIO_PN1
#define TOUCH_GPIO_RST_RAYDIUM_SPI	TEGRA_GPIO_PN2

//Ivan
#define SYN_320X_ADDR			0x72 >> 1


/* Audio-related GPIOs */
#define TEGRA_GPIO_CDC_IRQ		TEGRA_GPIO_PO5
#define TEGRA_GPIO_LDO1_EN		TEGRA_GPIO_PN3
#define TEGRA_GPIO_HP_DET		(MAX77660_GPIO_BASE + MAX77660_GPIO5)
#define TEGRA_GPIO_HP_MIC_DET		TEGRA_GPIO_PH6

#define TEGRA_GPIO_SPKR_EN		-1
#define TEGRA_GPIO_INT_MIC_EN	-1
#define TEGRA_GPIO_EXT_MIC_EN	-1

/* Invensense MPU Definitions */
#define MPU_GYRO_NAME		"mpu9150"

#define MPU_GYRO_IRQ_GPIO	TEGRA_GPIO_PM7

#define MPU_GYRO_ADDR			0x69
#define MPU_GYRO_BUS_NUM		0
#define MPU_GYRO_ORIENTATION		{ 0, 1, 0, -1, 0, 0, 0, 0, 1 }
#define MPU_GYRO_ORIENTATION_E1680	{ -1, 0, 0, 0, -1, 0, 0, 0, 1 }
#define MPU_COMPASS_NAME		"ak8963"
#define MPU_COMPASS_IRQ_GPIO		0
#define MPU_COMPASS_ADDR		0x0D
#define MPU_COMPASS_BUS_NUM		0
#define MPU_COMPASS_ORIENTATION		{ 1, 0, 0, 0, -1, 0, 0, 0, -1 }
//#define MPU_COMPASS_ORIENTATION_E1680	{ 0, -1, 0, -1, 0, 0, 0, 0, -1 }
//#define MPU_ACCE_ORIENTATION_E1680	{ -1, 0, 0, 0, -1, 0, 0, 0, 1 }
#define MPU_COMPASS_ORIENTATION_E1680	MTMAT_BOT_CCW_270

//#define MPU_COMPASS_ORIENTATION_E1680	MTMAT_TOP_CCW_270
#define MPU_ACCE_ORIENTATION_E1680	MTMAT_TOP_CCW_270

/* PCA954x I2C bus expander bus addresses */
#define PCA954x_I2C_BUS_BASE    6
#define PCA954x_I2C_BUS0        (PCA954x_I2C_BUS_BASE + 0)
#define PCA954x_I2C_BUS1        (PCA954x_I2C_BUS_BASE + 1)
#define PCA954x_I2C_BUS2        (PCA954x_I2C_BUS_BASE + 2)
#define PCA954x_I2C_BUS3        (PCA954x_I2C_BUS_BASE + 3)

/* Secondary modem related GPIOs */
#define MDM2_EN			TEGRA_GPIO_PG5
#define MDM2_RST		TEGRA_GPIO_PH0
#define MDM2_COLDBOOT		TEGRA_GPIO_PG6
#define MDM2AP_ACK2		TEGRA_GPIO_PG7

/* Baseband IDs */
enum tegra_bb_type {
	TEGRA_BB_INTEGRATED = 1,
	TEGRA_BB_I500SWD = 2,
	TEGRA_BB_HSIC_HUB = 6,
	TEGRA_BB_OEM4 = 7,
	TEGRA_BB_INTEGRATED_DISABLED = 8, /* bbc core power on,
					     no bbc mem regions,
					     bbc held in reset */
};


int ceres_sensors_init(void);
int ceres_keys_init(void);
int ceres_sdhci_init(void);
int ceres_regulator_init(void);
int ceres_suspend_init(void);
int ceres_pinmux_init(void);
int ceres_panel_init(void);
int ceres_soctherm_init(void);
int ceres_emc_init(void);
int ceres_pmon_init(void);
int ceres_edp_init(void);
int ceres_pinmux_suspend(void);
void ceres_sysedp_init(void);
void ceres_sysedp_psydepl_init(void);
void ceres_sysedp_core_init(void);

#endif

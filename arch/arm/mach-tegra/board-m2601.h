/*
 * arch/arm/mach-tegra/m2601/board-m2601.h
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

#ifndef _MACH_TEGRA_BOARD_M2601_H
#define _MACH_TEGRA_BOARD_M2601_H


/* M2601 skus */
#define TEGRA_M2601_SKU1_A00  0x010000UL
#define TEGRA_M2601_SKU1_B00  0x010100UL


extern unsigned int system_rev;
int m2601_sdhci_init(void);
int m2601_pinmux_init(void);
int m2601_gpio_init(void);
int m2601_pins_state_init(void);

#ifdef CONFIG_SENSORS_TMON_TMP411
	/* Thermal monitor data */
	#define DELTA_TEMP 4000
	#define DELTA_TIME 2000
	#define REMT_OFFSET 8000
	#define I2C_ADDR_TMP411 0x4c
	#define I2C_BUS_TMP411 4
#endif

#endif

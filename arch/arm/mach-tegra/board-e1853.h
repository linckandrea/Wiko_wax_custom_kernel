/*
 * arch/arm/mach-tegra/e1853/board-e1853.h
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

#ifndef _MACH_TEGRA_BOARD_E1853_H
#define _MACH_TEGRA_BOARD_E1853_H

#include <linux/mfd/tps6591x.h>
#include <mach/gpio-tegra.h>

int e1853_sdhci_init(void);
int e1853_pinmux_init(void);
int e1853_touch_init(void);
int e1853_panel_init(void);
int e1853_gpio_init(void);
int e1853_pins_state_init(void);
int e1853_suspend_init(void);
int e1853_regulator_init(void);
int e1853_wifi_init(void);
int e1853_pca953x_init(void);

/* External peripheral act as gpio */
/* TPS6591x GPIOs */
#define TPS6591X_GPIO_BASE	TEGRA_NR_GPIOS
#define TPS6591X_GPIO_0		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP0)
#define TPS6591X_GPIO_1		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP1)
#define TPS6591X_GPIO_2		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP2)
#define TPS6591X_GPIO_3		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP3)
#define TPS6591X_GPIO_4		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP4)
#define TPS6591X_GPIO_5		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP5)
#define TPS6591X_GPIO_6		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP6)
#define TPS6591X_GPIO_7		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP7)
#define TPS6591X_GPIO_8		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP8)
#define TPS6591X_GPIO_END	(TPS6591X_GPIO_BASE + TPS6591X_GPIO_NR)

#define TPS6591X_IRQ_BASE	TEGRA_NR_IRQS
#define TPS6591X_IRQ_END	(TPS6591X_IRQ_BASE + 18)

/* PCA953X - MISC SYSTEM IO */
#define PCA953X_MISCIO_GPIO_BASE	TPS6591X_GPIO_END
#define MISCIO_BT_RST_GPIO			(PCA953X_MISCIO_GPIO_BASE + 0)
#define MISCIO_GPS_RST_GPIO			(PCA953X_MISCIO_GPIO_BASE + 1)
#define MISCIO_GPS_EN_GPIO			(PCA953X_MISCIO_GPIO_BASE + 2)
#define MISCIO_WF_EN_GPIO			(PCA953X_MISCIO_GPIO_BASE + 3)
#define MISCIO_WF_RST_GPIO			(PCA953X_MISCIO_GPIO_BASE + 4)
#define MISCIO_BT_EN_GPIO			(PCA953X_MISCIO_GPIO_BASE + 5)
/* GPIO6 is not used */
#define MISCIO_NOT_USED0			(PCA953X_MISCIO_GPIO_BASE + 6)
#define MISCIO_BT_WAKEUP_GPIO		(PCA953X_MISCIO_GPIO_BASE + 7)
#define MISCIO_FAN_SEL_GPIO			(PCA953X_MISCIO_GPIO_BASE + 8)
#define MISCIO_EN_MISC_BUF_GPIO		(PCA953X_MISCIO_GPIO_BASE + 9)
#define MISCIO_EN_MSATA_GPIO		(PCA953X_MISCIO_GPIO_BASE + 10)
#define MISCIO_EN_SDCARD_GPIO		(PCA953X_MISCIO_GPIO_BASE + 11)
/* GPIO12 is not used */
#define MISCIO_NOT_USED1			(PCA953X_MISCIO_GPIO_BASE + 12)
#define MISCIO_ABB_RST_GPIO			(PCA953X_MISCIO_GPIO_BASE + 13)
#define MISCIO_USER_LED2_GPIO		(PCA953X_MISCIO_GPIO_BASE + 14)
#define MISCIO_USER_LED1_GPIO		(PCA953X_MISCIO_GPIO_BASE + 15)
#define PCA953X_MISCIO_GPIO_END		(PCA953X_MISCIO_GPIO_BASE + 16)

/* PCA953X I2C IO expander bus addresses */
#define PCA953X_MISCIO_ADDR		0x75

/* Thermal monitor data */
#define DELTA_TEMP 4000
#define DELTA_TIME 2000
#define REMT_OFFSET 8000
#define I2C_ADDR_TMP411 0x4c
#define I2C_BUS_TMP411 4

#endif

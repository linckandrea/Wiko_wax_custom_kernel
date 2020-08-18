/*
 * arch/arm/mach-tegra/board-dolak.h
 *
 * Copyright (C) 2011-2013 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _MACH_TEGRA_BOARD_DOLAK_H
#define _MACH_TEGRA_BOARD_DOLAK_H

int dolak_regulator_init(void);
int dolak_suspend_init(void);
int dolak_sdhci_init(void);
int dolak_pinmux_init(void);
int dolak_panel_init(void);
int dolak_sensors_init(void);
int dolak_emc_init(void);

#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM
#define DOLAK_BOARD_NAME "dolak_sim"
#else
#define DOLAK_BOARD_NAME "dolak"
#endif

#endif

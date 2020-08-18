/*
 * arch/arm/mach-tegra/board-atlantis.h
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

#define PALMAS_TEGRA_IRQ_BASE       TEGRA_NR_IRQS
#define PALMAS_TEGRA_GPIO_BASE      TEGRA_NR_GPIOS

#define PMC_WAKE_STATUS		0x14
#define PMC_WAKE2_STATUS	0x168

#define MPU_GYRO_ORIENTATION_E1670      { -1, 0, 0, 0, -1, 0, 0, 0, 1 }
#define MPU_COMPASS_ORIENTATION_E1670   { 0, -1, 0, -1, 0, 0, 0, 0, -1 }

int atlantis_regulator_init(void);
int atlantis_fixed_regulator_init(void);
int atlantis_vibrator_init(void);
int atlantis_pwm_init(void);
void atlantis_sysedp_psydepl_init(void);
int atlantis_soctherm_init(void);

/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __MACH_TEGRA_BBC_POWER_H
#define __MACH_TEGRA_BBC_POWER_H

#ifdef CONFIG_TEGRA_BBC_POWER
extern int tegra_bbc_power_init(struct platform_device *pdev);
extern void bbc_power_set_memfreq(u32 mem_freq);
extern void bbc_power_emc_floor_set(u32 set);
#else
static inline int tegra_bbc_power_init(struct platform_device *pdev)
{
	return 0;
}

static inline void bbc_power_set_memfreq(u32 mem_freq)
{
}

static inline void bbc_power_emc_floor_set(u32 set)
{
}
#endif

#endif /* __MACH_TEGRA_BBC_POWER_H */

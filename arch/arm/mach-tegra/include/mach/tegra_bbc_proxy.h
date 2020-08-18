/*
 * arch/arm/mach-tegra/include/mach/tegra_usb_modem_power.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#ifndef __MACH_TEGRA_BBC_PROXY_H
#define __MACH_TEGRA_BBC_PROXY_H

#include <linux/edp.h>

struct tegra_bbc_proxy_platform_data {
	struct edp_client *modem_boot_edp_client;
	char *edp_manager_name;
	char *ap_name;
};

int tegra_bbc_proxy_edp_register(struct device *dev, u32 num_states,
				u32 *states);
int tegra_bbc_proxy_edp_request(struct device *dev, u32 mode, u32 state,
				u32 threshold);
int tegra_bbc_proxy_bw_register(struct device *dev, u32 bw);
int tegra_bbc_proxy_bw_request(struct device *dev, u32 mode, u32 bw, u32 lt,
				u32 margin);
int tegra_bbc_proxy_la_request(struct device *dev, s32 bbcllr_la_bw,
				s32 bbcr_la_bw, s32 bbcw_la_bw);

int tegra_bbc_proxy_restore_iso(struct device *dev);
int tegra_bbc_proxy_clear_iso(struct device *dev);

int tegra_bbc_proxy_set_rf_mode(struct device *dev, bool fpwm);

int tegra_bbc_proxy_bb_fusing_voltage(struct device *dev, bool enabled);

/* values in KBps */
#define BBC_ISO_CRASH_BW	500000
#define BBC_ISO_BOOT_BW		1200000
#define BBC_ISO_MARGIN_BW	1500000

/* values in KBps */
#define BBCLLR_LA_MAX_BW	640000
#define BBCLLR_LA_CRASH_BW	640000
#define BBCW_LA_CRASH_BW	500000
#define BBCR_LA_CRASH_BW	500000

#define BBC_EDP_E0_INDEX	0
#define BBC_EDP_E0_THRESHOLD	3600

#endif /* __MACH_TEGRA_BBC_PROXY_H */

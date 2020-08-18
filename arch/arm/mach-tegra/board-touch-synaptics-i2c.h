/*
 * arch/arm/mach-tegra/board-touch.h
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
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


#ifndef _MACH_TEGRA_BOARD_TOUCH_SYNAPTICS_H
#define _MACH_TEGRA_BOARD_TOUCH_SYNAPTICS_H
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include "board-touch-synaptics-i2c.h"
//#include "board-ceres.h"
#include <linux/input/synaptics_dsx.h> 
#include "board.h"

//#define TINNO_TP_2_CAM 
//extern struct synaptics_dsx_cap_button_map synaptics_button_map;

//int synaptics_touchpad_gpio_setup(void *gpio_data, bool configure);
//int __init touch_init_synaptics_i2c(struct spi_board_info *board_info,int board_info_size);
int __init touch_init_synaptics_i2c(void);
#endif

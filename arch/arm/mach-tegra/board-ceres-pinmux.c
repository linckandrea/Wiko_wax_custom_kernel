/*
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <mach/pinmux.h>
#include <mach/gpio-tegra.h>
#include "board.h"
#include "tegra-board-id.h"
#include "board-ceres.h"
#include "devices.h"
#include "gpio-names.h"

#include <mach/pinmux-t14.h>

#define DEFAULT_DRIVE(_name)					\
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_DISABLE,			\
		.schmitt = TEGRA_SCHMITT_ENABLE,		\
		.drive = TEGRA_DRIVE_DIV_1,			\
		.pull_down = TEGRA_PULL_31,			\
		.pull_up = TEGRA_PULL_31,			\
		.slew_rising = TEGRA_SLEW_SLOWEST,		\
		.slew_falling = TEGRA_SLEW_SLOWEST,		\
	}
/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 */
#define SET_DRIVE(_name, _hsm, _schmitt, _drive, _pulldn_drive, _pullup_drive, _pulldn_slew, _pullup_slew) \
	{                                               \
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,   \
		.hsm = TEGRA_HSM_##_hsm,                    \
		.schmitt = TEGRA_SCHMITT_##_schmitt,        \
		.drive = TEGRA_DRIVE_##_drive,              \
		.pull_down = TEGRA_PULL_##_pulldn_drive,    \
		.pull_up = TEGRA_PULL_##_pullup_drive,		\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,   \
		.slew_falling = TEGRA_SLEW_##_pullup_slew,	\
	}

/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * drive_type - Drive type to be used depending on the resistors.
 */

#define SET_DRIVE_WITH_TYPE(_name, _hsm, _schmitt, _drive, _pulldn_drive,\
		_pullup_drive, _pulldn_slew, _pullup_slew, _drive_type)	\
	{								\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,		\
		.hsm = TEGRA_HSM_##_hsm,				\
		.schmitt = TEGRA_SCHMITT_##_schmitt,			\
		.drive = TEGRA_DRIVE_##_drive,				\
		.pull_down = TEGRA_PULL_##_pulldn_drive,		\
		.pull_up = TEGRA_PULL_##_pullup_drive,			\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,		\
		.slew_falling = TEGRA_SLEW_##_pullup_slew,		\
		.drive_type = TEGRA_DRIVE_TYPE_##_drive_type,		\
	}

#define DEFAULT_PINMUX(_pingroup, _mux, _pupd, _tri, _io)	\
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_DEFAULT,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define I2C_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_##_od,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
	}

#define CEC_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od)   \
	{							\
		.pingroup   = TEGRA_PINGROUP_##_pingroup,	\
		.func       = TEGRA_MUX_##_mux,			\
		.pupd       = TEGRA_PUPD_##_pupd,		\
		.tristate   = TEGRA_TRI_##_tri,			\
		.io         = TEGRA_PIN_##_io,			\
		.lock       = TEGRA_PIN_LOCK_##_lock,		\
		.od         = TEGRA_PIN_OD_##_od,		\
		.ioreset    = TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define GPIO_PINMUX(_pingroup, _pupd, _tri, _io)	\
{							\
	.pingroup   = TEGRA_PINGROUP_##_pingroup,	\
	.func       = TEGRA_MUX_SAFE,			\
	.pupd       = TEGRA_PUPD_##_pupd,		\
	.tristate   = TEGRA_TRI_##_tri,			\
	.io         = TEGRA_PIN_##_io,			\
	.lock       = TEGRA_PIN_LOCK_DEFAULT,		\
	.od         = TEGRA_PIN_OD_DEFAULT,		\
	.ioreset    = TEGRA_PIN_IO_RESET_DEFAULT,	\
}

#define UNUSED_PINMUX(_pingroup)			\
{							\
	.pingroup   = TEGRA_PINGROUP_##_pingroup,	\
	.func       = TEGRA_MUX_SAFE,			\
	.pupd       = TEGRA_PUPD_PULL_DOWN,		\
	.tristate   = TEGRA_TRI_TRISTATE,		\
	.io         = TEGRA_PIN_OUTPUT,			\
	.lock       = TEGRA_PIN_LOCK_DEFAULT,		\
	.od         = TEGRA_PIN_OD_DEFAULT,		\
	.ioreset    = TEGRA_PIN_IO_RESET_DEFAULT,	\
}

/* We are disabling this code for now. */
#define GPIO_INIT_PIN_MODE(_gpio, _is_input, _value)	\
	{					\
		.gpio_nr	= _gpio,	\
		.is_input	= _is_input,	\
		.value		= _value,	\
	}

static __initdata struct tegra_drive_pingroup_config ceres_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
	/* Audio DAP */
	SET_DRIVE(DAP1, DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),
	SET_DRIVE(DAP2, DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* SDMMC1 */
	SET_DRIVE(SDIO1, ENABLE, DISABLE, DIV_1, 54, 70, FASTEST, FASTEST),

	/* SDMMC3 */
	SET_DRIVE(SDIO3, ENABLE, DISABLE, DIV_1, 23, 41, FASTEST, FASTEST),

	/* SDMMC4 */
	SET_DRIVE_WITH_TYPE(GMA, ENABLE, DISABLE, DIV_1, 1, 2, FASTEST,
								FASTEST, 1),
	/* SPI2 */
	SET_DRIVE(SPI2, ENABLE, ENABLE, DIV_1, 16, 16, SLOWEST, SLOWEST),

	/* SPI3 */
	SET_DRIVE(SPI3, ENABLE, ENABLE, DIV_1, 16, 16, SLOWEST, SLOWEST),

	SET_DRIVE(AO1, ENABLE, ENABLE, DIV_1, 0, 0, SLOWEST, SLOWEST),

	SET_DRIVE(DDC, ENABLE, ENABLE, DIV_1, 0, 0, FASTEST, SLOWEST),

	SET_DRIVE(GME, ENABLE, ENABLE, DIV_1, 0, 0, SLOWEST, SLOWEST),
};


#include "board-ceres-pinmux-t14x.h"
#include "board-atlantis-pinmux-t14x.h"

static void __init ceres_gpio_init_configure(void)
{
	int len;
	int i;
	struct gpio_init_pin_info *pins_info;

	len = ARRAY_SIZE(init_gpio_mode_ceres_common);
	pins_info = init_gpio_mode_ceres_common;

	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}
}

static void __init atlantis_gpio_init_configure(void)
{
	int len;
	int i;
	struct gpio_init_pin_info *pins_info;

	len = ARRAY_SIZE(init_gpio_mode_atlantis_common);
	pins_info = init_gpio_mode_atlantis_common;

	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}
}

int __init ceres_pinmux_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	/*BUG_ON(board_info.board_id != BOARD_E1565);*/

	switch (board_info.board_id) {
	case BOARD_E1680:
	case BOARD_E1681:
	case BOARD_E1683:
	case BOARD_E1690:
		ceres_gpio_init_configure();
		tegra_pinmux_config_table(ceres_pinmux_common,
			ARRAY_SIZE(ceres_pinmux_common));
		break;
	case BOARD_E1670:
	case BOARD_E1671:
	case BOARD_E1740:
		atlantis_gpio_init_configure();
		tegra_pinmux_config_table(atlantis_pinmux_common,
			ARRAY_SIZE(atlantis_pinmux_common));
		break;
	}

	tegra_drive_pinmux_config_table(ceres_drive_pinmux,
					ARRAY_SIZE(ceres_drive_pinmux));

	switch (board_info.board_id) {
	case BOARD_E1680:
	case BOARD_E1681:
	case BOARD_E1683:
	case BOARD_E1690:
		tegra_pinmux_config_table(ceres_unused_pins_lowpower,
			ARRAY_SIZE(ceres_unused_pins_lowpower));
		break;
	case BOARD_E1670:
	case BOARD_E1671:
	case BOARD_E1740:
		tegra_pinmux_config_table(atlantis_unused_pins_lowpower,
			ARRAY_SIZE(atlantis_unused_pins_lowpower));
		break;
	default:
		pr_err("%s: board_id=%#x unknown. Tegra14x board detect failed.\n",
			__func__, board_info.board_id);
		return -EINVAL;
	}

	return 0;
}

static struct gpio_init_pin_info ceres_suspend_pin_state[] = {
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PH6, false, 1),
};

static struct gpio_init_pin_info atlantis_suspend_pin_state[] = {
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PH6, false, 1),
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PL3, false, 0),
};

int ceres_pinmux_suspend(void)
{
	struct board_info board_info;
	int len;
	struct gpio_init_pin_info *pins_info;
	int i;

	tegra_get_board_info(&board_info);
	switch (board_info.board_id) {
	case BOARD_E1680:
	case BOARD_E1681:
	case BOARD_E1683:
	case BOARD_E1690:
		pins_info = ceres_suspend_pin_state;
		len = ARRAY_SIZE(ceres_suspend_pin_state);
		break;
	case BOARD_E1670:
	case BOARD_E1671:
	case BOARD_E1740:
		pins_info = atlantis_suspend_pin_state;
		len = ARRAY_SIZE(atlantis_suspend_pin_state);
		break;
	default:
		pr_err("%s: board_id=%#x unknown. Tegra14x board detect failed.\n",
			__func__, board_info.board_id);
		return -EINVAL;
	}
	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}
	return 0;
}

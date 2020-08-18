/*
 * arch/arm/mach-tegra/board-dolak-pinmux.c
 *
 * Copyright (C) 2011-2012 NVIDIA Corporation
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
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/bug.h>
#include <mach/pinmux.h>
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


static __initdata struct tegra_drive_pingroup_config dolak_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
};

#define DEFAULT_PINMUX(_pingroup, _mux, _pupd, _tri, _io)	\
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
	}

static __initdata struct tegra_pingroup_config dolak_pinmux[] = {
	DEFAULT_PINMUX(SDMMC1_CLK,      SDMMC1,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_CMD,      SDMMC1,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT3,     SDMMC1,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT2,     SDMMC1,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT1,     SDMMC1,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT0,     SDMMC1,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(UART1_TXD,       UARTA,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(UART1_RXD,       UARTA,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART1_RTS_N,     UARTA,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(UART1_CTS_N,     UARTA,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART2_RXD,       UARTB,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART2_TXD,       UARTB,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(UART2_RTS_N,     UARTB,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(UART2_CTS_N,     UARTB,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART3_TXD,       UARTC,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(UART3_RXD,       UARTC,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART3_CTS_N,     UARTC,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART3_RTS_N,     UARTC,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(I2C1_SCL,	I2C1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(I2C1_SDA,	I2C1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GEN2_I2C_SCL,    I2C2,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GEN2_I2C_SDA,    I2C2,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_CLK,      SDMMC4,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_CMD,      SDMMC4,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT0,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT1,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT2,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT3,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT4,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT5,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT6,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT7,     SDMMC4,          PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(CAM_I2C_SCL,     I2C3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(CAM_I2C_SDA,     I2C3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(PWR_I2C_SCL,     I2CPWR,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(PWR_I2C_SDA,     I2CPWR,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_ROW0,         KBC,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_ROW1,         KBC,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_ROW2,         KBC,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_COL0,         KBC,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_COL1,         KBC,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(KB_COL2,         KBC,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(OWR,             OWR,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP1_FS,         I2S0,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP1_DIN,        I2S0,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP1_DOUT,       I2S0,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP1_SCLK,       I2S0,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(CLK1_OUT,        EXTPERIPH1,      NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_FS,         I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_DIN,        I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_DOUT,       I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_SCLK,       I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_CLK,      SDMMC3,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_CMD,      SDMMC3,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT0,     SDMMC3,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT1,     SDMMC3,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT2,     SDMMC3,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT3,     SDMMC3,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(BCL,             BCL,             NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GPIO_PO4,	RSVD0,		PULL_UP,    NORMAL,     OUTPUT),
};

void __init dolak_pinmux_init(void)
{
	tegra14x_default_pinmux();
	tegra_pinmux_config_table(dolak_pinmux, ARRAY_SIZE(dolak_pinmux));
	tegra_drive_pinmux_config_table(dolak_drive_pinmux,
					ARRAY_SIZE(dolak_drive_pinmux));
}

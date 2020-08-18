/*
 * arch/arm/mach-tegra/board-m2601.c
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
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/panjit_ts.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/platform_data/tegra_nor.h>
#include <linux/spi/spi.h>
#include <linux/mtd/partitions.h>
#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/pci.h>
#include <mach/audio.h>
#include <asm/mach/flash.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/system.h>
#include <mach/usb_phy.h>
#include <mach/tegra_asoc_vcm_pdata.h>
#include <mach/tegra_fiq_debugger.h>
#include <sound/wm8903.h>
#include <mach/tsensor.h>
#include "board.h"
#include "clock.h"
#include "board-m2601.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "common.h"
#ifdef CONFIG_SENSORS_TMON_TMP411
#include <linux/platform_data/tmon_tmp411.h>
#include "therm-monitor.h"
#endif

/* B00 boards shared GMI address space */
#define GMI_SRAM_BASE_OFFSET	0x0000000
#define GMI_SRAM_PCA9665_BASE_OFFSET	0x1000000

static __initdata struct tegra_clk_init_table m2601_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",		NULL,		0,		true},
	{ "hda",		"pll_p",	108000000,	false},
	{ "hda2codec_2x",	"pll_p",	48000000,	false},
	{ "pwm",		"clk_32k",	32768,		false},
	{ "blink",		"clk_32k",	32768,		true},
	{ "pll_a",		NULL,		552960000,	false},
	/* audio cif clock should be faster than i2s */
	{ "pll_a_out0",		NULL,		24576000,	false},
	{ "d_audio",		"pll_a_out0",	24576000,	false},
	{ "nor",		"pll_p",	102000000,	true},
	{ "uarta",		"pll_p",	480000000,	true},
	{ "uartb",		"pll_p",	480000000,	true},
	{ "uartd",		"pll_p",	480000000,	true},
	{ "uarte",		"pll_p",	480000000,	true},
	{ "sdmmc2",		"pll_p",	52000000,	true},
	{ "sbc2",		"pll_m",	100000000,	true},
	{ "sbc3",		"pll_m",	100000000,	true},
	{ "sbc4",		"pll_m",	100000000,	true},
	{ "sbc5",		"pll_m",	100000000,	true},
	{ "sbc6",		"pll_m",	100000000,	true},
	{ "cpu_g",		"cclk_g",	900000000,	true},
	{ "i2s0",		"pll_a_out0",	24576000,	false},
	{ "i2s1",		"pll_a_out0",	24576000,	false},
	{ "i2s2",		"pll_a_out0",	24576000,	false},
	{ "i2s3",		"pll_a_out0",	24576000,	false},
	{ "i2s4",		"pll_a_out0",	24576000,	false},
	{ "audio0",		"i2s0_sync",	12288000,	false},
	{ "audio1",		"i2s1_sync",	12288000,	false},
	{ "audio2",		"i2s2_sync",	12288000,	false},
	{ "audio3",		"i2s3_sync",	12288000,	false},
	{ "audio4",		"i2s4_sync",	12288000,	false},
	{ "apbif",		"clk_m",	12000000,	false},
	{ "dam0",		"clk_m",	12000000,	true},
	{ "dam1",		"clk_m",	12000000,	true},
	{ "dam2",		"clk_m",	12000000,	true},
	{ "vi",			"pll_p",	470000000,	false},
	{ "vi_sensor",		"pll_p",	150000000,	false},
	{ "vde",		"pll_c",	484000000,	true},
	{ "mpe",		"pll_c",	484000000,	true},
	{ "se",			"pll_m",	625000000,	true},
	{ "i2c1",		"pll_p",	3200000,	true},
	{ "i2c2",		"pll_p",	3200000,	true},
	{ "i2c3",		"pll_p",	3200000,	true},
	{ "i2c4",		"pll_p",	3200000,	true},
	{ "i2c5",		"pll_p",	3200000,	true},
	{ "sdmmc2",		"pll_p",	104000000,	false},
	{"wake.sclk",		NULL,		334000000,	true },
	{ NULL,			NULL,		0,		0},
};

#ifdef CONFIG_SENSORS_TMON_TMP411
struct therm_monitor_data m2601_therm_monitor_data = {
	.brd_ltemp_reg_data = NULL,
	.delta_temp = 4000,
	.delta_time = 2000,
	.remote_offset = 8000,
	.local_temp_update = false,
	.utmip_reg_update = false, /* USB registers update is not
						required for now */
	.i2c_bus_num = I2C_BUS_TMP411,
	.i2c_dev_addrs = I2C_ADDR_TMP411,
	.i2c_dev_name = "tmon-tmp411-sensor",
};
#endif

static struct tegra_i2c_platform_data m2601_i2c1_platform_data = {
	.bus_clk_rate	= 100000,
	.scl_gpio	= -1,
	.sda_gpio	= -1,
};

static struct tegra_i2c_platform_data m2601_i2c2_platform_data = {
	.bus_clk_rate	= 100000,
	.scl_gpio	= -1,
	.sda_gpio	= -1,
	.is_clkon_always = true,
};

static struct tegra_i2c_platform_data m2601_i2c5_platform_data = {
	.bus_clk_rate	= 100000,
	.scl_gpio	= -1,
	.sda_gpio	= -1,
};
static struct tegra_pci_platform_data m2601_pci_platform_data = {
	.port_status[0] = 1,
	.port_status[1] = 1,
	.port_status[2] = 1,
	.use_dock_detect = 0,
	.gpio = 0,
};

static void m2601_pcie_init(void)
{
	tegra_pci_device.dev.platform_data = &m2601_pci_platform_data;
	platform_device_register(&tegra_pci_device);
}
static void m2601_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &m2601_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &m2601_i2c2_platform_data;
	tegra_i2c_device5.dev.platform_data = &m2601_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);
}

static struct platform_device *m2601_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&debug_uartb_device,
	&debug_uartd_device,
	&debug_uarte_device,
};
static struct clk *debug_uart_clk;

static void __init uart_debug_init(void)
{
	/* UARTA is the debug port. */
	pr_info("Selecting UARTA as the debug console\n");
	m2601_uart_devices[0] = &debug_uarta_device;
	debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
	debug_uartb_device.id = PLAT8250_DEV_PLATFORM1;
	debug_uartd_device.id = PLAT8250_DEV_PLATFORM2;
	debug_uarte_device.id = PLAT8250_DEV_FOURPORT;
}

static void __init m2601_uart_init(void)
{
	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, 408000000);
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(m2601_uart_devices,
				ARRAY_SIZE(m2601_uart_devices));
}

#if defined(CONFIG_SPI_TEGRA) && defined(CONFIG_SPI_SPIDEV)
static struct spi_board_info tegra_spi_devices[] __initdata = {
	{
		.modalias = "spidev",
		.bus_num = 1,
		.chip_select = 1,
		.mode = SPI_MODE_0,
		.max_speed_hz = 18000000,
		.platform_data = NULL,
		.irq = 0,
	},
};

static void __init m2601_register_spidev(void)
{
	spi_register_board_info(tegra_spi_devices,
			ARRAY_SIZE(tegra_spi_devices));
}
#else
#define m2601_register_spidev() do {} while (0)
#endif

#ifdef CONFIG_SATA_AHCI_TEGRA
static void m2601_sata_init(void)
{
	platform_device_register(&tegra_sata_device);
}
#else
static void m2601_sata_init(void) { }
#endif


static void m2601_spi_init(void)
{
	tegra_spi_device3.name = "spi_slave_tegra";
	platform_device_register(&tegra_spi_device2);
	m2601_register_spidev();
}
static struct platform_device *m2601_devices[] __initdata = {
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_wdt0_device,
};

static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 63,
		.xcvr_setup_offset = 6,
		.xcvr_use_fuses = 1,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_use_lsb = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 63,
		.xcvr_setup_offset = 6,
		.xcvr_use_fuses = 1,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_use_lsb = 1,
	},
};

static void m2601_usb_init(void)
{
	tegra_ehci1_device.dev.platform_data = &tegra_ehci1_utmi_pdata;
	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_utmi_pdata;
	platform_device_register(&tegra_ehci1_device);
	platform_device_register(&tegra_ehci2_device);
}

#define CHIP_SIZE_MSP14LV320 0x40000000ULL

static struct tegra_nor_platform_data m2601_nor_data = {
	.flash = {
		.map_name = "jedec_probe",
		.width = 4,
	},
	.chip_parms = {
		.MuxMode = NorMuxMode_ADNonMux,
		.ReadMode = NorReadMode_Page,
		.PageLength = NorPageLength_16Word,
		.ReadyActive = NorReadyActive_WithData,
		.timing_default = {
			.timing0 = 0xB040C310,
			.timing1 = 0x000a0a04,
		},
		.timing_read = {
			.timing0 = 0xB040C310,
			.timing1 = 0x000a0a04,
		},
	},
	.gmi_rst_n_gpio = TEGRA_GPIO_PI4,
	.gmi_oe_n_gpio = TEGRA_GPIO_PI1,
};

#define M2601_NUM_CS	8
static struct cs_info m2601_cs_info[M2601_NUM_CS] = {
			{
				.cs = CS_0,
				.num_cs_gpio = 0,
				.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE),
				.size = CHIP_SIZE_MSP14LV320,
				.phys = TEGRA_NOR_FLASH_BASE,
			},
			{
				.cs = CS_1,
				.num_cs_gpio = 0,
				.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE),
				.size = CHIP_SIZE_MSP14LV320,
				.phys = TEGRA_NOR_FLASH_BASE,
			},
			{
				.cs = CS_2,
				.num_cs_gpio = 0,
				.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE),
				.size = CHIP_SIZE_MSP14LV320,
				.phys = TEGRA_NOR_FLASH_BASE,
			},
			{
				.cs = CS_3,
				.num_cs_gpio = 0,
				.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE),
				.size = CHIP_SIZE_MSP14LV320,
				.phys = TEGRA_NOR_FLASH_BASE,
			},
			{
				.cs = CS_4,
				.gpio_cs = {
						"PX7",
						TEGRA_GPIO_PX7,
						LOW
					},
				.num_cs_gpio = 1,
				.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE),
				.size = CHIP_SIZE_MSP14LV320,
				.phys = TEGRA_NOR_FLASH_BASE,
			},
			{
				.cs = CS_4,
				.gpio_cs = {
						"PX7",
						TEGRA_GPIO_PX7,
						HIGH
					},
				.num_cs_gpio = 1,
				.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE),
				.size = CHIP_SIZE_MSP14LV320,
				.phys = TEGRA_NOR_FLASH_BASE,
			},
			{
				.cs = CS_5,
				.gpio_cs = {
						"PX7",
						TEGRA_GPIO_PX7,
						LOW
					},
				.num_cs_gpio = 1,
				.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE),
				.size = CHIP_SIZE_MSP14LV320,
				.phys = TEGRA_NOR_FLASH_BASE,
			},
			{
				.cs = CS_5,
				.gpio_cs = {
						"PX7",
						TEGRA_GPIO_PX7,
						HIGH
					},
				.num_cs_gpio = 1,
				.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE),
				.size = CHIP_SIZE_MSP14LV320,
				.phys = TEGRA_NOR_FLASH_BASE,
			}
};

#define M2601_NUM_GPIO_ADDR	3
static struct gpio_addr	m2601_gpio_addr[M2601_NUM_GPIO_ADDR] = {
			{TEGRA_GPIO_PK6, 27},
			{TEGRA_GPIO_PK5, 28},
			{TEGRA_GPIO_PV3, 29},
};


static struct tegra_asoc_vcm_platform_data m2601_audio_pdata = {
	.codec_info[0] = {
		.i2s_format = format_tdm,
		/* Audio Codec is Master */
		.master = 1,
		.num_slots = 8,
		.slot_width = 32,
		.tx_mask = 0xff,
		.rx_mask = 0xff,
	},
};

static struct platform_device tegra_snd_m2601 = {
	.name       = "tegra-snd-m2601",
	.id = 0,
	.dev    = {
		.platform_data = &m2601_audio_pdata,
	},
};

static struct i2c_board_info __initdata ad1937_i2s2_board_info = {
	I2C_BOARD_INFO("ad1937", 0x07),
};

static void m2601_i2s_audio_init(void)
{
	/* Register the PCM driver for TDM mode */
	platform_device_register(&tegra_tdm_pcm_device);
	/* Register the I2S2 controller driver */
	platform_device_register(&tegra_i2s_device2);
	/* Register the AHUB controller */
	platform_device_register(&tegra_ahub_device);

	i2c_register_board_info(0, &ad1937_i2s2_board_info, 1);
	platform_device_register(&tegra_snd_m2601);
}

static void m2601_nor_init(void)
{
	tegra_nor_device.resource[2].end = TEGRA_NOR_FLASH_BASE +
						TEGRA_NOR_FLASH_SIZE - 1;
	m2601_nor_data.info.cs = kzalloc(sizeof(struct cs_info) *
						M2601_NUM_CS, GFP_KERNEL);
	if (!m2601_nor_data.info.cs)
		BUG();

	m2601_nor_data.info.num_chips = M2601_NUM_CS;

	memcpy(m2601_nor_data.info.cs, m2601_cs_info,
				sizeof(struct cs_info) * M2601_NUM_CS);

	m2601_nor_data.addr.addr = kzalloc(sizeof(struct gpio_addr) *
					M2601_NUM_GPIO_ADDR, GFP_KERNEL);
	if (!m2601_nor_data.addr.addr)
		BUG();

	m2601_nor_data.addr.num_gpios = M2601_NUM_GPIO_ADDR;

	memcpy(m2601_nor_data.addr.addr, m2601_gpio_addr,
			sizeof(struct gpio_addr) * M2601_NUM_GPIO_ADDR);

	tegra_nor_device.dev.platform_data = &m2601_nor_data;
	platform_device_register(&tegra_nor_device);
}

static struct tegra_nor_chip_parms m2601_gmi_sram_data = {
			.MuxMode = NorMuxMode_ADNonMux,
			.ReadMode = NorReadMode_Async,
			.PageLength = NorPageLength_8Word,
			.ReadyActive = NorReadyActive_WithData,
			.BusWidth = 4,
			.timing_default = {
				.timing0 = 0xB050c110,
				.timing1 = 0x00050901,
			},
			.timing_read = {
				.timing0 = 0xB050C110,
				.timing1 = 0x00050901,
			},
			.csinfo = {
				.cs = CS_7,
				.num_cs_gpio = 0,
				.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE +
							GMI_SRAM_BASE_OFFSET),
				.size = 32768,
				.phys = TEGRA_NOR_FLASH_BASE,
			}
};
struct platform_device tegra_gmi_sram_device = {
	.name = "tegra-gmi-char",
	.id = -1,
	.dev = {
		.coherent_dma_mask = 0xffffffff,
	},
};
static void m2601_gmi_sram_init(void)
{
	tegra_gmi_sram_device.dev.platform_data = &m2601_gmi_sram_data;
	platform_device_register(&tegra_gmi_sram_device);

}

static struct tegra_nor_chip_parms m2601_gmi_pca_data = {
			.MuxMode = NorMuxMode_ADNonMux,
			.ReadMode = NorReadMode_Async,
			.PageLength = NorPageLength_8Word,
			.ReadyActive = NorReadyActive_WithData,
			.BusWidth = 4,
			.timing_default = {
				.timing0 = 0xB050c110,
				.timing1 = 0x00050901,
			},
			.timing_read = {
				.timing0 = 0xB050C110,
				.timing1 = 0x00050901,
			},
			.csinfo = {
				.cs = CS_6,
				 .gpio_cs = {
							"K05",
							TEGRA_GPIO_PK5,
							LOW
					},
				.num_cs_gpio = 0,
				.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE),
				.size = 32768,
				.phys = TEGRA_NOR_FLASH_BASE,
			}

};

static struct resource tegra_gmi_pca_resources[] = {
	[0] = {
			.name	= "TEGRA_GPIO_PW2_INT",
			.start  = TEGRA_GPIO_PW2,
			.end    = TEGRA_GPIO_PW2,
			.flags  = IORESOURCE_IO,
	},
};

struct platform_device tegra_gmi_pca_device = {
	.name = "i2c-pca-gmi",
	.id = -1,
	.dev = {
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(tegra_gmi_pca_resources),
	.resource       = tegra_gmi_pca_resources,

};

static void m2601_gmi_pca_init(void)
{
	if (system_rev != TEGRA_M2601_SKU1_A00) {

		m2601_gmi_pca_data.csinfo.cs = CS_7;
		m2601_gmi_pca_data.csinfo.virt =
				IO_ADDRESS(TEGRA_NOR_FLASH_BASE +
					(m2601_gmi_pca_data.BusWidth *
						GMI_SRAM_PCA9665_BASE_OFFSET));
		m2601_gmi_pca_data.csinfo.phys =
			TEGRA_NOR_FLASH_BASE + GMI_SRAM_PCA9665_BASE_OFFSET;
		m2601_gmi_pca_data.csinfo.size = 4;
		m2601_gmi_pca_data.csinfo.gpio_cs.gpio_num = TEGRA_GPIO_PZ0;

		/* LOW selects PCA9665 & HIGH selects PCA9663 */
		m2601_gmi_pca_data.csinfo.gpio_cs.value = LOW;
		strcpy(m2601_gmi_pca_data.csinfo.gpio_cs.label, "PZ0");
	}
	tegra_gmi_pca_device.dev.platform_data = &m2601_gmi_pca_data;
	platform_device_register(&tegra_gmi_pca_device);

}

static void __init tegra_m2601_init(void)
{
	tegra_init_board_info();
	tegra_clk_init_from_table(m2601_clk_init_table);
	tegra_enable_pinmux();
	tegra_soc_device_init("m2601");
	m2601_pinmux_init();
	m2601_i2c_init();
	m2601_sata_init();
	m2601_i2s_audio_init();
	m2601_uart_init();
	m2601_usb_init();
	m2601_sdhci_init();
	m2601_spi_init();
	platform_add_devices(m2601_devices, ARRAY_SIZE(m2601_devices));
	m2601_nor_init();
	m2601_pcie_init();
	m2601_gmi_sram_init();
	m2601_gmi_pca_init();
	m2601_gpio_init();
#ifdef CONFIG_SENSORS_TMON_TMP411
	register_therm_monitor(&m2601_therm_monitor_data);
#endif
}

MACHINE_START(M2601, "m2601")
	.atag_offset    = 0x100,
	.soc            = &tegra_soc_desc,
	.init_irq       = tegra_dt_init_irq,
	.init_early     = tegra30_init_early,
	.init_machine   = tegra_m2601_init,
	.map_io         = tegra_map_common_io,
	.timer          = &tegra_timer,
	.restart	= tegra_assert_system_reset,
	.handle_irq     = gic_handle_irq,
MACHINE_END

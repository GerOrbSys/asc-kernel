/*
 * DM368 Leopard Board
 *
 * Derived from: arch/arm/mach-davinci/board-dm365-evm.c
 * RidgeRun Copyright (C) 2011.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**************************************************************************
 * Included Files
 **************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/autoconf.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/leds.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/mux.h>
#include <mach/hardware.h>
#include <mach/dm365.h>
#include <mach/psc.h>
#include <mach/common.h>
#include <mach/i2c.h>
#include <mach/serial.h>
#include <mach/mmc.h>
#include <mach/nand.h>
#include <mach/keyscan.h>
#include <mach/gpio.h>
#include <linux/videodev2.h>
#include <media/davinci/videohd.h>

#define DM365_EVM_PHY_MASK		(0x2)
#define DM365_EVM_MDIO_FREQUENCY	(2200000) /* PHY bus frequency */

#define DM365_ASYNC_EMIF_CONTROL_BASE	0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE	0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE	0x04000000

static struct i2c_board_info i2c_info[] = {
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq	      = 400	/* kHz */,
	.bus_delay    	= 0	/* usec */,
	.sda_pin        = 21,
	.scl_pin        = 20,
};

static struct v4l2_input mt9j003_inputs[] = {
	{
		.index = 0,
		.name  = "Camera",
		.type  = V4L2_INPUT_TYPE_CAMERA
	}
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
		.module_name = "mt9j003",
		.is_camera = 1,
		.grp_id = VPFE_SUBDEV_MT9J003,
		.num_inputs = ARRAY_SIZE(mt9j003_inputs),
		.inputs = mt9j003_inputs,
		.ccdc_if_params = {
			.if_type = VPFE_RAW_BAYER,
			.hdpol = VPFE_PINPOL_NEGATIVE,
			.vdpol = VPFE_PINPOL_NEGATIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("mt9j003", 0x10),
			.platform_data = (void *)1,
		},
	}
};

static struct vpfe_config vpfe_cfg = {
       .num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
       .sub_devs = vpfe_sub_devs,
       .card_name = "DM368 ASC",
       .ccdc = "DM365 ISIF",
       .num_clocks = 1,
       .clocks = {"vpss_master"},
};

/*Need to review if this is necessary*/
static struct davinci_mmc_config dm368asc_mmc_config = {
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

static void dm368asc_emac_configure(void)
{
	/*
	 * EMAC pins are multiplexed with GPIO and UART
	 * Further details are available at the DM368 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 125 - 127
	 */
	davinci_cfg_reg(DM365_EMAC_TX_EN);
	davinci_cfg_reg(DM365_EMAC_TX_CLK);
	davinci_cfg_reg(DM365_EMAC_COL);
	davinci_cfg_reg(DM365_EMAC_TXD3);
	davinci_cfg_reg(DM365_EMAC_TXD2);
	davinci_cfg_reg(DM365_EMAC_TXD1);
	davinci_cfg_reg(DM365_EMAC_TXD0);
	davinci_cfg_reg(DM365_EMAC_RXD3);
	davinci_cfg_reg(DM365_EMAC_RXD2);
	davinci_cfg_reg(DM365_EMAC_RXD1);
	davinci_cfg_reg(DM365_EMAC_RXD0);
	davinci_cfg_reg(DM365_EMAC_RX_CLK);
	davinci_cfg_reg(DM365_EMAC_RX_DV);
	davinci_cfg_reg(DM365_EMAC_RX_ER);
	davinci_cfg_reg(DM365_EMAC_CRS);
	davinci_cfg_reg(DM365_EMAC_MDIO);
	davinci_cfg_reg(DM365_EMAC_MDCLK);

	/*
	 * EMAC interrupts are multiplexed with GPIO interrupts
	 * Details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 133 - 134
	 */
	davinci_cfg_reg(DM365_INT_EMAC_RXTHRESH);
	davinci_cfg_reg(DM365_INT_EMAC_RXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_TXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_MISCPULSE);
}

static void dm368asc_mmc_configure(void)
{
	/*
	 * MMC/SD pins are multiplexed with GPIO and EMIF
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 118, 128 - 131
	 */
	davinci_cfg_reg(DM365_SD1_CLK);
	davinci_cfg_reg(DM365_SD1_CMD);
	davinci_cfg_reg(DM365_SD1_DATA3);
	davinci_cfg_reg(DM365_SD1_DATA2);
	davinci_cfg_reg(DM365_SD1_DATA1);
	davinci_cfg_reg(DM365_SD1_DATA0);
}

static void __init asc_init_i2c(void)
{
	davinci_cfg_reg(DM365_GPIO20);
	gpio_request(20, "i2c-scl");
	gpio_direction_output(20, 0);
	davinci_cfg_reg(DM365_I2C_SCL);

	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, i2c_info, ARRAY_SIZE(i2c_info));
}

static struct davinci_nand_pdata davinci_nand_data = {
	.mask_chipsel		= 0,
	.ecc_mode		= NAND_ECC_HW,
	.options		= NAND_SKIP_BBTSCAN | NAND_NO_SUBPAGE_WRITE,
	.ecc_bits		= 4,
};

static struct resource davinci_nand_resources[] = {
	{
		.start		= DM365_ASYNC_EMIF_DATA_CE0_BASE,
		.end		= DM365_ASYNC_EMIF_DATA_CE0_BASE + SZ_32M - 1,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= DM365_ASYNC_EMIF_CONTROL_BASE,
		.end		= DM365_ASYNC_EMIF_CONTROL_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device davinci_nand_device = {
	.name			= "davinci_nand",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(davinci_nand_resources),
	.resource		= davinci_nand_resources,
	.dev			= {
		.platform_data	= &davinci_nand_data,
	},
};

static struct platform_device *dm368_asc_devices[] __initdata = {
	&davinci_nand_device,
};

void enable_lcd(void)
{
}
EXPORT_SYMBOL(enable_lcd);

void enable_hd_clk(void)
{
}
EXPORT_SYMBOL(enable_hd_clk);


static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};

static void __init dm368_asc_map_io(void)
{
	/* setup input configuration for VPFE input devices */
	dm365_set_vpfe_config(&vpfe_cfg);
	dm365_init();
}

static __init void dm368_asc_init(void)
{
	asc_init_i2c();
	davinci_serial_init(&uart_config);

	dm368asc_emac_configure();
	dm368asc_mmc_configure();

	davinci_setup_mmc(0, &dm368asc_mmc_config);

	// Mux pin for Sensor module power and request gpio for sensor power control
  davinci_cfg_reg(DM365_GPIO30);
	gpio_request(30, "camera-power");

	dm365_init_rtc();

	platform_add_devices(dm368_asc_devices,
		ARRAY_SIZE(dm368_asc_devices));
}

static __init void dm368_asc_irq_init(void)
{
	davinci_irq_init();
}

MACHINE_START(DAVINCI_DM368_ASC, "DM368 ASC")
	.phys_io	    = IO_PHYS,
	.io_pg_offst	= (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params	= (0x80000100),
	.map_io		    = dm368_asc_map_io,
	.init_irq	    = dm368_asc_irq_init,
	.timer		    = &davinci_timer,
	.init_machine	= dm368_asc_init,
MACHINE_END

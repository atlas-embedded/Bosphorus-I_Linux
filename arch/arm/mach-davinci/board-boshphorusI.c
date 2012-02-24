/*
 * ATLAS EMBEDDED SYSTEMS LTD. (c) 2011
 * 
 * Bosphorus-I Computer On Modules
 * 
 * 2011-12-28, 15:00
 * 
 * based on TI DA850/OMAP-L138 EVM board
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Derived from: arch/arm/mach-davinci/board-da830-evm.c
 * Original Copyrights follow:
 *
 * 2007, 2009 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/machine.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/delay.h>
#include <linux/i2c-gpio.h>
#include <linux/pwm_backlight.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/cp_intc.h>
#include <mach/da8xx.h>
#include <mach/nand.h>
#include <mach/mux.h>
#include <mach/aemif.h>
#include <mach/spi.h>
#include <mach/flash.h>
#include <mach/usb.h>
#include <mach/vpif.h>
#include <media/davinci/videohd.h>

#include <media/tvp514x.h>
#include <linux/spi/ads7846.h>

#define BOSPHORUSI_PHY_ID		"0:00"
// LCD GPIO signals
#define DA850_LCD_PWR_PIN		GPIO_TO_PIN(2, 8)
#define DA850_LCD_BL_PIN		GPIO_TO_PIN(1, 12)
// MMCSD GPIO signals
#define DA850_MMCSD_CD_PIN		GPIO_TO_PIN(4, 0)
#define DA850_MMCSD_WP_PIN		GPIO_TO_PIN(4, 1) // for SD CARD 

#define TS_PEN_PIN              	GPIO_TO_PIN(0, 13) //PEN IRQ PIN


#ifdef CONFIG_BOSP_MICROSD_WP_SUPPORT
#define MMCSD_WP_USING 0
#else
#define MMCSD_WP_USING 1
#endif

#define DAVINCI_BACKLIGHT_MAX_BRIGHTNESS	250
#define DAVINVI_BACKLIGHT_DEFAULT_BRIGHTNESS	250
#define DAVINCI_PWM_PERIOD_NANO_SECONDS		(10000 * 10)

#define PWM_DEVICE_ID	"ehrpwm.1"	// This value must be changed for Redriver 1.1


#if defined (CONFIG_BOSP_VGA_640_480) || defined (CONFIG_BOSP_VGA_800_600)
#define HAS_VGA 1
#else
#define HAS_VGA 0
#endif

// PWM BACKLIGHT
static struct platform_pwm_backlight_data bosphorusI_backlight_data = {
	.pwm_id		= PWM_DEVICE_ID,
	.ch		= 0,
	.max_brightness	= DAVINCI_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness	= DAVINVI_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns	= DAVINCI_PWM_PERIOD_NANO_SECONDS,
};

static struct platform_device bosphorusI_backlight = {
	.name		= "pwm-backlight",
	.id		= -1,
	.dev		= {
	.platform_data	= &bosphorusI_backlight_data,
	}
};

// SPI NOR FLASH
static struct davinci_spi_platform_data bosphorusI_spi1_pdata = {
	.version		= SPI_VERSION_2,
	.num_chipselect = 2,
	.intr_line      = 1,
};

static struct mtd_partition bosphorusI_spiflash_part[] = {
	[0] = {
		.name 		= "UBL",
		.offset 	= 0,
		.size 		= SZ_64K,
		.mask_flags 	= MTD_WRITEABLE,
	},
	[1] = {
		.name 		= "U-Boot",
		.offset 	= MTDPART_OFS_APPEND,
		.size 		= SZ_512K,
		.mask_flags 	= MTD_WRITEABLE,
	},
	[2] = {
		.name 		= "U-Boot-Env",
		.offset 	= MTDPART_OFS_APPEND,
		.size 		= SZ_64K,
		.mask_flags 	= MTD_WRITEABLE,
	},
	[3] = {
		.name 		= "Kernel",
		.offset 	= MTDPART_OFS_APPEND,
		.size 		= SZ_2M + SZ_512K,
		.mask_flags 	= 0,
	},
	[4] = {
		.name 		= "Filesystem",
		.offset 	= MTDPART_OFS_APPEND,
		.size 		= SZ_4M,
		.mask_flags 	= 0,
	},
	[5] = {
		.name 		= "MAC-Address",
		.offset 	= SZ_8M - SZ_64K,
		.size 		= SZ_64K,
		.mask_flags 	= MTD_WRITEABLE,
	},
};

static struct flash_platform_data bosphorusI_spiflash_data = {
	.name		= "m25p80",
	.parts		= bosphorusI_spiflash_part,
	.nr_parts	= ARRAY_SIZE(bosphorusI_spiflash_part),
};

static struct davinci_spi_config bosphorusI_spiflash_cfg = {
	.io_type	= SPI_IO_TYPE_DMA,
	.c2tdelay	= 8,
	.t2cdelay	= 8,
};

static struct davinci_spi_config spi_ts_config = {
         .io_type        = SPI_IO_TYPE_DMA,
         .c2tdelay       = 8,
         .t2cdelay       = 8,
};

static int ads7846_get_pendown_state(void) {
        return !gpio_get_value(TS_PEN_PIN);
}

static struct ads7846_platform_data ts_pdata = {
	.x_max                  = 0x0fff,
        .y_max                  = 0x0fff,
        .x_plate_ohms           = 100,
        .pressure_max           = 255,
        .debounce_max           = 30,
        .debounce_tol           = 10,
        .debounce_rep           = 1,
        .get_pendown_state      = ads7846_get_pendown_state,
        .keep_vref_on           = 1,
        .vref_mv                = 2500,        
        .settle_delay_usecs     = 100,
        .gpio_pendown           = TS_PEN_PIN,
        .wakeup                 = true,
};

static struct spi_board_info bosphorusI_spi1_info[] = {
	[0]={
		.modalias		= "m25p80",
		.platform_data		= &bosphorusI_spiflash_data,
		.controller_data	= &bosphorusI_spiflash_cfg,
		.mode			= SPI_MODE_0,
		.max_speed_hz		= 30000000,
		.bus_num		= 1,
		.chip_select		= 0,
	      },
	[1]={
                .modalias               = "ads7846",
                .platform_data          = &ts_pdata,
                .controller_data        = &spi_ts_config,
                .mode                   = SPI_MODE_0,
                .max_speed_hz           = 1000000,
                .bus_num                = 1,
                .chip_select            = 1,
	      },
};

static void m25p80_notify_add(struct mtd_info *mtd)
{
	char *mac_addr = davinci_soc_info.emac_pdata->mac_addr;
	size_t retlen;

	if (!strcmp(mtd->name, "MAC-Address")) {
		mtd->read(mtd, 0, ETH_ALEN, &retlen, mac_addr);
		if (retlen == ETH_ALEN)
			pr_info("Read MAC addr from SPI Flash: %pM\n",
					mac_addr);
	}
}

static struct mtd_notifier spi_notifier = {
	.add    = m25p80_notify_add,
};

static void __init bosphorusI_init_spi1(struct spi_board_info *info, unsigned len)
{
	int ret;
	if (gpio_request(TS_PEN_PIN, "TSC2046 pendown") < 0) {
                printk(KERN_ERR "can't get TSC2046 pen down GPIO\n");
                return;
        }
	gpio_direction_input(TS_PEN_PIN);
	gpio_export(TS_PEN_PIN, 0);
        gpio_set_debounce(TS_PEN_PIN, 0xa);
	bosphorusI_spi1_info[1].irq = gpio_to_irq(TS_PEN_PIN);
	
	ret = spi_register_board_info(info, len);
	if (ret)
		pr_warning("failed to register board info : %d\n", ret);

	ret = da8xx_register_spi(1, &bosphorusI_spi1_pdata);
	if (ret)
		pr_warning("failed to register spi 1 device : %d\n", ret);

	if (!(system_rev & 0x100))
		register_mtd_user(&spi_notifier);
}

// SWITCHING FROM SLEEP
static struct davinci_pm_config da850_pm_pdata = { 
	.sleepcount = 128,
};

static struct platform_device da850_pm_device = {
	.name           = "pm-davinci",
	.dev = {
		.platform_data	= &da850_pm_pdata,
	},
	.id             = -1,
};

// EEPROM SUPPORT for RedRiver Baseboard
/*
static struct at24_platform_data bosphorusI_i2c_eeprom_info = {
	.byte_len	= SZ_256K / 8,
	.page_size	= 64,
	.flags		= AT24_FLAG_ADDR16,
	.setup		= davinci_get_mac_addr,
	.context	= (void *)0x7f00,
};
*/

// NAND FLASH
/*
 * Bosphorus-I includes a 256MB, 8-bit NAND (2K page, 128KB block) flash with ONFI compliant
 */
static struct mtd_partition bosphorusI_nandflash_partition[] = {
  #ifdef CONFIG_BOSP_NAND_GENERAL
	{
		.name		= "u-boot env",
		.offset		= 0,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	 },
	{
		.name		= "UBL",		// UBL must be in Block 1, see chip revision
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "u-boot",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 4 * SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "kernel",
		.offset		= 0x200000,
		.size		= SZ_4M,
		.mask_flags	= 0,
	},
	{
		.name		= "filesystem1",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128M,
		.mask_flags	= 0,
	},
	{
		.name		= "filesystem2",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	},
#else
       {
		.name		= "filesystem1",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128M,
		.mask_flags	= 0,
	},
        {
		.name		= "filesystem2",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	},
#endif
};
// Bosphorus-I update (/arch/arm/mach-davinci/include/mach/aemif.h)
static struct davinci_aemif_timing bosphorusI_nandflash_timing = {  
	.wsetup		= 24,
	.wstrobe	= 21,
	.whold		= 14,
	.rsetup		= 19,
	.rstrobe	= 50,
	.rhold		= 0,
	.ta		= 20,
};

static struct davinci_nand_pdata bosphorusI_nandflash_data = {
	.parts		= bosphorusI_nandflash_partition,
	.nr_parts	= ARRAY_SIZE(bosphorusI_nandflash_partition),
	.ecc_mode	= NAND_ECC_HW,  // See u-boot config file
	.ecc_bits	= 4, 		// Bosphorus-I supports 4-bit ECC mode
	.options	= NAND_USE_FLASH_BBT,
	.timing		= &bosphorusI_nandflash_timing,
};

static struct resource bosphorusI_nandflash_resource[] = {
	{
		.start	= DA8XX_AEMIF_CS3_BASE,
		.end	= DA8XX_AEMIF_CS3_BASE + SZ_512K + 2 * SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= DA8XX_AEMIF_CTL_BASE,
		.end	= DA8XX_AEMIF_CTL_BASE + SZ_32K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device bosphorusI_nandflash_device = {
	.name		= "davinci_nand",
	.id		= 1,
	.dev		= {
	.platform_data	= &bosphorusI_nandflash_data,
	},
	.num_resources	= ARRAY_SIZE(bosphorusI_nandflash_resource),
	.resource	= bosphorusI_nandflash_resource,
};

static struct platform_device *bosphorusI_devices[] __initdata = {
	&bosphorusI_nandflash_device,
};

static const short bosphorusI_nand_pins[] = { //We do not need to CS4 because of single die NAND is been using on Bosphorus-I.
	DA850_EMA_D_0, DA850_EMA_D_1, DA850_EMA_D_2, DA850_EMA_D_3,
	DA850_EMA_D_4, DA850_EMA_D_5, DA850_EMA_D_6, DA850_EMA_D_7,
	DA850_EMA_A_1, DA850_EMA_A_2, DA850_NEMA_CS_3, DA850_NEMA_CS_4,
	DA850_NEMA_WE, DA850_NEMA_OE,
	-1
};

static inline void bosphorusI_setup_nand(void)
{
	int ret = 0;
	
	ret = davinci_cfg_reg_list(bosphorusI_nand_pins);
	if (ret)
			pr_warning("BosphorusI_init: NAND setup failed: ""%d\n", ret);		
	
	platform_add_devices(bosphorusI_devices,
			ARRAY_SIZE(bosphorusI_devices));		
}

// SDMMC
#if defined(CONFIG_MMC_DAVINCI) || \
    defined(CONFIG_MMC_DAVINCI_MODULE)
#define HAS_MMC 1
#else
#define HAS_MMC 0
#endif

// RMII ETHERNET
static const short bosphorusI_rmii_pins[] = {
	DA850_RMII_TXD_0, DA850_RMII_TXD_1, DA850_RMII_TXEN,
	DA850_RMII_CRS_DV, DA850_RMII_RXD_0, DA850_RMII_RXD_1,
	DA850_RMII_RXER, DA850_RMII_MHZ_50_CLK, DA850_MDIO_CLK,
	DA850_MDIO_D,
	-1
};

static int __init bosphorusI_config_emac(void)
{
	
	void __iomem *cfg_chip3_base;
	int ret;
	u32 val;
	struct davinci_soc_info *soc_info = &davinci_soc_info;
	
	u8 rmii_en = 1 ;
		
	soc_info->emac_pdata->rmii_en = 1; // Default RMII enable 

	cfg_chip3_base = DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP3_REG);
	val = __raw_readl(cfg_chip3_base);
	
	if (rmii_en) {
		val |= BIT(8);
		ret = davinci_cfg_reg_list(bosphorusI_rmii_pins);
		pr_info("BosphorusI_init: Ethernet PHY - RMII configured\n");
	} else {
		val &= ~BIT(8);
		pr_info("BosphorusI_init: Ethernet PHY - RMII setup failed:\n");
	}
	
	__raw_writel(val, cfg_chip3_base);	

	soc_info->emac_pdata->phy_id = BOSPHORUSI_PHY_ID;

	ret = da8xx_register_emac();
	if (ret)
		pr_warning("BosphorusI_init: EMAC registration failed: %d\n", ret);

	return 0;	
}

// USB Control
/*
 * USB1 VBUS is controlled by GPIO6[12], over-current is reported on GPIO6[13].
 */
#define ON_BD_USB_DRV	GPIO_TO_PIN(6, 12) 
#define ON_BD_USB_OVC	GPIO_TO_PIN(6, 13)

static const short bosphorusI_usb11_pins[] = {
	DA850_GPIO6_12, DA850_GPIO6_13,
	-1
};

static da8xx_ocic_handler_t bosphorusI_usb_ocic_handler;

static int bosphorusI_usb_set_power(unsigned port, int on)
{
	gpio_set_value(ON_BD_USB_DRV, on);
	return 0;
}

static int bosphorusI_usb_get_power(unsigned port)
{
	return gpio_get_value(ON_BD_USB_DRV);
}

static int bosphorusI_usb_get_oci(unsigned port)
{
	return !gpio_get_value(ON_BD_USB_OVC);
}

static irqreturn_t bosphorusI_usb_ocic_irq(int, void *);

static int bosphorusI_usb_ocic_notify(da8xx_ocic_handler_t handler)
{
	int irq		= gpio_to_irq(ON_BD_USB_OVC);
	int error	= 0;

	if (handler != NULL) {
		bosphorusI_usb_ocic_handler = handler;

		error = request_irq(irq, bosphorusI_usb_ocic_irq, IRQF_DISABLED |
				    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				    "OHCI over-current indicator", NULL);
		if (error)
			printk(KERN_ERR "%s: could not request IRQ to watch "
			       "over-current indicator changes\n", __func__);
	} else
		free_irq(irq, NULL);

	return error;
}

static struct da8xx_ohci_root_hub bosphorusI_usb11_pdata = {
	.set_power	= bosphorusI_usb_set_power,
	.get_power	= bosphorusI_usb_get_power,
	.get_oci	= bosphorusI_usb_get_oci,
	.ocic_notify	= bosphorusI_usb_ocic_notify,

	/* TPS2065 switch @ 5V */
	.potpgt		= (3 + 1) / 2,	/* 3 ms max */
};

static irqreturn_t bosphorusI_usb_ocic_irq(int irq, void *dev_id)
{
	bosphorusI_usb_ocic_handler(&bosphorusI_usb11_pdata, 1);
	return IRQ_HANDLED;
}

static __init void bosphorusI_usb_init(void)
{
	u32 cfgchip2;
	int ret;

	/*
	 * Set up USB clock/mode in the CFGCHIP2 register.
	 * FYI:  CFGCHIP2 is 0x0000ef00 initially.
	 */
	cfgchip2 = __raw_readl(DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));

	/* USB2.0 PHY reference clock is 24 MHz */
	cfgchip2 &= ~CFGCHIP2_REFFREQ;
	cfgchip2 |=  CFGCHIP2_REFFREQ_24MHZ;

	/*
	 * Select internal reference clock for USB 2.0 PHY
	 * and use it as a clock source for USB 1.1 PHY
	 * (this is the default setting anyway).
	 */
	cfgchip2 &= ~CFGCHIP2_USB1PHYCLKMUX;
	cfgchip2 |=  CFGCHIP2_USB2PHYCLKMUX;

	/*
	 * We have to override VBUS/ID signals when MUSB is configured into the
	 * host-only mode -- ID pin will float if no cable is connected, so the
	 * controller won't be able to drive VBUS thinking that it's a B-device.
	 * Otherwise, we want to use the OTG mode and enable VBUS comparators.
	 */
	cfgchip2 &= ~CFGCHIP2_OTGMODE;
#ifdef	CONFIG_USB_MUSB_HOST
	cfgchip2 |=  CFGCHIP2_FORCE_HOST;
#else
	cfgchip2 |=  CFGCHIP2_SESENDEN | CFGCHIP2_VBDTCTEN;
#endif

	__raw_writel(cfgchip2, DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));

	/*
	 * TPS2065 switch @ 5V supplies 1 A (sustains 1.5 A),
	 * with the power on to power good time of 3 ms.
	 */
	ret = da8xx_register_usb20(1000, 3);
	if (ret)
		pr_warning("%s: USB 2.0 registration failed: %d\n",
			   __func__, ret);

	ret = davinci_cfg_reg_list(bosphorusI_usb11_pins);
	if (ret) {
		pr_warning("%s: USB 1.1 PinMux setup failed: %d\n",
			   __func__, ret);
		return;
	}

	ret = gpio_request(ON_BD_USB_DRV, "ON_BD_USB_DRV");
	if (ret) {
		printk(KERN_ERR "%s: failed to request GPIO for USB 1.1 port "
		       "power control: %d\n", __func__, ret);
		return;
	}
	gpio_direction_output(ON_BD_USB_DRV, 0);

	ret = gpio_request(ON_BD_USB_OVC, "ON_BD_USB_OVC");
	if (ret) {
		printk(KERN_ERR "%s: failed to request GPIO for USB 1.1 port "
		       "over-current indicator: %d\n", __func__, ret);
		return;
	}
	gpio_direction_input(ON_BD_USB_OVC);

	ret = da8xx_register_usb11(&bosphorusI_usb11_pdata);
	if (ret)
		pr_warning("%s: USB 1.1 registration failed: %d\n",
			   __func__, ret);
}

// UART 
static struct davinci_uart_config bosphorusI_uart_config __initdata = {
	.enabled_uarts = 0x7,
};

// AUDIO
/* davinci da850 evm audio machine driver */
static u8 da850_iis_serializer_direction[] = {
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	TX_MODE,
	RX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data bosphorusI_snd_data = {
	.tx_dma_offset		= 0x2000,
	.rx_dma_offset		= 0x2000,
	.op_mode		= DAVINCI_MCASP_IIS_MODE,
	.num_serializer		= ARRAY_SIZE(da850_iis_serializer_direction),
	.tdm_slots		= 2,
	.serial_dir		= da850_iis_serializer_direction,
	.asp_chan_q		= EVENTQ_1,
	.version		= MCASP_VERSION_2,
	.txnumevt		= 1,
	.rxnumevt		= 1,
};

//  MMCSD
static int bosphorusI_mmc_get_ro(int index)
{
	if (MMCSD_WP_USING)
		return gpio_get_value(DA850_MMCSD_WP_PIN);
	else
		return 0; // for microSD card on RedRiver
}

static int bosphorusI_mmc_get_cd(int index)
{
	return !gpio_get_value(DA850_MMCSD_CD_PIN);
}

static struct davinci_mmc_config da850_mmc_config = {
	.get_ro		= bosphorusI_mmc_get_ro,
	.get_cd		= bosphorusI_mmc_get_cd,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

// LCD BACKLIGHT AND POWER
static void da850_panel_power_ctrl(int val)
{
	/* lcd power */
	gpio_set_value(DA850_LCD_PWR_PIN, val);

	mdelay(200);

	/* lcd backlight */
	gpio_set_value(DA850_LCD_BL_PIN, val);
}

static int da850_lcd_hw_init(void)
{
	void __iomem *cfg_mstpri2_base;
	int status;
	u32 val;

	/*
	 * Reconfigure the LCDC priority to the highest to ensure that
	 * the throughput/latency requirements for the LCDC are met.
	 */
	cfg_mstpri2_base = DA8XX_SYSCFG0_VIRT(DA8XX_MSTPRI2_REG);

	val = __raw_readl(cfg_mstpri2_base);
	val &= 0x0fffffff;
	__raw_writel(val, cfg_mstpri2_base);

	status = gpio_request(DA850_LCD_BL_PIN, "lcd bl\n");
	if (status < 0)
		return status;

	status = gpio_request(DA850_LCD_PWR_PIN, "lcd pwr\n");
	if (status < 0) {
		gpio_free(DA850_LCD_BL_PIN);
		return status;
	}

	gpio_direction_output(DA850_LCD_BL_PIN, 0);
	gpio_direction_output(DA850_LCD_PWR_PIN, 0);

	return 0;
}

static struct i2c_gpio_platform_data da850_gpio_i2c_pdata = {
	.sda_pin	= GPIO_TO_PIN(1, 4),
	.scl_pin	= GPIO_TO_PIN(1, 5),
	.udelay		= 2,			/* 250 KHz */
};

static struct platform_device da850_gpio_i2c = {
	.name		= "i2c-gpio",
	.id			= 1,
	.dev		= {
	.platform_data	= &da850_gpio_i2c_pdata,
	},
};
/*
static struct davinci_i2c_platform_data bosphorusI_i2c0_pdata = {
	.bus_freq	= 100,	// kHz
	.bus_delay	= 0,	// us
};
*/

// TPS65023 PMIC

/* 1.2V */
struct regulator_consumer_supply tps65023_dcdc1_consumers[] = {
	{
		.supply = "cvdd",
	},
};

/* 1.8V */
struct regulator_consumer_supply tps65023_dcdc2_consumers[] = {
	{
		.supply = "usb0_vdda18",
	},
	{
		.supply = "usb1_vdda18",
	},
	{
		.supply = "ddr_dvdd18",
	},
	{
		.supply = "sata_vddr",
	},
	{
		.supply = "audio_dvdd",
	},
		{
		.supply = "DDR2SDRAM",
	},
};

/* 1.2V */
struct regulator_consumer_supply tps65023_dcdc3_consumers[] = {
	{
		.supply = "sata_vdd",
	},
	{
		.supply = "usb_cvdd",
	},
	{
		.supply = "pll0_vdda",
	},
	{
		.supply = "pll1_vdda",
	},
	{
		.supply = "usb0_vdaa2",
	},
};

/* 1.8V LDO - Bosphorus-I does not use this rail */
struct regulator_consumer_supply tps65023_ldo1_consumers[] = {
	{
		.supply = "no",
	},
};

/* 3.3V LDO - only supported by 3.6V - 5.5V supply of Bosphorus-I */
struct regulator_consumer_supply tps65023_ldo2_consumers[] = {
	{
		.supply = "board",
	},
};

struct regulator_init_data tps65023_regulator_data[] = {
	/* dcdc1 */
	{
		.constraints = {
			.min_uV = 1150000,
			.max_uV = 1350000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_consumers),
		.consumer_supplies = tps65023_dcdc1_consumers,
	},

	/* dcdc2 */
	{
		.constraints = {
			.min_uV = 1710000,
			.max_uV = 1910000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc2_consumers),
		.consumer_supplies = tps65023_dcdc2_consumers,
	},

	/* dcdc3 */
	{
		.constraints = {
			.min_uV = 1120000,
			.max_uV = 1320000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc3_consumers),
		.consumer_supplies = tps65023_dcdc3_consumers,
	},

	/* ldo1 */
	{
		.constraints = {
			.min_uV = 1710000,
			.max_uV = 1890000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_ldo1_consumers),
		.consumer_supplies = tps65023_ldo1_consumers,
	},

	/* ldo2 */
	{
		.constraints = {
			.min_uV = 3140000,
			.max_uV = 3420000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65023_ldo2_consumers),
		.consumer_supplies = tps65023_ldo2_consumers,
	},
};

// I2C DEVICES
static struct i2c_board_info __initdata bosphorusI_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.platform_data = &tps65023_regulator_data[0],
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
};

/*
static int __init bosphorusI_i2c_init(void)
{
	return i2c_register_board_info(1, bosphorusI_i2c_devices,
					ARRAY_SIZE(bosphorusI_i2c_devices));
}
*/

static const short bosphorusI_lcdc_pins[] = {
	DA850_GPIO2_8, DA850_GPIO1_12,
	-1
};

static const short bosphorusI_touchscreen_pins[] = {
	DA850_GPIO0_13, DA850_SPI1_CS1,
	-1
};

// EDMA CHANNELS
/*
 * The following EDMA channels/slots are not being used by drivers (for
 * example: Timer, GPIO, UART events etc) on da850/omap-l138 EVM, hence
 * they are being reserved for codecs on the DSP side.
 */
static const s16 da850_dma0_rsv_chans[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma0_rsv_slots[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30, 50},
	{-1, -1}
};

static const s16 da850_dma1_rsv_chans[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma1_rsv_slots[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30, 90},
	{-1, -1}
};

static struct edma_rsv_info da850_edma_cc0_rsv = {
	.rsv_chans	= da850_dma0_rsv_chans,
	.rsv_slots	= da850_dma0_rsv_slots,
};

static struct edma_rsv_info da850_edma_cc1_rsv = {
	.rsv_chans	= da850_dma1_rsv_chans,
	.rsv_slots	= da850_dma1_rsv_slots,
};

static struct edma_rsv_info *da850_edma_rsv[2] = {
	&da850_edma_cc0_rsv,
	&da850_edma_cc1_rsv,
};

// MICROPROCESSOR FREQUENCY
#ifdef CONFIG_CPU_FREQ
static __init int bosphorusI_init_cpufreq(void)
{
	switch (system_rev & 0xF) {
	case 3:
		da850_max_speed = 456000;
		break;
	case 2:
		da850_max_speed = 408000;
		break;
	case 1:
		da850_max_speed = 372000;
		break;
	}

	return da850_register_cpufreq("pll0_sysclk3");
}
#else
static __init int bosphorusI_init_cpufreq(void) { return 0; }
#endif

#if defined(CONFIG_TI_DAVINCI_EMAC) || \
	defined(CONFIG_TI_DAVINCI_EMAC_MODULE)
#define HAS_EMAC 1
#else
#define HAS_EMAC 0
#endif

#if defined(CONFIG_SND_DA850_SOC_BOSPHORUSI)
#define HAS_MCASP 1
#else
#define HAS_MCASP 0
#endif

#define BOSPHORUSI_SATA_REFCLKPN_RATE	(100 * 1000 * 1000)

#if defined(CONFIG_FB_DA8XX) && (defined (CONFIG_BOSP_LCD_4_3_SHARP)  || defined (CONFIG_BOSP_LCD_3_5_SHARP))
#define HAS_LCD	1
#else
#define HAS_LCD	0
#endif

static __init void bosphorusI_init(void)
{
	int ret;
	char mask = 0;
	// Bosphorus-I init START
	pr_info("BosphorusI_init: START...\n");
	ret = davinci_cfg_reg_list(bosphorusI_touchscreen_pins);
	if (ret)
		pr_warning("BosphorusI_init: touchscreen mux setup failed: %d\n",ret);
	
	/*
	// I2C PIN MUX
	ret = davinci_cfg_reg_list(da850_i2c0_pins);
	if (ret)
		pr_warning("BosphorusI_init: i2c0 mux setup failed: %d\n",
				ret);
	
	//I2C
	ret = da8xx_register_i2c(0, &bosphorusI_i2c0_pdata);
	if (ret)
		pr_warning("BosphorusI_init: i2c0 registration failed: %d\n",
				ret);
	
	// Board I2C init 
	ret = bosphorusI_i2c_init();
	if (ret)
		pr_warning("BosphorusI_init: PMU registration failed: %d\n",
				ret);
	*/
	
	ret = davinci_cfg_reg_list(da850_i2c0_pins);
	if (ret)
		pr_warning("BosphorusI_init: i2c0 mux setup failed: %d\n",
				ret);

	platform_device_register(&da850_gpio_i2c);
				
	// EDMA
	ret = da850_register_edma(da850_edma_rsv);
	if (ret)
		pr_warning("BosphorusI_init: edma registration failed: %d\n",
				ret);
		
	// ETHERNET
	bosphorusI_config_emac();
	
	// NAND
	bosphorusI_setup_nand();	
	
	// WATCHDOG
	ret = da8xx_register_watchdog();
	if (ret)
		pr_warning("BosphorusI_init: watchdog registration failed: %d\n",
				ret);
	/*
	// UART1
	ret = davinci_cfg_reg_list(da850_uart1_pins);
	if (ret)
		pr_warning("BosphorusI_init: UART 1 mux setup failed:"
						" %d\n", ret);
	*/
	
	// SATA 
	ret = da850_register_sata(BOSPHORUSI_SATA_REFCLKPN_RATE);
	if (ret)
		pr_warning("BosphorusI_init: sata registration failed: %d\n",
				ret);

	// MMC
	if (HAS_MMC) {
		ret = davinci_cfg_reg_list(da850_mmcsd0_pins);
		if (ret)
			pr_warning("BosphorusI_init: mmcsd0 mux setup failed:"
					" %d\n", ret);

		ret = gpio_request(DA850_MMCSD_CD_PIN, "MMC CD\n");
		if (ret)
			pr_warning("BosphorusI_init: can not open GPIO for MMC CD%d\n",
					DA850_MMCSD_CD_PIN);
		gpio_direction_input(DA850_MMCSD_CD_PIN);

		ret = gpio_request(DA850_MMCSD_WP_PIN, "MMC WP\n");
		if (ret)
			pr_warning("BosphorusI_init: can not open GPIO for MMC WP%d\n",
					DA850_MMCSD_WP_PIN);
		gpio_direction_input(DA850_MMCSD_WP_PIN);

		ret = da8xx_register_mmcsd0(&da850_mmc_config);
		if (ret)
			pr_warning("BosphorusI_init: mmcsd0 registration failed:"
					" %d\n", ret);
	}
	// 
	davinci_serial_init(&bosphorusI_uart_config);
	
	i2c_register_board_info(1, bosphorusI_i2c_devices,
					ARRAY_SIZE(bosphorusI_i2c_devices));

        
	/*
	 * shut down uart 0 and 1; they are not used on the board and
	 * accessing them causes endless "too much work in irq53" messages
	 * with arago fs
	 */
	__raw_writel(0, IO_ADDRESS(DA8XX_UART0_BASE) + 0x30);

	// SOUND
	if (HAS_MCASP) {	
	ret = davinci_cfg_reg_list(da850_mcasp_pins);
	if (ret){
		pr_warning("BosphorusI_init: mcasp mux setup failed:"
					"%d\n", ret);}
	else {
		pr_info("BosphorusI_init: Sound is now configuring\n");}	
	da8xx_register_mcasp(0, &bosphorusI_snd_data);
	}

    // LCD
	if (HAS_LCD){

	ret = davinci_cfg_reg_list(da850_lcdcntl_pins);
	if (ret)
		pr_warning("BosphorusI_init: lcdcntl mux setup failed: %d\n",
				ret);
						
	ret = davinci_cfg_reg_list(bosphorusI_lcdc_pins);
	if (ret)
		pr_warning("BosphorusI_init: Baseboard specific lcd mux setup "
				"failed: %d\n",	ret);
		
	ret = da850_lcd_hw_init();
	if (ret)
		pr_warning("BosphorusI_init: LCD initialization failed: %d\n",
				ret);
	
       #ifdef CONFIG_BOSP_LCD_4_3_SHARP	
	sharp_lk043t1dg01_pdata.panel_power_ctrl = da850_panel_power_ctrl,
	ret = da8xx_register_lcdc(&sharp_lk043t1dg01_pdata);
       #else
	sharp_lcd035q3dg01_pdata.panel_power_ctrl = da850_panel_power_ctrl,
	ret = da8xx_register_lcdc(&sharp_lcd035q3dg01_pdata);
       #endif
	if (ret)
		pr_warning("BosphorusI_init: lcdc registration failed: %d\n",
				ret);
        else 
		pr_info("BosphorusI_init: LCD setup is successful \n");
	}

       if (HAS_VGA){

        ret = davinci_cfg_reg_list(da850_lcdcntl_pins);
	if (ret)
		pr_warning("BosphorusI_init: lcdcntl mux setup failed: %d\n",
				ret);


        ret = davinci_cfg_reg_list(bosphorusI_lcdc_pins);
	if (ret)
		pr_warning("BosphorusI_init: Baseboard specific lcd mux setup "
				"failed: %d\n",	ret);
        
        ret = da850_lcd_hw_init();
	if (ret)
		pr_warning("BosphorusI_init: LCD initialization failed: %d\n",
				ret);
        ret = da8xx_register_lcdc(&vga_monitor_pdata);
	if (ret)
		pr_warning("BosphorusI_init: VGA registration failed: %d\n",
				ret);
        else 
		pr_info("BosphorusI_init: VGA setup is successful \n");
    
       }


	// RTC
	ret = da8xx_register_rtc();
	if (ret)
		pr_warning("BosphorusI_init: rtc setup failed: %d\n", ret);

	ret = bosphorusI_init_cpufreq();
	if (ret)
		pr_warning("BosphorusI_init: cpufreq registration failed: %d\n",
				ret);

	ret = da8xx_register_cpuidle();
	if (ret)
		pr_warning("BosphorusI_init: cpuidle registration failed: %d\n",
				ret);

	ret = da850_register_pm(&da850_pm_device);
	if (ret)
		pr_warning("BosphorusI_init: suspend registration failed: %d\n",
				ret);

	bosphorusI_init_spi1(bosphorusI_spi1_info, ARRAY_SIZE(bosphorusI_spi1_info));

	da850_register_ehrpwm(mask);

	ret = platform_device_register(&bosphorusI_backlight);
	if (ret)
		pr_warning("BosphorusI_init: backlight device registration"
				" failed: %d\n", ret);
				
	// USB
	bosphorusI_usb_init();
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init bosphorusI_console_init(void)
{
	return add_preferred_console("ttyS", 2, "115200");
}
console_initcall(bosphorusI_console_init);
#endif

static void __init bosphorusI_map_io(void)
{
	da850_init();
}

MACHINE_START(BOSPHORUS1, "Bosphorus-I CoM - ATLAS EMBEDDED SYSTEMS LTD")
	.boot_params	= (DA8XX_DDR_BASE + 0x100),
	.map_io			= bosphorusI_map_io,
	.init_irq		= cp_intc_init,
	.timer			= &davinci_timer,
	.init_machine	= bosphorusI_init,
MACHINE_END

/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/mx6.h>
#include <asm/arch/mx6_pins.h>
#if defined(CONFIG_SECURE_BOOT)
#include <asm/arch/mx6_secure.h>
#endif
#include <asm/arch/mx6dl_pins.h>
#include <asm/arch/iomux-v3.h>
#include <asm/errno.h>
#ifdef CONFIG_MXC_FEC
#include <miiphy.h>
#endif
#if defined(CONFIG_VIDEO_MX5)
#include <linux/list.h>
#include <linux/fb.h>
#include <linux/mxcfb.h>
#include <ipu.h>
#include <lcd.h>
#endif

#ifdef CONFIG_IMX_ECSPI
#include <imx_spi.h>
#endif

#if CONFIG_I2C_MXC
#include <i2c.h>
#endif

#ifdef CONFIG_CMD_MMC
#include <mmc.h>
#include <fsl_esdhc.h>
#endif

#ifdef CONFIG_ARCH_MMU
#include <asm/mmu.h>
#include <asm/arch/mmu.h>
#endif

#ifdef CONFIG_CMD_CLOCK
#include <asm/clock.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

static u32 system_rev;

static void set_gpio_output_val(unsigned base, unsigned mask, unsigned val)
{
	unsigned reg = readl(base + GPIO_DR);
	if (val & 1)
		reg |= mask;	/* set high */
	else
		reg &= ~mask;	/* clear low */
	writel(reg, base + GPIO_DR);

	reg = readl(base + GPIO_GDIR);
	reg |= mask;		/* configure GPIO line as output */
	writel(reg, base + GPIO_GDIR);
}

#ifdef CONFIG_VIDEO_MX5
extern unsigned char fsl_bmp_600x400[];
extern int fsl_bmp_600x400_size;
extern int g_ipu_hw_rev;

#if defined(CONFIG_BMP_8BPP)
unsigned short colormap[256];
#elif defined(CONFIG_BMP_16BPP)
unsigned short colormap[65536];
#else
unsigned short colormap[16777216];
#endif


static int di = 1;

extern int ipuv3_fb_init(struct fb_videomode *mode, int di,
			int interface_pix_fmt,
			ipu_di_clk_parent_t di_clk_parent,
			int di_clk_val);

static struct fb_videomode lvds_xga = {
	 "XGA", 60, 1024, 768, 15385, 220, 40, 21, 7, 60, 10,
	 FB_SYNC_EXT,
	 FB_VMODE_NONINTERLACED,
	 0,
};

vidinfo_t panel_info;
#endif


#ifdef CONFIG_ARCH_MMU
void board_mmu_init(void)
{
	unsigned long ttb_base = PHYS_SDRAM_1 + 0x4000;
	unsigned long i;

	/*
	* Set the TTB register
	*/
	asm volatile ("mcr  p15,0,%0,c2,c0,0" : : "r"(ttb_base) /*:*/);

	/*
	* Set the Domain Access Control Register
	*/
	i = ARM_ACCESS_DACR_DEFAULT;
	asm volatile ("mcr  p15,0,%0,c3,c0,0" : : "r"(i) /*:*/);

	/*
	* First clear all TT entries - ie Set them to Faulting
	*/
	memset((void *)ttb_base, 0, ARM_FIRST_LEVEL_PAGE_TABLE_SIZE);
	/* Actual   Virtual  Size   Attributes          Function */
	/* Base     Base     MB     cached? buffered?  access permissions */
	/* xxx00000 xxx00000 */
	X_ARM_MMU_SECTION(0x000, 0x000, 0x001,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* ROM, 1M */
	X_ARM_MMU_SECTION(0x001, 0x001, 0x008,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* 8M */
	X_ARM_MMU_SECTION(0x009, 0x009, 0x001,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* IRAM */
	X_ARM_MMU_SECTION(0x00A, 0x00A, 0x0F6,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* 246M */
#ifndef CONFIG_MX6Q_ARM2_LPDDR2POP
	/* 2 GB memory starting at 0x10000000, only map 1.875 GB */
	X_ARM_MMU_SECTION(0x100, 0x100, 0x780,
			ARM_CACHEABLE, ARM_BUFFERABLE,
			ARM_ACCESS_PERM_RW_RW);
	/* uncached alias of the same 1.875 GB memory */
	X_ARM_MMU_SECTION(0x100, 0x880, 0x780,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW);
#else
	/*
	 * Phys		Virtual		Size		Property
	 * ----------	----------	--------	----------
	 * 0x10000000	0x10000000	256M		cacheable
	 * 0x80000000	0x20000000	16M		uncacheable
	 * 0x81000000	0x21000000	240M		cacheable
	 */
	/* Reserve the first 256MB of bank 1 as cacheable memory */
	X_ARM_MMU_SECTION(0x100, 0x100, 0x100,
			ARM_CACHEABLE, ARM_BUFFERABLE,
			ARM_ACCESS_PERM_RW_RW);

	/* Reserve the first 16MB of bank 2 uncachable memory*/
	X_ARM_MMU_SECTION(0x800, 0x200, 0x010,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW);

	/* Reserve the remaining 240MB of bank 2 as cacheable memory */
	X_ARM_MMU_SECTION(0x810, 0x210, 0x0F0,
			ARM_CACHEABLE, ARM_BUFFERABLE,
			ARM_ACCESS_PERM_RW_RW);
#endif

	/* Enable MMU */
	MMU_ON();
}
#endif

#ifdef CONFIG_DWC_AHSATA

#define ANATOP_PLL_LOCK                 0x80000000
#define ANATOP_PLL_ENABLE_MASK          0x00002000
#define ANATOP_PLL_BYPASS_MASK          0x00010000
#define ANATOP_PLL_LOCK                 0x80000000
#define ANATOP_PLL_PWDN_MASK            0x00001000
#define ANATOP_PLL_HOLD_RING_OFF_MASK   0x00000800
#define ANATOP_SATA_CLK_ENABLE_MASK     0x00100000

int setup_sata(void)
{
	u32 reg = 0;
	s32 timeout = 100000;

	/* Enable sata clock */
	reg = readl(CCM_BASE_ADDR + 0x7c); /* CCGR5 */
	reg |= 0x30;
	writel(reg, CCM_BASE_ADDR + 0x7c);

	/* Enable PLLs */
	reg = readl(ANATOP_BASE_ADDR + 0xe0); /* ENET PLL */
	reg &= ~ANATOP_PLL_PWDN_MASK;
	writel(reg, ANATOP_BASE_ADDR + 0xe0);
	reg |= ANATOP_PLL_ENABLE_MASK;
	while (timeout--) {
		if (readl(ANATOP_BASE_ADDR + 0xe0) & ANATOP_PLL_LOCK)
			break;
	}
	if (timeout <= 0)
		return -1;
	reg &= ~ANATOP_PLL_BYPASS_MASK;
	writel(reg, ANATOP_BASE_ADDR + 0xe0);
	reg |= ANATOP_SATA_CLK_ENABLE_MASK;
	writel(reg, ANATOP_BASE_ADDR + 0xe0);

	/* Enable sata phy */
	reg = readl(IOMUXC_BASE_ADDR + 0x34); /* GPR13 */

	reg &= ~0x07ffffff;
	/*
	 * rx_eq_val_0 = 5 [26:24]
	 * los_lvl = 0x12 [23:19]
	 * rx_dpll_mode_0 = 0x3 [18:16]
	 * mpll_ss_en = 0x0 [14]
	 * tx_atten_0 = 0x4 [13:11]
	 * tx_boost_0 = 0x0 [10:7]
	 * tx_lvl = 0x11 [6:2]
	 * mpll_ck_off_b = 0x1 [1]
	 * tx_edgerate_0 = 0x0 [0]
	 * */
	reg |= 0x59124c6;
	writel(reg, IOMUXC_BASE_ADDR + 0x34);

	return 0;
}
#endif

int dram_init(void)
{
	/*
	 * Switch PL301_FAST2 to DDR Dual-channel mapping
	 * however this block the boot up, temperory redraw
	 */
	/*
	 * u32 reg = 1;
	 * writel(reg, GPV0_BASE_ADDR);
	 */

	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;
#ifdef CONFIG_MX6Q_ARM2_LPDDR2POP
	gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
	gd->bd->bi_dram[1].size = PHYS_SDRAM_2_SIZE;
#endif
	return 0;
}

static void setup_uart(void)
{
	/* UART3 RXD */
	mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_D25__UART3_RXD); //CON_UART3_RX
	/* UART3 TXD */
	mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_D24__UART3_TXD); //CON_UART3_TX


	/* UART2 TXD */
	mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_7__UART2_TXD); //CON_UART2_TX
	/* UART2 RXD */
	mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_8__UART2_RXD); //CON_UART2_RX
}

#ifdef CONFIG_I2C_MXC
static void setup_i2c(unsigned int module_base)
{
	unsigned int reg;

	//printf("Setting up I2C: %x\n", module_base);

	switch (module_base) 
	{
	case I2C2_BASE_ADDR:
		/* i2c2 SDA */
		mxc_iomux_v3_setup_pad(MX6Q_PAD_KEY_ROW3__I2C2_SDA);
		/* i2c2 SCL */
		mxc_iomux_v3_setup_pad(MX6Q_PAD_KEY_COL3__I2C2_SCL);

		/* Enable i2c clock */
		reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR2);
		reg |= 0x300;
		writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR2);

		break;
	case I2C3_BASE_ADDR:
	
		/* GPIO_5 for I2C3_SCL */
		mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_5__I2C3_SCL);

		/* GPIO_16 for I2C3_SDA */
		mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_16__I2C3_SDA);

		/* Enable i2c clock */
		reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR2);
		reg |= 0xC00;
		writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR2);

		break;
	default:
		printf("Invalid I2C base: 0x%x\n", module_base);
		break;
	}
}

#endif

#ifdef CONFIG_VIDEO_MX5

void setup_lvds_poweron(void)
{
	uchar value;
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	i2c_read(0x1f, 3, 1, &value, 1);
	value &= ~0x2;
	i2c_write(0x1f, 3, 1, &value, 1);

	i2c_read(0x1f, 1, 1, &value, 1);
	value |= 0x2;
	i2c_write(0x1f, 1, 1, &value, 1);
}
#endif

#ifdef CONFIG_IMX_ECSPI
s32 spi_get_cfg(struct imx_spi_dev_t *dev)
{
	if (dev->slave.bus == 0 && dev->slave.cs == 1)
	{
		// SPI NOR Flash on ECSPI1, SS1
		dev->base = ECSPI1_BASE_ADDR;
		dev->freq = 25000000;
		dev->ss_pol = IMX_SPI_ACTIVE_LOW;
		dev->ss = 1;
		dev->fifo_sz = 64 * 4;
		dev->us_delay = 0;
	}
	else if (dev->slave.bus == 4 && dev->slave.cs == 1)
	{
		// Host Board peripheral on SPI_CSA (ECSPI5, SS1)
		dev->base = ECSPI5_BASE_ADDR;
		dev->freq = 187500;
		dev->ss_pol = IMX_SPI_ACTIVE_LOW;
		dev->ss = 1;
		dev->fifo_sz = 64 * 4;
		dev->us_delay = 0;
	}
	else if (dev->slave.bus == 4 && dev->slave.cs == 3)
	{
		// Host Board peripheral on SPI_CSB (ECSPI5, SS3)
		dev->base = ECSPI5_BASE_ADDR;
		dev->freq = 25000000;
		dev->ss_pol = IMX_SPI_ACTIVE_LOW;
		dev->sclkpol = 0;
		dev->sclkpha = 1;
		dev->ss = 3;
		dev->fifo_sz = 64 * 4;
		dev->us_delay = 0;
	}
	else
	{
		printf("No device configured for ECSPI%d, SS%d (see %s, %s)\n", 
			   dev->slave.bus+1, dev->slave.cs, __FILE__, __FUNCTION__);
	}

	return 0;
}

void spi_io_init(struct imx_spi_dev_t *dev)
{
	switch (dev->base) {
	case ECSPI1_BASE_ADDR:
		/* SCLK */
		mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_D16__ECSPI1_SCLK);

		/* MISO */
		mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_D17__ECSPI1_MISO);

		/* MOSI */
		mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_D18__ECSPI1_MOSI);

		if (dev->ss == 1)
		{
			mxc_iomux_v3_setup_pad(MX6Q_PAD_EIM_D19__ECSPI1_SS1);
			return;
		}
		break;

	case ECSPI5_BASE_ADDR:
		/* SCLK */
		mxc_iomux_v3_setup_pad(MX6Q_PAD_SD2_CLK__ECSPI5_SCLK);

		/* MISO */
		mxc_iomux_v3_setup_pad(MX6Q_PAD_SD2_DAT0__ECSPI5_MISO);

		/* MOSI */
		mxc_iomux_v3_setup_pad(MX6Q_PAD_SD2_CMD__ECSPI5_MOSI);

		switch (dev->ss)
		{
			case 1 : /* SS1 = SPI_CSA*/
				mxc_iomux_v3_setup_pad(MX6Q_PAD_SD2_DAT2__ECSPI5_SS1);
				return;

			case 3: /* SS3 = SPI_CSB */
				mxc_iomux_v3_setup_pad(MX6Q_PAD_SD2_DAT3__ECSPI5_SS3);
				return;
		}
		break;
	}

	// If we get here, the I/O has not been configured 
	printf("No I/O configured for ECSPI%d, SS%d (see %s, %s)\n", 
			dev->slave.bus+1, dev->slave.cs, __FILE__, __FUNCTION__);
}
#endif

#ifdef CONFIG_NAND_GPMI
iomux_v3_cfg_t nfc_pads[] = {
	MX6Q_PAD_NANDF_CLE__RAWNAND_CLE,
	MX6Q_PAD_NANDF_ALE__RAWNAND_ALE,
	MX6Q_PAD_NANDF_WP_B__RAWNAND_RESETN,
	MX6Q_PAD_NANDF_RB0__RAWNAND_READY0,
	MX6Q_PAD_NANDF_CS0__RAWNAND_CE0N,
	MX6Q_PAD_NANDF_CS1__RAWNAND_CE1N,
	MX6Q_PAD_SD4_CMD__RAWNAND_RDN,
	MX6Q_PAD_SD4_CLK__RAWNAND_WRN,
	MX6Q_PAD_NANDF_D0__RAWNAND_D0,
	MX6Q_PAD_NANDF_D1__RAWNAND_D1,
	MX6Q_PAD_NANDF_D2__RAWNAND_D2,
	MX6Q_PAD_NANDF_D3__RAWNAND_D3,
	MX6Q_PAD_NANDF_D4__RAWNAND_D4,
	MX6Q_PAD_NANDF_D5__RAWNAND_D5,
	MX6Q_PAD_NANDF_D6__RAWNAND_D6,
	MX6Q_PAD_NANDF_D7__RAWNAND_D7
};

void setup_gpmi_nand(void)
{
	unsigned int reg;

	/* config gpmi nand iomux */
	mxc_iomux_v3_setup_multiple_pads(nfc_pads,
			ARRAY_SIZE(nfc_pads));


	/* config gpmi and bch clock to 11Mhz*/
	reg = readl(CCM_BASE_ADDR + CLKCTL_CS2CDR);
	reg &= 0xF800FFFF;
	reg |= 0x01E40000;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CS2CDR);

	/* enable gpmi and bch clock gating */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR4);
	reg |= 0xFF003000;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR4);

	/* enable apbh clock gating */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR0);
	reg |= 0x0030;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR0);

}

#endif

u32 get_board_rev(void)
{

	system_rev = 0x1;

	return system_rev;
}


#ifdef CONFIG_NET_MULTI
int board_eth_init(bd_t *bis)
{
	int rc = -ENODEV;

	return rc;
}
#endif

#ifdef CONFIG_CMD_MMC

/* On this board, only SD3 can support 1.8V signalling
 * that is required for UHS-I mode of operation.
 * Last element in struct is used to indicate 1.8V support.
 */
struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR, 1, 1, 1, 0},
	{USDHC3_BASE_ADDR, 1, 1, 1, 0},
};

iomux_v3_cfg_t usdhc1_pads[] = {
	MX6Q_PAD_SD1_CLK__USDHC1_CLK,
	MX6Q_PAD_SD1_CMD__USDHC1_CMD,
	MX6Q_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6Q_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6Q_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6Q_PAD_SD1_DAT3__USDHC1_DAT3,
};

iomux_v3_cfg_t usdhc3_pads[] = {
	MX6Q_PAD_SD3_CLK__USDHC3_CLK,
	MX6Q_PAD_SD3_CMD__USDHC3_CMD,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3,
	MX6Q_PAD_SD3_DAT4__USDHC3_DAT4,
	MX6Q_PAD_SD3_DAT5__USDHC3_DAT5,
	MX6Q_PAD_SD3_DAT6__USDHC3_DAT6,
	MX6Q_PAD_SD3_DAT7__USDHC3_DAT7,
};


int usdhc_gpio_init(bd_t *bis)
{
	s32 status = 0;

	mxc_iomux_v3_setup_multiple_pads(usdhc1_pads,
				ARRAY_SIZE(usdhc1_pads));

	status |= fsl_esdhc_initialize(bis, &usdhc_cfg[0]);

	mxc_iomux_v3_setup_multiple_pads(usdhc3_pads,
				ARRAY_SIZE(usdhc3_pads));
			
	status |= fsl_esdhc_initialize(bis, &usdhc_cfg[1]);

	return status;
}

int board_mmc_init(bd_t *bis)
{
	if (!usdhc_gpio_init(bis))
		return 0;
	else
		return -1;
}

/* For DDR mode operation, provide target delay parameter for each SD port.
 * Use cfg->esdhc_base to distinguish the SD port #. The delay for each port
 * is dependent on signal layout for that particular port.  If the following
 * CONFIG is not defined, then the default target delay value will be used.
 */
#ifdef CONFIG_GET_DDR_TARGET_DELAY
u32 get_ddr_delay(struct fsl_esdhc_cfg *cfg)
{
	/* No delay required on ARM2 board SD ports */
	return 0;
}
#endif

#endif

#ifdef CONFIG_LCD
void lcd_enable(void)
{
	char *s;
	int ret;
	unsigned int reg;

	s = getenv("lvds_num");
	di = simple_strtol(s, NULL, 10);

	/*
	* hw_rev 2: IPUV3DEX
	* hw_rev 3: IPUV3M
	* hw_rev 4: IPUV3H
	*/
	g_ipu_hw_rev = IPUV3_HW_REV_IPUV3H;

	/* set GPIO_9 to high so that backlight control could be high */
#if defined CONFIG_MX6Q
	mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_9__GPIO_1_9);
#elif defined CONFIG_MX6DL
	mxc_iomux_v3_setup_pad(MX6DL_PAD_GPIO_9__GPIO_1_9);
#endif
	reg = readl(GPIO1_BASE_ADDR + GPIO_GDIR);
	reg |= (1 << 9);
	writel(reg, GPIO1_BASE_ADDR + GPIO_GDIR);

	reg = readl(GPIO1_BASE_ADDR + GPIO_DR);
	reg |= (1 << 9);
	writel(reg, GPIO1_BASE_ADDR + GPIO_DR);

	/* Enable IPU clock */
	if (di == 1) {
		reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR3);
		reg |= 0xC033;
		writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR3);
	} else {
		reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR3);
		reg |= 0x300F;
		writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR3);
	}

	ret = ipuv3_fb_init(&lvds_xga, di, IPU_PIX_FMT_RGB666,
			DI_PCLK_LDB, 65000000);
	if (ret)
		puts("LCD cannot be configured\n");

	reg = readl(ANATOP_BASE_ADDR + 0xF0);
	reg &= ~0x00003F00;
	reg |= 0x00001300;
	writel(reg, ANATOP_BASE_ADDR + 0xF4);

	reg = readl(CCM_BASE_ADDR + CLKCTL_CS2CDR);
	reg &= ~0x00007E00;
	reg |= 0x00003600;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CS2CDR);

	reg = readl(CCM_BASE_ADDR + CLKCTL_CSCMR2);
	reg |= 0x00000C00;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CSCMR2);

	reg = 0x0002A953;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CHSCCDR);

	if (di == 1)
		writel(0x40C, IOMUXC_BASE_ADDR + 0x8);
	else
		writel(0x201, IOMUXC_BASE_ADDR + 0x8);
}
#endif

#ifdef CONFIG_VIDEO_MX5
void panel_info_init(void)
{
	panel_info.vl_bpix = LCD_BPP;
	panel_info.vl_col = lvds_xga.xres;
	panel_info.vl_row = lvds_xga.yres;
	panel_info.cmap = colormap;
}
#endif

#ifdef CONFIG_SPLASH_SCREEN
void setup_splash_image(void)
{
	char *s;
	ulong addr;

	s = getenv("splashimage");

	if (s != NULL) {
		addr = simple_strtoul(s, NULL, 16);

#if defined(CONFIG_ARCH_MMU)
		addr = ioremap_nocache(iomem_to_phys(addr),
				fsl_bmp_600x400_size);
#endif
		memcpy((char *)addr, (char *)fsl_bmp_600x400,
				fsl_bmp_600x400_size);
	}
}
#endif

int board_init(void)
{
	mxc_iomux_v3_init((void *)IOMUXC_BASE_ADDR);

	/* board id for linux */
	gd->bd->bi_arch_number = MACH_TYPE_BCTRM3S;

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	//Enable local Audio for now
	mxc_iomux_v3_setup_pad(MX6Q_PAD_SD4_DAT6__GPIO_2_14);
	set_gpio_output_val(GPIO2_BASE_ADDR, (1 << 14), 0);

	setup_uart();

#ifdef CONFIG_DWC_AHSATA
	setup_sata();
#endif


#ifdef CONFIG_I2C_MXC
	setup_i2c(CONFIG_SYS_I2C_PORT);
#endif

#ifdef CONFIG_VIDEO_MX5
	/* Enable lvds power */
	setup_lvds_poweron();

	panel_info_init();

	gd->fb_base = CONFIG_FB_BASE;
#ifdef CONFIG_ARCH_MMU
	gd->fb_base = ioremap_nocache(iomem_to_phys(gd->fb_base), 0);
#endif
#endif

#ifdef CONFIG_NAND_GPMI
	setup_gpmi_nand();
#endif
	return 0;
}

int board_late_init(void)
{
	return 0;
}

#ifdef CONFIG_SECURE_BOOT
/* -------- start of HAB API updates ------------*/
#define hab_rvt_report_event ((hab_rvt_report_event_t *)HAB_RVT_REPORT_EVENT)
#define hab_rvt_report_status ((hab_rvt_report_status_t *)HAB_RVT_REPORT_STATUS)
#define hab_rvt_authenticate_image \
	((hab_rvt_authenticate_image_t *)HAB_RVT_AUTHENTICATE_IMAGE)
#define hab_rvt_entry ((hab_rvt_entry_t *) HAB_RVT_ENTRY)
#define hab_rvt_exit ((hab_rvt_exit_t *) HAB_RVT_EXIT)
#define hab_rvt_clock_init HAB_RVT_CLOCK_INIT

/*
 * +------------+  0x0 (0x10800000)    -
 * |   Header   |                       |
 * +------------+  0x40                 |
 * |            |                       |
 * |            |                       |
 * |            |                       |
 * |            |                       |
 * | Image Data |                       |
 * .            |                       |
 * .            |                        > Stuff to be authenticated ------+
 * .            |                       |                                  |
 * |            |                       |                                  |
 * |            |                       |                                  |
 * +------------+                       |                                  |
 * |            |                       |                                  |
 * | Fill Data  |                       |                                  |
 * |            |                       |                                  |
 * +------------+ 0x003F_DFE0           |                                  |
 * |    IVT     |                       |                                  |
 * +------------+ 0x003F_E000          -                                   |
 * |            |                                                          |
 * |  CSF DATA  | <--------------------------------------------------------+
 * |            |
 * +------------+
 * |            |
 * | Fill Data  |
 * |            |
 * +------------+ 0x0040_0000
 */

#define DDR_UIMAGE_START 0x10800000
#define DDR_UIMAGE_LENGTH 0x400000
#define DDR_IVT_OFFSET 0x3FDFE0
#define DDR_UIMAGE_IVT_START (DDR_UIMAGE_START + DDR_IVT_OFFSET)

#define OCOTP_CFG5_OFFSET 0x460

int check_hab_enable(void)
{
	u32 reg = 0;
	int result = 0;

	reg = readl(IMX_OTP_BASE + OCOTP_CFG5_OFFSET);
	if ((reg & 0x2) == 0x2)
		result = 1;

	return result;
}

void display_event(uint8_t *event_data, size_t bytes)
{
	uint32_t i;
	if ((event_data) && (bytes > 0)) {
		for (i = 0; i < bytes; i++) {
			if (i == 0)
				printf("\t0x%02x", event_data[i]);
			else if ((i % 8) == 0)
				printf("\n\t0x%02x", event_data[i]);
			else
				printf(" 0x%02x", event_data[i]);
		}
	}
}

int get_hab_status(void)
{
	uint32_t index = 0; /* Loop index */
	uint8_t event_data[128]; /* Event data buffer */
	size_t bytes = sizeof(event_data); /* Event size in bytes */
	hab_config_t config = 0;
	hab_state_t state = 0;

	/* Check HAB status */
	if (hab_rvt_report_status(&config, &state) != HAB_SUCCESS) {
		printf("\nHAB Configuration: 0x%02x, HAB State: 0x%02x\n",
			config, state);

		/* Display HAB Error events */
		while (hab_rvt_report_event(HAB_FAILURE, index, event_data,
				&bytes) == HAB_SUCCESS) {
			printf("\n");
			printf("--------- HAB Event %d -----------------\n",
					index + 1);
			printf("event data:\n");
			display_event(event_data, bytes);
			printf("\n");
			bytes = sizeof(event_data);
			index++;
		}
	}
	/* Display message if no HAB events are found */
	else {
		printf("\nHAB Configuration: 0x%02x, HAB State: 0x%02x\n",
			config, state);
		printf("No HAB Events Found!\n\n");
	}
	return 0;
}

void hab_caam_clock_enable(void)
{
	u32 reg = 0;

	reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR0); /* CCGR0 */
	reg |= 0x3F00; /*CG4 ~ CG6, enable CAAM clocks*/
	writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR0);
}


void hab_caam_clock_disable(void)
{
	u32 reg = 0;

	reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR0); /* CCGR0 */
	reg &= ~0x3F00; /*CG4 ~ CG6, disable CAAM clocks*/
	writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR0);
}

uint32_t authenticate_image(ulong start)
{
	uint32_t load_addr = 0;
	size_t bytes;
	ptrdiff_t ivt_offset = DDR_IVT_OFFSET;
	int result = 0;

	if (check_hab_enable() == 1) {
		printf("\nAuthenticate uImage from DDR location 0x%lx...\n",
			start);

		hab_caam_clock_enable();

		if (hab_rvt_entry() == HAB_SUCCESS) {
			start = DDR_UIMAGE_START;
			bytes = DDR_UIMAGE_LENGTH;
			load_addr = (uint32_t)hab_rvt_authenticate_image(
					HAB_CID_UBOOT,
					ivt_offset, (void **)&start,
					(size_t *)&bytes, NULL);
			if (hab_rvt_exit() != HAB_SUCCESS) {
				printf("hab exit function fail\n");
				load_addr = 0;
			}
		} else
			printf("hab entry function fail\n");

		hab_caam_clock_disable();

		get_hab_status();
	}

	if ((!check_hab_enable()) || (load_addr != 0))
		result = 1;

	return result;
}
/* ----------- end of HAB API updates ------------*/
#endif

static int phy_write(char *devname, unsigned char addr, unsigned char reg,
		     unsigned short value)
{
	int ret = miiphy_write(devname, addr, reg, value);
	if (ret)
		printf("Error writing to %s PHY addr=%02x reg=%02x\n", devname,
		       addr, reg);

	return ret;
}

void phy_write_mmd(char *devname, unsigned char addr, unsigned char reg, short unsigned int value)
{
	phy_write(devname, addr, 0x0d, 0x0002);     // set ID = 0x02
	phy_write(devname, addr, 0x0e, reg);        // set register
	phy_write(devname, addr, 0x0d, 0x4002);     // write without post increment
	phy_write(devname, addr, 0x0e, value);      // write data
}

int mx6_rgmii_rework(char *devname, int phy_addr)
{

#if 0
	/* To adv
	advertise only 10 Mbs */
	phy_write(devname, phy_addr, 0x4, 0x61);
	phy_write(devname, phy_addr, 0x9, 0x1c00);

#else
	/* enable master mode, force phy to 100Mbps */
	phy_write(devname, phy_addr, 0x9, 0x1F00);
#endif

	// Use defaults for RX, max delay for TX (i.e. delay TX_CLK as much as possible, TXD as little as possible)
	phy_write_mmd(devname, phy_addr, 0x4, 0x0070);		// RX_CTL[7:4], TX_CTL[3:0]
	phy_write_mmd(devname, phy_addr, 0x5, 0x7777);		// RXDn
	phy_write_mmd(devname, phy_addr, 0x6, 0x0000);		// TXDn
	phy_write_mmd(devname, phy_addr, 0x8, 0x03EF);		// TX_CLK[9:5], RX_CLK[4:0]
 
	return 0;
}

iomux_v3_cfg_t enet_pads[] = {
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	/* pin 35 - 1 (PHY_AD2) on reset */
	MX6Q_PAD_RGMII_RXC__GPIO_6_30,
	/* pin 32 - 1 - (MODE0) all */
	MX6Q_PAD_RGMII_RD0__GPIO_6_25,
	/* pin 31 - 1 - (MODE1) all */
	MX6Q_PAD_RGMII_RD1__GPIO_6_27,
	/* pin 28 - 1 - (MODE2) all */
	MX6Q_PAD_RGMII_RD2__GPIO_6_28,
	/* pin 27 - 1 - (MODE3) all */
	MX6Q_PAD_RGMII_RD3__GPIO_6_29,
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	MX6Q_PAD_RGMII_RX_CTL__GPIO_6_24,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
};

iomux_v3_cfg_t enet_pads_final[] = {
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
};

void enet_board_init(void)
{
	iomux_v3_cfg_t enet_reset =
	    (MX6Q_PAD_SD4_DAT0__GPIO_2_8 &
	     ~MUX_PAD_CTRL_MASK) | MUX_PAD_CTRL(0x48);

	/* phy reset: gpio2-8 */
	set_gpio_output_val(GPIO2_BASE_ADDR, (1 << 8), 1);
	

	set_gpio_output_val(GPIO6_BASE_ADDR, (1 << 30), (CONFIG_FEC0_PHY_ADDR >> 2));
	set_gpio_output_val(GPIO6_BASE_ADDR, (1 << 25), 1);
	set_gpio_output_val(GPIO6_BASE_ADDR, (1 << 27), 1);
	set_gpio_output_val(GPIO6_BASE_ADDR, (1 << 28), 1);
	set_gpio_output_val(GPIO6_BASE_ADDR, (1 << 29), 1);

	mxc_iomux_v3_setup_multiple_pads(enet_pads,
			ARRAY_SIZE(enet_pads));

	mxc_iomux_v3_setup_pad(enet_reset);
	

	set_gpio_output_val(GPIO6_BASE_ADDR, (1 << 24), 1);

	udelay(500);
	set_gpio_output_val(GPIO2_BASE_ADDR, (1 << 8), 0);

	mxc_iomux_v3_setup_multiple_pads(enet_pads_final,
					 ARRAY_SIZE(enet_pads_final));

}

static void report_boot_mode(void)
{
	int boot_mode;
	char *boot_mode_names[] = { "Fuses", "USB", "Unknown", "Unknown" };
	
	boot_mode = ((__REG(SRC_BASE_ADDR + 0x1c)) >> 24) & 0x03;
	printf("Boot mode: %s (%d)\n", boot_mode_names[boot_mode], boot_mode);
}

int checkboard(void)
{
	printf("Board: BCTRM3S :[ ");

	switch (__REG(SRC_BASE_ADDR + 0x8)) {
	case 0x0001:
		printf("POR");
		break;
	case 0x0009:
		printf("RST");
		break;
	case 0x0010:
	case 0x0011:
		printf("WDOG");
		break;
	default:
		printf("unknown");
	}
	printf(" ]\n");
	report_boot_mode();

#ifdef CONFIG_SECURE_BOOT
	if (check_hab_enable() == 1)
		get_hab_status();
#endif

	return 0;
}

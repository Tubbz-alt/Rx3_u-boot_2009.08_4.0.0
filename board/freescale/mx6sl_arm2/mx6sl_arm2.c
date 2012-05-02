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
#include <asm/arch/mx6sl_pins.h>
#include <asm/arch/iomux-v3.h>
#include <asm/errno.h>
#ifdef CONFIG_MXC_FEC
#include <miiphy.h>
#endif

#if defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#endif

#ifdef CONFIG_IMX_ECSPI
#include <imx_spi.h>
#endif

#ifdef CONFIG_CMD_MMC
#include <mmc.h>
#include <fsl_esdhc.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

static u32 system_rev;
static enum boot_device boot_dev;

static inline void setup_boot_device(void)
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	uint bt_mem_ctl = (soc_sbmr & 0x000000FF) >> 4 ;
	uint bt_mem_type = (soc_sbmr & 0x00000008) >> 3;

	switch (bt_mem_ctl) {
	case 0x0:
		if (bt_mem_type)
			boot_dev = ONE_NAND_BOOT;
		else
			boot_dev = WEIM_NOR_BOOT;
		break;
	case 0x2:
			boot_dev = SATA_BOOT;
		break;
	case 0x3:
		if (bt_mem_type)
			boot_dev = I2C_BOOT;
		else
			boot_dev = SPI_NOR_BOOT;
		break;
	case 0x4:
	case 0x5:
		boot_dev = SD_BOOT;
		break;
	case 0x6:
	case 0x7:
		boot_dev = MMC_BOOT;
		break;
	case 0x8 ... 0xf:
		boot_dev = NAND_BOOT;
		break;
	default:
		boot_dev = UNKNOWN_BOOT;
		break;
	}
}

enum boot_device get_boot_device(void)
{
	return boot_dev;
}

u32 get_board_rev(void)
{

	system_rev = 0x60000;

	return system_rev;
}

int dram_init(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;

	return 0;
}

static void setup_uart(void)
{
	/* UART1 TXD */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_UART1_TXD__UART1_TXD);

	/* UART1 RXD */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_UART1_RXD__UART1_RXD);
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
struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC1_BASE_ADDR, 1, 1, 1, 0},
	{USDHC2_BASE_ADDR, 1, 1, 1, 0},
	{USDHC3_BASE_ADDR, 1, 1, 1, 1},
};

#ifdef CONFIG_DYNAMIC_MMC_DEVNO
int get_mmc_env_devno(void)
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);

	if (SD_BOOT == boot_dev || MMC_BOOT == boot_dev) {
		/* BOOT_CFG2[3] and BOOT_CFG2[4] */
		return (soc_sbmr & 0x00001800) >> 11;
	} else
		return -1;

}
#endif

iomux_v3_cfg_t usdhc1_pads[] = {
	/* 8 bit SD */
	MX6SL_PAD_SD1_CLK__USDHC1_CLK,
	MX6SL_PAD_SD1_CMD__USDHC1_CMD,
	MX6SL_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6SL_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6SL_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6SL_PAD_SD1_DAT3__USDHC1_DAT3,
	MX6SL_PAD_SD1_DAT4__USDHC1_DAT4,
	MX6SL_PAD_SD1_DAT5__USDHC1_DAT5,
	MX6SL_PAD_SD1_DAT6__USDHC1_DAT6,
	MX6SL_PAD_SD1_DAT7__USDHC1_DAT7,
};

iomux_v3_cfg_t usdhc2_pads[] = {
	/* boot SD */
	MX6SL_PAD_SD2_CLK__USDHC2_CLK,
	MX6SL_PAD_SD2_CMD__USDHC2_CMD,
	MX6SL_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6SL_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6SL_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6SL_PAD_SD2_DAT3__USDHC2_DAT3,
	MX6SL_PAD_SD2_DAT4__USDHC2_DAT4,
	MX6SL_PAD_SD2_DAT5__USDHC2_DAT5,
	MX6SL_PAD_SD2_DAT6__USDHC2_DAT6,
	MX6SL_PAD_SD2_DAT7__USDHC2_DAT7,
};

iomux_v3_cfg_t usdhc3_pads[] = {
	MX6SL_PAD_SD3_CLK__USDHC3_CLK,
	MX6SL_PAD_SD3_CMD__USDHC3_CMD,
	MX6SL_PAD_SD3_DAT0__USDHC3_DAT0,
	MX6SL_PAD_SD3_DAT1__USDHC3_DAT1,
	MX6SL_PAD_SD3_DAT2__USDHC3_DAT2,
	MX6SL_PAD_SD3_DAT3__USDHC3_DAT3,
};

int usdhc_gpio_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM;
		++index) {
		switch (index) {
		case 0:
			mxc_iomux_v3_setup_multiple_pads(usdhc1_pads,
						ARRAY_SIZE(usdhc1_pads));
			break;
		case 1:
			mxc_iomux_v3_setup_multiple_pads(usdhc2_pads,
						ARRAY_SIZE(usdhc2_pads));
			break;
		case 2:
			mxc_iomux_v3_setup_multiple_pads(usdhc3_pads,
						ARRAY_SIZE(usdhc3_pads));
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) then supported by the board (%d)\n",
				index+1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}
		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
	}

	return status;
}

int board_mmc_init(bd_t *bis)
{
	if (!usdhc_gpio_init(bis))
		return 0;
	else
		return -1;
}

#ifdef CONFIG_MXC_EPDC
#ifdef CONFIG_SPLASH_SCREEN
int setup_splash_img()
{
#ifdef CONFIG_SPLASH_IS_IN_MMC
	int mmc_dev = get_mmc_env_devno();
	ulong offset = CONFIG_SPLASH_IMG_OFFSET;
	ulong size = CONFIG_SPLASH_IMG_SIZE;
	ulong addr = 0;
	char *s = NULL;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	s = getenv("splashimage");

	if (NULL == s) {
		puts("env splashimage not found!\n");
		return -1;
	}
	addr = simple_strtoul(s, NULL, 16);

	if (!mmc) {
		printf("MMC Device %d not found\n",
			mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return  -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
					blk_cnt, (u_char *)addr);
	flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);

	return (n == blk_cnt) ? 0 : -1;
#endif
}
#endif

vidinfo_t panel_info = {
	.vl_refresh = 85,
	.vl_col = 800,
	.vl_row = 600,
	.vl_pixclock = 26666667,
	.vl_left_margin = 8,
	.vl_right_margin = 100,
	.vl_upper_margin = 4,
	.vl_lower_margin = 8,
	.vl_hsync = 4,
	.vl_vsync = 1,
	.vl_sync = 0,
	.vl_mode = 0,
	.vl_flag = 0,
	.vl_bpix = 3,
	cmap:0,
};

struct epdc_timing_params panel_timings = {
	.vscan_holdoff = 4,
	.sdoed_width = 10,
	.sdoed_delay = 20,
	.sdoez_width = 10,
	.sdoez_delay = 20,
	.gdclk_hp_offs = 419,
	.gdsp_offs = 20,
	.gdoe_offs = 0,
	.gdclk_offs = 5,
	.num_ce = 1,
};

static void setup_epdc_power()
{
	unsigned int reg;

	/* Setup epdc voltage */

	/* EPDC_PWRSTAT - GPIO2[13] for PWR_GOOD status */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRSTAT__GPIO_2_13);

	/* EPDC_VCOM0 - GPIO2[3] for VCOM control */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_VCOM0__GPIO_2_3);

	/* Set as output */
	reg = readl(GPIO2_BASE_ADDR + GPIO_GDIR);
	reg |= (1 << 3);
	writel(reg, GPIO2_BASE_ADDR + GPIO_GDIR);

	/* EPDC_PWRWAKEUP - GPIO2[14] for EPD PMIC WAKEUP */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRWAKEUP__GPIO_2_14);
	/* Set as output */
	reg = readl(GPIO2_BASE_ADDR + GPIO_GDIR);
	reg |= (1 << 14);
	writel(reg, GPIO2_BASE_ADDR + GPIO_GDIR);

	/* EPDC_PWRCTRL0 - GPIO2[7] for EPD PWR CTL0 */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRCTRL0__GPIO_2_7);
	/* Set as output */
	reg = readl(GPIO2_BASE_ADDR + GPIO_GDIR);
	reg |= (1 << 7);
	writel(reg, GPIO2_BASE_ADDR + GPIO_GDIR);
}

void epdc_power_on()
{
	unsigned int reg;

	/* Set EPD_PWR_CTL0 to high - enable EINK_VDD (3.15) */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg |= (1 << 7);
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);

	/* Set PMIC Wakeup to high - enable Display power */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg |= (1 << 14);
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);

	/* Wait for PWRGOOD == 1 */
	while (1) {
		reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
		if (!(reg & (1 << 13)))
			break;

		udelay(100);
	}

	/* Enable VCOM */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg |= (1 << 3);
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);

	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);

	udelay(500);
}

void  epdc_power_off()
{
	unsigned int reg;
	/* Set PMIC Wakeup to low - disable Display power */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg &= ~(1 << 14);
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);

	/* Disable VCOM */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg &= ~(1 << 3);
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);

	/* Set EPD_PWR_CTL0 to low - disable EINK_VDD (3.15) */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg &= ~(1 << 7);
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);
}

int setup_waveform_file()
{
#ifdef CONFIG_WAVEFORM_FILE_IN_MMC
	int mmc_dev = get_mmc_env_devno();
	ulong offset = CONFIG_WAVEFORM_FILE_OFFSET;
	ulong size = CONFIG_WAVEFORM_FILE_SIZE;
	ulong addr = CONFIG_WAVEFORM_BUF_ADDR;
	char *s = NULL;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	if (!mmc) {
		printf("MMC Device %d not found\n",
			mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
		blk_cnt, (u_char *)addr);
	flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);

	return (n == blk_cnt) ? 0 : -1;
#else
	return -1;
#endif
}

static void setup_epdc()
{
	unsigned int reg;

	/* epdc iomux settings */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D0__EPDC_SDDO_0);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D1__EPDC_SDDO_1);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D2__EPDC_SDDO_2);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D3__EPDC_SDDO_3);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D4__EPDC_SDDO_4);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D5__EPDC_SDDO_5);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D6__EPDC_SDDO_6);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D7__EPDC_SDDO_7);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDCLK__EPDC_GDCLK);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDSP__EPDC_GDSP);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDOE__EPDC_GDOE);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDRL__EPDC_GDRL);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCLK__EPDC_SDCLK);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDOE__EPDC_SDOE);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDLE__EPDC_SDLE);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDSHR__EPDC_SDSHR);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_BDR0__EPDC_BDR_0);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCE0__EPDC_SDCE_0);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCE1__EPDC_SDCE_1);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCE2__EPDC_SDCE_2);

	/*** epdc Maxim PMIC settings ***/

	/* EPDC PWRSTAT - GPIO2[13] for PWR_GOOD status */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRSTAT__GPIO_2_13);

	/* EPDC VCOM0 - GPIO2[3] for VCOM control */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_VCOM0__GPIO_2_3);

	/* UART4 TXD - GPIO2[14] for EPD PMIC WAKEUP */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRWAKEUP__GPIO_2_14);

	/* EIM_A18 - GPIO2[7] for EPD PWR CTL0 */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRCTRL0__GPIO_2_7);

	/*** Set pixel clock rates for EPDC ***/

	/* EPDC AXI clk from PFD_400M, set to 396/2 = 198MHz */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CHSCCDR);
	reg &= ~0x3F000;
	reg |= (0x4 << 15) | (1 << 12);
	writel(reg, CCM_BASE_ADDR + CLKCTL_CHSCCDR);

	/* EPDC AXI clk enable */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR3);
	reg |= 0x0030;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR3);

	/* EPDC PIX clk from PFD_540M, set to 540/4/5 = 27MHz */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CSCDR2);
	reg &= ~0x03F000;
	reg |= (0x5 << 15) | (4 << 12);
	writel(reg, CCM_BASE_ADDR + CLKCTL_CSCDR2);

	reg = readl(CCM_BASE_ADDR + CLKCTL_CBCMR);
	reg &= ~0x03800000;
	reg |= (0x3 << 23);
	writel(reg, CCM_BASE_ADDR + CLKCTL_CBCMR);

	/* EPDC PIX clk enable */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR3);
	reg |= 0x0C00;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR3);

	panel_info.epdc_data.working_buf_addr = CONFIG_WORKING_BUF_ADDR;
	panel_info.epdc_data.waveform_buf_addr = CONFIG_WAVEFORM_BUF_ADDR;

	panel_info.epdc_data.wv_modes.mode_init = 0;
	panel_info.epdc_data.wv_modes.mode_du = 1;
	panel_info.epdc_data.wv_modes.mode_gc4 = 3;
	panel_info.epdc_data.wv_modes.mode_gc8 = 2;
	panel_info.epdc_data.wv_modes.mode_gc16 = 2;
	panel_info.epdc_data.wv_modes.mode_gc32 = 2;

	panel_info.epdc_data.epdc_timings = panel_timings;

	setup_epdc_power();

	/* Assign fb_base */
	gd->fb_base = CONFIG_FB_BASE;
}
#endif

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

#ifdef CONFIG_IMX_ECSPI
s32 spi_get_cfg(struct imx_spi_dev_t *dev)
{
	switch (dev->slave.cs) {
	case 0:
		/* SPI-NOR */
		dev->base = ECSPI1_BASE_ADDR;
		dev->freq = 25000000;
		dev->ss_pol = IMX_SPI_ACTIVE_LOW;
		dev->ss = 0;
		dev->fifo_sz = 64 * 4;
		dev->us_delay = 0;
		break;
	default:
		printf("Invalid Bus ID!\n");
		break;
	}

	return 0;
}

void spi_io_init(struct imx_spi_dev_t *dev)
{
	u32 reg;

	switch (dev->base) {
	case ECSPI1_BASE_ADDR:
		/* Enable clock */
		reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR1);
		reg |= 0x3;
		writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR1);
		/* SCLK */
		mxc_iomux_v3_setup_pad(MX6SL_PAD_ECSPI1_SCLK__ECSPI1_SCLK);

		/* MISO */
		mxc_iomux_v3_setup_pad(MX6SL_PAD_ECSPI1_MISO__ECSPI1_MISO);

		/* MOSI */
		mxc_iomux_v3_setup_pad(MX6SL_PAD_ECSPI1_MOSI__ECSPI1_MOSI);

		if (dev->ss == 0)
			mxc_iomux_v3_setup_pad(MX6SL_PAD_ECSPI1_SS0__ECSPI1_SS0);
		break;
	case ECSPI2_BASE_ADDR:
	case ECSPI3_BASE_ADDR:
		/* ecspi2-3 fall through */
		break;
	default:
		break;
	}
}
#endif

#ifdef CONFIG_MXC_FEC
iomux_v3_cfg_t enet_pads[] = {
	/* LAN8720A */
	MX6SL_PAD_FEC_MDIO__FEC_MDIO,
	MX6SL_PAD_FEC_MDC__FEC_MDC,
	MX6SL_PAD_FEC_RXD0__FEC_RDATA_0,
	MX6SL_PAD_FEC_RXD1__FEC_RDATA_1,
	MX6SL_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX6SL_PAD_FEC_TXD0__FEC_TDATA_0,
	MX6SL_PAD_FEC_TXD1__FEC_TDATA_1,
	MX6SL_PAD_FEC_TX_EN__FEC_TX_EN,
#ifdef CONFIG_FEC_CLOCK_FROM_ANATOP
	MX6SL_PAD_FEC_REF_CLK__FEC_REF_OUT, /* clock from anatop */
#else
	MX6SL_PAD_FEC_REF_CLK__GPIO_4_26, /* clock from OSC */
#endif
	MX6SL_PAD_FEC_RX_ER__FEC_RX_ER,
	MX6SL_PAD_FEC_TX_CLK__GPIO_4_21, /* Phy power enable */
};

void enet_board_init(void)
{
	unsigned int reg;
	mxc_iomux_v3_setup_multiple_pads(enet_pads,
			ARRAY_SIZE(enet_pads));

	/*set GPIO4_26 input as FEC clock*/
	reg = readl(GPIO4_BASE_ADDR + 0x04);
	reg &= ~(1 << 26);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	/* phy power enable and reset: gpio4_21 */
	/* DR: High Level on: Power ON */
	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg |= (1 << 21);
	writel(reg, GPIO4_BASE_ADDR + 0x0);

	/* DIR: output */
	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= (1 << 21);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	/* wait RC ms for hw reset */
	udelay(500);
}

#define ANATOP_PLL_LOCK                 0x80000000
#define ANATOP_PLL_PWDN_MASK            0x00001000
#define ANATOP_PLL_BYPASS_MASK          0x00010000
#define ANATOP_FEC_PLL_ENABLE_MASK      0x00002000

static int setup_fec(void)
{
	u32 reg = 0;
	s32 timeout = 100000;

	/* get enet tx reference clk from internal clock from anatop
	 * GPR1[14] = 0, GPR1[18:17] = 00
	 */
	reg =  readl(IOMUXC_BASE_ADDR + 0x4);
	reg &= ~(0x3 << 17);
	reg &= ~(0x1 << 14);
	writel(reg, IOMUXC_BASE_ADDR + 0x4);

#ifdef CONFIG_FEC_CLOCK_FROM_ANATOP
	/* Enable PLLs */
	reg = readl(ANATOP_BASE_ADDR + 0xe0); /* ENET PLL */
	if ((reg & ANATOP_PLL_PWDN_MASK) || (!(reg & ANATOP_PLL_LOCK))) {
		reg &= ~ANATOP_PLL_PWDN_MASK;
		writel(reg, ANATOP_BASE_ADDR + 0xe0);
		while (timeout--) {
			if (readl(ANATOP_BASE_ADDR + 0xe0) & ANATOP_PLL_LOCK)
				break;
		}
		if (timeout <= 0)
			return -1;
	}

	/* Enable FEC clock */
	reg |= ANATOP_FEC_PLL_ENABLE_MASK;
	reg &= ~ANATOP_PLL_BYPASS_MASK;
	writel(reg, ANATOP_BASE_ADDR + 0xe0);
#endif
	return 0;
}
#endif

int board_init(void)
{
	mxc_iomux_v3_init((void *)IOMUXC_BASE_ADDR);
	setup_boot_device();

	/* board id for linux */
	gd->bd->bi_arch_number = MACH_TYPE_MX6SL_ARM2;

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	setup_uart();

#ifdef CONFIG_MXC_FEC
	setup_fec();
#endif

#ifdef CONFIG_MXC_EPDC
	setup_epdc();
#endif
	return 0;
}

int board_late_init(void)
{
	return 0;
}

int checkboard(void)
{
	printf("Board: MX6SoloLite-ARM2:[ ");

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

	printf("Boot Device: ");
	switch (get_boot_device()) {
	case WEIM_NOR_BOOT:
		printf("NOR\n");
		break;
	case ONE_NAND_BOOT:
		printf("ONE NAND\n");
		break;
	case PATA_BOOT:
		printf("PATA\n");
		break;
	case SATA_BOOT:
		printf("SATA\n");
		break;
	case I2C_BOOT:
		printf("I2C\n");
		break;
	case SPI_NOR_BOOT:
		printf("SPI NOR\n");
		break;
	case SD_BOOT:
		printf("SD\n");
		break;
	case MMC_BOOT:
		printf("MMC\n");
		break;
	case NAND_BOOT:
		printf("NAND\n");
		break;
	case UNKNOWN_BOOT:
	default:
		printf("UNKNOWN\n");
		break;
	}
	return 0;
}

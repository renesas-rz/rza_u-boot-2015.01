/*
 * Copyright (C) 2008-2013 Renesas Solutions Corp.
 * Copyright (C) 2012 Renesas Electronics Europe Ltd.
 * Copyright (C) 2012 Phil Edworthy
 * Copyright (C) 2008 Nobuhiro Iwamatsu
 *
 * Based on u-boot/board/rsk7264/rsk7264.c
 *
 * This file is released under the terms of GPL v2 and any later version.
 * See the file COPYING in the root directory of the source tree for details.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/processor.h>
#include <i2c.h>
#include <rtc.h>
#include <asm/arch/rza1-regs.h>

DECLARE_GLOBAL_DATA_PTR;

int checkboard(void)
{
	puts("BOARD: Renesas R7S72100\n");
	return 0;
}

int board_init(void)
{
	gd->bd->bi_boot_params = (CONFIG_SYS_SDRAM_BASE + 0x100);
	return 0;
}

int board_early_init_f(void)
{
	rtc_reset();	/* to start rtc */
	return 0;
}

int board_late_init(void)
{
	u8 mac[6];
	u8 tmp[1];

	/* Read Mac Address and set*/
	i2c_init(CONFIG_SYS_I2C_SPEED, 0);
	i2c_set_bus_num(CONFIG_SYS_I2C_MODULE);

	/*
	 * PORT EXPANDER
	 *
	 * PX1.0  LED1             O  1  0=ON, 1=OFF
	 * PX1.1  LED2             O  1  0=ON, 1=OFF
	 * PX1.2  LED3             O  1  0=ON, 1=OFF
	 * PX1.3  NOR_A25          O  0  Bit #25 of NOR Flash Addressing
	 * PX1.4  PMOD1_RST        O  1  Reset for PMOD channel 1
	 * PX1.5  PMOD2_RST        O  1  Reset for PMOD channel 2
	 * PX1.6  SD_CONN_PWR_EN   O  0  Enable power supply for external SD card
	 * PX1.7  SD_MMC_PWR_EN    O  1  Enable power supply for MMC card
	 * 
	 * PX2.0  PX1_EN0          O  0  0=LCD, 1=DV
	 * PX2.1  PX1_EN1          O  1  0=General Data, 1=Ethernet
	 * PX2.2  TFT_CS           O  0  Chip select for TFT
	 * PX2.3  PX1_EN3          O  0  0=PWM timer channels, 1=ADC/DAC I/O lines
	 * PX2.4  USB_OVR_CURRENT  I  1  Signal from USB power controller (over-current)
	 * PX2.5  USB_PWR_ENA      O  0  Enable power supply for USB channel 0
	 * PX2.6  USB_PWR_ENB      O  0  Enable power supply for USB channel 1
	 * PX2.7  PX1_EN7          O  0  0=A18-A21, 1=SGOUT0-SGOUT4
	 */

	/* init PX01(IC34) */
	tmp[0] = 0x00;
	i2c_write(0x20, 3, 1, tmp, 1); /* config */
	tmp[0] = 0xB7;
	i2c_write(0x20, 1, 1, tmp, 1); /* output */

	/*init PX02(IC35) */
	tmp[0] = 0x10;
	i2c_write(0x21, 3, 1, tmp, 1); /* config */
	tmp[0] = 0x12;
	i2c_write(0x21, 1, 1, tmp, 1); /* output */

	/* Read MAC address */
	i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR,
		 CONFIG_SH_ETHER_EEPROM_ADDR,
		 CONFIG_SYS_I2C_EEPROM_ADDR_LEN,
		 mac, 6);

	if (is_valid_ether_addr(mac))
		eth_setenv_enetaddr("ethaddr", mac);

	return 0;
}

int dram_init(void)
{
#if (1 !=  CONFIG_NR_DRAM_BANKS)
# error CONFIG_NR_DRAM_BANKS must set 1 in this board.
#endif
	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
	gd->bd->bi_dram[0].size = CONFIG_SYS_SDRAM_SIZE;
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE * CONFIG_NR_DRAM_BANKS;

	return 0;
}

void reset_cpu(ulong addr)
{
}

void led_set_state(unsigned short value)
{
}

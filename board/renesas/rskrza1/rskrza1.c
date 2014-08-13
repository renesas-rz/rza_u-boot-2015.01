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

#include <spi.h>
#include <spi_flash.h>

//#define DEBUG

int spi_flash_cmd(struct spi_slave *spi, u8 cmd, void *response, size_t len);
struct spi_flash *spi_flash_probe(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int spi_mode);
int spi_flash_cmd_write(struct spi_slave *spi, const u8 *cmd, size_t cmd_len,
		const void *data, size_t data_len);

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

#if !defined(CONFIG_BOOT_MODE0)
	printf(	"\t\t      SPI Flash Memory Map\n"
		"\t\t------------------------------------\n"
		"\t\t         Start      Size     SPI\n");
	printf(	"\t\tu-boot:  0x%08X 0x%06X 0\n", 0,CONFIG_ENV_OFFSET);
	printf(	"\t\t   env:  0x%08X 0x%06X 0\n", CONFIG_ENV_OFFSET, CONFIG_ENV_SIZE);
	printf(	"\t\t    DT:  0x%08X 0x%06X 0\n", CONFIG_ENV_OFFSET+CONFIG_ENV_SIZE,CONFIG_ENV_SECT_SIZE);
	printf(	"\t\tKernel:  0x%08X 0x%06X 0+1 (size*=2)\n",0x100000, 0x280000);
	printf(	"\t\trootfs:  0x%08X 0x%06X 0+1 (size*=2)\n",0x4000000, 0x4000000-0x4000000);
#endif

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

/* XIP Kernel boot */
int do_bootx(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong machid = MACH_TYPE_RSKRZA1;
	void (*kernel_entry)(int zero, int arch, uint params);
	ulong r2;
	ulong img_addr;
	char *endp;

	/* need at least two arguments */
	if (argc < 2)
		goto usage;
/*
	if (argc < 2) {
		if (strcmp(argv[3], "xxx") == 0) {
			; ; ;
		}
	}
*/
	img_addr = simple_strtoul(argv[1], &endp, 16);
	kernel_entry = (void (*)(int, int, uint))img_addr;

#ifdef CONFIG_USB_DEVICE
	udc_disconnect();
#endif
	cleanup_before_linux();

	r2 = simple_strtoul(argv[2], NULL, 16);
#if 0
#ifdef CONFIG_OF_LIBFDT
	if (images->ft_len)
		r2 = (unsigned long)images->ft_addr;
	else
#endif
		r2 = gd->bd->bi_boot_params;
#endif
	printf("Booting Linux...\n");

	kernel_entry(0, machid, r2);

	return 0;

usage:
	return CMD_RET_USAGE;
}
static char bootx_help_text[] =
	"x_addr dt_addr\n    - boot XIP kernel in Flash\n"
	"\t x_addr: Address of XIP kernel in Flash\n"
	"\tdt_addr: Address of Device Tree blob image";
U_BOOT_CMD(
	bootx,	CONFIG_SYS_MAXARGS,	1,	do_bootx,
	"boot XIP kernel in Flash", bootx_help_text
);


#define CMNCR_0	0x3FEFA000	/* Common control register */
#define DRCR_0	0x3FEFA00C	/* Data Read Control Register */
#define DRCMR_0	0x3FEFA010	/* Data Read Command Setting Register */
#define DRENR_0 0x3FEFA01C	/* Data read enable setting register */
#define DROPR_0 0x3FEFA018	/* Data read option setting register */
#define DMDMCR_0 0x3FEFA058	/* SPI Mode Dummy Cycle Setting Register */
#define DRDRENR_0 0x3FEFA05C	/* Data Read DDR Enable Register */


struct read_mode {
	u8 cmd;
	char name[50];
};
#define READ_MODES 5
const struct read_mode modes[READ_MODES] = {
	{0x03, "Read Mode (3-byte Addr)"},
	{0x0C, "Fast Read Mode (4-byte Addr)"},
	{0x6C, "Quad Read Mode (4-byte Addr)"},
	{0xEC, "Quad I/O Read Mode (4-byte Addr)"},
	{0xEE, "Quad I/O DDR Read Mode (4-byte Addr)"},
};

/* QUAD SPI MODE */
int do_qspi(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct spi_flash *my_spi_flash;
	int ret = 0;
	int i;
	u8 data[2];
	u8 cmd;
	u8 dual_chip;
	u8 quad_data;
	u8 quad_addr;
	u8 ddr;
	u32 dmdmcr, drenr, cmncr, drcmr, dropr, drdrenr;

	/* need at least two arguments */
	if (argc < 4)
		goto usage;

	if ( strcmp(argv[1], "single") == 0)
		dual_chip = 0;
	else if ( strcmp(argv[1], "dual") == 0)
		dual_chip = 1;
	else
		goto usage;

	if ( strcmp(argv[2], "a1") == 0)
		quad_addr = 0;
	else if ( strcmp(argv[2], "a4") == 0)
		quad_addr = 1;
	else
		goto usage;

	if ( strcmp(argv[3], "d1") == 0)
		quad_data = 0;
	else if ( strcmp(argv[3], "d4") == 0)
		quad_data = 1;
	else
		goto usage;

	if ( strcmp(argv[4], "sdr") == 0)
		ddr = 0;
	else if ( strcmp(argv[4], "ddr") == 0)
		ddr = 1;
	else
		goto usage;

	/* checks */
	if( quad_addr && !quad_data )
		return CMD_RET_USAGE;
	if( ddr && !quad_addr )
		return CMD_RET_USAGE;

	/* Read initial register values */
	dmdmcr = *(volatile u32 *)DMDMCR_0;
	drenr = *(volatile u32 *)DRENR_0;
	cmncr = *(volatile u32 *)CMNCR_0;
	drcmr = *(volatile u32 *)DRCMR_0;
	dropr = *(volatile u32 *)DROPR_0;
	drdrenr = *(volatile u32 *)DRDRENR_0;

	printf("Current Mode: ");
	cmd = (drcmr >> 16) & 0xFF;
	for( i=0; i < READ_MODES; i++) {
		if( modes[i].cmd == cmd )
			printf("%s\n",modes[i].name);
	}

	/* bus=0, cs=0, speed=1000000 */
	if( dual_chip )
		my_spi_flash = spi_flash_probe(0, 1, 1000000, SPI_MODE_3);
	else
		my_spi_flash = spi_flash_probe(0, 0, 1000000, SPI_MODE_3);

	if (!my_spi_flash) {
		printf("Failed to initialize SPI flash.");
		return 1;
	}

	/* Read Status register (RDSR1 05h) */
	ret |= spi_flash_cmd(my_spi_flash->spi, 0x05, &data[0], 1);

	/* Read Configuration register (RDCR 35h) */
	ret |= spi_flash_cmd(my_spi_flash->spi, 0x35, &data[1], 1);

#ifdef DEBUG
	printf("Initial Status register = %02X\n", data[0]);
	printf("Initial Configuration register = %02X\n", data[1]);
#endif

	/**********************/
	/* Spansion S25FL512S */
	/**********************/

	/* Skip SPI Flas configure if already correct */
//	if ( data[1] != 0x02 ) {
	if ( 1  ) {
		data[0] = 0x00;	/* status reg: Don't Care */
		if( quad_data )
			data[1] = 0x02; /* confg reg: Set QUAD, LC=00b */
		else
			data[1] = 0x02; /* confg reg: clear QUAD, LC=00b */
			//data[1] = 0x00; /* confg reg: clear QUAD, LC=00b */

		if( quad_addr )
#ifdef LC_SET_TO_11 /* Dangerous! Once you set LC=11, it break JTAG programming */
			data[1] = 0xC2; /* confg reg: Set QUAD, LC=11b */
#else
			data[1] = 0x02; /* confg reg: Set QUAD, LC=00b */
#endif
		/* Send Write Enable (WREN 06h) */
		ret |= spi_flash_cmd(my_spi_flash->spi, 0x06, NULL, 0);

		/* Send Write Registers (WRR 01h) */
		cmd = 0x01;
		ret |= spi_flash_cmd_write(my_spi_flash->spi, &cmd, 1, data, 2);

		/* Wait till WIP clears */
		do
			spi_flash_cmd(my_spi_flash->spi, 0x05, &data[0], 1);
		while( data[0] & 0x01 );

#ifdef DEBUG
		ret |= spi_flash_cmd(my_spi_flash->spi, 0x05, &data[0], 1);
		ret |= spi_flash_cmd(my_spi_flash->spi, 0x35, &data[1], 1);

		printf("Status register after setting = %02X\n", data[0]);
		printf("Configuration register after setting = %02X\n", data[1]);
#endif
	}

	/* Done with SPI Flash */
	spi_flash_free(my_spi_flash);

	if ( ret )
	{
		printf("Failed to set SPI Flash Configuratin register.\n");
		return 1;
	}

	/***************************/
	/* Set up RZ SPI Registers */
	/***************************/
	/* Enable data swap (SFDE) */
	/* Keeps the endian order of bytes the same on the internal bus
	   regardless of how you fetched them over SPI */
	cmncr |= 0x01000000UL;

	if( dual_chip ) {
		/* Switch to dual memory */
		cmncr |= 0x00000001UL;

	}
	else {
		/* Switch to single memory */
		cmncr &= ~0x00000001UL;
	}

	/* 1-bit address, 4-bit data */
	if( quad_data && !quad_addr ) {
		/* Set read cmd to 0x6C (Quad Read) */
		drcmr = 0x006C0000UL;

		/* width: 1-bit cmd, 1-bit addr, 4-bit data */
		/* address: 32 bits */
		drenr = 0x00024f00UL;

		/* According to the Spansion spec (Table 8.5), dummy cycles
		   are needed when LC=00b for QUAD READ commands */
		/* Add extra Dummy cycles between address and data */
		dmdmcr = 0x00020007; /* 4 bit width, 8 cycles */
		drenr |= 0x00008000; /* Set Dummy Cycle Enable (DME) */
	}

	/* 1-bit address, 1-bit data */
	if( !quad_data && !quad_addr ) {
		/* Set read cmd to 0x0C (FAST Read) */
		drcmr =0x000C0000;

		/* width: 1-bit cmd, 1-bit addr, 1-bit data */
		/* address: 32 bits */
		drenr = 0x00004f00;

		/* According to the Spansion spec (Table 8.5), dummy cycles
		   are needed when LC=00b for FAST READ commnds */
		/* Add extra Dummy cycles between address and data */
		dmdmcr = 0x00000007; /* 1 bit width, 8 cycles */
		drenr |= 0x00008000; /* Set Dummy Cycle Enable (DME) */
	}

	/* 4-bit address, 4-bit data */
	if( quad_addr ) {
		/* Spansion S25FL512S */
		/* Single Data Rate, Quad I/O Read, Latency Code =00b
			<> command = 1-bit, 8 clocks
			<> Addr(32bit) = 4-bit, 8 clocks,
			<> Mode = 4-bit, 2 clocks
			<> Dummy = 4-bit, 4 clocks
			<> Data = 4-bit, 2 clocks x {length}

			See "Figure 10.37 Quad I/O Read Command Sequence" in Spansion spec
		*/

		/* Set read cmd to 0xEC (Quad I/O) */
		drcmr =0x00EC0000;

		/* width: 1-bit cmd, 4-bit addr, 4-bit data */
		/* address: 32 bits */
		drenr = 0x02024f00;

		/* Use Option data regsiters to output 0x00 to write the
		   'mode' byte by sending OPD3 (at 4-bit) between address
		   and dummy */
		dropr = 0x00000000;
		drenr |= 0x00200080;	// send OPD3 at 4-bits

		/* According to the Spansion spec (Table 8.5), dummy cycles
		   are needed when LC=00b for QUAD I/O READ commnds */
		/* Add extra Dummy cycles between address and data */
#ifdef LC_SET_TO_11 /* Dangerous! Once you set LC=11, it break JTAG programming */
		dmdmcr = 0x00020000; /* 4 bit size, 1 cycle */
#else
		dmdmcr = 0x00020003; /* 4 bit size, 4 cycles */
#endif
		drenr |= 0x00008000; /* Set Dummy Cycle Enable (DME) */
	}

	if ( ddr ) {
		printf( "WARNING: DDR mode doesn't actually work yet on the RSKRZA1 board.\n"
			"   The Spansion SPI flash has an extra phase in the command stream\n"
			"   that we can't account for.\n");

		/* Set read cmd to 0xEE (Read DDR Quad I/O) */
		drcmr =0x00EE0000;

		/* Address, option and data all 4-bit DDR */
		drdrenr = 0x00000111;

		/* According to the Spansion spec (Table 8.5), dummy cycles
		   are needed when LC=00b for READ DDR QUAD I/O commnds */
		/* Add extra Dummy cycles between address and data */
		dmdmcr = 0x00020005; /* 4 bit size, 6 cycles */
		drenr |= 0x00008000; /* Set Dummy Cycle Enable (DME) */
	}
	else {
		drdrenr = 0;
	}

	/* Set new register values */
	*(volatile u32 *)DMDMCR_0 = dmdmcr;
	*(volatile u32 *)DRENR_0 = drenr;
	*(volatile u32 *)CMNCR_0 = cmncr;
	*(volatile u32 *)DRCMR_0 = drcmr;
	*(volatile u32 *)DROPR_0 = dropr;
	*(volatile u32 *)DRDRENR_0 = drdrenr;

	/* Turn Read Burst on, Burst Length=2 uints (also set cache flush) */
	/* Keep SSL low (SSLE=1) in case the next transfer is continugous with
	   our last...saves on address cycle. */
	*(u32 *)DRCR_0 = 0x00010301;
	*(volatile u32 *)DRCR_0;	/* Read must be done after cache flush */

	printf("New Mode: ");
	cmd = (*(volatile long *)DRCMR_0 >> 16) & 0xFF;
	for( i=0; i < READ_MODES; i++) {
		if( modes[i].cmd == cmd )
			printf("%s\n",modes[i].name);
	}

	return 0;
usage:
	return CMD_RET_USAGE;
}
static char qspi_help_text[] =
	"Set the XIP Mode for QSPI\n"
	"Usage: qspi [single|dual] [a1|a4] [d1|d4] [sdr|ddr]\n"
	"  'a4' requries 'd4' to be set\n"
	"  'ddr' requries 'd4' and 'a4' to be set\n";
U_BOOT_CMD(
	qspi,	CONFIG_SYS_MAXARGS,	1,	do_qspi,
	"Change QSPI XIP Mode", qspi_help_text
);


/*
 * Copyright (C) 2017 Renesas Electonics America
 * Copyright (C) 2017 Chris Brandt
 *
 * This file is released under the terms of GPL v2 and any later version.
 * See the file COPYING in the root directory of the source tree for details.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/processor.h>
#include <i2c.h>
#include <rtc.h>
#include <rtc.h>
#include <asm/arch/rza1-regs.h>

#include <spi.h>
#include <spi_flash.h>
//#include "command.h"

/* This function temporary disables the feature of OR-ing the results of
   commands together when using dual spi flash memory. When using single
   flash, it does nothing. */
void qspi_disable_auto_combine(void);

//#define DEBUG

/* Port Function Registers */
#define PORTn_base 0xFCFE3000
#define Pn(n)	(PORTn_base + 0x0000 + n * 4)	/* Port register R/W */
#define PSRn(n)	(PORTn_base + 0x0100 + n * 4)	/* Port set/reset register R/W */
#define PPRn(n)	(PORTn_base + 0x0200 + n * 4)	/* Port pin read register R */
#define PMn(n)	(PORTn_base + 0x0300 + n * 4)	/* Port mode register R/W */
#define PMCn(n)	(PORTn_base + 0x0400 + n * 4)	/* Port mode control register R/W */
#define PFCn(n)	(PORTn_base + 0x0500 + n * 4)	/* Port function control register R/W */
#define PFCEn(n)	(PORTn_base + 0x0600 + n * 4)	/* Port function control expansion register R/W */
#define PNOTn(n)	(PORTn_base + 0x0700 + n * 4)	/* Port NOT register W */
#define PMSRn(n)	(PORTn_base + 0x0800 + n * 4)	/* Port mode set/reset register R/W */
#define PMCSRn(n)	(PORTn_base + 0x0900 + n * 4)	/* Port mode control set/reset register R/W */
#define PFCAEn(n)	(PORTn_base + 0x0A00 + n * 4)	/* Port Function Control Additional Expansion register R/W */
#define PIBCn(n)	(PORTn_base + 0x4000 + n * 4)	/* Port input buffer control register R/W */
#define PBDCn(n)	(PORTn_base + 0x4100 + n * 4)	/* Port bi-direction control register R/W */
#define PIPCn(n)	(PORTn_base + 0x4200 + n * 4)	/* Port IP control register R/W */

enum pfc_pin_alt_mode {ALT1=1, ALT2, ALT3, ALT4, ALT5, ALT6, ALT7, ALT8};
enum pfc_pin_gpio_mode {GPIO_OUT=0, GPIO_IN=1};

const u32 alt_settings[9][3] = {
	/* PFCAEn, PFCEn, PFCn */
	{0,0,0},/* dummy */
	{0,0,0},/* 1st alternative function */
	{0,0,1},/* 2nd alternative function */
	{0,1,0},/* 3rd alternative function */
	{0,1,1},/* 4th alternative function */
	{1,0,0},/* 5th alternative function */
	{1,0,1},/* 6th alternative function */
	{1,1,0},/* 7th alternative function */
	{1,1,1},/* 8th alternative function */
};

/* Arguments:
   n = port(1-11)
   b = bit(0-15)
   d = direction('GPIO_IN','GPIO_OUT')
*/
void pfc_set_gpio(u8 n, u8 b, u8 d)
{
	*(u32 *)PMCSRn(n) = 1UL<<(b+16);		// Pin as GPIO
	*(u32 *)PMSRn(n) = 1UL<<(b+16) | (u32)d << b;	// Set pin IN/OUT

	if( d == GPIO_IN )
		*(u16 *)PIBCn(n) |= 1UL << b; 		// Enable input buffer
	else
		*(u16 *)PIBCn(n) &= ~(1UL << b); 	// Disable input buffer
}

/* Arguments:
   n = port(1-11)
   b = bit(0-15)
   v = value (0 or 1)
*/
void gpio_set(u8 n, u8 b, u8 v)
{
	/* The pin should have been configured as GPIO_OUT using pfc_set_gpio */

	/* Use the 'Port Set and Reset Register' to only effect the pin bit */
	/* Upper WORD is the bit mask, the lower WORD is the desire pin level */
	*(u32 *)PSRn(n) = 1UL<<(b+16) | (u32)v<< b;	// Set pin to 0/1
}

/* Arguments:
   n = port(1-11)
   b = bit(0-15)
   return = current pin level (0 or 1);
*/
u8 gpio_read(u8 n, u8 b)
{
	/* The pin should have been configured as GPIO_IN using pfc_set_gpio */
printf("PPRn(%d) %04X\n",n,*(u16 *)PPRn(n));
	return ( *(u16 *)PPRn(n) >> b ) & 0x0001;
}


/* Arguments:
    n = port number (P1-P11)
    b = bit number (0-15)
    alt = Alternative mode ('ALT1'-'ALT7')
    inbuf =  Input buffer (0=disabled, 1=enabled)
    bi = Bidirectional mode (0=disabled, 1=enabled)
*/
void pfc_set_pin_function(u16 n, u16 b, u16 alt, u16 inbuf, u16 bi)
{
	u16 value;

	/* Set PFCAEn */
	value = *(u16 *)PFCAEn(n);
	value &= ~(1UL<<b); // clear
	value |= (alt_settings[alt][0] & 1UL) << b; // set(maybe)
	*(u16 *)PFCAEn(n) = value;

	/* Set PFCEn */
	value = *(u16 *)PFCEn(n);
	value &= ~(1UL<<b); // clear
	value |= (alt_settings[alt][1] & 1UL) << b; // set(maybe)
	*(u16 *)PFCEn(n) = value;

	/* Set PFCn */
	value = *(u16 *)PFCn(n);
	value &= ~(1UL<<b); // clear
	value |= (alt_settings[alt][2] & 1UL) << b; // set(maybe)
	*(u16 *)PFCn(n) = value;

	/* Set Pn */
	/* Not used for alternative mode */
	/* NOTE: PIP must be set to '0' for the follow peripherals and Pn must be set instead
		<> Multi-function timer pulse unit
		<> LVDS output interface
		<> Serial sound interface
	   For those, use this to set Pn: *(u32 *)PSRn(n) = 1UL<<(b+16) | direction<<(b);
	*/

	/* Set PIBCn */
	value = *(u16 *)PIBCn(n);
	value &= ~(1UL<<b); // clear
	value |= inbuf << b; // set(maybe)
	*(u16 *)PIBCn(n) = value;

	/* Set PBDCn */
	value = *(u16 *)PBDCn(n);
	value &= ~(1UL<<b); // clear
	value |= bi << b; // set(maybe)
	*(u16 *)PBDCn(n) = value;

	/* Alternative mode '1' (not GPIO '0') */
	*(u32 *)PMCSRn(n) |= 1UL<<(b+16) | 1UL<<(b);

	/* Set PIPCn so pin used for function '1'(not GPIO '0') */
	*(u16 *)PIPCn(n) |= 1UL <<b;
}

enum mstp_clocks
{
	CLK_ALL=-1,
	CLK_DISABLE=0,
	CLK_ENABLE=1,
	CLK_MTU2=0x0303,	/* MSTP33 */
	CLK_SCIF0=0x0407,	/* MSTP47 */
	CLK_SCIF1=0x0406,	/* MSTP46 */
	CLK_SCIF2=0x0405,	/* MSTP45 */
	CLK_SCIF3=0x0404,	/* MSTP44 */
	CLK_SCIF4=0x0403,	/* MSTP43 */
	CLK_SCIF5=0x0402,	/* MSTP42 */
	CLK_SCIF6=0x0401,	/* MSTP41 */
	CLK_SCIF7=0x0400,	/* MSTP40 */

	CLK_SCI0=0x0507,	/* MSTP57 */
	CLK_SCI1=0x0506,	/* MSTP56 */
	CLK_OSTM0=0x0501,	/* MSTP51 */
	CLK_OSTM1=0x0500,	/* MSTP50 */

	CLK_RTC=0x0600,		/* MSTP60*/

	CLK_ETHERNET=0x0704,	/* MSTP74*/
};

void clock_enable(int clock, int enable)
{
	u8 offset = (clock >> 8) & 0xFF;
	u8 bit = 1 << (clock & 0xFF);
	u32 address = STBCR1 - 4;

	if (clock == CLK_ALL) {

	} else
	{
		address += offset * 4;
		if (enable)
			*(u8 *)address &= ~bit;
		else
			*(u8 *)address |= bit;
	}
}


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
	/* This function runs early in the boot process, before u-boot is relocated
	   to RAM (hence the '_f' in the function name stands for 'still running from
	   flash'). A temporary stack has been set up for us which is why we can
	   have this as C code. */
#ifndef SDRAM_NONE
	int i;
#endif

	rtc_reset();	/* to start rtc */

	/* =========== Pin Setup =========== */
	/* Specific for the RZ/A1L on the Stream it board. Adjust for your board as needed. */

	/* Serial Console */
	pfc_set_pin_function(7, 11, ALT5, 0, 0);/* P7_11 = TxD3 */
	pfc_set_pin_function(7, 10, ALT5, 0, 0);/* P7_11 = RxD3 */

	/* QSPI_0 ch0 (booted in 1-bit, need to change to 4-bit) */
	pfc_set_pin_function(4, 4, ALT2, 0, 0);	/* P4_4 = SPBCLK_0 */
	pfc_set_pin_function(4, 5, ALT2, 0, 0);	/* P4_5 = SPBSSL_0 */
	pfc_set_pin_function(4, 6, ALT2, 0, 1);	/* P4_6 = SPBIO00_0 (bi dir) */
	pfc_set_pin_function(4, 7, ALT2, 0, 1);	/* P4_7 = SPBIO10_0 (bi dir) */
	pfc_set_pin_function(4, 2, ALT2, 0, 1);	/* P4_2 = SPBIO20_0 (bi dir) */
	pfc_set_pin_function(4, 3, ALT2, 0, 1);	/* P4_3 = SPBIO30_0 (bi dir) */

	/* RIIC Ch 0 (EEPROM) */
	pfc_set_pin_function(1, 0, ALT1, 0, 1);	/* P1_0 = RIIC3SCL (bi dir) */
	pfc_set_pin_function(1, 1, ALT1, 0, 1);	/* P1_1 = RIIC3SDA (bi dir) */

	/* Enable USB 5 Volt supply */
	pfc_set_gpio(7, 1, GPIO_OUT); /* P7_1 = GPIO_OUT */
	gpio_set(7, 1, 1);

	/* Ethernet */
	pfc_set_pin_function(8, 14, ALT2, 0, 0); /* P8_14 = ET_COL */
	pfc_set_pin_function(9, 0, ALT2, 0, 0);	/* P9_0 = ET_MDC */
	pfc_set_pin_function(9, 1, ALT2, 0, 1);	/* P9_1 = ET_MDIO (bi dir) */
	pfc_set_pin_function(9, 2, ALT2, 0, 0);	/* P9_2 = ET_RXCLK */
	pfc_set_pin_function(9, 3, ALT2, 0, 0);	/* P9_3 = ET_RXER */
	pfc_set_pin_function(9, 4, ALT2, 0, 0);	/* P9_4 = ET_RXDV */
	pfc_set_pin_function(8, 4, ALT2, 0, 0);	/* P8_4 = ET_TXCLK */
	pfc_set_pin_function(8, 5, ALT2, 0, 0);	/* P8_5 = ET_TXER */
	pfc_set_pin_function(8, 6, ALT2, 0, 0);	/* P8_6 = ET_TXEN */
	pfc_set_pin_function(8, 15, ALT2, 0, 0);	/* P8_15 = ET_CRS */
	pfc_set_pin_function(8, 0, ALT2, 0, 0);	/* P8_0 = ET_TXD0 */
	pfc_set_pin_function(8, 1, ALT2, 0, 0);	/* P8_1 = ET_TXD1 */
	pfc_set_pin_function(8, 2, ALT2, 0, 0);	/* P8_2 = ET_TXD2 */
	pfc_set_pin_function(8, 3, ALT2, 0, 0);	/* P8_3 = ET_TXD3 */
	pfc_set_pin_function(8, 7, ALT2, 0, 0);	/* P8_7 = ET_RXD0 */
	pfc_set_pin_function(8, 8, ALT2, 0, 0);	/* P8_8 = ET_RXD1 */
	pfc_set_pin_function(8, 9, ALT2, 0, 0); /* P2_9 = ET_RXD2 */
	pfc_set_pin_function(8, 10, ALT2, 0, 0); /* P2_10 = ET_RXD3 */
	//pfc_set_pin_function(4, 14, ALT8, 0, 0); /* P4_14 = IRQ6 (ET_IRQ) */ /* NOTE: u-boot doesn't enable interrupts */

	/* Release Ethernet from reset  */
	pfc_set_gpio(2, 7, GPIO_OUT); /* P2_7 = GPIO_OUT */
	gpio_set(2, 7, 1);

#ifndef SDRAM_NONE /* If no SDRAM, skip this entire secton */

	/* SDRAM */
	pfc_set_pin_function(2, 0, ALT1, 0, 0);	/* P2_0 = CS3 */
	for(i=0;i<=15;i++)
		pfc_set_pin_function(5, i, ALT1, 0, 1);	/* P5_0~15 = D0-D15 (bi dir) */
	pfc_set_pin_function(2, 1, ALT1, 0, 0);	/* P2_1 = RAS */
	pfc_set_pin_function(2, 2, ALT1, 0, 0);	/* P2_2 = CAS */
	pfc_set_pin_function(2, 3, ALT1, 0, 0);	/* P2_3 = CKE */
	pfc_set_pin_function(2, 6, ALT1, 0, 0);	/* P2_6 = RD/WR */
	pfc_set_pin_function(2, 4, ALT1, 0, 0);	/* P2_4 = WE0/DQMLL */
	pfc_set_pin_function(2, 5, ALT1, 0, 0);	/* P2_5 = WE1/DQMLU */
#ifdef SDRAM_W9825G6KH_6I
	for(i=0;i<=14;i++)
		pfc_set_pin_function(3, i, ALT1, 0, 0);	/* P3_0~14: A1-A15 */
#endif
#ifdef SDRAM_IS42_45S16800F
	for(i=0;i<=13;i++)
		pfc_set_pin_function(3, i, ALT1, 0, 0);	/* P3_0~13: A1-A14 */
#endif

	/* RED LED */
	pfc_set_gpio(7, 8, GPIO_OUT); /* P7_8 = GPIO_OUT */

	/* USER_SW */
	pfc_set_gpio(7, 9, GPIO_IN); /* P7_9 = GPIO_IN */

	/**********************************************/
	/* Configure SDRAM (CS3)                      */
	/**********************************************/
	/*
	 * NOTE: Because the Stream it board was laid out for a 32MB SDRAM chip, when using a 16MB chip
	 * the BA1 pin (pin 21) on the SDRAM needs to be lifted and then jumpered to its A12 (pin 36)
	 * so that it can be connected to the RZ's A13 signal. Pin 36 on this SDRAM chip is a NC, so
	 * driving it with A13 is not an issue.
	 *
	 * Using a 16MB SDRAM instead of a 32MB SDRAM has the benefit of freeing up the CEU clock (VIO_CLK)
	 * that is shared with the A15 signal on port P3_14.
	 */

//	#define CS2BCR_D	0x00004C00	/* Type=SDRAM, 16-bit memory */
//	#define CS2WCR_D	0x00000480	/* CAS Latency = 2 */
	#define CS3BCR_D	0x00004C00	/* Type=SDRAM, 16-bit memory */

#ifdef SDRAM_IS42_45S16800F
	/* ISSI 128Mb SYNCHRONOUS DRAM (16MByte)
	 *   IS42/45S16800F
	 *   2M x 16 x 4 Banks
	 */
	#define CS3WCR_D	2 << 13	|	/* (CS2,CS3) WTRP (2 cycles) */\
				2 << 10 |	/* (CS2,CS3) WTRCD (2 cycles) */\
				1 <<  7 |	/*     (CS3) A3CL (CAS Latency = 2) */\
				2 <<  3 |	/* (CS2,CS3) TRWL (2 cycles) */\
				3 <<  0		/* (CS2,CS3) WTRC (8 cycles) */
	#define SDCR_D	0x00090809	/* 12-bit row, 9-bit col, auto-refresh */
#endif

#ifdef SDRAM_W9825G6KH_6I
	/*
	 *   Winbond 256Mb SYNCHRONOUS DRAM (32MByte)
	 *   W9825G6KH-6I
	 *   4M x 16 x 4 Banks
	 */
	#define CS3WCR_D	2 << 13	|	/* (CS2,CS3) WTRP (2 cycles) */\
				2 << 10 |	/* (CS2,CS3) WTRCD (2 cycles) */\
				2 <<  7 |	/*     (CS3) A3CL (CAS Latency = 3) */\
				2 <<  3 |	/* (CS2,CS3) TRWL (2 cycles) */\
				3 <<  0		/* (CS2,CS3) WTRC (8 cycles) */
	#define SDCR_D	0x00090811	/* 13-bit row, 9-bit col, auto-refresh */
#endif

	/*
	 * You must refresh all rows within the amount of time specified in the memory spec.
	 * Total Refresh time =  [Number_of_rows] / [Clock_Source / Refresh Counter]
	 * 31.5ms =  [4096] /  [(66.6MHz / 4) / 128]
	 */
	#define RTCOR_D		0xA55A0080	/* Refresh Counter = 128 */
	#define RTCSR_D		0xA55A0008	/* Clock Source=CKIO/4 (CKIO=66MHz) */

	//*(u32 *)CS2BCR = CS2BCR_D;
	//*(u32 *)CS2WCR = CS2WCR_D;
	*(u32 *)CS3BCR = CS3BCR_D;
	*(u32 *)CS3WCR = CS3WCR_D;
	*(u32 *)SDCR = SDCR_D;
	*(u32 *)RTCOR = RTCOR_D;
	*(u32 *)RTCSR = RTCSR_D;

	/* wait */
	#define REPEAT_D 0x000033F1
	for (i=0;i<REPEAT_D;i++) {
		asm("nop");
	}

	/* The final step is to set the SDRAM Mode Register by written to a
	   specific address (the data value is ignored) */
	/* Check the hardware manual (table 8.15) if your settings differ */
	/*   Burst Length = 1 (fixed)
	 *   Burst Type = Sequential (fixed)
	 *   CAS Latency = 2 or 3 (see table 8.15)
	 *   Write Burst Mode = [burst read/single write] or [burst read/burst write] (see table 8.15)
	 */
#ifdef SDRAM_IS42_45S16800F
	#define SDRAM_MODE_CS3  0x3FFFE040	/* CS3: CAS=2, burst write, 16bit bus */
#endif
#ifdef SDRAM_W9825G6KH_6I
	#define SDRAM_MODE_CS3  0x3FFFE060	/* CS3: CAS=3, burst write, 16bit bus */
#endif
	//*(u16 *)SDRAM_MODE_CS2 = 0;
	*(u16 *)SDRAM_MODE_CS3 = 0;


#endif /* SDRAM_NONE */

	return 0;
}

int board_late_init(void)
{
	u8 mac[6];

	/* Read Mac Address and set*/
	i2c_init(CONFIG_SYS_I2C_SPEED, 0);
	i2c_set_bus_num(CONFIG_SYS_I2C_MODULE);

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
	printf(	"\t\t    DT:  0x%08X 0x%06X 0\n", 0x000C0000,0x040000);
	printf(	"\t\tKernel:  0x%08X 0x%06X 0\n", 0x200000, 0x600000);
	printf(	"\t\trootfs:  0x%08X 0x%06X 0\n", 0x800000, 0x2000000-0x400000);
#endif

#ifndef SDRAM_NONE
	/* Boot uImage in external SDRAM */
	/* Rootfs is a squashfs image in memory mapped QSPI */
	/* => run s_boot */
	setenv("s1", "sf probe 0; sf read 0C800000 C0000 8000"); // Read out DT blob
	setenv("s2", "sf probe 0; sf read 0CC00000 100000 500000"); //Copy Kernel to SDRAM
#ifdef SDRAM_W9825G6KH_6I
	setenv("s3", "bootm start 0x0CC00000 - 0x0C800000 ; bootm loados ;"\
			"fdt memory 0x0C000000 0x02000000"); // Change memory address in DTB (32MB)
#else
	setenv("s3", "bootm start 0x0CC00000 - 0x0C800000 ; bootm loados ;"\
			"fdt memory 0x0C000000 0x01000000"); // Change memory address in DTB (16MB)
#endif
	setenv("s4", "qspi single"); // Change XIP interface to single QSPI
	setenv("sargs", "console=ttySC3,115200 ignore_loglevel root=/dev/mtdblock0"); // bootargs
	setenv("s_boot", "run s1 s2 s3 s4; setenv bootargs ${sargs}; fdt chosen; bootm go"); // run the commands
#endif

	/* Boot XIP using internal RAM only */
	/* Rootfs is a AXFS image in memory mapped QSPI */
	/* => run xa_boot */
	/* Read out DT blob */
	setenv("xa1", "sf probe 0; sf read 20200000 C0000 8000");
	/* Change memory address in DTB */
	setenv("xa2", "fdt addr 20200000 ; fdt memory 0x20000000 0x00300000"); /* 3MB RAM */
	/* Change XIP interface to single QSPI */
	setenv("xa3", "qspi single");
	setenv("xaargs", "console=ttySC3,115200 ignore_loglevel root=/dev/null rootflags=physaddr=0x18800000"); // bootargs
	setenv("xa_boot", "run xa1 xa2 xa3; setenv bootargs ${xaargs}; fdt chosen; bootx 18200000 20200000"); // run the commands

#ifndef SDRAM_NONE
	/* Boot XIP using external SDRAM RAM */
	/* Rootfs is a AXFS image in memory mapped QSPI */
	/* => run xsa_boot */
	/* Read out DT blob */
	setenv("xsa1", "sf probe 0; sf read 0C800000 C0000 8000");
	/* Change memory address in DTB */
#ifdef SDRAM_W9825G6KH_6I
	setenv("xsa2", "fdt addr 0C800000 ; fdt memory 0x0C000000 0x02000000"); /* 32MB SDRAM RAM */
#else
	setenv("xsa2", "fdt addr 0C800000 ; fdt memory 0x0C000000 0x01000000"); /* 16MB SDRAM RAM */
#endif
	/* Change XIP interface to single QSPI */
	setenv("xsa3", "qspi single");
	setenv("xsaargs", "console=ttySC3,115200 ignore_loglevel earlyprintk root=/dev/null rootflags=physaddr=0x18800000"); // bootargs
	setenv("xsa_boot", "run xsa1 xsa2 xsa3; setenv bootargs ${xsaargs}; fdt chosen; bootx 18200000 0C800000"); // run the commands
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
	/* If you have board specific stuff to do, you can do it
	here before you reboot */

	/* Dummy read (must read WRCSR:WOVF at least once before clearing) */
	*(volatile u8 *)(WRCSR) = *(u8 *)(WRCSR);

	*(volatile u16 *)(WRCSR) = 0xA500;     /* Clear WOVF */
	*(volatile u16 *)(WRCSR) = 0x5A5F;     /* Reset Enable */
	*(volatile u16 *)(WTCNT) = 0x5A00;     /* Counter to 00 */
	*(volatile u16 *)(WTCSR) = 0xA578;     /* Start timer */

	while(1); /* Wait for WDT overflow */
}

void led_set_state(unsigned short value)
{
	if (value)	/* turn LED on */
		gpio_set(7,8,0);
	else		/* turn LED off */
		gpio_set(7,8,1);
}

u8 button_check_state(void)
{
	/* returns: 1 = button up
		    0 = button pressed
	*/

	return gpio_read(7, 9);
}

/* XIP Kernel boot */
int do_bootx(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong machid = MACH_TYPE_STREAMIT;
	void (*kernel_entry)(int zero, int arch, uint params);
	ulong r2;
	ulong img_addr;
	char *endp;

	/* need at least two arguments */
	if (argc < 2)
		goto usage;

	img_addr = simple_strtoul(argv[1], &endp, 16);
	kernel_entry = (void (*)(int, int, uint))img_addr;

#ifdef CONFIG_USB_DEVICE
	udc_disconnect();
#endif
	cleanup_before_linux();

	r2 = simple_strtoul(argv[2], NULL, 16);

	/* The kernel expects the following when booting:
	 *  r0 - 0
	 *  r1 - machine type
	 *  r2 - boot data (atags/dt) pointer
	 *
	 * For more info, refer to:
	 *  https://www.kernel.org/doc/Documentation/arm/Booting
	 */

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
#define DREAR_0 0x3FEFA014	/* Data read extended address setting register */
#define DRENR_0 0x3FEFA01C	/* Data read enable setting register */
#define DROPR_0 0x3FEFA018	/* Data read option setting register */
#define DMDMCR_0 0x3FEFA058	/* SPI Mode Dummy Cycle Setting Register */
#define DRDRENR_0 0x3FEFA05C	/* Data Read DDR Enable Register */

struct read_mode {
	u8 cmd;
	char name[50];
};
#define READ_MODES 9
const struct read_mode modes[READ_MODES] = {
	{0x03, "Read Mode (3-byte Addr) (RZ/A1 reset value)"},
	{0x0C, "Fast Read Mode (4-byte Addr)"},
	{0x6C, "Quad Read Mode (4-byte Addr)"},
	{0xEC, "Quad I/O Read Mode (4-byte Addr)"},
	{0xEE, "Quad I/O DDR Read Mode (4-byte Addr)"},
	{0x0B, "Fast Read Mode (3-byte Addr)"},
	{0x6B, "Quad Read Mode (3-byte Addr)"},
	{0xEB, "Quad I/O Read Mode (3-byte Addr)"},
	{0xED, "Quad I/O DDR Read Mode (3-byte Addr)"},
};

/* If you are using a SPI Flash device that does not have 4-byte address
   commands (Flash size <= 16MB), then change the #if 0 to #if 1 */
#if 0
 #define ADDRESS_BYTE_SIZE 3	/* Addresses are 3-bytes (A0-A23) */
 #define FAST_READ 0x0B		/* Fast Read Mode (1-bit cmd, 1-bit addr, 1-bit data, 3-bytes of address) */
 #define QUAD_READ 0x6B		/* Quad Read Mode (1-bit cmd, 1-bit addr, 4-bit data, 3-bytes of address) */
 #define QUAD_IO_READ 0xEB	/* Quad I/O Read Mode (1-bit cmd, 4-bit addr, 4-bit data, 3-bytes of address) */
 #define QUAD_IO_DDR_READ 0xED	/* Quad I/O DDR Read Mode (1-bit cmd, 1-bit addr, 4-bit data, 3-bytes of address) */
#else
 #define ADDRESS_BYTE_SIZE 4	/* Addresses are 4-bytes (A0-A31) */
 #define FAST_READ 0x0C		/* Fast Read Mode (1-bit cmd, 1-bit addr, 1-bit data, 4-bytes of address) */
 #define QUAD_READ 0x6C		/* Quad Read Mode (1-bit cmd, 1-bit addr, 4-bit data, 4-bytes of address) */
 #define QUAD_IO_READ 0xEC	/* Quad I/O Read Mode (1-bit cmd, 4-bit addr, 4-bit data, 4-bytes of address) */
 #define QUAD_IO_DDR_READ 0xEE	/* Quad I/O DDR Read Mode (1-bit cmd, 1-bit addr, 4-bit data, 4-bytes of address) */
#endif

/* These should be filled out for each device */
u32 g_FAST_RD_DMY;		/* Fast Read Mode */
u32 g_QUAD_RD_DMY;		/* Quad Read Mode  */
u32 g_QUAD_IO_RD_DMY;		/* Quad I/O Read Mode  */
u32 g_QUAD_IO_DDR_RD_DMY;	/* Quad I/O DDR Read Mode  */
u32 g_QUAD_IO_RD_OPT;		/* Addtional option or 'mode' settings */

/**********************/
/* Spansion S25FL512S */
/**********************/
int enable_quad_spansion(struct spi_flash *sf, u8 quad_addr, u8 quad_data )
{
	/* NOTE: Macronix and Windbond are similar to Spansion */
	/* NOTE: Once quad comamnds are enabled, you don't need to disable
		 them to use the non-quad mode commands, so we just always
		 leave them on. */
	int ret = 0;
	u8 data[5];
	u8 cmd;
	u8 spi_cnt = 1;
	u8 st_reg[2];
	u8 cfg_reg[2];

	/* Read ID Register (for cases where not all parts are the same) */
	//ret |= spi_flash_cmd(sf->spi, 0x9F, &data[0], 5);

	if (sf->spi->cs)
		spi_cnt = 2; /* Dual SPI Flash */

	/* Read Status register (RDSR1 05h) */
	qspi_disable_auto_combine();
	ret |= spi_flash_cmd(sf->spi, 0x05, st_reg, 1*spi_cnt);

	/* Read Configuration register (RDCR 35h) */
	qspi_disable_auto_combine();
	ret |= spi_flash_cmd(sf->spi, 0x35, cfg_reg, 1*spi_cnt);

#ifdef DEBUG
	printf("Initial Values:\n");
	for(cmd = 0; cmd <= spi_cnt; cmd++) {
		printf("   SPI Flash %d:\n", cmd+1);
		printf("\tStatus register = %02X\n", st_reg[cmd]);
		printf("\tConfiguration register = %02X\n", cfg_reg[cmd]);
	}
#endif

	/* Skip SPI Flash configure if already correct */
	/* Note that if dual SPI flash, both have to be set */
	if ( (cfg_reg[0] != 0x02 ) ||
	     ((spi_cnt == 2) && (cfg_reg[1] != 0x02 ))) {

		data[0] = 0x00;	/* status reg: Don't Care */
		data[1] = 0x02; /* confg reg: Set QUAD, LC=00b */

		/* Send Write Enable (WREN 06h) */
		ret |= spi_flash_cmd(sf->spi, 0x06, NULL, 0);

		/* Send Write Registers (WRR 01h) */
		cmd = 0x01;
		ret |= spi_flash_cmd_write(sf->spi, &cmd, 1, data, 2);

		/* Wait till WIP clears */
		do
			spi_flash_cmd(sf->spi, 0x05, &data[0], 1);
		while( data[0] & 0x01 );

	}

#ifdef DEBUG
	/* Read Status register (RDSR1 05h) */
	qspi_disable_auto_combine();
	ret |= spi_flash_cmd(sf->spi, 0x05, st_reg, 1*spi_cnt);

	/* Read Configuration register (RDCR 35h) */
	qspi_disable_auto_combine();
	ret |= spi_flash_cmd(sf->spi, 0x35, cfg_reg, 1*spi_cnt);

	printf("New Values:\n");
	for(cmd = 0; cmd <= spi_cnt; cmd++) {
		printf("   SPI Flash %d:\n", cmd+1);
		printf("\tStatus register = %02X\n", st_reg[cmd]);
		printf("\tConfiguration register = %02X\n", cfg_reg[cmd]);
	}
#endif

	/* Finally, fill out the global settings for
	   Number of Dummy cycles between Address and Data */

	/* Spansion S25FL512S */
	/* According to the Spansion spec (Table 8.5), dummy cycles
	   are needed when LC=00 (chip default) for FAST READ,
	   QUAD READ, and QUAD I/O READ commands */
	g_FAST_RD_DMY = 8;		/* Fast Read Mode: 8 cycles */
	g_QUAD_RD_DMY = 8;		/* Quad Read Mode  */
	g_QUAD_IO_RD_DMY = 4;		/* Quad I/O Read Mode: 4 cycles */
	g_QUAD_IO_DDR_RD_DMY = 6;	/* Quad I/O DDR Read Mode  (NOT SUPPORTED) */

	/* When sending a QUAD I/O READ command, and extra MODE field
	   is needed.
	     [[ Single Data Rate, Quad I/O Read, Latency Code=00b ]]
		<> command = 1-bit, 8 clocks
		<> Addr(32bit) = 4-bit, 8 clocks,
		<> Mode = 4-bit, 2 clocks
		<> Dummy = 4-bit, 4 clocks
		<> Data = 4-bit, 2 clocks x {length}
	    See "Figure 10.37 Quad I/O Read Command Sequence" in Spansion spec
	*/
	/* Use Option data regsiters to output '0' as the
	   'Mode' field by sending OPD3 (at 4-bit) between address
	   and dummy */
	g_QUAD_IO_RD_OPT = 1;

	return ret;
}

/************************/
/* Macronix MX25L51235F */
/************************/
int enable_quad_macronix(struct spi_flash *sf, u8 quad_addr, u8 quad_data )
{
	/* NOTE: Macronix and Windbond are similar to Spansion */
	/* NOTE: Once quad comamnds are enabled, you don't need to disable
		 them to use the non-quad mode commands, so we just always
		 leave them on. */
	int ret = 0;
	u8 data[5];
	u8 cmd;
	u8 spi_cnt = 1;
	u8 st_reg[2];
	u8 cfg_reg[2];

	/* Read ID Register (for cases where not all parts are the same) */
	//ret |= spi_flash_cmd(sf->spi, 0x9F, &data[0], 5);

	if (sf->spi->cs)
		spi_cnt = 1; /* Single SPI Flash */

	/* Read Status register (RDSR1 05h) */
	qspi_disable_auto_combine();
	ret |= spi_flash_cmd(sf->spi, 0x05, st_reg, 1*spi_cnt);

	/* Read Configuration register (RDCR 35h) */
	qspi_disable_auto_combine();
	ret |= spi_flash_cmd(sf->spi, 0x35, cfg_reg, 1*spi_cnt);

#ifdef DEBUG
	printf("Initial Values:\n");
	for(cmd = 0; cmd <= spi_cnt; cmd++) {
		printf("   SPI Flash %d:\n", cmd+1);
		printf("\tStatus register = %02X\n", st_reg[cmd]);
		printf("\tConfiguration register = %02X\n", cfg_reg[cmd]);
	}
#endif

	/* Skip SPI Flash configure if already correct */
	/* Note that if dual SPI flash, both have to be set */
	if ( (cfg_reg[0] != 0x02 ) ||
	     ((spi_cnt == 1) && (cfg_reg[1] != 0x02 ))) {

		data[0] = 0x42; /* status reg: Don't Care */
		data[1] = 0x00; /* confg reg: Set QUAD, LC=00b */

		/* Send Write Enable (WREN 06h) */
		ret |= spi_flash_cmd(sf->spi, 0x06, NULL, 0);

		/* Send Write Registers (WRR 01h) */
		cmd = 0x01;
		ret |= spi_flash_cmd_write(sf->spi, &cmd, 1, data, 2);

		/* Wait till WIP clears */
		do
		{
			spi_flash_cmd(sf->spi, 0x05, &data[0], 1);
		}
		while( data[0] & 0x01 );
	}

#ifdef DEBUG
	/* Read Status register (RDSR1 05h) */
	qspi_disable_auto_combine();
	ret |= spi_flash_cmd(sf->spi, 0x05, st_reg, 1*spi_cnt);

	/* Read Configuration register (RDCR 35h) */
	qspi_disable_auto_combine();
	ret |= spi_flash_cmd(sf->spi, 0x35, cfg_reg, 1*spi_cnt);

	printf("New Values:\n");
	for(cmd = 0; cmd <= spi_cnt; cmd++) {
		printf("   SPI Flash %d:\n", cmd+1);
		printf("\tStatus register = %02X\n", st_reg[cmd]);
		printf("\tConfiguration register = %02X\n", cfg_reg[cmd]);
	}
#endif

	/* Finally, fill out the global settings for
	   Number of Dummy cycles between Address and Data */

	/* Macronix MX25L6405D */
	/* According to the Macronix spec (Table 7.5), dummy cycles
	   are needed when LC=00 (chip default) for FAST READ,
	   QUAD READ, and QUAD I/O READ commands */
	g_FAST_RD_DMY = 8;		/* Fast Read Mode: 8 cycles */
	g_QUAD_RD_DMY = 6; 		/* Quad Read Mode  */
	g_QUAD_IO_RD_DMY = 6;		/* Quad I/O Read Mode: 6 cycles */
	g_QUAD_IO_DDR_RD_DMY = 6;	/* Quad I/O DDR Read Mode  (NOT SUPPORTED) */

	/* When sending a QUAD I/O READ command, and extra MODE field
	   is needed.
	     [[ Single Data Rate, Quad I/O Read, Latency Code=00b ]]
		<> command = 1-bit, 8 clocks
		<> Addr(32bit) = 4-bit, 8 clocks,
		<> Mode = 4-bit, 2 clocks
		<> Dummy = 4-bit, 4 clocks
		<> Data = 4-bit, 2 clocks x {length}
	    See "Figure 10.37 Quad I/O Read Command Sequence" in Macronix spec
	*/
	/* Use Option data regsiters to output '0' as the
	   'Mode' field by sending OPD3 (at 4-bit) between address
	   and dummy */
	g_QUAD_IO_RD_OPT = 0;

	return ret;
}

/* This function is called when "sf probe" is issued, meaning that
   the user wants to access the deivce in normal single wire SPI mode.
   Since some SPI devices have to have special setups to enable QSPI mode
   or DDR QSPI mode, this function is used to reset those settings
   back to normal single wire FAST_READ mode. */
int qspi_reset_device(struct spi_flash *sf)
{
	int ret = 0;

	if( !strcmp(sf->name, "S25FL512S_256K") ) {
		/* Don't really need to do anything */
	}
	else if( !strcmp(sf->name, "MX25L51235F") ) {
	        /* Macronix and Windbond are similar to Spansion */
		/* Don't really need to do anything */
	}
	else if( !strcmp(sf->name, "N25Q512") ) {
		//ret = remove_dummy_micron(sf);
	}
	else {
		printf("\tWARNING: SPI Flash needs to be added to function %s() sf->name=%s\n",__func__, sf->name);
		return 1;
	}
	return ret;
}

/* QUAD SPI MODE */
int do_qspi(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct spi_flash *my_spi_flash;
	int ret = 0;
	int i;
	u8 cmd;
	u8 dual_chip;
	u8 quad_data;
	u8 quad_addr;
	u8 ddr;
	u32 dmdmcr, drenr, cmncr, drcmr, dropr, drdrenr;

	/* need at least 1 argument (single/dual) */
	if (argc < 2)
		goto usage;

	if ( strcmp(argv[1], "single") == 0)
		dual_chip = 0;
	else if ( strcmp(argv[1], "dual") == 0)
		dual_chip = 1;
	else
		goto usage;

	if ( argc <= 2 )
		quad_addr = 1;
	else if ( strcmp(argv[2], "a1") == 0)
		quad_addr = 0;
	else if ( strcmp(argv[2], "a4") == 0)
		quad_addr = 1;
	else
		goto usage;

	if ( argc <= 3 )
		quad_data = 1;
	else if ( strcmp(argv[3], "d1") == 0)
		quad_data = 0;
	else if ( strcmp(argv[3], "d4") == 0)
		quad_data = 1;
	else
		goto usage;

	if ( argc <= 4 )
		ddr = 0;
	else if ( strcmp(argv[4], "sdr") == 0)
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

	/* For Quad Mode operation, extra setup is needed in the SPI
	   Flash devices */
	if( !strcmp(my_spi_flash->name, "S25FL512S_256K") ) {
		ret = enable_quad_spansion(my_spi_flash, quad_addr, quad_data);
	}
	else if( !strcmp(my_spi_flash->name, "MX25L51235F") ) {
		ret = enable_quad_macronix(my_spi_flash, quad_addr, quad_data);
	}
	else if( !strcmp(my_spi_flash->name, "N25Q512") ) {
		//ret = enable_quad_micron(my_spi_flash, quad_addr, quad_data);
	}
	else
	{
		printf("ERROR: SPI Flash support needs to be added to function %s()\n",__func__);
		spi_flash_free(my_spi_flash);	/* Done with SPI Flash */
		return 1;
	}

	/* Done with SPI Flash */
	spi_flash_free(my_spi_flash);

	if ( ret )
	{
		printf("Failed to set SPI Flash Configuration register.\n");
		return 1;
	}

	/***************************/
	/* Set up RZ SPI Registers */
	/***************************/

	/* Set data pins HIGH when not being used. This helps make sure that
	if you go from dual chip to single chip, only FF will get
	transfered out tp the second chip. */
	cmncr &= ~0x00FF0000UL;
	cmncr |=  0x00550000UL;

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
		/* Set read cmd to Quad Read */
		drcmr = (u32)QUAD_READ << 16;

		/* width: 1-bit cmd, 1-bit addr, 4-bit data */
#if (ADDRESS_BYTE_SIZE == 4)
		/* address: 32 bits */
		drenr = 0x00024f00UL;
#else /* ADDRESS_BYTE_SIZE == 3 */
		/* address: 24 bits */
		drenr = 0x00024700UL;
#endif
		/* Add extra Dummy cycles between address and data */
		dmdmcr = 0x00020000 | (g_QUAD_RD_DMY-1); /* 4 bit width, x cycles */
		drenr |= 0x00008000; /* Set Dummy Cycle Enable (DME) */
	}

	/* 1-bit address, 1-bit data */
	if( !quad_data && !quad_addr ) {
		/* Set read cmd to FAST Read */
		drcmr = (u32)FAST_READ << 16;

		/* width: 1-bit cmd, 1-bit addr, 1-bit data */
#if (ADDRESS_BYTE_SIZE == 4)
		/* address: 32 bits */
		drenr = 0x00004f00;
#else /* ADDRESS_BYTE_SIZE == 3 */
		/* address: 24 bits */
		drenr = 0x00004700;
#endif
		/* Add extra Dummy cycles between address and data */
		dmdmcr = 0x00000000 | (g_FAST_RD_DMY-1); /* 1 bit width, x cycles */
		drenr |= 0x00008000; /* Set Dummy Cycle Enable (DME) */
	}

	/* 4-bit address, 4-bit data */
	if( quad_addr ) {
		/* Set read cmd to Quad I/O */
		drcmr = (u32)QUAD_IO_READ << 16;

		/* width: 1-bit cmd, 4-bit addr, 4-bit data */
#if (ADDRESS_BYTE_SIZE == 4)
		/* address: 32 bits */
		drenr = 0x02024f00;
#else /* ADDRESS_BYTE_SIZE == 3 */
		/* address: 24 bits */
		drenr = 0x02024700;
#endif

		/* Use Option data regsiters to output 0x00 to write the
		   'mode' byte by sending OPD3 (at 4-bit) between address
		   and dummy */
		if ( g_QUAD_IO_RD_OPT ) {
			dropr = 0x00000000;
			drenr |= 0x00200080;	// send OPD3(8-bit) at 4-bit width (2 cycles total)
		}

		/* Add extra Dummy cycles between address and data */
		dmdmcr = 0x00020000 | (g_QUAD_IO_RD_DMY-1); /* 4 bit size, x cycles */

		drenr |= 0x00008000; /* Set Dummy Cycle Enable (DME) */
	}

	if ( ddr ) {
		printf( "WARNING: DDR mode doesn't actually work yet on the RSKRZA1 board.\n"
			"   The Spansion SPI flash has an extra phase in the command stream\n"
			"   that we can't account for.\n");

		/* Set read cmd to Read DDR Quad I/O */
		drcmr = (u32)QUAD_IO_DDR_READ << 16;

		/* Address, option and data all 4-bit DDR */
		drdrenr = 0x00000111;

		/* According to the Spansion spec (Table 8.5), dummy cycles
		   are needed when LC=00b for READ DDR QUAD I/O commnds */
		/* Add extra Dummy cycles between address and data */
		dmdmcr = 0x00020000 | (g_QUAD_IO_DDR_RD_DMY-1); /* 4 bit size, x cycles */
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

	/* Allow 32MB of SPI addressing (POR default is only 16MB) */
	*(volatile u32 *)DREAR_0 = 0x00000001;

	/* Turn Read Burst on, Burst Length=2 uints (also set cache flush) */
	/* Keep SSL low (SSLE=1) in case the next transfer is continugous with
	   our last...saves on address cycle. */
	*(u32 *)DRCR_0 = 0x00010301;
	asm("nop");
	*(volatile u32 *)DRCR_0;	/* Read must be done after cache flush */

	/* Do some dummy reads (our of order) to help clean things up */
	*(volatile u32 *)0x18000010;
	*(volatile int *)0x18000000;

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
	"Usage: qspi [single|dual] [a1|(a4)] [d1|(d4)] [(sdr)|ddr]\n"
	"  (xx) means defualt value if not specified\n"
	"  'a4' requries 'd4' to be set\n"
	"  'ddr' requries 'd4' and 'a4' to be set\n";
U_BOOT_CMD(
	qspi,	CONFIG_SYS_MAXARGS,	1,	do_qspi,
	"Change QSPI XIP Mode", qspi_help_text
);

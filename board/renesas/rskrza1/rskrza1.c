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
	*(u32 *)PMCSRn(n) = 1UL<<(b+16);	// Pin as GPIO
	*(u32 *)PSRn(n) = 1UL<<(b+16) | (u32)d<< b;	// Set direction
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

	int i;

	/* When booting from Parallel NOR, some pins need to be bi-directional */
	/* CS0, RD, A1-A15 */
	#if defined(CONFIG_BOOT_MODE0)
	  #define NOR_BIDIR 1
	  /* TODO: Replace '0' with 'NOR_BIDIR for those pins below */
	#else
	  #define NOR_BIDIR 0
	#endif

	rtc_reset();	/* to start rtc */

	/* =========== Pin Setup =========== */
	/* Specific for the RZ/H on the RSK board. Adjust for your board as needed. */

	/* Serial Console */
	pfc_set_pin_function(3, 0, ALT6, 0, 0);	/* P3_0 = TxD2 */
	pfc_set_pin_function(3, 2, ALT4, 0, 0);	/* P3_2 = RxD2 */

	/* QSPI_0 ch0 (booted in 1-bit, need to change to 4-bit) */
	pfc_set_pin_function(9, 4, ALT2, 0, 1);	/* P9_4 = SPBIO00_0 (bi dir) */
	pfc_set_pin_function(9, 5, ALT2, 0, 1);	/* P9_5 = SPBIO10_0 (bi dir) */
	pfc_set_pin_function(9, 6, ALT2, 0, 1);	/* P9_6 = SPBIO20_0 (bi dir) */
	pfc_set_pin_function(9, 7, ALT2, 0, 1);	/* P9_7 = SPBIO30_0 (bi dir) */

	/* QSPI_0 ch1 (4-bit interface for dual QSPI mode) */
	pfc_set_pin_function(2, 12, ALT4, 0, 1); /* P2_12 = SPBIO01_0 (bi dir) */
	pfc_set_pin_function(2, 13, ALT4, 0, 1); /* P2_13 = SPBIO11_0 (bi dir) */
	pfc_set_pin_function(2, 14, ALT4, 0, 1); /* P2_14 = SPBIO21_0 (bi dir) */
	pfc_set_pin_function(2, 15, ALT4, 0, 1); /* P2_15 = SPBIO31_0 (bi dir) */

	/* RIIC Ch 3 */
	pfc_set_pin_function(1, 6, ALT1, 0, 1);	/* P1_6 = RIIC3SCL (bi dir) */
	pfc_set_pin_function(1, 7, ALT1, 0, 1);	/* P1_7 = RIIC3SDA (bi dir) */

	/* Ethernet */
	pfc_set_pin_function(1, 14, ALT4, 0, 0); /* P1_14 = ET_COL */
	pfc_set_pin_function(5, 9, ALT2, 0, 0);	/* P5_9 = ET_MDC */
	pfc_set_pin_function(3, 3, ALT2, 0, 1);	/* P3_3 = ET_MDIO (bi dir) */
	pfc_set_pin_function(3, 4, ALT2, 0, 0);	/* P3_4 = ET_RXCLK */
	pfc_set_pin_function(3, 5, ALT2, 0, 0);	/* P3_5 = ET_RXER */
	pfc_set_pin_function(3, 6, ALT2, 0, 0);	/* P3_6 = ET_RXDV */
	pfc_set_pin_function(2, 0, ALT2, 0, 0);	/* P2_0 = ET_TXCLK */
	pfc_set_pin_function(2, 1, ALT2, 0, 0);	/* P2_1 = ET_TXER */
	pfc_set_pin_function(2, 2, ALT2, 0, 0);	/* P2_2 = ET_TXEN */
	pfc_set_pin_function(2, 3, ALT2, 0, 0);	/* P2_3 = ET_CRS */
	pfc_set_pin_function(2, 4, ALT2, 0, 0);	/* P2_4 = ET_TXD0 */
	pfc_set_pin_function(2, 5, ALT2, 0, 0);	/* P2_5 = ET_TXD1 */
	pfc_set_pin_function(2, 6, ALT2, 0, 0);	/* P2_6 = ET_TXD2 */
	pfc_set_pin_function(2, 7, ALT2, 0, 0);	/* P2_7 = ET_TXD3 */
	pfc_set_pin_function(2, 8, ALT2, 0, 0);	/* P2_8 = ET_RXD0 */
	pfc_set_pin_function(2, 9, ALT2, 0, 0);	/* P2_9 = ET_RXD1 */
	pfc_set_pin_function(2, 10, ALT2, 0, 0); /* P2_10 = ET_RXD2 */
	pfc_set_pin_function(2, 11, ALT2, 0, 0); /* P2_11 = ET_RXD3 */
	//pfc_set_pin_function(4, 14, ALT8, 0, 0); /* P4_14 = IRQ6 (ET_IRQ) */ /* NOTE: u-boot doesn't enable interrupts */

	/* SDRAM */
	pfc_set_pin_function(5, 8, ALT6, 0, 0);	/* P5_8 = CS2 */
	for(i=0;i<=15;i++)
		pfc_set_pin_function(6, i, ALT1, 0, 1);	/* P6_0~15 = D0-D15 (bi dir) */
	pfc_set_pin_function(7, 2, ALT1, 0, 0);	/* P7_2 = RAS */
	pfc_set_pin_function(7, 3, ALT1, 0, 0);	/* P7_3 = CAS */
	pfc_set_pin_function(7, 4, ALT1, 0, 0);	/* P7_4 = CKE */
	pfc_set_pin_function(7, 5, ALT1, 0, 0);	/* P7_5 = RD/WR */
	pfc_set_pin_function(7, 6, ALT1, 0, 0);	/* P7_6 = WE0/DQMLL */
	pfc_set_pin_function(7, 7, ALT1, 0, 0);	/* P7_7 = WE1/DQMLU */
	for(i=9;i<=15;i++)
		pfc_set_pin_function(7, i, ALT1, 0, 0);	/* P7_9~15: A1-A7 */
	for(i=0;i<=15;i++)
		pfc_set_pin_function(8, i, ALT1, 0, 0);	/* P8_0~15 = A8-A23 */

	/* Parallel NOR Flash */
	/* Assumes previous SDRAM setup A1-A23,D0-D15,WE0 */
	pfc_set_pin_function(9, 0, ALT1, 0, 0);	/* P9_0 = A24 */
	pfc_set_pin_function(9, 1, ALT1, 0, 0);	/* P9_1 = A25 */
	pfc_set_pin_function(7, 8, ALT1, 0, 0);	/* P7_8 = RD */
	pfc_set_pin_function(7, 0, ALT1, 0, 0);	/* P7_0 = CS0 */

	/* LED 0 */
	pfc_set_gpio(7, 1, GPIO_OUT); /* P7_1 = GPIO_OUT */


	/**********************************************/
	/* Configure NOR Flash Chip Select (CS0, CS1) */
	/**********************************************/
	#define CS0WCR_D	0x00000b40
	#define CS0BCR_D	0x10000C00
	#define CS1WCR_D	0x00000b40
	#define CS1BCR_D	0x10000C00
	*(u32 *)CS0WCR = CS0WCR_D;
	*(u32 *)CS0BCR = CS0BCR_D;
	*(u32 *)CS1WCR = CS1WCR_D;
	*(u32 *)CS1BCR = CS1BCR_D;

	/**********************************************/
	/* Configure SDRAM (CS2, CS3)                 */
	/**********************************************/
	/* [[ RZ/A1H RSK BOARD ]]
	* Note : This configuration is invalid for a single SDRAM and is a
	*      : known limitation of the RSK+ board. The port pin used by
	*      : CS3 is configured for LED0. To allow SDRAM operation CS2
	*      : and CS3 must be configured to SDRAM. Option link R164 must
	*      : NOT be fitted to avoid a Data Bus conflict on the SDRAM
	*      : and expansion buffers. In a new application with one SDRAM
	*      : always connect the SDRAM to CS3. On this RSK+ CS3 can not
	*      : be used in another configuration including the expansion
	*      : headers unless the SDRAM is completely disabled. For other
	*      : external memory mapped devices CS1 is available for use
	*      : with the expansion headers.
	*      : See the hardware manual Bus State Controller
	*/
	/* Additionally, even though we are only using CS2, we need to set up
	   the CS3 register becase some bits are common for CS3 and CS2 */

	#define CS2BCR_D	0x00004C00
	#define CS2WCR_D	0x00000480
	#define CS3BCR_D	0x00004C00
	#define CS3WCR_D	0x00004492
	#define SDCR_D		0x00110811
	#define RTCOR_D		0xA55A0080
	#define RTCSR_D		0xA55A0008
	*(u32 *)CS2BCR = CS2BCR_D;
	*(u32 *)CS2WCR = CS2WCR_D;
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
	/* Check the hardware manual if your settings differ */
	#define SDRAM_MODE_CS2 0x3FFFD040
	#define SDRAM_MODE_CS3 0x3FFFE040
	*(u32 *)SDRAM_MODE_CS2 = 0;
	*(u32 *)SDRAM_MODE_CS3 = 0;

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
	tmp[0] = 0x37;
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
	printf(	"\t\trootfs:  0x%08X 0x%06X 0+1 (size*=2)\n",0x400000, 0x2000000-0x400000);
#endif

	/* Boot uImage in external SDRAM */
	/* Rootfs is a squashfs image in memory mapped QSPI */
	/* => run s_boot */
	setenv("s1", "sf probe 0; sf read 09800000 C0000 8000"); // Read out DT blob
	setenv("s2", "sf probe 0:1; sf read 09000000 100000 500000"); //Copy Kernel to SDRAM
	setenv("s3", "bootm start 0x09000000 - 0x09800000 ; bootm loados ;"\
			"fdt memory 0x08000000 0x02000000"); // Change memory address in DTB
	setenv("s4", "qspi dual"); // Change XIP interface to dual QSPI
	setenv("sargs", "console=ttySC2,115200 console=tty0 ignore_loglevel root=/dev/mtdblock0"); // bootargs
	setenv("s_boot", "run s1 s2 s3 s4; set bootargs ${sargs}; fdt chosen; bootm go"); // run the commands

	/* Boot XIP using internal RAM */
	/* Rootfs is a squashfs image in memory mapped QSPI */
	/* => run x_boot */
	/* Read out DT blob */
	setenv("x1", "sf probe 0; sf read 20500000 C0000 8000");
	/* Change memory address in DTB */
	setenv("x2", "fdt addr 20500000 ; fdt memory 0x20000000 0x00A00000"); /* 10MB RAM */
	/* Change XIP interface to dual QSPI */
	setenv("x3", "qspi dual");
	setenv("xargs", "console=ttySC2,115200 console=tty0 ignore_loglevel root=/dev/mtdblock0"); // bootargs
	setenv("x_boot", "run x1 x2 x3; set bootargs ${xargs}; fdt chosen; bootx 18200000 20500000"); // run the commands

	/* Boot XIP using internal RAM */
	/* Rootfs is a AXFS image in memory mapped QSPI */
	/* => run xa_boot */
	/* Read out DT blob */
	setenv("xa1", "sf probe 0; sf read 20500000 C0000 8000");
	/* Change memory address in DTB */
	setenv("xa2", "fdt addr 20500000 ; fdt memory 0x20000000 0x00A00000"); /* 10MB RAM */
	/* Change XIP interface to dual QSPI */
	setenv("xa3", "qspi dual");
	setenv("xaargs", "console=ttySC2,115200 console=tty0 ignore_loglevel root=/dev/null rootflags=physaddr=0x18800000"); // bootargs
	setenv("xa_boot", "run xa1 xa2 xa3; set bootargs ${xaargs}; fdt chosen; bootx 18200000 20500000"); // run the commands

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

/**********************/
/* Spansion S25FL512S */
/**********************/
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

/* Number of Dummy cycles between Address and data */
/* Spansion S25FL512S, Latency Code (LC)=00 (chip default) */
#define FAST_RD_DMY 8		/* Fast Read Mode */
#define QUAD_RD_DMY 8		/* Quad Read Mode  */
#define QUAD_IO_RD_DMY 4	/* Quad I/O Read Mode  */
#define QUAD_IO_DDR_RD_DMY 6	/* Quad I/O DDR Read Mode  */

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

	/* Skip SPI Flash configure if already correct */
//	if ( data[1] != 0x02 ) {
	if ( 1  ) { /* Do every time to keep dual SPI flash in sync*/
		data[0] = 0x00;	/* status reg: Don't Care */
		if( quad_data )
			data[1] = 0x02; /* confg reg: Set QUAD, LC=00b */
		else
			data[1] = 0x02; /* confg reg: clear QUAD, LC=00b */
			//data[1] = 0x00; /* confg reg: clear QUAD, LC=00b */

		if( quad_addr )
			data[1] = 0x02; /* confg reg: Set QUAD, LC=00b */

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
		printf("Failed to set SPI Flash Configuration register.\n");
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
		/* According to the Spansion spec (Table 8.5), dummy cycles
		   are needed when LC=00b for QUAD READ commands */
		/* Add extra Dummy cycles between address and data */
		dmdmcr = 0x00020000 | (QUAD_RD_DMY-1); /* 4 bit width, 8 cycles */
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

		/* According to the Spansion spec (Table 8.5), dummy cycles
		   are needed when LC=00b for FAST READ commnds */
		/* Add extra Dummy cycles between address and data */
		dmdmcr = 0x00000000 | (FAST_RD_DMY-1); /* 1 bit width, 8 cycles */
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
		dropr = 0x00000000;
		drenr |= 0x00200080;	// send OPD3 at 4-bits

		/* According to the Spansion spec (Table 8.5), dummy cycles
		   are needed when LC=00b for QUAD I/O READ commnds */
		/* Add extra Dummy cycles between address and data */
		dmdmcr = 0x00020000 | (QUAD_IO_RD_DMY-1); /* 4 bit size, 4 cycles */

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
		dmdmcr = 0x00020000 | (QUAD_IO_DDR_RD_DMY-1); /* 4 bit size, 6 cycles */
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


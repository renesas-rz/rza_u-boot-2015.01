#define DEBUG_hide
/*
 * Configuation settings for the Renesas Stream it board
 *
 * Copyright (C) 2017 Renesas Electronics America
 * Copyright (C) 2017 Chris Brandt
 *
 * This file is released under the terms of GPL v2 and any later version.
 * See the file COPYING in the root directory of the source tree for details.
 */

#ifndef __STREAMIT_H
#define __STREAMIT_H

#define CONFIG_ARMV7 1
#define CONFIG_CPU_RZA1 1
#define CONFIG_BOARD_LATE_INIT 1
#define CONFIG_MACH_TYPE MACH_TYPE_STREAMIT
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_SYS_GENERIC_BOARD

/**********************************************************************
 * SDRAM Selection
 *
 * Select what external SDRAM (if any) you have on the board by removing
 * the comments around the correct #define statement.
 * Note that Stream it V2 boards come populated with 32MByte SDRAM.
 **********************************************************************/

/* Winbond 256Mb (32MByte) SDRAM: Part number W9825G6KH-6I
 *  [ Stream it V2 boards ] */
/* #define SDRAM_W9825G6KH_6I */

/* ISSI 128Mb (16MByte) SDRAM: Part number IS42/45S16800F
 *   Requires pin lift and jumper wire */
/* #define SDRAM_IS42_45S16800F */

/* No SDRAM populated, just use the internal 3MB RAM */
/* #define SDRAM_NONE */


/* Print error and stop build if no selection has been made */
#ifndef SDRAM_W9825G6KH_6I
 #ifndef SDRAM_IS42_45S16800F
  #ifndef SDRAM_NONE
    #error "Please make your SDRAM selection in file streamit.h"
  #endif
 #endif
#endif

#define CONFIG_CMD_NET
#define CONFIG_CMD_MII
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_USB
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_FAT
#define CONFIG_CMD_SF
#define CONFIG_CMD_I2C
#define CONFIG_CMD_EEPROM
#define CONFIG_CMD_DATE
#define CONFIG_DOS_PARTITION
#define CONFIG_MAC_PARTITION
#define CONFIG_USB_STORAGE
#define CONFIG_CMD_SNTP
#define CONFIG_BOOTP_NTPSERVER
#define CONFIG_BOOTP_TIMEOFFSET
#define CONFIG_ZERO_BOOTDELAY_CHECK	/* check for keypress on bootdelay==0 */
#define CONFIG_CMD_SETEXPR
#define CONFIG_CMD_BOOTZ

#define CONFIG_OF_LIBFDT
#define CONFIG_CMDLINE_EDITING
#define CONFIG_CMDLINE_TAG

/* Uncomment for systems with No Parallel NOR Flash */
#define CONFIG_SYS_NO_FLASH

#ifndef _CONFIG_CMD_DEFAULT_H
# include <config_cmd_default.h>
#endif

#define CONFIG_BAUDRATE		115200
#define CONFIG_BOOTARGS		"console=ttySC2,115200"
#define CONFIG_IPADDR		192.168.0.55
#define CONFIG_SERVERIP		192.168.0.1
#define CONFIG_BOOTDELAY	3
#define CONFIG_SYS_BAUDRATE_TABLE	{ CONFIG_BAUDRATE }

#define CONFIG_SYS_LONGHELP		/* undef to save memory	*/
#define CONFIG_SYS_PROMPT	"=> "	/* Monitor Command Prompt */
#define CONFIG_SYS_CBSIZE	256	/* Boot Argument Buffer Size */
#define CONFIG_SYS_PBSIZE	256	/* Print Buffer Size */
#define CONFIG_SYS_MAXARGS	16	/* max number of command args */

#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2      "> "

#define CONFIG_SYS_ARM_CACHE_WRITETHROUGH

/* Serial */
#define CONFIG_SCIF_CONSOLE
#define CONFIG_CONS_SCIF3
#define SCIF0_BASE			0xE8007000
#define SCIF1_BASE			0xE8007800
#define SCIF2_BASE			0xE8008000
#define SCIF3_BASE			0xE8008800
#define SCIF4_BASE			0xE8009000
#define SCIF5_BASE			0xE8009800
#define SCIF6_BASE			0xE800A000
#define SCIF7_BASE			0xE800A800
#define CONFIG_SH_SCIF_CLK_FREQ CONFIG_SYS_CLK_FREQ

/* Memory */
/* u-boot relocated to top 256KB of ram */
#define CONFIG_NR_DRAM_BANKS		1
#if !defined(CONFIG_BOOT_MODE0)

/* SPI_FLASH_LOADER: Build a version that can be downloaded to RAM directly and run
   in order to be used to program QSPI flash for the first time. */
/* #define SPI_FLASH_LOADER */

#ifdef SPI_FLASH_LOADER
 #define CONFIG_SYS_TEXT_BASE		0x20020000
 #define CONFIG_ENV_IS_NOWHERE
#else
 #define CONFIG_SYS_TEXT_BASE		0x18000000
#endif
#else
#define CONFIG_SYS_TEXT_BASE		0x00000000
#endif
#define USE_INTERNAL_RAM
#ifdef USE_INTERNAL_RAM
 #define CONFIG_SYS_SDRAM_BASE		0x20000000
 #define CONFIG_SYS_SDRAM_SIZE		(3 * 1024 * 1024)
 #define CONFIG_SYS_INIT_SP_ADDR         (CONFIG_SYS_SDRAM_BASE + CONFIG_SYS_SDRAM_SIZE - 1*1024*1024)
#else
 #define CONFIG_SYS_SDRAM_BASE		0x0C000000
 #define CONFIG_SYS_SDRAM_SIZE		(16 * 1024 * 1024)
 /* NOTE: The pin setup and SDRAM configuration is done in C,
          so an initial stack has to be set up first, meaning you
          can't have your init stack in SDRAM before the SDRAM is setup.
	  Therfore, we need to use internal RAM for init stack.
          Later on in boot, it will automatically switch the stack to
          SDRAM. */
 #define CONFIG_SYS_INIT_SP_ADDR         0x20300000 /* Internal RAM @ 3MB */
#endif

#define CONFIG_SYS_MEMTEST_START	CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_SDRAM_BASE + 0x2000000)
#define CONFIG_SYS_MALLOC_LEN		(512 * 1024)
#define CONFIG_SYS_MONITOR_LEN		(128 * 1024)
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + 1*1024*1024)
#define	CONFIG_LOADADDR			CONFIG_SYS_SDRAM_BASE

#if !defined(CONFIG_BOOT_MODE0)
#ifndef SPI_FLASH_LOADER
  #define CONFIG_ENV_IS_IN_SPI_FLASH
#endif
#define CONFIG_ENV_OFFSET	0x80000
#define CONFIG_ENV_SECT_SIZE	0x40000		/* smallest erase sector size */
#else
#define CONFIG_ENV_IS_IN_FLASH
#define CONFIG_ENV_OFFSET	(512 * 1024)
#define CONFIG_ENV_SECT_SIZE	(256 * 1024)
#endif

#define CONFIG_ENV_SIZE		CONFIG_ENV_SECT_SIZE
#define CONFIG_ENV_ADDR		(CONFIG_SYS_FLASH_BASE + CONFIG_ENV_OFFSET)
#define CONFIG_ENV_OVERWRITE	1

#define __io

/* Spi-Flash configuration */
#define CONFIG_RZ_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_MACRONIX
#define CONFIG_SPI_FLASH_SPANSION
#define CONFIG_RZA1_BASE_QSPI0		0x3FEFA000
#define CONFIG_SPI_FLASH_BAR		/* For SPI Flash bigger than 16MB */
#define CONFIG_SF_SHOW_PROGRESS		/* Display status while erasing and writing */

/* I2C configuration */
#define CONFIG_SH_RIIC
#define CONFIG_HARD_I2C
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_MAX_I2C_BUS		4
#define CONFIG_SYS_I2C_SPEED		100000 /* 100 kHz */
#define CONFIG_SYS_I2C_EEPROM_ADDR	0x50
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	1
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	4	/* 16-Byte Write Mode */
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	10	/* 10ms of delay */
#define CONFIG_SYS_I2C_MODULE		0
#define CONFIG_SH_I2C_BASE0		0xFCFEE000
#define CONFIG_SH_I2C_BASE1		0xFCFEE400
#define CONFIG_SH_I2C_BASE2		0xFCFEE800
#define CONFIG_SH_I2C_BASE3		0xFCFEEc00

/* RTC configuration */
#define CONFIG_RTC_RZA1
#define CONFIG_RTC_RZA1_BASE_ADDR	0xFCFF1000

/* Board Clock */
#define CONFIG_SYS_CLK_FREQ	64000000 /* (USB CLOCK SOURCE) P1 clock = (48MHz / 4 x 32) / 6 = 64MHz*/
#define CONFIG_SYS_HZ		1000

/* Network interface */
#define CONFIG_SH_ETHER
#define CONFIG_SH_ETHER_USE_PORT	0
#define CONFIG_SH_ETHER_PHY_ADDR	0
#define CONFIG_SH_ETHER_PHY_MODE PHY_INTERFACE_MODE_MII
#define CONFIG_SH_ETHER_EEPROM_ADDR	2 /* MAC address offset in EEPROM */
#define CONFIG_PHYLIB
#define CONFIG_BITBANGMII
#define CONFIG_BITBANGMII_MULTI

/* USB host controller */
#define CONFIG_USB_R8A66597_HCD
#define CONFIG_R8A66597_BASE_ADDR	0xE8207000
#define CONFIG_R8A66597_XTAL		0x0000	/* 48MHz */
#define CONFIG_R8A66597_ENDIAN		0x0000	/* little */

/*
 * Lowlevel configuration
 */
/* Disable WDT */
#define WTCSR_D		0xA518
#define WTCNT_D		0x5A00

/* Set clocks based on 48MHz USB xtal */
#define FRQCR_D		0x1035	/* CPU= 300-400 MHz */

/* Enable all peripherals */
#define STBCR3_D	0x00000000
#define STBCR4_D	0x00000000
#define STBCR5_D	0x00000000
#define STBCR6_D	0x00000000
#define STBCR7_D	0x00000024
#define STBCR8_D	0x00000005
#define STBCR9_D	0x00000000
#define STBCR10_D	0x00000000
#define STBCR11_D	0x000000c0
#define STBCR12_D	0x000000f0


/* NOTE: Setup of pins, chip selects and SDRAM moved to board_early_init_f() */


#endif	/* __STREAMIT_H */
